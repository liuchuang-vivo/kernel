// Copyright (c) 2026 vivo Mobile Communication Co., Ltd.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//       http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

//! Retirement callbacks executed by an ordinary kernel worker.
//!
//! The context-switch path cannot block or make another scheduler operation.
//! It only posts preallocated tasks here. A worker polls with a bounded wait
//! because waking it while retiring would reenter scheduler/wait-list locks.
//! Callbacks own Send captures and run without the queue lock or an IRQ guard.
//! They do not inherit the retired thread's Process or file descriptor context.

use super::{Builder, Entry, GlobalQueueVisitor, Stack, IDLE};
use crate::{config, scheduler, support::DisableInterruptGuard, time::Tick};
use alloc::boxed::Box;
use core::sync::atomic::{AtomicUsize, Ordering};
use spin::{Mutex, Once};

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum CleanupError {
    NoMemory,
    QueueFailed,
}

/// Owns a callback and its queue node, prepared before the thread retires.
#[must_use = "install or execute the retirement callback before releasing its resources"]
pub struct DeferredCleanup(Box<Task>);

#[must_use = "retain queued resources until the callback executes"]
struct Task {
    callback: Box<dyn FnOnce() + Send>,
    next: Option<Box<Task>>,
}

impl core::fmt::Debug for DeferredCleanup {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        f.debug_struct("DeferredCleanup").finish_non_exhaustive()
    }
}

impl DeferredCleanup {
    /// Prepare from normal thread context, outside the retiring thread's lock.
    /// On failure, captures are dropped here in the caller's ordinary context.
    pub fn new(callback: impl FnOnce() + Send + 'static) -> Result<Self, CleanupError> {
        ensure_worker()?;
        Ok(Self(Box::new(Task {
            callback: Box::new(callback),
            next: None,
        })))
    }

    /// Execute a callback taken from a thread before it was posted. The caller
    /// must use ordinary thread context, with interrupts enabled and no locks
    /// that prevent the callback from blocking or scheduling.
    pub fn run(self) {
        assert!(
            crate::arch::local_irq_enabled(),
            "deferred cleanup requires interrupts enabled"
        );
        assert!(
            !crate::irq::is_in_irq(),
            "deferred cleanup requires thread context"
        );
        let Task { callback, next } = *self.0;
        debug_assert!(next.is_none());
        callback();
    }
}

static STARTED: Once<()> = Once::new();
static PENDING: Mutex<Option<Box<Task>>> = Mutex::new(None);
static GENERATION: AtomicUsize = AtomicUsize::new(0);
const POLL_MILLIS: u64 = 10;

fn ensure_worker() -> Result<(), CleanupError> {
    // Keep initialization from being preempted by a same-core caller spinning
    // on Once. The worker has a plain C entry and no captured startup owner.
    let _irq = DisableInterruptGuard::new();
    STARTED
        .try_call_once(|| {
            let stack =
                Stack::from_size(config::SYSTEM_THREAD_STACK_SIZE).ok_or(CleanupError::NoMemory)?;
            let mut worker = Builder::new(Entry::C(run)).set_stack(stack).build();
            if scheduler::queue_ready_thread(IDLE, worker.clone()).is_err() {
                GlobalQueueVisitor::remove(&mut worker);
                return Err(CleanupError::QueueFailed);
            }
            Ok(())
        })
        .map(|_| ())
}

/// Post without allocating, invoking callbacks, or touching scheduler locks.
pub(crate) fn post(cleanup: DeferredCleanup) {
    let _irq = DisableInterruptGuard::new();
    let mut queue = PENDING.lock();
    let mut task = cleanup.0;
    task.next = queue.take();
    *queue = Some(task);
    GENERATION.fetch_add(1, Ordering::Release);
}

extern "C" fn run() {
    loop {
        let generation = GENERATION.load(Ordering::Acquire);
        let mut tasks = {
            let _irq = DisableInterruptGuard::new();
            PENDING.lock().take()
        };
        while let Some(mut task) = tasks {
            tasks = task.next.take();
            DeferredCleanup(task).run();
        }
        // Only the ordinary worker takes wait-list and scheduler locks.
        let _result =
            crate::sync::atomic_wait(&GENERATION, generation, Tick::from_millis(POLL_MILLIS));
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::{
        arch,
        thread::{self, Cleanup},
    };
    use alloc::sync::Arc;
    use blueos_test_macro::test;

    static INLINE_COUNT: AtomicUsize = AtomicUsize::new(0);
    extern "C" fn empty() {}
    extern "C" fn inline() {
        assert!(!arch::local_irq_enabled());
        INLINE_COUNT.fetch_add(1, Ordering::Release);
    }
    extern "C" fn inline_posix(_arg: *mut core::ffi::c_void) {
        inline();
    }

    fn wait_for(value: &AtomicUsize, expected: usize) {
        let start = crate::time::now();
        while value.load(Ordering::Acquire) != expected {
            assert!(crate::time::now().saturating_sub(start) < core::time::Duration::from_secs(2));
            scheduler::yield_me();
        }
    }

    #[test]
    fn ordinary_cleanup_keeps_inline_execution_for_all_entry_kinds() {
        INLINE_COUNT.store(0, Ordering::Release);
        for (index, entry) in [
            Entry::C(inline),
            Entry::Posix(inline_posix, core::ptr::null_mut()),
            Entry::Closure(Box::new(|| inline())),
        ]
        .into_iter()
        .enumerate()
        {
            let thread = Builder::new(Entry::C(empty)).build();
            thread.lock().set_cleanup(entry);
            assert_eq!(scheduler::queue_ready_thread(IDLE, thread), Ok(()));
            wait_for(&INLINE_COUNT, index + 1);
        }
    }

    #[test]
    fn deferred_cleanup_runs_in_a_worker_and_can_schedule() {
        let done = Arc::new(AtomicUsize::new(0));
        let finished = done.clone();
        let thread = Builder::new(Entry::C(empty)).build();
        let retired_id = thread::Thread::id(&thread);
        let cleanup = DeferredCleanup::new(move || {
            assert!(arch::local_irq_enabled());
            assert!(!crate::irq::is_in_irq());
            assert_ne!(scheduler::current_thread_id(), retired_id);
            scheduler::yield_me();
            let nested_done = finished.clone();
            let nested = Builder::new(Entry::C(empty)).build();
            let nested_cleanup = DeferredCleanup::new(move || {
                nested_done.store(2, Ordering::Release);
            })
            .unwrap();
            nested.lock().set_deferred_cleanup(nested_cleanup);
            assert_eq!(scheduler::queue_ready_thread(IDLE, nested), Ok(()));
            finished.store(1, Ordering::Release);
        })
        .unwrap();
        thread.lock().set_deferred_cleanup(cleanup);
        assert!(matches!(thread.lock().cleanup, Some(Cleanup::Deferred(_))));
        assert_eq!(scheduler::queue_ready_thread(IDLE, thread), Ok(()));
        wait_for(&done, 2);
    }
}
