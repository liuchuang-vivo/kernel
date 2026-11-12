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

//! Single-image application ownership, membership and cooperative main-thread exit.
//! The live-thread registry owns groups; retirement callbacks hold weak handles.
//! Group ownership therefore survives a dropped observer without an Arc cycle.

use crate::{
    process::Process,
    scheduler,
    sync::SpinLock,
    thread::{self, DeferredCleanup, GlobalQueueVisitor, Thread, ThreadNode},
    time::Tick,
};
use alloc::{
    boxed::Box,
    collections::BTreeMap,
    sync::{Arc, Weak},
    vec::Vec,
};
use core::sync::atomic::{AtomicUsize, Ordering};
use spin::Mutex;

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum ApplicationState {
    Prepared,
    Running,
    Exiting,
    Exited,
}
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum ApplicationExit {
    /// Main return or exit completed libc finalization before retirement.
    Code(i32),
    /// Startup was cancelled or main retired without completing finalization.
    Aborted,
}
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum ApplicationError {
    AlreadyStarted,
    NotStarted,
    ProcessUnavailable,
    QueueFailed,
}

struct State {
    phase: ApplicationState,
    members: Vec<ThreadNode>,
    // Keep the main stack (argv/envp/auxv backing) after main retires abnormally.
    main: Option<ThreadNode>,
    main_id: usize,
    status: Option<i32>,
    fini_complete: bool,
    outcome: Option<ApplicationExit>,
}

#[must_use = "retain application resources until all member cleanups finish"]
pub(crate) struct Group {
    state: SpinLock<State>,
    // Exclusive ownership transfer needs Send, not shared access to the image.
    // Only the last member takes this value, outside the state lock.
    owner: Mutex<Option<Box<dyn Send>>>,
    changed: AtomicUsize,
}
static MEMBERS: SpinLock<BTreeMap<usize, Arc<Group>>> = SpinLock::new(BTreeMap::new());

/// Observe and start an application. Dropping a prepared handle cancels startup;
/// dropping a running handle leaves its members owning the image through exit.
#[must_use = "start the application or drop it to cancel its prepared resources"]
pub struct Application {
    group: Arc<Group>,
    thread: ThreadNode,
}

impl Application {
    /// Access the main thread for configuration. Use start() to schedule it;
    /// queuing this node separately bypasses application state transitions.
    pub fn thread(&self) -> &ThreadNode {
        &self.thread
    }
    pub fn state(&self) -> ApplicationState {
        self.group.state.irqsave_lock().phase
    }
    /// Includes retired members until their deferred cleanup has completed.
    pub fn member_count(&self) -> usize {
        self.group.state.irqsave_lock().members.len()
    }
    pub fn exit_status(&self) -> Option<ApplicationExit> {
        self.group.state.irqsave_lock().outcome
    }

    /// Schedule once, optionally associating the main thread with an existing
    /// Process. No Process is needed for a direct RTOS application.
    pub fn start(&mut self, process: Option<&Arc<Process>>) -> Result<(), ApplicationError> {
        if self.state() != ApplicationState::Prepared || self.thread.state() != thread::IDLE {
            return Err(ApplicationError::AlreadyStarted);
        }
        if let Some(process) = process
            && !process.add_thread(self.thread.clone())
        {
            self.cancel();
            return Err(ApplicationError::ProcessUnavailable);
        }
        self.group.state.irqsave_lock().phase = ApplicationState::Running;
        if scheduler::queue_ready_thread(thread::IDLE, self.thread.clone()).is_err() {
            if let Some(process) = process {
                process.remove_thread(&mut self.thread);
            }
            self.group.state.irqsave_lock().phase = ApplicationState::Prepared;
            self.cancel();
            return Err(ApplicationError::QueueFailed);
        }
        Ok(())
    }

    /// Wait until every member cleanup and image release have finished.
    pub fn wait(&self) -> Result<ApplicationExit, ApplicationError> {
        if self.state() == ApplicationState::Prepared {
            return Err(ApplicationError::NotStarted);
        }
        loop {
            let epoch = self.group.changed.load(Ordering::Acquire);
            if let Some(status) = self.exit_status() {
                return Ok(status);
            }
            let _result = crate::sync::atomic_wait(&self.group.changed, epoch, Tick::MAX);
        }
    }

    fn cancel(&mut self) {
        if self.thread.state() != thread::IDLE {
            return;
        }
        GlobalQueueVisitor::remove(&mut self.thread);
        let cleanup = self.thread.lock().take_cleanup();
        drop(cleanup);
        self.group.finish(Thread::id(&self.thread));
    }
}
impl Drop for Application {
    fn drop(&mut self) {
        if self.state() == ApplicationState::Prepared {
            self.cancel();
        }
    }
}

impl Group {
    pub(super) fn prepare(owner: Box<dyn Send>) -> Arc<Self> {
        Arc::new(Self {
            state: SpinLock::new(State {
                phase: ApplicationState::Prepared,
                members: Vec::new(),
                main: None,
                main_id: 0,
                status: None,
                fini_complete: false,
                outcome: None,
            }),
            owner: Mutex::new(Some(owner)),
            changed: AtomicUsize::new(0),
        })
    }

    pub(super) fn main_cleanup(self: &Arc<Self>) -> Result<DeferredCleanup, thread::CleanupError> {
        let group = Arc::downgrade(self);
        DeferredCleanup::new(move || {
            if let Some(group) = group.upgrade() {
                let id = group.state.irqsave_lock().main_id;
                group.finish(id);
            }
        })
    }

    pub(super) fn install(self: &Arc<Self>, thread: ThreadNode) -> Application {
        let id = Thread::id(&thread);
        {
            let mut state = self.state.irqsave_lock();
            state.main_id = id;
            state.main = Some(thread.clone());
            state.members.push(thread.clone());
        }
        assert!(MEMBERS.irqsave_lock().insert(id, self.clone()).is_none());
        Application {
            group: self.clone(),
            thread,
        }
    }

    pub(crate) fn attach(self: &Arc<Self>, thread: ThreadNode) -> Result<(), ()> {
        let id = Thread::id(&thread);
        {
            let mut state = self.state.irqsave_lock();
            if state.phase != ApplicationState::Running {
                return Err(());
            }
            state.members.push(thread);
        }
        // The member retains its ThreadNode, preventing id reuse until cleanup.
        assert!(MEMBERS.irqsave_lock().insert(id, self.clone()).is_none());
        Ok(())
    }

    pub(crate) fn child_cleanup(
        self: &Arc<Self>,
        id: usize,
        callback: Option<extern "C" fn(*mut core::ffi::c_void)>,
        argument: usize,
    ) -> Result<DeferredCleanup, thread::CleanupError> {
        let group: Weak<Self> = Arc::downgrade(self);
        DeferredCleanup::new(move || {
            // The C SpawnArgs contract retains argument storage until cleanup;
            // membership keeps the callback's image mapped through this call.
            if let Some(callback) = callback {
                callback(argument as *mut core::ffi::c_void);
            }
            if let Some(group) = group.upgrade() {
                group.finish(id);
            }
        })
    }

    pub(crate) fn finish(self: &Arc<Self>, id: usize) {
        let registered = { MEMBERS.irqsave_lock().remove(&id) };
        let (member, main, complete) = {
            let mut state = self.state.irqsave_lock();
            let Some(index) = state.members.iter().position(|t| Thread::id(t) == id) else {
                return;
            };
            let member = state.members.swap_remove(index);
            if id == state.main_id {
                state.phase = ApplicationState::Exiting;
            }
            if state.members.is_empty() {
                state.phase = ApplicationState::Exiting;
                (member, state.main.take(), true)
            } else {
                (member, None, false)
            }
        };
        let owner = if complete {
            self.owner.lock().take()
        } else {
            None
        };
        // Resource release may block. No group or registry lock survives here.
        drop(owner);
        drop(main);
        drop(member);
        drop(registered);
        if complete {
            let mut state = self.state.irqsave_lock();
            state.outcome = Some(if state.fini_complete {
                ApplicationExit::Code(state.status.expect("normal exit status"))
            } else {
                ApplicationExit::Aborted
            });
            state.phase = ApplicationState::Exited;
        }
        self.changed.fetch_add(1, Ordering::Release);
        let _result = crate::sync::atomic_wake(&self.changed, usize::MAX);
    }

    pub(crate) fn begin_exit(&self, id: usize, status: i32) -> Result<(), ()> {
        {
            let mut state = self.state.irqsave_lock();
            // First delivery coordinates normal exit from the main thread.
            // It waits for children to finish cooperatively; it does not kill them.
            if id != state.main_id || state.phase != ApplicationState::Running {
                return Err(());
            }
            state.status = Some(status);
            state.phase = ApplicationState::Exiting;
        }
        loop {
            let epoch = self.changed.load(Ordering::Acquire);
            if self.state.irqsave_lock().members.len() == 1 {
                return Ok(());
            }
            let _result = crate::sync::atomic_wait(&self.changed, epoch, Tick::MAX);
        }
    }

    pub(crate) fn finish_exit(&self, id: usize) -> Result<(), ()> {
        let mut state = self.state.irqsave_lock();
        if id != state.main_id
            || state.phase != ApplicationState::Exiting
            || state.members.len() != 1
        {
            return Err(());
        }
        state.fini_complete = true;
        Ok(())
    }
}

pub(crate) fn for_thread(id: usize) -> Option<Arc<Group>> {
    MEMBERS.irqsave_lock().get(&id).cloned()
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::{
        arch,
        process::ProcessManager,
        thread::{Builder, Entry},
    };
    use blueos_test_macro::test;
    use core::{ffi::c_void, sync::atomic::AtomicBool};

    #[derive(Default)]
    struct Observed {
        release_main: AtomicBool,
        release_child: AtomicBool,
        child_entered: AtomicBool,
        child_cleaned: AtomicBool,
        dropped: AtomicBool,
    }

    #[must_use]
    struct Owner(Arc<Observed>);
    impl Drop for Owner {
        fn drop(&mut self) {
            assert!(arch::local_irq_enabled());
            assert!(!crate::irq::is_in_irq());
            scheduler::yield_me();
            self.0.dropped.store(true, Ordering::Release);
        }
    }

    fn wait_for(mut ready: impl FnMut() -> bool) {
        let start = crate::time::now();
        while !ready() {
            assert!(crate::time::now().saturating_sub(start) < core::time::Duration::from_secs(2));
            scheduler::yield_me();
        }
    }
    extern "C" fn empty() {}
    extern "C" fn main_entry(argument: *mut c_void) {
        // The owner and the test retain Observed through all callbacks.
        let observed = unsafe { &*argument.cast::<Observed>() };
        wait_for(|| observed.release_main.load(Ordering::Acquire));
    }
    extern "C" fn child_entry(argument: *mut c_void) {
        let observed = unsafe { &*argument.cast::<Observed>() };
        observed.child_entered.store(true, Ordering::Release);
        wait_for(|| observed.release_child.load(Ordering::Acquire));
        assert!(!observed.dropped.load(Ordering::Acquire));
    }
    extern "C" fn child_cleanup(argument: *mut c_void) {
        let observed = unsafe { &*argument.cast::<Observed>() };
        assert!(arch::local_irq_enabled());
        assert!(!observed.dropped.load(Ordering::Acquire));
        observed.child_cleaned.store(true, Ordering::Release);
    }
    fn prepare() -> (Application, Arc<Observed>) {
        let observed = Arc::new(Observed::default());
        let group = Group::prepare(Box::new(Owner(observed.clone())));
        let cleanup = group.main_cleanup().unwrap();
        let thread = Builder::new(Entry::Posix(
            main_entry,
            Arc::as_ptr(&observed).cast_mut().cast(),
        ))
        .build();
        thread.lock().set_deferred_cleanup(cleanup);
        (group.install(thread), observed)
    }

    #[test]
    fn dropping_prepared_application_cancels_registration() {
        let (application, observed) = prepare();
        let id = Thread::id(application.thread());
        assert!(for_thread(id).is_some());
        assert_eq!(application.wait(), Err(ApplicationError::NotStarted));
        drop(application);
        assert!(observed.dropped.load(Ordering::Acquire));
        assert!(for_thread(id).is_none());
        assert!(GlobalQueueVisitor::find_if(|t| Thread::id(t) == id).is_none());
    }

    #[test]
    fn application_retains_image_after_abnormal_main_exit() {
        let (mut application, observed) = prepare();
        application.start(None).unwrap();
        let pointer = Arc::as_ptr(&observed).cast_mut().cast();
        let child = Builder::new(Entry::Posix(child_entry, pointer)).build();
        let id = Thread::id(&child);
        let cleanup = application
            .group
            .child_cleanup(id, Some(child_cleanup), pointer as usize)
            .unwrap();
        application.group.attach(child.clone()).unwrap();
        child.lock().set_deferred_cleanup(cleanup);
        assert_eq!(scheduler::queue_ready_thread(thread::IDLE, child), Ok(()));
        wait_for(|| observed.child_entered.load(Ordering::Acquire));
        observed.release_main.store(true, Ordering::Release);
        wait_for(|| {
            application.state() == ApplicationState::Exiting && application.member_count() == 1
        });
        assert_eq!(application.thread.state(), thread::RETIRED);
        assert!(!observed.dropped.load(Ordering::Acquire));
        assert_eq!(application.exit_status(), None);
        // New members cannot extend an application after its main exits.
        let mut rejected = Builder::new(Entry::C(empty)).build();
        assert!(application.group.attach(rejected.clone()).is_err());
        GlobalQueueVisitor::remove(&mut rejected);
        observed.release_child.store(true, Ordering::Release);
        assert_eq!(application.wait(), Ok(ApplicationExit::Aborted));
        assert!(observed.child_cleaned.load(Ordering::Acquire));
        assert!(observed.dropped.load(Ordering::Acquire));
        assert!(for_thread(id).is_none());
    }

    #[test]
    fn running_application_survives_dropped_observer() {
        let (mut application, observed) = prepare();
        application.start(None).unwrap();
        let group = Arc::downgrade(&application.group);
        drop(application);
        assert!(group.upgrade().is_some());
        assert!(!observed.dropped.load(Ordering::Acquire));
        observed.release_main.store(true, Ordering::Release);
        wait_for(|| group.upgrade().is_none());
        assert!(observed.dropped.load(Ordering::Acquire));
    }

    #[test]
    fn application_can_use_an_existing_process() {
        let manager = ProcessManager::get();
        let process = manager
            .create_process(Arc::downgrade(manager.root_process()))
            .unwrap();
        let (mut application, observed) = prepare();
        application.start(Some(&process)).unwrap();
        assert_eq!(application.thread.process().unwrap().pid(), process.pid());
        assert_eq!(process.thread_count(), 1);
        observed.release_main.store(true, Ordering::Release);
        assert_eq!(application.wait(), Ok(ApplicationExit::Aborted));
        assert_eq!(process.thread_count(), 0);
        assert!(manager.process_table().get(process.pid()).is_none());
    }

    #[test]
    fn application_rejects_dead_process_without_leaking_thread() {
        let manager = ProcessManager::get();
        let process = manager
            .create_process(Arc::downgrade(manager.root_process()))
            .unwrap();
        drop(manager.destroy_process(process.pid()).unwrap());
        let (mut application, observed) = prepare();
        let id = Thread::id(application.thread());
        assert_eq!(
            application.start(Some(&process)),
            Err(ApplicationError::ProcessUnavailable)
        );
        assert_eq!(application.wait(), Ok(ApplicationExit::Aborted));
        assert!(observed.dropped.load(Ordering::Acquire));
        assert!(for_thread(id).is_none());
        assert!(GlobalQueueVisitor::find_if(|t| Thread::id(t) == id).is_none());
    }
}
