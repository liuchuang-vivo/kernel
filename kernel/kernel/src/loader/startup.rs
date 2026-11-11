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

//! Standard ELF initial-stack construction for standalone native images.

use super::adapters::KernelCodeCache;
use crate::{
    scheduler,
    thread::{self, Builder, CleanupError, DeferredCleanup, Entry, Stack, ThreadNode},
};
use alloc::boxed::Box;
use blueos_header::startup::{auxv, STACK_ALIGNMENT};
use blueos_loader::{memory::TargetRange, CodeCache, ImageMapping, LoadedImage};
use core::{
    ffi::{c_int, c_void, CStr},
    mem::size_of,
    ptr, slice,
};

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum StartupError {
    MissingEntry,
    MissingProgramHeaders,
    InvalidAddress,
    InvalidPageSize,
    TooManyArguments,
    StackTooSmall,
    CleanupUnavailable(CleanupError),
}

/// Platform values and strings copied into the new application's stack.
/// `page_size` is the platform's ELF ABI page granularity, not a claim that
/// the flat backend implements page protection. Optional random bytes must
/// come from a suitable entropy source when the selected libc requires them.
pub struct StartupArguments<'a> {
    pub argv: &'a [&'a CStr],
    pub envp: &'a [&'a CStr],
    pub page_size: usize,
    pub execfn: Option<&'a CStr>,
    pub random: Option<&'a [u8; 16]>,
}

/// Owns an image and a stack containing its native ELF startup frame.
/// Strings, pointer arrays and auxiliary data all live in that allocation.
/// Moving the owner preserves every exported address. This is a stack-layout
/// adapter, not a claim of complete Linux/glibc runtime compatibility.
#[must_use = "retain the image and stack until application execution finishes"]
pub struct StartupStorage<A: ImageMapping> {
    image: LoadedImage<A>,
    stack: Stack,
    words: usize,
}

impl<A: ImageMapping> StartupStorage<A> {
    pub fn new(
        image: LoadedImage<A>,
        mut stack: Stack,
        args: StartupArguments<'_>,
    ) -> Result<Self, StartupError> {
        let entry = image.entry().ok_or(StartupError::MissingEntry)?;
        let entry = usize::try_from(entry.get()).map_err(|_| StartupError::InvalidAddress)?;
        let phdr = image
            .program_headers()
            .ok_or(StartupError::MissingProgramHeaders)?;
        let phdr_address =
            usize::try_from(phdr.address().get()).map_err(|_| StartupError::InvalidAddress)?;
        // The kernel's load_image profile guarantees the native machine and
        // byte order; this additional check catches a foreign word-size table.
        let native_phent = if size_of::<usize>() == 8 { 56 } else { 32 };
        if usize::from(phdr.entry_size()) != native_phent
            || !phdr_address.is_multiple_of(size_of::<usize>())
        {
            return Err(StartupError::InvalidAddress);
        }
        if args.argv.len() > c_int::MAX as usize {
            return Err(StartupError::TooManyArguments);
        }
        if !args.page_size.is_power_of_two() {
            return Err(StartupError::InvalidPageSize);
        }
        let overflow = || StartupError::StackTooSmall;
        let pairs = 7 + usize::from(args.execfn.is_some()) + usize::from(args.random.is_some());
        let words = args
            .argv
            .len()
            .checked_add(args.envp.len())
            .and_then(|n| n.checked_add(3 + pairs * 2))
            .ok_or_else(overflow)?;
        let mut bytes = words.checked_mul(size_of::<usize>()).ok_or_else(overflow)?;
        for string in args
            .argv
            .iter()
            .chain(args.envp)
            .copied()
            .chain(args.execfn)
        {
            bytes = bytes
                .checked_add(string.to_bytes_with_nul().len())
                .ok_or_else(overflow)?;
        }
        if args.random.is_some() {
            bytes = bytes.checked_add(16).ok_or_else(overflow)?;
        }
        let top = stack.top();
        let sp = top.addr().checked_sub(bytes).ok_or_else(overflow)? & !(STACK_ALIGNMENT - 1);
        let reserved = top.addr().checked_sub(sp).ok_or_else(overflow)?;
        let frame = stack
            .reserve_top(reserved)
            .ok_or_else(overflow)?
            .cast::<usize>();
        // SAFETY: reserve_top retains the allocation and guarantees the complete
        // payload lies above the reduced execution-stack top. Every word is
        // initialized below before any shared view or application can read it.
        let frame = unsafe {
            ptr::write_bytes(frame, 0, words);
            slice::from_raw_parts_mut(frame, words)
        };
        let mut payload = top;
        frame[0] = args.argv.len();
        let mut index = 1;
        for string in args.argv {
            frame[index] = unsafe { copy_string(&mut payload, string) };
            index += 1;
        }
        frame[index] = 0;
        index += 1;
        for string in args.envp {
            frame[index] = unsafe { copy_string(&mut payload, string) };
            index += 1;
        }
        frame[index] = 0;
        index += 1;
        let mut push = |key, value| {
            frame[index] = key;
            frame[index + 1] = value;
            index += 2;
        };
        push(auxv::AT_PHDR, phdr_address);
        push(auxv::AT_PHENT, usize::from(phdr.entry_size()));
        push(auxv::AT_PHNUM, usize::from(phdr.count()));
        push(auxv::AT_ENTRY, entry);
        // This adapter starts a standalone image, without an ELF interpreter.
        push(auxv::AT_BASE, 0);
        push(auxv::AT_PAGESZ, args.page_size);
        if let Some(name) = args.execfn {
            push(auxv::AT_EXECFN, unsafe { copy_string(&mut payload, name) });
        }
        if let Some(random) = args.random {
            // SAFETY: the byte budget includes these 16 disjoint payload bytes.
            unsafe {
                payload = payload.sub(16);
                ptr::copy_nonoverlapping(random.as_ptr(), payload, 16);
            }
            push(auxv::AT_RANDOM, payload as usize);
        }
        push(auxv::AT_NULL, 0);
        debug_assert_eq!(index, words);
        debug_assert!(payload.addr() >= sp + words * size_of::<usize>());
        Ok(Self {
            image,
            stack,
            words,
        })
    }

    pub fn image(&self) -> &LoadedImage<A> {
        &self.image
    }
    pub fn initial_sp(&self) -> *const usize {
        self.stack.top().cast()
    }
    pub fn initial_words(&self) -> &[usize] {
        // SAFETY: construction initialized this exact frame, retained by self.
        unsafe { slice::from_raw_parts(self.initial_sp(), self.words) }
    }

    /// Build an unscheduled RTOS thread. The caller can set its priority and
    /// enqueue it through the existing scheduler API. Retiring the thread
    /// posts image cleanup to an ordinary kernel worker; the thread owns the
    /// whole stack. RETIRED alone does not mean image cleanup has completed.
    ///
    /// # Safety
    /// The image must be trusted native code for this kernel, use the ELF
    /// initial-stack entry convention, and never return from its entry. Its
    /// allocator must expose executable backing in the kernel's flat address
    /// space. The caller must satisfy code publication on every CPU where it
    /// may run, and ensure no image code/data users survive this thread. This
    /// single-thread adapter does not implement Linux syscalls or child-thread
    /// ownership; those require a runtime with a longer-lived image owner.
    #[must_use = "enqueue the returned thread or explicitly cancel and unregister it"]
    pub unsafe fn into_thread(self) -> Result<ThreadNode, StartupError>
    where
        A: Send + 'static,
    {
        let data = Box::new(EntryData {
            sp: self.stack.top() as usize,
            image: self.image,
        });
        let pointer = ptr::from_ref(data.as_ref()).cast_mut().cast::<c_void>();
        // Prepare before registering a thread, so failure drops the image and
        // stack in the caller's context without leaving a global queue entry.
        let cleanup =
            DeferredCleanup::new(move || drop(data)).map_err(StartupError::CleanupUnavailable)?;
        let thread = Builder::new(Entry::Posix(bootstrap::<A>, pointer))
            .set_stack(self.stack)
            .build();
        thread.lock().set_deferred_cleanup(cleanup);
        Ok(thread)
    }
}

// Held by the thread's cleanup hook, not by an abandoned bootstrap stack frame.
#[must_use = "the thread cleanup hook retains this image through execution"]
struct EntryData<A: ImageMapping> {
    image: LoadedImage<A>,
    sp: usize,
}

extern "C" fn bootstrap<A: ImageMapping>(pointer: *mut c_void) {
    // SAFETY: into_thread keeps this immutable owner alive until retirement.
    let data = unsafe { &*pointer.cast::<EntryData<A>>() };
    let allocation = data.image.allocation();
    // Refresh the current CPU after scheduling as well. This does not replace
    // the caller's host-wide publication duty for subsequent migration.
    if KernelCodeCache::new()
        .synchronize(TargetRange::new(allocation.base(), allocation.len()))
        .is_err()
    {
        scheduler::retire_me();
    }
    let entry = data.image.entry().unwrap().get() as usize;
    // SAFETY: the caller authorizes native execution; the image and the
    // original stack allocation are retained through thread retirement.
    unsafe { enter_elf(entry, data.sp) }
}

unsafe fn copy_string(cursor: &mut *mut u8, string: &CStr) -> usize {
    let bytes = string.to_bytes_with_nul();
    // SAFETY: the caller budgets this string in the disjoint payload region.
    unsafe {
        *cursor = cursor.sub(bytes.len());
        ptr::copy_nonoverlapping(bytes.as_ptr(), *cursor, bytes.len());
    }
    *cursor as usize
}

#[cfg(any(target_arch = "riscv32", target_arch = "riscv64"))]
#[unsafe(naked)]
unsafe extern "C" fn enter_elf(_entry: usize, _sp: usize) -> ! {
    core::arch::naked_asm!("mv t0, a0", "mv sp, a1", "li a0, 0", "li ra, 0", "jr t0");
}

#[cfg(target_arch = "aarch64")]
#[unsafe(naked)]
unsafe extern "C" fn enter_elf(_entry: usize, _sp: usize) -> ! {
    core::arch::naked_asm!(
        "mov x16, x0",
        "mov sp, x1",
        "mov x0, xzr",
        "mov x30, xzr",
        "br x16"
    );
}
