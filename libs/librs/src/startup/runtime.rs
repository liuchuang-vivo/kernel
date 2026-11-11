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

//! Startup for a self-contained ELF with its own statically linked librs.
//! This copy of libc and its globals belong to one single-thread application.
//! Shared libc, interpreter startup and process-wide termination are separate
//! integration work; the current kernel exit operation retires one RTOS thread.

use super::StartupInfo;
use crate::{errno::ERRNO, sync::GenericMutex};
use alloc::vec::Vec;
use blueos_header::{startup::auxv, syscalls::NR::ExitThread};
use blueos_scal::bk_syscall;
use core::{
    ffi::{c_char, c_int, c_ulong, c_void},
    mem, ptr,
    sync::atomic::{AtomicBool, AtomicPtr, Ordering},
};

type Main = unsafe extern "C" fn(c_int, *mut *mut c_char, *mut *mut c_char) -> c_int;
type Init = unsafe extern "C" fn(c_int, *mut *mut c_char, *mut *mut c_char);
type Fini = unsafe extern "C" fn();

#[unsafe(no_mangle)]
pub static mut environ: *mut *mut c_char = ptr::null_mut();
static AUXV: AtomicPtr<usize> = AtomicPtr::new(ptr::null_mut());
static STARTED: AtomicBool = AtomicBool::new(false);
static EXITING: AtomicBool = AtomicBool::new(false);
static CALLBACKS: GenericMutex<Vec<Fini>> = GenericMutex::new(Vec::new());

unsafe extern "C" {
    // Linker-provided byte markers are only addressed, never read as u8.
    // Ordinary static users do not reference this application entry.
    static __preinit_array_start: u8;
    static __preinit_array_end: u8;
    static __init_array_start: u8;
    static __init_array_end: u8;
    static __fini_array_start: u8;
    static __fini_array_end: u8;
    #[linkage = "extern_weak"]
    static _init: Option<Init>;
    #[linkage = "extern_weak"]
    static _fini: Option<Fini>;
}

/// The native seven-argument libc entry convention used by the application CRT.
/// A non-null legacy init/fini callback replaces the corresponding array walker.
/// Relocation is already complete; libc alone executes this image's callbacks.
///
/// # Safety
/// The caller must enter exactly once, on the application's sole thread, with
/// its own statically linked copy of librs. `argv - 1` must identify the live,
/// native ELF frame described by StartupInfo::from_raw, with matching `argc`.
/// Strings and arrays remain mapped through exit; main may modify argv/envp
/// after decoding finishes. The linker arrays and supplied callbacks must be
/// valid native functions, and no execution or data users may outlive exit.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn __libc_start_main(
    main: Main,
    argc: c_int,
    argv: *mut *mut c_char,
    init: Option<Init>,
    fini: Option<Fini>,
    rtld_fini: Option<Fini>,
    _stack_end: *mut c_void,
) -> ! {
    if argc < 0 || argv.is_null() || STARTED.swap(true, Ordering::AcqRel) {
        retire();
    }
    // SAFETY: the CRT supplies the complete standard frame and its argv slot.
    let decoded = unsafe { StartupInfo::from_raw(argv.cast::<usize>().sub(1)) };
    let Some(decoded) = decoded.filter(|info| info.argc() == argc) else {
        retire();
    };
    AUXV.store(decoded.auxv_ptr().cast_mut(), Ordering::Release);
    // No references into argv/envp survive application callbacks.
    drop(decoded);
    // Derive the writable array from the CRT's original pointer, rather than
    // casting a pointer obtained from the decoder's shared slice to mutable.
    let envp = unsafe { argv.add(argc as usize + 1) };
    unsafe { environ = envp };
    crate::pthread::register_my_posix_tcb();
    crate::stdio::init();
    if let Some(callback) = rtld_fini
        && unsafe { atexit(callback) } != 0
    {
        unsafe { exit(127) };
    }
    if unsafe { atexit(fini.unwrap_or(call_fini)) } != 0 {
        unsafe { exit(127) };
    }
    // SAFETY: this standalone path has no interpreter running preinit/init.
    unsafe {
        run_init_array(
            &raw const __preinit_array_start,
            &raw const __preinit_array_end,
            argc,
            argv,
            envp,
        );
        if let Some(init) = init {
            init(argc, argv, envp);
        } else {
            if let Some(callback) = _init {
                callback(argc, argv, envp);
            }
            run_init_array(
                &raw const __init_array_start,
                &raw const __init_array_end,
                argc,
                argv,
                envp,
            );
        }
        exit(main(argc, argv, envp));
    }
}

/// Register a callback in this application's libc. No callback runs under the
/// list lock, so callbacks may register further callbacks during termination.
///
/// # Safety
/// The callback and everything it accesses must remain valid until application
/// termination, where it is called without arguments.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn atexit(callback: Fini) -> c_int {
    let mut callbacks = CALLBACKS.lock();
    if callbacks.try_reserve(1).is_err() {
        return -1;
    }
    callbacks.push(callback);
    0
}

#[unsafe(no_mangle)]
pub extern "C" fn getauxval(key: c_ulong) -> c_ulong {
    let mut pair = AUXV.load(Ordering::Acquire);
    while !pair.is_null() {
        // SAFETY: startup retains the immutable auxv through all exit callbacks.
        let tag = unsafe { pair.read() };
        if tag == auxv::AT_NULL {
            break;
        }
        if tag == key as usize {
            return unsafe { pair.add(1).read() as c_ulong };
        }
        pair = unsafe { pair.add(2) };
    }
    ERRNO.set(libc::ENOENT);
    0
}

/// Terminate the current standalone, single-thread ELF application. This is
/// not process-wide exit: child-thread quiescence and status reporting require
/// the later kernel application owner. Explicit exit and main return share it.
///
/// # Safety
/// This must be the sole thread of an application started by __libc_start_main,
/// with no surviving users of its image, streams or thread-local state.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn exit(_status: c_int) -> ! {
    if EXITING.swap(true, Ordering::AcqRel) {
        retire();
    }
    loop {
        let callback = CALLBACKS.lock().pop();
        let Some(callback) = callback else { break };
        // SAFETY: only valid application callback pointers are registered.
        unsafe { callback() };
    }
    // Release the Vec buffer before this image can be unmapped.
    drop(mem::take(&mut *CALLBACKS.lock()));
    crate::pthread::run_my_key_destructors();
    // SAFETY: this path requires that no other threads or stream users remain.
    unsafe { crate::stdio::finish() };
    crate::pthread::cleanup_my_tcb();
    AUXV.store(ptr::null_mut(), Ordering::Release);
    unsafe { environ = ptr::null_mut() };
    retire()
}

unsafe fn run_init_array(
    start: *const u8,
    end: *const u8,
    argc: c_int,
    argv: *mut *mut c_char,
    envp: *mut *mut c_char,
) {
    let mut current = start.cast::<usize>();
    while current.addr() < end.addr() {
        // SAFETY: the linker supplies a contiguous, relocated function array.
        let address = unsafe { current.read() };
        if address != 0 && address != usize::MAX {
            let callback: Init = unsafe { mem::transmute(address) };
            unsafe { callback(argc, argv, envp) };
        }
        current = unsafe { current.add(1) };
    }
}

unsafe extern "C" fn call_fini() {
    let start = &raw const __fini_array_start;
    let mut end = (&raw const __fini_array_end).cast::<usize>();
    while end.addr() > start.addr() {
        end = unsafe { end.sub(1) };
        let address = unsafe { end.read() };
        if address != 0 && address != usize::MAX {
            let callback: Fini = unsafe { mem::transmute(address) };
            unsafe { callback() };
        }
    }
    if let Some(callback) = unsafe { _fini } {
        unsafe { callback() };
    }
}

fn retire() -> ! {
    bk_syscall!(ExitThread);
    unreachable!("ExitThread returned")
}
