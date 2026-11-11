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

//! CRT probe linked against the production standalone librs runtime.
#![no_std]
#![no_main]

use blueos_header::{startup::auxv, syscalls::NR::ExitThread};
use blueos_scal::bk_syscall;
use core::{
    alloc::{GlobalAlloc, Layout},
    ffi::{c_char, c_int, c_void, CStr},
    ptr,
    sync::atomic::{AtomicU8, AtomicUsize, Ordering},
};
use librs::startup::{__libc_start_main, atexit, environ, exit, getauxval};

type Init = unsafe extern "C" fn(c_int, *mut *mut c_char, *mut *mut c_char);
static mut RESULT: *mut u8 = ptr::null_mut();
static mut LENGTH: usize = 0;
static ALLOCATIONS: AtomicUsize = AtomicUsize::new(0);

struct Allocator;
unsafe impl GlobalAlloc for Allocator {
    unsafe fn alloc(&self, layout: Layout) -> *mut u8 {
        let mut result = ptr::null_mut();
        let rc = unsafe {
            librs::stdlib::malloc::posix_memalign(
                &raw mut result,
                layout.align().max(size_of::<*mut c_void>()),
                layout.size(),
            )
        };
        if rc == 0 {
            ALLOCATIONS.fetch_add(1, Ordering::Relaxed);
            result.cast()
        } else {
            ptr::null_mut()
        }
    }
    unsafe fn dealloc(&self, pointer: *mut u8, _layout: Layout) {
        unsafe {
            librs::stdlib::malloc::free(pointer.cast());
            let remaining = ALLOCATIONS.fetch_sub(1, Ordering::AcqRel) - 1;
            if !RESULT.is_null() {
                // Outside the event string: publish the outstanding allocation
                // count while image globals still exist, before kernel cleanup.
                RESULT.add(30).write(u8::try_from(remaining).unwrap());
            }
        };
    }
}
#[global_allocator]
static ALLOCATOR: Allocator = Allocator;

#[cfg(any(target_arch = "riscv32", target_arch = "riscv64"))]
#[unsafe(no_mangle)]
#[unsafe(naked)]
unsafe extern "C" fn _start() -> ! {
    core::arch::naked_asm!(
        ".option push", ".option norelax", "la gp, __global_pointer$", ".option pop",
        "mv a1, a0", "mv a0, sp", "tail {entry}", entry = sym entry,
    );
}
#[cfg(target_arch = "aarch64")]
#[unsafe(no_mangle)]
#[unsafe(naked)]
unsafe extern "C" fn _start() -> ! {
    core::arch::naked_asm!("mov x1, x0", "mov x0, sp", "b {entry}", entry = sym entry);
}

unsafe extern "C" fn entry(sp: *const usize, rtld_fini: Option<unsafe extern "C" fn()>) -> ! {
    let argc = unsafe { sp.read() } as c_int;
    let argv = unsafe { sp.add(1) }.cast_mut().cast::<*mut c_char>();
    let legacy = unsafe { CStr::from_ptr(argv.read()) } == c"legacy";
    // The CRT passes only the native ABI arguments, with no kernel-private plan.
    unsafe {
        __libc_start_main(
            app_main,
            argc,
            argv,
            if legacy { Some(legacy_init) } else { None },
            if legacy { Some(legacy_fini) } else { None },
            rtld_fini,
            sp.cast_mut().cast::<c_void>(),
        )
    }
}

unsafe fn event(byte: u8) {
    // The host supplies a writable 32-byte argument for the event log.
    unsafe {
        assert!(!RESULT.is_null() && LENGTH < 31);
        RESULT.add(LENGTH).write(byte);
        LENGTH += 1;
        RESULT.add(LENGTH).write(0);
    }
}

unsafe extern "C" fn preinit(argc: c_int, argv: *mut *mut c_char, envp: *mut *mut c_char) {
    unsafe {
        RESULT = argv.add(1).read().cast();
        assert_eq!(argc, 3);
        assert_eq!(envp, environ);
        assert_eq!(getauxval(auxv::AT_PAGESZ as _), 4096);
        assert_eq!(getauxval(usize::MAX as _), 0);
        assert_eq!(librs::errno::ERRNO.get(), libc::ENOENT);
        librs::errno::ERRNO.set(17);
        assert_eq!(getauxval(auxv::AT_BASE as _), 0);
        assert_eq!(librs::errno::ERRNO.get(), 17);
        assert_eq!(
            CStr::from_ptr(librs::stdlib::getenv(c"MODE".as_ptr())),
            c"test"
        );
        assert!(librs::stdlib::getenv(c"MISSING".as_ptr()).is_null());
        assert!(librs::stdlib::getenv(c"MODE=x".as_ptr()).is_null());
        assert!(!librs::stdio::stdin.is_null());
        assert!(!librs::stdio::stdout.is_null());
        event(b'P');
    }
}
#[used]
#[unsafe(link_section = ".preinit_array")]
static PREINIT: Init = preinit;

#[unsafe(no_mangle)]
unsafe extern "C" fn _init(_argc: c_int, _argv: *mut *mut c_char, _envp: *mut *mut c_char) {
    unsafe { event(b'I') };
}
unsafe extern "C" fn init_one(_argc: c_int, _argv: *mut *mut c_char, _envp: *mut *mut c_char) {
    unsafe {
        event(b'A');
        assert_eq!(atexit(ctor_exit), 0);
        let mut key = 0;
        assert_eq!(
            librs::pthread::pthread_key_create(&raw mut key, Some(tls_exit)),
            0
        );
        assert_eq!(
            librs::pthread::pthread_setspecific(key, ptr::dangling::<c_void>()),
            0
        );
    }
}
unsafe extern "C" fn init_two(_argc: c_int, _argv: *mut *mut c_char, _envp: *mut *mut c_char) {
    unsafe { event(b'B') };
}
#[used]
#[unsafe(link_section = ".init_array.00100")]
static INIT_ONE: Init = init_one;
#[used]
#[unsafe(link_section = ".init_array.00200")]
static INIT_TWO: Init = init_two;

unsafe extern "C" fn legacy_init(argc: c_int, argv: *mut *mut c_char, envp: *mut *mut c_char) {
    unsafe {
        _init(argc, argv, envp);
        init_one(argc, argv, envp);
        init_two(argc, argv, envp);
    }
}
unsafe extern "C" fn app_main(
    argc: c_int,
    argv: *mut *mut c_char,
    envp: *mut *mut c_char,
) -> c_int {
    unsafe {
        assert_eq!(argc, 3);
        assert_eq!(envp, environ);
        assert_eq!(CStr::from_ptr(argv.add(2).read()), c"");
        // Main can mutate argv after the borrowed decoder is gone.
        argv.add(2).write(argv.read());
        envp.write(envp.read());
        event(b'M');
        assert_eq!(atexit(exit_one), 0);
        assert_eq!(atexit(exit_two), 0);
        let mode = CStr::from_ptr(argv.read());
        if mode == c"exit" {
            exit(23);
        }
        if mode == c"join" || mode == c"detach" {
            let mut child = 0;
            assert_eq!(
                librs::pthread::pthread_create(
                    &raw mut child,
                    ptr::null(),
                    child_main,
                    argv.read().cast(),
                ),
                0
            );
            if mode == c"join" {
                let mut result = ptr::null_mut();
                assert_eq!(librs::pthread::pthread_join(child, &raw mut result), 0);
                assert_eq!(result as usize, 42);
            } else {
                assert_eq!(librs::pthread::pthread_detach(child), 0);
                // The host releases the child only after the kernel records
                // Exiting. Main has already returned when it touches env/auxv.
            }
        }
    }
    17
}
extern "C" fn child_main(argument: *mut c_void) -> *mut c_void {
    unsafe {
        if CStr::from_ptr(argument.cast()) == c"detach" {
            let gate = AtomicU8::from_ptr(RESULT.add(29));
            while gate.load(Ordering::Acquire) != b'G' {
                bk_syscall!(blueos_header::syscalls::NR::SchedYield);
            }
        }
        // Both envp strings and auxv live in the main thread's initial stack.
        assert_eq!(
            CStr::from_ptr(librs::stdlib::getenv(c"MODE".as_ptr())),
            c"test"
        );
        assert_eq!(getauxval(auxv::AT_PAGESZ as _), 4096);
        librs::errno::ERRNO.set(42);
        let mut key = 0;
        assert_eq!(
            librs::pthread::pthread_key_create(&raw mut key, Some(child_tls_exit)),
            0
        );
        assert_eq!(
            librs::pthread::pthread_setspecific(key, ptr::dangling::<c_void>()),
            0
        );
        event(b'H');
    }
    42usize as *mut c_void
}
extern "C" fn child_tls_exit(_value: *mut c_void) {
    unsafe { event(b'K') };
}
extern "C" fn forbidden_child(_argument: *mut c_void) -> *mut c_void {
    panic!("a finalizer must not create a new application member");
}

unsafe extern "C" fn exit_one() {
    unsafe { event(b'1') };
}
unsafe extern "C" fn exit_two() {
    unsafe {
        event(b'2');
        assert_eq!(atexit(exit_three), 0);
    }
}
unsafe extern "C" fn exit_three() {
    unsafe { event(b'3') };
}
unsafe extern "C" fn ctor_exit() {
    unsafe { event(b'c') };
}
extern "C" fn tls_exit(_value: *mut c_void) {
    // A prior emutls destructor may already have run. errno must create fresh
    // storage, and a subsequent destructor pass must release it again.
    assert_eq!(getauxval(usize::MAX as _), 0);
    assert_eq!(librs::errno::ERRNO.get(), libc::ENOENT);
    unsafe { event(b'T') };
}
unsafe extern "C" fn fini_one() {
    unsafe { event(b'X') };
}
unsafe extern "C" fn fini_two() {
    unsafe { event(b'Y') };
}
#[used]
#[unsafe(link_section = ".fini_array.00100")]
static FINI_ONE: unsafe extern "C" fn() = fini_one;
#[used]
#[unsafe(link_section = ".fini_array.00200")]
static FINI_TWO: unsafe extern "C" fn() = fini_two;
#[unsafe(no_mangle)]
unsafe extern "C" fn _fini() {
    unsafe { event(b'F') };
    let mut child = 0;
    assert_eq!(
        librs::pthread::pthread_create(
            &raw mut child,
            ptr::null(),
            forbidden_child,
            ptr::null_mut(),
        ),
        -1
    );
}
unsafe extern "C" fn legacy_fini() {
    unsafe {
        fini_two();
        fini_one();
        _fini();
    }
}

#[panic_handler]
fn panic(_: &core::panic::PanicInfo<'_>) -> ! {
    // A failing probe still retires, allowing the parent to inspect its log.
    bk_syscall!(ExitThread);
    loop {
        core::hint::spin_loop();
    }
}
