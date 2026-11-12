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

//! Ordinary Rust main linked with the production initial-stack CRT and librs.
extern crate librs;

use blueos_header::startup::auxv;
use std::{
    cell::RefCell,
    ffi::{c_char, c_int, CStr},
    process::ExitCode,
    ptr,
    sync::atomic::{AtomicPtr, AtomicUsize, Ordering},
    thread,
};

static RESULT: AtomicPtr<u8> = AtomicPtr::new(ptr::null_mut());
static LENGTH: AtomicUsize = AtomicUsize::new(0);
fn event(byte: u8) {
    let offset = LENGTH.fetch_add(1, Ordering::Relaxed);
    assert!(offset < 31);
    let result = RESULT.load(Ordering::Acquire);
    assert!(!result.is_null());
    // The test supplies a writable argument retained through application exit.
    unsafe {
        result.add(offset).write(byte);
        result.add(offset + 1).write(0);
    }
}

unsafe extern "C" fn init(argc: c_int, argv: *mut *mut c_char, envp: *mut *mut c_char) {
    unsafe {
        assert_eq!(argc, 2);
        assert_eq!(CStr::from_ptr(argv.read()), c"rust-std-probe");
        assert_eq!(envp, librs::startup::environ);
        RESULT.store(argv.add(1).read().cast(), Ordering::Release);
        assert_eq!(librs::startup::getauxval(auxv::AT_PAGESZ as _), 4096);
    }
    event(b'I');
}
#[used]
#[unsafe(link_section = ".init_array")]
static INIT: unsafe extern "C" fn(c_int, *mut *mut c_char, *mut *mut c_char) = init;

#[must_use]
struct ThreadLocal;
impl Drop for ThreadLocal {
    fn drop(&mut self) {
        event(b'D');
    }
}
thread_local! { static LOCAL: RefCell<Option<ThreadLocal>> = const { RefCell::new(None) }; }

fn main() -> ExitCode {
    event(b'M');
    assert_eq!(std::env::var("MODE").unwrap(), "rust-std");
    let answer = thread::Builder::new()
        .name("elf-worker".into())
        .stack_size(64 * 1024)
        .spawn(|| {
            LOCAL.with(|slot| *slot.borrow_mut() = Some(ThreadLocal));
            let mut values = Vec::new();
            for value in 0..1024 {
                values.push(value);
            }
            let zeroed = vec![0u64; 256];
            assert!(zeroed.iter().all(|value| *value == 0));
            assert_eq!(values.iter().sum::<usize>(), 1023 * 1024 / 2);
            assert_eq!(std::env::var("MODE").unwrap(), "rust-std");
            event(b'T');
            42
        })
        .unwrap()
        .join()
        .unwrap();
    assert_eq!(answer, 42);
    event(b'R');
    ExitCode::from(37)
}

unsafe extern "C" fn fini() {
    event(b'F');
}
#[used]
#[unsafe(link_section = ".fini_array")]
static FINI: unsafe extern "C" fn() = fini;
