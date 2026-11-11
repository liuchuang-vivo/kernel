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

use alloc::{boxed::Box, sync::Arc};
use blueos::{
    loader::{StartupArguments, StartupError, StartupStorage},
    thread::{Entry, Stack},
};
use blueos_header::startup::{auxv, STACK_ALIGNMENT};
use blueos_test_macro::test;
use core::{
    ffi::{c_char, CStr},
    ptr,
    sync::atomic::{AtomicBool, Ordering},
};
use librs::startup::StartupInfo;

unsafe extern "C" {
    static STARTUP_TEST_ELF_PATH: *const c_char;
}

fn prepare(
    stack_size: usize,
    page_size: usize,
) -> Result<StartupStorage<blueos::loader::adapters::FlatImageAllocation>, StartupError> {
    // SAFETY: the linker provides this live host fixture path.
    let path = unsafe { CStr::from_ptr(STARTUP_TEST_ELF_PATH) };
    let reader = super::runner::FileReader::open(path).unwrap();
    let image = blueos::loader::load_image(reader).unwrap();
    let random = core::array::from_fn(|index| index as u8);
    StartupStorage::new(
        image,
        Stack::from_size(stack_size).unwrap(),
        StartupArguments {
            argv: &[c"startup-probe", c"pending", c""],
            envp: &[c"MODE=test"],
            page_size,
            execfn: Some(c"/startup-probe"),
            random: Some(&random),
        },
    )
}

#[test]
fn test_startup_standard_stack_round_trip() {
    let storage = prepare(16 * 1024, 4096).unwrap();
    let sp = storage.initial_sp();
    let storage = Box::new(storage);
    assert_eq!(storage.initial_sp(), sp);
    assert!(sp.addr().is_multiple_of(STACK_ALIGNMENT));
    // SAFETY: storage retains the complete initialized ELF frame.
    let decoded = unsafe { StartupInfo::from_raw(sp) }.unwrap();
    assert_eq!(decoded.argc(), 3);
    // SAFETY: these addresses refer to copied, retained stack strings.
    unsafe {
        assert_eq!(
            CStr::from_ptr(decoded.argv()[0] as *const c_char),
            c"startup-probe"
        );
        assert_eq!(
            CStr::from_ptr(decoded.argv()[1] as *const c_char),
            c"pending"
        );
        assert_eq!(CStr::from_ptr(decoded.argv()[2] as *const c_char), c"");
        assert_eq!(
            CStr::from_ptr(decoded.envp()[0] as *const c_char),
            c"MODE=test"
        );
        assert!(decoded.argv_ptr().add(3).read().is_null());
        assert!(decoded.envp_ptr().add(1).read().is_null());
        let name = decoded.aux_value(auxv::AT_EXECFN).unwrap();
        assert_eq!(CStr::from_ptr(name as *const c_char), c"/startup-probe");
        let random = decoded.aux_value(auxv::AT_RANDOM).unwrap() as *const u8;
        for index in 0..16 {
            assert_eq!(random.add(index).read(), index as u8);
        }
    }
    let phdr = storage.image().program_headers().unwrap();
    assert_eq!(
        decoded.aux_value(auxv::AT_PHDR),
        Some(phdr.address().get() as usize)
    );
    assert_eq!(
        decoded.aux_value(auxv::AT_PHENT),
        Some(usize::from(phdr.entry_size()))
    );
    assert_eq!(
        decoded.aux_value(auxv::AT_PHNUM),
        Some(usize::from(phdr.count()))
    );
    assert_eq!(
        decoded.aux_value(auxv::AT_ENTRY),
        Some(storage.image().entry().unwrap().get() as usize)
    );
    assert_eq!(decoded.aux_value(auxv::AT_BASE), Some(0));
    assert_eq!(decoded.aux_value(auxv::AT_PAGESZ), Some(4096));
    assert_eq!(decoded.auxv().last().unwrap().key, auxv::AT_NULL);
    assert_eq!(
        StartupInfo::from_words(storage.initial_words())
            .unwrap()
            .argc(),
        3
    );
}

#[test]
fn test_startup_bounded_decoder_rejects_bad_frames() {
    assert!(StartupInfo::from_words(&[0, 0, 0, 0, 0]).is_some());
    for words in [
        &[][..],
        &[usize::MAX],
        &[1, 0, 0, 0, 0, 0],
        &[1, 0x1000, 1],
        &[0, 0, 1],
        &[0, 0, 0],
        &[0, 0, 0, auxv::AT_ENTRY],
        &[0, 0, 0, auxv::AT_ENTRY, 1],
        &[0, 0, 0, auxv::AT_NULL],
    ] {
        assert!(StartupInfo::from_words(words).is_none());
    }
    // SAFETY: invalid pointer shapes are rejected without reading them.
    assert!(unsafe { StartupInfo::from_raw(ptr::null()) }.is_none());
    let words = [0usize; 8];
    let bad = words.as_ptr().cast::<u8>().wrapping_add(1).cast();
    assert!(unsafe { StartupInfo::from_raw(bad) }.is_none());
}

#[test]
fn test_startup_rejects_insufficient_stack_and_invalid_page_size() {
    assert!(matches!(
        prepare(64, 4096),
        Err(StartupError::StackTooSmall)
    ));
    assert!(matches!(
        prepare(16 * 1024, 0),
        Err(StartupError::InvalidPageSize)
    ));
    assert!(matches!(
        prepare(16 * 1024, 3),
        Err(StartupError::InvalidPageSize)
    ));
}

#[test]
fn test_startup_enters_independent_elf_in_rtos_thread() {
    let storage = prepare(16 * 1024, 4096).unwrap();
    // argv[1] is writable application argument storage. Keep the ThreadNode
    // alive after retirement so its original stack allocation stays mapped.
    let result = storage.initial_words()[2] as *const c_char;
    // SAFETY: this trusted, single-thread probe implements native ELF entry,
    // checks the frame independently, and exits using BlueKernel's syscall.
    // QEMU's coherent test execution and bootstrap refresh publish its code.
    let thread = unsafe { storage.into_thread() };
    assert!(thread.process().is_none());
    let cleanup = thread.lock().take_cleanup().unwrap();
    let done = Arc::new(AtomicBool::new(false));
    let finished = done.clone();
    thread.lock().set_cleanup(Entry::Closure(Box::new(move || {
        let Entry::Closure(cleanup) = cleanup else {
            panic!("startup cleanup");
        };
        cleanup();
        // The scheduler calls this after execution has stopped. Avoid another
        // scheduler operation from inside the retirement cleanup callback.
        finished.store(true, Ordering::Release);
    })));
    assert_eq!(
        blueos::scheduler::queue_ready_thread(blueos::thread::IDLE, thread.clone()),
        Ok(())
    );
    while !done.load(Ordering::Acquire) {
        blueos::scheduler::yield_me();
    }
    assert_eq!(thread.state(), blueos::thread::RETIRED);
    // SAFETY: cleanup published completion and this node still owns its stack.
    assert_eq!(unsafe { CStr::from_ptr(result) }, c"passed!");
}
