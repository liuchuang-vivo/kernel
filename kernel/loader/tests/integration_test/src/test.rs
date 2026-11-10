// NEWLINE-TIMEOUT: 10
// ASSERT-SUCC: Loader integration test ended
// ASSERT-FAIL: Backtrace in Panic.*

#![no_main]
#![no_std]
#![feature(custom_test_frameworks)]
#![test_runner(loader_test_runner)]
#![reexport_test_harness_main = "loader_test_main"]

extern crate alloc;
extern crate rsrt;

mod runner;
use blueos::loader::adapters::{FlatImageAllocation, SliceElfReader};
use runner::FileReader;
// Import it just for the global allocator.
use alloc::{boxed::Box, sync::Arc, vec::Vec};
use blueos_loader as loader;
use core::{
    ffi::{c_char, CStr},
    sync::atomic::{AtomicBool, AtomicUsize, Ordering},
};
use librs::pthread;
use semihosting::{io::Read, println};

unsafe extern "C" {
    static LOADER_TEST_ELF_PATH: *const c_char;
    static INVALID_MAGIC_ELF_PATH: *const c_char;
    static INVALID_ENTRY_ELF_PATH: *const c_char;
    static INVALID_SEGMENT_SIZE_ELF_PATH: *const c_char;
}

fn read_all(ptr: *const core::ffi::c_char) -> semihosting::io::Result<Vec<u8>> {
    let path = unsafe { core::ffi::CStr::from_ptr(ptr) };
    let mut file = semihosting::fs::File::open(path)?;
    // Debug fixtures are several MiB and the boards' heap is small, so size
    // the buffer up front: growing it would hold the old and new blocks at
    // once.
    let len = usize::try_from(file.metadata()?.len()).unwrap_or(0);
    let mut buf = Vec::with_capacity(len);
    let mut tmp = [0u8; 64];
    loop {
        let size = file.read(&mut tmp)?;
        if size == 0 {
            break;
        }
        buf.extend_from_slice(&tmp[..size]);
    }
    Ok(buf)
}

mod test_elf_loader {
    #[cfg(loader_test_exec)]
    use super::runner::fixed::{TEST_REGION_END, TEST_REGION_START};
    use super::*;
    use blueos_test_macro::test;

    #[cfg(loader_test_exec)]
    const EXPECTED_RESULT: u32 = 0x9afc_e987;

    fn run(image: &loader::LoadedImage<FlatImageAllocation>) {
        // These fixtures do not define runtime lifecycle hooks.
        assert!(image.lifecycle().preinit().is_empty());
        assert!(image.lifecycle().init().is_empty());
        assert!(image.lifecycle().fini().is_empty());
        let entry = usize::try_from(image.entry().expect("application entry").get()).unwrap();
        #[cfg(loader_test_exec)]
        {
            // SAFETY: the configured fixture declares a synchronous C entry
            // returning u32; image and the pinned cache context remain alive.
            let function: extern "C" fn() -> u32 = unsafe { core::mem::transmute(entry) };
            assert_eq!(function(), EXPECTED_RESULT);
        }
        #[cfg(not(loader_test_exec))]
        {
            // SAFETY: the PIC fixture declares a synchronous C fn() entry;
            // image retains its initialized backing in the local cache context.
            let function: extern "C" fn() = unsafe { core::mem::transmute(entry) };
            function();
        }
    }

    // The release-only slice case retains the complete file, including debug
    // sections. The positional reader case below also runs in debug builds.
    // TODO(paging): Re-enable once the loader uses executable page allocations;
    // ordinary heap allocations must remain non-executable.
    #[cfg(all(not(debug_assertions), not(paging)))]
    #[test]
    fn test_load_elf_and_run() {
        let bytes = read_all(unsafe { LOADER_TEST_ELF_PATH }).unwrap();
        let _pinned = blueos::support::DisableInterruptGuard::new();
        let image =
            blueos::loader::load_image(SliceElfReader::new(&bytes)).expect("load ELF from memory");
        run(&image);
    }

    // A direct RTOS thread without a Process uses the kernel's production
    // reader, flat allocator and cache backend. Loading and handing off the
    // image must not depend on Process/VFS state. Both ELF types go through
    // the same entry; fixed fixtures use the shared platform table.
    #[test]
    fn test_load_elf_in_unowned_rtos_thread() {
        let bytes = read_all(unsafe { LOADER_TEST_ELF_PATH }).unwrap();

        let done = Arc::new(AtomicUsize::new(0));
        let loaded = Arc::new(AtomicBool::new(false));
        let done_w = done.clone();
        let loaded_w = loaded.clone();
        // The default thread stack is too small for ELF inspection on 32-bit
        // release builds; the load runs in a thread with the app stack size.
        let stack = blueos::thread::Stack::from_size(16 << 10).expect("loader test stack");
        let mut t =
            blueos::thread::Builder::new(blueos::thread::Entry::Closure(Box::new(move || {
                {
                    // The fixture executes synchronously on this CPU. Keep its
                    // local cache publication and execution in one context.
                    let _pinned = blueos::support::DisableInterruptGuard::new();
                    let image = blueos::loader::load_image(SliceElfReader::new(&bytes))
                        .expect("load ELF in an unowned RTOS thread");
                    run(&image);
                }
                loaded_w.store(true, Ordering::Release);
                // Return normally: run_closure releases this closure and its
                // captures before retiring the thread.
            })))
            .set_stack(stack)
            .build();
        assert!(t.process().is_none());
        // This hook runs after the entry closure and its captures are dropped.
        t.lock()
            .set_cleanup(blueos::thread::Entry::Closure(Box::new(move || {
                done_w.store(1, Ordering::Release);
                blueos::sync::wake(&done_w);
            })));
        assert_eq!(
            blueos::scheduler::queue_ready_thread(blueos::thread::IDLE, t),
            Ok(())
        );
        blueos::sync::wait_until(1, &done);
        assert!(loaded.load(Ordering::Acquire));
        assert_eq!(Arc::strong_count(&loaded), 1);
    }

    #[test]
    fn test_load_elf_from_file_and_run() {
        let input = open(unsafe { LOADER_TEST_ELF_PATH });
        let _pinned = blueos::support::DisableInterruptGuard::new();
        let image =
            blueos::loader::load_image(input).expect("load ELF through positional file reads");
        run(&image);
    }

    fn open(ptr: *const c_char) -> FileReader {
        // SAFETY: the linker-generated test symbols point to live NUL-terminated paths.
        FileReader::open(unsafe { CStr::from_ptr(ptr) }).expect("open ELF fixture")
    }

    #[test]
    fn test_invalid_entry() {
        let input = open(unsafe { INVALID_ENTRY_ELF_PATH });
        let _pinned = blueos::support::DisableInterruptGuard::new();
        assert!(blueos::loader::load_image(input).is_err());
    }

    #[test]
    fn test_invalid_magic() {
        let input = open(unsafe { INVALID_MAGIC_ELF_PATH });
        let _pinned = blueos::support::DisableInterruptGuard::new();
        assert!(blueos::loader::load_image(input).is_err());
    }

    #[test]
    fn test_invalid_segment_size() {
        let input = open(unsafe { INVALID_SEGMENT_SIZE_ELF_PATH });
        let _pinned = blueos::support::DisableInterruptGuard::new();
        assert!(blueos::loader::load_image(input).is_err());
    }

    // These fixed fixtures are ELF32 little-endian images produced by exec_app.
    #[cfg(loader_test_exec)]
    fn exec_word(bytes: &[u8], offset: usize) -> u32 {
        u32::from_le_bytes(bytes[offset..offset + 4].try_into().unwrap())
    }

    #[cfg(loader_test_exec)]
    fn set_exec_word(bytes: &mut [u8], offset: usize, value: u32) {
        bytes[offset..offset + 4].copy_from_slice(&value.to_le_bytes());
    }

    #[cfg(loader_test_exec)]
    fn exec_load_headers(bytes: &[u8]) -> Vec<usize> {
        assert_eq!(bytes[4], 1);
        assert_eq!(bytes[5], 1);
        let start = exec_word(bytes, 28) as usize;
        let stride = u16::from_le_bytes(bytes[42..44].try_into().unwrap()) as usize;
        let count = u16::from_le_bytes(bytes[44..46].try_into().unwrap()) as usize;
        (0..count)
            .map(|index| start + index * stride)
            .filter(|&offset| exec_word(bytes, offset) == 1)
            .collect()
    }

    #[cfg(loader_test_exec)]
    #[test]
    fn test_exec_rejects_unregistered_addresses() {
        let mut bytes = read_all(unsafe { LOADER_TEST_ELF_PATH }).unwrap();
        let delta = u32::try_from(TEST_REGION_END - TEST_REGION_START).unwrap();
        let entry = exec_word(&bytes, 24).checked_add(delta).unwrap();
        set_exec_word(&mut bytes, 24, entry);
        for header in exec_load_headers(&bytes) {
            for field in [8, 12] {
                let value = exec_word(&bytes, header + field)
                    .checked_add(delta)
                    .unwrap();
                set_exec_word(&mut bytes, header + field, value);
            }
        }
        let error = blueos::loader::load_image(SliceElfReader::new(&bytes))
            .err()
            .expect("unregistered placement must fail");
        assert_eq!(error.stage(), Some(loader::error::LoadStage::Allocate));
        assert_eq!(error.kind(), loader::error::LoadErrorKind::OutOfBounds);
    }

    #[cfg(loader_test_exec)]
    #[test]
    fn test_exec_rejects_out_of_range_without_writing() {
        let mut bytes = read_all(unsafe { LOADER_TEST_ELF_PATH }).unwrap();
        let header = *exec_load_headers(&bytes).last().unwrap();
        let size = exec_word(&bytes, header + 20)
            .checked_add(u32::try_from(TEST_REGION_END - TEST_REGION_START).unwrap())
            .unwrap();
        set_exec_word(&mut bytes, header + 20, size);
        let before = unsafe { (TEST_REGION_START as *const u32).read_volatile() };
        let error = blueos::loader::load_image(SliceElfReader::new(&bytes))
            .err()
            .expect("crossing the window boundary must fail");
        assert_eq!(error.stage(), Some(loader::error::LoadStage::Allocate));
        assert_eq!(error.kind(), loader::error::LoadErrorKind::OutOfBounds);
        let after = unsafe { (TEST_REGION_START as *const u32).read_volatile() };
        assert_eq!(after, before);
    }

    #[cfg(loader_test_exec)]
    #[test]
    fn test_exec_releases_fixed_region_after_image_drop() {
        let bytes = read_all(unsafe { LOADER_TEST_ELF_PATH }).unwrap();
        let _pinned = blueos::support::DisableInterruptGuard::new();
        let image =
            blueos::loader::load_image(SliceElfReader::new(&bytes)).expect("first fixed mapping");
        run(&image);
        blueos::loader::init_fixed_regions(&runner::fixed::TEST_REGIONS).unwrap();
        assert!(blueos::loader::load_image(SliceElfReader::new(&bytes)).is_err());
        drop(image);
        let image = blueos::loader::load_image(SliceElfReader::new(&bytes))
            .expect("fixed region released and prepared again");
        run(&image);
    }
}
#[unsafe(no_mangle)]
pub fn loader_test_runner(tests: &[&dyn Fn()]) {
    println!("Loader integration test started");
    println!("Running {} tests", tests.len());
    for test in tests {
        test();
    }
    println!("Loader integration test ended");
}
#[unsafe(no_mangle)]
pub extern "C" fn main() -> i32 {
    pthread::register_my_posix_tcb();
    #[cfg(loader_test_exec)]
    blueos::loader::init_fixed_regions(&runner::fixed::TEST_REGIONS)
        .expect("validate and install the platform EXEC window");
    loader_test_main();
    #[cfg(coverage)]
    common_cov::write_coverage_data();
    0
}
