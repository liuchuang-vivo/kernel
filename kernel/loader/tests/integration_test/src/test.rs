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
#[cfg(all(not(loader_test_exec), enable_vfs))]
mod shell;
#[cfg(not(loader_test_exec))]
mod startup;
use blueos::loader::adapters::{FlatImageAllocation, SliceElfReader};
#[cfg(enable_vfs)]
use blueos::{loader::adapters::VfsElfReader, vfs::FileOps};
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
    // TODO(paging): the flat allocator dereferences the fixed EXEC VA
    // (0x40280000) directly; under paging that VA is unmapped after
    // `boards::init` drops the boot identity map. Re-enable once the loader
    // test runs against a paging-aware backend.
    #[cfg(not(paging))]
    #[test]
    fn test_load_elf_in_unowned_rtos_thread() {
        let bytes = read_all(unsafe { LOADER_TEST_ELF_PATH }).unwrap();
        run_in_unowned_thread(move || {
            let _pinned = blueos::support::DisableInterruptGuard::new();
            let image = blueos::loader::load_image(SliceElfReader::new(&bytes))
                .expect("load ELF in an unowned RTOS thread");
            run(&image);
        });
    }

    fn run_in_unowned_thread(work: impl FnOnce() + Send + 'static) {
        let done = Arc::new(AtomicUsize::new(0));
        let loaded = Arc::new(AtomicBool::new(false));
        let done_w = done.clone();
        let loaded_w = loaded.clone();
        // The default thread stack is too small for ELF inspection on 32-bit
        // release builds; the load runs in a thread with the app stack size.
        let stack = blueos::thread::Stack::from_size(16 << 10).expect("loader test stack");
        let mut t =
            blueos::thread::Builder::new(blueos::thread::Entry::Closure(Box::new(move || {
                assert!(blueos::process::current_process().is_none());
                work();
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

    // TODO(paging): see `test_load_elf_in_unowned_rtos_thread` — the flat
    // allocator's fixed-VA dereference faults once TTBR0 walks are disabled.
    #[cfg(all(enable_vfs, not(paging)))]
    #[test]
    fn test_load_elf_from_vfs_in_unowned_rtos_thread() {
        const PATH: &str = "/loader-vfs-test.elf";
        // Copy the host fixture into target tmpfs in small chunks. The ELF
        // reader under test then reads the target file directly, without
        // keeping a second complete input buffer or allocating a descriptor.
        let mut source =
            semihosting::fs::File::open(unsafe { CStr::from_ptr(LOADER_TEST_ELF_PATH) })
                .expect("open host ELF fixture");
        let file =
            blueos::vfs::open_path(PATH, libc::O_CREAT | libc::O_TRUNC | libc::O_WRONLY, 0o600)
                .expect("create target ELF file");
        let len = usize::try_from(source.metadata().unwrap().len()).unwrap();
        file.resize(len).expect("size target file before copying");
        let mut copied = 0;
        let mut buffer = [0; 512];
        loop {
            let count = source.read(&mut buffer).unwrap();
            if count == 0 {
                break;
            }
            let mut written = 0;
            while written < count {
                let size = file.write(&buffer[written..count]).unwrap();
                assert!(size != 0);
                written += size;
            }
            copied += count;
        }
        assert_eq!(copied, len);
        drop(file);
        drop(source);

        run_in_unowned_thread(|| {
            let file = blueos::vfs::open_path(PATH, libc::O_RDONLY, 0)
                .expect("open target ELF without a process or fd");
            let reader = VfsElfReader::new(file).expect("readable regular ELF file");
            // This tmpfs file is resident and has no concurrent users. Keep
            // local cache publication and fixture execution on this CPU.
            let _pinned = blueos::support::DisableInterruptGuard::new();
            let image = blueos::loader::load_image(reader).expect("load ELF from target VFS");
            run(&image);
        });
        assert_eq!(
            blueos::vfs::syscalls::unlink(c"/loader-vfs-test.elf".as_ptr()),
            0
        );
    }

    // TODO(paging): see `test_load_elf_in_unowned_rtos_thread`.
    #[cfg(not(paging))]
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

    // exec_app is a fixed-position ET_EXEC image. Its ELF class depends on the
    // target: 32-bit boards (esp32c3 / RV32) produce ELFCLASS32, the 64-bit
    // aarch64 board produces ELFCLASS64. The program-header field offsets and
    // widths differ between the two classes, so the helpers below branch on
    // `bytes[4]` (EI_CLASS) instead of assuming one layout.
    #[cfg(loader_test_exec)]
    fn exec_is_elf64(bytes: &[u8]) -> bool {
        match bytes[4] {
            1 => false,
            2 => true,
            class => panic!("unexpected ELF class: {}", class),
        }
    }

    // e_phoff / e_phentsize / e_phnum offsets and the per-field offsets below
    // come straight from the ELF32/ELF64 specifications; both classes store the
    // width fields as u16 and p_type as u32, so only the offset differs.
    #[cfg(loader_test_exec)]
    fn exec_phoff(bytes: &[u8]) -> usize {
        if exec_is_elf64(bytes) {
            u64::from_le_bytes(bytes[32..40].try_into().unwrap()) as usize
        } else {
            u32::from_le_bytes(bytes[28..32].try_into().unwrap()) as usize
        }
    }

    #[cfg(loader_test_exec)]
    fn exec_phentsize(bytes: &[u8]) -> usize {
        let off = if exec_is_elf64(bytes) { 54 } else { 42 };
        u16::from_le_bytes(bytes[off..off + 2].try_into().unwrap()) as usize
    }

    #[cfg(loader_test_exec)]
    fn exec_phnum(bytes: &[u8]) -> usize {
        let off = if exec_is_elf64(bytes) { 56 } else { 44 };
        u16::from_le_bytes(bytes[off..off + 2].try_into().unwrap()) as usize
    }

    // p_vaddr / p_paddr / p_memsz are u32 in ELF32 and u64 in ELF64. exec_app's
    // fixed addresses (e.g. 0x40280000) fit in a u32, so reading/writing the low
    // 4 bytes at the class-correct offset is sufficient for the rejection tests,
    // which only add a delta that stays within 32 bits.
    #[cfg(loader_test_exec)]
    fn exec_segment_addr_offsets(bytes: &[u8]) -> [usize; 2] {
        if exec_is_elf64(bytes) {
            [16, 24] // p_vaddr, p_paddr
        } else {
            [8, 12]
        }
    }

    #[cfg(loader_test_exec)]
    fn exec_pmemsz_off(bytes: &[u8]) -> usize {
        if exec_is_elf64(bytes) {
            40
        } else {
            20
        }
    }

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
        assert_eq!(bytes[5], 1); // ELFDATA2LSB, little-endian, both classes
        let start = exec_phoff(bytes);
        let stride = exec_phentsize(bytes);
        let count = exec_phnum(bytes);
        (0..count)
            .map(|index| start + index * stride)
            .filter(|&offset| exec_word(bytes, offset) == 1) // PT_LOAD
            .collect()
    }

    // TODO(paging): the EXEC fixtures dereference `TEST_REGION_START`
    // (0x40280000) via the flat allocator and via `read_volatile`; under paging
    // that VA is unmapped after `boards::init` drops the boot identity map.
    #[cfg(all(loader_test_exec, not(paging)))]
    #[test]
    fn test_exec_rejects_unregistered_addresses() {
        let mut bytes = read_all(unsafe { LOADER_TEST_ELF_PATH }).unwrap();
        let delta = u32::try_from(TEST_REGION_END - TEST_REGION_START).unwrap();
        // e_entry is at offset 24 in both ELF32 and ELF64.
        let entry = exec_word(&bytes, 24).checked_add(delta).unwrap();
        set_exec_word(&mut bytes, 24, entry);
        for header in exec_load_headers(&bytes) {
            for field in exec_segment_addr_offsets(&bytes) {
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

    // TODO(paging): see `test_exec_rejects_unregistered_addresses`.
    #[cfg(all(loader_test_exec, not(paging)))]
    #[test]
    fn test_exec_rejects_out_of_range_without_writing() {
        let mut bytes = read_all(unsafe { LOADER_TEST_ELF_PATH }).unwrap();
        let header = *exec_load_headers(&bytes).last().unwrap();
        let memsz_off = exec_pmemsz_off(&bytes);
        let size = exec_word(&bytes, header + memsz_off)
            .checked_add(u32::try_from(TEST_REGION_END - TEST_REGION_START).unwrap())
            .unwrap();
        set_exec_word(&mut bytes, header + memsz_off, size);
        let before = unsafe { (TEST_REGION_START as *const u32).read_volatile() };
        let error = blueos::loader::load_image(SliceElfReader::new(&bytes))
            .err()
            .expect("crossing the window boundary must fail");
        assert_eq!(error.stage(), Some(loader::error::LoadStage::Allocate));
        assert_eq!(error.kind(), loader::error::LoadErrorKind::OutOfBounds);
        let after = unsafe { (TEST_REGION_START as *const u32).read_volatile() };
        assert_eq!(after, before);
    }

    // TODO(paging): see `test_exec_rejects_unregistered_addresses`.
    #[cfg(all(loader_test_exec, not(paging)))]
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
