// Copyright (c) 2025 vivo Mobile Communication Co., Ltd.
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

#[cfg(kernel_async)]
use crate::asynk;
#[cfg(enable_net)]
use crate::net;
#[cfg(enable_vfs)]
use crate::vfs;
use crate::{
    allocator, arch, boards,
    devices::{
        console,
        tty::{n_tty::Tty, termios::Termios},
        DeviceManager,
    },
    logger, scheduler,
    sync::SpinLock,
    thread, time,
};
use alloc::string::String;
use blueos_driver::uart::UartConfig;
use blueos_hal::{Configuration, PlatPeri};
use core::{
    ptr::{addr_of, addr_of_mut},
    sync::atomic::{AtomicBool, Ordering},
};
use spin::Once;

// We have to put these globals in the .data section. If not specified explicitly,
// they might be put in the .bss section and might be used before they are initialized.
#[unsafe(link_section = ".data")]
pub(crate) static mut INIT_BSS_DONE: bool = false;
#[unsafe(link_section = ".data")]
pub(crate) static mut INIT_ARRAY_DONE: bool = false;
#[unsafe(link_section = ".data")]
pub(crate) static INIT_HEAP_DONE: AtomicBool = AtomicBool::new(false);
#[unsafe(link_section = ".data")]
pub(crate) static mut INIT_VFS_DONE: bool = false;

// See https://github.com/rust-lang/rust/pull/134213 for more details about naked function.
#[unsafe(no_mangle)]
#[unsafe(naked)]
pub unsafe extern "C" fn _start() {
    // Arch is responsible to init cores. After initializing
    // cores, arch_bootstrap should continue with `init`.
    crate::arch_bootstrap!(__sys_stack_start, __sys_stack_end, init);
}

unsafe extern "C" {
    pub static __init_array_start: extern "C" fn();
    pub static __init_array_end: extern "C" fn();
    // Apps' entries should be put in bk_app_array section.
    pub static __bk_app_array_start: extern "C" fn();
    pub static __bk_app_array_end: extern "C" fn();
    pub static mut __bss_start: u8;
    pub static mut __bss_end: u8;
    pub static mut __sys_stack_start: u8;
    pub static mut __sys_stack_end: u8;
    #[cfg(not(paging))]
    pub static mut __heap_start: u8;
    #[cfg(not(paging))]
    pub static mut __heap_end: u8;
    pub static mut _end: u8;
}

use crate::{
    devices::{bus::Bus, i2c_core::block_i2c::BlockI2c},
    drivers::InitDriver,
};

fn init_pin_states<P: blueos_hal::pinctrl::AlterFuncPin>(pin_states: &[&P]) {
    for pin_state in pin_states {
        pin_state.init();
    }
}

extern "C" fn init() {
    boards::init();
    #[cfg(not(paging))]
    init_heap();
    init_runtime();
    init_pin_states(crate::boards::PIN_STATES);

    let uart = crate::boards::get_device!(console_uart);
    uart.configure(&UartConfig::default()).unwrap();
    uart.enable();

    let tty0 = Tty::init(&crate::drivers::serial::TTY_SERIAL, Termios::default());
    DeviceManager::get().register_device(String::from("ttyS0"), tty0.clone());
    match console::init_console(tty0.clone()) {
        Ok(_) => {}
        Err(err) => panic!("Failed to init console: {}", crate::error::Error::from(err)),
    }

    #[cfg(virtio)]
    {
        use crate::devices::virtio;
        use flat_device_tree::Fdt;
        // initialize fdt
        // SAFETY: We trust that the FDT pointer we were given is valid, and this is the only time we
        // use it.
        let fdt = unsafe { Fdt::from_ptr(crate::boards::DRAM_BASE as *const u8).unwrap() };
        // initialize virtio
        virtio::init_virtio(&fdt);
    }
    #[cfg(spi_core)]
    crate::boards::init_spi_bus();
    #[cfg(i2c_core)]
    crate::boards::init_i2c_bus();
    #[cfg(gpio)]
    crate::boards::init_gpio();

    #[cfg(enable_vfs)]
    init_vfs();

    scheduler::init();
    logger::logger_init();
    time::timer::init();
    #[cfg(kernel_async)]
    asynk::init();
    #[cfg(enable_net)]
    {
        net::init();
        net::net_manager::init();
    }

    // it's an bug in fact, but at now we use a workaround let newlib do the c++ runtime initialization
    #[cfg(not(target_board = "newlib_mps3_an547"))]
    run_init_array();

    init_apps();

    // Spawn the first user-mode process once the scheduler side is ready: ELF
    // loading, user stack and EL0 entry. Only the aarch64 virt board has the
    // paging + user-entry path wired end to end; the ELF bytes are embedded
    // directly into the kernel image.
    //
    // `not(test)` keeps this out of `--test` harness builds. The standalone
    // `loader_integration_test` binary has its own `main` and never enters
    // `boot::init`, so no `loader_test_exec` exclusion is needed here — and
    // that cfg is not set on the `blueos` rlib anyway (cfgs are per-crate), so
    // such a gate would be a no-op.
    #[cfg(all(paging, target_board = "qemu_virt64_aarch64", not(test)))]
    init_user_app();

    arch::start_schedule(scheduler::schedule);
    unreachable!("We should have jumped to the schedule loop!");
}

pub(crate) fn init_runtime() {
    init_bss();
}

#[cfg(enable_vfs)]
pub(crate) fn init_vfs() {
    unsafe {
        if INIT_VFS_DONE {
            return;
        }
        if let Err(err) = vfs::vfs_init() {
            panic!("{}", err);
        };
        INIT_VFS_DONE = true;
    }
}

#[inline]
fn init_bss() {
    unsafe {
        if INIT_BSS_DONE {
            return;
        }
        // FIXME: Use memset?
        let mut ptr = addr_of_mut!(__bss_start);
        while ptr != addr_of_mut!(__bss_end) {
            ptr.write(0u8);
            ptr = ptr.offset(1);
        }
        INIT_BSS_DONE = true;
    }
}

#[inline(never)]
pub(crate) fn run_init_array() {
    unsafe {
        if INIT_ARRAY_DONE {
            return;
        }
        let mut my_init = addr_of!(__init_array_start);
        while my_init < addr_of!(__init_array_end) {
            (*my_init)();
            my_init = my_init.offset(1);
        }
        INIT_ARRAY_DONE = true;
    }
}

/// The first user ELF image, embedded into the kernel image at compile time.
///
/// The build resolves `EXEC_APP_ELF_PATH` (set in `blueos`'s `rustenv` only for
/// `qemu_virt64_aarch64`) to the absolute path of the linked `boot_exec_app`
/// binary and feeds it to `include_bytes!`, so the ELF bytes live in the
/// kernel's `.rodata`. This bypasses the FAT/virtio/VFS pipeline to validate
/// the spawn chain with the fewest moving parts; the eventual end state is to
/// read the image from `/fat/exec_app.elf` via the VFS, at which point this
/// static and its `rustenv`/`inputs` wiring are removed.
#[cfg(all(paging, target_board = "qemu_virt64_aarch64", not(test)))]
static EXEC_APP_ELF: &[u8] = include_bytes!(env!("EXEC_APP_ELF_PATH"));

/// Spawn the first user-mode process end to end.
///
/// Builds a child process under the root, prepares its user stack, feeds the
/// embedded ELF bytes to the loader pipeline to map the image into the user
/// address space. Per path 3, the user table is activated for the duration of
/// the pipeline so `write_image`, `zero_image`, and `CodeCache::synchronize`
/// translate user VAs through hardware `TTBR0`, then deactivated before the
/// thread is forged — the first context switch re-activates it and `eret`s to
/// EL0 at the entry.
///
/// Every failure is logged and skipped (no panic): a missing image or a stack
/// shortage must not abort the kernel bring-up that already started the
/// scheduler-bound kernel threads in [`init_apps`].
#[cfg(all(paging, target_board = "qemu_virt64_aarch64", not(test)))]
#[inline(never)]
fn init_user_app() {
    use crate::{
        arch::Aarch64CodeCache, loader::adapters::SliceElfReader, mm::AddressSpaceImageAllocator,
    };
    use alloc::sync::Arc;
    use blueos_loader::{memory::TargetAddress, profile, LoadRequest, LoadedImage};

    let root = crate::process::ProcessManager::get().root_process();
    let proc = match crate::process::ProcessManager::get().create_process(Arc::downgrade(root)) {
        Ok(proc) => proc,
        Err(err) => {
            log::warn!("init_user_app: create_process failed: {}", err);
            return;
        }
    };
    let aspace = proc.address_space().clone();

    // Prepare the initial user stack: backing pages, then the argc/argv/envp/auxv
    // startup frame. The returned SP is the 16-byte-aligned first EL0 SP.
    let stack_size = blueos_kconfig::CONFIG_MAIN_THREAD_STACK_SIZE as usize;
    if let Err(err) = aspace.init_user_stack(stack_size) {
        log::warn!("init_user_app: init_user_stack failed: {}", err);
        return;
    }
    let start = match aspace.build_user_stack(&[b"exec_app"], &[], &[]) {
        Ok(start) => start,
        Err(err) => {
            log::warn!("init_user_app: build_user_stack failed: {}", err);
            return;
        }
    };
    let sp = start.sp();

    // Feed the embedded ELF bytes to the loader pipeline. The slice reader
    // issues positional reads over the in-image `.rodata` bytes — no VFS, no
    // virtio, no full-image heap buffer; the bytes flow reader → loader's
    // 512-byte scratch → user pages.
    let reader = SliceElfReader::new(EXEC_APP_ELF);

    // Path 3: activate the user table for the duration of the loader pipeline
    // so write_image/zero_image and CodeCache::synchronize translate user VAs
    // through hardware TTBR0 rather than software walks. boot runs before
    // start_schedule, so this activation is a non-concurrent transient — no
    // scheduler, no other cores, no threads can touch TTBR0 in parallel.
    arch::switch_user_address_space(None, Some(aspace.root_address()));

    let loaded: LoadedImage<_> = match blueos_loader::load_elf(
        reader,
        LoadRequest::new(
            profile::LoadProfile::aarch64(),
            profile::LoadLimits::DEFAULT,
        ),
        AddressSpaceImageAllocator::new(aspace.clone()),
        &mut Aarch64CodeCache::new(),
    ) {
        Ok(loaded) => loaded,
        Err(err) => {
            // Deactivate before returning so a failed load does not leak an
            // active user table into the rest of bring-up.
            arch::switch_user_address_space(Some(aspace.root_address()), None);
            log::warn!("init_user_app: loader pipeline failed: {:?}", err);
            return;
        }
    };

    // Load done: drop back to kernel-only TTBR0 (TTBR1 is unchanged). Entry
    // extraction, kernel-stack allocation, and spawn are all kernel-side and
    // never touch user VAs. The scheduler re-activates this table on the first
    // context switch into the user thread.
    arch::switch_user_address_space(Some(aspace.root_address()), None);

    let entry: TargetAddress = match loaded.entry() {
        Some(entry) => entry,
        None => {
            log::warn!("init_user_app: ELF has no entry point");
            return;
        }
    };
    // exec_app is ET_EXEC, so the header entry is the PC directly.
    let pc = entry.get() as usize;

    // Kernel-side stack for the user thread's saved context.
    let kstack = match thread::Stack::from_size(stack_size) {
        Some(stack) => stack,
        None => {
            log::warn!("init_user_app: kernel stack allocation failed");
            return;
        }
    };
    // `.start()` enqueues the thread ready (GlobalQueueVisitor::add +
    // queue_ready_thread); the first schedule picks it up.
    let node = thread::Builder::user(pc, sp).set_stack(kstack).start();
    if !proc.add_thread(node) {
        log::warn!("init_user_app: user thread already linked to a process");
    }

    // Leak the loaded image: it owns the `AddressSpaceImageMapping` that backs
    // the user pages the thread now executes. `LoadedImage` has no
    // take-ownership accessor (the old `into_mapping()` was removed), so forget
    // the whole image — its `Drop` is skipped, which in turn skips the
    // mapping's `Drop` (the `release_image` in `AddressSpaceImageMapping::drop`),
    // keeping the backing alive for the life of the booted image. A clean
    // teardown path is D-trap-fault/exit work.
    core::mem::forget(loaded);
    log::info!(
        "init_user_app: spawned user process at pc={:#x} sp={:#x}",
        pc,
        sp
    );
}

#[inline(never)]
fn init_apps() {
    let root = crate::process::ProcessManager::get().root_process();

    unsafe {
        let mut app = addr_of!(__bk_app_array_start);
        while app < addr_of!(__bk_app_array_end) {
            let stack =
                thread::Stack::from_size(blueos_kconfig::CONFIG_MAIN_THREAD_STACK_SIZE as usize)
                    .expect("Invalid main thread stack size");
            let node = thread::Builder::new(thread::Entry::C(*app))
                .set_stack(stack)
                .start();
            let proc = crate::process::ProcessManager::get()
                .create_process(alloc::sync::Arc::downgrade(root))
                .unwrap_or_else(|e| {
                    panic!("failed to create app process. app addr: {:p}: {}", app, e)
                });
            if !proc.add_thread(node) {
                log::warn!(
                    "app thread already linked to a process. app addr: {:p}",
                    app
                );
            }
            app = app.offset(1);
        }
    }
}

#[inline(never)]
#[cfg(all(paging, not(allocator_buddy)))]
pub(crate) fn init_frames(regions: &[crate::mm::memory_bank::MemoryRegion]) {
    use crate::mm::paging::first_fit::FirstFitFrameAllocator;

    // SAFETY: CPU0 calls this once after runtime initialization. The static
    // array is exclusively transferred to FirstFit. The reserved regions cover
    // this array, the kernel, stacks and boot table; heap backing follows later.
    unsafe {
        FirstFitFrameAllocator::instance()
            .init(regions)
            .expect("failed to initialize FirstFit frame allocator");
    }
}

#[cfg(not(paging))]
#[inline(never)]
pub(crate) fn init_heap() {
    if INIT_HEAP_DONE.load(Ordering::Acquire) {
        return;
    }

    allocator::init_heap(addr_of_mut!(__heap_start), addr_of_mut!(__heap_end));
    INIT_HEAP_DONE.store(true, Ordering::Release);
}
