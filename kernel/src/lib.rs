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

#![no_std]
#![allow(internal_features)]
#![allow(incomplete_features)]
#![allow(clippy::crate_in_macro_def)]
#![allow(clippy::drop_non_drop)]
#![feature(alloc_error_handler)]
#![feature(alloc_layout_extra)]
#![feature(allocator_api)]
#![feature(associated_type_defaults)]
#![feature(async_closure)]
#![feature(box_as_ptr)]
#![feature(c_size_t)]
#![feature(c_variadic)]
#![feature(const_trait_impl)]
#![feature(core_intrinsics)]
#![feature(coverage_attribute)]
#![feature(fn_align)]
#![feature(generic_arg_infer)]
#![feature(inherent_associated_types)]
#![feature(lazy_get)]
#![feature(let_chains)]
#![feature(link_llvm_intrinsics)]
#![feature(linkage)]
#![feature(macro_metavar_expr)]
#![feature(map_try_insert)]
#![feature(naked_functions)]
#![feature(negative_impls)]
#![feature(new_zeroed_alloc)]
#![feature(non_null_from_ref)]
#![feature(noop_waker)]
#![feature(pointer_is_aligned_to)]
#![feature(trait_upcasting)]
#![feature(trivial_bounds)]
// Attributes applied when we're testing the kernel.
#![cfg_attr(test, no_main)]
#![cfg_attr(test, feature(custom_test_frameworks))]
#![cfg_attr(test, test_runner(tests::kernel_unittest_runner))]
#![cfg_attr(test, reexport_test_harness_main = "run_kernel_unittests")]

// #[cfg(test)]
// blueos_test_macro::test_only!();

extern crate alloc;
pub mod allocator;
pub mod arch;
#[cfg(kernel_async)]
pub mod asynk;
pub mod boards;
#[cfg(use_kernel_boot)]
pub(crate) mod boot;
pub mod config;
pub mod console;
#[cfg(coverage)]
pub mod coverage;
pub(crate) mod devices;
pub(crate) mod drivers;
pub mod error;
pub mod ffi;
pub mod irq;
pub mod logger;
pub mod mm;
#[cfg(enable_net)]
pub mod net;
pub mod scheduler;
pub mod support;
pub mod sync;
pub mod syscall_handlers;
pub mod thread;
pub mod time;
pub mod types;
#[cfg(enable_vfs)]
pub mod vfs;

pub use syscall_handlers as syscalls;
pub(crate) mod signal;

#[macro_export]
macro_rules! debug {
    ($($tt:tt)*) => {{}};
}

pub(crate) static TRACER: spin::Mutex<()> = spin::Mutex::new(());

#[macro_export]
macro_rules! trace {
    ($($tt:tt)*) => {{
        let dig = $crate::support::DisableInterruptGuard::new();
        let l = $crate::TRACER.lock();
        #[cfg(target_pointer_width="32")]
        semihosting::eprint!("[C:{:02} SP:0x{:08x}] ",
                             $crate::arch::current_cpu_id(),
                             $crate::arch::current_sp());
        #[cfg(target_pointer_width="64")]
        semihosting::eprint!("[C:{:02} SP:0x{:016x}] ",
                             $crate::arch::current_cpu_id(),
                             $crate::arch::current_sp());
        semihosting::eprintln!($($tt)*);
        drop(l);
        drop(dig);
    }};
}

#[cfg(test)]
mod tests {
    extern crate alloc;
    use super::*;
    use crate as blueos;
    use crate::{
        allocator,
        allocator::KernelAllocator,
        config,
        support::DisableInterruptGuard,
        sync,
        sync::{wait_until, wake, ConstBarrier},
        time::Tick,
        types::Arc,
    };
    use alloc::vec::Vec;
    use blueos_header::syscalls::NR::Nop;
    use blueos_test_macro::{only_test, test};
    use core::{
        mem::MaybeUninit,
        panic::PanicInfo,
        ptr,
        sync::atomic::{AtomicBool, AtomicUsize, Ordering},
    };
    #[cfg(use_defmt)]
    use defmt_rtt as _;
    use spin::Lazy;
    use thread::{Entry, SystemThreadStorage, Thread, ThreadKind, ThreadNode};

    #[used]
    #[link_section = ".bk_app_array"]
    static INIT_TEST: extern "C" fn() = init_test;

    extern "C" fn test_main() {
        run_kernel_unittests();
    }

    #[cfg(target_pointer_width = "32")]
    const K: usize = 1;

    #[cfg(all(debug_assertions, target_pointer_width = "64"))]
    pub const K: usize = 1;
    #[cfg(all(not(debug_assertions), target_pointer_width = "64"))]
    pub const K: usize = 64;

    const NUM_CORES: usize = blueos_kconfig::CONFIG_NUM_CORES as usize;
    static mut TEST_THREAD_STORAGES: [SystemThreadStorage; NUM_CORES * K] =
        [const { SystemThreadStorage::new(ThreadKind::Normal) }; NUM_CORES * K];
    static mut TEST_THREADS: [MaybeUninit<ThreadNode>; NUM_CORES * K] =
        [const { MaybeUninit::zeroed() }; NUM_CORES * K];

    static mut MAIN_THREAD_STORAGE: SystemThreadStorage =
        SystemThreadStorage::new(ThreadKind::Normal);
    static mut MAIN_THREAD: MaybeUninit<ThreadNode> = MaybeUninit::zeroed();

    fn reset_and_queue_test_thread(
        i: usize,
        entry: extern "C" fn(),
        cleanup: Option<extern "C" fn()>,
    ) {
        unsafe {
            let t = TEST_THREADS[i].assume_init_ref();
            t.set_preempt_count(0);
            let mut w = t.lock();
            let stack = &mut TEST_THREAD_STORAGES[i].stack;
            let Some(stack) = thread::Stack::from_raw(stack.rep.as_mut_ptr(), stack.rep.len())
            else {
                panic!("Invalid stack");
            };
            w.init(stack, thread::Entry::C(entry));
            if let Some(cleanup) = cleanup {
                w.set_cleanup(Entry::C(cleanup));
            };
            let ok = scheduler::queue_ready_thread(w.state(), t.clone());
            assert_eq!(ok, Ok(()));
        }
    }

    fn reset_and_queue_test_threads(entry: extern "C" fn(), cleanup: Option<extern "C" fn()>) {
        unsafe {
            for i in 0..TEST_THREADS.len() {
                reset_and_queue_test_thread(i, entry, cleanup);
            }
        }
    }

    fn init_test_thread(i: usize) {
        let t = thread::build_static_thread(
            unsafe { &mut TEST_THREADS[i] },
            unsafe { &mut TEST_THREAD_STORAGES[i] },
            config::MAX_THREAD_PRIORITY / 2,
            thread::IDLE,
            Entry::C(test_main),
            ThreadKind::Normal,
        );
    }

    extern "C" fn init_test() {
        let l = unsafe { TEST_THREADS.len() };
        for i in 0..l {
            init_test_thread(i);
        }
        let t = thread::build_static_thread(
            unsafe { &mut MAIN_THREAD },
            unsafe { &mut MAIN_THREAD_STORAGE },
            config::MAX_THREAD_PRIORITY / 2,
            thread::IDLE,
            Entry::C(test_main),
            ThreadKind::Normal,
        );
        let ok = scheduler::queue_ready_thread(thread::IDLE, t.clone());
        assert_eq!(ok, Ok(()));
    }

    #[cfg(target_pointer_width = "64")]
    const EMBALLOC_SIZE: usize = 8 << 20;
    #[cfg(target_pointer_width = "32")]
    const EMBALLOC_SIZE: usize = 2 << 20;

    #[global_allocator]
    static ALLOCATOR: KernelAllocator = KernelAllocator;
    // Emballoc is for correctness reference.
    //static ALLOCATOR: emballoc::Allocator<{ EMBALLOC_SIZE }> = emballoc::Allocator::new();

    #[panic_handler]
    fn oops(info: &PanicInfo) -> ! {
        let _guard = DisableInterruptGuard::new();
        #[cfg(not(use_defmt))]
        {
            semihosting::println!("{}", info);
            semihosting::println!("Oops: {}", info.message());
            let mem_info = allocator::memory_info();
            semihosting::println!(
                "Memory: total={} used={} max={}",
                mem_info.total,
                mem_info.used,
                mem_info.max_used
            );
        }

        #[cfg(use_defmt)]
        {
            defmt::error!("{}", defmt::Display2Format(info));
            defmt::error!("Oops: {}", defmt::Display2Format(&info.message()));
            let mem_info = allocator::memory_info();
            defmt::error!(
                "Memory: total={} used={} max={}",
                mem_info.total,
                mem_info.used,
                mem_info.max_used
            );
        }
        loop {}
    }

    #[test]
    fn test_spinlock() {
        let lock = sync::spinlock::SpinLock::new(0);
        let mut w = lock.irqsave_lock();
        *w = 1;
        drop(w);

        assert!(scheduler::current_thread().validate_sp());
        scheduler::yield_me_now_or_later();
        assert!(scheduler::current_thread().validate_sp());

        let r = lock.irqsave_lock();
        assert_eq!(*r, 1);
    }

    #[test]
    fn test_spinlock_loop() {
        let lock = sync::spinlock::SpinLock::new(0);
        loop {
            let mut w = lock.irqsave_lock();
            *w += 1;
            drop(w);

            scheduler::yield_me_now_or_later();

            let r = lock.irqsave_lock();
            if *r == 100 {
                break;
            }
        }
    }

    #[cfg(cortex_m)]
    #[test]
    fn test_sys_tick() {
        let tick = Tick::now();
        assert!(scheduler::current_thread().validate_sp());
        scheduler::suspend_me_for::<()>(Tick(10), None);
        assert!(scheduler::current_thread().validate_sp());
        let tick2 = Tick::now();
        assert!(tick2.0 - tick.0 >= 10);
        assert!(tick2.0 - tick.0 <= 11);
    }

    // In esp32c3, we use usb-serial as the console output,
    // which does not support on qemu yet, so we skip this test on esp32c3 for now.
    // See https://github.com/espressif/esp-toolchain-docs/blob/main/qemu/README.md
    #[cfg_attr(not(any(soc_esp32c3, soc_esp32c6)), test)]
    fn test_early_printk() {
        kearly_println!("Hello from early_printk!");
    }

    #[test]
    fn test_local_irq() {
        assert!(arch::local_irq_enabled());
    }

    #[test(thread = 2, repeat = 2)]
    fn test_harness_thread_attribute() {
        static ARRIVED: AtomicUsize = AtomicUsize::new(0);

        let arrived = ARRIVED.fetch_add(1, Ordering::AcqRel) + 1;
        let target = if arrived % 2 == 0 {
            arrived
        } else {
            arrived + 1
        };
        while ARRIVED.load(Ordering::Acquire) < target {
            scheduler::yield_me();
        }
    }

    #[cfg(mpu_stack_guard)]
    extern "C" {
        static __sys_stack_guard_start: u8;
    }

    #[cfg(mpu_stack_guard)]
    static MEMFAULT_TRIGGERED: AtomicBool = AtomicBool::new(false);

    #[cfg(mpu_stack_guard)]
    fn thumb_instruction_len(pc: usize) -> usize {
        let first_halfword = unsafe { (pc as *const u16).read_volatile() };
        if (first_halfword & 0xF800) == 0xE800 || (first_halfword & 0xF000) == 0xF000 {
            4
        } else {
            2
        }
    }

    #[cfg(mpu_stack_guard)]
    #[inline]
    const fn align_up(addr: usize, align: usize) -> usize {
        (addr + align - 1) & !(align - 1)
    }

    #[cfg(mpu_stack_guard)]
    extern "C" fn handle_memfault_impl(ctx: &mut crate::arch::IsrContext) {
        let scb = unsafe { &*cortex_m::peripheral::SCB::PTR };
        let cfsr = scb.cfsr.read();
        // MMFSR is in CFSR[7:0].
        assert_ne!(
            cfsr & 0xff,
            0,
            "MemManage handler entered without MMFSR status"
        );
        assert_ne!(
            cfsr & (1 << 1),
            0,
            "MemManage triggered but DACCVIOL is not set"
        );
        MEMFAULT_TRIGGERED.store(true, Ordering::Release);
        // Clear MMFSR bits and skip the faulting instruction.
        unsafe { scb.cfsr.write(cfsr & 0xff) };
        ctx.pc = ctx.pc.wrapping_add(thumb_instruction_len(ctx.pc));
    }

    #[cfg(mpu_stack_guard)]
    #[naked]
    #[no_mangle]
    pub unsafe extern "C" fn handle_memfault() {
        core::arch::naked_asm!(
            "
            mrs r0, msp
            tst lr, #0x04
            beq 1f
            mrs r0, psp
            1:
            b {handler}
            ",
            handler = sym handle_memfault_impl
        )
    }

    #[cfg(mpu_stack_guard)]
    #[test]
    fn test_mpu_sys_stack_guard_write_fault() {
        MEMFAULT_TRIGGERED.store(false, Ordering::Release);
        let addr = unsafe { core::ptr::addr_of!(__sys_stack_guard_start) as *mut u32 };
        unsafe { core::ptr::write_volatile(addr, 0x5A5A_A5A5) };
        assert!(
            MEMFAULT_TRIGGERED.load(Ordering::Acquire),
            "MPU guard write did not trigger MemManage"
        );
    }

    #[cfg(mpu_stack_guard)]
    #[test]
    fn test_mpu_thread_stack_guard_write_fault() {
        const MPU_REGION_ALIGN: usize = 32;
        let current = scheduler::current_thread_ref();
        let guard_size = blueos_kconfig::CONFIG_STACK_GUARD_ALIGN_AND_SIZE as usize;
        assert!(
            guard_size >= MPU_REGION_ALIGN && guard_size % MPU_REGION_ALIGN == 0,
            "Invalid stack guard size: {guard_size}"
        );

        let stack_base = current.stack_base();
        let stack_top = stack_base + current.stack_size();
        let guard_start = align_up(stack_base, MPU_REGION_ALIGN);
        assert!(
            guard_start < stack_top,
            "No valid guard start in stack range"
        );

        MEMFAULT_TRIGGERED.store(false, Ordering::Release);
        unsafe { core::ptr::write_volatile(guard_start as *mut u32, 0xA5A5_5A5A) };
        assert!(
            MEMFAULT_TRIGGERED.load(Ordering::Acquire),
            "Per-thread MPU stack guard write did not trigger MemManage"
        );
    }

    #[test]
    fn stress_trap() {
        #[cfg(target_pointer_width = "32")]
        let n = 16;
        #[cfg(target_pointer_width = "64")]
        let n = 256;
        for _i in 0..n {
            #[cfg(any(target_arch = "riscv64", target_arch = "riscv32"))]
            unsafe {
                core::arch::asm!(
                    "ecall",
                    in("a7") Nop as usize,
                    inlateout("a0") 0 => _,
                    options(nostack),
                );
            };
        }
    }

    #[derive(Default)]
    struct CleanupCounter {
        counter: AtomicUsize,
    }

    impl CleanupCounter {
        pub const fn new() -> Self {
            Self {
                counter: AtomicUsize::new(0),
            }
        }
        pub fn spin_until_eq(&self, n: usize) {
            while self.counter.load(Ordering::Relaxed) != n {
                scheduler::yield_me();
            }
        }
        pub fn increment(&self) {
            self.counter.fetch_add(1, Ordering::Relaxed);
        }
        pub fn reset(&self) {
            self.counter.store(0, Ordering::Relaxed);
        }
    }

    static SEMA_CLEANUP_COUNTER: CleanupCounter = CleanupCounter::new();
    static mut SEMA_COUNTER: usize = 0usize;
    static SEMA: sync::semaphore::Semaphore = sync::semaphore::Semaphore::new();

    extern "C" fn test_semaphore() {
        SEMA.acquire_notimeout::<scheduler::InsertToEnd>();
        let n = unsafe { SEMA_COUNTER };
        unsafe { SEMA_COUNTER += 1 };
        SEMA.release();
    }

    extern "C" fn test_semaphore_cleanup() {
        SEMA_CLEANUP_COUNTER.increment();
    }

    #[test]
    fn stress_semaphore() {
        SEMA_CLEANUP_COUNTER.reset();
        unsafe { SEMA_COUNTER = 0 };
        SEMA.init(1);
        reset_and_queue_test_threads(test_semaphore, Some(test_semaphore_cleanup));
        let l = unsafe { TEST_THREADS.len() };
        loop {
            SEMA.acquire_notimeout::<scheduler::InsertToEnd>();
            let n = unsafe { SEMA_COUNTER };
            if n == l {
                SEMA.release();
                break;
            }
            SEMA.release();
            scheduler::yield_me();
        }
        SEMA_CLEANUP_COUNTER.spin_until_eq(l);
    }

    static ATOMIC_WAIT_CLEANUP: CleanupCounter = CleanupCounter::new();
    static TEST_ATOMIC_WAIT: AtomicUsize = AtomicUsize::new(0);

    extern "C" fn test_atomic_wait_cleanup() {
        ATOMIC_WAIT_CLEANUP.increment();
    }

    extern "C" fn test_atomic_wait() {
        TEST_ATOMIC_WAIT.fetch_add(1, Ordering::Release);
        sync::atomic_wait::atomic_wake(&TEST_ATOMIC_WAIT, 1);
    }

    #[test]
    fn stress_atomic_wait() {
        reset_and_queue_test_threads(test_atomic_wait, Some(test_atomic_wait_cleanup));
        let l = unsafe { TEST_THREADS.len() };
        loop {
            let n = TEST_ATOMIC_WAIT.load(Ordering::Acquire);
            if n == l {
                break;
            }
            sync::atomic_wait::atomic_wait(&TEST_ATOMIC_WAIT, n, Tick::MAX);
        }
        ATOMIC_WAIT_CLEANUP.spin_until_eq(l);
    }

    static MUTEX_CLEANUP: CleanupCounter = CleanupCounter::new();
    static_arc! {
        MUTEX(sync::mutex::Mutex, sync::mutex::Mutex::new()),
    }
    static mut MUTEX_COUNTER: usize = 0usize;

    extern "C" fn test_mutex() {
        MUTEX.pend_for(Tick::MAX);
        unsafe { MUTEX_COUNTER += 1 };
        MUTEX.post();
    }

    extern "C" fn test_mutex_cleanup() {
        MUTEX_CLEANUP.increment();
    }

    #[test]
    fn stress_mutex() {
        MUTEX.init();
        reset_and_queue_test_threads(test_mutex, Some(test_mutex_cleanup));
        let l = unsafe { TEST_THREADS.len() };
        loop {
            MUTEX.pend_for(Tick::MAX);
            let n = unsafe { MUTEX_COUNTER };
            if n == l {
                MUTEX.post();
                break;
            }
            MUTEX.post();
            scheduler::yield_me();
        }
        MUTEX_CLEANUP.spin_until_eq(l);
    }

    static MQUEUE: Lazy<Arc<sync::mqueue::MessageQueue>> =
        Lazy::new(|| Arc::new(sync::mqueue::MessageQueue::new(4, 2, ptr::null_mut())));
    static TEST_SEND_CNT: AtomicUsize = AtomicUsize::new(0);

    extern "C" fn test_mqueue() {
        let buffer = [1u8; 4];
        let result = MQUEUE.send(&buffer, 4, Tick(512), sync::mqueue::SendMode::Normal);
        assert!(result.is_ok());
    }

    extern "C" fn test_mqueue_cleanup() {
        TEST_SEND_CNT.fetch_add(1, Ordering::Relaxed);
    }

    // FIXME: We have performance issue on SMP. See
    // https://github.com/vivoblueos/kernel/issues/111 for details.
    #[cfg_attr(not(target_board = "qemu_riscv64"), test)]
    #[cfg_attr(target_board = "qemu_riscv64", blueos_test_macro::ignore)]
    fn stress_mqueue() {
        MQUEUE.init();
        reset_and_queue_test_threads(test_mqueue, Some(test_mqueue_cleanup));
        let l = unsafe { TEST_THREADS.len() };
        let mut recv_cnt = 0;
        let mut buffer = [0u8; 4];
        loop {
            if recv_cnt == l {
                break;
            }
            let result = MQUEUE.recv(&mut buffer, 4, Tick(512));
            recv_cnt += 1;
            assert!(result.is_ok());
            assert_eq!(buffer, [1u8, 1u8, 1u8, 1u8]);
            scheduler::relinquish_me();
        }
        while TEST_SEND_CNT.load(Ordering::Acquire) != l {}
    }

    static TEST_SWITCH_CONTEXT: AtomicUsize = AtomicUsize::new(0);

    extern "C" fn test_switch_context() {
        let n = 4;
        for _i in 0..n {
            assert!(scheduler::current_thread().validate_sp());
            scheduler::yield_me();
            assert!(scheduler::current_thread().validate_sp());
        }
    }

    extern "C" fn test_switch_context_cleanup() {
        TEST_SWITCH_CONTEXT.fetch_add(1, Ordering::Relaxed);
    }

    #[test]
    fn stress_context_switch() {
        reset_and_queue_test_threads(test_switch_context, Some(test_switch_context_cleanup));
        loop {
            let n = TEST_SWITCH_CONTEXT.load(Ordering::Relaxed);
            if n == unsafe { TEST_THREADS.len() } {
                break;
            }
            assert!(scheduler::current_thread().validate_sp());
            scheduler::yield_me();
            assert!(scheduler::current_thread().validate_sp());
        }
    }

    static BUILT_THREADS: AtomicUsize = AtomicUsize::new(0);

    extern "C" fn do_it() {
        BUILT_THREADS.fetch_add(1, Ordering::Relaxed);
    }

    #[test]
    fn stress_build_threads() {
        #[cfg(target_pointer_width = "32")]
        let n = blueos_kconfig::CONFIG_UNITTEST_THREAD_NUM as usize / 2;
        #[cfg(all(debug_assertions, target_pointer_width = "64"))]
        let n = 32;
        #[cfg(all(not(debug_assertions), target_pointer_width = "64"))]
        let n = 512;
        for _i in 0..n {
            let t = thread::Builder::new(thread::Entry::C(do_it)).build();
            let ok = scheduler::queue_ready_thread(t.state(), t);
            assert_eq!(ok, Ok(()));
        }
        loop {
            let m = BUILT_THREADS.load(Ordering::Relaxed);
            if m == n {
                break;
            }
            scheduler::yield_me();
        }
    }

    static SPAWNED_THREADS: AtomicUsize = AtomicUsize::new(0);
    #[test]
    fn stress_spawn_threads() {
        #[cfg(target_pointer_width = "32")]
        let n = blueos_kconfig::CONFIG_UNITTEST_THREAD_NUM as usize / 2;
        #[cfg(all(debug_assertions, target_pointer_width = "64"))]
        let n = 32;
        #[cfg(all(not(debug_assertions), target_pointer_width = "64"))]
        let n = 512;
        for _i in 0..n {
            thread::spawn(move || {
                SPAWNED_THREADS.fetch_add(1, Ordering::Relaxed);
            });
        }
        loop {
            let m = SPAWNED_THREADS.load(Ordering::Relaxed);
            if m == n {
                break;
            }
            scheduler::yield_me();
        }
    }

    // Should not hang.
    #[test]
    fn test_simple_signal() {
        let a = Arc::new(ConstBarrier::<{ 2 }>::new());
        let b = Arc::new(AtomicUsize::new(0));
        let closure = {
            let a = a.clone();
            let b = b.clone();
            move || {
                a.wait();
                sync::atomic_wait::atomic_wait(&b, 0, Tick::MAX);
            }
        };
        let t = crate::thread::spawn(closure).unwrap();
        // Send SIGTERM after t enters its entry function.
        a.wait();
        // FIXME: Memory leaks since we are using a boxed closure as t's entry.
        t.lock().kill(libc::SIGTERM as i32);
        // At this point, t is either
        // 0: waking up from "a" or
        // 1: is suspended on "b".
        // We solve both cases by invoking yield_me and atomic_wake, which
        // should not hang.
        b.store(1, Ordering::Release);
        sync::atomic_wait::atomic_wake(&b, 1);
        scheduler::yield_me();
    }

    async fn foo(i: usize) -> usize {
        i
    }

    async fn bar() -> usize {
        42
    }

    async fn is_asynk_working() {
        let a = foo(42).await;
        let b = bar().await;
        assert_eq!(a - b, 0);
    }

    #[test]
    fn stress_async_basic() {
        let n = 1024;
        for _i in 0..n {
            asynk::block_on(is_asynk_working());
        }
    }

    async fn yield_now() {
        asynk::yield_now().await;
    }

    #[test]
    fn test_yield_now() {
        asynk::block_on(yield_now());
    }

    #[cfg(target_abi = "eabihf")]
    #[test]
    fn test_basic_float_add_sub() {
        let a: f32 = 1.0;
        let b = 2.0;
        let c = 3.0;
        let epsilon = 1e-6;
        assert!((a + b - c).abs() <= epsilon);
    }

    #[cfg(target_abi = "eabihf")]
    #[test]
    fn test_basic_float_mul_div() {
        let a: f32 = 2.0;
        let b = 3.0;
        let c = 6.0;
        let epsilon = 1e-6;
        assert!((a * b / c - 1.0).abs() <= epsilon);
    }

    #[inline(never)]
    pub fn kernel_unittest_runner(tests: &[&dyn Fn()]) {
        let t = scheduler::current_thread();
        #[cfg(use_defmt)]
        use defmt::println;
        #[cfg(not(use_defmt))]
        use semihosting::println;

        println!("---- Running {} kernel unittests...", tests.len());
        #[cfg(use_defmt)]
        println!(
            "Before test, thread 0x{:x}, rc: {}, heap status: {:?}, sp: 0x{:x}",
            Thread::id(&t),
            ThreadNode::strong_count(&t),
            defmt::Debug2Format(&allocator::memory_info()),
            arch::current_sp(),
        );
        #[cfg(not(use_defmt))]
        println!(
            "Before test, thread 0x{:x}, rc: {}, heap status: {:?}, sp: 0x{:x}",
            Thread::id(&t),
            ThreadNode::strong_count(&t),
            allocator::memory_info(),
            arch::current_sp(),
        );
        for test in tests {
            test();
        }
        #[cfg(use_defmt)]
        println!(
            "After test, thread 0x{:x}, heap status: {:?}, sp: 0x{:x}",
            Thread::id(&t),
            defmt::Debug2Format(&allocator::memory_info()),
            arch::current_sp()
        );
        #[cfg(not(use_defmt))]
        println!(
            "After test, thread 0x{:x}, heap status: {:?}, sp:  0x{:x}",
            Thread::id(&t),
            allocator::memory_info(),
            arch::current_sp()
        );
        println!("---- Done kernel unittests.");
        #[cfg(coverage)]
        crate::coverage::write_coverage_data();
        #[cfg(use_defmt)]
        cortex_m_semihosting::debug::exit(cortex_m_semihosting::debug::EXIT_SUCCESS);
    }

    #[cfg(event_flags)]
    static EVENT_COUNTER: AtomicUsize = AtomicUsize::new(0);
    #[cfg(event_flags)]
    static EVENT: sync::event_flags::EventFlags = sync::event_flags::EventFlags::new();
    #[cfg(event_flags)]
    extern "C" fn test_event_flags() {
        EVENT.wait::<scheduler::InsertToEnd>(
            1 << 0,
            sync::event_flags::EventFlagsMode::ANY,
            Tick(100),
        );
    }
    #[cfg(event_flags)]
    extern "C" fn test_event_flags_cleanup() {
        EVENT_COUNTER.fetch_add(1, Ordering::Relaxed);
    }
    #[cfg(event_flags)]
    #[test]
    fn stress_event_flags() {
        EVENT.init(0);
        reset_and_queue_test_threads(test_event_flags, Some(test_event_flags_cleanup));
        let l = unsafe { TEST_THREADS.len() };
        loop {
            EVENT.set(1 << 0);
            let n = EVENT_COUNTER.load(Ordering::Relaxed);
            if n == l {
                break;
            }
            scheduler::yield_me();
        }
    }

    extern "C" fn test_sched_timers() {
        scheduler::suspend_me_for::<()>(Tick(10), None);
    }

    extern "C" fn test_sched_timers_cleanup() {
        SCHED_TIMERS_CLEANUP.increment();
    }

    static SCHED_TIMERS_CLEANUP: CleanupCounter = CleanupCounter::new();

    #[test]
    fn stress_sched_timers() {
        reset_and_queue_test_threads(test_sched_timers, Some(test_sched_timers_cleanup));
        let l = unsafe { TEST_THREADS.len() };
        SCHED_TIMERS_CLEANUP.spin_until_eq(l);
    }

    static ALLOCATOR_STRESS_THREAD1_DONE: AtomicUsize = AtomicUsize::new(0);
    static ALLOCATOR_STRESS_THREAD2_DONE: AtomicUsize = AtomicUsize::new(0);
    const MAX_TEST_HEAP_SIZE: usize = 8 * 1024 * 1024;
    fn alloc_test() {
        // Get memory info and calculate test size
        let mem_info = allocator::memory_info();
        let available = mem_info.total.saturating_sub(mem_info.used);
        let test_size = core::cmp::min(available * 3 / 4, MAX_TEST_HEAP_SIZE);

        // Test with different allocation sizes
        let sizes = [8, 16, 32, 64, 128, 256, 512, 1024, 2048, 4096];
        let mut allocations: Vec<Vec<u8>> = Vec::new();
        let mut current_used = 0;

        // Allocate memory in chunks
        for _iter in 0..1000 {
            for &size in &sizes {
                // More aggressive memory management
                if current_used > test_size / 2 {
                    allocations.clear();
                    current_used = 0;
                }

                let vec = alloc::vec::Vec::<u8>::with_capacity(size);
                allocations.push(vec);
                current_used += size;

                // Yield to allow other thread to run
                scheduler::relinquish_me();
            }
        }

        // Clean up remaining allocations
        drop(allocations);
    }

    extern "C" fn allocator_stress_thread1() {
        alloc_test();
        ALLOCATOR_STRESS_THREAD1_DONE.store(1, Ordering::Release);
        wake(&ALLOCATOR_STRESS_THREAD1_DONE);
    }

    extern "C" fn allocator_stress_thread2() {
        alloc_test();
        ALLOCATOR_STRESS_THREAD2_DONE.store(1, Ordering::Release);
        wake(&ALLOCATOR_STRESS_THREAD2_DONE);
    }

    #[test]
    fn stress_allocator() {
        // Get initial memory info
        let initial_info = allocator::memory_info();
        let available = initial_info.total.saturating_sub(initial_info.used);
        let test_size = (available * 3) / 4;
        assert!(test_size > 0, "Not enough available memory for stress test");

        // Reset completion flags
        ALLOCATOR_STRESS_THREAD1_DONE.store(0, Ordering::Release);
        ALLOCATOR_STRESS_THREAD2_DONE.store(0, Ordering::Release);

        // Start two threads for concurrent allocation/deallocation
        let t1 = thread::Builder::new(thread::Entry::C(allocator_stress_thread1))
            .set_priority(config::MAX_THREAD_PRIORITY / 2)
            .build();
        let ok1 = scheduler::queue_ready_thread(t1.state(), t1);
        assert_eq!(ok1, Ok(()));

        let t2 = thread::Builder::new(thread::Entry::C(allocator_stress_thread2))
            .set_priority(config::MAX_THREAD_PRIORITY / 2)
            .build();
        let ok2 = scheduler::queue_ready_thread(t2.state(), t2);
        assert_eq!(ok2, Ok(()));

        // Wait for both threads to complete
        wait_until(1, &ALLOCATOR_STRESS_THREAD1_DONE);
        wait_until(1, &ALLOCATOR_STRESS_THREAD2_DONE);

        #[cfg(allocator = "slab_dynamic")]
        {
            allocator::reclaim_page_pool();
        }

        let final_info = allocator::memory_info();
        // Memory should be back to a reasonable state (allowing for some fragmentation)
        assert!(
            final_info.used <= initial_info.used + test_size / 10,
            "Memory usage after stress test is too high: initial_used={}, final_used={}, test_size={}",
            initial_info.used,
            final_info.used,
            test_size
        );
    }

    // The test always panicks since we are using 0xdeadbeef as magic number.
    #[blueos_test_macro::ignore]
    fn test_always_double_free() {
        let mut spray_vecs = alloc::vec::Vec::new();
        for size in (32..256).step_by(8) {
            for _ in 0..20 {
                let num_usizes = size / core::mem::size_of::<usize>();
                let mut spray = alloc::vec![0usize; num_usizes];
                for i in 0..num_usizes {
                    spray[i] = 0xDEAD_BEEF;
                }
                spray_vecs.push(spray);
            }
        }
    }

    // ─────────────────────────────────────────────────────────────────
    //  Tier 0 — DMA descriptor / capability pure-function unit tests.
    //  RAM-only: no register reads, so safe on any target that links the
    //  esp32c6 GDMA driver. Gated on `soc_esp32c6` because the symbols live
    //  in `blueos_driver::dma::esp32c6_gdma`, which is only compiled for
    //  that SoC. These tests *must* live in the kernel crate (not in the
    //  driver rlib): BlueOS's GN build applies `--cfg test` only to the
    //  `kernel_unittest` bin, so a `#[cfg(test)]` module in a dependency
    //  rlib is dead code and never linked.
    // ─────────────────────────────────────────────────────────────────
    #[cfg(soc_esp32c6)]
    mod gdma_unit {
        use super::*;
        use blueos_driver::dma::esp32c6_gdma::{
            DmaDescriptor, Esp32c6GdmaChannel, DW0_LENGTH_SHIFT, DW0_OWNER_DMA,
            DW0_SIZE_MASK, DW0_SUC_EOF,
        };
        use blueos_hal::dma::{DmaChannel, DmaCaps, DmaDirection, DmaPeriphId, DmaSlaveConfig, DmaWidth};
        use blueos_test_macro::test;

        fn cfg(direction: DmaDirection) -> DmaSlaveConfig {
            DmaSlaveConfig {
                direction,
                width: DmaWidth::Bits8,
                periph: DmaPeriphId(0),
                periph_addr: 0,
            }
        }

        // ── for_tx / for_rx bit-fields ───────────────────────────────

        #[test]
        fn for_tx_sets_size_length_owner_and_eof() {
            let buf = [0u8; 1]; // dummy; address value is not asserted here
            let d = DmaDescriptor::for_tx(buf.as_ptr() as *mut u8, 100, true);
            assert_eq!(d.dw0 & DW0_SIZE_MASK, 100, "size field");
            assert_eq!((d.dw0 >> DW0_LENGTH_SHIFT) & DW0_SIZE_MASK, 100, "length field");
            assert_ne!(d.dw0 & DW0_OWNER_DMA, 0, "owner = DMA");
            assert_ne!(d.dw0 & DW0_SUC_EOF, 0, "suc_eof set when eof=true");
        }

        #[test]
        fn for_tx_clears_eof_when_not_last() {
            let buf = [0u8; 1];
            let d = DmaDescriptor::for_tx(buf.as_ptr() as *mut u8, 50, false);
            assert_eq!(d.dw0 & DW0_SUC_EOF, 0, "suc_eof cleared when eof=false");
            assert_ne!(d.dw0 & DW0_OWNER_DMA, 0, "owner still DMA");
        }

        #[test]
        fn for_rx_sets_owner_and_no_eof() {
            let mut buf = [0u8; 64];
            let d = DmaDescriptor::for_rx(buf.as_mut_ptr(), 64);
            assert_ne!(d.dw0 & DW0_OWNER_DMA, 0, "owner = DMA");
            assert_eq!(d.dw0 & DW0_SUC_EOF, 0, "suc_eof not set on rx descriptor");
            assert_eq!(d.dw0 & DW0_SIZE_MASK, 64, "size = capacity");
            assert_eq!(
                (d.dw0 >> DW0_LENGTH_SHIFT) & DW0_SIZE_MASK,
                0,
                "length starts at 0 (filled by hardware)"
            );
        }

        #[test]
        fn received_len_round_trip() {
            let mut buf = [0u8; 1];
            let mut d = DmaDescriptor::for_rx(buf.as_mut_ptr(), 0);
            // Simulate the hardware writing the received length.
            let want = 0x123;
            d.dw0 = (d.dw0 & !(DW0_SIZE_MASK << DW0_LENGTH_SHIFT))
                | (want << DW0_LENGTH_SHIFT);
            assert_eq!(d.received_len(), want as usize);
        }

        // ── ring / chain linkage ──────────────────────────────────────

        #[test]
        fn link_cyclic_closes_the_ring() {
            let chan = Esp32c6GdmaChannel::<0>;
            let mut descs = [
                DmaDescriptor::default(),
                DmaDescriptor::default(),
                DmaDescriptor::default(),
            ];
            DmaChannel::link_cyclic(&chan, &mut descs);
            assert_eq!(
                descs[2].next as usize,
                &descs[0] as *const _ as usize,
                "tail.next must point back to head"
            );
            assert_eq!(
                descs[0].next as usize,
                &descs[1] as *const _ as usize,
                "head.next must point to second"
            );
        }

        #[test]
        fn link_chain_terminates_with_null() {
            let chan = Esp32c6GdmaChannel::<0>;
            let mut descs = [
                DmaDescriptor::default(),
                DmaDescriptor::default(),
                DmaDescriptor::default(),
            ];
            DmaChannel::link_chain(&chan, &mut descs);
            assert_eq!(
                descs[2].next as usize,
                0,
                "tail.next must be null (chain terminator)"
            );
            assert_ne!(
                descs[0].next as usize,
                0,
                "non-tail next must not be null"
            );
        }

        #[test]
        fn refill_keeps_linkage_and_resets_fields() {
            let chan = Esp32c6GdmaChannel::<0>;
            let mut desc = DmaDescriptor::default();
            let link_target = desc.next;
            let mut buf = [0xAAu8; 32];
            DmaChannel::refill(&chan, &mut desc, &mut buf, true);
            assert_eq!(desc.next, link_target, "next pointer must be untouched");
            assert_ne!(desc.dw0 & DW0_OWNER_DMA, 0, "owner = DMA after refill");
            assert_ne!(desc.dw0 & DW0_SUC_EOF, 0, "eof flag set when requested");
            assert_eq!(desc.dw0 & DW0_SIZE_MASK, 32, "size = buf len");
        }

        // ── owner reclamation ─────────────────────────────────────────

        #[test]
        fn is_consumed_true_when_owner_cleared() {
            let chan = Esp32c6GdmaChannel::<0>;
            let desc = DmaDescriptor::empty(); // owner bit clear → DMA has consumed it
            assert!(DmaChannel::is_consumed(&chan, &desc));
        }

        #[test]
        fn is_consumed_false_when_owner_set() {
            let chan = Esp32c6GdmaChannel::<0>;
            let desc = DmaDescriptor {
                dw0: DW0_OWNER_DMA, // owner still CPU
                ..DmaDescriptor::empty()
            };
            assert!(!DmaChannel::is_consumed(&chan, &desc));
        }

        // ── capability set ────────────────────────────────────────────

        #[test]
        fn caps_self_containment_and_union() {
            assert!(DmaCaps::MEMCPY.contains(DmaCaps::MEMCPY));
            assert!(DmaCaps::MEMCPY.union(DmaCaps::SLAVE).contains(DmaCaps::SLAVE));
            assert!(!DmaCaps::EMPTY.contains(DmaCaps::MEMCPY));
        }

        // ── prep_desc direction dispatch ──────────────────────────────

        #[test]
        fn prep_desc_m2m_yields_tx_descriptor() {
            let chan = Esp32c6GdmaChannel::<0>;
            let mut buf = [0u8; 16];
            let d = DmaChannel::prep_desc(&chan, &mut buf, &cfg(DmaDirection::MemToMem), true)
                .expect("prep_desc M2M");
            assert_ne!(d.dw0 & DW0_OWNER_DMA, 0, "M2M → TX descriptor (owner=DMA)");
            assert_ne!(d.dw0 & DW0_SUC_EOF, 0, "eof requested");
        }

        #[test]
        fn prep_desc_m2p_yields_tx_descriptor() {
            let chan = Esp32c6GdmaChannel::<0>;
            let mut buf = [0u8; 16];
            let d = DmaChannel::prep_desc(&chan, &mut buf, &cfg(DmaDirection::MemToPeriph), true)
                .expect("prep_desc M2P");
            assert_ne!(d.dw0 & DW0_OWNER_DMA, 0, "M2P → TX descriptor");
        }

        #[test]
        fn prep_desc_p2m_yields_rx_descriptor() {
            let chan = Esp32c6GdmaChannel::<0>;
            let mut buf = [0u8; 16];
            let d = DmaChannel::prep_desc(&chan, &mut buf, &cfg(DmaDirection::PeriphToMem), false)
                .expect("prep_desc P2M");
            assert_ne!(d.dw0 & DW0_OWNER_DMA, 0, "P2M → RX descriptor (owner=DMA)");
            assert_eq!(d.dw0 & DW0_SUC_EOF, 0, "P2M never sets suc_eof on prep");
        }
    }

    // ─────────────────────────────────────────────────────────────────
    //  Tier 1 — M2M poll baseline (real hardware, no interrupt).
    //  Drives the ESP32-C6 GDMA controller in memory-to-memory mode via the
    //  poll-only `m2m_transfer` helper (ported from the VDC driver): outlink
    //  reads `src`, inlink writes `dst`, `MEM_TRANS_EN` bridges the TX FIFO
    //  back into the RX FIFO, completion is observed on the RX side as
    //  `IN_SUC_EOF`. Gated on `soc_esp32c6`; must be run on a real
    //  ESP32-C6 devkit via `probe-rs` (QEMU does not model GDMA).
    //
    //  Channel 2 is used for the poll path: channels 0/1 are reserved for
    //  I2S TX/RX, and the poll path needs no interrupt routing (only OUT0's
    //  interrupt is wired at the board level, and this test does not enable
    //  interrupts). This mirrors the VDC `/dev/gdma_test` device's channel
    //  choice.
    // ─────────────────────────────────────────────────────────────────
    #[cfg(soc_esp32c6)]
    mod gdma_m2m {
        use super::*;
        use blueos_driver::dma::esp32c6_gdma::{
            capture_gdma_status, Esp32c6GdmaChannel,
        };
        use blueos_test_macro::test;

        /// M2M test buffer size (matches VDC `/dev/gdma_test`).
        const TEST_BUF_SIZE: usize = 256;

        /// Channel 2: avoids I2S TX/RX (0/1) and needs no interrupt routing.
        type TestChannel = Esp32c6GdmaChannel<2>;

        /// Print a diagnostic line. Uses semihosting (or defmt when
        /// `use_defmt` is configured), matching the kernel test runner.
        macro_rules! diag {
            ($($t:tt)*) => {{
                #[cfg(use_defmt)]
                use defmt::println;
                #[cfg(not(use_defmt))]
                use semihosting::println;
                println!($($t)*);
            }};
        }

        #[test]
        fn test_dma_m2m_poll() {
            // Statically allocate the buffers so their addresses are stable
            // and sit in internal SRAM (the DMA descriptor address field is
            // 20 bits, so buffers must live in the low 1 MiB).
            static mut SRC_BUF: [u8; TEST_BUF_SIZE] = [0; TEST_BUF_SIZE];
            static mut DST_BUF: [u8; TEST_BUF_SIZE] = [0; TEST_BUF_SIZE];

            // SAFETY: single-threaded unit-test context, no aliasing of
            // these `static mut`s by anyone else.
            let (src, dst) = unsafe {
                (
                    &mut *core::ptr::addr_of_mut!(SRC_BUF),
                    &mut *core::ptr::addr_of_mut!(DST_BUF),
                )
            };

            // Fill source with a recognizable, non-trivial pattern.
            for (i, b) in src.iter_mut().enumerate() {
                *b = (i as u8).wrapping_mul(7).wrapping_add(0xAB);
            }
            // Destination starts zeroed.
            dst.fill(0);

            diag!(
                "[GDMA] M2M: src=0x{:08x} dst=0x{:08x} len={}",
                src.as_ptr() as usize,
                dst.as_ptr() as usize,
                src.len()
            );

            // Run the poll-only M2M transfer.
            let result = TestChannel::m2m_transfer(src, dst);

            if result.is_err() {
                let status = capture_gdma_status();
                diag!("[GDMA] M2M FAILED, status: {}", status);
            }
            result.expect("m2m_transfer succeeded");

            // Byte-for-byte comparison.
            let mut mismatches = 0usize;
            let mut first_mismatch: Option<usize> = None;
            for i in 0..TEST_BUF_SIZE {
                if src[i] != dst[i] {
                    mismatches += 1;
                    if first_mismatch.is_none() {
                        first_mismatch = Some(i);
                    }
                }
            }
            assert_eq!(
                mismatches, 0,
                "M2M data mismatch: {} bytes differ, first at idx {:?} (src=0x{:02x} dst=0x{:02x})",
                mismatches,
                first_mismatch,
                first_mismatch.map(|i| src[i]).unwrap_or(0),
                first_mismatch.map(|i| dst[i]).unwrap_or(0),
            );
        }
    }

    // ─────────────────────────────────────────────────────────────────
    //  Tier 2 — Interrupt callback validation (real hardware, core path).
    //  Verifies that `set_callback` + `enable_interrupt` + `service_interrupt`
    //  actually fire the registered callback with the correct `DmaEvent` when
    //  a transfer completes. This is the design's central interrupt-path
    //  validation.
    //
    //  Must use channel 0 (TX/out): the board routes only GDMA OUT_CH0
    //  (source 74) to CPU line 17, and `handle_intc_irq`'s
    //  `GDMA_OUT0_INT_NUM` branch dispatches `GDMA_OUT0_ISR.service_isr()`,
    //  which calls `DMA0_TX.service_interrupt()`. The test thread runs with
    //  MIE enabled (the scheduler enables local IRQ on entry to `schedule`),
    //  so a hardware completion interrupt is delivered to the callback.
    //
    //  Two sub-cases mirror `dmatest`'s oneshot-vs-cyclic split:
    //   - 2A: a single-descriptor oneshot chain → `OneshotDone(cookie)`.
    //   - 2B: a 4-segment perpetual ring → `SegmentConsumed(idx)`.
    //  Both wait with a bounded poll budget so a lost interrupt fails the
    //  test instead of hanging the harness.
    // ─────────────────────────────────────────────────────────────────
    #[cfg(soc_esp32c6)]
    mod gdma_irq {
        use super::*;
        use blueos_driver::dma::esp32c6_gdma::{
            capture_gdma_status, DmaDescriptor, Esp32c6GdmaChannel,
        };
        use blueos_hal::dma::{
            DmaChannel, DmaDirection, DmaEvent, DmaPeriphId, DmaSlaveConfig, DmaWidth,
        };
        use blueos_test_macro::test;
        use core::sync::atomic::{AtomicUsize, Ordering};

        /// The board-routed channel. Only OUT0's interrupt is wired, so the
        /// interrupt sub-tests must use channel 0. ZST — `new()` is a
        /// zero-cost instance for method calls.
        type IrqChannel = Esp32c6GdmaChannel<0>;
        fn chan() -> IrqChannel {
            Esp32c6GdmaChannel::<0>::new()
        }

        /// How many `yield_me()` spins to budget before declaring the
        /// interrupt lost. Each yield reschedules; the DMA transfer completes
        /// in microseconds, so this is generous.
        const IRQ_WAIT_BUDGET: usize = 1_000_000;

        macro_rules! diag {
            ($($t:tt)*) => {{
                #[cfg(use_defmt)]
                use defmt::println;
                #[cfg(not(use_defmt))]
                use semihosting::println;
                println!($($t)*);
            }};
        }

        fn m2p_cfg() -> DmaSlaveConfig {
            DmaSlaveConfig {
                direction: DmaDirection::MemToPeriph,
                width: DmaWidth::Bits8,
                periph: DmaPeriphId(0),
                periph_addr: 0,
            }
        }

        /// Spin (yielding to the scheduler) until `flag` reaches `expected`,
        /// or `IRQ_WAIT_BUDGET` yields elapse. Returns `true` if the flag was
        /// reached, `false` on timeout.
        fn wait_for_flag(flag: &AtomicUsize, expected: usize) -> bool {
            for _ in 0..IRQ_WAIT_BUDGET {
                if flag.load(Ordering::Acquire) == expected {
                    return true;
                }
                scheduler::yield_me();
            }
            flag.load(Ordering::Acquire) == expected
        }

        // ── 2A: oneshot chain → OneshotDone(cookie) ──────────────────

        static DONE_FLAG: AtomicUsize = AtomicUsize::new(0);
        static DONE_COOKIE: AtomicUsize = AtomicUsize::new(0);

        fn oneshot_cb(e: DmaEvent) {
            if let DmaEvent::OneshotDone(c) = e {
                DONE_COOKIE.store(c, Ordering::Release);
                DONE_FLAG.store(1, Ordering::Release);
            }
        }

        #[test]
        fn test_dma_oneshot_irq() {
            // Reset the shared flags before registering.
            DONE_FLAG.store(0, Ordering::Release);
            DONE_COOKIE.store(usize::MAX, Ordering::Release);

            // Build a 1-descriptor oneshot chain (eof=true) in static memory.
            static mut TX_DESC: DmaDescriptor = DmaDescriptor::empty();
            static mut TX_BUF: [u8; 64] = [0xA5; 64];
            // SAFETY: single-threaded test context; no aliasing.
            let desc = unsafe { &mut *core::ptr::addr_of_mut!(TX_DESC) };
            let buf = unsafe { &mut *core::ptr::addr_of_mut!(TX_BUF) };
            let ch = chan();
            *desc = DmaChannel::prep_desc(&ch, buf, &m2p_cfg(), true)
                .expect("prep_desc");
            DmaChannel::link_chain(&ch, core::slice::from_mut(desc));

            DmaChannel::set_callback(&ch, oneshot_cb);
            DmaChannel::enable_interrupt(&ch);
            // start_chain hardcodes oneshot_cookie=0 (the trait has no cookie
            // parameter), so the callback must report OneshotDone(0).
            DmaChannel::start_chain(&ch, desc, &m2p_cfg())
                .expect("start_chain");

            let ok = wait_for_flag(&DONE_FLAG, 1);
            if !ok {
                // Diagnostic: did TX actually complete? Use the TX-side
                // total-EOF poll helper to tell "TX done but callback lost"
                // from "TX never completed".
                let tx_done = Esp32c6GdmaChannel::<0>::is_tx_total_eof();
                let status = capture_gdma_status();
                diag!(
                    "[GDMA] oneshot IRQ timeout: tx_done={}, status: {}",
                    tx_done,
                    status
                );
            }
            assert!(ok, "oneshot completion interrupt did not fire within budget");

            // The callback contract: cookie == 0 (start_chain hardcodes it).
            assert_eq!(
                DONE_COOKIE.load(Ordering::Acquire),
                0,
                "OneshotDone cookie must be 0 (start_chain hardcodes oneshot_cookie=0)"
            );

            // Tear down: disable the interrupt and reset the channel FSM.
            DmaChannel::disable_interrupt(&ch);
            DmaChannel::terminate(&ch).expect("terminate");
        }

        // ── 2B: cyclic ring → SegmentConsumed(idx) ────────────────────

        static SEG_COUNT: AtomicUsize = AtomicUsize::new(0);
        static LAST_SEG: AtomicUsize = AtomicUsize::new(usize::MAX);

        fn cyclic_cb(e: DmaEvent) {
            if let DmaEvent::SegmentConsumed(i) = e {
                LAST_SEG.store(i, Ordering::Release);
                SEG_COUNT.fetch_add(1, Ordering::Release);
            }
        }

        #[test]
        fn test_dma_cyclic_irq() {
            const NSEGS: usize = 4;
            const SEG_BYTES: usize = 64;

            SEG_COUNT.store(0, Ordering::Release);
            LAST_SEG.store(usize::MAX, Ordering::Release);

            // 4-segment ring in static memory.
            static mut RING: [DmaDescriptor; NSEGS] =
                [DmaDescriptor::empty(); NSEGS];
            static mut RING_BUFS: [[u8; SEG_BYTES]; NSEGS] =
                [[0xCC; SEG_BYTES]; NSEGS];
            // SAFETY: single-threaded test context; no aliasing.
            let descs = unsafe { &mut *core::ptr::addr_of_mut!(RING) };
            let bufs = unsafe { &mut *core::ptr::addr_of_mut!(RING_BUFS) };

            // Prepare each segment as a TX descriptor (no eof — a ring has no
            // "last" descriptor).
            let ch = chan();
            let cfg = m2p_cfg();
            for i in 0..NSEGS {
                descs[i] = DmaChannel::prep_desc(&ch, &mut bufs[i], &cfg, false)
                    .expect("prep_desc");
            }
            DmaChannel::link_cyclic(&ch, descs);

            DmaChannel::set_callback(&ch, cyclic_cb);
            DmaChannel::enable_interrupt(&ch);
            DmaChannel::start_ring(&ch, &descs[0], &cfg, NSEGS)
                .expect("start_ring");

            // Wait until the DMA has consumed at least one full ring traversal
            // (NSEGS segments), so the incremental-dispatch path in
            // service_interrupt is exercised.
            let ok = wait_for_flag(&SEG_COUNT, NSEGS);
            if !ok {
                let status = capture_gdma_status();
                diag!(
                    "[GDMA] cyclic IRQ timeout: seg_count={}, last_seg={:?}, status: {}",
                    SEG_COUNT.load(Ordering::Acquire),
                    LAST_SEG.load(Ordering::Acquire),
                    status
                );
            }
            assert!(
                ok,
                "cyclic ring did not consume {} segments within budget (got {})",
                NSEGS,
                SEG_COUNT.load(Ordering::Acquire)
            );

            // The reported segment index must be within [0, NSEGS).
            let last = LAST_SEG.load(Ordering::Acquire);
            assert!(
                last < NSEGS,
                "SegmentConsumed idx {} out of range [0, {})",
                last,
                NSEGS
            );

            // Tear down: disable interrupt and break the ring.
            DmaChannel::disable_interrupt(&ch);
            DmaChannel::terminate(&ch).expect("terminate");
        }
    }
}
