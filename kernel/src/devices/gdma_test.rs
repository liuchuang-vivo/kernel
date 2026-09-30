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

//! GDMA memory-to-memory character device.
//!
//! Exposes `/dev/gdma_test` so userspace can drive a GDMA M2M transfer
//! through the kernel driver. Two ioctl commands are supported:
//!
//! - `CMD_M2M_TEST` (0x1001): the device allocates both buffers internally,
//!   fills the source with a known pattern, runs the DMA copy, and reports
//!   PASS/FAIL via the kernel log. Kept for backward compatibility with the
//!   boot-time self-test.
//!
//! - `CMD_M2M_USER` (0x1002): the device runs *one* GDMA M2M copy between two
//!   caller-supplied buffers. The `arg` points to an `M2mUserReq { src, dst,
//!   len }` describing the transfer. The device does NOT allocate, fill, or
//!   compare the buffers — it is a pure "move these bytes via GDMA" primitive.
//!   The caller owns buffer lifetime, contents, and verification.
//!
//! Both commands reuse the driver-level M2M path (poll, channel 2 — avoids
//! I2S channels 0/1 and needs no interrupt wiring) reached through the
//! standard `DmaChannel::m2m_transfer` entry point rather than the trait's
//! `start` (single outlink, cannot do full M2M).
//!
//! # Why userspace-supplied pointers are safe
//!
//! ESP32-C6 is RV32IMAC with no MMU: `CONFIG_KERNEL_VIRT_OFFSET = 0x0`, so
//! `kernel_phys_to_virt(addr) == addr` (see `kernel/kernel/src/mm.rs`).
//! Userspace heap pointers (from `posix_memalign` → `blueos::allocator`)
//! ARE physical SRAM addresses — the GDMA engine reads them directly with
//! no address translation and no cache maintenance (SRAM is uncached on
//! ESP32-C6; only flash goes through the cache).

use crate::devices::{Device, DeviceClass, DeviceId, DeviceManager};
use alloc::{format, string::String, sync::Arc};
use blueos_driver::dma::esp32c6_gdma::Esp32c6GdmaChannel;
use blueos_hal::dma::{DmaChannel, DmaEvent};
use core::ptr::addr_of_mut;
use core::sync::atomic::{AtomicUsize, Ordering};
use embedded_io::ErrorKind;

/// ioctl command: run the self-contained M2M self-test. The device allocates
/// both buffers, fills the source, copies via DMA, and compares. `arg` is
/// ignored. Result goes to the kernel log; `Ok` = pass, `Err` = fail.
pub const CMD_M2M_TEST: u32 = 0x1001;

/// ioctl command: run one GDMA M2M copy between caller-supplied buffers.
/// `arg` points to an `M2mUserReq`. The device does not allocate, fill, or
/// compare — it is a pure DMA-move primitive. Returns `Ok` on a successful
/// transfer (no data-comparison), `Err` on a hardware error.
pub const CMD_M2M_USER: u32 = 0x1002;

/// ioctl command: run the interrupt-driven M2M verification test. `arg` is
/// ignored. Uses channel 0 (the only channel whose IN0/OUT0 interrupts are
/// wired at the board level). Sets up the full M2M bridge (`MEM_TRANS_EN`
/// + TX/RX descriptors), registers a completion callback, and arms both
/// IN+OUT interrupts; the RX-side `IN_SUC_EOF` (the true M2M completion the
/// poll path waits on) fires the board's IN0 ISR → `service_interrupt`
/// (RX block) → callback → a flag the handler busy-waits on (MIE is on
/// during ioctl handlers via `SyscallGuard`, so the interrupt is delivered
/// mid-spin). Also verifies the M2M data copy (`dst == src`) after the
/// interrupt fires. Returns `Ok` on pass, `Err` on timeout or data
/// mismatch.
pub const CMD_M2M_IRQ_TEST: u32 = 0x1003;

/// Caller-supplied M2M transfer descriptor, passed via `ioctl(CMD_M2M_USER)`
/// `arg`. `src` and `dst` are userspace buffer pointers; `len` is the byte
/// count (must be equal for both buffers and ≤ 4095 — the GDMA descriptor
/// `size` field is 12 bits, see `DW0_SIZE_MASK`).
///
/// `#[repr(C)]` makes the layout ABI-stable across the userspace app and
/// the kernel ioctl handler. The struct is small enough to pass by pointer
/// through the single `arg: usize` ioctl argument.
#[repr(C)]
pub struct M2mUserReq {
    /// Source buffer (write-back side) address.
    pub src: usize,
    /// Destination buffer (read-back side) address.
    pub dst: usize,
    /// Transfer length in bytes (≤ 4095).
    pub len: usize,
}

/// Buffer size for the M2M test (must be word-aligned).
const TEST_BUF_SIZE: usize = 256;

/// GDMA channel 2 is used for the M2M test (channels 0/1 are reserved for
/// I2S TX/RX). Poll path — no interrupt wiring required.
type TestChannel = Esp32c6GdmaChannel<2>;

/// Channel 0 for the IRQ test: the only channel whose IN0/OUT0 interrupts
/// are wired at the board level (`GDMA_IN0_ISR` / `GDMA_OUT0_ISR`).
type IrqChannel = Esp32c6GdmaChannel<0>;

/// IRQ test buffer size (matches the poll test's `TEST_BUF_SIZE`).
const IRQ_BUF_SIZE: usize = 256;

/// Busy-wait budget for the IRQ test (matches `tests/gdma_irq.rs`).
const IRQ_WAIT_BUDGET: usize = 1_000_000;

/// Set by the ISR callback, read by `run_irq_test`'s bounded-wait loop.
static IRQ_DONE: AtomicUsize = AtomicUsize::new(0);

/// Completion callback for `CMD_M2M_IRQ_TEST`. Runs in interrupt context
/// (called from `service_interrupt` via the board's `GDMA_IN0_ISR`); must
/// not sleep or re-enter the channel's irq-save lock (re-entrancy contract,
/// `hal/dma.rs` `DmaCallback` docs).
fn irq_cb(e: DmaEvent) {
    if let DmaEvent::OneshotDone(_) = e {
        IRQ_DONE.store(1, Ordering::Release);
    }
}

pub struct GdmaTestDevice {
    src: core::sync::atomic::AtomicUsize,
    dst: core::sync::atomic::AtomicUsize,
}

impl GdmaTestDevice {
    pub fn new() -> Self {
        Self {
            src: core::sync::atomic::AtomicUsize::new(0),
            dst: core::sync::atomic::AtomicUsize::new(0),
        }
    }

    pub fn register(self) -> Result<(), ErrorKind> {
        let device = Arc::new(self);
        DeviceManager::get().register_device(String::from("gdma_test"), device)
    }

    /// Run the M2M DMA test: fill a source buffer with a known pattern, copy
    /// it to a destination buffer via GDMA, then compare. Returns a human-
    /// readable result string.
    pub fn run_m2m_test(&self) -> String {
        // Use a heap allocation that we know is in SRAM.
        let mut src: alloc::vec::Vec<u8> = alloc::vec![0u8; TEST_BUF_SIZE];
        let mut dst: alloc::vec::Vec<u8> = alloc::vec![0u8; TEST_BUF_SIZE];

        // Fill source with a recognizable pattern.
        for (i, byte) in src.iter_mut().enumerate() {
            *byte = (i as u8).wrapping_mul(7).wrapping_add(0xAB);
        }
        // Destination starts zeroed.
        dst.fill(0);

        // Run the DMA transfer via the standard `DmaChannel` M2M method.
        let result = <TestChannel as DmaChannel>::m2m_transfer(&mut src, &mut dst);

        // Diagnostic: log buffer/descriptor addresses
        log::info!(
            "[GDMA] M2M: src=0x{:08x} dst=0x{:08x} len={}",
            src.as_ptr() as usize,
            dst.as_ptr() as usize,
            src.len()
        );

        match result {
            Ok(()) => {
                // Compare byte-by-byte.
                let mut mismatches = 0;
                let mut first_mismatch = None;
                for i in 0..TEST_BUF_SIZE {
                    if src[i] != dst[i] {
                        mismatches += 1;
                        if first_mismatch.is_none() {
                            first_mismatch = Some(i);
                        }
                    }
                }
                if mismatches == 0 {
                    format!(
                        "M2M DMA test PASSED: {} bytes copied correctly",
                        TEST_BUF_SIZE
                    )
                } else {
                    format!(
                        "M2M DMA test FAILED: {}/{} bytes mismatch, first at offset {}",
                        mismatches,
                        TEST_BUF_SIZE,
                        first_mismatch.unwrap_or(0)
                    )
                }
            }
            Err(e) => {
                let status = blueos_driver::dma::esp32c6_gdma::capture_gdma_status();
                log::error!("[GDMA] M2M DMA error: {:?} | {}", e, status);
                format!("M2M DMA test ERROR: {:?} | {}", e, status)
            }
        }
    }

    /// Run one GDMA M2M copy between caller-supplied buffers. Unlike
    /// [`run_m2m_test`](Self::run_m2m_test), this does NOT allocate, fill, or
    /// compare — the caller owns buffer lifetime and contents. Returns `Ok`
    /// when the DMA engine reports `IN_SUC_EOF` (transfer complete), `Err`
    /// on a hardware error.
    ///
    /// # Safety
    ///
    /// The caller must pass valid SRAM pointers with `len` bytes accessible
    /// on both sides. On ESP32-C6 (no MMU, `VIRT_OFFSET = 0`) userspace
    /// pointers are physical SRAM addresses, so no translation is needed.
    pub fn run_m2m_user(&self, req: &M2mUserReq) -> Result<(), ErrorKind> {
        // GDMA descriptor `size` field is 12 bits (DW0_SIZE_MASK = 0xfff).
        if req.len == 0 || req.len > 0xfff {
            return Err(ErrorKind::InvalidInput);
        }
        if req.src == 0 || req.dst == 0 {
            return Err(ErrorKind::InvalidInput);
        }

        // Build mutable slices over caller memory and let the driver move
        // the bytes. `m2m_transfer` takes `&mut [u8]` on both sides; the src
        // side is only read by the DMA outlink, so the `&mut` aliasing is
        // benign in practice.
        // SAFETY: the caller guarantees `src`/`dst` point to accessible SRAM
        // of `len` bytes. No MMU → the pointers are physical addresses.
        let src = unsafe { core::slice::from_raw_parts_mut(req.src as *mut u8, req.len) };
        let dst = unsafe { core::slice::from_raw_parts_mut(req.dst as *mut u8, req.len) };

        log::info!(
            "[GDMA] M2M user: src=0x{:08x} dst=0x{:08x} len={}",
            req.src,
            req.dst,
            req.len
        );

        match <TestChannel as DmaChannel>::m2m_transfer(src, dst) {
            Ok(()) => Ok(()),
            Err(e) => {
                let status = blueos_driver::dma::esp32c6_gdma::capture_gdma_status();
                log::error!("[GDMA] M2M user error: {:?} | {}", e, status);
                Err(ErrorKind::Other)
            }
        }
    }

    /// Run the interrupt-driven M2M verification test. Sets up the M2M
    /// bridge on channel 0 via the driver's `m2m_transfer_irq` helper
    /// (which registers `irq_cb` on the kernel's `CHAN_STATE[0]` — the
    /// copy the board's `GDMA_IN0_ISR` reads), arms both IN+OUT interrupts,
    /// and starts TX+RX. Completion arrives asynchronously as RX-side
    /// `IN_SUC_EOF` (the true M2M completion) → IN0 ISR →
    /// `service_interrupt` → `irq_cb` → flag, which this handler
    /// busy-waits on (MIE is on during ioctl handlers via `SyscallGuard`).
    /// Also verifies the M2M data copy after the interrupt fires.
    pub fn run_irq_test(&self) -> Result<(), ErrorKind> {
        IRQ_DONE.store(0, Ordering::Release);

        // Static buffers so descriptors/pointers outlive the call (mirrors
        // `tests/gdma_m2m.rs`). Channel 0's descriptors sit in the low
        // 1 MiB SRAM the GDMA 20-bit address field can reach.
        static mut TX_BUF: [u8; IRQ_BUF_SIZE] = [0; IRQ_BUF_SIZE];
        static mut RX_BUF: [u8; IRQ_BUF_SIZE] = [0; IRQ_BUF_SIZE];
        // SAFETY: single ioctl call; no aliasing of these `static mut`s by
        // anyone else (the DMA engine reads TX / writes RX).
        let (tx, rx) = unsafe {
            (
                &mut *addr_of_mut!(TX_BUF),
                &mut *addr_of_mut!(RX_BUF),
            )
        };

        // Fill TX with a recognizable pattern; zero RX.
        for (i, b) in tx.iter_mut().enumerate() {
            *b = (i as u8).wrapping_mul(7).wrapping_add(0xAB);
        }
        rx.fill(0);

        log::info!(
            "[GDMA] M2M IRQ: tx=0x{:08x} rx=0x{:08x} len={}",
            tx.as_ptr() as usize,
            rx.as_ptr() as usize,
            tx.len()
        );

        let ch = IrqChannel::new();

        // Kick off the interrupt-driven M2M transfer. Sets up the full
        // bridge, registers `irq_cb` on the kernel's `CHAN_STATE[0]`
        // (the copy `GDMA_IN0_ISR` reads), arms IN+OUT interrupts, starts
        // TX+RX. Returns immediately; completion comes via the IN0 ISR.
        if let Err(e) = ch.m2m_transfer_irq(tx, rx, irq_cb) {
            let status = blueos_driver::dma::esp32c6_gdma::capture_gdma_status();
            log::error!("[GDMA] M2M IRQ setup error: {:?} | {}", e, status);
            let _ = ch.terminate();
            return Err(ErrorKind::Other);
        }

        // Bounded busy-wait for the IN0 interrupt. The handler runs in the
        // app thread's syscall context (MIE on via `SyscallGuard`), not a
        // scheduler thread — so it must NOT call `scheduler::yield_me` (that
        // asserts a scheduler context). The IN0 interrupt is delivered
        // during the spin.
        let mut timed_out = true;
        for _ in 0..IRQ_WAIT_BUDGET {
            if IRQ_DONE.load(Ordering::Acquire) == 1 {
                timed_out = false;
                break;
            }
            core::hint::spin_loop();
        }

        // Stop the interrupt regardless of outcome.
        ch.disable_interrupt();

        if timed_out {
            let status = blueos_driver::dma::esp32c6_gdma::capture_gdma_status();
            log::error!(
                "[GDMA] M2M IRQ test FAILED: timeout (no IN_SUC_EOF) | {}",
                status
            );
            let _ = ch.terminate();
            return Err(ErrorKind::Other);
        }

        // The interrupt fired — verify the data copy too (the M2M bridge
        // moved tx→rx). A mismatch means the interrupt path fired but the
        // DMA engine did not actually complete the copy correctly.
        let mut mismatches = 0usize;
        let mut first_mismatch: Option<usize> = None;
        for i in 0..IRQ_BUF_SIZE {
            if tx[i] != rx[i] {
                mismatches += 1;
                if first_mismatch.is_none() {
                    first_mismatch = Some(i);
                }
            }
        }

        // M2M cleanup: disable the bridge. `terminate` resets the TX/RX
        // FSM and clears `CHAN_STATE[0]`.
        let _ = ch.terminate();

        if mismatches != 0 {
            log::error!(
                "[GDMA] M2M IRQ test FAILED: {}/{} bytes mismatch, first at offset {}",
                mismatches,
                IRQ_BUF_SIZE,
                first_mismatch.unwrap_or(0)
            );
            return Err(ErrorKind::Other);
        }

        log::info!(
            "[GDMA] M2M IRQ test PASSED: {} bytes copied via interrupt path",
            IRQ_BUF_SIZE
        );
        Ok(())
    }
}

impl Device for GdmaTestDevice {
    fn name(&self) -> String {
        String::from("gdma_test")
    }

    fn class(&self) -> DeviceClass {
        DeviceClass::Char
    }

    fn id(&self) -> DeviceId {
        DeviceId::new(1, 11)
    }

    fn open(&self) -> Result<(), ErrorKind> {
        Ok(())
    }

    fn read(&self, _pos: u64, _buf: &mut [u8], _is_nonblocking: bool) -> Result<usize, ErrorKind> {
        Err(ErrorKind::Unsupported)
    }

    fn write(&self, _pos: u64, _buf: &[u8], _is_nonblocking: bool) -> Result<usize, ErrorKind> {
        Err(ErrorKind::Unsupported)
    }

    fn ioctl(&self, request: u32, _arg: usize) -> Result<(), ErrorKind> {
        match request {
            CMD_M2M_TEST => {
                let result = self.run_m2m_test();
                log::info!("[GDMA] {}", result);
                if result.contains("PASSED") {
                    Ok(())
                } else {
                    Err(ErrorKind::Other)
                }
            }
            CMD_M2M_USER => {
                // `arg` points to an `M2mUserReq` in the caller's address
                // space. No MMU on ESP32-C6 → the pointer is a physical SRAM
                // address the kernel can dereference directly.
                if _arg == 0 {
                    return Err(ErrorKind::InvalidInput);
                }
                let req = unsafe { &*(_arg as *const M2mUserReq) };
                self.run_m2m_user(req)
            }
            // Interrupt-driven M2M verification. `arg` is ignored — the
            // device owns its own buffers (static `TX_BUF`/`RX_BUF`), so
            // there is nothing for the caller to pass. Returns `Ok` only if
            // the RX-side `IN_SUC_EOF` fires the board's IN0 ISR → callback
            // → flag within the bounded wait AND the M2M data copy matches.
            CMD_M2M_IRQ_TEST => self.run_irq_test(),
            _ => Err(ErrorKind::Unsupported),
        }
    }
}
