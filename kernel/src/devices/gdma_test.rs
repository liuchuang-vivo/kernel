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

//! GDMA memory-to-memory self-test character device.
//!
//! Exposes `/dev/gdma_test` so userspace can trigger a GDMA M2M transfer and
//! verify the result. The ioctl `CMD_M2M_TEST` copies an internal source
//! buffer to a destination buffer via DMA and returns the comparison result
//! via the kernel log (0 = pass, -1 = fail).
//!
//! Ported from the VDC `lc/vdc_ui_audio` branch's `kernel/src/devices/
//! gdma_test.rs`. The `Device` trait / `DeviceManager` API on the current
//! branch is identical, so the port is near-verbatim; only the unused
//! `SRC_BUF`/`DST_BUF` AtomicU32 statics (which the original declared but
//! never used — it allocates on the heap) were dropped.
//!
//! This is the Tier 3 deliverable of the DMA test/verification plan
//! (`~/.claude/plans/linux-dma-wild-clock.md`). It reuses the driver-level
//! `m2m_transfer` (poll path, channel 2 — avoids I2S channels 0/1 and needs
//! no interrupt wiring) rather than the trait's `start_chain` (single
//! outlink, cannot do full M2M).

use crate::devices::{Device, DeviceClass, DeviceId, DeviceManager};
use alloc::{format, string::String, sync::Arc};
use blueos_driver::dma::esp32c6_gdma::Esp32c6GdmaChannel;
use embedded_io::ErrorKind;

/// ioctl command: run M2M DMA self-test.
/// The `arg` parameter is ignored; the result is written to the kernel log
/// and returned as the ioctl return value (Ok = pass, Err = fail).
pub const CMD_M2M_TEST: u32 = 0x1001;

/// Buffer size for the M2M test (must be word-aligned).
const TEST_BUF_SIZE: usize = 256;

/// GDMA channel 2 is used for the M2M test (channels 0/1 are reserved for
/// I2S TX/RX). Poll path — no interrupt wiring required.
type TestChannel = Esp32c6GdmaChannel<2>;

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

        // Run the DMA transfer.
        let result = TestChannel::m2m_transfer(&mut src, &mut dst);

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
        if request == CMD_M2M_TEST {
            let result = self.run_m2m_test();
            log::info!("[GDMA] {}", result);
            if result.contains("PASSED") {
                Ok(())
            } else {
                Err(ErrorKind::Other)
            }
        } else {
            Err(ErrorKind::Unsupported)
        }
    }
}
