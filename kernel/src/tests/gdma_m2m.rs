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
    use blueos_hal::dma::DmaChannel;
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

        // Run the poll-only M2M transfer via the standard `DmaChannel`
        // trait method (M2M entry point, delegated to the driver's
        // inherent `m2m_transfer_impl`).
        let result = <TestChannel as DmaChannel>::m2m_transfer(src, dst);

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
