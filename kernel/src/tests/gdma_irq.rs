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
    use crate::scheduler;
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
