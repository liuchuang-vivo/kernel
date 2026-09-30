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
