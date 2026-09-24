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

//! ESP32-C6 General DMA (GDMA) register-level driver.
//!
//! Implements [`blueos_hal::dma::DmaChannel`] for the ESP32-C6 GDMA block
//! (3 RX + 3 TX channels). The channel is a zero-sized type
//! [`Esp32c6GdmaChannel`]`<CH>`; per-channel runtime state lives in the
//! [`static_mut CHAN_STATE`] array indexed by `CH`, keeping the HAL crate
//! dependency-free and heapless.
//!
//! # Interrupt path
//!
//! This driver adds the interrupt + completion-callback layer on top of the
//! pre-existing poll-only register primitives (`reset_tx`, `start_tx`, …).
//! The board wires GDMA OUT0 to a free CPU interrupt line via the interrupt
//! matrix and calls [`DmaChannel::service_interrupt`] from the trap handler
//! (see the board's `handle_intc_irq`). Consumers stop polling
//! `wait_tx_done()` and instead call `set_callback` + `enable_interrupt`.
//!
//! # Concurrency
//!
//! Only the `callback` field of [`ChanState`] is read by the ISR while a
//! thread may write it; that single true race is guarded by disabling local
//! interrupts via the architecture-agnostic [`arch_crate`] primitives
//! (`disable_local_irq_save` / `enable_local_irq_restore`). The other
//! `ChanState` fields are ordered by the "thread writes → enable_interrupt
//! → ISR may run" sequence and need no lock. See the design doc §5.1 for
//! the per-field analysis.

use blueos_hal::dma::{
    DmaCallback, DmaCaps, DmaChannel, DmaDirection, DmaEvent, DmaSlaveConfig,
};
use blueos_hal::{err::Result, isr::IsrDesc, Configuration, PlatPeri};
// Architecture-agnostic IRQ primitives. The driver depends on the arch
// *abstraction* layer only — never on a concrete architecture — so adding
// a future ARM path needs no driver change.
use arch_crate::{disable_local_irq_save, enable_local_irq_restore};

use crate::static_ref::StaticRef;
use tock_registers::{
    interfaces::{Readable, ReadWriteable, Writeable},
    register_bitfields, register_structs,
    registers::ReadWrite,
};

// ─────────────────────────────────────────────────────────────────────────
//  Hardware constants (verified against ESP32-C6 PAC dma.rs / dma/ch/*)
// ─────────────────────────────────────────────────────────────────────────

/// GDMA register-block base.
const DMA_BASE: usize = 0x6008_0000;
/// Offset of the first `CH` cluster within the GDMA block.
const CH_OFFSET: usize = 0x70;
/// Stride between consecutive `CH` clusters.
const CH_STRIDE: usize = 0xC0;
/// Offset of the first `IN_INT_CH` cluster.
const IN_INT_BASE: usize = 0x00;
/// Offset of the first `OUT_INT_CH` cluster.
const OUT_INT_BASE: usize = 0x30;
/// Stride between consecutive interrupt clusters.
const INT_STRIDE: usize = 0x10;

/// `dw0[11:0]` — buffer size in bytes.
pub const DW0_SIZE_MASK: u32 = 0xfff;
/// `dw0[23:12]` — buffer length (number of bytes the DMA may move).
pub const DW0_LENGTH_SHIFT: u32 = 12;
/// `dw0[30]` — set on the last descriptor of an oneshot chain to trigger
/// `OUT_TOTAL_EOF`.
pub const DW0_SUC_EOF: u32 = 1 << 30;
/// `dw0[31]` — owner. CPU sets it; DMA clears it after consumption when
/// `OUT_AUTO_WRBACK=1`.
pub const DW0_OWNER_DMA: u32 = 1 << 31;

/// `PERI_OUT_SEL` / `PERI_IN_SEL` value for I2S0.
#[allow(dead_code)]
const PERI_I2S0: u32 = 3;

/// Polling iteration cap before declaring a DMA transfer timeout.
const DMA_POLL_LIMIT: u32 = 10_000_000;

// ─────────────────────────────────────────────────────────────────────────
//  Diagnostic snapshot (ported from VDC poll-only driver)
// ─────────────────────────────────────────────────────────────────────────

/// Per-channel GDMA register snapshot captured at the point of failure.
///
/// Raw `u32` fields so the snapshot is independent of the `tock-registers`
/// field types — a caller can log it without pulling in the register defs.
#[derive(Copy, Clone)]
pub struct GdmaChStatus {
    pub in_raw: u32,
    pub out_raw: u32,
    pub in_conf0: u32,
    pub in_link: u32,
    pub in_state: u32,
    pub out_conf0: u32,
    pub out_peri_sel: u32,
    pub out_link: u32,
    pub out_state: u32,
}

/// Diagnostic snapshot of the GDMA controller: one entry per channel (3 on
/// ESP32-C6) plus the global `MISC_CONF` register. Returned by
/// [`capture_gdma_status`] when a transfer times out or errors, so the caller
/// can log a single line that captures the controller's state at the point
/// of failure. Ported verbatim from the VDC poll-only driver
/// (`lc/vdc_ui_audio:driver/src/dma/esp32c6_gdma.rs`).
#[derive(Copy, Clone)]
pub struct GdmaStatus {
    pub channels: [GdmaChStatus; 3],
    pub misc_conf: u32,
}

impl core::fmt::Display for GdmaStatus {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        for (ch, s) in self.channels.iter().enumerate() {
            write!(
                f,
                "CH{} IN_RAW=0x{:08x} OUT_RAW=0x{:08x} | ",
                ch, s.in_raw, s.out_raw
            )?;
        }
        for (ch, s) in self.channels.iter().enumerate() {
            write!(
                f,
                "CH{} IN[c0=0x{:08x} lk=0x{:08x} st=0x{:08x}] OUT[c0=0x{:08x} peri=0x{:08x} lk=0x{:08x} st=0x{:08x}] | ",
                ch,
                s.in_conf0,
                s.in_link,
                s.in_state,
                s.out_conf0,
                s.out_peri_sel,
                s.out_link,
                s.out_state
            )?;
        }
        write!(f, "MISC=0x{:08x}", self.misc_conf)
    }
}

/// Capture the interrupt RAW registers and channel state for all 3 GDMA
/// channels. Called on timeout/error and returned to the caller for logging.
///
/// Reads via `core::ptr::read_volatile` (bypassing the typed register structs)
/// so a corrupted register map still yields a snapshot rather than a panic.
pub fn capture_gdma_status() -> GdmaStatus {
    let mut channels = [GdmaChStatus {
        in_raw: 0,
        out_raw: 0,
        in_conf0: 0,
        in_link: 0,
        in_state: 0,
        out_conf0: 0,
        out_peri_sel: 0,
        out_link: 0,
        out_state: 0,
    }; 3];

    for ch in 0..3usize {
        let in_raw = unsafe {
            core::ptr::read_volatile((DMA_BASE + IN_INT_BASE + ch * INT_STRIDE) as *const u32)
        };
        let out_raw = unsafe {
            core::ptr::read_volatile((DMA_BASE + OUT_INT_BASE + ch * INT_STRIDE) as *const u32)
        };
        let ch_base = DMA_BASE + CH_OFFSET + ch * CH_STRIDE;
        let in_conf0 = unsafe { core::ptr::read_volatile((ch_base + 0x00) as *const u32) };
        let in_link = unsafe { core::ptr::read_volatile((ch_base + 0x10) as *const u32) };
        let in_state = unsafe { core::ptr::read_volatile((ch_base + 0x14) as *const u32) };
        let out_conf0 = unsafe { core::ptr::read_volatile((ch_base + 0x60) as *const u32) };
        let out_peri_sel = unsafe { core::ptr::read_volatile((ch_base + 0x90) as *const u32) };
        let out_link = unsafe { core::ptr::read_volatile((ch_base + 0x70) as *const u32) };
        let out_state = unsafe { core::ptr::read_volatile((ch_base + 0x74) as *const u32) };
        channels[ch] = GdmaChStatus {
            in_raw,
            out_raw,
            in_conf0,
            in_link,
            in_state,
            out_conf0,
            out_peri_sel,
            out_link,
            out_state,
        };
    }

    let misc_conf = unsafe { core::ptr::read_volatile((DMA_BASE + 0x64) as *const u32) };

    GdmaStatus { channels, misc_conf }
}

// ─────────────────────────────────────────────────────────────────────────
//  Register layout (tock-registers)
// ─────────────────────────────────────────────────────────────────────────

register_bitfields! {
    u32,

    /// `IN_INT_CH.{raw,st,ena,clr}` — identical bit layout in all four.
    /// RX (input) interrupt bits. Bit offsets verified against the VDC
    /// poll-only driver (`lc/vdc_ui_audio:driver/src/dma/esp32c6_gdma.rs`)
    /// and the ESP32-C6 TRM.
    InInt [
        IN_DONE       OFFSET(0) NUMBITS(1) [],
        IN_SUC_EOF    OFFSET(1) NUMBITS(1) [],
        IN_ERR_EOF    OFFSET(2) NUMBITS(1) [],
        IN_DSCR_ERR   OFFSET(3) NUMBITS(1) [],
        IN_DSCR_EMPTY OFFSET(4) NUMBITS(1) [],
        INFIFO_OVF    OFFSET(5) NUMBITS(1) [],
        INFIFO_UDF    OFFSET(6) NUMBITS(1) [],
    ],

    /// `OUT_INT_CH.{raw,st,ena,clr}` — identical bit layout in all four.
    OutInt [
        OUT_DONE      OFFSET(0) NUMBITS(1) [],
        OUT_EOF       OFFSET(1) NUMBITS(1) [],
        OUT_DSCR_ERR  OFFSET(2) NUMBITS(1) [],
        OUT_TOTAL_EOF OFFSET(3) NUMBITS(1) [],
        OUTFIFO_OVF  OFFSET(4) NUMBITS(1) [],
        OUTFIFO_UDF  OFFSET(5) NUMBITS(1) [],
    ],

    /// `CH[CH].out_conf0` — TX channel configuration 0.
    OutConf0 [
        OUT_RST            OFFSET(0) NUMBITS(1) [],
        OUT_LOOP_TEST      OFFSET(1) NUMBITS(1) [],
        OUT_AUTO_WRBACK    OFFSET(2) NUMBITS(1) [],
        OUT_EOF_MODE       OFFSET(3) NUMBITS(1) [],
        OUTDSCR_BURST_EN   OFFSET(4) NUMBITS(1) [],
        OUT_DATA_BURST_EN  OFFSET(5) NUMBITS(1) [],
        OUT_ETM_EN         OFFSET(6) NUMBITS(1) [],
    ],

    /// `CH[CH].out_link` — TX link-list control.
    OutLink [
        OUTLINK_ADDR    OFFSET(0)  NUMBITS(20) [],
        OUTLINK_STOP    OFFSET(20) NUMBITS(1)  [],
        OUTLINK_START   OFFSET(21) NUMBITS(1)  [],
        OUTLINK_RESTART OFFSET(22) NUMBITS(1) [],
        OUTLINK_PARK    OFFSET(23) NUMBITS(1) [],
    ],

    /// `CH[CH].out_state` — TX link-list current descriptor address (RO).
    OutState [
        OUTLINK_DSCR_ADDR OFFSET(0) NUMBITS(18) [],
    ],

    /// `CH[CH].out_peri_sel` — TX peripheral request-line select.
    OutPeriSel [
        PERI_OUT_SEL OFFSET(0) NUMBITS(6) [],
    ],

    /// `CH[CH].in_conf0` — RX channel configuration 0.
    InConf0 [
        IN_RST            OFFSET(0) NUMBITS(1) [],
        IN_LOOP_TEST      OFFSET(1) NUMBITS(1) [],
        INDSCR_BURST_EN   OFFSET(2) NUMBITS(1) [],
        IN_DATA_BURST_EN  OFFSET(3) NUMBITS(1) [],
        MEM_TRANS_EN      OFFSET(4) NUMBITS(1) [],
        IN_ETM_EN         OFFSET(5) NUMBITS(1) [],
    ],

    /// `CH[CH].in_link` — RX link-list control.
    InLink [
        INLINK_ADDR      OFFSET(0)  NUMBITS(20) [],
        INLINK_AUTO_RET  OFFSET(20) NUMBITS(1)  [],
        INLINK_STOP      OFFSET(21) NUMBITS(1)  [],
        INLINK_START     OFFSET(22) NUMBITS(1)  [],
        INLINK_RESTART   OFFSET(23) NUMBITS(1) [],
        INLINK_PARK      OFFSET(24) NUMBITS(1) [],
    ],

    /// `CH[CH].in_peri_sel` — RX peripheral request-line select.
    InPeriSel [
        PERI_IN_SEL OFFSET(0) NUMBITS(6) [],
    ],

    /// `CH[CH].in_state` — RX link-list current descriptor address (RO).
    InState [
        INLINK_DSCR_ADDR OFFSET(0) NUMBITS(18) [],
    ],
}

/// One `IN_INT_CH` cluster: `raw`(+0x00) `st`(+0x04) `ena`(+0x08)
/// `clr`(+0x0c). Stride `0x10`. Layout mirrors [`OutIntCh`]; the field
/// type carries the `InInt` bit definitions.
register_structs! {
    InIntCh {
        (0x00 => raw: ReadWrite<u32, InInt::Register>),
        (0x04 => st:  ReadWrite<u32, InInt::Register>),
        (0x08 => ena: ReadWrite<u32, InInt::Register>),
        (0x0c => clr: ReadWrite<u32, InInt::Register>),
        (0x10 => @END),
    }
}

/// One `OUT_INT_CH` cluster: `raw`(+0x00) `st`(+0x04) `ena`(+0x08)
/// `clr`(+0x0c). Stride `0x10`.
register_structs! {
    OutIntCh {
        (0x00 => raw: ReadWrite<u32, OutInt::Register>),
        (0x04 => st:  ReadWrite<u32, OutInt::Register>),
        (0x08 => ena: ReadWrite<u32, OutInt::Register>),
        (0x0c => clr: ReadWrite<u32, OutInt::Register>),
        (0x10 => @END),
    }
}

/// One `CH` cluster. Offsets verified against the ESP32-C6 PAC `dma/ch.rs`:
/// `in_conf0`+0x00, `in_link`+0x10, `in_state`+0x14, `in_suc_eof_des_addr`+0x18,
/// `in_peri_sel`+0x30, `out_conf0`+0x60, `out_conf1`+0x64, `out_link`+0x70,
/// `out_state`+0x74, `out_eof_des_addr`+0x78, `out_peri_sel`+0x90.
register_structs! {
    Ch {
        (0x00 => in_conf0:            ReadWrite<u32, InConf0::Register>),
        (0x04 => in_conf1:            ReadWrite<u32>),
        (0x08 => _reserved_in_0),
        (0x10 => in_link:             ReadWrite<u32, InLink::Register>),
        (0x14 => in_state:            ReadWrite<u32, InState::Register>),
        (0x18 => in_suc_eof_des_addr: ReadWrite<u32>),
        (0x1c => _reserved_in_1),
        (0x30 => in_peri_sel:         ReadWrite<u32, InPeriSel::Register>),
        (0x34 => _reserved_out_0),
        (0x60 => out_conf0:           ReadWrite<u32, OutConf0::Register>),
        (0x64 => out_conf1:           ReadWrite<u32>),
        (0x68 => _reserved_out_1),
        (0x70 => out_link:            ReadWrite<u32, OutLink::Register>),
        (0x74 => out_state:           ReadWrite<u32, OutState::Register>),
        (0x78 => out_eof_des_addr:    ReadWrite<u32>),
        (0x7c => _reserved_out_2),
        (0x90 => out_peri_sel:        ReadWrite<u32, OutPeriSel::Register>),
        (0x94 => @END),
    }
}

// ─────────────────────────────────────────────────────────────────────────
//  Static register accessors
// ─────────────────────────────────────────────────────────────────────────

const fn in_int_regs<const CH: usize>() -> StaticRef<InIntCh> {
    const { assert!(CH < 3) };
    unsafe {
        StaticRef::new((DMA_BASE + IN_INT_BASE + CH * INT_STRIDE) as *const InIntCh)
    }
}

const fn out_int_regs<const CH: usize>() -> StaticRef<OutIntCh> {
    const { assert!(CH < 3) };
    unsafe {
        StaticRef::new((DMA_BASE + OUT_INT_BASE + CH * INT_STRIDE) as *const OutIntCh)
    }
}

const fn channel_regs<const CH: usize>() -> StaticRef<Ch> {
    const { assert!(CH < 3) };
    unsafe { StaticRef::new((DMA_BASE + CH_OFFSET + CH * CH_STRIDE) as *const Ch) }
}

// ─────────────────────────────────────────────────────────────────────────
//  Descriptor (SoC-private, 12-byte linked-list node)
// ─────────────────────────────────────────────────────────────────────────

/// ESP32-C6 GDMA linked-list descriptor.
///
/// `dw0` packs `size` / `length` / `suc_eof` / `owner`; the HAL layer never
/// inspects these bits — all bit-field work is hoisted into the `DmaChannel`
/// methods below. The consumer holds descriptor storage (a `no_std` reality:
/// rings live in `static` memory) but must not touch `dw0` directly.
#[repr(C, align(4))]
#[derive(Clone, Copy)]
pub struct DmaDescriptor {
    /// `size[11:0]` / `length[23:12]` / `suc_eof[30]` / `owner[31]`.
    pub dw0: u32,
    /// Buffer pointer (RAM address the DMA reads from / writes to).
    pub buffer: *mut u8,
    /// Next descriptor in the ring/chain (`null` terminates a chain).
    pub next: *mut DmaDescriptor,
}

// `*mut` is not `Default`; provide a safe null default for ergonomic
// non-const contexts. For `static` initialisers use the `const` `empty()`
// below — trait methods cannot be `const` on stable Rust, so
// `Default::default()` is not usable in a `static`.
impl Default for DmaDescriptor {
    fn default() -> Self {
        Self {
            dw0: 0,
            buffer: core::ptr::null_mut(),
            next: core::ptr::null_mut(),
        }
    }
}

impl DmaDescriptor {
    /// All-zero/null placeholder, callable in `const` (e.g. `static`
    /// descriptor arrays: `[DmaDescriptor::empty(); N]`).
    pub const fn empty() -> Self {
        Self {
            dw0: 0,
            buffer: core::ptr::null_mut(),
            next: core::ptr::null_mut(),
        }
    }

    /// Build a TX descriptor: `owner=DMA`, `suc_eof` set when `eof` is true.
    pub const fn for_tx(buf: *mut u8, len: usize, suc_eof: bool) -> Self {
        let size = (len as u32) & DW0_SIZE_MASK;
        let length = (len as u32) << DW0_LENGTH_SHIFT;
        Self {
            dw0: size | length | DW0_OWNER_DMA | if suc_eof { DW0_SUC_EOF } else { 0 },
            buffer: buf,
            next: core::ptr::null_mut(),
        }
    }

    /// Build an RX descriptor: `owner=DMA`, no EOF (RX side reports EOF via
    /// `IN_SUC_EOF` on descriptor ownership hand-back).
    pub const fn for_rx(buf: *mut u8, capacity: usize) -> Self {
        let size = (capacity as u32) & DW0_SIZE_MASK;
        Self {
            dw0: size | DW0_OWNER_DMA,
            buffer: buf,
            next: core::ptr::null_mut(),
        }
    }

    /// Number of bytes the DMA actually moved into this RX descriptor.
    pub fn received_len(&self) -> usize {
        ((self.dw0 >> DW0_LENGTH_SHIFT) & DW0_SIZE_MASK) as usize
    }
}

// ─────────────────────────────────────────────────────────────────────────
//  Per-channel runtime state
// ─────────────────────────────────────────────────────────────────────────

/// Per-channel runtime state (indexed by `CH`, 0..=2). Heapless — lives in
/// the [`static_mut CHAN_STATE`] array.
///
/// # Concurrency
///
/// `service_interrupt` (ISR) and `set_callback` / `prepare_ring` /
/// `prepare_chain` / `start` (thread) may run concurrently. Field-by-field:
/// - `ring_base` / `cyclic_nsegs` / `oneshot_cookie` / `first_desc` /
///   `direction` / `peri` / `prepared`: thread writes once before
///   `enable_interrupt`, ISR reads only after. Ordered by the
///   prepare→enable sequence; additionally wrapped in an irq-save for
///   consistency.
/// - `cyclic_last`: ISR-exclusive (single-core, no nested IRQ). No lock.
/// - `callback`: the only true race (thread write vs ISR read). Guarded
///   by `disable_local_irq_save` / `enable_local_irq_restore`.
struct ChanState {
    callback: Option<DmaCallback>,
    /// oneshot: cookie reported on `OneshotDone` (0 reserved for "not
    /// submitted"); written by `prepare_chain`, read by the ISR.
    oneshot_cookie: usize,
    /// cyclic: index of the last segment already reported, for incremental
    /// dispatch (avoids under-reporting when the DMA crosses several
    /// segments between ISRs). ISR-exclusive.
    cyclic_last: usize,
    /// cyclic: segment count (0 = ring not configured). Thread→ISR.
    cyclic_nsegs: usize,
    /// cyclic: ring base address, for computing the current segment index.
    /// Thread→ISR.
    ring_base: *const DmaDescriptor,
    /// Head descriptor address recorded by `prepare_ring` / `prepare_chain`,
    /// read by the argless `start` to mount the outlink/inlink. Stored as
    /// a raw `usize` rather than `&Desc` so the trait's `start` takes no
    /// arguments (the borrow is released before `start` runs; the
    /// underlying `static` storage keeps the descriptor live).
    first_desc: *const DmaDescriptor,
    /// Direction recorded by `prepare_*`, read by `start` to pick the
    /// RX-vs-TX kickoff path.
    direction: DmaDirection,
    /// Peripheral request-line id recorded by `prepare_*`, read by `start`.
    peri: u8,
    /// Whether `prepare_ring` / `prepare_chain` has armed the channel
    /// since the last `terminate`. `start` returns `Err(NotReady)` when
    /// false. Thread→ISR-irrelevant (ISR never reads it).
    prepared: bool,
}

static mut CHAN_STATE: [ChanState; 3] = [ChanState::NULL, ChanState::NULL, ChanState::NULL];

impl ChanState {
    const NULL: ChanState = ChanState {
        callback: None,
        oneshot_cookie: 0,
        cyclic_last: 0,
        cyclic_nsegs: 0,
        ring_base: core::ptr::null(),
        first_desc: core::ptr::null(),
        direction: DmaDirection::MemToMem,
        peri: 0,
        prepared: false,
    };
}

/// Per-channel `ChanState` accessor.
///
/// # Safety
///
/// `CH < 3` is guaranteed by the `const { assert!(CH < 3) }` in
/// [`Esp32c6GdmaChannel::new`] / the const-register accessors. Mutation of
/// the returned reference must respect the concurrency rules documented on
/// [`ChanState`] (irq-save for `callback`, prepare→enable ordering for the
/// rest, ISR-exclusivity for `cyclic_last`).
fn state(ch: usize) -> &'static mut ChanState {
    debug_assert!(ch < 3);
    // SAFETY: CHAN_STATE is `'static`; `ch < 3` is asserted at construction.
    // Callers honour the field-level concurrency contract documented above.
    unsafe { &mut CHAN_STATE[ch] }
}

// ─────────────────────────────────────────────────────────────────────────
//  Channel type + register-level associated functions
// ─────────────────────────────────────────────────────────────────────────

/// ESP32-C6 GDMA channel. Zero-sized — runtime state is in [`CHAN_STATE`].
///
/// `CH` selects the channel (0..=2). TX and RX of the same channel number
/// share a `CH` cluster but have separate interrupt clusters and link
/// control; this driver currently exposes the TX (out) path + the RX start
/// primitive used by [`DmaChannel::start`] when armed for `PeriphToMem`.
pub struct Esp32c6GdmaChannel<const CH: usize>;

unsafe impl<const CH: usize> Send for Esp32c6GdmaChannel<CH> {}
unsafe impl<const CH: usize> Sync for Esp32c6GdmaChannel<CH> {}

impl<const CH: usize> Esp32c6GdmaChannel<CH> {
    pub const fn new() -> Self {
        const { assert!(CH < 3) };
        Self
    }

    // ── block-level init ──────────────────────────────────────────────

    /// Bring the GDMA block out of reset and enable its clock.
    ///
    /// On ESP32-C6 the PCR peripheral clock gate for GDMA and the
    /// `DMA.CLK_EN` would be programmed here. The prior VDC poll-only driver
    /// relied on the bootloader / system init having already enabled the
    /// GDMA clock; this interrupt-layer driver keeps that assumption (the
    /// board's early init enables system clocks before any driver runs) and
    /// leaves an explicit hook for the day PCR gating is wired in. Nothing
    /// here touches `ENA` / the interrupt path — that is reserved for
    /// [`DmaChannel::enable_interrupt`].
    fn init_dma() {
        // Clock gating is done by the board's system init; no register
        // write needed here yet.
    }

    // ── RX (in) primitives ───────────────────────────────────────────

    #[allow(dead_code)]
    fn reset_rx() {
        let ch = channel_regs::<CH>();
        ch.in_conf0.write(InConf0::IN_RST::SET);
        ch.in_conf0.write(InConf0::IN_RST::CLEAR);
    }

    /// Select the peripheral request line feeding the RX FIFO.
    fn set_rx_peri(peri: u32) {
        let ch = channel_regs::<CH>();
        ch.in_peri_sel.write(InPeriSel::PERI_IN_SEL.val(peri & 0x3f));
    }

    /// Start the RX link from `desc` (with FIFO reset).
    fn start_rx(desc: &DmaDescriptor) {
        let ch = channel_regs::<CH>();
        Self::reset_rx();
        ch.in_link.write(InLink::INLINK_ADDR.val((desc as *const _ as usize) as u32));
        ch.in_link.write(InLink::INLINK_START::SET);
    }

    // ── TX (out) primitives ───────────────────────────────────────────

    fn reset_tx() {
        let ch = channel_regs::<CH>();
        ch.out_conf0.write(OutConf0::OUT_RST::SET);
        ch.out_conf0.write(OutConf0::OUT_RST::CLEAR);
    }

    /// Select the peripheral request line fed by the TX FIFO.
    fn set_tx_peri(peri: u32) {
        let ch = channel_regs::<CH>();
        ch.out_peri_sel.write(OutPeriSel::PERI_OUT_SEL.val(peri & 0x3f));
    }

    /// Start the TX link from `desc` **with** FIFO reset.
    ///
    /// Resets the out-FSM first, which introduces an inter-transfer gap —
    /// unsuitable for glitch-free audio. Use [`Self::start_tx_no_reset`]
    /// for perpetual rings.
    fn start_tx(desc: &DmaDescriptor) {
        let ch = channel_regs::<CH>();
        Self::reset_tx();
        ch.out_link.write(OutLink::OUTLINK_ADDR.val((desc as *const _ as usize) as u32));
        ch.out_link.write(OutLink::OUTLINK_START::SET);
    }

    /// Start the TX link from `desc` **without** FIFO reset — no inter-
    /// segment gap, the mode used for perpetual audio rings.
    fn start_tx_no_reset(desc: &DmaDescriptor) {
        let ch = channel_regs::<CH>();
        ch.out_link.write(OutLink::OUTLINK_ADDR.val((desc as *const _ as usize) as u32));
        ch.out_link.write(OutLink::OUTLINK_START::SET);
    }

    /// Restart the outlink after the CPU has refilled a parked descriptor's
    /// owner bit. Does not reset the FIFO.
    fn restart_tx() {
        let ch = channel_regs::<CH>();
        ch.out_link.write(OutLink::OUTLINK_RESTART::SET);
    }

    /// `OUT_STATE[17:0]` — the descriptor address the DMA is currently on.
    fn read_outlink_dscr_addr() -> u32 {
        channel_regs::<CH>().out_state.read(OutState::OUTLINK_DSCR_ADDR)
    }

    /// Enable `OUT_AUTO_WRBACK` (DMA clears owner after consuming a
    /// descriptor) and `OUT_EOF_MODE` (EOF fires after the peripheral has
    /// drained the data). Both default to 1 on reset, so this is mostly a
    /// self-documenting re-affirmation.
    #[allow(dead_code)]
    fn enable_out_autowrback() {
        let ch = channel_regs::<CH>();
        ch.out_conf0.modify(OutConf0::OUT_AUTO_WRBACK::SET);
        ch.out_conf0.modify(OutConf0::OUT_EOF_MODE::SET);
    }

    // ── Poll primitives (TX-side diagnostics) ──────────────────────────

    /// Returns `true` if the `OUT_TOTAL_EOF` interrupt raw bit is set —
    /// i.e. the DMA engine has consumed the last descriptor of a
    /// terminated chain (`next = null`) and come to a halt.
    ///
    /// This is the user-approved `poll_tx_done()` helper. It inspects the
    /// TX-side total-EOF and is used as a diagnostic when an interrupt
    /// callback fails to fire within the expected window (so the caller can
    /// tell "TX actually completed but the callback was lost" from "TX
    /// never completed"). It is **not** the M2M completion criterion — M2M
    /// completion is observed on the RX side via [`Self::wait_rx_done`].
    pub fn is_tx_total_eof() -> bool {
        out_int_regs::<CH>().raw.get() & OutInt::OUT_TOTAL_EOF.mask != 0
    }

    /// Clear the `OUT_TOTAL_EOF` interrupt raw bit.
    #[allow(dead_code)]
    fn clear_out_total_eof() {
        out_int_regs::<CH>()
            .clr
            .set(OutInt::OUT_TOTAL_EOF.val(1).value);
    }

    /// Check if `OUT_DSCR_ERR` is set — descriptor error (e.g. invalid
    /// address).
    #[allow(dead_code)]
    fn is_out_dscr_err() -> bool {
        out_int_regs::<CH>().raw.get() & OutInt::OUT_DSCR_ERR.mask != 0
    }

    /// Clear the `OUT_DSCR_ERR` interrupt raw bit.
    #[allow(dead_code)]
    fn clear_out_dscr_err() {
        out_int_regs::<CH>()
            .clr
            .set(OutInt::OUT_DSCR_ERR.val(1).value);
    }

    /// Block with a spin-loop until `mask` is set in the RX interrupt raw
    /// register, or [`DMA_POLL_LIMIT`] is exhausted. Returns `true` if the
    /// bit appeared, `false` on timeout. Mirrors the VDC poll-only driver.
    fn wait_for_in_bit(mask: u32) -> bool {
        let int = in_int_regs::<CH>();
        for _ in 0..DMA_POLL_LIMIT {
            if int.raw.get() & mask != 0 {
                return true;
            }
            core::hint::spin_loop();
        }
        false
    }

    /// Block until the RX transfer completes. Returns `Err` on timeout or
    /// descriptor error. The M2M completion criterion: the TX descriptor's
    /// `suc_eof` propagates through the M2M bridge and surfaces as
    /// `IN_SUC_EOF` on the RX side.
    pub(crate) fn wait_rx_done() -> Result<()> {
        use blueos_hal::err::HalError;
        let int = in_int_regs::<CH>();
        let mask = InInt::IN_SUC_EOF.mask
            | InInt::IN_DSCR_ERR.mask
            | InInt::IN_DSCR_EMPTY.mask;
        loop {
            let raw = int.raw.get();
            if raw & InInt::IN_SUC_EOF.mask != 0 {
                return Ok(());
            }
            if raw & InInt::IN_DSCR_ERR.mask != 0 {
                return Err(HalError::Fail);
            }
            if raw & InInt::IN_DSCR_EMPTY.mask != 0 {
                return Err(HalError::Fail);
            }
            if !Self::wait_for_in_bit(mask) {
                return Err(HalError::Timeout);
            }
        }
    }

    // ── M2M (memory-to-memory) self-test ──────────────────────────────

    /// Run a memory-to-memory DMA transfer to verify both outlink (read)
    /// and inlink (write) paths without any peripheral attached.
    ///
    /// `src` is the source buffer (read by outlink), `dst` is the
    /// destination buffer (written by inlink). Both must reside in internal
    /// SRAM. The caller fills `src` with a known pattern and zeroes `dst`
    /// before calling; after `Ok(())` returns, `dst` should match `src`.
    ///
    /// This uses the same channel for both TX and RX: it sets
    /// `MEM_TRANS_EN` (bridging the TX FIFO into the RX FIFO), starts the
    /// outlink first (to feed data into the DMA internal FIFO), then the
    /// inlink (to drain the FIFO into `dst`). Completion is observed on the
    /// RX side via `IN_SUC_EOF` — the TX descriptor's `suc_eof` propagates
    /// through the M2M bridge — **not** via `OUT_TOTAL_EOF`. This is a
    /// port of the VDC poll-only driver's `m2m_transfer`.
    pub fn m2m_transfer_impl(src: &mut [u8], dst: &mut [u8]) -> Result<()> {
        use blueos_hal::err::HalError;
        if src.len() != dst.len() || src.is_empty() {
            return Err(HalError::InvalidParam);
        }
        let len = src.len();

        // Clock gating is done by board system init; re-affirm idempotently.
        Self::init_dma();

        let ch = channel_regs::<CH>();

        // ── Step 1: Reset TX FSM and FIFO pointer ──
        ch.out_conf0.modify(OutConf0::OUT_RST::SET);
        ch.out_conf0.modify(OutConf0::OUT_RST::CLEAR);

        // ── Step 2: Reset RX FSM and FIFO pointer ──
        ch.in_conf0.modify(InConf0::IN_RST::SET);
        ch.in_conf0.modify(InConf0::IN_RST::CLEAR);

        // Select a pseudo-peripheral (SPI2, peri_id = 1) for both
        // directions. In M2M mode the hardware ignores the peripheral FIFO,
        // but esp-hal still sets peri_sel — we match that to be safe.
        Self::set_tx_peri(1);
        Self::set_rx_peri(1);

        // Enable descriptor-burst reads on both sides.
        ch.out_conf0
            .modify(OutConf0::OUTDSCR_BURST_EN::SET + OutConf0::OUT_EOF_MODE::SET);
        ch.in_conf0.modify(InConf0::INDSCR_BURST_EN::SET);

        // Build descriptors: outlink reads from `src`, inlink writes to
        // `dst`. Both have owner = DMA (bit 31 set). The TX descriptor has
        // suc_eof = 1 (propagates to IN_SUC_EOF via the M2M bridge).
        let tx_desc = DmaDescriptor::for_tx(src.as_ptr() as *mut u8, len, true);
        let rx_desc = DmaDescriptor::for_rx(dst.as_mut_ptr(), len);

        // Ensure descriptor writes are visible to the DMA engine.
        core::sync::atomic::compiler_fence(core::sync::atomic::Ordering::SeqCst);

        // Clear pending interrupts on both sides.
        let in_int = in_int_regs::<CH>();
        in_int.clr.set(
            InInt::IN_DONE.val(1).value
                | InInt::IN_SUC_EOF.val(1).value
                | InInt::IN_ERR_EOF.val(1).value
                | InInt::IN_DSCR_ERR.val(1).value
                | InInt::IN_DSCR_EMPTY.val(1).value,
        );
        let out_int = out_int_regs::<CH>();
        out_int.clr.set(
            OutInt::OUT_DONE.val(1).value
                | OutInt::OUT_EOF.val(1).value
                | OutInt::OUT_TOTAL_EOF.val(1).value
                | OutInt::OUT_DSCR_ERR.val(1).value,
        );

        // ── Step 3: Mount TX outlink — set OUTLINK_ADDR ──
        let tx_desc_addr =
            (core::ptr::addr_of!(tx_desc) as usize as u32) & ((1 << 20) - 1);
        ch.out_link
            .modify(OutLink::OUTLINK_ADDR.val(tx_desc_addr));

        // ── Step 4: Mount RX inlink — set INLINK_ADDR ──
        let rx_desc_addr =
            (core::ptr::addr_of!(rx_desc) as usize as u32) & ((1 << 20) - 1);
        ch.in_link.modify(InLink::INLINK_ADDR.val(rx_desc_addr));

        // ── Step 5: Enable memory-to-memory mode (TX FIFO → RX FIFO) ──
        ch.in_conf0.modify(InConf0::MEM_TRANS_EN::SET);

        // ── Step 6: Start TX channel — OUTLINK_START ──
        ch.out_link.modify(OutLink::OUTLINK_START::SET);

        // ── Step 7: Start RX channel — INLINK_START ──
        ch.in_link.modify(InLink::INLINK_START::SET);

        // ── Step 8: Wait for IN_SUC_EOF (TX's suc_eof propagates through
        //            the M2M bridge to trigger IN_SUC_EOF) ──
        let result = Self::wait_rx_done();

        // Clean up: disable M2M mode.
        ch.in_conf0.modify(InConf0::MEM_TRANS_EN::CLEAR);

        result
    }

    /// Interrupt-driven memory-to-memory transfer.
    ///
    /// Identical setup to [`m2m_transfer_impl`] (reset TX/RX FSM, pseudo-peri
    /// sel, burst bits, build TX+RX descriptors, clear pending IN/OUT
    /// interrupts, mount OUTLINK+INLINK_ADDR, `MEM_TRANS_EN` SET), but
    /// completion is observed **asynchronously**: instead of polling
    /// `wait_rx_done()`, this registers `cb` via [`DmaChannel::set_callback`]
    /// (writing the kernel's `CHAN_STATE[CH].callback` under irq-save) and
    /// arms both IN+OUT interrupts via [`DmaChannel::enable_interrupt`],
    /// then starts TX and RX. When the TX descriptor's `suc_eof` propagates
    /// through the M2M bridge to an RX-side `IN_SUC_EOF`, the board's IN0
    /// ISR fires → [`DmaChannel::service_interrupt`] (RX block) → `cb` is
    /// invoked with [`DmaEvent::OneshotDone`].
    ///
    /// Returns `Ok(())` immediately after starting both links — the caller
    /// owns waiting on `cb`'s flag, then [`DmaChannel::disable_interrupt`] +
    /// [`DmaChannel::terminate`] for cleanup. This is the HAL docstring's
    /// "interrupt-driven M2M completion, if ever needed, would be layered on
    /// top of `set_callback`" (`hal/dma.rs` M2M docstring), made concrete.
    ///
    /// Inherent (not a trait method) because it is an ESP32-C6-specific
    /// interrupt primitive, not part of the SoC-agnostic HAL contract; `pub`
    /// so kernel-side consumers sharing the kernel's `CHAN_STATE` can call it.
    ///
    /// # Re-entrancy
    ///
    /// `cb` runs in interrupt context and must not re-enter this channel's
    /// irq-save lock (see [`DmaCallback`] re-entrancy contract).
    pub fn m2m_transfer_irq(
        &self,
        src: &mut [u8],
        dst: &mut [u8],
        cb: DmaCallback,
    ) -> Result<()> {
        use blueos_hal::err::HalError;
        if src.len() != dst.len() || src.is_empty() {
            return Err(HalError::InvalidParam);
        }
        let len = src.len();

        // Clock gating is done by board system init; re-affirm idempotently.
        Self::init_dma();

        let ch = channel_regs::<CH>();

        // ── Step 1: Reset TX FSM and FIFO pointer ──
        ch.out_conf0.modify(OutConf0::OUT_RST::SET);
        ch.out_conf0.modify(OutConf0::OUT_RST::CLEAR);

        // ── Step 2: Reset RX FSM and FIFO pointer ──
        ch.in_conf0.modify(InConf0::IN_RST::SET);
        ch.in_conf0.modify(InConf0::IN_RST::CLEAR);

        // Select a pseudo-peripheral (SPI2, peri_id = 1) for both
        // directions. In M2M mode the hardware ignores the peripheral FIFO,
        // but esp-hal still sets peri_sel — we match that to be safe.
        Self::set_tx_peri(1);
        Self::set_rx_peri(1);

        // Enable descriptor-burst reads on both sides.
        ch.out_conf0
            .modify(OutConf0::OUTDSCR_BURST_EN::SET + OutConf0::OUT_EOF_MODE::SET);
        ch.in_conf0.modify(InConf0::INDSCR_BURST_EN::SET);

        // Build descriptors in STATIC storage so they outlive this function's
        // return. Unlike the poll path (`m2m_transfer_impl`), which waits for
        // `IN_SUC_EOF` inline before returning, this function returns as soon
        // as both links are started — completion arrives asynchronously via
        // the IN0 ISR. The DMA engine keeps referencing the descriptors after
        // we return: `OUT_AUTO_WRBACK` writes the completed `dw0` back to the
        // TX descriptor and the engine walks `next`. Stack-local descriptors
        // would be reclaimed on return and that writeback would clobber
        // whatever the caller's frame now holds at the same address —
        // observed in practice as a Load Access Fault (mcause 0x5) in
        // `run_irq_test`'s verification loop, where the `rx` slice pointer
        // got overwritten with the writeback value `0x40100100` (= owner=0 |
        // suc_eof<<30 | length=0x100<<12 | size=0x100). Each
        // monomorphization of `CH` gets its own pair; the M2M test runs
        // serially, so there is no aliasing.
        // SAFETY: single in-flight use of channel CH; the caller waits for
        // completion and terminates before re-entering this channel.
        static mut TX_DESC: DmaDescriptor = DmaDescriptor::empty();
        static mut RX_DESC: DmaDescriptor = DmaDescriptor::empty();
        let (tx_desc, rx_desc) = unsafe {
            TX_DESC = DmaDescriptor::for_tx(src.as_ptr() as *mut u8, len, true);
            RX_DESC = DmaDescriptor::for_rx(dst.as_mut_ptr(), len);
            (core::ptr::addr_of_mut!(TX_DESC), core::ptr::addr_of_mut!(RX_DESC))
        };

        // Ensure descriptor writes are visible to the DMA engine.
        core::sync::atomic::compiler_fence(core::sync::atomic::Ordering::SeqCst);

        // Clear pending interrupts on both sides.
        let in_int = in_int_regs::<CH>();
        in_int.clr.set(
            InInt::IN_DONE.val(1).value
                | InInt::IN_SUC_EOF.val(1).value
                | InInt::IN_ERR_EOF.val(1).value
                | InInt::IN_DSCR_ERR.val(1).value
                | InInt::IN_DSCR_EMPTY.val(1).value,
        );
        let out_int = out_int_regs::<CH>();
        out_int.clr.set(
            OutInt::OUT_DONE.val(1).value
                | OutInt::OUT_EOF.val(1).value
                | OutInt::OUT_TOTAL_EOF.val(1).value
                | OutInt::OUT_DSCR_ERR.val(1).value,
        );

        // ── Step 3: Mount TX outlink — set OUTLINK_ADDR ──
        let tx_desc_addr = (tx_desc as usize as u32) & ((1 << 20) - 1);
        ch.out_link
            .modify(OutLink::OUTLINK_ADDR.val(tx_desc_addr));

        // ── Step 4: Mount RX inlink — set INLINK_ADDR ──
        let rx_desc_addr = (rx_desc as usize as u32) & ((1 << 20) - 1);
        ch.in_link.modify(InLink::INLINK_ADDR.val(rx_desc_addr));

        // ── Step 5: Enable memory-to-memory mode (TX FIFO → RX FIFO) ──
        ch.in_conf0.modify(InConf0::MEM_TRANS_EN::SET);

        // ── Step 6: Register the completion callback (writes the kernel's
        //             CHAN_STATE[CH].callback under irq-save — the one the
        //             ISR reads) ──
        self.set_callback(cb);

        // ── Step 7: Arm IN+OUT interrupts (IN_SUC_EOF + IN_DSCR_ERR +
        //            IN_DSCR_EMPTY on the RX side, OUT_TOTAL_EOF +
        //            OUT_EOF + OUT_DSCR_ERR on the TX side) ──
        self.enable_interrupt();

        // Order the setup writes before kickoff.
        core::sync::atomic::compiler_fence(core::sync::atomic::Ordering::SeqCst);

        // ── Step 8: Start TX channel — OUTLINK_START ──
        ch.out_link.modify(OutLink::OUTLINK_START::SET);

        // ── Step 9: Start RX channel — INLINK_START ──
        ch.in_link.modify(InLink::INLINK_START::SET);

        // Completion is delivered asynchronously via the IN0 ISR →
        // `service_interrupt` (RX IN_SUC_EOF block) → callback. The caller
        // waits on the callback's flag, then disables the interrupt and
        // terminates.
        Ok(())
    }

    // ── Topology helpers (used by prepare_ring / prepare_chain) ─────

    /// Link `descs` into a perpetual closed ring:
    /// `descs[i].next = &descs[i+1]`, `descs[n-1].next = &descs[0]`.
    ///
    /// ESP32-C6 uses a 12-byte linked-list descriptor whose `next` field
    /// encodes the topology. GD32 has no such descriptor (the ring is the
    /// M0/M1 ping-pong hardware), so a GD32 `prepare_ring` would not call
    /// this. Kept as a separate inherent helper so the trait impl stays
    /// readable and a future GD32 driver can omit it.
    pub fn link_cyclic(descs: &mut [DmaDescriptor]) {
        let n = descs.len();
        for i in 0..n {
            descs[i].next = &mut descs[(i + 1) % n] as *mut _;
        }
        // tail → head closes the ring (perpetual motion); matches the I2S
        // driver's existing `desc[RING_SIZE-1].next = &desc[0]`.
    }

    /// Link `descs` into an open chain: `descs[i].next = &descs[i+1]`,
    /// tail `next` = null. For oneshot transfers.
    pub fn link_chain(descs: &mut [DmaDescriptor]) {
        let n = descs.len();
        for i in 0..n.saturating_sub(1) {
            descs[i].next = &mut descs[i + 1] as *mut _;
        }
        descs[n.saturating_sub(1)].next = core::ptr::null_mut();
    }
}

// ─────────────────────────────────────────────────────────────────────────
//  HAL trait impls
// ─────────────────────────────────────────────────────────────────────────

impl<const CH: usize> PlatPeri for Esp32c6GdmaChannel<CH> {
    fn enable(&self) {
        // ESP32-C6: PCR peripheral clock gating + init_dma's CLK_EN.
        // Board system init enables the GDMA clock; init_dma is the hook
        // for the day PCR gating is wired into the driver.
        Self::init_dma();
    }
    fn disable(&self) {}
}

impl<const CH: usize> Configuration<DmaSlaveConfig> for Esp32c6GdmaChannel<CH> {
    type Target = ();
    fn configure(&self, cfg: &DmaSlaveConfig) -> Result<()> {
        match cfg.direction {
            DmaDirection::MemToMem => {
                // M2M bridges the TX FIFO back into the RX FIFO so the
                // controller moves data between two RAM buffers with no
                // peripheral involved. Without this bit the TX and RX sides
                // run independently and no data crosses between them.
                channel_regs::<CH>()
                    .in_conf0
                    .modify(InConf0::MEM_TRANS_EN::SET);
                Self::set_tx_peri(cfg.periph.0 as u32);
            }
            DmaDirection::MemToPeriph => {
                Self::set_tx_peri(cfg.periph.0 as u32);
            }
            DmaDirection::PeriphToMem => {
                Self::set_rx_peri(cfg.periph.0 as u32);
            }
        }
        Ok(())
    }
}

impl<const CH: usize> DmaChannel for Esp32c6GdmaChannel<CH> {
    type Desc = DmaDescriptor;

    fn capabilities(&self) -> DmaCaps {
        // `DmaCaps` is a dependency-free newtype (no BitOr impl) so the
        // union is spelled out via the const `.union()` helper.
        DmaCaps::MEMCPY
            .union(DmaCaps::SLAVE)
            .union(DmaCaps::CYCLIC)
            .union(DmaCaps::SG)
    }

    fn prep_desc(
        &self,
        buf: &mut [u8],
        cfg: &DmaSlaveConfig,
        eof: bool,
    ) -> Result<DmaDescriptor> {
        Ok(match cfg.direction {
            DmaDirection::MemToPeriph | DmaDirection::MemToMem => {
                DmaDescriptor::for_tx(buf.as_mut_ptr(), buf.len(), eof)
            }
            DmaDirection::PeriphToMem => {
                DmaDescriptor::for_rx(buf.as_mut_ptr(), buf.len())
            }
        })
    }

    fn is_consumed(&self, desc: &DmaDescriptor) -> bool {
        // ESP: owner bit dw0[31], DMA clears it after consumption when
        // OUT_AUTO_WRBACK=1.
        desc.dw0 & DW0_OWNER_DMA == 0
    }

    fn refill(&self, desc: &mut DmaDescriptor, buf: &mut [u8], eof: bool) {
        desc.buffer = buf.as_mut_ptr();
        let size = (buf.len() as u32) & DW0_SIZE_MASK;
        desc.dw0 = size
            | ((buf.len() as u32) << DW0_LENGTH_SHIFT)
            | if eof { DW0_SUC_EOF } else { 0 }
            | DW0_OWNER_DMA;
        // `next` is left untouched (ring linkage stays intact).
    }

    fn current_segment(
        &self,
        ring_base: *const DmaDescriptor,
        nsegs: usize,
    ) -> usize {
        let dma_addr = Self::read_outlink_dscr_addr() as usize;
        let base = ring_base as usize;
        let idx = dma_addr.saturating_sub(base) / core::mem::size_of::<DmaDescriptor>();
        idx % nsegs
    }

    fn prepare_ring(
        &self,
        descs: &mut [DmaDescriptor],
        cfg: &DmaSlaveConfig,
    ) -> Result<()> {
        // Stage 2a: build the SoC ring topology — ESP32-C6 closes the
        // next-pointer ring (`descs[n-1].next = &descs[0]`). On GD32 the
        // ring is the M0/M1 ping-pong hardware and this is a no-op.
        Self::link_cyclic(descs);

        // Stage 2b: record cyclic ISR-dispatch mode + the head/direction/
        // peripheral that the argless `start` will consume. Under irq-save:
        // the prepare→enable ordering alone would suffice, but the
        // irq-save is zero-cost here and matches set_callback's discipline.
        let nsegs = descs.len();
        let first = &descs[0] as *const DmaDescriptor;
        let g = disable_local_irq_save();
        let st = state(CH);
        // SAFETY: irq-save critical section; no concurrent ISR.
        unsafe {
            (*st).ring_base = first;
            (*st).cyclic_nsegs = nsegs;
            (*st).cyclic_last = 0;
            (*st).oneshot_cookie = 0;
            (*st).first_desc = first;
            (*st).direction = cfg.direction;
            (*st).peri = cfg.periph.0;
            (*st).prepared = true;
        }
        enable_local_irq_restore(g);
        Ok(())
    }

    fn prepare_chain(
        &self,
        descs: &mut [DmaDescriptor],
        cfg: &DmaSlaveConfig,
    ) -> Result<()> {
        // Stage 2a: build the SoC chain topology — ESP32-C6 opens the
        // next-pointer chain (tail `next` = null). On GD32 the single
        // configuration snapshot is its own chain and this is a no-op.
        Self::link_chain(descs);

        let first = &descs[0] as *const DmaDescriptor;
        let g = disable_local_irq_save();
        let st = state(CH);
        // SAFETY: irq-save critical section; no concurrent ISR.
        unsafe {
            (*st).cyclic_nsegs = 0;
            (*st).cyclic_last = 0;
            (*st).oneshot_cookie = 0;
            (*st).first_desc = first;
            (*st).direction = cfg.direction;
            (*st).peri = cfg.periph.0;
            (*st).prepared = true;
        }
        enable_local_irq_restore(g);
        Ok(())
    }

    fn start(&self) -> Result<()> {
        // Stage 3: pure kickoff — the head, direction, peripheral, and
        // dispatch mode were all recorded by `prepare_*`. Reads them, picks
        // the FIFO-reset policy per mode (ring = no reset, glitch-free
        // audio; chain = reset), and mounts the outlink/inlink.
        let st = state(CH);
        // SAFETY: reads are ordered before enable_interrupt by the
        // prepare→enable sequence; `start` itself runs before enable.
        let (prepared, first_desc, direction, peri, nsegs) = unsafe {
            (
                (*st).prepared,
                (*st).first_desc,
                (*st).direction,
                (*st).peri,
                (*st).cyclic_nsegs,
            )
        };
        if !prepared || first_desc.is_null() {
            return Err(blueos_hal::err::HalError::NotReady);
        }
        // SAFETY: `first_desc` was taken from a `&mut [DmaDescriptor]` the
        // caller owns (a `static` array in `no_std`); the caller keeps it
        // live for the transfer lifetime. The pointer is valid to
        // dereference here for the register address write.
        let first = unsafe { &*first_desc };

        if nsegs > 0 {
            // Cyclic ring: no-reset start (eliminates the inter-segment
            // gap required for glitch-free audio).
            Self::set_tx_peri(peri as u32);
            Self::start_tx_no_reset(first);
        } else {
            // Oneshot chain: reset-start (clears stale FIFO state). RX
            // direction uses the inlink path; TX uses the outlink path.
            match direction {
                DmaDirection::PeriphToMem => Self::start_rx(first),
                _ => {
                    Self::set_tx_peri(peri as u32);
                    Self::start_tx(first);
                }
            }
        }
        Ok(())
    }

    fn restart(&self) {
        Self::restart_tx();
    }

    fn terminate(&self) -> Result<()> {
        // Break the ring + reset the FSM (mirrors the I2S drain_and_stop).
        Self::reset_tx();
        Self::reset_rx();
        let g = disable_local_irq_save();
        let st = state(CH);
        // SAFETY: irq-save critical section.
        unsafe {
            (*st).cyclic_nsegs = 0;
            (*st).cyclic_last = 0;
            (*st).oneshot_cookie = 0;
            (*st).first_desc = core::ptr::null();
            (*st).prepared = false;
            (*st).direction = DmaDirection::MemToMem;
            (*st).peri = 0;
        }
        enable_local_irq_restore(g);
        Ok(())
    }

    fn set_callback(&self, cb: DmaCallback) {
        // The single true race: thread writes `callback` while the ISR may
        // read it. `Option<fn>` is not `Copy` so it cannot be atomic — the
        // irq-save guarantees the write is not torn.
        let g = disable_local_irq_save();
        let st = state(CH);
        // SAFETY: irq-save critical section; no concurrent ISR.
        unsafe {
            (*st).callback = Some(cb);
        }
        enable_local_irq_restore(g);
    }

    fn enable_interrupt(&self) {
        // Write OUT_INT_CH[CH].ENA: OUT_TOTAL_EOF | OUT_EOF | OUT_DSCR_ERR.
        // `ena` is a tock `ReadWrite` (same register type as `clr`), so the
        // `.set()` write primitive used by the existing `clr` path applies
        // here too.
        let out_int = out_int_regs::<CH>();
        let out_ena = OutInt::OUT_TOTAL_EOF.val(1).value
            | OutInt::OUT_EOF.val(1).value
            | OutInt::OUT_DSCR_ERR.val(1).value;
        out_int.ena.set(out_ena);
        // Also enable RX-side completion + error bits so the IN0 ISR fires
        // on IN_SUC_EOF (the true M2M completion the poll `wait_rx_done`
        // waits on). IN_DSCR_ERR/IN_DSCR_EMPTY mirror the OUT_DSCR_ERR
        // error path on the receive side.
        let in_int = in_int_regs::<CH>();
        let in_ena = InInt::IN_SUC_EOF.val(1).value
            | InInt::IN_DSCR_ERR.val(1).value
            | InInt::IN_DSCR_EMPTY.val(1).value;
        in_int.ena.set(in_ena);
    }

    fn disable_interrupt(&self) {
        out_int_regs::<CH>().ena.set(0);
        in_int_regs::<CH>().ena.set(0);
    }

    fn service_interrupt(&self) {
        let int = out_int_regs::<CH>();
        let raw = int.raw.get();

        // ── oneshot: OUT_TOTAL_EOF (whole chain consumed) ──────────────
        if raw & OutInt::OUT_TOTAL_EOF.mask != 0 {
            int.clr.set(OutInt::OUT_TOTAL_EOF.val(1).value);
            let st = state(CH);
            // SAFETY: ISR-exclusive on single-core, no nested IRQ. The
            // `oneshot_cookie` was written by `prepare_chain` before
            // enable_interrupt (release/acquire ordering).
            let (cookie, cb) = unsafe { ((*st).oneshot_cookie, (*st).callback) };
            if let Some(cb) = cb {
                cb(DmaEvent::OneshotDone(cookie));
            }
        }

        // ── cyclic: OUT_EOF (one segment consumed) ─────────────────────
        if raw & OutInt::OUT_EOF.mask != 0 {
            int.clr.set(OutInt::OUT_EOF.val(1).value);
            let st = state(CH);
            // SAFETY: ISR-exclusive; cyclic_last is only touched here.
            let (nsegs, last, base, cb) = unsafe {
                ((*st).cyclic_nsegs, (*st).cyclic_last, (*st).ring_base, (*st).callback)
            };
            if nsegs > 0 {
                let cur = self.current_segment(base, nsegs);
                // Incremental dispatch: report every segment between the
                // last reported one and the one the DMA is now on. This
                // avoids under-reporting when the DMA crosses several
                // segments before the ISR runs.
                let mut last = last;
                while last != cur {
                    last = (last + 1) % nsegs;
                    if let Some(cb) = cb {
                        cb(DmaEvent::SegmentConsumed(last));
                    }
                }
                // SAFETY: ISR-exclusive.
                unsafe {
                    (*st).cyclic_last = last;
                }
            }
        }

        // ── error: OUT_DSCR_ERR (bad descriptor / list error) ──────────
        if raw & OutInt::OUT_DSCR_ERR.mask != 0 {
            int.clr.set(OutInt::OUT_DSCR_ERR.val(1).value);
            // The driver crate has no `log` dependency, so the DSCR_ERR
            // is cleared-but-quiet for now. Wiring a diagnostic sink
            // (kearly_println / a board-provided logger) is deferred —
            // the interrupt is already cleared so the channel stays usable.
        }

        // ── RX-side dispatch (IN_INT_CH[CH].raw) ───────────────────────
        // Mirrors the OUT structure above. The load-bearing event for the
        // M2M interrupt test is IN_SUC_EOF: the TX descriptor's `suc_eof`
        // propagates through the M2M bridge to an RX-side IN_SUC_EOF (the
        // same condition the poll `wait_rx_done` waits on). Dispatch it as
        // OneshotDone so the registered callback fires once.
        let in_int = in_int_regs::<CH>();
        let in_raw = in_int.raw.get();

        // ── oneshot: IN_SUC_EOF (M2M / RX-chain completion) ────────────
        if in_raw & InInt::IN_SUC_EOF.mask != 0 {
            in_int.clr.set(InInt::IN_SUC_EOF.val(1).value);
            let st = state(CH);
            // SAFETY: ISR-exclusive on single-core, no nested IRQ.
            let (cookie, cb) = unsafe { ((*st).oneshot_cookie, (*st).callback) };
            if let Some(cb) = cb {
                cb(DmaEvent::OneshotDone(cookie));
            }
        }

        // ── error: IN_DSCR_ERR / IN_DSCR_EMPTY (bad descriptor / list
        //    drained before completion) — clear-silent (mirror OUT_DSCR_ERR)
        if in_raw & (InInt::IN_DSCR_ERR.mask | InInt::IN_DSCR_EMPTY.mask) != 0 {
            in_int.clr.set(
                InInt::IN_DSCR_ERR.val(1).value | InInt::IN_DSCR_EMPTY.val(1).value,
            );
        }
    }

    /// Memory-to-memory copy, overriding the [`DmaChannel`] default
    /// (which returns `NotSupport` for slave-only controllers).
    ///
    /// Delegates to the poll-only M2M primitive. The body (outlink + inlink,
    /// `MEM_TRANS_EN`, `IN_SUC_EOF` completion) lives as the inherent
    /// [`Esp32c6GdmaChannel::m2m_transfer_impl`] above; exposing it through
    /// the trait lets a generic client call
    /// [`DmaChannel::m2m_transfer`](blueos_hal::dma::DmaChannel::m2m_transfer)
    /// without naming the concrete `Esp32c6GdmaChannel` type — the same
    /// decoupling Linux's `device_prep_dma_memcpy` gives `dmatest`.
    fn m2m_transfer(src: &mut [u8], dst: &mut [u8]) -> Result<()> {
        Esp32c6GdmaChannel::<CH>::m2m_transfer_impl(src, dst)
    }
}

// ─────────────────────────────────────────────────────────────────────────
//  ISR adapter (board-facing)
// ─────────────────────────────────────────────────────────────────────────

/// Interrupt-vector adapter for a [`DmaChannel`].
///
/// The board instantiates this as a **plain `static`** (no `#[interrupt]`
/// macro) and dispatches it from `handle_intc_irq` — the same pattern as
/// [`blueos_driver::uart::esp32_usb_serial::Esp32UsbSerialIsr`]. On RISC-V
/// there is no `.isr.reg.*` section scanning (that is ARM-only), so a macro-
/// generated ISR vector would never fire; a direct call from the board's
/// trap dispatcher is the only path.
///
/// `service_isr` delegates to [`DmaChannel::service_interrupt`], which reads
/// raw status, clears flags, and invokes the registered callback.
pub struct DmaChanIsr<C: DmaChannel + Sync> {
    /// The channel this ISR services. A `&'static` reference because the
    /// channel is a ZST singleton registered via `define_peripheral!`.
    pub chan: &'static C,
}

// SAFETY: `DmaChanIsr` holds only a `&'static C` with `C: Sync`, so the
// adapter itself is `Sync` (required by `IsrDesc: Sync`).
unsafe impl<C: DmaChannel + Sync> Sync for DmaChanIsr<C> {}

impl<C: DmaChannel + Sync> IsrDesc for DmaChanIsr<C> {
    fn service_isr(&self) {
        self.chan.service_interrupt();
    }
}

// ─────────────────────────────────────────────────────────────────────────
//  Tier 0 unit tests live in the *kernel* crate (`kernel/kernel/src/lib.rs`,
//  `mod tests::gdma_unit`), not here. BlueOS's GN build applies `--cfg test`
//  only to the `kernel_unittest` bin crate, so a `#[cfg(test)]` module in
//  this dependency rlib is dead code — it compiles but is never linked.
//  The bit-field constants above (`DW0_SIZE_MASK` / `DW0_LENGTH_SHIFT` /
//  `DW0_SUC_EOF` / `DW0_OWNER_DMA`) are `pub` precisely so the kernel-crate
//  tests can assert on `dw0` bits.
