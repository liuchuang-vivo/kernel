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

//! DMA subsystem hardware abstraction layer.
//!
//! Models the Linux dmaengine provider-consumer split:
//! - [`DmaChannel`] trait plays the role of `struct dma_device` (method-set
//!   contract that a controller driver implements).
//! - The associated [`DmaChannel::Desc`] type plays the role of
//!   `dma_async_tx_descriptor`: the concrete descriptor layout is SoC-private
//!   and stays inside the driver crate, invisible to this layer.
//! - The `prep_desc` / `prepare_*` / `start` methods mirror the
//!   `prep_*` / `tx_submit` / `issue_pending` three-stage async flow.
//! - [`DmaCallback`] plays the role of the `dmaengine` completion callback.
//!
//! # Design constraints
//!
//! The HAL crate is `#![no_std]` without `alloc`. All runtime state therefore
//! lives in `static` storage inside the driver crate (per-channel state
//! arrays indexed by the channel number), protected by disabling interrupts
//! — never on the heap.
//!
//! # Descriptor ownership split
//!
//! `no_std` without `alloc` forces descriptor storage to live with the
//! consumer (e.g. an I2S ring is a `static` array). The `Desc` type is
//! therefore exposed to the consumer as an associated type, but **the
//! consumer must not touch its bit-fields** — all bit-field manipulation
//! (owner/eof/size, register address reads) is hoisted into the trait
//! methods below. That keeps the unsafe, SoC-specific detail in one place
//! (the driver `impl`), so swapping SoCs only swaps the `impl DmaChannel`.

use crate::{
    err::Result,
    Configuration, PlatPeri,
};

/// Transfer direction.
///
/// Mirrors the Linux `DMA_MEM_TO_MEM` / `DMA_MEM_TO_DEV` /
/// `DMA_DEV_TO_MEM` enumeration.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum DmaDirection {
    /// Memory to memory (the controller moves data between two RAM
    /// buffers, no peripheral involved).
    MemToMem,
    /// Memory to peripheral (TX path: RAM → peripheral FIFO).
    MemToPeriph,
    /// Peripheral to memory (RX path: peripheral FIFO → RAM).
    PeriphToMem,
}

/// Transfer data width.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum DmaWidth {
    Bits8,
    Bits16,
    Bits32,
}

/// Peripheral request-line selector.
///
/// The encoding is vendor-specific: on ESP32-C6 this is the `PERI_SEL`
/// 6-bit field (SPI2=0, UHCI0=2, I2S0=3, AES=6, SHA=7, ADC_DAC=8,
/// Parallel_IO=9); on GD32VW55x it is `SUBPERI0..7`. Drivers translate
/// this opaque id into the hardware encoding internally.
#[derive(Debug, Clone, Copy)]
pub struct DmaPeriphId(pub u8);

/// Channel capability flags.
///
/// Mirrors the Linux `dma_cap_mask_t` / `DMA_CAP_*` set. Implemented as a
/// newtype over `u32` with named constants rather than a `bitflags!` macro
/// so the HAL crate stays dependency-free; driver and consumer code can
/// wrap this in `bitflags!` if convenient.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct DmaCaps(pub u32);

impl DmaCaps {
    /// Empty capability set.
    pub const EMPTY: Self = Self(0);
    /// Channel supports memory-to-memory transfers (`DMA_MEMCPY`).
    pub const MEMCPY: Self = Self(1 << 0);
    /// Channel supports peripheral slave TX/RX (`DMA_SLAVE`).
    pub const SLAVE: Self = Self(1 << 1);
    /// Channel supports a perpetual ring with per-segment reclamation
    /// (`DMA_CYCLIC`, e.g. continuous audio).
    pub const CYCLIC: Self = Self(1 << 2);
    /// Channel supports scatter-gather chains (`DMA_SG`).
    pub const SG: Self = Self(1 << 3);

    /// Whether `self` contains all bits of `other`.
    pub const fn contains(self, other: Self) -> bool {
        (self.0 & other.0) == other.0
    }

    /// Set union: `self | other`.
    pub const fn union(self, other: Self) -> Self {
        Self(self.0 | other.0)
    }
}

/// Completion event delivered to a [`DmaCallback`].
///
/// DMA hardware reports completion through two distinct mechanisms, so the
/// trait models two event kinds rather than one:
///
/// - [`DmaEvent::OneshotDone`] fires once when an entire chain has been
///   consumed (ESP32-C6 `IN_SUC_EOF` / `OUT_TOTAL_EOF`).
/// - [`DmaEvent::SegmentConsumed`] fires for each segment of a perpetual
///   ring that the DMA has consumed, so the CPU can refill it
///   (ESP32-C6 `OUT_EOF` with `OUT_AUTO_WRBACK=1`).
#[derive(Debug, Clone, Copy)]
pub enum DmaEvent {
    /// An oneshot chain has fully completed. Carries the cookie returned
    /// by the submit path (0 is reserved for "not submitted").
    OneshotDone(usize),
    /// A ring segment has been consumed and may be refilled. Carries the
    /// index of the consumed segment relative to the ring base.
    SegmentConsumed(usize),
}

/// Completion callback.
///
/// Invoked in interrupt context — implementations must not sleep, block,
/// or take long. The callback receives a [`DmaEvent`] describing what
/// completed.
///
/// # Re-entrancy contract
///
/// Drivers invoke the callback **directly** from the channel's interrupt
/// service routine (single-core, no nested IRQ). Because the ISR may have
/// disabled local interrupts around the dispatch, a callback **must not**
/// re-enter the channel's own [`DmaChannel::set_callback`] /
/// [`DmaChannel::prepare_ring`] / [`DmaChannel::prepare_chain`] /
/// [`DmaChannel::start`] / other methods that take the channel's irq-save
/// lock — doing so would self-deadlock in ISR context. Consumer bookkeeping
/// (buffer-queue rotation, refilling already-consumed descriptors via
/// [`DmaChannel::refill`] which is lock-free) is the intended use.
pub type DmaCallback = fn(DmaEvent);

/// Slave (peripheral) configuration.
///
/// Mirrors the Linux `struct dma_slave_config`. Consumed by
/// [`DmaChannel::configure`] (via the inherited [`Configuration`] trait)
/// and by the `start_*` methods to select direction / peripheral / width
/// for a transfer.
#[derive(Debug, Clone, Copy)]
pub struct DmaSlaveConfig {
    /// Transfer direction.
    pub direction: DmaDirection,
    /// Transfer data width.
    pub width: DmaWidth,
    /// Peripheral request-line selector.
    pub periph: DmaPeriphId,
    /// Peripheral FIFO address (ignored for [`DmaDirection::MemToMem`]).
    pub periph_addr: usize,
}

impl Default for DmaSlaveConfig {
    fn default() -> Self {
        Self {
            direction: DmaDirection::MemToMem,
            width: DmaWidth::Bits8,
            periph: DmaPeriphId(0),
            periph_addr: 0,
        }
    }
}

/// DMA channel contract.
///
/// A controller driver implements this trait for each channel type it
/// exposes. The trait inherits [`PlatPeri`] (for clock/reset gating) and
/// [`Configuration`]`<`[`DmaSlaveConfig`]`>` (for direction/peripheral
/// selection), mirroring how UART/SPI/I2C compose their capability traits.
///
/// It deliberately does **not** inherit [`crate::HasInterruptReg`]: DMA
/// interrupt types are SoC-specific bit-fields (ESP32-C6 `OutInt`/`InInt`,
/// GD32 `INTF`) and must not leak into the HAL. Instead the channel owns
/// interrupt enable/disable/service as whole-channel operations.
///
/// # The `Desc` associated type
///
/// The concrete descriptor layout is defined by the driver (ESP32-C6's
/// 12-byte linked `DmaDescriptor`, GD32's configuration snapshot), so it
/// is an associated type here. The `Copy + Default` bounds let a consumer
/// initialise a `static` descriptor array — `Default` yields an empty
/// descriptor (`next` = null, `owner` = 0) for placeholder slots.
///
/// The consumer holds descriptor storage (a `no_std` reality: rings must
/// live in `static` memory, and segment count is a consumer policy) but
/// must not touch descriptor bit-fields directly. All bit-field work
/// (owner/eof/size construction, owner-bit completion detection, register
/// address reads) is hoisted into the methods below so the `unsafe`,
/// SoC-specific detail concentrates in the driver `impl`.
pub trait DmaChannel:
    PlatPeri + Configuration<DmaSlaveConfig>
{
    /// SoC-private descriptor type. Layout and bit-field encoding are
    /// defined by the driver crate; this layer never inspects them.
    type Desc: Copy + Default;

    /// Return this channel's capability set (see [`DmaCaps`]).
    ///
    /// Consumers consult this to pick a channel matching their need
    /// (e.g. an audio sink requires [`DmaCaps::CYCLIC`]).
    fn capabilities(&self) -> DmaCaps;

    // ── Descriptor construction (bit-field work lives here) ────────

    /// Package `buf` into a single descriptor. When `eof` is true the
    /// descriptor is marked as the chain tail (triggers a total-EOF on
    /// ESP32-C6).
    ///
    /// Stage 1 of the three-stage flow: `prep_desc` (1:1 per-segment
    /// node) → [`DmaChannel::prepare_ring`] / [`DmaChannel::prepare_chain`]
    /// (build topology + record ISR-dispatch mode) → [`DmaChannel::start`]
    /// (kickoff only). Splitting node construction from topology lets the
    /// caller choose per-segment `eof` policy while the driver owns
    /// linkage, which keeps the SoC-specific link mechanism (ESP32-C6
    /// `next`-pointer ring, GD32 M0/M1 ping-pong where the ring is
    /// implicit in hardware) sealed inside the driver `impl`.
    ///
    /// Mirrors ESP32-C6 `DmaDescriptor::for_tx` / `for_rx`.
    fn prep_desc(
        &self,
        buf: &mut [u8],
        cfg: &DmaSlaveConfig,
        eof: bool,
    ) -> Result<Self::Desc>;

    // ── Topology + ISR-dispatch-mode setup (stage 2) ──────────────

    /// Build `descs` into a perpetual closed ring and arm the channel
    /// for cyclic ISR dispatch.
    ///
    /// Stage 2 of the three-stage flow. The driver:
    /// - links the descriptors into the SoC's ring representation. On
    ///   ESP32-C6 that is `descs[i].next = &descs[i+1]`,
    ///   `descs[n-1].next = &descs[0]`. On GD32 there is no linked-list
    ///   descriptor — the ring is the M0/M1 ping-pong hardware, so this
    ///   step is a no-op there;
    /// - records the ring head, segment count, and cyclic dispatch mode
    ///   into the channel's runtime state, so [`DmaChannel::service_interrupt`]
    ///   can fire [`DmaEvent::SegmentConsumed`] per consumed segment
    ///   without re-reading them;
    /// - records `cfg` (direction / peripheral / width) so the subsequent
    ///   argless [`DmaChannel::start`] can kick off without re-taking it.
    ///
    /// After this call the channel holds enough state for `start` to run.
    /// The caller still owns `descs` storage (a `no_std` reality: rings
    /// live in `static` memory) and must keep it live for the lifetime of
    /// the transfer.
    ///
    /// Mirrors the I2S driver's existing ring construction, folded
    /// together with the ISR-mode bookkeeping the old `start_ring`
    /// performed.
    fn prepare_ring(
        &self,
        descs: &mut [Self::Desc],
        cfg: &DmaSlaveConfig,
    ) -> Result<()>;

    /// Build `descs` into an open chain and arm the channel for oneshot
    /// ISR dispatch.
    ///
    /// Stage 2 of the three-stage flow. The driver:
    /// - links the descriptors into the SoC's chain representation. On
    ///   ESP32-C6 that is `descs[i].next = &descs[i+1]`,
    ///   `descs[n-1].next = null`. On GD32 the single configuration
    ///   snapshot is its own chain, so this step is a no-op there;
    /// - records the chain head and oneshot dispatch mode into the
    ///   channel's runtime state, so [`DmaChannel::service_interrupt`]
    ///   can fire [`DmaEvent::OneshotDone`] once when the tail is
    ///   consumed;
    /// - records `cfg` (direction / peripheral / width) so the subsequent
    ///   argless [`DmaChannel::start`] can kick off without re-taking it.
    ///
    /// Mirrors ESP32-C6 `start_chain`'s state-setup half, folded together
    /// with the old `link_chain` linkage.
    fn prepare_chain(
        &self,
        descs: &mut [Self::Desc],
        cfg: &DmaSlaveConfig,
    ) -> Result<()>;

    // ── Completion detection (hardware state — no register reads in
    //     the consumer) ──────────────────────────────────────────────

    /// Whether `desc` has been consumed by the DMA.
    ///
    /// On ESP32-C6 this tests the owner bit (`dw0[31] == 0` once
    /// `OUT_AUTO_WRBACK=1` has the DMA clear it after consumption).
    fn is_consumed(&self, desc: &Self::Desc) -> bool;

    /// Refill a consumed descriptor: reset the owner bit, optionally swap
    /// the buffer, and keep the chain linkage intact.
    ///
    /// Mirrors the I2S driver's existing
    /// `descs[widx] = DmaDescriptor::for_tx(new_buf, len, true)` rewrite.
    fn refill(&self, desc: &mut Self::Desc, buf: &mut [u8], eof: bool);

    /// Index of the ring segment the DMA is currently processing,
    /// relative to `ring_base`.
    ///
    /// On ESP32-C6 this reads `OUT_STATE[17:0]` (the descriptor address the
    /// DMA is on), subtracts `ring_base`, and divides by
    /// `size_of::<Desc>`. On GD32's M0/M1 ping-pong it returns 0/1.
    ///
    /// The arguments are supplied by the driver's `prepare_ring` (which
    /// also stores them in its runtime state); the trait surface stays
    /// generic across SoCs that locate the current segment differently.
    fn current_segment(
        &self,
        ring_base: *const Self::Desc,
        nsegs: usize,
    ) -> usize;

    // ── Start / stop ───────────────────────────────────────────────

    /// Kick off the transfer armed by the last
    /// [`DmaChannel::prepare_ring`] or [`DmaChannel::prepare_chain`].
    ///
    /// Stage 3 of the three-stage flow. Takes no arguments: the head
    /// descriptor, direction, peripheral, and dispatch mode were all
    /// recorded by the `prepare_*` step, so `start` is a pure kickoff
    /// (≈ Linux `issue_pending`). Whether the FIFO is reset on start is
    /// driver-internal: an ESP32-C6 ring starts without a reset
    /// (glitch-free audio) while a chain starts with one; GD32 has its
    /// own M0/M1 setup. The driver reads its runtime state to pick.
    ///
    /// Must be called *after* `prepare_*` (which wrote that state) and
    /// `enable_interrupt` if the completion callback is wanted. Calling
    /// `start` without a preceding `prepare_*` is a usage error
    /// (driver-defined: the ESP32-C6 impl returns `Err(NotReady)`).
    fn start(&self) -> Result<()>;

    /// Restart the outlink after the CPU has refilled descriptors.
    ///
    /// The DMA parks at a descriptor whose owner bit is clear; refilling
    /// the owner then calling this resumes the transfer. Mirrors
    /// ESP32-C6 `restart_tx`.
    fn restart(&self);

    /// Terminate all activity on this channel: break the ring, wait for
    /// total-EOF, reset the FSM. Mirrors the I2S `drain_and_stop` plus
    /// ESP32-C6 `reset_tx`/`reset_rx`.
    fn terminate(&self) -> Result<()>;

    // ── Callback registration (replaces polling) ───────────────────

    /// Register the completion callback.
    ///
    /// - oneshot chains: fires [`DmaEvent::OneshotDone`] once.
    /// - perpetual rings: fires [`DmaEvent::SegmentConsumed`] per
    ///   consumed segment.
    fn set_callback(&self, cb: DmaCallback);

    /// Enable the channel's completion interrupt (write the `ENA`
    /// register). Which bits get set is driver-internal.
    fn enable_interrupt(&self);

    /// Disable the channel's completion interrupt.
    fn disable_interrupt(&self);

    /// Interrupt service entry point: read raw status, clear flags,
    /// dispatch the callback. Called from the board's ISR adapter.
    ///
    /// SoC differences are sealed inside the driver `impl`.
    fn service_interrupt(&self);

    /// Copy `src` to `dst` via the DMA engine, blocking until completion.
    ///
    /// This is the memory-to-memory (memcpy) entry point — it corresponds
    /// to Linux's `device_prep_dma_memcpy` op, folded onto the same
    /// `dma_device` as the slave prep callbacks rather than living in a
    /// separate trait (the slave outlink/inlink flow modeled by
    /// [`prepare_chain`](DmaChannel::prepare_chain) /
    /// [`prepare_ring`](DmaChannel::prepare_ring) /
    /// [`start`](DmaChannel::start) cannot express the M2M case on its
    /// own: M2M needs the outlink and inlink mounted *together* plus the
    /// TX→RX FIFO bridge, with completion observed on the RX side as an
    /// RX-side EOF). A client that needs a pure RAM-to-RAM copy consults
    /// [`DmaCaps::MEMCPY`] first, then calls this method.
    ///
    /// # Completion
    ///
    /// Unlike the slave path, M2M completion is observed on the receive
    /// side: the source descriptor's EOF marker propagates through the M2M
    /// bridge to an RX-side EOF (ESP32-C6 `IN_SUC_EOF`). Implementations
    /// must wait for that condition synchronously — this is a blocking,
    /// poll-style call, not a callback-based one. (Interrupt-driven M2M
    /// completion, if ever needed, would be layered on top of
    /// [`set_callback`](DmaChannel::set_callback).)
    ///
    /// # Default
    ///
    /// The default returns [`HalError::NotSupport`](crate::err::HalError),
    /// so slave-only controllers (those that do *not* advertise
    /// [`DmaCaps::MEMCPY`]) compile without providing a stub.
    ///
    /// # Re-entrancy
    ///
    /// Must not be called re-entrantly on the same channel: it touches
    /// the channel's outlink/inlink and M2M-bridge registers without
    /// external serialization.
    fn m2m_transfer(src: &mut [u8], dst: &mut [u8]) -> crate::err::Result<()> {
        let _ = (src, dst);
        Err(crate::err::HalError::NotSupport)
    }
}
