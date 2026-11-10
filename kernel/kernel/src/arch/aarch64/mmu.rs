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

//! AArch64 MMU register setup and address-space activation.

use crate::mm::{paging::PageFormat, PhysicalAddress, VirtualAddress};
use aarch64_cpu::{
    asm::barrier::{dsb, isb, SY},
    registers::{MAIR_EL1, SCTLR_EL1, TCR_EL1, TTBR0_EL1, TTBR1_EL1},
};
use tock_registers::interfaces::*;

const KERNEL_VA_BITS: u64 = super::page::PageEntry::VIRT_ADDR_BITS as u64;
const TCR_TSZ: u64 = u64::BITS as u64 - KERNEL_VA_BITS;
pub(crate) const KERNEL_VIRT_START: u64 = crate::mm::KERNEL_VIRT_OFFSET as u64;
pub use crate::mm::{kernel_phys_to_virt, kernel_virt_to_phys};

#[inline]
fn flush_tlb_all() {
    super::asm::tlbi_all();
    dsb(SY);
    isb(SY);
}

/// Inner-shareable TLB invalidate for the whole EL1 translation regime.
///
/// Self-contained: `dsb ishst` makes prior page-table writes visible to the
/// inner-shareable domain's walkers, `tlbi vmalle1is` broadcasts the
/// invalidate, `dsb ish` waits for completion, `isb` synchronizes this CPU's
/// subsequent translations. Unlike [`flush_tlb_all`] (non-shareable
/// `tlbi vmalle1`), this reaches every core — required for user translations
/// that may be cached elsewhere (SMP) or whose backing frames may be reused.
/// See W4 for ASID-tagged shootdowns.
#[inline]
fn flush_tlb_all_inner_shareable() {
    // Canonical barrier sequence, matching `sync_kernel_mappings` below.
    unsafe {
        core::arch::asm!(
            "dsb ishst",
            "tlbi vmalle1is",
            "dsb ish",
            "isb",
            options(nostack, preserves_flags),
        );
    }
}

/// Publish kernel page-table changes and retire translations on every CPU
/// in the inner-shareable domain before backing/table frames can be reused.
pub(crate) fn sync_kernel_mappings() {
    unsafe {
        core::arch::asm!(
            "dsb ishst",
            "tlbi vmalle1is",
            "dsb ish",
            "isb",
            options(nostack, preserves_flags),
        );
    }
}

pub fn init_el1_enable_mmu() {
    let phys_addr = unsafe { crate::mm::paging::init_boot_pgtable::<super::page::PageEntry>() };
    dsb(SY);
    TTBR0_EL1.set(phys_addr as u64);
    TTBR1_EL1.set(phys_addr as u64);

    // Keep slots 0..3 in sync with the attribute indices in arch::page.
    MAIR_EL1.write(
        MAIR_EL1::Attr1_Normal_Outer::WriteBack_NonTransient_ReadWriteAlloc
            + MAIR_EL1::Attr1_Normal_Inner::WriteBack_NonTransient_ReadWriteAlloc
            + MAIR_EL1::Attr0_Device::nonGathering_nonReordering_EarlyWriteAck
            + MAIR_EL1::Attr2_Normal_Outer::NonCacheable
            + MAIR_EL1::Attr2_Normal_Inner::NonCacheable
            + MAIR_EL1::Attr3_Normal_Outer::WriteThrough_NonTransient_ReadWriteAlloc
            + MAIR_EL1::Attr3_Normal_Inner::WriteThrough_NonTransient_ReadWriteAlloc,
    );
    // Configure address translation related control information.
    TCR_EL1.write(
        TCR_EL1::TBI0::Used
            + TCR_EL1::IPS::Bits_32
            + TCR_EL1::TG0::KiB_4
            + TCR_EL1::SH0::Inner
            + TCR_EL1::ORGN0::WriteBack_ReadAlloc_WriteAlloc_Cacheable
            + TCR_EL1::IRGN0::WriteBack_ReadAlloc_WriteAlloc_Cacheable
            + TCR_EL1::EPD0::EnableTTBR0Walks
            + TCR_EL1::T0SZ.val(TCR_TSZ)
            + TCR_EL1::TG1::KiB_4
            + TCR_EL1::SH1::Inner
            + TCR_EL1::ORGN1::WriteBack_ReadAlloc_WriteAlloc_Cacheable
            + TCR_EL1::IRGN1::WriteBack_ReadAlloc_WriteAlloc_Cacheable
            + TCR_EL1::EPD1::EnableTTBR1Walks
            + TCR_EL1::T1SZ.val(TCR_TSZ),
    );

    // Clear tlb.
    flush_tlb_all();
    // Enable the MMU.
    SCTLR_EL1.modify(
        SCTLR_EL1::M::Enable
            + SCTLR_EL1::C::Cacheable
            + SCTLR_EL1::I::Cacheable
            + SCTLR_EL1::SA::Enable,
    );
    isb(SY);
}

/// Install a completed kernel page table on this CPU.
///
/// # Safety
/// The root and all child tables must be valid for the configured format and
/// remain live. The new table must map the executing code, stack, data and
/// devices. Tables must be fully built and must not be concurrently modified.
pub unsafe fn activate_kernel_table(root: PhysicalAddress) {
    dsb(SY);
    TCR_EL1.modify(TCR_EL1::EPD0::DisableTTBR0Walks);
    TTBR0_EL1.set(0);
    TTBR1_EL1.set(root.into_raw() as u64);
    isb(SY);
    flush_tlb_all();
}

/// Activate a user address space on this CPU by loading its root into
/// `TTBR0_EL1` and enabling TTBR0 walks.
///
/// Unlike [`activate_kernel_table`], this leaves `TTBR1_EL1` (the kernel
/// mapping) untouched — the kernel high-VA mapping stays live across the
/// switch, so the switch path itself keeps running on `TTBR1`. Only the
/// lower-VA half (user space) is re-pointed.
///
/// `asid` is currently ignored (ASID allocation arrives in W4); the inner
/// shareable `tlbi vmalle1is` here flushes any stale user translations from
/// the previous tenant before the new root takes effect. Callers MUST have
/// built the user table fully (mapped PT_LOAD segments + user stack) before
/// the first activation; the forged user `Context` then `eret`s into EL0 with
/// `TTBR0` pointing at this root.
///
/// # Safety
/// `root` must be a valid, live physical address of a page-table root built
/// for the configured format, and must not be concurrently modified after
/// activation. The caller must run at EL1 with the MMU on.
pub unsafe fn activate_user_table(root: PhysicalAddress, _asid: u16) {
    dsb(SY);
    // Re-enable TTBR0 walks (boot disables them via activate_kernel_table /
    // init_el1_enable_mmu sets EPD0::Enable, but kernel-thread switches
    // deactivate them). Kernel TTBR1 walks stay enabled throughout.
    TCR_EL1.modify(TCR_EL1::EPD0::EnableTTBR0Walks);
    TTBR0_EL1.set(root.into_raw() as u64);
    isb(SY);
    // Inner-shareable: even on single-core boards the frames backing the
    // previous user table may be reused, so translations must retire on all
    // cores. ASID-tagged flush (tlbi asid) arrives with W4.
    flush_tlb_all_inner_shareable();
}

/// Deactivate the user address space on this CPU when switching from a user
/// thread to a kernel/idle thread.
///
/// Disables TTBR0 walks and zeroes `TTBR0_EL1` so subsequent kernel-side
/// accesses cannot ride on stale user translations. The kernel continues to
/// run on `TTBR1_EL1`. This is the inverse of [`activate_user_table`] for the
/// user→kernel transition; kernel→kernel transitions do not call it.
///
/// # Safety
/// Caller must run at EL1 with the MMU on. After this call, any access to a
/// user VA faults (TTBR0 walk disabled) until a user table is reactivated.
pub unsafe fn deactivate_user_table() {
    dsb(SY);
    TCR_EL1.modify(TCR_EL1::EPD0::DisableTTBR0Walks);
    TTBR0_EL1.set(0);
    isb(SY);
    flush_tlb_all_inner_shareable();
}

/// Switch this CPU's user address space to the one the next thread needs.
///
/// `prev` is the root the outgoing (`old`) thread was running under, `next` is
/// the root the incoming thread needs: `Some(root)` for a user thread, `None`
/// for a kernel/idle thread. Both are resolved by the scheduler from the
/// threads themselves (a user thread's root is its process's
/// `address_space().root_address()`; kernel/idle threads have none) — this
/// function holds NO cache of its own. The outgoing thread *is* the authority
/// for what `TTBR0_EL1` currently holds, so the `prev == next` short-circuit
/// covers both the no-op cases without any stored state:
///
/// | prev        | next      | action              |
/// |-------------|-----------|---------------------|
/// | None        | None      | nothing             | kernel→kernel
/// | None        | Some(r)   | activate r          | kernel→user
/// | Some(r)     | Some(r)   | nothing             | user→same-user
/// | Some(other) | Some(r)   | activate r          | user→user (diff root)
/// | Some(r)     | None      | deactivate          | user→kernel
///
/// This is the sole call site the scheduler needs: it passes both resolved
/// roots and this function owns the comparison and the TTBR0/TCR/TLB sequence.
/// ASID is currently 0 (W4 will thread `{root, asid}` pairs through both args).
///
/// Runs with local IRQs disabled (the switch path holds them off).
pub(crate) fn switch_user_address_space(
    prev: Option<PhysicalAddress>,
    next: Option<PhysicalAddress>,
) {
    // Same root (including both None) => TTBR0 already correct; skip the write.
    if prev == next {
        return;
    }

    // SAFETY: the switch path runs at EL1 with the MMU on and IRQs off. `next`
    // (when Some) is a valid, live page-table root built by the loader before
    // the thread was enqueued. The kernel mapping (TTBR1) stays live throughout
    // both branches.
    unsafe {
        match next {
            Some(root) => activate_user_table(root, 0),
            None => deactivate_user_table(),
        }
    }
}
