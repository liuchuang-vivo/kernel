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

//! D-/I-cache maintenance for user images at seal time.
//!
//! The loader's `seal` stage calls [`CodeCache::synchronize`] with the
//! executable segment's *target* (user virtual) address range. The code bytes
//! were copied into backing frames through the kernel direct-map alias (see
//! `AddressSpace::write_image`), so the dirty D-cache lines are tagged with
//! that alias, not the user VA.
//!
//! Per path 3, `boot::init_user_app` activates the user translation table for
//! the duration of the loader pipeline (before this stage runs) via
//! `arch::switch_user_address_space`. With `TTBR0_EL1` walks enabled, `DC CVAU,
//! <user_va>` translates through hardware and cleans the same physical line the
//! alias wrote — the aarch64 D-cache is PIPT (physically tagged), so user-VA
//! and alias-VA clean operations reach the same line when both map the backing
//! PA. No software VA→PA→alias walk is needed.
//!
//! The I-cache is invalidated with `IC IALLUIS` (inner-shareable full
//! invalidate), which has no VA or translation dependency and is correct under
//! SMP (`qemu_virt64_aarch64` is `CONFIG_SMP=y`).

use core::arch::asm;

use aarch64_cpu::asm::barrier::{dsb, isb, ISH, SY};

use blueos_loader::{
    error::{ErrorContext, LoadErrorKind},
    memory::TargetRange,
    CodeCache, LoadError, LoadResult,
};

/// A [`CodeCache`] that maintains aarch64 D-/I-cache coherency for executable
/// segments of a user image.
///
/// The user translation table must already be active when `seal` runs, so that
/// `DC CVAU` by user VA reaches the backing frames through hardware
/// translation. `boot::init_user_app` establishes that activation (path 3); the
/// scheduler re-establishes it on every context switch into the owning thread.
pub struct Aarch64CodeCache;

impl Aarch64CodeCache {
    /// Construct a cache maintainer. No address-space binding is needed: the
    /// active user table at seal time supplies the translations.
    pub fn new() -> Self {
        Self
    }

    /// Size in bytes of one D-cache line, derived from `CTR_EL0.DminLine` as
    /// `4 << DminLine`. Reads `CTR_EL0` directly via `mrs`; the `aarch64-cpu`
    /// crate exposes no register struct for it and EL1 reads are untrapped by
    /// the kernel's `SCTLR_EL1` configuration.
    #[inline]
    fn d_cache_line_size() -> usize {
        let ctr: usize;
        // SAFETY: `mrs` of CTR_EL0 is a register read with no side effects at
        // EL1; the value is constant for the implementation. Condition flags
        // are unchanged.
        unsafe {
            asm!(
                "mrs {ctr}, ctr_el0",
                ctr = out(reg) ctr,
                options(nostack, preserves_flags),
            );
        }
        4usize << (ctr & 0xf)
    }
}

impl CodeCache for Aarch64CodeCache {
    fn synchronize(&mut self, executable: TargetRange) -> LoadResult<()> {
        // A DminLine is always a power of two (CTR_EL0 guarantee), so masking
        // is a valid alignment. Compute it once; an empty range still needs the
        // I-cache invalidate + barriers to order any prior writes.
        let d_line = Self::d_cache_line_size();

        if !executable.is_empty() {
            let start = executable.start().get();
            // `end` checks `start + len` overflow and returns the exclusive end.
            let end = executable.end()?.get();

            // Align the first VA down to its cache line; `DC CVAU` cleans the
            // whole line containing the supplied VA, so one call per line is
            // sufficient. Stepping by `d_line` then visits every line in
            // `[start, end)`. The user table is active (path 3), so hardware
            // translates each user VA to the backing frame directly — no
            // software walk, unlike the inactive-table design this replaced.
            let mut line_start = start & !(d_line as u64);
            while line_start < end {
                // SAFETY: `line_start` is a mapped user VA (the user table is
                // active per path 3); `DC CVAU` is a side-effecting register
                // operation with no soundness precondition beyond mapping.
                unsafe { clean_d_cache_line(line_start as usize) };
                line_start = line_start
                    .checked_add(d_line as u64)
                    .ok_or_else(|| LoadError::new(LoadErrorKind::IntegerOverflow, ErrorContext::None))?;
            }
        }

        // Publish the D-cache clean before I-cache invalidation.
        dsb(ISH);
        // Full inner-shareable I-cache invalidate: no VA/translation dependency,
        // correct under SMP. Simpler and SMP-safe versus per-line `IC IVAU`, and
        // matches the canonical post-write icache sequence.
        // SAFETY: `IC IALLUIS` is a global icache invalidate with no soundness
        // precondition beyond running at EL1; it does not touch memory.
        unsafe { invalidate_i_cache_all() };
        dsb(ISH);
        // `isb` only accepts `SY` (the `aarch64-cpu` crate implements `Isb` for
        // `SY` alone), which matches the canonical post-ic-invalidate sequence.
        isb(SY);
        Ok(())
    }
}

/// `dc cvau, <va>` — clean data cache by VA to the point of unification.
///
/// `DC CVAU` operates on the cache line containing the supplied VA; passing
/// any address within the line is sufficient. The VA must be mapped in the
/// current translation regime — here the active user table (path 3), which
/// maps the executable segment's backing frames at the user VA.
#[inline]
unsafe fn clean_d_cache_line(va: usize) {
    unsafe {
        asm!(
            "dc cvau, {va}",
            va = in(reg) va,
            options(nostack, preserves_flags),
        );
    }
}

/// `ic ialluis` — invalidate the entire I-cache, inner-shareable. Callers
/// issue `dsb ish` + `isb` to complete the sequence.
#[inline]
unsafe fn invalidate_i_cache_all() {
    unsafe {
        asm!("ic ialluis", options(nostack, preserves_flags));
    }
}
