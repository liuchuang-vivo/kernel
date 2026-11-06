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

extern crate alloc;

#[cfg(paging)]
mod user_stack;
#[cfg(paging)]
pub use user_stack::UserStackLayout;
#[cfg(paging)]
mod stack_startup;
#[cfg(paging)]
pub use stack_startup::UserStackStart;

#[cfg(paging)]
use crate::{
    arch::page::PageEntry,
    mm::{
        paging::{
            first_fit::FirstFitFrameAllocator, mapper::PageMapper, FrameAllocator, PageFormat,
        },
        PhysicalAddress,
    },
};
use crate::{error, mm::MemFlags, sync::SpinLock};
use alloc::collections::BTreeMap;

#[cfg(paging)]
pub const MMAP_MIN_DEFAULT: usize = PageEntry::PAGE_SIZE;
// Automatic-placement preference; explicit reservations may use lower addresses.
#[cfg(not(paging))]
pub const MMAP_MIN_DEFAULT: usize = 4096;

/// A contiguous virtual-address range `[start, end)`
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct Vma {
    start: usize,
    end: usize,
    flags: MemFlags,
}

impl Vma {
    /// Returns [`error::code::EINVAL`] if the range is empty or inverted
    /// (`start >= end`).
    pub const fn new(start: usize, end: usize, flags: MemFlags) -> Result<Self, error::Error> {
        if start >= end {
            return Err(error::code::EINVAL);
        }
        Ok(Self { start, end, flags })
    }

    pub fn start(&self) -> usize {
        self.start
    }

    pub fn end(&self) -> usize {
        self.end
    }

    pub fn flags(&self) -> MemFlags {
        self.flags
    }

    pub fn len(&self) -> usize {
        self.end - self.start
    }

    pub fn is_empty(&self) -> bool {
        self.end == self.start
    }

    /// `true` when `addr` lies in `[start, end)`.
    pub fn contains(&self, addr: usize) -> bool {
        self.start <= addr && addr < self.end
    }

    pub(crate) fn set_end(&mut self, new_end: usize) -> Result<(), error::Error> {
        if new_end <= self.start {
            return Err(error::code::EINVAL);
        }
        self.end = new_end;
        Ok(())
    }

    /// Change `start`. Because `start` is the BTreeMap key, the caller **must**
    /// remove the node from the tree and re-insert it under the new key after
    /// this call (see the [`AddressSpace`] invariant). Re-validates
    /// `start < end`.
    pub(crate) fn set_start(&mut self, new_start: usize) -> Result<(), error::Error> {
        if new_start >= self.end {
            return Err(error::code::EINVAL);
        }
        self.start = new_start;
        Ok(())
    }

    pub(crate) fn set_flags(&mut self, flags: MemFlags) {
        self.flags = flags;
    }
}

#[cfg(paging)]
#[must_use = "dropping the state releases its inactive mappings and tables"]
struct AddressSpaceState<F: FrameAllocator> {
    vmas: BTreeMap<usize, Vma>,
    mapper: PageMapper<F>,
    user_stack: Option<UserStackLayout>,
}

#[cfg(not(paging))]
struct AddressSpaceState {
    vmas: BTreeMap<usize, Vma>,
}

#[cfg(not(paging))]
impl AddressSpaceState {
    fn manages_vma(&self, _start: usize) -> bool {
        false
    }
}

#[cfg(paging)]
type State = AddressSpaceState<&'static FirstFitFrameAllocator>;
#[cfg(not(paging))]
type State = AddressSpaceState;

/// A process's independent virtual address space.
///
/// Paging spaces own inactive page tables and any initialized user-stack backing.
/// Destruction releases backing before the tables. Hardware activation and TLB
/// lifetime management must be integrated before installing the root on a CPU.
#[must_use = "dropping an address space releases its inactive mappings and tables"]
pub struct AddressSpace {
    state: SpinLock<State>,
    mmap_min: usize,
}

impl AddressSpace {
    /// Create an empty address space without changing the active CPU mappings.
    ///
    /// Paging systems require the boot frame allocator and kernel direct map
    /// to be initialized. Returns [`error::code::ENOMEM`] if no root-table frame
    /// can be allocated.
    pub fn new() -> Result<Self, error::Error> {
        #[cfg(paging)]
        {
            Self::new_with_allocator(FirstFitFrameAllocator::instance())
        }
        #[cfg(not(paging))]
        {
            Ok(Self {
                state: SpinLock::new(AddressSpaceState {
                    vmas: BTreeMap::new(),
                }),
                mmap_min: MMAP_MIN_DEFAULT,
            })
        }
    }

    #[cfg(paging)]
    fn new_with_allocator(
        allocator: &'static FirstFitFrameAllocator,
    ) -> Result<Self, error::Error> {
        Ok(Self {
            state: SpinLock::new(AddressSpaceState::new(allocator)?),
            mmap_min: MMAP_MIN_DEFAULT,
        })
    }

    /// Physical root of this space's inactive user page table.
    #[cfg(paging)]
    pub fn root_address(&self) -> PhysicalAddress {
        self.state.irqsave_lock().mapper.root_address()
    }

    /// Allocate a fixed-size initial user stack at the user address limit.
    ///
    /// The usable size must be a nonzero page multiple. One additional lower
    /// guard page is reserved but remains unmapped. Stack pages are zeroed,
    /// user-readable/writable and non-executable. The table must remain inactive;
    /// this does not set a CPU stack pointer or write a process startup frame.
    ///
    /// # Errors
    /// - [`error::code::EINVAL`] — invalid size or insufficient address range.
    /// - [`error::code::EEXIST`] — a stack exists or either reservation overlaps.
    /// - [`error::code::ENOMEM`] — backing or page-table frame allocation failed.
    #[cfg(paging)]
    pub fn init_user_stack(&self, size: usize) -> Result<UserStackLayout, error::Error> {
        let top = self.user_end();
        self.state.irqsave_lock().init_user_stack(top, size)
    }

    /// Snapshot of the initial user stack; the address space retains ownership.
    #[cfg(paging)]
    pub fn user_stack(&self) -> Option<UserStackLayout> {
        self.state.irqsave_lock().user_stack
    }

    /// Copy kernel bytes into this space's initial user stack, without activating it.
    ///
    /// The entire destination is checked before copying. Empty writes may use
    /// the exclusive stack top. All mappings must still be inactive; callers
    /// must finish initialization before publishing the stack to a user thread.
    /// This is a preparation interface, not runtime `copy_to_user`.
    ///
    /// Returns `ENOENT` without a stack, `EINVAL` for an out-of-stack or
    /// overflowing range, and `EIO` for inconsistent stack metadata/backing.
    #[cfg(paging)]
    pub fn write_user_stack(&self, destination: usize, bytes: &[u8]) -> Result<(), error::Error> {
        self.state
            .irqsave_lock()
            .write_user_stack(destination, bytes)
    }

    /// Build a native-word ELF-style startup stack in the inactive address space.
    ///
    /// Writes `argc`, null-terminated `argv`/`envp` pointer arrays, and `auxv`
    /// pairs followed by `(AT_NULL, 0)`. Input strings exclude their terminating
    /// NUL and may contain arbitrary non-NUL bytes. Auxiliary keys must be
    /// nonzero; values and ABI-specific entries are supplied by the loader.
    /// The returned SP is 16-byte aligned. No CPU registers are changed.
    ///
    /// Validation and backing checks precede all writes, under the state lock.
    /// A successful rebuild replaces the prior startup image; callers must not
    /// publish its addresses until preparation is complete.
    ///
    /// Returns `ENOENT` without a stack, `EINVAL` for embedded NUL/AT_NULL,
    /// `ENOSPC` if the image does not fit, or `EIO` for inconsistent backing.
    #[cfg(paging)]
    pub fn build_user_stack(
        &self,
        argv: &[&[u8]],
        envp: &[&[u8]],
        auxv: &[(usize, usize)],
    ) -> Result<UserStackStart, error::Error> {
        self.state.irqsave_lock().build_user_stack(argv, envp, auxv)
    }

    /// Default lower bound for future mmap address selection.
    pub fn mmap_min(&self) -> usize {
        self.mmap_min
    }

    /// Exclusive upper bound for this space's virtual-address intervals.
    pub fn user_end(&self) -> usize {
        #[cfg(paging)]
        {
            1usize << PageEntry::VIRT_ADDR_BITS
        }
        #[cfg(not(paging))]
        {
            usize::MAX
        }
    }

    fn validate_range(&self, start: usize, end: usize) -> Result<(), error::Error> {
        if start >= end || end > self.user_end() {
            return Err(error::code::EINVAL);
        }
        Ok(())
    }

    /// Insert `vma` into this address space.
    ///
    /// Returns [`error::code::EINVAL`] if the range is outside the user address
    /// space, or [`error::code::EEXIST`] if it overlaps an existing VMA. On
    /// failure the space is left unchanged. Byte-granular explicit reservations
    /// may start below [`Self::mmap_min`]; this only registers metadata.
    pub fn insert(&self, vma: Vma) -> Result<(), error::Error> {
        self.validate_range(vma.start(), vma.end())?;
        let mut state = self.state.irqsave_lock();
        let vmas = &mut state.vmas;
        let (start, end) = (vma.start(), vma.end());
        if Self::overlaps(vmas, start, end) {
            return Err(error::code::EEXIST);
        }
        vmas.insert(start, vma);
        Ok(())
    }

    /// Reserve a `[start, end)` VA range with `flags` at the caller-chosen
    /// address. Returns a snapshot of the inserted [`Vma`].
    /// This only registers metadata; byte-granular intervals and addresses
    /// below [`Self::mmap_min`] are allowed.
    ///
    /// # Errors
    /// - [`error::code::EINVAL`] — empty, inverted or outside the user range.
    /// - [`error::code::EEXIST`] — the range overlaps an existing VMA.
    pub fn reserve(&self, start: usize, end: usize, flags: MemFlags) -> Result<Vma, error::Error> {
        let vma = Vma::new(start, end, flags)?;
        self.insert(vma)?;
        Ok(vma)
    }

    /// Remove and return a snapshot of the VMA whose start address equals
    /// `start`, if any. A managed user-stack or guard VMA remains reserved and
    /// returns `None`.
    pub fn remove(&self, start: usize) -> Option<Vma> {
        let mut state = self.state.irqsave_lock();
        if state.manages_vma(start) {
            return None;
        }
        state.vmas.remove(&start)
    }

    /// Grow the VMA at `start` so its `end` becomes `new_end` (`> start`).
    /// The key (`start`) is unchanged, so the node is **not** re-keyed. The
    /// grown range must not overlap the next VMA.
    ///
    /// # Errors
    /// - [`error::code::ENOENT`] — no VMA at `start`.
    /// - [`error::code::EINVAL`] — `new_end <= start`, outside the user range,
    ///   or the grown range would overlap the next VMA.
    /// - [`error::code::EBUSY`] — the target is a managed user stack or guard.
    pub fn extend_up(&self, start: usize, new_end: usize) -> Result<(), error::Error> {
        use core::ops::Bound::{Excluded, Unbounded};

        let mut state = self.state.irqsave_lock();
        let current = state.vmas.get(&start).copied().ok_or(error::code::ENOENT)?;
        if state.manages_vma(start) {
            return Err(error::code::EBUSY);
        }
        self.validate_range(current.start(), new_end)?;
        let vmas = &mut state.vmas;
        // Reject overlap with the immediately-following VMA: its start must be
        // >= new_end. The next node's start is the BTreeMap key (free).
        if let Some((next_start, _)) = vmas.range((Excluded(start), Unbounded)).next() {
            if *next_start < new_end {
                return Err(error::code::EINVAL);
            }
        }
        vmas.get_mut(&start).unwrap().set_end(new_end)?;
        Ok(())
    }

    /// Merge the VMA at `left_start` with the immediately-following VMA.
    ///
    /// # Errors
    /// - [`error::code::ENOENT`] — no VMA at `left_start`, or no adjacent
    ///   right VMA.
    /// - [`error::code::EINVAL`] — flag mismatch.
    /// - [`error::code::EBUSY`] — an operand is a managed user stack or guard.
    pub fn merge_with_next(&self, left_start: usize) -> Result<(), error::Error> {
        let mut state = self.state.irqsave_lock();
        let left = state
            .vmas
            .get(&left_start)
            .copied()
            .ok_or(error::code::ENOENT)?;
        if state.manages_vma(left_start) {
            return Err(error::code::EBUSY);
        }

        let right_start = left.end();
        let left_flags = left.flags();

        let right = state
            .vmas
            .get(&right_start)
            .copied()
            .filter(|r| r.start() == right_start)
            .ok_or(error::code::ENOENT)?;
        if state.manages_vma(right_start) {
            return Err(error::code::EBUSY);
        }
        if right.flags() != left_flags {
            return Err(error::code::EINVAL);
        }

        let vmas = &mut state.vmas;
        let new_end = vmas.remove(&right_start).unwrap().end();
        vmas.get_mut(&left_start).unwrap().set_end(new_end)?;
        Ok(())
    }

    /// Split the VMA at `start` into two: `[start, addr)` and `[addr, old_end)`.
    ///
    /// # Errors
    /// - [`error::code::ENOENT`] — no VMA at `start`.
    /// - [`error::code::EINVAL`] — `addr` is not strictly interior
    ///   (`<= start` or `>= old_end`).
    /// - [`error::code::EBUSY`] — the target is a managed user stack or guard.
    pub fn split_at(&self, start: usize, addr: usize) -> Result<(), error::Error> {
        let mut state = self.state.irqsave_lock();
        let current = state.vmas.get(&start).copied().ok_or(error::code::ENOENT)?;
        if state.manages_vma(start) {
            return Err(error::code::EBUSY);
        }
        let vmas = &mut state.vmas;
        let (old_end, flags) = (current.end(), current.flags());
        if addr >= old_end {
            return Err(error::code::EINVAL);
        }
        vmas.get_mut(&start).unwrap().set_end(addr)?;
        let right = Vma::new(addr, old_end, flags)?;
        vmas.insert(addr, right);
        Ok(())
    }

    /// Return a snapshot of the [`Vma`] covering `addr`, if any.
    pub fn find_containing(&self, addr: usize) -> Option<Vma> {
        let state = self.state.irqsave_lock();
        let vmas = &state.vmas;
        vmas.range(..=addr)
            .next_back()
            .map(|(_, vma)| *vma)
            .filter(|vma| vma.contains(addr))
    }

    /// Invoke `f` for every VMA in ascending start-address order.
    ///
    /// The tree lock is held for the duration of the call, so `f` must not
    /// re-enter this `AddressSpace`. Each `&Vma` is borrowed straight from
    /// the tree guard and is invalidated once the closure returns — `f` must
    /// not retain the reference.
    pub fn for_each<F: FnMut(&Vma)>(&self, mut f: F) {
        let state = self.state.irqsave_lock();
        let vmas = &state.vmas;
        for vma in vmas.values() {
            f(vma);
        }
    }

    pub fn is_empty(&self) -> bool {
        self.state.irqsave_lock().vmas.is_empty()
    }

    pub fn count(&self) -> usize {
        self.state.irqsave_lock().vmas.len()
    }

    /// `true` if a `[start, end)` range would overlap any existing VMA.
    /// Caller holds the tree lock. `start` is the BTreeMap key
    fn overlaps(vmas: &BTreeMap<usize, Vma>, start: usize, end: usize) -> bool {
        if let Some((_, prev)) = vmas.range(..=start).next_back() {
            if prev.end() > start {
                return true;
            }
        }
        if vmas.range(start..end).next().is_some() {
            return true;
        }
        false
    }
}

impl core::fmt::Debug for AddressSpace {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        let count = self.count();
        let mut debug = f.debug_struct("AddressSpace");
        debug
            .field("vma_count", &count)
            .field("mmap_min", &self.mmap_min)
            .field("user_end", &self.user_end());
        #[cfg(paging)]
        debug.field("root_address", &self.root_address());
        debug.finish()
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use blueos_test_macro::test;

    /// Build a VMA spanning `[start, end)`. Addresses are arbitrary — no
    /// alignment is imposed — so call sites pass the `end` explicitly rather
    /// than a page count.
    fn vma(start: usize, end: usize, flags: MemFlags) -> Vma {
        Vma::new(start, end, flags).unwrap()
    }

    #[test]
    fn vma_basic() {
        let v = Vma::new(0x1000, 0x3000, MemFlags::READ | MemFlags::WRITE).unwrap();
        assert_eq!(v.start(), 0x1000);
        assert_eq!(v.end(), 0x3000);
        assert_eq!(v.len(), 0x2000);
        assert!(v.flags().contains(MemFlags::READ));
        assert!(v.flags().contains(MemFlags::WRITE));
        assert!(!v.flags().contains(MemFlags::EXECUTE));
        assert!(v.contains(0x1000));
        assert!(v.contains(0x2fff));
        assert!(!v.contains(0x3000));

        // Empty range.
        assert!(Vma::new(0x1000, 0x1000, MemFlags::READ).is_err());
        // Inverted range.
        assert!(Vma::new(0x3000, 0x1000, MemFlags::READ).is_err());
        // Non-page-aligned ranges are now legal (the invariant was removed).
        assert!(Vma::new(0x1, 0x3, MemFlags::READ).is_ok());
        assert!(Vma::new(0x1000, 0x1001, MemFlags::READ).is_ok());

        // Mutators. `set_end`/`set_start`/`set_flags` validate ordering only.
        let mut v = Vma::new(0x1000, 0x3000, MemFlags::READ).unwrap();
        assert!(v.set_end(0x4000).is_ok());
        assert_eq!(v.end(), 0x4000);
        // Shrinking below `start` or to `start` is rejected.
        assert!(v.set_end(0x1000).is_err());
        // Non-page-aligned `end` is now accepted.
        assert!(v.set_end(0x4001).is_ok());
        assert_eq!(v.end(), 0x4001);
        // `set_start` keeps ordering.
        assert!(v.set_start(0x2000).is_ok());
        assert_eq!(v.start(), 0x2000);
        assert!(v.set_start(v.end()).is_err());
        // Non-page-aligned `start` is now accepted.
        assert!(v.set_start(0x2001).is_ok());
        assert_eq!(v.start(), 0x2001);
        v.set_flags(MemFlags::WRITE);
        assert!(v.flags().contains(MemFlags::WRITE));
    }

    #[test]
    fn address_space_starts_empty() {
        let aspace = AddressSpace::new().unwrap();
        assert!(aspace.is_empty());
        assert_eq!(aspace.count(), 0);
        assert_eq!(aspace.mmap_min(), MMAP_MIN_DEFAULT);
        #[cfg(paging)]
        assert_eq!(aspace.user_end(), 1usize << PageEntry::VIRT_ADDR_BITS);
        #[cfg(not(paging))]
        assert_eq!(aspace.user_end(), usize::MAX);
        assert!(aspace.find_containing(0).is_none());
        assert!(aspace.find_containing(0x1000).is_none());
    }

    #[cfg(paging)]
    #[test]
    fn address_space_roots_are_private_and_empty() {
        use crate::mm::{kernel_phys_to_virt, kernel_space::KernelAddressSpace};
        use aarch64_cpu::registers::{TTBR0_EL1, TTBR1_EL1};
        use tock_registers::interfaces::Readable;

        let before = (TTBR0_EL1.get(), TTBR1_EL1.get());
        let first = AddressSpace::new().unwrap();
        let second = AddressSpace::new().unwrap();
        assert_ne!(first.root_address(), second.root_address());
        let kernel_root = KernelAddressSpace::get().unwrap().root_address();
        for space in [&first, &second] {
            let root = space.root_address();
            assert_ne!(root, kernel_root);
            assert_eq!(root.into_raw() % PageEntry::PAGE_SIZE, 0);
            let ptr = kernel_phys_to_virt(root).into_raw() as *const u8;
            // SAFETY: this space owns a live, inactive root accessible via the
            // kernel direct map. No operation in this test changes the table.
            let bytes = unsafe { core::slice::from_raw_parts(ptr, PageEntry::PAGE_SIZE) };
            assert!(bytes.iter().all(|byte| *byte == 0));
        }
        assert_eq!((TTBR0_EL1.get(), TTBR1_EL1.get()), before);
    }

    #[cfg(paging)]
    #[test]
    fn address_space_creation_reports_missing_root_frame() {
        static EMPTY: FirstFitFrameAllocator = FirstFitFrameAllocator::new();
        assert_eq!(
            AddressSpace::new_with_allocator(&EMPTY).unwrap_err(),
            error::code::ENOMEM,
        );
        assert!(EMPTY.usage().is_none());
    }

    #[test]
    fn address_space_explicit_low_addresses_ignore_mmap_min() {
        let space = AddressSpace::new().unwrap();
        assert!(space.mmap_min() > 2);
        let flags = MemFlags::READ | MemFlags::GLOBAL;
        let first = space.reserve(0, 1, flags).unwrap();
        space
            .insert(Vma::new(1, 2, MemFlags::WRITE).unwrap())
            .unwrap();
        assert_eq!(first.flags(), flags);
        assert_eq!(space.find_containing(0), Some(first));
        assert_eq!(space.count(), 2);
    }

    #[test]
    fn address_space_upper_bound_is_exclusive() {
        let space = AddressSpace::new().unwrap();
        let end = space.user_end();
        let last = space.reserve(end - 1, end, MemFlags::READ).unwrap();
        assert_eq!(space.find_containing(end - 1), Some(last));
        assert!(space.find_containing(end).is_none());
        assert!(space.remove(end).is_none());
        assert_eq!(
            space.reserve(end, end.saturating_add(1), MemFlags::READ),
            Err(error::code::EINVAL),
        );
        #[cfg(paging)]
        {
            assert_eq!(
                space.reserve(end - 1, end + 1, MemFlags::READ),
                Err(error::code::EINVAL),
            );
            assert_eq!(space.extend_up(end - 1, end + 1), Err(error::code::EINVAL));
        }
        assert_eq!(space.count(), 1);
        assert_eq!(space.find_containing(end - 1), Some(last));
    }

    #[test]
    fn address_space_missing_extreme_keys_do_not_overflow() {
        let space = AddressSpace::new().unwrap();
        assert_eq!(space.extend_up(usize::MAX, 0), Err(error::code::ENOENT));
        assert_eq!(space.merge_with_next(usize::MAX), Err(error::code::ENOENT));
        assert_eq!(space.split_at(usize::MAX, 0), Err(error::code::ENOENT));
        assert!(space.find_containing(usize::MAX).is_none());
        assert!(space.remove(usize::MAX).is_none());
        assert_eq!(
            space.reserve(usize::MAX, 0, MemFlags::READ),
            Err(error::code::EINVAL)
        );
        assert!(space.is_empty());
    }

    #[cfg(paging)]
    #[test]
    fn address_space_rejects_kernel_intervals_without_restricting_vma() {
        let space = AddressSpace::new().unwrap();
        let existing = space.reserve(0x1000, 0x2000, MemFlags::READ).unwrap();
        let start = crate::mm::KERNEL_VIRT_OFFSET;
        let kernel_vma = Vma::new(start, start + 1, MemFlags::READ).unwrap();
        assert_eq!(space.insert(kernel_vma), Err(error::code::EINVAL));
        assert_eq!(
            space.reserve(start, start + 1, MemFlags::READ),
            Err(error::code::EINVAL)
        );
        let end = space.user_end();
        assert_eq!(
            space.insert(Vma::new(end - 1, end + 1, MemFlags::READ).unwrap()),
            Err(error::code::EINVAL),
        );
        assert_eq!(space.count(), 1);
        assert_eq!(space.find_containing(0x1000), Some(existing));
    }

    #[test]
    fn address_space_split_merge_preserves_boundary_and_flags() {
        let space = AddressSpace::new().unwrap();
        let end = space.user_end();
        let flags = MemFlags::WRITE | MemFlags::GLOBAL;
        let original = space.reserve(end - 3, end, flags).unwrap();
        space.split_at(end - 3, end - 2).unwrap();
        assert_eq!(space.count(), 2);
        assert_eq!(space.find_containing(end - 3).unwrap().flags(), flags);
        assert_eq!(space.find_containing(end - 1).unwrap().flags(), flags);
        space.merge_with_next(end - 3).unwrap();
        assert_eq!(space.count(), 1);
        assert_eq!(space.find_containing(end - 1), Some(original));
    }

    #[test]
    fn address_space_insert_find() {
        let aspace = AddressSpace::new().unwrap();
        aspace.insert(vma(0x0000, 0x2000, MemFlags::READ)).unwrap();
        aspace
            .insert(vma(0x5000, 0x6000, MemFlags::READ | MemFlags::WRITE))
            .unwrap();
        aspace
            .insert(vma(0x9000, 0xc000, MemFlags::READ | MemFlags::EXECUTE))
            .unwrap();
        assert_eq!(aspace.count(), 3);

        assert_eq!(aspace.find_containing(0x0).unwrap().start(), 0x0000);
        assert_eq!(aspace.find_containing(0x1fff).unwrap().start(), 0x0000);
        assert_eq!(aspace.find_containing(0x5000).unwrap().start(), 0x5000);
        assert_eq!(aspace.find_containing(0xbfff).unwrap().start(), 0x9000);

        // A gap between VMAs returns None.
        assert!(aspace.find_containing(0x4000).is_none());
        // Just past the last VMA returns None.
        assert!(aspace.find_containing(0xc000).is_none());
    }

    #[test]
    fn address_space_rejects_overlap() {
        let aspace = AddressSpace::new().unwrap();
        aspace
            .insert(vma(0x10000, 0x14000, MemFlags::READ))
            .unwrap();

        // Shares a start -> overlap -> EEXIST.
        let dup = vma(0x10000, 0x11000, MemFlags::READ);
        assert_eq!(aspace.insert(dup), Err(error::code::EEXIST));
        // Begins inside the existing VMA -> overlap.
        assert!(aspace
            .insert(vma(0x11000, 0x12000, MemFlags::READ))
            .is_err());
        // Begins before and extends into the existing VMA -> overlap.
        assert!(aspace.insert(vma(0xf000, 0x13000, MemFlags::READ)).is_err());
        // Adjacent (not overlapping) is fine.
        aspace
            .insert(vma(0x14000, 0x15000, MemFlags::READ))
            .unwrap();

        assert_eq!(aspace.count(), 2);
    }

    #[test]
    fn address_space_remove() {
        let aspace = AddressSpace::new().unwrap();
        aspace
            .insert(vma(0x20000, 0x28000, MemFlags::READ))
            .unwrap();
        aspace
            .insert(vma(0x28000, 0x30000, MemFlags::WRITE))
            .unwrap();

        let removed = aspace.remove(0x20000);
        assert_eq!(removed.map(|v| v.start()), Some(0x20000));
        assert_eq!(aspace.count(), 1);
        assert!(aspace.find_containing(0x20000).is_none());
        assert_eq!(aspace.find_containing(0x28000).unwrap().start(), 0x28000);

        // Removing a missing start is a no-op.
        assert!(aspace.remove(0xdeadb000).is_none());
        assert_eq!(aspace.count(), 1);
    }

    #[test]
    fn address_space_for_each() {
        let aspace = AddressSpace::new().unwrap();
        aspace
            .insert(vma(0x30000, 0x31000, MemFlags::READ))
            .unwrap();
        aspace
            .insert(vma(0x10000, 0x11000, MemFlags::READ))
            .unwrap();
        aspace
            .insert(vma(0x20000, 0x21000, MemFlags::READ))
            .unwrap();

        let mut starts = alloc::vec::Vec::new();
        aspace.for_each(|v| starts.push(v.start()));
        assert_eq!(starts, alloc::vec![0x10000, 0x20000, 0x30000]);
    }

    #[test]
    fn address_space_reserve_basic() {
        let aspace = AddressSpace::new().unwrap();
        let v = aspace
            .reserve(0x1000, 0x3000, MemFlags::READ | MemFlags::WRITE)
            .unwrap();
        let (vs, ve, vf) = (v.start(), v.end(), v.flags());
        assert_eq!(vs, 0x1000);
        assert_eq!(ve, 0x3000);
        assert!(vf.contains(MemFlags::READ));
        assert!(vf.contains(MemFlags::WRITE));
        assert!(!vf.contains(MemFlags::EXECUTE));
        assert_eq!(aspace.count(), 1);
        // The reserved range is findable as a VMA.
        assert_eq!(aspace.find_containing(vs).unwrap().start(), vs);
    }

    #[test]
    fn address_space_reserve_fits_between() {
        let aspace = AddressSpace::new().unwrap();
        // Two occupying VMAs; a free gap sits between them.
        aspace.insert(vma(0x1000, 0x2000, MemFlags::READ)).unwrap();
        aspace.insert(vma(0x3000, 0x4000, MemFlags::READ)).unwrap();
        // Reserve exactly the gap [0x2000, 0x3000).
        let v = aspace.reserve(0x2000, 0x3000, MemFlags::READ).unwrap();
        let (vs, ve, _) = (v.start(), v.end(), v.flags());
        assert_eq!(vs, 0x2000);
        assert_eq!(ve, 0x3000);
        assert_eq!(aspace.count(), 3);
        // All three ranges are independently findable, no overlap.
        assert_eq!(aspace.find_containing(0x1000).unwrap().start(), 0x1000);
        assert_eq!(aspace.find_containing(0x2000).unwrap().start(), 0x2000);
        assert_eq!(aspace.find_containing(0x3000).unwrap().start(), 0x3000);
    }

    #[test]
    fn address_space_reserve_rejects_empty_range() {
        let aspace = AddressSpace::new().unwrap();
        // `start >= end` is rejected by `Vma::new` as EINVAL.
        assert_eq!(
            aspace.reserve(0x1000, 0x1000, MemFlags::READ).err(),
            Some(error::code::EINVAL)
        );
        // Inverted range likewise.
        assert_eq!(
            aspace.reserve(0x3000, 0x1000, MemFlags::READ).err(),
            Some(error::code::EINVAL)
        );
        assert!(aspace.is_empty());
    }

    #[test]
    fn address_space_reserve_rejects_overlap() {
        let aspace = AddressSpace::new().unwrap();
        aspace.reserve(0x1000, 0x2000, MemFlags::READ).unwrap();
        // Overlapping range → EEXIST.
        assert_eq!(
            aspace.reserve(0x1500, 0x2500, MemFlags::READ).err(),
            Some(error::code::EEXIST)
        );
        assert_eq!(aspace.count(), 1);
    }

    #[test]
    fn address_space_reserve_then_remove() {
        let aspace = AddressSpace::new().unwrap();
        let v = aspace.reserve(0x1000, 0x2000, MemFlags::READ).unwrap();
        let first_start = v.start();
        assert_eq!(aspace.count(), 1);

        aspace.remove(first_start);
        assert!(aspace.is_empty());

        // Re-reserve the same range → succeeds at the same address.
        let v2 = aspace.reserve(0x1000, 0x2000, MemFlags::READ).unwrap();
        assert_eq!(v2.start(), first_start);
        assert_eq!(aspace.count(), 1);
    }

    #[test]
    fn address_space_extend_up_grows_unbacked() {
        let aspace = AddressSpace::new().unwrap();
        let v = aspace
            .reserve(0x1000, 0x2000, MemFlags::READ | MemFlags::WRITE)
            .unwrap();
        let (vs, ve, _) = (v.start(), v.end(), v.flags());
        // Grow; key (start) unchanged, count unchanged.
        let new_end = ve + 0x1000;
        aspace.extend_up(vs, new_end).unwrap();
        // `v` is a stale snapshot; re-lookup to see the grown end.
        let ve2 = aspace.find_containing(vs).unwrap().end();
        assert_eq!(ve2, new_end);
        assert_eq!(aspace.count(), 1);
        // The grown range is findable.
        assert_eq!(aspace.find_containing(new_end - 1).unwrap().start(), vs);
        // No new node was added.
        assert_eq!(aspace.count(), 1);
        aspace.remove(vs);
    }

    #[test]
    fn address_space_extend_up_rejects_overlap() {
        let aspace = AddressSpace::new().unwrap();
        let a = aspace.reserve(0x1000, 0x2000, MemFlags::READ).unwrap();
        let (as_, ae, _) = (a.start(), a.end(), a.flags());
        // Place a second VMA right after `a`.
        let b = aspace.reserve(0x2000, 0x3000, MemFlags::READ).unwrap();
        let (bs, _, _) = (b.start(), b.end(), b.flags());
        assert_eq!(bs, ae); // adjacent
                            // Extending `a` into `b` must fail.
        assert_eq!(aspace.extend_up(as_, bs + 1), Err(error::code::EINVAL));
        // `a` is unchanged on failure.
        assert_eq!(aspace.find_containing(as_).unwrap().end(), ae);
        aspace.remove(as_);
        aspace.remove(bs);
    }

    #[test]
    fn address_space_merge_with_next_same_flags() {
        let aspace = AddressSpace::new().unwrap();
        // Two adjacent same-flag VMAs.
        let a = aspace.reserve(0x1000, 0x2000, MemFlags::READ).unwrap();
        let (as_, ae, _) = (a.start(), a.end(), a.flags());
        let b = aspace.reserve(0x2000, 0x3000, MemFlags::READ).unwrap();
        let (bs, be, _) = (b.start(), b.end(), b.flags());
        assert_eq!(bs, ae); // adjacency
        assert_eq!(aspace.count(), 2);

        aspace.merge_with_next(as_).unwrap();
        // One VMA now spans both; count drops by 1.
        assert_eq!(aspace.count(), 1);
        let m = aspace.find_containing(as_).unwrap();
        assert_eq!(m.start(), as_);
        assert_eq!(m.end(), be); // merged end == right's old end
                                 // The right VMA's start is no longer a separate node.
        assert!(aspace.find_containing(bs).is_none_or(|n| n.start() == as_));
        aspace.remove(as_);
    }

    #[test]
    fn address_space_merge_with_next_rejects_flag_mismatch() {
        let aspace = AddressSpace::new().unwrap();
        let a = aspace.reserve(0x1000, 0x2000, MemFlags::READ).unwrap();
        let (as_, _, _) = (a.start(), a.end(), a.flags());
        let b = aspace.reserve(0x2000, 0x3000, MemFlags::WRITE).unwrap();
        let (bs, _, _) = (b.start(), b.end(), b.flags());
        assert_eq!(aspace.merge_with_next(as_), Err(error::code::EINVAL));
        // Both VMAs intact on failure.
        assert_eq!(aspace.count(), 2);
        assert!(aspace.find_containing(bs).is_some());
        aspace.remove(as_);
        aspace.remove(bs);
    }

    #[test]
    fn address_space_merge_with_next_rejects_gap() {
        let aspace = AddressSpace::new().unwrap();
        // Two VMAs with a gap between them.
        aspace
            .insert(vma(0x10000, 0x11000, MemFlags::READ))
            .unwrap();
        aspace
            .insert(vma(0x20000, 0x21000, MemFlags::READ))
            .unwrap();
        // 0x11000 != 0x20000 → not adjacent.
        assert_eq!(aspace.merge_with_next(0x10000), Err(error::code::ENOENT));
        assert_eq!(aspace.count(), 2);
        aspace.remove(0x10000);
        aspace.remove(0x20000);
    }

    #[test]
    fn address_space_split_at_interior() {
        let aspace = AddressSpace::new().unwrap();
        let v = aspace.reserve(0x1000, 0x5000, MemFlags::READ).unwrap();
        let (vs, ve, vf) = (v.start(), v.end(), v.flags());
        let split = vs + 0x2000;
        aspace.split_at(vs, split).unwrap();
        // Two VMAs now; left shrunk to [vs, split), right [split, ve).
        assert_eq!(aspace.count(), 2);
        let l = aspace.find_containing(vs).unwrap();
        assert_eq!(l.start(), vs);
        assert_eq!(l.end(), split);
        assert_eq!(l.flags(), vf);
        let r = aspace.find_containing(split).unwrap();
        assert_eq!(r.start(), split);
        assert_eq!(r.end(), ve);
        assert_eq!(r.flags(), vf);
        // The boundary: just left of `split` is the left VMA.
        assert_eq!(aspace.find_containing(split - 1).unwrap().start(), vs);
        aspace.remove(vs);
        aspace.remove(split);
    }

    #[test]
    fn address_space_split_at_rejects_boundary() {
        let aspace = AddressSpace::new().unwrap();
        let v = aspace.reserve(0x1000, 0x3000, MemFlags::READ).unwrap();
        let (vs, ve, _) = (v.start(), v.end(), v.flags());
        // `addr == start` or `addr == end` are not interior.
        assert_eq!(aspace.split_at(vs, vs), Err(error::code::EINVAL));
        assert_eq!(aspace.split_at(vs, ve), Err(error::code::EINVAL));
        // A non-page-aligned but strictly-interior `addr` is now valid.
        aspace.split_at(vs, vs + 1).unwrap();
        assert_eq!(aspace.count(), 2);
        aspace.remove(vs);
        aspace.remove(vs + 1);
    }
}
