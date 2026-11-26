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

//! Initial user-stack backing and layout for inactive address spaces.

use super::{AddressSpace, AddressSpaceState, PageEntry, Vma};
use crate::{
    error,
    mm::{
        kernel_phys_to_virt,
        paging::{
            mapper::{FrameInit, PageMapper},
            FrameAllocator, PageFormat,
        },
        CachePolicy, MemAttribute, MemFlags, MemType, VirtualAddress,
    },
};
use alloc::collections::BTreeMap;

/// A snapshot of an initial user stack's virtual layout.
///
/// This value does not own pages or keep the address space alive. The owner
/// must remain live while the stack is used. Its table is still inactive.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct UserStackLayout {
    guard_start: usize,
    bottom: usize,
    top: usize,
}

impl UserStackLayout {
    fn new(top: usize, size: usize) -> Result<Self, error::Error> {
        let page = PageEntry::PAGE_SIZE;
        if size == 0
            || !size.is_multiple_of(page)
            || !top.is_multiple_of(page)
            || top > (1usize << PageEntry::VIRT_ADDR_BITS)
        {
            return Err(error::code::EINVAL);
        }
        let bottom = top.checked_sub(size).ok_or(error::code::EINVAL)?;
        let guard_start = bottom.checked_sub(page).ok_or(error::code::EINVAL)?;
        Ok(Self {
            guard_start,
            bottom,
            top,
        })
    }
    pub fn guard_start(&self) -> usize {
        self.guard_start
    }
    pub fn bottom(&self) -> usize {
        self.bottom
    }
    pub fn top(&self) -> usize {
        self.top
    }
    pub fn size(&self) -> usize {
        self.top - self.bottom
    }
    pub fn initial_sp(&self) -> usize {
        self.top
    }
}

impl<F: FrameAllocator> AddressSpaceState<F> {
    /// Validate every destination page before a write. The caller holds the
    /// state lock across this check and the copy, so mappings cannot disappear.
    pub(super) fn validate_stack_write(
        &self,
        destination: usize,
        len: usize,
    ) -> Result<(), error::Error> {
        let layout = self.user_stack.ok_or(error::code::ENOENT)?;
        let end = destination.checked_add(len).ok_or(error::code::EINVAL)?;
        if destination < layout.bottom() || end > layout.top() {
            return Err(error::code::EINVAL);
        }
        let vma = self.vmas.get(&layout.bottom()).ok_or(error::code::EIO)?;
        if vma.end() != layout.top()
            || !vma
                .flags()
                .contains(MemFlags::READ | MemFlags::WRITE | MemFlags::USER)
        {
            return Err(error::code::EIO);
        }
        let mut cursor = destination;
        while cursor < end {
            self.mapper.translate(cursor).ok_or(error::code::EIO)?;
            cursor += (PageEntry::PAGE_SIZE - cursor % PageEntry::PAGE_SIZE).min(end - cursor);
        }
        Ok(())
    }

    pub(super) fn write_user_stack(
        &mut self,
        destination: usize,
        bytes: &[u8],
    ) -> Result<(), error::Error> {
        self.validate_stack_write(destination, bytes.len())?;
        let mut copied = 0;
        while copied < bytes.len() {
            let va = destination + copied;
            let len = (PageEntry::PAGE_SIZE - va % PageEntry::PAGE_SIZE).min(bytes.len() - copied);
            let physical = self.mapper.translate(va).expect("validated stack page");
            let target = kernel_phys_to_virt(physical).into_raw() as *mut u8;
            // SAFETY: validation restricts the complete destination to live,
            // exclusively owned stack backing. The state lock excludes other
            // walkers/writers; no user thread can access this inactive space.
            // The source slice is kernel memory and both chunks are in bounds.
            unsafe { core::ptr::copy(bytes.as_ptr().add(copied), target, len) };
            copied += len;
        }
        Ok(())
    }

    pub(super) fn manages_vma(&self, start: usize) -> bool {
        self.user_stack
            .is_some_and(|layout| start == layout.guard_start() || start == layout.bottom())
    }

    pub(super) fn new(allocator: F) -> Result<Self, error::Error> {
        // SAFETY: FrameAllocator guarantees disjoint, direct-map-writable
        // frames. PageEntry is configured for this kernel. Tables stay inactive.
        let mapper = unsafe { PageMapper::create(allocator) }.map_err(|_| error::code::ENOMEM)?;
        Ok(Self {
            vmas: BTreeMap::new(),
            mapper,
            user_stack: None,
        })
    }

    pub(super) fn init_user_stack(
        &mut self,
        top: usize,
        size: usize,
    ) -> Result<UserStackLayout, error::Error> {
        if self.user_stack.is_some() {
            return Err(error::code::EEXIST);
        }
        let layout = UserStackLayout::new(top, size)?;
        if AddressSpace::overlaps(&self.vmas, layout.guard_start(), layout.top()) {
            return Err(error::code::EEXIST);
        }
        let flags = MemFlags::READ | MemFlags::WRITE | MemFlags::USER;
        let guard = Vma::new(layout.guard_start(), layout.bottom(), MemFlags::empty())?;
        let stack = Vma::new(layout.bottom(), layout.top(), flags)?;
        self.vmas.insert(guard.start(), guard);
        self.vmas.insert(stack.start(), stack);
        // SAFETY: the caller serializes the whole state; this range was
        // exclusively reserved, has no users, and this table was never active.
        let mapped = unsafe {
            self.mapper.map_owned_range(
                VirtualAddress::new(layout.bottom()),
                layout.size(),
                MemAttribute {
                    flags,
                    memory_type: MemType::Normal(CachePolicy::WriteBack),
                },
                FrameInit::Zeroed,
                || {},
            )
        };
        if mapped.is_err() {
            self.vmas.remove(&guard.start());
            self.vmas.remove(&stack.start());
            return Err(error::code::ENOMEM);
        }
        self.user_stack = Some(layout);
        Ok(layout)
    }
}

impl<F: FrameAllocator> AddressSpaceState<F> {
    /// Find the lowest `align`-aligned, `len`-sized free hole in
    /// `[floor, ceil)`. `len` and `align` must be nonzero page multiples.
    /// Returns the hole start, or `None` if no such hole fits.
    pub(super) fn find_free_range(
        &self,
        floor: usize,
        ceil: usize,
        len: usize,
        align: usize,
    ) -> Option<usize> {
        if len == 0 || align == 0 || !align.is_power_of_two() {
            return None;
        }
        let mut cursor = floor.checked_add(align - 1)? & !(align - 1);
        while cursor <= ceil.checked_sub(len)? {
            let end = cursor.checked_add(len)?;
            // The first VMA whose start is >= cursor bounds this hole on the
            // right; any VMA overlapping [cursor, end) forces a skip past it.
            let conflict = self
                .vmas
                .range(cursor..end)
                .next()
                .map(|(start, vma)| (*start, vma.end()));
            match conflict {
                None => return Some(cursor),
                Some((vma_start, vma_end)) => {
                    // Resume scanning at the end of the conflicting VMA,
                    // re-aligned upward.
                    cursor = vma_end.max(vma_start.checked_add(1)?);
                    cursor = cursor.checked_add(align - 1)? & !(align - 1);
                }
            }
        }
        None
    }

    /// Back an exclusively reserved `[start, start+len)` image range with
    /// zeroed, user read/write frames and register its VMA. The table stays
    /// inactive. On failure no VMA or mapping is left behind.
    pub(super) fn load_image_range(
        &mut self,
        start: usize,
        len: usize,
        flags: MemFlags,
    ) -> Result<(), error::Error> {
        let end = start.checked_add(len).ok_or(error::code::EINVAL)?;
        if AddressSpace::overlaps(&self.vmas, start, end) {
            return Err(error::code::EEXIST);
        }
        let vma = Vma::new(start, end, flags)?;
        self.vmas.insert(start, vma);
        // SAFETY: the caller serialized this state, the range was just reserved
        // exclusively, has no users, and this table was never active.
        let mapped = unsafe {
            self.mapper.map_owned_range(
                VirtualAddress::new(start),
                len,
                MemAttribute {
                    flags,
                    memory_type: MemType::Normal(CachePolicy::WriteBack),
                },
                FrameInit::Zeroed,
                || {},
            )
        };
        if mapped.is_err() {
            self.vmas.remove(&start);
            return Err(error::code::ENOMEM);
        }
        Ok(())
    }

    /// Release a loaded image range: retire its mappings and drop its VMA.
    pub(super) fn release_image(&mut self, start: usize, len: usize) {
        // SAFETY: the caller exclusively owns this range and serializes the
        // state; tables have never been active.
        unsafe {
            self.mapper
                .release_owned_range(VirtualAddress::new(start), len, || {});
        }
        self.vmas.remove(&start);
    }

    /// Rewrite the access attributes of a loaded image range in place.
    /// `flags` with no READ/WRITE/EXECUTE revokes access (NONE) while retaining
    /// backing; otherwise the range becomes accessible with `flags`.
    ///
    /// The image is backed by one VMA spanning the whole allocation, while this
    /// call narrows per-segment sub-ranges. The VMA is a bounds reservation, not
    /// a per-page permission record: the page table is the live authority, and
    /// `translate` (used by read/write/zero) succeeds only while a leaf is valid.
    /// So the VMA's flags are left at the loader's initial READ|WRITE|USER and
    /// are not rewritten here — `validate_image_read`/`validate_image_write`
    /// additionally require a valid leaf on every page, which is the real gate.
    pub(super) fn protect_image(
        &mut self,
        start: usize,
        len: usize,
        flags: MemFlags,
    ) -> Result<(), error::Error> {
        // SAFETY: the caller serializes this state and exclusively owns the
        // range; the table is inactive.
        unsafe {
            self.mapper.reprotect_range(
                VirtualAddress::new(start),
                len,
                MemAttribute {
                    flags,
                    memory_type: MemType::Normal(CachePolicy::WriteBack),
                },
                || {},
            )
        }
        .map_err(|e| {
            log::warn!("reprotect_range({:#x}, {}, {:?}): {}", start, len, flags, e);
            error::code::EIO
        })
    }

    /// Read bytes from a loaded image range into a kernel buffer, checking the
    /// source VMA and every backing page before reading. Mirrors `write_image`.
    /// The page table is the live permission authority: a `translate` succeeds
    /// only while a leaf is valid, so a NONE-revoked range correctly rejects
    /// reads even though the backing PFN is retained.
    pub(super) fn read_image(
        &self,
        source: usize,
        bytes: &mut [u8],
    ) -> Result<(), error::Error> {
        self.validate_image_read(source, bytes.len())?;
        let mut copied = 0;
        while copied < bytes.len() {
            let va = source + copied;
            let len = (PageEntry::PAGE_SIZE - va % PageEntry::PAGE_SIZE).min(bytes.len() - copied);
            let physical = self.mapper.translate(va).expect("validated image page");
            let src = kernel_phys_to_virt(physical).into_raw() as *const u8;
            // SAFETY: validation restricts the complete source to live, valid
            // image backing. The state lock excludes other walkers; no user
            // thread can access this inactive space. Both slices are in bounds.
            unsafe { core::ptr::copy(src, bytes.as_mut_ptr().add(copied), len) };
            copied += len;
        }
        Ok(())
    }

    /// Copy kernel bytes into a loaded image range, checking the destination
    /// VMA and every backing page before writing. Mirrors `write_user_stack`.
    pub(super) fn write_image(
        &mut self,
        destination: usize,
        bytes: &[u8],
    ) -> Result<(), error::Error> {
        self.validate_image_write(destination, bytes.len())?;
        let mut copied = 0;
        while copied < bytes.len() {
            let va = destination + copied;
            let len = (PageEntry::PAGE_SIZE - va % PageEntry::PAGE_SIZE).min(bytes.len() - copied);
            let physical = self.mapper.translate(va).expect("validated image page");
            let target = kernel_phys_to_virt(physical).into_raw() as *mut u8;
            // SAFETY: validation restricts the complete destination to live,
            // exclusively owned image backing. The state lock excludes other
            // walkers/writers; no user thread can access this inactive space.
            // The source slice is kernel memory and both chunks are in bounds.
            unsafe { core::ptr::copy(bytes.as_ptr().add(copied), target, len) };
            copied += len;
        }
        Ok(())
    }

    /// Zero `len` bytes of a loaded image range starting at `destination`.
    pub(super) fn zero_image(
        &mut self,
        destination: usize,
        len: usize,
    ) -> Result<(), error::Error> {
        self.validate_image_write(destination, len)?;
        let mut zeroed = 0;
        while zeroed < len {
            let va = destination + zeroed;
            let chunk = (PageEntry::PAGE_SIZE - va % PageEntry::PAGE_SIZE).min(len - zeroed);
            let physical = self.mapper.translate(va).expect("validated image page");
            let target = kernel_phys_to_virt(physical).into_raw() as *mut u8;
            // SAFETY: same exclusivity and validation as write_image.
            unsafe { core::ptr::write_bytes(target, 0, chunk) };
            zeroed += chunk;
        }
        Ok(())
    }

    /// Validate every destination page before an image write. The caller holds
    /// the state lock across this check and the copy, so mappings cannot vanish.
    fn validate_image_write(
        &self,
        destination: usize,
        len: usize,
    ) -> Result<(), error::Error> {
        if len == 0 {
            return Ok(());
        }
        let end = destination.checked_add(len).ok_or(error::code::EINVAL)?;
        let vma = self
            .vmas
            .range(..=destination)
            .next_back()
            .map(|(_, vma)| *vma)
            .filter(|vma| vma.contains(destination) && vma.end() >= end)
            .ok_or(error::code::EIO)?;
        if !vma
            .flags()
            .contains(MemFlags::READ | MemFlags::WRITE | MemFlags::USER)
        {
            return Err(error::code::EIO);
        }
        let mut cursor = destination;
        while cursor < end {
            self.mapper.translate(cursor).ok_or(error::code::EIO)?;
            cursor += (PageEntry::PAGE_SIZE - cursor % PageEntry::PAGE_SIZE).min(end - cursor);
        }
        Ok(())
    }

    /// Validate every source page before an image read. The caller holds the
    /// state lock across this check and the copy, so mappings cannot vanish.
    /// The VMA must cover the range and be a user image mapping; the page table
    /// is the live permission authority, so a valid leaf (`translate` succeeds)
    /// is required on every page — a NONE-revoked page correctly fails here.
    fn validate_image_read(
        &self,
        source: usize,
        len: usize,
    ) -> Result<(), error::Error> {
        if len == 0 {
            return Ok(());
        }
        let end = source.checked_add(len).ok_or(error::code::EINVAL)?;
        let vma = self
            .vmas
            .range(..=source)
            .next_back()
            .map(|(_, vma)| *vma)
            .filter(|vma| vma.contains(source) && vma.end() >= end)
            .ok_or(error::code::EIO)?;
        if !vma.flags().contains(MemFlags::READ | MemFlags::USER) {
            return Err(error::code::EIO);
        }
        let mut cursor = source;
        while cursor < end {
            self.mapper.translate(cursor).ok_or(error::code::EIO)?;
            cursor += (PageEntry::PAGE_SIZE - cursor % PageEntry::PAGE_SIZE).min(end - cursor);
        }
        Ok(())
    }
}

impl<F: FrameAllocator> Drop for AddressSpaceState<F> {
    fn drop(&mut self) {
        if let Some(layout) = self.user_stack.take() {
            // SAFETY: state destruction has exclusive access, the recorded
            // interval owns all its backing, and tables have never been active.
            unsafe {
                self.mapper.release_owned_range(
                    VirtualAddress::new(layout.bottom()),
                    layout.size(),
                    || {},
                );
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::mm::{
        kernel_phys_to_virt,
        paging::first_fit::{tests::Fixture, FrameState},
    };
    use alloc::sync::Arc;
    use blueos_test_macro::test;
    const P: usize = 4096;
    const TOP: usize = 0x40000;

    #[test]
    fn user_stack_is_zeroed_guarded_and_owned() {
        let fixture = Fixture::new([FrameState::Free; 16]);
        let allocator = fixture.frame_allocator();
        let seed = allocator.alloc_owned_frames(16).unwrap();
        let ptr = kernel_phys_to_virt(seed.address()).into_raw() as *mut u8;
        // SAFETY: this guard exclusively owns all fixture backing, before mapping.
        unsafe { ptr.write_bytes(0xa5, 16 * P) };
        drop(seed);
        let mut state = AddressSpaceState::new(allocator).unwrap();
        assert!(state.user_stack.is_none());
        let layout = state.init_user_stack(TOP, 2 * P).unwrap();
        assert_eq!(
            (layout.guard_start(), layout.bottom(), layout.top()),
            (0x3d000, 0x3e000, 0x40000)
        );
        assert_eq!((layout.size(), layout.initial_sp()), (8192, 0x40000));
        assert_eq!(layout.initial_sp() % 16, 0);
        assert_eq!(state.vmas.len(), 2);
        assert_eq!(state.vmas[&0x3d000].flags(), MemFlags::empty());
        assert_eq!(
            state.vmas[&0x3e000].flags(),
            MemFlags::READ | MemFlags::WRITE | MemFlags::USER
        );
        assert!(state.mapper.translate(0x3d000).is_none());
        assert!(state.mapper.translate(0x3cfff).is_none());
        assert!(state.mapper.translate(0x40000).is_none());
        let first = state.mapper.translate(0x3e000).unwrap();
        assert_ne!(Some(first), state.mapper.translate(0x3f000));
        for va in [0x3e000, 0x3f000] {
            let physical = state.mapper.translate(va).unwrap();
            let ptr = kernel_phys_to_virt(physical).into_raw() as *const u8;
            // SAFETY: the inactive state owns these live mapped frames.
            assert!(unsafe { core::slice::from_raw_parts(ptr, P) }
                .iter()
                .all(|byte| *byte == 0));
            let entry = state.mapper.leaf_descriptor(va).unwrap();
            assert_eq!((entry >> 6) & 3, 1); // user read/write
            assert_ne!(entry & (1 << 11), 0); // non-global
            assert_ne!(entry & (1usize << 53), 0); // PXN
            assert_ne!(entry & (1usize << 54), 0); // UXN
        }
        assert_eq!(
            allocator.frame_stats().unwrap().allocated,
            PageEntry::LEVELS as usize + 2
        );
        drop(state);
        assert_eq!(allocator.frame_stats().unwrap().free, 16);
    }

    #[test]
    fn user_stack_rejects_invalid_layout_without_allocating() {
        let fixture = Fixture::new([FrameState::Free; 16]);
        let allocator = fixture.frame_allocator();
        let mut state = AddressSpaceState::new(allocator).unwrap();
        let before = allocator.frame_stats().unwrap();
        for size in [0, 1, P + 1, TOP, TOP + P, usize::MAX] {
            assert_eq!(state.init_user_stack(TOP, size), Err(error::code::EINVAL));
            assert!(state.vmas.is_empty());
            assert!(state.user_stack.is_none());
            assert_eq!(allocator.frame_stats().unwrap(), before);
        }
    }

    #[test]
    fn user_stack_partial_oom_rolls_back_and_allows_retry() {
        // Two backing pages plus LEVELS-1 child tables are needed beyond the root.
        let required = PageEntry::LEVELS as usize + 1;
        for budget in 0..required {
            let fixture = Fixture::new([FrameState::Free; 16]);
            let allocator = fixture.frame_allocator();
            let mut state = AddressSpaceState::new(allocator).unwrap();
            let original = Vma::new(0x1000, 0x2000, MemFlags::READ).unwrap();
            state.vmas.insert(original.start(), original);
            let held = allocator.alloc_owned_frames(15 - budget).unwrap();
            let before = allocator.frame_stats().unwrap();
            assert_eq!(state.init_user_stack(TOP, 2 * P), Err(error::code::ENOMEM));
            let after = allocator.frame_stats().unwrap();
            assert_eq!(
                (after.free, after.allocated),
                (before.free, before.allocated)
            );
            assert_eq!(state.vmas.len(), 1);
            assert_eq!(state.vmas[&0x1000], original);
            assert!(state.user_stack.is_none());
            assert!(state.mapper.translate(0x3e000).is_none());
            assert!(state.mapper.translate(0x3f000).is_none());
            drop(held);
            state.init_user_stack(TOP, 2 * P).unwrap();
            drop(state);
            assert_eq!(allocator.frame_stats().unwrap().free, 16);
        }
    }

    #[test]
    fn user_stack_drop_preserves_unrelated_leaf_backing() {
        let fixture = Fixture::new([FrameState::Free; 16]);
        let allocator = fixture.frame_allocator();
        let unrelated = allocator.alloc_zeroed_frame().unwrap();
        let mut state = AddressSpaceState::new(allocator).unwrap();
        state
            .mapper
            .map_range(
                VirtualAddress::new(0x2000),
                unrelated.address(),
                P,
                MemAttribute {
                    flags: MemFlags::READ | MemFlags::WRITE | MemFlags::USER,
                    memory_type: MemType::Normal(CachePolicy::WriteBack),
                },
            )
            .unwrap();
        state
            .vmas
            .insert(0x2000, Vma::new(0x2000, 0x3000, MemFlags::READ).unwrap());
        state.init_user_stack(TOP, 2 * P).unwrap();
        drop(state);
        assert_eq!(allocator.frame_stats().unwrap().free, 15);
        drop(unrelated);
        assert_eq!(allocator.frame_stats().unwrap().free, 16);
    }
    #[test]
    fn user_stack_public_layout_leaves_cpu_tables_unchanged() {
        use aarch64_cpu::registers::{TTBR0_EL1, TTBR1_EL1};
        use tock_registers::interfaces::Readable;
        let before = (TTBR0_EL1.get(), TTBR1_EL1.get());
        let space = AddressSpace::new().unwrap();
        assert!(space.user_stack().is_none());
        let end = space.user_end();
        let layout = space.init_user_stack(2 * P).unwrap();
        assert_eq!(layout.top(), end);
        assert_eq!(layout.bottom(), end - 8192);
        assert_eq!(layout.guard_start(), end - 12288);
        assert_eq!(layout.initial_sp(), end);
        assert_eq!(layout.initial_sp() % 16, 0);
        assert_eq!(space.user_stack(), Some(layout));
        assert_eq!(space.init_user_stack(P), Err(error::code::EEXIST));
        assert_eq!(space.user_stack(), Some(layout));
        assert_eq!((TTBR0_EL1.get(), TTBR1_EL1.get()), before);
    }

    #[test]
    fn user_stack_rejects_guard_stack_and_crossing_conflicts() {
        for offsets in [(3 * P, 2 * P), (2 * P, P), (2 * P + 1, 2 * P - 1)] {
            let space = AddressSpace::new().unwrap();
            let top = space.user_end();
            let original = space
                .reserve(top - offsets.0, top - offsets.1, MemFlags::READ)
                .unwrap();
            assert_eq!(space.init_user_stack(2 * P), Err(error::code::EEXIST));
            assert!(space.user_stack().is_none());
            assert_eq!(space.count(), 1);
            assert_eq!(space.find_containing(original.start()), Some(original));
        }
    }

    #[test]
    fn user_stack_managed_vmas_reject_metadata_mutation() {
        let space = AddressSpace::new().unwrap();
        let layout = space.init_user_stack(2 * P).unwrap();
        for start in [layout.guard_start(), layout.bottom()] {
            let original = space.find_containing(start).unwrap();
            assert!(space.remove(start).is_none());
            assert_eq!(
                space.extend_up(start, original.end()),
                Err(error::code::EBUSY)
            );
            assert_eq!(space.split_at(start, start + 1), Err(error::code::EBUSY));
            assert_eq!(space.merge_with_next(start), Err(error::code::EBUSY));
            assert_eq!(space.find_containing(start), Some(original));
            assert_eq!(
                space.reserve(start, start + 1, MemFlags::READ),
                Err(error::code::EEXIST)
            );
            assert_eq!(
                space.insert(Vma::new(start, start + 1, MemFlags::READ).unwrap()),
                Err(error::code::EEXIST)
            );
        }
        let neighbor = space
            .reserve(
                layout.guard_start() - P,
                layout.guard_start(),
                MemFlags::empty(),
            )
            .unwrap();
        assert_eq!(
            space.merge_with_next(neighbor.start()),
            Err(error::code::EBUSY)
        );
        assert_eq!(
            space.extend_up(neighbor.start(), layout.guard_start() + 1),
            Err(error::code::EINVAL)
        );
        assert_eq!(space.find_containing(neighbor.start()), Some(neighbor));
        assert_eq!(space.count(), 3);
        assert_eq!(space.extend_up(usize::MAX, 0), Err(error::code::ENOENT));
        assert_eq!(space.split_at(usize::MAX, 0), Err(error::code::ENOENT));
        assert_eq!(space.merge_with_next(usize::MAX), Err(error::code::ENOENT));
    }

    #[test]
    fn user_stack_concurrent_initialization_has_one_winner() {
        use crate::sync::{ConstBarrier, SpinLock};
        let space = Arc::new(AddressSpace::new().unwrap());
        let barrier = Arc::new(ConstBarrier::<2>::new());
        let result = Arc::new(SpinLock::new(None));
        let other_space = space.clone();
        let other_barrier = barrier.clone();
        let other_result = result.clone();
        let _worker = crate::thread::spawn(move || {
            other_barrier.wait();
            *other_result.irqsave_lock() = Some(other_space.init_user_stack(2 * P));
        })
        .unwrap();
        barrier.wait();
        let local = space.init_user_stack(2 * P);
        let deadline = crate::time::Tick::after(crate::time::Tick::from_millis(5000));
        let remote = loop {
            if let Some(value) = *result.irqsave_lock() {
                break value;
            }
            assert!(
                !deadline.is_elapsed(),
                "stack initialization worker timed out"
            );
            crate::scheduler::yield_me();
        };
        let winner = match (local, remote) {
            (Ok(layout), Err(err)) | (Err(err), Ok(layout)) => {
                assert_eq!(err, error::code::EEXIST);
                layout
            }
            pair => panic!("expected exactly one initialized stack: {:?}", pair),
        };
        assert_eq!(space.user_stack(), Some(winner));
        assert_eq!(space.count(), 2);
    }
}
