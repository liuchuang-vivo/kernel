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

//! Hardware-enforced image allocator backed by a user [`AddressSpace`].
//!
//! Implements the loader crate's [`ImageAllocator`] and [`ImageMapping`] traits
//! by mediating through [`AddressSpace`]'s pub surface, so the page-table
//! mapper stays private. One allocation backs the whole image span with zeroed,
//! user read/write frames; the loader writes file bytes and clears BSS through
//! [`ImageMapping::write`]/[`ImageMapping::zero`], reads back through
//! [`ImageMapping::read`] during relocation decode and seal verification, then
//! narrows per-segment access through [`ImageMapping::set_permissions`].
//! Dropping the allocation retires its mappings and releases its VMA. All work
//! happens on the inactive table before the address space is published to a
//! user thread.
//!
//! The page table is the live permission authority: a `translate` (and thus
//! `read`/`write`/`zero`) succeeds only while a leaf is valid, so a
//! NONE-revoked range correctly rejects access even though its backing PFN is
//! retained for a later non-NONE reprotect. The VMA is a bounds reservation,
//! not a duplicate per-page permission record.

#![cfg(paging)]

use alloc::sync::Arc;

use crate::mm::{address_space::AddressSpace, MemFlags};

use blueos_loader::{
    error::{ErrorContext, LoadErrorKind},
    memory::{
        AllocationOffset, ImageAllocator, ImageLayout, ImageMapping, MemoryPermissions,
        Placement, ProtectionLevel, TargetAddress,
    },
    LoadError, LoadResult,
};

/// Translate loader [`MemoryPermissions`] to kernel [`MemFlags`] for a user
/// mapping. Loader permissions are independent of the kernel bit layout, so the
/// conversion is direct; every image mapping is user-accessible and non-global.
/// NONE yields only `USER` (no READ/WRITE/EXECUTE): the reprotect then writes
/// the bare PFN with the valid bit clear, revoking access while retaining
/// backing for a later non-NONE restore.
fn to_mem_flags(permissions: MemoryPermissions) -> MemFlags {
    let mut flags = MemFlags::USER;
    if permissions.contains(MemoryPermissions::READ) {
        flags |= MemFlags::READ;
    }
    if permissions.contains(MemoryPermissions::WRITE) {
        flags |= MemFlags::WRITE;
    }
    if permissions.contains(MemoryPermissions::EXECUTE) {
        flags |= MemFlags::EXECUTE;
    }
    flags
}

/// Image allocator that loads segments into a user [`AddressSpace`].
///
/// Holds a shared reference to the address space; each allocation owns its VMA
/// and backing until it is dropped. The space must outlive every allocation
/// prepared from it. Prepare succeeds only while the table is still inactive.
#[must_use = "dropping the allocator does not release its allocations"]
pub struct AddressSpaceImageAllocator {
    aspace: Arc<AddressSpace>,
}

impl AddressSpaceImageAllocator {
    /// Wrap a user address space for image loading.
    pub fn new(aspace: Arc<AddressSpace>) -> Self {
        Self { aspace }
    }
}

impl ImageAllocator for AddressSpaceImageAllocator {
    type Allocation = AddressSpaceImageMapping;

    fn prepare_image(&mut self, layout: &ImageLayout) -> LoadResult<Self::Allocation> {
        let size = usize::try_from(layout.size()).map_err(|_| allocation_error(layout))?;
        let align = usize::try_from(layout.align()).map_err(|_| allocation_error(layout))?;
        let (base, start) = match layout.placement() {
            Placement::Anywhere => {
                let start = self
                    .aspace
                    .find_free_range(size, align)
                    .ok_or_else(|| allocation_error(layout))?;
                (TargetAddress::new(start as u64), start)
            }
            Placement::Fixed(range) => {
                let start =
                    usize::try_from(range.start().get()).map_err(|_| allocation_error(layout))?;
                (range.start(), start)
            }
        };
        // Initial access is loader read/write everywhere, so file bytes can be
        // copied, BSS cleared, and relocation tables read back before per-segment
        // permissions narrow it.
        let init_flags = MemFlags::READ | MemFlags::WRITE | MemFlags::USER;
        self.aspace
            .load_image_range(start, size, init_flags)
            .map_err(|_| allocation_error(layout))?;
        Ok(AddressSpaceImageMapping {
            aspace: self.aspace.clone(),
            base,
            len: layout.size(),
            align: layout.align(),
        })
    }
}

/// One image allocation inside a user [`AddressSpace`].
///
/// The allocation does not own the address space; the caller must keep it alive
/// while the mapping is in use. Offsets are relative to `base`; the whole span
/// `[base, base+len)` is backed by one VMA and one owned mapping.
#[must_use = "keep the mapping alive while its image is in use"]
pub struct AddressSpaceImageMapping {
    aspace: Arc<AddressSpace>,
    base: TargetAddress,
    len: u64,
    align: u64,
}

impl AddressSpaceImageMapping {
    fn raw_base(&self) -> Result<usize, LoadError> {
        usize::try_from(self.base.get())
            .map_err(|_| memory_access_error(self, AllocationOffset::new(0), 0))
    }

    /// Resolve `[base + offset, base + offset + len)` to a kernel VA, after
    /// bounds-checking the offset/length against the allocation.
    fn translate_offset(
        &self,
        offset: AllocationOffset,
        len: u64,
    ) -> Result<usize, LoadError> {
        let end = offset
            .value()
            .checked_add(len)
            .filter(|end| *end <= self.len)
            .ok_or_else(|| memory_access_error(self, offset, len))?;
        let base = self.raw_base()?;
        let start = usize::try_from(offset.value())
            .map_err(|_| memory_access_error(self, offset, len))?;
        let _end = usize::try_from(end).map_err(|_| memory_access_error(self, offset, len))?;
        base.checked_add(start).ok_or(memory_access_error(self, offset, len))
    }
}

impl ImageMapping for AddressSpaceImageMapping {
    fn base(&self) -> TargetAddress {
        self.base
    }

    fn len(&self) -> u64 {
        self.len
    }

    fn align(&self) -> u64 {
        self.align
    }

    fn read(&self, offset: AllocationOffset, dst: &mut [u8]) -> LoadResult<()> {
        let len =
            u64::try_from(dst.len()).map_err(|_| memory_access_error(self, offset, u64::MAX))?;
        let source = self.translate_offset(offset, len)?;
        self.aspace
            .read_image(source, dst)
            .map_err(|_| memory_access_error(self, offset, len))
    }

    fn write(&mut self, offset: AllocationOffset, data: &[u8]) -> LoadResult<()> {
        let len =
            u64::try_from(data.len()).map_err(|_| memory_access_error(self, offset, u64::MAX))?;
        let destination = self.translate_offset(offset, len)?;
        self.aspace
            .write_image(destination, data)
            .map_err(|_| memory_access_error(self, offset, len))
    }

    fn zero(&mut self, offset: AllocationOffset, len: u64) -> LoadResult<()> {
        let destination = self.translate_offset(offset, len)?;
        let count = usize::try_from(len).map_err(|_| memory_access_error(self, offset, len))?;
        self.aspace
            .zero_image(destination, count)
            .map_err(|_| memory_access_error(self, offset, len))
    }

    fn protection_level(&self) -> ProtectionLevel {
        ProtectionLevel::HardwareEnforced
    }

    fn set_permissions(
        &mut self,
        offset: AllocationOffset,
        len: u64,
        permissions: MemoryPermissions,
    ) -> LoadResult<()> {
        let start = self.translate_offset(offset, len)?;
        let count = usize::try_from(len).map_err(|_| memory_access_error(self, offset, len))?;
        self.aspace
            .protect_image(start, count, to_mem_flags(permissions))
            .map_err(|_| memory_access_error(self, offset, len))
    }
}

impl Drop for AddressSpaceImageMapping {
    fn drop(&mut self) {
        // Drop must not allocate or panic. Release failures are ignored: the
        // backing frames are leaked on a best-effort cleanup, matching the
        // trait contract that Drop releases resources without failing.
        if let Ok(start) = self.raw_base()
            && let Ok(len) = usize::try_from(self.len)
        {
            self.aspace.release_image(start, len);
        }
    }
}

fn allocation_error(layout: &ImageLayout) -> LoadError {
    LoadError::new(
        LoadErrorKind::OutOfMemory,
        ErrorContext::Allocation {
            base: TargetAddress::new(0),
            len: layout.size(),
            align: layout.align(),
        },
    )
}

fn memory_access_error(
    allocation: &AddressSpaceImageMapping,
    offset: AllocationOffset,
    len: u64,
) -> LoadError {
    LoadError::new(
        LoadErrorKind::Backend,
        ErrorContext::MemoryAccess {
            allocation_base: allocation.base,
            allocation_len: allocation.len,
            allocation_align: allocation.align,
            offset: offset.value(),
            len,
        },
    )
}
