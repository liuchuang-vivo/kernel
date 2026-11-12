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

//! Loader entry points backed by the kernel's flat execution domain.

use super::adapters::{FlatImageAllocation, KernelCodeCache, MemoryMapper};
use blueos_loader::{
    load_elf, profile::LoadLimits, ElfReader, LoadRequest, LoadResult, LoadedImage,
};

/// Native loader profile selected by the build target.
#[cfg(target_arch = "riscv32")]
pub fn board_load_profile() -> blueos_loader::profile::LoadProfile {
    blueos_loader::profile::LoadProfile::riscv32()
}

#[cfg(target_arch = "riscv64")]
pub fn board_load_profile() -> blueos_loader::profile::LoadProfile {
    blueos_loader::profile::LoadProfile::riscv64()
}

#[cfg(target_arch = "aarch64")]
pub fn board_load_profile() -> blueos_loader::profile::LoadProfile {
    blueos_loader::profile::LoadProfile::aarch64()
}

/// Default request for a standalone executable using the native profile.
pub fn load_request() -> LoadRequest {
    LoadRequest::new(board_load_profile(), LoadLimits::DEFAULT)
}

/// Load and finalize one standalone image through a host-provided reader.
///
/// Relocatable (`ET_DYN`) images use the kernel heap. Fixed-address (`ET_EXEC`)
/// images use the shared platform table installed by `init_fixed_regions`.
/// The backend checks the ELF's requested range and claims a region before
/// changing bytes. Without an authorized region, fixed placement is rejected.
/// The returned image retains its mapping and entry/lifecycle addresses; it
/// does not execute them.
/// With VFS enabled, callers can pass `VfsElfReader::new(file)?` after opening
/// a regular file through `vfs::open_path()`. This also works on direct RTOS
/// threads without a Process or a file descriptor table.
///
/// Code publication is current-execution-context only. The caller must keep
/// the image's execution on this context until a host-wide rendezvous is used;
/// otherwise it must provide a different `CodeCache` backend.
pub fn load_image<R: ElfReader>(reader: R) -> LoadResult<LoadedImage<FlatImageAllocation>> {
    let mut allocator = MemoryMapper::new();
    let mut cache = KernelCodeCache::new();
    load_elf(reader, load_request(), &mut allocator, &mut cache)
}
