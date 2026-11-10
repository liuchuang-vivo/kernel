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

//! Positional ELF reads from an owned VFS file, without a descriptor table.

use crate::{
    error::{code, Error},
    vfs::File,
};
use blueos_loader::{
    error::{ErrorContext, LoadErrorKind},
    ElfReader, LoadError, LoadResult,
};

/// An opened regular file used as the source of one image load.
///
/// This reader retains the file object and its initial length, and reads
/// directly into the loader's buffers without moving the file cursor. It
/// neither copies the complete file nor allocates an integer descriptor.
///
/// The caller must keep the file contents and length unchanged throughout the
/// load attempt, as required by `ElfReader`. Owning the file retains its inode
/// even if its path is removed, but does not prevent other handles from writing
/// or truncating it. This adapter does not provide a filesystem snapshot.
#[must_use = "retain the open ELF file until its positional reads finish"]
pub struct VfsElfReader {
    file: File,
    len: u64,
}

impl VfsElfReader {
    /// Take ownership of an already opened, readable regular file.
    pub fn new(file: File) -> LoadResult<Self> {
        if !file.type_().is_regular_file() || !file.access_mode().is_readable() {
            return Err(LoadError::new(LoadErrorKind::Io, ErrorContext::None));
        }
        let len = file
            .file_len()
            .map_err(|error| file_error(error, 0, 0, 0))?;
        Ok(Self { file, len })
    }
}

impl ElfReader for VfsElfReader {
    fn len(&self) -> LoadResult<u64> {
        Ok(self.len)
    }

    fn read_exact_at(&self, offset: u64, dst: &mut [u8]) -> LoadResult<()> {
        let len = u64::try_from(dst.len())
            .map_err(|_| file_error(code::EOVERFLOW, offset, u64::MAX, self.len))?;
        let end = offset
            .checked_add(len)
            .ok_or_else(|| file_error(code::EOVERFLOW, offset, len, self.len))?;
        if end > self.len {
            return Err(file_error(code::ENODATA, offset, len, self.len));
        }
        self.file
            .read_exact_at(offset, dst)
            .map_err(|error| file_error(error, offset, len, self.len))
    }
}

fn file_error(error: Error, offset: u64, len: u64, file_len: u64) -> LoadError {
    let kind = if error == code::ENOMEM {
        LoadErrorKind::OutOfMemory
    } else if error == code::EOVERFLOW {
        LoadErrorKind::IntegerOverflow
    } else if error == code::ENODATA {
        LoadErrorKind::OutOfBounds
    } else {
        LoadErrorKind::Io
    };
    LoadError::new(
        kind,
        ErrorContext::FileRange {
            offset,
            len,
            file_len,
        },
    )
}
