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

//! File-backed [`ElfReader`] over an open VFS [`File`][crate::vfs::File].
//!
//! Used by `boot::init_user_app` to feed the loader pipeline directly from the
//! ELF image opened on the VFS, without staging a full intermediate kernel
//! buffer. The reader borrows the `File` by shared reference (`&self`
//! throughout) and issues positional reads via [`File::read_exact_at`], which
//! leave the shared file offset unchanged — the loader calls `read_exact_at`
//! many times per load, so a stable offset is required.
//!
//! With this adapter the ELF bytes flow file → loader's 512-byte scratch →
//! user pages, and no full-image heap buffer is needed. The kernel depends on
//! the loader crate one-way, so this adapter lives in the kernel, not in
//! `kernel/loader`.

#![cfg(all(paging, enable_vfs))]

use crate::vfs::File;
use blueos_loader::{
    error::{ErrorContext, LoadErrorKind},
    ElfReader, LoadError, LoadResult,
};

/// An [`ElfReader`] backed by a borrowed VFS [`File`][File].
///
/// The file must remain open and its length/contents stable for the lifetime
/// of every load attempt that uses the reader. Reads are positional and do not
/// touch the shared file offset, so a single `&File` is safe across the
/// repeated `read_exact_at` calls the loader issues.
pub struct VfsElfReader<'a> {
    file: &'a File,
}

impl<'a> VfsElfReader<'a> {
    /// Wrap an open file for positional ELF reads.
    #[inline]
    pub const fn new(file: &'a File) -> Self {
        Self { file }
    }

    /// Map a VFS [`Error`][crate::error::Error] into a loader [`LoadError`].
    ///
    /// `Error::to_errno()` is `0` for success and a negative libc errno
    /// otherwise; any failure during an ELF read is reported as
    /// [`LoadErrorKind::Io`], annotated with the byte range the loader asked
    /// for. The errno is captured into the log line so a real I/O fault is
    /// distinguishable from a bounds mismatch when diagnosing a failed load.
    #[inline]
    fn io_error(err: crate::error::Error, offset: u64, len: u64, file_len: u64) -> LoadError {
        let errno = err.to_errno();
        debug_assert!(errno != 0, "Error::to_errno == 0 must not reach io_error");
        log::debug!(
            "VfsElfReader: read failed at offset {:#x} len {:#x} (file_len {:#x}): errno {}",
            offset,
            len,
            file_len,
            errno,
        );
        LoadError::new(
            LoadErrorKind::Io,
            ErrorContext::FileRange { offset, len, file_len },
        )
    }
}

impl ElfReader for VfsElfReader<'_> {
    fn len(&self) -> LoadResult<u64> {
        self.file.file_len().map_err(|err| {
            // No range is in scope yet; report the whole-file ask.
            Self::io_error(err, 0, u64::MAX, 0)
        })
    }

    fn read_exact_at(&self, offset: u64, dst: &mut [u8]) -> LoadResult<()> {
        let file_len = self.len()?;
        let len = u64::try_from(dst.len()).map_err(|_| {
            LoadError::new(
                LoadErrorKind::IntegerOverflow,
                ErrorContext::FileRange {
                    offset,
                    len: u64::MAX,
                    file_len,
                },
            )
        })?;
        let end = offset.checked_add(len).ok_or_else(|| {
            LoadError::new(
                LoadErrorKind::IntegerOverflow,
                ErrorContext::FileRange { offset, len, file_len },
            )
        })?;
        if end > file_len {
            return Err(LoadError::new(
                LoadErrorKind::OutOfBounds,
                ErrorContext::FileRange { offset, len, file_len },
            ));
        }

        // `File::read_exact_at` is positional (offset-stable) and loops until
        // `dst` is filled or EOF (ENODATA). We have already bounds-checked
        // against `file_len`, so EOF inside the call is an I/O error rather
        // than a benign short read.
        self.file.read_exact_at(offset, dst).map_err(|err| {
            Self::io_error(err, offset, len, file_len)
        })
    }
}
