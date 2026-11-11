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
//
// This code is based on relibc (original license follows):
// https://github.com/redox-os/relibc/blob/master/LICENSE
// standard MIT license

use super::{constants, helpers::File, Buffer, BUFSIZ, FILE};
use crate::{
    io::LineWriter,
    sync::{once::Once, GenericMutex},
};
use alloc::{boxed::Box, vec::Vec};
use core::{cell::UnsafeCell, ptr};
use libc::c_int;

#[must_use = "retain the standard stream until its final flush"]
pub struct GlobalFile(UnsafeCell<Option<FILE>>);

impl GlobalFile {
    fn new(file: c_int, flags: c_int) -> Self {
        let file = File::new(file);
        let writer = Box::new(LineWriter::new(unsafe { file.get_ref() }));
        GlobalFile(UnsafeCell::new(Some(FILE {
            lock: GenericMutex::new(()),
            file,
            flags: constants::F_PERM | flags,
            read_buf: Buffer::Owned(vec![0; BUFSIZ as usize]),
            read_pos: 0,
            read_size: 0,
            unget: Vec::new(),
            writer,
            orientation: 0,
        })))
    }
    pub fn get(&self) -> *mut FILE {
        // SAFETY: init obtains pointers before publishing the streams; terminal
        // cleanup runs only after their final user has finished.
        unsafe {
            (*self.0.get())
                .as_mut()
                .expect("standard stream already released")
        }
    }

    unsafe fn finish(&self) {
        // SAFETY: the standalone runtime excludes all stream users at exit.
        if let Some(mut stream) = unsafe { (*self.0.get()).take() } {
            // The RTOS application borrows inherited descriptors. Release libc
            // buffers without closing another owner's descriptor table entries.
            stream.file.reference = true;
            drop(stream);
        }
    }
}
// statics need to be Sync
unsafe impl Sync for GlobalFile {}

static DEFAULT_STDIN: Once<GlobalFile> = Once::new();
static DEFAULT_STDOUT: Once<GlobalFile> = Once::new();
static DEFAULT_STDERR: Once<GlobalFile> = Once::new();

pub fn default_stdin() -> &'static GlobalFile {
    DEFAULT_STDIN.call_once(|| GlobalFile::new(0, constants::F_NOWR))
}
pub fn default_stdout() -> &'static GlobalFile {
    DEFAULT_STDOUT.call_once(|| GlobalFile::new(1, constants::F_NORD))
}
pub fn default_stderr() -> &'static GlobalFile {
    DEFAULT_STDERR.call_once(|| GlobalFile::new(2, constants::F_NORD))
}

#[unsafe(no_mangle)]
pub static mut stdin: *mut FILE = ptr::null_mut();
#[unsafe(no_mangle)]
pub static mut stdout: *mut FILE = ptr::null_mut();
#[unsafe(no_mangle)]
pub static mut stderr: *mut FILE = ptr::null_mut();

/// Flush and release this image's standard stream buffers once.
///
/// # Safety
/// No code may access these streams again, including through default_*().
/// The caller must exclude all concurrent users and finish TLS destructors first.
pub(crate) unsafe fn finish() {
    // SAFETY: runtime initialized the streams and has finished all callbacks.
    unsafe {
        super::fflush(ptr::null_mut());
        stdin = ptr::null_mut();
        stdout = ptr::null_mut();
        stderr = ptr::null_mut();
        default_stdin().finish();
        default_stdout().finish();
        default_stderr().finish();
    }
}
