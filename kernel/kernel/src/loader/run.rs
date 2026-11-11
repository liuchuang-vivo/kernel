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

//! Run a standalone ELF from a VFS path in one call.
//!
//! This is the kernel-side convenience entry above the loader pipeline: it
//! opens the image through the VFS, loads it, builds the ELF startup frame,
//! schedules the application's main thread and waits for every member to
//! exit. The public stages (`load_image`, `StartupStorage`, `Application`)
//! remain available when a caller needs to observe or configure between them;
//! this helper covers the common "start this file now" flow without a
//! syscall. Applications run as bare RTOS thread groups; no Process or file
//! descriptor table is involved yet.

use super::{
    adapters::VfsElfReader, load_image, ApplicationError, ApplicationExit, StartupArguments,
    StartupError, StartupStorage,
};
use crate::{thread::Stack, vfs};
use alloc::{ffi::CString, string::String};
use blueos_loader::LoadError;
use core::ffi::CStr;

/// Default stack for the application's main thread, in bytes. Runtime
/// services a program spawns itself use their own stacks; this one carries
/// the startup frame and libc bootstrap.
const DEFAULT_STACK_SIZE: usize = 64 * 1024;
/// ELF ABI page granularity reported through AT_PAGESZ.
const ABI_PAGE_SIZE: usize = 4096;

/// One ELF launch from a VFS path.
pub struct RunRequest<'a> {
    pub argv: &'a [&'a CStr],
    pub envp: &'a [&'a CStr],
    pub stack_size: usize,
}

impl<'a> Default for RunRequest<'a> {
    fn default() -> Self {
        Self {
            argv: &[],
            envp: &[],
            stack_size: DEFAULT_STACK_SIZE,
        }
    }
}

impl<'a> core::fmt::Debug for RunRequest<'a> {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        f.debug_struct("RunRequest")
            .field("argv", &self.argv.len())
            .field("envp", &self.envp.len())
            .field("stack_size", &self.stack_size)
            .finish()
    }
}

#[derive(Debug)]
pub enum RunError {
    OpenImage(crate::error::Error),
    Load(LoadError),
    Startup(StartupError),
    StackUnavailable,
    Start(ApplicationError),
    /// The path is not representable as AT_EXECFN.
    ExecPath,
}

impl From<LoadError> for RunError {
    fn from(error: LoadError) -> Self {
        Self::Load(error)
    }
}
impl From<StartupError> for RunError {
    fn from(error: StartupError) -> Self {
        Self::Startup(error)
    }
}
impl From<ApplicationError> for RunError {
    fn from(error: ApplicationError) -> Self {
        Self::Start(error)
    }
}

/// Load `path` through the VFS and run it to completion on this thread's
/// scheduler, returning its exit status. Blocks until every member thread of
/// the application has finished cleanup; the image is then released. Code
/// publication is current-context only, as documented on `load_image`;
/// callers on platforms without coherent instruction fetches must keep
/// execution on the loading context.
///
/// # Safety
/// The file at `path` must be trusted native code using the standard ELF
/// initial-stack entry convention. Its users must remain application members;
/// see `StartupStorage::into_application`.
pub unsafe fn run(path: &str, request: &RunRequest<'_>) -> Result<ApplicationExit, RunError> {
    let file = vfs::open_path(path, libc::O_RDONLY, 0).map_err(RunError::OpenImage)?;
    let image = load_image(VfsElfReader::new(file)?)?;
    let stack = Stack::from_size(request.stack_size).ok_or(RunError::StackUnavailable)?;
    let execfn = CString::new(path).map_err(|_| RunError::ExecPath)?;
    let storage = StartupStorage::new(
        image,
        stack,
        StartupArguments {
            argv: request.argv,
            envp: request.envp,
            page_size: ABI_PAGE_SIZE,
            execfn: Some(&execfn),
            random: None,
        },
    )?;
    // SAFETY: the caller guarantees trusted native code; the group retains
    // the image and stack through all member cleanups.
    let mut application = unsafe { storage.into_application()? };
    application.start(None)?;
    application.wait().map_err(Into::into)
}
