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

//! Run the production Rust shell through the kernel's one-call loader entry.
//! The test stages the shell ELF in the target VFS, then `loader::run`
//! performs the complete flow: open, load, startup frame, thread scheduling
//! and exit. No Process or descriptor table is created; the application runs
//! as a bare RTOS thread group.

use blueos::{
    loader::{ApplicationExit, RunRequest},
    vfs::{self, FileOps},
};
use blueos_test_macro::test;
use core::ffi::{c_char, CStr};

unsafe extern "C" {
    static SHELL_TEST_ELF_PATH: *const c_char;
}

const IMAGE_PATH: &str = "/shell.elf";

/// Copy a host fixture into the target VFS so the run entry opens it by path.
fn stage_image(path: &str, host: &CStr) {
    use semihosting::io::Read;
    let mut source = semihosting::fs::File::open(host).expect("open host shell ELF");
    let file = vfs::open_path(path, libc::O_CREAT | libc::O_TRUNC | libc::O_WRONLY, 0o600)
        .expect("create target shell ELF");
    let len = usize::try_from(source.metadata().unwrap().len()).unwrap();
    file.resize(len).expect("size target shell ELF");
    let mut copied = 0;
    let mut buffer = [0; 512];
    loop {
        let count = source.read(&mut buffer).unwrap();
        if count == 0 {
            break;
        }
        let mut written = 0;
        while written < count {
            let size = file.write(&buffer[written..count]).unwrap();
            assert!(size != 0);
            written += size;
        }
        copied += count;
    }
    assert_eq!(copied, len);
}

#[test]
fn test_run_loads_shell_as_bare_thread_group() {
    // SAFETY: the linker provides this live host fixture path.
    let host = unsafe { CStr::from_ptr(SHELL_TEST_ELF_PATH) };
    stage_image(IMAGE_PATH, host);

    let request = RunRequest {
        argv: &[c"shell"],
        ..Default::default()
    };
    // SAFETY: the staged shell is trusted native code using the standard ELF
    // initial-stack entry; QEMU's coherent test execution publishes its code.
    let status = unsafe { blueos::loader::run(IMAGE_PATH, &request) }.unwrap();
    // The shell's main returns after its service thread exits; libc fini runs.
    assert_eq!(status, ApplicationExit::Code(0));

    assert_eq!(vfs::syscalls::unlink(c"/shell.elf".as_ptr()), 0);
}
