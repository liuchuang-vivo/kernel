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

//! Native ELF process-entry definitions shared by the host and CRT.
//!
//! At the initial SP: argc, argv pointers, NULL, envp pointers, NULL, then
//! auxiliary key/value pairs ending in AT_NULL. Each slot is one native word.
//! There is no BlueOS-specific version prefix or lifecycle-plan argument.

/// Initial SP alignment for the supported RISC-V and AArch64 ELF profiles.
pub const STACK_ALIGNMENT: usize = 16;

#[repr(C)]
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct AuxvEntry {
    pub key: usize,
    pub value: usize,
}

pub mod auxv {
    pub const AT_NULL: usize = 0;
    pub const AT_PHDR: usize = 3;
    pub const AT_PHENT: usize = 4;
    pub const AT_PHNUM: usize = 5;
    pub const AT_PAGESZ: usize = 6;
    pub const AT_BASE: usize = 7;
    pub const AT_ENTRY: usize = 9;
    pub const AT_RANDOM: usize = 25;
    pub const AT_EXECFN: usize = 31;
}

#[cfg(target_pointer_width = "32")]
const _: () = {
    assert!(core::mem::size_of::<AuxvEntry>() == 8);
    assert!(core::mem::align_of::<AuxvEntry>() == 4);
    assert!(core::mem::offset_of!(AuxvEntry, value) == 4);
};
#[cfg(target_pointer_width = "64")]
const _: () = {
    assert!(core::mem::size_of::<AuxvEntry>() == 16);
    assert!(core::mem::align_of::<AuxvEntry>() == 8);
    assert!(core::mem::offset_of!(AuxvEntry, value) == 8);
};
