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

//! Reading a native ELF initial stack. This does not initialize libc or run
//! constructors; the CRT owns that sequence and must execute it exactly once.

use blueos_header::startup::{auxv, AuxvEntry, STACK_ALIGNMENT};
use core::{
    ffi::{c_char, c_int},
    mem::size_of,
    slice,
};

/// Borrowed argument addresses and auxiliary entries from an ELF startup frame.
/// Drop this view before allowing main/environ code to mutate the arrays.
#[must_use = "retain the startup backing while using this borrowed view"]
pub struct StartupInfo<'a> {
    argv: &'a [usize],
    envp: &'a [usize],
    auxv: &'a [usize],
}

impl<'a> StartupInfo<'a> {
    /// Validate a bounded, initialized frame without dereferencing its addresses.
    /// Extra words after the terminating auxv pair are ignored.
    pub fn from_words(words: &'a [usize]) -> Option<Self> {
        let argc = *words.first()?;
        if argc > c_int::MAX as usize {
            return None;
        }
        let argv_end = 1usize.checked_add(argc)?;
        let argv = words.get(1..argv_end)?;
        if argv.contains(&0) || *words.get(argv_end)? != 0 {
            return None;
        }
        let remaining = words.get(argv_end.checked_add(1)?..)?;
        let envc = remaining.iter().position(|&value| value == 0)?;
        let envp = &remaining[..envc];
        let remaining = remaining.get(envc.checked_add(1)?..)?;
        let pairs = remaining
            .chunks_exact(2)
            .position(|pair| pair[0] == auxv::AT_NULL)?
            + 1;
        Some(Self {
            argv,
            envp,
            auxv: &remaining[..pairs * 2],
        })
    }

    /// Read the terminated frame at the SP supplied by the ELF host.
    ///
    /// # Safety
    /// `sp` must describe a live, initialized native ELF frame: argc, its
    /// counted argv array and NULL, a terminated envp array, and complete auxv
    /// pairs through AT_NULL. The whole frame must lie within one allocation
    /// and stay immutable for `'a`. The pointed-to strings are not read here.
    /// These shape checks do not make arbitrary or unterminated memory safe.
    pub unsafe fn from_raw(sp: *const usize) -> Option<Self> {
        if sp.is_null() || !sp.addr().is_multiple_of(STACK_ALIGNMENT) {
            return None;
        }
        // SAFETY: the caller supplies the initialized argc slot.
        let argc = unsafe { sp.read() };
        if argc > c_int::MAX as usize {
            return None;
        }
        let mut index = argc.checked_add(1)?;
        // SAFETY: a valid frame includes argv's terminating slot.
        if unsafe { word_at(sp, index)? } != 0 {
            return None;
        }
        index = index.checked_add(1)?;
        // SAFETY: the caller supplies terminated environment and auxv arrays.
        unsafe {
            while word_at(sp, index)? != 0 {
                index = index.checked_add(1)?;
            }
            index = index.checked_add(1)?;
            loop {
                let tag = word_at(sp, index)?;
                word_at(sp, index.checked_add(1)?)?;
                index = index.checked_add(2)?;
                if tag == auxv::AT_NULL {
                    break;
                }
            }
            Self::from_words(slice::from_raw_parts(sp, index))
        }
    }

    pub fn argc(&self) -> c_int {
        self.argv.len() as c_int
    }
    pub fn argv(&self) -> &'a [usize] {
        self.argv
    }
    pub fn envp(&self) -> &'a [usize] {
        self.envp
    }
    pub fn argv_ptr(&self) -> *const *const c_char {
        self.argv.as_ptr().cast()
    }
    pub fn envp_ptr(&self) -> *const *const c_char {
        self.envp.as_ptr().cast()
    }
    pub fn auxv(&self) -> impl Iterator<Item = AuxvEntry> + '_ {
        self.auxv.chunks_exact(2).map(|pair| AuxvEntry {
            key: pair[0],
            value: pair[1],
        })
    }
    pub fn aux_value(&self, key: usize) -> Option<usize> {
        self.auxv()
            .find(|entry| entry.key == key)
            .map(|entry| entry.value)
    }
}

unsafe fn word_at(sp: *const usize, index: usize) -> Option<usize> {
    let bytes = index.checked_add(1)?.checked_mul(size_of::<usize>())?;
    if bytes > isize::MAX as usize {
        return None;
    }
    sp.addr().checked_add(bytes)?;
    // SAFETY: arithmetic is checked; allocation validity is the caller's contract.
    Some(unsafe { sp.add(index).read() })
}
