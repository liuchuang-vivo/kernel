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

//! Startup data for an eagerly backed, inactive initial user stack.

use super::{AddressSpaceState, UserStackLayout};
use crate::{error, mm::paging::FrameAllocator};

const WORD: usize = core::mem::size_of::<usize>();
const STACK_ALIGNMENT: usize = 16;

/// User virtual addresses of a prepared native-word startup stack.
///
/// This snapshot does not own memory or retain the address space. Its owner
/// must keep the stack alive and unchanged until the user thread consumes it.
/// Auxiliary-vector values are supplied by the loader, not validated here.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct UserStackStart {
    sp: usize,
    argc: usize,
    argv: usize,
    envp: usize,
    auxv: usize,
}

impl UserStackStart {
    /// Final, 16-byte-aligned user SP. The word at this address is `argc`.
    pub fn sp(&self) -> usize {
        self.sp
    }
    pub fn argc(&self) -> usize {
        self.argc
    }
    pub fn argv(&self) -> usize {
        self.argv
    }
    pub fn envp(&self) -> usize {
        self.envp
    }
    pub fn auxv(&self) -> usize {
        self.auxv
    }
}

/// A checked layout; construction does not access or modify backing memory.
struct StartupPlan {
    start: UserStackStart,
    strings_start: usize,
    frame_end: usize,
}

impl StartupPlan {
    fn new(
        layout: UserStackLayout,
        argv: &[&[u8]],
        envp: &[&[u8]],
        auxv: &[(usize, usize)],
    ) -> Result<Self, error::Error> {
        let mut strings_start = layout.top();
        for string in argv.iter().chain(envp) {
            if string.contains(&0) {
                return Err(error::code::EINVAL);
            }
            strings_start = string
                .len()
                .checked_add(1)
                .and_then(|len| strings_start.checked_sub(len))
                .filter(|start| *start >= layout.bottom())
                .ok_or(error::code::ENOSPC)?;
        }
        if auxv.iter().any(|(key, _)| *key == 0) {
            return Err(error::code::EINVAL);
        }
        // argc + argv NULL + envp NULL + the terminating auxv pair.
        let frame_size = auxv
            .len()
            .checked_mul(2)
            .and_then(|words| words.checked_add(argv.len()))
            .and_then(|words| words.checked_add(envp.len()))
            .and_then(|words| words.checked_add(5))
            .and_then(|words| words.checked_mul(WORD))
            .ok_or(error::code::ENOSPC)?;
        let sp = strings_start
            .checked_sub(frame_size)
            .map(|sp| sp & !(STACK_ALIGNMENT - 1))
            .filter(|sp| *sp >= layout.bottom())
            .ok_or(error::code::ENOSPC)?;
        let argv_address = sp + WORD;
        let envp_address = argv_address + (argv.len() + 1) * WORD;
        let auxv_address = envp_address + (envp.len() + 1) * WORD;
        Ok(Self {
            start: UserStackStart {
                sp,
                argc: argv.len(),
                argv: argv_address,
                envp: envp_address,
                auxv: auxv_address,
            },
            strings_start,
            frame_end: sp + frame_size,
        })
    }
}

impl<F: FrameAllocator> AddressSpaceState<F> {
    pub(super) fn build_user_stack(
        &mut self,
        argv: &[&[u8]],
        envp: &[&[u8]],
        auxv: &[(usize, usize)],
    ) -> Result<UserStackStart, error::Error> {
        let layout = self.user_stack.ok_or(error::code::ENOENT)?;
        let plan = StartupPlan::new(layout, argv, envp, auxv)?;
        // Check the whole image before modifying a single byte. Under the
        // caller's state lock, all subsequent writes are guaranteed to fit
        // live pages. This path needs no temporary heap buffer or user mapping.
        self.validate_stack_write(plan.start.sp, layout.top() - plan.start.sp)?;

        self.write_stack_word(plan.start.sp, argv.len())?;
        let mut strings = layout.top();
        for (items, mut pointers) in [(argv, plan.start.argv), (envp, plan.start.envp)] {
            for string in items {
                strings -= string.len() + 1;
                self.write_user_stack(strings, string)?;
                self.write_user_stack(strings + string.len(), &[0])?;
                self.write_stack_word(pointers, strings)?;
                pointers += WORD;
            }
            self.write_stack_word(pointers, 0)?;
        }
        let mut auxiliary = plan.start.auxv;
        for &(key, value) in auxv {
            self.write_stack_word(auxiliary, key)?;
            self.write_stack_word(auxiliary + WORD, value)?;
            auxiliary += 2 * WORD;
        }
        self.write_stack_word(auxiliary, 0)?;
        self.write_stack_word(auxiliary + WORD, 0)?;
        let padding = plan.strings_start - plan.frame_end;
        self.write_user_stack(plan.frame_end, &[0; STACK_ALIGNMENT][..padding])?;
        Ok(plan.start)
    }

    fn write_stack_word(&mut self, destination: usize, word: usize) -> Result<(), error::Error> {
        self.write_user_stack(destination, &word.to_ne_bytes())
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::mm::{
        address_space::AddressSpace,
        kernel_phys_to_virt,
        paging::{
            first_fit::{tests::Fixture, FrameState},
            PageFormat,
        },
        VirtualAddress,
    };
    use alloc::{vec, vec::Vec};
    use blueos_test_macro::test;

    const P: usize = 4096;
    const TOP: usize = 0x40000;

    fn read_bytes<F: FrameAllocator>(
        state: &AddressSpaceState<F>,
        mut address: usize,
        mut len: usize,
    ) -> Vec<u8> {
        let mut result = Vec::with_capacity(len);
        while len != 0 {
            let chunk = (P - address % P).min(len);
            let physical = state.mapper.translate(address).unwrap();
            let pointer = kernel_phys_to_virt(physical).into_raw() as *const u8;
            // SAFETY: tests hold the inactive state with exclusive ownership
            // of live backing; the chunk stays inside this translated page.
            result.extend_from_slice(unsafe { core::slice::from_raw_parts(pointer, chunk) });
            address += chunk;
            len -= chunk;
        }
        result
    }

    fn read_word<F: FrameAllocator>(state: &AddressSpaceState<F>, address: usize) -> usize {
        usize::from_ne_bytes(read_bytes(state, address, WORD).try_into().unwrap())
    }

    #[test]
    fn user_stack_write_crosses_fragmented_frames_and_checks_bounds() {
        let mut frames = [FrameState::Reserved; 16];
        for index in (0..16).step_by(2) {
            frames[index] = FrameState::Free;
        }
        let fixture = Fixture::new(frames);
        let allocator = fixture.frame_allocator();
        let mut state = AddressSpaceState::new(allocator).unwrap();
        let layout = state.init_user_stack(TOP, 2 * P).unwrap();
        let first = state.mapper.translate(layout.bottom()).unwrap().into_raw();
        let second = state
            .mapper
            .translate(layout.bottom() + P)
            .unwrap()
            .into_raw();
        assert_ne!(first + P, second);
        let before = allocator.frame_stats().unwrap();
        let destination = layout.bottom() + P - 3;
        state.write_user_stack(destination, b"cross-page").unwrap();
        assert_eq!(read_bytes(&state, destination - 1, 12), b"\0cross-page\0");
        state.write_user_stack(layout.bottom(), b"B").unwrap();
        state.write_user_stack(layout.top() - 1, b"T").unwrap();
        state.write_user_stack(layout.top(), b"").unwrap();
        let original = read_bytes(&state, layout.bottom(), layout.size());
        for (address, bytes) in [
            (layout.guard_start(), &b"x"[..]),
            (layout.bottom() - 1, &b"xx"[..]),
            (layout.top() - 1, &b"xx"[..]),
            (layout.top(), &b"x"[..]),
            (layout.top() + 1, &b""[..]),
            (usize::MAX, &b"xx"[..]),
        ] {
            assert_eq!(
                state.write_user_stack(address, bytes),
                Err(error::code::EINVAL)
            );
        }
        assert_eq!(read_bytes(&state, layout.bottom(), layout.size()), original);
        assert!(state.mapper.translate(layout.guard_start()).is_none());
        assert_eq!(allocator.frame_stats().unwrap(), before);
        drop(state);
        assert_eq!(allocator.frame_stats().unwrap().free, 8);
    }

    #[test]
    fn user_stack_startup_requires_an_initialized_stack() {
        let fixture = Fixture::new([FrameState::Free; 16]);
        let allocator = fixture.frame_allocator();
        let mut state = AddressSpaceState::new(allocator).unwrap();
        let before = allocator.frame_stats().unwrap();
        assert_eq!(state.write_user_stack(TOP, b""), Err(error::code::ENOENT));
        assert_eq!(
            state.build_user_stack(&[], &[], &[]),
            Err(error::code::ENOENT)
        );
        assert!(state.vmas.is_empty());
        assert_eq!(allocator.frame_stats().unwrap(), before);
    }

    #[test]
    fn user_stack_startup_rejects_missing_backing_before_writing() {
        let fixture = Fixture::new([FrameState::Free; 16]);
        let mut state = AddressSpaceState::new(fixture.frame_allocator()).unwrap();
        let layout = state.init_user_stack(TOP, 2 * P).unwrap();
        state.write_user_stack(TOP - P - 2, b"AB").unwrap();
        // SAFETY: the fixture exclusively owns this never-active space. Remove
        // one page to exercise preflight failure; drop skips the missing leaf.
        unsafe {
            state
                .mapper
                .release_owned_range(VirtualAddress::new(TOP - P), P, || {})
        };
        assert_eq!(
            state.write_user_stack(TOP - P - 2, b"xxxx"),
            Err(error::code::EIO)
        );
        assert_eq!(read_bytes(&state, TOP - P - 2, 2), b"AB");
        let original = read_bytes(&state, layout.bottom(), P);
        let long = vec![b'x'; P];
        assert_eq!(
            state.build_user_stack(&[&long], &[], &[]),
            Err(error::code::EIO)
        );
        assert_eq!(read_bytes(&state, layout.bottom(), P), original);
        drop(state);
        assert_eq!(fixture.frame_allocator().frame_stats().unwrap().free, 16);
    }

    #[test]
    fn user_stack_startup_encodes_arguments_environment_and_auxiliary_vector() {
        let fixture = Fixture::new([FrameState::Free; 16]);
        let allocator = fixture.frame_allocator();
        let mut state = AddressSpaceState::new(allocator).unwrap();
        let layout = state.init_user_stack(TOP, 3 * P).unwrap();
        let long = vec![b'x'; P + 31];
        let before = allocator.frame_stats().unwrap();
        let start = state
            .build_user_stack(
                &[b"/bin/init", &long, b""],
                &[b"MODE=test", b"RAW=\xff"],
                &[(6, P), (9, 0x10000)],
            )
            .unwrap();
        assert_eq!(start.sp() % 16, 0);
        assert!(start.sp() >= layout.bottom());
        assert_eq!(start.argc(), 3);
        assert_eq!(read_word(&state, start.sp()), 3);
        assert_eq!(start.argv(), start.sp() + WORD);
        for (index, string) in [&b"/bin/init"[..], &long, &b""[..]].iter().enumerate() {
            let pointer = read_word(&state, start.argv() + index * WORD);
            assert!(pointer >= start.sp() && pointer < TOP);
            assert_eq!(read_bytes(&state, pointer, string.len()), *string);
            assert_eq!(read_bytes(&state, pointer + string.len(), 1), [0]);
        }
        assert_eq!(read_word(&state, start.argv() + 3 * WORD), 0);
        assert_eq!(start.envp(), start.argv() + 4 * WORD);
        for (index, string) in [&b"MODE=test"[..], &b"RAW=\xff"[..]].iter().enumerate() {
            let pointer = read_word(&state, start.envp() + index * WORD);
            assert_eq!(read_bytes(&state, pointer, string.len()), *string);
            assert_eq!(read_bytes(&state, pointer + string.len(), 1), [0]);
        }
        assert_eq!(read_word(&state, start.envp() + 2 * WORD), 0);
        assert_eq!(start.auxv(), start.envp() + 3 * WORD);
        for (index, word) in [6, P, 9, 0x10000, 0, 0].into_iter().enumerate() {
            assert_eq!(read_word(&state, start.auxv() + index * WORD), word);
        }
        let lowest_string = read_word(&state, start.envp() + WORD);
        let frame_end = start.auxv() + 6 * WORD;
        assert!(lowest_string - frame_end < 16);
        assert!(read_bytes(&state, frame_end, lowest_string - frame_end)
            .iter()
            .all(|b| *b == 0));
        assert!(state.mapper.translate(layout.guard_start()).is_none());
        assert_eq!(allocator.frame_stats().unwrap(), before);
    }

    #[test]
    fn user_stack_startup_empty_tables_and_rebuild_clear_padding() {
        let fixture = Fixture::new([FrameState::Free; 16]);
        let mut state = AddressSpaceState::new(fixture.frame_allocator()).unwrap();
        state.init_user_stack(TOP, P).unwrap();
        state.write_user_stack(TOP - 64, &[0xa5; 64]).unwrap();
        let start = state.build_user_stack(&[], &[], &[]).unwrap();
        assert_eq!(start.argc(), 0);
        assert_eq!(start.sp(), TOP - 48);
        assert_eq!(read_bytes(&state, start.sp(), 48), vec![0; 48]);
        let second = state.build_user_stack(&[b"init"], &[], &[]).unwrap();
        assert_eq!(second.argc(), 1);
        assert_eq!(read_word(&state, second.sp()), 1);
        let pointer = read_word(&state, second.argv());
        assert_eq!(read_bytes(&state, pointer, 5), b"init\0");
        assert_eq!(second.sp() % 16, 0);
    }

    #[test]
    fn user_stack_startup_invalid_inputs_preserve_previous_image() {
        let fixture = Fixture::new([FrameState::Free; 16]);
        let mut state = AddressSpaceState::new(fixture.frame_allocator()).unwrap();
        let layout = state.init_user_stack(TOP, P).unwrap();
        state
            .build_user_stack(&[b"init"], &[b"A=B"], &[(6, P)])
            .unwrap();
        let original = read_bytes(&state, layout.bottom(), P);
        assert_eq!(
            state.build_user_stack(&[b"a\0b"], &[], &[]),
            Err(error::code::EINVAL)
        );
        assert_eq!(
            state.build_user_stack(&[], &[b"A=\0"], &[]),
            Err(error::code::EINVAL)
        );
        assert_eq!(
            state.build_user_stack(&[], &[], &[(0, 1)]),
            Err(error::code::EINVAL)
        );
        assert_eq!(read_bytes(&state, layout.bottom(), P), original);
        assert_eq!(state.vmas.len(), 2);
    }

    #[test]
    fn user_stack_startup_exact_capacity_and_overflow_preserve_guard() {
        let fixture = Fixture::new([FrameState::Free; 16]);
        let mut state = AddressSpaceState::new(fixture.frame_allocator()).unwrap();
        let layout = state.init_user_stack(TOP, P).unwrap();
        // One argv entry uses six words, plus the string's terminating NUL.
        let mut argument = vec![b'x'; P - 6 * WORD - 1];
        let start = state.build_user_stack(&[&argument], &[], &[]).unwrap();
        assert_eq!(start.sp(), layout.bottom());
        let original = read_bytes(&state, layout.bottom(), P);
        argument.push(b'x');
        assert_eq!(
            state.build_user_stack(&[&argument], &[], &[]),
            Err(error::code::ENOSPC)
        );
        let oversized = vec![b'x'; P + 1];
        assert_eq!(
            state.build_user_stack(&[&oversized], &[], &[]),
            Err(error::code::ENOSPC)
        );
        assert_eq!(read_bytes(&state, layout.bottom(), P), original);
        assert!(state.mapper.translate(layout.guard_start()).is_none());
    }

    #[test]
    fn user_stack_startup_public_api_keeps_cpu_tables_inactive() {
        use aarch64_cpu::registers::{TTBR0_EL1, TTBR1_EL1};
        use tock_registers::interfaces::Readable;
        let before = (TTBR0_EL1.get(), TTBR1_EL1.get());
        let space = AddressSpace::new().unwrap();
        let layout = space.init_user_stack(2 * P).unwrap();
        space.write_user_stack(layout.top() - 1, b"x").unwrap();
        let start = space
            .build_user_stack(&[b"init"], &[b"A=B"], &[(6, P)])
            .unwrap();
        assert_eq!(start.argc(), 1);
        assert_eq!(start.sp() % 16, 0);
        assert_eq!(read_word(&space.state.irqsave_lock(), start.sp()), 1);
        assert_eq!(space.user_stack(), Some(layout));
        assert_eq!((TTBR0_EL1.get(), TTBR1_EL1.get()), before);
        assert_eq!(crate::arch::page::PageEntry::PAGE_SIZE, P);
    }
}
