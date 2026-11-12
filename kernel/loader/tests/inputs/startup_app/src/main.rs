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

//! Independent process-entry probe: it reads the raw ABI layout directly,
//! without using the kernel serializer or librs startup decoder.

#![no_std]
#![no_main]

use blueos_header::syscalls::NR::ExitThread;
use blueos_scal::bk_syscall;
use core::{mem::size_of, ptr};

#[cfg(any(target_arch = "riscv32", target_arch = "riscv64"))]
#[unsafe(no_mangle)]
#[unsafe(naked)]
unsafe extern "C" fn _start() -> ! {
    core::arch::naked_asm!(
        ".option push", ".option norelax", "la gp, __global_pointer$", ".option pop",
        "mv a1, a0", "mv a0, sp", "tail {check}", check = sym check,
    );
}

#[cfg(target_arch = "aarch64")]
#[unsafe(no_mangle)]
#[unsafe(naked)]
unsafe extern "C" fn _start() -> ! {
    core::arch::naked_asm!("mov x1, x0", "mov x0, sp", "b {check}", check = sym check);
}

unsafe extern "C" fn check(sp: *const usize, rtld_fini: usize) -> ! {
    // SAFETY: the test host supplies a live native ELF frame with three argv
    // strings, one environment string and initialized auxiliary storage.
    unsafe {
        let result = *sp.add(2) as *mut u8;
        let passed = sp.addr().is_multiple_of(16)
            && rtld_fini == 0
            && sp.read() == 3
            && matches(*sp.add(1), b"startup-probe\0")
            && matches(*sp.add(2), b"pending\0")
            && matches(*sp.add(3), b"\0")
            && *sp.add(4) == 0
            && matches(*sp.add(5), b"MODE=test\0")
            && *sp.add(6) == 0
            && check_auxv(sp.add(7));
        ptr::copy_nonoverlapping(
            if passed { b"passed!\0" } else { b"failed!\0" }.as_ptr(),
            result,
            8,
        );
    }
    finish()
}

unsafe fn matches(address: usize, expected: &[u8]) -> bool {
    if address == 0 {
        return false;
    }
    // SAFETY: the trusted fixture's host supplies these readable strings.
    expected
        .iter()
        .enumerate()
        .all(|(index, &byte)| unsafe { (address as *const u8).add(index).read() == byte })
}

unsafe fn check_auxv(mut entry: *const usize) -> bool {
    let mut present = 0;
    for _ in 0..16 {
        // SAFETY: the host supplies complete pairs through AT_NULL.
        let (tag, value) = unsafe { (entry.read(), entry.add(1).read()) };
        match tag {
            0 => return present == 0xff,
            3 => {
                if value == 0 || !value.is_multiple_of(size_of::<usize>()) {
                    return false;
                }
                // Toolchains emit either PT_LOAD or PT_PHDR first. Both are
                // actual records in this fixture's readable mapped table.
                if !matches!(unsafe { (value as *const u32).read() }, 1 | 6) {
                    return false;
                }
                present |= 1;
            }
            4 => {
                if value != if size_of::<usize>() == 8 { 56 } else { 32 } {
                    return false;
                }
                present |= 2;
            }
            5 => {
                if value == 0 || value > 128 {
                    return false;
                }
                present |= 4;
            }
            6 => {
                if value != 4096 {
                    return false;
                }
                present |= 8;
            }
            7 => {
                if value != 0 {
                    return false;
                }
                present |= 16;
            }
            9 => {
                if value != _start as *const () as usize {
                    return false;
                }
                present |= 32;
            }
            25 => {
                for i in 0..16 {
                    if unsafe { (value as *const u8).add(i).read() } != i as u8 {
                        return false;
                    }
                }
                present |= 64;
            }
            31 => {
                if !unsafe { matches(value, b"/startup-probe\0") } {
                    return false;
                }
                present |= 128;
            }
            _ => return false,
        }
        // SAFETY: each initialized pair occupies two words.
        entry = unsafe { entry.add(2) };
    }
    false
}

fn finish() -> ! {
    bk_syscall!(ExitThread);
    loop {
        core::hint::spin_loop();
    }
}

#[panic_handler]
fn panic(_: &core::panic::PanicInfo<'_>) -> ! {
    finish()
}
