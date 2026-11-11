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

// Native ELF entry: argc/argv on the stack, optional rtld_fini in a0.
// The loader has already relocated this self-contained PIE.
    .text
    .option pic
    .p2align 2
    .globl _start
    .type _start, @function
_start:
    .option push
    .option norelax
    lla gp, __global_pointer$
    .option pop
    li s0, 0
    mv a5, a0
    lw a1, 0(sp)
    addi a2, sp, (__riscv_xlen / 8)
    la a0, main
    li a3, 0
    li a4, 0
    mv a6, sp
    call __libc_start_main
    unimp
    .size _start, .-_start

    .section .note.GNU-stack,"",@progbits
