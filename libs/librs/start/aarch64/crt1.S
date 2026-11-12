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

// Native ELF entry: argc/argv on the stack, optional rtld_fini in x0.
// The loader has already relocated this self-contained PIE.
    .text
    .p2align 2
    .globl _start
    .type _start, %function
_start:
    mov x29, xzr
    mov x30, xzr
    mov x5, x0
    ldr x1, [sp]
    add x2, sp, #8
    adrp x0, main
    add x0, x0, :lo12:main
    mov x3, xzr
    mov x4, xzr
    mov x6, sp
    bl __libc_start_main
    brk #0
    .size _start, .-_start

    .section .note.GNU-stack,"",%progbits
