#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# Copyright (c) 2025 vivo Mobile Communication Co., Ltd.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#       http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import os
import sys
import argparse

GEN_BLOCK_IMG = r"""
rm -f {block_img}
dd if=/dev/zero of={block_img} bs=1M count={block_size}
"""

# A pre-populated block image (e.g. a FAT image produced at build time by
# `gen_fat_image`) is used verbatim: no `dd`, so build-time content survives to
# boot. The path is already on disk when the runner script runs.
USE_PREBUILT_BLOCK_IMG = r"""
# Pre-populated block image supplied at build time; use verbatim (no dd).
if [ ! -f {block_img} ]; then
  echo "error: prebuilt block image not found: {block_img}" >&2
  exit 1
fi
"""

TEST = r"""
exec {qemu} {semihosting} -M {machine} {qemu_args} {block_args} {net_args} -kernel {image} -nographic -serial \
       file:{logfile} -d int,cpu_reset,guest_errors,unimp -D {syslog}
"""

DBG = r"""
exec {qemu} {semihosting} -M {machine} {qemu_args} {block_args} {net_args} -kernel {image} -nographic -s -S
"""

DEFAULT = r"""
exec {qemu} {semihosting} -M {machine} {qemu_args} {block_args} {net_args} -kernel {image} -nographic
"""


def do_gen(config, template, suffix='', need_log=False):
    out_file = os.path.join(config.out_dir, f'{config.name}-qemu{suffix}.sh')
    logfile = None
    if need_log:
        logfile = os.path.abspath(
            os.path.join(config.out_dir, f'{config.name}-qemu{suffix}.log'))
        syslog = os.path.abspath(
            os.path.join(config.out_dir, f'{config.name}-qemu{suffix}.syslog'))
    machine = config.machine
    with open(out_file, 'w') as f:
        f.write(r"#!/bin/bash")

        block_args = ''
        if config.prebuilt_block_img:
            # Build-time FAT image: verify presence and use verbatim. The path
            # is already absolute (rebased by the GN template). `snapshot=on`
            # redirects writes to a per-QEMU temporary overlay so the shared
            # build-time image needs no exclusive write lock — without this,
            # `run_unittest` and `run_integration_test` (both shipping the same
            # `user_app_fat.img`) fail to start in parallel with "Failed to get
            # 'write' lock". Each test's writes (e.g. the VFS `mkdir` tests)
            # stay isolated to its own overlay.
            f.write(
                USE_PREBUILT_BLOCK_IMG.format(
                    block_img=config.prebuilt_block_img))
            block_args += f'-drive file={config.prebuilt_block_img},if=none,format=raw,snapshot=on,id=hd'
        elif config.block_img:
            dirname = os.path.dirname(config.block_img)
            os.makedirs(dirname, exist_ok=True)
            f.write(
                GEN_BLOCK_IMG.format(block_img=config.block_img,
                                     block_size=config.block_size))
            block_args += f'-drive file={config.block_img},if=none,format=raw,snapshot=on,id=hd'

        if config.block_args:
            block_args += ' ' + config.block_args

        if not need_log:
            f.write(
                template.format(
                    qemu=config.qemu,
                    semihosting='' if not config.semihosting else
                    '-accel tcg -semihosting-config enable=on',
                    machine=machine,
                    qemu_args=''
                    if not config.qemu_args != "" else config.qemu_args,
                    block_args=block_args,
                    net_args='' if not config.net_args else config.net_args,
                    image=os.path.abspath(config.image)))
        else:
            f.write(
                template.format(
                    qemu=config.qemu,
                    machine=machine,
                    semihosting='' if not config.semihosting else
                    '-accel tcg -semihosting-config enable=on',
                    qemu_args=''
                    if config.qemu_args != "" else config.qemu_args,
                    block_args=block_args,
                    net_args='' if not config.net_args else config.net_args,
                    logfile=logfile,
                    syslog=syslog,
                    image=os.path.abspath(config.image)))
    os.chmod(out_file, 0o755)
    print(f'Generated {out_file}')


def gen(config):
    do_gen(config, TEST, suffix='-test', need_log=True)
    do_gen(config, DBG, suffix='-dbg')
    do_gen(config, DEFAULT)


def main():
    parser = argparse.ArgumentParser(
        description='Generate QEMU runner script for BlueOS kernel image')
    parser.add_argument("--qemu",
                        help="Executable of QEMU emulator",
                        required=True)
    parser.add_argument("--machine",
                        help="Select emulated machine",
                        required=True)
    parser.add_argument("--name",
                        help="The id of the output script",
                        required=True)
    parser.add_argument("--out_dir", help="Output directory", required=True)
    parser.add_argument("--qemu_args", help="Extra qmeu args", default="")
    parser.add_argument("--block_img", help="Add block image", default="")
    parser.add_argument("--prebuilt_block_img",
                        help="Use a pre-populated block image verbatim "
                             "(no dd); content placed at build time survives",
                        default="")
    parser.add_argument("--block_size",
                        help="Specify block image size",
                        default=32)
    parser.add_argument("--block_args",
                        help="Args for block device",
                        default="")
    parser.add_argument("--net_args", help="Network args", default="")
    parser.add_argument("--semihosting",
                        help="Enable semihosting",
                        action='store_true',
                        default=False)
    parser.add_argument("image", help="Image file path")
    config = parser.parse_args()
    return gen(config)


if __name__ == '__main__':
    sys.exit(main())
