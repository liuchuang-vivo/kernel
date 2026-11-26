#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# Copyright (c) 2026 vivo Mobile Communication Co., Ltd.
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

"""Build a pre-populated FAT16 image for the QEMU virtio block device.

The kernel's `FatFileSystem::new` (`vfs/fatfs.rs`) only falls back to
`format_volume` when `fatfs::FileSystem::new` fails on the backing storage.
A well-formed FAT written here is opened as-is, so files placed on the image
at build time survive to boot and are readable from `/fat/...` via the VFS.

This is a self-contained FAT16 writer: no `mtools`/`dosfstools` host
dependency. The image uses 512-byte sectors, 4096-byte clusters to match the
kernel `BLOCK_SIZE`, so FAT16 applies (< 65525 clusters).

IMPORTANT: rust-fatfs derives the FAT type from the *cluster count*
(`FatType::from_clusters`), NOT from the BPB FilSysType string. FAT16 needs at
least 4085 data clusters; below that the volume is read as FAT12 and the
16-bit FAT entries this writer produces are misdecoded as 12-bit packed
entries, silently corrupting every chain (mkdir/alloc_cluster then fails). At
4096-byte clusters, 4085 clusters need ~16.7 MiB of data area, so the default
size is 32 MiB. `build_image` enforces the >= 4085 floor as a hard assertion.

Only the subset of FAT16 needed to ship a flat directory of files is
implemented: one root directory, 8.3 short names, contiguous cluster
allocation per file. That is enough for `boot::init_user_app` reading
`/fat/exec_app.elf`.
"""

import argparse
import datetime
import os
import pathlib
import struct
import sys


SECTOR_SIZE = 512
CLUSTER_SIZE = 4096
SECTORS_PER_CLUSTER = CLUSTER_SIZE // SECTOR_SIZE  # 8

# FAT entry values.
FAT_FREE = 0x0000
FAT_EOC = 0xFFF8  # end-of-cluster chain marker (FAT16)


def parse_args():
    parser = argparse.ArgumentParser(
        description="Build a populated FAT16 image for the QEMU block device")
    parser.add_argument("--output", required=True,
                        help="Path to the FAT image to produce")
    parser.add_argument("--size", default="32MiB",
                        help="Image size, e.g. 32MiB, 8MiB, 512K (bytes). "
                             "Must hold the FAT + root dir + all file data "
                             "AND keep the cluster count >= 4085 so the "
                             "volume is genuinely FAT16 (a too-small image "
                             "is silently read as FAT12 and the 16-bit FAT "
                             "entries are misdecoded).")
    parser.add_argument("--volume-label", default="BLUEOS",
                        help="11-byte (padded) volume label")
    parser.add_argument("files", nargs="*", metavar="SRC=DST",
                        help="Files to place at the image root. "
                             "Each argument is `src_path=dest_name` "
                             "(dest_name is an 8.3 name placed at the root).")
    return parser.parse_args()


def parse_size(text):
    text = text.strip()
    if not text:
        raise ValueError("empty size")
    units = {
        "": 1, "B": 1,
        "K": 1024, "KB": 1024, "KiB": 1024,
        "M": 1024 * 1024, "MB": 1024 * 1024, "MiB": 1024 * 1024,
    }
    number = ""
    idx = 0
    while idx < len(text) and (text[idx].isdigit() or text[idx] == "."):
        number += text[idx]
        idx += 1
    unit = text[idx:].strip()
    if not number:
        raise ValueError(f"invalid size (no number): {text}")
    if unit not in units:
        raise ValueError(f"invalid size unit: {unit!r}")
    value = float(number) * units[unit]
    return int(value)


def parse_file_specs(specs):
    """Yield (src_path, dest_name) pairs. Validate dest_name is 8.3-ish.

    The dest name's original case is preserved: it drives the NTRes lowercase
    bits written to the directory entry (see `build_image`), which the fatfs
    crate honors via `DirFileEntryData::lowercase_name`. The on-disk short-name
    field is always uppercase (FAT spec); the NTRes bits tell readers to display
    it lowercased. Without those bits, `file_name()` returns e.g.
    `EXEC_APP.ELF`, and the kernel VFS dcache (case-sensitive `BTreeMap::get`)
    won't find a `/fat/exec_app.elf` lookup — so preserving case here matters.
    """
    parsed = []
    for spec in specs:
        if "=" not in spec:
            raise ValueError(
                f"file spec must be SRC=DST, got {spec!r}")
        src, dst = spec.split("=", 1)
        src = src.strip()
        dst = dst.strip()
        if not src or not dst:
            raise ValueError(f"empty src or dst in {spec!r}")
        if "/" in dst or "\\" in dst:
            raise ValueError(
                f"dest must be a flat root entry, got {dst!r}")
        # Validate 8.3: NAME.EXT, NAME ≤ 8 chars, EXT ≤ 3 chars.
        if "." in dst:
            base, _, ext = dst.partition(".")
        else:
            base, ext = dst, ""
        if not base or len(base) > 8 or len(ext) > 3:
            raise ValueError(
                f"dest must be valid 8.3 (base ≤ 8, ext ≤ 3): {dst!r}")
        if not all(c.isalnum() or c in "_-." for c in dst):
            raise ValueError(
                f"dest contains invalid characters: {dst!r}")
        parsed.append((src, dst))
    return parsed


def short_date_time(when=None):
    """FAT 16-bit date and time fields for the given datetime (or now)."""
    when = when or datetime.datetime.now()
    fat_date = ((when.year - 1980) << 9) | (when.month << 5) | when.day
    fat_time = (when.hour << 11) | (when.minute << 5) | (when.second // 2)
    return fat_date & 0xFFFF, fat_time & 0xFFFF


def build_image(output_path, image_bytes, label, files):
    total_sectors = image_bytes // SECTOR_SIZE
    if total_sectors * SECTOR_SIZE != image_bytes:
        raise ValueError("image size must be a multiple of the sector size")

    # Reserve one sector for the boot sector + backup boot sector at sector 6.
    reserved_sectors = 1
    num_fats = 1  # one FAT copy is plenty for a build artifact
    # Match rust-fatfs's own FormatVolumeOptions default (512): with LFN
    # enabled (the crate is built with feature "lfn"), each long-named entry
    # consumes 1 LFN slot + 1 SFN slot in the root. The tiny 16-entry root
    # filled quickly once the vfs test creates a directory tree, so give it
    # real headroom. The cost is one sector per 16 entries (16 KiB here).
    root_entries = 512
    root_dir_sectors = (
        (root_entries * 32) + (SECTOR_SIZE - 1)
    ) // SECTOR_SIZE

    # Cluster count assuming zero data first, to size the FAT, then recompute
    # with the FAT size fixed (one iteration converges for these magnitudes).
    def cluster_count(fat_sectors):
        first_data_sector = (
            reserved_sectors
            + num_fats * fat_sectors
            + root_dir_sectors
        )
        data_sectors = total_sectors - first_data_sector
        return data_sectors // SECTORS_PER_CLUSTER

    # FAT16: each entry is 2 bytes; a FAT sector holds 256 entries.
    # Start with a guess, then clamp to the count that fits the remaining data.
    fat_sectors = 1
    for _ in range(8):
        clusters = cluster_count(fat_sectors)
        needed_entries = clusters + 2  # cluster 0,1 reserved
        needed_sectors = (needed_entries * 2 + SECTOR_SIZE - 1) // SECTOR_SIZE
        if needed_sectors <= fat_sectors:
            break
        fat_sectors = needed_sectors
    clusters = cluster_count(fat_sectors)
    if clusters >= 0xFFF5:
        raise ValueError("image too large for FAT16; reduce --size")
    if clusters < 1:
        raise ValueError("image too small to hold any data cluster")
    # FAT16 only applies with >= 4085 data clusters. rust-fatfs derives the FAT
    # type from the cluster count (FatType::from_clusters), NOT from the BPB
    # FilSysType string, and validate_total_clusters only rejects a FAT32
    # mismatch — so a too-small FAT16 image silently mounts as FAT12 while the
    # 16-bit FAT entries we write below are decoded as 12-bit packed entries,
    # corrupting every chain (find_free_cluster sees bogus non-zero values and
    # fails on the first alloc_cluster). Enforce the floor here so the format
    # and the crate's type detection can never disagree.
    FAT16_MIN_CLUSTERS = 4085
    if clusters < FAT16_MIN_CLUSTERS:
        raise ValueError(
            f"image has {clusters} data clusters, but FAT16 requires at least "
            f"{FAT16_MIN_CLUSTERS} (below this, the crate reads the volume as "
            f"FAT12 and the 16-bit FAT entries here are misdecoded). At "
            f"{CLUSTER_SIZE}-byte clusters, --size needs at least "
            f"{FAT16_MIN_CLUSTERS * CLUSTER_SIZE} bytes of data area; raise "
            f"--size (e.g. 32MiB).")

    first_data_sector = (
        reserved_sectors
        + num_fats * fat_sectors
        + root_dir_sectors
    )

    # Lay out the image as a bytearray.
    image = bytearray(image_bytes)

    # --- Boot sector (BPB) ---
    # jmpBoot + NOP
    image[0:3] = b"\xEB\x3C\x90"
    # OEM name (8 bytes)
    image[3:11] = b"MSWIN4.1"
    # BPB:
    struct.pack_into("<H", image, 11, SECTOR_SIZE)             # BytsPerSec
    image[13] = SECTORS_PER_CLUSTER                            # SecPerClus
    struct.pack_into("<H", image, 14, reserved_sectors)        # RsvdSecCnt
    image[16] = num_fats                                       # NumFATs
    struct.pack_into("<H", image, 17, root_entries)            # RootEntCnt
    if total_sectors < 0x10000:
        struct.pack_into("<H", image, 19, total_sectors)       # TotSec16
        struct.pack_into("<I", image, 32, 0)                   # TotSec32
    else:
        struct.pack_into("<H", image, 19, 0)
        struct.pack_into("<I", image, 32, total_sectors)       # TotSec32
    image[21] = 0xF8                                           # Media (fixed)
    struct.pack_into("<H", image, 22, fat_sectors)             # FATSz16
    struct.pack_into("<H", image, 24, 63)                      # SecPerTrk
    struct.pack_into("<H", image, 26, 255)                     # NumHeads
    struct.pack_into("<I", image, 28, 0)                       # HiddSec
    # Extended BPB (FAT16):
    image[36] = 0x80                                           # DrvNum
    image[37] = 0                                              # Reserved1
    image[38] = 0x29                                           # BootSig
    struct.pack_into("<I", image, 39, 0x12345678)              # VolID
    vol_label = label.encode("ascii", "replace")[:11]
    image[43:43 + 11] = vol_label.ljust(11, b" ")
    image[54:62] = b"FAT16   "                                  # FilSysType
    # Boot signature.
    image[510] = 0x55
    image[511] = 0xAA

    # --- FAT ---
    fat_start = reserved_sectors * SECTOR_SIZE
    # Entry 0: media byte + 0xFF (low 8 bits = media, rest = 0xFF).
    fat = bytearray(fat_sectors * SECTOR_SIZE)
    struct.pack_into("<H", fat, 0, 0xFF00 | 0xF8)   # media descriptor pattern
    struct.pack_into("<H", fat, 2, FAT_EOC)         # entry 1: reserved EOC

    next_cluster = 2  # first usable data cluster

    # --- Root directory ---
    root_start = (reserved_sectors + num_fats * fat_sectors) * SECTOR_SIZE
    root = bytearray(root_dir_sectors * SECTOR_SIZE)
    fat_date, fat_time = short_date_time()

    # Volume label entry first (optional but conventional).
    entry = bytearray(32)
    entry[0:11] = vol_label.ljust(11, b" ")
    entry[11] = 0x08  # ATTR_VOLUME_ID
    struct.pack_into("<H", entry, 22, fat_time)
    struct.pack_into("<H", entry, 24, fat_date)
    root[0:32] = entry

    root_slot = 1  # next 32-byte slot in the root directory

    for src_path, dst_name in files:
        data = pathlib.Path(src_path).read_bytes()
        if not data:
            # Zero-length file: no clusters allocated, first cluster = 0.
            first_cluster = 0
        else:
            n_clusters = (len(data) + CLUSTER_SIZE - 1) // CLUSTER_SIZE
            if next_cluster + n_clusters - 1 > clusters + 1:
                raise ValueError(
                    f"file {dst_name} ({len(data)} bytes) does not fit in "
                    f"the image (need cluster {next_cluster + n_clusters - 1}, "
                    f"have {clusters + 1})")
            first_cluster = next_cluster
            for i in range(n_clusters):
                cur = next_cluster + i
                nxt = next_cluster + i + 1
                is_last = (i == n_clusters - 1)
                struct.pack_into(
                    "<H", fat, cur * 2,
                    FAT_EOC if is_last else nxt)
                # Write the cluster's bytes into the data region.
                cluster_byte = (
                    (first_data_sector + (cur - 2) * SECTORS_PER_CLUSTER)
                    * SECTOR_SIZE)
                chunk = data[i * CLUSTER_SIZE:(i + 1) * CLUSTER_SIZE]
                image[cluster_byte:cluster_byte + len(chunk)] = chunk
            next_cluster += n_clusters

        # Root directory entry for this file.
        if root_slot * 32 + 32 > len(root):
            raise ValueError("root directory is full; raise root_entries")
        if "." in dst_name:
            base, _, ext = dst_name.partition(".")
        else:
            base, ext = dst_name, ""
        # FAT short names are stored uppercase on disk. The NTRes byte (offset
        # 12) bit3 = lowercase base, bit4 = lowercase ext: when set, readers
        # like the fatfs crate's `lowercase_name()` display the field lowercased
        # (so `file_name()` returns "exec_app.elf" not "EXEC_APP.ELF"). The
        # kernel VFS dcache keys on `file_name()` and does case-sensitive
        # `BTreeMap::get`, so the displayed case must match the lookup path.
        # Mixed-case names can't be represented this way (would need LFN
        # entries); reject them so a silent mismatch can't slip in.
        if base.islower() or base == "":
            ntres_base_lower = 0x08 if base else 0
        elif base.isupper():
            ntres_base_lower = 0
        else:
            raise ValueError(
                f"dest base {base!r} is mixed-case; FAT 8.3 short names "
                f"are either all-upper or all-lower (use LFN for mixed)")
        if ext.islower():
            ntres_ext_lower = 0x10
        elif ext.isupper() or ext == "":
            ntres_ext_lower = 0
        else:
            raise ValueError(
                f"dest ext {ext!r} is mixed-case; FAT 8.3 short names "
                f"are either all-upper or all-lower (use LFN for mixed)")
        name_field = base.upper().ljust(8, " ")[:8].encode("ascii")
        ext_field = ext.upper().ljust(3, " ")[:3].encode("ascii")
        e = bytearray(32)
        e[0:8] = name_field
        e[8:11] = ext_field
        e[11] = 0x20  # ATTR_ARCHIVE
        e[12] = ntres_base_lower | ntres_ext_lower  # NTRes (lowercase bits)
        e[13] = 0     # CrtTimeTenth
        struct.pack_into("<H", e, 14, fat_time)   # CrtTime
        struct.pack_into("<H", e, 16, fat_date)   # CrtDate
        struct.pack_into("<H", e, 18, fat_date)   # LstAccDate
        struct.pack_into("<H", e, 20, ((fat_time >> 11) & 0x1F))  # FstClusHI
        struct.pack_into("<H", e, 22, fat_time)   # WrtTime
        struct.pack_into("<H", e, 24, fat_date)   # WrtDate
        struct.pack_into("<H", e, 26, first_cluster & 0xFFFF)  # FstClusLO
        struct.pack_into("<I", e, 28, len(data))  # FileSize
        root[root_slot * 32:root_slot * 32 + 32] = e
        root_slot += 1

    # Commit FAT and root directory into the image.
    image[fat_start:fat_start + len(fat)] = fat
    image[root_start:root_start + len(root)] = root

    pathlib.Path(output_path).parent.mkdir(parents=True, exist_ok=True)
    with open(output_path, "wb") as f:
        f.write(image)
    print(f"Wrote FAT16 image {output_path} "
          f"({image_bytes} bytes, {clusters} clusters, "
          f"{root_slot - 1} files)")


def main():
    args = parse_args()
    image_bytes = parse_size(args.size)
    files = parse_file_specs(args.files)
    if not files:
        print("warning: no files given; image will be an empty FAT16 volume",
          file=sys.stderr)
    build_image(args.output, image_bytes, args.volume_label, files)
    return 0


if __name__ == "__main__":
    sys.exit(main())
