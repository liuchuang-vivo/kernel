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

//! Logical layout and allocation-offset validation.

use crate::{
    error::{ErrorContext, LoadErrorKind},
    memory::{
        AllocationOffset, ImageLayout, MemoryPermissions, Placement, SegmentLayout, TargetAddress,
        TargetRange,
    },
};
use alloc::vec;
use blueos_test_macro::test;

fn segment(allocation_offset: u64, len: u64) -> SegmentLayout {
    SegmentLayout::new(
        AllocationOffset::new(allocation_offset),
        len,
        MemoryPermissions::READ,
    )
}

#[test]
fn image_layout_rejects_invalid_reservations_and_segments() {
    let cases = [
        (
            Placement::Anywhere,
            0,
            16,
            vec![segment(0, 1)],
            LoadErrorKind::IncorrectLayout,
        ),
        (
            Placement::Anywhere,
            64,
            0,
            vec![segment(0, 1)],
            LoadErrorKind::InvalidAlignment,
        ),
        (
            Placement::Anywhere,
            64,
            3,
            vec![segment(0, 1)],
            LoadErrorKind::InvalidAlignment,
        ),
        (
            Placement::Anywhere,
            64,
            16,
            vec![],
            LoadErrorKind::IncorrectLayout,
        ),
        (
            Placement::Anywhere,
            64,
            16,
            vec![segment(0, 0)],
            LoadErrorKind::IncorrectLayout,
        ),
        (
            Placement::Anywhere,
            64,
            16,
            vec![segment(8, 32), segment(16, 4)],
            LoadErrorKind::IncorrectLayout,
        ),
        (
            Placement::Anywhere,
            64,
            16,
            vec![segment(32, 8), segment(0, 8)],
            LoadErrorKind::IncorrectLayout,
        ),
        (
            Placement::Anywhere,
            64,
            16,
            vec![segment(63, 2)],
            LoadErrorKind::OutOfBounds,
        ),
        (
            Placement::Anywhere,
            64,
            16,
            vec![segment(u64::MAX, 2)],
            LoadErrorKind::IntegerOverflow,
        ),
        (
            Placement::Fixed(TargetRange::new(TargetAddress::new(0x1000), 63)),
            64,
            16,
            vec![segment(0, 64)],
            LoadErrorKind::IncorrectLayout,
        ),
        (
            Placement::Fixed(TargetRange::new(TargetAddress::new(0x1001), 64)),
            64,
            16,
            vec![segment(0, 64)],
            LoadErrorKind::IncorrectLayout,
        ),
        (
            Placement::Fixed(TargetRange::new(TargetAddress::new(u64::MAX - 15), 32)),
            32,
            16,
            vec![segment(0, 32)],
            LoadErrorKind::IntegerOverflow,
        ),
    ];
    for (placement, size, align, segments, kind) in cases {
        assert_eq!(
            ImageLayout::new(placement, size, align, segments)
                .unwrap_err()
                .kind(),
            kind
        );
    }
    let layout = ImageLayout::new(
        Placement::Anywhere,
        128,
        16,
        vec![segment(0, 16), segment(16, 16), segment(96, 32)],
    )
    .unwrap();
    assert_eq!(layout.segments().len(), 3);
}

#[test]
fn allocation_offsets_report_addition_overflow() {
    let error = AllocationOffset::new(u64::MAX).checked_add(1).unwrap_err();
    assert_eq!(error.kind(), LoadErrorKind::IntegerOverflow);
    assert!(matches!(
        error.context(),
        ErrorContext::MemoryAccess {
            offset: u64::MAX,
            len: 1,
            ..
        }
    ));
}
