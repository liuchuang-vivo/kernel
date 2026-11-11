// Copyright (c) 2025 vivo Mobile Communication Co., Ltd.
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

// The current flat backend allocates through kernel syscalls. This libc owns
// no separate heap metadata; blocks remain compatible across this API family.
use crate::errno::ERRNO;
use blueos_header::syscalls::NR::{AllocMem, FreeMem, ReallocMem};
use blueos_scal::bk_syscall;
use core::{alloc::Layout, ptr};
use libc::{c_int, c_void, size_t, EINVAL, ENOMEM};

// Cover fundamental C alignment, including long double, on current ELF ABIs.
// Rust std also relies on malloc/realloc providing its system minimum alignment.
const MALLOC_ALIGNMENT: usize = 16;

/// Allocate without modifying errno. Failure leaves the output unchanged.
///
/// # Safety
/// output must name an exclusive writable pointer slot. On success the caller
/// owns size uninitialized bytes, to be released through this allocator family.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn posix_memalign(
    output: *mut *mut c_void,
    align: size_t,
    size: size_t,
) -> c_int {
    if output.is_null() || !align.is_power_of_two() || align < size_of::<*mut c_void>() {
        return EINVAL;
    }
    if Layout::from_size_align(size, align).is_err() {
        return ENOMEM;
    }
    if size == 0 {
        unsafe { output.write(ptr::null_mut()) };
        return 0;
    }
    if bk_syscall!(AllocMem, output, size, align) != 0 {
        return ENOMEM;
    }
    0
}

/// Release an allocation; NULL is accepted.
///
/// # Safety
/// ptr must be null or a live base returned by this allocator family, with
/// exclusive ownership and no subsequent access through the released pointers.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn free(ptr: *mut c_void) {
    bk_syscall!(FreeMem, ptr);
}

/// Allocate uninitialized bytes, or return NULL for zero size or failure.
///
/// # Safety
/// A returned block must be initialized before read, accessed within size,
/// and eventually released using this allocator family.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn malloc(size: size_t) -> *mut c_void {
    unsafe { memalign(MALLOC_ALIGNMENT, size) }
}

/// Allocate zeroed bytes; an overflowing element count fails with ENOMEM.
///
/// # Safety
/// A returned block owns count * size bytes and must eventually be freed.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn calloc(count: size_t, size: size_t) -> *mut c_void {
    let Some(total) = count.checked_mul(size) else {
        ERRNO.set(ENOMEM);
        return ptr::null_mut();
    };
    let block = unsafe { malloc(total) };
    if !block.is_null() {
        unsafe { ptr::write_bytes(block.cast::<u8>(), 0, total) };
    }
    block
}

/// Allocate with any nonzero power-of-two alignment; failure sets errno.
///
/// # Safety
/// A returned block owns size uninitialized bytes and must eventually be freed.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn memalign(align: size_t, size: size_t) -> *mut c_void {
    if !align.is_power_of_two() {
        ERRNO.set(EINVAL);
        return ptr::null_mut();
    }
    let mut block = ptr::null_mut();
    let rc = unsafe { posix_memalign(&raw mut block, align.max(size_of::<*mut c_void>()), size) };
    if rc != 0 {
        ERRNO.set(rc);
    }
    block
}

/// Resize a block, preserving the smaller size's prefix. Failure preserves the
/// old allocation; size zero frees it. Only the kernel knows its old size.
///
/// # Safety
/// ptr must be null or an exclusively owned live base from this allocator
/// family. A successful resize invalidates every old pointer into the block.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn realloc(ptr: *mut c_void, size: size_t) -> *mut c_void {
    let mut moved = ptr::null_mut();
    if bk_syscall!(ReallocMem, &raw mut moved, ptr, size, MALLOC_ALIGNMENT) != 0 {
        ERRNO.set(ENOMEM);
    }
    moved
}

#[cfg(test)]
mod tests {
    use super::*;
    use blueos_test_macro::test;

    #[test]
    fn posix_alignment_failure_preserves_output_and_errno() {
        let sentinel = ptr::dangling_mut::<c_void>();
        let mut block = sentinel;
        ERRNO.set(libc::EBUSY);
        for align in [0, 3, size_of::<*mut c_void>() / 2] {
            assert_eq!(unsafe { posix_memalign(&raw mut block, align, 64) }, EINVAL);
            assert_eq!(block, sentinel);
            assert_eq!(ERRNO.get(), libc::EBUSY);
        }
        assert_eq!(
            unsafe { posix_memalign(&raw mut block, MALLOC_ALIGNMENT, usize::MAX) },
            ENOMEM
        );
        assert_eq!(block, sentinel);
        assert_eq!(ERRNO.get(), libc::EBUSY);
        assert_eq!(
            unsafe { posix_memalign(&raw mut block, MALLOC_ALIGNMENT, 0) },
            0
        );
        assert!(block.is_null());
    }

    #[test]
    fn calloc_checks_overflow_and_zeroes_storage() {
        unsafe {
            // Force the exported C entry point rather than allocator-call folding.
            let allocate = core::hint::black_box(
                calloc as unsafe extern "C" fn(size_t, size_t) -> *mut c_void,
            );
            assert!(allocate(usize::MAX / 2 + 1, 2).is_null());
            assert_eq!(ERRNO.get(), ENOMEM);
            let block = calloc(32, 3).cast::<u8>();
            assert!(!block.is_null());
            assert!(block.addr().is_multiple_of(MALLOC_ALIGNMENT));
            assert!(core::slice::from_raw_parts(block, 96)
                .iter()
                .all(|byte| *byte == 0));
            free(block.cast());
        }
    }

    #[test]
    fn realloc_preserves_prefix_alignment_and_failed_allocation() {
        unsafe {
            let block = malloc(24).cast::<u8>();
            assert!(!block.is_null());
            for index in 0..24 {
                block.add(index).write(index as u8);
            }
            let moved = realloc(block.cast(), 4096).cast::<u8>();
            assert!(!moved.is_null());
            assert!(moved.addr().is_multiple_of(MALLOC_ALIGNMENT));
            for index in 0..24 {
                assert_eq!(moved.add(index).read(), index as u8);
            }
            assert!(realloc(moved.cast(), core::hint::black_box(usize::MAX)).is_null());
            assert_eq!(ERRNO.get(), ENOMEM);
            for index in 0..24 {
                assert_eq!(moved.add(index).read(), index as u8);
            }
            let shrunk = realloc(moved.cast(), 8).cast::<u8>();
            assert!(!shrunk.is_null());
            for index in 0..8 {
                assert_eq!(shrunk.add(index).read(), index as u8);
            }
            assert!(realloc(shrunk.cast(), 0).is_null());
            let fresh = realloc(ptr::null_mut(), 32);
            assert!(!fresh.is_null());
            free(fresh);
        }
    }

    #[test]
    fn memalign_supports_small_and_extended_alignments() {
        unsafe {
            for align in [1, 2, 64, 4096] {
                let block = memalign(align, 17);
                assert!(!block.is_null());
                assert!(block.addr().is_multiple_of(align));
                free(block);
            }
            assert!(memalign(3, 17).is_null());
            assert_eq!(ERRNO.get(), EINVAL);
        }
    }
}
