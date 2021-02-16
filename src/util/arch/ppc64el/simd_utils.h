/*
 * Copyright (c) 2015-2020, Intel Corporation
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *  * Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *  * Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *  * Neither the name of Intel Corporation nor the names of its contributors
 *    may be used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

/** \file
 * \brief SIMD types and primitive operations.
 */

#ifndef ARCH_PPC64EL_SIMD_UTILS_H
#define ARCH_PPC64EL_SIMD_UTILS_H

#include <stdio.h>

#include "ue2common.h"
#include "util/simd_types.h"
#include "util/unaligned.h"
#include "util/intrinsics.h"

#include <string.h> // for memcpy

typedef __vector  uint64_t uint64x2_t;
typedef __vector   int64_t  int64x2_t;
typedef __vector  uint32_t uint32x4_t;
typedef __vector   int32_t  int32x4_t;
typedef __vector  uint16_t uint16x8_t;
typedef __vector   int16_t  int16x8_t;
typedef __vector   uint8_t uint8x16_t;
typedef __vector    int8_t  int8x16_t;

static really_inline m128 ones128(void) {
    return (m128) vec_splat_s8(-1); // 0xff
}

static really_inline m128 zeroes128(void) {
    return (m128) vec_splat_s32(0);
}

/** \brief Bitwise not for m128*/
static really_inline m128 not128(m128 a) {
    return (m128) vec_xor(a, ones128());
}

/** \brief Return 1 if a and b are different otherwise 0 */
static really_inline int diff128(m128 a, m128 b) {
    return vec_any_ne(a, b);
}

static really_inline int isnonzero128(m128 a) {
    return diff128(a, zeroes128());
}

/**
 * "Rich" version of diff128(). Takes two vectors a and b and returns a 4-bit
 * mask indicating which 32-bit words contain differences.
 */
static really_inline u32 diffrich128(m128 a, m128 b) {
    static const m128 movemask = { 1, 2, 4, 8 };
    m128 mask = (m128) vec_cmpeq(a, b);
    mask = vec_and(not128(mask), movemask);
    m128 sum = vec_sums(mask, zeroes128());
    return vec_extract((int32x4_t)sum, 3);
}

/**
 * "Rich" version of diff128(), 64-bit variant. Takes two vectors a and b and
 * returns a 4-bit mask indicating which 64-bit words contain differences.
 */
static really_inline u32 diffrich64_128(m128 a, m128 b) {
    static const uint64x2_t movemask = { 1, 4 };
    m128 mask = (m128) vec_cmpeq((uint64x2_t)a, (uint64x2_t)b);
    mask = vec_and(not128(mask), (m128)movemask);
    m128 sum = vec_sums((m128)mask, zeroes128());
    return vec_extract((int32x4_t)sum, 3);
}

static really_really_inline
m128 add_2x64(m128 a, m128 b) {
    return (m128) vec_add((uint64x2_t)a, (uint64x2_t)b);
}

static really_really_inline
m128 sub_2x64(m128 a, m128 b) {
    return (m128) vec_sub((uint64x2_t)a, (uint64x2_t)b);
}

static really_really_inline
m128 lshift_m128(m128 a, u32 b) {
    uint32x4_t shift_mask = vec_splats(b);
    return (m128) vec_sl(a, shift_mask);
}

static really_really_inline
m128 rshift_m128(m128 a, u32 b) {
    uint32x4_t shift_mask = vec_splats(b);
    return (m128) vec_sr(a, shift_mask);
}

static really_really_inline
m128 lshift64_m128(m128 a, unsigned b) {
    uint64x2_t shift_mask = vec_splats((u64a)b);
    return (m128) vec_sl((int64x2_t)a, shift_mask);
}

static really_really_inline
m128 rshift64_m128(m128 a, unsigned b) {
    uint64x2_t shift_mask = vec_splats((u64a)b);
    return (m128) vec_sl((int64x2_t)a, shift_mask);
}

static really_inline m128 eq128(m128 a, m128 b) {
    return (m128) vec_cmpeq((int8x16_t)a, (int8x16_t)b);
}

static really_inline m128 eq64_m128(m128 a, m128 b) {
    return (m128) vec_cmpeq((int64x2_t)a, (int64x2_t)b);
}

static really_inline u32 movemask128(m128 a) {
    static const uint8x16_t powers = { 1, 2, 4, 8, 16, 32, 64, 128, 1, 2, 4, 8, 16, 32, 64, 128 };
    static const uint32x4_t shift_mask = { 0, 0, 8, 8 };

    // Compute the mask from the input
    uint8x16_t mask = (uint8x16_t) vec_and((uint8x16_t)a, powers);
    mask = (uint8x16_t) vec_sum4s(mask, (uint32x4_t)zeroes128());
    mask = (uint8x16_t) vec_sl((uint32x4_t) mask, shift_mask);
    mask = (uint8x16_t) vec_sums((int32x4_t)mask, (int32x4_t)zeroes128());

    // Get the resulting bytes
    return vec_extract((uint16x8_t)mask, 6);
}

static really_inline m128 set1_16x8(u8 c) {
    return (m128) vec_splats(c);
}

static really_inline m128 set1_4x32(u32 c) {
    return (m128) vec_splats(c);
}

static really_inline m128 set1_2x64(u64a c) {
    return (m128) vec_splats(c);
}

static really_inline u32 movd(const m128 in) {
    return vec_extract((uint32x4_t)in, 0);
}

static really_inline u64a movq(const m128 in) {
    return vec_extract((uint64x2_t)in, 0);
}

/* another form of movq */
static really_inline
m128 load_m128_from_u64a(const u64a *p) {
    return (m128) vec_insert(*p, zeroes128(), 0);
}

static really_inline u32 extract32from128(const m128 in, unsigned imm) {
#if defined(HS_OPTIMIZE)
    return vec_extract((uint32x4_t) in, imm);
#else
    switch (imm) {
    case 0:
        return vec_extract((uint32x4_t)in, 0);
	break;
    case 1:
        return vec_extract((uint32x4_t)in, 1);
	break;
    case 2:
        return vec_extract((uint32x4_t)in, 2);
	break;
    case 3:
        return vec_extract((uint32x4_t)in, 3);
	break;
    default:
	return 0;
	break;
    }
#endif
}

static really_inline u64a extract64from128(const m128 in, unsigned imm) {
#if defined(HS_OPTIMIZE)
    return vec_extract((uint64x2_t)in, imm);
#else
    switch (imm) {
    case 0:
        return vec_extract((uint64x2_t)in, 0);
	break;
    case 1:
        return vec_extract((uint64x2_t)in, 1);
	break;
    default:
	return 0;
	break;
    }
#endif
}

static really_inline m128 low64from128(const m128 in) {
    return vec_sld(zeroes128(), in, 8);
}

static really_inline m128 high64from128(const m128 in) {
    return vec_sld(in, zeroes128(), 8);
}

static really_inline m128 add128(m128 a, m128 b) {
    return (m128) vec_add((uint64x2_t)a, (uint64x2_t)b);
}

static really_inline m128 and128(m128 a, m128 b) {
    return (m128) vec_and(a, b);
}

static really_inline m128 xor128(m128 a, m128 b) {
    return (m128) vec_xor(a, b);
}

static really_inline m128 or128(m128 a, m128 b) {
    return (m128) vec_or(a, b);
}

static really_inline m128 andnot128(m128 a, m128 b) {
    return (m128) vec_and( not128(a), b);
}

// aligned load
static really_inline m128 load128(const void *ptr) {
    assert(ISALIGNED_N(ptr, alignof(m128)));
    return (m128) vec_xl(0, (const s32 *)ptr);
}

// aligned store
static really_inline void store128(void *ptr, m128 a) {
    assert(ISALIGNED_N(ptr, alignof(m128)));
    vec_xst(a, 0, (int32_t *)ptr);
}

// unaligned load
static really_inline m128 loadu128(const void *ptr) {
    return (m128) vec_xl(0, (const s32 *)ptr);
}

// unaligned store
static really_inline void storeu128(void *ptr, m128 a) {
    vec_xst(a, 0, (s32 *)ptr);
}

// packed unaligned store of first N bytes
static really_inline
void storebytes128(void *ptr, m128 a, unsigned int n) {
    assert(n <= sizeof(a));
    memcpy(ptr, &a, n);
}

// packed unaligned load of first N bytes, pad with zero
static really_inline
m128 loadbytes128(const void *ptr, unsigned int n) {
    m128 a = zeroes128();
    assert(n <= sizeof(a));
    memcpy(&a, ptr, n);
    return a;
}


#define CASE_ALIGN_VECTORS(a, b, offset)  case offset: return (m128)vec_sld((uint8x16_t)a, (uint8x16_t)b, 16 - offset); break;

static really_really_inline
m128 palignr_imm(m128 r, m128 l, int offset) {
    switch (offset) {
    case 0: return l; break;
    CASE_ALIGN_VECTORS(r, l, 1);
    CASE_ALIGN_VECTORS(r, l, 2);
    CASE_ALIGN_VECTORS(r, l, 3);
    CASE_ALIGN_VECTORS(r, l, 4);
    CASE_ALIGN_VECTORS(r, l, 5);
    CASE_ALIGN_VECTORS(r, l, 6);
    CASE_ALIGN_VECTORS(r, l, 7);
    CASE_ALIGN_VECTORS(r, l, 8);
    CASE_ALIGN_VECTORS(r, l, 9);
    CASE_ALIGN_VECTORS(r, l, 10);
    CASE_ALIGN_VECTORS(r, l, 11);
    CASE_ALIGN_VECTORS(r, l, 12);
    CASE_ALIGN_VECTORS(r, l, 13);
    CASE_ALIGN_VECTORS(r, l, 14);
    CASE_ALIGN_VECTORS(r, l, 15);
    case 16: return r; break;
    default:
	return zeroes128();
	break;
    }
}

static really_really_inline
m128 palignr(m128 r, m128 l, int offset) {
/*#if defined(HS_OPTIMIZE)
    return (m128)vec_sld(l, r, offset);
#else*/
    return palignr_imm(r, l, offset);
//#endif
}
#undef CASE_ALIGN_VECTORS

static really_really_inline
m128 rshiftbyte_m128(m128 a, unsigned b) {
    return palignr(zeroes128(), a, b);
}

static really_really_inline
m128 lshiftbyte_m128(m128 a, unsigned b) {
    return palignr(a, zeroes128(), 16 - b);
}

static really_inline
m128 variable_byte_shift_m128(m128 in, s32 amount) {
    assert(amount >= -16 && amount <= 16);
    if (amount < 0) {
        return palignr(zeroes128(), in, -amount);
    } else {
        return palignr(in, zeroes128(), 16 - amount);
    }
}

#ifdef __cplusplus
extern "C" {
#endif
extern const u8 simd_onebit_masks[];
#ifdef __cplusplus
}
#endif

static really_inline
m128 mask1bit128(unsigned int n) {
    assert(n < sizeof(m128) * 8);
    u32 mask_idx = ((n % 8) * 64) + 95;
    mask_idx -= n / 8;
    return loadu128(&simd_onebit_masks[mask_idx]);
}

// switches on bit N in the given vector.
static really_inline
void setbit128(m128 *ptr, unsigned int n) {
    *ptr = or128(mask1bit128(n), *ptr);
}

// switches off bit N in the given vector.
static really_inline
void clearbit128(m128 *ptr, unsigned int n) {
    *ptr = andnot128(mask1bit128(n), *ptr);
}

// tests bit N in the given vector.
static really_inline
char testbit128(m128 val, unsigned int n) {
    const m128 mask = mask1bit128(n);

    return isnonzero128(and128(mask, val));
}

static really_inline
m128 pshufb_m128(m128 a, m128 b) {
    /* On Intel, if bit 0x80 is set, then result is zero, otherwise which the lane it is &0xf.
       In VSX, if >=16, then the result is from the second argument in vec_perm, otherwise it is the first.
       btranslated is the version that is converted from Intel to VSX.*/
    uint8x16_t all80 = vec_splats((uint8_t)0x80);
    uint8x16_t mask80 = (uint8x16_t) vec_and((uint8x16_t)b, all80);
    mask80 = (uint8x16_t) vec_cmpeq(mask80, all80);
    uint8x16_t btranslated = vec_sel((uint8x16_t)b, (uint8x16_t)vec_splats((int8_t)16), mask80);
    return (m128)vec_perm((uint8x16_t)a, (uint8x16_t)zeroes128(), btranslated);
}

static really_inline
m128 max_u8_m128(m128 a, m128 b) {
    return (m128) vec_max((uint8x16_t)a, (uint8x16_t)b);
}

static really_inline
m128 min_u8_m128(m128 a, m128 b) {
    return (m128) vec_min((uint8x16_t)a, (uint8x16_t)b);
}

static really_inline
m128 sadd_u8_m128(m128 a, m128 b) {
    return (m128) vec_adds((uint8x16_t)a, (uint8x16_t)b);
}

static really_inline
m128 sub_u8_m128(m128 a, m128 b) {
    return (m128) vec_sub((uint8x16_t)a, (uint8x16_t)b);
}

static really_inline
m128 set4x32(u32 x3, u32 x2, u32 x1, u32 x0) {
    u32 ALIGN_ATTR(16) data[4] = { x0, x1, x2, x3 };
    return (m128) vec_xl(0, (u32 *) data);
}

static really_inline
m128 set2x64(u64a hi, u64a lo) {
    u64a ALIGN_ATTR(16) data[2] = { lo, hi };
    return (m128) vec_xl(0, (u64a *) data);
}

#endif // ARCH_ARM_SIMD_UTILS_H
