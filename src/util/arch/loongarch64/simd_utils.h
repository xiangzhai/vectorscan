/*
 * Copyright (c) 2015-2020, Intel Corporation
 * Copyright (c) 2020-2021, VectorCamp PC
 * Copyright (c) 2023, Loongson Technology
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

#ifndef ARCH_LOONGARCH64_SIMD_UTILS_H
#define ARCH_LOONGARCH64_SIMD_UTILS_H

#include <stdio.h>
#include <stdbool.h>

#include "ue2common.h"
#include "util/simd_types.h"
#include "util/unaligned.h"
#include "util/intrinsics.h"

#include <string.h> // for memcpy

static really_inline m128 ones128(void) {
    return __lsx_vreplgr2vr_b(0xFF);
}

static really_inline m128 zeroes128(void) {
    return __lsx_vreplgr2vr_w(0);
}

/** \brief Bitwise not for m128*/
static really_inline m128 not128(m128 a) {
    return __lsx_vxor_v(a, ones128());
}

/** \brief Return 1 if a and b are different otherwise 0 */
static really_inline int diff128(m128 a, m128 b) {
    uint64_t res = __lsx_vpickve2gr_du(__lsx_vsrlni_b_h(zeroes128(), __lsx_vseq_w(a, b), 4), 0);
    return (~0ull != res);
}

static really_inline int isnonzero128(m128 a) {
    return diff128(a, zeroes128());
}

/**
 * "Rich" version of diff128(). Takes two vectors a and b and returns a 4-bit
 * mask indicating which 32-bit words contain differences.
 */
static really_inline u32 diffrich128(m128 a, m128 b) {
    static const v4u32 movemask = { 1, 2, 4, 8 };
    m128 tmp = __lsx_vand_v(not128(__lsx_vseq_w(a, b)), movemask);
    return __lsx_vpickve2gr_wu(tmp, 0) + __lsx_vpickve2gr_wu(tmp, 1) +
           __lsx_vpickve2gr_wu(tmp, 2) + __lsx_vpickve2gr_wu(tmp, 3);
}

/**
 * "Rich" version of diff128(), 64-bit variant. Takes two vectors a and b and
 * returns a 4-bit mask indicating which 64-bit words contain differences.
 */
static really_inline u32 diffrich64_128(m128 a, m128 b) {
    static const v2u64 movemask = { 1, 4 };
    m128 tmp = __lsx_vand_v(not128(__lsx_vseq_d(a, b)), movemask);
    return __lsx_vpickve2gr_du(tmp, 0) + __lsx_vpickve2gr_du(tmp, 1);
}

static really_really_inline
m128 add_2x64(m128 a, m128 b) {
    return __lsx_vadd_d(a, b);
}

static really_really_inline
m128 sub_2x64(m128 a, m128 b) {
    return __lsx_vsub_d(a, b);
}

static really_inline
m128 lshift_m128(m128 a, unsigned b) {
#if defined(HAVE__BUILTIN_CONSTANT_P)
    if (__builtin_constant_p(b)) {
        return __lsx_vslli_w(a, b);
    }
#endif
  v4i32_w shift_indices = __lsx_vreplgr2vr_w(b);
  return __lsx_vsll_w(a, shift_indices);
}

static really_really_inline
m128 rshift_m128(m128 a, unsigned b) {
#if defined(HAVE__BUILTIN_CONSTANT_P)
    if (__builtin_constant_p(b)) {
        return __lsx_vsrli_w(a, b);
    }
#endif
  v4i32 shift_indices = __lsx_vreplgr2vr_w(b);
  return __lsx_vsrl_w(a, shift_indices);
}

static really_really_inline
m128 lshift64_m128(m128 a, unsigned b) {
#if defined(HAVE__BUILTIN_CONSTANT_P)
    if (__builtin_constant_p(b)) {
        return __lsx_vslli_d(a, b);
    }
#endif
  v2i64 shift_indices = __lsx_vreplgr2vr_d(b);
  return __lsx_vsll_d(a, shift_indices);
}

static really_really_inline
m128 rshift64_m128(m128 a, unsigned b) {
#if defined(HAVE__BUILTIN_CONSTANT_P)
    if (__builtin_constant_p(b)) {
        return __lsx_vsrl_d(a, b);
    }
#endif
  v2i64 shift_indices = __lsx_vreplgr2vr_d(b);
  return __lsx_vsrl_d(a, shift_indices);
}

static really_inline m128 eq128(m128 a, m128 b) {
    return __lsx_vseq_b(a, b);
}

static really_inline m128 eq64_m128(m128 a, m128 b) {
    return __lsx_vseq_d(a, b);
}

static really_inline u32 movemask128(m128 a) {
    v16u8 input = (v16u8) a;
    v8u16 high_bits = (v8u16) __lsx_vsrli_b(input, 7);
    v4u32 paired16  = (v4u32) __lsx_vadd_h(high_bits, __lsx_vsrli_h(high_bits, 7));
    v2u64 paired32  = (v2u64) __lsx_vadd_w(paired16,  __lsx_vsrli_w(paired16, 14));
    v16u8 paired64  = (v16u8) __lsx_vadd_d(paired32,  __lsx_vsrli_d(paired32, 28));
    return __lsx_vpickve2gr_bu(paired64, 0) | ((int) __lsx_vpickve2gr_bu(paired64, 8) << 8);
}

static really_inline m128 set1_16x8(u8 c) {
    return __lsx_vreplgr2vr_b(c);
}

static really_inline m128 set1_4x32(u32 c) {
    return __lsx_vreplgr2vr_w(c);
}

static really_inline m128 set1_2x64(u64a c) {
    return __lsx_vreplgr2vr_d(c);
}

static really_inline u32 movd(const m128 in) {
    return __lsx_vpickve2gr_wu(in, 0);
}

static really_inline u64a movq(const m128 in) {
    return __lsx_vpickve2gr_du(in, 0);
}

/* another form of movq */
static really_inline
m128 load_m128_from_u64a(const u64a *p) {
    m128 tmp = zeroes128();
    return __lsx_vinsgr2vr_d(tmp, *p, 0);
}

static really_inline u32 extract32from128(const m128 in, unsigned imm) {
#if defined(HAVE__BUILTIN_CONSTANT_P)
    if (__builtin_constant_p(imm)) {
        return __lsx_vpickve2gr_wu(in, imm);
    }
#endif
    switch (imm) {
    case 0:
        return __lsx_vpickve2gr_wu(in, 0);
	break;
    case 1:
        return __lsx_vpickve2gr_wu(in, 1);
	break;
    case 2:
        return __lsx_vpickve2gr_wu(in, 2);
	break;
    case 3:
        return __lsx_vpickve2gr_wu(in, 3);
	break;
    default:
	return 0;
	break;
    }
}

static really_inline u64a extract64from128(const m128 in, unsigned imm) {
#if defined(HAVE__BUILTIN_CONSTANT_P)
    if (__builtin_constant_p(imm)) {
        return __lsx_vpickve2gr_du(in, imm);
    }
#endif
    switch (imm) {
    case 0:
        return __lsx_vpickve2gr_du(in, 0);
	break;
    case 1:
        return __lsx_vpickve2gr_du(in, 1);
	break;
    default:
	return 0;
	break;
    }
}

static really_inline m128 low64from128(const m128 in) {
    m128 ret = zeroes128();
    __lsx_vinsgr2vr_d(ret, __lsx_vpickve2gr_d(in, 0), 0);
    return ret;
}

static really_inline m128 high64from128(const m128 in) {
    m128 ret = zeroes128();
    __lsx_vinsgr2vr_d(ret, __lsx_vpickve2gr_d(in, 1), 0);
    return ret;
}

static really_inline m128 add128(m128 a, m128 b) {
    return __lsx_vadd_q(a, b);
}

static really_inline m128 and128(m128 a, m128 b) {
    return __lsx_vand_v(a, b);
}

static really_inline m128 xor128(m128 a, m128 b) {
    return __lsx_vxor_v(a, b);
}

static really_inline m128 or128(m128 a, m128 b) {
    return __lsx_vor_v(a, b);
}

static really_inline m128 andnot128(m128 a, m128 b) {
    return __lsx_vandn_v(a, b);
}

// aligned load
static really_inline m128 load128(const void *ptr) {
    assert(ISALIGNED_N(ptr, alignof(m128)));
    return __lsx_vld((const int32_t *)ptr, 0);
}

// aligned store
static really_inline void store128(void *ptr, m128 a) {
    assert(ISALIGNED_N(ptr, alignof(m128)));
    __lsx_vst(a, (int32_t *)ptr, 0);
}

// unaligned load
static really_inline m128 loadu128(const void *ptr) {
    return __lsx_vld((const int32_t *)ptr, 0);
}

// unaligned store
static really_inline void storeu128(void *ptr, m128 a) {
    __lsx_vst(a, (int32_t *)ptr, 0);
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

static really_inline m128 case_algin_vectors(m128 a,m128 b,int offset) {
    u8 index_shuf[16];
    for(int i = 0; i < 16; i++) {
        index_shuf[i] = (uint8_t)offset;
        offset += 1;
    }
    v16u8 index = __lsx_vld((uint8_t *)index_shuf, 0);
    return __lsx_vshuf_b(b, a, index);
}
static really_really_inline
m128 palignr_imm(m128 r, m128 l, int offset) {
    switch (offset) {
    case  0: return l; break;
    case  1: return case_algin_vectors(l, r, 1);  break;
    case  2: return case_algin_vectors(l, r, 2);  break;
    case  3: return case_algin_vectors(l, r, 3);  break;
    case  4: return case_algin_vectors(l, r, 4);  break;
    case  5: return case_algin_vectors(l, r, 5);  break;
    case  6: return case_algin_vectors(l, r, 6);  break;
    case  7: return case_algin_vectors(l, r, 7);  break;
    case  8: return case_algin_vectors(l, r, 8);  break;
    case  9: return case_algin_vectors(l, r, 9);  break;
    case 10: return case_algin_vectors(l, r, 10); break;
    case 11: return case_algin_vectors(l, r, 11); break;
    case 12: return case_algin_vectors(l, r, 12); break;
    case 13: return case_algin_vectors(l, r, 13); break;
    case 14: return case_algin_vectors(l, r, 14); break;
    case 15: return case_algin_vectors(l, r, 15); break;
    case 16: return r; break;
    default:
	return zeroes128();
	break;
    }
}

static really_really_inline
m128 palignr(m128 r, m128 l, int offset) {

#if defined(HAVE__BUILTIN_CONSTANT_P)
    u8 index_shuf[16];
    for (int i = 0; i < 16; i++) {
        index_shuf[i] = (uint8_t)offset;
        offset += 1;
    }
    v16u8 index = __lsx_vld((uint8_t *)index_shuf, 0);
    if (__builtin_constant_p(index)) {
        return __lsx_vshuf_b(r, l, index);
    }
#endif
    return palignr_imm(r, l, offset);
}
//#undef CASE_ALIGN_VECTORS

static really_really_inline
m128 rshiftbyte_m128(m128 a, unsigned b) {
    if (b == 0) {
        return a;
    }
    return palignr(zeroes128(), a, b);
}

static really_really_inline
m128 lshiftbyte_m128(m128 a, unsigned b) {
    if (b == 0) {
        return a;
    }
    return palignr(a, zeroes128(), 16 - b);
}

static really_inline
m128 variable_byte_shift_m128(m128 in, s32 amount) {
    assert(amount >= -16 && amount <= 16);
    if (amount < 0) {
        return palignr_imm(zeroes128(), in, -amount);
    } else {
        return palignr_imm(in, zeroes128(), 16 - amount);
    }
}

static really_inline
m128 mask1bit128(unsigned int n) {
    assert(n < sizeof(m128) * 8);
    static m128 onebit = { 1, 0 };
    m128 mask = lshiftbyte_m128( onebit, n / 8 );
    return lshift64_m128( mask, n % 8 );
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
     v16u8 tmp = __lsx_vand_v((v16u8)b,__lsx_vreplgr2vr_b(0x8f));
     return __lsx_vshuf_b(zeroes128(),a, tmp);
}

static really_inline
m128 max_u8_m128(m128 a, m128 b) {
    return __lsx_vmax_bu(a, b);
}

static really_inline
m128 min_u8_m128(m128 a, m128 b) {
    return __lsx_vmin_bu(a, b);
}

static really_inline
m128 sadd_u8_m128(m128 a, m128 b) {
    return __lsx_vsadd_bu(a, b);
}

static really_inline
m128 sub_u8_m128(m128 a, m128 b) {
    return __lsx_vssub_bu(a, b);
}

static really_inline
m128 set4x32(u32 x3, u32 x2, u32 x1, u32 x0) {
    uint32_t ALIGN_ATTR(16) data[4] = { x0, x1, x2, x3 };
    return __lsx_vld((uint32_t *) data, 0);
}

static really_inline
m128 set2x64(u64a hi, u64a lo) {
    uint64_t ALIGN_ATTR(16) data[2] = { lo, hi };
    return __lsx_vld((uint64_t *) data, 0);
}

#endif // ARCH_LOONGARCH64_SIMD_UTILS_H
