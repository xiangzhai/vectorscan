/*
 * Copyright (c) 2015-2017, Intel Corporation
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

#ifndef SIMD_IMPL_HPP
#define SIMD_IMPL_HPP

#include <cstdint>

#include "ue2common.h"
#include "util/supervector/supervector.hpp"

// 128-bit LSX implementation

template<>
really_inline SuperVector<16>::SuperVector(typename base_type::type const v)
{
    u.v128[0] = v;
}

template<>
template<>
really_inline SuperVector<16>::SuperVector(v8i16_h other)
{
    u.s8x16[0] = other;
}

template<>
template<>
really_inline SuperVector<16>::SuperVector(v8u16_h other)
{
    u.u8x16[0] = other;
}

template<>
template<>
really_inline SuperVector<16>::SuperVector(v16i8_b other)
{
    u.s16x8[0] = other;
}

template<>
template<>
really_inline SuperVector<16>::SuperVector(v16u8_b other)
{
    u.u16x8[0] = other;
}

template<>
template<>
really_inline SuperVector<16>::SuperVector(v4i32_w other)
{
    u.s32x4[0] = other;
}

template<>
template<>
really_inline SuperVector<16>::SuperVector(v4u32_w other)
{
    u.u32x4[0] = other;
}

template<>
template<>
really_inline SuperVector<16>::SuperVector(v2i64_d other)
{
    u.s64x2[0] = other;
}

template<>
template<>
really_inline SuperVector<16>::SuperVector(v2u64_d other)
{
    u.u64x2[0] = other;
}

template<>
template<>
really_inline SuperVector<16>::SuperVector(int8_t const other)
{
    u.s8x16[0] = __lsx_vreplgr2vr_b(other);
}

template<>
template<>
really_inline SuperVector<16>::SuperVector(uint8_t const other)
{
    u.u8x16[0] = (v16u8)__lsx_vreplgr2vr_b(other);
}

template<>
template<>
really_inline SuperVector<16>::SuperVector(int16_t const other)
{
    u.s16x8[0] = __lsx_vreplgr2vr_h(other);
}

template<>
template<>
really_inline SuperVector<16>::SuperVector(uint16_t const other)
{
    u.u16x8[0] = (v8u16)__lsx_vreplgr2vr_h(other);
}

template<>
template<>
really_inline SuperVector<16>::SuperVector(int32_t const other)
{
    u.s32x4[0] = __lsx_vreplgr2vr_w(other);
}

template<>
template<>
really_inline SuperVector<16>::SuperVector(uint32_t const other)
{
    u.u32x4[0] = (v4u32)__lsx_vreplgr2vr_w(other);
}

template<>
template<>
really_inline SuperVector<16>::SuperVector(int64_t const other)
{
    u.s64x2[0] = __lsx_vreplgr2vr_d(other);
}

template<>
template<>
really_inline SuperVector<16>::SuperVector(uint64_t const other)
{
    u.u64x2[0] = (v2u64)__lsx_vreplgr2vr_d(other);
}

// Constants
template<>
really_inline SuperVector<16> SuperVector<16>::Ones(void)
{
    return {__lsx_vreplgr2vr_b(0xFF)};
}

template<>
really_inline SuperVector<16> SuperVector<16>::Zeroes(void)
{
    return {__lsx_vreplgr2vr_b(0)};
}

// Methods

template <>
really_inline void SuperVector<16>::operator=(SuperVector<16> const &other)
{
    u.v128[0] = other.u.v128[0];
}

template <>
really_inline SuperVector<16> SuperVector<16>::operator&(SuperVector<16> const &b) const
{
    return {__lsx_vand_v(u.u8x16[0], b.u.u8x16[0])};
}

template <>
really_inline SuperVector<16> SuperVector<16>::operator|(SuperVector<16> const &b) const
{
    return {__lsx_vor_v(u.u8x16[0], b.u.u8x16[0])};
}

template <>
really_inline SuperVector<16> SuperVector<16>::operator^(SuperVector<16> const &b) const
{
    return {__lsx_vxor_v(u.u8x16[0], b.u.u8x16[0])};
}

template <>
really_inline SuperVector<16> SuperVector<16>::operator!() const
{
    return {__lsx_vnor_v(u.u8x16[0], u.u8x16[0])};
}

template <>
really_inline SuperVector<16> SuperVector<16>::opandnot(SuperVector<16> const &b) const
{
    return {__lsx_vand_v(__lsx_vnor_v(u.u8x16[0], u.u8x16[0]), b.u.u8x16[0])};
}

template <>
really_inline SuperVector<16> SuperVector<16>::operator==(SuperVector<16> const &b) const
{
    return {__lsx_vseq_b(u.u8x16[0], b.u.u8x16[0])};
}

template <>
really_inline SuperVector<16> SuperVector<16>::operator!=(SuperVector<16> const &b) const
{
    return !(*this == b);
}

template <>
really_inline SuperVector<16> SuperVector<16>::operator>(SuperVector<16> const &b) const
{
    return {__lsx_vslt_b(b.u.s8x16[0], u.s8x16[0])};
}

template <>
really_inline SuperVector<16> SuperVector<16>::operator>=(SuperVector<16> const &b) const
{
    return {__lsx_vsle_bu(b.u.u8x16[0], u.u8x16[0])};
}

template <>
really_inline SuperVector<16> SuperVector<16>::operator<(SuperVector<16> const &b) const
{
    return {__lsx_vslt_b(u.s8x16[0], b.u.s8x16[0])};
}

template <>
really_inline SuperVector<16> SuperVector<16>::operator<=(SuperVector<16> const &b) const
{
    return {__lsx_vsle_b(u.s8x16[0], b.u.s8x16[0])};
}

template <>
really_inline SuperVector<16> SuperVector<16>::eq(SuperVector<16> const &b) const
{
    return (*this == b);
}

template <>
really_inline typename SuperVector<16>::comparemask_type
SuperVector<16>::comparemask(void) const {
    return static_cast<typename SuperVector<16>::comparemask_type>(
        __lsx_vpickve2gr_du(__lsx_vsrlni_b_h(__lsx_vreplgr2vr_w(0), u.u16x8[0], 4), 0));
}

template <>
really_inline typename SuperVector<16>::comparemask_type
SuperVector<16>::eqmask(SuperVector<16> const b) const {
    return eq(b).comparemask();
}

template <> really_inline u32 SuperVector<16>::mask_width() { return 4; }

template <>
really_inline typename SuperVector<16>::comparemask_type
SuperVector<16>::iteration_mask(
    typename SuperVector<16>::comparemask_type mask) {
    return mask & 0x1111111111111111ull;
}

template <>
template<uint8_t N>
really_inline SuperVector<16> SuperVector<16>::vshl_8_imm() const
{
    return {__lsx_vslli_b(u.u8x16[0], N)};
}

template <>
template<uint8_t N>
really_inline SuperVector<16> SuperVector<16>::vshl_16_imm() const
{
    return {__lsx_vslli_h(u.u16x8[0], N)};
}

template <>
template<uint8_t N>
really_inline SuperVector<16> SuperVector<16>::vshl_32_imm() const
{
    return {__lsx_vslli_w(u.u32x4[0], N)};
}

template <>
template<uint8_t N>
really_inline SuperVector<16> SuperVector<16>::vshl_64_imm() const
{
    return {__lsx_vslli_d(u.u64x2[0], N)};
}

static really_inline m128 create_index(int offset){
    u8 index_shuf[16];
    for (int i = 0; i < 16; i++) {
        index_shuf[i] = (uint8_t)offset;
        offset += 1;
    }
    v16u8 index = __lsx_vld((uint8_t *)index_shuf,0);
    return index;
}


template <>
template<uint8_t N>
really_inline SuperVector<16> SuperVector<16>::vshl_128_imm() const
{
    return {__lsx_vshuf_b(u.u8x16[0], __lsx_vreplgr2vr_b(0), create_index(16 - N))};
}

template <>
template<uint8_t N>
really_inline SuperVector<16> SuperVector<16>::vshl_imm() const
{
    return vshl_128_imm<N>();
}

template <>
template<uint8_t N>
really_inline SuperVector<16> SuperVector<16>::vshr_8_imm() const
{
    return {__lsx_vsrli_b(u.u8x16[0], N)};
}

template <>
template<uint8_t N>
really_inline SuperVector<16> SuperVector<16>::vshr_16_imm() const
{
    return {__lsx_vsrli_h(u.u16x8[0], N)};
}

template <>
template<uint8_t N>
really_inline SuperVector<16> SuperVector<16>::vshr_32_imm() const
{
    return {__lsx_vsrli_w(u.u32x4[0], N)};
}

template <>
template<uint8_t N>
really_inline SuperVector<16> SuperVector<16>::vshr_64_imm() const
{
    return {__lsx_vsrli_d(u.u64x2[0], N)};
}

template <>
template<uint8_t N>
really_inline SuperVector<16> SuperVector<16>::vshr_128_imm() const
{
    return {__lsx_vshuf_b(__lsx_vreplgr2vr_b(0), u.u8x16[0], create_index(N))};
}

template <>
template<uint8_t N>
really_inline SuperVector<16> SuperVector<16>::vshr_imm() const
{
    return vshr_128_imm<N>();
}

#if !defined(HS_OPTIMIZE)
template SuperVector<16> SuperVector<16>::vshl_8_imm<4>() const;
template SuperVector<16> SuperVector<16>::vshl_16_imm<1>() const;
template SuperVector<16> SuperVector<16>::vshl_64_imm<1>() const;
template SuperVector<16> SuperVector<16>::vshl_64_imm<4>() const;
template SuperVector<16> SuperVector<16>::vshl_128_imm<1>() const;
template SuperVector<16> SuperVector<16>::vshl_128_imm<4>() const;
template SuperVector<16> SuperVector<16>::vshr_8_imm<1>() const;
template SuperVector<16> SuperVector<16>::vshr_8_imm<4>() const;
template SuperVector<16> SuperVector<16>::vshr_16_imm<1>() const;
template SuperVector<16> SuperVector<16>::vshr_64_imm<1>() const;
template SuperVector<16> SuperVector<16>::vshr_64_imm<4>() const;
template SuperVector<16> SuperVector<16>::vshr_128_imm<1>() const;
template SuperVector<16> SuperVector<16>::vshr_128_imm<4>() const;
#endif

template <>
really_inline SuperVector<16> SuperVector<16>::vshl_8  (uint8_t const N) const
{
    if (N == 0) return *this;
    if (N == 8) return Zeroes();
    v16i8 shift_indices = __lsx_vreplgr2vr_b(N);
    return { __lsx_vsll_b(u.s8x16[0], shift_indices) };
}

template <>
really_inline SuperVector<16> SuperVector<16>::vshl_16 (uint8_t const N) const
{
    if (N == 0) return *this;
    if (N == 16) return Zeroes();
    v8i16 shift_indices = __lsx_vreplgr2vr_h(N);
    return { __lsx_vsll_h(u.s16x8[0], shift_indices) };
}

template <>
really_inline SuperVector<16> SuperVector<16>::vshl_32 (uint8_t const N) const
{
    if (N == 0) return *this;
    if (N == 32) return Zeroes();
    v4i32 shift_indices = __lsx_vreplgr2vr_w(N);
    return { __lsx_vsll_w(u.s32x4[0], shift_indices) };
}

template <>
really_inline SuperVector<16> SuperVector<16>::vshl_64 (uint8_t const N) const
{
    if (N == 0) return *this;
    if (N == 64) return Zeroes();
    v2i64 shift_indices = __lsx_vreplgr2vr_d(N);
    return { __lsx_vsll_d(u.s64x2[0], shift_indices) };
}

template <>
really_inline SuperVector<16> SuperVector<16>::vshl_128(uint8_t const N) const
{
    if (N == 0) return *this;
    if (N == 16) return Zeroes();
#if defined(HAVE__BUILTIN_CONSTANT_P)
    u8 index_shuf[16];
    for(int i = 0; i < 16; i++) {
        index_shuf[i] = (uint8_t)(16-N);
        offset += 1;
    }
    v16u8 index = __lsx_vld((uint8_t *)index_shuf, 0);
    if (__builtin_constant_p(index)) {
        return {__lsx_vshuf_b(u.u8x16[0], __lsx_vreplgr2vr_b(0), index)};
    }
#endif
    SuperVector result;
    Unroller<1, 16>::iterator([&,v=this](auto const i) { constexpr uint8_t n = i.value; if (N == n) result = {__lsx_vshuf_b(v->u.u8x16[0], __lsx_vreplgr2vr_b(0), create_index(16 - n))}; });
    return result;
}

template <>
really_inline SuperVector<16> SuperVector<16>::vshl(uint8_t const N) const
{
    return vshl_128(N);
}

template <>
really_inline SuperVector<16> SuperVector<16>::vshr_8  (uint8_t const N) const
{
    if (N == 0) return *this;
    if (N == 8) return Zeroes();
    v16i8 shift_indices = __lsx_vreplgr2vr_b(N);
    return { __lsx_vsrl_b(u.s8x16[0], shift_indices) };
}

template <>
really_inline SuperVector<16> SuperVector<16>::vshr_16 (uint8_t const N) const
{
    if (N == 0) return *this;
    if (N == 16) return Zeroes();
    v8i16 shift_indices = __lsx_vreplgr2vr_h(N);
    return { __lsx_vsrl_h(u.s16x8[0], shift_indices) };
}

template <>
really_inline SuperVector<16> SuperVector<16>::vshr_32 (uint8_t const N) const
{
    if (N == 0) return *this;
    if (N == 32) return Zeroes();
    v4i32 shift_indices = __lsx_vreplgr2vr_w(N);
    return { __lsx_vsrl_w(u.s32x4[0], shift_indices) };
}

template <>
really_inline SuperVector<16> SuperVector<16>::vshr_64 (uint8_t const N) const
{
    if (N == 0) return *this;
    if (N == 64) return Zeroes();
    v2i64 shift_indices = __lsx_vreplgr2vr_d(N);
    return { __lsx_vsrl_d(u.s64x2[0], shift_indices) };
}

template <>
really_inline SuperVector<16> SuperVector<16>::vshr_128(uint8_t const N) const
{
    if (N == 0) return *this;
    if (N == 16) return Zeroes();
#if defined(HAVE__BUILTIN_CONSTANT_P)
    u8 index_shuf[16];
    for (int i = 0; i < 16; i++) {
        index_shuf[i] = (uint8_t)N;
        offset += 1;
    }
    v16u8 index = __lsx_vld((uint8_t *)index_shuf, 0);
    if (__builtin_constant_p(index)) {
        return {__lsx_vshuf_b(__lsx_vreplgr2vr_b(0), u.u8x16[0], index)};
    }
#endif
    SuperVector result;
    Unroller<1, 16>::iterator([&,v=this](auto const i) { constexpr uint8_t n = i.value; if (N == n) result = {__lsx_vshuf_b(__lsx_vreplgr2vr_b(0), v->u.u8x16[0], create_index(n))}; });
    return result;
}

template <>
really_inline SuperVector<16> SuperVector<16>::vshr(uint8_t const N) const
{
    return vshr_128(N);
}

template <>
really_inline SuperVector<16> SuperVector<16>::operator>>(uint8_t const N) const
{
    return vshr_128(N);
}

template <>
really_inline SuperVector<16> SuperVector<16>::operator<<(uint8_t const N) const
{
    return vshl_128(N);
}

template<>
really_inline SuperVector<16> SuperVector<16>::Ones_vshr(uint8_t const N)
{
    return Ones().vshr_128(N);
}

template<>
really_inline SuperVector<16> SuperVector<16>::Ones_vshl(uint8_t const N)
{
    return Ones().vshl_128(N);
}

template <>
really_inline SuperVector<16> SuperVector<16>::loadu(void const *ptr)
{
    return {__lsx_vld((const int32_t *)ptr, 0)};
}

template <>
really_inline SuperVector<16> SuperVector<16>::load(void const *ptr)
{
    assert(ISALIGNED_N(ptr, alignof(SuperVector::size)));
    ptr = vectorscan_assume_aligned(ptr, SuperVector::size);
    return {__lsx_vld((const int32_t *)ptr, 0)};
}

template <>
really_inline SuperVector<16> SuperVector<16>::loadu_maskz(void const *ptr, uint8_t const len)
{
    SuperVector mask = Ones_vshr(16 - len);
    SuperVector<16> v = loadu(ptr);
    return mask & v;
}

template<>
really_inline SuperVector<16> SuperVector<16>::alignr(SuperVector<16> &other, int8_t offset)
{
    if (offset == 0) return other;
    if (offset == 16) return *this;
#if defined(HAVE__BUILTIN_CONSTANT_P)
    u8 index_shuf[16];
    for (int i = 0; i < 16; i++) {
        index_shuf[i] = (uint8_t)offset;
        offset += 1;
    }
    v16u8 index = __lsx_vld((uint8_t *)index_shuf, 0);
    if (__builtin_constant_p(index)) {
        return {__lsx_vshuf_b(u.u8x16[0], other.u.u8x16[0], index)};
    }
#endif
    SuperVector result;
    Unroller<1, 16>::iterator([&,v=this](auto const i) { constexpr uint8_t n = i.value; if (offset == n) result = {__lsx_vshuf_b(v->u.u8x16[0], other.u.u8x16[0], create_index(n))}; });
    return result;
}

template<>
template<>
really_inline SuperVector<16> SuperVector<16>::pshufb<false>(SuperVector<16> b)
{
    return {__lsx_vshuf_b(__lsx_vreplgr2vr_b(0), u.u8x16[0], b.u.u8x16[0])};
}

template<>
template<>
really_inline SuperVector<16> SuperVector<16>::pshufb<true>(SuperVector<16> b)
{
    /* On Intel, if bit 0x80 is set, then result is zero, otherwise which the lane it is &0xf.
       In LOONGARCH, if >=16, then the result is zero, otherwise it is that lane.
       btranslated is the version that is converted from Intel to LOONGARCH.  */
    SuperVector<16> btranslated = b & SuperVector<16>::dup_s8(0x8f);
    return pshufb<false>(btranslated);
}

template<>
really_inline SuperVector<16> SuperVector<16>::pshufb_maskz(SuperVector<16> b, uint8_t const len)
{
    SuperVector mask = Ones_vshr(16 -len);
    return mask & pshufb(b);
}

#endif // SIMD_IMPL_HPP
