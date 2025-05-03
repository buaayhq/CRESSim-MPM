/*
 * BSD 3-Clause License
 *
 * Copyright (c) 2025, Yafei Ou and Mahdi Tavakoli
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived from
 *    this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef CR_MATH_HELPERS_H
#define CR_MATH_HELPERS_H

#include "cuda_compat.h"
#include "preprocessor.h"
#include "vec3.h"

/* In case, undef min/max */
#ifdef min
#undef min
#endif

#ifdef max
#undef max
#endif

namespace crmpm
{
    /*-------------------------*/
    // float4 basic operators
    /*-------------------------*/

    // float4 + float4
    CR_FORCE_INLINE CR_CUDA_HOST CR_CUDA_DEVICE float4 operator+(const float4 &lhs, const float4 &rhs)
    {
        return make_float4(lhs.x + rhs.x, lhs.y + rhs.y, lhs.z + rhs.z, lhs.w + rhs.w);
    }

    // float4 - float4
    CR_FORCE_INLINE CR_CUDA_HOST CR_CUDA_DEVICE float4 operator-(const float4 &lhs, const float4 &rhs)
    {
        return make_float4(lhs.x - rhs.x, lhs.y - rhs.y, lhs.z - rhs.z, lhs.w - rhs.w);
    }

    // float4 * float4
    CR_FORCE_INLINE CR_CUDA_HOST CR_CUDA_DEVICE float4 operator*(const float4 &lhs, const float4 &rhs)
    {
        return make_float4(lhs.x * rhs.x, lhs.y * rhs.y, lhs.z * rhs.z, lhs.w * rhs.w);
    }

    // float4 / float4
    CR_FORCE_INLINE CR_CUDA_HOST CR_CUDA_DEVICE float4 operator/(const float4 &lhs, const float4 &rhs)
    {
        return make_float4(lhs.x / rhs.x, lhs.y / rhs.y, lhs.z / rhs.z, lhs.w / rhs.w);
    }

    // float4 + float
    CR_FORCE_INLINE CR_CUDA_HOST CR_CUDA_DEVICE float4 operator+(const float4 &lhs, const float &rhs)
    {
        return make_float4(lhs.x + rhs, lhs.y + rhs, lhs.z + rhs, lhs.w + rhs);
    }

    // float + float4
    CR_FORCE_INLINE CR_CUDA_HOST CR_CUDA_DEVICE float4 operator+(const float &lhs, const float4 &rhs)
    {
        return make_float4(lhs + rhs.x, lhs + rhs.y, lhs + rhs.z, lhs + rhs.w);
    }

    // float4 * float
    CR_FORCE_INLINE CR_CUDA_HOST CR_CUDA_DEVICE float4 operator*(const float4 &lhs, const float &rhs)
    {
        return make_float4(lhs.x * rhs, lhs.y * rhs, lhs.z * rhs, lhs.w * rhs);
    }

    // float * float4
    CR_FORCE_INLINE CR_CUDA_HOST CR_CUDA_DEVICE float4 operator*(const float &lhs, const float4 &rhs)
    {
        return make_float4(lhs * rhs.x, lhs * rhs.y, lhs * rhs.z, lhs * rhs.w);
    }

    // float4 / float
    CR_FORCE_INLINE CR_CUDA_HOST CR_CUDA_DEVICE float4 operator/(const float4 &lhs, const float &rhs)
    {
        return make_float4(lhs.x / rhs, lhs.y / rhs, lhs.z / rhs, lhs.w / rhs);
    }

    // float / float4
    CR_FORCE_INLINE CR_CUDA_HOST CR_CUDA_DEVICE float4 operator/(const float &lhs, const float4 &rhs)
    {
        return make_float4(lhs / rhs.x, lhs / rhs.y, lhs / rhs.z, lhs / rhs.w);
    }

    CR_FORCE_INLINE CR_CUDA_HOST CR_CUDA_DEVICE float4 &operator+=(float4 &lhs, const float4 &rhs)
    {
        lhs.x += rhs.x;
        lhs.y += rhs.y;
        lhs.z += rhs.z;
        lhs.w += rhs.w;
        return lhs;
    }

    CR_FORCE_INLINE CR_CUDA_HOST CR_CUDA_DEVICE float4 &operator-=(float4 &lhs, const float4 &rhs)
    {
        lhs.x -= rhs.x;
        lhs.y -= rhs.y;
        lhs.z -= rhs.z;
        lhs.w -= rhs.w;
        return lhs;
    }

    /*-------------------------*/
    // float3 basic operators
    /*-------------------------*/

    // float3 + float3
    CR_FORCE_INLINE CR_CUDA_HOST CR_CUDA_DEVICE float3 operator+(const float3 &lhs, const float3 &rhs)
    {
        return make_float3(lhs.x + rhs.x, lhs.y + rhs.y, lhs.z + rhs.z);
    }

    // float3 - float3
    CR_FORCE_INLINE CR_CUDA_HOST CR_CUDA_DEVICE float3 operator-(const float3 &lhs, const float3 &rhs)
    {
        return make_float3(lhs.x - rhs.x, lhs.y - rhs.y, lhs.z - rhs.z);
    }

    // float3 * float3
    CR_FORCE_INLINE CR_CUDA_HOST CR_CUDA_DEVICE float3 operator*(const float3 &lhs, const float3 &rhs)
    {
        return make_float3(lhs.x * rhs.x, lhs.y * rhs.y, lhs.z * rhs.z);
    }

    // float3 / float3
    CR_FORCE_INLINE CR_CUDA_HOST CR_CUDA_DEVICE float3 operator/(const float3 &lhs, const float3 &rhs)
    {
        return make_float3(lhs.x / rhs.x, lhs.y / rhs.y, lhs.z / rhs.z);
    }

    // float3 + float
    CR_FORCE_INLINE CR_CUDA_HOST CR_CUDA_DEVICE float3 operator+(const float3 &lhs, const float &rhs)
    {
        return make_float3(lhs.x + rhs, lhs.y + rhs, lhs.z + rhs);
    }

    // float + float3
    CR_FORCE_INLINE CR_CUDA_HOST CR_CUDA_DEVICE float3 operator+(const float &lhs, const float3 &rhs)
    {
        return make_float3(lhs + rhs.x, lhs + rhs.y, lhs + rhs.z);
    }

    // float3 * float
    CR_FORCE_INLINE CR_CUDA_HOST CR_CUDA_DEVICE float3 operator*(const float3 &lhs, const float &rhs)
    {
        return make_float3(lhs.x * rhs, lhs.y * rhs, lhs.z * rhs);
    }

    // float * float3
    CR_FORCE_INLINE CR_CUDA_HOST CR_CUDA_DEVICE float3 operator*(const float &lhs, const float3 &rhs)
    {
        return make_float3(lhs * rhs.x, lhs * rhs.y, lhs * rhs.z);
    }

    // float3 / float
    CR_FORCE_INLINE CR_CUDA_HOST CR_CUDA_DEVICE float3 operator/(const float3 &lhs, const float &rhs)
    {
        return make_float3(lhs.x / rhs, lhs.y / rhs, lhs.z / rhs);
    }

    // float / float3
    CR_FORCE_INLINE CR_CUDA_HOST CR_CUDA_DEVICE float3 operator/(const float &lhs, const float3 &rhs)
    {
        return make_float3(lhs / rhs.x, lhs / rhs.y, lhs / rhs.z);
    }

    // compound assignment: float3 += float3
    CR_FORCE_INLINE CR_CUDA_HOST CR_CUDA_DEVICE float3 &operator+=(float3 &lhs, const float3 &rhs)
    {
        lhs.x += rhs.x;
        lhs.y += rhs.y;
        lhs.z += rhs.z;
        return lhs;
    }

    // compound assignment: float3 -= float3
    CR_FORCE_INLINE CR_CUDA_HOST CR_CUDA_DEVICE float3 &operator-=(float3 &lhs, const float3 &rhs)
    {
        lhs.x -= rhs.x;
        lhs.y -= rhs.y;
        lhs.z -= rhs.z;
        return lhs;
    }

    CR_FORCE_INLINE CR_CUDA_HOST CR_CUDA_DEVICE int3 toInt3(const float4 &vec)
    {
        return make_int3(static_cast<int>(vec.x), static_cast<int>(vec.y), static_cast<int>(vec.z));
    }

    CR_FORCE_INLINE CR_CUDA_HOST CR_CUDA_DEVICE int4 toInt4(const float4 &vec)
    {
        return make_int4(static_cast<int>(vec.x), static_cast<int>(vec.y), static_cast<int>(vec.z), static_cast<int>(vec.w));
    }

    CR_FORCE_INLINE CR_CUDA_HOST CR_CUDA_DEVICE int4 toInt(const float4 &vec)
    {
        return toInt4(vec);
    }

    CR_FORCE_INLINE CR_CUDA_HOST CR_CUDA_DEVICE float clamp(float f, float min, float max)
    {
        return fminf(fmaxf(f, min), max);
    }

    CR_FORCE_INLINE CR_CUDA_HOST CR_CUDA_DEVICE void clamp(float4 &vec, const float &min, const float &max)
    {
        vec.x = fminf(fmaxf(vec.x, min), max);
        vec.y = fminf(fmaxf(vec.y, min), max);
        vec.z = fminf(fmaxf(vec.z, min), max);
        vec.w = fminf(fmaxf(vec.w, min), max);
    }

    CR_FORCE_INLINE CR_CUDA_HOST CR_CUDA_DEVICE void clamp(float4 &vec, const float4 &min, const float4 &max)
    {
        vec.x = fminf(fmaxf(vec.x, min.x), max.x);
        vec.y = fminf(fmaxf(vec.y, min.y), max.y);
        vec.z = fminf(fmaxf(vec.z, min.z), max.z);
        vec.w = fminf(fmaxf(vec.w, min.w), max.w);
    }

    CR_FORCE_INLINE CR_CUDA_HOST CR_CUDA_DEVICE void clamp(float4 &vec, const Vec3f &min, const Vec3f &max)
    {
        vec.x = fminf(fmaxf(vec.x, min.data.x), max.data.x);
        vec.y = fminf(fmaxf(vec.y, min.data.y), max.data.y);
        vec.z = fminf(fmaxf(vec.z, min.data.z), max.data.z);
    }

    CR_FORCE_INLINE CR_CUDA_HOST CR_CUDA_DEVICE float4 min(const float4 &vec, const float4 &min)
    {
        const float x = fminf(vec.x, min.x);
        const float y = fminf(vec.y, min.y);
        const float z = fminf(vec.z, min.z);
        const float w = fminf(vec.w, min.w);

        return make_float4(x, y, z, w);
    }

    CR_FORCE_INLINE CR_CUDA_HOST CR_CUDA_DEVICE float4 max(const float4 &vec, const float4 &min)
    {
        const float x = fmaxf(vec.x, min.x);
        const float y = fmaxf(vec.y, min.y);
        const float z = fmaxf(vec.z, min.z);
        const float w = fmaxf(vec.w, min.w);

        return make_float4(x, y, z, w);
    }

    CR_FORCE_INLINE CR_CUDA_HOST CR_CUDA_DEVICE float3 min(const float3 &vec, const float3 &min)
    {
        const float x = fminf(vec.x, min.x);
        const float y = fminf(vec.y, min.y);
        const float z = fminf(vec.z, min.z);

        return make_float3(x, y, z);
    }

    CR_FORCE_INLINE CR_CUDA_HOST CR_CUDA_DEVICE float3 max(const float3 &vec, const float3 &min)
    {
        const float x = fmaxf(vec.x, min.x);
        const float y = fmaxf(vec.y, min.y);
        const float z = fmaxf(vec.z, min.z);

        return make_float3(x, y, z);
    }


    /*-------------------------*/
    // int4 basic operators
    /*-------------------------*/

    CR_FORCE_INLINE CR_CUDA_HOST CR_CUDA_DEVICE int4 operator+(const int4 &lhs, const int4 &rhs)
    {
        return make_int4(lhs.x + rhs.x, lhs.y + rhs.y, lhs.z + rhs.z, lhs.w + rhs.w);
    }

    /*-------------------------*/
    // Vec3 CUDA device atomic add
    /*-------------------------*/

    CR_CUDA_DEVICE CR_FORCE_INLINE void atomicAddVec3f(Vec3f *address, const Vec3f &val)
    {
        atomicAdd(&(address->data.x), val.data.x);
        atomicAdd(&(address->data.y), val.data.y);
        atomicAdd(&(address->data.z), val.data.z);
    }

    CR_CUDA_DEVICE CR_FORCE_INLINE void atomicAddVec3f(Vec3f *address, const float val)
    {
        atomicAdd(&(address->data.x), val);
        atomicAdd(&(address->data.y), val);
        atomicAdd(&(address->data.z), val);
    }

    CR_CUDA_DEVICE CR_FORCE_INLINE void atomicAddVec3f(float4 *address, const Vec3f &val)
    {
        atomicAdd(&(address->x), val.data.x);
        atomicAdd(&(address->y), val.data.y);
        atomicAdd(&(address->z), val.data.z);
    }
}


#endif // !CR_MATH_HELPERS_H