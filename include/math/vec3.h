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

#ifndef CR_VEC3_H
#define CR_VEC3_H

#include "cuda_compat.h"
#include "preprocessor.h"

/* In case, undef min/max */
#ifdef min
#undef min
#endif

#ifdef max
#undef max
#endif

namespace crmpm
{
    struct Mat3x3f;

    template <typename T, int StorageDim>
    struct vec_type;

    template <>
    struct vec_type<int, 3>
    {
        using type = int3;
    };

    template <>
    struct vec_type<int, 4>
    {
        using type = int4;
    };

    template <>
    struct vec_type<float, 3>
    {
        using type = float3;
    };

    template <>
    struct vec_type<float, 4>
    {
        using type = float4;
    };

    template <typename T, int StorageDim>
    using vec_type_t = typename vec_type<T, StorageDim>::type;

    template <typename T, int StorageDim>
    vec_type_t<T, StorageDim> make_vector3(T x, T y, T z);

    template <>
    CR_FORCE_INLINE CR_CUDA_HOST CR_CUDA_DEVICE float4 make_vector3<float, 4>(float x, float y, float z)
    {
        return make_float4(x, y, z, 0);
    }

    template <>
    CR_FORCE_INLINE CR_CUDA_HOST CR_CUDA_DEVICE float3 make_vector3<float, 3>(float x, float y, float z)
    {
        return make_float3(x, y, z);
    }

    template <>
    CR_FORCE_INLINE CR_CUDA_HOST CR_CUDA_DEVICE int4 make_vector3<int, 4>(int x, int y, int z)
    {
        return make_int4(x, y, z, 0);
    }

    template <>
    CR_FORCE_INLINE CR_CUDA_HOST CR_CUDA_DEVICE int3 make_vector3<int, 3>(int x, int y, int z)
    {
        return make_int3(x, y, z);
    }

    template <typename T, int StorageDim>
    struct Vec3
    {
        vec_type_t<T, StorageDim> data;

        // -------------------
        // Constructors
        // -------------------

        CR_FORCE_INLINE CR_CUDA_HOST CR_CUDA_DEVICE Vec3() : data(make_vector3<T, StorageDim>(0, 0, 0)) {}

        CR_FORCE_INLINE CR_CUDA_HOST CR_CUDA_DEVICE Vec3(T x) : data(make_vector3<T, StorageDim>(x, x, x)) {}

        CR_FORCE_INLINE CR_CUDA_HOST CR_CUDA_DEVICE Vec3(T x, T y, T z) : data(make_vector3<T, StorageDim>(x, y, z)) {}

        CR_FORCE_INLINE CR_CUDA_HOST CR_CUDA_DEVICE Vec3(const Vec3 &other) : data(other.data) {}

        CR_FORCE_INLINE CR_CUDA_HOST CR_CUDA_DEVICE Vec3(const vec_type_t<T, StorageDim> &other) : data(other) {}

        // -------------------
        // Assignment Operator
        // -------------------

        CR_FORCE_INLINE CR_CUDA_HOST CR_CUDA_DEVICE Vec3 &operator=(const Vec3 &other)
        {
            if (this != &other)
            {
                data = other.data;
            }
            return *this;
        }

        // -------------------
        // Arithmetic Operators
        // -------------------

        // Vector addition
        CR_FORCE_INLINE CR_CUDA_HOST CR_CUDA_DEVICE Vec3 operator+(const Vec3 &other) const
        {
            return Vec3(data.x + other.data.x,
                        data.y + other.data.y,
                        data.z + other.data.z);
        }

        // Vector subtraction
        CR_FORCE_INLINE CR_CUDA_HOST CR_CUDA_DEVICE Vec3 operator-(const Vec3 &other) const
        {
            return Vec3(data.x - other.data.x,
                        data.y - other.data.y,
                        data.z - other.data.z);
        }

        // Vector addition
        CR_FORCE_INLINE CR_CUDA_HOST CR_CUDA_DEVICE Vec3 operator+(const vec_type_t<T, StorageDim> &other) const
        {
            return Vec3(data.x + other.x,
                        data.y + other.y,
                        data.z + other.z);
        }

        // Vector subtraction
        CR_FORCE_INLINE CR_CUDA_HOST CR_CUDA_DEVICE Vec3 operator-(const vec_type_t<T, StorageDim> &other) const
        {
            return Vec3(data.x - other.x,
                        data.y - other.y,
                        data.z - other.z);
        }

        // Friend vector addition
        CR_FORCE_INLINE CR_CUDA_HOST CR_CUDA_DEVICE friend Vec3 operator+(const vec_type_t<T, StorageDim> &lhs, const Vec3 &rhs)
        {
            return Vec3(lhs.x + rhs.data.x,
                        lhs.y + rhs.data.y,
                        lhs.z + rhs.data.z);
        }

        // Friend vector subtraction
        CR_FORCE_INLINE CR_CUDA_HOST CR_CUDA_DEVICE friend Vec3 operator-(const vec_type_t<T, StorageDim> &lhs, const Vec3 &rhs)
        {
            return Vec3(lhs.x - rhs.data.x,
                        lhs.y - rhs.data.y,
                        lhs.z - rhs.data.z);
        }

        // Scalar addition
        CR_FORCE_INLINE CR_CUDA_HOST CR_CUDA_DEVICE Vec3 operator+(T scalar) const
        {
            return Vec3(data.x + scalar,
                        data.y + scalar,
                        data.z + scalar);
        }

        // Scalar subtraction
        CR_FORCE_INLINE CR_CUDA_HOST CR_CUDA_DEVICE Vec3 operator-(T scalar) const
        {
            return Vec3(data.x - scalar,
                        data.y - scalar,
                        data.z - scalar);
        }

        // Scalar multiplication
        CR_FORCE_INLINE CR_CUDA_HOST CR_CUDA_DEVICE Vec3 operator*(T scalar) const
        {
            return Vec3(data.x * scalar,
                        data.y * scalar,
                        data.z * scalar);
        }

        // Scalar division
        CR_FORCE_INLINE CR_CUDA_HOST CR_CUDA_DEVICE Vec3 operator/(T scalar) const
        {
            return Vec3(data.x / scalar,
                        data.y / scalar,
                        data.z / scalar);
        }

        // Friend scalar division
        CR_FORCE_INLINE CR_CUDA_HOST CR_CUDA_DEVICE friend Vec3 operator/(const T &lhs, const Vec3 &rhs)
        {
            return Vec3(lhs / rhs.data.x,
                        lhs / rhs.data.y,
                        lhs / rhs.data.z);
        }

        // Vector element-wise multiplication
        CR_FORCE_INLINE CR_CUDA_HOST CR_CUDA_DEVICE Vec3 operator*(const Vec3 &other) const
        {
            return Vec3(data.x * other.data.x,
                        data.y * other.data.y,
                        data.z * other.data.z);
        }

        // Vector element-wise division
        CR_FORCE_INLINE CR_CUDA_HOST CR_CUDA_DEVICE Vec3 operator/(const Vec3 &other) const
        {
            return Vec3(data.x / other.data.x,
                        data.y / other.data.y,
                        data.z / other.data.z);
        }

        // Vector multiplication (a * b.transpose())
        CR_FORCE_INLINE CR_CUDA_HOST CR_CUDA_DEVICE Mat3x3f outerProduct(const Vec3 &other);

        // -------------------
        // Compound Assignment Operators
        // -------------------

        // Compound addition
        CR_FORCE_INLINE CR_CUDA_HOST CR_CUDA_DEVICE Vec3 &operator+=(const Vec3 &other)
        {
            data.x += other.data.x;
            data.y += other.data.y;
            data.z += other.data.z;
            return *this;
        }

        // Compound subtraction
        CR_FORCE_INLINE CR_CUDA_HOST CR_CUDA_DEVICE Vec3 &operator-=(const Vec3 &other)
        {
            data.x -= other.data.x;
            data.y -= other.data.y;
            data.z -= other.data.z;
            return *this;
        }

        // Compound scalar multiplication
        CR_FORCE_INLINE CR_CUDA_HOST CR_CUDA_DEVICE Vec3 &operator*=(T scalar)
        {
            data.x *= scalar;
            data.y *= scalar;
            data.z *= scalar;
            return *this;
        }

        // Compound scalar division
        CR_FORCE_INLINE CR_CUDA_HOST CR_CUDA_DEVICE Vec3 &operator/=(T scalar)
        {
            data.x /= scalar;
            data.y /= scalar;
            data.z /= scalar;
            return *this;
        }

        // Compound element-wise multiplication
        CR_FORCE_INLINE CR_CUDA_HOST CR_CUDA_DEVICE Vec3 &operator*=(Vec3 vec)
        {
            data.x *= vec.x();
            data.y *= vec.y();
            data.z *= vec.z();
            return *this;
        }

        // -------------------
        // Comparison Operators
        // -------------------

        // Equality comparison
        CR_FORCE_INLINE CR_CUDA_HOST CR_CUDA_DEVICE bool operator==(const Vec3 &other) const
        {
            return (data.x == other.data.x) &&
                   (data.y == other.data.y) &&
                   (data.z == other.data.z);
        }

        // Inequality comparison
        CR_FORCE_INLINE CR_CUDA_HOST CR_CUDA_DEVICE bool operator!=(const Vec3 &other) const
        {
            return !(*this == other);
        }

        // -------------------
        // Dot and Cross Products
        // -------------------

        // Dot product
        CR_FORCE_INLINE CR_CUDA_HOST CR_CUDA_DEVICE T dot(const Vec3 &other) const
        {
            return data.x * other.data.x +
                   data.y * other.data.y +
                   data.z * other.data.z;
        }

        // Cross product
        CR_FORCE_INLINE CR_CUDA_HOST CR_CUDA_DEVICE Vec3 cross(const Vec3 &other) const
        {
            return Vec3(
                data.y * other.data.z - data.z * other.data.y,
                data.z * other.data.x - data.x * other.data.z,
                data.x * other.data.y - data.y * other.data.x);
        }

        CR_FORCE_INLINE CR_CUDA_HOST CR_CUDA_DEVICE T squaredNorm() const
        {
            return data.x * data.x + data.y * data.y + data.z * data.z;
        }

        CR_FORCE_INLINE CR_CUDA_HOST CR_CUDA_DEVICE T norm() const
        {
            return sqrtf(squaredNorm());
        }

        CR_FORCE_INLINE CR_CUDA_HOST CR_CUDA_DEVICE T invNorm() const
        {
            return rsqrtf(squaredNorm());
        }

        CR_FORCE_INLINE CR_CUDA_HOST CR_CUDA_DEVICE void normalize()
        {
            float sqrN = (*this).squaredNorm();
            if (sqrN > 0)
            {
                (*this) *= rsqrtf(sqrN);
            }
        }

        CR_FORCE_INLINE CR_CUDA_HOST CR_CUDA_DEVICE Vec3 getNormalized() const
        {
            float sqrN = (*this).squaredNorm();
            return sqrN > 0 ? (*this) * rsqrtf(sqrN) : Vec3(0);
        }

        // -------------------
        // Accessors and Mutators
        // -------------------

        // Indexing operator
        CR_FORCE_INLINE CR_CUDA_HOST CR_CUDA_DEVICE const T &operator[](int index) const
        {
            const T *base = &data.x;
            return base[index];
        }

        CR_FORCE_INLINE CR_CUDA_HOST CR_CUDA_DEVICE T &operator[](int index)
        {
            T *base = &data.x;
            return base[index];
        }

        // Getters
        CR_FORCE_INLINE CR_CUDA_HOST CR_CUDA_DEVICE const T &x() const { return data.x; }

        CR_FORCE_INLINE CR_CUDA_HOST CR_CUDA_DEVICE const T &y() const { return data.y; }

        CR_FORCE_INLINE CR_CUDA_HOST CR_CUDA_DEVICE const T &z() const { return data.z; }

        CR_FORCE_INLINE CR_CUDA_HOST CR_CUDA_DEVICE T &x() { return data.x; }

        CR_FORCE_INLINE CR_CUDA_HOST CR_CUDA_DEVICE T &y() { return data.y; }

        CR_FORCE_INLINE CR_CUDA_HOST CR_CUDA_DEVICE T &z() { return data.z; }

        CR_FORCE_INLINE CR_CUDA_HOST CR_CUDA_DEVICE void setZero()
        {
            data.x = 0;
            data.y = 0;
            data.z = 0;
        }

        // -------------------
        // Type casting
        // -------------------

        template <typename U, int TargetStorageDim = StorageDim>
        CR_FORCE_INLINE CR_CUDA_HOST CR_CUDA_DEVICE Vec3<U, TargetStorageDim> cast() const
        {
            return Vec3<U, TargetStorageDim>(static_cast<U>(data.x), static_cast<U>(data.y), static_cast<U>(data.z));
        }
    };

    typedef Vec3<float, 4> Vec3f;
    typedef Vec3<int, 4> Vec3i;
    typedef Vec3<float, 3> Vec3f3;
    typedef Vec3<int, 3> Vec3i3;

    static_assert(sizeof(Vec3f) == sizeof(float4) && sizeof(Vec3i) == sizeof(int4),
                  "Vec3 must be the same size as float4/int4");
    static_assert(alignof(Vec3f) == alignof(float4) && alignof(Vec3i) == alignof(int4),
                  "Vec3 must have the same alignment as float4/int4");
    static_assert(sizeof(Vec3f3) == sizeof(float3) && sizeof(Vec3i3) == sizeof(int3),
                  "Vec3 must be the same size as float4/int4");
    static_assert(alignof(Vec3f3) == alignof(float3) && alignof(Vec3i3) == alignof(int3),
                  "Vec3 must have the same alignment as float4/int4");

    CR_FORCE_INLINE CR_CUDA_HOST CR_CUDA_DEVICE float4 &operator+=(float4 &lhs, const Vec3f &rhs)
    {
        lhs.x += rhs.data.x;
        lhs.y += rhs.data.y;
        lhs.z += rhs.data.z;
        return lhs;
    }

    // Only works for Vec3<float, 3/4>

    template <typename T, int StorageDim>
    CR_FORCE_INLINE CR_CUDA_HOST CR_CUDA_DEVICE Vec3<T, StorageDim> abs(const Vec3<T, StorageDim> &vec)
    {
        return Vec3<T, StorageDim>(fabsf(vec.data.x), fabsf(vec.data.y), fabsf(vec.data.z));
    }

    template <typename T, int StorageDim>
    CR_FORCE_INLINE CR_CUDA_HOST CR_CUDA_DEVICE Vec3<T, StorageDim> min(const Vec3<T, StorageDim> &a, const Vec3<T, StorageDim> &b)
    {
        return Vec3<T, StorageDim>(fminf(a.data.x, b.data.x), fminf(a.data.y, b.data.y), fminf(a.data.z, b.data.z));
    }

    template <typename T, int StorageDim>
    CR_FORCE_INLINE CR_CUDA_HOST CR_CUDA_DEVICE Vec3<T, StorageDim> max(const Vec3<T, StorageDim> &a, const Vec3<T, StorageDim> &b)
    {
        return Vec3<T, StorageDim>(fmaxf(a.data.x, b.data.x), fmaxf(a.data.y, b.data.y), fmaxf(a.data.z, b.data.z));
    }

    template <typename T, int StorageDim>
    CR_FORCE_INLINE CR_CUDA_HOST CR_CUDA_DEVICE Vec3<T, StorageDim> max(const Vec3<T, StorageDim> &vec, float a)
    {
        return Vec3<T, StorageDim>(fmaxf(vec.data.x, a), fmaxf(vec.data.y, a), fmaxf(vec.data.z, a));
    }
}

#endif // !CR_VEC3_H