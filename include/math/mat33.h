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

#ifndef CR_MAT33_H
#define CR_MAT33_H

#include "cuda_compat.h"
#include "preprocessor.h"
#include "vec3.h"
#include "math_helpers.h"

namespace crmpm
{
    /*-------------------------*/
    // Column major matrix 3x3
    /*-------------------------*/

    struct Mat3x3f
    {
        float4 col0;
        float4 col1;
        float4 col2;

        CR_FORCE_INLINE CR_CUDA_HOST CR_CUDA_DEVICE Mat3x3f(float4 column0, float4 column1, float4 column2)
            : col0(column0), col1(column1), col2(column2) {}

        CR_FORCE_INLINE CR_CUDA_HOST CR_CUDA_DEVICE Mat3x3f()
            : col0(make_float4(0.0f, 0.0f, 0.0f, 0.0f)),
              col1(make_float4(0.0f, 0.0f, 0.0f, 0.0f)),
              col2(make_float4(0.0f, 0.0f, 0.0f, 0.0f)) {}

        CR_FORCE_INLINE CR_CUDA_HOST CR_CUDA_DEVICE Mat3x3f(
            float m00, float m10, float m20,
            float m01, float m11, float m21,
            float m02, float m12, float m22)
            : col0(make_float4(m00, m10, m20, 0.0f)),
              col1(make_float4(m01, m11, m21, 0.0f)),
              col2(make_float4(m02, m12, m22, 0.0f)) {}

        Mat3x3f(const Mat3x3f &other) = default;

        Mat3x3f &operator=(const Mat3x3f &other) = default;

        CR_FORCE_INLINE CR_CUDA_HOST CR_CUDA_DEVICE static Mat3x3f Identity()
        {
            return Mat3x3f(
                make_float4(1.0f, 0.0f, 0.0f, 0.0f),
                make_float4(0.0f, 1.0f, 0.0f, 0.0f),
                make_float4(0.0f, 0.0f, 1.0f, 0.0f));
        }

        /**
         * @brief Scalar matrix.
         */
        CR_FORCE_INLINE CR_CUDA_HOST CR_CUDA_DEVICE static Mat3x3f Scalar(float value)
        {
            return Mat3x3f(
                make_float4(value, 0.0f, 0.0f, 0.0f),
                make_float4(0.0f, value, 0.0f, 0.0f),
                make_float4(0.0f, 0.0f, value, 0.0f));
        }

        CR_FORCE_INLINE CR_CUDA_HOST CR_CUDA_DEVICE void setIdentity()
        {
            col0 = make_float4(1.0f, 0.0f, 0.0f, 0.0f);
            col1 = make_float4(0.0f, 1.0f, 0.0f, 0.0f);
            col2 = make_float4(0.0f, 0.0f, 1.0f, 0.0f);
        }

        CR_FORCE_INLINE CR_CUDA_HOST CR_CUDA_DEVICE void setZero()
        {
            col0 = make_float4(0.0f, 0.0f, 0.0f, 0.0f);
            col1 = make_float4(0.0f, 0.0f, 0.0f, 0.0f);
            col2 = make_float4(0.0f, 0.0f, 0.0f, 0.0f);
        }

        CR_FORCE_INLINE CR_CUDA_HOST CR_CUDA_DEVICE float &operator()(int row, int col)
        {
            float *base = &col0.x;
            return base[col * 4 + row];
        }

        CR_FORCE_INLINE CR_CUDA_HOST CR_CUDA_DEVICE const float &operator()(int row, int col) const
        {
            const float *base = &col0.x;
            return base[col * 4 + row];
        }

        /**
         * @brief Mat3x3f * scalar
         */
        CR_FORCE_INLINE CR_CUDA_HOST CR_CUDA_DEVICE Mat3x3f operator*(float other) const
        {
            return Mat3x3f(col0 * other, col1 * other, col2 * other);
        }

        /**
         * @brief Scalar * Mat3x3f
         */
        CR_FORCE_INLINE CR_CUDA_HOST CR_CUDA_DEVICE friend Mat3x3f operator*(float scalar, const Mat3x3f &mat)
        {
            return Mat3x3f(mat.col0 * scalar, mat.col1 * scalar, mat.col2 * scalar);
        }

        /**
         * @brief Mat3x3f * Vec3f
         */
        CR_FORCE_INLINE CR_CUDA_HOST CR_CUDA_DEVICE Vec3f operator*(const Vec3f &other) const
        {
            return Vec3f(col0 * other.x() + col1 * other.y() + col2 * other.z());
        }

        /**
         * @brief Mat3x3f * float4
         */
        CR_FORCE_INLINE CR_CUDA_HOST CR_CUDA_DEVICE float4 operator*(const float4 &other) const
        {
            return col0 * other.x + col1 * other.y + col2 * other.z;
        }

        /**
         * @brief Mat3x3f + Mat3x3f
         */
        CR_FORCE_INLINE CR_CUDA_HOST CR_CUDA_DEVICE Mat3x3f operator+(const Mat3x3f &other) const
        {
            return Mat3x3f(col0 + other.col0, col1 + other.col1, col2 + other.col2);
        }

        /**
         * @brief Mat3x3f + Mat3x3f
         */
        CR_FORCE_INLINE CR_CUDA_HOST CR_CUDA_DEVICE Mat3x3f &operator+=(const Mat3x3f &other)
        {
            col0 += other.col0;
            col1 += other.col1;
            col2 += other.col2;
            return *this;
        }

        /**
         * @brief Mat3x3f - Mat3x3f
         */
        CR_FORCE_INLINE CR_CUDA_HOST CR_CUDA_DEVICE Mat3x3f operator-(const Mat3x3f &other) const
        {
            return Mat3x3f(col0 - other.col0, col1 - other.col1, col2 - other.col2);
        }

        /**
         * @brief Mat3x3f - Mat3x3f
         */
        CR_FORCE_INLINE CR_CUDA_HOST CR_CUDA_DEVICE Mat3x3f &operator-=(const Mat3x3f &other)
        {
            col0 -= other.col0;
            col1 -= other.col1;
            col2 -= other.col2;
            return *this;
        }

        /**
         * @brief Mat3x3f * Mat3x3f
         */
        CR_FORCE_INLINE CR_CUDA_HOST CR_CUDA_DEVICE Mat3x3f operator*(const Mat3x3f &other) const
        {
            return Mat3x3f(
                (*this) * other.col0,
                (*this) * other.col1,
                (*this) * other.col2);
        }

        /**
         * @brief In-place left multiplication (this = other * this)
         */
        CR_FORCE_INLINE CR_CUDA_HOST CR_CUDA_DEVICE Mat3x3f &leftMulInPlace(const Mat3x3f &other)
        {
            col0 = other * this->col0;
            col1 = other * this->col1;
            col2 = other * this->col2;
            return *this;
        }

        /**
         * @brief Matrix determinant
         */
        CR_FORCE_INLINE CR_CUDA_HOST CR_CUDA_DEVICE float determinant() const
        {
            float a = (*this)(0, 0);
            float b = (*this)(0, 1);
            float c = (*this)(0, 2);
            float d = (*this)(1, 0);
            float e = (*this)(1, 1);
            float f = (*this)(1, 2);
            float g = (*this)(2, 0);
            float h = (*this)(2, 1);
            float i = (*this)(2, 2);

            return a * (e * i - f * h) - b * (d * i - f * g) + c * (d * h - e * g);
        }

        /**
         * @brief Matrix transpose
         */
        CR_FORCE_INLINE CR_CUDA_HOST CR_CUDA_DEVICE Mat3x3f transpose() const
        {
            return Mat3x3f{
                make_float4(col0.x, col1.x, col2.x, 0.0f),
                make_float4(col0.y, col1.y, col2.y, 0.0f),
                make_float4(col0.z, col1.z, col2.z, 0.0f)};
        }

        /**
         * @brief Matrix trace
         */
        CR_FORCE_INLINE CR_CUDA_HOST CR_CUDA_DEVICE float trace() const
        {
            return col0.x + col1.y + col2.z;
        }

        /**
         * @brief Matrix inverse
         */
        CR_FORCE_INLINE CR_CUDA_HOST CR_CUDA_DEVICE Mat3x3f inverse() const
        {
            float a = (*this)(0, 0);
            float b = (*this)(0, 1);
            float c = (*this)(0, 2);
            float d = (*this)(1, 0);
            float e = (*this)(1, 1);
            float f = (*this)(1, 2);
            float g = (*this)(2, 0);
            float h = (*this)(2, 1);
            float i = (*this)(2, 2);

            float det = a * (e * i - f * h) - b * (d * i - f * g) + c * (d * h - e * g);

            if (fabs(det) < 1e-6f)
            {
                return Mat3x3f{};
            }

            float invDet = 1.0f / det;

            Mat3x3f adj = Mat3x3f{
                make_float4((e * i - f * h), -(b * i - c * h), (b * f - c * e), 0.0f),
                make_float4(-(d * i - f * g), (a * i - c * g), -(a * f - c * d), 0.0f),
                make_float4((d * h - e * g), -(a * h - b * g), (a * e - b * d), 0.0f)};

            return adj * invDet;
        }
    };

    /**
     * @brief Vec3 outer product.
     * 
     * Should only be used for Vec3<float, 3/4>.
     */
    template <typename U, int StorageDim>
    CR_FORCE_INLINE CR_CUDA_HOST CR_CUDA_DEVICE Mat3x3f crmpm::Vec3<U, StorageDim>::outerProduct(const Vec3<U, StorageDim> &other)
    {
        float ax = data.x;
        float ay = data.y;
        float az = data.z;

        float bx = other.data.x;
        float by = other.data.y;
        float bz = other.data.z;

        // Outer product calculation
        return Mat3x3f(
            ax * bx, ax * by, ax * bz,
            ay * bx, ay * by, ay * bz,
            az * bx, az * by, az * bz);
    }

} // namespace crmpm

#endif // !CR_MAT33_H