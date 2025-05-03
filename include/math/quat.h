/**
 * Some parts taken from PhysX PxQuat:
 * https://github.com/NVIDIA-Omniverse/PhysX/blob/dd587fedd79836442a4117164ea8c46685453c34/physx/include/foundation/PxQuat.h.
 * License:

    Redistribution and use in source and binary forms, with or without
    modification, are permitted provided that the following conditions
    are met:
    * Redistributions of source code must retain the above copyright
    notice, this list of conditions and the following disclaimer.
    * Redistributions in binary form must reproduce the above copyright
    notice, this list of conditions and the following disclaimer in the
    documentation and/or other materials provided with the distribution.
    * Neither the name of NVIDIA CORPORATION nor the names of its
    contributors may be used to endorse or promote products derived
    from this software without specific prior written permission.

    THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS ''AS IS'' AND ANY
    EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
    IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
    PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL THE COPYRIGHT OWNER OR
    CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
    EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
    PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
    PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY
    OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
    (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
    OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

    Copyright (c) 2008-2024 NVIDIA Corporation. All rights reserved.
    Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
    Copyright (c) 2001-2004 NovodeX AG. All rights reserved.
 *
 */

#ifndef CR_QUAT_H
#define CR_QUAT_H

#include "preprocessor.h"
#include "vec3.h"
#include "constants.h"

namespace crmpm
{
    /**
     * Quaternion in floats.
     */
    struct Quat
    {
        float4 data;

        // -------------------
        // Constructors
        // -------------------

        CR_FORCE_INLINE CR_CUDA_HOST CR_CUDA_DEVICE Quat() : data(make_float4(0.0f, 0.0f, 0.0f, 1.0f)) {}

        CR_FORCE_INLINE CR_CUDA_HOST CR_CUDA_DEVICE Quat(float x, float y, float z, float w) : data(make_float4(x, y, z, w)) {}

        CR_FORCE_INLINE CR_CUDA_HOST CR_CUDA_DEVICE Quat(const Quat &other) : data(other.data) {}

        CR_FORCE_INLINE CR_CUDA_HOST CR_CUDA_DEVICE Quat(const float4 &other) : data(other) {}

        // -------------------
        // Operators
        // -------------------

        CR_FORCE_INLINE CR_CUDA_HOST CR_CUDA_DEVICE Quat &operator*=(const Quat &q)
        {
            const float x = data.x;
            const float y = data.y;
            const float z = data.z;
            const float w = data.w;

            const float qx = q.data.x;
            const float qy = q.data.y;
            const float qz = q.data.z;
            const float qw = q.data.w;

            const float tx = w * qx + qw * x + y * qz - qy * z;
            const float ty = w * qy + qw * y + z * qx - qz * x;
            const float tz = w * qz + qw * z + x * qy - qx * y;

            data.w = w * qw - qx * x - y * qy - qz * z;
            data.x = tx;
            data.y = ty;
            data.z = tz;
            return *this;
        }

        /**
         * Quaternion multiplication
         */
        CR_FORCE_INLINE CR_CUDA_HOST CR_CUDA_DEVICE Quat operator*(const Quat &q) const
        {
            const float x = data.x;
            const float y = data.y;
            const float z = data.z;
            const float w = data.w;

            const float qx = q.data.x;
            const float qy = q.data.y;
            const float qz = q.data.z;
            const float qw = q.data.w;

            return Quat(w * qx + qw * x + y * qz - qy * z, w * qy + qw * y + z * qx - qz * x,
                        w * qz + qw * z + x * qy - qx * y, w * qw - x * qx - y * qy - z * qz);
        }

        // -------------------
        // Rotation
        // -------------------

        /**
         * Rotates passed vec by this (assumed unitary)
         */
        CR_FORCE_INLINE CR_CUDA_HOST CR_CUDA_DEVICE Vec3f rotate(const Vec3f &v) const
        {
            const float x = data.x;
            const float y = data.y;
            const float z = data.z;
            const float w = data.w;

            const float vx = 2.0f * v.x();
            const float vy = 2.0f * v.y();
            const float vz = 2.0f * v.z();
            const float w2 = w * w - 0.5f;
            const float dot2 = (x * vx + y * vy + z * vz);
            return Vec3f((vx * w2 + (y * vz - z * vy) * w + x * dot2),
                         (vy * w2 + (z * vx - x * vz) * w + y * dot2),
                         (vz * w2 + (x * vy - y * vx) * w + z * dot2));
        }

        /**
         * Inverse rotates passed vec by this (assumed unitary)
         */
        CR_FORCE_INLINE CR_CUDA_HOST CR_CUDA_DEVICE Vec3f rotateInv(const Vec3f &v) const
        {
            const float x = data.x;
            const float y = data.y;
            const float z = data.z;
            const float w = data.w;

            const float vx = 2.0f * v.x();
            const float vy = 2.0f * v.y();
            const float vz = 2.0f * v.z();
            const float w2 = w * w - 0.5f;
            const float dot2 = (x * vx + y * vy + z * vz);
            return Vec3f((vx * w2 - (y * vz - z * vy) * w + x * dot2),
                         (vy * w2 - (z * vx - x * vz) * w + y * dot2),
                         (vz * w2 - (x * vy - y * vx) * w + z * dot2));
        }

        /**
         * @brief The conjugate. Inverse rotation for a unit quaternion.
         */
        CR_FORCE_INLINE CR_CUDA_HOST CR_CUDA_DEVICE Quat getConjugate() const
        {
            return Quat(-data.x, -data.y, -data.z, data.w);
        }

        /**
         * @brief Converts this quaternion to angle-axis representation
        */
        CR_FORCE_INLINE CR_CUDA_HOST CR_CUDA_DEVICE void toRadiansAndUnitAxis(float &angle, Vec3f &axis) const
        {
            const float x = data.x;
            const float y = data.y;
            const float z = data.z;
            const float w = data.w;

            const float quatEpsilon = 1.0e-8f;
            const float s2 = x * x + y * y + z * z;
            if (s2 < quatEpsilon * quatEpsilon) // can't extract a sensible axis
            {
                angle = 0.0f;
                axis = Vec3f(1.0f, 0.0f, 0.0f);
            }
            else
            {
                const float s = rsqrtf(s2);
                axis = Vec3f(x, y, z) * s;
                angle = fabsf(w) < quatEpsilon ? CR_PI : atan2f(s2 * s, w) * 2.0f;
            }
        }
    };

} // namespace crmpm

#endif // !CR_QUAT_H