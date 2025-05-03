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

#ifndef CR_BOUNDS3_H
#define CR_BOUNDS3_H

#include "vec3.h"
#include "preprocessor.h"

namespace crmpm
{
    struct Bounds3
    {
        Vec3f minimum;
        Vec3f maximum;

        Bounds3() = default;

        Bounds3(const Vec3f &minValue, const Vec3f &maxValue) : minimum(minValue), maximum(maxValue) {}

        Bounds3(const Bounds3 &other) = default;

        Bounds3 &operator=(const Bounds3 &other) = default;

        ~Bounds3() = default;

        /**
         * @brief Get bound center.
         */
        CR_FORCE_INLINE CR_CUDA_HOST CR_CUDA_DEVICE Vec3f getCenter() const
        {
            return (minimum + maximum) * 0.5f;
        }

        /**
         * @brief Get bound center.
         */
        CR_FORCE_INLINE CR_CUDA_HOST CR_CUDA_DEVICE float getCenter(unsigned int axis) const
        {
            return (minimum[axis] + maximum[axis]) * 0.5f;
        }

        /**
         * @brief Enlarge the bound if needed to include a point.
         */
        CR_FORCE_INLINE CR_CUDA_HOST CR_CUDA_DEVICE void include(const Vec3f &v)
        {
            minimum = min(minimum, v);
            maximum = max(maximum, v);
        }

        /**
         * @brief Fatten the bound.
         * 
         * @param[in] distance The fatten distance. It should always be possitive.
         */
        CR_FORCE_INLINE CR_CUDA_HOST CR_CUDA_DEVICE void fattenFast(float distance)
        {
            minimum.x() -= distance;
            minimum.y() -= distance;
            minimum.z() -= distance;

            maximum.x() += distance;
            maximum.y() += distance;
            maximum.z() += distance;
        }

        /**
         * @brief Check if a point is inside the bound.
         * 
         * @param[in] v the point to check.
         */
        CR_FORCE_INLINE CR_CUDA_HOST CR_CUDA_DEVICE bool contains(const Vec3f &v) const
        {
            return !(v.x() < minimum.x() || v.x() > maximum.x() || v.y() < minimum.y() || v.y() > maximum.y() || v.z() < minimum.z() ||
                     v.z() > maximum.z());
        }
    };
}

#endif // !CR_BOUNDS3_H
