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

#ifndef CR_SHAPE_FUNCTIONS_H
#define CR_SHAPE_FUNCTIONS_H

#include "vec3.h"
#include "mat33.h"
#include "preprocessor.h"

namespace crmpm
{
    /**
     * @brief Quadratic B-spline interpolation weights
     */
    CR_CUDA_HOST CR_CUDA_DEVICE CR_FORCE_INLINE static void computeQuadraticBSplineWeights(const Vec3f &fx, Mat3x3f &w)
    {
        w(0, 0) = 0.5f * powf(1.5f - fx.x(), 2);
        w(1, 0) = 0.75f - powf(fx.x() - 1.0f, 2);
        w(2, 0) = 0.5f * powf(fx.x() - 0.5f, 2);

        w(0, 1) = 0.5f * powf(1.5f - fx.y(), 2);
        w(1, 1) = 0.75f - powf(fx.y() - 1.0f, 2);
        w(2, 1) = 0.5f * powf(fx.y() - 0.5f, 2);

        w(0, 2) = 0.5f * powf(1.5f - fx.z(), 2);
        w(1, 2) = 0.75f - powf(fx.z() - 1.0f, 2);
        w(2, 2) = 0.5f * powf(fx.z() - 0.5f, 2);
    }

    /**
     * @brief Quadratic B-spline interpolation weights gradient
     */
    CR_CUDA_HOST CR_CUDA_DEVICE CR_FORCE_INLINE static void computeQuadraticBSplineWeightGradients(const Vec3f &fx, Mat3x3f &dw)
    {
        dw(0, 0) = fx.x() - 1.5;
        dw(1, 0) = 2.0f * (1.0f - fx.x());
        dw(2, 0) = fx.x() - 0.5f;

        dw(0, 1) = fx.y() - 1.5;
        dw(1, 1) = 2.0f * (1.0f - fx.y());
        dw(2, 1) = fx.y() - 0.5f;

        dw(0, 2) = fx.z() - 1.5;
        dw(1, 2) = 2.0f * (1.0f - fx.z());
        dw(2, 2) = fx.z() - 0.5f;
    }
}

#endif // !SHAPE_FUNCTIONS_H
