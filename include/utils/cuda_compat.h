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

#ifndef CR_CUDA_COMPAT_H
#define CR_CUDA_COMPAT_H

/**
 * If using cuda_runtime.h, this file must be included after it.
 * Otherwise, this file defines the structs and helper functions
 * for compatibility with CUDA code on host.
 */

#include "preprocessor.h"

#if defined(__CUDACC__)
#include <vector_types.h>     // defines float4, int4
#include <vector_functions.h> // defines make_float4, make_int4
#else

/* If not using NVCC, bring in common math intrinsics for host compiler */

#include <cmath>
#include "math.h"

/* math intrinsics */

CR_FORCE_INLINE float rsqrtf(float v) { return 1.0f / std::sqrt(v); }

/* this should work for most compilers. */
using ::isnan;
using ::sqrtf;
using ::fminf;
using ::fmaxf;
using ::fabsf;
using ::signbit;

/**
 * This is only for compatibilty, not for actual multi-threading usage.
 */
CR_FORCE_INLINE float atomicAdd(float *address, float val)
{
    float old = *address;
    *address = old + val;
    return old;
}
#endif // __CUDACC__

#ifndef CUDART_VERSION

/* If not using cuda_runtime.h, define CUDA built-in struct and helpers for host */

#include <cmath>
#include "math.h"

struct CR_ALIGN(16) float4
{
    float x, y, z, w;
};

struct CR_ALIGN(16) int4
{
    int x, y, z, w;
};

struct float3
{
    float x, y, z;
};

struct int3
{
    int x, y, z;
};

/* constructor helpers */

CR_FORCE_INLINE float4 make_float4(float x, float y, float z, float w)
{
    return float4{x, y, z, w};
}
CR_FORCE_INLINE int4 make_int4(int x, int y, int z, int w)
{
    return int4{x, y, z, w};
}

CR_FORCE_INLINE float3 make_float3(float x, float y, float z)
{
    return float3{x, y, z};
}
CR_FORCE_INLINE int3 make_int3(int x, int y, int z)
{
    return int3{x, y, z};
}

#endif  // CUDART_VERSION

#endif // !CR_CUDA_COMPAT_H