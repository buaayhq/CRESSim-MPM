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

#ifndef CR_PREPROCESSOR_H
#define CR_PREPROCESSOR_H

/**
 * CUDA preprocessors
 */
#ifdef __CUDACC__
#define CR_CUDA_GLOBAL __global__
#define CR_CUDA_DEVICE __device__
#define CR_CUDA_HOST __host__
#else
#define CR_CUDA_GLOBAL
#define CR_CUDA_DEVICE
#define CR_CUDA_HOST
#endif

#if defined(__CUDACC__) // NVCC
#define CR_ALIGN(n) __align__(n)
#elif defined(__GNUC__) // GCC
#define CR_ALIGN(n) __attribute__((aligned(n)))
#elif defined(_MSC_VER) // MSVC
#define CR_ALIGN(n) __declspec(align(n))
#else
#error "No align for compiler!"
#endif

/**
 * Hint inline
 */
#if defined(__CUDACC__)
#define CR_INLINE __inline__
#elif defined(_MSC_VER)
#define CR_INLINE inline
#elif defined(__GNUC__) || defined(__clang__)
#define CR_INLINE inline __attribute__((gnu_inline))
#else
#define CR_INLINE inline
#endif


/**
 * Force inline
 */
#if defined(__CUDACC__)
#define CR_FORCE_INLINE __forceinline__
#elif defined(_MSC_VER)
#define CR_FORCE_INLINE __forceinline
#elif defined(__GNUC__) || defined(__clang__)
#define CR_FORCE_INLINE inline __attribute__((always_inline))
#else
#define CR_FORCE_INLINE inline
#endif

/**
 * Restrict keyward
 */
#if defined(_MSC_VER)
#define CR_RESTRICT __restrict
#elif defined(__CUDACC__) || defined(__GNUC__) || defined(__clang__)
#define CR_RESTRICT __restrict__
#else
#define CR_RESTRICT
#endif

#endif // !CR_PREPROCESSOR_H