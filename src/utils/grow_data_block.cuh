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

#ifndef CR_GROW_DATA_BLOCK_CUH
#define CR_GROW_DATA_BLOCK_CUH

namespace crmpm
{
    /**
     * Enlarge a data block on CPU to newSize. newSize must be larger than the old size.
     * Illegal memory access otherwise!
     */
    template <typename T>
    void growCpuData(T *&data, size_t oldSize, size_t newSize)
    {
        T *newData = new T[newSize];
        if (data)
        {
            memcpy(newData, data, sizeof(T) * oldSize);
            delete[] data;
        }
        data = newData;
    }

    /**
     * Enlarge a data block in CUDA pinned memory to newSize. newSize must be larger than oldSize.
     */
    template <typename T>
    void growPinnedData(T *&data, size_t oldSize, size_t newSize)
    {
        T *newData = nullptr;
        CR_CHECK_CUDA(cudaMallocHost<T>(&newData, newSize * sizeof(T)));

        if (data)
        {
            memcpy(newData, data, sizeof(T) * oldSize);
            cudaFreeHost(data);
        }

        data = newData;
    }

    /**
     * Enlarge a data block on GPU to newSize without copying old data.
     * Old data will all be lost!
     */
    template <typename T>
    void growGpuDataDiscardOld(T *&data, size_t newSize)
    {
        T *newData;
        CR_CHECK_CUDA(cudaMalloc<T>(&newData, sizeof(T) * newSize));
        if (data)
            CR_CHECK_CUDA(cudaFree(data));

        data = newData;
    }

    /**
     * Enlarge a data block on GPU to newSize. newSize must be larger than the old size.
     * Illegal memory access otherwise!
     */
    template <typename T>
    void growGpuData(T *&data, size_t oldSize, size_t newSize)
    {
        T *newData;
        CR_CHECK_CUDA(cudaMalloc<T>(&newData, sizeof(T) * newSize));
        if (data)
        {
            CR_CHECK_CUDA(cudaMemcpy(newData, data, sizeof(T) * oldSize, cudaMemcpyKind::cudaMemcpyDeviceToDevice));
            CR_CHECK_CUDA(cudaFree(data));
        }
        data = newData;
    }
}
#endif // !CR_GROW_DATA_BLOCK_CUH