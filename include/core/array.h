/**
 * Modified from PhysX PxArray:
 * https://github.com/NVIDIA-Omniverse/PhysX/blob/dd587fedd79836442a4117164ea8c46685453c34/physx/include/foundation/PxArray.h
 * 
 * Original code licensed under the BSD 3-Clause License by NVIDIA Corporation.
 * 
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *  * Redistributions of source code must retain the above copyright
 *  notice, this list of conditions and the following disclaimer.
 *  * Redistributions in binary form must reproduce the above copyright
 *  notice, this list of conditions and the following disclaimer in the
 *  documentation and/or other materials provided with the distribution.
 *  * Neither the name of NVIDIA CORPORATION nor the names of its
 *  contributors may be used to endorse or promote products derived
 *  from this software without specific prior written permission.
 * 
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS ''AS IS'' AND ANY
 *  EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 *  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 *  PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 *  CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 *  EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 *  PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
 *  PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY
 *  OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 *  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 *  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 * 
 *  Copyright (c) 2008-2024 NVIDIA Corporation. All rights reserved.
 *  Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
 *  Copyright (c) 2001-2004 NovodeX AG. All rights reserved.
 *
 * Modifications by:
 *  Copyright (c) 2025, Yafei Ou and Mahdi Tavakoli
 *  Licensed under the BSD 3-Clause License.
 */

#ifndef CR_ARRAY_H
#define CR_ARRAY_H

#include <cstring>
#include <cstdint>
#include <new>

#include "disable_copy.h"

namespace crmpm
{
    /**
     * Simple resizable array.
     */
    template <typename T>
    class Array
    {
    public:
        typedef T *Iterator;
        typedef const T *ConstIterator;

        // Constructor
        Array()
            : mData(nullptr), mSize(0), mCapacity(0) {}

        // Constructor with capacity
        Array(uint32_t capacity)
            : mData(nullptr), mSize(0), mCapacity(capacity)
        {
            if (capacity > 0)
            {
                mData = new T[capacity];
            }
        }

        // Destructor
        ~Array()
        {
            clear();
        }

        // Size accessor
        uint32_t size() const
        {
            return mSize;
        }

        // If empty
        bool empty() const
        {
            return mSize == 0;
        }

        // Capacity accessor
        uint32_t capacity() const
        {
            return mCapacity;
        }

        // Access element
        T &operator[](uint32_t index)
        {
            return mData[index];
        }

        const T &operator[](uint32_t index) const
        {
            return mData[index];
        }

        /**
         * Returns a reference to the last element of the array. Undefined if the array is empty
         * @return a reference to the last element of the array
        */
        const T &back() const
        {
            return mData[mSize - 1];
        }

        T &back()
        {
            return mData[mSize - 1];
        }

        // Add element to the end
        void pushBack(const T &value)
        {
            if (mSize >= mCapacity)
            {
                reserve(mCapacity == 0 ? 1 : mCapacity * 2);
            }
            mData[mSize++] = value;
        }

        /**
         * Attempt to add an element to the array without resizing.
         * The element is added only if there is sufficient capacity (mSize < mCapacity).
         * Return false if there is no available space.
         */
        bool tryPushBack(const T &value)
        {
            if (mSize < mCapacity)
            {
                mData[mSize++] = value;
                return true;
            }
            return false;
        }

        // Remove last element
        T popBack()
        {
            T t = mData[mSize - 1];
            mData[--mSize].~T();

            return t;
        }

        // Clear all elements
        void clear()
        {
            delete[] mData;
            mData = nullptr;
            mSize = 0;
            mCapacity = 0;
        }

        // Remove element by index
        void remove(uint32_t i)
        {
            if (i >= mSize)
                return;

            T *it = mData + i;
            it->~T();
            while (++i < mSize)
            {
                new (it) T(mData[i]);
                ++it;
                it->~T();
            }
            --mSize;
        }

        // Remove a range of elements
        void removeRange(uint32_t begin, uint32_t count)
        {
            if (begin >= mSize || (begin + count) > mSize)
                return;

            for (uint32_t i = 0; i < count; ++i)
            {
                mData[begin + i].~T();
            }

            T *dest = mData + begin;
            T *src = mData + begin + count;
            uint32_t moveCount = mSize - (begin + count);

            for (uint32_t i = 0; i < moveCount; ++i)
            {
                new (dest) T(*src);
                src->~T();
                ++dest;
                ++src;
            }

            mSize -= count;
        }

        // Reserve capacity
        void reserve(uint32_t newCapacity)
        {
            if (newCapacity > mCapacity)
            {
                T *newData = new T[newCapacity];
                if (mData)
                {
                    memcpy(newData, mData, mSize * sizeof(T));
                    delete[] mData;
                }
                mData = newData;
                mCapacity = newCapacity;
            }
        }

        static void create(T *first, T *last, const T &a)
        {
            for (; first < last; ++first)
                ::new(first) T(a);
        }

        static void destroy(T *first, T *last)
        {
            for (; first < last; ++first)
                first->~T();
        }

        void resize(const uint32_t size, const T &a = T())
        {
            reserve(size);
            create(mData + mSize, mData + size, a);
            destroy(mData + size, mData + mSize);
            mSize = size;
        }

        void forceSizeUnsafe(const uint32_t size)
        {
            reserve(size);
            mSize = size;
        }

        // Find
        Iterator find(const T &a)
        {
            uint32_t index;
            for (index = 0; index < mSize && mData[index] != a; index++)
                ;
            return mData + index;
        }

        ConstIterator find(const T &a) const
        {
            uint32_t index;
            for (index = 0; index < mSize && mData[index] != a; index++)
                ;
            return mData + index;
        }

        // Begin iterator
        Iterator begin()
        {
            return mData;
        }

        ConstIterator begin() const
        {
            return mData;
        }

        // End iterator
        Iterator end()
        {
            return mData + mSize;
        }

        ConstIterator end() const
        {
            return mData + mSize;
        }

    protected:
        T *mData;
        uint32_t mSize;
        uint32_t mCapacity;

    private:
        CR_DISABLE_COPY(Array);
    };
} // namespace crmpm

#endif // !CR_ARRAY_H