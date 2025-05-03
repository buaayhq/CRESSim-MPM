// BSD 3-Clause License
//
// Copyright (c) 2025, Yafei Ou and Mahdi Tavakoli
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// 1. Redistributions of source code must retain the above copyright notice, this
//    list of conditions and the following disclaimer.
//
// 2. Redistributions in binary form must reproduce the above copyright notice,
//    this list of conditions and the following disclaimer in the documentation
//    and/or other materials provided with the distribution.
//
// 3. Neither the name of the copyright holder nor the names of its
//    contributors may be used to endorse or promote products derived from
//    this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
// FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
// DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
// SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
// CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
// OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

/**
 * Naive implementation of a single thread manager
 */

#include "single_thread_manager.h"
#include <cstring>
#ifdef _WIN32
#include <Windows.h>
#else
#include <pthread.h>
#include <unistd.h>
#endif

namespace crmpm
{
    SingleThreadManager::SingleThreadManager() : mThreadHandle(nullptr), mStatus(ThreadStatus::eStopped), mStopRequested(false), mCurrentTask(nullptr), mTaskData(nullptr) {}

    SingleThreadManager::~SingleThreadManager()
    {
        stop();
    }

    bool SingleThreadManager::start()
    {
        if (!(mStatus == ThreadStatus::eStopped))
        {
            return false;
        }

        mStopRequested = false;

#ifdef _WIN32
        mThreadHandle = CreateThread(
            nullptr,
            0,
            (LPTHREAD_START_ROUTINE)threadEntryPoint,
            this,
            0,
            nullptr);
        if (mThreadHandle == nullptr)
        {
            return false;
        }
#else
        pthread_t *thread = new pthread_t;
        mThreadHandle = thread;
        if (pthread_create(thread, nullptr, threadEntryPoint, this) != 0)
        {
            delete thread;
            mThreadHandle = nullptr;
            return false;
        }
#endif

        mStatus = ThreadStatus::eIdle;
        return true;
    }

    void SingleThreadManager::stop()
    {
        if (mStatus == ThreadStatus::eStopped)
            return;

        {
            std::lock_guard<std::mutex> lock(mMtx);
            mStopRequested = true;
        }
        mCv.notify_all();

#ifdef _WIN32
        WaitForSingleObject(mThreadHandle, INFINITE);
        CloseHandle(mThreadHandle);
#else
        pthread_t *thread = (pthread_t *)mThreadHandle;
        pthread_join(*thread, nullptr);
        delete thread;
#endif

        mThreadHandle = nullptr;
        mStatus = ThreadStatus::eStopped;
    }

    bool SingleThreadManager::submitTask(Task task, void *data, size_t dataSize)
    {
        if (mStatus == ThreadStatus::eBusy || mStopRequested || task == nullptr)
        {
            return false;
        }

        // Copy the function args
        void *copiedData = operator new(dataSize);
        memcpy(copiedData, data, dataSize);
        {
            std::lock_guard<std::mutex> lock(mMtx);
            mCurrentTask = task;
            mTaskData = copiedData;
            mStatus = ThreadStatus::eBusy;
        }

        mCv.notify_one();
        return true;
    }

    ThreadStatus SingleThreadManager::getStatus() const
    {
        return mStatus;
    }

    void SingleThreadManager::waitForIdle()
    {
        std::unique_lock<std::mutex> lock(mMtx);
        mCv.wait(lock, [this]()
                 { return mStatus == ThreadStatus::eIdle; });
    }

    void *SingleThreadManager::threadEntryPoint(void *arg)
    {
        SingleThreadManager *manager = (SingleThreadManager *)arg;

        while (!manager->mStopRequested)
        {
            std::unique_lock<std::mutex> lock(manager->mMtx);
            manager->mCv.wait(lock, [&]
                              { return manager->mCurrentTask || manager->mStopRequested; });

            if (manager->mStopRequested)
                break;

            // Execute the task
            if (manager->mCurrentTask)
            {
                manager->mCurrentTask(manager->mTaskData);
                manager->mCurrentTask = nullptr;
                manager->mTaskData = nullptr;
                manager->mStatus = ThreadStatus::eIdle;
                manager->mCv.notify_all();
            }
        }

#ifdef _WIN32
        ExitThread(0);
#else
        pthread_exit(nullptr);
#endif
        return nullptr;
    }

} // namespace crmpm