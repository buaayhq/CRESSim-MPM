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

#include <iostream>

#include "single_thread_manager.h"

// Example task
void exampleTask(void *data)
{
    int *num = (int *)data;
    std::cout << "Executing task with value: " << *num << std::endl;
}

int main()
{
    crmpm::SingleThreadManager threadManager;

    if (!threadManager.start())
    {
        std::cerr << "Failed to start the thread manager!" << std::endl;
        return -1;
    }

    int taskData1 = 42;
    threadManager.submitTask(exampleTask, &taskData1, sizeof(int));

    int taskData2 = 99;
    threadManager.submitTask(exampleTask, &taskData2, sizeof(int));

    threadManager.waitForIdle();

    // Stop the thread manager after all tasks are done
    threadManager.stop();

    if (!threadManager.start())
    {
        std::cerr << "Failed to start the thread manager!" << std::endl;
        return -1;
    }

    taskData2 = 102;
    threadManager.submitTask(exampleTask, &taskData2, sizeof(int));

    threadManager.waitForIdle();

    return 0;
}
