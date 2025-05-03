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

#include "svd3.h"
#include "mat33.h"
#include "debug_logger.h"

void testSvd()
{
    const crmpm::Mat3x3f mat(1, 2, 5, 3, 1, 4, 2, 9, 3);
    crmpm::Mat3x3f u, vt, sigma;
    svd3(mat, u, vt, sigma);
    CR_DEBUG_LOG_INFO("%s", "u");
    CR_DEBUG_LOG_INFO("%f", u.col0.x);
    CR_DEBUG_LOG_INFO("%f", u.col0.y);
    CR_DEBUG_LOG_INFO("%f", u.col0.z);
    CR_DEBUG_LOG_INFO("%f", u.col1.x);
    CR_DEBUG_LOG_INFO("%f", u.col1.y);
    CR_DEBUG_LOG_INFO("%f", u.col1.z);
    CR_DEBUG_LOG_INFO("%f", u.col2.x);
    CR_DEBUG_LOG_INFO("%f", u.col2.y);
    CR_DEBUG_LOG_INFO("%f", u.col2.z);

    CR_DEBUG_LOG_INFO("%s", "v");
    CR_DEBUG_LOG_INFO("%f", vt.col0.x);
    CR_DEBUG_LOG_INFO("%f", vt.col0.y);
    CR_DEBUG_LOG_INFO("%f", vt.col0.z);
    CR_DEBUG_LOG_INFO("%f", vt.col1.x);
    CR_DEBUG_LOG_INFO("%f", vt.col1.y);
    CR_DEBUG_LOG_INFO("%f", vt.col1.z);
    CR_DEBUG_LOG_INFO("%f", vt.col2.x);
    CR_DEBUG_LOG_INFO("%f", vt.col2.y);
    CR_DEBUG_LOG_INFO("%f", vt.col2.z);

    CR_DEBUG_LOG_INFO("%s", "u");
    CR_DEBUG_LOG_INFO("%f", sigma.col0.x);
    CR_DEBUG_LOG_INFO("%f", sigma.col0.y);
    CR_DEBUG_LOG_INFO("%f", sigma.col0.z);
    CR_DEBUG_LOG_INFO("%f", sigma.col1.x);
    CR_DEBUG_LOG_INFO("%f", sigma.col1.y);
    CR_DEBUG_LOG_INFO("%f", sigma.col1.z);
    CR_DEBUG_LOG_INFO("%f", sigma.col2.x);
    CR_DEBUG_LOG_INFO("%f", sigma.col2.y);
    CR_DEBUG_LOG_INFO("%f", sigma.col2.z);
}

int main()
{
    testSvd();
    return 0;
}