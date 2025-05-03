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

#ifndef CR_DEBUG_LOGGER_H
#define CR_DEBUG_LOGGER_H

#include <cstdarg>
#include <cstdio>
#include <ctime>
#include <cstring>

#ifdef _WIN32
#define NOMINMAX  // Avoid min/max macros in Windows.h
#include <Windows.h>
#endif

// Function for formatted logging
static inline void cr_debug_log(const char *level, const char *color_code, const char *file, int line, const char *format, ...)
{
    char buffer[512];
    va_list args;

    va_start(args, format);
    vsnprintf(buffer, sizeof(buffer), format, args);
    va_end(args);

#ifdef _WIN32
    // Console output
    printf("%s[%s]\033[0m %s, Line: %d, %s\n", color_code, level, file, line, buffer);

    // OutputDebugString for Windows Debug Logger
    char dbg_buffer[600];
    snprintf(dbg_buffer, sizeof(dbg_buffer), "[%s] %s, Line: %d, %s\n", level, file, line, buffer);
    OutputDebugStringA(dbg_buffer);
#else
    // Console output (Linux/macOS)
    printf("%s[%s]\033[0m %s, Line: %d, %s\n", color_code, level, file, line, buffer);
    fflush(stdout);
#endif
}

#if defined(CR_ENABLE_DEBUG_LOGGER)
#define CR_DEBUG_LOG_INFO(format, ...) \
    cr_debug_log("INFO", "\033[1;32m", __FILE__, __LINE__, format, ##__VA_ARGS__)

#define CR_DEBUG_LOG_WARNING(format, ...) \
    cr_debug_log("WARNING", "\033[1;33m", __FILE__, __LINE__, format, ##__VA_ARGS__)

#define CR_DEBUG_LOG_ERROR(format, ...) \
    cr_debug_log("ERROR", "\033[1;31m", __FILE__, __LINE__, format, ##__VA_ARGS__)

#define CR_DEBUG_LOG_EXECUTION_TIME(expr)                                                                                 \
    do                                                                                                                    \
    {                                                                                                                     \
        clock_t CR_DEBUG_LOG_startTime = clock();                                                                         \
        expr;                                                                                                             \
        clock_t CR_DEBUG_LOG_endTime = clock();                                                                           \
        double CR_DEBUG_LOG_duration = ((double)(CR_DEBUG_LOG_endTime - CR_DEBUG_LOG_startTime)) / CLOCKS_PER_SEC * 1000; \
        printf("Execution time for %s: %.2f ms\n", #expr, CR_DEBUG_LOG_duration);                                         \
    } while (0)

#else

#define CR_DEBUG_LOG_INFO(format, ...)
#define CR_DEBUG_LOG_WARNING(format, ...)
#define CR_DEBUG_LOG_EXECUTION_TIME(expr) \
    do                                    \
    {                                     \
        expr;                             \
    } while (0)

#endif // _DEBUG

#endif // !CR_DEBUG_LOGGER_H
