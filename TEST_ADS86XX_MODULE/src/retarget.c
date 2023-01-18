/**
 *******************************************************************************
 * @file    retarget.c
 * @version 1.0.0
 * @date    2023-01-18
 * @brief   Retargeting libc functions
 * @author  Tomasz Osypinski <br>
 *

 * Change History
 * --------------
 *
 * 2023-01-18:
 *      - Initial <br>
 *******************************************************************************
 */

/*
 * Copyright (C) 2023, Tomasz Osypinski. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *   notice, this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright
 *   notice, this list of conditions and the following disclaimer in the
 *   documentation and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR AND CONTRIBUTORS ``AS IS'' AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE AUTHOR OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 */

/*
 *******************************************************************************
 * the includes
 *******************************************************************************
 */
#include "test_assert.h"

#include <xmc_common.h>

/* libc headers */
#include <errno.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/time.h>
#include <sys/times.h>

/* Switch on pedantic checking */
#if defined ( __GNUC__ )
#pragma GCC diagnostic push
#pragma GCC diagnostic warning "-Wpedantic"
#pragma GCC diagnostic warning "-Wall"
#pragma GCC diagnostic warning "-Wextra"
#pragma GCC diagnostic warning "-Wconversion"
#pragma GCC diagnostic warning "-Wmissing-declarations"
#pragma GCC diagnostic ignored "-Wmissing-field-initializers"
#endif
/*
 *******************************************************************************
 * #defines
 *******************************************************************************
 */
/**
 * Redirect printf output to Serial Wire Debug. Only XMC family
 */
#define PRINTF_2_SWV    (0)

/**
 * Redirect printf output to RS232
 */
#define PRINTF_2_RS232  (1)

/**
 * Set printf output
 */
#define PRINTF_OUTPUT PRINTF_2_SWV
/*
 *******************************************************************************
 * global variables
 *******************************************************************************
 */

/*
 *******************************************************************************
 * local variables
 *******************************************************************************
 */

/*
 *******************************************************************************
 * the function prototypes
 *******************************************************************************
 */

/*
 *******************************************************************************
 * the external functions
 *******************************************************************************
 */
int
_chown(const char* path, uid_t owner, gid_t group);

int
_close(int fildes);

int
_execve(char* name, char** argv, char** env);

int
_fork(void);

int
_fstat(int fildes, struct stat* st);

int
_getpid(void);

int
_gettimeofday(struct timeval* ptimeval, void* ptimezone);

int
_isatty(int file);

int
_kill(int pid, int sig);

int
_link(char* existing, char* _new);

int
_lseek(int file, int ptr, int dir);

int
_open(char* file, int flags, int mode);

int
_read(int file, char* ptr, int len);

int
_readlink(const char* path, char* buf, size_t bufsize);

int
_stat(const char* file, struct stat* st);

int
_symlink(const char* path1, const char* path2);

clock_t
_times(struct tms* buf);

int
_unlink(char* name);

int
_wait(int* status);

int
_write(int file, char* ptr, int len);

/*
 *******************************************************************************
 * the functions
 *******************************************************************************
 */

int __attribute__((weak))
_chown(const char* path __attribute__((unused)),
uid_t owner __attribute__((unused)), gid_t group __attribute__((unused)))
{
    errno = ENOSYS;
    return -1;
}

int __attribute__((weak))
_close(int fildes __attribute__((unused)))
{
    errno = ENOSYS;
    return -1;
}

int __attribute__((weak))
_execve(char* name __attribute__((unused)), char** argv __attribute__((unused)),
char** env __attribute__((unused)))
{
    errno = ENOSYS;
    return -1;
}

int __attribute__((weak))
_fork(void)
{
    errno = ENOSYS;
    return -1;
}

int __attribute__((weak))
_fstat(int fildes __attribute__((unused)),
struct stat* st __attribute__((unused)))
{
    errno = ENOSYS;
    return -1;
}

int __attribute__((weak))
_getpid(void)
{
    errno = ENOSYS;
    return -1;
}

int __attribute__((weak))
_gettimeofday(struct timeval* ptimeval __attribute__((unused)),
void* ptimezone __attribute__((unused)))
{
    errno = ENOSYS;
    return -1;
}

int __attribute__((weak))
_isatty(int file __attribute__((unused)))
{
    errno = ENOSYS;
    return 0;
}

int __attribute__((weak))
_kill(int pid __attribute__((unused)), int sig __attribute__((unused)))
{
    errno = ENOSYS;
    return -1;
}

int __attribute__((weak))
_link(char* existing __attribute__((unused)),
char* _new __attribute__((unused)))
{
    errno = ENOSYS;
    return -1;
}

int __attribute__((weak))
_lseek(int file __attribute__((unused)), int ptr __attribute__((unused)),
int dir __attribute__((unused)))
{
    errno = ENOSYS;
    return -1;
}

int __attribute__((weak))
_open(char* file __attribute__((unused)), int flags __attribute__((unused)),
int mode __attribute__((unused)))
{
    errno = ENOSYS;
    return -1;
}

int __attribute__((weak))
_read(int file __attribute__((unused)), char* ptr __attribute__((unused)),
int len __attribute__((unused)))
{
    errno = ENOSYS;
    return -1;
}

int __attribute__((weak))
_readlink(const char* path __attribute__((unused)),
char* buf __attribute__((unused)), size_t bufsize __attribute__((unused)))
{
    errno = ENOSYS;
    return -1;
}

int __attribute__((weak))
_stat(const char* file __attribute__((unused)),
struct stat* st __attribute__((unused)))
{
    errno = ENOSYS;
    return -1;
}

int __attribute__((weak))
_symlink(const char* path1 __attribute__((unused)),
const char* path2 __attribute__((unused)))
{
    errno = ENOSYS;
    return -1;
}

clock_t __attribute__((weak))
_times(struct tms* buf __attribute__((unused)))
{
    errno = ENOSYS;
    return ((clock_t)-1);
}

int __attribute__((weak))
_unlink(char* name __attribute__((unused)))
{
    errno = ENOSYS;
    return -1;
}

int __attribute__((weak))
_wait(int* status __attribute__((unused)))
{
    errno = ENOSYS;
    return -1;
}

#ifndef PRINT_INFO
int __attribute__((weak))
_write(int file __attribute__((unused)), char* ptr __attribute__((unused)),
int len __attribute__((unused)))
{
    errno = ENOSYS;
    return -1;
}
#else
static void sendChar(const uint8_t *ptr)
{
    #if (PRINTF_OUTPUT == PRINTF_2_SWV)
        ITM_SendChar((uint32_t)(*ptr));
    #elif (PRINTF_OUTPUT == PRINTF_2_RS232)
    #error "Not implemented!"
    #else
    #error "Set PRINTF_OUTPUT!"
    #endif
}

int __attribute__((weak))
_write(int file __attribute__((unused)), char* ptr, int len)
{
    const int ret_val = len;

    while(len > 0)
    {
        sendChar((const uint8_t *)ptr);
        ptr++;
        len--;
    }

    return ret_val;
}
#endif

/* Switch off pedantic checking */
#if defined ( __GNUC__ )
#pragma GCC diagnostic pop
#endif
