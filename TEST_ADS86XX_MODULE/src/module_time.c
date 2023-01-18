/**
 *******************************************************************************
 * @file    module_time.c
 * @version 1.0.0
 * @date    2023-01-19
 * @brief   Utility functions for time calculation e.g. execution time,
 *          time stamp
 * @author  Tomasz Osypinski<br>
 *
 *
 * Change History
 * --------------
 *
 * 2023-01-19:
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
#include "module_time.h"

/* Switch on pedantic checking */
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
static volatile uint32_t timer_inc_1ms;

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


/*
 *******************************************************************************
 * the functions
 *******************************************************************************
 */
void MODULE_TIME_InitDWT(void)
{
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0UL;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;

    TEST_ASSERT_EQUAL_MESSAGE(MODULE_TIME_SYS_CORE_CLK, SystemCoreClock,\
                              "MODULE_TIME_SYS_CORE_CLK is not equal SystemCoreClock!");
}

ts_hr_t MODULE_TIME_GetTimeStampHR(const ts_hr_t period_in_us)
{
    TEST_ASSERT_MESSAGE((uint32_t)((float32_t)(1U << 31U) * MODULE_TIME_CLK_TICKS_TO_US) > period_in_us,\
                         "Period in [us] is too much!");

    uint32_t time_now = (DWT->CYCCNT);

    return (ts_hr_t)(time_now  + ((uint32_t)period_in_us * MODULE_TIME_US_TO_CLK_TICKS));
}

bool MODULE_TIME_IsTimeExpiredHR(const ts_hr_t timeStamp)
{
    bool status;
    uint32_t time_now = (DWT->CYCCNT);

    if(time_now > (uint32_t)timeStamp)
    {
        if((time_now - (uint32_t)timeStamp) < 0x80000000UL)
        {
            status = true;
        }
        else
        {
            status = false;
        }
    }
    else
    {
        if(((uint32_t)timeStamp - time_now) > 0x80000000UL)
        {
            status = true;
        }
        else
        {
            status = false;
        }
    }

    return status;
}

ts_t MODULE_TIME_GetTimeStamp(const ts_t period_in_ms)
{
    TEST_ASSERT_MESSAGE((uint32_t)((uint32_t)(1U << 31U)) > period_in_ms,\
                                   "Period in [ms] is too much!");

    uint32_t time_now = timer_inc_1ms;

    return (ts_t)(time_now + period_in_ms);
}

bool MODULE_TIME_IsTimeExpired(const ts_t timeStamp)
{
    bool status;

    uint32_t time_now = timer_inc_1ms;

    if(time_now > (uint32_t)timeStamp)
    {
        if((time_now - timeStamp) < 0x80000000UL)
        {
            status = true;
        }
        else
        {
            status = false;
        }
    }
    else
    {
        if(((uint32_t)timeStamp - time_now) > 0x80000000UL)
        {
            status = true;
        }
        else
        {
            status =  false;
        }
    }

    return status;
}

void MODULE_TIME_Inc_Timer(void)
{
    timer_inc_1ms++;
}

/* Switch off pedantic checking */
#if defined ( __GNUC__ )
#pragma GCC diagnostic pop
#endif
