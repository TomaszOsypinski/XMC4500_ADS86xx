/**
 *******************************************************************************
 * @file    module_time.h
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

#ifndef MODULE_TIME_H_
#define MODULE_TIME_H_

/*
 *******************************************************************************
 * the includes
 *******************************************************************************
 */
#include "type.h"
#include "test_assert.h"
#include <xmc_common.h>

/**
 * @addtogroup TEST_ADS86XX_MODULE
 * @{
 */

/**
 * @addtogroup Utility
 * @brief Utility functions for time calculation e.g. execution time, time stamp
 * @{
 */

/*
 *******************************************************************************
 * #defines
 *******************************************************************************
 */
#define MODULE_TIME_SYS_CORE_CLK       (120000000.0f)
#define MODULE_TIME_CLK_TICKS_TO_US    (1.0e6f / MODULE_TIME_SYS_CORE_CLK)
#define MODULE_TIME_US_TO_CLK_TICKS    ((uint32_t)(MODULE_TIME_SYS_CORE_CLK / 1.0e6f))

/*
 *******************************************************************************
 * typedefs
 *******************************************************************************
 */
typedef struct proc_time
{
    uint32_t t0;
    uint32_t t1;
    volatile float32_t isr1;
    volatile float32_t isr1_max;
} proc_time_t;

typedef uint32_t ts_hr_t;

typedef uint32_t ts_t;

/*
 *******************************************************************************
 * globals
 *******************************************************************************
 */

/*
 *******************************************************************************
 * the function prototypes
 *******************************************************************************
 */
#ifdef __cplusplus
extern "C"
{
#endif
/**
 *******************************************************************************
 * @brief   Get value of Cycle Count Register. Incremented by each tick of main
 *          clock OSCHP_FREQUENCY
 * @param   none
 * @return  Value of Cycle Count Register
 *******************************************************************************
 */
__STATIC_INLINE uint32_t MODULE_TIME_GetCYCCNT(void)
{
    uint32_t tmp = (DWT->CYCCNT);

    return (tmp);
}
/**
 *******************************************************************************
 * @brief
 * @param
 * @return
 *******************************************************************************
 */
#ifdef CALC_PROC_TIME
__STATIC_INLINE void MODULE_TIME_ProcTimeSet(uint32_t * const t)
{
    *t = (DWT->CYCCNT);
}
#else
__STATIC_INLINE void MODULE_TIME_ProcTimeSet(uint32_t * const t)
{
    (void)t;
}
#endif

/**
 *******************************************************************************
 * @brief
 * @param
 * @return
 *******************************************************************************
 */
#ifdef CALC_PROC_TIME
__STATIC_INLINE void MODULE_TIME_ProcTimeCalc(proc_time_t * const s)
{
    uint32_t delta;

    MODULE_TIME_ProcTimeSet(&(s->t1));

    if(s->t1 > s->t0)
    {
        delta = s->t1 - s->t0;
    }
    else
    {
        delta = s->t1 - s->t0 + 0xFFFFFFFFUL;
    }

    s->isr1 = ((float32_t)delta * MODULE_TIME_CLK_TICKS_TO_US);

    if(s->isr1 > s->isr1_max)
    {
        s->isr1_max = s->isr1;
    }
}
#else
__STATIC_INLINE void MODULE_TIME_ProcTimeCalc(proc_time_t * const s)
{
    (void)s;
}
#endif

/**
 *******************************************************************************
 * @brief   Initialization of core debug registers to enable CYCCNT register in
 *          Data Watchpoint and Trace
 * @param   none
 * @return  none
 *******************************************************************************
 */
void MODULE_TIME_InitDWT(void);

/**
 *******************************************************************************
 * @brief   Get time stamp to count time [period_in_us].
 *          Using CYCCNT register with resolution SystemCoreClock (120MHz).
 *          High resolution version.
 *          Note: Maximum time to count is (2^31*1000000)/SystemCoreClock in us
 * @param   Period in [us]
 * @return  time stamp
 *******************************************************************************
 */
ts_hr_t MODULE_TIME_GetTimeStampHR(const ts_hr_t period_in_us);

/**
 *******************************************************************************
 * @brief   Check [period_in_us] time is expired
 *          High resolution version.
 * @param   timeStamp from MODULE_GetTimeStampHR(period_in_us)
 * @return  true - time [period_in_us] is expired, otherwise
 *          false time is not expired
 *******************************************************************************
 */
bool MODULE_TIME_IsTimeExpiredHR(const ts_hr_t timeStamp);

/**
 *******************************************************************************
 * @brief   Get time stamp to count time [period_in_ms].
 *          Using gSysTickGlobalTimer
 *          witch is incremented in SysTick interrupt. Resolution 1ms.
 *          Note: Maximum time to count is 2^31* in ms
 * @param   Period in [ms]
 * @return  time stamp
 *******************************************************************************
 */
ts_t MODULE_TIME_GetTimeStamp(const ts_t period_in_ms);

/**
 *******************************************************************************
 * @brief   Check [period_in_ms] time is expired
 * @param   timeStamp from MODULE_GetTimeStamp(period_in_ms)
 * @return  true - time [period_in_ms] is expired, otherwise
 *          false time is not expired
 *******************************************************************************
 */
bool MODULE_TIME_IsTimeExpired(const ts_t timeStamp);

/**
 *******************************************************************************
 * @brief   Increment timer in 1ms SysTick interrupt
 * @param   none
 * @return  none
 *******************************************************************************
 */
void MODULE_TIME_Inc_Timer(void);
#ifdef __cplusplus
}
#endif

/**
 * @}
 */

/**
 * @}
 */

#endif /* end of MODULE_TIME_H_ */
