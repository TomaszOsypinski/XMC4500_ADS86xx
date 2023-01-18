/**
 *******************************************************************************
 * @file    dac.h
 * @version 1.0.0
 * @date    2023-01-18
 * @brief   DAC functions
 * @author  Tomasz Osypinski<br>
 *
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

#ifndef DAC_H_
#define DAC_H_

/*
 *******************************************************************************
 * the includes
 *******************************************************************************
 */
#include "type.h"
#include <xmc_dac.h>

/**
 * @addtogroup TEST_ADS86XX_MODULE
 * @{
 */

/**
 * @addtogroup DAC
 * @brief DAC functions
 * @{
 */

/*
 *******************************************************************************
 * #defines
 *******************************************************************************
 */
/* Switch on pedantic checking */
#if defined ( __GNUC__ )
#pragma GCC diagnostic push
#pragma GCC diagnostic warning "-Wpedantic"
#pragma GCC diagnostic warning "-Wextra"
#endif

/* DAC HW channels */
#define DAC_CH_NR_0 (0U)

#define DAC_CH_NR_1 (1U)

/* DAC max value for signed configuration. DAC is 12 bit */
#define DAC_MAX_VAL_I16 ((int16_t)((1 << 12) - 1))          /* 2.5V at output */
#define DAC_MAX_VAL_F32 ((float32_t)DAC_MAX_VAL_I16)

#define DAC_MIN_VAL_I16 ((int16_t)(-DAC_MAX_VAL_I16 - 1))   /* 0.3V at output */
#define DAC_MIN_VAL_F32 ((float32_t)DAC_MIN_VAL_I16)

/*
 *******************************************************************************
 * typedefs
 *******************************************************************************
 */

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
__STATIC_INLINE void DAC_Ch0_Int16(int16_t val)
{
    val = val > DAC_MAX_VAL_I16 ? DAC_MAX_VAL_I16 : val;
    val = val < DAC_MIN_VAL_I16 ? DAC_MIN_VAL_I16 : val;

    XMC_DAC_CH_Write(XMC_DAC0, DAC_CH_NR_0, (uint16_t)(val & 0x0FFF));
}

__STATIC_INLINE void DAC_Ch0_PU_2_DAC(float32_t val)
{
    val *= DAC_MAX_VAL_F32;

    val = val > DAC_MAX_VAL_F32 ? DAC_MAX_VAL_F32 : val;
    val = val < DAC_MIN_VAL_F32 ? DAC_MIN_VAL_F32 : val;

    XMC_DAC_CH_Write(XMC_DAC0, DAC_CH_NR_0, (uint16_t)((int16_t)val & 0x0FFF));
}

__STATIC_INLINE void DAC_Ch1_Int16(int16_t val)
{
    val = val > DAC_MAX_VAL_I16 ? DAC_MAX_VAL_I16 : val;
    val = val < DAC_MIN_VAL_I16 ? DAC_MIN_VAL_I16 : val;

    XMC_DAC_CH_Write(XMC_DAC0, DAC_CH_NR_1, (uint16_t)(val & 0x0FFF));
}

__STATIC_INLINE void DAC_Ch1_PU_2_DAC(float32_t val)
{
    val *= DAC_MAX_VAL_F32;

    val = val > DAC_MAX_VAL_F32 ? DAC_MAX_VAL_F32 : val;
    val = val < DAC_MIN_VAL_F32 ? DAC_MIN_VAL_F32 : val;

    XMC_DAC_CH_Write(XMC_DAC0, DAC_CH_NR_1, (uint16_t)((int16_t)val & 0x0FFF));
}

/**
 *******************************************************************************
 * @brief Initialization function of DAC module
 *******************************************************************************
 */
void DAC_Init(void);

/* Switch off pedantic checking */
#if defined ( __GNUC__ )
#pragma GCC diagnostic pop
#endif

#ifdef __cplusplus
}
#endif

/**
 * @}
 */

/**
 * @}
 */

#endif /* end of DAC_H_ */
