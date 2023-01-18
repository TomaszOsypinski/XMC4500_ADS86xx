/**
 *******************************************************************************
 * @file    dac.c
 * @version 1.0.0
 * @date    2023-01-18
 * @brief   Description of this module.
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

/*
 *******************************************************************************
 * the includes
 *******************************************************************************
 */
#include "dac.h"
#include "module_time.h"

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
void DAC_Init(void)
{
    const XMC_DAC_CH_CONFIG_t ch_config =
    {
        .data_type       = XMC_DAC_CH_DATA_TYPE_SIGNED,
        .output_negation = XMC_DAC_CH_OUTPUT_NEGATION_DISABLED,
        .output_scale    = XMC_DAC_CH_OUTPUT_SCALE_NONE,
        .output_offset   = 0U
    };
    XMC_DAC_CH_Init(XMC_DAC0, DAC_CH_NR_0, &ch_config);
    XMC_DAC_CH_Init(XMC_DAC0, DAC_CH_NR_1, &ch_config);

    XMC_DAC_CH_StartSingleValueMode(XMC_DAC0, DAC_CH_NR_0);
    XMC_DAC_CH_StartSingleValueMode(XMC_DAC0, DAC_CH_NR_1);

    /* Startup time is 30 μs. Time from output enabling till code valid ±16 LSB */
    uint32_t timeStamp = MODULE_TIME_GetTimeStampHR(35UL);
    do
    {

    } while(!(MODULE_TIME_IsTimeExpiredHR(timeStamp)));

    /* Set middle value (1.4V) on DAC outputs */
    DAC_Ch0_Int16(0);
    DAC_Ch1_Int16(0);
}

/* Switch off pedantic checking */
#if defined ( __GNUC__ )
#pragma GCC diagnostic pop
#endif
