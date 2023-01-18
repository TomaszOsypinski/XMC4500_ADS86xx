/**
 *******************************************************************************
 * @file    main.c
 * @version 1.0.0
 * @date    2023-01-19
 * @brief   main function
 * @author  Tomasz Osypinski<br>
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
#include "ads86xx.h"
#include "dac.h"

#include <xmc_gpio.h>
#include <xmc_common.h>

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
#define PERIOD_1S (1000000) /* in us */
#define LED1 P6_6

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
static volatile ts_hr_t ts_1s                = 0UL;
static volatile bool ads86_goto_polling_mode = false;
static volatile bool ads86_goto_isr_mode     = false;
/*
 *******************************************************************************
 * the function prototypes
 *******************************************************************************
 */
int main(void);

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
int main(void)
{
    XMC_GPIO_CONFIG_t config = {.mode            = XMC_GPIO_MODE_OUTPUT_PUSH_PULL,
                                .output_level    = XMC_GPIO_OUTPUT_LEVEL_HIGH,
                                .output_strength = XMC_GPIO_OUTPUT_STRENGTH_STRONG_SOFT_EDGE};

    XMC_GPIO_Init(LED1, &config);

    MODULE_TIME_InitDWT();

    ADS86XX_Init(ADS86_RANGE_SEL_BI_0_625_VREF);

    DAC_Init();

    ts_1s = MODULE_TIME_GetTimeStampHR(PERIOD_1S);

    /* forever loop */
    while(1U)
    {
        if(MODULE_TIME_IsTimeExpiredHR(ts_1s))
        {
            ts_1s = MODULE_TIME_GetTimeStampHR(PERIOD_1S);
            XMC_GPIO_ToggleOutput(LED1);
            ADS86XX_PrintInfo();
        }

        if(ads86_goto_polling_mode)
        {
            ads86_goto_polling_mode = false;
            /* stop ISR mode and go to polling mode */
            ADS86XX_Start_ServiceInPolling();
        }

        if(ads86_goto_isr_mode)
        {
            ads86_goto_isr_mode = false;
            /* stop polling mode and go to ISR mode */
            ADS86XX_Start_ServiceInIsr();
        }

        /* Service in polling if was selected */
        ADS86XX_ServiceInPolling();
    }
}
/* Switch off pedantic checking */
#if defined ( __GNUC__ )
#pragma GCC diagnostic pop
#endif
