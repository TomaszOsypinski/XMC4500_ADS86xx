/**
 *******************************************************************************
 * @file    pin_interrupt.h
 * @version 1.0.0
 * @date    2023-01-27
 * @brief   Pin interrupt initialization function
 * @author  Tomasz Osypinski
 *
 * Change History
 * --------------
 *
 * 2023-01-27:
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
#include "pin_interrupt.h"

/* Switch on pedantic checking */
#if defined ( __GNUC__ )
#pragma GCC diagnostic push
#pragma GCC diagnostic warning "-Wpedantic"
#pragma GCC diagnostic warning "-Wall"
#pragma GCC diagnostic warning "-Wextra"
#pragma GCC diagnostic warning "-Wconversion"
#pragma GCC diagnostic warning "-Wunused-function"
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
void PIN_INTERRUPT_Init(const pin_interrupt_t * const handle)
{
    XMC_ASSERT("pin_interrupt_Init: pin_interrupt APP handle function pointer uninitialized", (handle != NULL));

    /* Initializes input pin characteristics */
    XMC_GPIO_Init(handle->port, handle->pin, &handle->gpio_config);
    /* ERU Event Trigger Logic Hardware initialization based on UI */
    XMC_ERU_ETL_Init(handle->eru, handle->etl, &handle->etl_config);
    /* OGU is configured to generate event on configured trigger edge */
    XMC_ERU_OGU_SetServiceRequestMode(handle->eru, handle->ogu, XMC_ERU_OGU_SERVICE_REQUEST_ON_TRIGGER);
    #if (UC_FAMILY == XMC1)
    /* Configure NVIC node and priority */
    NVIC_SetPriority((IRQn_Type)handle->IRQn, handle->irq_priority);
    #else
    /* Configure NVIC node, priority and subpriority */
    NVIC_SetPriority((IRQn_Type)handle->IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),
    handle->irq_priority, handle->irq_subpriority));
    #endif
    #if (UC_SERIES == XMC14)
    XMC_SCU_SetInterruptControl((IRQn_Type)handle->IRQn, (XMC_SCU_IRQCTRL_t)handle->irqctrl);
    #endif
    if(true == handle->enable_at_init)
    {
        /* Clear pending interrupt before enabling it */
        NVIC_ClearPendingIRQ((IRQn_Type)handle->IRQn);
        /* Enable NVIC node */
        NVIC_EnableIRQ((IRQn_Type)handle->IRQn);
    }
}
/* Switch off pedantic checking */
#if defined ( __GNUC__ )
#pragma GCC diagnostic pop
#endif
