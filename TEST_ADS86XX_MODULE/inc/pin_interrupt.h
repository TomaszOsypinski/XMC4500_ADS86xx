/**
 *******************************************************************************
 * @file    pin_interrupt.h
 * @version 1.0.0
 * @date    2023-01-27
 * @brief   Pin interrupt utilities
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

#ifndef PIN_INTERRUPT_H_
#define PIN_INTERRUPT_H_

/*
 *******************************************************************************
 * the includes
 *******************************************************************************
 */
#include <xmc_eru.h>
#include <xmc_gpio.h>
#include <xmc_common.h>

/**
 * @addtogroup TEST_ADS86XX_MODULE
 * @{
 */

/**
 * @addtogroup Utility
 * @brief Pin interrupt utilities
 * @{
 */

/*
 *******************************************************************************
 * #defines
 *******************************************************************************
 */

/*
 *******************************************************************************
 * typedefs
 *******************************************************************************
 */
/**
 * @brief Configuration structure for pin_interrupt APP
 */
typedef struct pin_interrupt
{
    XMC_ERU_t *eru;                     /**< Mapped ERU module */
    XMC_GPIO_PORT_t *port;              /**< Mapped port number */
    XMC_GPIO_CONFIG_t gpio_config;      /**< Initializes the input pin characteristics */
    XMC_ERU_ETL_CONFIG_t etl_config;    /**< reference to ERUx_ETLy (x = [0..1], y = [0..4])
     module configuration */
    #if (UC_SERIES == XMC14)
    XMC_SCU_IRQCTRL_t irqctrl;          /**< selects the interrupt source for a NVIC interrupt node*/
    #endif
    IRQn_Type IRQn;                     /**< Mapped NVIC Node */
    uint8_t irq_priority;               /**< Node Interrupt Priority */
    #if (UC_FAMILY == XMC4)
    uint8_t irq_subpriority;            /**< Node Interrupt SubPriority only valid for XMC4x */
    #endif
    uint8_t etl;                        /** < ETLx channel (x = [0..3])*/
    uint8_t ogu;                        /** < OGUy channel (y = [0..3])*/
    uint8_t pin;                        /** < Mapped pin number */
    bool enable_at_init;                /**< Interrupt enable for Node at initialization*/
} pin_interrupt_t;

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
void PIN_INTERRUPT_Init(const pin_interrupt_t * const handle);
#ifdef __cplusplus
}
#endif

/**
 * @}
 */

/**
 * @}
 */

#endif /* end of PIN_INTERRUPT_H_ */
