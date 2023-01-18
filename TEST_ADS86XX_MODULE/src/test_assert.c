/**
 *******************************************************************************
 * @file    test_assert.h
 * @version 1.0.0
 * @date    2023-01-18
 * @brief   API for assertion
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
#include <stdio.h>

#if (TEST_ASSERT_BKPT_EN == 1)
/*
 * Use appropriate header file to service break point. Depends on used
 * platform
 */
#if (DEVICE_FAMILY == DEVICE_FAMILY_XMC)
#include <xmc_common.h>
#elif (DEVICE_FAMILY == DEVICE_FAMILY_TMS)
/* header for TMS */
#else
#warning "Set DEVICE_FAMILY!"
#endif
#endif

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
#ifdef TEST_ASSERT_FULL
void TEST_ASSERT_MSG_Handler(uint8_t  const *file,
                             uint32_t const line,
                             uint8_t  const *expr,
                             uint8_t  const *msg)
{

    printf("'%s' at %s line %lu %s\n", expr, file, line, msg);

  #if (TEST_ASSERT_BKPT_EN == 1)
    #if (DEVICE_FAMILY == DEVICE_FAMILY_XMC)
    __BKPT(0);
    #elif (DEVICE_FAMILY == DEVICE_FAMILY_TMS)
    #error "Not implemented!"
    #else
    #warning "Set DEVICE_FAMILY!"
    #endif
 #endif

  #ifndef CODE_PROVE_CHECK
    while(1)
    {
        /* Endless loop */
    }
  #endif
}
#endif

 /* Switch off pedantic checking */
 #if defined ( __GNUC__ )
 #pragma GCC diagnostic pop
 #endif
