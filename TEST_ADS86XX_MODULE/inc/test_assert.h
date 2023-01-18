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

#ifndef TEST_ASSERT_H_
#define TEST_ASSERT_H_

/*
 *******************************************************************************
 * the includes
 *******************************************************************************
 */
#include <stddef.h>
#include <stdint.h>

/**
 * @addtogroup TEST_ADS86XX_MODULE
 * @{
 */

/**
 * @addtogroup Utility
 * @brief API for assertion
 * @{
 */

/*
 *******************************************************************************
 * #defines
 *******************************************************************************
 */
/**
 * XMC microcontroller family: e.g. XMC4400, XMC4500, XMC4700
 */
#define DEVICE_FAMILY_XMC (0)

/**
 * TMS microcontroller family: e.g. TMS320F28335, TMS320F28377D
 */
#define DEVICE_FAMILY_TMS (1)

/**
 * Set device family for service break point in TEST_ASSERT_Handler()
 */
#define DEVICE_FAMILY DEVICE_FAMILY_XMC

/**
 * Enable / disable break point in TEST_ASSERT_Handler()
 * 0 - disable, 1 - enable
 */
#ifdef MODULE_DEBUG
#define TEST_ASSERT_BKPT_EN (1)
#else
#define TEST_ASSERT_BKPT_EN (0)
#endif

#ifdef  TEST_ASSERT_FULL
#ifndef PRINT_INFO
#define PRINT_INFO
#endif

/**
 * Macro for assertion. Full version if TEST_ASSERT_FULL == 1, otherwise void
 * version .
 */
#define TEST_ASSERT_MSG(expr, msg) do { if(!((volatile bool)expr))\
                                        {\
                                            TEST_ASSERT_MSG_Handler((uint8_t *)__FILE__,\
                                                                    __LINE__,\
                                                                    (uint8_t *)#expr,\
                                                                    (uint8_t *)msg);\
                                        }\
                                      } while(0)

#else
#define TEST_ASSERT_MSG(expr, msg) { ; }
#endif    /* ACU_ASSERT_FULL */

/*-------------------------------------------------------
 * Test Asserts (simple)
 *-------------------------------------------------------*/
/**
 * Assert expression to TRUE
 */
#define TEST_ASSERT(condition)\
        TEST_ASSERT_MSG((condition), "Expression Evaluated To FALSE")

/**
 * Assert expression to expected TRUE
 */
#define TEST_ASSERT_TRUE(condition)\
        TEST_ASSERT_MSG((condition), "Expected TRUE Was FALSE")

/**
 * Assert expression to expected FALSE
 */
#define TEST_ASSERT_FALSE(condition)\
        TEST_ASSERT_MSG(!(condition), "Expected FALSE Was TRUE")

/**
 * Assert expression to expected NULL pointer
 */
#define TEST_ASSERT_NULL(pointer)\
        TEST_ASSERT_MSG(((void *)pointer == NULL), "Expected NULL")

/**
 * Assert expression to expected differ then NULL pointer
 */
#define TEST_ASSERT_NOT_NULL(pointer)\
    T   EST_ASSERT_MSG(((void *)pointer != NULL), "Expected Non-NULL")

/**
 * Assert expression to equal
 */
#define TEST_ASSERT_EQUAL(expected, actual)\
        TEST_ASSERT_MSG(((expected) == (actual)), "Expected equal")

/**
 * Assert expression to not equal
 */
#define TEST_ASSERT_NOT_EQUAL(expected, actual)\
        TEST_ASSERT_MSG(((expected) !=  (actual)), "Expected not equal")

/**
 * Assert expression to TRUE with additional messages
 */
#define TEST_ASSERT_MESSAGE(condition, message)\
        TEST_ASSERT_MSG((condition), (message))

/**
 *  Assert expression to expected TRUE with additional messages
 */
#define TEST_ASSERT_TRUE_MESSAGE(condition, message)\
        TEST_ASSERT_MSG((condition), (message))

/**
 * Assert expression to expected FALSE with additional messages
 */
#define TEST_ASSERT_FALSE_MESSAGE(condition, message)\
        TEST_ASSERT_MSG(!(condition), (message))

/**
 * Assert expression to expected NULL pointer with additional messages
 */
#define TEST_ASSERT_NULL_MESSAGE(pointer, message)\
        TEST_ASSERT_MSG(((void *)pointer == NULL), (message))

/**
 * Assert expression to expected differ then NULL pointer with additional
 * messages
 */
#define TEST_ASSERT_NOT_NULL_MESSAGE(pointer, message)\
        TEST_ASSERT_MSG(((void *)pointer != NULL), (message))

/**
 * Assert expression to equal with additional messages
 */
#define TEST_ASSERT_EQUAL_MESSAGE(expected, actual, message)\
        TEST_ASSERT_MSG(((expected) == (actual)), (message))

/**
 * Assert expression to not equal with additional messages
 */
#define TEST_ASSERT_NOT_EQUAL_MESSAGE(expected, actual, message)\
        TEST_ASSERT_MSG(((expected) !=  (actual)), (message))

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

#ifdef TEST_ASSERT_FULL
/**
 *******************************************************************************
 * @brief   Reports the name of the source file and the source line number
 *          where the assert_param error has occurred
 * @param   file pointer to file name where assert occurred
 * @param   line line number in file where assert occurred
 * @param   expr pointer expression string caused assert
 * @param   msg  pointer to additional message
 *******************************************************************************
 */
void TEST_ASSERT_MSG_Handler(uint8_t  const *file,
                             uint32_t const line,
                             uint8_t  const *expr,
                             uint8_t  const *msg);
#endif    /* ACU_ASSERT_FULL */

#ifdef __cplusplus
}
#endif

/**
 * @}
 */

/**
 * @}
 */
#endif /* end of TEST_ASSERT_H_ */
