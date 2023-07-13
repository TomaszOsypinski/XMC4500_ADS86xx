/**
 *******************************************************************************
 * @file    common.h
 * @version 1.0.0
 * @date    2023-01-26
 * @brief   Set of common macros and functions etc.
 * @author  Tomasz Osypinski
 *
 * Change History
 * --------------
 *
 * 2023-01-26:
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

#ifndef COMMON_H_
#define COMMON_H_

/*
 *******************************************************************************
 * the includes
 *******************************************************************************
 */

/**
 * @addtogroup TEST_ADS86XX_MODULE
 * @{
 */

/**
 * @addtogroup Utility
 * @brief Set of common macros and functions etc.
 * @{
 */

/*
 *******************************************************************************
 * #defines
 *******************************************************************************
 */
/**
 * Per Unit maximum vale
 */
#define COMMON_MAX_PU ( 1.0f)

/**
 * Per Unit minimum vale
 */
#define COMMON_MIN_PU (-1.0f)

/**
 * Saturation macro to max/min value
 */
#define COMMON_SAT(k, max, min)         \
do {                                    \
    (k) = (((k) > (max)) ? (max) : (k));\
    (k) = (((k) < (min)) ? (min) : (k));\
}while(0)

/**
 * GCC fast optimize atribute
 */
#define COMMON_OPTIMIZE_FAST __attribute__((optimize("Ofast")))

/*
 *******************************************************************************
 * typedefs
 *******************************************************************************
 */
/**
 * Enumeric type of return function's status
 */
typedef enum common_status
{
    STATUS_SUCCESS = 0, /*!< Success */
    STATUS_ERROR        /*!< Error */
} common_status_t;

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
extern "C" {
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

#endif /* end of COMMON_H_ */
