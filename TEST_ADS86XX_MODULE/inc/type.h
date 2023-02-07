/**
 *******************************************************************************
 * @file    type.h
 * @version 1.0.1
 * @date    2023-02-07
 * @brief   Definition of float32_t and float64_t
 * @author  Tomasz Osypinski <br>
 *

 * Change History
 * --------------
 *
 * 2023-01-18:
 *      - Initial <br>
 *
 * 2023-02-07:
 *      - Fix in type of float64_t <br>
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

#ifndef TYPE_H_
#define TYPE_H_

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
 * @brief Definition of float32_t and float64_t
 * @{
 */

/*
 *******************************************************************************
 * #defines
 *******************************************************************************
 */
/**
 * @brief 32-bit floating-point type definition.
 */
#ifndef float32_t_DEFINED
    #define float32_t_DEFINED
    typedef float float32_t;
#endif

/**
 * @brief 64-bit floating-point type definition.
 */
#ifndef float64_t_DEFINED
    #define float64_t_DEFINED
    typedef double float64_t;
#endif

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

#endif /* end of TYPE_H_ */
