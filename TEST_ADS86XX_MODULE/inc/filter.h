/**
 *******************************************************************************
 * @file    filter.h
 * @version 1.0.0
 * @date    2023-01-30
 * @brief   Digital filters
 * @author  Tomasz Osypinski
 *
 * Change History
 * --------------
 *
 * 2023-01-30:
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

#ifndef FILTER_H_
#define FILTER_H_

/*
 *******************************************************************************
 * the includes
 *******************************************************************************
 */
#include "type.h"
#include <xmc_common.h>

/**
 * @addtogroup TEST_ADS86XX_MODULE
 * @{
 */

/**
 * @addtogroup Utility
 * @brief Digital filters
 * @{
 */

/*
 *******************************************************************************
 * #defines
 *******************************************************************************
 */
#ifndef PI
#define PI (3.14159265358979f)
#endif

/**
 * Init @ref filter_iir_1_f32_t with numerator and denominator coefficients
 */
#define FILTER_IIR_1_INIT(B0, B1, A1)   {.x = {0.0f, 0.0f}, \
                                         .y = {0.0f, 0.0f}, \
                                         .b0 = (B0),  \
                                         .b1 = (B1),  \
                                         .a1 = (A1)}

/**
 * Init @ref filter_iir_2_f32_t with numerator and denominator coefficients
 */
#define FILTER_IIR_2_INIT(B0, B1, B2, A1, A2) {.x = {0.0f, 0.0f, 0.0f}, \
                                               .y = {0.0f, 0.0f, 0.0f}, \
                                               .b0 = (B0),              \
                                               .b1 = (B1),              \
                                               .b2 = (B2),              \
                                               .a1 = (A1),              \
                                               .a2 = (A2)}

/* IIR_1 low pass coefficients calculation */
#define FILTER_LP1_F32_CALC_ALPHA(fc, fs)   (((((float32_t)fc) / ((float32_t)fs))) * (float32_t)PI)
#define FILTER_LP1_F32_CALC_B0(fc, fs)      (FILTER_LP1_F32_CALC_ALPHA(fc, fs) / (1.0f + FILTER_LP1_F32_CALC_ALPHA(fc, fs)))
#define FILTER_LP1_F32_CALC_B1(fc, fs)      FILTER_LP1_F32_CALC_B0(fc, fs)
#define FILTER_LP1_F32_CALC_A1(fc, fs)      ((FILTER_LP1_F32_CALC_ALPHA(fc, fs) - 1.0f) / (FILTER_LP1_F32_CALC_ALPHA(fc, fs) + 1.0f))

/**
 * Init @ref filter_iir_2_f32_t for setting frequency corner fc and sampling
 */
#define FILTER_IIR_1_LP_INIT(fc, fs) {.x = {0.0f, 0.0f},                        \
                                      .y = {0.0f, 0.0f},                        \
                                      .b0 = FILTER_LP1_F32_CALC_B0((fc), (fs)), \
                                      .b1 = FILTER_LP1_F32_CALC_B1((fc), (fs)), \
                                      .a1 = FILTER_LP1_F32_CALC_A1((fc), (fs))}

/*
 *******************************************************************************
 * typedefs
 *******************************************************************************
 */
typedef struct filter_iir_1_f32
{
    union{
        volatile float32_t x[2]; /*!< x[0] = inputs  + delays */
        struct
        {
            volatile float32_t input;
            volatile float32_t in_delay_1;
        };
    };

    union{
        volatile float32_t y[2]; /*!< y[0] = outputs + delays */
        struct
        {
            volatile float32_t output;
            volatile float32_t out_delay_1;
        };
    };

    /* numerator */
    const float32_t b0;
    const float32_t b1;

    /* denominator */
    const float32_t a1;
} filter_iir_1_f32_t;

typedef struct filter_iir_2_f32
{
    union{
        volatile float32_t x[3]; /*!< x[0] = inputs  + delays */
        struct
        {
            volatile float32_t input;
            volatile float32_t in_delay_1;
            volatile float32_t in_delay_2;
        };
    };

    union{
        volatile float32_t y[3]; /*!< y[0] = outputs + delays */
        struct
        {
            volatile float32_t output;
            volatile float32_t out_delay_1;
            volatile float32_t out_delay_2;
        };
    };

    /* numerator */
    const float32_t b0;
    const float32_t b1;
    const float32_t b2;

    /* denominator */
    const float32_t a1;
    const float32_t a2;
} filter_iir_2_f32_t;

typedef struct filter_avg_f32
{
    float32_t input;            /*!< Input value */
    float32_t output;           /*!< Output value */
    float32_t s;                /*!< Storage element */
    uint16_t counter;           /*!< Counter of samples */
    const uint16_t numOfPoints; /*!< Number of samples to averaging  */
} filter_avg_f32_t;

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
__STATIC_FORCEINLINE void FILTER_IIR_1_f32(filter_iir_1_f32_t * const f)
{
    f->y[0] = (f->b0 * f->x[0]) +
              (f->b1 * f->x[1]) +
              (f->a1 * f->y[1]);

    /* Update array */
    f->x[1] = f->x[0];
    f->y[1] = f->y[0];
}

__STATIC_FORCEINLINE void FILTER_IIR_2_f32(filter_iir_2_f32_t * const f)
{
    f->y[0] =      (f->b0 * f->x[0]) +
                   (f->b1 * f->x[1]) +
                   (f->b2 * f->x[2]) +
                   (f->a1 * f->y[1]) +
                   (f->a2 * f->y[2]);

    /* Update array */
    f->x[2] = f->x[1];
    f->x[1] = f->x[0];
    f->y[2] = f->y[1];
    f->y[1] = f->y[0];
}

__STATIC_FORCEINLINE void FILTER_AVG_f32(filter_avg_f32_t  * const f)
{
    f->s += f->input;
    f->counter++;
    if(f->counter >= f->numOfPoints)
    {
        f->counter = 0UL;
        f->output  = f->s / (float32_t)(f->numOfPoints);
        f->s = 0.0f;
    }
}

#ifdef __cplusplus
}
#endif

/**
 * @}
 */

/**
 * @}
 */

#endif /* end of FILTER_H_ */
