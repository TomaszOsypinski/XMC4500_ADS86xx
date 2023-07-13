/**
 *******************************************************************************
 * @file    ads86xx.h
 * @version 1.0.1
 * @date    2023-03-22
 * @brief   Low level driver for ADS868x and ADS869x with XMC4500
 * @author  Tomasz Osypinski<br>
 *
 *
 * Change History
 * --------------
 *
 * 2023-02-23:
 *      - Initial <br>
 *
 * 2023-02-23 v1.0.1:
 *      - Optimization, added function ADS86XX_GetMeasure_PU,
 *        Analog input configuration <br>
 *******************************************************************************
 */

/*
 * Copyright (C) 2022, Tomasz Osypinski. All rights reserved.
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

#ifndef ADS86XX_H_
#define ADS86XX_H_

/*
 *******************************************************************************
 * the includes
 *******************************************************************************
 */
#include "type.h"
#include "common.h"
#include <xmc_common.h>
#include <stdint.h>
#include <stdbool.h>

/**
 * @addtogroup TEST_ADS86XX_MODULE
 * @{
 */

/**
 * @addtogroup ADS86xx
 * @brief Low level driver for ADS868x and ADS869x with XMC4500
 * @{
 */

/*
 *******************************************************************************
 * #defines
 *******************************************************************************
 */
/**
 * 16-Bit HS SAR ADC
 * x = 1 = 1MSPS, 5 = 500kSPS, 9 = 100kSPS
 */
#define ADS868x (0)
#define ADS868X_DEV_NAME "ADS868x"
/**
 * 18-Bit HS SAR ADC
 * x = 1 = 1MSPS, 5 = 500kSPS, 9 = 100kSPS
 */
#define ADS869x (1)
#define ADS869X_DEV_NAME "ADS869x"

/**
 * ADS869x Sampling frequency in kHz
 */
#define ADS86_SAMPLING_FREQ  (40.0e3f)

/**
 * ADS869x Start sampling global = 1 or local = 0
 */
#define ADS86_SAMPLING_START_GLOBALY    (0)

/**
 * Check parity on output data frames and ADC conversions enable/disable
 */
#define ADS86_CHECK_PARITY_BITS_EN      (1)

/**
 * Put conversion result to DAC ch_0 output of XMC4500 enable/disable
 */
#define ADS86_PUT_DATA_TO_DAC_EN        (1)

/**
 * Analog input configuration
 */
#define ADS86_ANALOG_IN_TYPE_BIPOLAR    (0)
#define ADS86_ANALOG_IN_TYPE_UNIPOLAR   (1)
#define ADS86_ANALOG_IN_TYPE            ADS86_ANALOG_IN_TYPE_BIPOLAR
#define ADS86_ANALOG_IN_RANGE           ADS86_RANGE_SEL_BI_1_25_VREF
#define ADS86_ANALOG_IN_MAX_VAL         (1.25f * ADS86_REF * 1.0e-6f)
/**
 * Select device
 */
#define ADS86_DEVICE_TYPE ADS869x

/* Internal reference is Vref = 4.096V */
#define ADS86_REF               (4096000.0f)    /* in uV */

/* ADS86xx registers address*/
#define ADS86_DEVICE_ID_REG     (0x00U)
#define ADS86_RST_PWCTRL_REG    (0x04U)
#define ADS86_SDI_CTL_REG       (0x08U)
#define ADS86_SDO_CTL_REG       (0x0CU)
#define ADS86_DATAOUT_CTL_REG   (0x10U)
#define ADS86_RANGE_SEL_REG     (0x14U)
#define ADS86_ALARM_REG         (0x20U)
#define ADS86_ALARM_H_TH_REG    (0x24U)
#define ADS86_ALARM_L_TH_REG    (0x28U)

#define ADS86_CMD_NOP           (0x00U)
#define ADS86_CMD_CLEAR_HWORD   (0x60U)  /* 110_0000 any bits marked 1 in data will be set to 0 in register at those positions */
#define ADS86_CMD_READ_HWORD    (0x64U)  /* 110_0100 read 16 bits from register */
#define ADS86_CMD_READ          (0x24U)  /* 010_0100 read 8  bits from register */
#define ADS86_CMD_WRITE         (0x68U)  /* 110_1000 write 16 bits to register */
#define ADS86_CMD_WRITE_MSBYTE  (0x69U)  /* 110_1001 write 8 bits to register (MS Byte of 16-bit data) */
#define ADS86_CMD_WRITE_LSBYTE  (0x6AU)  /* 110_1010 write 8 bits to register (LS Byte of 16-bit data) */
#define ADS86_CMD_SET_HWORD     (0x6CU)  /* 110_1100 any bits marked 1 in data will be set to 1 in register at those positions */

#if     (ADS86_DEVICE_TYPE == ADS868x)
#define ADS86_DEVICE_NAME       ADS868X_DEV_NAME
#define ADS86_MIDDLE_CODE       (0x8000)
#define ADS86_MAX_VAL           (0xFFFF)
#define ADS86_CONV_RESAULT_POS  (16)
#define ADS86_MASK_CONV_RESAULT ((uint32_t)(0xFFFFUL << ADS86_CONV_RESAULT_POS))
#define ADS86_MASK_OUTPUT_FLAGS ((uint32_t)0xFFFFUL)
#elif   (ADS86_DEVICE_TYPE == ADS869x)
#define ADS86_DEVICE_NAME       ADS869X_DEV_NAME
#define ADS86_MIDDLE_CODE       (0x20000)
#define ADS86_MAX_VAL           (0x3FFFF)
#define ADS86_CONV_RESAULT_POS  (14)
#define ADS86_MASK_CONV_RESAULT ((uint32_t)(0x3FFFFUL << ADS86_CONV_RESAULT_POS))
#define ADS86_MASK_OUTPUT_FLAGS ((uint32_t)0x3FFFUL)
#else
#error "Define ADS86xx device"
#endif
/*
 *******************************************************************************
 * typedefs
 *******************************************************************************
 */
typedef enum ads86_range_sel
{
    /* Range for bipolar inputs. Vref = 4.096V */
    ADS86_RANGE_SEL_BI_3_VREF       = 0x0U,
    ADS86_RANGE_SEL_BI_2_5_VREF     = 0x1U,
    ADS86_RANGE_SEL_BI_1_5_VREF     = 0x2U,
    ADS86_RANGE_SEL_BI_1_25_VREF    = 0x3U,
    ADS86_RANGE_SEL_BI_0_625_VREF   = 0x4U,
    /*Range for unipolar inputs. Vref = 4.096V */
    ADS86_RANGE_SEL_UNI_3_VREF      = 0x8U,
    ADS86_RANGE_SEL_UNI_2_5_VREF    = 0x9U,
    ADS86_RANGE_SEL_UNI_1_5_VREF    = 0xAU,
    ADS86_RANGE_SEL_UNI_1_25_VREF   = 0xBU
} ads86_range_sel_t;

/* Command frame format 7-bit-cmd_9-bit-address_16-bit-data. Format MSB on bus*/
typedef union ads86_ctrl_frame
{
    uint8_t  Data_U8[4];    /* Index 0 is LSB, Index 3 is MSB */
    uint16_t Data_U16[2];   /* Index 0 is LSB, Index 3 is MSB */
    uint32_t Data_U32;
    struct
    {
        uint32_t Data     : 16;
        uint32_t Address  : 9;
        uint32_t Cmd      : 7;
    };
} ads86_ctrl_frame_t;

typedef union ads86_dataout_ctl_reg
{
    uint16_t Data_U16;
    struct
    {
        uint16_t DataVal            : 3;
        uint16_t ParEn              : 1;
        uint16_t Re00               : 4;
        uint16_t RangeIncl          : 1;
        uint16_t Re01               : 1;
        uint16_t InActiveAlarmIncl  : 2;
        uint16_t VddActiveAlarmIncl : 2;
        uint16_t DeviceAddrIncl     : 1;
        uint16_t Re02               : 1;
    };
} ads86_dataout_ctl_reg_t;

typedef union ads86_alarm_reg
{
    uint16_t Data_U16;
    struct
    {
        uint16_t OVW_ALARM    : 1;
        uint16_t Re00         : 3;
        uint16_t TRIP_IN_H    : 1;
        uint16_t TRIP_IN_L    : 1;
        uint16_t TRIP_VDD_H   : 1;
        uint16_t TRIP_VDD_L   : 1;
        uint16_t Re01         : 2;
        uint16_t ACTIVE_IN_H  : 1;
        uint16_t ACTIVE_IN_L  : 1;
        uint16_t Re02         : 2;
        uint16_t ACTIVE_VDD_H : 1;
        uint16_t ACTIVE_VDD_L : 1;
    };
} ads86_alarm_reg_t;

typedef union ads86_range_sel_reg
{
    uint16_t Data_U16;
    struct
    {
        uint16_t RangeSel  : 4;
        uint16_t Re00      : 2;
        uint16_t IntRefDis : 1;
        uint16_t Re01      : 1;
        uint16_t Re02      : 8;
    };
} ads86_range_sel_reg_t;

#if (ADS86_DEVICE_TYPE == ADS868x)
/* All flags enable ! */
typedef union ads86_output_data
{
    uint8_t  Data_U8[4];
    uint32_t Data_U32;
    struct
    {
        uint32_t Re00                     : 2;
        uint32_t OutputDataFrameParitBits : 1;
        uint32_t ConvResultParitBits      : 1;
        uint32_t InputAlarmFlags          : 2;
        uint32_t AvddAlarmFlags           : 2;
        uint32_t AdcInputRange            : 4;
        uint32_t DeviceAddress            : 4;
        uint32_t ConvResult               : 16;
    };
} ads86_output_data_t;
#elif (ADS86_DEVICE_TYPE == ADS869x)
/* All flags enable ! */
typedef union ads86_output_data
{
    uint8_t  Data_U8[4];
    uint32_t Data_U32;
    struct
    {
        uint32_t OutputDataFrameParitBits : 1;
        uint32_t ConvResultParitBits      : 1;
        uint32_t InputAlarmFlags          : 2;
        uint32_t AvddAlarmFlags           : 2;
        uint32_t AdcInputRange            : 4;
        uint32_t DeviceAddress            : 4;
        uint32_t ConvResult               : 18;
    };
} ads86_output_data_t;
#else
#error "Define ADS86xx device"
#endif

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
/**
 *******************************************************************************
 * @brief   Convert ADS86XX bipolar measured data to PU <-1.0, 1.0)
 * @param   Raw data from ADS86XX converter
 * @return  PU value
 *******************************************************************************
 */
COMMON_OPTIMIZE_FAST static inline
float32_t ADS86XX_bipolar_2_pu(uint32_t val)
{
    /* Input signal for ADS86 is bipolar.
     * <NFS = 0, PFS = ADS86_MAX_VAL>
     * AIN_P - AIN_N = 0V = ADS86_MIDDLE_CODE
     * Remove offset at middle scale and convert to per unit -1 to 1 */
    val = val > ADS86_MAX_VAL ? ADS86_MAX_VAL : val;

    float32_t tmp_pu =  (float32_t)((int32_t)val - ADS86_MIDDLE_CODE) * (1.0f / (float32_t)ADS86_MIDDLE_CODE);

    return tmp_pu;
}

/**
 *******************************************************************************
 * @brief   Convert ADS86XX unipolar measured data to PU <0.0, 1.0)
 * @param   Raw data from ADS86XX converter
 * @return  PU value
 *******************************************************************************
 */
COMMON_OPTIMIZE_FAST static inline
float32_t ADS86XX_unipolar_2_pu(uint32_t val)
{
    /* Input signal for ADS86 is unipolar.
     * <NFS = 0, PFS = ADS86_MAX_VAL>
     * AIN_P - AIN_N = 0V = 0
     * convert to per unit 0 to 1 */
    val = val > ADS86_MAX_VAL ? ADS86_MAX_VAL : val;

    float32_t tmp_pu =  (float32_t)val * (1.0f / (float32_t)ADS86_MAX_VAL);

    return tmp_pu;
}

/**
 *******************************************************************************
 * @brief   Initializes the ADS86xx channel
 * @param   in_range input voltage range.\n
 *          See: @ref ads86_range_sel_t
 * @return  None
 *******************************************************************************
 */
void ADS86XX_Init(const ads86_range_sel_t in_range);

/**
 *******************************************************************************
 * @brief   Get output data form ADS86xx
 * @return  Copy of @ref adsOutputData.
 *          For data format see @ref ads86_output_data_t
 *******************************************************************************
 */
uint32_t ADS86XX_GetOutputData(void);

/**
 *******************************************************************************
 * @brief   Get result, signed value. For bipolar inputs middle scale value is
 *          subtracted.
 * @return  Result of ADC. For bipolar input center point is 0
 *******************************************************************************
 */
int32_t ADS86XX_GetResault_I32(void);

/**
 *******************************************************************************
 * @brief   Get raw result, unsigned value.
 * @return  Raw result of ADC.
 *******************************************************************************
 */
uint32_t ADS86XX_GetResault_U32(void);

/**
 *******************************************************************************
 * @brief   Get ADC measurements in uV
 * @return  Result of ADC conversion in uV
 *******************************************************************************
 */
float32_t ADS86XX_GetMeasure_uV(void);

/**
 *******************************************************************************
 * @brief   Get output data frame flags
 * @return  Output data frame flags
 *          See @ref ads86_output_data_t
 *******************************************************************************
 */
uint32_t ADS86XX_GetOutputDataFlags(void);

/**
 *******************************************************************************
 * @brief   Get alarm flags from @ref ADS86_ALARM_REG
 * @return  Alarm flags from ADS86_ALARM_REG.
 *          See: @ref ads86_alarm_reg_t
 *******************************************************************************
 */
uint16_t  ADS86XX_GetAlarmsReg(void);

/**
 *******************************************************************************
 * @brief   Print useful information at console output if PRINT_INFO is globally
 *          defined
 * @return  None
 *******************************************************************************
 */
void ADS86XX_PrintInfo(void);

/**
 *******************************************************************************
 * @brief   Stop ISR mode. Device is serviced in poll mode\n
 *          See @ref ADS86XX_ServiceInPolling
 * @return  None
 *******************************************************************************
 */
void ADS86XX_Start_ServiceInPolling(void);

/**
 *******************************************************************************
 * @brief   Start ISR mode. Device is serviced vis ISR\n
 *          See @ref ADS86_Sampling_Isr,
 *              @ref ADS86_RVS_PIN_Isr,
 *              @ref ADS86_RxFifo_Isr
 * @return  None
 *******************************************************************************
 */
void ADS86XX_Start_ServiceInIsr(void);

/**
 *******************************************************************************
 * @brief   Service the ADS86xx channel in polling mode\n
 *          Internal variable @ref adsOutputData and @ref adsAlarmsReg are
 *          updated. @ref ADS86_CHECK_PARITY_BITS_EN to
 *          enable/disable parity check.
 * @return  None
 *******************************************************************************
 */
void ADS86XX_ServiceInPolling(void);

/**
 *******************************************************************************
 * @brief   Get ADC measurements in PU (per units)
 * @return  Result of ADC conversion in (per units)
 *******************************************************************************
 */
float32_t ADS86XX_GetMeasure_PU(void);

#ifdef __cplusplus
}
#endif

/**
 * @}
 */

/**
 * @}
 */

#endif /* end of ADS86XX_H_ */
