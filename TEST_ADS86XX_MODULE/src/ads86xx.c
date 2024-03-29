/**
 *******************************************************************************
 * @file    ads86xx.c
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
 * 2023-02-23:
 *      - Optimization, added adsMeasure_pu,  <br>
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

/*
 *******************************************************************************
 * the includes
 *******************************************************************************
 */
#include "ads86xx.h"
#include "common.h"
#include "dac.h"
#include "test_assert.h"
#include "pin_interrupt.h"

#define ADS86_SOS_FILTER_EN (1)

#if ADS86_SOS_FILTER_EN
#include "sos_coff.h"
#endif

#include <xmc_spi.h>
#include <xmc_ccu8.h>

#ifdef PRINT_INFO
#include <stdio.h>
#endif

/* Switch on pedantic and extra checking */
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
/* SPI interface for ADS86xx */
#define ADS86_SPI_MISO                   P2_15
#define ADS86_SPI_MOSI                   P2_14
#define ADS86_SPI_CS                     P5_9
#define ADS86_SPI_SCLK                   P5_8

/* MOSI, CS, CLK are ALT1 outputs */
#define ADS86_SPI_OUTPUTS_ALT            XMC_GPIO_MODE_OUTPUT_PUSH_PULL_ALT2

/* MISO input selection */
#define ADS86_SPI_MISO_INPUT_SEL         XMC_SPI_CH_INPUT_DIN0, USIC1_C0_DX0_P2_15

/* RVS and ALARM signals connected to ERU */
#define ADS86_RVS_PORT                   XMC_GPIO_PORT2
#define ADS86_RVS_PIN                    (5U)
#define ADS86_ALARM_PORT                 XMC_GPIO_PORT2
#define ADS86_ALARM_PIN                  (6U)

/* SPI Channel for ADS86xx */
#define ADS86_SPI                        XMC_SPI1_CH0
#define ADS86_CS                         XMC_SPI_CH_SLAVE_SELECT_0
#define ADS86_SPI_MODE                   XMC_SPI_CH_MODE_STANDARD
#define ADS86_SPI_BAUDRATE               (10000000UL)   /* 10MHz SPI CLK */

/* SPI word size is a bit faster then byte */
#define ADS86_SPI_WORD_BYTE              (8U)
#define ADS86_SPI_WORD_WORD              (16U)
#define ADS86_SPI_WORD_SIZE              ADS86_SPI_WORD_WORD

#define ADS86_SAMPLING_CCU8_SLICE_PTR    CCU81_CC80
#define ADS86_SAMPLING_CCU8_MODULE_PTR   CCU81
#define ADS86_SAMPLING_CCU8_SLICE_NUMBER (0U)
#define ADS86_SAMPLING_CCU8_CLK_FREQ     (120.0e6f)

#define ADS86_Sampling_Isr               IRQ_Hdlr_64
#define ADS86_SAMPLING_Isr_Type          CCU81_0_IRQn
#define ADS86_SAMPLING_Isr_Priority      (2)
#define ADS86_SAMPLING_Isr_Subpririty    (0)

#define ADS86_RVS_PIN_Isr                IRQ_Hdlr_1
#define ADS86_RVS_PIN_Isr_Type           ERU0_0_IRQn
#define ADS86_RVS_PIN_Isr_Priority       (2)
#define ADS86_RVS_PIN_Isr_Subpririty     (0)

#define ADS86_RxFifo_Isr                 IRQ_Hdlr_90
#define ADS86_RXFIFO_Isr_Type            USIC1_0_IRQn
#define ADS86_RXFIFO_Isr_Priority        (2)
#define ADS86_RXFIFO_Isr_Subpririty      (0)

#define ADS86_ALARM_PIN_Isr              IRQ_Hdlr_2
#define ADS86_ALARM_PIN_Isr_Type         ERU0_1_IRQn
#define ADS86_ALARM_PIN_Isr_Priority     (63)
#define ADS86_ALARM_PIN_Isr_Subpririty   (0)

/*
 *******************************************************************************
 * local typedefs
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
/**
 * This flag enables service ADS86xx in pooling mode e.g. in main loop. Data
 * outputs and alarm can be read by API function. See ads86xx.h
 */
static volatile bool     enServiceInPolling = false;

/**
 * Read alarm register flag. See @ref ADS86_ALARM_PIN_Isr
 */
static volatile bool     readAdsAlarms = false;

/**
 * Last read ADS alarm register value.
 * For data format see @ref ads86_output_data_t and @ref ads86_alarm_reg_t
 */
static volatile uint16_t adsAlarmsReg  = 0U;

/**
 * Output data from ADS86xx. For data format see @ref ads86_output_data_t
 */
static volatile uint32_t adsOutputData = 0UL;

/**
 * Measured value in PU (per units)
 */
static volatile float32_t adsMeasure_pu = 0.0f;

/* Form http://www.graphics.stanford.edu/~seander/bithacks.html#ParityLookupTable */
static const bool parityTable256[256] =
{
    #define P2(n) n, n^1, n^1, n
    #define P4(n) P2(n), P2(n^1), P2(n^1), P2(n)
    #define P6(n) P4(n), P4(n^1), P4(n^1), P4(n)

    P6(0), P6(1), P6(1), P6(0)
};

/*
 *******************************************************************************
 * the function prototypes
 *******************************************************************************
 */
/* ISR prototypes */
void ADS86_Sampling_Isr(void);
void ADS86_RVS_PIN_Isr(void);
void ADS86_RxFifo_Isr(void);
void ADS86_ALARM_PIN_Isr(void);
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
#if ADS86_SOS_FILTER_EN
__STATIC_FORCEINLINE float32_t iir_sos(const float32_t in)
{
    s[0]->x[0] = in;

    uint32_t i = 0U;
    do
    {
        FILTER_IIR_2_f32(s[i]);

        #if (SOS_COFF_NUM_OF_STAGES > 1)
        if(i < (SOS_COFF_NUM_OF_STAGES - 1))
        {
            s[i + 1]->x[0] = s[i]->y[0];
        }
        #endif

        i++;
    } while(i < SOS_COFF_NUM_OF_STAGES);

    return s[SOS_COFF_NUM_OF_STAGES - 1]->output;
}
#endif

static void ads86_spi_init(void)
{
    const XMC_SPI_CH_CONFIG_t spiConfig =
    {
       .baudrate       = ADS86_SPI_BAUDRATE,
       .bus_mode       = XMC_SPI_CH_BUS_MODE_MASTER,
       .selo_inversion = XMC_SPI_CH_SLAVE_SEL_INV_TO_MSLS,
       .parity_mode    = XMC_USIC_CH_PARITY_MODE_NONE
    };

    XMC_GPIO_CONFIG_t pinConfig =
    {
       .output_level    = XMC_GPIO_OUTPUT_LEVEL_LOW,
       .output_strength = XMC_GPIO_OUTPUT_STRENGTH_STRONG_SOFT_EDGE
    };

    /*Initialize and Start SPI*/
    XMC_SPI_CH_Init(ADS86_SPI, &spiConfig);

    /* CLK configuration */
    XMC_SPI_CH_ConfigureShiftClockOutput(ADS86_SPI,
                                        XMC_SPI_CH_BRG_SHIFT_CLOCK_PASSIVE_LEVEL_0_DELAY_ENABLED,
                                        XMC_SPI_CH_BRG_SHIFT_CLOCK_OUTPUT_SCLK);

    /* World length */
    XMC_SPI_CH_SetWordLength(ADS86_SPI, ADS86_SPI_WORD_SIZE);
    /* Frame length is 32 bits */
    XMC_SPI_CH_SetFrameLength(ADS86_SPI, 32U);
    /* MSB byte first */
    XMC_SPI_CH_SetBitOrderMsbFirst(ADS86_SPI);

    /*Input source selected*/
    XMC_SPI_CH_SetInputSource(ADS86_SPI, ADS86_SPI_MISO_INPUT_SEL);
    /* Must start here, before settings outputs */
    XMC_SPI_CH_Start(ADS86_SPI);

    /* FIFO configuration */
    #if (ADS86_SPI_WORD_SIZE == ADS86_SPI_WORD_BYTE)
    /* 4 entries for TxFIFO from point 0, LIMIT = 1. Interrupt is not used */
    XMC_USIC_CH_TXFIFO_Configure(ADS86_SPI, 0, XMC_USIC_CH_FIFO_SIZE_4WORDS, 1);
    /* 4 entries for RxFIFO from point 8, LIMIT = 3. */
    XMC_USIC_CH_RXFIFO_Configure(ADS86_SPI, 8, XMC_USIC_CH_FIFO_SIZE_4WORDS, 3);
    #elif (ADS86_SPI_WORD_SIZE == ADS86_SPI_WORD_WORD)
    /* 4 entries for TxFIFO from point 0, LIMIT = 1. Interrupt is not used */
    XMC_USIC_CH_TXFIFO_Configure(ADS86_SPI, 0, XMC_USIC_CH_FIFO_SIZE_2WORDS, 1);
    /* 4 entries for RxFIFO from point 8, LIMIT = 3. */
    XMC_USIC_CH_RXFIFO_Configure(ADS86_SPI, 8, XMC_USIC_CH_FIFO_SIZE_2WORDS, 1);
    #else
    #error "Define SPI WORD size!!!"
    #endif
    /* Outputs */
    /* MOSI, CS, CLK are ALT2 outputs */
    pinConfig.mode = ADS86_SPI_OUTPUTS_ALT;

    /* MOSI */
    XMC_GPIO_Init(ADS86_SPI_MOSI, &pinConfig);

    /* CLK */
    XMC_GPIO_Init(ADS86_SPI_SCLK, &pinConfig);

    /* CS */
    pinConfig.output_level = XMC_GPIO_OUTPUT_LEVEL_HIGH;
    XMC_GPIO_Init(ADS86_SPI_CS, &pinConfig);

    /* Input */
    pinConfig.mode = XMC_GPIO_MODE_INPUT_PULL_UP;
    XMC_GPIO_Init(ADS86_SPI_MISO, &pinConfig);

    /* Enable HW for CS */
    XMC_SPI_CH_EnableSlaveSelect(ADS86_SPI, ADS86_CS);
}

static uint32_t ads86_send_cmd(const uint8_t op,
                               const uint8_t address,
                               const uint16_t data)
{
    ads86_ctrl_frame_t rx_frame;
    ads86_ctrl_frame_t tx_frame;

    tx_frame.Cmd      = (uint8_t)(op & 0x7F);
    tx_frame.Address  = address;
    tx_frame.Data     = data;

    #if (ADS86_SPI_WORD_SIZE == ADS86_SPI_WORD_BYTE)
    /* Tx data */
    int32_t i = 3; /* Index 3 is MSB  */
    do
    {
        XMC_USIC_CH_TXFIFO_PutData(ADS86_SPI, tx_frame.Data_U8[i]);
        i--;
    } while(i >= 0);

    /* RX data */
    while(!XMC_USIC_CH_RXFIFO_IsFull(ADS86_SPI));
    i = 3; /* Index 3 is MSB  */
    do
    {
        rx_frame.Data_U8[i] = (uint8_t)XMC_USIC_CH_RXFIFO_GetData(ADS86_SPI);
        i--;
    } while(i >= 0);
    #elif (ADS86_SPI_WORD_SIZE == ADS86_SPI_WORD_WORD)
    /* Tx data, Index 1 is MSB */
    XMC_USIC_CH_TXFIFO_PutData(ADS86_SPI, tx_frame.Data_U16[1]);
    XMC_USIC_CH_TXFIFO_PutData(ADS86_SPI, tx_frame.Data_U16[0]);

    /* RX data, Index 1 is MSB */
    while(!XMC_USIC_CH_RXFIFO_IsFull(ADS86_SPI));
    rx_frame.Data_U16[1] = (uint16_t)XMC_USIC_CH_RXFIFO_GetData(ADS86_SPI);
    rx_frame.Data_U16[0] = (uint16_t)XMC_USIC_CH_RXFIFO_GetData(ADS86_SPI);
    #else
    #error "Define SPI WORD size!!!"
    #endif

    return rx_frame.Data_U32;
}

static void ads_alarm_pin_init(void)
{
    const pin_interrupt_t config_pin_alarm =
    {
        .eru = XMC_ERU0,
        .port = ADS86_ALARM_PORT,
        .gpio_config =
        {
            .mode = XMC_GPIO_MODE_INPUT_PULL_UP
        },
        .etl_config =
        {
            .input_b                = (uint32_t)XMC_ERU_ETL_INPUT_B3,
            .enable_output_trigger  = (uint32_t)1,
            .status_flag_mode       = (uint32_t)1,
            .edge_detection         = XMC_ERU_ETL_EDGE_DETECTION_RISING,
            .output_trigger_channel = XMC_ERU_ETL_OUTPUT_TRIGGER_CHANNEL1,
            .source                 = XMC_ERU_ETL_SOURCE_B
        },
        .IRQn         = ADS86_ALARM_PIN_Isr_Type,
        .irq_priority = ADS86_ALARM_PIN_Isr_Priority,
        #if(UC_FAMILY == XMC4)
        .irq_subpriority = ADS86_ALARM_PIN_Isr_Subpririty,
        #endif
        .etl = 1U,
        .ogu = 1U,
        .pin = ADS86_ALARM_PIN,
        .enable_at_init = true
    };

    PIN_INTERRUPT_Init(&config_pin_alarm);

    /* force to read alarm register if ALARM pin is HI */
    readAdsAlarms = (bool)XMC_GPIO_GetInput(ADS86_ALARM_PORT, ADS86_ALARM_PIN);
}

static void ads_rvs_pin_init(void)
{
    /* ERU */
    const pin_interrupt_t config_pin_rvs =
    {
       .eru = XMC_ERU0,
       .port = ADS86_RVS_PORT,
       .gpio_config =
        {
          .mode = XMC_GPIO_MODE_INPUT_PULL_DOWN
        },
       .etl_config =
       {
         .input_a                = (uint32_t)XMC_ERU_ETL_INPUT_A2,
         .enable_output_trigger  = (uint32_t)1,
         .status_flag_mode       = (uint32_t)1,
         .edge_detection         = XMC_ERU_ETL_EDGE_DETECTION_RISING,
         .output_trigger_channel = XMC_ERU_ETL_OUTPUT_TRIGGER_CHANNEL0,
         .source                 = XMC_ERU_ETL_SOURCE_A
       },
       .IRQn = ADS86_RVS_PIN_Isr_Type,
       .irq_priority = ADS86_RVS_PIN_Isr_Priority,
     #if(UC_FAMILY == XMC4)
       .irq_subpriority = ADS86_RVS_PIN_Isr_Subpririty,
     #endif
       .etl = 0U,
       .ogu = 0U,
       .pin = ADS86_RVS_PIN,
       .enable_at_init = true
     };

    PIN_INTERRUPT_Init(&config_pin_rvs);
}

static void ads_reconfig_pin_cs_to_convst(void)
{
    XMC_GPIO_CONFIG_t pinConfig =
    {
        .mode            = XMC_GPIO_MODE_OUTPUT_PUSH_PULL,
        .output_level    = XMC_GPIO_OUTPUT_LEVEL_HIGH,
        .output_strength = XMC_GPIO_OUTPUT_STRENGTH_STRONG_SHARP_EDGE
    };

    /* Disable HW for CS */
    XMC_SPI_CH_DisableSlaveSelect(ADS86_SPI);

    XMC_GPIO_Init(ADS86_SPI_CS, &pinConfig);
}

static void ads_reconfig_pin_convst_to_cs(void)
{
    XMC_GPIO_CONFIG_t pinConfig =
    {
        .mode            = XMC_GPIO_MODE_OUTPUT_PUSH_PULL_ALT2,
        .output_level    = XMC_GPIO_OUTPUT_LEVEL_HIGH,
        .output_strength = XMC_GPIO_OUTPUT_STRENGTH_STRONG_SHARP_EDGE
    };
    XMC_GPIO_Init(ADS86_SPI_CS, &pinConfig);

    /* Enable HW for CS */
    XMC_SPI_CH_EnableSlaveSelect(ADS86_SPI, ADS86_CS);
}

static void rx_fifo_isr_init(void)
{
    XMC_USIC_CH_RXFIFO_ClearEvent(ADS86_SPI, (uint32_t)XMC_USIC_CH_RXFIFO_EVENT_STANDARD |
                                             (uint32_t)XMC_USIC_CH_RXFIFO_EVENT_ERROR);

    XMC_USIC_CH_RXFIFO_SetInterruptNodePointer(ADS86_SPI,
                                               XMC_USIC_CH_RXFIFO_INTERRUPT_NODE_POINTER_STANDARD,
                                               0UL);

    XMC_USIC_CH_RXFIFO_EnableEvent(ADS86_SPI, (uint32_t)XMC_USIC_CH_RXFIFO_EVENT_CONF_STANDARD);

    NVIC_SetPriority(ADS86_RXFIFO_Isr_Type,
                     NVIC_EncodePriority(NVIC_GetPriorityGrouping(),
                                         ADS86_RXFIFO_Isr_Priority,
                                         ADS86_RXFIFO_Isr_Subpririty));

    NVIC_EnableIRQ(ADS86_RXFIFO_Isr_Type);
}

static void sampling_timer_init(void)
{
    const XMC_CCU8_SLICE_COMPARE_CONFIG_t timer_config =
    {
      .timer_mode          = (uint32_t) XMC_CCU8_SLICE_TIMER_COUNT_MODE_EA,
      .monoshot            = (uint32_t) XMC_CCU8_SLICE_TIMER_REPEAT_MODE_REPEAT,
      .shadow_xfer_clear   = (uint32_t) 0,
      .dither_timer_period = (uint32_t) 0,
      .dither_duty_cycle   = (uint32_t) 0,
      .mcm_ch1_enable      = (uint32_t) false,
      .mcm_ch2_enable      = (uint32_t) false,
      .slice_status        = (uint32_t) XMC_CCU8_SLICE_STATUS_CHANNEL_1,
      .prescaler_mode      = (uint32_t) XMC_CCU8_SLICE_PRESCALER_MODE_NORMAL,
      .passive_level_out0  = (uint32_t) XMC_CCU8_SLICE_OUTPUT_PASSIVE_LEVEL_LOW,
      .passive_level_out1  = (uint32_t) XMC_CCU8_SLICE_OUTPUT_PASSIVE_LEVEL_LOW,
      .passive_level_out2  = (uint32_t) XMC_CCU8_SLICE_OUTPUT_PASSIVE_LEVEL_LOW,
      .passive_level_out3  = (uint32_t) XMC_CCU8_SLICE_OUTPUT_PASSIVE_LEVEL_LOW,
      .asymmetric_pwm      = (uint32_t) 0,
      .invert_out0         = (uint32_t) 0,
      .invert_out1         = (uint32_t) 0,
      .invert_out2         = (uint32_t) 0,
      .invert_out3         = (uint32_t) 0,
      .prescaler_initval   = (uint32_t) 0,
      .float_limit         = (uint32_t) 0,
      .dither_limit        = (uint32_t) 0,
      .timer_concatenation = (uint32_t) 0
    };

    const XMC_CCU8_SLICE_EVENT_CONFIG_t cfg_event0_config =
         {
            .mapped_input        = XMC_CCU8_SLICE_INPUT_H,
            .edge                = XMC_CCU8_SLICE_EVENT_EDGE_SENSITIVITY_RISING_EDGE,
            .level               = XMC_CCU8_SLICE_EVENT_LEVEL_SENSITIVITY_ACTIVE_LOW,
            .duration            = XMC_CCU8_SLICE_EVENT_FILTER_DISABLED,
         };


    XMC_ASSERT("To small ADS86_SAMPLING_FREQ!!!",
              (ADS86_SAMPLING_CCU8_CLK_FREQ / ADS86_SAMPLING_FREQ < (float32_t)UINT16_MAX));

    XMC_CCU8_Init(ADS86_SAMPLING_CCU8_MODULE_PTR, XMC_CCU8_SLICE_MCMS_ACTION_TRANSFER_PR_CR);
    XMC_CCU8_StartPrescaler(ADS86_SAMPLING_CCU8_MODULE_PTR);
    XMC_CCU8_SLICE_CompareInit(ADS86_SAMPLING_CCU8_SLICE_PTR, &timer_config);
    XMC_CCU8_SLICE_SetTimerPeriodMatch(ADS86_SAMPLING_CCU8_SLICE_PTR, ((uint16_t)(ADS86_SAMPLING_CCU8_CLK_FREQ / ADS86_SAMPLING_FREQ) - 1U));
    XMC_CCU8_SLICE_EnableEvent(ADS86_SAMPLING_CCU8_SLICE_PTR, XMC_CCU8_SLICE_IRQ_ID_PERIOD_MATCH);
    XMC_CCU8_SLICE_SetInterruptNode(ADS86_SAMPLING_CCU8_SLICE_PTR, XMC_CCU8_SLICE_IRQ_ID_PERIOD_MATCH, XMC_CCU8_SLICE_SR_ID_0);
    XMC_CCU8_EnableShadowTransfer(ADS86_SAMPLING_CCU8_MODULE_PTR, (uint32_t)XMC_CCU8_SHADOW_TRANSFER_SLICE_0);

    XMC_CCU8_SLICE_ConfigureEvent(ADS86_SAMPLING_CCU8_SLICE_PTR, XMC_CCU8_SLICE_EVENT_0, &cfg_event0_config);
    XMC_CCU8_SLICE_StartConfig(ADS86_SAMPLING_CCU8_SLICE_PTR, XMC_CCU8_SLICE_EVENT_0, XMC_CCU8_SLICE_START_MODE_TIMER_START_CLEAR);

    NVIC_SetPriority(ADS86_SAMPLING_Isr_Type,
                     NVIC_EncodePriority(NVIC_GetPriorityGrouping(),
                     ADS86_SAMPLING_Isr_Priority,
                     ADS86_SAMPLING_Isr_Subpririty));

    NVIC_EnableIRQ(ADS86_SAMPLING_Isr_Type);

    /* Start the CCU81_0 timer */
    XMC_CCU8_EnableClock(ADS86_SAMPLING_CCU8_MODULE_PTR, ADS86_SAMPLING_CCU8_SLICE_NUMBER);
    /* Check clocks */
    do { } while(((ADS86_SAMPLING_CCU8_MODULE_PTR->GSTAT & CCU8_GSTAT_S0I_Msk) != 0UL));

    XMC_CCU8_SLICE_StopClearTimer(ADS86_SAMPLING_CCU8_SLICE_PTR);

    #if (ADS86_SAMPLING_START_GLOBALY == 0)
    XMC_CCU8_SLICE_StartTimer(ADS86_SAMPLING_CCU8_SLICE_PTR);
    #endif
}

/* Conversion start */
__STATIC_FORCEINLINE void ads_conv_start(void)
{
    /* Rising edge force conversion */
    XMC_GPIO_SetOutputHigh(ADS86_SPI_CS);
}

/* Acquisition start */
__STATIC_FORCEINLINE void ads_acq_start(void)
{
    /* Falling edge force acquisition */
    XMC_GPIO_SetOutputLow(ADS86_SPI_CS);
}

#if (ADS86_CHECK_PARITY_BITS_EN == 1)
/* true if parity OK, otherwise false */
COMMON_OPTIMIZE_FAST static inline
bool checkParity(const uint32_t data)
{
    ads86_output_data_t calc_par;

    calc_par.Data_U32             = data;
    bool outputDataFrameParitBits = (bool)calc_par.OutputDataFrameParitBits;
    bool convResultParitBits      = (bool)calc_par.ConvResultParitBits;

    /* Calculate parity - data output frame
     * ADC output parity bits (ConvResultParitBits) are not included in
     * the data output frame parity bit computation  */
    calc_par.ConvResultParitBits      = 0U;
    calc_par.OutputDataFrameParitBits = 0U;
    bool parity_OutputDataFrame = parityTable256[calc_par.Data_U8[0] ^
                                                 calc_par.Data_U8[1] ^
                                                 calc_par.Data_U8[2] ^
                                                 calc_par.Data_U8[3]];

    /* Calculate ADC output (conversion result) parity bits */
    calc_par.Data_U32 &= ADS86_MASK_CONV_RESAULT;
    bool parity_ConvResult = parityTable256[calc_par.Data_U8[0] ^
                                            calc_par.Data_U8[1] ^
                                            calc_par.Data_U8[2] ^
                                            calc_par.Data_U8[3]];

    return ((parity_OutputDataFrame == outputDataFrameParitBits) &&
            (parity_ConvResult      == convResultParitBits));

}
#endif

#ifdef PRINT_INFO
static const char * getStringRange(const ads86_range_sel_t range)
{
    switch(range)
    {
        case ADS86_RANGE_SEL_BI_3_VREF:
            return "BI_3_VREF";
            break;
        case ADS86_RANGE_SEL_BI_2_5_VREF:
            return "BI_2_5_VREF";
            break;
        case ADS86_RANGE_SEL_BI_1_5_VREF:
            return "BI_1_5_VREF";
            break;
        case ADS86_RANGE_SEL_BI_1_25_VREF:
            return "BI_1_25_VREF";
            break;
        case ADS86_RANGE_SEL_BI_0_625_VREF:
            return "BI_0_625_VREF";
            break;
        case ADS86_RANGE_SEL_UNI_3_VREF:
            return "UNI_3_VREF";
            break;
        case ADS86_RANGE_SEL_UNI_2_5_VREF:
            return "UNI_2_5_VREF";
            break;
        case ADS86_RANGE_SEL_UNI_1_5_VREF:
            return "UNI_1_5_VREF";
            break;
        case ADS86_RANGE_SEL_UNI_1_25_VREF:
            return "UNI_1_25_VREF";
            break;
        default:
            return "Out of range of: ads86_range_sel_t!!!";
            break;
    }
}
#endif

/**
 *******************************************************************************
 * @brief   ISR service routine for CCU8 period match handler\n
 *          Force new conversion.
 * @return  None
 *******************************************************************************
 */
COMMON_OPTIMIZE_FAST void ADS86_Sampling_Isr(void)
{
    ads_conv_start();
}

/**
 *******************************************************************************
 * @brief   ISR service routine for RVS pin\n
 *          ISR is forced on rising edge of RVS pin which means conversion
 *          is done.
 * @return  None
 *******************************************************************************
 */
COMMON_OPTIMIZE_FAST void ADS86_RVS_PIN_Isr(void)
{
    /* Go to accusation and get data */
    ads_acq_start();

    #if (ADS86_SPI_WORD_SIZE == ADS86_SPI_WORD_BYTE)
    register int32_t i = 3;  /* Index 3 is MSB  */
    /* Tx data */
    do
    {
        XMC_USIC_CH_TXFIFO_PutData(ADS86_SPI, 0);
        i--;
    } while(i >= 0);
    #elif (ADS86_SPI_WORD_SIZE == ADS86_SPI_WORD_WORD)
    /* Tx data */
    XMC_USIC_CH_TXFIFO_PutData(ADS86_SPI, 0);
    XMC_USIC_CH_TXFIFO_PutData(ADS86_SPI, 0);
    #else
    #error "Define SPI WORD size!!!"
    #endif
}

/**
 *******************************************************************************
 * @brief   ISR service routine for SPI RxFifo\n
 *          RXFIFO is ready to read
 * @return  None
 *******************************************************************************
 */
COMMON_OPTIMIZE_FAST void ADS86_RxFifo_Isr(void)
{
    ads86_ctrl_frame_t rx_frame;

    #if (ADS86_SPI_WORD_SIZE == ADS86_SPI_WORD_BYTE)
    register int32_t i = 3; /* Index 3 is MSB  */
    do
    {
        rx_frame.Data_U8[i] = (uint8_t)XMC_USIC_CH_RXFIFO_GetData(ADS86_SPI);
        i--;
    } while(i >= 0);
    #elif (ADS86_SPI_WORD_SIZE == ADS86_SPI_WORD_WORD)
    /* Index 1 is MSB  */
    rx_frame.Data_U16[1] = (uint16_t)XMC_USIC_CH_RXFIFO_GetData(ADS86_SPI);
    rx_frame.Data_U16[0] = (uint16_t)XMC_USIC_CH_RXFIFO_GetData(ADS86_SPI);
    #else
    #error "Define SPI WORD size!!!"
    #endif

    #if   (ADS86_ANALOG_IN_TYPE == ADS86_ANALOG_IN_TYPE_BIPOLAR)
    const float32_t tmp_pu = ADS86XX_bipolar_2_pu(rx_frame.Data_U32 >> ADS86_CONV_RESAULT_POS);
    #elif (ADS86_ANALOG_IN_TYPE == ADS86_ANALOG_IN_TYPE_UNIPOLAR)
    float32_t tmp_pu = ADS86XX_unipolar_2_pu(rx_frame.Data_U32 >> ADS86_CONV_RESAULT_POS);
    #else
    #error "ADS86_ANALOG_IN_TYPE wrong configuration!!!"
    #endif

    /* Update global variable */
    adsMeasure_pu = tmp_pu;
    adsOutputData = rx_frame.Data_U32;

    #if (ADS86_PUT_DATA_TO_DAC_EN == 1)
    DAC_Ch0_PU_2_DAC(tmp_pu);
    #endif
    
    #if ADS86_SOS_FILTER_EN
    DAC_Ch1_PU_2_DAC(iir_sos(tmp_pu));
    #endif
}

/**
 *******************************************************************************
 * @brief   ISR service routine for Alarm pin\n
 *          ISR is force on rising edge of ALARM pin. Alarm/s appearer/s in device
 * @return  None
 *******************************************************************************
 */
COMMON_OPTIMIZE_FAST void ADS86_ALARM_PIN_Isr(void)
{
    readAdsAlarms = true;
}

void ADS86XX_Init(const ads86_range_sel_t in_range)
{
    uint32_t ret_data;

    ads86_spi_init();

    /* Set input range */
    ads86_range_sel_reg_t range_reg = {.Data_U16 = 0U};
    range_reg.RangeSel = in_range;
    ads86_send_cmd(ADS86_CMD_WRITE, ADS86_RANGE_SEL_REG, range_reg.Data_U16);

    /* get RANGE_SEL_REG */
    ads86_send_cmd(ADS86_CMD_READ_HWORD, ADS86_RANGE_SEL_REG, 0U);
    #ifdef PRINT_INFO
    ret_data = ads86_send_cmd(ADS86_CMD_NOP, 0U, 0U);
    printf("\n\n%s", ADS86_DEVICE_NAME " Init ...");
    const ads86_range_sel_t range = (ads86_range_sel_t)((uint16_t)(ret_data >> 16) & 0xFU);
    printf("\n%-32s : %s", "ADS86_RANGE_SEL_REG", getStringRange(range));
    #endif

    /* Set all flags in output data */
    ads86_dataout_ctl_reg_t ctl_reg = {.Data_U16 = 0U};
    ctl_reg.DataVal            = 0U;    /* conversion data        */
                                        /* 110b alter 0's and 1's */
    #if (ADS86_CHECK_PARITY_BITS_EN == 1)
    ctl_reg.ParEn              = 1U;    /* Parity bits included */
    #endif
    ctl_reg.RangeIncl          = 1U;    /* Range included       */
    ctl_reg.InActiveAlarmIncl  = 0x3U;  /* Include both flags   */
    ctl_reg.VddActiveAlarmIncl = 0x3U;  /* Include both flags   */
    ctl_reg.DeviceAddrIncl     = 1U;    /* Include dev. address */
    ads86_send_cmd(ADS86_CMD_WRITE, ADS86_DATAOUT_CTL_REG, ctl_reg.Data_U16);

    /* get ADS86_DATAOUT_CTL_REG */
    ads86_send_cmd(ADS86_CMD_READ_HWORD, ADS86_DATAOUT_CTL_REG, 0U);
    #ifdef PRINT_INFO
    ret_data = ads86_send_cmd(ADS86_CMD_NOP, 0U, 0U);
    ctl_reg.Data_U16 = 0U;
    ctl_reg.Data_U16 = (uint16_t)(ret_data >> 16);
    printf("\n%-32s : 0x%.4X", "ADS86_DATAOUT_CTL_REG", ctl_reg.Data_U16);
    #endif

    /* configure SDO_1 as ALARM output */
    ads86_send_cmd(ADS86_CMD_WRITE, ADS86_SDO_CTL_REG, (uint16_t)(1U << 8));

    /* Read alarm register to clear all flags  */
    ads86_send_cmd(ADS86_CMD_READ_HWORD, ADS86_ALARM_REG, 0U);
    ret_data = ads86_send_cmd(ADS86_CMD_NOP, 0U, 0U);
    /* Only lower words is occupied by alarm flags */
    adsAlarmsReg |= (uint16_t)(ret_data >> 16);
    #ifdef PRINT_INFO
    printf("\n%-32s : 0x%.4X", "ADS86_ALARM_REG", adsAlarmsReg);
    #endif

    /* Initializes ISR for ALARM pin */
    ads_alarm_pin_init();

    /* Initializes ISR for RVS pin */
    ads_rvs_pin_init();

    /* Prepare ADS to service via ISRs */
    ads_reconfig_pin_cs_to_convst();
    ads_acq_start();
    rx_fifo_isr_init();
    sampling_timer_init();
}

uint32_t ADS86XX_GetOutputData(void)
{
    #if (ADS86_CHECK_PARITY_BITS_EN == 1)
    uint32_t tmp_u32 = adsOutputData;
    tmp_u32 = checkParity(tmp_u32) ? tmp_u32 : UINT32_MAX;
    #else
    uint32_t tmp_u32 = adsOutputData;
    #endif

    return tmp_u32;
}

int32_t ADS86XX_GetResault_I32(void)
{
    ads86_output_data_t data;

    #if (ADS86_CHECK_PARITY_BITS_EN == 1)
    data.Data_U32 = adsOutputData;
    data.Data_U32 = checkParity(data.Data_U32) ? data.Data_U32 : UINT32_MAX;
    #else
    data.Data_U32 = adsOutputData;
    #endif

    int32_t tmp_i32 = 0L;

    switch((ads86_range_sel_t)data.AdcInputRange)
    {
        case ADS86_RANGE_SEL_BI_3_VREF:
        case ADS86_RANGE_SEL_BI_2_5_VREF:
        case ADS86_RANGE_SEL_BI_1_5_VREF:
        case ADS86_RANGE_SEL_BI_1_25_VREF:
        case ADS86_RANGE_SEL_BI_0_625_VREF:
            tmp_i32 = (int32_t)data.ConvResult - ADS86_MIDDLE_CODE;
            break;

        case ADS86_RANGE_SEL_UNI_3_VREF:
        case ADS86_RANGE_SEL_UNI_2_5_VREF:
        case ADS86_RANGE_SEL_UNI_1_5_VREF:
        case ADS86_RANGE_SEL_UNI_1_25_VREF:
            tmp_i32 = (int32_t)data.ConvResult;
            break;

        default:
            break;
    }

    return tmp_i32;
}

uint32_t ADS86XX_GetResault_U32(void)
{
    ads86_output_data_t data;

    #if (ADS86_CHECK_PARITY_BITS_EN == 1)
    data.Data_U32 = adsOutputData;
    data.Data_U32 = checkParity(data.Data_U32) ? data.Data_U32 : 0xFFFFFFFFUL;
    #else
    data.Data_U32 = adsOutputData;
    #endif

    uint32_t tmp_u32 = (uint32_t)data.ConvResult;

    return tmp_u32;
}

uint32_t ADS86XX_GetOutputDataFlags(void)
{
    #if (ADS86_CHECK_PARITY_BITS_EN == 1)
    uint32_t tmp_u32 = adsOutputData;
    tmp_u32 = checkParity(tmp_u32) ? tmp_u32 : 0xFFFFFFFFUL;
    #else
    uint32_t tmp_u32 = adsOutputData;
    #endif

    /* Clear ADC conversion fields */
    tmp_u32 &= ADS86_MASK_OUTPUT_FLAGS;

    return tmp_u32;
}

uint16_t ADS86XX_GetAlarmsReg(void)
{
    uint16_t tmp_u16 = adsAlarmsReg;
    adsAlarmsReg     = 0U;

    return tmp_u16;
}

float32_t ADS86XX_GetMeasure_uV(void)
{
    ads86_output_data_t data;
    int32_t             tmp_i32;
    float32_t ret_val = 0.0f;

    /* Get data from ADS */
    data.Data_U32 = ADS86XX_GetOutputData();

    switch((ads86_range_sel_t)data.AdcInputRange)
    {
        case ADS86_RANGE_SEL_BI_3_VREF:
            tmp_i32 = (int32_t)data.ConvResult - ADS86_MIDDLE_CODE;
            ret_val = ((float32_t)tmp_i32 * (1.0f / (float32_t)ADS86_MIDDLE_CODE)) * (3.0f * ADS86_REF);
            break;
        case ADS86_RANGE_SEL_BI_2_5_VREF:
            tmp_i32 = (int32_t)data.ConvResult - ADS86_MIDDLE_CODE;
            ret_val = ((float32_t)tmp_i32 * (1.0f / (float32_t)ADS86_MIDDLE_CODE)) * (2.5f * ADS86_REF);
            break;
        case ADS86_RANGE_SEL_BI_1_5_VREF:
            tmp_i32 = (int32_t)data.ConvResult - ADS86_MIDDLE_CODE;
            ret_val = ((float32_t)tmp_i32 * (1.0f / (float32_t)ADS86_MIDDLE_CODE)) * (1.5f * ADS86_REF);
            break;
        case ADS86_RANGE_SEL_BI_1_25_VREF:
            tmp_i32 = (int32_t)data.ConvResult - ADS86_MIDDLE_CODE;
            ret_val = ((float32_t)tmp_i32 * (1.0f / (float32_t)ADS86_MIDDLE_CODE)) * (1.25f * ADS86_REF);
            break;
        case ADS86_RANGE_SEL_BI_0_625_VREF:
            tmp_i32 = (int32_t)data.ConvResult - ADS86_MIDDLE_CODE;
            ret_val = ((float32_t)tmp_i32 * (1.0f / (float32_t)ADS86_MIDDLE_CODE)) * (0.625f * ADS86_REF);
            break;
        case ADS86_RANGE_SEL_UNI_3_VREF:
            tmp_i32 = (int32_t)data.ConvResult;
            ret_val = ((float32_t)tmp_i32 * (1.0f / (float32_t)ADS86_MAX_VAL)) * (3.0f * ADS86_REF);
            break;
        case ADS86_RANGE_SEL_UNI_2_5_VREF:
            tmp_i32 = (int32_t)data.ConvResult;
            ret_val = ((float32_t)tmp_i32 * (1.0f / (float32_t)ADS86_MAX_VAL)) * (2.5f * ADS86_REF);
            break;
        case ADS86_RANGE_SEL_UNI_1_5_VREF:
            tmp_i32 = (int32_t)data.ConvResult;
            ret_val = ((float32_t)tmp_i32 * (1.0f / (float32_t)ADS86_MAX_VAL)) * (1.5f * ADS86_REF);
            break;
        case ADS86_RANGE_SEL_UNI_1_25_VREF:
            tmp_i32 = (int32_t)data.ConvResult;
            ret_val = ((float32_t)tmp_i32 * (1.0f / (float32_t)ADS86_MAX_VAL)) * (1.25f * ADS86_REF);
            break;
        default:
            break;
    }

    return ret_val;
}

void ADS86XX_PrintInfo(void)
{
    #ifdef PRINT_INFO
    ads86_output_data_t out_info;
    ads86_alarm_reg_t alarms;

    uint32_t output_raw = ADS86XX_GetOutputData();
    int32_t in_mv       = (int32_t)ADS86XX_GetMeasure_uV();
    out_info.Data_U32   = ADS86XX_GetOutputDataFlags();
    alarms.Data_U16     = ADS86XX_GetAlarmsReg();

    printf("\n\n%s", ADS86_DEVICE_NAME " Service ...");
    printf("\n%-32s : %ld [uV]", "Input voltage", (long int)in_mv);
    printf("\n%-32s : 0x%.8lX",  "Raw data", (unsigned long)output_raw);

    printf("\n%-32s : 0x%.4X", "Raw Out Info", (uint16_t)out_info.Data_U32);
    printf("\n%-32s : 0x%.1X", "InputAlarmFlags", out_info.InputAlarmFlags);
    printf("\n%-32s : 0x%.1X", "AvddAlarmFlags",  out_info.AvddAlarmFlags);
    printf("\n%-32s : 0x%.1X", "AdcInputRange",   out_info.AdcInputRange);
    printf("\n%-32s : 0x%.1X", "DeviceAddress",   out_info.DeviceAddress);

    if(alarms.Data_U16 != 0U)
    {
        printf("\n%-32s : %u", "OVW_ALARM",    alarms.OVW_ALARM);
        printf("\n%-32s : %u", "TRIP_IN_H",    alarms.TRIP_IN_H);
        printf("\n%-32s : %u", "TRIP_IN_L",    alarms.TRIP_IN_L);
        printf("\n%-32s : %u", "TRIP_VDD_H",   alarms.TRIP_VDD_H);
        printf("\n%-32s : %u", "TRIP_VDD_L",   alarms.TRIP_VDD_L);
        printf("\n%-32s : %u", "ACTIVE_IN_H",  alarms.ACTIVE_IN_H);
        printf("\n%-32s : %u", "ACTIVE_IN_L",  alarms.ACTIVE_IN_L);
        printf("\n%-32s : %u", "ACTIVE_VDD_H", alarms.ACTIVE_VDD_H);
        printf("\n%-32s : %u", "ACTIVE_VDD_L", alarms.ACTIVE_VDD_L);
        printf("\n%-32s : %u", "OVW_ALARM",    alarms.OVW_ALARM);
    }
    #endif
}

void ADS86XX_Start_ServiceInPolling(void)
{
    if(!enServiceInPolling)
    {
        XMC_CCU8_SLICE_StopClearTimer(ADS86_SAMPLING_CCU8_SLICE_PTR);

        NVIC_DisableIRQ(ADS86_RVS_PIN_Isr_Type);
        NVIC_DisableIRQ(ADS86_RXFIFO_Isr_Type);

        XMC_CCU8_SLICE_ClearEvent(ADS86_SAMPLING_CCU8_SLICE_PTR, XMC_CCU8_SLICE_IRQ_ID_PERIOD_MATCH);

        XMC_USIC_CH_RXFIFO_Flush(ADS86_SPI);
        XMC_USIC_CH_TXFIFO_Flush(ADS86_SPI);

        XMC_USIC_CH_RXFIFO_ClearEvent(ADS86_SPI, (uint32_t)XMC_USIC_CH_RXFIFO_EVENT_STANDARD |
                                                 (uint32_t)XMC_USIC_CH_RXFIFO_EVENT_ERROR);

        enServiceInPolling = true;

        ads_acq_start();

        #ifdef PRINT_INFO
        printf("\nGoto polling mode...\n");
        #endif
    }
}

void ADS86XX_Start_ServiceInIsr(void)
{
    if(enServiceInPolling)
    {
        enServiceInPolling = false;

        XMC_USIC_CH_RXFIFO_Flush(ADS86_SPI);
        XMC_USIC_CH_TXFIFO_Flush(ADS86_SPI);

        XMC_USIC_CH_RXFIFO_ClearEvent(ADS86_SPI, (uint32_t)XMC_USIC_CH_RXFIFO_EVENT_STANDARD |
                                                 (uint32_t)XMC_USIC_CH_RXFIFO_EVENT_ERROR);

        NVIC_ClearPendingIRQ(ADS86_RXFIFO_Isr_Type);
        NVIC_ClearPendingIRQ(ADS86_RVS_PIN_Isr_Type);

        NVIC_EnableIRQ(ADS86_RXFIFO_Isr_Type);
        NVIC_EnableIRQ(ADS86_RVS_PIN_Isr_Type);

        ads_acq_start();

        XMC_CCU8_SLICE_StartTimer(ADS86_SAMPLING_CCU8_SLICE_PTR);

        #ifdef PRINT_INFO
        printf("\nGoto ISR mode...\n");
        #endif
    }
}

void ADS86XX_ServiceInPolling(void)
{
    if(enServiceInPolling)
    {
        /* Force conversion */
        ads_conv_start();
        while(XMC_GPIO_GetInput(ADS86_RVS_PORT, ADS86_RVS_PIN) == 0UL);
        ads_acq_start();

        /* Get data */
        uint32_t tmp_u32 = ads86_send_cmd(ADS86_CMD_NOP, 0U, 0U);

        #if   (ADS86_ANALOG_IN_TYPE == ADS86_ANALOG_IN_TYPE_BIPOLAR)
        float32_t tmp_pu = ADS86XX_bipolar_2_pu(tmp_u32 >> ADS86_CONV_RESAULT_POS);
        #elif (ADS86_ANALOG_IN_TYPE == ADS86_ANALOG_IN_TYPE_UNIPOLAR)
        float32_t tmp_pu = ADS86XX_unipolar_2_pu(tmp_u32 >> ADS86_CONV_RESAULT_POS);
        #else
        #error "ADS86_ANALOG_IN_TYPE wrong configuration!!!"
        #endif

        /* Update global variable */
        adsMeasure_pu = tmp_pu;
        adsOutputData = tmp_u32;

        #if (ADS86_PUT_DATA_TO_DAC_EN == 1)
        DAC_Ch0_PU_2_DAC(tmp_pu);
        #endif

        if(readAdsAlarms)
        {
            /* Set CS to HI and wait for finish conversion */
            ads_conv_start();
            while(XMC_GPIO_GetInput(ADS86_RVS_PORT, ADS86_RVS_PIN) == 0UL);

            /* Reconfigure CONVST to CS pin  */
            ads_reconfig_pin_convst_to_cs();

            /* Read alarm register */
            ads86_send_cmd(ADS86_CMD_READ_HWORD, ADS86_ALARM_REG, 0U);
            tmp_u32 = ads86_send_cmd(ADS86_CMD_NOP, 0U, 0U);
            /* Only lower words is occupied by alarm flags */
            adsAlarmsReg |= (uint16_t)(tmp_u32 >> 16);

            /* Reconfigure CS to CONVST pin and start acquisition */
            ads_reconfig_pin_cs_to_convst();
            ads_acq_start();

            /* force to read alarm register if ALARM pin is still HI */
            readAdsAlarms = (bool)XMC_GPIO_GetInput(ADS86_ALARM_PORT, ADS86_ALARM_PIN);
        }
    }
}

float32_t ADS86XX_GetMeasure_PU(void)
{
    float32_t u = adsMeasure_pu;

    return u;
}
/* Switch off pedantic checking */
#if defined ( __GNUC__ )
#pragma GCC diagnostic pop
#endif
