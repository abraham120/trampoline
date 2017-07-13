/*
 * Copyright (c) 2013 - 2015, Freescale Semiconductor, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * o Redistributions of source code must retain the above copyright notice, this list
 *   of conditions and the following disclaimer.
 *
 * o Redistributions in binary form must reproduce the above copyright notice, this
 *   list of conditions and the following disclaimer in the documentation and/or
 *   other materials provided with the distribution.
 *
 * o Neither the name of Freescale Semiconductor, Inc. nor the names of its
 *   contributors may be used to endorse or promote products derived from this
 *   software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 * ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef FSL_TRGMUX_HAL_H
#define FSL_TRGMUX_HAL_H

#include <stdint.h>
#include <stdbool.h>
#include "fsl_device_registers.h"

/**
 * @page misra_violations MISRA-C:2012 violations
 *
 * @section [global]
 * Violates MISRA 2012 Required Rule 5.1, identifier clash
 * The supported compilers use more than 31 significant characters for identifiers.
 *
 * @section [global]
 * Violates MISRA 2012 Required Rule 5.2, identifier clash
 * The supported compilers use more than 31 significant characters for identifiers.
 *
 * @section [global]
 * Violates MISRA 2012 Required Rule 5.4, identifier clash
 * The supported compilers use more than 31 significant characters for identifiers.
 *
 * @section [global]
 * Violates MISRA 2012 Required Rule 5.5, identifier clash
 * The supported compilers use more than 31 significant characters for identifiers.
 */

/*! @file fsl_trgmux_hal.h */

/*!
 * @defgroup trgmux_hal TRGMUX HAL
 * @ingroup trgmux
 * @brief Trigmux Hardware Abstraction Layer.
 * This HAL provides low-level access to all hardware features of TRIGMUX.
 *
 * @addtogroup trgmux_hal
 * @{
 */

/*******************************************************************************
 * Definitions
 *******************************************************************************/

/*! @brief Describes all possible inputs (trigger sources) of the TRGMUX IP
 * Implements : trgmux_trigger_source_t_Class
 */
typedef enum
{
    TRGMUX_TRIG_SOURCE_DISABLED              = 0x0U,
    TRGMUX_TRIG_SOURCE_VDD                   = 0x1U,
    TRGMUX_TRIG_SOURCE_TRGMUX_IN0            = 0x2U,
    TRGMUX_TRIG_SOURCE_TRGMUX_IN1            = 0x3U,
    TRGMUX_TRIG_SOURCE_TRGMUX_IN2            = 0x4U,
    TRGMUX_TRIG_SOURCE_TRGMUX_IN3            = 0x5U,
    TRGMUX_TRIG_SOURCE_TRGMUX_IN4            = 0x6U,
    TRGMUX_TRIG_SOURCE_TRGMUX_IN5            = 0x7U,
    TRGMUX_TRIG_SOURCE_TRGMUX_IN6            = 0x8U,
    TRGMUX_TRIG_SOURCE_TRGMUX_IN7            = 0x9U,
    TRGMUX_TRIG_SOURCE_TRGMUX_IN8            = 0xAU,
    TRGMUX_TRIG_SOURCE_TRGMUX_IN9            = 0xBU,
    TRGMUX_TRIG_SOURCE_TRGMUX_IN10           = 0xCU,
    TRGMUX_TRIG_SOURCE_TRGMUX_IN11           = 0xDU,
    TRGMUX_TRIG_SOURCE_CMP0_OUT              = 0xEU,
    TRGMUX_TRIG_SOURCE_LPIT_CH0              = 0x11U,
    TRGMUX_TRIG_SOURCE_LPIT_CH1              = 0x12U,
    TRGMUX_TRIG_SOURCE_LPIT_CH2              = 0x13U,
    TRGMUX_TRIG_SOURCE_LPIT_CH3              = 0x14U,
    TRGMUX_TRIG_SOURCE_LPTMR0                = 0x15U,
    TRGMUX_TRIG_SOURCE_FTM0_INIT_TRIG        = 0x16U,
    TRGMUX_TRIG_SOURCE_FTM0_EXT_TRIG         = 0x17U,
    TRGMUX_TRIG_SOURCE_FTM1_INIT_TRIG        = 0x18U,
    TRGMUX_TRIG_SOURCE_FTM1_EXT_TRIG         = 0x19U,
    TRGMUX_TRIG_SOURCE_FTM2_INIT_TRIG        = 0x1AU,
    TRGMUX_TRIG_SOURCE_FTM2_EXT_TRIG         = 0x1BU,
    TRGMUX_TRIG_SOURCE_FTM3_INIT_TRIG        = 0x1CU,
    TRGMUX_TRIG_SOURCE_FTM3_EXT_TRIG         = 0x1DU,
    TRGMUX_TRIG_SOURCE_ADC0_SC1A_COCO        = 0x1EU,
    TRGMUX_TRIG_SOURCE_ADC0_SC1B_COCO        = 0x1FU,
    TRGMUX_TRIG_SOURCE_ADC1_SC1A_COCO        = 0x20U,
    TRGMUX_TRIG_SOURCE_ADC1_SC1B_COCO        = 0x21U,
    TRGMUX_TRIG_SOURCE_PDB0_CH0_TRIG         = 0x22U,
    TRGMUX_TRIG_SOURCE_PDB0_PULSE_OUT        = 0x24U,
    TRGMUX_TRIG_SOURCE_PDB1_CH0_TRIG         = 0x25U,
    TRGMUX_TRIG_SOURCE_PDB1_PULSE_OUT        = 0x27U,
    TRGMUX_TRIG_SOURCE_RTC_ALARM             = 0x2BU,
    TRGMUX_TRIG_SOURCE_RTC_SECOND            = 0x2CU,
    TRGMUX_TRIG_SOURCE_FLEXIO_TRIG0          = 0x2DU,
    TRGMUX_TRIG_SOURCE_FLEXIO_TRIG1          = 0x2EU,
    TRGMUX_TRIG_SOURCE_FLEXIO_TRIG2          = 0x2FU,
    TRGMUX_TRIG_SOURCE_FLEXIO_TRIG3          = 0x30U,
    TRGMUX_TRIG_SOURCE_LPUART0_RX_DATA       = 0x31U,
    TRGMUX_TRIG_SOURCE_LPUART0_TX_DATA       = 0x32U,
    TRGMUX_TRIG_SOURCE_LPUART0_RX_IDLE       = 0x33U,
    TRGMUX_TRIG_SOURCE_LPUART1_RX_DATA       = 0x34U,
    TRGMUX_TRIG_SOURCE_LPUART1_TX_DATA       = 0x35U,
    TRGMUX_TRIG_SOURCE_LPUART1_RX_IDLE       = 0x36U,
    TRGMUX_TRIG_SOURCE_LPI2C0_MASTER_TRIGGER = 0x37U,
    TRGMUX_TRIG_SOURCE_LPI2C0_SLAVE_TRIGGER  = 0x38U,
    TRGMUX_TRIG_SOURCE_LPSPI0_FRAME          = 0x3BU,
    TRGMUX_TRIG_SOURCE_LPSPI0_RX_DATA        = 0x3CU,
    TRGMUX_TRIG_SOURCE_LPSPI1_FRAME          = 0x3DU,
    TRGMUX_TRIG_SOURCE_LPSPI1_RX_DATA        = 0x3EU,
    TRGMUX_TRIG_SOURCE_SIM_SW_TRIG           = 0x3FU
} trgmux_trigger_source_t;

/*! @brief Describes all possible outputs (target modules) of the TRGMUX IP
 * Implements : trgmux_target_module_t_Class
 */
typedef enum
{
    TRGMUX_TARGET_MODULE_DMA_CH0            = 0U,
    TRGMUX_TARGET_MODULE_DMA_CH1            = 1U,
    TRGMUX_TARGET_MODULE_DMA_CH2            = 2U,
    TRGMUX_TARGET_MODULE_DMA_CH3            = 3U,
    TRGMUX_TARGET_MODULE_TRGMUX_OUT0        = 4U,
    TRGMUX_TARGET_MODULE_TRGMUX_OUT1        = 5U,
    TRGMUX_TARGET_MODULE_TRGMUX_OUT2        = 6U,
    TRGMUX_TARGET_MODULE_TRGMUX_OUT3        = 7U,
    TRGMUX_TARGET_MODULE_TRGMUX_OUT4        = 8U,
    TRGMUX_TARGET_MODULE_TRGMUX_OUT5        = 9U,
    TRGMUX_TARGET_MODULE_TRGMUX_OUT6        = 10U,
    TRGMUX_TARGET_MODULE_TRGMUX_OUT7        = 11U,
    TRGMUX_TARGET_MODULE_ADC0_ADHWT_TLA0    = 12U,
    TRGMUX_TARGET_MODULE_ADC0_ADHWT_TLA1    = 13U,
    TRGMUX_TARGET_MODULE_ADC0_ADHWT_TLA2    = 14U,
    TRGMUX_TARGET_MODULE_ADC0_ADHWT_TLA3    = 15U,
    TRGMUX_TARGET_MODULE_ADC1_ADHWT_TLA0    = 16U,
    TRGMUX_TARGET_MODULE_ADC1_ADHWT_TLA1    = 17U,
    TRGMUX_TARGET_MODULE_ADC1_ADHWT_TLA2    = 18U,
    TRGMUX_TARGET_MODULE_ADC1_ADHWT_TLA3    = 19U,
    TRGMUX_TARGET_MODULE_CMP0_SAMPLE_INPUT  = 28U,
    TRGMUX_TARGET_MODULE_FTM0_HWTRIG0       = 40U,
    TRGMUX_TARGET_MODULE_FTM0_FAULT0        = 41U,
    TRGMUX_TARGET_MODULE_FTM0_FAULT1        = 42U,
    TRGMUX_TARGET_MODULE_FTM0_FAULT2        = 43U,
    TRGMUX_TARGET_MODULE_FTM1_HWTRIG0       = 44U,
    TRGMUX_TARGET_MODULE_FTM1_FAULT0        = 45U,
    TRGMUX_TARGET_MODULE_FTM1_FAULT1        = 46U,
    TRGMUX_TARGET_MODULE_FTM1_FAULT2        = 47U,
    TRGMUX_TARGET_MODULE_FTM2_HWTRIG0       = 48U,
    TRGMUX_TARGET_MODULE_FTM2_FAULT0        = 49U,
    TRGMUX_TARGET_MODULE_FTM2_FAULT1        = 50U,
    TRGMUX_TARGET_MODULE_FTM2_FAULT2        = 51U,
    TRGMUX_TARGET_MODULE_FTM3_HWTRIG0       = 52U,
    TRGMUX_TARGET_MODULE_FTM3_FAULT0        = 53U,
    TRGMUX_TARGET_MODULE_FTM3_FAULT1        = 54U,
    TRGMUX_TARGET_MODULE_FTM3_FAULT2        = 55U,
    TRGMUX_TARGET_MODULE_PDB0_TRG_IN        = 56U,
    TRGMUX_TARGET_MODULE_PDB1_TRG_IN        = 60U,
    TRGMUX_TARGET_MODULE_FLEXIO_TRG_TIM0    = 68U,
    TRGMUX_TARGET_MODULE_FLEXIO_TRG_TIM1    = 69U,
    TRGMUX_TARGET_MODULE_FLEXIO_TRG_TIM2    = 70U,
    TRGMUX_TARGET_MODULE_FLEXIO_TRG_TIM3    = 71U,
    TRGMUX_TARGET_MODULE_LPIT_TRG_CH0       = 72U,
    TRGMUX_TARGET_MODULE_LPIT_TRG_CH1       = 73U,
    TRGMUX_TARGET_MODULE_LPIT_TRG_CH2       = 74U,
    TRGMUX_TARGET_MODULE_LPIT_TRG_CH3       = 75U,
    TRGMUX_TARGET_MODULE_LPUART0_TRG        = 76U,
    TRGMUX_TARGET_MODULE_LPUART1_TRG        = 80U,
    TRGMUX_TARGET_MODULE_LPI2C0_TRG         = 84U,
    TRGMUX_TARGET_MODULE_LPSPI0_TRG         = 92U,
    TRGMUX_TARGET_MODULE_LPSPI1_TRG         = 96U,
    TRGMUX_TARGET_MODULE_LPTMR0_ALT0        = 100U
} trgmux_target_module_t;


/*! @brief trgmux status return codes.
 * Implements : trgmux_status_t_Class
 */
typedef enum
{
    TRGMUX_STATUS_SUCCESS                    = 0x0U, /*!< TRGMUX operation Succeed  */
	TRGMUX_STATUS_ERROR_TARGET_LOCKED        = 0x1U
} trgmux_status_t;

/*******************************************************************************
 * API
 *******************************************************************************/

#if defined(__cplusplus)
extern "C" {
#endif


/*!
 * @brief Restores the TRGMUX module to reset value.
 *
 * This function restores the TRGMUX module to reset value.
 *
 * @param base    The TRGMUX peripheral base address
 * @return        Execution status:
 *   TRGMUX_STATUS_SUCCESS
 *   TRGMUX_STATUS_ERROR_TARGET_LOCKED - if at least one of the target module register is locked.
 */
trgmux_status_t TRGMUX_HAL_Init(TRGMUX_Type * const base);

/*!
 * @brief Configures a source trigger for a target module.
 *
 * This function configures in TRGMUX IP the link between an input (source trigger) and
 * and output (target module).
 *
 * @param base              The TRGMUX peripheral base address
 * @param triggerSource     One of the values in the trgmux_trigger_source_t enumeration
 * @param targetModule      One of the values in the trgmux_target_module_t enumeration
 */
void TRGMUX_HAL_SetTrigSourceForTargetModule(
                                                TRGMUX_Type * const             base,
                                                const trgmux_trigger_source_t   triggerSource,
                                                const trgmux_target_module_t    targetModule
                                            );

/*!
 * @brief Gets the source trigger configured for a target module.
 *
 * This function reads from the TRGMUX IP the input (source trigger) that is linked
 * to a given output (target module).
 *
 * @param base              The TRGMUX peripheral base address
 * @param targetModule      One of the values in the trgmux_target_module_t enumeration
 * @return                  Enum value corresponding to the trigger source configured
 *                          for the given target module
 */
trgmux_trigger_source_t TRGMUX_HAL_GetTrigSourceForTargetModule(
                                                   const TRGMUX_Type * const     base,
                                                   const trgmux_target_module_t  targetModule
                                                               );

/*!
 * @brief Locks the TRGMUX register of a target module.
 *
 * This function sets to 0x1 the LK bit of the TRGMUX register containing SEL bitfield for
 * a given target module. Please note that some TRGMUX registers can contain up to 4
 * SEL bitfields, meaning that these registers can be used to configure up to 4 target
 * modules independently. Because of the fact that the LK bit is only one per register,
 * the first target module that locks the register will lock it for the other 3 target
 * modules too.
 *
 * @param base             The TRGMUX peripheral base address
 * @param targetModule     One of the values in the trgmux_target_module_t enumeration
 */
void TRGMUX_HAL_SetLockForTargetModule(
                                        TRGMUX_Type * const           base,
                                        const trgmux_target_module_t  targetModule
                                      );

/*!
 * @brief Get the Lock bit status of the TRGMUX register of a target module.
 *
 * This function gets the value of the LK bit of the TRGMUX register containing SEL bitfield for
 * a given target module.
 *
 * @param base              The TRGMUX peripheral base address
 * @param targetModule      One of the values in the trgmux_target_module_t enumeration
 * @return                  true or false depending on the state of the LK bit
 */
bool TRGMUX_HAL_GetLockForTargetModule(
                                        const TRGMUX_Type * const     base,
                                        const trgmux_target_module_t  targetModule
                                       );


/*! @} */

#if defined(__cplusplus)
}
#endif

#endif /* FSL_TRGMUX_HAL_H */
/*******************************************************************************
 * EOF
 *******************************************************************************/
