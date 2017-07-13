/*
 * Copyright (c) 2016, Freescale Semiconductor, Inc.
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

#ifndef FSL_ADC_HAL_H
#define FSL_ADC_HAL_H

#include <stdint.h>
#include <stdbool.h>
#include "fsl_device_registers.h"

/**
* @page misra_violations MISRA-C:2012 violations
*
* @section [global]
* Violates MISRA 2012 Advisory Rule 10.5,
* Impermissible cast; cannot cast from 'essentially unsigned' to 'essentially enum<i>'.
* All possible values are covered by the enumeration, direct casting is used to optimize code.
*
* @section [global]
* Violates MISRA 2012 Required Rule 10.3,
* Expression assigned to a narrower or different essential type.
* All possible values are covered by the enumeration, direct casting is used to optimize code.
*
*/

/*! @file */

/*!
 * @addtogroup adc_hal
 * @{
 */

/*******************************************************************************
 * Definitions
 ******************************************************************************/
/*!
 * @brief Clock Divider selection
 * Implements : adc_clk_divide_t_Class
 */
typedef enum
{
    ADC_CLK_DIVIDE_1 = 0x00U,   /*!< Input clock divided by 1. */
    ADC_CLK_DIVIDE_2 = 0x01U,   /*!< Input clock divided by 2. */
    ADC_CLK_DIVIDE_4 = 0x02U,   /*!< Input clock divided by 4. */
    ADC_CLK_DIVIDE_8 = 0x03U    /*!< Input clock divided by 8. */
} adc_clk_divide_t;

/*!
 * @brief Conversion resolution selection
 * Implements : adc_resolution_t_Class
 */
typedef enum
{
    ADC_RESOLUTION_8BIT = 0x00U,    /*!< 8-bit resolution mode */
    ADC_RESOLUTION_12BIT = 0x01U,   /*!< 12-bit resolution mode */
    ADC_RESOLUTION_10BIT = 0x02U    /*!< 10-bit resolution mode */
} adc_resolution_t;

/*!
 * @brief Input clock source selection
 * Implements : adc_input_clock_t_Class
 */
typedef enum
{
    ADC_CLK_ALT_1 = 0x00U,  /*!< Input clock alternative 1. */
    ADC_CLK_ALT_2 = 0x01U,  /*!< Input clock alternative 2. */
    ADC_CLK_ALT_3 = 0x02U,  /*!< Input clock alternative 3. */
    ADC_CLK_ALT_4 = 0x03U   /*!< Input clock alternative 4. */
} adc_input_clock_t;

/*!
 * @brief Trigger type selection
 * Implements : adc_trigger_t_Class
 */
typedef enum
{
    ADC_TRIGGER_SOFTWARE = 0x00U,   /*!< Software trigger. */
    ADC_TRIGGER_HARDWARE = 0x01U     /*!< Hardware trigger. */
} adc_trigger_t;

/*!
 * @brief Voltage reference selection
 * Implements : adc_voltage_reference_t_Class
 */
typedef enum
{
    ADC_VOLTAGEREF_VREF = 0x00U,    /*!< VrefH and VrefL as Voltage reference. */
    ADC_VOLTAGEREF_VALT = 0x01U     /*!< ValtH and ValtL as Voltage reference. */
} adc_voltage_reference_t;

/*!
 * @brief Hardware average selection
 * Implements : adc_average_t_Class
 */
typedef enum
{
    ADC_AVERAGE_4 = 0x00U,  /*!< Hardware average of 4 samples. */
    ADC_AVERAGE_8 = 0x01U,  /*!< Hardware average of 8 samples. */
    ADC_AVERAGE_16 = 0x02U, /*!< Hardware average of 16 samples. */
    ADC_AVERAGE_32 = 0x03U  /*!< Hardware average of 32 samples. */
} adc_average_t;

/*!
 * @name Input Channel selection.
 * Enumerations and macros to select Input channel.
 */
/*! @{*/

/*!
 * @brief Input channel selection
 * Implements : adc_inputchannel_t_Class
 */
typedef enum
{
    ADC_INPUTCHAN_AD0 = 0x00U,      /*!< AD0  */
    ADC_INPUTCHAN_AD1 = 0x01U,      /*!< AD1  */
    ADC_INPUTCHAN_AD2 = 0x02U,      /*!< AD2  */
    ADC_INPUTCHAN_AD3 = 0x03U,      /*!< AD3  */
    ADC_INPUTCHAN_AD4 = 0x04U,      /*!< AD4  */
    ADC_INPUTCHAN_AD5 = 0x05U,      /*!< AD5  */
    ADC_INPUTCHAN_AD6 = 0x06U,      /*!< AD6  */
    ADC_INPUTCHAN_AD7 = 0x07U,      /*!< AD7  */
    ADC_INPUTCHAN_AD8 = 0x08U,      /*!< AD8  */
    ADC_INPUTCHAN_AD9 = 0x09U,      /*!< AD9  */
    ADC_INPUTCHAN_AD10 = 0x0AU,     /*!< AD10 */
    ADC_INPUTCHAN_AD11 = 0x0BU,     /*!< AD11 */
    ADC_INPUTCHAN_AD12 = 0x0CU,     /*!< AD12 */
    ADC_INPUTCHAN_AD13 = 0x0DU,     /*!< AD13 */
    ADC_INPUTCHAN_AD14 = 0x0EU,     /*!< AD14 */
    ADC_INPUTCHAN_AD15 = 0x0FU,     /*!< AD15 */
    ADC_INPUTCHAN_AD16 = 0x10U,     /*!< AD16 */
    ADC_INPUTCHAN_AD17 = 0x11U,     /*!< AD17 */
    ADC_INPUTCHAN_AD18 = 0x12U,     /*!< AD18 */
    ADC_INPUTCHAN_AD19 = 0x13U,     /*!< AD19 */
    ADC_INPUTCHAN_AD20 = 0x14U,     /*!< AD20 */
    ADC_INPUTCHAN_AD21 = 0x15U,     /*!< AD21 */
    ADC_INPUTCHAN_AD22 = 0x16U,     /*!< AD22 */
    ADC_INPUTCHAN_AD23 = 0x17U,     /*!< AD23 */
    ADC_INPUTCHAN_AD24 = 0x18U,     /*!< AD24 */
    ADC_INPUTCHAN_AD25 = 0x19U,     /*!< AD25 */
    ADC_INPUTCHAN_AD26 = 0x1AU,     /*!< AD26 */
    ADC_INPUTCHAN_AD27 = 0x1BU,     /*!< AD27 */
    ADC_INPUTCHAN_AD28 = 0x1CU,     /*!< AD28 */
    ADC_INPUTCHAN_AD29 = 0x1DU,     /*!< AD29 */
    ADC_INPUTCHAN_AD30 = 0x1EU,     /*!< AD30 */
    ADC_INPUTCHAN_AD31 = 0x1FU      /*!< AD31 */
} adc_inputchannel_t;

/*! @}*/

/*******************************************************************************
 * API
 ******************************************************************************/

 #if defined(__cplusplus)
extern "C" {
#endif

/*!
 * @name Converter
 * General ADC functions.
 */
/*! @{*/

/*!
 * @brief Initializes the ADC instance to reset values.
 *
 * This function initializes the ADC instance to a known
 * state (the register are written with their reset values
 * from the Reference Manual).
 *
 *
 * @param[in] baseAddr adc base pointer
 */
void ADC_HAL_Init(ADC_Type* const baseAddr);

/*!
 * @brief Gets the Conversion Active Flag
 *
 * This function checks whether a conversion is currently
 * taking place on the ADC module.
 *
 *
 * @param[in] baseAddr adc base pointer
 * @return Conversion Active Flag state
 *
 * Implements : ADC_HAL_GetConvActiveFlag_Activity
 */
static inline bool ADC_HAL_GetConvActiveFlag(const ADC_Type* const baseAddr)
{
    uint32_t tmp = (uint32_t)baseAddr->SC2;
    tmp = (tmp & ADC_SC2_ADACT_MASK) >> ADC_SC2_ADACT_SHIFT;
    return (tmp != 0u) ? true : false;
}

/*!
 * @brief Gets the current ADC clock divider configuration.
 *
 * This function returns the configured clock divider
 * bitfield value for the ADC instance.
 *
 * @param[in] baseAddr adc base pointer
 * @return the clock divider value. Possible values:
 *        - ADC_CLK_DIVIDE_1 : Divider set to 1.
 *        - ADC_CLK_DIVIDE_2 : Divider set to 2.
 *        - ADC_CLK_DIVIDE_4 : Divider set to 4.
 *        - ADC_CLK_DIVIDE_8 : Divider set to 8.
 *
 * Implements : ADC_HAL_GetClockDivide_Activity
 */
static inline adc_clk_divide_t ADC_HAL_GetClockDivide(const ADC_Type* const baseAddr)
{
    uint32_t tmp = baseAddr->CFG1;
    tmp = (tmp & ADC_CFG1_ADIV_MASK) >> ADC_CFG1_ADIV_SHIFT;
    /* Enum defines all possible values, so casting is safe */
    return (adc_clk_divide_t)(tmp);
}

/*!
 * @brief Sets the ADC clock divider configuration.
 *
 * This functions configures the ADC instance clock divider.
 *
 * @param[in] baseAddr adc base pointer
 * @param[in] clockDivide clk divider
 *        - ADC_CLK_DIVIDE_1 : Divider set to 1.
 *        - ADC_CLK_DIVIDE_2 : Divider set to 2.
 *        - ADC_CLK_DIVIDE_4 : Divider set to 4.
 *        - ADC_CLK_DIVIDE_8 : Divider set to 8.
 *
 * Implements : ADC_HAL_SetClockDivide_Activity
 */
static inline void ADC_HAL_SetClockDivide(ADC_Type* const baseAddr, const adc_clk_divide_t clockDivide)
{
    uint32_t tmp = baseAddr->CFG1;
    tmp &= ~(ADC_CFG1_ADIV_MASK);
    tmp |= ADC_CFG1_ADIV(clockDivide);
    baseAddr->CFG1 = tmp;
}

/*!
 * @brief Gets the Sample time in AD clock cycles
 *
 * This function gets the sample time (in AD clocks)
 * configured for the ADC. Selection of 2 to 256 ADCK is
 * possible. The value returned by this function is the
 * sample time minus 1. A sample time of 1 is not supported.
 *
 * @param[in] baseAddr adc base pointer
 * @return the Sample Time in AD Clocks
 *
 * Implements : ADC_HAL_GetSampleTime_Activity
 */
static inline uint8_t ADC_HAL_GetSampleTime(const ADC_Type* const baseAddr)
{
    uint32_t tmp = baseAddr->CFG2;
    tmp = (tmp & ADC_CFG2_SMPLTS_MASK) >> ADC_CFG2_SMPLTS_SHIFT;
    return (uint8_t)(tmp);
}

/*!
 * @brief Sets the Sample time in AD clock cycles
 *
 * This function configures the sample time for the ADC (in
 * ADCK clocks). The actual sample time will be the value
 * provided plus 1.  Selection of 2 to 256 ADCK is possible.
 * A real sample time of 1 is not supported (a parameter value of 0
 * will be automatically be changed to 1).
 *
 * @param[in] baseAddr adc base pointer
 * @param[in] sampletime Sample time in AD Clocks
 *
 * Implements : ADC_HAL_SetSampleTime_Activity
 */
static inline void ADC_HAL_SetSampleTime(ADC_Type* const baseAddr, uint8_t sampletime)
{
    /* Clip sample time to minimum value */
    uint8_t rsampletime = (sampletime > 0U) ? sampletime : 1U;
    uint32_t tmp = baseAddr->CFG2;
    tmp &= ~(ADC_CFG2_SMPLTS_MASK);
    tmp |= ADC_CFG2_SMPLTS(rsampletime);
    baseAddr->CFG2 = tmp;
}

/*!
 * @brief Gets the Resolution Mode configuration
 *
 * This function returns the configured resolution mode for
 * the ADC.
 *
 * @param[in] baseAddr adc base pointer
 * @return the ADC resolution mode. Possible values:
 *        - ADC_RESOLUTION_8BIT : 8-bit resolution mode.
 *        - ADC_RESOLUTION_10BIT : 10-bit resolution mode.
 *        - ADC_RESOLUTION_12BIT : 12-bit resolution mode.
 *
 * Implements : ADC_HAL_GetResolution_Activity
 */
static inline adc_resolution_t ADC_HAL_GetResolution(const ADC_Type* const baseAddr)
{
    uint32_t tmp = baseAddr->CFG1;
    tmp = (tmp & ADC_CFG1_MODE_MASK) >> ADC_CFG1_MODE_SHIFT;
    adc_resolution_t retValue;
    /* Enum does not define all possible values, do a swith-case */
    switch (tmp)
    {
    case 0x00U: retValue = ADC_RESOLUTION_8BIT; break;
    case 0x01U: retValue = ADC_RESOLUTION_12BIT; break;
    case 0x02U: retValue = ADC_RESOLUTION_10BIT; break;
    default: retValue = ADC_RESOLUTION_8BIT; break;
    }
    return retValue;
}

/*!
 * @brief Sets the Resolution Mode configuration
 *
 * This function configures the ADC resolution mode.
 *
 * @param[in] baseAddr adc base pointer
 * @param[in] resolution the adc resolution mode
 *        - ADC_RESOLUTION_8BIT : 8-bit resolution mode.
 *        - ADC_RESOLUTION_10BIT : 10-bit resolution mode.
 *        - ADC_RESOLUTION_12BIT : 12-bit resolution mode.
 *
 * Implements : ADC_HAL_SetResolution_Activity
 */
static inline void ADC_HAL_SetResolution(ADC_Type* const baseAddr, const adc_resolution_t resolution)
{
    uint32_t tmp = baseAddr->CFG1;
    tmp &= ~(ADC_CFG1_MODE_MASK);
    tmp |= ADC_CFG1_MODE(resolution);
    baseAddr->CFG1 = tmp;
}

/*!
 * @brief Gets the AD Clock Input configuration
 *
 * This function returns the configured clock input source
 * for the ADC.
 *
 * @param[in] baseAddr adc base pointer
 * @return the input clock source. Possible values:
 *        - ADC_CLK_ALT_1 : ADC Input clock source alternative 1.
 *        - ADC_CLK_ALT_2 : ADC Input clock source alternative 2.
 *        - ADC_CLK_ALT_3 : ADC Input clock source alternative 3.
 *        - ADC_CLK_ALT_4 : ADC Input clock source alternative 4.
 *
 * Implements : ADC_HAL_GetInputClock_Activity
 */
static inline adc_input_clock_t ADC_HAL_GetInputClock(const ADC_Type* const baseAddr)
{
    uint32_t tmp = baseAddr->CFG1;
    tmp = (tmp & ADC_CFG1_ADICLK_MASK) >> ADC_CFG1_ADICLK_SHIFT;
    /* Enum defines all possible values, so casting is safe */
    return (adc_input_clock_t)(tmp);
}

/*!
 * @brief Sets the AD Clock Input configuration
 *
 * This function configures the clock input source for the
 * ADC.
 *
 * @param[in] baseAddr adc base pointer
 * @param[in] inputClock the new input clock source
 *        - ADC_CLK_ALT_1 : ADC Input clock source alternative 1.
 *        - ADC_CLK_ALT_2 : ADC Input clock source alternative 2.
 *        - ADC_CLK_ALT_3 : ADC Input clock source alternative 3.
 *        - ADC_CLK_ALT_4 : ADC Input clock source alternative 4.
 *
 * Implements : ADC_HAL_SetInputClock_Activity
 */
static inline void ADC_HAL_SetInputClock(ADC_Type* const baseAddr, const adc_input_clock_t inputClock)
{
    uint32_t tmp = baseAddr->CFG1;
    tmp &= ~(ADC_CFG1_ADICLK_MASK);
    tmp |= ADC_CFG1_ADICLK(inputClock);
    baseAddr->CFG1 = tmp;
}

/*!
 * @brief Gets the ADC Trigger Mode
 *
 * This function returns the configured triggering mode
 * for the ADC. In Software Triggering Mode, the user can
 * start conversions by setting an input channel in the
 * ADC measurement channel A (index 0). When in Hardware
 * trigger mode, a conversion is started by another peripheral (
 * like PDB or TRGMUX).
 *
 * @param[in] baseAddr adc base pointer
 * @return the current trigger mode. Possible values:
 *        - ADC_TRIGGER_SOFTWARE : Software triggering.
 *        - ADC_TRIGGER_HARDWARE : Hardware triggering.
 *
 * Implements : ADC_HAL_GetTriggerMode_Activity
 */
static inline adc_trigger_t ADC_HAL_GetTriggerMode(const ADC_Type* const baseAddr)
{
    uint32_t tmp = baseAddr->SC2;
    tmp = (tmp & ADC_SC2_ADTRG_MASK) >> ADC_SC2_ADTRG_SHIFT;
    /* Enum defines all possible values, so casting is safe */
    return (adc_trigger_t)(tmp);
}

/*!
 * @brief Sets the ADC Trigger Mode
 *
 * This function configures the ADC triggering mode. In
 * Software Triggering Mode, the user can start conversions
 * by setting an input channel in the ADC measurement channel
 * A (index 0). When in Hardware trigger mode, a conversion
 * is started by another peripheral (like PDB or TRGMUX).
 *
 * @param[in] baseAddr adc base pointer
 * @param[in] trigger the desired trigger mode
 *        - ADC_TRIGGER_SOFTWARE : Software triggering.
 *        - ADC_TRIGGER_HARDWARE : Hardware triggering.
 *
 * Implements : ADC_HAL_SetTriggerMode_Activity
 */
static inline void ADC_HAL_SetTriggerMode(ADC_Type* const baseAddr, const adc_trigger_t trigger)
{
    uint32_t tmp = baseAddr->SC2;
    tmp &= ~(ADC_SC2_ADTRG_MASK);
    tmp |= ADC_SC2_ADTRG(trigger);
    baseAddr->SC2 = tmp;
}

/*!
 * @brief Gets the DMA Enable Flag state
 *
 * This function returns the state of the DMA Enable flag.
 * DMA can be used to transfer completed conversion values
 * from the result registers to RAM without CPU intervention.
 *
 * @param[in] baseAddr adc base pointer
 * @return the DMA Enable Flag state
 *
 * Implements : ADC_HAL_GetDMAEnableFlag_Activity
 */
static inline bool ADC_HAL_GetDMAEnableFlag(const ADC_Type* const baseAddr)
{
    uint32_t tmp = (uint32_t)baseAddr->SC2;
    tmp = (tmp & ADC_SC2_DMAEN_MASK) >> ADC_SC2_DMAEN_SHIFT;
    return (tmp != 0u) ? true : false;
}

/*!
 * @brief Sets the DMA Enable Flag state
 *
 * This function configures the DMA Enable Flag. DMA can be
 * used to transfer completed conversion values from the
 * result registers to RAM without CPU intervention.
 *
 * @param[in] baseAddr adc base pointer
 * @param[in] state the new DMA Enable Flag state
 *
 * Implements : ADC_HAL_SetDMAEnableFlag_Activity
 */
static inline void ADC_HAL_SetDMAEnableFlag(ADC_Type* const baseAddr, const bool state)
{
    uint32_t tmp = (uint32_t)baseAddr->SC2;
    tmp &= (uint32_t)(~(ADC_SC2_DMAEN_MASK));
    tmp |= ADC_SC2_DMAEN(state ? (uint32_t)1u : (uint32_t)0u);
    baseAddr->SC2 = (uint32_t)tmp;
}

/*!
 * @brief Gets the ADC Reference Voltage selection
 *
 * This function returns the configured reference voltage
 * selection for the ADC. Reference voltage can be selected
 * between the pairs (VrefH, VrefL) and (ValtH, ValtL).
 *
 * @param[in] baseAddr adc base pointer
 * @return the voltage reference input pair. Possible values:
 *        - ADC_VOLTAGEREF_VREF : VrefL and VrefH used as voltage reference.
 *        - ADC_VOLTAGEREF_VALT : ValtL and ValtH used as voltage reference.
 *
 * Implements : ADC_HAL_GetVoltageReference_Activity
 */
static inline adc_voltage_reference_t ADC_HAL_GetVoltageReference(const ADC_Type* const baseAddr)
{
    uint32_t tmp = baseAddr->SC2;
    tmp = (tmp & ADC_SC2_REFSEL_MASK) >> ADC_SC2_REFSEL_SHIFT;
    /* Enum defines all possible values, so casting is safe */
    return (adc_voltage_reference_t)(tmp);
}

/*!
 * @brief Sets the ADC Reference Voltage selection
 *
 * This function configures the ADC Reference Voltage. Reference
 * voltage can be selected between the pairs (VrefH, VrefL)
 * and (ValtH, ValtL).
 *
 * @param[in] baseAddr adc base pointer
 * @param[in] voltageRef the new voltage reference input
 *        - ADC_VOLTAGEREF_VREF : VrefL and VrefH used as voltage reference.
 *        - ADC_VOLTAGEREF_VALT : ValtL and ValtH used as voltage reference.
 *
 * Implements : ADC_HAL_SetVoltageReference_Activity
 */
static inline void ADC_HAL_SetVoltageReference(ADC_Type* const baseAddr, const adc_voltage_reference_t voltageRef)
{
    uint32_t tmp = baseAddr->SC2;
    tmp &= ~(ADC_SC2_REFSEL_MASK);
    tmp |= ADC_SC2_REFSEL(voltageRef);
    baseAddr->SC2 = tmp;
}

/*!
 * @brief Gets the Continuous Conversion Flag state
 *
 * This functions returns the state of the Continuous Conversion
 * Flag. This feature can be used to continuously sample a
 * single channel. When this is active, the channel cannot be
 * changed (by software or hardware trigger) until this feature
 * is turned off.
 *
 * @param[in] baseAddr adc base pointer
 * @return the Continuous Conversion Flag state
 *
 * Implements : ADC_HAL_GetContinuousConvFlag_Activity
 */
static inline bool ADC_HAL_GetContinuousConvFlag(const ADC_Type* const baseAddr)
{
    uint32_t tmp = (uint32_t)baseAddr->SC3;
    tmp = (tmp & ADC_SC3_ADCO_MASK) >> ADC_SC3_ADCO_SHIFT;
    return (tmp != 0u) ? true : false;
}

/*!
 * @brief Sets the Continuous Conversion Flag state
 *
 * This function configures the Continuous Conversion. This
 * feature can be used to continuously sample a single channel.
 * When this is active, the channel cannot be changed (by
 * software or hardware trigger) until this feature is turned
 * off.
 *
 * @param[in] baseAddr adc base pointer
 * @param[in] state the new Continuous Conversion Flag state
 *
 * Implements : ADC_HAL_SetContinuousConvFlag_Activity
 */
static inline void ADC_HAL_SetContinuousConvFlag(ADC_Type* const baseAddr, const bool state)
{
    uint32_t tmp = (uint32_t)baseAddr->SC3;
    /* Clear the affected bitfield and write '0' to the w1c bits to avoid side-effects */
    tmp &= ~(ADC_SC3_ADCO_MASK);
    tmp |= ADC_SC3_ADCO(state ? (uint32_t)1u : (uint32_t)0u);
    baseAddr->SC3 = (uint32_t)tmp;
}

/*! @}*/

/*!
 * @name Hardware Compare.
 * Functions to configure the Hardware Compare feature.
 */
/*! @{*/

/*!
 * @brief Gets the Hardware Compare Enable Flag state
 *
 * This function returns the state of the Hardware Compare
 * Enable Flag. Hardware Compare can be used to check if the
 * ADC result is within or outside of a predefined range.
 *
 * @param[in] baseAddr adc base pointer
 * @return the Hardware Compare Enable Flag state
 *
 * Implements : ADC_HAL_GetHwCompareEnableFlag_Activity
 */
static inline bool ADC_HAL_GetHwCompareEnableFlag(const ADC_Type* const baseAddr)
{
    uint32_t tmp = (uint32_t)baseAddr->SC2;
    tmp = (tmp & ADC_SC2_ACFE_MASK) >> ADC_SC2_ACFE_SHIFT;
    return (tmp != 0u) ? true : false;
}

/*!
 * @brief Sets the Hardware Compare Enable Flag state
 *
 * This functions configures the Hardware Compare Enable Flag.
 * Hardware Compare can be used to check if the ADC result
 * is within or outside of a predefined range.
 *
 * @param[in] baseAddr adc base pointer
 * @param[in] state the new Hardware Compare Enable Flag state
 *
 * Implements : ADC_HAL_SetHwCompareEnableFlag_Activity
 */
static inline void ADC_HAL_SetHwCompareEnableFlag(ADC_Type* const baseAddr, const bool state)
{
    uint32_t tmp = (uint32_t)baseAddr->SC2;
    tmp &= (uint32_t)(~(ADC_SC2_ACFE_MASK));
    tmp |= ADC_SC2_ACFE(state ? (uint32_t)1u : (uint32_t)0u);
    baseAddr->SC2 = (uint32_t)tmp;
}

/*!
 * @brief Gets the Hardware Compare Greater Than Enable Flag state
 *
 * This function returns the Hardware Compare Greater Than
 * Enable Flag. Using this feature, the ADC can be configured
 * to check if the measured value is within or outside of a
 * predefined range.
 *
 * @param[in] baseAddr adc base pointer
 * @return the Hardware Compare Greater Than Enable Flag state
 *
 * Implements : ADC_HAL_GetHwCompareGtEnableFlag_Activity
 */
static inline bool ADC_HAL_GetHwCompareGtEnableFlag(const ADC_Type* const baseAddr)
{
    uint32_t tmp = (uint32_t)baseAddr->SC2;
    tmp = (tmp & ADC_SC2_ACFGT_MASK) >> ADC_SC2_ACFGT_SHIFT;
    return (tmp != 0u) ? true : false;
}

/*!
 * @brief Sets the Hardware Compare Greater Than Enable Flag state
 *
 * This function configures the Hardware Compare Greater Than
 * Enable Flag. Using this feature, the ADC can be configured
 * to check if the measured value is within or outside of a
 * predefined range.
 *
 * @param[in] baseAddr adc base pointer
 * @param[in] state the new Hardware Compare Greater Than Enable Flag state
 *
 * Implements : ADC_HAL_SetHwCompareGtEnableFlag_Activity
 */
static inline void ADC_HAL_SetHwCompareGtEnableFlag(ADC_Type* const baseAddr, const bool state)
{
    uint32_t tmp = (uint32_t)baseAddr->SC2;
    tmp &= (uint32_t)(~(ADC_SC2_ACFGT_MASK));
    tmp |= ADC_SC2_ACFGT(state ? (uint32_t)1u : (uint32_t)0u);
    baseAddr->SC2 = (uint32_t)tmp;
}

/*!
 * @brief Gets the Hardware Compare Range Enable state
 *
 * This function returns the state of the Hardware Compare
 * Range Enable Flag. This feature allows configuration
 * of a range with two non-zero values or with a non-zero
 * and zero value.
 *
 * @param[in] baseAddr adc base pointer
 * @return the Hardware Compare Range Enable Flag state
 *
 * Implements : ADC_HAL_GetHwCompareRangeEnableFlag_Activity
 */
static inline bool ADC_HAL_GetHwCompareRangeEnableFlag(const ADC_Type* const baseAddr)
{
    uint32_t tmp = (uint32_t)baseAddr->SC2;
    tmp = (tmp & ADC_SC2_ACREN_MASK) >> ADC_SC2_ACREN_SHIFT;
    return (tmp != 0u) ? true : false;
}

/*!
 * @brief Sets the Hardware Compare Range Enable state
 *
 * This function configures the Hardware Compare Range
 * Enable Flag. This feature allows configuration
 * of a range with two non-zero values or with a non-zero
 * and zero value.
 *
 * @param[in] baseAddr adc base pointer
 * @param[in] state the new Hardware Compare Range Enable Flag state
 *
 * Implements : ADC_HAL_SetHwCompareRangeEnableFlag_Activity
 */
static inline void ADC_HAL_SetHwCompareRangeEnableFlag(ADC_Type* const baseAddr, const bool state)
{
    uint32_t tmp = (uint32_t)baseAddr->SC2;
    tmp &= (uint32_t)(~(ADC_SC2_ACREN_MASK));
    tmp |= ADC_SC2_ACREN(state ? (uint32_t)1u : (uint32_t)0u);
    baseAddr->SC2 = (uint32_t)tmp;
}

/*!
 * @brief Gets the Compare Register 1 value
 *
 * This function returns the value written in the Hardware
 * Compare Register 1. This value defines the upper or lower
 * limit for the Hardware Compare Range. This value is always
 * 12-bit resolution value (for lower resolution modes, internal
 * bit shifting will take place).
 *
 * @param[in] baseAddr adc base pointer
 * @return the Compare Register 1 value
 *
 * Implements : ADC_HAL_GetHwCompareComp1Value_Activity
 */
static inline uint16_t ADC_HAL_GetHwCompareComp1Value(const ADC_Type* const baseAddr)
{
    return (uint16_t)baseAddr->CV[0U];
}

/*!
 * @brief Sets the Compare Register 1 value
 *
 * This function writes a 12-bit value in the Hardware
 * Compare Register 1. This value defines the upper or lower
 * limit for the Hardware Compare Range. This value is always
 * 12-bit resolution (for lower resolution modes, internal
 * bit shifting will take place).
 *
 * @param[in] baseAddr adc base pointer
 * @param[in] value the new Compare Register 1 value
 *
 * Implements : ADC_HAL_SetHwCompareComp1Value_Activity
 */
static inline void ADC_HAL_SetHwCompareComp1Value(ADC_Type* const baseAddr, const uint16_t value)
{
    baseAddr->CV[0U] = ADC_CV_CV(value);
}

/*!
 * @brief Gets the Compare Register 2 value
 *
 * This function returns the value written in the Hardware
 * Compare Register 2. This value defines the upper or lower
 * limit for the Hardware Compare Range. This value is always
 * 12-bit resolution (for lower resolution modes, internal
 * bit shifting will take place).
 *
 * @param[in] baseAddr adc base pointer
 * @return the Compare Register 2 value
 *
 * Implements : ADC_HAL_GetHwCompareComp2Value_Activity
 */
static inline uint16_t ADC_HAL_GetHwCompareComp2Value(const ADC_Type* const baseAddr)
{
    return (uint16_t)baseAddr->CV[1U];
}

/*!
 * @brief Sets the Compare Register 2 value
 *
 * This function writes a 12-bit value in the Hardware
 * Compare Register 2. This value defines the upper or lower
 * limit for the Hardware Compare Range. This value is always
 * 12-bit resolution value (for lower resolution modes, internal
 * bit shifting will take place).
 *
 * @param[in] baseAddr adc base pointer
 * @param[in] value the new Compare Register 2 value
 *
 * Implements : ADC_HAL_SetHwCompareComp2Value_Activity
 */
static inline void ADC_HAL_SetHwCompareComp2Value(ADC_Type* const baseAddr, const uint16_t value)
{
    baseAddr->CV[1U] = ADC_CV_CV(value);
}

/*! @}*/

/*!
 * @name Hardware Average.
 * Functions to configure the Hardware Averaging feature.
 */
/*! @{*/

/*!
 * @brief Gets the Hardware Average Enable Flag state
 *
 * This function returns the state of the Hardware Average
 * Enable Flag. Hardware averaging can be used to obtain an
 * average value over multiple consecutive conversions on
 * the same channel.
 *
 * @param[in] baseAddr adc base pointer
 * @return the Hardware Average Enable Flag state
 *
 * Implements : ADC_HAL_GetHwAverageEnableFlag_Activity
 */
static inline bool ADC_HAL_GetHwAverageEnableFlag(const ADC_Type* const baseAddr)
{
    uint32_t tmp = (uint32_t)baseAddr->SC3;
    tmp = (tmp & ADC_SC3_AVGE_MASK) >> ADC_SC3_AVGE_SHIFT;
    return (tmp != 0u) ? true : false;
}

/*!
 * @brief Sets the Hardware Average Enable Flag state
 *
 * This function configures the Hardware Average Enable Flag.
 * Hardware averaging can be used to obtain an average value
 * over multiple consecutive conversions on the same channel.
 *
 * @param[in] baseAddr adc base pointer
 * @param[in] state the new Hardware Average Enable Flag state
 *
 * Implements : ADC_HAL_SetHwAverageEnableFlag_Activity
 */
static inline void ADC_HAL_SetHwAverageEnableFlag(ADC_Type* const baseAddr, const bool state)
{
    uint32_t tmp = baseAddr->SC3;
    /* Clear the affected bitfield */
    tmp &= ~(ADC_SC3_AVGE_MASK);
    tmp |= ADC_SC3_AVGE(state ? (uint32_t)1u : (uint32_t)0u);
    baseAddr->SC3 = tmp;
}

/*!
 * @brief Gets the Hardware Average Mode
 *
 * This function returns the configured Hardware Average Mode.
 * The mode selects the number of samples to average: 4, 8, 16
 * or 32.
 *
 * @param[in] baseAddr adc base pointer
 * @return the Hardware Average Mode selection. Possible values:
 *        - ADC_AVERAGE_4 : Hardware average of 4 samples..
 *        - ADC_AVERAGE_8 : Hardware average of 8 samples.
 *        - ADC_AVERAGE_16 : Hardware average of 16 samples.
 *        - ADC_AVERAGE_32 : Hardware average of 32 samples.
 *
 * Implements : ADC_HAL_GetHwAverageMode_Activity
 */
static inline adc_average_t ADC_HAL_GetHwAverageMode(const ADC_Type* const baseAddr)
{
    uint32_t tmp = baseAddr->SC3;
    tmp = (tmp & ADC_SC3_AVGS_MASK) >> ADC_SC3_AVGS_SHIFT;
    /* Enum defines all possible values, so casting is safe */
    return (adc_average_t)(tmp);
}

/*!
 * @brief Sets the Hardware Average Mode
 *
 * This function configures the Hardware Average Mode. The
 * mode selects the number of samples to average: 4, 8, 16
 * or 32.
 *
 * @param[in] baseAddr adc base pointer
 * @param[in] averageMode the new Hardware Average Mode.
 *        - ADC_AVERAGE_4 : Hardware average of 4 samples..
 *        - ADC_AVERAGE_8 : Hardware average of 8 samples.
 *        - ADC_AVERAGE_16 : Hardware average of 16 samples.
 *        - ADC_AVERAGE_32 : Hardware average of 32 samples.
 *
 * Implements : ADC_HAL_SetHwAverageMode_Activity
 */
static inline void ADC_HAL_SetHwAverageMode(ADC_Type* const baseAddr, const adc_average_t averageMode)
{
    uint32_t tmp = baseAddr->SC3;
    /* Clear the affected bitfield */
    tmp &= ~(ADC_SC3_AVGS_MASK);
    tmp |= ADC_SC3_AVGS(averageMode);
    baseAddr->SC3 = tmp;
}

/*! @}*/

/*!
 * @name Automatic Calibration.
 * Functions configure and use the Automatic Calibration feature.
 */
/*! @{*/

/*!
 * @brief Gets the Calibration Active Flag state
 *
 * This function returns the state of the Calibration Active
 * Flag. This flag is set if an Auto-Calibration sequence is
 * taking place.
 *
 * @param[in] baseAddr adc base pointer
 * @return the Calibration Active Flag state
 *
 * Implements : ADC_HAL_GetCalibrationActiveFlag_Activity
 */
static inline bool ADC_HAL_GetCalibrationActiveFlag(const ADC_Type* const baseAddr)
{
    uint32_t tmp = (uint32_t)baseAddr->SC3;
    tmp = (tmp & ADC_SC3_CAL_MASK) >> ADC_SC3_CAL_SHIFT;
    return (tmp != 0u) ? true : false;
}

/*!
 * @brief Sets the Calibration Active Flag state
 *
 * This functions starts or aborts an Auto-Calibration
 * sequence. If this is set, it will remain set until the
 * sequence is finished.
 *
 * @param[in] baseAddr adc base pointer
 * @param[in] state the new Calibration Active Flag state
 *
 * Implements : ADC_HAL_SetCalibrationActiveFlag_Activity
 */
static inline void ADC_HAL_SetCalibrationActiveFlag(ADC_Type* const baseAddr, const bool state)
{
    uint32_t tmp = baseAddr->SC3;
    tmp &= ~(ADC_SC3_CAL_MASK);
    tmp |= ADC_SC3_CAL(state ? (uint32_t)1u : (uint32_t)0u);
    baseAddr->SC3 = tmp;
}

/*!
 * @brief Gets the User Gain Register value
 *
 * This function returns the value in the User Gain Register.
 * The value in this register is the amplification applied
 * to the measured data before being written in the result
 * register.
 *
 * @param[in] baseAddr adc base pointer
 * @return the User Gain Register value
 *
 * Implements : ADC_HAL_GetUserGainValue_Activity
 */
static inline uint16_t ADC_HAL_GetUserGainValue(const ADC_Type* const baseAddr)
{
    uint32_t tmp = baseAddr->UG;
    tmp = (tmp & ADC_UG_UG_MASK) >> ADC_UG_UG_SHIFT;
    return (uint16_t)tmp;
}

/*!
 * @brief Sets the User Gain Register value
 *
 * This function configures the User Gain Register. The value
 * in this register is the amplification applied to the
 * measured data before being written in the result register.
 *
 * @param[in] baseAddr adc base pointer
 * @param[in] value the new User Gain Register value
 *
 * Implements : ADC_HAL_SetUserGainValue_Activity
 */
static inline void ADC_HAL_SetUserGainValue(ADC_Type* const baseAddr, const uint16_t value)
{
    uint16_t clp0 = (uint16_t)baseAddr->CLP0;
    uint16_t clp1 = (uint16_t)baseAddr->CLP1;
    uint16_t clp2 = (uint16_t)baseAddr->CLP2;
    uint16_t clp3 = (uint16_t)baseAddr->CLP3;
    uint16_t clps = (uint16_t)baseAddr->CLPS;
    /* Add CLP0, CLP1, CLP2, CLP3 and CLPS */
    uint16_t sum = value + clp0 + clp1 + clp2 + clp3 + clps;
    /* If OR of bits [15:11] from the sum is 1 (set), write 0xFFFFU to Gain register */
    uint16_t temp = sum & 0xF800U;
    if (temp != 0x0000U){
        temp = 0xFFFFU;
    }
    baseAddr->UG = (uint32_t)value;
    baseAddr->G = (uint32_t)temp;
}

/*!
 * @brief Gets the User Offset Register value
 *
 * This function returns the value in the User Offset Register.
 * The value in this register is subtracted from the measured
 * data before being written in the result register. This value
 * is 16-bit signed value. To preserve resolution, lower-order
 * bits will be ignored in low resolution-modes.
 *
 * @param[in] baseAddr adc base pointer
 * @return the User Offset Register value
 *
 * Implements : ADC_HAL_GetUserOffsetValue_Activity
 */
static inline uint16_t ADC_HAL_GetUserOffsetValue(const ADC_Type* const baseAddr)
{
    uint32_t tmp = baseAddr->USR_OFS;
    tmp = (tmp & ADC_USR_OFS_USR_OFS_MASK) >> ADC_USR_OFS_USR_OFS_SHIFT;
    return (uint16_t)tmp;
}

/*!
 * @brief Sets the User Offset Register value
 *
 * This function configures the User Offset Register. The value
 * in this register is subtracted from the measured data before
 * being written in the result register. This value is 16-bit
 * signed value. To preserve resolution, lower-order bits
 * will be ignored in low resolution-modes.
 *
 * @param[in] baseAddr adc base pointer
 * @param[in] value the new User Offset Register value
 *
 * Implements : ADC_HAL_SetUserOffsetValue_Activity
 */
static inline void ADC_HAL_SetUserOffsetValue(ADC_Type* const baseAddr, const uint16_t value)
{
    baseAddr->USR_OFS = ADC_USR_OFS_USR_OFS(value);
}

/*! @}*/

/*!
 * @name Converter channels.
 * Functions to configure and access each ADC converter channel.
 */
/*! @{*/

/*!
 * @brief Gets the Channel Interrupt Enable state
 *
 * This function returns the state of the Channel Interrupt
 * Enable Flag. If the flag is set, an interrupt is generated
 * when the a conversion is completed for the channel.
 *
 * @param[in] baseAddr adc base pointer
 * @param[in] chanIndex the adc measurement channel index
 * @return the Channel Interrupt Enable Flag state
 *
 * Implements : ADC_HAL_GetChanInterruptEnableFlag_Activity
 */
static inline bool ADC_HAL_GetChanInterruptEnableFlag(const ADC_Type* const baseAddr, const uint8_t chanIndex)
{
    uint32_t tmp = (uint32_t)baseAddr->SC1[chanIndex];
    tmp = (tmp & ADC_SC1_AIEN_MASK) >> ADC_SC1_AIEN_SHIFT;
    return (tmp != 0u) ? true : false;
}

/*!
 * @brief Sets the Channel Interrupt Enable to a new state
 *
 * This function configures the state of the Interrupt Enable
 * Flag for a measurement channel. If the flag is set, an
 * interrupt is generated when the a conversion is completed
 * for the channel.
 *
 * @param[in] baseAddr adc base pointer
 * @param[in] chanIndex the adc measurement channel index
 * @param[in] state the new Channel Interrupt Enable Flag state
 *
 * Implements : ADC_HAL_SetChanInterruptEnableFlag_Activity
 */
static inline void ADC_HAL_SetChanInterruptEnableFlag(ADC_Type* const baseAddr, const uint8_t chanIndex, const bool state)
{
    uint32_t tmp = baseAddr->SC1[chanIndex];
    tmp &= ~(ADC_SC1_AIEN_MASK);
    tmp |= ADC_SC1_AIEN(state ? (uint32_t)1u : (uint32_t)0u);
    /* If software trigger is configured and chanIndex is 0, this will trigger a new conversion */
    baseAddr->SC1[chanIndex] = tmp;
}

/*!
 * @brief Gets the configured input channel for the selected measurement channel
 *
 * This function returns the configured input channel for a
 * measurement channel.
 *
 * @param[in] baseAddr adc base pointer
 * @param[in] chanIndex the adc measurement channel index
 * @return the Input Channel selected for the Measurement Channel. Possible values:
 *        - ADC_INPUTCHAN_AD0 : AD0 selected as input.
 *        - ADC_INPUTCHAN_AD1 : AD1 selected as input.
 *        - ADC_INPUTCHAN_AD2 : AD2 selected as input.
 *        - ADC_INPUTCHAN_AD3 : AD3 selected as input.
 *        - ADC_INPUTCHAN_AD4 : AD4 selected as input.
 *        - ADC_INPUTCHAN_AD5 : AD5 selected as input.
 *        - ADC_INPUTCHAN_AD6 : AD6 selected as input.
 *        - ADC_INPUTCHAN_AD7 : AD7 selected as input.
 *        - ADC_INPUTCHAN_AD8 : AD8 selected as input.
 *        - ADC_INPUTCHAN_AD9 : AD9 selected as input.
 *        - ADC_INPUTCHAN_AD10 : AD10 selected as input.
 *        - ADC_INPUTCHAN_AD11 : AD11 selected as input.
 *        - ADC_INPUTCHAN_AD12 : AD12 selected as input.
 *        - ADC_INPUTCHAN_AD13 : AD13 selected as input.
 *        - ADC_INPUTCHAN_AD14 : AD14 selected as input.
 *        - ADC_INPUTCHAN_AD15 : AD15 selected as input.
 *        - ADC_INPUTCHAN_TEMP : Temp Sensor selected as input.
 *        - ADC_INPUTCHAN_BANDGAP : Band Gap selected as input.
 *        - ADC_INPUTCHAN_VREFSH : VREFSH selected as input.
 *        - ADC_INPUTCHAN_VREFSL : VREFSL selected as input.
 *        - ADC_INPUTCHAN_DISABLED : Channel Disabled.
 *
 * Implements : ADC_HAL_GetInputChannel_Activity
 */
static inline adc_inputchannel_t ADC_HAL_GetInputChannel(const ADC_Type* const baseAddr, const uint8_t chanIndex)
{
    uint32_t tmp = baseAddr->SC1[chanIndex];
    tmp = (tmp & ADC_SC1_ADCH_MASK) >> ADC_SC1_ADCH_SHIFT;
    /* Enum defines all possible values, so casting is safe */
    return (adc_inputchannel_t)(tmp);
}

/*!
 * @brief Sets the input channel configuration for the measurement channel.
 *
 * This function configures the input channel for a measurement
 * channel. In software trigger mode, configuring channel
 * A (index 0) will start a new conversion immediately.
 *
 * @param[in] baseAddr adc base pointer
 * @param[in] chanIndex the adc measurement channel index
 * @param[in] inputChan the Input Channel selected for the Measurement Channel
 *        - ADC_INPUTCHAN_AD0 : AD0 selected as input.
 *        - ADC_INPUTCHAN_AD1 : AD1 selected as input.
 *        - ADC_INPUTCHAN_AD2 : AD2 selected as input.
 *        - ADC_INPUTCHAN_AD3 : AD3 selected as input.
 *        - ADC_INPUTCHAN_AD4 : AD4 selected as input.
 *        - ADC_INPUTCHAN_AD5 : AD5 selected as input.
 *        - ADC_INPUTCHAN_AD6 : AD6 selected as input.
 *        - ADC_INPUTCHAN_AD7 : AD7 selected as input.
 *        - ADC_INPUTCHAN_AD8 : AD8 selected as input.
 *        - ADC_INPUTCHAN_AD9 : AD9 selected as input.
 *        - ADC_INPUTCHAN_AD10 : AD10 selected as input.
 *        - ADC_INPUTCHAN_AD11 : AD11 selected as input.
 *        - ADC_INPUTCHAN_AD12 : AD12 selected as input.
 *        - ADC_INPUTCHAN_AD13 : AD13 selected as input.
 *        - ADC_INPUTCHAN_AD14 : AD14 selected as input.
 *        - ADC_INPUTCHAN_AD15 : AD15 selected as input.
 *        - ADC_INPUTCHAN_TEMP : Temp Sensor selected as input.
 *        - ADC_INPUTCHAN_BANDGAP : Band Gap selected as input.
 *        - ADC_INPUTCHAN_VREFSH : VREFSH selected as input.
 *        - ADC_INPUTCHAN_VREFSL : VREFSL selected as input.
 *        - ADC_INPUTCHAN_DISABLED : Channel Disabled.
 *
 * Implements : ADC_HAL_SetInputChannel_Activity
 */
static inline void ADC_HAL_SetInputChannel(ADC_Type* const baseAddr, const uint8_t chanIndex, const adc_inputchannel_t inputChan)
{
    uint32_t tmp = baseAddr->SC1[chanIndex];
    tmp &= ~(ADC_SC1_ADCH_MASK);
    tmp |= ADC_SC1_ADCH(inputChan);
    baseAddr->SC1[chanIndex] = tmp;
}

/*!
 * @brief Gets the measurement channel Conversion Complete Flag state
 *
 * This function returns the state of the Conversion Complete
 * Flag for a measurement channel. This flag is set when a conversion
 * is complete or the the condition generated by the Hardware
 * Compare feature is evaluated to true.
 *
 * @param[in] baseAddr adc base pointer
 * @param[in] chanIndex the adc measurement channel index
 * @return the Conversion Complete Flag state
 *
 * Implements : ADC_HAL_GetConvCompleteFlag_Activity
 */
static inline bool ADC_HAL_GetConvCompleteFlag(const ADC_Type* const baseAddr, const uint8_t chanIndex)
{
    uint32_t tmp = baseAddr->SC1[chanIndex];
    tmp = (tmp & ADC_SC1_COCO_MASK) >> ADC_SC1_COCO_SHIFT;
    return (tmp != 0u) ? true : false;
}

/*!
 * @brief Gets the conversion result for the selected measurement channel
 *
 * This function returns the conversion result from a
 * measurement channel. This automatically clears the
 * Conversion Complete Flag (CoCo flag) for that channel.
 *
 * @param[in] baseAddr adc base pointer
 * @param[in] chanIndex the adc measurement channel index
 * @return the Measurement Channel Conversion Result
 *
 * Implements : ADC_HAL_GetChanResult_Activity
 */
static inline uint16_t ADC_HAL_GetChanResult(const ADC_Type* const baseAddr, const uint8_t chanIndex)
{
    uint32_t tmp = baseAddr->R[chanIndex];
    tmp = (tmp & ADC_R_D_MASK) >> ADC_R_D_SHIFT;
    return (uint16_t)tmp;
}

/*! @}*/

/*!
 * @name Trigger latches.
 * Functions using the trigger latch mechanism.
 */
/*! @{*/

/*!
 * @brief Clear the latched triggers
 *
 * This function clears the latched triggers, except for the one under process.
 * Before calling this function, make sure the hardware trigger source of the ADC is disabled.
 *
 * @param[in] baseAddr adc base pointer
 *
 * Implements : ADC_HAL_ClearLatchTriggers_Activity
 */
static inline void ADC_HAL_ClearLatchTriggers(ADC_Type* const baseAddr)
{
    baseAddr->CFG1 |= ADC_CFG1_CLRLTRG(0x01u);
}

/*!
 * @brief Get the trigger latch error flags
 *
 * This function returns the latch trigger error flags
 *
 * @param[in] baseAddr adc base pointer
 * @return The trigger latch error flags
 *
 * Implements : ADC_HAL_GetTriggerErrorFlags_Activity
 */
static inline uint32_t ADC_HAL_GetTriggerErrorFlags(const ADC_Type* const baseAddr)
{
    uint32_t tmp = baseAddr->SC2;
    tmp = (tmp & ADC_SC2_TRGSTERR_MASK) >> ADC_SC2_TRGSTERR_SHIFT;
    return tmp;

}

/*!
 * @brief Clear the latch trigger error flags
 *
 * This function clears the trigger latch error flags.
 *
 * @param[in] baseAddr adc base pointer
 *
 * Implements : ADC_HAL_ClearTriggerErrorFlags_Activity
 */
static inline void ADC_HAL_ClearTriggerErrorFlags(ADC_Type* const baseAddr)
{
    baseAddr->SC2 |= ADC_SC2_TRGSTERR(0x0Fu);
}

/*!
 * @brief Get the trigger latch flags bits.
 *
 / This function returns the trigger latch flags.
 *
 * @param[in] baseAddr adc base pointer
 * @return trigger latch flags
 *
 * Implements : ADC_HAL_GetTriggerLatchFlags_Activity
 */
static inline uint32_t ADC_HAL_GetTriggerLatchFlags(const ADC_Type* const baseAddr)
{
    uint32_t tmp = baseAddr->SC2;
    tmp = (tmp & ADC_SC2_TRGSTLAT_MASK) >> ADC_SC2_TRGSTLAT_SHIFT;
    return tmp;
}

/*!
 * @brief Get the index of the trigger under process.
 *
 * This function returns the index of the trigger currently being processed.
 *
 * @param[in] baseAddr adc base pointer
 * @return the trigger index
 *
 * Implements : ADC_HAL_GetTriggerProcNumber_Activity
 */
static inline uint32_t ADC_HAL_GetTriggerProcNumber(const ADC_Type* const baseAddr)
{
    uint32_t tmp = baseAddr->SC2;
    tmp = (tmp & ADC_SC2_TRGPRNUM_MASK) >> ADC_SC2_TRGPRNUM_SHIFT;
    return tmp;
}

/*! @}*/

#if defined(__cplusplus)
}
#endif

/*! @}*/

#endif /* FSL_ADC_HAL_H*/
/*******************************************************************************
 * EOF
 ******************************************************************************/
