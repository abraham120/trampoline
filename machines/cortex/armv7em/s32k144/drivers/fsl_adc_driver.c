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


/*!
 * @file fsl_adc_driver.c
 *
 * @page misra_violations MISRA-C:2012 violations
 *
 * @section [global]
 * Violates MISRA 2012 Advisory Rule 11.4, Conversion between a pointer and integer type.
 * The cast is required to initialize a pointer with an unsigned long define, representing an address.
 *
 * @section [global]
 * Violates MISRA 2012 Required Rule 11.6, Cast from unsigned int to pointer.
 * The cast is required to initialize a pointer with an unsigned long define, representing an address.
 *
 * @section [global]
 * Violates MISRA 2012 Required Rule 11.3, Cast performed between a pointer to object type and a pointer to a different
 * object type.
 * Base address and offset are used to compute register address.
 *
 * @section [global]
 * Violates MISRA 2012 Advisory Rule 18.4, Pointer arithmetic other than array indexing used.
 * Base address and offset are used to compute register address.
 *
 * @section [global]
 * Violates MISRA 2012 Advisory Rule 8.7, External could be made static.
 * Function is defined for usage by application code.
 *
 */

#include <stddef.h>
#include "fsl_adc_driver.h"
#include "fsl_adc_hal.h"

/*******************************************************************************
 * Variables
 ******************************************************************************/

/* Table of base addresses for ADC instances. */
static ADC_Type * const s_adcBase[ADC_INSTANCE_COUNT] = ADC_BASE_PTRS;

/*FUNCTION**********************************************************************
 *
 * Function Name : ADC_DRV_InitConverterStruct
 * Description   : This function initializes the members of the adc_converter_config_t
 * structure to default values (Reference Manual resets). This function should be called
 * on a structure before using it to configure the converter (ADC_DRV_ConfigConverter), otherwise all members
 * must be written (initialized) by the caller. This function insures that all members are written
 * with safe values, so the user can modify only the desired members.
 *
 * Implements : ADC_DRV_InitConverterStruct_Activity
 *END**************************************************************************/
void ADC_DRV_InitConverterStruct(adc_converter_config_t* const config)
{
    #ifdef DEV_ERROR_DETECT
    DEV_ASSERT(config != NULL);
    #endif

    config->clockDivide = ADC_CLK_DIVIDE_1;
    config->sampleTime = (uint8_t)0x0CU;
    config->resolution = ADC_RESOLUTION_8BIT;
    config->inputClock = ADC_CLK_ALT_1;
    config->trigger = ADC_TRIGGER_SOFTWARE;
    config->dmaEnable = false;
    config->voltageRef = ADC_VOLTAGEREF_VREF;
    config->continuousConvEnable = false;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : ADC_DRV_ConfigConverter
 * Description   : This function configures the ADC converter with the options
 * provided in the configuration structure.
 *
 * Implements : ADC_DRV_ConfigConverter_Activity
 *END**************************************************************************/
void ADC_DRV_ConfigConverter(const uint32_t instance, const adc_converter_config_t* const config)
{
    #ifdef DEV_ERROR_DETECT
    DEV_ASSERT(instance < ADC_INSTANCE_COUNT);
    DEV_ASSERT(config != NULL);
    #endif

    ADC_Type* const base = s_adcBase[instance];
    ADC_HAL_SetClockDivide(base, config->clockDivide);
    ADC_HAL_SetSampleTime(base, config->sampleTime);
    ADC_HAL_SetResolution(base, config->resolution);
    ADC_HAL_SetInputClock(base, config->inputClock);
    ADC_HAL_SetTriggerMode(base, config->trigger);
    ADC_HAL_SetDMAEnableFlag(base, config->dmaEnable);
    ADC_HAL_SetVoltageReference(base, config->voltageRef);
    ADC_HAL_SetContinuousConvFlag(base, config->continuousConvEnable);
}

/*FUNCTION**********************************************************************
 *
 * Function Name : ADC_DRV_GetConverterConfig
 * Description   : This functions returns the current converter configuration in
 * the form of a configuration structure.
 *
 * Implements : ADC_DRV_GetConverterConfig_Activity
 *END**************************************************************************/
void ADC_DRV_GetConverterConfig(const uint32_t instance, adc_converter_config_t* const config)
{
    #ifdef DEV_ERROR_DETECT
    DEV_ASSERT(instance < ADC_INSTANCE_COUNT);
    DEV_ASSERT(config != NULL);
    #endif

    const ADC_Type* const base = s_adcBase[instance];
    config->clockDivide = ADC_HAL_GetClockDivide(base);
    config->sampleTime = ADC_HAL_GetSampleTime(base);
    config->resolution = ADC_HAL_GetResolution(base);
    config->inputClock = ADC_HAL_GetInputClock(base);
    config->trigger = ADC_HAL_GetTriggerMode(base);
    config->dmaEnable = ADC_HAL_GetDMAEnableFlag(base);
    config->voltageRef = ADC_HAL_GetVoltageReference(base);
    config->continuousConvEnable = ADC_HAL_GetContinuousConvFlag(base);
}

/*FUNCTION**********************************************************************
 *
 * Function Name : ADC_DRV_Reset
 * Description   : This function writes all the internal ADC registers with
 * their Reference Manual reset values.
 *
 * Implements : ADC_DRV_Reset_Activity
 *END**************************************************************************/
void ADC_DRV_Reset(const uint32_t instance)
{
    #ifdef DEV_ERROR_DETECT
    DEV_ASSERT(instance < ADC_INSTANCE_COUNT);
    #endif

    ADC_Type* const base = s_adcBase[instance];
    ADC_HAL_Init(base);
}

/*FUNCTION**********************************************************************
 *
 * Function Name : ADC_DRV_InitHwCompareStruct
 * Description   : This function initializes the Hardware Compare configuration
 * structure to default values (Reference Manual resets). This function should be
 * called before configuring the Hardware Compare feature (ADC_DRV_ConfigHwCompare),
 * otherwise all members must be written by the caller. This function insures that all
 * members are written with safe values, so the user can modify the desired members.
 *
 * Implements : ADC_DRV_InitHwCompareStruct_Activity
 *END**************************************************************************/
void ADC_DRV_InitHwCompareStruct(adc_compare_config_t* const config)
{
    #ifdef DEV_ERROR_DETECT
    DEV_ASSERT(config != NULL);
    #endif

    config->compareEnable = false;
    config->compareGreaterThanEnable = false;
    config->compareRangeFuncEnable = false;
    config->compVal1 = 0U;
    config->compVal2 = 0U;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : ADC_DRV_ConfigHwCompare
 * Description   : This functions sets the configuration for the Hardware
 * Compare feature using the configuration structure.
 *
 * Implements : ADC_DRV_ConfigHwCompare_Activity
 *END**************************************************************************/
void ADC_DRV_ConfigHwCompare(const uint32_t instance, const adc_compare_config_t* const config)
{
    #ifdef DEV_ERROR_DETECT
    DEV_ASSERT(instance < ADC_INSTANCE_COUNT);
    DEV_ASSERT(config != NULL);
    #endif

    ADC_Type* const base = s_adcBase[instance];
    ADC_HAL_SetHwCompareEnableFlag(base, config->compareEnable);
    ADC_HAL_SetHwCompareGtEnableFlag(base, config->compareGreaterThanEnable);
    ADC_HAL_SetHwCompareRangeEnableFlag(base, config->compareRangeFuncEnable);
    ADC_HAL_SetHwCompareComp1Value(base, config->compVal1);
    ADC_HAL_SetHwCompareComp2Value(base, config->compVal2);
}

/*FUNCTION**********************************************************************
 *
 * Function Name : ADC_DRV_GetHwCompareConfig
 * Description   : This function returns the configuration for the Hardware
 * Compare feature.
 *
 * Implements : ADC_DRV_GetHwCompareConfig_Activity
 *END**************************************************************************/
void ADC_DRV_GetHwCompareConfig(const uint32_t instance, adc_compare_config_t* const config)
{
    #ifdef DEV_ERROR_DETECT
    DEV_ASSERT(instance < ADC_INSTANCE_COUNT);
    DEV_ASSERT(config != NULL);
    #endif

    const ADC_Type* const base = s_adcBase[instance];
    config->compareEnable = ADC_HAL_GetHwCompareEnableFlag(base);
    config->compareGreaterThanEnable = ADC_HAL_GetHwCompareGtEnableFlag(base);
    config->compareRangeFuncEnable = ADC_HAL_GetHwCompareRangeEnableFlag(base);
    config->compVal1 = ADC_HAL_GetHwCompareComp1Value(base);
    config->compVal2 = ADC_HAL_GetHwCompareComp2Value(base);
}

/*FUNCTION**********************************************************************
 *
 * Function Name : ADC_DRV_InitHwAverageStruct
 * Description   : This function initializes the Hardware Average configuration
 * structure to default values (Reference Manual resets). This function should be
 * called before configuring the Hardware Average feature (ADC_DRV_ConfigHwAverage),
 * otherwise all members must be written by the caller. This function insures that all
 * members are written with safe values, so the user can modify the desired members.
 *
 * Implements : ADC_DRV_InitHwAverageStruct_Activity
 *END**************************************************************************/
void ADC_DRV_InitHwAverageStruct(adc_average_config_t* const config)
{
    #ifdef DEV_ERROR_DETECT
    DEV_ASSERT(config != NULL);
    #endif

    config->hwAvgEnable = false;
    config->hwAverage = ADC_AVERAGE_4;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : ADC_DRV_ConfigHwAverage
 * Description   : This function sets the configuration for the Hardware
 * Average feature.
 *
 * Implements : ADC_DRV_ConfigHwAverage_Activity
 *END**************************************************************************/
void ADC_DRV_ConfigHwAverage(const uint32_t instance, const adc_average_config_t* const config)
{
    #ifdef DEV_ERROR_DETECT
    DEV_ASSERT(instance < ADC_INSTANCE_COUNT);
    DEV_ASSERT(config != NULL);
    #endif

    ADC_Type* const base = s_adcBase[instance];
    ADC_HAL_SetHwAverageEnableFlag(base, config->hwAvgEnable);
    ADC_HAL_SetHwAverageMode(base, config->hwAverage);
}

/*FUNCTION**********************************************************************
 *
 * Function Name : ADC_DRV_GetHwAverageConfig
 * Description   : This function returns the configuration for the Hardware
 * Average feature.
 *
 * Implements : ADC_DRV_GetHwAverageConfig_Activity
 *END**************************************************************************/
void ADC_DRV_GetHwAverageConfig(const uint32_t instance, adc_average_config_t* const config)
{
    #ifdef DEV_ERROR_DETECT
    DEV_ASSERT(instance < ADC_INSTANCE_COUNT);
    DEV_ASSERT(config != NULL);
    #endif

    const ADC_Type* const base = s_adcBase[instance];
    config->hwAvgEnable = ADC_HAL_GetHwAverageEnableFlag(base);
    config->hwAverage = ADC_HAL_GetHwAverageMode(base);
}

/*FUNCTION**********************************************************************
 *
 * Function Name : ADC_DRV_InitChanStruct
 * Description   : This function initializes the measurement channel
 * configuration structure to default values (Reference Manual resets). This function should
 * be called on a structure before using it to configure a channel (ADC_DRV_ConfigChan), otherwise
 * all members must be written by the caller. This function insures that all members are written
 * with safe values, so the user can modify only the desired members.
 *
 * Implements : ADC_DRV_InitChanStruct_Activity
 *END**************************************************************************/
void ADC_DRV_InitChanStruct(adc_chan_config_t* const config)
{
    #ifdef DEV_ERROR_DETECT
    DEV_ASSERT(config != NULL);
    #endif

    config->interruptEnable = false;
    config->channel = ADC_INPUTCHAN_DISABLED;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : ADC_DRV_ConfigChan
 * Description   : This function sets a measurement channel configuration.
 *
 * Implements : ADC_DRV_ConfigChan_Activity
 *END**************************************************************************/
void ADC_DRV_ConfigChan(const uint32_t instance, const uint8_t chanIndex, const adc_chan_config_t* const config)
{
    #ifdef DEV_ERROR_DETECT
    DEV_ASSERT(instance < ADC_INSTANCE_COUNT);
    DEV_ASSERT(chanIndex < ADC_SC1_COUNT);
    DEV_ASSERT(config != NULL);
    #endif

    ADC_Type* const base = s_adcBase[instance];
    ADC_HAL_SetChanInterruptEnableFlag(base, chanIndex, config->interruptEnable);
    ADC_HAL_SetInputChannel(base, chanIndex, config->channel);
}

/*FUNCTION**********************************************************************
 *
 * Function Name : ADC_DRV_GetChanConfig
 * Description   : This function returns the current configuration for a measurement
 * channel.
 *
 * Implements : ADC_DRV_GetChanConfig_Activity
 *END**************************************************************************/
void ADC_DRV_GetChanConfig(const uint32_t instance, const uint8_t chanIndex, adc_chan_config_t* const config)
{
    #ifdef DEV_ERROR_DETECT
    DEV_ASSERT(instance < ADC_INSTANCE_COUNT);
    DEV_ASSERT(chanIndex < ADC_SC1_COUNT);
    DEV_ASSERT(config != NULL);
    #endif

    const ADC_Type* const base = s_adcBase[instance];
    config->interruptEnable = ADC_HAL_GetChanInterruptEnableFlag(base, chanIndex);
    config->channel = ADC_HAL_GetInputChannel(base, chanIndex);
}

/*FUNCTION**********************************************************************
 *
 * Function Name : ADC_DRV_WaitConvDone
 * Description   : This functions waits for a conversion to complete by
 * continuously polling the Conversion Active Flag.
 *
 * Implements : ADC_DRV_WaitConvDone_Activity
 *END**************************************************************************/
void ADC_DRV_WaitConvDone(const uint32_t instance)
{
    #ifdef DEV_ERROR_DETECT
    DEV_ASSERT(instance < ADC_INSTANCE_COUNT);
    #endif

    const ADC_Type* const base = s_adcBase[instance];
    while(ADC_HAL_GetConvActiveFlag(base) == true)
    {
        /* Wait for conversion to finish */
    }
}

/*FUNCTION**********************************************************************
 *
 * Function Name : ADC_DRV_GetChanResult
 * Description   : This function returns the conversion result from a
 * measurement channel.
 *
 * Implements : ADC_DRV_GetChanResult_Activity
 *END**************************************************************************/
void ADC_DRV_GetChanResult(const uint32_t instance, const uint8_t chanIndex, uint16_t* const result)
{
    #ifdef DEV_ERROR_DETECT
    DEV_ASSERT(instance < ADC_INSTANCE_COUNT);
    DEV_ASSERT(chanIndex < ADC_R_COUNT);
    DEV_ASSERT(result != NULL);
    #endif

    const ADC_Type* const base = s_adcBase[instance];
    *result = ADC_HAL_GetChanResult(base, chanIndex);
}

/*FUNCTION**********************************************************************
 *
 * Function Name : ADC_DRV_AutoCalibration
 * Description   : This functions executes an Auto-Calibration sequence. It
 * is recommended to run this sequence before using the ADC converter.
 *
 * Implements : ADC_DRV_AutoCalibration_Activity
 *END**************************************************************************/
void ADC_DRV_AutoCalibration(const uint32_t instance)
{
    #ifdef DEV_ERROR_DETECT
    DEV_ASSERT(instance < ADC_INSTANCE_COUNT);
    #endif

    ADC_Type* const base = s_adcBase[instance];
    /* set hardware average to maximum and set software trigger*/
    bool hwavgen = ADC_HAL_GetHwAverageEnableFlag(base);
    adc_average_t hwavg = ADC_HAL_GetHwAverageMode(base);
    adc_trigger_t trig = ADC_HAL_GetTriggerMode(base);
    ADC_HAL_SetHwAverageMode(base, ADC_AVERAGE_32);
    ADC_HAL_SetHwAverageEnableFlag(base, true);
    ADC_HAL_SetTriggerMode(base, ADC_TRIGGER_SOFTWARE);
    
    base->CLPS = 0x00u;
    base->CLP3 = 0x00u;
    base->CLP2 = 0x00u;
    base->CLP1 = 0x00u;
    base->CLP0 = 0x00u;
    base->CLPX = 0x00u;
    base->CLP9 = 0x00u;
    
    /* start calibration */
    ADC_HAL_SetCalibrationActiveFlag(base, true);
    while(ADC_HAL_GetCalibrationActiveFlag(base))
    {
        /* Wait for calibration to finish */
    }

    /* restore hardware average and trigger settings*/
    ADC_HAL_SetHwAverageEnableFlag(base, hwavgen);
    ADC_HAL_SetHwAverageMode(base, hwavg);
    ADC_HAL_SetTriggerMode(base, trig);
}

/*FUNCTION**********************************************************************
 *
 * Function Name : ADC_DRV_InitUserCalibrationStruct
 * Description   : This function initializes the User Calibration configuration
 * structure to default values (Reference Manual resets). This function should be called
 * on a structure before using it to configure the User Calibration feature (ADC_DRV_ConfigUserCalibration),
 * otherwise all members must be written by the caller. This function insures that all members are written
 * with safe values, so the user can modify only the desired members.
 *
 * Implements : ADC_DRV_InitUserCalibrationStruct_Activity
 *END**************************************************************************/
void ADC_DRV_InitUserCalibrationStruct(adc_calibration_t* const config)
{
    #ifdef DEV_ERROR_DETECT
    DEV_ASSERT(config != NULL);
    #endif

    config->userGain = (uint16_t)4U;
    config->userOffset = (uint16_t)0U;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : ADC_DRV_ConfigUserCalibration
 * Description   : This function sets the configuration for the user calibration
 * registers.
 *
 * Implements : ADC_DRV_ConfigUserCalibration_Activity
 *END**************************************************************************/
void ADC_DRV_ConfigUserCalibration(const uint32_t instance, const adc_calibration_t* const config)
{
    #ifdef DEV_ERROR_DETECT
    DEV_ASSERT(instance < ADC_INSTANCE_COUNT);
    DEV_ASSERT(config != NULL);
    #endif

    ADC_Type* const base = s_adcBase[instance];
    ADC_HAL_SetUserGainValue(base, config->userGain);
    ADC_HAL_SetUserOffsetValue(base, config->userOffset);
}

/*FUNCTION**********************************************************************
 *
 * Function Name : ADC_DRV_GetUserCalibration
 * Description   : This function returns the current user calibration
 * register values.
 *
 * Implements : ADC_DRV_GetUserCalibration_Activity
 *END**************************************************************************/
void ADC_DRV_GetUserCalibration(const uint32_t instance, adc_calibration_t* const config)
{
    #ifdef DEV_ERROR_DETECT
    DEV_ASSERT(instance < ADC_INSTANCE_COUNT);
    DEV_ASSERT(config != NULL);
    #endif

    const ADC_Type* const base = s_adcBase[instance];
    config->userGain = ADC_HAL_GetUserGainValue(base);
    config->userOffset = ADC_HAL_GetUserOffsetValue(base);
}

/*FUNCTION**********************************************************************
 *
 * Function Name : ADC_DRV_GetInterruptNumber
 * Description   : This function returns the interrupt number for the specified ADC instance.
 *
 * Implements : ADC_DRV_GetInterruptNumber_Activity
 *END**************************************************************************/
IRQn_Type ADC_DRV_GetInterruptNumber(const uint32_t instance)
{
    #ifdef DEV_ERROR_DETECT
    DEV_ASSERT(instance < ADC_INSTANCE_COUNT);
    #endif

    static const IRQn_Type adcIrqId[ADC_INSTANCE_COUNT] = ADC_IRQS;
    IRQn_Type irqId = adcIrqId[instance];
    return irqId;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : ADC_DRV_ClearLatchedTriggers
 * Description   : This function returns the interrupt number for the specified ADC instance.
 * This function must be called after the hardware trigger source for the ADC has been deactivated.
 *
 * Implements : ADC_DRV_ClearLatchedTriggers_Activity
 *END**************************************************************************/
void ADC_DRV_ClearLatchedTriggers(const uint32_t instance, const adc_latch_clear_t clearMode)
{
    #ifdef DEV_ERROR_DETECT
    DEV_ASSERT(instance < ADC_INSTANCE_COUNT);
    DEV_ASSERT((clearMode == ADC_LATCH_CLEAR_WAIT) || (clearMode == ADC_LATCH_CLEAR_FORCE));
    #endif

    ADC_Type* const base = s_adcBase[instance];
    if (clearMode == ADC_LATCH_CLEAR_FORCE)
    {
        ADC_HAL_ClearLatchTriggers(base);
    }
    while(ADC_HAL_GetTriggerLatchFlags(base) != 0u)
    {
        /* Wait for latched triggers to be processed */
    }

}

/*FUNCTION**********************************************************************
 *
 * Function Name : ADC_DRV_ClearTriggerErrors
 * Description   : This function clears all trigger error flags of the ADC instance.
 *
 * Implements : ADC_DRV_ClearTriggerErrors_Activity
 *END**************************************************************************/
void ADC_DRV_ClearTriggerErrors(const uint32_t instance)
{
    #ifdef DEV_ERROR_DETECT
    DEV_ASSERT(instance < ADC_INSTANCE_COUNT);
    #endif

    ADC_Type* const base = s_adcBase[instance];
    ADC_HAL_ClearTriggerErrorFlags(base);
}

/*FUNCTION**********************************************************************
 *
 * Function Name : ADC_DRV_GetTriggerErrorFlags
 * Description   : This function returns the trigger error flags bits of the ADC instance.
 *
 * Implements : ADC_DRV_GetTriggerErrorFlags_Activity
 *END**************************************************************************/
uint32_t ADC_DRV_GetTriggerErrorFlags(const uint32_t instance)
{
    #ifdef DEV_ERROR_DETECT
    DEV_ASSERT(instance < ADC_INSTANCE_COUNT);
    #endif

    const ADC_Type* const base = s_adcBase[instance];
    uint32_t trig_errors = ADC_HAL_GetTriggerErrorFlags(base);
    return trig_errors;
}

/******************************************************************************
 * EOF
 *****************************************************************************/
