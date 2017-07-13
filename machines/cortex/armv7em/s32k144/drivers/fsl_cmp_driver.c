/*
 * Copyright (c) 2014 - 2016, Freescale Semiconductor, Inc.
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
 /**
* @page misra_violations MISRA-C:2012 violations
*
* @section [global]
* Violates MISRA 2012 Advisory Rule 11.4, Conversion between a pointer and integer type.
* The cast is required to initialize a pointer with an unsigned long define, representing an address. 
*
* @section [global]
* Violates MISRA 2012 Required Rule 11.6, Cast from unsigned int to pointer.
* The cast is required to initialize a pointer with an unsigned long define, representing an address. 
*/
#include "fsl_cmp_driver.h"
#include <stddef.h>

/*******************************************************************************
 * Variables
 ******************************************************************************/
/* Table of base addresses for CMP instances. */
static CMP_Type * const g_cmpBase[] = CMP_BASE_PTRS;
/*FUNCTION**********************************************************************
 *
 * Function Name : CMP_DRV_Reset
 * Description   : This function set all CMP registers to reset values.
 *
 * Implements : CMP_DRV_Reset_Activity
 *END**************************************************************************/
cmp_drv_status_t CMP_DRV_Reset(const uint32_t instance)
{
    cmp_drv_status_t  status = CMP_DRV_SUCCESS;
    #ifdef DEV_ERROR_DETECT
    DEV_ASSERT(instance < CMP_INSTANCE_COUNT);
    #endif
    CMP_Type* base = g_cmpBase[instance];
    CMP_HAL_Init(base);
    return status;
}
/*FUNCTION**********************************************************************
 *
 * Function Name : CMP_DRV_GetInitConfigAll
 * Description   : Return a configuration structure with reset values for all components
 * from comparator module.
 * Implements : CMP_DRV_GetInitConfigAll_Activity
 *
 *END**************************************************************************/
void CMP_DRV_GetInitConfigAll(cmp_module_t* config)
{
    #ifdef DEV_ERROR_DETECT
    DEV_ASSERT(config != NULL);
    #endif
    (config->comparator).outputInterruptTrigger = CMP_NO_EVENT;
    (config->comparator).mode = CMP_DISABLED; 
    (config->comparator).filterSampleCount = 0U;
    (config->comparator).filterSamplePeriod = 0U;
    (config->comparator).powerMode = CMP_LOW_SPEED;
    (config->comparator).inverterState = CMP_NORMAL;
    (config->comparator).outputSelect = CMP_COUT;
    (config->comparator).pinState = CMP_UNAVAILABLE; 
    (config->comparator).offsetLevel = CMP_LEVEL_OFFSET_0;
    (config->comparator).hysteresisLevel = CMP_LEVEL_HYS_0;
    (config->dac).state = false;
    (config->dac).voltageReferenceSource = CMP_VIN1;
    (config->dac).voltage = 0U;
    (config->mux).negativePortMux = CMP_DAC;
    (config->mux).positivePortMux = CMP_DAC;
    (config->mux).negativeInputMux = 0U;
    (config->mux).positiveInputMux = 0U;
    (config->triggerMode).roundRobinState = false;
    (config->triggerMode).roundRobinInterruptState = false; 
    (config->triggerMode).fixedPort = CMP_PLUS_FIXED;
    (config->triggerMode).fixedChannel = 0U;
    (config->triggerMode).samples = 0U;
    (config->triggerMode).roundRobinChannelsState = 0U;
    (config->triggerMode).programedState = 0U;
    (config->triggerMode).initializationDelay = 0U;
}
/*FUNCTION**********************************************************************
 *
 * Function Name : CMP_DRV_Init
 * Description   : Configure all components from comparator module.
 *
 * Implements : CMP_DRV_Init_Activity 
 *END**************************************************************************/
cmp_drv_status_t CMP_DRV_Init(const uint32_t instance, const cmp_module_t* const config)
{
    cmp_drv_status_t  status = CMP_DRV_SUCCESS;
    #ifdef DEV_ERROR_DETECT
    DEV_ASSERT(config != NULL);
    DEV_ASSERT(instance < CMP_INSTANCE_COUNT);
    #endif
    CMP_Type* base = g_cmpBase[instance];
    CMP_HAL_SetDMATriggerState(base, (config->comparator).dmaTriggerState );
    CMP_HAL_SetOutputInterruptTrigger(base, (config->comparator).outputInterruptTrigger);
    CMP_HAL_SetFunctionalMode(base, (config->comparator).mode, (config->comparator).filterSampleCount, (config->comparator).filterSamplePeriod);
    CMP_HAL_SetPowerMode(base, (config->comparator).powerMode);
    CMP_HAL_SetInverterState(base, (config->comparator).inverterState);
    CMP_HAL_SetComparatorOutputSource(base, (config->comparator).outputSelect);
    CMP_HAL_SetOutputPinState(base, (config->comparator).pinState);
    CMP_HAL_SetHysteresis(base, (config->comparator).hysteresisLevel);
    CMP_HAL_SetOffset(base, (config->comparator).offsetLevel);
    CMP_HAL_SetDACState(base, (config->dac).state);
    CMP_HAL_SetVoltageReference(base, (config->dac).voltageReferenceSource);
    CMP_HAL_SetVoltage(base, (config->dac).voltage);
    CMP_HAL_SetNegativePortInput(base, (config->mux).negativePortMux);
    CMP_HAL_SetPositivePortInput(base, (config->mux).positivePortMux);
    CMP_HAL_SetMinusMUXControl(base, (config->mux).negativeInputMux);
    CMP_HAL_SetPlusMuxControl(base, (config->mux).positiveInputMux);
    CMP_HAL_SetFixedPort(base, (config->triggerMode).fixedPort);
    CMP_HAL_SetFixedChannel(base, (config->triggerMode).fixedChannel);
    CMP_HAL_SetRoundRobinSamplesNumber(base, (config->triggerMode).samples);
    CMP_HAL_SetInitDelay(base, (config->triggerMode).initializationDelay);
    CMP_HAL_SetRoundRobinChannels(base, (config->triggerMode).roundRobinChannelsState);
    CMP_HAL_SetPresetState(base, (config->triggerMode).programedState);
    CMP_HAL_SetRoundRobinInterruptState(base, (config->triggerMode).roundRobinInterruptState);
    CMP_HAL_SetRoundRobinState(base, (config->triggerMode).roundRobinState);
    /* Clear all flags*/
    CMP_HAL_ClearOutputEvent(base);
    CMP_HAL_ClearInputChangedFlags(base);
    return status;
}
/*FUNCTION**********************************************************************
 *
 * Function Name : CMP_DRV_GetConfigAll
 * Description   : This function returns the configuration for all components 
 * from comparator module.
 * Implements : CMP_DRV_GetConfigAll_Activity  
 *
 *END**************************************************************************/
cmp_drv_status_t CMP_DRV_GetConfigAll(const uint32_t instance, cmp_module_t* const config)
{
    cmp_drv_status_t  status = CMP_DRV_SUCCESS;
    #ifdef DEV_ERROR_DETECT
    DEV_ASSERT(config != NULL);
    DEV_ASSERT(instance < CMP_INSTANCE_COUNT);
    #endif
    const CMP_Type* base = g_cmpBase[instance];
    (config->comparator).dmaTriggerState = CMP_HAL_GetDMATriggerState(base);
    (config->comparator).outputInterruptTrigger = CMP_HAL_GetOutputInterruptTrigger(base);
    (config->comparator).mode = CMP_HAL_GetFunctionalMode(base);
    (config->comparator).filterSampleCount = CMP_HAL_GetFilterSampleCount(base);
    (config->comparator).filterSamplePeriod = CMP_HAL_GetFilterSamplePeriod(base);
    (config->comparator).powerMode = CMP_HAL_GetPowerMode(base);
    (config->comparator).inverterState = CMP_HAL_GetInverterState(base);
    (config->comparator).outputSelect = CMP_HAL_GetComparatorOutputSource(base);
    (config->comparator).pinState = CMP_HAL_GetOutputPinState(base);
    (config->comparator).offsetLevel = CMP_HAL_GetOffset(base);
    (config->comparator).hysteresisLevel = CMP_HAL_GetHysteresis(base);
    (config->dac).state = CMP_HAL_GetDACState(base);
    (config->dac).voltageReferenceSource = CMP_HAL_GetVoltageReference(base);
    (config->dac).voltage = CMP_HAL_GetVoltage(base);
    (config->mux).negativePortMux = CMP_HAL_GetNegativePortInput(base);
    (config->mux).positivePortMux = CMP_HAL_GetPositivePortInput(base);
    (config->mux).negativeInputMux = CMP_HAL_GetMinusMUXControl(base);
    (config->mux).positiveInputMux = CMP_HAL_GetPlusMUXControl(base);
    (config->triggerMode).roundRobinState = CMP_HAL_GetRoundRobinState(base);
    (config->triggerMode).roundRobinInterruptState = CMP_HAL_GetRoundRobinInterruptState(base);
    (config->triggerMode).fixedPort = CMP_HAL_GetFixedPort(base);
    (config->triggerMode).fixedChannel = CMP_HAL_GetFixedChannel(base);
    (config->triggerMode).samples = CMP_HAL_GetRoundRobinSamplesNumber(base);
    (config->triggerMode).roundRobinChannelsState = CMP_HAL_GetRoundRobinChannels(base);
    (config->triggerMode).programedState = CMP_HAL_GetLastComparisonResult(base);
    (config->triggerMode).initializationDelay = CMP_HAL_GetInitDelay(base);
    return status;
}
/*FUNCTION**********************************************************************
 *
 * Function Name : CMP_DRV_GetInitConfigDAC
 * Description   : Return configuration structure with reset values for DAC 
 * component from comparator module.
 * Implements : CMP_DRV_GetInitConfigDAC_Activity 
 *
 *END**************************************************************************/
void CMP_DRV_GetInitConfigDAC(cmp_dac_t* config)
{
    #ifdef DEV_ERROR_DETECT
    DEV_ASSERT(config != NULL);
    #endif
    config->state = false;
    config->voltageReferenceSource = CMP_VIN1;
    config->voltage = 0U;
}
/*FUNCTION**********************************************************************
 *
 * Function Name : CMP_DRV_ConfigDAC
 * Description   : Configure only DAC component from comparator module.
 * Implements : CMP_DRV_ConfigDAC_Activity  
 *
 *END**************************************************************************/
cmp_drv_status_t CMP_DRV_ConfigDAC(const uint32_t instance, const cmp_dac_t* config)
{
    cmp_drv_status_t  status = CMP_DRV_SUCCESS;
    #ifdef DEV_ERROR_DETECT
    DEV_ASSERT(config != NULL);
    DEV_ASSERT(instance < CMP_INSTANCE_COUNT);
    #endif
    CMP_Type* base = g_cmpBase[instance];
    CMP_HAL_SetDACState(base, config->state);
    CMP_HAL_SetVoltageReference(base, config->voltageReferenceSource);
    CMP_HAL_SetVoltage(base, config->voltage);
    return status;
}
/*FUNCTION**********************************************************************
 *
 * Function Name : CMP_DRV_GetDAC
 * Description   : Return configuration for DAC component from comparator module.
 * Implements : CMP_DRV_GetDACConfig_Activity 
 *
 *END**************************************************************************/
cmp_drv_status_t CMP_DRV_GetDACConfig(const uint32_t instance, cmp_dac_t* const config)
{
    cmp_drv_status_t  status = CMP_DRV_SUCCESS;
    #ifdef DEV_ERROR_DETECT
    DEV_ASSERT(config != NULL);
    DEV_ASSERT(instance < CMP_INSTANCE_COUNT);
    #endif
    const CMP_Type* base = g_cmpBase[instance];
    config->state = CMP_HAL_GetDACState(base);
    config->voltageReferenceSource = CMP_HAL_GetVoltageReference(base);
    config->voltage = CMP_HAL_GetVoltage(base);
    return status;
}
/*FUNCTION**********************************************************************
 *
 * Function Name : CMP_DRV_GetInitConfigMUX
 * Description   : Return configuration structure with reset values for 
 * the MUX component which select source signals for comparator ports.
 * Implements : CMP_DRV_GetInitConfigMUX_Activity
 *
 *END**************************************************************************/
void CMP_DRV_GetInitConfigMUX(cmp_anmux_t* config)
{
#ifdef DEV_ERROR_DETECT
    DEV_ASSERT(config != NULL);
#endif
    config->negativePortMux = CMP_DAC;
    config->positivePortMux = CMP_DAC;
    config->negativeInputMux = 0U;
    config->positiveInputMux = 0U;
}
/*FUNCTION**********************************************************************
 *
 * Function Name : CMP_DRV_ConfigMUX
 * Description   : Configure only MUX component from comparator module to select
 * source signals for comparator ports.
 * Implements : CMP_DRV_ConfigMUX_Activity
 *
 *END**************************************************************************/
cmp_drv_status_t CMP_DRV_ConfigMUX(const uint32_t instance, const cmp_anmux_t* config)
{
    cmp_drv_status_t  status = CMP_DRV_SUCCESS;
    #ifdef DEV_ERROR_DETECT
    DEV_ASSERT(config != NULL);
    DEV_ASSERT(instance < CMP_INSTANCE_COUNT);
    #endif
    CMP_Type* base = g_cmpBase[instance];
    CMP_HAL_SetNegativePortInput(base, config->negativePortMux);
    CMP_HAL_SetPositivePortInput(base, config->positivePortMux);
    CMP_HAL_SetMinusMUXControl(base, config->negativeInputMux);
    CMP_HAL_SetPlusMuxControl(base, config->positiveInputMux);
    return status;
}
/*FUNCTION**********************************************************************
 *
 * Function Name : CMP_DRV_GetMUX
 * Description   : Return configuration for the MUX component which select 
 * source signals for comparator ports.
 * Implements : CMP_DRV_GetMUXConfig_Activity
 *
 *END**************************************************************************/
cmp_drv_status_t CMP_DRV_GetMUXConfig(const uint32_t instance, cmp_anmux_t* const config)
{
    cmp_drv_status_t  status = CMP_DRV_SUCCESS;
    #ifdef DEV_ERROR_DETECT
    DEV_ASSERT(config != NULL);
    DEV_ASSERT(instance < CMP_INSTANCE_COUNT);
    #endif
    const CMP_Type* base = g_cmpBase[instance];
    config->negativePortMux = CMP_HAL_GetNegativePortInput(base);
    config->positivePortMux = CMP_HAL_GetPositivePortInput(base);
    config->negativeInputMux = CMP_HAL_GetMinusMUXControl(base);
    config->positiveInputMux = CMP_HAL_GetPlusMUXControl(base);
    return status;
}
/*FUNCTION**********************************************************************
 *
 * Function Name : CMP_DRV_GetInitTriggerMode
 * Description   : Return configuration structure with reset values for Trigger Mode
 * from comparator module.
 * Implements : CMP_DRV_GetInitTriggerMode_Activity
 *
 *END**************************************************************************/
void CMP_DRV_GetInitTriggerMode(cmp_trigger_mode_t* config)
{
    #ifdef DEV_ERROR_DETECT
    DEV_ASSERT(config != NULL);
    #endif
    config->roundRobinState = false;
    config->roundRobinInterruptState = false; 
    config->fixedPort = CMP_PLUS_FIXED;
    config->fixedChannel = 0U;
    config->samples = 0U;
    config->roundRobinChannelsState = 0U;
    config->programedState = 0U;
    config->initializationDelay = 0U;
}
/*FUNCTION**********************************************************************
 *
 * Function Name : CMP_DRV_ConfigTriggerMode
 * Description   : Configure comparator in trigger mode.
 * Implements : CMP_DRV_ConfigTriggerMode_Activity
 *
 *END**************************************************************************/
cmp_drv_status_t CMP_DRV_ConfigTriggerMode(const uint32_t instance, const cmp_trigger_mode_t* config)
{
    cmp_drv_status_t  status = CMP_DRV_SUCCESS;
    #ifdef DEV_ERROR_DETECT
    DEV_ASSERT(config != NULL);
    DEV_ASSERT(instance < CMP_INSTANCE_COUNT);
    #endif
    CMP_Type* base = g_cmpBase[instance];
    CMP_HAL_SetRoundRobinState(base, config->roundRobinState);
    CMP_HAL_SetRoundRobinInterruptState(base, config->roundRobinInterruptState);
    CMP_HAL_SetFixedPort(base, config->fixedPort);
    CMP_HAL_SetFixedChannel(base, config->fixedChannel);
    CMP_HAL_SetRoundRobinSamplesNumber(base, config->samples);
    CMP_HAL_SetInitDelay(base, config->initializationDelay);
    CMP_HAL_SetRoundRobinChannels(base, config->roundRobinChannelsState);
    CMP_HAL_SetPresetState(base, config->programedState);
    return status;
}
/*FUNCTION**********************************************************************
 *
 * Function Name : CMP_DRV_GetTriggerMode
 * Description   : Return configuration for the trigger mode.
 * Implements : CMP_DRV_GetTriggerModeConfig_Activity
 *
 *END**************************************************************************/
cmp_drv_status_t CMP_DRV_GetTriggerModeConfig(const uint32_t instance, cmp_trigger_mode_t* const config)
{
    cmp_drv_status_t  status = CMP_DRV_SUCCESS;
    #ifdef DEV_ERROR_DETECT
    DEV_ASSERT(config != NULL);
    DEV_ASSERT(instance < CMP_INSTANCE_COUNT);
    #endif
    const CMP_Type* base = g_cmpBase[instance];
    config->roundRobinState = CMP_HAL_GetRoundRobinState(base);
    config->roundRobinInterruptState = CMP_HAL_GetRoundRobinInterruptState(base);
    config->fixedPort = CMP_HAL_GetFixedPort(base);
    config->fixedChannel = CMP_HAL_GetFixedChannel(base);
    config->samples = CMP_HAL_GetRoundRobinSamplesNumber(base);
    config->roundRobinChannelsState = CMP_HAL_GetRoundRobinChannels(base);
    config->programedState = CMP_HAL_GetLastComparisonResult(base);
    config->initializationDelay =  CMP_HAL_GetInitDelay(base);
    return status;
}
/*FUNCTION**********************************************************************
 *
 * Function Name : CMP_DRV_GetOutputFlags
 * Description   : Return in <flags> comparator output flags(rising and falling edge on output).
 * Implements : CMP_DRV_GetOutputFlags_Activity
 *
 *END**************************************************************************/
cmp_drv_status_t CMP_DRV_GetOutputFlags(const uint32_t instance, cmp_output_trigger_t *flags)
{
    cmp_drv_status_t  status = CMP_DRV_SUCCESS;
    #ifdef DEV_ERROR_DETECT
    DEV_ASSERT(flags != NULL);
    DEV_ASSERT(instance < CMP_INSTANCE_COUNT);
    #endif
    const CMP_Type* base = g_cmpBase[instance];
    *flags = CMP_HAL_GetOutputEvent(base);
    return status;
}
/*FUNCTION**********************************************************************
 *
 * Function Name : CMP_DRV_ClearOutputFlags
 * Description   : Clear comparator output flags(rising and falling edge on output).
 * Implements : CMP_DRV_ClearOutputFlags_Activity
 *
 *END**************************************************************************/
cmp_drv_status_t CMP_DRV_ClearOutputFlags(const uint32_t instance)
{
    cmp_drv_status_t  status = CMP_DRV_SUCCESS;
    #ifdef DEV_ERROR_DETECT
    DEV_ASSERT(instance < CMP_INSTANCE_COUNT);
    #endif
    CMP_Type* base = g_cmpBase[instance];
    CMP_HAL_ClearOutputEvent(base);
    return status;
 }
 
 /*FUNCTION**********************************************************************
 *
 * Function Name : CMP_DRV_GetInputFlags
 * Description   : Return all input change flags in <flags>.
 * <flags> format : Flag_Ch7 Flag_Ch6 ... Flag_Ch0
 * Implements : CMP_DRV_GetInputFlags_Activity
 *
 *END**************************************************************************/
cmp_drv_status_t CMP_DRV_GetInputFlags(const uint32_t instance,  cmp_ch_list_t *flags)
{
    cmp_drv_status_t  status = CMP_DRV_SUCCESS;
    #ifdef DEV_ERROR_DETECT
    DEV_ASSERT(flags != NULL);
    DEV_ASSERT(instance < CMP_INSTANCE_COUNT);
    #endif
    const CMP_Type* base = g_cmpBase[instance];
    *flags = CMP_HAL_GetInputChangedFlags(base);
    return status;
 }
 /*FUNCTION**********************************************************************
 *
 * Function Name : CMP_DRV_ClearInputFlags
 * Description   : Clear all input change flags .
 * Implements : CMP_DRV_ClearInputFlags_Activity
 *
 *END**************************************************************************/
cmp_drv_status_t CMP_DRV_ClearInputFlags(const uint32_t instance)
{
    cmp_drv_status_t  status = CMP_DRV_SUCCESS;
    #ifdef DEV_ERROR_DETECT
    DEV_ASSERT(instance < CMP_INSTANCE_COUNT);
    #endif
    CMP_Type* base = g_cmpBase[instance];
    CMP_HAL_ClearInputChangedFlags(base);
    return status;
}
/*FUNCTION**********************************************************************
 *
 * Function Name : CMP_DRV_GetInitConfigComparator
 * Description   : Return configuration structure with reset values for comparator features (functional mode, power mode,
 * inverter, hysteresis, offset, filter sampling period and samples count).
 * Implements : CMP_DRV_GetInitConfigComparator_Activity
 *
 *END**************************************************************************/
void CMP_DRV_GetInitConfigComparator(cmp_comparator_t* config)
{
    #ifdef DEV_ERROR_DETECT
    DEV_ASSERT(config != NULL);
    #endif
    config->outputInterruptTrigger = CMP_NO_EVENT;
    config->dmaTriggerState = false;
    config->mode = CMP_DISABLED; 
    config->filterSampleCount = 0U;
    config->filterSamplePeriod = 0U;
    config->powerMode = CMP_LOW_SPEED;
    config->inverterState = CMP_NORMAL;
    config->outputSelect = CMP_COUT;
    config->pinState = CMP_UNAVAILABLE; 
    config->offsetLevel = CMP_LEVEL_OFFSET_0;
    config->hysteresisLevel = CMP_LEVEL_HYS_0;
}
/*FUNCTION**********************************************************************
 *
 * Function Name : CMP_DRV_ConfigComparator
 * Description   : Configure only comparator features (functional mode, power mode,
 * inverter, hysteresis, offset, filter sampling period and samples count).
 * Implements : CMP_DRV_ConfigComparator_Activity
 *
 *END**************************************************************************/
cmp_drv_status_t CMP_DRV_ConfigComparator(const uint32_t instance, const cmp_comparator_t *config)
{
    cmp_drv_status_t  status = CMP_DRV_SUCCESS;
    #ifdef DEV_ERROR_DETECT
    DEV_ASSERT(config != NULL);
    DEV_ASSERT(instance < CMP_INSTANCE_COUNT);
    #endif
    CMP_Type* base = g_cmpBase[instance];
    CMP_HAL_SetDMATriggerState(base, config->dmaTriggerState );
    CMP_HAL_SetOutputInterruptTrigger(base, config->outputInterruptTrigger);
    CMP_HAL_SetFunctionalMode(base, config->mode, config->filterSampleCount, config->filterSamplePeriod);
    CMP_HAL_SetFilterSamplePeriod(base, config->filterSampleCount );
    CMP_HAL_SetFilterSampleCount(base, config->filterSamplePeriod);
    CMP_HAL_SetPowerMode(base, config->powerMode);
    CMP_HAL_SetInverterState(base, config->inverterState);
    CMP_HAL_SetComparatorOutputSource(base, config->outputSelect);
    CMP_HAL_SetOutputPinState(base, config->pinState);
    CMP_HAL_SetOffset(base, config->offsetLevel);
    CMP_HAL_SetHysteresis(base, config->hysteresisLevel);
    return status;
}
/*FUNCTION**********************************************************************
 *
 * Function Name : CMP_DRV_GetComparator
 * Description   : Return configuration for comparator components from CMP module.
 * Implements : CMP_DRV_GetComparatorConfig_Activity
 *
 *END**************************************************************************/
cmp_drv_status_t CMP_DRV_GetComparatorConfig(const uint32_t instance, cmp_comparator_t *config)
{
    cmp_drv_status_t  status = CMP_DRV_SUCCESS;
    #ifdef DEV_ERROR_DETECT
    DEV_ASSERT(config != NULL);
    DEV_ASSERT(instance < CMP_INSTANCE_COUNT);
    #endif
    const CMP_Type* base = g_cmpBase[instance];
    config->dmaTriggerState = CMP_HAL_GetDMATriggerState(base);
    config->outputInterruptTrigger = CMP_HAL_GetOutputInterruptTrigger(base);
    config->mode = CMP_HAL_GetFunctionalMode(base);
    config->filterSampleCount = CMP_HAL_GetFilterSamplePeriod(base);
    config->filterSamplePeriod = CMP_HAL_GetFilterSampleCount(base);
    config->powerMode = CMP_HAL_GetPowerMode(base);
    config->inverterState = CMP_HAL_GetInverterState(base);
    config->outputSelect = CMP_HAL_GetComparatorOutputSource(base);
    config->pinState = CMP_HAL_GetOutputPinState(base);
    config->offsetLevel = CMP_HAL_GetOffset(base);
    config->hysteresisLevel = CMP_HAL_GetHysteresis(base);
    return status;
}
/******************************************************************************
 * EOF
 *****************************************************************************/

