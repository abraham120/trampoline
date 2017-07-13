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

#include "fsl_pdb_driver.h"
#include "fsl_interrupt_manager.h"
#include <stddef.h>

 /**
* @page misra_violations MISRA-C:2012 violations
*
* @section [global]
* Violates MISRA 2012 Advisory Rule 11.4, Conversion between a pointer and integer type
 * The base addresses are provided as integers so they need to be cast to
 * pointers.
*
* @section [global]
* Violates MISRA 2012 Required Rule 11.6, Cast from unsigned int to pointer
 * The base addresses are provided as integers so they need to be cast to
 * pointers.
 *
*/

/*******************************************************************************
 * Variables
 ******************************************************************************/

/* Table of base addresses for PDB instances. */
static PDB_Type * const s_pdbBase[PDB_INSTANCE_COUNT] = PDB_BASE_PTRS;

/* Table to save PDB IRQ enum numbers. */
static const IRQn_Type s_pdbIrqId[PDB_INSTANCE_COUNT] = PDB_IRQS;

/*FUNCTION*********************************************************************
 *
 * Function Name : PDB_DRV_Init
 * Description   : Initialize the PDB counter and trigger input for PDB module.
 * It resets PDB registers and enables the clock for PDB. So it should be
 * called before any operation to PDB module. After initialized, the PDB can
 * ack as a triggered timer, which lays the foundation for other features in
 * PDB module.
 *
 * Implements : PDB_DRV_Init_Activity
 *END*************************************************************************/
pdb_status_t PDB_DRV_Init(const uint32_t instance, const pdb_timer_config_t * const userConfigPtr)
{
#ifdef DEV_ERROR_DETECT
    DEV_ASSERT(instance < PDB_INSTANCE_COUNT);
    DEV_ASSERT(userConfigPtr != NULL);
#endif
    /* Table of PDB clock names as defined in clock manager. */
    static const clock_names_t s_pdbClkNames[PDB_INSTANCE_COUNT] = {PCC_PDB0_CLOCK, PCC_PDB1_CLOCK};

    pdb_status_t returnCode = PDB_STATUS_SUCCESS;
    clock_manager_error_code_t clkStatus;

    PDB_Type * base = s_pdbBase[instance];

    clock_names_t instanceClkName = s_pdbClkNames[instance];

    /* Get the PDB clock as configured in the clock manager */
    clkStatus = CLOCK_SYS_GetFreq(instanceClkName, NULL);

    /* Exit if clock is not enabled for current instance. */
    if (clkStatus != CLOCK_MANAGER_SUCCESS)
    {
        /* PDB module is not clocked */
        returnCode = PDB_STATUS_CLOCK_DISABLED;
    }  else
    {
        /* Reset the registers for PDB module to reset state. */
        PDB_HAL_Init(base);
        PDB_HAL_Enable(base);
        PDB_HAL_ConfigTimer(base, userConfigPtr);

        /* Configure NVIC. */
        if (userConfigPtr->intEnable || userConfigPtr->seqErrIntEnable)
        {
            INT_SYS_EnableIRQ(s_pdbIrqId[instance] );/* Enable PDB interrupt in NVIC level.*/
        } else
        {
            INT_SYS_DisableIRQ(s_pdbIrqId[instance] );/* Disable PDB interrupt in NVIC level.*/
        }
    }
    return returnCode;
}


/*FUNCTION*********************************************************************
 *
 * Function Name : PDB_DRV_Deinit
 * Description   : De-initialize the PDB module.
 * When the PDB module is not used. Calling this function would shutdown the
 * PDB module and reduce the power consumption.
 *
 * Implements : PDB_DRV_Deinit_Activity
 *END*************************************************************************/
pdb_status_t PDB_DRV_Deinit(const uint32_t instance)
{
#ifdef DEV_ERROR_DETECT
    DEV_ASSERT(instance < PDB_INSTANCE_COUNT);
#endif
    PDB_Type * base = s_pdbBase[instance];

    INT_SYS_DisableIRQ( s_pdbIrqId[instance] );
    PDB_HAL_Disable(base);

    return PDB_STATUS_SUCCESS;
}

/*FUNCTION*********************************************************************
 *
 * Function Name : PDB_DRV_SoftTriggerCmd
 * Description   : Trigger PDB by software trigger.
 * When the PDB is set to use software trigger as input, Calling this function
 * would trigger the PDB.
 *
 * Implements : PDB_DRV_SoftTriggerCmd_Activity
 *END*************************************************************************/
void PDB_DRV_SoftTriggerCmd(const uint32_t instance)
{
#ifdef DEV_ERROR_DETECT
    DEV_ASSERT(instance < PDB_INSTANCE_COUNT);
#endif
    PDB_Type * base = s_pdbBase[instance];

    PDB_HAL_SetSoftTriggerCmd(base);
}

/*FUNCTION*********************************************************************
 *
 * Function Name : PDB_DRV_GetTimerValue
 * Description   : Get the current counter value in PDB module.
 *
 * Implements : PDB_DRV_GetTimerValue_Activity
 *END*************************************************************************/
uint32_t PDB_DRV_GetTimerValue(const uint32_t instance)
{
#ifdef DEV_ERROR_DETECT
    DEV_ASSERT(instance < PDB_INSTANCE_COUNT);
#endif
    PDB_Type const * base = s_pdbBase[instance];

    return PDB_HAL_GetTimerValue(base);
}

/*FUNCTION*********************************************************************
 *
 * Function Name : PDB_DRV_GetTimerIntFlag
 * Description   : Get the interrupt flag for PDB module. It will be
 * asserted if the PDB interrupt occurs.
 *
 * Implements : PDB_DRV_GetTimerIntFlag_Activity
 *END*************************************************************************/
bool PDB_DRV_GetTimerIntFlag(const uint32_t instance)
{
#ifdef DEV_ERROR_DETECT
    DEV_ASSERT(instance < PDB_INSTANCE_COUNT);
#endif
    PDB_Type const * base = s_pdbBase[instance];

    return PDB_HAL_GetTimerIntFlag(base);
}

/*FUNCTION*********************************************************************
 *
 * Function Name : PDB_DRV_ClearTimerIntFlag
 * Description   : Clear the interrupt flag for PDB module.
 *
 * Implements : PDB_DRV_ClearTimerIntFlag_Activity
 *END*************************************************************************/
void PDB_DRV_ClearTimerIntFlag(const uint32_t instance)
{
#ifdef DEV_ERROR_DETECT
    DEV_ASSERT(instance < PDB_INSTANCE_COUNT);
#endif
    PDB_Type * base = s_pdbBase[instance];

    PDB_HAL_ClearTimerIntFlag(base);
}

/*FUNCTION*********************************************************************
 *
 * Function Name : PDB_DRV_LoadValuesCmd
 * Description   : Execute the command of loading values.
 *
 * Implements : PDB_DRV_LoadValuesCmd_Activity
 *END*************************************************************************/
void PDB_DRV_LoadValuesCmd(const uint32_t instance)
{
#ifdef DEV_ERROR_DETECT
    DEV_ASSERT(instance < PDB_INSTANCE_COUNT);
#endif
    PDB_Type * base = s_pdbBase[instance];

    PDB_HAL_SetLoadValuesCmd(base);
}

/*FUNCTION*********************************************************************
 *
 * Function Name : PDB_DRV_SetTimerModulusValue
 * Description   : Set the value of timer modulus.
 *
 * Implements : PDB_DRV_SetTimerModulusValue_Activity
 *END*************************************************************************/
void PDB_DRV_SetTimerModulusValue(const uint32_t instance, const uint32_t value)
{
#ifdef DEV_ERROR_DETECT
    DEV_ASSERT(instance < PDB_INSTANCE_COUNT);
#endif
    PDB_Type * base = s_pdbBase[instance];

    PDB_HAL_SetTimerModulusValue(base, value);
}

/*FUNCTION*********************************************************************
 *
 * Function Name : PDB_DRV_SetValueForTimerInterrupt
 * Description   : Set the value for the timer interrupt.
 *
 * Implements : PDB_DRV_SetValueForTimerInterrupt_Activity
 *END*************************************************************************/
void PDB_DRV_SetValueForTimerInterrupt(const uint32_t instance, const uint32_t value)
{
#ifdef DEV_ERROR_DETECT
    DEV_ASSERT(instance < PDB_INSTANCE_COUNT);
#endif
    PDB_Type * base = s_pdbBase[instance];

    PDB_HAL_SetValueForTimerInterrupt(base, value);
}

/*FUNCTION*********************************************************************
 *
 * Function Name : PDB_DRV_ConfigAdcPreTrigger
 * Description   : Configure the ADC pre_trigger in the PDB module.
 *
 * Implements : PDB_DRV_ConfigAdcPreTrigger_Activity
 *END*************************************************************************/
pdb_status_t PDB_DRV_ConfigAdcPreTrigger(const uint32_t instance, const uint32_t chn, const pdb_adc_pretrigger_config_t *configPtr)
{
#ifdef DEV_ERROR_DETECT
    DEV_ASSERT(instance < PDB_INSTANCE_COUNT);
    DEV_ASSERT(chn < FSL_FEATURE_PDB_ADC_CHANNEL_COUNT);
    DEV_ASSERT(configPtr != NULL);
#endif
    PDB_Type * base = s_pdbBase[instance];
    pdb_status_t returnCode = PDB_STATUS_SUCCESS;

    PDB_HAL_SetAdcPreTriggerEnable(base, chn, ((uint32_t)1U << (configPtr->adcPreTriggerIdx)), configPtr->preTriggerEnable);
    PDB_HAL_SetAdcPreTriggerOutputEnable(base, chn, ((uint32_t)1U << (configPtr->adcPreTriggerIdx)), configPtr->preTriggerOutputEnable);
    PDB_HAL_SetAdcPreTriggerBackToBackEnable(base, chn, ((uint32_t)1U << (configPtr->adcPreTriggerIdx)), configPtr->preTriggerBackToBackEnable);

    return returnCode;
}

/*FUNCTION*********************************************************************
 *
 * Function Name : PDB_DRV_GetAdcPreTriggerFlags
 * Description   : Get the ADC pre_trigger flag in the PDB module.
 *
 * Implements : PDB_DRV_GetAdcPreTriggerFlags_Activity
 *END*************************************************************************/
uint32_t PDB_DRV_GetAdcPreTriggerFlags(const uint32_t instance, const uint32_t chn, const uint32_t preChnMask)
{
#ifdef DEV_ERROR_DETECT
    DEV_ASSERT(instance < PDB_INSTANCE_COUNT);
    DEV_ASSERT(chn < FSL_FEATURE_PDB_ADC_CHANNEL_COUNT);
#endif
    PDB_Type const * base = s_pdbBase[instance];

    return PDB_HAL_GetAdcPreTriggerFlags(base, chn, preChnMask);
}

/*FUNCTION*********************************************************************
 *
 * Function Name : PDB_DRV_ClearAdcPreTriggerFlags
 * Description   : Clear the ADC pre_trigger flag in the PDB module.
 *
 * Implements : PDB_DRV_ClearAdcPreTriggerFlags_Activity
 *END*************************************************************************/
void PDB_DRV_ClearAdcPreTriggerFlags(const uint32_t instance, const uint32_t chn, const uint32_t preChnMask)
{
#ifdef DEV_ERROR_DETECT
    DEV_ASSERT(instance < PDB_INSTANCE_COUNT);
    DEV_ASSERT(chn < FSL_FEATURE_PDB_ADC_CHANNEL_COUNT);
#endif
    PDB_Type * base = s_pdbBase[instance];

    PDB_HAL_ClearAdcPreTriggerFlags(base, chn, preChnMask);
}

/*FUNCTION*********************************************************************
 *
 * Function Name : PDB_DRV_GetAdcPreTriggerSeqErrFlags
 * Description   : Get the ADC pre_trigger flag in the PDB module.
 *
 * Implements : PDB_DRV_GetAdcPreTriggerSeqErrFlags_Activity
 *END*************************************************************************/
uint32_t PDB_DRV_GetAdcPreTriggerSeqErrFlags(const uint32_t instance, const uint32_t chn, const uint32_t preChnMask)
{
#ifdef DEV_ERROR_DETECT
    DEV_ASSERT(instance < PDB_INSTANCE_COUNT);
    DEV_ASSERT(chn < FSL_FEATURE_PDB_ADC_CHANNEL_COUNT);
#endif
    PDB_Type const * base = s_pdbBase[instance];

    return PDB_HAL_GetAdcPreTriggerSeqErrFlags(base, chn, preChnMask);
}

/*FUNCTION*********************************************************************
 *
 * Function Name : PDB_DRV_ClearAdcPreTriggerSeqErrFlags
 * Description   : Clear the ADC pre_trigger flag in the PDB module.
 *
 * Implements : PDB_DRV_ClearAdcPreTriggerSeqErrFlags_Activity
 *END*************************************************************************/
void PDB_DRV_ClearAdcPreTriggerSeqErrFlags(const uint32_t instance, const uint32_t chn, const uint32_t preChnMask)
{
#ifdef DEV_ERROR_DETECT
    DEV_ASSERT(instance < PDB_INSTANCE_COUNT);
    DEV_ASSERT(chn < FSL_FEATURE_PDB_ADC_CHANNEL_COUNT);
#endif
    PDB_Type * base = s_pdbBase[instance];

    PDB_HAL_ClearAdcPreTriggerSeqErrFlags(base, chn, preChnMask);
}

/*FUNCTION*********************************************************************
 *
 * Function Name : PDB_DRV_SetAdcPreTriggerDelayValue
 * Description   : Set the ADC pre_trigger delay value in the PDB module.
 *
 * Implements : PDB_DRV_SetAdcPreTriggerDelayValue_Activity
 *END*************************************************************************/
void PDB_DRV_SetAdcPreTriggerDelayValue(const uint32_t instance, const uint32_t chn, const uint32_t preChn, const uint32_t value)
{
#ifdef DEV_ERROR_DETECT
    DEV_ASSERT(instance < PDB_INSTANCE_COUNT);
    DEV_ASSERT(chn < FSL_FEATURE_PDB_ADC_CHANNEL_COUNT);
#endif
    PDB_Type * base = s_pdbBase[instance];

    PDB_HAL_SetAdcPreTriggerDelayValue(base, chn, preChn, value);
}

/*FUNCTION*********************************************************************
 *
 * Function Name : PDB_DRV_SetCmpPulseOutEnable
 * Description   : Switch on/off the CMP pulse out in the PDB module.
 *
 * Implements : PDB_DRV_SetCmpPulseOutEnable_Activity
 *END*************************************************************************/
void PDB_DRV_SetCmpPulseOutEnable(const uint32_t instance, const uint32_t pulseChnMask, const bool enable)
{
#ifdef DEV_ERROR_DETECT
    DEV_ASSERT(instance < PDB_INSTANCE_COUNT);
#endif
    PDB_Type * base = s_pdbBase[instance];

    PDB_HAL_SetCmpPulseOutEnable(base, pulseChnMask, enable);
}

/*FUNCTION*********************************************************************
 *
 * Function Name : PDB_DRV_SetCmpPulseOutDelayForHigh
 * Description   : Set the CMP pulse out delay value for high in the PDB module.
 *
 * Implements : PDB_DRV_SetCmpPulseOutDelayForHigh_Activity
 *END*************************************************************************/
void PDB_DRV_SetCmpPulseOutDelayForHigh(const uint32_t instance, const uint32_t pulseChn, const uint32_t value)
{
#ifdef DEV_ERROR_DETECT
    DEV_ASSERT(instance < PDB_INSTANCE_COUNT);
    DEV_ASSERT(pulseChn < FSL_FEATURE_PDB_PODLY_COUNT);
#endif
    PDB_Type * base = s_pdbBase[instance];

    PDB_HAL_SetCmpPulseOutDelayForHigh(base, pulseChn, value);
}

/*FUNCTION*********************************************************************
 *
 * Function Name : PDB_DRV_SetCmpPulseOutDelayForLow
 * Description   : Set the CMP pulse out delay value for low in the PDB module.
 *
 * Implements : PDB_DRV_SetCmpPulseOutDelayForLow_Activity
 *END*************************************************************************/
void PDB_DRV_SetCmpPulseOutDelayForLow(const uint32_t instance, const uint32_t pulseChn, const uint32_t value)
{
#ifdef DEV_ERROR_DETECT
    DEV_ASSERT(instance < PDB_INSTANCE_COUNT);
    DEV_ASSERT(pulseChn < FSL_FEATURE_PDB_PODLY_COUNT);
#endif
    PDB_Type * base = s_pdbBase[instance];

    PDB_HAL_SetCmpPulseOutDelayForLow(base, pulseChn, value);
}

/*******************************************************************************
 * EOF
 ******************************************************************************/
