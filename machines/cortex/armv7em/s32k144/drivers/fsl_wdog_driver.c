/*
 * Copyright (c) 2015, Freescale Semiconductor, Inc.
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
 * @file fsl_wdog_driver.c
 *
 * @page misra_violations MISRA-C:2012 violations
 *
 * @section [global]
 * Violates MISRA 2012 Required Rule 1.3, Taking address of near auto variable.
 * The code is not dynamically linked. An absolute stack address is obtained
 * when taking the address of the near auto variable. A source of error in
 * writing dynamic code is that the stack segment may be different from the data
 * segment.
 *
 * @section [global]
 * Violates MISRA 2012 Advisory Rule 11.4, Conversion between a pointer and
 * integer type.
 * The cast is required to initialize a pointer with an unsigned long define,
 * representing an address.
 *
 * @section [global]
 * Violates MISRA 2012 Required Rule 11.6, Cast from unsigned int to pointer.
 * The cast is required to initialize a pointer with an unsigned long define,
 * representing an address.
 */

#include "fsl_wdog_driver.h"


/*******************************************************************************
 * Variables
 ******************************************************************************/

/*! @brief Table of base addresses for WDOG instances. */
WDOG_Type * const g_wdogBase[] = WDOG_BASE_PTRS;

/*! @brief Table to save WDOG IRQ enum numbers defined in CMSIS header file. */
const IRQn_Type g_wdogIrqId[] = WDOG_IRQS;

/*******************************************************************************
 * Private Functions
 ******************************************************************************/
static wdog_status_t WDOG_DRV_Config(uint32_t instance, wdog_user_config_t wdogUserConfig);

static uint32_t WDOG_DRV_GetClockSourceFreq(wdog_clk_source_t wdogClk)
{
    uint32_t freq = 0;

    switch (wdogClk)
    {
    case WDOG_BUS_CLOCK:
        (void)CLOCK_SYS_GetFreq(BUS_CLOCK, &freq);
        break;
    case WDOG_SIRC_CLOCK:
        (void)CLOCK_SYS_GetFreq(SIRC_CLOCK, &freq);
        break;
    case WDOG_SOSC_CLOCK:
        (void)CLOCK_SYS_GetFreq(SOSC_CLOCK, &freq);
        break;
    case WDOG_LPO_CLOCK:
        (void)CLOCK_SYS_GetFreq(SIM_LPO_CLOCK, &freq);
        break;
    default:
        /* Should not get here */
        break;
    }

    return freq;
 }

/*******************************************************************************
 * Code
 ******************************************************************************/

/*FUNCTION**********************************************************************
 *
 * Function Name : WDOG_DRV_Init
 * Description   : initialize the WDOG driver
 *
 * Implements    : WDOG_DRV_Init_Activity
 *END**************************************************************************/
wdog_status_t WDOG_DRV_Init(uint32_t instance,
                            const wdog_user_config_t *userConfigPtr)
{
    const WDOG_Type *baseAddr;
    uint32_t prevClockHz, crtClockHz;
    wdog_status_t status = WDOG_STATUS_SUCCESS;

#ifdef DEV_ERROR_DETECT
    DEV_ASSERT(instance < WDOG_INSTANCE_COUNT);
#endif


    if (userConfigPtr == NULL)
    {
        status = WDOG_STATUS_NULL_PARAM;
    }
    else
    {
        baseAddr = g_wdogBase[instance];

        /* Check if the previous clock source and the configuration clock source
         * are enabled (if not, the counter will not be incremented) */
        prevClockHz = WDOG_DRV_GetClockSourceFreq(WDOG_HAL_GetConfig(baseAddr).clkSource);
        crtClockHz = WDOG_DRV_GetClockSourceFreq(userConfigPtr->clkSource);

        if ((prevClockHz == 0U) || (crtClockHz == 0U))
        {
             status = WDOG_STATUS_FAIL;
        }
        else
        {
            /* Configure module */
            status = WDOG_DRV_Config(instance, *userConfigPtr);

            if (status == WDOG_STATUS_SUCCESS)
            {
                /* enable WDOG timeout interrupt */
                INT_SYS_EnableIRQ(g_wdogIrqId[instance]);
            }
        }
    }

    return status;
}


/*FUNCTION**********************************************************************
 *
 * Function Name : WDOG_DRV_Deinit
 * Description   : deinitialize the WDOG driver
 *
 * Implements    : WDOG_DRV_Deinit_Activity
 *END**************************************************************************/
wdog_status_t WDOG_DRV_Deinit(uint32_t instance)
{
    WDOG_Type *baseAddr;

#ifdef DEV_ERROR_DETECT
    DEV_ASSERT(instance < WDOG_INSTANCE_COUNT);
#endif

    baseAddr = g_wdogBase[instance];

    INT_SYS_DisableIRQGlobal();

    /* Disable WDOG */
    WDOG_HAL_Disable(baseAddr);

    INT_SYS_EnableIRQGlobal();

    /* Disable WDOG timeout interrupt */
    INT_SYS_DisableIRQ(g_wdogIrqId[instance]);

    return WDOG_STATUS_SUCCESS;
}


/*FUNCTION**********************************************************************
 *
 * Function Name : WDOG_DRV_Config
 * Description   : config the WDOG driver
 *
 *END**************************************************************************/
static wdog_status_t WDOG_DRV_Config(uint32_t instance, wdog_user_config_t wdogUserConfig)
{
    WDOG_Type *baseAddr;
    wdog_status_t status = WDOG_STATUS_SUCCESS;

#ifdef DEV_ERROR_DETECT
    DEV_ASSERT(instance < WDOG_INSTANCE_COUNT);
#endif

    baseAddr = g_wdogBase[instance];

    INT_SYS_DisableIRQGlobal();

    if (WDOG_HAL_IsUpdateEnabled(baseAddr))
    {
	    WDOG_HAL_Config(baseAddr, &wdogUserConfig);
	    while (WDOG_HAL_IsUnlocked(baseAddr))
        {
            /* Wait until the unlock window closes */
        }
    }
    else
    {
        status = WDOG_STATUS_FAIL;
    }
    
    INT_SYS_EnableIRQGlobal();

    return status;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : WDOG_DRV_GetConfig
 * Description   : get the current configuration of the WDOG driver
 *
 * Implements    : WDOG_DRV_GetConfig_Activity
 *END**************************************************************************/
 wdog_status_t WDOG_DRV_GetConfig(uint32_t instance, wdog_user_config_t *config)
{
    const WDOG_Type *baseAddr;

#ifdef DEV_ERROR_DETECT
    DEV_ASSERT(instance < WDOG_INSTANCE_COUNT);
#endif

    baseAddr = g_wdogBase[instance];

    *config = WDOG_HAL_GetConfig(baseAddr);

    return WDOG_STATUS_SUCCESS;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : WDOG_DRV_SetInt
 * Description   : enable/disable the WDOG timeout interrupt and  set handler
 *
 * Implements    : WDOG_DRV_SetInt_Activity
 *END**************************************************************************/
 wdog_status_t WDOG_DRV_SetInt(uint32_t instance, bool enable, void (*handler)(void))
{
    WDOG_Type *baseAddr;
    wdog_status_t status = WDOG_STATUS_SUCCESS;
    interrupt_manager_error_code_t intManErr;
    
#ifdef DEV_ERROR_DETECT
    DEV_ASSERT(instance < WDOG_INSTANCE_COUNT);
#endif

    baseAddr = g_wdogBase[instance];

    if (enable && (handler == NULL))
    {
        status = WDOG_STATUS_NULL_PARAM;
    }
    if (enable && (handler != NULL))
    {
        intManErr = INT_SYS_InstallHandler(g_wdogIrqId[instance], handler, (isr_t*) 0);
        if (intManErr != INTERRUPT_MANAGER_SUCCESS)
        {
            status = WDOG_STATUS_FAIL;
        }
    }

    if (status == WDOG_STATUS_SUCCESS)
    {
        if (WDOG_HAL_IsUpdateEnabled(baseAddr))
        {
            INT_SYS_DisableIRQGlobal();

            WDOG_HAL_SetInt(baseAddr, enable);
            while (WDOG_HAL_IsUnlocked(baseAddr))
            {
                /* Wait until the unlock window closes */
            }

            INT_SYS_EnableIRQGlobal();
        }
        else
        {
            status = WDOG_STATUS_FAIL;
        }
    }

    return status;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : WDOG_DRV_Trigger
 * Description   : reset the WDOG counter
 *
 * Implements    : WDOG_DRV_Trigger_Activity
 *END**************************************************************************/
 wdog_status_t WDOG_DRV_Trigger(uint32_t instance)
{
    WDOG_Type *baseAddr;

#ifdef DEV_ERROR_DETECT
    DEV_ASSERT(instance < WDOG_INSTANCE_COUNT);
#endif

    baseAddr = g_wdogBase[instance];

    WDOG_HAL_Trigger(baseAddr);

    return WDOG_STATUS_SUCCESS;
}

/*******************************************************************************
 * EOF
 ******************************************************************************/
