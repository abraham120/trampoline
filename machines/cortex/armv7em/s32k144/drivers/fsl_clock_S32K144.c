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

/**
 * @page misra_violations MISRA-C:2012 violations
 *
 * @section [global]
 * Violates MISRA 2012 Required Rule 1.3, There shall be no occurrence of
 * undefined or critical unspecified behaviour.
 * The addresses of the stack variables are only used at local scope.
 *
 * @section [global]
 * Violates MISRA 2012 Required Rule 11.6, A cast shall not be performed
 * between pointer to void and an arithmetic type.
 * The base address parameter from HAL functions is provided as integer so
 * it needs to be cast to pointer.
 *
 * @section [global]
 * Violates MISRA 2012 Advisory Rule 11.4, A conversion should not be performed
 * between a pointer to object and an integer type.
 * The base address parameter from HAL functions is provided as integer so
 * a conversion between a pointer and an integer has to be performed
 *
 * @section [global]
 * Violates MISRA 2012 Advisory Rule 8.7, External could be made static.
 * Function is defined for usage by application code.
 */

 
#include "fsl_clock_S32K144.h"
#include "fsl_clock_manager.h"
#include "fsl_device_registers.h"
#include <stddef.h>
/*
 * README:
 * This file should provide these APIs:
 * 1. APIs to get the frequency of output clocks in Reference Manual ->
 * Chapter Clock Distribution -> Figure Clocking diagram.
 * 2. APIs for IP modules listed in Reference Manual -> Chapter Clock Distribution
 * -> Module clocks.
 */

/*******************************************************************************
 * Definitions
 ******************************************************************************/
/* This frequency values should be set by different boards. */
/* SIM */
uint32_t g_TClkFreq[NUMBER_OF_TCLK_INPUTS];      /* TCLKx clocks    */

/* RTC */
uint32_t g_RtcClkInFreq;                         /* RTC CLKIN clock */

/* SCG */
uint32_t g_xtal0ClkFreq;                         /* EXTAL0 clock    */

/*******************************************************************************
 * PROTOTYPES
 ******************************************************************************/

static clock_manager_error_code_t CLOCK_SYS_GetScgClockFreq(clock_names_t clockName, uint32_t * frequency);
static clock_manager_error_code_t CLOCK_SYS_GetSimClockFreq(clock_names_t clockName, uint32_t * frequency);
static clock_manager_error_code_t CLOCK_SYS_GetPccClockFreq(clock_names_t clockName, uint32_t * frequency);
static uint32_t CLOCK_SYS_GetPeripheralClock(clock_names_t clockName, scg_async_clock_type_t divider);
static scg_system_clock_mode_t CLOCK_SYS_GetCurrentRunMode(const SMC_Type * smc_base);
static bool CLOCK_SYS_SwitchSystemClock(scg_system_clock_src_t to_clk);
static uint32_t CLOCK_SYS_GetSimClkOutFreq(const SIM_Type * base);
static uint32_t CLOCK_SYS_GetScgClkOutFreq(const SCG_Type * base);
static uint32_t CLOCK_SYS_GetSimRtcClkFreq(const SIM_Type * base);

/*******************************************************************************
 * Code
 ******************************************************************************/

 
/*FUNCTION**********************************************************************
 *
 * Function Name : CLOCK_SYS_SetConfiguration
 * Description   : This function sets the system to target configuration, it
 * only sets the clock modules registers for clock mode change, but not send
 * notifications to drivers.
 *
 * Implements CLOCK_SYS_SetConfiguration_Activity
 *END**************************************************************************/
clock_manager_error_code_t CLOCK_SYS_SetConfiguration(clock_manager_user_config_t const* config)
{
    clock_manager_error_code_t result;
#ifdef DEV_ERROR_DETECT
    DEV_ASSERT(config);
#endif

    /* Set SCG settings. */
    if(STATUS_SCG_SUCCESS != CLOCK_SYS_SetScgConfiguration(&config->scgConfig))
    {
        result = CLOCK_MANAGER_INVALID_PARAM;
    }
    else
    {
        result = CLOCK_MANAGER_SUCCESS;
    }

    /* Set PCC settings. */
    CLOCK_SYS_SetPccConfiguration(&config->pccConfig);

    /* Set SIM settings. */
    CLOCK_SYS_SetSimConfiguration(&config->simConfig);

    /* Set PMC settings. */
    CLOCK_SYS_SetPmcConfiguration(&config->pmcConfig);

    return result;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : CLOCK_SYS_SetScgConfiguration
 * Description   : This function configures the SCG blocks
 *
 * Implements CLOCK_SYS_SetScgConfiguration_Activity
 *END**************************************************************************/
scg_status_t CLOCK_SYS_SetScgConfiguration(const scg_config_t *scgConfig)
{
    scg_status_t status;
    uint32_t timeout;
    scg_system_clock_config_t current_config;
#ifdef DEV_ERROR_DETECT
    DEV_ASSERT(scgConfig);
#endif

    status = STATUS_SCG_SUCCESS;

    if (scgConfig != NULL)
    {

        /* Get CURRENT mode configuration */
        SCG_HAL_GetSystemClockConfig(SCG, SCG_SYSTEM_CLOCK_MODE_CURRENT, &current_config);

        /* Configure SIRC. */
        if (scgConfig->sircConfig.initialize)
        {
            /* Check and switch to alternate */
            if ( current_config.src == SCG_SYSTEM_CLOCK_SRC_SIRC )
            {
                (void)CLOCK_SYS_SwitchSystemClock(scgConfig->clockModeConfig.alternateClock);
            }

            /* Disable the SIRC. */
            status = SCG_HAL_DeinitSirc(SCG);
            if (status == STATUS_SCG_SUCCESS)
            {
                /* Setup SIRC. */
                status = SCG_HAL_InitSirc(SCG, &(scgConfig->sircConfig));

                if (status == STATUS_SCG_SUCCESS)
                {
                    /* Wait for SIRC to initialize - typical 6us*/
                    /* f core = max 112mhz=>9ns/cycle, ~26 cycles per loop=> 26 loops*/
                    timeout = 26U;
                    while((SCG_HAL_GetSircFreq(SCG) == 0U) && (timeout > 0U))
                    {
                        timeout--;
                    }

                    /* Revert from alternate */
                    if ( current_config.src == SCG_SYSTEM_CLOCK_SRC_SIRC )
                    {
                        status = CLOCK_SYS_SwitchSystemClock( SCG_SYSTEM_CLOCK_SRC_SIRC ) ?
                                  STATUS_SCG_SUCCESS : STATUS_SCG_FAILED;
                    }

                    #ifdef DEV_ERROR_DETECT
                        DEV_ASSERT(STATUS_SCG_SUCCESS == status);
                    #endif
                }
            }
        }

        /* Configure FIRC. */
        if (scgConfig->fircConfig.initialize && (status == STATUS_SCG_SUCCESS))
        {
            /* Check and switch to alternate */
            if ( current_config.src == SCG_SYSTEM_CLOCK_SRC_FIRC )
            {
                (void)CLOCK_SYS_SwitchSystemClock(scgConfig->clockModeConfig.alternateClock);
            }

            /* Disable the FIRC. */
            status = SCG_HAL_DeinitFirc(SCG);
            if (status == STATUS_SCG_SUCCESS)
            {
                /* Setup FIRC. */
                status = SCG_HAL_InitFirc(SCG, &(scgConfig->fircConfig));
                if (status == STATUS_SCG_SUCCESS)
                {
                    /* Revert from alternate */
                    if ( current_config.src == SCG_SYSTEM_CLOCK_SRC_FIRC )
                    {
                        status = CLOCK_SYS_SwitchSystemClock( SCG_SYSTEM_CLOCK_SRC_FIRC ) ?
                                  STATUS_SCG_SUCCESS : STATUS_SCG_FAILED;
                    }

                    #ifdef DEV_ERROR_DETECT
                        DEV_ASSERT(STATUS_SCG_SUCCESS == status);
                    #endif
                }
            }
        }

        /* Configure SOSC. */
        if (scgConfig->soscConfig.initialize && (status == STATUS_SCG_SUCCESS))
        {
            /* Check and switch to alternate */
            if ( current_config.src == SCG_SYSTEM_CLOCK_SRC_SYS_OSC )
            {
                (void)CLOCK_SYS_SwitchSystemClock(scgConfig->clockModeConfig.alternateClock);
            }

            /* Disable the SOSC. */
            status = SCG_HAL_DeinitSysOsc(SCG);
            if (status == STATUS_SCG_SUCCESS)
            {
                /* Setup System OSC. */
                status = SCG_HAL_InitSysOsc(SCG, &(scgConfig->soscConfig));
                if (status == STATUS_SCG_SUCCESS)
                {
                    /* Wait for System OSC to initialize - max 750ms*/
                    /* f core = max 112mhz=>9ns/cycle, ~26 cycles per loop=> 3205000 loops*/
                    timeout = 3205000U;
                    while((SCG_HAL_GetSysOscFreq(SCG) == 0U) && (timeout > 0U))
                    {
                        timeout--;
                    }

                    /* Revert from alternate */
                    if ( current_config.src == SCG_SYSTEM_CLOCK_SRC_SYS_OSC )
                    {
                        status = CLOCK_SYS_SwitchSystemClock( SCG_SYSTEM_CLOCK_SRC_SYS_OSC ) ?
                                  STATUS_SCG_SUCCESS : STATUS_SCG_FAILED;
                    }

                    #ifdef DEV_ERROR_DETECT
                        DEV_ASSERT(STATUS_SCG_SUCCESS == status);
                    #endif
                }
            }
        }

        /* Configure SPLL. */
        if (scgConfig->spllConfig.initialize && (status == STATUS_SCG_SUCCESS))
        {
            /* Check and switch to alternate */
            if ( current_config.src == SCG_SYSTEM_CLOCK_SRC_SYS_PLL )
            {
                (void)CLOCK_SYS_SwitchSystemClock(scgConfig->clockModeConfig.alternateClock);
            }

            /* Disable the SPLL. */
            status = SCG_HAL_DeinitSysPll(SCG);
            if (status == STATUS_SCG_SUCCESS)
            {

                /* Setup SPLL. */
                status = SCG_HAL_InitSysPll(SCG, &(scgConfig->spllConfig));
                if (status == STATUS_SCG_SUCCESS)
                {
                    /* Revert from alternate */
                    if ( current_config.src == SCG_SYSTEM_CLOCK_SRC_SYS_PLL )
                    {
                        status = CLOCK_SYS_SwitchSystemClock( SCG_SYSTEM_CLOCK_SRC_SYS_PLL ) ?
                                  STATUS_SCG_SUCCESS : STATUS_SCG_FAILED;
                    }
                }

                #ifdef DEV_ERROR_DETECT
                    DEV_ASSERT(STATUS_SCG_SUCCESS == status);
                #endif
            }
        }

        if (status == STATUS_SCG_SUCCESS)
        {
            /* Configure RTC. */
            if (scgConfig->rtcConfig.initialize )
            {
                /* RTC Clock settings. */
                SCG_HAL_SetRtcClkInFreq(SCG, scgConfig->rtcConfig.rtcClkInFreq);
            }

            /* Configure SCG ClockOut. */
            if (scgConfig->clockOutConfig.initialize)
            {
                /* ClockOut settings. */
                SCG_HAL_SetClockoutSourceSel(SCG, scgConfig->clockOutConfig.source);
            }

            /* Configure SCG clock modes. */
            if (scgConfig->clockModeConfig.initialize)
            {
                /* Configure SCG clock modes */
                SCG_HAL_SetSystemClockConfig(SCG, SCG_SYSTEM_CLOCK_MODE_RUN,   &(scgConfig->clockModeConfig.rccrConfig));
                SCG_HAL_SetSystemClockConfig(SCG, SCG_SYSTEM_CLOCK_MODE_VLPR,  &(scgConfig->clockModeConfig.vccrConfig));
                SCG_HAL_SetSystemClockConfig(SCG, SCG_SYSTEM_CLOCK_MODE_HSRUN, &(scgConfig->clockModeConfig.hccrConfig));
            }
        }
    }

    return status;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : CLOCK_SYS_SetPccConfiguration
 * Description   : This function configures the PCC block
 *
 * Implements CLOCK_SYS_SetPccConfiguration_Activity
 *END**************************************************************************/
void CLOCK_SYS_SetPccConfiguration(const pcc_config_t *peripheralClockConfig)
{
#ifdef DEV_ERROR_DETECT
    DEV_ASSERT(peripheralClockConfig);
#endif
    PCC_HAL_SetPeripheralClockConfig(PCC, peripheralClockConfig);
}

/*FUNCTION**********************************************************************
 *
 * Function Name : CLOCK_SYS_SetSimConfiguration
 * Description   : This function configures the PCC block
 *
 * Implements CLOCK_SYS_SetSimConfiguration_Activity
 *END**************************************************************************/
void CLOCK_SYS_SetSimConfiguration(const sim_clock_config_t *simClockConfig)
{
#ifdef DEV_ERROR_DETECT
    DEV_ASSERT(simClockConfig);
#endif
    uint8_t i;

    /* ClockOut settings. */
    if (simClockConfig->clockOutConfig.initialize)
    {
        SIM_HAL_InitClkout(SIM, &(simClockConfig->clockOutConfig));
    }

    /* Low Power Clock settings from SIM. */
    if (simClockConfig->lpoClockConfig.initialize)
    {
        SIM_HAL_SetLpoClocks(SIM, simClockConfig->lpoClockConfig);
    }

    /* Platform Gate Clock settings. */
    if (simClockConfig->platGateConfig.initialize)
    {
        SIM_HAL_SetMscmClockGate(SIM, simClockConfig->platGateConfig.enableMscm);
        SIM_HAL_SetMpuClockGate(SIM,  simClockConfig->platGateConfig.enableMpu);
        SIM_HAL_SetDmaClockGate(SIM,  simClockConfig->platGateConfig.enableDma);
        SIM_HAL_SetErmClockGate(SIM,  simClockConfig->platGateConfig.enableErm);
        SIM_HAL_SetEimClockGate(SIM,  simClockConfig->platGateConfig.enableEim);
    }

    /* TCLK Clock settings. */
    if (simClockConfig->tclkConfig.initialize)
    {
        for( i = 0; i< NUMBER_OF_TCLK_INPUTS; i++)
        {
            SIM_HAL_SetTClkFreq(SIM, i, simClockConfig->tclkConfig.tclkFreq[i]);
        }
    }

    /* Debug trace Clock settings. */
    if (simClockConfig->traceClockConfig.initialize)
    {
        SIM_HAL_InitTraceClock(SIM, &(simClockConfig->traceClockConfig));
    }
}


/*FUNCTION**********************************************************************
 *
 * Function Name : CLOCK_SYS_SetPmcConfiguration
 * Description   : This function configures the PMC block
 *
 * Implements CLOCK_SYS_SetPmcConfiguration_Activity
 *END**************************************************************************/
void CLOCK_SYS_SetPmcConfiguration(const pmc_config_t *pmcConfig)
{
#ifdef DEV_ERROR_DETECT
    DEV_ASSERT(pmcConfig);
#endif

    /* Low Power Clock settings from PMC. */
    if (pmcConfig->lpoClockConfig.initialize)
    {
        /* Enable/disable the low power oscillator. */
        if (pmcConfig->lpoClockConfig.enable)
        {
            PMC_HAL_SetLpoMode(PMC,false);
        }
        else
        {
            PMC_HAL_SetLpoMode(PMC,true);
        }

        /* Manual calibration selected, (over)write trimming values */
        if (PMC_MANUAL_CALIBRATION_MODE == pmcConfig->lpoClockConfig.calibrationMode)
        {
            PMC_HAL_SetLpoTrimValue(PMC,pmcConfig->lpoClockConfig.trimValue);
        }
        /* Automatic trimming option is selected, calibration is done in clock manager driver. */
        else
        {
             //TODO  ASDK-1594
        }
    }
}


/*FUNCTION**********************************************************************
 *
 * Function Name : CLOCK_SYS_GetScgClockFreq
 * Description   : This function returns the frequency of a given clock from SCG
 *
 *END**************************************************************************/
static clock_manager_error_code_t CLOCK_SYS_GetScgClockFreq(clock_names_t clockName,
                                                            uint32_t *frequency)
{
    clock_manager_error_code_t returnCode = CLOCK_MANAGER_SUCCESS;
    /* if memory is allocated it must be cleared. */
    if (frequency != NULL)
    {
        *frequency = 0U;
    }


    switch (clockName)
    {
        /* Main clocks */
        case CORE_CLOCK:
            *frequency = SCG_HAL_GetSystemClockFreq(SCG,SCG_SYSTEM_CLOCK_CORE);
            break;
        case BUS_CLOCK:
            *frequency = SCG_HAL_GetSystemClockFreq(SCG,SCG_SYSTEM_CLOCK_BUS);
            break;
        case SLOW_CLOCK:
            *frequency = SCG_HAL_GetSystemClockFreq(SCG,SCG_SYSTEM_CLOCK_SLOW);
            break;
        case CLKOUT_CLOCK:
            *frequency = CLOCK_SYS_GetSimClkOutFreq(SIM);
            break;

        /* Other internal clocks used by peripherals. */
        case SIRC_CLOCK:
            *frequency = SCG_HAL_GetSircFreq(SCG);
            break;
        case FIRC_CLOCK:
            *frequency = SCG_HAL_GetFircFreq(SCG);
            break;
        case SOSC_CLOCK:
            *frequency = SCG_HAL_GetSysOscFreq(SCG);
            break;
        case SPLL_CLOCK:
            *frequency = SCG_HAL_GetSysPllFreq(SCG);
            break;
        case RTC_CLKIN_CLOCK:
            *frequency = SCG_HAL_GetRtcClkInFreq(SCG);
            break;
        case SCG_CLKOUT_CLOCK:
            *frequency = CLOCK_SYS_GetScgClkOutFreq(SCG);
            break;
        default:
            returnCode = CLOCK_MANAGER_NO_SUCH_CLOCK_NAME;
            break;
    }
    return returnCode;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : CLOCK_SYS_GetSimClockFreq
 * Description   : This function returns the frequency of a given clock from SIM
 *
 *END**************************************************************************/
static clock_manager_error_code_t CLOCK_SYS_GetSimClockFreq(clock_names_t clockName,
                                                            uint32_t *frequency)
{
    clock_manager_error_code_t returnCode = CLOCK_MANAGER_SUCCESS;
    /* if memory is allocated it must be cleared. */
    if (frequency != NULL)
    {
        *frequency = 0U;
    }

    switch (clockName)
    {
        /* SIM clocks */
        case SIM_FTM0_CLOCKSEL:
            *frequency = SIM_HAL_GetTClkFreq(SIM,(uint8_t)SIM_HAL_GetFtmExternalClkPinMode(SIM, 0U));
            break;
        case SIM_FTM1_CLOCKSEL:
            *frequency = SIM_HAL_GetTClkFreq(SIM,(uint8_t)SIM_HAL_GetFtmExternalClkPinMode(SIM, 1U));
            break;
        case SIM_FTM2_CLOCKSEL:
            *frequency = SIM_HAL_GetTClkFreq(SIM,(uint8_t)SIM_HAL_GetFtmExternalClkPinMode(SIM, 2U));
            break;
        case SIM_FTM3_CLOCKSEL:
            *frequency = SIM_HAL_GetTClkFreq(SIM,(uint8_t)SIM_HAL_GetFtmExternalClkPinMode(SIM, 3U));
            break;
        case SIM_CLKOUTSELL:
            *frequency = CLOCK_SYS_GetSimClkOutFreq(SIM);
            break;
        case SIM_RTCCLK_CLOCK:
            *frequency = CLOCK_SYS_GetSimRtcClkFreq(SIM);
            break;
        case SIM_LPO_CLOCK:
            *frequency = SIM_HAL_GetLpoFreq(SIM);
            break;
        case SIM_LPO_1K_CLOCK:
            *frequency = SIM_HAL_GetLpo1KFreq(SIM);
            break;
        case SIM_LPO_32K_CLOCK:
            *frequency = SIM_HAL_GetLpo32KFreq(SIM);
            break;
        case SIM_LPO_128K_CLOCK:
            if (PMC_HAL_GetLpoMode(PMC))
            {
                *frequency = LPO_128K_FREQUENCY;
            }
            break;
        case SIM_EIM_CLOCK:
            if (!SIM_HAL_GetEimClockGate(SIM))
            {

                /* EIM is not clocked. */
                returnCode = CLOCK_MANAGER_NO_CLOCK;
            }
            break;
        case SIM_ERM_CLOCK:
            if (!SIM_HAL_GetErmClockGate(SIM))
            {
                /* ERM is not clocked. */
                returnCode = CLOCK_MANAGER_NO_CLOCK;
            }
            break;
        case SIM_DMA_CLOCK:
            if (!SIM_HAL_GetDmaClockGate(SIM))
            {
                /* DMA is not clocked. */
                returnCode = CLOCK_MANAGER_NO_CLOCK;
            }
            break;
        case SIM_MPU_CLOCK:
            if (!SIM_HAL_GetMpuClockGate(SIM))
            {
                /* MPU is not clocked. */
                returnCode = CLOCK_MANAGER_NO_CLOCK;
            }
            break;
        case SIM_MSCM_CLOCK:
            if (!SIM_HAL_GetMscmClockGate(SIM))
            {
                /* MSCM is not clocked. */
                returnCode = CLOCK_MANAGER_NO_CLOCK;
            }
            break;
        default:
            returnCode = CLOCK_MANAGER_NO_SUCH_CLOCK_NAME;
            break;
    }
    return returnCode;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : CLOCK_SYS_GetPccClockFreq
 * Description   : This function returns the clock frequency of peripheral functional clock.
 *END**************************************************************************/
static clock_manager_error_code_t CLOCK_SYS_GetPccClockFreq(clock_names_t clockName,
                                                            uint32_t *frequency)
{   bool clockMode;
    clock_manager_error_code_t returnCode = CLOCK_MANAGER_SUCCESS;
    /* if memory is allocated it must be cleared. */
    if (frequency != NULL)
    {
        *frequency = 0U;
    }

    /* Invalid PCC clock names */
    if ((clockName <= SIM_END_OF_CLOCKS) ||
        (clockName == PCC_END_OF_BUS_CLOCKS) ||
        (clockName == PCC_END_OF_SYS_CLOCKS) ||
        (clockName == PCC_END_OF_SLOW_CLOCKS) ||
        (clockName == PCC_END_OF_ASYNCH_DIV1_CLOCKS) ||
        (clockName == PCC_END_OF_ASYNCH_DIV2_CLOCKS))
    {
        returnCode = CLOCK_MANAGER_NO_SUCH_CLOCK_NAME;
    }
    else
    {
        clockMode = PCC_HAL_GetClockMode(PCC,clockName);
        if (!clockMode)
        {
            /* Module is not clocked. */
            returnCode = CLOCK_MANAGER_NO_CLOCK;
        }
        /* Peripheral is clocked. Peripheral has interface clock only: SYS_CLK, BUS_CLK or SLOW_CLK. */
        else if (clockName < PCC_END_OF_SLOW_CLOCKS)
        {
            /* Nothing to do for peripherals that have no functional clock. */
        }
        /* Peripheral is clocked. Peripheral has functional clock.
         * The functional clock comes from asynchronous sources 1th divider. */
        else if (clockName < PCC_END_OF_ASYNCH_DIV1_CLOCKS)
        {
            *frequency = CLOCK_SYS_GetPeripheralClock(clockName,SCG_ASYNC_CLOCK_DIV1);
        }
        /* Peripheral is clocked. Peripheral has functional clock.
         * The functional clock comes from asynchronous sources 2th divider. */
        else  /* clockName < PCC_END_OF_ASYNCH_DIV2_CLOCKS */
        {
            *frequency = CLOCK_SYS_GetPeripheralClock(clockName,SCG_ASYNC_CLOCK_DIV2);
        }
    }
    return returnCode;
}




/*FUNCTION**********************************************************************
 *
 * Function Name : CLOCK_SYS_GetFreq
 * Description   : This function returns the frequency of a given clock
 *
 * Implements CLOCK_SYS_GetFreq_Activity
 *END**************************************************************************/
clock_manager_error_code_t CLOCK_SYS_GetFreq(clock_names_t clockName,
                                             uint32_t *frequency)
{
    clock_manager_error_code_t returnCode;

    /* Frequency of the clock name from SCG */
    if (clockName < SCG_END_OF_CLOCKS)
    {
        returnCode = CLOCK_SYS_GetScgClockFreq(clockName, frequency);
    }
    /* Frequency of the clock name from SIM */
    else if (clockName < SIM_END_OF_CLOCKS)
    {
        returnCode = CLOCK_SYS_GetSimClockFreq(clockName, frequency);
    }
    /* Frequency of the clock name from PCC */
    else if (clockName < PCC_END_OF_CLOCKS)
    {
        returnCode = CLOCK_SYS_GetPccClockFreq(clockName, frequency);
    }
    /* Invalid clock name */
    else
    {
        returnCode = CLOCK_MANAGER_NO_SUCH_CLOCK_NAME;
    }
    return returnCode;
}

/*FUNCTION**********************************************************************
 * Function Name : CLOCK_SYS_GetPeripheralClock
 * Description   : Internal function used by CLOCK_SYS_GetFreq function
 *END**************************************************************************/

static uint32_t CLOCK_SYS_GetPeripheralClock(clock_names_t clockName, scg_async_clock_type_t divider)
{
    uint32_t frequency = 0;
    peripheral_clock_frac_t  fracValue = PCC_HAL_GetFracValueSel(PCC,clockName);
    peripheral_clock_divider_t divValue = PCC_HAL_GetDividerSel(PCC,clockName);

    /* Check division factor */
    if (((uint32_t)fracValue) <= ((uint32_t)divValue))
    {
        /* Check clock gate */
        if (PCC_HAL_GetClockMode(PCC,clockName))
        {
            /* Check clock source */
            switch (PCC_HAL_GetClockSourceSel(PCC,clockName))
            {
                case CLK_SRC_SOSC:
                    frequency = SCG_HAL_GetSysOscAsyncFreq(SCG,divider);
                    break;
                case CLK_SRC_SIRC:
                    frequency = SCG_HAL_GetSircAsyncFreq(SCG,divider);
                    break;
                case CLK_SRC_FIRC:
                    frequency = SCG_HAL_GetFircAsyncFreq(SCG,divider);
                    break;
                case CLK_SRC_SPLL:
                    frequency = SCG_HAL_GetSysPllAsyncFreq(SCG,divider);
                    break;
                default:
                    frequency = 0;
                    break;
            }      
            frequency = frequency / (((uint32_t)divValue)+1U);
            frequency = frequency * (((uint32_t)fracValue)+1U);
        }
    }
    return frequency;
}

/*FUNCTION**********************************************************************
 * Function Name : CLOCK_SYS_GetCurrentRunMode
 * Description   : Internal function used by CLOCK_SYS_SetScgConfiguration function
 *END**************************************************************************/
static scg_system_clock_mode_t CLOCK_SYS_GetCurrentRunMode(const SMC_Type * smc_base)
{
    scg_system_clock_mode_t mode;

    /* Read and convert from SMC run mode to SCG defines*/
    switch (SMC_HAL_GetPowerModeStatus(smc_base))
    {
        /* High speed run mode */
        case STAT_HSRUN:
            mode = SCG_SYSTEM_CLOCK_MODE_HSRUN;
            break;
        /* Run mode */
        case STAT_RUN:
            mode = SCG_SYSTEM_CLOCK_MODE_RUN;
            break;
        /* Very low power run mode */
        case STAT_VLPR:
            mode = SCG_SYSTEM_CLOCK_MODE_VLPR;
            break;
        /* This should never happen - core has to be in some run mode to execute code */
        default:
            mode = SCG_SYSTEM_CLOCK_MODE_NONE;
            break;
    }

    return mode;
}

/*FUNCTION**********************************************************************
 * Function Name : CLOCK_SYS_SwitchSystemClock
 * Description   : Internal function used by CLOCK_SYS_SetScgConfiguration function
 *END**************************************************************************/
static bool CLOCK_SYS_SwitchSystemClock(scg_system_clock_src_t to_clk) 
{
    scg_system_clock_config_t config;
    scg_system_clock_mode_t run_mode;
    bool retValue = false;
    uint32_t timeout;

    /* Check destination clock */
    if ( to_clk != SCG_SYSTEM_CLOCK_SRC_NONE )
    {
        /* Get & Convert Run mode from SMC to SCG defines*/
        run_mode = CLOCK_SYS_GetCurrentRunMode(SMC);

        if ( run_mode != SCG_SYSTEM_CLOCK_MODE_NONE )
        {
            /* Read system clock configuration*/
            SCG_HAL_GetSystemClockConfig(SCG, run_mode, &config);

            /* Use to_clk as system clock source*/
            config.src = to_clk;

            /* Update run mode configuration */
            SCG_HAL_SetSystemClockConfig(SCG, run_mode, &config);
    
            /* Wait for system clock to switch. */
            timeout = 100U;
            do {
                /* Read new system clock configuration. */
                SCG_HAL_GetSystemClockConfig(SCG, SCG_SYSTEM_CLOCK_MODE_CURRENT, &config);
                timeout--;
                
            } while ((timeout > 0U) && (config.src != to_clk));

            if ( config.src == to_clk )
            {
                retValue = true;
            }
        }
    }

    return retValue;
}
/*FUNCTION**********************************************************************
 * Function Name : CLOCK_SYS_GetSimClkOutFreq
 * Description   : Internal function used by CLOCK_SYS_GetFreq function
 *END**************************************************************************/
static uint32_t CLOCK_SYS_GetSimClkOutFreq(const SIM_Type * base)
{
    uint32_t frequency;
    
    /* Check CLKOUT Select */
    sim_clock_out_config_t sim_clkout_config;
    SIM_HAL_GetClkoutConfig(base, &sim_clkout_config);

    if (sim_clkout_config.enable)
    {
        switch (sim_clkout_config.source)
        {              
            case SIM_CLKOUT_SEL_SYSTEM_SCG_CLKOUT:
                frequency = CLOCK_SYS_GetScgClkOutFreq(SCG);
                break;
            case SIM_CLKOUT_SEL_SYSTEM_SOSC_DIV2_CLK:
                frequency = SCG_HAL_GetSysOscAsyncFreq(SCG, SCG_ASYNC_CLOCK_DIV2);
                break;
            case SIM_CLKOUT_SEL_SYSTEM_SIRC_DIV2_CLK:
                frequency = SCG_HAL_GetSircAsyncFreq(SCG, SCG_ASYNC_CLOCK_DIV2);
                break;
            case SIM_CLKOUT_SEL_SYSTEM_FIRC_DIV2_CLK:
                frequency = SCG_HAL_GetFircAsyncFreq(SCG, SCG_ASYNC_CLOCK_DIV2);
                break;
            case SIM_CLKOUT_SEL_SYSTEM_SPLL_DIV2_CLK:
                frequency = SCG_HAL_GetSysPllAsyncFreq(SCG, SCG_ASYNC_CLOCK_DIV2);
                break;
            case SIM_CLKOUT_SEL_SYSTEM_LPO_128K_CLK:
                frequency = LPO_128K_FREQUENCY;
                break;
            case SIM_CLKOUT_SEL_SYSTEM_LPO_CLK:
                frequency = SIM_HAL_GetLpoFreq(SIM);;
                break;
            case SIM_CLKOUT_SEL_SYSTEM_RTC_CLK:
                frequency = CLOCK_SYS_GetSimRtcClkFreq(SIM);
                break;
            default:
                /* Invalid SIM CLKOUT selection.*/
                frequency = 0U;
                break;
        }

        /* Apply Divide Ratio */
        frequency /= (((uint32_t)sim_clkout_config.divider) + 1U);
    }
    else
    {
        /* Output disabled. */
        frequency = 0U;
    }
    
    return frequency;
}
/*FUNCTION**********************************************************************
 * Function Name : CLOCK_SYS_GetScgClkOutFreq
 * Description   : Internal function used by CLOCK_SYS_GetFreq function
 *END**************************************************************************/
static uint32_t CLOCK_SYS_GetScgClkOutFreq(const SCG_Type * base)
{
    uint32_t frequency;
    
    switch (SCG_HAL_GetClockoutSourceSel(base))
    {
        case SCG_CLOCKOUT_SRC_SCG_SLOW:
            frequency = SCG_HAL_GetSystemClockFreq(SCG,SCG_SYSTEM_CLOCK_SLOW);
            break;
        case SCG_CLOCKOUT_SRC_SOSC:
            frequency = SCG_HAL_GetSysOscFreq(SCG);
            break;
        case SCG_CLOCKOUT_SRC_SIRC:
            frequency = SCG_HAL_GetSircFreq(SCG);
            break;
        case SCG_CLOCKOUT_SRC_FIRC:
            frequency = SCG_HAL_GetFircFreq(SCG);
            break;
        case SCG_CLOCKOUT_SRC_SPLL:
            frequency = SCG_HAL_GetSysPllFreq(SCG);
            break;
        default:
            /* Invalid SCG CLKOUT selection.*/
            frequency = 0U;
            break;
    }

    return frequency;
}

/*FUNCTION**********************************************************************
 * Function Name : CLOCK_SYS_GetSimRtcClkFreq
 * Description   : Internal function used by CLOCK_SYS_GetFreq function
 *END**************************************************************************/
static uint32_t CLOCK_SYS_GetSimRtcClkFreq(const SIM_Type * base)
{
    uint32_t frequency;
    
    /* Check RTCCLK Select */
    switch (SIM_HAL_GetRtcClkSrc(base))
    {
        case SIM_RTCCLK_SEL_SOSCDIV1_CLK:
            frequency = SCG_HAL_GetSysOscAsyncFreq(SCG,SCG_ASYNC_CLOCK_DIV1);
            break;
        case SIM_RTCCLK_SEL_LPO_32K:
            frequency = SIM_HAL_GetLpo32KFreq(SIM);
            break;
        case SIM_RTCCLK_SEL_RTC_CLKIN:
            frequency = SCG_HAL_GetRtcClkInFreq(SCG);
            break;
        case SIM_RTCCLK_SEL_FIRCDIV1_CLK:
            frequency = SCG_HAL_GetFircAsyncFreq(SCG,SCG_ASYNC_CLOCK_DIV1);
            break;
        default:
            /* Invalid RTCCLK selection.*/
            frequency = 0U;
            break;
    }

    return frequency;
}

/*******************************************************************************
 * EOF
 ******************************************************************************/
