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
 * @file fsl_wdog_hal.h
 */

#include "fsl_wdog_hal.h"


/*******************************************************************************
 * Code
 ******************************************************************************/
/*FUNCTION**********************************************************************
 *
 * Function Name : WDOG_HAL_Init
 * Description   : Initializes the WDOG to known state.
 *
 * Implements    : WDOG_HAL_Init_Activity
 *END**************************************************************************/
void WDOG_HAL_Init(WDOG_Type *base)
{
    WDOG_UNLOCK(base);

    /* Set all R/W registers to their reset value. Exception: set UPDATE bit in
     * order to allow further configurations of the WDOG */
    base->CS    = WDOG_CS_UPDATE_MASK | WDOG_CS_EN_MASK | FSL_FEATURE_WDOG_CS_RESERVED_MASK |
                  WDOG_CS_CLK(WDOG_LPO_CLOCK);
    base->TOVAL = FSL_FEATURE_WDOG_TO_RESET_VALUE;
    base->WIN   = FSL_FEATURE_WDOG_WIN_RESET_VALUE;

    /* Reset interrupt flags */
    base->CS = (base->CS & ~WDOG_CS_FLG_MASK) | WDOG_CS_FLG(1U);
}

/*FUNCTION**********************************************************************
 *
 * Function Name : WDOG_HAL_Config
 * Description   : Configures all WDOG registers.
 *
 * Implements    : WDOG_HAL_Config_Activity
 *END**************************************************************************/
void WDOG_HAL_Config(WDOG_Type *base, const wdog_user_config_t *config)
{
    uint32_t cs = base->CS;

    /* Clear the bits used for configuration */
    cs &= ~(WDOG_CS_WIN_MASK | WDOG_CS_PRES_MASK | WDOG_CS_CLK_MASK |
            WDOG_CS_INT_MASK | WDOG_CS_UPDATE_MASK | WDOG_CS_DBG_MASK |
            WDOG_CS_WAIT_MASK | WDOG_CS_STOP_MASK);
    /* Construct CS register new value */
    cs |= WDOG_CS_WIN(config->winEnable? 1UL : 0UL);
    cs |= WDOG_CS_PRES(config->prescalerEnable? 1UL : 0UL);
    cs |= WDOG_CS_CLK(config->clkSource);
    cs |= WDOG_CS_INT(config->intEnable? 1UL : 0UL);
    cs |= WDOG_CS_UPDATE(config->updateEnable? 1UL : 0UL);
    cs |= WDOG_CS_DBG(config->opMode.debug? 1UL : 0UL);
    cs |= WDOG_CS_WAIT(config->opMode.wait? 1UL : 0UL);
    cs |= WDOG_CS_STOP(config->opMode.stop? 1UL : 0UL);
    /* Reset interrupt flags */
    cs |= WDOG_CS_FLG_MASK;
    /* Enable WDOG */
    cs |= WDOG_CS_EN_MASK;
    
    WDOG_UNLOCK(base);
  
    base->CS    = cs;
    base->TOVAL = config->timeoutValue;
    if (config->winEnable)
    {
        base->WIN = config->windowValue;
    }
}

/*FUNCTION**********************************************************************
 *
 * Function Name : WDOG_HAL_GetConfig
 * Description   : Gets the current WDOG configuration.
 *
 * Implements    : WDOG_HAL_GetConfig_Activity
 *END**************************************************************************/
wdog_user_config_t WDOG_HAL_GetConfig(const WDOG_Type *base)
{
    wdog_user_config_t config;
    uint32_t cs = base->CS;

    /* Construct CS register new value */
    config.winEnable       = ((cs & WDOG_CS_WIN_MASK) != 0U);
    config.prescalerEnable = ((cs & WDOG_CS_PRES_MASK) != 0U);
    config.intEnable       = ((cs & WDOG_CS_INT_MASK) != 0U);
    config.updateEnable    = ((cs & WDOG_CS_UPDATE_MASK) != 0U);
    config.opMode.debug    = ((cs & WDOG_CS_DBG_MASK) != 0U);
    config.opMode.wait     = ((cs & WDOG_CS_WAIT_MASK) != 0U);
    config.opMode.stop     = ((cs & WDOG_CS_STOP_MASK) != 0U);
    config.timeoutValue    = base->TOVAL;
    config.windowValue     = base->WIN;

    switch((cs & WDOG_CS_CLK_MASK) >> WDOG_CS_CLK_SHIFT)
    {
    case 0U:
        config.clkSource = WDOG_BUS_CLOCK;
        break;
    case 1U:
        config.clkSource = WDOG_LPO_CLOCK;
        break;
    case 2U:
        config.clkSource = WDOG_SOSC_CLOCK;
        break;
    case 3U:
        config.clkSource = WDOG_SIRC_CLOCK;
        break;
    default:
        config.clkSource = WDOG_BUS_CLOCK;
        break;
    }

    return config;
}
/*******************************************************************************
 * EOF
 ******************************************************************************/

