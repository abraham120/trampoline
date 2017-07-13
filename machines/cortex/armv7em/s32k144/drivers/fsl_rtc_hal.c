/*
 * Copyright (c) 2014 - 2015, Freescale Semiconductor, Inc.
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

#include "fsl_rtc_hal.h"

/*******************************************************************************
 * Code
 ******************************************************************************/

/*FUNCTION**********************************************************************
 *
 * Function Name RTC_HAL_Init
 * Description   This function initializes the RTC instance
 * Return        RTC_STATUS_SUCCESS if the operation was successful, RTC_STATUS_ERROR
 *               if there was a problem or RTC_STATUS_LOCKED if at least one
 *               register is locked
 *
 *END**************************************************************************/

rtc_status_t RTC_HAL_Init(RTC_Type * const base)
{
    rtc_status_t statusCode = RTC_STATUS_SUCCESS;

    /* Check if the registers are locked */
    if ((base->LR & 0xFFu) != 0xFFu)
    {
        statusCode =  RTC_STATUS_LOCKED;
    }

    /* Set all registers to default values, except for RTC IER */
    /* Disable all interrupts */
    base->IER = 0UL;
    /* Clear all flags and disable the counter */
    base->SR  = 0UL;
    /* Set Time Seconds Registers to 1 to avoid triggering Time
     * Invalid Interrupt
     */
    base->TSR = 1UL;
    /* Clear Time Prescaler Register */
    base->TPR = 0UL;
    /* Clear Time Alarm Register */
    base->TAR = 0UL;
    /* Set Configuration Register to reset value */
    base->CR  = 0UL;
    /* Set Lock Register to default value */
    base->LR  = 0xFFUL;

    /* Check if the configuration was successful */
    if (RTC_HAL_GetTimeInvalidFlag(base) == true)
    {
        statusCode = RTC_STATUS_ERROR;
    }
    /* Return the exit code */
    return statusCode;
}


/*FUNCTION**********************************************************************
 *
 * Function Name RTC_HAL_Enable
 * Description   This function enables the RTC counter
 * Return        RTC_STATUS_SUCCESS if the operation was successful, RTC_STATUS_ERROR
 *               if there was a problem
 *
 *END**************************************************************************/
rtc_status_t RTC_HAL_Enable(RTC_Type * const base)
{
    rtc_status_t statusCode = RTC_STATUS_SUCCESS;

    /* Check if the RTC counter is enabled or if the Time setup is invalid */
    if ((RTC_HAL_GetTimeCounterEnable(base) == true) || (RTC_HAL_GetTimeInvalidFlag(base) == true))
    {
        statusCode = RTC_STATUS_ERROR;
    }
    else
    {
        /* Enable oscillator and seconds counter */
        RTC_HAL_SetTimeCounterEnable(base, true);
    }
    /* Return the exit code */
    return statusCode;
}

/*FUNCTION**********************************************************************
 *
 * Function Name RTC_HAL_Disable
 * Description   This function disables the RTC counter
 * Return        RTC_STATUS_SUCCESS if the operation was successful, RTC_STATUS_ERROR
 *               if there was a problem
 *
 *END**************************************************************************/
rtc_status_t RTC_HAL_Disable(RTC_Type * const base)
{
    if (RTC_HAL_GetTimeCounterEnable(base) == true)
    {
        RTC_HAL_SetTimeCounterEnable(base, false);
    }

    /* Read TCE bit to check if the counter is really disabled and return the
     * corresponding result.
     *  -   Error if the timer is still enabled (The register can be locked)
     *  -   Success if the timer is disabled
     */
    return (RTC_HAL_GetTimeCounterEnable(base) ? RTC_STATUS_ERROR : RTC_STATUS_SUCCESS);
}

/*FUNCTION**********************************************************************
 *
 * Function Name RTC_HAL_SetTimeSecondsRegister
 * Description
 *        This function along with SetTimePrescalerRegister will help you set
 *        the starting time at a specified value.
 *        The write will fail if the Time Counter is enabled and will return
 *        RTC_STATUS_ERROR, otherwise the return will be RTC_STATUS_SUCCESS
 *
 * Return RTC_STATUS_SUCCESS if the write is succeeded or RTC_STATUS_ERROR if
 *        the counter is enabled.
 *
 *END**************************************************************************/
rtc_status_t RTC_HAL_SetTimeSecondsRegister(RTC_Type * const base, uint32_t seconds)
{
    rtc_status_t statusCode = RTC_STATUS_SUCCESS;

    if (RTC_HAL_GetTimeCounterEnable(base) == true)
    {
        statusCode = RTC_STATUS_ERROR;
    }
    else
    {
        uint32_t tmp = base->TSR;
        tmp &= ~(RTC_TSR_TSR_MASK);
        tmp |= RTC_TSR_TSR(seconds);
        base->TSR = tmp;
        statusCode = RTC_STATUS_SUCCESS;
    }
    /* Return the exit code */
    return statusCode;
}

/*FUNCTION**********************************************************************
 *
 * Function Name RTC_HAL_SetTimePrescalerRegister
 * Description
 *        This function along with SetTimeSecondsRegister will help you set
 *        the starting time at a specified value.
 *        The write will fail if the Time Counter is enabled and will return
 *        RTC_STATUS_ERROR, otherwise the return will be RTC_STATUS_SUCCESS
 *
 * Return RTC_STATUS_SUCCESS if the write is succeeded or RTC_STATUS_ERROR if
 *        the counter is enabled.
 *
 *END**************************************************************************/
rtc_status_t RTC_HAL_SetTimePrescalerRegister(RTC_Type * const base, uint16_t value)
{
    rtc_status_t statusCode = RTC_STATUS_SUCCESS;

    if (RTC_HAL_GetTimeCounterEnable(base) == true)
    {
        statusCode = RTC_STATUS_ERROR;
    }
    else
    {
        uint32_t tmp = base->TPR;
        tmp &= ~(RTC_TPR_TPR_MASK);
        tmp |= RTC_TPR_TPR(value);
        base->TPR = tmp;
        statusCode = RTC_STATUS_SUCCESS;
    }
    /* Return the exit code */
    return statusCode;
}

/*FUNCTION**********************************************************************
 *
 * Function Name RTC_HAL_ConfigureRegisterLock
 * Description
 *          This method will allow you to lock the registers. It will return
 *          RTC_STATUS_SUCCESS if the lock was successful or if the register
 *          was already locked, RTC_STATUS_LOCKED if the Lock Register is
 *          already locked and RTC_STATUS_ERROR if the registerToConfig
 *          parameter is not a valid register.
 *
 *
 * Return Status of the operation
 *
 *END**************************************************************************/
rtc_status_t RTC_HAL_ConfigureRegisterLock(RTC_Type * const base, rtc_lock_register_select_t registerToConfig)
{
    rtc_status_t statusCode = RTC_STATUS_SUCCESS;

    /* Check if the Lock Register is already locked,
     * if true, any other register lock status cannot
     * be modified.
     */
    if (RTC_HAL_GetLockRegisterLock(base) == true)
    {
        statusCode = RTC_STATUS_LOCKED;
    }
    else
    {
        /* If the Lock Register is not locked we can
         * configure the register lock.
         */
        switch (registerToConfig)
        {
            case RTC_LOCK_REG_LOCK:
                RTC_HAL_LockRegisterLock(base);
                break;
            case RTC_STATUS_REG_LOCK:
                RTC_HAL_StatusRegisterLock(base);
                break;
            case RTC_CTRL_REG_LOCK:
                RTC_HAL_ControlRegisterLock(base);
                break;
            case RTC_TCE_REG_LOCK:
                RTC_HAL_TimeCompensationLock(base);
                break;
            default:
                /* If the register is not recognized, return error */
                statusCode = RTC_STATUS_ERROR;
                break;
        }
    }
    /* Return the exit code */
    return statusCode;
}

/*FUNCTION**********************************************************************
 *
 * Function Name RTC_HAL_IsRegisterLocked
 * Description
 *              This method will get the register lock status
 *
 *
 * Return True if the register is locked, false if not
 *
 *END**************************************************************************/
bool RTC_HAL_IsRegisterLocked(const RTC_Type * const base, rtc_lock_register_select_t reg)
{
    bool state = false;

    switch (reg)
    {
        case RTC_LOCK_REG_LOCK:
            state = RTC_HAL_GetLockRegisterLock(base);
            break;
        case RTC_CTRL_REG_LOCK:
            state = RTC_HAL_GetControlRegisterLock(base);
            break;
        case RTC_STATUS_REG_LOCK:
            state = RTC_HAL_GetStatusRegisterLock(base);
            break;
        case RTC_TCE_REG_LOCK:
            state = RTC_HAL_GetTimeCompensationLock(base);
            break;
    }
    /* Return the exit code */
    return state;
}

/*FUNCTION**********************************************************************
 *
 * Function Name RTC_HAL_ConfigureClockOut
 * Description
 *          This method will allow you to configure the RTC Clock out pin.
 *          It will return RTC_STATUS_SUCCESS if the configuration was successful
 *          RTC_STATUS_LOCKED if the Control Register is locked.
 *
 * Return Status of the operation
 *
 *END**************************************************************************/
rtc_status_t RTC_HAL_ConfigureClockOut(RTC_Type * const base, rtc_clk_out_config_t config)
{
    rtc_status_t statusCode = RTC_STATUS_SUCCESS;

    /* Check if the Lock Register is already locked,
     * if true, any other register lock status cannot
     * be modified.
     */
    if (RTC_HAL_GetControlRegisterLock(base) == true)
    {
        statusCode = RTC_STATUS_LOCKED;
    }
    else
    {
        switch (config)
        {
            case RTC_CLKOUT_DISABLED:
                /* Disable the clock out pin */
                base->CR &= ~RTC_CR_CPE_MASK;
                break;
            case RTC_CLKOUT_SRC_TSIC:
                /* Select clock out source as Time Seconds Interrupt and enable the pin */
                base->CR &= ~(RTC_CR_CPE_MASK | RTC_CR_CPS_MASK);
                base->CR |= (RTC_CR_CPE(1U) | RTC_CR_CPS(0U));
                break;
            case RTC_CLKOUT_SRC_32KHZ:
                /* Select clock out source as the 32 KHz clock and enable the pin */
                base->CR &= ~(RTC_CR_CPE_MASK | RTC_CR_CPS_MASK);
                base->CR |= (RTC_CR_CPE(1U) | RTC_CR_CPS(1U));
                break;
        }
    }
    /* Return the exit code */
    return statusCode;
}

/*******************************************************************************
 * EOF
 ******************************************************************************/
