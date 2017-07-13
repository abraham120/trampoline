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
 * @file fsl_lpit_driver.c
 *
 * @page misra_violations MISRA-C:2012 violations
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

#include "fsl_lpit_driver.h"

/*******************************************************************************
 * Variables
 ******************************************************************************/

/* Table of base addresses for LPIT instances */
static LPIT_Type * const s_lpitBase[] = LPIT_BASE_PTRS;
/* Table to save LPIT indexes in PCC register map for clock configuration */
static const clock_names_t s_lpitClkNames[LPIT_INSTANCE_COUNT] = {PCC_LPIT0_CLOCK};
/* LPIT functional clock variable which will be updated in some driver functions */
static uint32_t s_lpitSourceClockFrequency[LPIT_INSTANCE_COUNT] = {0};

/******************************************************************************
 * Code
 *****************************************************************************/

/*FUNCTION**********************************************************************
 *
 * Function Name : LPIT_DRV_Init
 * Description   : Initializes LPIT module.
 * This function resets LPIT module, enables the LPIT module, configures LPIT
 * module operation in Debug and DOZE mode. The LPIT configuration structure shall
 * be passed as arguments.
 * This configuration structure affects all timer channels.
 * This function should be called before calling any other LPIT driver function.
 *
 * Implements    : LPIT_DRV_Init_Activity
 *END**************************************************************************/
lpit_status_t LPIT_DRV_Init(uint32_t instance,
                            const lpit_user_config_t *userConfig)
{
#ifdef DEV_ERROR_DETECT
    DEV_ASSERT(instance < LPIT_INSTANCE_COUNT);
    DEV_ASSERT(userConfig != NULL);
#endif

    LPIT_Type * base = s_lpitBase[instance];
    clock_names_t instanceClkName = s_lpitClkNames[instance];
    lpit_status_t reVal = LPIT_STATUS_SUCCESS;

    /* Gets current functional clock frequency of LPIT instance */
    (void)CLOCK_SYS_GetFreq(instanceClkName, &s_lpitSourceClockFrequency[instance]);
    /* Checks the functional clock of LPIT module */
    if (s_lpitSourceClockFrequency[instance] == 0U)
    {
        reVal = LPIT_STATUS_FAIL;
    }
    else
    {
        /* Resets LPIT module */
        LPIT_HAL_Reset(base);
        /* Enables functional clock of LPIT module*/
        LPIT_HAL_Enable(base);
        /* Sets LPIT operation in Debug and DOZE mode*/
        LPIT_HAL_SetTimerRunInDebugCmd(base, userConfig->enableRunInDebug);
        LPIT_HAL_SetTimerRunInDozeCmd(base, userConfig->enableRunInDoze);
    }
    return reVal;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : LPIT_DRV_Deinit
 * Description   : De-initializes LPIT module.
 * This function disables LPIT module.
 * In order to use the LPIT module again, LPIT_DRV_Init must be called.
 *
 * Implements    : LPIT_DRV_Deinit_Activity
 *END**************************************************************************/
lpit_status_t LPIT_DRV_Deinit(uint32_t instance)
{
#ifdef DEV_ERROR_DETECT
    DEV_ASSERT(instance < LPIT_INSTANCE_COUNT);
#endif

    LPIT_Type * base = s_lpitBase[instance];
    lpit_status_t reVal = LPIT_STATUS_SUCCESS;
    clock_names_t instanceClkName = s_lpitClkNames[instance];

    /* Gets current functional clock frequency of LPIT instance */
    (void)CLOCK_SYS_GetFreq(instanceClkName, &s_lpitSourceClockFrequency[instance]);
    /* Checks the functional clock of LPIT module  */
    if (s_lpitSourceClockFrequency[instance] == 0U)
    {
        reVal = LPIT_STATUS_FAIL;
    }
    else
    {
        /* Disables LPIT module functional clock*/
        LPIT_HAL_Disable(base);
    }
    return reVal;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : LPIT_DRV_InitChannel
 * Description   : Initializes LPIT channel.
 * This function initializes the LPIT timers by using a channel, this function
 * configures timer channel chaining, timer channel mode, timer channel period,
 * interrupt generation, trigger source, trigger select, reload on trigger,
 * stop on interrupt and start on trigger.
 * The timer channel number and its configuration structure shall be passed as arguments.
 * Timer channels do not start counting by default after calling this function.
 * The function LPIT_DRV_StartTimerChannels must be called to start the timer channel counting.
 * In order to re-configures the period, call the LPIT_DRV_SetTimerPeriodByUs or
 * LPIT_DRV_SetTimerPeriodByCount.
 *
 * Implements    : LPIT_DRV_InitChannel_Activity
 *END**************************************************************************/
lpit_status_t LPIT_DRV_InitChannel(uint32_t instance,
                                   uint32_t channel,
                                   const lpit_user_channel_config_t * userChannelConfig)
{
#ifdef DEV_ERROR_DETECT
    DEV_ASSERT(instance < LPIT_INSTANCE_COUNT);
    DEV_ASSERT(userChannelConfig != NULL);
    DEV_ASSERT(channel < LPIT_TMR_COUNT);
#endif

    LPIT_Type * base = s_lpitBase[instance];
    lpit_status_t reVal = LPIT_STATUS_SUCCESS;
    const IRQn_Type lpitIrqId[] = LPIT_IRQS;

    /* Checks for the channel 0 cannot be chained*/
    if ((channel == 0U) && (userChannelConfig->chainChannel))
    {
        reVal = LPIT_STATUS_FAIL;
    }
    else
    {
        /* Setups the timer channel chaining  */
        LPIT_HAL_SetTimerChannelChainCmd(base, channel, userChannelConfig->chainChannel);
        /*  Setups the timer channel operation mode */
        LPIT_HAL_SetTimerChannelModeCmd(base, channel, userChannelConfig->timerMode);

        /* Setups timer channel period in microsecond unit */
        reVal = LPIT_DRV_SetTimerPeriodByUs(instance, channel, userChannelConfig->periodUs);

        if (reVal == LPIT_STATUS_VALID_PERIOD)
        {
            /* Setups the timer channel trigger source, trigger select, reload on trigger,
            stop on timeout, start on trigger */
            LPIT_HAL_SetTriggerSelectCmd(base, channel, userChannelConfig->triggerSelect);
            LPIT_HAL_SetTriggerSourceCmd(base, channel, userChannelConfig->triggerSource);
            LPIT_HAL_SetReloadOnTriggerCmd(base, channel, userChannelConfig->enableReloadOnTrigger);
            LPIT_HAL_SetStopOnInterruptCmd(base, channel, userChannelConfig->enableStopOnInterrupt);
            LPIT_HAL_SetStartOnTriggerCmd(base, channel, userChannelConfig->enableStartOnTrigger);
            /* Setups interrupt generation for timer channel */
            if (userChannelConfig->isInterruptEnabled)
            {
                /* Enables interrupt generation */
                LPIT_HAL_EnableInterruptTimerChannels(base, (uint32_t)1U << channel);
                INT_SYS_EnableIRQ(lpitIrqId[channel]);
            }
            else
            {
                /* Disables interrupt generation */
                LPIT_HAL_DisableInterruptTimerChannels(base, (uint32_t)1U << channel);
                INT_SYS_DisableIRQ(lpitIrqId[channel]);
            }
            reVal = LPIT_STATUS_SUCCESS;
        }
        else
        {
            reVal = LPIT_STATUS_FAIL;
        }
    }
    return reVal;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : LPIT_DRV_StartTimerChannels
 * Description   : Starts timer channel counting.
 * This function allows starting timer channels simultaneously .
 * After calling this function, timer channels are going operate depend on mode and
 * control bits which controls timer channel start, reload and restart.
 *
 * Implements    : LPIT_DRV_StartTimerChannels_Activity
 *END**************************************************************************/
void LPIT_DRV_StartTimerChannels(uint32_t instance,
                                 uint32_t mask)
{
#ifdef DEV_ERROR_DETECT
    DEV_ASSERT(instance < LPIT_INSTANCE_COUNT);
    DEV_ASSERT(mask < (1U << LPIT_TMR_COUNT));
#endif

    LPIT_Type * base = s_lpitBase[instance];

    /* Starts timer channel counting */
    LPIT_HAL_StartTimerChannels(base, mask);
}

/*FUNCTION**********************************************************************
 *
 * Function Name : LPIT_DRV_StopTimerChannels
 * Description   : Stop timer channel from counting.
 * This function allows stop timer channels simultaneously from counting.
 * Timer channels reload their periods respectively after the next time
 * they call the LPIT_DRV_StartTimerChannels. Note that: In 32-bit Trigger Accumulator
 * mode, the counter will load on the first trigger rising edge.
 *
 * Implements    : LPIT_DRV_StopTimerChannels_Activity
 *END**************************************************************************/
void LPIT_DRV_StopTimerChannels(uint32_t instance,
                                uint32_t mask)
{
#ifdef DEV_ERROR_DETECT
    DEV_ASSERT(instance < LPIT_INSTANCE_COUNT);
    DEV_ASSERT(mask < (1U << LPIT_TMR_COUNT));
#endif

    LPIT_Type * base = s_lpitBase[instance];

    /* Stops timer channel from counting */
    LPIT_HAL_StopTimerChannels(base, mask);
}

/*FUNCTION**********************************************************************
 *
 * Function Name : LPIT_DRV_SetTimerPeriodByUs
 * Description   : Sets timer channel period in microseconds unit.
 * This function sets the timer channel period in microseconds.
 * The required period makes sense if the operation mode of timer channel
 * is 32 bit periodic counter or dual 16 bit periodic counter.
 * The period range depends on the frequency of the LPIT functional clock and
 * operation mode of timer channel.
 * If the required period is out of range, use the suitable mode if applicable.
 * This function is only valid for one single channel.
 *
 * Implements    : LPIT_DRV_SetTimerPeriodByUs_Activity
 *END**************************************************************************/
lpit_status_t LPIT_DRV_SetTimerPeriodByUs(uint32_t instance,
                                          uint32_t channel,
                                          uint32_t us)
{
#ifdef DEV_ERROR_DETECT
    DEV_ASSERT(instance < LPIT_INSTANCE_COUNT);
    DEV_ASSERT(channel < LPIT_TMR_COUNT);
#endif

    LPIT_Type * base = s_lpitBase[instance];
    lpit_timer_modes_t timerMode;
    lpit_status_t reVal = LPIT_STATUS_VALID_PERIOD;
    uint64_t count;
    clock_names_t instanceClkName = s_lpitClkNames[instance];

    /* Gets current functional clock frequency of LPIT instance */
    (void)CLOCK_SYS_GetFreq(instanceClkName, &s_lpitSourceClockFrequency[instance]);
    /* Calculates the count value, assign it to timer channel counter register.*/
    count = ((uint64_t)us) * s_lpitSourceClockFrequency[instance];
    count = (count / 1000000U) - 1U;
    /* Gets current timer channel operation mode */
    timerMode = LPIT_HAL_GetTimerChannelModeCmd(base, channel);
    /* Checks whether the count is valid with timer channel operation mode */
    if (count <= MAX_PERIOD_COUNT)
    {
        if (timerMode == LPIT_DUAL_PERIODIC_COUNTER)
        {
            if (count > MAX_PERIOD_COUNT_IN_DUAL_16BIT_MODE)
            {
                reVal = LPIT_STATUS_INVALID_PERIOD;
            }
            else
            {
                if (count > (MAX_PERIOD_COUNT_IN_DUAL_16BIT_MODE / 2U))
                {
                    /* Calculates the count value for dual 16 bit periodic counter mode */
                    count = (((count - (MAX_PERIOD_COUNT_IN_DUAL_16BIT_MODE / 2U)) << 16U) \
                            | (MAX_PERIOD_COUNT_IN_DUAL_16BIT_MODE / 2U));
                }
            }
        }
    }
    else
    {
        reVal = LPIT_STATUS_INVALID_PERIOD;
    }
    if (reVal == LPIT_STATUS_VALID_PERIOD)
    {
        /* Sets the timer channel period in count unit */
        LPIT_HAL_SetTimerPeriodByCount(base, channel, (uint32_t)count);
    }
    return reVal;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : LPIT_DRV_GetTimerPeriodByUs
 * Description   : Gets the timer channel period in microseconds.
 * The returned period here makes sense if the operation mode of timer channel
 * is 32 bit periodic counter or dual 16 bit periodic counter.
 *
 * Implements    : LPIT_DRV_GetTimerPeriodByUs_Activity
 *END**************************************************************************/
uint64_t LPIT_DRV_GetTimerPeriodByUs(uint32_t instance,
                                     uint32_t channel)
{
#ifdef DEV_ERROR_DETECT
    DEV_ASSERT(instance < LPIT_INSTANCE_COUNT);
    DEV_ASSERT(channel < LPIT_TMR_COUNT);
#endif

    const LPIT_Type * base = s_lpitBase[instance];
    lpit_timer_modes_t timerMode;
    uint64_t currentPeriod;
    clock_names_t instanceClkName = s_lpitClkNames[instance];

    /* Gets current functional clock frequency of LPIT instance */
    (void)CLOCK_SYS_GetFreq(instanceClkName, &s_lpitSourceClockFrequency[instance]);
    /* Gets current timer channel period in count.*/
    currentPeriod = LPIT_HAL_GetTimerPeriodByCount(base, channel);
    /* Gets current timer channel operation mode */
    timerMode = LPIT_HAL_GetTimerChannelModeCmd(base, channel);

    if (timerMode == LPIT_DUAL_PERIODIC_COUNTER)
    {
        /* Converts period from count unit to microseconds unit for dual 16 bit periodic counter mode.*/
        currentPeriod = ((((currentPeriod & ((MAX_PERIOD_COUNT_IN_DUAL_16BIT_MODE / 2U) << 16U)) >> 16U) \
                        + (currentPeriod & (MAX_PERIOD_COUNT_IN_DUAL_16BIT_MODE / 2U)) + 1U) * 1000000U) \
                        / s_lpitSourceClockFrequency[instance];
    }
    else
    {
        /* Converts period from count unit to microseconds unit for other modes */
        currentPeriod = ((currentPeriod + 1U) * 1000000U) / s_lpitSourceClockFrequency[instance];
    }
    return currentPeriod;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : LPIT_DRV_GetCurrentTimerUs
 * Description   : Gets current timer channel counting value in microseconds unit.
 * This function returns an absolute time stamp in microseconds.
 * One common use of this function is to measure the running time of a part of
 * code. Call this function at both the beginning and end of code. The time
 * difference between these two time stamps is the running time.
 * The return counting value here makes sense if the operation mode of timer channel
 * is 32 bit periodic counter or dual 16 bit periodic counter or 32-bit trigger input capture.
 * Need to make sure the running time will not exceed the timer channel period.
 *
 * Implements    : LPIT_DRV_GetCurrentTimerUs_Activity
 *END**************************************************************************/
uint64_t LPIT_DRV_GetCurrentTimerUs(uint32_t instance,
                                    uint32_t channel)
{
#ifdef DEV_ERROR_DETECT
    DEV_ASSERT(instance < LPIT_INSTANCE_COUNT);
    DEV_ASSERT(channel < LPIT_TMR_COUNT);
#endif

    const LPIT_Type * base = s_lpitBase[instance];
    lpit_timer_modes_t timerMode;
    uint64_t currentTime;
    clock_names_t instanceClkName = s_lpitClkNames[instance];

    /* Gets current functional clock frequency of LPIT instance */
    (void)CLOCK_SYS_GetFreq(instanceClkName, &s_lpitSourceClockFrequency[instance]);
    /* Gets current timer channel counting value */
    currentTime = LPIT_HAL_GetCurrentTimerCount(base, channel);
    /* Gets current timer channel operation mode */
    timerMode = LPIT_HAL_GetTimerChannelModeCmd(base, channel);

    if (timerMode == LPIT_DUAL_PERIODIC_COUNTER)
    {
        /* Converts counting value to microseconds unit for dual 16 bit periodic counter mode.*/
        currentTime = ((((currentTime & ((MAX_PERIOD_COUNT_IN_DUAL_16BIT_MODE / 2U) << 16U)) >> 16U) \
                      + (currentTime & (MAX_PERIOD_COUNT_IN_DUAL_16BIT_MODE / 2U))) * 1000000U) \
                      / s_lpitSourceClockFrequency[instance];
    }
    else
    {
        /* Converts counting value to microseconds unit for other modes */
        currentTime = (currentTime * 1000000U) / s_lpitSourceClockFrequency[instance];
    }
    return currentTime;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : LPIT_DRV_SetTimerPeriodByCount
 * Description   : Sets the timer channel period in count unit.
 * This function sets the timer channel period in count unit.
 * The period range depends on the frequency of the LPIT functional clock and
 * operation mode of timer channel.
 * If the required period is out of range, use the suitable mode if applicable.
 * Timer channel begins counting from the value that is set by this function.
 * The counter period of a running timer channel can be modified by first setting
 * a new load value, the value will be loaded after the timer channel expires.
 * To abort the current cycle and start a timer channel period with the new value,
 * the timer channel must be disabled and enabled again.
 *
 * Implements    : LPIT_DRV_SetTimerPeriodByCount_Activity
 *END**************************************************************************/
lpit_status_t LPIT_DRV_SetTimerPeriodByCount(uint32_t instance,
                                             uint32_t channel,
                                             uint32_t count)
{
#ifdef DEV_ERROR_DETECT
    DEV_ASSERT(instance < LPIT_INSTANCE_COUNT);
    DEV_ASSERT(channel < LPIT_TMR_COUNT);
#endif

    LPIT_Type * base = s_lpitBase[instance];
    lpit_timer_modes_t timerMode;
    lpit_status_t reVal = LPIT_STATUS_VALID_PERIOD;
    uint32_t tmp = count;

    /* Gets current timer channel operation mode */
    timerMode = LPIT_HAL_GetTimerChannelModeCmd(base, channel);
    /* If timer channel operation mode is dual 16 bit counter, check whether the count is valid */
    if (timerMode == LPIT_DUAL_PERIODIC_COUNTER)
    {
        if (tmp > MAX_PERIOD_COUNT_IN_DUAL_16BIT_MODE)
        {
            reVal = LPIT_STATUS_INVALID_PERIOD;
        }
        else
        {
            if (tmp > (MAX_PERIOD_COUNT_IN_DUAL_16BIT_MODE / 2U))
            {
                /* Calculates the count value for dual 16 bit periodic counter mode */
                tmp = (((tmp - (MAX_PERIOD_COUNT_IN_DUAL_16BIT_MODE / 2U)) << 16U) \
                        | (MAX_PERIOD_COUNT_IN_DUAL_16BIT_MODE / 2U));
            }
        }
    }
    if (reVal == LPIT_STATUS_VALID_PERIOD)
    {
        /* Sets the timer channel period in count unit */
        LPIT_HAL_SetTimerPeriodByCount(base, channel, tmp);
    }
    return reVal;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : LPIT_DRV_GetTimerPeriodByCount
 * Description   : Gets the current timer channel period in count unit.
 *
 * Implements    : LPIT_DRV_GetTimerPeriodByCount_Activity
 *END**************************************************************************/
uint32_t LPIT_DRV_GetTimerPeriodByCount(uint32_t instance,
                                        uint32_t channel)
{
#ifdef DEV_ERROR_DETECT
    DEV_ASSERT(instance < LPIT_INSTANCE_COUNT);
    DEV_ASSERT(channel < LPIT_TMR_COUNT);
#endif

    const LPIT_Type * base = s_lpitBase[instance];
    lpit_timer_modes_t timerMode;
    uint32_t currentPeriod;

    /* Gets current timer channel period by count.*/
    currentPeriod = LPIT_HAL_GetTimerPeriodByCount(base, channel);
    /* Gets current timer channel operation mode */
    timerMode = LPIT_HAL_GetTimerChannelModeCmd(base, channel);

    if (timerMode == LPIT_DUAL_PERIODIC_COUNTER)
    {
        /* Calculates the period for dual 16 bit periodic counter mode */
        currentPeriod = ((currentPeriod & ((MAX_PERIOD_COUNT_IN_DUAL_16BIT_MODE / 2U) << 16U)) >> 16U) \
                        + (currentPeriod & (MAX_PERIOD_COUNT_IN_DUAL_16BIT_MODE / 2U));
    }
    return currentPeriod;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : LPIT_DRV_GetCurrentTimerCount
 * Description   : Gets the current timer channel counting value in count.
 * This function returns the real-time timer channel counting value, the value in
 * a range from 0 to timer channel period.
 * Need to make sure the running time does not exceed the timer channel period.
 *
 * Implements    : LPIT_DRV_GetCurrentTimerCount_Activity
 *END**************************************************************************/
uint32_t LPIT_DRV_GetCurrentTimerCount(uint32_t instance,
                                       uint32_t channel)
{
#ifdef DEV_ERROR_DETECT
    DEV_ASSERT(instance < LPIT_INSTANCE_COUNT);
    DEV_ASSERT(channel < LPIT_TMR_COUNT);
#endif

    const LPIT_Type * base = s_lpitBase[instance];
    lpit_timer_modes_t timerMode;
    uint32_t currentTime;

    /* Gets current timer channel counting value */
    currentTime = LPIT_HAL_GetCurrentTimerCount(base, channel);
    /* Gets current timer channel operation mode */
    timerMode = LPIT_HAL_GetTimerChannelModeCmd(base, channel);

    if (timerMode == LPIT_DUAL_PERIODIC_COUNTER)
    {
        /* Calculates the current counting value for dual 16 bit periodic counter mode */
        currentTime = ((currentTime & ((MAX_PERIOD_COUNT_IN_DUAL_16BIT_MODE / 2U) << 16U)) >> 16U) \
                      + (currentTime & (MAX_PERIOD_COUNT_IN_DUAL_16BIT_MODE / 2U));
    }
    return currentTime;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : LPIT_DRV_GetInterruptFlagTimerChannels
 * Description   : Gets the current interrupt flag of timer channels.
 * In compare modes, the flag sets to 1 at the end of the timer period.
 * In capture modes, the flag sets to 1 when the trigger asserts.
 *
 * Implements    : LPIT_DRV_GetInterruptFlagTimerChannels_Activity
 *END**************************************************************************/
uint32_t LPIT_DRV_GetInterruptFlagTimerChannels(uint32_t instance,
                                                uint32_t mask)
{
#ifdef DEV_ERROR_DETECT
    DEV_ASSERT(instance < LPIT_INSTANCE_COUNT);
    DEV_ASSERT(mask < (1U << LPIT_TMR_COUNT));
#endif

    const LPIT_Type * base = s_lpitBase[instance];

    /* Gets the interrupt flag for timer channels */
    return LPIT_HAL_GetInterruptFlagTimerChannels(base, mask);
}

/*FUNCTION**********************************************************************
 *
 * Function Name : LPIT_DRV_ClearInterruptFlagTimerChannels
 * Description   : Clears the interrupt flag of timer channels.
 * This function clears the interrupt flag of timer channels after
 * their interrupt event occurred.
 * Implements    : LPIT_DRV_ClearInterruptFlagTimerChannels_Activity
 *END**************************************************************************/
void LPIT_DRV_ClearInterruptFlagTimerChannels(uint32_t instance,
                                              uint32_t mask)
{
#ifdef DEV_ERROR_DETECT
    DEV_ASSERT(instance < LPIT_INSTANCE_COUNT);
    DEV_ASSERT(mask < (1U << LPIT_TMR_COUNT));
#endif

    LPIT_Type * base = s_lpitBase[instance];

    /* Clears the interrupt flag for timer channels */
    LPIT_HAL_ClearInterruptFlagTimerChannels(base, mask);
}

/*******************************************************************************
 * EOF
 ******************************************************************************/
