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
 * @file fsl_lpit_driver.h
 */

#ifndef FSL_LPIT_DRIVER_H
#define FSL_LPIT_DRIVER_H

#include "fsl_lpit_hal.h"
#include "fsl_interrupt_manager.h"
#include "fsl_clock_manager.h"

/*!
 * @defgroup lpit_drv LPIT Driver
 * @ingroup lpit
 * @brief Low Power Interrupt Timer Peripheral Driver.@n
 * LPIT PD provides a set of high-level APIs/services to configure the
 * Low Power Interrupt Timer (LPIT) module.
 * @addtogroup lpit_drv
 * @{
 */

/*******************************************************************************
 * Definitions
 ******************************************************************************/
/*! @brief Max period in count of all operation mode except for dual 16 bit periodic counter mode */
#define MAX_PERIOD_COUNT                    (0xFFFFFFFFU)
/*! @brief Max period in count of dual 16 bit periodic counter mode                               */
#define MAX_PERIOD_COUNT_IN_DUAL_16BIT_MODE (0x1FFFEU)

/*!
 * @brief LPIT configuration structure
 *
 * This structure holds the configuration settings for the LPIT peripheral to
 * enable or disable LPIT module in DEBUG and DOZE mode
 * Implements : lpit_user_config_t_Class
 */
typedef struct
{
    bool enableRunInDebug; /*!< True: Timer channels continue to run in debug mode
                                False: Timer channels stop in debug mode            */
    bool enableRunInDoze;  /*!< True: Timer channels continue to run in doze mode
                                False: Timer channels stop in doze mode             */
} lpit_user_config_t;

/*! @brief Structure to configure the channel timer
 *
 * This structure holds the configuration settings for the LPIT timer channel
 * Implements : lpit_user_channel_config_t_Class
 */
typedef struct
{
    bool chainChannel;                   /*!< True: Channel chaining is enabled
                                              False: Channel chaining is disabled                       */
    bool isInterruptEnabled;             /*!< True: Timer channel interrupt generation is enabled
                                              False : Timer channel interrupt generation is disabled    */
    lpit_timer_modes_t timerMode;        /*!< Operation mode of timer channel                           */
    lpit_trigger_source_t triggerSource; /*!< Selects between internal and external trigger sources     */
    uint32_t triggerSelect;              /*!< Selects one trigger from the set of internal triggers     */
    bool enableReloadOnTrigger;          /*!< True: Timer channel will reload on selected trigger
                                              False: Timer channel will not reload on selected trigger  */
    bool enableStopOnInterrupt;          /*!< True: Timer will stop after timeout
                                              False: Timer channel does not stop after timeout          */
    bool enableStartOnTrigger;           /*!< True: Timer channel starts to decrement when rising edge
                                              on selected trigger is detected.
                                              False: Timer starts to decrement immediately based on
                                              restart condition                                         */
    uint32_t periodUs;                   /*!< Period of timer channel in microsecond unit               */
} lpit_user_channel_config_t;

/*******************************************************************************
 * Enumerations
 ******************************************************************************/

/*!
 * @brief Error codes for LPIT driver
 * Implements : lpit_status_t_Class
 */
typedef enum
{
    LPIT_STATUS_SUCCESS        = 0x00U,        /*!< Operation successful                     */
    LPIT_STATUS_FAIL           = 0x01U,        /*!< Operation failed                         */
    LPIT_STATUS_VALID_PERIOD   = 0x02U,        /*!< Input period of timer channel is valid   */
    LPIT_STATUS_INVALID_PERIOD = 0x03U         /*!< Input period of timer channel is invalid */
} lpit_status_t;

/*******************************************************************************
 * API
 ******************************************************************************/

#if defined(__cplusplus)
extern "C" {
#endif

/*!
 * @name Initialization and De-initialization
 * @{
 */

/*!
 * @brief Initializes the LPIT module.
 *
 * This function resets LPIT module, enables the LPIT module, configures LPIT
 * module operation in Debug and DOZE mode. The LPIT configuration structure shall
 * be passed as arguments.
 * This configuration structure affects all timer channels.
 * This function should be called before calling any other LPIT driver function.
 *
 * This is an example demonstrating how to define a LPIT configuration structure:
   @code
   lpit_user_config_t lpitInit =
   {
        .enableRunInDebug = false,
        .enableRunInDoze = true
   };
   @endcode
 *
 * @param[in] instance LPIT module instance number.
 * @param[in] userConfig Pointer to LPIT configuration structure.
 * @return Operation status
 *         - LPIT_STATUS_SUCCESS:    Operation was successful.
 *         - LPIT_STATUS_FAIL:       Operation failed.
 */
lpit_status_t LPIT_DRV_Init(uint32_t instance,
                            const lpit_user_config_t * userConfig);

/*!
 * @brief De-Initializes the LPIT module.
 *
 * This function disables LPIT module.
 * In order to use the LPIT module again, LPIT_DRV_Init must be called.
 *
 * @param[in] instance LPIT module instance number
 * @return Operation status
 *         - LPIT_STATUS_SUCCESS:    Operation was successful.
 *         - LPIT_STATUS_FAIL:       Operation failed.
 */
lpit_status_t LPIT_DRV_Deinit(uint32_t instance);

/*!
 * @brief Initializes the LPIT channel.
 *
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
 * This is an example demonstrating how to define a LPIT channel configuration structure:
   @code
   lpit_user_channel_config_t lpitTestInit =
   {
        .chainChannel = false,
        .isInterruptEnabled = true,
        .timerMode = LPIT_PERIODIC_COUNTER,
        .triggerSelect = 0x01U,
        .triggerSource = LPIT_TRIGGER_SOURCE_INTERNAL,
        .enableReloadOnTrigger = false,
        .enableStopOnTimeout = false,
        .enableStartOnTrigger = false,
        .periodUs = 1000000
   };
   @endcode
 *
 * @param[in] instance LPIT module instance number
 * @param[in] channel Timer channel number
 * @param[in] userChannelConfig Pointer to LPIT channel configuration structure
 * @return Operation status
 *         - LPIT_STATUS_SUCCESS:    Operation was successful.
 *         - LPIT_STATUS_FAIL:       Operation failed.
 */
lpit_status_t LPIT_DRV_InitChannel(uint32_t instance,
                                   uint32_t channel,
                                   const lpit_user_channel_config_t * userChannelConfig);

/* @} */

/*!
 * @name Timer Start and Stop
 * @{
 */

/*!
 * @brief Starts the timer channel counting.
 *
 * This function allows starting timer channels simultaneously .
 * After calling this function, timer channels are going operate depend on mode and
 * control bits which controls timer channel start, reload and restart.
 *
 * @param[in] instance LPIT module instance number
 * @param[in] mask Timer channels starting mask that decides which channels
 * will be started
 * - For example:
 *      - with mask = 0x01U then channel 0 will be started
 *      - with mask = 0x02U then channel 1 will be started
 *      - with mask = 0x03U then channel 0 and channel 1 will be started
 */
void LPIT_DRV_StartTimerChannels(uint32_t instance,
                                 uint32_t mask);

/*!
 * @brief Stops the timer channel counting.
 *
 * This function allows stop timer channels simultaneously from counting.
 * Timer channels reload their periods respectively after the next time
 * they call the LPIT_DRV_StartTimerChannels. Note that: In 32-bit Trigger Accumulator
 * mode, the counter will load on the first trigger rising edge.
 *
 * @param[in] instance LPIT module instance number
 * @param[in] mask Timer channels stopping mask that decides which channels
 * will be stopped
 * - For example:
 *      - with mask = 0x01U then channel 0 will be stopped
 *      - with mask = 0x02U then channel 1 will be stopped
 *      - with mask = 0x03U then channel 0 and channel 1 will be stopped
 */
void LPIT_DRV_StopTimerChannels(uint32_t instance,
                                uint32_t mask);

/* @} */

/*!
 * @name Timer Period
 * @{
 */

/*!
 * @brief Sets the timer channel period in microseconds.
 *
 * This function sets the timer channel period in microseconds.
 * The required period makes sense if the operation mode of timer channel
 * is 32 bit periodic counter or dual 16 bit periodic counter.
 * The period range depends on the frequency of the LPIT functional clock and
 * operation mode of timer channel.
 * If the required period is out of range, use the suitable mode if applicable.
 * This function is only valid for one single channel.
 *
 * @param[in] instance LPIT module instance number
 * @param[in] channel Timer channel number
 * @param[in] us Timer channel period in microseconds
 * @return Operation status
 *         - LPIT_STATUS_VALID_PERIOD:    Input period of timer channel is valid.
 *         - LPIT_STATUS_INVALID_PERIOD:  Input period of timer channel is invalid.
 */
lpit_status_t LPIT_DRV_SetTimerPeriodByUs(uint32_t instance,
                                          uint32_t channel,
                                          uint32_t us);

/*!
 * @brief Gets the timer channel period in microseconds.
 *
 * This function gets the timer channel period in microseconds.
 * The returned period here makes sense if the operation mode of timer channel
 * is 32 bit periodic counter or dual 16 bit periodic counter.
 *
 * @param[in] instance LPIT module instance number
 * @param[in] channel Timer channel number
 * @return Timer channel period in microseconds
 */
uint64_t LPIT_DRV_GetTimerPeriodByUs(uint32_t instance,
                                     uint32_t channel);

/*!
 * @brief Gets the current timer channel counting value in microseconds.
 *
 * This function returns an absolute time stamp in microseconds.
 * One common use of this function is to measure the running time of a part of
 * code. Call this function at both the beginning and end of code. The time
 * difference between these two time stamps is the running time.
 * The return counting value here makes sense if the operation mode of timer channel
 * is 32 bit periodic counter or dual 16 bit periodic counter or 32-bit trigger input capture.
 * Need to make sure the running time will not exceed the timer channel period.
 *
 * @param[in] instance LPIT module instance number
 * @param[in] channel Timer channel number
 * @return Current timer channel counting value in microseconds
 */
uint64_t LPIT_DRV_GetCurrentTimerUs(uint32_t instance,
                                    uint32_t channel);

/*!
 * @brief Sets the timer channel period in count unit.
 *
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
 * @param[in] instance LPIT module instance number
 * @param[in] channel Timer channel number
 * @param[in] count Timer channel period in count unit
 * @return Operation status
 *         - LPIT_STATUS_VALID_PERIOD:    Input period of timer channel is valid.
 *         - LPIT_STATUS_INVALID_PERIOD:  Input period of timer channel is invalid.
 */
lpit_status_t LPIT_DRV_SetTimerPeriodByCount(uint32_t instance,
                                             uint32_t channel,
                                             uint32_t count);

/*!
 * @brief Gets the current timer channel period in count unit.
 *
 * This function returns current period of timer channel given as argument.
 *
 * @param[in] instance LPIT module instance number
 * @param[in] channel Timer channel number
 * @return Timer channel period in count unit
 */
uint32_t LPIT_DRV_GetTimerPeriodByCount(uint32_t instance,
                                        uint32_t channel);

/*!
 * @brief Gets the current timer channel counting value in count.
 *
 * This function returns the real-time timer channel counting value, the value in
 * a range from 0 to timer channel period.
 * Need to make sure the running time does not exceed the timer channel period.
 *
 * @param[in] instance LPIT module instance number
 * @param[in] channel Timer channel number
 * @return Current timer channel counting value in count
 */
uint32_t LPIT_DRV_GetCurrentTimerCount(uint32_t instance,
                                       uint32_t channel);


/* @} */

/*!
 * @name Interrupt
 * @{
 */

/*!
 * @brief Gets the current interrupt flag of timer channels.
 *
 * This function gets the current interrupt flag of timer channels.
 * In compare modes, the flag sets to 1 at the end of the timer period.
 * In capture modes, the flag sets to 1 when the trigger asserts.
 *
 * @param[in] instance LPIT module instance number.
 * @param[in] mask The interrupt flag getting mask that decides which channels will
 * be got interrupt flag.
 * - For example:
 *      - with mask = 0x01u then the interrupt flag of channel 0 only will be got
 *      - with mask = 0x02u then the interrupt flag of channel 1 only will be got
 *      - with mask = 0x03u then the interrupt flags of channel 0 and channel 1 will be got
 * @return Current the interrupt flag of timer channels
 */
uint32_t LPIT_DRV_GetInterruptFlagTimerChannels(uint32_t instance,
                                                uint32_t mask);

/*!
 * @brief Clears the interrupt flag of timer channels.
 *
 * This function clears the interrupt flag of timer channels after
 * their interrupt event occurred.
 *
 * @param[in] instance LPIT module instance number
 * @param[in] mask The interrupt flag clearing mask that decides which channels will
 * be cleared interrupt flag
 * - For example:
 *      - with mask = 0x01u then the interrupt flag of channel 0 only will be cleared
 *      - with mask = 0x02u then the interrupt flag of channel 1 only will be cleared
 *      - with mask = 0x03u then the interrupt flags of channel 0 and channel 1 will be cleared
 */
void LPIT_DRV_ClearInterruptFlagTimerChannels(uint32_t instance,
                                              uint32_t mask);

/* @} */

#if defined(__cplusplus)
}
#endif

/*! @}*/

#endif /* FSL_LPIT_DRIVER_H*/
/*******************************************************************************
 * EOF
 ******************************************************************************/
