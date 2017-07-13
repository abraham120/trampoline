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
 * @file fsl_lpit_hal.h
 */

#ifndef FSL_LPIT_HAL_H
#define FSL_LPIT_HAL_H

#include "fsl_device_registers.h"
#include <stdbool.h>
#include <stddef.h>

/*!
 * @defgroup lpit_hal LPIT HAL
 * @ingroup lpit
 * @brief Low Power Interrupt Timer module Hardware Abstraction Level.
 * LPIT HAL provides low level APIs for reading and writing register bit-fields
 * belonging to the LPIT module.
 * @addtogroup lpit_hal
 * @{
 */

/*******************************************************************************
 * Definitions
 *****************************************************************************/

/*!
 * @brief Hardware information of LPIT module
 * Implements : lpit_module_information_t_Class
 */
typedef struct
{
    uint32_t majorVersionNumber;                /*!< The major version number for the LPIT module specification */
    uint32_t minorVersionNumber;                /*!< The minor version number for the LPIT module specification */
    uint32_t featureNumber;                     /*!< The feature set number                                     */
    uint32_t numberOfExternalTriggerInputs;     /*!< Number of external triggers implemented                    */
    uint32_t numberOfTimerChannels;             /*!< Number of timer channels implemented                       */
} lpit_module_information_t;

/*******************************************************************************
 * Enumerations
 ******************************************************************************/

/*!
 * @brief Mode options available for the LPIT timer
 * Implements : lpit_timer_modes_t_Class
 */
typedef enum
{
    LPIT_PERIODIC_COUNTER      = 0x00U,  /*!< 32-bit Periodic Counter        */
    LPIT_DUAL_PERIODIC_COUNTER = 0x01U,  /*!< Dual 16-bit Periodic Counter   */
    LPIT_TRIGGER_ACCUMULATOR   = 0x02U,  /*!< 32-bit Trigger Accumulator     */
    LPIT_INPUT_CAPTURE         = 0x03U   /*!< 32-bit Trigger Input Capture   */
} lpit_timer_modes_t;

/*!
 * @brief Trigger source options.
 *
 * This is used for both internal and external trigger sources. The actual trigger
 * options available is SoC specific, user should refer to the reference manual.
 * Implements : lpit_trigger_source_t_Class
 */
typedef enum
{
    LPIT_TRIGGER_SOURCE_EXTERNAL = 0x00U, /*!< Use external trigger  */
    LPIT_TRIGGER_SOURCE_INTERNAL = 0x01U  /*!< Use internal trigger  */
}  lpit_trigger_source_t;

/*******************************************************************************
 * API
 ******************************************************************************/

#if defined(__cplusplus)
extern "C" {
#endif

/*!
  @name Timer Initialization
 * @{
 */

/*!
 * @brief Gets the information of LPIT module.
 *
 * This function gets the informations of LPIT module as:
 * major version number, minor version number, feature number,
 * number of external trigger inputs, number of timer channels.
 *
 * @param[in] base LPIT peripheral base address
 * @param[out] moduleInfomation Pointer to lpit_module_information_t structure which
 * is used to contain LPIT information.
 */
void LPIT_HAL_GetModuleInformation(const LPIT_Type * base,
                                   lpit_module_information_t * moduleInfomation);

/*!
 * @brief Enables the LPIT module.
 *
 * This function enables the functional clock of LPIT module (Note: this function
 * does not un-gate the system clock gating control). It should be called before
 * setup any timer channel.
 *
 * @param[in] base LPIT peripheral base address
 * Implements : LPIT_HAL_Enable_Activity
 */
static inline void LPIT_HAL_Enable(LPIT_Type * const base)
{
    base->MCR |= LPIT_MCR_M_CEN_MASK;
}

/*!
 * @brief Disables the LPIT module.
 *
 * This function disables functional clock of LPIT module (Note: it does not
 * affect the system clock gating control).
 *
 * @param[in] base LPIT peripheral base address
 * Implements : LPIT_HAL_Disable_Activity
 */
static inline void LPIT_HAL_Disable(LPIT_Type * const base)
{
    base->MCR &= ~LPIT_MCR_M_CEN_MASK;
}

/*!
 * @brief Resets the LPIT module.
 *
 * This function sets all LPIT registers to reset value,
 * except the Module Control Register.
 *
 * @param[in] base LPIT peripheral base address
 * Implements : LPIT_HAL_Reset_Activity
 */
static inline void LPIT_HAL_Reset(LPIT_Type * const base)
{
    base->MCR |= LPIT_MCR_SW_RST_MASK;
    base->MCR &= ~LPIT_MCR_SW_RST_MASK;
}

/*! @} */

/*!
  @name Timer Start and Stop
 * @{
 */

/*!
  @brief Starts the timer channel counting.
 *
 * This function allows starting timer channels simultaneously .
 * After calling this function, timer channels are going operate depend on mode and
 * control bits which controls timer channel start, reload and restart.
 *
 * @param[in] base LPIT peripheral base address
 * @param[in] mask Timer channels starting mask that decides which channels
 * will be started
 * - For example:
 *      - with mask = 0x01U then channel 0 will be started
 *      - with mask = 0x02U then channel 1 will be started
 *      - with mask = 0x03U then channel 0 and channel 1 will be started
 * Implements : LPIT_HAL_StartTimerChannels_Activity
 */
static inline void LPIT_HAL_StartTimerChannels(LPIT_Type * const base,
                                               uint32_t mask)
{
    base->SETTEN |= mask;
}

/*!
 * @brief Stops the timer channel from counting.
 *
 * This function allows stop timer channels simultaneously from counting.
 * Timer channels reload their periods respectively after the next time
 * they call the LPIT_DRV_StartTimerChannels. Note that: In 32-bit Trigger Accumulator
 * mode, the counter will load on the first trigger rising edge.
 *
 * @param[in] base LPIT peripheral base address
 * @param[in] mask Timer channels stopping mask that decides which channels
 * will be stopped
 * - For example:
 *      - with mask = 0x01U then channel 0 will be stopped
 *      - with mask = 0x02U then channel 1 will be stopped
 *      - with mask = 0x03U then channel 0 and channel 1 will be stopped
 * Implements : LPIT_HAL_StopTimerChannels_Activity
 */
static inline void LPIT_HAL_StopTimerChannels(LPIT_Type * const base,
                                              uint32_t mask)
{
    base->CLRTEN |= mask;
}

/*!
 * @brief Checks timer channel operation status.
 *
 * This function checks to see whether the timer channel given as argument is running.
 *
 * @param[in] base LPIT peripheral base address
 * @param[in] channel Timer channel number
 * @return The timer channel operation status
 *         -True: the timer channel is running
 *         -False: the timer channel is not running
 * Implements : LPIT_HAL_IsTimerChannelRunning_Activity
 */
static inline bool LPIT_HAL_IsTimerChannelRunning(const LPIT_Type * base,
                                                  uint32_t channel)
{
    return (((base->TMR[channel].TCTRL & LPIT_TMR_TCTRL_T_EN_MASK) >> \
            LPIT_TMR_TCTRL_T_EN_SHIFT) != 0U);
}

/*! @} */

/*!
 * @name Timer Period
 * @{
 */

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
 * @param[in] base LPIT peripheral base address
 * @param[in] channel Timer channel number
 * @param[in] count Timer channel period in count unit
 * Implements : LPIT_HAL_SetTimerPeriodByCount_Activity
 */
static inline void LPIT_HAL_SetTimerPeriodByCount(LPIT_Type * const base,
                                                  uint32_t channel,
                                                  uint32_t count)
{
    base->TMR[channel].TVAL = count;
}

/*!
 * @brief Gets the timer channel period in count unit.
 *
 * This function returns current period of timer channel given as argument.
 *
 * @param[in] base LPIT peripheral base address
 * @param[in] channel Timer channel number
 * @return Timer channel period in count unit
 * Implements : LPIT_HAL_GetTimerPeriodByCount_Activity
 */
static inline uint32_t LPIT_HAL_GetTimerPeriodByCount(const LPIT_Type * base,
                                                      uint32_t channel)
{
    return (base->TMR[channel].TVAL);
}

/*!
 * @brief Gets the current timer channel counting value.
 *
 * This function returns the real-time timer channel counting value, the value in
 * a range from 0 to timer channel period.
 * Need to make sure the running time does not exceed the timer channel period.
 *
 * @param[in] base LPIT peripheral base address
 * @param[in] channel Timer channel number
 * @return Current timer channel counting value
 * Implements : LPIT_HAL_GetCurrentTimerCount_Activity
 */
static inline uint32_t LPIT_HAL_GetCurrentTimerCount(const LPIT_Type * base,
                                                     uint32_t channel)
{
    return (base->TMR[channel].CVAL);
}

/*! @} */

/*!
 * @name Timer Interrupt
 * @{
 */

/*!
 * @brief Enables the interrupt generation for timer channels.
 *
 * This function allows enabling interrupt generation for timer channels simultaneously.
 *
 * @param[in] base LPIT peripheral base address
 * @param[in] mask The interrupt enabling mask that decides which channels will
 * be enabled interrupt.
 * - For example:
 *      - with mask = 0x01u then will enable interrupt for channel 0 only
 *      - with mask = 0x02u then will enable interrupt for channel 1 only
 *      - with mask = 0x03u then will enable interrupt for channel 0 and channel 1
 * Implements : LPIT_HAL_EnableInterruptTimerChannels_Activity
 */
static inline void LPIT_HAL_EnableInterruptTimerChannels(LPIT_Type * const base,
                                                         uint32_t mask)
{
    base->MIER |= mask;
}

/*!
 * @brief Disables the interrupt generation for timer channels.
 *
 * This function allows disabling interrupt generation for timer channels simultaneously.
 *
 * @param[in] base LPIT peripheral base address
 * @param[in] mask The interrupt disabling mask that decides which channels will
 * be disabled interrupt.
 * - For example:
 *      - with mask = 0x01u then will disable interrupt for channel 0 only
 *      - with mask = 0x02u then will disable interrupt for channel 1 only
 *      - with mask = 0x03u then will disable interrupt for channel 0 and channel 1
 * Implements : LPIT_HAL_DisableInterruptTimerChannels_Activity
 */
static inline void LPIT_HAL_DisableInterruptTimerChannels(LPIT_Type * const base,
                                                          uint32_t mask)
{
    base->MIER &= ~mask;
}

/*!
 * @brief Gets the interrupt flag of timer channels.
 *
 * This function gets current interrupt flag of timer channels.
 *
 * @param[in] base LPIT peripheral base address
 * @param[in] mask The interrupt flag getting mask that decides which channels will
 * be got interrupt flag.
 * - For example:
 *      - with mask = 0x01u then the interrupt flag of channel 0 only will be got
 *      - with mask = 0x02u then the interrupt flag of channel 1 only will be got
 *      - with mask = 0x03u then the interrupt flags of channel 0 and channel 1 will be got
 * @return The interrupt flag of timer channels.
 * Implements : LPIT_HAL_GetInterruptFlagTimerChannels_Activity
 */
static inline uint32_t LPIT_HAL_GetInterruptFlagTimerChannels(const LPIT_Type * base,
                                                              uint32_t mask)
{
    return (base->MSR) & mask;
}

/*!
 * @brief Clears the interrupt flag of timer channels.
 *
 * This function clears current interrupt flag of timer channels.
 *
 * @param[in] base LPIT peripheral base address
 * @param[in] mask The interrupt flag clearing mask that decides which channels will
 * be cleared interrupt flag.
 * - For example:
 *      - with mask = 0x01u then the interrupt flag of channel 0 only will be cleared
 *      - with mask = 0x02u then the interrupt flag of channel 1 only will be cleared
 *      - with mask = 0x03u then the interrupt flags of channel 0 and channel 1 will be cleared
 * Implements : LPIT_HAL_ClearInterruptFlagTimerChannels_Activity
 */
static inline void LPIT_HAL_ClearInterruptFlagTimerChannels(LPIT_Type * const base,
                                                            uint32_t mask)
{
    /* Write 1 to clear the interrupt flag. */
    base->MSR = mask;
    /* Read-after-write sequence to guarantee required serialization of memory operations */
    base->MSR;
}

/*! @} */

/*!
 * @name Timer Configuration
 * @{
 */

/*!
 * @brief Sets operation mode of timer channel
 *
 * This function sets the timer channel operation mode which control how
 * the timer channel decrements.
 *
 * @param[in] base LPIT peripheral base address
 * @param[in] channel Timer channel number
 * @param[in] mode Operation mode of timer channel that is member of lpit_timer_modes_t
 * Implements : LPIT_HAL_SetTimerChannelModeCmd_Activity
 */
static inline void LPIT_HAL_SetTimerChannelModeCmd(LPIT_Type * const base,
                                                   uint32_t channel,
                                                   lpit_timer_modes_t mode)
{
    base->TMR[channel].TCTRL &= ~LPIT_TMR_TCTRL_MODE_MASK;
    base->TMR[channel].TCTRL |=  LPIT_TMR_TCTRL_MODE(mode);
}

/*!
 * @brief Gets current operation mode of timer channel.
 *
 * This function gets current operation mode of the timer channel given as argument.
 *
 * @param[in] base LPIT peripheral base address
 * @param[in] channel Timer channel number
 * @return Operation mode of timer channel that is one of lpit_timer_modes_t
 * Implements : LPIT_HAL_GetTimerChannelModeCmd_Activity
 */
static inline lpit_timer_modes_t LPIT_HAL_GetTimerChannelModeCmd(const LPIT_Type * base,
                                                                 uint32_t channel)
{
    uint32_t tmp;
    lpit_timer_modes_t mode;

    tmp = (((base->TMR[channel].TCTRL) & LPIT_TMR_TCTRL_MODE_MASK)
                                 >> LPIT_TMR_TCTRL_MODE_SHIFT);
    switch (tmp)
    {
        case 0x00U:
            mode = LPIT_PERIODIC_COUNTER;
            break;
        case 0x01U:
            mode = LPIT_DUAL_PERIODIC_COUNTER;
            break;
        case 0x02U:
            mode = LPIT_TRIGGER_ACCUMULATOR;
            break;
        case 0x03U:
            mode = LPIT_INPUT_CAPTURE;
            break;
        default:
            mode = LPIT_PERIODIC_COUNTER;
            break;
    }
    return mode;
}

/*!
 * @brief Sets internal trigger source for timer channel
 *
 * This function selects one trigger from the set of internal triggers that is
 * generated by other timer channels.
 * The selected trigger is used for starting and/or reloading the timer channel.
 *
 * @param[in] base LPIT peripheral base address
 * @param[in] channel Timer channel number
 * @param[in] triggerChannelSelect Number of the channel which is selected to be trigger source
 * Implements : LPIT_HAL_SetTriggerSelectCmd_Activity
 */
static inline void LPIT_HAL_SetTriggerSelectCmd(LPIT_Type * const base,
                                                uint32_t channel,
                                                uint32_t triggerChannelSelect)
{
    base->TMR[channel].TCTRL &= ~LPIT_TMR_TCTRL_TRG_SEL_MASK;
    base->TMR[channel].TCTRL |=  LPIT_TMR_TCTRL_TRG_SEL(triggerChannelSelect);
}

/*!
 * @brief Sets trigger source of timer channel.
 *
 * This function sets trigger source of the timer channel to be internal or external trigger.
 *
 * @param[in] base LPIT peripheral base address
 * @param[in] channel Timer channel number
 * @param[in] triggerSource Trigger source of timer channel(internal or external source)
 * Implements : LPIT_HAL_SetTriggerSourceCmd_Activity
 */
static inline void LPIT_HAL_SetTriggerSourceCmd(LPIT_Type * const base,
                                                uint32_t channel,
                                                lpit_trigger_source_t triggerSource)
{
    base->TMR[channel].TCTRL &= ~LPIT_TMR_TCTRL_TRG_SRC_MASK;
    base->TMR[channel].TCTRL |=  LPIT_TMR_TCTRL_TRG_SRC(triggerSource);
}

/*!
 * @brief Sets timer channel reload on trigger.
 *
 * This function sets the timer channel to reload/don't reload on trigger.
 *
 * @param[in] base LPIT peripheral base address
 * @param[in] channel Timer channel number
 * @param[in] isReloadOnTrigger Timer channel reload on trigger
 *        - True : timer channel will reload on trigger
 *        - False : timer channel will not reload on trigger
 * Implements : LPIT_HAL_SetReloadOnTriggerCmd_Activity
 */
static inline void LPIT_HAL_SetReloadOnTriggerCmd(LPIT_Type * const base,
                                                  uint32_t channel,
                                                  bool isReloadOnTrigger)
{
    base->TMR[channel].TCTRL &= ~LPIT_TMR_TCTRL_TROT_MASK;
    base->TMR[channel].TCTRL |=  LPIT_TMR_TCTRL_TROT(isReloadOnTrigger ? 1UL : 0UL);
}

/*!
 * @brief Sets timer channel stop on interrupt.
 *
 * This function sets the timer channel to stop or don't stop after it times out.
 *
 * @param[in] base LPIT peripheral base address
 * @param[in] channel Timer channel number
 * @param[in] isStopOnInterrupt Timer channel stop on interrupt
 *        - True : Timer channel will stop after it times out
 *        - False : Timer channel will not stop after it times out
 * Implements : LPIT_HAL_SetStopOnInterruptCmd_Activity
 */
static inline void LPIT_HAL_SetStopOnInterruptCmd(LPIT_Type * const base,
                                                  uint32_t channel,
                                                  bool isStopOnInterrupt)
{
    base->TMR[channel].TCTRL &= ~LPIT_TMR_TCTRL_TSOI_MASK;
    base->TMR[channel].TCTRL |=  LPIT_TMR_TCTRL_TSOI(isStopOnInterrupt ? 1UL : 0UL);
}

/*!
 * @brief Sets timer channel start on trigger.
 *
 * This function sets the timer channel to starts/don't start on trigger.
 *
 * @param[in] base LPIT peripheral base address
 * @param[in] channel Timer channel number
 * @param[in] isStartOnTrigger Timer channel start on trigger
 *        - True : Timer channel starts to decrement when rising edge on selected trigger is detected
 *        - False : Timer channel starts to decrement immediately based on restart condition
 *                      (controlled by Timer Stop On Interrupt bit)
 * Implements : LPIT_HAL_SetStartOnTriggerCmd_Activity
 */
static inline void LPIT_HAL_SetStartOnTriggerCmd(LPIT_Type * const base,
                                                 uint32_t channel,
                                                 bool isStartOnTrigger)
{
    base->TMR[channel].TCTRL &= ~LPIT_TMR_TCTRL_TSOT_MASK;
    base->TMR[channel].TCTRL |=  LPIT_TMR_TCTRL_TSOT(isStartOnTrigger ? 1UL : 0UL);
}

/*!
 * @brief Sets timer channel chaining.
 *
 * This function sets the timer channel to be chained or not chained.
 *
 * @param[in] base LPIT peripheral base address
 * @param[in] channel Timer channel number(Note: The timer channel 0 cannot be chained)
 * @param[in] isChannelChained Timer channel chaining
 *        - True : Timer channel is chained. Timer channel decrements on previous channel's timeout
 *        - False : Timer channel is not chained. Timer channel runs independently
 * Implements : LPIT_HAL_SetTimerChannelChainCmd_Activity
 */
static inline void LPIT_HAL_SetTimerChannelChainCmd(LPIT_Type * const base,
                                                    uint32_t channel,
                                                    bool isChannelChained)
{
    base->TMR[channel].TCTRL &= ~LPIT_TMR_TCTRL_CHAIN_MASK;
    base->TMR[channel].TCTRL |=  LPIT_TMR_TCTRL_CHAIN(isChannelChained ? 1UL : 0UL);
}

/*!
 * @brief Sets operation of LPIT in debug mode.
 *
 * When the device enters debug mode, the timer channels may or may not be frozen,
 * based on the configuration of this function. This is intended to aid software development,
 * allowing the developer to halt the processor, investigate the current state of
 * the system (for example, the timer channel values), and continue the operation.
 *
 * @param[in] base LPIT peripheral base address
 * @param[in] isRunInDebug LPIT run in debug mode
 *        - True: LPIT continue to run when the device enters debug mode
 *        - False: LPIT stop when the device enters debug mode
 * Implements : LPIT_HAL_SetTimerRunInDebugCmd_Activity
 */
static inline void LPIT_HAL_SetTimerRunInDebugCmd(LPIT_Type * const base,
                                                  bool isRunInDebug)
{
    base->MCR &= ~LPIT_MCR_DBG_EN_MASK;
    base->MCR |= LPIT_MCR_DBG_EN(isRunInDebug ? 1UL: 0UL);
}

/*!
 * @brief Sets operation of LPIT in DOZE mode.
 *
 * When the device enters debug mode, the timer channels may or may not be frozen,
 * based on the configuration of this function. The LPIT must use an external or
 * internal clock source which remains operating during DOZE modes(low power mode).
 *
 * @param[in] base LPIT peripheral base address
 * @param[in] isRunInDoze LPIT run in DOZE mode
 *        - True: LPIT continue to run when the device enters DOZE mode
 *        - False: LPIT channels stop when the device enters DOZE mode
 * Implements : LPIT_HAL_SetTimerRunInDozeCmd_Activity
 */
static inline void LPIT_HAL_SetTimerRunInDozeCmd(LPIT_Type * const base,
                                                 bool isRunInDoze)
{
    base->MCR &= ~LPIT_MCR_DOZE_EN_MASK;
    base->MCR |= LPIT_MCR_DOZE_EN(isRunInDoze ? 1UL : 0UL);
}

/*! @} */

#if defined(__cplusplus)
}
#endif

/*! @} */

#endif /* FSL_LPIT_HAL_H*/
/*******************************************************************************
 * EOF
 *******************************************************************************/
