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

/*!
 * @file fsl_ewm_hal.h
 */

#ifndef FSL_EWM_HAL_H
#define FSL_EWM_HAL_H

#include <stdint.h>
#include <stdbool.h>
#include "fsl_device_registers.h"

/*!
 * @addtogroup ewm_hal
 * @{
 */

 /*******************************************************************************
 * Definitions
 ******************************************************************************/

 /*! @brief EWM return codes
  * Implements : ewm_status_t_Class
  */
typedef enum
{
    EWM_STAT_SUCCESS = 0x00U,     /*!< Everything went normal */
    EWM_STAT_CLOCK_OFF,           /*!< EWM instance clock is off */
    EWM_STAT_ERROR                /*!< Some error occurred during configuration,
                                   * check function description for specific
                                   * details
                                   */
} ewm_status_t;

/*! @brief EWM input pin configuration
 * Configures if the input pin is enabled and when is asserted
 * Implements : ewm_in_assert_logic_t_Class
 */
typedef enum
{
    EWM_IN_ASSERT_DISABLED      = 0x00U,    /*!< Input pin disabled                    */
    EWM_IN_ASSERT_ON_LOGIC_ZERO = 0x01U,    /*!< Input pin asserts EWM when on logic 0 */
    EWM_IN_ASSERT_ON_LOGIC_ONE  = 0x02U     /*!< Input pin asserts EWM when on logic 1 */
} ewm_in_assert_logic_t;


/*******************************************************************************
 * API
 ******************************************************************************/
/*!
 * @name External Watchdog Module HAL
 * @{
 */

#if defined(__cplusplus)
extern "C" {
#endif

/*!
 * @brief Init EWM. This method configures the EWM instance Control Register
 * fields such as interrupt enable, input pin, instance enablement.
 * The user must make sure that the prescaler, compare high and compare low
 * registers are configured prior to this function call
 *
 * @param[in] base            EWM base pointer
 * @param[in] interruptEnable Enable or disable EWM interrupt
 * @param[in] assertLogic     Configure when the EWM input pin asserts
 * @param[in] enable          Enable or disable the EWM instance
 */
void EWM_HAL_Init(EWM_Type * const base,
                  bool interruptEnable,
                  ewm_in_assert_logic_t assertLogic,
                  bool enable);

/*!
 * @brief Get the Input pin assert logic.
 * @param[in] base EWM base pointer
 * @return The input pin assert logic:
 *      -   EWM_IN_ASSERT_DISABLED      - EWM in disabled
 *      -   EWM_IN_ASSERT_ON_LOGIC_ZERO - EWM is asserted when EWM_in is logic 0
 *      -   EWM_IN_ASSERT_ON_LOGIC_ONE  - EWM is asserted when EWM_in is logic 1
 */
ewm_in_assert_logic_t EWM_HAL_GetInputPinAssertLogic(const EWM_Type* const base);

/*!
 * @brief Refresh EWM. This method needs to be called within the window period
 * specified by the Compare Low and Compare High registers
 * @param[in] base EWM base pointer
 * Implements : EWM_HAL_Refresh_Activity
 */
static inline void EWM_HAL_Refresh(EWM_Type* const base)
{
    /* Write first byte of the service key */
    base->SERV = FSL_FEATURE_EWM_KEY_FIRST_BYTE;
    /* Write second byte of the service key */
    base->SERV = FSL_FEATURE_EWM_KEY_SECOND_BYTE;
}

/*!
 * @brief Get the Interrupt Enable bit.
 * @param[in] base EWM base pointer
 * @return The state of the interrupt enable bit:
 *      -   false - interrupt disabled
 *      -   true  - interrupt enabled
 * Implements : EWM_HAL_IsInterruptEnabled_Activity
 */
static inline bool EWM_HAL_IsInterruptEnabled(const EWM_Type * const base)
{
    uint8_t value = (((base->CTRL) & EWM_CTRL_INTEN_MASK) >> EWM_CTRL_INTEN_SHIFT);
    return (value == 0U) ? false : true;
}

/*!
 * @brief Get the EWM enable bit.
 * @param[in] base EWM base pointer
 * @return The state of the device enable bit:
 *      -   false - EWM disabled
 *      -   true  - EWM enabled
 * Implements : EWM_HAL_IsEnabled_Activity
 */
static inline bool EWM_HAL_IsEnabled(const EWM_Type * const base)
{
    uint8_t value = (((base->CTRL) & EWM_CTRL_EWMEN_MASK) >> EWM_CTRL_EWMEN_SHIFT);
    return (value == 0U) ? false : true;
}

/*!
 * @brief Get the Control register Value.
 * @param[in] base EWM base pointer
 * @return Value stored in Control register
 * Implements : EWM_HAL_GetControl_Activity
 */
static inline uint8_t EWM_HAL_GetControl(const EWM_Type* const base)
{
    return base->CTRL;
}

/*!
 * @brief Set the Compare Low Value. This register can be only written once after
 * a CPU reset. The user must make sure that the Compare High is greater than
 * Compare Low value
 * @param[in] base  EWM base pointer
 * @param[in] value Value to write into Compare Low register
 * Implements : EWM_HAL_SetCompareLow_Activity
 */
static inline void EWM_HAL_SetCompareLow(EWM_Type* const base, uint8_t value)
{
    base->CMPL = value;
}

/*!
 * @brief Get the Compare Low Value.
 * @param[in] base EWM base pointer
 * @return Value stored in Compare Low register
 * Implements : EWM_HAL_GetCompareLow_Activity
 */
static inline uint8_t EWM_HAL_GetCompareLow(const EWM_Type* const base)
{
    return base->CMPL;
}

/*!
 * @brief Set the Compare High Value. This register can be only written once after
 * a CPU reset. The user must make sure that the Compare High is greater than
 * Compare Low value
 * Note: The maximum Compare High value is 0xFE
 * @param[in] base  EWM base pointer
 * @param[in] value Value to write into Compare High register
 * Implements : EWM_HAL_SetCompareHigh_Activity
 */
static inline void EWM_HAL_SetCompareHigh(EWM_Type* const base, uint8_t value)
{
    base->CMPH = value;
}

/*!
 * @brief Get the Compare High Value.
 * @param[in] base   EWM base pointer
 * @return Value stored in Compare High register
 * Implements : EWM_HAL_GetCompareHigh_Activity
 */
static inline uint8_t EWM_HAL_GetCompareHigh(const EWM_Type* const base)
{
    return base->CMPH;
}

/*!
 * @brief Set the Clock Prescaler Value. This register can be only written once
 * after a CPU reset and it must be written before enabling the EWM
 * @param[in] base  EWM base pointer
 * @param[in] value Prescaler Value
 * Implements : EWM_HAL_SetPrescaler_Activity
 */
static inline void EWM_HAL_SetPrescaler(EWM_Type* const base, uint8_t value)
{
    base->CLKPRESCALER = value;
}

/*!
 * @brief Get the Clock Prescaler Value.
 * @param[in] base EWM base pointer
 * @return Value stored in Clock Prescaler register
 * Implements : EWM_HAL_GetPrescaler_Activity
 */
static inline uint8_t EWM_HAL_GetPrescaler(const EWM_Type* const base)
{
    return base->CLKPRESCALER;
}

/*! @}*/
#if defined(__cplusplus)
}
#endif

/*! @}*/

#endif /* FSL_EWM_HAL_H */
/*******************************************************************************
 * EOF
 ******************************************************************************/
