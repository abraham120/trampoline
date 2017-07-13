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
 * @file fsl_ewm_driver.h
 */

#ifndef FSL_EWM_DRIVER_H
#define FSL_EWM_DRIVER_H

#include <stddef.h>
#include "fsl_ewm_hal.h"
#include "fsl_interrupt_manager.h"

/*!
 * @addtogroup ewm_drv
 * @{
 */
 /*******************************************************************************
 * Definitions
 ******************************************************************************/
/*
 * @brief EWM configuration structure
 * This structure is used to configure the EWM prescaler, window, interrupt
 * and input pin.
 *
 * Implements : ewm_init_config_t_Class
 */
typedef struct
{
    ewm_in_assert_logic_t assertLogic; /*!< Assert logic for EWM input pin */
    bool       interruptEnable;        /*!< Enable EWM interrupt           */
    uint8_t    prescaler;              /*!< EWM clock prescaler            */
    uint8_t    compareLow;             /*!< Compare low value              */
    uint8_t    compareHigh;            /*!< Compare high value             */
} ewm_init_config_t;

/*******************************************************************************
 * API
 ******************************************************************************/
 /*!
  * @name EWM Driver API
  * @{
  */

#if defined(__cplusplus)
extern "C" {
#endif

/*!
 * @brief Init EWM. This method initializes EWM instance to the configuration
 * from the passed structure. The user must make sure that the clock is
 * enabled. This is the only method needed to be called to start the module.
 *
 *  Example configuration structure:
 *  @code
 *      ewm_init_config_t ewmUserCfg = {
 *          .assertLogic        = EWM_IN_ASSERT_ON_LOGIC_ZERO,
 *          .interruptEnable    = true,
 *          .prescaler          = 128,
 *          .compareLow         = 0,
 *          .compareHigh        = 254
 *      };
 *  @endcode
 *      This configuration will enable the peripheral, with input pin configured
 *      to assert on logic low, interrupt enabled, prescaler 128 and maximum
 *      refresh window.
 *
 *      The EWM can be initialized only once per CPU reset as the registers
 *      are write once.
 *
 * @param[in] instance EWM instance number
 * @param[in] config  Pointer to the module configuration structure.
 *
 * @return ewm_status_t Will return the status of the operation:
 *          - EWM_STAT_SUCCESS              if the operation is successful
 *          - EWM_STAT_CLOCK_OFF            if the clock is disabled
 *          - EWM_STAT_ERROR                if the windows values are not
 *                                          correct or if the instance is
 *                                          already enabled
 *
 */
ewm_status_t EWM_DRV_Init(uint32_t instance, const ewm_init_config_t * config);

/*!
 * @brief Init configuration structure to default values.
 *
 * @param[out] config Pointer to the configuration structure to initialize
 *
 * @return None
 *
 */
void EWM_DRV_GetDefaultConfig(ewm_init_config_t * config);

/*!
 * @brief Refresh EWM. This method needs to be called within the window period
 * specified by the Compare Low and Compare High registers.
 *
 * @param[in] instance EWM instance number
 *
 * @return None
 *
 */
void EWM_DRV_Refresh(uint32_t instance);


/*! @}*/
#if defined(__cplusplus)
}
#endif

/*! @}*/

#endif /* FSL_EWM_DRIVER_H */
/*******************************************************************************
 * EOF
 ******************************************************************************/
