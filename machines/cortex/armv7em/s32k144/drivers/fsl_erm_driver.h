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
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES,
 * LOSS OF USE, DATA, OR PROFITS, OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 * ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef FSL_ERM_DRIVER_H
#define FSL_ERM_DRIVER_H

/*! @file fsl_erm_driver.h */

#include "fsl_erm_hal.h"

/*!
 * @defgroup erm_driver ERM Driver
 * @ingroup erm
 * @details This section describes the programming interface of the ERM driver.
 * @{
 */

/*******************************************************************************
 * Variables
 ******************************************************************************/
/*! @brief Table of base addresses for ERM instances */
extern ERM_Type * const g_ermBase[ERM_INSTANCE_COUNT];

/*******************************************************************************
 * Definitions
 ******************************************************************************/
/*!
 * @brief ERM interrupt notification configuration structure
 * Implements : erm_interrupt_config_t_Class
 */
typedef struct
{
    bool enableSingleCorrection;    /*!< Enable Single Correction Interrupt Notification */
    bool enableNonCorrectable;      /*!< Enable Non-Correctable Interrupt Notification */
} erm_interrupt_config_t;

/*!
 * @brief ERM user configuration structure
 * Implements : erm_user_config_t_Class
 */
typedef struct
{
    uint8_t channel;                                /*!< The channel assignments */
    const erm_interrupt_config_t * interruptCfg;    /*!< Interrupt configuration */
} erm_user_config_t;

/*******************************************************************************
 * API
 ******************************************************************************/
/*!
 * @name ERM DRIVER API
 * @{
 */

#if defined(__cplusplus)
extern "C" {
#endif

/*!
 * @brief Initializes the ERM module
 *
 * This function initializes ERM driver based on user configuration input,
 * channelCnt takes values between 1 and the maximum channel count supported by the hardware
 *
 * @param[in] instance The ERM instance number
 * @param[in] channelCnt The number of channels
 * @param[in] userConfigArr The pointer to the array of ERM user configure structure
 */
void ERM_DRV_Init(uint32_t instance,
                  uint8_t channelCnt,
                  const erm_user_config_t * userConfigArr);

/*!
 * @brief Sets the default configuration
 *
 * This function sets the default configuration
 *
 * @param[in] instance The ERM instance number
 */
void ERM_DRV_Deinit(uint32_t instance);

/*!
 * @brief Sets interrupt notification
 *
 * This function sets interrupt notification based on interrupt notification configuration input
 *
 * @param[in] instance The ERM instance number
 * @param[in] channel The configured memory channel
 * @param[in] interruptCfg The ERM interrupt configuration structure
 */
void ERM_DRV_SetInterruptConfig(uint32_t instance,
                                uint8_t channel,
                                erm_interrupt_config_t interruptCfg);

/*!
 * @brief Gets interrupt notification
 *
 * This function gets the current interrupt configuration of the available events
 * (which interrupts are enabled/disabled)
 *
 * @param[in] instance The ERM instance number
 * @param[in] channel The examined memory channel
 * @param[out] interruptPtr The pointer to the ERM interrupt configuration structure
 */
void ERM_DRV_GetInterruptConfig(uint32_t instance,
                                uint8_t channel,
                                erm_interrupt_config_t * const interruptPtr);

/*!
 * @brief Clears error event and the corresponding interrupt notification
 *
 * This function clears the record of an event. If the corresponding interrupt is enabled,
 * the interrupt notification will be cleared
 *
 * @param[in] instance The ERM instance number
 * @param[in] channel The configured memory channel
 * @param[in] eccEvent The types of ECC events
 */
void ERM_DRV_ClearEvent(uint32_t instance,
                        uint8_t channel,
                        erm_ecc_event_t eccEvent);

/*!
 * @brief Gets the address of the last ECC event in Memory n and ECC event
 *
 * This function gets the address of the last ECC event in Memory n and the types of the event
 *
 * @param[in] instance The ERM instance number
 * @param[in] channel The examined memory channel
 * @param[out] addressPtr The pointer to address of the last ECC event in Memory n with ECC event
 * @return The last occurred ECC event
 */
erm_ecc_event_t ERM_DRV_GetErrorDetail(uint32_t instance,
                                       uint8_t channel,
                                       uint32_t * addressPtr);
/*! @} */

#if defined(__cplusplus)
}
#endif

/*! @} */

#endif /* FSL_ERM_DRIVER_H */
/*******************************************************************************
 * EOF
 ******************************************************************************/
