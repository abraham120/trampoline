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

#ifndef FSL_CRC_DRIVER_H
#define FSL_CRC_DRIVER_H

/*! @file fsl_crc_driver.h */

#include "fsl_crc_hal.h"
#include "fsl_clock_manager.h"

/*!
 * @defgroup crc_driver CRC Driver
 * @ingroup crc
 * @details This section describes the programming interface of the CRC driver.
 * @{
 */

/*******************************************************************************
 * Variables
 ******************************************************************************/
/*! @brief Table of base addresses for CRC instances */
extern CRC_Type * const g_crcBase[CRC_INSTANCE_COUNT];

/*******************************************************************************
 * Definitions
 ******************************************************************************/
/*!
 * @brief CRC status return codes.
 * Implements : crc_status_t_Class
 */
typedef enum
{
    CRC_STATUS_SUCCESS      = 0x00U,    /*!< Success */
    CRC_STATUS_CLOCK_OFF    = 0x01U     /*!< CRC instance clock is off */
} crc_status_t;

/*!
 * @brief CRC configuration structure.
 * Implements : crc_user_config_t_Class
 */
typedef struct
{
    crc_bit_width_t crcWidth;       /*!< Selects 16-bit or 32-bit CRC protocol. */
    uint32_t seed;                  /*!< Starting checksum value. */
    uint32_t polynomial;            /*!< CRC Polynomial, MSBit first.<br/>
                                         Example polynomial: 0x1021U = 1_0000_0010_0001 = x^12+x^5+1 */
    crc_transpose_t writeTranspose; /*!< Type of transpose when writing CRC input data. */
    crc_transpose_t readTranspose;  /*!< Type of transpose when reading CRC result. */
    bool complementChecksum;        /*!< True if the result shall be complement of the actual checksum. */
} crc_user_config_t;

/*******************************************************************************
 * API
 ******************************************************************************/
/*!
 * @name CRC DRIVER API
 * @{
 */

#if defined(__cplusplus)
extern "C" {
#endif

/*!
 * @brief Initializes the CRC module
 *
 * This function initializes CRC driver based on user configuration input
 *
 * @param[in] instance The CRC instance number
 * @param[in] userConfigPtr Pointer to structure of initialization
 * @return Execution status (success/clock-off)
 */
crc_status_t CRC_DRV_Init(uint32_t instance,
                          const crc_user_config_t * userConfigPtr);

/*!
 * @brief Sets the default configuration
 *
 * This function sets the default configuration
 *
 * @param[in] instance The CRC instance number
 * @return Execution status (success)
 */
crc_status_t CRC_DRV_Deinit(uint32_t instance);

/*!
 * @brief Appends a block of bytes to the current CRC calculation
 *
 * This function appends a block of bytes to the current CRC calculation
 *
 * @param[in] instance The CRC instance number
 * @param[in] data Data for current CRC calculation
 * @param[in] dataSize Length of data to be calculated
 */
void CRC_DRV_WriteData(uint32_t instance,
                       const uint8_t * data,
                       uint32_t dataSize);

/*!
 * @brief Returns the current result of the CRC calculation
 *
 * This function returns the current result of the CRC calculation
 *
 * @param[in] instance The CRC instance number
 * @return Result of CRC calculation
 */
uint32_t CRC_DRV_GetCrcResult(uint32_t instance);

/*!
 * @brief Configures the CRC module from a user configuration structure
 *
 * This function configures the CRC module from a user configuration structure
 *
 * @param[in] instance The CRC instance number
 * @param[in] userConfigPtr Pointer to structure of initialization
 * @return Execution status (success)
 */
crc_status_t CRC_DRV_Configure(uint32_t instance,
                               const crc_user_config_t * userConfigPtr);
/*! @} */

#if defined(__cplusplus)
}
#endif

/*! @} */

#endif /* FSL_CRC_DRIVER_H */
/*******************************************************************************
 * EOF
 ******************************************************************************/
