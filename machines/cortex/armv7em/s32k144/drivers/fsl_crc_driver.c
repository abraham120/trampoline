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

/**
 * @page misra_violations MISRA-C:2012 violations
 *
 * @section [global]
 * Violates MISRA 2012 Required Rule 1.3, Taking address of near auto variable.
 * The code is not dynamically linked. An absolute stack address is obtained
 * when taking the address of the near auto variable. A source of error in
 * writing dynamic code is that the stack segment may be different from the data
 * segment.
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

#include "fsl_crc_driver.h"

/*******************************************************************************
 * Variables
 ******************************************************************************/
/* Table of base addresses for CRC instances. */
CRC_Type * const g_crcBase[] = CRC_BASE_PTRS;

/*******************************************************************************
 * Code
 ******************************************************************************/
/*FUNCTION**********************************************************************
 *
 * Function Name : CRC_DRV_Init
 * Description   : This function initializes CRC driver based on user configuration input.
 *
 * Implements    : CRC_DRV_Init_Activity
 *END**************************************************************************/
crc_status_t CRC_DRV_Init(uint32_t instance,
                          const crc_user_config_t * userConfigPtr)
{
#ifdef DEV_ERROR_DETECT
    DEV_ASSERT(instance < CRC_INSTANCE_COUNT);
    DEV_ASSERT(userConfigPtr != NULL);
#endif
    CRC_Type * base = g_crcBase[instance];
    crc_status_t retStatus = CRC_STATUS_SUCCESS;
    clock_manager_error_code_t clkErr;

    /* PCC clock sources, for checking whether the module is clocked */
    static const clock_names_t s_crcClockNames[] = { PCC_CRC0_CLOCK };

    /* Get the CRC clock gate status as configured in the clock manager */
    clkErr = CLOCK_SYS_GetFreq(s_crcClockNames[instance], NULL);

    if (clkErr != CLOCK_MANAGER_SUCCESS)
    {
        /* The CRC clock gate was not initialized */
        retStatus = CRC_STATUS_CLOCK_OFF;
    }
    else
    {
        /* Set the default configuration */
        CRC_HAL_Init(base);
        /* Set the CRC configuration */
        retStatus = CRC_DRV_Configure(instance, userConfigPtr);
    }

    return retStatus;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : CRC_DRV_Deinit
 * Description   : This function sets the default configuration.
 *
 * Implements    : CRC_DRV_Deinit_Activity
 *END**************************************************************************/
crc_status_t CRC_DRV_Deinit(uint32_t instance)
{
#ifdef DEV_ERROR_DETECT
    DEV_ASSERT(instance < CRC_INSTANCE_COUNT);
#endif
    CRC_Type * base = g_crcBase[instance];

    /* Set the default configuration */
    CRC_HAL_Init(base);

    return CRC_STATUS_SUCCESS;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : CRC_DRV_WriteData
 * Description   : This function appends a block of bytes to the current CRC calculation.
 *
 * Implements    : CRC_DRV_WriteData_Activity
 *END**************************************************************************/
void CRC_DRV_WriteData(uint32_t instance,
                       const uint8_t * data,
                       uint32_t dataSize)
{
#ifdef DEV_ERROR_DETECT
    DEV_ASSERT(instance < CRC_INSTANCE_COUNT);
    DEV_ASSERT(data != NULL);
#endif
    CRC_Type * base = g_crcBase[instance];
    uint32_t i;

    /* 8-bit reads and writes till end of data buffer */
    for (i = 0U; i < dataSize; i++)
    {
        CRC_HAL_SetDataLLReg(base, data[i]);
    }
}

/*FUNCTION**********************************************************************
 *
 * Function Name : CRC_DRV_GetCrcResult
 * Description   : This function returns the current result of the CRC calculation.
 *
 * Implements    : CRC_DRV_GetCrcResult_Activity
 *END**************************************************************************/
uint32_t CRC_DRV_GetCrcResult(uint32_t instance)
{
#ifdef DEV_ERROR_DETECT
    DEV_ASSERT(instance < CRC_INSTANCE_COUNT);
#endif
    const CRC_Type * base = g_crcBase[instance];

    /* Result of the CRC calculation */
    return CRC_HAL_GetCrcResult(base);
}

/*FUNCTION**********************************************************************
 *
 * Function Name : CRC_DRV_Configure
 * Description   : This function configures the CRC module from a user configuration structure.
 *
 * Implements    : CRC_DRV_Configure_Activity
 *END**************************************************************************/
crc_status_t CRC_DRV_Configure(uint32_t instance,
                               const crc_user_config_t * userConfigPtr)
{
#ifdef DEV_ERROR_DETECT
    DEV_ASSERT(instance < CRC_INSTANCE_COUNT);
    DEV_ASSERT(userConfigPtr != NULL);
#endif
    CRC_Type * base = g_crcBase[instance];

    /* 1. Set CRC mode */
    CRC_HAL_SetProtocolWidth(base, userConfigPtr->crcWidth);

    /* 2. Set transposes and complement options */
    CRC_HAL_SetWriteTranspose(base, userConfigPtr->writeTranspose);
    CRC_HAL_SetReadTranspose(base, userConfigPtr->readTranspose);
    CRC_HAL_SetFXorMode(base, userConfigPtr->complementChecksum);

    /* 3. Write a polynomial */
    CRC_HAL_SetPolyReg(base, userConfigPtr->polynomial);

    /* 4. Write a seed (initial checksum) */
    CRC_HAL_SetSeedOrDataMode(base, true);
    CRC_HAL_SetDataReg(base, userConfigPtr->seed);
    CRC_HAL_SetSeedOrDataMode(base, false);

    return CRC_STATUS_SUCCESS;
}

/*******************************************************************************
 * EOF
 ******************************************************************************/
