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
 * @file fsl_mpu_hal.c
 *
 * @page misra_violations MISRA-C:2012 violations
 *
 * @section [global]
 * Violates MISRA 2012 Required Rule 10.3, Expression assigned to a narrower or
 * different essential type.
 * This is required by the conversion of a bit-field of a register into a enum type.
 *
 * @section [global]
 * Violates MISRA 2012 Advisory Rule 10.5, Impermissible cast; cannot cast from
 * 'essentially unsigned' type to 'essentially enum<i>'.
 * This is required by the conversion of a bit-field of a register into a enum type.
 *
 * @section [global]
 * Violates MISRA 2012 Required Rule 10.8, Impermissible cast of composite
 * expression (different essential type categories).
 * This is required by the conversion of a bit-field of a register into enum type.
 */

#include <stddef.h>
#include "fsl_mpu_hal.h"

/*******************************************************************************
 * Definitions
 *******************************************************************************/
/* Default value of access right */
#define DEFAULT_ACCESS_RIGHT    (0x0061F7DFU)
/* Default value of end address */
#define DEFAULT_END_ADDRESS     (0x1FU)

/*******************************************************************************
 * Code
 *******************************************************************************/
/*FUNCTION**********************************************************************
 *
 * Function Name : MPU_HAL_GetSlavePortErrorStatus
 * Description   : Gets the error status of a specified slave port.
 *
 * Implements    : MPU_HAL_GetSlavePortErrorStatus_Activity
 *END**************************************************************************/
bool MPU_HAL_GetSlavePortErrorStatus(const MPU_Type * const base,
                                     uint8_t slaveNum)
{
#ifdef DEV_ERROR_DETECT
    DEV_ASSERT(slaveNum < FSL_FEATURE_MPU_SLAVE_COUNT);
#endif
    uint32_t sperr;

    /* The following shows the correspondence between the bit number
     * and slave port number:
     * Bit 31 corresponds to slave port 0.
     * Bit 30 corresponds to slave port 1.
     * Bit 29 corresponds to slave port 2.
     * Bit 28 corresponds to slave port 3.
     */
    sperr = ((base->CESR & MPU_CESR_SPERR_MASK) & (1UL << (31U - slaveNum)));

    return (sperr != 0U) ? true : false;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : MPU_HAL_GetDetailErrorAccessInfo
 * Description   : Gets MPU detail error access info.
 *
 * Implements    : MPU_HAL_GetDetailErrorAccessInfo_Activity
 *END**************************************************************************/
void MPU_HAL_GetDetailErrorAccessInfo(MPU_Type * const base,
                                      uint8_t slaveNum,
                                      mpu_access_err_info_t *errInfoPtr)
{
#ifdef DEV_ERROR_DETECT
    DEV_ASSERT(slaveNum < FSL_FEATURE_MPU_SLAVE_COUNT);
    DEV_ASSERT(errInfoPtr != NULL);
#endif
    uint32_t temp;

    /* Read the Error Detail Register for the slave port */
    temp = base->EAR_EDR[slaveNum].EDR;

    /* Read Error Access Control Detail of slave port (EACD) in EDRn register:
     * If EDRn contains a captured error and EACD is cleared, an access did not hit
     * in any region descriptor.
     * If only a single EACD bit is set, the protection error was caused by a single
     * non-overlapping region descriptor.
     * If two or more EACD bits are set, the protection error was caused by an overlapping
     * set of region descriptors.
     */
    errInfoPtr->accessCtr = (uint16_t)((temp & MPU_EDR_EACD_MASK) >> MPU_EDR_EACD_SHIFT);

    /* Report Error Master Number to user */
    errInfoPtr->master = (uint8_t)((temp & MPU_EDR_EMN_MASK) >> MPU_EDR_EMN_SHIFT);

    /* Report Error Attributes to user */
    errInfoPtr->attributes = (mpu_err_attributes_t)((temp & MPU_EDR_EATTR_MASK) >> MPU_EDR_EATTR_SHIFT);

    /* Report Error Read/Write to user */
    errInfoPtr->accessType = (mpu_err_access_type_t)((temp & MPU_EDR_ERW_MASK) >> MPU_EDR_ERW_SHIFT);

    /* Report Error Address to user */
    errInfoPtr->addr = base->EAR_EDR[slaveNum].EAR;

#if FSL_FEATURE_MPU_HAS_PROCESS_IDENTIFIER
    /* Report Error Process Identification to user */
    errInfoPtr->processorIdentification = (uint8_t)((temp & MPU_EDR_EPID_MASK) >> MPU_EDR_EPID_SHIFT);
#endif

    /* Write 1 to clear the Slave Port Error */
    temp = ((base->CESR & ~MPU_CESR_SPERR_MASK) | (1UL << (31U - slaveNum)));
    base->CESR = temp;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : MPU_HAL_SetRegionAddr
 * Description   : Sets region start and end address.
 * Please note that using this function will clear the valid bit of the region,
 * and a further validation might be needed.
 *
 * Implements    : MPU_HAL_SetRegionAddr_Activity
 *END**************************************************************************/
void MPU_HAL_SetRegionAddr(MPU_Type * const base,
                           uint8_t regionNum,
                           uint32_t startAddr,
                           uint32_t endAddr)
{
#ifdef DEV_ERROR_DETECT
    DEV_ASSERT(regionNum < MPU_RGD_COUNT);
    DEV_ASSERT(startAddr <= endAddr);
#endif

    /* Write start address to RGD_WORD0 */
    base->RGD[regionNum].WORD0 = startAddr;

    /* Write end address to RGD_WORD1 */
    base->RGD[regionNum].WORD1 = endAddr;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : MPU_HAL_SetLowMasterAccessRights
 * Description   : Sets access permission for master which has separated privilege rights
 * for user and supervisor mode accesses in a specific region.
 * Please note that using this function will clear the valid bit of the region.
 * In order to keep the region valid,
 * the MPU_HAL_SetLowMasterAccessRightsByAlternateReg function can be used.
 *
 * Implements    : MPU_HAL_SetLowMasterAccessRights_Activity
 *END**************************************************************************/
void MPU_HAL_SetLowMasterAccessRights(MPU_Type * const base,
                                      uint8_t regionNum,
                                      uint8_t masterNum,
                                      const mpu_low_masters_access_rights_t *accessRightsPtr)
{
#ifdef DEV_ERROR_DETECT
    DEV_ASSERT(regionNum < MPU_RGD_COUNT);
    DEV_ASSERT(masterNum <= FSL_FEATURE_MPU_MAX_LOW_MASTER_NUMBER);
    DEV_ASSERT(accessRightsPtr != NULL);
#endif
    uint32_t accRight;
    uint32_t accMask;
    uint32_t accShift;
    uint32_t temp;

    /* Prepare Supervisor Mode Access Control and User Mode Access Control value */
    accRight = MPU_RGD_WORD2_M0UM(accessRightsPtr->userAccessRights) |
               MPU_RGD_WORD2_M0SM(accessRightsPtr->superAccessRights);
    accMask  = MPU_RGD_WORD2_M0UM_MASK | MPU_RGD_WORD2_M0SM_MASK;
#if FSL_FEATURE_MPU_HAS_PROCESS_IDENTIFIER
    if (masterNum < FSL_FEATURE_MPU_MASTER_DMA)
    {
        accRight |= MPU_RGD_WORD2_M0PE(accessRightsPtr->processIdentifierEnable ? 1UL : 0UL);
    }
    accMask  |= MPU_RGD_WORD2_M0PE_MASK;
#endif

    /* Shift FSL_FEATURE_MPU_LOW_MASTER_CONTROL_WIDTH bit field defining separate privilege rights
     * depend on bus master number
     */
    accShift = ((uint32_t)masterNum * FSL_FEATURE_MPU_LOW_MASTER_CONTROL_WIDTH);
    accRight = accRight << accShift;
    accMask  = accMask << accShift;

    /* Set access rights */
    temp = base->RGD[regionNum].WORD2;
    temp = (temp & ~accMask) | accRight;
    base->RGD[regionNum].WORD2 = temp;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : MPU_HAL_SetHighMasterAccessRights
 * Description   : Sets access permission for master which has only read and write
 * permissions in a specific region.
 * Please note that using this function will clear the valid bit of the region.
 * In order to keep the region valid,
 * the MPU_HAL_SetHighMasterAccessRightsByAlternateReg function can be used.
 *
 * Implements    : MPU_HAL_SetHighMasterAccessRights_Activity
 *END**************************************************************************/
void MPU_HAL_SetHighMasterAccessRights(MPU_Type * const base,
                                       uint8_t regionNum,
                                       uint8_t masterNum,
                                       const mpu_high_masters_access_rights_t *accessRightsPtr)
{
#ifdef DEV_ERROR_DETECT
    DEV_ASSERT(regionNum < MPU_RGD_COUNT);
    DEV_ASSERT(masterNum > FSL_FEATURE_MPU_MAX_LOW_MASTER_NUMBER);
    DEV_ASSERT(masterNum <= FSL_FEATURE_MPU_MAX_HIGH_MASTER_NUMBER);
    DEV_ASSERT(accessRightsPtr != NULL);
#endif
    uint32_t accRight;
    uint32_t accMask;
    uint32_t accShift;
    uint32_t temp;

    /* Prepare Read Enable and Write Enable value */
    accRight = MPU_RGD_WORD2_M4RE(accessRightsPtr->readEnable ? 1UL : 0UL) |
               MPU_RGD_WORD2_M4WE(accessRightsPtr->writeEnable ? 1UL : 0UL);

    /* Low master number is numbered from 0 so master number count will be FSL_FEATURE_MPU_MAX_LOW_MASTER_NUMBER added to 1 */
    accShift = FSL_FEATURE_MPU_HIGH_MASTER_CONTROL_WIDTH * (masterNum - (FSL_FEATURE_MPU_MAX_LOW_MASTER_NUMBER + 1U));
    accMask  = (MPU_RGD_WORD2_M4RE_MASK | MPU_RGD_WORD2_M4WE_MASK) << accShift;
    accRight = accRight << accShift;

    /* Set access right */
    temp = base->RGD[regionNum].WORD2;
    temp = (temp & ~accMask) | accRight;
    base->RGD[regionNum].WORD2 = temp;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : MPU_HAL_SetAlternateLowMasterAccessRights
 * Description   : Sets access permission for master which has separated privilege rights
 * for user and supervisor mode accesses in a specific region by alternate register.
 *
 * Implements    : MPU_HAL_SetAlternateLowMasterAccessRights_Activity
 *END**************************************************************************/
void MPU_HAL_SetAlternateLowMasterAccessRights(MPU_Type * const base,
                                               uint8_t regionNum,
                                               uint8_t masterNum,
                                               const mpu_low_masters_access_rights_t *accessRightsPtr)
{
#ifdef DEV_ERROR_DETECT
    DEV_ASSERT(regionNum < MPU_RGD_COUNT);
    DEV_ASSERT(masterNum <= FSL_FEATURE_MPU_MAX_LOW_MASTER_NUMBER);
    DEV_ASSERT(accessRightsPtr != NULL);
#endif
    uint32_t accRight;
    uint32_t accMask;
    uint32_t accShift;
    uint32_t temp;

    /* Prepare Supervisor Mode Access Control and User Mode Access Control value */
    accRight = MPU_RGDAAC_M0UM(accessRightsPtr->userAccessRights) |
               MPU_RGDAAC_M0SM(accessRightsPtr->superAccessRights);
    accMask  = MPU_RGDAAC_M0UM_MASK | MPU_RGDAAC_M0SM_MASK;
#if FSL_FEATURE_MPU_HAS_PROCESS_IDENTIFIER
    if (masterNum < FSL_FEATURE_MPU_MASTER_DMA)
    {
        accRight |= MPU_RGDAAC_M0PE(accessRightsPtr->processIdentifierEnable ? 1UL : 0UL);
    }
    accMask  |= MPU_RGDAAC_M0PE_MASK;
#endif

    /* Shift FSL_FEATURE_MPU_LOW_MASTER_CONTROL_WIDTH-bit field defining
     * separate privilege rights depend on bus master number
     */
    accShift = ((uint32_t)masterNum * FSL_FEATURE_MPU_LOW_MASTER_CONTROL_WIDTH);
    accRight = accRight << accShift;
    accMask  = accMask << accShift;

    /* Set access rights */
    temp = base->RGDAAC[regionNum];
    temp = (temp & ~accMask) | accRight;
    base->RGDAAC[regionNum] = temp;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : MPU_HAL_SetAlternateHighMasterAccessRights
 * Description   : Sets access permission for master which has only read and write
 * permissions in a specific region by alternate register.
 *
 * Implements    : MPU_HAL_SetAlternateHighMasterAccessRights_Activity
 *END**************************************************************************/
void MPU_HAL_SetAlternateHighMasterAccessRights(MPU_Type * const base,
                                                uint8_t regionNum,
                                                uint8_t masterNum,
                                                const mpu_high_masters_access_rights_t *accessRightsPtr)
{
#ifdef DEV_ERROR_DETECT
    DEV_ASSERT(regionNum < MPU_RGD_COUNT);
    DEV_ASSERT(masterNum > FSL_FEATURE_MPU_MAX_LOW_MASTER_NUMBER);
    DEV_ASSERT(masterNum <= FSL_FEATURE_MPU_MAX_HIGH_MASTER_NUMBER);
    DEV_ASSERT(accessRightsPtr != NULL);
#endif
    uint32_t accRight;
    uint32_t accMask;
    uint32_t accShift;
    uint32_t temp;

    /* Prepare Read Enable and Write Enable value */
    accRight = MPU_RGDAAC_M4RE(accessRightsPtr->readEnable ? 1UL : 0UL) |
               MPU_RGDAAC_M4WE(accessRightsPtr->writeEnable ? 1UL : 0UL);

    /* Low master number is numbered from 0 so master number count will be FSL_FEATURE_MPU_MAX_LOW_MASTER_NUMBER added to 1 */
    accShift = FSL_FEATURE_MPU_HIGH_MASTER_CONTROL_WIDTH * (masterNum -(FSL_FEATURE_MPU_MAX_LOW_MASTER_NUMBER + 1U));
    accMask  = (MPU_RGDAAC_M4RE_MASK | MPU_RGDAAC_M4WE_MASK) << accShift;
    accRight = accRight << accShift;

    /* Set access right */
    temp = base->RGDAAC[regionNum];
    temp = (temp & ~accMask) | accRight;
    base->RGDAAC[regionNum] = temp;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : MPU_HAL_Init
 * Description   : Initialize MPU module and all regions will be invalid after cleared access permission.
 *
 * Implements    : MPU_HAL_Init_Activity
 *END**************************************************************************/
void MPU_HAL_Init(MPU_Type * const base)
{
    uint8_t i;

    /* Global disable for the MPU */
    MPU_HAL_Disable(base);

    /* Sets default access right for region 0 */
    base->RGDAAC[0U] = DEFAULT_ACCESS_RIGHT;

    /* Clear access permission form Region 1 because the MPU includes default settings and protections for
     * the Region Descriptor 0 (RGD0) such that the Debugger always has access to the entire address space
     * and those rights cannot be changed by the core or any other bus master.
     */
    for (i = 1U; i < MPU_RGD_COUNT; i++)
    {
        MPU_HAL_SetRegionAddr(base, i, 0U, DEFAULT_END_ADDRESS);
        base->RGD[i].WORD2 = 0U;
        base->RGD[i].WORD3 = 0U;
    }
}

/*******************************************************************************
 * EOF
 *******************************************************************************/
