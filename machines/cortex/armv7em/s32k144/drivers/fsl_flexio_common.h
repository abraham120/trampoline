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

#ifndef FSL_FLEXIO_COMMON_DRIVER_H
#define FSL_FLEXIO_COMMON_DRIVER_H

#include <stddef.h>
#include <stdbool.h>
#include "fsl_device_registers.h"
#include "fsl_flexio_hal.h"
#include "fsl_flexio.h"
#include "fsl_edma_driver.h"


 /*******************************************************************************
 * Variables
 ******************************************************************************/

/* Table of base addresses for FLEXIO instances. */
extern FLEXIO_Type * const g_flexioBase[FLEXIO_INSTANCE_COUNT];

/* Pointer to device state structures. This structure contains data common to all drivers on one device */
extern flexio_device_state_t *g_flexioDeviceStatePtr[FLEXIO_INSTANCE_COUNT];

/* Table for FLEXIO IRQ numbers */
extern const IRQn_Type g_flexioIrqId[FLEXIO_INSTANCE_COUNT];

/* PCC clock sources, for getting the input clock frequency */
extern const clock_names_t g_flexioClock[FLEXIO_INSTANCE_COUNT];

/* FlexIO DMA request sources */
extern const dma_request_source_t g_flexioDMASrc[FLEXIO_INSTANCE_COUNT][FSL_FEATURE_FLEXIO_MAX_SHIFTER_COUNT];

/*******************************************************************************
 * Enumerations.
 ******************************************************************************/


/*******************************************************************************
* Definitions
******************************************************************************/


/*******************************************************************************
 * API
 ******************************************************************************/


#if defined(__cplusplus)
extern "C" {
#endif


/*FUNCTION**********************************************************************
 *
 * Function Name : FLEXIO_DRV_InitDriver
 * Description   : Initializes an instance of FlexIO driver
 *
 *END**************************************************************************/
flexio_status_t FLEXIO_DRV_InitDriver(uint32_t instance, flexio_common_state_t *driver);


/*FUNCTION**********************************************************************
 *
 * Function Name : FLEXIO_DRV_DeinitDriver
 * Description   : De-initializes an instance of FlexIO driver
 *
 *END**************************************************************************/
flexio_status_t FLEXIO_DRV_DeinitDriver(const flexio_common_state_t *driver);


#if defined(__cplusplus)
}
#endif


#endif /* FSL_FLEXIO_COMMON_DRIVER_H */
/*******************************************************************************
 * EOF
 ******************************************************************************/
