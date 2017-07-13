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

#ifndef FSL_FLEXIO_DRIVER_H
#define FSL_FLEXIO_DRIVER_H

#include <stddef.h>
#include <stdbool.h>
#include "fsl_device_registers.h"
#include "fsl_flexio_hal.h"
#include "fsl_osif.h"

/*!
 * @addtogroup flexio_drv
 * @{
 */

/*******************************************************************************
 * Enumerations.
 ******************************************************************************/

/*! @brief FLEXIO drivers status codes
 * Implements : flexio_status_t_Class
 */
typedef enum
{
    FLEXIO_STATUS_SUCCESS            = 0U,  /*!< Operation successful */
    FLEXIO_STATUS_FAIL               = 1U,  /*!< Operation failed */
    FLEXIO_STATUS_SIZE               = 2U,  /*!< Transfer size is not in admissible range */
    FLEXIO_STATUS_BUSY               = 3U,  /*!< Already busy with a transfer */
    FLEXIO_STATUS_BUS_BUSY           = 4U,  /*!< Bus is busy, cannot start transfer */
    FLEXIO_STATUS_ABORTED            = 5U,  /*!< Transfer aborted */
    FLEXIO_STATUS_NACK               = 6U,  /*!< Received NACK */
    FLEXIO_STATUS_TX_UNDERFLOW       = 7U,  /*!< Transmitter underflow */
    FLEXIO_STATUS_RX_OVERFLOW        = 8U,  /*!< Receiver overflow */
    FLEXIO_STATUS_TIMEOUT            = 9U,  /*!< Timeout during blocking transfers */
} flexio_status_t;

/*! @brief Driver type: interrupts/polling/DMA
 * Implements : flexio_driver_type_t_Class
 */
typedef enum
{
    FLEXIO_DRIVER_TYPE_INTERRUPTS    = 0U,  /*!< Driver uses interrupts for data transfers */
    FLEXIO_DRIVER_TYPE_POLLING       = 1U,  /*!< Driver is based on polling */
    FLEXIO_DRIVER_TYPE_DMA           = 2U,  /*!< Driver uses DMA for data transfers */
} flexio_driver_type_t;

/*! @brief flexio events
 * Implements : flexio_event_t_Class
 */
typedef enum
{
    FLEXIO_EVENT_RX_FULL      = 0x00U,    /*!< Rx buffer is full */
    FLEXIO_EVENT_TX_EMPTY     = 0x01U,    /*!< Tx buffer is empty */
    FLEXIO_EVENT_END_TRANSFER = 0x02U,    /*!< The current transfer is ending */
} flexio_event_t;


/*******************************************************************************
* Definitions
******************************************************************************/


/*!
 * @brief flexio callback function
 *
 * Callback functions are called by flexio drivers when relevant events must be reported.
 * See type flexio_event_t for a list of events. The callback can then react to the event, for example
 * providing the buffers for transmission or reception, or waking a task to use the received data. Note
 * that callback functions are called from interrupts, so the callback execution time should be as
 * small as possible.
 */
typedef void (*flexio_callback_t)(void *driverState, flexio_event_t event, void *userData);


/*! @cond DRIVER_INTERNAL_USE_ONLY */

/*
 * FlexIO interrupt service routine
 */
typedef void (*flexio_isr_t)(void *isrParam);

/*
 * FlexIO common context structure
 * This is a common structure used by all FlexIO drivers as a part of their context structure.
 * It is needed for common operations such as interrupt handling.
 */
typedef struct
{
    flexio_isr_t isr;         /* Interrupt handler for this driver instance */
    uint32_t instance;        /* FlexIO device instance number */
    uint8_t resourceCount;    /* Count of internal resources used (shifters and timers) */
    uint8_t resourceIndex;    /* Index of first used internal resource instance (shifter and timer) */
} flexio_common_state_t;

/*
 * FlexIO device context structure
 * This is a structure containing data common to all drivers on one device
 */
typedef struct
{
    uint8_t resourceAllocation;    /* Mask to keep track of resources allocated on current device */
    mutex_t resourceLock;          /* Mutex for guarding channel allocation. */
    /* Array of pointers to runtime state structures. Each FlexIO instance can have at most
       one driver instance per shifter. */
    flexio_common_state_t *flexioStatePtr[FSL_FEATURE_FLEXIO_MAX_SHIFTER_COUNT];
} flexio_device_state_t;

/*! @endcond */


/*******************************************************************************
 * API
 ******************************************************************************/
/*!
 * @name FLEXIO_I2C Driver
 * @{
 */

#if defined(__cplusplus)
extern "C" {
#endif

/*!
 * @brief Initializes the FlexIO device
 *
 * This function resets the FlexIO device, enables interrupts
 * in interrupt manager and enables the device.
 *
 * @param instance  FLEXIO peripheral instance number
 * @return    Error or success status returned by API
 */
flexio_status_t FLEXIO_DRV_InitDevice(uint32_t instance, flexio_device_state_t *deviceState);


/*!
 * @brief De-initializes the FlexIO device
 *
 * This function de-initializes the FlexIO device.
 *
 * @param instance  FLEXIO peripheral instance number
 * @return    Error or success status returned by API
 */
flexio_status_t FLEXIO_DRV_DeinitDevice(uint32_t instance);


/*!
 * @brief Resets the FlexIO device
 *
 * This function resets the FlexIO device.
 *
 * @param instance  FLEXIO peripheral instance number
 * @return    Error or success status returned by API
 */
flexio_status_t FLEXIO_DRV_Reset(uint32_t instance);



/*! @}*/
#if defined(__cplusplus)
}
#endif

/*! @}*/

#endif /* FSL_FLEXIO_DRIVER_H */
/*******************************************************************************
 * EOF
 ******************************************************************************/
