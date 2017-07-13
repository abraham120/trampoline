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

#include "fsl_flexio_uart_driver.h"
#include "fsl_flexio_hal.h"
#include "fsl_flexio_common.h"
#include "fsl_clock_manager.h"

/**
 * @page misra_violations MISRA-C:2012 violations
 *
 * @section [global]
 * Violates MISRA 2012 Advisory Directive 4.9, Function-like macro
 * These are very simple macros used for accessing the hardware resources (shifters and timers) 
 * allocated for each driver instance. They help make the code easy to understand.
 */

/*******************************************************************************
 * Variables
 ******************************************************************************/

/*! @cond DRIVER_INTERNAL_USE_ONLY */

/* Constraints used for baud rate computation */
#define DIVIDER_MIN_VALUE  1U
#define DIVIDER_MAX_VALUE  0xFFU

/* Shifters/Timers used for UART simulation The parameter x represents the 
   resourceIndex value for the current driver instance */
#define TX_SHIFTER(x)     (x)
#define RX_SHIFTER(x)     (x)
#define TX_TIMER(x)       (x)
#define RX_TIMER(x)       (x)

/*******************************************************************************
 * Private Functions
 ******************************************************************************/


/*FUNCTION**********************************************************************
 *
 * Function Name : FLEXIO_UART_DRV_ComputeBaudRateDivider
 * Description   : Computes the baud rate divider for a target baud rate
 *
 *END**************************************************************************/
static flexio_status_t FLEXIO_UART_DRV_ComputeBaudRateDivider(flexio_uart_state_t *state, 
                                                              uint32_t baudRate, 
                                                              uint16_t *divider,
                                                              uint32_t inputClock)
{
    uint32_t tmpDiv;

    (void)state;
    /* Compute divider: ((input_clock / baud_rate) / 2) - 1. Round to nearest integer */
    tmpDiv = (inputClock + baudRate) / (2U * baudRate) - 1U;
    /* Enforce upper/lower limits */
    if (tmpDiv < DIVIDER_MIN_VALUE)
    {
        tmpDiv = DIVIDER_MIN_VALUE;
    }
    if (tmpDiv > DIVIDER_MAX_VALUE)
    {
        tmpDiv = DIVIDER_MAX_VALUE;
    }
    
    *divider = (uint16_t)tmpDiv;
    return FLEXIO_STATUS_SUCCESS;
}


/*FUNCTION**********************************************************************
 *
 * Function Name : FLEXIO_UART_DRV_Configure
 * Description   : configures the FLEXIO module for UART
 *
 *END**************************************************************************/
static flexio_status_t FLEXIO_UART_DRV_ConfigureTx(flexio_uart_state_t *state,
                                                   const flexio_uart_user_config_t * userConfigPtr,
                                                   uint32_t inputClock)
{
    FLEXIO_Type *baseAddr;
    uint16_t divider;
    uint16_t bits;
    uint8_t resourceIndex;    /* Index of first used internal resource instance (shifter and timer) */

    baseAddr = g_flexioBase[state->flexioCommon.instance];
    resourceIndex = state->flexioCommon.resourceIndex;

    /* Compute divider. */
    FLEXIO_UART_DRV_ComputeBaudRateDivider(state, userConfigPtr->baudRate, &divider, inputClock);
    bits = userConfigPtr->bitCount;

    /* Configure tx shifter */
    FLEXIO_HAL_SetShifterConfig(baseAddr,
                                TX_SHIFTER(resourceIndex),
                                FLEXIO_SHIFTER_START_BIT_0,
                                FLEXIO_SHIFTER_STOP_BIT_1,
                                FLEXIO_SHIFTER_SOURCE_PIN);
    FLEXIO_HAL_SetShifterControl(baseAddr,
                                 TX_SHIFTER(resourceIndex),
                                 FLEXIO_SHIFTER_MODE_TRANSMIT,
                                 userConfigPtr->dataPin,             /* output on tx pin */
                                 FLEXIO_PIN_POLARITY_HIGH,
                                 FLEXIO_PIN_CONFIG_OUTPUT,
                                 TX_TIMER(resourceIndex),
                                 FLEXIO_TIMER_POLARITY_POSEDGE);

    /* Configure tx timer */
    FLEXIO_HAL_SetTimerCompare(baseAddr, TX_TIMER(resourceIndex), (((bits << 1U) - 1U) << 8U) + divider);
    FLEXIO_HAL_SetTimerConfig(baseAddr, 
                              TX_TIMER(resourceIndex),
                              FLEXIO_TIMER_START_BIT_ENABLED,
                              FLEXIO_TIMER_STOP_BIT_TIM_DIS,
                              FLEXIO_TIMER_ENABLE_TRG_HIGH,         /* enable when Tx data is available */
                              FLEXIO_TIMER_DISABLE_TIM_CMP,
                              FLEXIO_TIMER_RESET_NEVER,
                              FLEXIO_TIMER_DECREMENT_CLK_SHIFT_TMR, /* decrement on FlexIO clock */
                              FLEXIO_TIMER_INITOUT_ONE);
    FLEXIO_HAL_SetTimerControl(baseAddr,
                               TX_TIMER(resourceIndex),
                               (TX_SHIFTER(resourceIndex) << 2U)+1U,  /* trigger on tx shifter status flag */
                               FLEXIO_TRIGGER_POLARITY_LOW,
                               FLEXIO_TRIGGER_SOURCE_INTERNAL,
                               0U,                                     /* pin unused */
                               FLEXIO_PIN_POLARITY_HIGH,
                               FLEXIO_PIN_CONFIG_DISABLED,
                               FLEXIO_TIMER_MODE_8BIT_BAUD);

    return FLEXIO_STATUS_SUCCESS;
}



/*FUNCTION**********************************************************************
 *
 * Function Name : FLEXIO_UART_DRV_Configure
 * Description   : configures the FLEXIO module for UART
 *
 *END**************************************************************************/
static flexio_status_t FLEXIO_UART_DRV_ConfigureRx(flexio_uart_state_t *state,
                                                   const flexio_uart_user_config_t * userConfigPtr,
                                                   uint32_t inputClock)
{
    FLEXIO_Type *baseAddr;
    uint16_t divider;
    uint16_t bits;
    uint8_t resourceIndex;    /* Index of first used internal resource instance (shifter and timer) */

    baseAddr = g_flexioBase[state->flexioCommon.instance];
    resourceIndex = state->flexioCommon.resourceIndex;

    /* Compute divider. */
    FLEXIO_UART_DRV_ComputeBaudRateDivider(state, userConfigPtr->baudRate, &divider, inputClock);
    bits = userConfigPtr->bitCount;

    /* Configure rx shifter */
    FLEXIO_HAL_SetShifterConfig(baseAddr,
                                RX_SHIFTER(resourceIndex),
                                FLEXIO_SHIFTER_START_BIT_0,
                                FLEXIO_SHIFTER_STOP_BIT_1,
                                FLEXIO_SHIFTER_SOURCE_PIN);
    FLEXIO_HAL_SetShifterControl(baseAddr,
                                 RX_SHIFTER(resourceIndex),
                                 FLEXIO_SHIFTER_MODE_RECEIVE,
                                 userConfigPtr->dataPin,             /* input from rx pin */
                                 FLEXIO_PIN_POLARITY_HIGH,
                                 FLEXIO_PIN_CONFIG_DISABLED,
                                 RX_TIMER(resourceIndex),
                                 FLEXIO_TIMER_POLARITY_NEGEDGE);

    /* Configure rx timer */
    FLEXIO_HAL_SetTimerCompare(baseAddr, RX_TIMER(resourceIndex), (((bits << 1U) - 1U) << 8U) + divider);
    FLEXIO_HAL_SetTimerConfig(baseAddr, 
                              RX_TIMER(resourceIndex),
                              FLEXIO_TIMER_START_BIT_ENABLED,
                              FLEXIO_TIMER_STOP_BIT_TIM_DIS,
                              FLEXIO_TIMER_ENABLE_PIN_POSEDGE,         /* enable when data is available */
                              FLEXIO_TIMER_DISABLE_TIM_CMP,
                              FLEXIO_TIMER_RESET_PIN_RISING,
                              FLEXIO_TIMER_DECREMENT_CLK_SHIFT_TMR,    /* decrement on FlexIO clock */
                              FLEXIO_TIMER_INITOUT_ONE_RESET);
    FLEXIO_HAL_SetTimerControl(baseAddr,
                               RX_TIMER(resourceIndex),
                               0U,                                      /* trigger unused */
                               FLEXIO_TRIGGER_POLARITY_HIGH,
                               FLEXIO_TRIGGER_SOURCE_EXTERNAL,
                               userConfigPtr->dataPin,                   /* input from rx pin */
                               FLEXIO_PIN_POLARITY_LOW,
                               FLEXIO_PIN_CONFIG_DISABLED,
                               FLEXIO_TIMER_MODE_8BIT_BAUD);

    return FLEXIO_STATUS_SUCCESS;
}


/*FUNCTION**********************************************************************
 *
 * Function Name : FLEXIO_UART_DRV_EndTransfer
 * Description   : End the current transfer
 *
 *END**************************************************************************/
static void FLEXIO_UART_DRV_EndTransfer(flexio_uart_state_t *state)
{
    FLEXIO_Type *baseAddr;
    uint8_t resourceIndex;    /* Index of first used internal resource instance (shifter and timer) */

    baseAddr = g_flexioBase[state->flexioCommon.instance];
    resourceIndex = state->flexioCommon.resourceIndex;

    /* Disable transfer engine */
    switch (state->driverType)
    {
        case FLEXIO_DRIVER_TYPE_INTERRUPTS:
            /* Disable interrupts for Rx / Tx shifter */
            FLEXIO_HAL_SetShifterInterrupt(baseAddr, (1U << TX_SHIFTER(resourceIndex)), false);
            FLEXIO_HAL_SetShifterErrorInterrupt(baseAddr, (1U << TX_SHIFTER(resourceIndex)), false);
            /* disable timer interrupt  */
            FLEXIO_HAL_SetTimerInterrupt(baseAddr, (1U << TX_TIMER(resourceIndex)), false);
            break;
        case FLEXIO_DRIVER_TYPE_POLLING:
            /* Nothing to do here */
            break;
        case FLEXIO_DRIVER_TYPE_DMA:
            /* for Tx we need to disable timer interrupt */
            FLEXIO_HAL_SetTimerInterrupt(baseAddr, (1U << TX_TIMER(resourceIndex)), false);
            /* Disable the FlexIO DMA request */
            FLEXIO_HAL_SetShifterDMARequest(baseAddr, (1U << TX_SHIFTER(resourceIndex)), false);
            /* Release the DMA channel */
            (void)EDMA_DRV_ReleaseChannel(&(state->dmaChannel));
            break;
        default: 
            /* Impossible type - do nothing */
            break;
    }

    state->driverIdle = true;
}


/*FUNCTION**********************************************************************
 *
 * Function Name : FLEXIO_UART_DRV_StopTransfer
 * Description   : Forcefully stops the current transfer
 *
 *END**************************************************************************/
static void FLEXIO_UART_DRV_StopTransfer(flexio_uart_state_t *state)
{
    FLEXIO_Type *baseAddr;
    uint8_t resourceIndex;    /* Index of first used internal resource instance (shifter and timer) */


    resourceIndex = state->flexioCommon.resourceIndex;
    baseAddr = g_flexioBase[state->flexioCommon.instance];

    /* disable and re-enable timers and shifters to reset them */
    FLEXIO_HAL_SetShifterMode(baseAddr, TX_SHIFTER(resourceIndex), FLEXIO_SHIFTER_MODE_DISABLED);
    FLEXIO_HAL_SetTimerMode(baseAddr, TX_TIMER(resourceIndex), FLEXIO_TIMER_MODE_DISABLED);

    /* clear any leftover error flags */
    FLEXIO_HAL_ClearShifterErrorStatus(baseAddr, TX_SHIFTER(resourceIndex));

    /* end the transfer */
    FLEXIO_UART_DRV_EndTransfer(state);

    /* re-enable timers and shifters */
    if (state->direction == FLEXIO_UART_DIRECTION_TX)
    {
        FLEXIO_HAL_SetShifterMode(baseAddr, TX_SHIFTER(resourceIndex), FLEXIO_SHIFTER_MODE_TRANSMIT);
    }
    else
    {
        FLEXIO_HAL_SetShifterMode(baseAddr, RX_SHIFTER(resourceIndex), FLEXIO_SHIFTER_MODE_RECEIVE);
    }
    FLEXIO_HAL_SetTimerMode(baseAddr, TX_TIMER(resourceIndex), FLEXIO_TIMER_MODE_8BIT_BAUD);
}


/*FUNCTION**********************************************************************
 *
 * Function Name : FLEXIO_UART_DRV_WaitTransferEnd
 * Description   : waits for the end of a blocking transfer
 *
 *END**************************************************************************/
static flexio_status_t FLEXIO_UART_DRV_WaitTransferEnd(flexio_uart_state_t *state, uint32_t timeout)
{
    osif_status_t osifError = OSIF_STATUS_SUCCESS;

    switch (state->driverType)
    {
        case FLEXIO_DRIVER_TYPE_INTERRUPTS:
            /* Wait for transfer to be completed by the IRQ */
            osifError = OSIF_SemaWait(&(state->idleSemaphore), timeout);
            break;
        case FLEXIO_DRIVER_TYPE_POLLING:
            /* Call FLEXIO_UART_DRV_GetStatus() to do the transfer */
            while (FLEXIO_UART_DRV_GetStatus(state, NULL) == FLEXIO_STATUS_BUSY);
            break;
        case FLEXIO_DRIVER_TYPE_DMA:
            /* Wait for transfer completion to be signaled by the DMA or IRQ */
            osifError = OSIF_SemaWait(&(state->idleSemaphore), timeout);
            break;
        default: 
            /* Impossible type - do nothing */
            break;
    }
    if (osifError == OSIF_STATUS_TIMEOUT)
    {
        /* abort current transfer */
        state->status = FLEXIO_STATUS_TIMEOUT;
        FLEXIO_UART_DRV_StopTransfer(state);
    }

    /* blocking transfer is over */
    state->blocking = false;
    return state->status;
}


/*FUNCTION**********************************************************************
 *
 * Function Name : FLEXIO_UART_DRV_ReadData
 * Description   : reads data received by the module
 *
 *END**************************************************************************/
static void FLEXIO_UART_DRV_ReadData(flexio_uart_state_t *state)
{
    FLEXIO_Type *baseAddr;
    uint32_t data;
    uint8_t resourceIndex;    /* Index of first used internal resource instance (shifter and timer) */

#ifdef DEV_ERROR_DETECT
    DEV_ASSERT(state->remainingBytes > 0U);
    DEV_ASSERT(state->data != NULL);
#endif
    baseAddr = g_flexioBase[state->flexioCommon.instance];
    resourceIndex = state->flexioCommon.resourceIndex;

    /* Read data from shifter buffer */
    data = FLEXIO_HAL_ReadShifterBuffer(baseAddr, RX_SHIFTER(resourceIndex), FLEXIO_SHIFTER_RW_MODE_NORMAL);
    data >>= 32U - (uint32_t)(state->bitCount);

    if (state->bitCount <= 8U)
    {
        *(uint8_t *)state->data = (uint8_t)data;
        /* Update rx buffer pointer and remaining bytes count */
        state->data += 1U;
        state->remainingBytes -= 1U;
    }
    else
    {
        /* for more than 8 bits per word 2 bytes are needed */
        *(uint16_t *)state->data = (uint16_t)data;
        /* Update rx buffer pointer and remaining bytes count */
        state->data += 2U;
        state->remainingBytes -= 2U;
    }
}


/*FUNCTION**********************************************************************
 *
 * Function Name : FLEXIO_UART_DRV_WriteData
 * Description   : writes data to be transmitted by the module
 *
 *END**************************************************************************/
static void FLEXIO_UART_DRV_WriteData(flexio_uart_state_t *state)
{
    FLEXIO_Type *baseAddr;
    uint32_t data;
    uint8_t resourceIndex;    /* Index of first used internal resource instance (shifter and timer) */


#ifdef DEV_ERROR_DETECT
    DEV_ASSERT(state->data != NULL);
#endif
    baseAddr = g_flexioBase[state->flexioCommon.instance];
    resourceIndex = state->flexioCommon.resourceIndex;

    if (state->remainingBytes == 0U)
    {
        /* Done transmitting */
        return;
    }
    /* Read data from user buffer and update tx buffer pointer and remaining bytes count */
    if (state->bitCount <= 8U)
    {
        data = (uint32_t)(*(uint8_t *)state->data);
        state->data += 1U;
        state->remainingBytes -= 1U;
    }
    else
    {
        /* for more than 8 bits per word 2 bytes are needed */
        data = (uint32_t)(*(uint16_t *)state->data);
        state->data += 2U;
        state->remainingBytes -= 2U;
    }

    FLEXIO_HAL_WriteShifterBuffer(baseAddr, TX_SHIFTER(resourceIndex), data, FLEXIO_SHIFTER_RW_MODE_NORMAL);
}


/*FUNCTION**********************************************************************
 *
 * Function Name : FLEXIO_UART_DRV_CheckStatusTx
 * Description   : Check status of the UART transmission. This function can be 
 *                 called either in an interrupt routine or directly in polling 
 *                 mode to advance the transfer.
 *
 *END**************************************************************************/
static void FLEXIO_UART_DRV_CheckStatusTx(void *stateStruct)
{
    FLEXIO_Type *baseAddr;
    uint8_t resourceIndex;    /* Index of first used internal resource instance (shifter and timer) */
    flexio_uart_state_t *state;

#ifdef DEV_ERROR_DETECT
    DEV_ASSERT(stateStruct != NULL);
#endif

    state = (flexio_uart_state_t *)stateStruct;
    baseAddr = g_flexioBase[state->flexioCommon.instance];
    resourceIndex = state->flexioCommon.resourceIndex;

    /* No need to check for Tx underflow since timer is controlled by the shifter status flag */
    /* Check for transfer end */
    if ((state->remainingBytes == 0U) && FLEXIO_HAL_GetTimerStatus(baseAddr, TX_TIMER(resourceIndex)))
    {
        /* Clear timer status */
        FLEXIO_HAL_ClearTimerStatus(baseAddr, TX_TIMER(resourceIndex));
        if (FLEXIO_HAL_GetShifterStatus(baseAddr, TX_SHIFTER(resourceIndex)))
        {
            /* Record success if there was no error */
            if (state->status == FLEXIO_STATUS_BUSY)
            {
                state->status = FLEXIO_STATUS_SUCCESS;
            }
            FLEXIO_UART_DRV_EndTransfer(state);
            /* Call callback to announce the end transfer event to the user */
            if (state->callback != NULL)
            {
                state->callback(state, FLEXIO_EVENT_END_TRANSFER, state->callbackParam);
            }
            /* Signal transfer end for blocking transfers */
            if (state->blocking == true)
            {
                (void)OSIF_SemaPost(&(state->idleSemaphore));
            }
        }
    }
    /* Check if transmitter needs more data */
    else if (FLEXIO_HAL_GetShifterStatus(baseAddr, TX_SHIFTER(resourceIndex)) && (state->remainingBytes > 0U))
    {
        FLEXIO_UART_DRV_WriteData(state);
        if (state->remainingBytes == 0U)
        {
            /* Out of data, call callback to allow user to provide a new buffer */
            if (state->callback != NULL)
            {
                state->callback(state, FLEXIO_EVENT_TX_EMPTY, state->callbackParam);
            }
        }
        if ((state->remainingBytes == 0U) && (state->driverType == FLEXIO_DRIVER_TYPE_INTERRUPTS))
        {
            /* transmission completed; disable interrupt */
            FLEXIO_HAL_SetShifterInterrupt(baseAddr, (1U << TX_SHIFTER(resourceIndex)), false);
            /* enable timer interrupt to ensure that transfer is completed */
            FLEXIO_HAL_SetTimerInterrupt(baseAddr, (1U << TX_TIMER(resourceIndex)), true);
        }
    }
}

/*FUNCTION**********************************************************************
 *
 * Function Name : FLEXIO_UART_DRV_CheckStatusRx
 * Description   : Check status of the UART reception. This function can be 
 *                 called either in an interrupt routine or directly in polling 
 *                 mode to advance the transfer.
 *
 *END**************************************************************************/
static void FLEXIO_UART_DRV_CheckStatusRx(void *stateStruct)
{
    FLEXIO_Type *baseAddr;
    uint8_t resourceIndex;    /* Index of first used internal resource instance (shifter and timer) */
    flexio_uart_state_t *state;

#ifdef DEV_ERROR_DETECT
    DEV_ASSERT(stateStruct != NULL);
#endif

    state = (flexio_uart_state_t *)stateStruct;
    baseAddr = g_flexioBase[state->flexioCommon.instance];
    resourceIndex = state->flexioCommon.resourceIndex;

    /* Check for errors */
    if (FLEXIO_HAL_GetShifterErrorStatus(baseAddr, RX_SHIFTER(resourceIndex)))
    {
        state->status = FLEXIO_STATUS_RX_OVERFLOW;
        FLEXIO_HAL_ClearShifterErrorStatus(baseAddr, RX_SHIFTER(resourceIndex));
        state->remainingBytes = 0U;
        /* Continue processing events */
    }
    /* Check if data was received */
    else if (FLEXIO_HAL_GetShifterStatus(baseAddr, RX_SHIFTER(resourceIndex)))
    {
        FLEXIO_UART_DRV_ReadData(state);
        if (state->remainingBytes == 0U)
        {
            /* Out of data, call callback to allow user to provide a new buffer */
            if (state->callback != NULL)
            {
                state->callback(state, FLEXIO_EVENT_RX_FULL, state->callbackParam);
            }
        }
    }
    /* Check if transfer is over */
    if (state->remainingBytes == 0U)
    {
        /* Record success if there was no error */
        if (state->status == FLEXIO_STATUS_BUSY)
        {
            state->status = FLEXIO_STATUS_SUCCESS;
        }
        /* End transfer */
        FLEXIO_UART_DRV_EndTransfer(state);
        /* Call callback to announce the event to the user */
        if (state->callback != NULL)
        {
            state->callback(state, FLEXIO_EVENT_END_TRANSFER, state->callbackParam);
        }
        /* Signal transfer end for blocking transfers */
        if (state->blocking == true)
        {
            (void)OSIF_SemaPost(&(state->idleSemaphore));
        }
    }
}

/*FUNCTION**********************************************************************
 *
 * Function Name : FLEXIO_UART_DRV_CheckStatus
 * Description   : Check status of the UART transfer. This function can be 
 *                 called either in an interrupt routine or directly in polling 
 *                 mode to advance the transfer.
 *
 *END**************************************************************************/
void FLEXIO_UART_DRV_CheckStatus(void *stateStruct)
{
    flexio_uart_state_t *state;

#ifdef DEV_ERROR_DETECT
    DEV_ASSERT(stateStruct != NULL);
#endif

    state = (flexio_uart_state_t *)stateStruct;
    if (state->direction == FLEXIO_UART_DIRECTION_TX)
    {
        FLEXIO_UART_DRV_CheckStatusTx(stateStruct);
    }
    else
    {
        FLEXIO_UART_DRV_CheckStatusRx(stateStruct);
    }
}


/*FUNCTION**********************************************************************
 *
 * Function Name : FLEXIO_UART_DRV_EndDmaTxTransfer
 * Description   : function called at the end of a DMA Tx transfer
 *
 *END**************************************************************************/
static void FLEXIO_UART_DRV_EndDmaTxTransfer(void *stateStruct, edma_chn_status_t status)
{
    flexio_uart_state_t *state;
    uint8_t dmaChn;
    DMA_Type *edmaBase;
    uint32_t byteCount;
    FLEXIO_Type *baseAddr;

#ifdef DEV_ERROR_DETECT
    DEV_ASSERT(stateStruct != NULL);
#endif

    (void)status;
    state = (flexio_uart_state_t *)stateStruct;
    baseAddr = g_flexioBase[state->flexioCommon.instance];

    /* Call callback to allow user to provide a new buffer */
    if (state->callback != NULL)
    {
        state->callback(state, FLEXIO_EVENT_TX_EMPTY, state->callbackParam);
    }
    if (state->remainingBytes == 0U)
    {
        /* No more data to transmit, transmission will stop */
        /* enable timer interrupt to let IRQ ensure that transfer is completed */
        FLEXIO_HAL_SetTimerInterrupt(baseAddr, (1U << TX_TIMER(state->flexioCommon.resourceIndex)), true);
    }
    else
    {
        /* There is more data to transfer, restart DMA channel */
        /* Update buffer address and size */
        dmaChn = state->dmaChannel.channel;
        edmaBase = g_edmaBase[0U];
        if (state->bitCount <= 8U)
        {
            byteCount = 1U;
        }
        else
        {
            byteCount = 2U;
        }
        EDMA_HAL_TCDSetSrcAddr(edmaBase, dmaChn, (uint32_t)(state->data));
        EDMA_HAL_TCDSetMajorCount(edmaBase, dmaChn, state->remainingBytes / byteCount);
        /* Now that this tx is set up, clear remaining bytes count */
        state->remainingBytes = 0U;
        /* Start the channel */
        (void)EDMA_DRV_StartChannel(&(state->dmaChannel));
    }
}


/*FUNCTION**********************************************************************
 *
 * Function Name : FLEXIO_UART_DRV_EndDmaRxTransfer
 * Description   : function called at the end of a DMA Rx transfer
 *
 *END**************************************************************************/
static void FLEXIO_UART_DRV_EndDmaRxTransfer(void *stateStruct, edma_chn_status_t status)
{
    flexio_uart_state_t *state;
    uint8_t dmaChn;
    DMA_Type *edmaBase;
    uint32_t byteCount;

#ifdef DEV_ERROR_DETECT
    DEV_ASSERT(stateStruct != NULL);
#endif

    (void)status;
    state = (flexio_uart_state_t *)stateStruct;

    /* Call callback to allow user to provide a new buffer */
    if (state->callback != NULL)
    {
        state->callback(state, FLEXIO_EVENT_RX_FULL, state->callbackParam);
    }
    if (state->remainingBytes == 0U)
    {
        /* No more data to transmit, reception will stop */
        state->status = FLEXIO_STATUS_SUCCESS;
        FLEXIO_UART_DRV_EndTransfer(state);
        /* Call callback to announce the event to the user */
        if (state->callback != NULL)
        {
            state->callback(state, FLEXIO_EVENT_END_TRANSFER, state->callbackParam);
        }
        /* Signal transfer end for blocking transfers */
        if (state->blocking == true)
        {
            (void)OSIF_SemaPost(&(state->idleSemaphore));
        }
    }
    else
    {
        /* There is more data to transfer, restart DMA channel */
        /* Update buffer address and size */
        dmaChn = state->dmaChannel.channel;
        edmaBase = g_edmaBase[0U];
        if (state->bitCount <= 8U)
        {
            byteCount = 1U;
        }
        else
        {
            byteCount = 2U;
        }
        EDMA_HAL_TCDSetDestAddr(edmaBase, dmaChn, (uint32_t)(state->data));
        EDMA_HAL_TCDSetMajorCount(edmaBase, dmaChn, state->remainingBytes / byteCount);
        /* Now that this tx is set up, clear remaining bytes count */
        state->remainingBytes = 0U;
        /* Start the channel */
        (void)EDMA_DRV_StartChannel(&(state->dmaChannel));
    }
}



/*FUNCTION**********************************************************************
 *
 * Function Name : FLEXIO_UART_DRV_ComputeTxRegAddr
 * Description   : Computes the address of the register used for DMA tx transfer
 *
 *END**************************************************************************/
static uint32_t FLEXIO_UART_DRV_ComputeTxRegAddr(flexio_uart_state_t *state)
{
    uint32_t addr;
    FLEXIO_Type *baseAddr;
    uint8_t shifter;

    baseAddr = g_flexioBase[state->flexioCommon.instance];
    shifter = TX_SHIFTER(state->flexioCommon.resourceIndex);
    addr = (uint32_t)(&(baseAddr->SHIFTBUF[shifter]));
    return addr;
}


/*FUNCTION**********************************************************************
 *
 * Function Name : FLEXIO_UART_DRV_ComputeRxRegAddr
 * Description   : Computes the address of the register used for DMA rx transfer
 *
 *END**************************************************************************/
static uint32_t FLEXIO_UART_DRV_ComputeRxRegAddr(flexio_uart_state_t *state)
{
    uint32_t addr;
    FLEXIO_Type *baseAddr;
    uint8_t shifter;
    uint32_t byteCount;

    if (state->bitCount <= 8U)
    {
        byteCount = 1U;
    }
    else
    {
        byteCount = 2U;
    }
    baseAddr = g_flexioBase[state->flexioCommon.instance];
    shifter = RX_SHIFTER(state->flexioCommon.resourceIndex);
    addr = (uint32_t)(&(baseAddr->SHIFTBUF[shifter])) + (sizeof(uint32_t) - byteCount);
    return addr;
}


/*FUNCTION**********************************************************************
 *
 * Function Name : FLEXIO_UART_DRV_StartTxDmaTransfer
 * Description   : Starts a Tx DMA transfer
 *
 *END**************************************************************************/
static flexio_status_t FLEXIO_UART_DRV_StartTxDmaTransfer(flexio_uart_state_t *state)
{
    dma_request_source_t dmaReq;
    uint32_t instance;
    uint8_t resourceIndex;    /* Index of first used internal resource instance (shifter and timer) */
    uint8_t dmaChn;
    DMA_Type *edmaBase = g_edmaBase[0U];
    FLEXIO_Type *baseAddr;
    edma_status_t dmaStatus;
    edma_transfer_size_t dmaTransferSize;
    uint32_t byteCount;

    baseAddr = g_flexioBase[state->flexioCommon.instance];
    resourceIndex = state->flexioCommon.resourceIndex;
    instance = state->flexioCommon.instance;

    /* Request a DMA channel */
    dmaReq = g_flexioDMASrc[instance][TX_SHIFTER(resourceIndex)];
    dmaStatus = EDMA_DRV_RequestChannel((uint8_t)EDMA_ANY_CHANNEL, dmaReq, &(state->dmaChannel), &dmaChn);
    if (dmaStatus == EDMA_STATUS_FAIL)
    {
        return FLEXIO_STATUS_FAIL;
    }
    /* Configure the transfer control descriptor for the previously allocated channel */
    if (state->bitCount <= 8U)
    {
        dmaTransferSize = EDMA_TRANSFER_SIZE_1B;
        byteCount = 1U;
    }
    else
    {
        dmaTransferSize = EDMA_TRANSFER_SIZE_2B;
        byteCount = 2U;
    }
    (void)EDMA_DRV_ConfigSingleBlockTransfer(&(state->dmaChannel), 
                                             EDMA_TRANSFER_MEM2PERIPH, 
                                             (uint32_t)(state->data),
                                             FLEXIO_UART_DRV_ComputeTxRegAddr(state), 
                                             dmaTransferSize, 
                                             byteCount);
    EDMA_HAL_TCDSetMajorCount(edmaBase, dmaChn, state->remainingBytes / byteCount);
    EDMA_HAL_TCDSetDisableDmaRequestAfterTCDDoneCmd(edmaBase, dmaChn, true);
    /* Now that this transfer is set up, clear remaining bytes count */
    state->remainingBytes = 0U;

    /* Setup callback for DMA tx transfer end */
    (void)EDMA_DRV_InstallCallback(&(state->dmaChannel),
                                   (edma_callback_t)(FLEXIO_UART_DRV_EndDmaTxTransfer),
                                   (void*)(state));
    /* Start tx DMA channel */
    (void)EDMA_DRV_StartChannel(&(state->dmaChannel));

    /* Enable FlexIO DMA requests */
    FLEXIO_HAL_SetShifterDMARequest(baseAddr, (1U << TX_SHIFTER(resourceIndex)), true);

    return FLEXIO_STATUS_SUCCESS;
}


/*FUNCTION**********************************************************************
 *
 * Function Name : FLEXIO_UART_DRV_StartRxDmaTransfer
 * Description   : Starts an Rx DMA transfer
 *
 *END**************************************************************************/
static flexio_status_t FLEXIO_UART_DRV_StartRxDmaTransfer(flexio_uart_state_t *state)
{
    dma_request_source_t dmaReq;
    uint32_t instance;
    uint8_t resourceIndex;    /* Index of first used internal resource instance (shifter and timer) */
    uint8_t dmaChn;
    DMA_Type *edmaBase = g_edmaBase[0U];
    FLEXIO_Type *baseAddr;
    edma_status_t dmaStatus;
    edma_transfer_size_t dmaTransferSize;
    uint32_t byteCount;

    baseAddr = g_flexioBase[state->flexioCommon.instance];
    resourceIndex = state->flexioCommon.resourceIndex;
    instance = state->flexioCommon.instance;

    /* Request a DMA channel */
    dmaReq = g_flexioDMASrc[instance][TX_SHIFTER(resourceIndex)];
    dmaStatus = EDMA_DRV_RequestChannel((uint8_t)EDMA_ANY_CHANNEL, dmaReq, &(state->dmaChannel), &dmaChn);
    if (dmaStatus == EDMA_STATUS_FAIL)
    {
        return FLEXIO_STATUS_FAIL;
    }
    /* Configure the transfer control descriptor for the previously allocated channel */
    if (state->bitCount <= 8U)
    {
        dmaTransferSize = EDMA_TRANSFER_SIZE_1B;
        byteCount = 1U;
    }
    else
    {
        dmaTransferSize = EDMA_TRANSFER_SIZE_2B;
        byteCount = 2U;
    }
    (void)EDMA_DRV_ConfigSingleBlockTransfer(&(state->dmaChannel), 
                                             EDMA_TRANSFER_PERIPH2MEM, 
                                             FLEXIO_UART_DRV_ComputeRxRegAddr(state), 
                                             (uint32_t)(state->data),
                                             dmaTransferSize, 
                                             byteCount);
    EDMA_HAL_TCDSetMajorCount(edmaBase, dmaChn, state->remainingBytes / byteCount);
    EDMA_HAL_TCDSetDisableDmaRequestAfterTCDDoneCmd(edmaBase, dmaChn, true);
    /* Now that this transfer is set up, clear remaining bytes count */
    state->remainingBytes = 0U;

    /* Setup callback for DMA tx transfer end */
    (void)EDMA_DRV_InstallCallback(&(state->dmaChannel),
                                   (edma_callback_t)(FLEXIO_UART_DRV_EndDmaRxTransfer),
                                   (void*)(state));
    /* Start tx DMA channel */
    (void)EDMA_DRV_StartChannel(&(state->dmaChannel));

    /* Enable FlexIO DMA requests */
    FLEXIO_HAL_SetShifterDMARequest(baseAddr, (1U << RX_SHIFTER(resourceIndex)), true);

    return FLEXIO_STATUS_SUCCESS;
}

/*! @endcond */

/*******************************************************************************
 * Code
 ******************************************************************************/

/*FUNCTION**********************************************************************
 *
 * Function Name : FLEXIO_UART_DRV_Init
 * Description   : Initialize the FLEXIO_UART driver
 *
 *END**************************************************************************/
flexio_status_t FLEXIO_UART_DRV_Init(uint32_t instance,
                                     const flexio_uart_user_config_t * userConfigPtr,
                                     flexio_uart_state_t * state)
{
    uint32_t inputClock;
    clock_manager_error_code_t clkErr;
    flexio_status_t retCode;

#ifdef DEV_ERROR_DETECT
    DEV_ASSERT(state != NULL);
    DEV_ASSERT(instance < FLEXIO_INSTANCE_COUNT);
    /* Check that device was initialized */
    DEV_ASSERT(g_flexioDeviceStatePtr[instance] != NULL);
#endif

    /* Get the protocol clock frequency */
    clkErr = CLOCK_SYS_GetFreq(g_flexioClock[instance], &inputClock);
    if ((clkErr != CLOCK_MANAGER_SUCCESS) || (inputClock == 0U))
    {
        /* No clock configured for this module */
        return FLEXIO_STATUS_FAIL;
    }

    /* Initialize the semaphore */
    if (OSIF_SemaCreate(&(state->idleSemaphore), 0U) == OSIF_STATUS_FAIL)
    {
        return FLEXIO_STATUS_FAIL;
    }

    /* Instruct the resource allocator that we need one shifter/timer */
    state->flexioCommon.resourceCount = 1U;
    /* Common FlexIO driver initialization */
    retCode = FLEXIO_DRV_InitDriver(instance, (flexio_common_state_t *)state);
    if (retCode != FLEXIO_STATUS_SUCCESS)
    {   /* Initialization failed, not enough resources */
        (void)OSIF_SemaDestroy(&(state->idleSemaphore));
        return retCode;
    }

    /* Initialize driver-specific context structure */
    state->data = NULL;
    state->remainingBytes = 0U;
    state->callback = userConfigPtr->callback;
    state->callbackParam = userConfigPtr->callbackParam;
    state->blocking = false;
    state->driverType = userConfigPtr->driverType;
    state->direction = userConfigPtr->direction;
    state->status = FLEXIO_STATUS_SUCCESS;
    state->driverIdle = true;
    state->bitCount = userConfigPtr->bitCount;

    if (state->direction == FLEXIO_UART_DIRECTION_TX)
    {
        /* Configure device for UART Tx mode */
        FLEXIO_UART_DRV_ConfigureTx(state, userConfigPtr, inputClock);
    }
    else
    {
        /* Configure device for UART Rx mode */
        FLEXIO_UART_DRV_ConfigureRx(state, userConfigPtr, inputClock);
    }

    /* Set up transfer engine */
    switch (state->driverType)
    {
        case FLEXIO_DRIVER_TYPE_INTERRUPTS:
            if (state->direction == FLEXIO_UART_DIRECTION_TX)
            {
                state->flexioCommon.isr = FLEXIO_UART_DRV_CheckStatusTx;
            }
            else
            {
                state->flexioCommon.isr = FLEXIO_UART_DRV_CheckStatusRx;
            }
            break;
        case FLEXIO_DRIVER_TYPE_POLLING:
            /* Nothing to do here, FLEXIO_UART_DRV_GetStatus() will handle the transfer */
            break;
        case FLEXIO_DRIVER_TYPE_DMA:
            /* For Tx we will still need interrupt to signal end of transfer */
            if (state->direction == FLEXIO_UART_DIRECTION_TX)
            {
                state->flexioCommon.isr = FLEXIO_UART_DRV_CheckStatusTx;
            }
            break;
        default: 
            /* Impossible type - do nothing */
            break;
    }

    return FLEXIO_STATUS_SUCCESS;
}


/*FUNCTION**********************************************************************
 *
 * Function Name : FLEXIO_UART_DRV_Deinit
 * Description   : De-initialize the FLEXIO_UART driver
 *
 *END**************************************************************************/
flexio_status_t FLEXIO_UART_DRV_Deinit(flexio_uart_state_t *state)
{
#ifdef DEV_ERROR_DETECT
    DEV_ASSERT(state != NULL);
#endif

    /* Check if driver is busy */
    if (!state->driverIdle)
    {
        return FLEXIO_STATUS_BUSY;
    }

    /* Destroy the semaphore */
    (void)OSIF_SemaDestroy(&(state->idleSemaphore));

    return FLEXIO_DRV_DeinitDriver((flexio_common_state_t *)state);
}


/*FUNCTION**********************************************************************
 *
 * Function Name : FLEXIO_UART_DRV_SetConfig
 * Description   : Set the baud rate and bit width for any subsequent UART transfer
 *
 *END**************************************************************************/
flexio_status_t FLEXIO_UART_DRV_SetConfig(flexio_uart_state_t * state, 
                                          uint32_t baudRate, 
                                          uint8_t bitCount)
{
    FLEXIO_Type *baseAddr;
    uint16_t divider;
    uint32_t inputClock;
    clock_manager_error_code_t clkErr;
    uint8_t resourceIndex;    /* Index of first used internal resource instance (shifter and timer) */

#ifdef DEV_ERROR_DETECT
    DEV_ASSERT(state != NULL);
    DEV_ASSERT(baudRate > 0U);
    DEV_ASSERT(bitCount > 0U);
    DEV_ASSERT(bitCount <= 16U);
    /* for DMA transfers bitCount must 8 */
    DEV_ASSERT(!((state->driverType == FLEXIO_DRIVER_TYPE_DMA) && (bitCount != 8U)));
#endif

    baseAddr = g_flexioBase[state->flexioCommon.instance];
    resourceIndex = state->flexioCommon.resourceIndex;

    /* Check if driver is busy */
    if (!state->driverIdle)
    {
        return FLEXIO_STATUS_BUSY;
    }
    /* Get the protocol clock frequency */
    clkErr = CLOCK_SYS_GetFreq(g_flexioClock[state->flexioCommon.instance], &inputClock);
    if ((clkErr != CLOCK_MANAGER_SUCCESS) || (inputClock == 0U))
    {
        /* No clock configured for this module */
        return FLEXIO_STATUS_FAIL;
    }

    /* Compute divider */
    FLEXIO_UART_DRV_ComputeBaudRateDivider(state, baudRate, &divider, inputClock);

    if (state->direction == FLEXIO_UART_DIRECTION_TX)
    {
        /* Configure tx timer */
        FLEXIO_HAL_SetTimerCompare(baseAddr, TX_TIMER(resourceIndex), (((bitCount << 1U) - 1U) << 8U) + divider);
    }
    else
    {
        /* Configure rx timer */
        FLEXIO_HAL_SetTimerCompare(baseAddr, RX_TIMER(resourceIndex), (((bitCount << 1U) - 1U) << 8U) + divider);
    }

    state->bitCount = bitCount;

    return FLEXIO_STATUS_SUCCESS;
}


/*FUNCTION**********************************************************************
 *
 * Function Name : FLEXIO_UART_DRV_GetBaudRate
 * Description   : Get the currently configured baud rate
 *
 *END**************************************************************************/
flexio_status_t FLEXIO_UART_DRV_GetBaudRate(flexio_uart_state_t *state, uint32_t *baudRate)
{
    FLEXIO_Type *baseAddr;
    uint32_t inputClock;
    uint32_t divider;
    uint16_t timerCmp;
    clock_manager_error_code_t clkErr;
    uint8_t resourceIndex;    /* Index of first used internal resource instance (shifter and timer) */

#ifdef DEV_ERROR_DETECT
    DEV_ASSERT(state != NULL);
#endif

    baseAddr = g_flexioBase[state->flexioCommon.instance];
    resourceIndex = state->flexioCommon.resourceIndex;

    /* Get the protocol clock frequency */
    clkErr = CLOCK_SYS_GetFreq(g_flexioClock[state->flexioCommon.instance], &inputClock);
    if ((clkErr != CLOCK_MANAGER_SUCCESS) || (inputClock == 0U))
    {
        /* No clock configured for this module */
        return FLEXIO_STATUS_FAIL;
    }

    /* Get the currently configured divider */
    timerCmp = FLEXIO_HAL_GetTimerCompare(baseAddr, TX_TIMER(resourceIndex));
    divider = (uint32_t)(timerCmp & 0x00FFU);

    /* Compute baud rate: input_clock / (2 * (divider + 1)). Round to nearest integer */
    *baudRate = (inputClock + divider + 1U) / (2U * (divider + 1U));

    return FLEXIO_STATUS_SUCCESS;
}



/*FUNCTION**********************************************************************
 *
 * Function Name : FLEXIO_UART_DRV_SendData
 * Description   : Perform a non-blocking UART transmission
 *
 *END**************************************************************************/
flexio_status_t FLEXIO_UART_DRV_SendData(flexio_uart_state_t *state,
                                         const uint8_t * txBuff,
                                         uint32_t txSize)
{
    FLEXIO_Type *baseAddr;
    uint8_t resourceIndex;    /* Index of first used internal resource instance (shifter and timer) */
    flexio_status_t retCode;
    flexio_status_t status;

#ifdef DEV_ERROR_DETECT
    DEV_ASSERT(state != NULL);
    DEV_ASSERT(txBuff != NULL);
    DEV_ASSERT(txSize > 0U);
    DEV_ASSERT(state->direction == FLEXIO_UART_DIRECTION_TX)
#endif

    baseAddr = g_flexioBase[state->flexioCommon.instance];
    resourceIndex = state->flexioCommon.resourceIndex;

    /* Check if driver is busy */
    if (!state->driverIdle)
    {
        return FLEXIO_STATUS_BUSY;
    }

    status = state->status;
    state->data = (uint8_t *)txBuff;
    state->remainingBytes = txSize;
    state->status = FLEXIO_STATUS_BUSY;
    state->driverIdle = false;

    /* Enable transfer engine */
    switch (state->driverType)
    {
        case FLEXIO_DRIVER_TYPE_INTERRUPTS:
            /* Enable interrupts for Tx shifter */
            FLEXIO_HAL_SetShifterInterrupt(baseAddr, (1U << TX_SHIFTER(resourceIndex)), true);
            FLEXIO_HAL_SetShifterErrorInterrupt(baseAddr, (1U << TX_SHIFTER(resourceIndex)), true);
            break;
        case FLEXIO_DRIVER_TYPE_POLLING:
            /* Call FLEXIO_UART_DRV_CheckStatus once to send the first byte */
            FLEXIO_UART_DRV_CheckStatus(state);
            break;
        case FLEXIO_DRIVER_TYPE_DMA:
            retCode = FLEXIO_UART_DRV_StartTxDmaTransfer(state);
            if (retCode != FLEXIO_STATUS_SUCCESS)
            {
                /* Restore status and idle indicator */
                state->status = status;
                state->driverIdle = true;
                return retCode;
            }
            break;
        default: 
            /* Impossible type - do nothing */
            break;
    }

    return FLEXIO_STATUS_SUCCESS;
}


/*FUNCTION**********************************************************************
 *
 * Function Name : FLEXIO_UART_DRV_SendDataBlocking
 * Description   : Perform a blocking UART transmission
 *
 *END**************************************************************************/
flexio_status_t FLEXIO_UART_DRV_SendDataBlocking(flexio_uart_state_t *state,
                                                 const uint8_t * txBuff,
                                                 uint32_t txSize,
                                                 uint32_t timeout)
{
    flexio_status_t status;

    /* mark transfer as blocking */
    if (state->driverType != FLEXIO_DRIVER_TYPE_POLLING)
    {
        state->blocking = true;
    }
    /* Call FLEXIO_UART_DRV_SendData to start transfer */
    status = FLEXIO_UART_DRV_SendData(state, txBuff, txSize);
    if (status != FLEXIO_STATUS_SUCCESS)
    {
        /* Transfer could not be started */
        state->blocking = false;
        return status; 
    }

    /* Wait for transfer to end */
    return FLEXIO_UART_DRV_WaitTransferEnd(state, timeout);
}


/*FUNCTION**********************************************************************
 *
 * Function Name : FLEXIO_UART_DRV_ReceiveData
 * Description   : Perform a non-blocking UART reception
 *
 *END**************************************************************************/
flexio_status_t FLEXIO_UART_DRV_ReceiveData(flexio_uart_state_t *state,
                                                     uint8_t * rxBuff,
                                                     uint32_t rxSize)
{
    FLEXIO_Type *baseAddr;
    uint8_t resourceIndex;    /* Index of first used internal resource instance (shifter and timer) */
    flexio_status_t retCode;
    flexio_status_t status;

#ifdef DEV_ERROR_DETECT
    DEV_ASSERT(state != NULL);
    DEV_ASSERT(rxBuff != NULL);
    DEV_ASSERT(rxSize > 0U);
    DEV_ASSERT(state->direction == FLEXIO_UART_DIRECTION_RX)
#endif

    baseAddr = g_flexioBase[state->flexioCommon.instance];
    resourceIndex = state->flexioCommon.resourceIndex;

    /* Check if driver is busy */
    if (!state->driverIdle)
    {
        return FLEXIO_STATUS_BUSY;
    }

    status = state->status;
    state->data = rxBuff;
    state->remainingBytes = rxSize;
    state->status = FLEXIO_STATUS_BUSY;
    state->driverIdle = false;

    /* Enable transfer engine */
    switch (state->driverType)
    {
        case FLEXIO_DRIVER_TYPE_INTERRUPTS:
            /* Enable interrupts for Rx shifter */
            FLEXIO_HAL_SetShifterInterrupt(baseAddr, (1U << RX_SHIFTER(resourceIndex)), true);
            FLEXIO_HAL_SetShifterErrorInterrupt(baseAddr, (1U << RX_SHIFTER(resourceIndex)), true);
            break;
        case FLEXIO_DRIVER_TYPE_POLLING:
            /* Call FLEXIO_UART_DRV_CheckStatus once to send the first byte */
            FLEXIO_UART_DRV_CheckStatus(state);
            break;
        case FLEXIO_DRIVER_TYPE_DMA:
            retCode = FLEXIO_UART_DRV_StartRxDmaTransfer(state);
            if (retCode != FLEXIO_STATUS_SUCCESS)
            {
                /* Restore status and idle indicator */
                state->status = status;
                state->driverIdle = true;
                return retCode;
            }
            break;
        default: 
            /* Impossible type - do nothing */
            break;
    }

    return FLEXIO_STATUS_SUCCESS;
}


/*FUNCTION**********************************************************************
 *
 * Function Name : FLEXIO_UART_DRV_ReceiveDataBlocking
 * Description   : Perform a blocking UART reception
 *
 *END**************************************************************************/
flexio_status_t FLEXIO_UART_DRV_ReceiveDataBlocking(flexio_uart_state_t *state,
                                                    uint8_t * rxBuff,
                                                    uint32_t rxSize,
                                                    uint32_t timeout)
{
    flexio_status_t status;

    /* mark transfer as blocking */
    if (state->driverType != FLEXIO_DRIVER_TYPE_POLLING)
    {
        state->blocking = true;
    }
    /* Call FLEXIO_UART_DRV_ReceiveData to start transfer */
    status = FLEXIO_UART_DRV_ReceiveData(state, rxBuff, rxSize);
    if (status != FLEXIO_STATUS_SUCCESS)
    {
        /* Transfer could not be started */
        state->blocking = false;
        return status; 
    }

    /* Wait for transfer to end */
    return FLEXIO_UART_DRV_WaitTransferEnd(state, timeout);
}


/*FUNCTION**********************************************************************
 *
 * Function Name : FLEXIO_UART_DRV_TransferAbort
 * Description   : Aborts a non-blocking UART transfer
 *
 *END**************************************************************************/
flexio_status_t FLEXIO_UART_DRV_TransferAbort(flexio_uart_state_t *state)
{
#ifdef DEV_ERROR_DETECT
    DEV_ASSERT(state != NULL);
#endif

    /* Check if driver is busy */
    if (state->driverIdle)
    {
        return FLEXIO_STATUS_FAIL;
    }

    state->status = FLEXIO_STATUS_ABORTED;
    FLEXIO_UART_DRV_StopTransfer(state);

    return FLEXIO_STATUS_SUCCESS;
}


/*FUNCTION**********************************************************************
 *
 * Function Name : FLEXIO_UART_DRV_GetStatus
 * Description   : Get the status of the current non-blocking UART transaction
 *
 *END**************************************************************************/
flexio_status_t FLEXIO_UART_DRV_GetStatus(flexio_uart_state_t *state, uint32_t *bytesRemaining)
{
#ifdef DEV_ERROR_DETECT
    DEV_ASSERT(state != NULL);
#endif

    if ((!state->driverIdle) && (state->driverType == FLEXIO_DRIVER_TYPE_POLLING))
    {
        /* In polling mode advance the UART transfer here */
        FLEXIO_UART_DRV_CheckStatus(state);
    }

    if (bytesRemaining != NULL)
    {
        *bytesRemaining = state->remainingBytes;
    }

    if (!state->driverIdle)
    {
        return FLEXIO_STATUS_BUSY;
    }
    else
    {
        return state->status;
    }
}


/*FUNCTION**********************************************************************
 *
 * Function Name : FLEXIO_UART_DRV_SetRxBuffer
 * Description   : Provide a buffer for receiving data.
 *
 *END**************************************************************************/
flexio_status_t FLEXIO_UART_DRV_SetRxBuffer(flexio_uart_state_t *state,
                                            uint8_t * rxBuff,
                                            uint32_t rxSize)
{
#ifdef DEV_ERROR_DETECT
    DEV_ASSERT(state != NULL);
    DEV_ASSERT(rxBuff != NULL);
    DEV_ASSERT(rxSize > 0U);
#endif
    state->data = rxBuff;
    state->remainingBytes = rxSize;

    return FLEXIO_STATUS_SUCCESS;
}


/*FUNCTION**********************************************************************
 *
 * Function Name : FLEXIO_UART_DRV_SetTxBuffer
 * Description   : Provide a buffer for transmitting data.
 *
 *END**************************************************************************/
flexio_status_t FLEXIO_UART_DRV_SetTxBuffer(flexio_uart_state_t *state,
                                            uint8_t * txBuff,
                                            uint32_t txSize)
{
#ifdef DEV_ERROR_DETECT
    DEV_ASSERT(state != NULL);
    DEV_ASSERT(txBuff != NULL);
    DEV_ASSERT(txSize > 0U);
#endif
    state->data = (uint8_t *)txBuff;
    state->remainingBytes = txSize;

    return FLEXIO_STATUS_SUCCESS;
}


/*******************************************************************************
 * EOF
 ******************************************************************************/
