/*
 * Copyright (c) 2015-2016, Freescale Semiconductor, Inc.
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
 * @fsl_lpspi_slave_driver.c
 *
 * @page misra_violations MISRA-C:2012 violations
 *
 * @section [global]
 * Violates MISRA 2012 Required Rule 1.3, Taking address of near auto variable.
 * The code is not dynamically linked. An absolute stack address is obtained
 * when taking the address of the near auto variable. A source of error in writing
 * dynamic code is that the stack segment may be different from the data segment.
 *
 * @section [global]
 * Violates MISRA 2012 Advisory Rule 2.7, Symbol 'status' (line 783) not referenced.
 * This parameter is not used because the DMA callback doesn't need this, but must be defined to
 * ensure the API compatibility for callback.
 *
 * @section [global]
 * Violates MISRA 2012 Required Rule 5.1, Object/function previously declared.
 * This requirement is fulfilled since the function is declared as external in and only in
 * one configuration C file.
 *
 * @section [global]
 * Violates MISRA 2012 Required Rule 5.2, Identifier clash.
 * The supported compilers use more than 31 significant characters for identifiers.
 *
 * @section [global]
 * Violates MISRA 2012 Required Rule 5.4, Identifier clash.
 * The supported compilers use more than 31 significant characters for identifiers.
 *
 * @section [global]
 * Violates MISRA 2012 Required Rule 5.5, Identifier clash.
 * The supported compilers use more than 31 significant characters for identifiers.
 *
 * @section [global]
 * Violates MISRA 2012 Advisory Rule 8.7, External could be made static.
 * Function is defined for usage by application code.
 * 
 * @section [global]
 * Violates MISRA 2012 Advisory Rule 8.9, Could define variable at block scope.
 * The variables are defined in the common source file and this rule can't be
 * applied.
 *
 * @section [global]
 * Violates MISRA 2012 Advisory Rule 11.4, Conversion between a pointer and integer type.
 * This conversion is required because the converted values are the addresses used in DMA transfer.
 *
 * @section [global]
 * Violates MISRA 2012 Required Rule 11.6, Cast from pointer to unsigned long.
 * The cast is required to initialize a DMA transfer. The converted value is the address of a buffer.
 * Cast from unsigned long to pointer. The cast is required to perform a conversion between a pointer
 * and an unsigned long define, representing an address or vice versa.
 *
 * @section [global]
 * Violates MISRA 2012 Advisory Rule 15.5, Return statement before end of function.
 * The return statement before end of function is used for simpler code
 * structure and better readability.
 *
 */

#include <string.h>
#include "fsl_lpspi_slave_driver.h"
#include "fsl_clock_manager.h"
#include "fsl_interrupt_manager.h"
#include "fsl_lpspi_shared_function.h"
#include "S32K144_features.h"


/*******************************************************************************
 * Definitions
 ******************************************************************************/

/* Callback for DMA transfer done.*/
static void LPSPI_DRV_SlaveCompleteDMATransfer(void* parameter, edma_chn_status_t status);

/*******************************************************************************
 * Code
 ******************************************************************************/
 /*
  * Implements : LPSPI_DRV_SlaveInit_Activity
  */
lpspi_status_t LPSPI_DRV_SlaveInit(uint32_t instance,
                               lpspi_state_t * lpspiState,
                               const lpspi_slave_config_t * slaveConfig)
{
    if (g_lpspiStatePtr[instance] != NULL)
    {
        return LPSPI_STATUS_INITIALIZED;
    }
    if(lpspiState == NULL)
    {
        return LPSPI_STATUS_INVALID_PARAMETER;
    }
    if(slaveConfig == NULL)
    {
        return LPSPI_STATUS_INVALID_PARAMETER;
    }
    LPSPI_Type * base = g_lpspiBase[instance];
    lpspiState->lsb = slaveConfig->lsbFirst;
    lpspiState->bitsPerFrame = slaveConfig->bitcount;
    lpspiState->transferType = slaveConfig->transferType;
    lpspiState->isBlocking = false;
    /* Calculate the bytes/frame for lpspiState->bytesPerFrame. */
    if ((uint8_t)(lpspiState->bitsPerFrame % 8U )!= (uint8_t)0)
    {
        lpspiState->bytesPerFrame = ((lpspiState->bitsPerFrame)/8U) + 1U;
    }
    else
    {
        lpspiState->bytesPerFrame = (lpspiState->bitsPerFrame)/8U;
    }
    /* For DMA transfers bytes per frame must be equal to 1, 2 or multiple of 4. */
    if ((lpspiState->transferType == LPSPI_USING_DMA) && (!(((uint8_t)(lpspiState->bytesPerFrame % 4U) == (uint8_t)0) ||
            (lpspiState->bytesPerFrame<=2U))))
    {
        return LPSPI_STATUS_INVALID_PARAMETER;
    }
    lpspiState->isTransferInProgress = false;
    /* Initialize the semaphore */
    if (OSIF_SemaCreate(&(lpspiState->lpspiSemaphore), 0) == OSIF_STATUS_FAIL)
    {
        return LPSPI_STATUS_ERROR;
    }
    g_lpspiStatePtr[instance] = lpspiState;

    /* Configure registers */
    LPSPI_HAL_Init(base);
    LPSPI_HAL_SetFlushFifoCmd(base, true, true);
    /* Configure lpspi to slave mode */
    (void)LPSPI_HAL_SetMasterSlaveMode(base, LPSPI_SLAVE);
    /* Set Pin settings */
    (void)LPSPI_HAL_SetPinConfigMode(base, LPSPI_SDI_IN_SDO_OUT, LPSPI_DATA_OUT_RETAINED, true);
    /* Calculate the FIFO size for the LPSPI */
    LPSPI_HAL_GetFifoSizes(base, &(lpspiState->fifoSize), NULL);

    /* Set polarity */
    (void)LPSPI_HAL_SetPcsPolarityMode(base, LPSPI_PCS0,slaveConfig->pcsPolarity);

     /* Write the TCR for this transfer */
    lpspi_tx_cmd_config_t txCmdCfg = {
        .frameSize = lpspiState->bitsPerFrame,
        .width = LPSPI_SINGLE_BIT_XFER,
        .txMask = false,
        .rxMask = false,
        .byteSwap = false,
        .lsbFirst = slaveConfig->lsbFirst,
        .clkPhase = slaveConfig->clkPhase,
        .clkPolarity = slaveConfig->clkPolarity,
        .whichPcs = slaveConfig->whichPcs
    };

    /* Write to the TX CMD register */
    LPSPI_HAL_SetTxCommandReg(base, &txCmdCfg);
    LPSPI_HAL_Enable(base);
    /* Enable the interrupt source */
    INT_SYS_EnableIRQ(g_lpspiIrqId[instance]);
    return LPSPI_STATUS_SUCCESS;

}

 /*
  * Implements : LPSPI_DRV_SlaveDeinit_Activity
  */
lpspi_status_t LPSPI_DRV_SlaveDeinit(uint32_t instance)
{
    /* Instantiate local variable of type lpspi_master_state_t and point to global state */
    const lpspi_state_t * lpspiState = (lpspi_state_t *)g_lpspiStatePtr[instance];
    LPSPI_Type *base = g_lpspiBase[instance];

    /* Return error if a transfer is still in progress */
    if (lpspiState->isTransferInProgress == true)
    {
        return LPSPI_STATUS_TRANSFER_IN_PROGRESS;
    }
    /* Destroy the semaphore */
    if (OSIF_SemaDestroy(&(lpspiState->lpspiSemaphore)) == OSIF_STATUS_FAIL)
    {
        return LPSPI_STATUS_ERROR;
    }
    /* Reset the LPSPI registers to their default state, inlcuding disabling the LPSPI */
    LPSPI_HAL_Init(base);

    /* Disable the interrupt*/
    INT_SYS_DisableIRQ(g_lpspiIrqId[instance]);

    /* Clear the state pointer. */
    g_lpspiStatePtr[instance] = NULL;

    return LPSPI_STATUS_SUCCESS;
}

 /*
  * Implements : LPSPI_DRV_SlaveTransferBlocking_Activity
  */
lpspi_status_t LPSPI_DRV_SlaveTransferBlocking(uint32_t instance,
                                           const uint8_t *sendBuffer,
                                           uint8_t *receiveBuffer,
                                           uint16_t transferByteCount,
                                           uint32_t timeout)
{
    lpspi_state_t * state = (lpspi_state_t *)g_lpspiStatePtr[instance];
    lpspi_status_t error;
    osif_status_t osifError;
    state->isBlocking = true;
    error = LPSPI_DRV_SlaveTransfer(instance, sendBuffer, receiveBuffer, transferByteCount);
    if(error != LPSPI_STATUS_SUCCESS)
    {
        LPSPI_DRV_DisableTEIEInterrupts(instance);
        return error;
    }

    /* As this is a synchronous transfer, wait until the transfer is complete.*/
    osifError = OSIF_SemaWait(&(state->lpspiSemaphore), timeout);

    if (osifError == OSIF_STATUS_TIMEOUT)
    {
        /* Set isBlocking variable to false to avoid dummy semaphore post. */
        state->isBlocking = false;
        /* Complete transfer. */
        (void)LPSPI_DRV_SlaveAbortTransfer(instance);
        return(LPSPI_STATUS_TIMEOUT);
    }

    LPSPI_DRV_DisableTEIEInterrupts(instance);

    return LPSPI_STATUS_SUCCESS;
}

 /*
  * Implements : LPSPI_DRV_SlaveTransfer_Activity
  */
lpspi_status_t LPSPI_DRV_SlaveTransfer(uint32_t instance,
                                   const uint8_t *sendBuffer,
                                   uint8_t *receiveBuffer,
                                   uint16_t transferByteCount)
{
    LPSPI_Type * base = g_lpspiBase[instance];
    DMA_Type *baseDma = g_edmaBase[LPSPI_DMA_INSTANCE];
    lpspi_state_t * state = (lpspi_state_t *)g_lpspiStatePtr[instance];
    edma_transfer_size_t dmaTransferSize = EDMA_TRANSFER_SIZE_1B;
    uint8_t dmaChn;
    edma_status_t error;
    if (state->isTransferInProgress == true)
    {
        return LPSPI_STATUS_BUSY;
    }
    if ((sendBuffer == NULL) && (receiveBuffer == NULL))
    {
        return LPSPI_STATUS_INVALID_PARAMETER;
    }
    /* The number of transferred bytes should be divisible by frame size */
    if ((uint16_t)(transferByteCount % state->bytesPerFrame) != (uint16_t)0)
    {
        return LPSPI_STATUS_INVALID_PARAMETER;
    }
    /* Initialize the status of the current transfer */
    state->status = LPSPI_TRANSFER_OK;
    /* Clear all interrupts sources */
    (void)LPSPI_HAL_ClearStatusFlag(base, LPSPI_ALL_STATUS);
    /* Enable fault interrupts sources */
    LPSPI_HAL_SetIntMode(base,LPSPI_TRANSMIT_ERROR , true);
    LPSPI_HAL_SetIntMode(base,LPSPI_RECEIVE_ERROR , true);
    if (state->transferType == LPSPI_USING_INTERRUPTS)
    {
        state->rxBuff = receiveBuffer;
        state->txBuff = sendBuffer;
        state->txCount = transferByteCount;
        state->rxCount = transferByteCount;
        state->txFrameCnt = 0;
        state->rxFrameCnt = 0;
        state->isPcsContinuous = false;
        /* Configure watermarks */
        LPSPI_HAL_SetRxWatermarks(base, 0U);
        LPSPI_HAL_SetTxWatermarks(base, 2U);

        /* Clear register */
        LPSPI_HAL_SetFlushFifoCmd(base, true, true);
        state->isTransferInProgress = true;
        /* Enable interrupts for RX and TX only if it's necessary */
        if(state->txBuff != NULL)
        {
            LPSPI_HAL_SetIntMode(base,LPSPI_TX_DATA_FLAG , true);
        }
        else
        {
            state->txCount = 0;
        }
        if(state->rxBuff != NULL)
        {
            LPSPI_HAL_SetIntMode(base, LPSPI_RX_DATA_FLAG, true);
        }
        else
        {
            state->rxCount = 0;
        }
    }
    else
    {
        /* Configure watermarks */
        LPSPI_HAL_SetRxWatermarks(base, 0U);
        LPSPI_HAL_SetTxWatermarks(base, 3U);
        /* When LPSPI use DMA frames with 3 bytes size are not accepted. */
        switch(state->bytesPerFrame)
        {
            case 1: dmaTransferSize = EDMA_TRANSFER_SIZE_1B; break;
            case 2: dmaTransferSize = EDMA_TRANSFER_SIZE_2B; break;
            case 4: dmaTransferSize = EDMA_TRANSFER_SIZE_4B; break;
            default: dmaTransferSize = EDMA_TRANSFER_SIZE_1B; break;
        }
        if(receiveBuffer != NULL)
        {
            error = EDMA_DRV_RequestChannel((uint8_t)EDMA_ANY_CHANNEL, rxDmaSource[instance], &(state->rxDMAChannel), &(dmaChn));
            if (error == EDMA_STATUS_FAIL)
            {
                return LPSPI_STATUS_DMA_CHANNEL_INVALID;
            }
            (void)EDMA_DRV_ConfigSingleBlockTransfer(&(state->rxDMAChannel), EDMA_TRANSFER_PERIPH2MEM,
                    (uint32_t)(&(base->RDR)),(uint32_t)receiveBuffer, dmaTransferSize, (uint32_t)1U<<(uint8_t)(dmaTransferSize));
            EDMA_HAL_TCDSetMajorCount(baseDma, state->rxDMAChannel.channel, (uint32_t)transferByteCount/(uint32_t)((uint32_t)1U <<(uint8_t)(dmaTransferSize)));
            /* Disable DMA requests for RX channel when transfer is done. */
            EDMA_HAL_TCDSetDisableDmaRequestAfterTCDDoneCmd(baseDma, state->rxDMAChannel.channel, true);
            state->rxCount = transferByteCount;
            /* Start RX channel */
            (void)EDMA_DRV_StartChannel(&(state->rxDMAChannel));
        }
        else
        {
            state->rxCount = 0;
        }
        if(sendBuffer != NULL)
        {
            error = EDMA_DRV_RequestChannel((uint8_t)EDMA_ANY_CHANNEL, txDmaSource[instance], &(state->txDMAChannel), &(dmaChn));
            if (error == EDMA_STATUS_FAIL)
            {
                /* Release RX channel if it was requested */
                if (state->rxCount != (uint8_t) 0)
                {
                    (void)EDMA_DRV_ReleaseChannel(&(state->rxDMAChannel));
                }
                return LPSPI_STATUS_DMA_CHANNEL_INVALID;
            }
            (void)EDMA_DRV_ConfigSingleBlockTransfer(&(state->txDMAChannel), EDMA_TRANSFER_MEM2PERIPH,
                            (uint32_t)sendBuffer, (uint32_t)(&(base->TDR)), dmaTransferSize, (uint32_t)1U<<(uint8_t)(dmaTransferSize));
            EDMA_HAL_TCDSetMajorCount(baseDma, state->txDMAChannel.channel, (uint32_t)transferByteCount/(uint32_t)((uint32_t)1U <<(uint8_t)(dmaTransferSize)));
            /* Disable DMA requests for RX channel when transfer is done. */
            EDMA_HAL_TCDSetDisableDmaRequestAfterTCDDoneCmd(baseDma, state->txDMAChannel.channel, true);
            state->txCount = transferByteCount;
            /* Start TX channel */
            (void)EDMA_DRV_StartChannel(&(state->txDMAChannel));
        }
        else
        {
            state->txCount = 0;
        }
        /* Configure which channel will generate transfer complete */
        /* If current transfer uses both buffers (RX and TX) RX transfer done will generate transfer complete
         * interrupt. Otherwise transfer complete will be generate by available channel(RX or TX).
         */
        if(receiveBuffer != NULL)
        {
            (void)EDMA_DRV_InstallCallback(&(state->rxDMAChannel), (LPSPI_DRV_SlaveCompleteDMATransfer),(void*)(instance));
        }
        else
        {
            (void)EDMA_DRV_InstallCallback(&(state->txDMAChannel), (LPSPI_DRV_SlaveCompleteDMATransfer),(void*)(instance));
        }
        state->isTransferInProgress = true;
        /* Enable LPSPI DMA request */
        if (receiveBuffer != NULL)
        {
            LPSPI_HAL_SetRxDmaCmd(base, true);
        }
        if (sendBuffer != NULL)
        {
            LPSPI_HAL_SetTxDmaCmd(base, true);
        }
    }
    return LPSPI_STATUS_SUCCESS;
}

void LPSPI_DRV_SlaveIRQHandler(uint32_t instance)
{
    LPSPI_Type * base = g_lpspiBase[instance];
    lpspi_state_t * lpspiState = (lpspi_state_t *)g_lpspiStatePtr[instance];
    uint16_t txCount, rxCount;

    /* If an error is detected the transfer will be aborted */
    if ((bool)LPSPI_HAL_GetStatusFlag(base, LPSPI_TRANSMIT_ERROR))
    {
        lpspiState->status = LPSPI_TRANSMIT_FAIL;
        (void)LPSPI_DRV_SlaveAbortTransfer(instance);
        return;
    }
    if (LPSPI_HAL_GetStatusFlag(base, LPSPI_RECEIVE_ERROR))
    {
        lpspiState->status = LPSPI_RECEIVE_FAIL;
        (void)LPSPI_DRV_SlaveAbortTransfer(instance);
        return;
    }

    /* Receive data */
    if(LPSPI_HAL_GetStatusFlag(base,LPSPI_RX_DATA_FLAG))
    {
        LPSPI_DRV_ReadRXBuffer(instance);
    }
    /* Transmit data */
    txCount = lpspiState->txCount;
    if (LPSPI_HAL_GetStatusFlag(base,LPSPI_TX_DATA_FLAG) && ((txCount != (uint8_t)0)))
    {
        LPSPI_DRV_FillupTxBuffer(instance);
    }
    txCount = lpspiState->txCount;
    rxCount = lpspiState->rxCount;
    /* If all bytes are sent disable interrupt TDF */
    if (txCount == (uint8_t)0)
    {
        LPSPI_HAL_SetIntMode(base, LPSPI_TX_DATA_FLAG, false);
    }
    /* If all bytes are received disable interrupt RDF */
    if (rxCount == (uint8_t)0)
    {
        LPSPI_HAL_SetIntMode(base, LPSPI_RX_DATA_FLAG, false);
    }
    if ((rxCount == (uint8_t)0) && (txCount == (uint8_t)0))
    {
        lpspiState->isTransferInProgress = false;
        if(lpspiState->isBlocking == true)
        {
            (void)OSIF_SemaPost(&(lpspiState->lpspiSemaphore));
            lpspiState->isBlocking = false;
        }
    }
}

 /*
  * Implements : LPSPI_DRV_SlaveAbortTransfer_Activity
  */
lpspi_status_t LPSPI_DRV_SlaveAbortTransfer(uint32_t instance)
{
    LPSPI_Type * base = g_lpspiBase[instance];
    lpspi_state_t * state = (lpspi_state_t *)g_lpspiStatePtr[instance];

    /*Check if a transfer is in progress */
    if (state->isTransferInProgress == false)
    {
        return LPSPI_STATUS_NO_TRANSFER_IN_PROGRESS;
    }
    if (state->transferType == LPSPI_USING_INTERRUPTS)
    {
        /* Disable interrupts */
        LPSPI_HAL_SetIntMode(base, LPSPI_TX_DATA_FLAG, false);
        LPSPI_HAL_SetIntMode(base, LPSPI_RX_DATA_FLAG, false);
    }
    else
    {
        /* Disable LPSPI DMA request */
        LPSPI_HAL_SetRxDmaCmd(base, false);
        LPSPI_HAL_SetTxDmaCmd(base, false);
        /* Release DMA channels */
        if (state->rxCount != (uint8_t)0)
        {
            (void)EDMA_DRV_ReleaseChannel(&(state->rxDMAChannel));
        }
        if (state->txCount != (uint8_t)0)
        {
            (void)EDMA_DRV_ReleaseChannel(&(state->txDMAChannel));
        }
    }

    LPSPI_DRV_DisableTEIEInterrupts(instance);

    state->isTransferInProgress = false;
    LPSPI_HAL_SetFlushFifoCmd(base, true, true);
    if(state->isBlocking == true)
    {
        (void)OSIF_SemaPost(&(state->lpspiSemaphore));
        state->isBlocking = false;
    }
    return LPSPI_STATUS_SUCCESS;
}

 /*
  * Implements : LPSPI_DRV_SlaveGetTransferStatus_Activity
  */
lpspi_status_t LPSPI_DRV_SlaveGetTransferStatus(uint32_t instance,uint32_t * bytesRemained)
{
    const lpspi_state_t * lpspiState = (lpspi_state_t *)g_lpspiStatePtr[instance];

    /* Fill in the bytes transferred.*/
    if (bytesRemained != NULL)
    {
        *bytesRemained = lpspiState->txCount;
    }
    if (lpspiState->status == LPSPI_TRANSFER_OK)
    {
        return (lpspiState->isTransferInProgress ? LPSPI_STATUS_BUSY : LPSPI_STATUS_SUCCESS);
    }
    else
    {
        return LPSPI_STATUS_ERROR;
    }
}

/*!
 * @brief Finish up a transfer DMA.
 * The main purpose of this function is to create a function compatible with DMA callback type
 */
static void LPSPI_DRV_SlaveCompleteDMATransfer(void* parameter, edma_chn_status_t status)
{
    uint32_t instance = (uint32_t)parameter;

    (void)status;
    (void)LPSPI_DRV_SlaveAbortTransfer(instance);
}
