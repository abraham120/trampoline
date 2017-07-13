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
 * @fsl_lpspi_shared_function.c
 *
 * @page misra_violations MISRA-C:2012 violations
 *
 * @section [global]
 * Violates MISRA 2012 Required Rule 5.1, Object/function previously declared.
 * This requirement is fulfilled since the function is declared as external in and only in
 * one configuration C file.
 *
 * @section [global]
 * Violates MISRA 2012 Required Rule 5.2, Identifier clash
 * The supported compilers use more than 31 significant characters for identifiers.
 *
 * @section [global]
 * Violates MISRA 2012 Required Rule 5.4, Identifier clash
 * The supported compilers use more than 31 significant characters for identifiers.
 *
 * @section [global]
 * Violates MISRA 2012 Required Rule 5.5, Identifier clash
 * The supported compilers use more than 31 significant characters for identifiers.
 *
 * @section [global]
 * Violates MISRA 2012 Required Rule 8.4, A compatible declaration shall be
 * visible when an object or function with external linkage is defined.
 *
 * @section [global]
 * Violates MISRA 2012 Required Rule 11.6, A cast shall not be performed
 * between pointer to void and an arithmetic type.
 * The address of hardware modules is provided as integer so
 * it needs to be cast to pointer.
 *
 * @section [global]
 * Violates MISRA 2012 Advisory Rule 11.4, A conversion should not be performed
 * between a pointer to object and an integer type.
 * The address of hardware modules is provided as integer so
 * a conversion between a pointer and an integer has to be performed.
 * 
 * @section [global]
 * Violates MISRA 2012 Advisory Rule 15.4, More than one 'break' terminates loop
 * This operation is necessary because more than one source can stop the execution of the loop.
 * Also these cases can't be merged to use a single one break because for each case must be done some
 * specific operation.
  *
 * @section [global]
 * Violates MISRA 2012 Advisory Rule 18.4, Pointer arithmetic other than array indexing used.
 * This operation it's necessary to handle the buffers in LPSPI interrupt.
 *
 */

#include <assert.h>
#include "fsl_lpspi_shared_function.h"

/*******************************************************************************
 * Variables
 ******************************************************************************/

/*! @brief Table of base pointers for SPI instances. */
LPSPI_Type * g_lpspiBase[LPSPI_INSTANCE_COUNT] =  LPSPI_BASE_PTRS;

/*! @brief Table to save LPSPI IRQ enumeration numbers defined in the CMSIS header file. */
IRQn_Type g_lpspiIrqId[LPSPI_INSTANCE_COUNT] =  LPSPI_IRQS;

/* Pointer to runtime state structure.*/
lpspi_state_t * g_lpspiStatePtr[LPSPI_INSTANCE_COUNT]={NULL, NULL, NULL};

/* DMA  rx sources*/
dma_request_source_t rxDmaSource[LPSPI_INSTANCE_COUNT] = {EDMA_REQ_LPSPI0_RX, EDMA_REQ_LPSPI1_RX, EDMA_REQ_LPSPI2_RX};
/* DMA  rx sources*/
dma_request_source_t txDmaSource[LPSPI_INSTANCE_COUNT] = {EDMA_REQ_LPSPI0_TX, EDMA_REQ_LPSPI1_TX, EDMA_REQ_LPSPI2_TX};

/*******************************************************************************
 * Code
 ******************************************************************************/

/*!
 * @brief The function LPSPI_DRV_IRQHandler passes IRQ control to either the master or
 * slave driver.
 *
 * The address of the IRQ handlers are checked to make sure they are non-zero before
 * they are called. If the IRQ handler's address is zero, it means that driver was
 * not present in the link (because the IRQ handlers are marked as weak). This would
 * actually be a program error, because it means the master/slave config for the IRQ
 * was set incorrectly.
 */
void LPSPI_DRV_IRQHandler(uint32_t instance)
{
    if(instance < LPSPI_INSTANCE_COUNT)
    {
        const LPSPI_Type *base = g_lpspiBase[instance];

        if (LPSPI_HAL_IsMaster(base))
        {
            /* Master mode.*/
            LPSPI_DRV_MasterIRQHandler(instance);
        }
        else
        {
            /* Slave mode.*/
            LPSPI_DRV_SlaveIRQHandler(instance);
        }
    }
}
/*!
 * @brief Fill up the TX FIFO with data.
 * This function fills up the TX FIFO with data based on the bytes/frame.
 * This is not a public API as it is called from other driver functions.
 */
void LPSPI_DRV_FillupTxBuffer(uint32_t instance)
{
    /* Instantiate local variable of type dspi_master_state_t and point to global state. */
    lpspi_state_t * lpspiState = g_lpspiStatePtr[instance];
    LPSPI_Type *base = g_lpspiBase[instance];
    uint32_t wordToSend = 0;
    uint16_t numOfBytes;
    uint8_t j;
    uint16_t index;
    uint8_t availableSpace = lpspiState->fifoSize - (uint8_t)LPSPI_HAL_ReadTxCount(base);

    /* Fill the TX buffer. */
    while(availableSpace != 0U)
    {
        if (lpspiState->isPcsContinuous == true)
        {
            if(lpspiState->txCount == 1U)
            {
                /* Disable continuous PCS */
                LPSPI_HAL_ClearContCBit(base);
                lpspiState->txCount  = 0U;
                break;
            }
        }
        /* Get the number of bytes which can be written in a single 32 bits word. */
        if ((lpspiState->bytesPerFrame - lpspiState->txFrameCnt) <= (uint16_t)4)
        {
            numOfBytes = lpspiState->bytesPerFrame - lpspiState->txFrameCnt;
        }
        else
        {
            numOfBytes = 4U;
        }
        wordToSend = 0;
        /* Generate the word which will be written in buffer. */
        /* For the case when frame size > 4 bytes and MSB the words must be written in
         * specific order because the hardware MSB/LSB is not enough.
         */
        if ((lpspiState->lsb == false) && (lpspiState->bytesPerFrame > (uint32_t)4))
        {
            index = lpspiState->bytesPerFrame - (uint16_t)1 - lpspiState->txFrameCnt;
            for (j=0; j<numOfBytes; j++)
            {
                wordToSend = ((wordToSend)<<8) + (lpspiState->txBuff[index]);
                index--;
            }
            lpspiState->txFrameCnt = (lpspiState->txFrameCnt + numOfBytes) % lpspiState->bytesPerFrame;
            if (lpspiState->txFrameCnt == (uint16_t)0)
            {
                lpspiState->txBuff = lpspiState->txBuff + lpspiState->bytesPerFrame;
            }
        }
        else
        {
            for (j=0; j<numOfBytes; j++)
            {
                wordToSend = wordToSend + ((uint32_t)*(lpspiState->txBuff) << ((uint32_t)8U*j));
                lpspiState->txBuff++;
            }
            lpspiState->txFrameCnt = (lpspiState->txFrameCnt + numOfBytes) % lpspiState->bytesPerFrame;
        }
        LPSPI_HAL_WriteData(base, wordToSend);
        /* Update internal variable used in transmission. */
        lpspiState->txCount = lpspiState->txCount - numOfBytes;
        /* Verify if all bytes were send. */
        if (lpspiState->txCount == (uint16_t)0)
        {
            break;
        }
    availableSpace = availableSpace - 1U;
    }
}
/*!
 * @brief Read all data from RX FIFO
 * This function will read all data from RX FIFO and will transfer this infromation in
 * RX software bufeer.
 * This is not a public API as it is called from other driver functions.
 */
void LPSPI_DRV_ReadRXBuffer(uint32_t instance)
{
    lpspi_state_t * lpspiState = g_lpspiStatePtr[instance];
    const LPSPI_Type *base = g_lpspiBase[instance];
    uint32_t receivedWord;
    uint16_t numOfBytes;
    int16_t j;
    uint16_t index;
    uint8_t filledSpace = (uint8_t)LPSPI_HAL_ReadRxCount(base);
    while (filledSpace!= 0U)
    {
        receivedWord = LPSPI_HAL_ReadData(base);
        /* Get the number of bytes which can be read from this 32 bites */
        if ((lpspiState->bytesPerFrame - lpspiState->rxFrameCnt) <= (uint16_t)4)
        {
            numOfBytes = lpspiState->bytesPerFrame - lpspiState->rxFrameCnt;
        }
        else
        {
            numOfBytes = 4U;
        }
        /* Generate the word which will be write in buffer. */
        if ((lpspiState->lsb == false) && (lpspiState->bytesPerFrame > (uint16_t)4))
        {
            index = lpspiState->bytesPerFrame - lpspiState->rxFrameCnt - 1U;
            for (j= (int16_t)numOfBytes - (int16_t)1; j >= (int16_t)0; j--)
            {
                lpspiState->rxBuff[index] = (uint8_t)(receivedWord >> ((uint8_t)j*(uint8_t)8));
                index--;
            }
            lpspiState->rxFrameCnt = (lpspiState->rxFrameCnt + numOfBytes) % lpspiState->bytesPerFrame;
            if (lpspiState->rxFrameCnt == (uint16_t)0)
            {
                lpspiState->rxBuff = lpspiState->rxBuff + lpspiState->bytesPerFrame;
            }
        }
        else
        {
            for (j = (int16_t)0; j < (int16_t)numOfBytes; j++)
            {
                *(lpspiState->rxBuff) = (uint8_t)(receivedWord >> ((uint8_t)j*(uint8_t)8));
                lpspiState->rxBuff++;
            }
            lpspiState->rxFrameCnt = (lpspiState->rxFrameCnt + numOfBytes) % lpspiState->bytesPerFrame;
        }

        /* Update internal variable used in transmission. */
        lpspiState->rxCount = lpspiState->rxCount - numOfBytes;
        /* Verify if all bytes were sent. */
        if (lpspiState->rxCount == (uint16_t)0)
        {
            break;
        }
    filledSpace = filledSpace - 1U;
    }
}

/*!
 * @brief Disable the TEIE interrupts at the end of a transfer.
 * Disable the interrupts and clear the status for transmit/receive errors.
 */
void LPSPI_DRV_DisableTEIEInterrupts(uint32_t instance)
{
    LPSPI_Type *base = g_lpspiBase[instance];

    LPSPI_HAL_SetIntMode(base, LPSPI_TRANSMIT_ERROR, false);
    LPSPI_HAL_SetIntMode(base, LPSPI_RECEIVE_ERROR, false);
    (void)LPSPI_HAL_ClearStatusFlag(base, LPSPI_TRANSMIT_ERROR);
    (void)LPSPI_HAL_ClearStatusFlag(base, LPSPI_RECEIVE_ERROR);
}

/*******************************************************************************
 * EOF
 ******************************************************************************/

