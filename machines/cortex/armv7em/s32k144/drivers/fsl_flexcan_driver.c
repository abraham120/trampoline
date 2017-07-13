/*
 * Copyright (c) 2013 - 2014, Freescale Semiconductor, Inc.
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
 * @file fsl_flexcan_driver.c
 *
 * @page misra_violations MISRA-C:2012 violations
 *
 * @section [global]
 * Violates MISRA 2012 Advisory Rule 15.5, Return statement before end of function.
 * The return statement before end of function is used for simpler code structure
 * and better readability.
 *
 * @section [global]
 * Violates MISRA 2012 Required Rule 1.3, Taking address of near auto variable
 * The code is not dynamically linked. An absolute stack address is obtained when
 * taking the address of the near auto variable. A source of error in writing
 * dynamic code is that the stack segment may be different from the data segment.
 *
 * @section [global]
 * Violates MISRA 2012 Required Rule 11.6, cast from unsigned char to pointer
 * The cast is needed for a function which has a generic parameter of type void*.
 *
 * @section [global]
 * Violates MISRA 2012 Advisory Rule 11.4, conversion between a pointer and integer
 * type.
 * The cast is needed to obtain an address for a DMA call.
 *
 * @section [global]
 * Violates MISRA 2012 Required Rule 11.6, cast from pointer to unsigned long
 * The cast is needed to obtain an address for a DMA call.
 *
 * @section [global]
 * Violates MISRA 2012 Required Rule 2.2, Highest operation, function
 * 'FLEXCAN_HAL_UnlockRxMsgBuff', lacks side-effects.
 * The function is used to unlock the mailbox, which is done by reading the
 * free running timer.
 *
 * @section [global]
 * Violates MISRA 2012 Advisory Rule 8.7, External could be made static.
 * Function is defined for usage by application code.
 *
 */

#include "fsl_flexcan_driver.h"
#include "fsl_interrupt_manager.h"



/*******************************************************************************
 * Definitions
 ******************************************************************************/

/*******************************************************************************
 * Variables
 ******************************************************************************/
   
/* Pointer to runtime state structure.*/
static flexcan_state_t * g_flexcanStatePtr[CAN_INSTANCE_COUNT] = { NULL };

/*******************************************************************************
 * Private Functions
 ******************************************************************************/
static flexcan_status_t FLEXCAN_DRV_StartSendData(
                    uint8_t instance,
                    uint32_t mb_idx,
                    const flexcan_data_info_t *tx_info,
                    uint32_t msg_id,
                    const uint8_t *mb_data,
                    bool isBlocking
                    );
static flexcan_status_t FLEXCAN_DRV_StartRxMessageBufferData(
                    uint8_t instance,
                    uint32_t mb_idx,
                    flexcan_msgbuff_t *data,
                    bool isBlocking
                    );
static flexcan_status_t FLEXCAN_DRV_StartRxMessageFifoData(
                    uint8_t instance,
                    flexcan_msgbuff_t *data,
                    bool isBlocking
                    );
static void FLEXCAN_DRV_CompleteSendData(uint32_t instance);
static void FLEXCAN_DRV_CompleteRxMessageBufferData(uint32_t instance);
static void FLEXCAN_DRV_CompleteRxMessageFifoData(uint32_t instance);
#if FSL_FEATURE_CAN_HAS_DMA_ENABLE
static void FLEXCAN_DRV_CompleteRxFifoDataDMA(void *parameter, edma_chn_status_t status);
#endif

/*******************************************************************************
 * Code
 ******************************************************************************/

/*FUNCTION**********************************************************************
 *
 * Function Name : FLEXCAN_DRV_SetBitrate
 * Description   : Set FlexCAN baudrate.
 * This function will set up all the time segment values. Those time segment
 * values are passed in by the user and are based on the required baudrate.
 *
 * Implements    : FLEXCAN_DRV_SetBitrate_Activity
 *END**************************************************************************/
flexcan_status_t FLEXCAN_DRV_SetBitrate(uint8_t instance, const flexcan_time_segment_t *bitrate)
{
#ifdef DEV_ERROR_DETECT
    DEV_ASSERT(instance < CAN_INSTANCE_COUNT);
#endif

    /* Set time segments*/
    FLEXCAN_HAL_SetTimeSegments(g_flexcanBase[instance], bitrate);

    return FLEXCAN_STATUS_SUCCESS;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : FLEXCAN_DRV_SetBitrateCbt
 * Description   : Set FlexCAN bitrate.
 * This function will set up all the time segment values. Those time segment
 * values are passed in by the user and are based on the required baudrate.
 *
 * Implements    : FLEXCAN_DRV_SetBitrateCbt_Activity
 *END**************************************************************************/
flexcan_status_t FLEXCAN_DRV_SetBitrateCbt(uint8_t instance, const flexcan_time_segment_t *bitrate)
{
#ifdef DEV_ERROR_DETECT
    DEV_ASSERT(instance < CAN_INSTANCE_COUNT);
#endif

    /* Set time segments*/
    FLEXCAN_HAL_SetTimeSegmentsCbt(g_flexcanBase[instance], bitrate);

    return FLEXCAN_STATUS_SUCCESS;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : FLEXCAN_DRV_GetBitrate
 * Description   : Get FlexCAN baudrate.
 * This function will be return the current bit rate settings
 *
 * Implements    : FLEXCAN_DRV_GetBitrate_Activity
 *END**************************************************************************/
flexcan_status_t  FLEXCAN_DRV_GetBitrate(uint8_t instance, flexcan_time_segment_t *bitrate)
{

#ifdef DEV_ERROR_DETECT
    DEV_ASSERT(instance < CAN_INSTANCE_COUNT);
#endif
    /* Get the time segments*/
    FLEXCAN_HAL_GetTimeSegments(g_flexcanBase[instance], bitrate);

    return FLEXCAN_STATUS_SUCCESS;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : FLEXCAN_DRV_SetMasktype
 * Description   : Set RX masking type.
 * This function will set RX masking type as RX global mask or RX individual
 * mask.
 *
 * Implements    : FLEXCAN_DRV_SetRxMaskType_Activity
 *END**************************************************************************/
void  FLEXCAN_DRV_SetRxMaskType(uint8_t instance, flexcan_rx_mask_type_t type)
{

#ifdef DEV_ERROR_DETECT
    DEV_ASSERT(instance < CAN_INSTANCE_COUNT);
#endif

    FLEXCAN_HAL_SetRxMaskType(g_flexcanBase[instance], type);
}

/*FUNCTION**********************************************************************
 *
 * Function Name : FLEXCAN_DRV_SetRxFifoGlobalMask
 * Description   : Set Rx FIFO global mask as the 11-bit standard mask or the
 * 29-bit extended mask.
 *
 * Implements    : FLEXCAN_DRV_SetRxFifoGlobalMask_Activity
 *END**************************************************************************/
flexcan_status_t FLEXCAN_DRV_SetRxFifoGlobalMask(
    uint8_t instance,
    flexcan_msgbuff_id_type_t id_type,
    uint32_t mask)
{
#ifdef DEV_ERROR_DETECT
    DEV_ASSERT(instance < CAN_INSTANCE_COUNT);
#endif

    CAN_Type * base = g_flexcanBase[instance];
    flexcan_status_t stat = FLEXCAN_STATUS_SUCCESS;

    if (id_type == FLEXCAN_MSG_ID_STD)
    {
        /* Set standard global mask for RX FIOF*/
        FLEXCAN_HAL_SetRxFifoGlobalStdMask(base, mask);
    }
    else if (id_type == FLEXCAN_MSG_ID_EXT)
    {
        /* Set extended global mask for RX FIFO*/
        FLEXCAN_HAL_SetRxFifoGlobalExtMask(base, mask);
    }
    else
    {
        stat = FLEXCAN_STATUS_INVALID_ARGUMENT;
    }

    return stat;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : FLEXCAN_DRV_SetRxMbGlobalMask
 * Description   : Set Rx Message Buffer global mask as the 11-bit standard mask
 * or the 29-bit extended mask.
 *
 * Implements    : FLEXCAN_DRV_SetRxMbGlobalMask_Activity
 *END**************************************************************************/
flexcan_status_t FLEXCAN_DRV_SetRxMbGlobalMask(
    uint8_t instance,
    flexcan_msgbuff_id_type_t id_type,
    uint32_t mask)
{
#ifdef DEV_ERROR_DETECT
    DEV_ASSERT(instance < CAN_INSTANCE_COUNT);
#endif
    CAN_Type * base = g_flexcanBase[instance];
    flexcan_status_t stat = FLEXCAN_STATUS_SUCCESS;

    if (id_type == FLEXCAN_MSG_ID_STD)
    {
        /* Set standard global mask for RX MB*/
        FLEXCAN_HAL_SetRxMsgBuffGlobalStdMask(base, mask);
    }
    else if (id_type == FLEXCAN_MSG_ID_EXT)
    {
        /* Set extended global mask for RX MB*/
        FLEXCAN_HAL_SetRxMsgBuffGlobalExtMask(base, mask);
    }
    else
    {
        stat = FLEXCAN_STATUS_INVALID_ARGUMENT;
    }

    return stat;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : FLEXCAN_DRV_SetRxIndividualMask
 * Description   : Set Rx individual mask as the 11-bit standard mask or the
 * 29-bit extended mask.
 *
 * Implements    : FLEXCAN_DRV_SetRxIndividualMask_Activity
 *END**************************************************************************/
flexcan_status_t FLEXCAN_DRV_SetRxIndividualMask(
    uint8_t instance,
    flexcan_msgbuff_id_type_t id_type,
    uint32_t mb_idx,
    uint32_t mask)
{
#ifdef DEV_ERROR_DETECT
    DEV_ASSERT(instance < CAN_INSTANCE_COUNT);
#endif
    CAN_Type * base = g_flexcanBase[instance];
    flexcan_status_t stat = FLEXCAN_STATUS_SUCCESS;

    if (id_type == FLEXCAN_MSG_ID_STD)
    {
        /* Set standard individual mask*/
        stat = FLEXCAN_HAL_SetRxIndividualStdMask(base, mb_idx, mask);
    }
    else if (id_type == FLEXCAN_MSG_ID_EXT)
    {
        /* Set extended individual mask*/
        stat = FLEXCAN_HAL_SetRxIndividualExtMask(base, mb_idx, mask);
    }
    else
    {
        stat = FLEXCAN_STATUS_INVALID_ARGUMENT;
    }
    
    return stat;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : FLEXCAN_DRV_Init
 * Description   : Initialize FlexCAN driver.
 * This function will select a source clock, reset FlexCAN module, set maximum
 * number of message buffers, initialize all message buffers as inactive, enable
 * RX FIFO if needed, mask all mask bits, disable all MB interrupts, enable
 * FlexCAN normal mode, and enable all the error interrupts if needed.
 *
 * Implements    : FLEXCAN_DRV_Init_Activity
 *END**************************************************************************/
flexcan_status_t FLEXCAN_DRV_Init(
   uint32_t instance,
   flexcan_state_t *state,
   const flexcan_user_config_t *data)
{
#ifdef DEV_ERROR_DETECT
    DEV_ASSERT(instance < CAN_INSTANCE_COUNT);
    DEV_ASSERT(state);
#endif

    flexcan_status_t result;
    CAN_Type * base = g_flexcanBase[instance];
    flexcan_time_segment_t bitrate;
    osif_status_t osifStat;
    uint32_t i;

    result = FLEXCAN_HAL_Disable(base);
    if (result != FLEXCAN_STATUS_SUCCESS)
    {
        return result;
    }

#if FSL_FEATURE_CAN_HAS_PE_CLKSRC_SELECT
    /* Select a source clock for the FlexCAN engine */
    result = FLEXCAN_HAL_SelectClock(base, data->pe_clock);
    if (result != FLEXCAN_STATUS_SUCCESS)
    {
        return result;
    }
#endif

    /* Enable the CAN clock */
    result = FLEXCAN_HAL_Enable(base);
    if (result != FLEXCAN_STATUS_SUCCESS)
    {
        return result;
    }

    /* Initialize FLEXCAN device */
    result = FLEXCAN_HAL_Init(base);
    if (result != FLEXCAN_STATUS_SUCCESS)
    {
        return result;
    }

    /* Enable/Disable FD and check FD was set as expected. Setting FD as enabled
     * might fail if the current CAN instance does not support FD. */
    FLEXCAN_HAL_SetFDEnabled(base, data->fd_enable);    
    if (FLEXCAN_HAL_IsFDEnabled(base) != data->fd_enable)
    {
        return FLEXCAN_STATUS_FAIL;
    }

    /* If the FD feature is enabled, enable the Stuff Bit Count, in order to be
     * ISO-compliant. */
    FLEXCAN_HAL_SetStuffBitCount(base, data->fd_enable);
    
    /* Disable the self reception feature if FlexCAN is not in loopback mode. */
    if (data->flexcanMode != FLEXCAN_LOOPBACK_MODE)
    {
        FLEXCAN_HAL_SetSelfReception(base, false);
    }
    
    /* Enable RxFIFO feature, if requested. This might fail if the FD mode is
     * enabled. */
    if (data->is_rx_fifo_needed)
    {
        result = FLEXCAN_HAL_EnableRxFifo(base, (uint32_t)data->num_id_filters);
        if (result != FLEXCAN_STATUS_SUCCESS)
        {
            return result;
        }
    }
    
#if FSL_FEATURE_CAN_HAS_DMA_ENABLE
    /* Enable DMA support for RxFIFO transfer, if requested. */
    if (data->transfer_type == FLEXCAN_RXFIFO_USING_DMA)
    {
        result = FLEXCAN_HAL_SetRxFifoDMA(base, true);
        if (result != FLEXCAN_STATUS_SUCCESS)
        {
            return result;
        }
    }
#endif

    /* Select mode */
    result = FLEXCAN_HAL_SetOperationMode(base, data->flexcanMode);
    if (result != FLEXCAN_STATUS_SUCCESS)
    {
        return result;
    }

    /* Set payload size. */
    result = FLEXCAN_HAL_SetPayloadSize(base, data->payload);
    if (result != FLEXCAN_STATUS_SUCCESS)
    {
        return result;
    }

    result = FLEXCAN_HAL_SetMaxMsgBuffNum(base, data->max_num_mb);
    if (result != FLEXCAN_STATUS_SUCCESS)
    {
        return result;
    }

    /* Set bit rate. */
    bitrate = data->bitrate;
    FLEXCAN_HAL_SetTimeSegments(base, &bitrate);
    if (FLEXCAN_HAL_IsFDEnabled(base))
    {
        bitrate = data->bitrate_cbt;
        FLEXCAN_HAL_SetTimeSegmentsCbt(base, &bitrate);
    }

    /* Enable FlexCAN interrupts.*/
    INT_SYS_EnableIRQ(g_flexcanWakeUpIrqId[instance]);
    INT_SYS_EnableIRQ(g_flexcanErrorIrqId[instance]);
    INT_SYS_EnableIRQ(g_flexcanBusOffIrqId[instance]);
    for (i = 0; i < CAN_ORed_Message_buffer_IRQS_CH_COUNT; i++)
    {
        INT_SYS_EnableIRQ(g_flexcanOredMessageBufferIrqId[instance][i]);
    }

    state->isTxBusy = false;
    state->isRxBusy = false;
    state->fifo_message = NULL;
    state->rx_mb_idx = 0;
    state->tx_mb_idx = 0;
    osifStat = OSIF_SemaCreate(&state->rxSema, 0U);
    if (osifStat != OSIF_STATUS_SUCCESS)
    {
        return FLEXCAN_STATUS_FAIL;
    }
    osifStat = OSIF_SemaCreate(&state->txSema, 0U);
    if (osifStat != OSIF_STATUS_SUCCESS)
    {
        return FLEXCAN_STATUS_FAIL;
    }
    state->transferType = data->transfer_type;
    /* Save runtime structure pointers so irq handler can point to the correct state structure */
    g_flexcanStatePtr[instance] = state;

    return (FLEXCAN_STATUS_SUCCESS);
}

/*FUNCTION**********************************************************************
 *
 * Function Name : FLEXCAN_DRV_ConfigTxMb
 * Description   : Configure a Tx message buffer.
 * This function will first check if RX FIFO is enabled. If RX FIFO is enabled,
 * the function will make sure if the MB requested is not occupied by RX FIFO
 * and ID filter table. Then this function will set up the message buffer fields,
 * configure the message buffer code for Tx buffer as INACTIVE, and enable the
 * Message Buffer interrupt.
 *
 * Implements    : FLEXCAN_DRV_ConfigTxMb_Activity
 *END**************************************************************************/
flexcan_status_t FLEXCAN_DRV_ConfigTxMb(
    uint8_t instance,
    uint32_t mb_idx,
    const flexcan_data_info_t *tx_info,
    uint32_t msg_id)
{
#ifdef DEV_ERROR_DETECT
    DEV_ASSERT(instance < CAN_INSTANCE_COUNT);
#endif
    flexcan_msgbuff_code_status_t cs;
    CAN_Type * base = g_flexcanBase[instance];
    flexcan_state_t * state = g_flexcanStatePtr[instance];

    state->tx_mb_idx = mb_idx;
    /* Initialize transmit mb*/
    cs.dataLen = tx_info->data_length;
    cs.msgIdType = tx_info->msg_id_type;
    cs.code = (uint32_t)FLEXCAN_TX_INACTIVE;
    return FLEXCAN_HAL_SetTxMsgBuff(base, mb_idx, &cs, msg_id, NULL);
}

/*FUNCTION**********************************************************************
 *
 * Function Name : FLEXCAN_DRV_Send_Blocking
 * Description   : Set up FlexCAN Message buffer for transmitting data.
 * This function will set the MB CODE field as DATA for Tx buffer. Then this
 * function will copy user's buffer into the message buffer data area, and wait
 * for the Message Buffer interrupt.
 *
 * Implements    : FLEXCAN_DRV_SendBlocking_Activity
 *END**************************************************************************/
flexcan_status_t FLEXCAN_DRV_SendBlocking(
    uint8_t instance,
    uint32_t mb_idx,
    const flexcan_data_info_t *tx_info,
    uint32_t msg_id,
    const uint8_t *mb_data,
    uint32_t timeout_ms)
{
#ifdef DEV_ERROR_DETECT
    DEV_ASSERT(instance < CAN_INSTANCE_COUNT);
#endif
    flexcan_status_t result;
    flexcan_state_t * state = g_flexcanStatePtr[instance];
    CAN_Type * base  = g_flexcanBase[instance];

    result = FLEXCAN_DRV_StartSendData(instance, mb_idx, tx_info, msg_id, mb_data, true);

    if(result == FLEXCAN_STATUS_SUCCESS)
    {
        osif_status_t status;
        
        /* Enable message buffer interrupt*/
        (void)FLEXCAN_HAL_SetMsgBuffIntCmd(base, mb_idx, true);
        /* Enable error interrupts */
        FLEXCAN_HAL_SetErrIntCmd(base,FLEXCAN_INT_ERR,true);
        
        status = OSIF_SemaWait(&state->txSema, timeout_ms);
        
        if (status == OSIF_STATUS_SUCCESS)
        {
            state->isTxBusy = false;
            FLEXCAN_HAL_ClearMsgBuffIntStatusFlag(base, (1UL << mb_idx));
            result =  FLEXCAN_STATUS_SUCCESS;
        }
        else
        {
            state->isTxBusy = false;
            result = FLEXCAN_STATUS_TIME_OUT;
        }
    }

    return result;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : FLEXCAN_DRV_Send
 * Description   : Set up FlexCAN Message buffer for transmitting data.
 * This function will set the MB CODE field as DATA for Tx buffer. Then this
 * function will copy user's buffer into the message buffer data area.
 *
 * Implements    : FLEXCAN_DRV_Send_Activity
 *END**************************************************************************/
flexcan_status_t FLEXCAN_DRV_Send(
    uint8_t instance,
    uint32_t mb_idx,
    const flexcan_data_info_t *tx_info,
    uint32_t msg_id,
    const uint8_t *mb_data)
{
#ifdef DEV_ERROR_DETECT
    DEV_ASSERT(instance < CAN_INSTANCE_COUNT);
#endif
    flexcan_status_t result;
    CAN_Type * base = g_flexcanBase[instance];

    result = FLEXCAN_DRV_StartSendData(instance, mb_idx, tx_info, msg_id, mb_data, false);
    if(result == FLEXCAN_STATUS_SUCCESS)
    {
        /* Enable message buffer interrupt*/
        result = FLEXCAN_HAL_SetMsgBuffIntCmd(base, mb_idx, true);
        /* Enable error interrupts */
        FLEXCAN_HAL_SetErrIntCmd(base,FLEXCAN_INT_ERR,true);
    }

    return result;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : FLEXCAN_DRV_ConfigMb
 * Description   : Configure a Rx message buffer.
 * This function will first check if RX FIFO is enabled. If RX FIFO is enabled,
 * the function will make sure if the MB requested is not occupied by RX FIFO
 * and ID filter table. Then this function will set up the message buffer fields,
 * configure the message buffer code for Rx message buffer as NOT_USED, enable
 * the Message Buffer interrupt, configure the message buffer code for Rx
 * message buffer as INACTIVE, copy user's buffer into the message buffer data
 * area, and configure the message buffer code for Rx message buffer as EMPTY.
 *
 * Implements    : FLEXCAN_DRV_ConfigRxMb_Activity
 *END**************************************************************************/
flexcan_status_t FLEXCAN_DRV_ConfigRxMb(
    uint8_t instance,
    uint32_t mb_idx,
    const flexcan_data_info_t *rx_info,
    uint32_t msg_id)
{
#ifdef DEV_ERROR_DETECT
    DEV_ASSERT(instance < CAN_INSTANCE_COUNT);
#endif
    flexcan_status_t result;
    flexcan_msgbuff_code_status_t cs;
    CAN_Type * base = g_flexcanBase[instance];
    flexcan_state_t * state = g_flexcanStatePtr[instance];

    state->rx_mb_idx = mb_idx;
    cs.dataLen = rx_info->data_length;
    cs.msgIdType = rx_info->msg_id_type;
    cs.fd_enable = rx_info->fd_enable;
    /* Initialize rx mb*/
    cs.code = (uint32_t)FLEXCAN_RX_NOT_USED;
    result = FLEXCAN_HAL_SetRxMsgBuff(base, mb_idx, &cs, msg_id);
    if (result != FLEXCAN_STATUS_SUCCESS)
    {
         return result;
    }

    /* Initialize receive MB*/
    cs.code = (uint32_t)FLEXCAN_RX_INACTIVE;
    result = FLEXCAN_HAL_SetRxMsgBuff(base, mb_idx, &cs, msg_id);
    if (result != FLEXCAN_STATUS_SUCCESS)
    {
         return result;
    }

    /* Set up FlexCAN message buffer fields for receiving data*/
    cs.code = (uint32_t)FLEXCAN_RX_EMPTY;
    return FLEXCAN_HAL_SetRxMsgBuff(base, mb_idx, &cs, msg_id);
}

/*FUNCTION**********************************************************************
 *
 * Function Name : FLEXCAN_DRV_ConfigRxFifo
 * Description   : Confgure RX FIFO ID filter table elements.
 * This function will confgure RX FIFO ID filter table elements, and enable RX
 * FIFO interrupts.
 *
 * Implements    : FLEXCAN_DRV_ConfigRxFifo_Activity
 *END**************************************************************************/
flexcan_status_t FLEXCAN_DRV_ConfigRxFifo(
    uint8_t instance,
    flexcan_rx_fifo_id_element_format_t id_format,
    const flexcan_id_table_t *id_filter_table)
{
#ifdef DEV_ERROR_DETECT
    DEV_ASSERT(instance < CAN_INSTANCE_COUNT);
#endif
    flexcan_status_t result;
    CAN_Type * base = g_flexcanBase[instance];

    /* Initialize rx fifo*/
    result = FLEXCAN_HAL_SetRxFifoFilter(base, id_format, id_filter_table);

    return result;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : FLEXCAN_DRV_RxMessageBufferBlocking
 * Description   : Start receive data after a Rx MB interrupt occurs.
 * This function will lock Rx MB after a Rx MB interrupt occurs.
 *
 * Implements    : FLEXCAN_DRV_RxMessageBufferBlocking_Activity
 *END**************************************************************************/
flexcan_status_t FLEXCAN_DRV_RxMessageBufferBlocking(
    uint8_t instance,
    uint32_t mb_idx,
    flexcan_msgbuff_t *data,
    uint32_t timeout_ms)
{
#ifdef DEV_ERROR_DETECT
    DEV_ASSERT(instance < CAN_INSTANCE_COUNT);
    DEV_ASSERT(data);
#endif
    flexcan_status_t result;
    flexcan_state_t * state = g_flexcanStatePtr[instance];
    CAN_Type * base = g_flexcanBase[instance];

    result = FLEXCAN_DRV_StartRxMessageBufferData(instance, mb_idx, data, true);

    if(result == FLEXCAN_STATUS_SUCCESS)
    {
        osif_status_t status;
        
        status = OSIF_SemaWait(&state->rxSema, timeout_ms);
        
        if (status == OSIF_STATUS_SUCCESS)
        {
            state->isRxBusy = false;
            FLEXCAN_HAL_ClearMsgBuffIntStatusFlag(base, (1UL << mb_idx));
            result = FLEXCAN_HAL_GetMsgBuff(base, mb_idx, data);
        }
        else
        {
            state->isRxBusy = false;
            return FLEXCAN_STATUS_TIME_OUT;
        }
    }

    return result;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : FLEXCAN_DRV_RxMessageBuffer
 * Description   : Start receive data after a Rx MB interrupt occurs.
 *
 * Implements    : FLEXCAN_DRV_RxMessageBuffer_Activity
 *END**************************************************************************/
flexcan_status_t FLEXCAN_DRV_RxMessageBuffer(
    uint8_t instance,
    uint32_t mb_idx,
    flexcan_msgbuff_t *data)
{
#ifdef DEV_ERROR_DETECT
    DEV_ASSERT(instance < CAN_INSTANCE_COUNT);
    DEV_ASSERT(data);
#endif

    flexcan_status_t result;

    result = FLEXCAN_DRV_StartRxMessageBufferData(instance, mb_idx, data, false);

    return result;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : FLEXCAN_DRV_RxFifoBlocking
 * Description   : Start receive data after a Rx FIFO interrupt occurs.
 * This function will lock Rx FIFO after a Rx FIFO interrupt occurs
 *
 * Implements    : FLEXCAN_DRV_RxFifoBlocking_Activity
 *END**************************************************************************/
flexcan_status_t FLEXCAN_DRV_RxFifoBlocking(
    uint8_t instance,
    flexcan_msgbuff_t *data,
    uint32_t timeout_ms)
{
#ifdef DEV_ERROR_DETECT
    DEV_ASSERT(instance < CAN_INSTANCE_COUNT);
    DEV_ASSERT(data);
#endif
    flexcan_status_t result;
    flexcan_state_t * state = g_flexcanStatePtr[instance];
#if FSL_FEATURE_CAN_HAS_DMA_ENABLE
    edma_status_t edmaStat;
#endif

    result = FLEXCAN_DRV_StartRxMessageFifoData(instance, data, true);

    if(result == FLEXCAN_STATUS_SUCCESS)
    {
        osif_status_t status;
        
        status = OSIF_SemaWait(&state->rxSema, timeout_ms);
        
        state->isRxBusy = false;
        if (status == OSIF_STATUS_TIMEOUT)
        {
#if FSL_FEATURE_CAN_HAS_DMA_ENABLE
            if (state->transferType == FLEXCAN_RXFIFO_USING_DMA)
            {
                edmaStat = EDMA_DRV_ReleaseChannel(&(state->rxFifoDMAChannel));
                if (edmaStat != EDMA_STATUS_SUCCESS)
                {
                    result = FLEXCAN_STATUS_FAIL;
                }
                else
                {
                    result = FLEXCAN_STATUS_TIME_OUT;
                }
            }
#endif
        }
    }

    return result;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : FLEXCAN_DRV_RxFifoBlocking
 * Description   : Start receive data after a Rx FIFO interrupt occurs.
 *
 * Implements    : FLEXCAN_DRV_RxFifo_Activity
 *END**************************************************************************/
flexcan_status_t FLEXCAN_DRV_RxFifo(
    uint8_t instance,
    flexcan_msgbuff_t *data)
{
#ifdef DEV_ERROR_DETECT
    DEV_ASSERT(instance < CAN_INSTANCE_COUNT);
    DEV_ASSERT(data);
#endif
    flexcan_status_t result;

    result = FLEXCAN_DRV_StartRxMessageFifoData(instance, data, false);

    return result;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : FLEXCAN_DRV_Deinit
 * Description   : Shutdown a FlexCAN module.
 * This function will disable all FlexCAN interrupts, and disable the FlexCAN.
 *
 * Implements    : FLEXCAN_DRV_Deinit_Activity
 *END**************************************************************************/
flexcan_status_t FLEXCAN_DRV_Deinit(uint8_t instance)
{
#ifdef DEV_ERROR_DETECT
    DEV_ASSERT(instance < CAN_INSTANCE_COUNT);
#endif

    const flexcan_state_t * state = g_flexcanStatePtr[instance];
    flexcan_status_t result;
    osif_status_t osifStat;
    uint32_t i;

    /* Disable FlexCAN interrupts.*/
    INT_SYS_DisableIRQ(g_flexcanWakeUpIrqId[instance]);
    INT_SYS_DisableIRQ(g_flexcanErrorIrqId[instance]);
    INT_SYS_DisableIRQ(g_flexcanBusOffIrqId[instance]);
    for (i = 0; i < CAN_ORed_Message_buffer_IRQS_CH_COUNT; i++)
    {
        INT_SYS_DisableIRQ(g_flexcanOredMessageBufferIrqId[instance][i]);
    }

    /* Disable FlexCAN.*/
    result = FLEXCAN_HAL_Disable(g_flexcanBase[instance]);

    osifStat = OSIF_SemaDestroy(&state->rxSema);
    if (osifStat == OSIF_STATUS_SUCCESS)
    {
        osifStat = OSIF_SemaDestroy(&state->txSema);
        if (osifStat != OSIF_STATUS_SUCCESS)
        {
            result = FLEXCAN_STATUS_SUCCESS;
        }
    }
    else
    {
        result = FLEXCAN_STATUS_SUCCESS;
    }

    return result;
}
/*FUNCTION**********************************************************************
 *
 * Function Name : FLEXCAN_DRV_IRQHandler
 * Description   : Interrupt handler for FLEXCAN.
 * This handler read data from MB or FIFO, and then clear the interrupt flags.
 * This is not a public API as it is called whenever an interrupt occurs.
 *
 * Implements    : FLEXCAN_DRV_IRQHandler_Activity
 *END**************************************************************************/
void FLEXCAN_DRV_IRQHandler(uint8_t instance)
{
    uint32_t flag_reg;
    uint32_t temp;
    CAN_Type * base = g_flexcanBase[instance];
    flexcan_state_t * state = g_flexcanStatePtr[instance];
    flexcan_status_t result = FLEXCAN_STATUS_SUCCESS;

    /* Get the interrupts that are enabled and ready */
    flag_reg = FLEXCAN_HAL_GetAllMsgBuffIntStatusFlag(base);

    /* Check Tx/Rx interrupt flag and clear the interrupt */
    if(flag_reg != 0U)
    {
        bool rxfifoEnabled = FLEXCAN_HAL_IsRxFifoEnabled(base);
        if (((flag_reg & 0x20U) != 0U) && rxfifoEnabled)
        {
            if (state->fifo_message != NULL)
            {
                /* Get RX FIFO field values */
                result = FLEXCAN_HAL_ReadRxFifo(base, state->fifo_message);
                
                if (result == FLEXCAN_STATUS_SUCCESS)
                {
                    /* Complete receive data */
                    FLEXCAN_DRV_CompleteRxMessageFifoData(instance);
                    FLEXCAN_HAL_ClearMsgBuffIntStatusFlag(base, flag_reg);

                    /* Invoke callback */
                    if (state->callback != NULL)
                    {
                        state->callback(instance, FLEXCAN_EVENT_RXFIFO_COMPLETE, state);
                    }
                }
            }
        }
        else
        {
            bool isRxBusy = state->isRxBusy;

            /* Check mailbox completed reception */
            temp = (1UL << state->rx_mb_idx) & flag_reg;
            if ((temp != 0U) && isRxBusy)
            {
                /* Unlock RX message buffer and RX FIFO*/
                result = FLEXCAN_HAL_LockRxMsgBuff(base, state->rx_mb_idx);
                if (result == FLEXCAN_STATUS_SUCCESS)
                {
                    /* Get RX MB field values*/
                    result = FLEXCAN_HAL_GetMsgBuff(base, state->rx_mb_idx, state->mb_message);
                }
                if (result == FLEXCAN_STATUS_SUCCESS)
                {
                    /* Unlock RX message buffer and RX FIFO*/
                    FLEXCAN_HAL_UnlockRxMsgBuff(base);

                    /* Complete receive data */
                    FLEXCAN_DRV_CompleteRxMessageBufferData(instance);
                    FLEXCAN_HAL_ClearMsgBuffIntStatusFlag(base, temp);

                    /* Invoke callback */
                    if (state->callback != NULL)
                    {
                        state->callback(instance, FLEXCAN_EVENT_RX_COMPLETE, state);
                    }
                }
            }
        }

        bool isTxBusy = state->isTxBusy;

        /* Check mailbox completed transmission */
        temp = (1UL << state->tx_mb_idx) & flag_reg;
        if ((temp != 0U) && isTxBusy)
        {
            /* Complete transmit data */
            FLEXCAN_DRV_CompleteSendData(instance);
            FLEXCAN_HAL_ClearMsgBuffIntStatusFlag(base, temp);

            /* Invoke callback */
            if (state->callback != NULL)
            {
                state->callback(instance, FLEXCAN_EVENT_TX_COMPLETE, state);
            }
        }
    }

    /* Clear all other interrupts in ERRSTAT register (Error, Busoff, Wakeup) */
    FLEXCAN_HAL_ClearErrIntStatusFlag(base);

    return;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : FLEXCAN_DRV_GetTransmitStatus
 * Description   : This function returns whether the previous FLEXCAN receive is
 *                 completed.
 * When performing a non-blocking receive, the user can call this function to
 * ascertain the state of the current receive progress: in progress (or busy)
 * or complete (success).
 *
 * Implements    : FLEXCAN_DRV_GetTransmitStatus_Activity
 *END**************************************************************************/
flexcan_status_t FLEXCAN_DRV_GetTransmitStatus(uint32_t instance)
{
#ifdef DEV_ERROR_DETECT
    DEV_ASSERT(instance < CAN_INSTANCE_COUNT);
#endif
    const flexcan_state_t * state = g_flexcanStatePtr[instance];

    return (state->isTxBusy ? FLEXCAN_STATUS_TX_BUSY : FLEXCAN_STATUS_SUCCESS);
}

/*FUNCTION**********************************************************************
 *
 * Function Name : FLEXCAN_DRV_GetReceiveStatus
 * Description   : This function returns whether the previous FLEXCAN receive is
 *                 completed.
 * When performing a non-blocking receive, the user can call this function to
 * ascertain the state of the current receive progress: in progress (or busy)
 * or complete (success).
 *
 * Implements    : FLEXCAN_DRV_GetReceiveStatus_Activity
 *END**************************************************************************/
flexcan_status_t FLEXCAN_DRV_GetReceiveStatus(uint32_t instance)
{
#ifdef DEV_ERROR_DETECT
    DEV_ASSERT(instance < CAN_INSTANCE_COUNT);
#endif
    const flexcan_state_t * state = g_flexcanStatePtr[instance];

    return (state->isRxBusy ? FLEXCAN_STATUS_RX_BUSY : FLEXCAN_STATUS_SUCCESS);
}

/*FUNCTION**********************************************************************
 *
 * Function Name : FLEXCAN_DRV_AbortSendingData
 * Description   : This function ends a non-blocking FLEXCAN transmission early.
 * During a non-blocking FLEXCAN transmission, the user has the option to terminate
 * the transmission early if the transmission is still in progress.
 *
 * Implements    : FLEXCAN_DRV_AbortSendingData_Activity
 *END**************************************************************************/
flexcan_status_t FLEXCAN_DRV_AbortSendingData(uint32_t instance)
{
#ifdef DEV_ERROR_DETECT
    DEV_ASSERT(instance < CAN_INSTANCE_COUNT);
#endif
    const flexcan_state_t * state = g_flexcanStatePtr[instance];

    /* Check if a transfer is running. */
    if (!state->isTxBusy)
    {
        return FLEXCAN_STATUS_NO_TRANSMIT_IN_PROGRESS;
    }

    /* Stop the running transfer. */
    FLEXCAN_DRV_CompleteSendData(instance);

    return FLEXCAN_STATUS_SUCCESS;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : FLEXCAN_DRV_AbortReceivingData
 * Description   : This function shuts down the FLEXCAN by disabling interrupts and
 *                 the transmitter/receiver.
 * This function disables the FLEXCAN interrupts, disables the transmitter and
 * receiver.
 *
 * Implements    : FLEXCAN_DRV_AbortReceivingData_Activity
 *END**************************************************************************/
flexcan_status_t FLEXCAN_DRV_AbortReceivingData(uint32_t instance)
{
#ifdef DEV_ERROR_DETECT
    DEV_ASSERT(instance < CAN_INSTANCE_COUNT);
#endif
    const flexcan_state_t * state = g_flexcanStatePtr[instance];

    /* Check if a transfer is running. */
    if (!state->isRxBusy)
    {
        return FLEXCAN_STATUS_NO_RECEIVE_IN_PROGRESS;
    }

    /* Stop the running transfer. */
    FLEXCAN_DRV_CompleteRxMessageBufferData(instance);

    return FLEXCAN_STATUS_SUCCESS;
}


/*FUNCTION**********************************************************************
 *
 * Function Name : FLEXCAN_DRV_StartSendData
 * Description   : Initiate (start) a transmit by beginning the process of
 * sending data.
 * This is not a public API as it is called from other driver functions.
 *
 *END**************************************************************************/
static flexcan_status_t FLEXCAN_DRV_StartSendData(
                    uint8_t instance,
                    uint32_t mb_idx,
                    const flexcan_data_info_t *tx_info,
                    uint32_t msg_id,
                    const uint8_t *mb_data,
                    bool isBlocking
                    )
{
    flexcan_status_t result;
    flexcan_msgbuff_code_status_t cs;
    flexcan_state_t * state = g_flexcanStatePtr[instance];
    CAN_Type * base = g_flexcanBase[instance];

    if (state->isTxBusy)
    {
        return FLEXCAN_STATUS_TX_BUSY;
    }
    state->isTxBusy = true;

    state->isTxBlocking = isBlocking;

    state->tx_mb_idx = mb_idx;
    cs.dataLen = tx_info->data_length;
    cs.msgIdType = tx_info->msg_id_type;
    
    cs.fd_enable = tx_info->fd_enable;
    cs.fd_padding = tx_info->fd_padding;
    cs.enable_brs = tx_info->enable_brs;
    cs.code = (uint32_t)FLEXCAN_TX_DATA;
    result = FLEXCAN_HAL_SetTxMsgBuff(base, mb_idx, &cs, msg_id, mb_data);
    return result;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : FLEXCAN_DRV_StartRxMessageBufferData
 * Description   : Initiate (start) a receive by beginning the process of
 * receiving data and enabling the interrupt.
 * This is not a public API as it is called from other driver functions.
 *
 *END**************************************************************************/
static flexcan_status_t FLEXCAN_DRV_StartRxMessageBufferData(
                    uint8_t instance,
                    uint32_t mb_idx,
                    flexcan_msgbuff_t *data,
                    bool isBlocking
                    )
{
    flexcan_status_t result = FLEXCAN_STATUS_SUCCESS;
    CAN_Type * base = g_flexcanBase[instance];
    flexcan_state_t * state = g_flexcanStatePtr[instance];

    /* Start receiving mailbox */
    if(state->isRxBusy)
    {
        return FLEXCAN_STATUS_RX_BUSY;
    }
    state->isRxBusy = true;
    state->mb_message = data;
    state->isRxBlocking = isBlocking;

    /* Enable MB interrupt*/
    result = FLEXCAN_HAL_SetMsgBuffIntCmd(base, mb_idx, true);
    /* Enable error interrupts */
    FLEXCAN_HAL_SetErrIntCmd(base,FLEXCAN_INT_ERR,true);

    return result;
}


/*FUNCTION**********************************************************************
 *
 * Function Name : FLEXCAN_DRV_StartRxMessageFifoData
 * Description   : Initiate (start) a receive by beginning the process of
 * receiving data and enabling the interrupt.
 * This is not a public API as it is called from other driver functions.
 *
 *END**************************************************************************/
static flexcan_status_t FLEXCAN_DRV_StartRxMessageFifoData(
                    uint8_t instance,
                    flexcan_msgbuff_t *data,
                    bool isBlocking
                    )
{
    CAN_Type * base = g_flexcanBase[instance];
    flexcan_state_t * state = g_flexcanStatePtr[instance];
#if FSL_FEATURE_CAN_HAS_DMA_ENABLE
    edma_status_t edmaStat;
#endif
    
    /* Start receiving fifo */
    if(state->isRxBusy)
    {
        return FLEXCAN_STATUS_RX_BUSY;
    }
    /* Check if RxFIFO feature is enabled */
    if (!FLEXCAN_HAL_IsRxFifoEnabled(base))
    {
        return FLEXCAN_STATUS_FAIL;
    }
    
    state->isRxBusy = true;
    
    state->isRxBlocking = isBlocking;

    /* This will get filled by the interrupt handler */
    state->fifo_message = data;

#if FSL_FEATURE_CAN_HAS_DMA_ENABLE
    if (state->transferType == FLEXCAN_RXFIFO_USING_DMA)
    {
        edma_status_t edmaStatus;
        edma_channel_config_t edmaConfig = {
            EDMA_CHN_DEFAULT_PRIORITY, 
            (uint8_t)EDMA_ANY_CHANNEL,
            g_flexcanDmaRequests[instance],
            FLEXCAN_DRV_CompleteRxFifoDataDMA,
            (void *)((uint32_t)instance)
        };

        edmaStatus = EDMA_DRV_ChannelInit(&(state->rxFifoDMAChannel), &edmaConfig);

        if (edmaStatus != EDMA_STATUS_SUCCESS)
        {
            return FLEXCAN_STATUS_FAIL;
        }
 
        edmaStatus = EDMA_DRV_ConfigSingleBlockTransfer(&(state->rxFifoDMAChannel),
            EDMA_TRANSFER_MEM2MEM, (uint32_t)(base->RAMn), (uint32_t)(state->fifo_message),
            EDMA_TRANSFER_SIZE_4B, 16U);

        if (edmaStatus != EDMA_STATUS_SUCCESS)
        {
            (void)EDMA_DRV_ReleaseChannel(&(state->rxFifoDMAChannel));
            return FLEXCAN_STATUS_FAIL;
        }

        edmaStat = EDMA_DRV_StartChannel(&(state->rxFifoDMAChannel));
        if (edmaStat != EDMA_STATUS_SUCCESS)
        {
            return FLEXCAN_STATUS_FAIL;
        }

        return FLEXCAN_STATUS_SUCCESS;
    }
#endif
    
    /* Enable RX FIFO interrupts*/
    (void)FLEXCAN_HAL_SetMsgBuffIntCmd(base, FSL_FEATURE_CAN_RXFIFO_FRAME_AVAILABLE, true);
    (void)FLEXCAN_HAL_SetMsgBuffIntCmd(base, FSL_FEATURE_CAN_RXFIFO_WARNING, true);
    (void)FLEXCAN_HAL_SetMsgBuffIntCmd(base, FSL_FEATURE_CAN_RXFIFO_OVERFLOW, true);
    
    /* Enable error interrupts */
    FLEXCAN_HAL_SetErrIntCmd(base,FLEXCAN_INT_ERR,true);

    return FLEXCAN_STATUS_SUCCESS;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : FLEXCAN_DRV_CompleteSendData
 * Description   : Finish up a transmit by completing the process of sending
 * data and disabling the interrupt.
 * This is not a public API as it is called from other driver functions.
 *
 *END**************************************************************************/
static void FLEXCAN_DRV_CompleteSendData(uint32_t instance)
{
#ifdef DEV_ERROR_DETECT
    DEV_ASSERT(instance < CAN_INSTANCE_COUNT);
#endif
    CAN_Type * base = g_flexcanBase[instance];
    flexcan_state_t * state = g_flexcanStatePtr[instance];

    /* Disable the transmitter data register empty interrupt */
    (void)FLEXCAN_HAL_SetMsgBuffIntCmd(base, state->tx_mb_idx, false);
    /* Disable error interrupts */
    FLEXCAN_HAL_SetErrIntCmd(base,FLEXCAN_INT_ERR,false);

    /* Update the information of the module driver state */
    if (state->isTxBlocking)
    {
        (void)OSIF_SemaPost(&state->txSema);
    }
    state->isTxBusy = false;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : FLEXCAN_DRV_CompleteRxMessageBufferData
 * Description   : Finish up a receive by completing the process of receiving
 * data and disabling the interrupt.
 * This is not a public API as it is called from other driver functions.
 *
 *END**************************************************************************/
static void FLEXCAN_DRV_CompleteRxMessageBufferData(uint32_t instance)
{
#ifdef DEV_ERROR_DETECT
    DEV_ASSERT(instance < CAN_INSTANCE_COUNT);
#endif
    CAN_Type * base = g_flexcanBase[instance];
    flexcan_state_t * state = g_flexcanStatePtr[instance];

    (void)FLEXCAN_HAL_SetMsgBuffIntCmd(base, state->rx_mb_idx, false);
    /* Disable error interrupts */
    FLEXCAN_HAL_SetErrIntCmd(base,FLEXCAN_INT_ERR,false);

    /* Update the information of the module driver state */
    if (state->isRxBlocking)
    {
        (void)OSIF_SemaPost(&state->rxSema);
    }
    state->isRxBusy = false;
}

#if FSL_FEATURE_CAN_HAS_DMA_ENABLE
/*FUNCTION**********************************************************************
 *
 * Function Name : FLEXCAN_DRV_CompleteRxFifoDataDMA
 * Description   : Finish up a DMA transfer (this is just a wrapper over
 * FLEXCAN_DRV_CompleteRxMessageFifoData).
 * This is not a public API as it is called from other driver functions.
 *
 *END**************************************************************************/
static void FLEXCAN_DRV_CompleteRxFifoDataDMA(void *parameter, edma_chn_status_t status)
{
    uint32_t instance = (uint32_t)parameter;
    (void)status;

    FLEXCAN_DRV_CompleteRxMessageFifoData(instance);
}
#endif

/*FUNCTION**********************************************************************
 *
 * Function Name : FLEXCAN_DRV_CompleteRxMessageFifoData
 * Description   : Finish up a receive by completing the process of receiving
 * data and disabling the interrupt.
 * This is not a public API as it is called from other driver functions.
 *
 *END**************************************************************************/
static void FLEXCAN_DRV_CompleteRxMessageFifoData(uint32_t instance)
{
#ifdef DEV_ERROR_DETECT
    DEV_ASSERT(instance < CAN_INSTANCE_COUNT);
#endif

    CAN_Type * base = g_flexcanBase[instance];
    flexcan_state_t * state = g_flexcanStatePtr[instance];

    if (state->transferType == FLEXCAN_RXFIFO_USING_INTERRUPTS)
    {
        /* Disable RX FIFO interrupts*/
        (void)FLEXCAN_HAL_SetMsgBuffIntCmd(base, FSL_FEATURE_CAN_RXFIFO_FRAME_AVAILABLE, false);
        (void)FLEXCAN_HAL_SetMsgBuffIntCmd(base, FSL_FEATURE_CAN_RXFIFO_WARNING, false);
        (void)FLEXCAN_HAL_SetMsgBuffIntCmd(base, FSL_FEATURE_CAN_RXFIFO_OVERFLOW, false);

        /* Disable error interrupts */
        FLEXCAN_HAL_SetErrIntCmd(base,FLEXCAN_INT_ERR,false);
    }
#if FSL_FEATURE_CAN_HAS_DMA_ENABLE
    else
    {
        (void)EDMA_DRV_ReleaseChannel(&(state->rxFifoDMAChannel));
        /* Adjust the ID if it is not extended */
        if (((state->fifo_message->cs) & CAN_CS_IDE_MASK) == 0U)
        {
            state->fifo_message->msgId = state->fifo_message->msgId  >> CAN_ID_STD_SHIFT;
        }
        /* Extract the data length */
        state->fifo_message->dataLen = (uint8_t)((state->fifo_message->cs & CAN_CS_DLC_MASK) >> CAN_CS_DLC_SHIFT);
    }
#endif
    /* Clear fifo message*/
    state->fifo_message = NULL;

    /* Update status for receive by using fifo*/
    if (state->isRxBlocking)
    {
        (void)OSIF_SemaPost(&state->rxSema);
    }
    state->isRxBusy = false;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : FLEXCAN_DRV_InstallEventCallback
 * Description   : Installs a callback function for the IRQ handler.
 *
 * Implements    : FLEXCAN_DRV_InstallEventCallback_Activity
 *END**************************************************************************/
void FLEXCAN_DRV_InstallEventCallback(uint8_t instance, flexcan_callback_t callback, void *callbackParam)
{
#ifdef DEV_ERROR_DETECT
    DEV_ASSERT(instance < CAN_INSTANCE_COUNT);
#endif

    flexcan_state_t * state = g_flexcanStatePtr[instance];
    
    state->callback = callback;
    state->callbackParam = callbackParam;
}

/*******************************************************************************
 * EOF
 ******************************************************************************/
