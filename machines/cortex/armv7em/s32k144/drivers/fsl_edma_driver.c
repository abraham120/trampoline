/*
 * Copyright (c) 2013 - 2016, Freescale Semiconductor, Inc.
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
 * @file fsl_edma_driver.c
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
 * Violates MISRA 2012 Required Rule 10.3, Expression assigned to a narrower or different essential type.
 * The cast is required to perform a conversion between an unsigned integer and an enum type with many values.
 *
 * @section [global]
 * Violates MISRA 2012 Advisory Rule 10.5, Impermissible cast.
 * The cast is required to perform a conversion between an unsigned integer and an enum type with many values.
 *
 * @section [global]
 * Violates MISRA 2012 Advisory Rule 11.4, Conversion between a pointer and
 * integer type.
 * The cast is required to perform a conversion between a pointer and an unsigned long define,
 * representing an address.
 *
 * @section [global]
 * Violates MISRA 2012 Required Rule 11.6, Cast from pointer to unsigned long, Cast from unsigned long to pointer.
 * The cast is required to perform a conversion between a pointer and an unsigned long define,
 * representing an address.
 */

#include "fsl_edma_driver.h"
#include "fsl_clock_manager.h"
#include "fsl_interrupt_manager.h"

/*******************************************************************************
 * Variables
 ******************************************************************************/
/*! @brief EDMA global structure to maintain eDMA state */
static edma_state_t *s_edma = NULL;

/*******************************************************************************
 * PROTOTYPES
 ******************************************************************************/
static edma_status_t EDMA_DRV_ClaimChannel(uint8_t channel, dma_request_source_t source, edma_chn_state_t *chn);
static void EDMA_DRV_ClearIntStatus(uint8_t channel);
static void EDMA_DRV_ClearSoftwareTCD(edma_software_tcd_t * stcd);
static bool EDMA_DRV_ValidTransferSize(edma_transfer_size_t size);

/*******************************************************************************
 * Code
 ******************************************************************************/
/*FUNCTION**********************************************************************
 *
 * Function Name : EDMA_DRV_Init
 * Description   : Initializes the eDMA module.
 *
 * Implements    : EDMA_DRV_Init_Activity
 *END**************************************************************************/
edma_status_t EDMA_DRV_Init(edma_state_t *edmaState, const edma_user_config_t *userConfig)
{
    /* Variables storing the DMA clock names, as defined in the SoC specific files. */
    clock_names_t dmaClockName = SIM_DMA_CLOCK;
    clock_names_t dmamuxClockName = PCC_DMAMUX0_CLOCK;
    uint32_t i;
    DMA_Type * edmaRegBase;
    DMAMUX_Type * dmamuxRegBase;
    IRQn_Type irqNumber;
    edma_status_t edmaStatus = EDMA_STATUS_SUCCESS;
    clock_manager_error_code_t clockStatus = CLOCK_MANAGER_ERROR_UNKNOWN;
    if ((edmaState == NULL) || (userConfig == NULL))
    {
        edmaStatus = EDMA_STATUS_INVALID_ARGUMENT;
    }
    if (edmaStatus == EDMA_STATUS_SUCCESS)
    {
        if (s_edma != NULL)
        {
            edmaStatus = EDMA_STATUS_ALREADY_INITIALIZED;
        }
    }
    if (edmaStatus == EDMA_STATUS_SUCCESS)
    {
        clockStatus = CLOCK_SYS_GetFreq(dmaClockName, NULL);
        
        if (clockStatus == CLOCK_MANAGER_NO_CLOCK)
        {
            /* Return an error if the clock is disabled for DMA operations */
            edmaStatus = EDMA_STATUS_CLOCK_GATED_OFF;
        }
        else if (clockStatus == CLOCK_MANAGER_SUCCESS)
        {
            clockStatus = CLOCK_SYS_GetFreq(dmamuxClockName, NULL);
            if (clockStatus == CLOCK_MANAGER_NO_CLOCK)
            {
                /* Return an error if the clock is disabled for DMA operations */
                edmaStatus = EDMA_STATUS_CLOCK_GATED_OFF;
            }
            else if (clockStatus != CLOCK_MANAGER_SUCCESS)
            {
                edmaStatus = EDMA_STATUS_FAIL;

            }
            if (edmaStatus == EDMA_STATUS_SUCCESS)
            {
                s_edma = edmaState;

                /* Clear the state structure. */
                volatile uint8_t *clearStructPtr = (volatile uint8_t *)s_edma;
                size_t clearSize = sizeof(edma_state_t);
                while (clearSize > 0U)
                {
                    *clearStructPtr = 0;
                    clearStructPtr ++;
                    clearSize --;
                }

                edmaRegBase = g_edmaBase[0U];

                /* Init eDMA module on hardware level. */
                EDMA_HAL_Init(edmaRegBase);

                /* Set arbitration mode */
                EDMA_HAL_SetChannelArbitrationMode(edmaRegBase, userConfig->chnArbitration);

                /* Set 'Halt on error' configuration */
                EDMA_HAL_SetHaltOnErrorCmd(edmaRegBase, !userConfig->notHaltOnError);

            #if defined FSL_FEATURE_EDMA_HAS_ERROR_IRQ
                /* Enable the error interrupt for eDMA module. */
                irqNumber = g_edmaErrIrqId[0U];
                INT_SYS_EnableIRQ(irqNumber);
            #endif

                /* Register all edma channel interrupt handler into vector table. */
                for (i = 0U; i < FSL_FEATURE_EDMA_MODULE_CHANNELS; i++)
                {
                    /* Enable channel interrupt ID. */
                    irqNumber = g_edmaIrqId[i];
                    INT_SYS_EnableIRQ(irqNumber);
                }

                /* Initialize DMAMUX */
                dmamuxRegBase = g_dmamuxBase[0U];
                DMAMUX_HAL_Init(dmamuxRegBase);
            }
            else
            {
               /* Complete if-elseif-else block to avoid violating MISRA 2012 Rule 15.7 */
            }
        }
        else
        {
            edmaStatus = EDMA_STATUS_FAIL;
        }
    }
    return edmaStatus;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : EDMA_DRV_Deinit
 * Description   : Deinitialize EDMA.
 *
 * Implements    : EDMA_DRV_Deinit_Activity
 *END**************************************************************************/
edma_status_t EDMA_DRV_Deinit(void)
{
    uint32_t i;
    IRQn_Type irqNumber;
    edma_chn_state_t *chn;
    edma_status_t status = EDMA_STATUS_SUCCESS;

    /* Release all edma channel. */
#if defined FSL_FEATURE_EDMA_HAS_ERROR_IRQ
    /* Disable the error interrupt for eDMA module. */
    irqNumber = g_edmaErrIrqId[0U];
    INT_SYS_DisableIRQ(irqNumber);
#endif
    if (s_edma != NULL)
    {
        for (i = 0U; i < FSL_FEATURE_EDMA_MODULE_CHANNELS; i++)
        {
            /* Release all channels. */
            chn = s_edma->chn[i];
            if (chn != NULL)
            {
                status = EDMA_DRV_ReleaseChannel(chn);
            }

            /* Disable channel interrupt ID. */
            irqNumber = g_edmaIrqId[i];
            INT_SYS_DisableIRQ(irqNumber);
        }

        s_edma = NULL;
    }

    return status;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : EDMA_DRV_ChannelInit
 * Description   : Initialize EDMA channel.
 *
 * Implements    : EDMA_DRV_ChannelInit_Activity
 *END**************************************************************************/
edma_status_t EDMA_DRV_ChannelInit(edma_chn_state_t *edmaChannelState, const edma_channel_config_t *edmaChannelConfig)
{
    uint8_t allocatedChannel = (uint8_t) EDMA_INVALID_CHANNEL;
    DMA_Type * edmaRegBase = g_edmaBase[0U];
    edma_status_t status = EDMA_STATUS_SUCCESS;

    if ((edmaChannelState == NULL) || (edmaChannelConfig == NULL))
    {
        status = EDMA_STATUS_INVALID_ARGUMENT;
    }
    if (status == EDMA_STATUS_SUCCESS)
    {
        /* Check if DMA is initialized */
        if (s_edma == NULL)
        {
            status = EDMA_STATUS_FAIL;
        }
    }
    /* Check if the channel defined by user in the channel configuration structure is valid */
    if (status == EDMA_STATUS_SUCCESS)
    {
        if ((edmaChannelConfig->channel > FSL_FEATURE_EDMA_MODULE_CHANNELS) &&
            (edmaChannelConfig->channel != (uint8_t) EDMA_ANY_CHANNEL))
        {
            status = EDMA_STATUS_INVALID_ARGUMENT;
        }
    }
    if (status == EDMA_STATUS_SUCCESS)
    {
        /* Request the channel */
        status = EDMA_DRV_RequestChannel(edmaChannelConfig->channel, edmaChannelConfig->source, edmaChannelState, &allocatedChannel);
    }
    if (status == EDMA_STATUS_SUCCESS)
    {
        /* Return error code if no channel has been allocated */
        if (allocatedChannel == (uint8_t) EDMA_INVALID_CHANNEL)
        {
            status = EDMA_STATUS_FAIL;
        }
    }
    if (status == EDMA_STATUS_SUCCESS)
    {
        /* Set the channel priority, as defined in the configuration structure, only if fixed arbitration mode is selected */
        if ((EDMA_HAL_GetChannelArbitrationMode(edmaRegBase) == EDMA_ARBITRATION_FIXED_PRIORITY) &&
            (edmaChannelConfig->priority != EDMA_CHN_DEFAULT_PRIORITY))
        {
            EDMA_HAL_SetChannelPriority(edmaRegBase, allocatedChannel, edmaChannelConfig->priority);
        }

        /* Install the user callback */
        status = EDMA_DRV_InstallCallback(edmaChannelState, edmaChannelConfig->callback, edmaChannelConfig->callbackParam);
    }
    return status;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : EDMA_DRV_InstallCallback
 * Description   : Register callback function and parameter.
 *
 * Implements    : EDMA_DRV_InstallCallback_Activity
 *END**************************************************************************/
edma_status_t EDMA_DRV_InstallCallback(edma_chn_state_t *chn, edma_callback_t callback, void *parameter)
{
    edma_status_t status = EDMA_STATUS_SUCCESS;
    if (chn == NULL)
    {
        status = EDMA_STATUS_INVALID_ARGUMENT;
    }
    else
    {
        chn->callback = callback;
        chn->parameter = parameter;
    }
    return status;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : EDMA_DRV_RequestChannel
 * Description   : Request an eDMA channel.
 *
 * Implements    : EDMA_DRV_RequestChannel_Activity
 *END**************************************************************************/
edma_status_t EDMA_DRV_RequestChannel(uint8_t requestedChannel, dma_request_source_t source,
                                      edma_chn_state_t *chn, uint8_t * allocatedChannel)
{
    edma_status_t status = EDMA_STATUS_SUCCESS;
    if ((chn == NULL) || (allocatedChannel == NULL))
    {
        status = EDMA_STATUS_INVALID_ARGUMENT;
    }
    if (status == EDMA_STATUS_SUCCESS)
    {
        /* Check if DMA is initialized */
        if (s_edma == NULL)
        {
            status = EDMA_STATUS_FAIL;
        }
    }
    if (status == EDMA_STATUS_SUCCESS)
    {
        /*Check if dynamic allocation is requested */
        if (requestedChannel == (uint8_t) EDMA_ANY_CHANNEL)
        {
            uint8_t i;

            /* Default: no available channel. */
            *allocatedChannel = (uint8_t) EDMA_INVALID_CHANNEL;
            status = EDMA_STATUS_FAIL;

            for (i=0U; i < FSL_FEATURE_DMAMUX_MODULE_CHANNELS; i++)
            {
                if (OSIF_MutexLock(&s_edma->lock, OSIF_WAIT_FOREVER) == OSIF_STATUS_SUCCESS)
                {
                    bool exitLoop = false;
                    if (!s_edma->chn[i])
                    {
                        s_edma->chn[i] = chn;
                        if (OSIF_MutexUnlock(&s_edma->lock) == OSIF_STATUS_SUCCESS)
                        {
                            status = EDMA_DRV_ClaimChannel(i, source, chn);
                            *allocatedChannel = i;
                        }
                        exitLoop = true;
                    }
                    else
                    {
                        if (OSIF_MutexUnlock(&s_edma->lock) != OSIF_STATUS_SUCCESS)
                        {
                            status = EDMA_STATUS_FAIL;
                            exitLoop = true;
                        }
                    }
                    if (exitLoop)
                    {
                        break;
                    }
                }
            }
        }
        else
        {
            /* Default value: return error code if no channel has been allocated */
            *allocatedChannel = (uint8_t) EDMA_INVALID_CHANNEL;
            status = EDMA_STATUS_FAIL;

            /* Static allocation */
            if (OSIF_MutexLock(&s_edma->lock, OSIF_WAIT_FOREVER) == OSIF_STATUS_SUCCESS)
            {
                if (!s_edma->chn[requestedChannel])
                {
                    s_edma->chn[requestedChannel] = chn;
                    if (OSIF_MutexUnlock(&s_edma->lock) == OSIF_STATUS_SUCCESS)
                    {
                        status = EDMA_DRV_ClaimChannel(requestedChannel, source, chn);
                        *allocatedChannel = requestedChannel;
                    }
                }
                else
                {
                    (void) OSIF_MutexUnlock(&s_edma->lock);
                }
            }
        }
    }
    return status;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : EDMA_DRV_ClaimChannel
 * Description   : Claim an edma channel.
 *
 *END**************************************************************************/
static edma_status_t EDMA_DRV_ClaimChannel(uint8_t channel, dma_request_source_t source, edma_chn_state_t *chn)
{
    edma_status_t status = EDMA_STATUS_SUCCESS;
    if (chn == NULL)
    {
        status = EDMA_STATUS_INVALID_ARGUMENT;
    }
    if (status == EDMA_STATUS_SUCCESS)
    {
        DMA_Type * edmaRegBase = g_edmaBase[0U];
        DMAMUX_Type * dmamuxRegBase = g_dmamuxBase[0U];

        /* Reset the channel state structure to default value. */
        uint8_t *clearStructPtr = (uint8_t *)chn;
        size_t clearSize = sizeof(edma_chn_state_t);
        while (clearSize > 0U)
        {
            *clearStructPtr = 0;
            clearStructPtr ++;
            clearSize --;
        }

        /* Init the channel state structure to the allocated channel number. */
        chn->channel = channel;

        /* Enable error interrupt for this channel */
        EDMA_HAL_SetErrorIntCmd(edmaRegBase, (edma_channel_indicator_t)channel, true);

        /* Configure the DMAMUX for edma channel */
        DMAMUX_HAL_SetChannelCmd(dmamuxRegBase, channel, false);
        DMAMUX_HAL_SetChannelSource(dmamuxRegBase, channel, (uint8_t)source);
        DMAMUX_HAL_SetChannelCmd(dmamuxRegBase, channel, true);

        /* Clear the TCD registers for this channel */
        EDMA_HAL_TCDClearReg(edmaRegBase, channel);
    }
    return status;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : EDMA_DRV_ReleaseChannel
 * Description   : Free eDMA channel's hardware and software resource.
 *
 * Implements    : EDMA_DRV_ReleaseChannel_Activity
 *END**************************************************************************/
edma_status_t EDMA_DRV_ReleaseChannel(edma_chn_state_t *chn)
{
    uint8_t channel;
    DMA_Type * edmaRegBase;
    edma_status_t status = EDMA_STATUS_SUCCESS;
    if (chn == NULL)
    {
        status = EDMA_STATUS_INVALID_ARGUMENT;
    }
    else
    {
        /* Check if DMA is initialized */
        if (s_edma == NULL)
        {
            status = EDMA_STATUS_FAIL;
        }
    }
    if (status == EDMA_STATUS_SUCCESS)
    {
        channel = chn->channel;
        edmaRegBase = g_edmaBase[0U];

        if (!s_edma->chn[channel])
        {
            status = EDMA_STATUS_INVALID_ARGUMENT;
        }
        if (status == EDMA_STATUS_SUCCESS)
        {

            /* Stop edma channel. */
            EDMA_HAL_SetDmaRequestCmd(edmaRegBase, (edma_channel_indicator_t)channel, false);

            /* Reset the channel state structure to default value. */
            uint8_t *clearStructPtr = (uint8_t *)chn;
            size_t clearSize = sizeof(edma_chn_state_t);
            while (clearSize > 0U)
            {
                *clearStructPtr = 0;
                clearStructPtr ++;
                clearSize --;
            }

            s_edma->chn[channel] = NULL;
        }
    }
    return status;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : EDMA_DRV_ClearIntStatus
 * Description   : Clear done and interrupt status.
 *
 *END**************************************************************************/
static void EDMA_DRV_ClearIntStatus(uint8_t channel)
{
    DMA_Type * edmaRegBase = g_edmaBase[0U];
    EDMA_HAL_ClearDoneStatusFlag(edmaRegBase, (edma_channel_indicator_t)channel);
    EDMA_HAL_ClearIntStatusFlag(edmaRegBase, (edma_channel_indicator_t)channel);
}

/*FUNCTION**********************************************************************
 *
 * Function Name : EDMA_DRV_ClearSoftwareTCD
 * Description   : Clear the software tcd structure.
 *
 *END**************************************************************************/
static void EDMA_DRV_ClearSoftwareTCD(edma_software_tcd_t * stcd)
{
    uint8_t *byteAccess = (uint8_t *)stcd;
    size_t clearSize = sizeof(edma_software_tcd_t);
    while (clearSize > 0U)
    {
        *byteAccess = 0;
        byteAccess ++;
        clearSize --;
    }
}

/*FUNCTION**********************************************************************
 *
 * Function Name : EDMA_DRV_IRQHandler
 * Description   : EDMA IRQ handler.
 *
 * Implements    : EDMA_DRV_IRQHandler_Activity
 *END**************************************************************************/
void EDMA_DRV_IRQHandler(uint8_t channel)
{
    const edma_chn_state_t *chn = s_edma->chn[channel];

    EDMA_DRV_ClearIntStatus(channel);

    if (chn != NULL)
    {
        if (chn->callback != NULL)
        {
            chn->callback(chn->parameter, chn->status);
        }
    }
}

/*FUNCTION**********************************************************************
 *
 * Function Name : EDMA_DRV_ErrorIRQHandler
 * Description   : EDMA error IRQ handler
 *
 * Implements    : EDMA_DRV_ErrorIRQHandler_Activity
 *END**************************************************************************/
void EDMA_DRV_ErrorIRQHandler(void)
{
    uint8_t channel = 0U;
    uint32_t error;
    DMA_Type * edmaRegBase = g_edmaBase[0U];
    edma_chn_state_t *chn;

    error = EDMA_HAL_GetErrorIntStatusFlag(edmaRegBase);

    while ((error != 0UL) && (channel < (uint8_t) FSL_FEATURE_EDMA_MODULE_CHANNELS))
    {
        if ((error & EDMA_ERR_LSB_MASK) != 0UL)
        {
            EDMA_HAL_SetDmaRequestCmd(edmaRegBase, (edma_channel_indicator_t)channel, false);
            chn = s_edma->chn[channel];
            if (chn != NULL)
            {
                EDMA_DRV_ClearIntStatus(channel);
                EDMA_HAL_ClearErrorIntStatusFlag(edmaRegBase, (edma_channel_indicator_t)channel);
                chn->status = EDMA_CHN_ERROR;
                if (chn->callback != NULL)
                {
                    chn->callback(chn->parameter, chn->status);
                }
            }
        }
        error = error >> 1U;
        channel++;
    }
    EDMA_HAL_SetHaltCmd(edmaRegBase, false);
}

/*FUNCTION**********************************************************************
 *
 * Function Name : EDMA_DRV_ConfigSingleBlockTransfer
 * Description   : Configures a DMA single block transfer.
 *
 * Implements    : EDMA_DRV_ConfigSingleBlockTransfer_Activity
 *END**************************************************************************/
edma_status_t EDMA_DRV_ConfigSingleBlockTransfer(
                            const edma_chn_state_t *chn, edma_transfer_type_t type,
                            uint32_t srcAddr, uint32_t destAddr,
                            edma_transfer_size_t transferSize, uint32_t dataBufferSize)
{
#ifdef DEV_ERROR_DETECT
    DEV_ASSERT(chn != NULL);
#endif
    DMA_Type * edmaRegBase = g_edmaBase[0U];
    uint8_t transferOffset;
    edma_status_t status = EDMA_STATUS_SUCCESS;
    if (chn == NULL)
    {
        status = EDMA_STATUS_INVALID_ARGUMENT;
    }
    /* Check if the value passed for 'transferSize' is valid */
    if (!EDMA_DRV_ValidTransferSize(transferSize))
    {
        status = EDMA_STATUS_INVALID_ARGUMENT;
    }

    /* Compute the transfer offset, based on transfer size. 
     * The number of bytes transferred in each source read/destination write
     * is obtained with the following formula:
     *    source_read_size = 2^SSIZE
     *    destination_write_size = 2^DSIZE
     */
    transferOffset = (uint8_t) (1U << ((uint8_t)transferSize));

    /* The number of bytes to be transferred (buffer size) must
     * be a multiple of the source read/destination write size
     */
    if ((dataBufferSize % transferOffset) != 0U)
    {
        status = EDMA_STATUS_INVALID_ARGUMENT;
    }

    if (status == EDMA_STATUS_SUCCESS)
    {
        /* Clear transfer control descriptor for the current channel */
        EDMA_HAL_TCDClearReg(edmaRegBase, chn->channel);

        /* Configure source and destination addresses */
        EDMA_HAL_TCDSetSrcAddr(edmaRegBase, chn->channel, srcAddr);
        EDMA_HAL_TCDSetDestAddr(edmaRegBase, chn->channel, destAddr);

        /* Set transfer size (1B/2B/4B/16B/32B) */
        EDMA_HAL_TCDSetAttribute(edmaRegBase, chn->channel, EDMA_MODULO_OFF, EDMA_MODULO_OFF, transferSize, transferSize);

        /* Configure source/destination offset. */
        switch (type)
        {
            case EDMA_TRANSFER_PERIPH2MEM:
                EDMA_HAL_TCDSetSrcOffset(edmaRegBase, chn->channel, 0);
                EDMA_HAL_TCDSetDestOffset(edmaRegBase, chn->channel, (int8_t) transferOffset);
                break;
            case EDMA_TRANSFER_MEM2PERIPH:
                EDMA_HAL_TCDSetSrcOffset(edmaRegBase, chn->channel, (int8_t) transferOffset);
                EDMA_HAL_TCDSetDestOffset(edmaRegBase, chn->channel, 0);
                break;
            case EDMA_TRANSFER_MEM2MEM:
                EDMA_HAL_TCDSetSrcOffset(edmaRegBase, chn->channel, (int8_t) transferOffset);
                EDMA_HAL_TCDSetDestOffset(edmaRegBase, chn->channel, (int8_t) transferOffset);
                break;
            default:
                /* This should never be reached - all the possible values have been handled. */
                break;
        }

        /* Set the total number of bytes to be transfered */
        EDMA_HAL_TCDSetNbytes(edmaRegBase, chn->channel, dataBufferSize);

        /* Set major iteration count to 1 (single block mode) */
        EDMA_HAL_TCDSetMajorCount(edmaRegBase, chn->channel, 1U);

        /* Enable interrupt when the transfer completes */
        EDMA_HAL_TCDSetIntCmd(edmaRegBase, chn->channel, true);
    }
    return status;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : EDMA_DRV_ConfigLoopTransfer
 * Description   : Configures the DMA transfer in a loop.
 *
 * Implements    : EDMA_DRV_ConfigLoopTransfer_Activity
 *END**************************************************************************/
edma_status_t EDMA_DRV_ConfigLoopTransfer(const edma_chn_state_t *chn, const edma_transfer_config_t *transferConfig)
{
#ifdef DEV_ERROR_DETECT
    DEV_ASSERT(transferConfig != 0)
    DEV_ASSERT(chn != NULL);
#endif
    edma_status_t status = EDMA_STATUS_SUCCESS;
    if ((chn == NULL) || (transferConfig == NULL))
    {
        status = EDMA_STATUS_INVALID_ARGUMENT;
    }

    /* Check if the minor/major loop properties are defined */
    if (status == EDMA_STATUS_SUCCESS)
    {
        if (transferConfig->loopTransferConfig == NULL)
        {
            status = EDMA_STATUS_INVALID_ARGUMENT;
        }
    }

    if (status == EDMA_STATUS_SUCCESS)
    {
        /* Enable minor loop mapping */
        DMA_Type * edmaRegBase = g_edmaBase[0U];
        EDMA_HAL_SetMinorLoopMappingCmd(edmaRegBase, true);

        /* Write the configuration in the transfer control descriptor registers */
        status = EDMA_DRV_PushConfigToReg(chn, transferConfig);

        /* Enable interrupt when major loop count completes */
        EDMA_HAL_TCDSetIntCmd(edmaRegBase, chn->channel, true);
    }

    return status;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : EDMA_DRV_ConfigScatterGatherTransfer
 * Description   : Configure eDMA for scatter/gather operation
 *
 * Implements    : EDMA_DRV_ConfigScatterGatherTransfer_Activity
 *END**************************************************************************/
edma_status_t EDMA_DRV_ConfigScatterGatherTransfer(
                        const edma_chn_state_t *chn, edma_software_tcd_t *stcd,
                        edma_transfer_size_t transferSize, uint32_t bytesOnEachRequest,
                        const edma_scatter_gather_list_t *srcList, const edma_scatter_gather_list_t *destList,
                        uint8_t tcdCount)
{
#ifdef DEV_ERROR_DETECT
    DEV_ASSERT(stcd != 0)
    DEV_ASSERT(chn != NULL);
#endif

    uint8_t i;
    uint16_t transferOffset;
    uint32_t stcdAlignedAddr = STCD_ADDR(stcd);
    edma_software_tcd_t *stcdAddr = (edma_software_tcd_t *)stcdAlignedAddr;
    edma_loop_transfer_config_t loopConfig;
    edma_transfer_config_t config;
    edma_status_t status = EDMA_STATUS_SUCCESS;
    if ((chn == NULL) || (stcd == NULL) || (srcList == NULL) || (destList == NULL))
    {
        status = EDMA_STATUS_INVALID_ARGUMENT;
    }

    /* Check if the value passed for 'transferSize' is valid */
    if (!EDMA_DRV_ValidTransferSize(transferSize))
    {
        status = EDMA_STATUS_INVALID_ARGUMENT;
    }
    
    /* Compute the transfer offset, based on transfer size. 
     * The number of bytes transferred in each source read/destination write
     * is obtained with the following formula:
     *    source_read_size = 2^SSIZE
     *    destination_write_size = 2^DSIZE
     */
    transferOffset = (uint16_t) (1UL << ((uint16_t)transferSize));

    /* The number of bytes to be transferred on each request must
     * be a multiple of the source read/destination write size
     */
    if ((bytesOnEachRequest % transferOffset) != 0U)
    {
        status = EDMA_STATUS_INVALID_ARGUMENT;
    }

    if (status == EDMA_STATUS_SUCCESS)
    {
        /* Clear the configuration structures before initializing them. */
        uint8_t *clearStructPtr = (uint8_t *)(&config);
        size_t clearSize = sizeof(edma_transfer_config_t);
        while (clearSize > 0U)
        {
            *clearStructPtr = 0;
            clearStructPtr ++;
            clearSize --;
        }

        clearStructPtr = (uint8_t *)(&loopConfig);
        clearSize = sizeof(edma_loop_transfer_config_t);
        while (clearSize > 0U)
        {
            *clearStructPtr = 0;
            clearStructPtr ++;
            clearSize --;
        }

        /* Configure the transfer for scatter/gather mode. */
        config.srcLastAddrAdjust = 0;
        config.destLastAddrAdjust = 0;
        config.srcModulo = EDMA_MODULO_OFF;
        config.destModulo = EDMA_MODULO_OFF;
        config.srcTransferSize = transferSize;
        config.destTransferSize = transferSize;
        config.minorByteTransferCount = bytesOnEachRequest;
        config.interruptEnable = true;
        config.scatterGatherEnable = true;
        config.loopTransferConfig = &loopConfig;
        config.loopTransferConfig->srcOffsetEnable = false;
        config.loopTransferConfig->dstOffsetEnable = false;
        config.loopTransferConfig->minorLoopChnLinkEnable = false;
        config.loopTransferConfig->majorLoopChnLinkEnable = false;

        /* Copy scatter/gather lists to transfer configuration*/
        for (i = 0U; (i < tcdCount) && (status == EDMA_STATUS_SUCCESS); i++)
        {
            config.srcAddr = srcList[i].address;
            config.destAddr = destList[i].address;
            if ((srcList[i].length != destList[i].length) || (srcList[i].type != destList[i].type))
            {
                status = EDMA_STATUS_INVALID_ARGUMENT;
                break;
            }
            config.loopTransferConfig->majorLoopIterationCount = srcList[i].length/bytesOnEachRequest;

            switch (srcList[i].type)
            {
                case EDMA_TRANSFER_PERIPH2MEM:
                    /* Configure Source Read. */
                    config.srcOffset = 0;
                    /* Configure Dest Write. */
                    config.destOffset = (int16_t) transferOffset;
                    break;
                case EDMA_TRANSFER_MEM2PERIPH:
                    /* Configure Source Read. */
                    config.srcOffset = (int16_t) transferOffset;
                    /* Configure Dest Write. */
                    config.destOffset = 0;
                    break;
                case EDMA_TRANSFER_MEM2MEM:
                    /* Configure Source Read. */
                    config.srcOffset = (int16_t) transferOffset;
                    /* Configure Dest Write. */
                    config.destOffset = (int16_t) transferOffset;
                    break;
                default:
                    /* This should never be reached - all the possible values have been handled. */
                    break;
            }

            /* Configure the pointer to next software TCD structure; for the last one, this address should be 0 */
            if (i == ((uint8_t)(tcdCount - 1U)))
            {
                config.scatterGatherNextDescAddr = 0U;
            }
            else
            {
                edma_software_tcd_t *nextAddr = &stcdAddr[i];
                config.scatterGatherNextDescAddr = ((uint32_t) nextAddr);
            }


            if (i == 0U)
            {
                /* Push the configuration for the first descriptor to registers */
                status = EDMA_DRV_PushConfigToReg(chn, &config);
            }
            else
            {
                /* Copy configuration to software TCD structure */
                EDMA_DRV_PushConfigToSTCD(chn, &config, &stcdAddr[i - 1U]);
            }
        }
    }
    return status;

}

/*FUNCTION**********************************************************************
 *
 * Function Name : EDMA_DRV_StartChannel
 * Description   : Starts an eDMA channel.
 *
 * Implements    : EDMA_DRV_StartChannel_Activity
 *END**************************************************************************/
edma_status_t EDMA_DRV_StartChannel(const edma_chn_state_t *chn)
{
    uint8_t channel = chn->channel;
    DMA_Type * edmaRegBase = g_edmaBase[0U];
    edma_status_t status = EDMA_STATUS_SUCCESS;
    if (chn == NULL)
    {
        status = EDMA_STATUS_INVALID_ARGUMENT;
    }

    if (status == EDMA_STATUS_SUCCESS)
    {
        EDMA_HAL_SetDmaRequestCmd(edmaRegBase, (edma_channel_indicator_t)channel, true);
    }
    return status;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : EDMA_DRV_StopChannel
 * Description   : Stops an eDMA channel.
 *
 * Implements    : EDMA_DRV_StopChannel_Activity
 *END**************************************************************************/
edma_status_t EDMA_DRV_StopChannel(const edma_chn_state_t *chn)
{
    uint8_t channel = chn->channel;
    DMA_Type * edmaRegBase = g_edmaBase[0U];
    edma_status_t status = EDMA_STATUS_SUCCESS;
    if (chn == NULL)
    {
        status = EDMA_STATUS_INVALID_ARGUMENT;
    }

    if (status == EDMA_STATUS_SUCCESS)
    {
        EDMA_HAL_SetDmaRequestCmd(edmaRegBase, (edma_channel_indicator_t)channel, false);
    }
    return status;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : EDMA_DRV_PushConfigToSTCD
 * Description   : Copy the configuration to the software TCD structure.
 *
 * Implements    : EDMA_DRV_PushConfigToSTCD_Activity
 *END**************************************************************************/
void EDMA_DRV_PushConfigToSTCD(const edma_chn_state_t *chn, const edma_transfer_config_t *config, edma_software_tcd_t *stcd)
{
    if ((chn != NULL) && (config != NULL) && (stcd != NULL))
    {
        /* Clear the array of software TCDs passed by the user */
        EDMA_DRV_ClearSoftwareTCD(stcd);

        /* Set the software TCD fields */
        stcd->ATTR = (((uint16_t)config->srcModulo) << DMA_TCD_ATTR_SMOD_SHIFT) | (((uint16_t)config->srcTransferSize) << DMA_TCD_ATTR_SSIZE_SHIFT) |
                     (((uint16_t)config->destModulo) << DMA_TCD_ATTR_DMOD_SHIFT) | (((uint16_t)config->destTransferSize) << DMA_TCD_ATTR_DSIZE_SHIFT);
        stcd->SADDR = config->srcAddr;
        stcd->SOFF = config->srcOffset;
        stcd->NBYTES = config->minorByteTransferCount;
        stcd->SLAST = config->srcLastAddrAdjust;
        stcd->DADDR = config->destAddr;
        stcd->DOFF = config->destOffset;
        stcd->CITER = (uint16_t) config->loopTransferConfig->majorLoopIterationCount;
        if (config->scatterGatherEnable)
        {
            stcd->DLAST_SGA = (int32_t) config->scatterGatherNextDescAddr;
        }
        else
        {
            stcd->DLAST_SGA = config->destLastAddrAdjust;
        }
        stcd->CSR = (uint16_t) (((config->interruptEnable ? 1UL : 0UL) << DMA_TCD_CSR_INTMAJOR_SHIFT) |
                                ((config->scatterGatherEnable ? 1UL : 0UL) << DMA_TCD_CSR_ESG_SHIFT));
        stcd->BITER = (uint16_t) config->loopTransferConfig->majorLoopIterationCount;
    }
}

/*FUNCTION**********************************************************************
 *
 * Function Name : EDMA_DRV_PushConfigToReg
 * Description   : Copy the configuration to the TCD registers.
 *
 * Implements    : EDMA_DRV_PushConfigToReg_Activity
 *END**************************************************************************/
edma_status_t EDMA_DRV_PushConfigToReg(const edma_chn_state_t *chn, const edma_transfer_config_t *tcd)
{
    uint32_t channel;
    DMA_Type * edmaRegBase = g_edmaBase[0U];
    edma_status_t status = EDMA_STATUS_SUCCESS;
    if ((chn == NULL) || (tcd == NULL))
    {
        status = EDMA_STATUS_INVALID_ARGUMENT;
    }
    else
    {
        channel = chn->channel;
        /* Clear TCD registers */
        EDMA_HAL_TCDClearReg(edmaRegBase, channel);

        /* Set source and destination addresses */
        EDMA_HAL_TCDSetSrcAddr(edmaRegBase, channel, tcd->srcAddr);
        EDMA_HAL_TCDSetDestAddr(edmaRegBase, channel, tcd->destAddr);

        /* Set source/destination modulo feature and transfer size */
        EDMA_HAL_TCDSetAttribute(edmaRegBase, channel, tcd->srcModulo, tcd->destModulo, tcd->srcTransferSize, tcd->destTransferSize);

        /* Set source/destination offset and last adjustment; for scatter/gather operation, destination
         * last adjustment is the address of the next TCD structure to be loaded by the eDMA engine */
        EDMA_HAL_TCDSetSrcOffset(edmaRegBase, channel, tcd->srcOffset);
        EDMA_HAL_TCDSetDestOffset(edmaRegBase, channel, tcd->destOffset);
        EDMA_HAL_TCDSetSrcLastAdjust(edmaRegBase, channel, tcd->srcLastAddrAdjust);
        if (tcd->scatterGatherEnable)
        {
            EDMA_HAL_TCDSetScatterGatherCmd(edmaRegBase, channel, true);
            EDMA_HAL_TCDSetScatterGatherLink(edmaRegBase, channel, tcd->scatterGatherNextDescAddr);
        }
        else
        {
            EDMA_HAL_TCDSetScatterGatherCmd(edmaRegBase, channel, false);
            EDMA_HAL_TCDSetDestLastAdjust(edmaRegBase, channel, tcd->destLastAddrAdjust);
        }

        /* Configure channel interrupt */
        EDMA_HAL_TCDSetIntCmd(edmaRegBase, channel, tcd->interruptEnable);

        /* If loop configuration is available, copy minor/major loop setup to registers */
        if (tcd->loopTransferConfig != NULL)
        {
            EDMA_HAL_TCDSetSrcMinorLoopOffsetCmd(edmaRegBase, channel, tcd->loopTransferConfig->srcOffsetEnable);
            EDMA_HAL_TCDSetDestMinorLoopOffsetCmd(edmaRegBase, channel, tcd->loopTransferConfig->dstOffsetEnable);
            EDMA_HAL_TCDSetMinorLoopOffset(edmaRegBase, channel, tcd->loopTransferConfig->minorLoopOffset);
            EDMA_HAL_TCDSetNbytes(edmaRegBase, channel, tcd->minorByteTransferCount);

            EDMA_HAL_TCDSetChannelMinorLink(edmaRegBase, channel, tcd->loopTransferConfig->minorLoopChnLinkNumber, tcd->loopTransferConfig->minorLoopChnLinkEnable);
            EDMA_HAL_TCDSetChannelMajorLink(edmaRegBase, channel, tcd->loopTransferConfig->majorLoopChnLinkNumber, tcd->loopTransferConfig->majorLoopChnLinkEnable);

            EDMA_HAL_TCDSetMajorCount(edmaRegBase, channel, tcd->loopTransferConfig->majorLoopIterationCount);
        }
        else
        {
            EDMA_HAL_TCDSetNbytes(edmaRegBase, channel, tcd->minorByteTransferCount);
        }
    }
    return status;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : EDMA_DRV_ValidTransferSize
 * Description   : Check if the transfer size value is legal (0/1/2/4/5).
 *
 *END**************************************************************************/
static bool EDMA_DRV_ValidTransferSize(edma_transfer_size_t size)
{
    bool isValid;
    switch (size)
    {
        case EDMA_TRANSFER_SIZE_1B:
        case EDMA_TRANSFER_SIZE_2B:
        case EDMA_TRANSFER_SIZE_4B:
        case EDMA_TRANSFER_SIZE_16B:
        case EDMA_TRANSFER_SIZE_32B:
            isValid = true;
            break;
        default:
            isValid = false;
            break;
    }
    return isValid;
}

/*******************************************************************************
 * EOF
 ******************************************************************************/

