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

/*!
 * @fsl_lpi2c_driver.c
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
 * Violates MISRA 2012 Required Rule 5.1, Identifier clash
 * The supported compilers use more than 31 significant characters for identifiers.
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
 * Violates MISRA 2012 Advisory Rule 8.7, External could be made static.
 * Function is defined for usage by application code.
 *
 * @section [global]
 * Violates MISRA 2012 Advisory Rule 8.9, Could define variable at block scope
 * The variables are defined in the common source file and this rule can't be
 * applied.
 *
 * @section [global]
 * Violates MISRA 2012 Required Rule 10.3, Expression assigned to a narrower or different essential type.
 * The cast is required to perform a conversion between an unsigned integer and an enum type with many values.
 *
 * @section [global]
 * Violates MISRA 2012 Advisory Rule 10.5, Impermissible cast; cannot cast from 'essentially Boolean'
 * to 'essentially unsigned'. This is required by the conversion of a bool into a bit.
 * Impermissible cast; cannot cast from 'essentially unsigned' to 'essentially enum<i>'.
 * This is required by the conversion of a bitfield of a register into a enum.
 *
 * @section [global]
 * Violates MISRA 2012 Advisory Rule 11.4, Conversion between a pointer and integer type.
 * The cast is required to initialize a pointer with an unsigned long define, representing an address.
 * This conversion is required because the converted values are the addresses used in DMA transfer.
 *
 * @section [global]
 * Violates MISRA 2012 Required Rule 11.6, Cast from pointer to unsigned long, cast from unsigned long to pointer
 * and cast from unsigned int to pointer. The cast is required to perform a conversion between a pointer
 * and an unsigned long define, representing an address or vice versa. The cast is required to initialize a DMA
 * transfer. The converted value is the address of a buffer.
 *
 * @section [global]
 * Violates MISRA 2012 Required Rule 11.8, Attempt to cast away const/volatile from a pointer or reference.
 * The cast is required to initialize a DMA transfer. The converted value is the address of a register.
 *
 * @section [global]
 * Violates MISRA 2012 Advisory Rule 15.5, Return statement before end of function.
 * The return statement before end of function is used for simpler code
 * structure and better readability.
 */

#include "fsl_lpi2c_driver.h"
#include "fsl_interrupt_manager.h"
#include "fsl_lpi2c_hal.h"
#include "fsl_clock_manager.h"

/*******************************************************************************
 * Variables
 ******************************************************************************/

/*! @cond DRIVER_INTERNAL_USE_ONLY */

    /* Constraints used for baud rate computation */
#define CLKHI_MIN_VALUE 1U
#define CLKLO_MIN_VALUE 3U
#define CLKHI_MAX_VALUE (((1U << LPI2C_MCCR0_CLKHI_WIDTH) - 1U) >> 1U)
#define CLKLO_MAX_VALUE (CLKHI_MAX_VALUE << 1U)
#define DATAVD_MIN_VALUE 1U
#define SETHOLD_MIN_VALUE 2U

/* Table of base addresses for LPI2C instances. */
LPI2C_Type * const g_lpi2cBase[LPI2C_INSTANCE_COUNT] = LPI2C_BASE_PTRS;

/* Pointer to runtime state structure.*/
lpi2c_master_state_t* g_lpi2cMasterStatePtr[LPI2C_INSTANCE_COUNT] = {NULL};
lpi2c_slave_state_t* g_lpi2cSlaveStatePtr[LPI2C_INSTANCE_COUNT] = {NULL};

/* Table for lpi2c IRQ numbers */
const IRQn_Type g_lpi2cMasterIrqId[LPI2C_INSTANCE_COUNT] = LPI2C_MASTER_IRQS;
const IRQn_Type g_lpi2cSlaveIrqId[LPI2C_INSTANCE_COUNT] = LPI2C_SLAVE_IRQS;

/* PCC clock sources, for getting the input clock frequency */
const clock_names_t g_lpi2cClock[LPI2C_INSTANCE_COUNT] = {PCC_LPI2C0_CLOCK};

/* DMA  rx sources*/
const dma_request_source_t lpi2c_rxDmaSource[LPI2C_INSTANCE_COUNT] = {EDMA_REQ_LPI2C0_RX};
/* DMA  rx sources*/
const dma_request_source_t lpi2c_txDmaSource[LPI2C_INSTANCE_COUNT] = {EDMA_REQ_LPI2C0_TX};

/* Callback for master DMA transfer done.*/
static void LPI2C_DRV_MasterCompleteDMATransfer(void* parameter, edma_chn_status_t status);

/*! @brief Direction of a LPI2C transfer - transmit or receive. */
typedef enum
{
   LPI2C_TX_REQ = 0,    /*!< The driver will perform an I2C transmit transfer */
   LPI2C_RX_REQ = 1,    /*!< The driver will perform an I2C receive transfer */
} lpi2c_transfer_direction_t;

/*!
 * @brief DMA internal parameters structure
 *
 * This structure is used in DMA transfers. It contains different
 * variables required for setting and maintaining a DMA transfer.
 */
typedef struct
{
    /*! @cond DRIVER_INTERNAL_USE_ONLY */
    edma_chn_state_t dmaChannel;                    /* Structure for the eDMA channel state */
    edma_transfer_type_t dmaTransferType;           /* Type for the DMA transfer */
    uint32_t i2cDataRegAddr;                        /* An i2c data register address */
    uint8_t *bufferTransfer;                        /* Buffer used for transfer */
    uint32_t transferSize;                          /* Size of the data to be transfered */
    lpi2c_transfer_direction_t transferDirection;   /* Tells if the driver will make a receive or transmit DMA transfer */
    /*! @endcond */
} lpi2c_dma_transfer_params_t;


/*******************************************************************************
 * Private Functions
 ******************************************************************************/


/*FUNCTION**********************************************************************
 *
 * Function Name : LPI2C_DRV_MasterCmdQueueEmpty
 * Description   : checks if there are any commands in the master software queue
 *
 *END**************************************************************************/
static inline bool LPI2C_DRV_MasterCmdQueueEmpty(const lpi2c_master_state_t * master)
{
#ifdef DEV_ERROR_DETECT
    DEV_ASSERT(master != NULL);
#endif

    return (master->cmdQueue.writeIdx == master->cmdQueue.readIdx);
}


/*FUNCTION**********************************************************************
 *
 * Function Name : LPI2C_DRV_MasterResetQueue
 * Description   : resets the master software queue
 *
 *END**************************************************************************/
static inline void LPI2C_DRV_MasterResetQueue(lpi2c_master_state_t * master)
{
#ifdef DEV_ERROR_DETECT
    DEV_ASSERT(master != NULL);
#endif

    master->cmdQueue.readIdx = 0U;
    master->cmdQueue.writeIdx = 0U;
}


/*FUNCTION**********************************************************************
 *
 * Function Name : LPI2C_DRV_MasterQueueCmd
 * Description   : queues a command in the hardware FIFO or in the master software queue
 *
 *END**************************************************************************/
static inline void LPI2C_DRV_MasterQueueCmd(LPI2C_Type *baseAddr,
                                            lpi2c_master_state_t * master,
                                            lpi2c_master_command_t cmd,
                                            uint8_t data)
{
#ifdef DEV_ERROR_DETECT
    DEV_ASSERT(master != NULL);
    DEV_ASSERT(baseAddr != NULL);
#endif

    uint8_t txFIFOCount = LPI2C_HAL_MasterGetTxFIFOCount(baseAddr);
    uint16_t txFIFOSize = LPI2C_HAL_MasterGetTxFIFOSize(baseAddr);

    /* Check if there is room in the hardware FIFO */
    if (txFIFOCount < (uint8_t)txFIFOSize)
    {
        LPI2C_HAL_MasterTransmitCmd(baseAddr, cmd, data);
    }
    else
    {
        /* Hardware FIFO full, use software FIFO */
#ifdef DEV_ERROR_DETECT
        DEV_ASSERT(master->cmdQueue.writeIdx < LPI2C_MASTER_CMD_QUEUE_SIZE);
#endif
        master->cmdQueue.cmd[master->cmdQueue.writeIdx] = cmd;
        master->cmdQueue.data[master->cmdQueue.writeIdx] = data;
        master->cmdQueue.writeIdx++;
    }
}


/*FUNCTION**********************************************************************
 *
 * Function Name : LPI2C_DRV_MasterSendQueuedCmd
 * Description   : transfers commands from the master software queue to the hardware FIFO
 *
 *END**************************************************************************/
static inline void LPI2C_DRV_MasterSendQueuedCmd(LPI2C_Type *baseAddr, lpi2c_master_state_t * master)
{
#ifdef DEV_ERROR_DETECT
    DEV_ASSERT(master != NULL);
    DEV_ASSERT(baseAddr != NULL);
#endif

    uint8_t txFIFOCount = LPI2C_HAL_MasterGetTxFIFOCount(baseAddr);
    uint16_t txFifoSize = LPI2C_HAL_MasterGetTxFIFOSize(baseAddr);

    while ((!LPI2C_DRV_MasterCmdQueueEmpty(master)) && (txFIFOCount < (uint8_t)txFifoSize))
    {
        LPI2C_HAL_MasterTransmitCmd(baseAddr,
                                    master->cmdQueue.cmd[master->cmdQueue.readIdx],
                                    master->cmdQueue.data[master->cmdQueue.readIdx]);
        master->cmdQueue.readIdx++;

        txFIFOCount = LPI2C_HAL_MasterGetTxFIFOCount(baseAddr);
    }

    if (LPI2C_DRV_MasterCmdQueueEmpty(master))
    {
        /* Reset queue */
        LPI2C_DRV_MasterResetQueue(master);
    }
}


/*FUNCTION**********************************************************************
 *
 * Function Name : LPI2C_DRV_MasterSendAddress
 * Description   : send start event and slave address
 *                 parameter receive specifies the direction of the transfer
 *
 *END**************************************************************************/
static void LPI2C_DRV_MasterSendAddress(LPI2C_Type *baseAddr,
                                        lpi2c_master_state_t * master,
                                        bool receive)
{
    uint8_t addrByte;
    lpi2c_master_command_t startCommand;

#ifdef DEV_ERROR_DETECT
    DEV_ASSERT(master != NULL);
    DEV_ASSERT(baseAddr != NULL);
#endif

    if ((master->operatingMode == LPI2C_HIGHSPEED_MODE) && (master->highSpeedInProgress == false))
    {
        /* Initiating High-speed mode - send master code first */
        LPI2C_DRV_MasterQueueCmd(baseAddr, master, LPI2C_MASTER_COMMAND_START_NACK, master->masterCode);
        master->highSpeedInProgress = true;
    }

    if (master->highSpeedInProgress == true)
    {
        /* Use high-speed settings after start event in High Speed mode */
        startCommand = LPI2C_MASTER_COMMAND_START_HS;
    }
    else
    {
        /* Normal START command */
        startCommand = LPI2C_MASTER_COMMAND_START;
    }

    if (master->is10bitAddr)
    {
        /* 10-bit addressing */
        /* First address byte: 1111 0XXD, where XX are bits 10 and 9 of address, and D = 0(transmit) */
        addrByte = (uint8_t)(0xF0U + ((master->slaveAddress >> 7U) & 0x6U) + (uint8_t)0U);
        LPI2C_DRV_MasterQueueCmd(baseAddr, master, startCommand, addrByte);
        /* Second address byte: Remaining 8 bits of address */
        addrByte = (uint8_t)(master->slaveAddress & 0xFFU);
        LPI2C_DRV_MasterQueueCmd(baseAddr, master, LPI2C_MASTER_COMMAND_TRANSMIT, addrByte);
        if (receive == true)
        {
            /* Receiving from 10-bit slave - must send repeated start and resend first address byte */
            /* First address byte: 1111 0XXD, where XX are bits 10 and 9 of address, and D = 1 (receive) */
            addrByte = (uint8_t)(0xF0U + ((master->slaveAddress >> 7U) & 0x6U) + (uint8_t)1U);
            LPI2C_DRV_MasterQueueCmd(baseAddr, master, startCommand, addrByte);
        }
    }
    else
    {
        /* 7-bit addressing */
        /* Address byte: slave 7-bit address + D = 0(transmit) or 1 (receive) */
        addrByte = (uint8_t)((master->slaveAddress << 1U) + (uint8_t)receive);
        LPI2C_DRV_MasterQueueCmd(baseAddr, master, startCommand, addrByte);
    }
}


/*FUNCTION**********************************************************************
 *
 * Function Name : LPI2C_DRV_MasterQueueData
 * Description   : queues transmit data in the LPI2C tx fifo until it is full
 *
 *END**************************************************************************/
static void LPI2C_DRV_MasterQueueData(LPI2C_Type *baseAddr,
                                      lpi2c_master_state_t * master)
{
#ifdef DEV_ERROR_DETECT
    DEV_ASSERT(master != NULL);
    DEV_ASSERT(baseAddr != NULL);
#endif

    uint8_t txFIFOCount = LPI2C_HAL_MasterGetTxFIFOCount(baseAddr);
    uint16_t txFifoSize = LPI2C_HAL_MasterGetTxFIFOSize(baseAddr);

    /* Don't queue any data if there are commands in the software queue */
    if (LPI2C_DRV_MasterCmdQueueEmpty(master))
    {
        while ((master->txSize > 0U) && (txFIFOCount < (uint8_t)txFifoSize))
        {
            LPI2C_HAL_MasterTransmitCmd(baseAddr, LPI2C_MASTER_COMMAND_TRANSMIT, master->txBuff[0U]);
            master->txBuff++;
            master->txSize--;

            txFIFOCount = LPI2C_HAL_MasterGetTxFIFOCount(baseAddr);
        }
    }
}


/*FUNCTION**********************************************************************
 *
 * Function Name : LPI2C_DRV_MasterEndTransfer
 * Description   : ends current transmission or reception
 *
 *END**************************************************************************/
static void LPI2C_DRV_MasterEndTransfer(LPI2C_Type *baseAddr,
                                        lpi2c_master_state_t * master,
                                        bool sendStop,
                                        bool resetFIFO)
{
#ifdef DEV_ERROR_DETECT
    DEV_ASSERT(master != NULL);
    DEV_ASSERT(baseAddr != NULL);
#endif

    /* Disable all events */
    LPI2C_HAL_MasterSetInt(baseAddr, LPI2C_HAL_MASTER_FIFO_ERROR_INT |
                                     LPI2C_HAL_MASTER_ARBITRATION_LOST_INT |
                                     LPI2C_HAL_MASTER_NACK_DETECT_INT |
                                     LPI2C_HAL_MASTER_TRANSMIT_DATA_INT |
                                     LPI2C_HAL_MASTER_RECEIVE_DATA_INT,
                           false);

    if (resetFIFO == true)
    {
        /* Reset FIFOs if requested */
        LPI2C_HAL_MasterTxFIFOResetCmd(baseAddr);
        LPI2C_HAL_MasterRxFIFOResetCmd(baseAddr);
    }

    /* Queue STOP command if requested */
    if (sendStop == true)
    {
        LPI2C_HAL_MasterTransmitCmd(baseAddr, LPI2C_MASTER_COMMAND_STOP, 0U);
        master->highSpeedInProgress = false; /* High-speed transfers end at STOP condition */
    }

    if (master->transferType == LPI2C_USING_DMA)
    {
        /*
         * Disable LPI2C DMA request.
         * Release DMA channel that was used in current transfer.
         */
        if (master->rxSize != (uint16_t)0)
        {
            (void)LPI2C_HAL_MasterSetRxDMA(baseAddr, false);
        }
        else
        {
            (void)LPI2C_HAL_MasterSetTxDMA(baseAddr, false);
        }
        (void)EDMA_DRV_ReleaseChannel(&(master->dmaChannel));
    }

    master->txBuff = NULL;
    master->txSize = 0;
    master->rxBuff = NULL;
    master->rxSize = 0;
    master->i2cIdle = true;
}


/*FUNCTION**********************************************************************
 *
 * Function Name : LPI2C_DRV_SlaveEndTransfer
 * Description   : ends current transmission or reception
 *
 *END**************************************************************************/
static void LPI2C_DRV_SlaveEndTransfer(LPI2C_Type *baseAddr,
                                    lpi2c_slave_state_t *slave)
{
#ifdef DEV_ERROR_DETECT
    DEV_ASSERT(slave != NULL);
    DEV_ASSERT(baseAddr != NULL);
#endif

    /* Deactivate events */
    LPI2C_HAL_SlaveSetInt(baseAddr, LPI2C_HAL_SLAVE_BIT_ERROR_INT |
                                    LPI2C_HAL_SLAVE_FIFO_ERROR_INT |
                                    LPI2C_HAL_SLAVE_STOP_DETECT_INT |
                                    LPI2C_HAL_SLAVE_REPEATED_START_INT |
                                    LPI2C_HAL_SLAVE_ADDRESS_VALID_INT |
                                    LPI2C_HAL_SLAVE_RECEIVE_DATA_INT |
                                    LPI2C_HAL_SLAVE_TRANSMIT_DATA_INT,
                          false);

    /* For DMA we must release the channel used and disable the DMA request. */
    if (slave->transferType == LPI2C_USING_DMA)
    {
        if (slave->rxSize != (uint16_t)0)
        {
            (void)LPI2C_HAL_SlaveSetRxDMA(baseAddr, false);
            (void)EDMA_DRV_ReleaseChannel(&(slave->rxDMAChannel));
        }
        else
        {
            (void)LPI2C_HAL_SlaveSetTxDMA(baseAddr, false);
            (void)EDMA_DRV_ReleaseChannel(&(slave->txDMAChannel));
        }
    }

    /* Disable LPI2C slave */
    LPI2C_HAL_SlaveSetEnable(baseAddr, false);

    slave->isTransferInProgress = false;
    slave->rxBuff = NULL;
    slave->rxSize = 0U;
    slave->txBuff = NULL;
    slave->rxSize = 0U;
}


/*FUNCTION**********************************************************************
 *
 * Function Name : LPI2C_DRV_MasterSetOperatingMode
 * Description   : sets the operating mode of the I2C master
 *
 *END**************************************************************************/
static void LPI2C_DRV_MasterSetOperatingMode(uint32_t instance, lpi2c_mode_t operatingMode)
{
    LPI2C_Type *baseAddr;
    lpi2c_master_state_t * master;

    baseAddr = g_lpi2cBase[instance];
    master = g_lpi2cMasterStatePtr[instance];
#ifdef DEV_ERROR_DETECT
    DEV_ASSERT(master != NULL);
#endif

    if (operatingMode == LPI2C_ULTRAFAST_MODE)
    {
        LPI2C_HAL_MasterSetPinConfig(baseAddr, LPI2C_CFG_2PIN_OUTPUT_ONLY);
        LPI2C_HAL_MasterSetNACKConfig(baseAddr, LPI2C_NACK_IGNORE);
    }
    else
    {
        LPI2C_HAL_MasterSetPinConfig(baseAddr, LPI2C_CFG_2PIN_OPEN_DRAIN);
        LPI2C_HAL_MasterSetNACKConfig(baseAddr, LPI2C_NACK_RECEIVE);
    }

    master->operatingMode = operatingMode;
}


/*FUNCTION**********************************************************************
 *
 * Function Name : LPI2C_DRV_SlaveSetOperatingMode
 * Description   : sets the operating mode of the I2C slave
 *
 *END**************************************************************************/
static void LPI2C_DRV_SlaveSetOperatingMode(uint32_t instance, lpi2c_mode_t operatingMode)
{
    LPI2C_Type *baseAddr;
    lpi2c_slave_state_t * slave;

    baseAddr = g_lpi2cBase[instance];
    slave = g_lpi2cSlaveStatePtr[instance];
#ifdef DEV_ERROR_DETECT
    DEV_ASSERT(slave != NULL);
#endif

    if (operatingMode == LPI2C_ULTRAFAST_MODE)
    {
        LPI2C_HAL_SlaveSetIgnoreNACK(baseAddr, LPI2C_SLAVE_NACK_CONTINUE_TRANSFER);
        LPI2C_HAL_SlaveSetTransmitNACK(baseAddr, LPI2C_SLAVE_TRANSMIT_NACK);
        /* Disable all clock stretching in ultra-fast mode */
        LPI2C_HAL_SlaveSetACKStall(baseAddr, false);
        LPI2C_HAL_SlaveSetTXDStall(baseAddr, false);
        LPI2C_HAL_SlaveSetRXStall(baseAddr, false);
        LPI2C_HAL_SlaveSetAddrStall(baseAddr, false);
    }
    else
    {
        LPI2C_HAL_SlaveSetIgnoreNACK(baseAddr, LPI2C_SLAVE_NACK_END_TRANSFER);
        LPI2C_HAL_SlaveSetTransmitNACK(baseAddr, LPI2C_SLAVE_TRANSMIT_ACK);
        /* Enable clock stretching except ACKSTALL (we don't need to send ACK/NACK manually) */
        LPI2C_HAL_SlaveSetACKStall(baseAddr, false);
        LPI2C_HAL_SlaveSetTXDStall(baseAddr, true);
        LPI2C_HAL_SlaveSetRXStall(baseAddr, true);
        LPI2C_HAL_SlaveSetAddrStall(baseAddr, true);
    }

    if (operatingMode == LPI2C_HIGHSPEED_MODE)
    {
        /* Enable detection of the High-speed Mode master code */
        LPI2C_HAL_SlaveSetHighSpeedModeDetect(baseAddr, true);
    }
    else
    {
        /* Disable detection of the High-speed Mode master code */
        LPI2C_HAL_SlaveSetHighSpeedModeDetect(baseAddr, false);
    }

    slave->operatingMode = operatingMode;
}


/*FUNCTION**********************************************************************
 *
 * Function Name : LPI2C_DRV_ConfigureDmaTransfer
 * Description   : configures the DMA transfer
 *
 *END**************************************************************************/
static void LPI2C_DRV_ConfigureDmaTransfer(const lpi2c_dma_transfer_params_t *dmaTransParams)
{
    DMA_Type *baseDma = g_edmaBase[LPSPI_DMA_INSTANCE];

    /* Configure DMA channel */
    if (dmaTransParams->transferDirection == LPI2C_TX_REQ)
    {
        (void)EDMA_DRV_ConfigSingleBlockTransfer(&(dmaTransParams->dmaChannel), dmaTransParams->dmaTransferType, (uint32_t)dmaTransParams->bufferTransfer,
                                                (uint32_t)dmaTransParams->i2cDataRegAddr, EDMA_TRANSFER_SIZE_1B, (uint32_t)1U);
    }
    else
    {
        (void)EDMA_DRV_ConfigSingleBlockTransfer(&(dmaTransParams->dmaChannel), dmaTransParams->dmaTransferType, (uint32_t)dmaTransParams->i2cDataRegAddr,
                                                (uint32_t)dmaTransParams->bufferTransfer, EDMA_TRANSFER_SIZE_1B, (uint32_t)1U);
    }
    (void)EDMA_HAL_TCDSetMajorCount(baseDma, dmaTransParams->dmaChannel.channel, (uint32_t)dmaTransParams->transferSize);

    /* Disable DMA requests for channel when transfer is done. */
    (void)EDMA_HAL_TCDSetDisableDmaRequestAfterTCDDoneCmd(baseDma, dmaTransParams->dmaChannel.channel, true);
}


/*FUNCTION**********************************************************************
 *
 * Function Name : LPI2C_DRV_MasterStartDmaTransfer
 * Description   : starts the DMA transfer for master
 *
 *END**************************************************************************/
static void LPI2C_DRV_MasterStartDmaTransfer(uint32_t instance)
{
    LPI2C_Type *baseAddr = g_lpi2cBase[instance];
    lpi2c_master_state_t *master = g_lpi2cMasterStatePtr[instance];
    lpi2c_dma_transfer_params_t dmaTransParams;
    bool receive = false;

    dmaTransParams.dmaChannel = master->dmaChannel;
    if (master->txSize > (uint32_t)0U)
    {
        /* Configure watermarks for transmit DMA for master */
        uint32_t txBytes = LPI2C_HAL_MasterGetTxFIFOSize(baseAddr);
        if (txBytes > master->txSize)
        {
            txBytes = master->txSize;
        }
        LPI2C_HAL_MasterSetTxFIFOWatermark(baseAddr, txBytes - 1U);

        dmaTransParams.dmaTransferType = EDMA_TRANSFER_MEM2PERIPH;
        dmaTransParams.i2cDataRegAddr = (uint32_t)(&(baseAddr->MTDR));
        dmaTransParams.bufferTransfer = (uint8_t *)master->txBuff;
        dmaTransParams.transferDirection = LPI2C_TX_REQ;
        dmaTransParams.transferSize = master->txSize;
    }
    else
    {
        /* Configure watermarks for receive DMA for master */
        LPI2C_HAL_MasterSetRxFIFOWatermark(baseAddr, 0U);

        receive = true;

        dmaTransParams.dmaTransferType = EDMA_TRANSFER_PERIPH2MEM;
        dmaTransParams.i2cDataRegAddr = (uint32_t)(&(baseAddr->MRDR));
        dmaTransParams.bufferTransfer = master->rxBuff;
        dmaTransParams.transferDirection = LPI2C_RX_REQ;
        dmaTransParams.transferSize = master->rxSize;
    }

    (void)LPI2C_DRV_ConfigureDmaTransfer(&dmaTransParams);

    /* Call callback function when all the bytes were transfered. */
    (void)EDMA_DRV_InstallCallback(&(dmaTransParams.dmaChannel), (LPI2C_DRV_MasterCompleteDMATransfer), (void*)(instance));

    /* Start channel */
    (void)EDMA_DRV_StartChannel(&(dmaTransParams.dmaChannel));

    LPI2C_DRV_MasterSendAddress(baseAddr, master, receive);

    /* Enable transmit/receive DMA requests */
    master->dmaChannel = dmaTransParams.dmaChannel;
    if (master->txSize > (uint32_t)0U)
    {
        (void)LPI2C_HAL_MasterSetTxDMA(baseAddr, true);
    }
    else
    {
        LPI2C_DRV_MasterQueueCmd(baseAddr, master, LPI2C_MASTER_COMMAND_RECEIVE, master->rxSize - 1U);
        (void)LPI2C_HAL_MasterSetRxDMA(baseAddr, true);
    }
}


/*FUNCTION**********************************************************************
 *
 * Function Name : LPI2C_DRV_SlaveStartDmaTransfer
 * Description   : starts the DMA transfer for slave
 *
 *END**************************************************************************/
static void LPI2C_DRV_SlaveStartDmaTransfer(uint32_t instance)
{
    LPI2C_Type *baseAddr = g_lpi2cBase[instance];
    lpi2c_slave_state_t *slave = g_lpi2cSlaveStatePtr[instance];
    lpi2c_dma_transfer_params_t dmaTransParams;

    if (slave->txSize > (uint32_t)0U)
    {
        dmaTransParams.dmaChannel = slave->txDMAChannel;
        dmaTransParams.dmaTransferType = EDMA_TRANSFER_MEM2PERIPH;
        dmaTransParams.i2cDataRegAddr = (uint32_t)(&(baseAddr->STDR));
        dmaTransParams.bufferTransfer = (uint8_t*)slave->txBuff;
        dmaTransParams.transferDirection = LPI2C_TX_REQ;
        dmaTransParams.transferSize = slave->txSize;
    }
    else
    {
        dmaTransParams.dmaChannel = slave->rxDMAChannel;
        dmaTransParams.dmaTransferType = EDMA_TRANSFER_PERIPH2MEM;
        dmaTransParams.i2cDataRegAddr = (uint32_t)(&(baseAddr->SRDR));
        dmaTransParams.bufferTransfer = slave->rxBuff;
        dmaTransParams.transferDirection = LPI2C_RX_REQ;
        dmaTransParams.transferSize = slave->rxSize;
    }

    (void)LPI2C_DRV_ConfigureDmaTransfer(&dmaTransParams);

    /* Start channel */
    (void)EDMA_DRV_StartChannel(&(dmaTransParams.dmaChannel));

    /* Enable transmit/receive DMA requests */
    if (slave->txSize > (uint32_t)0U)
    {
        slave->txDMAChannel = dmaTransParams.dmaChannel;
        (void)LPI2C_HAL_SlaveSetTxDMA(baseAddr, true);
    }
    else
    {
        slave->rxDMAChannel = dmaTransParams.dmaChannel;
        (void)LPI2C_HAL_SlaveSetRxDMA(baseAddr, true);
    }
}


/*FUNCTION**********************************************************************
 *
 * Function Name : LPI2C_DRV_MasterCompleteDMATransfer
 * Description   : Finish up a transfer DMA for master. The main purpose of
 *                 this function is to create a function compatible with DMA
 *                 callback type
 *
 *END**************************************************************************/
static void LPI2C_DRV_MasterCompleteDMATransfer(void* parameter, edma_chn_status_t status)
{
    LPI2C_Type *baseAddr;
    lpi2c_master_state_t *master;

    (void)status;

    uint32_t instance = (uint32_t)parameter;

    baseAddr = g_lpi2cBase[instance];
    master = g_lpi2cMasterStatePtr[instance];

    LPI2C_DRV_MasterEndTransfer(baseAddr, master, master->sendStop, false);

    /* Signal transfer end for blocking transfers */
    if (master->blocking == true)
    {
        (void)OSIF_SemaPost(&(master->idleSemaphore));
    }

    master->status = LPI2C_STATUS_SUCCESS;
}


/*FUNCTION**********************************************************************
 *
 * Function Name : LPI2C_DRV_MasterWaitTransferEnd
 * Description   : waits for the end of a blocking transfer
 *
 *END**************************************************************************/
static lpi2c_status_t LPI2C_DRV_MasterWaitTransferEnd(uint32_t instance, uint32_t timeout)
{
    osif_status_t osifError = OSIF_STATUS_SUCCESS;
    LPI2C_Type *baseAddr;
    lpi2c_master_state_t *master;

    baseAddr = g_lpi2cBase[instance];
    master = g_lpi2cMasterStatePtr[instance];

    /* Wait for transfer to be completed by the IRQ */
    osifError = OSIF_SemaWait(&(master->idleSemaphore), timeout);

    if (osifError == OSIF_STATUS_TIMEOUT)
    {
        LPI2C_DRV_MasterEndTransfer(baseAddr, master, false, true);
        master->status = LPI2C_STATUS_TIMEOUT;
    }

    /* Blocking transfer is over */
    master->blocking = false;
    return master->status;
}


/*FUNCTION**********************************************************************
 *
 * Function Name : LPI2C_DRV_SlaveWaitTransferEnd
 * Description   : waits for the end of a blocking transfer
 *
 *END**************************************************************************/
static lpi2c_status_t LPI2C_DRV_SlaveWaitTransferEnd(uint32_t instance, uint32_t timeout)
{
    osif_status_t osifError = OSIF_STATUS_SUCCESS;
    LPI2C_Type *baseAddr;
    lpi2c_slave_state_t *slave;

    baseAddr = g_lpi2cBase[instance];
    slave = g_lpi2cSlaveStatePtr[instance];

    /* Wait for transfer to be completed by the IRQ */
    osifError = OSIF_SemaWait(&(slave->idleSemaphore), timeout);

    if (osifError == OSIF_STATUS_TIMEOUT)
    {
        LPI2C_DRV_SlaveEndTransfer(baseAddr, slave);
        slave->status = LPI2C_STATUS_TIMEOUT;
    }

    /* Blocking transfer is over */
    slave->blocking = false;
    return slave->status;
}


/*FUNCTION**********************************************************************
 *
 * Function Name : LPI2C_DRV_MasterHandleTransmitDataRequest
 * Description   : handle a transmit request for master
 *
 *END**************************************************************************/
static void LPI2C_DRV_MasterHandleTransmitDataRequest(uint32_t instance, LPI2C_Type *baseAddr, lpi2c_master_state_t *master)
{
    /* More data needed for transmission */
    if (!LPI2C_DRV_MasterCmdQueueEmpty(master))
    {
        /* If there are queued commands, send them */
        LPI2C_DRV_MasterSendQueuedCmd(baseAddr, master);
    }
    else if (master->txBuff != NULL)
    {
        /* A transmission is in progress */
        if (master->txSize == 0U)
        {
            /* There is no more data in buffer, the transmission is over */
            LPI2C_DRV_MasterEndTransfer(baseAddr, master, master->sendStop, false);

            /* Signal transfer end for blocking transfers */
            if (master->blocking == true)
            {
                (void)OSIF_SemaPost(&(master->idleSemaphore));
            }

            master->status = LPI2C_STATUS_SUCCESS;

            if (master->masterCallback != NULL)
            {
                master->masterCallback(instance, LPI2C_MASTER_EVENT_TX, master->callbackParam);
            }
        }
        else
        {
            /* Queue data bytes to fill tx fifo */
            LPI2C_DRV_MasterQueueData(baseAddr, master);
        }
    }
    else
    {
        /* No more commands and no transmission in progress - disable tx event */
        LPI2C_HAL_MasterSetInt(baseAddr, LPI2C_HAL_MASTER_TRANSMIT_DATA_INT, false);
    }
}


/*FUNCTION**********************************************************************
 *
 * Function Name : LPI2C_DRV_MasterHandleReceiveDataRequest
 * Description   : handle a receive request for master
 *
 *END**************************************************************************/
static void LPI2C_DRV_MasterHandleReceiveDataReadyEvent(uint32_t instance, LPI2C_Type *baseAddr, lpi2c_master_state_t *master)
{
    /* Received data ready */
#ifdef DEV_ERROR_DETECT
    DEV_ASSERT(master->rxBuff != NULL);
#endif
    /* Transfer received data to user buffer */
    while ((LPI2C_HAL_MasterGetRxFIFOCount(baseAddr) > 0U) && (master->rxSize > 0U))
    {
        master->rxBuff[0U] = LPI2C_HAL_MasterGetRxData(baseAddr);
        master->rxBuff++;
        master->rxSize--;
    }
    if (master->rxSize == 0U)
    {
        /* Done receiving */
        LPI2C_DRV_MasterEndTransfer(baseAddr, master, master->sendStop, false);

        /* Signal transfer end for blocking transfers */
        if (master->blocking == true)
        {
            (void)OSIF_SemaPost(&(master->idleSemaphore));
        }

        master->status = LPI2C_STATUS_SUCCESS;

        if (master->masterCallback != NULL)
        {
            master->masterCallback(instance, LPI2C_MASTER_EVENT_RX, master->callbackParam);
        }
    }
    else if (master->rxSize <= LPI2C_HAL_MasterGetRxFIFOWatermark(baseAddr))
    {
        /* Reduce rx watermark to receive the last few bytes */
        LPI2C_HAL_MasterSetRxFIFOWatermark(baseAddr, master->rxSize - 1U);
    }
    else
    {
        /* Continue reception */
    }
}


/*FUNCTION**********************************************************************
 *
 * Function Name : LPI2C_DRV_SlaveHandleAddressValidEvent
 * Description   : handle an address valid event for slave
 *
 *END**************************************************************************/
static void LPI2C_DRV_SlaveHandleAddressValidEvent(uint32_t instance, const LPI2C_Type *baseAddr, lpi2c_slave_state_t *slave)
{
    uint16_t receivedAddr;

    receivedAddr = LPI2C_HAL_SlaveGetReceivedAddr(baseAddr);
    if ((receivedAddr & 1U) != (uint16_t)0U)
    {
        /* Request from master to transmit data */
        if ((slave->slaveCallback != NULL) && slave->slaveListening)
        {
            slave->slaveCallback(instance, LPI2C_SLAVE_EVENT_TX_REQ, slave->callbackParam);
        }

        slave->txUnderrunWarning = false;

        if ((slave->transferType == LPI2C_USING_DMA) && slave->slaveListening)
        {
            (void)LPI2C_DRV_SlaveStartDmaTransfer(instance);
        }
    }
    else
    {
        /* Request from master to receive data */
        if ((slave->slaveCallback != NULL) && slave->slaveListening)
        {
            slave->slaveCallback(instance, LPI2C_SLAVE_EVENT_RX_REQ, slave->callbackParam);
        }

        if ((slave->transferType == LPI2C_USING_DMA) && slave->slaveListening)
        {
            (void)LPI2C_DRV_SlaveStartDmaTransfer(instance);
        }
    }

    slave->status = LPI2C_STATUS_BUSY;
}


/*FUNCTION**********************************************************************
 *
 * Function Name : LPI2C_DRV_SlaveHandleTransmitDataEvent
 * Description   : handle a transmit data event for slave
 *
 *END**************************************************************************/
static void LPI2C_DRV_SlaveHandleTransmitDataEvent(uint32_t instance, LPI2C_Type *baseAddr, lpi2c_slave_state_t *slave)
{
    if (slave->txUnderrunWarning == true)
    {
        /* Another Tx event after underflow warning means the dummy char was sent */
        slave->status = LPI2C_STATUS_SLAVE_TX_UNDERRUN;
    }

    if (slave->txSize == 0U)
    {
        /* Out of data, call callback to allow user to provide a new buffer */
        if (slave->slaveCallback != NULL)
        {
            slave->slaveCallback(instance, LPI2C_SLAVE_EVENT_TX_EMPTY, slave->callbackParam);
        }
    }

    if (slave->txSize == 0U)
    {
        /*
         * Still no data, record tx underflow event and send dummy char.
         * Special case after the last tx byte: the device will ask for more data
         * but the dummy char will not be sent if NACK and then STOP condition are
         * received from master. So only record a "warning" for now.
         */
        slave->txUnderrunWarning = true;
        LPI2C_HAL_SlaveTransmitData(baseAddr, (uint8_t)0xFFU);
    }
    else
    {
        LPI2C_HAL_SlaveTransmitData(baseAddr, slave->txBuff[0U]);
        slave->txBuff++;
        slave->txSize--;
    }
}


/*FUNCTION**********************************************************************
 *
 * Function Name : LPI2C_DRV_SlaveHandleReceiveDataEvent
 * Description   : handle a receive data event for slave
 *
 *END**************************************************************************/
static void LPI2C_DRV_SlaveHandleReceiveDataEvent(uint32_t instance, const LPI2C_Type *baseAddr, lpi2c_slave_state_t *slave)
{
    if (slave->rxSize == 0U)
    {
        /* No more room for data, call callback to allow user to provide a new buffer */
        if (slave->slaveCallback != NULL)
        {
            slave->slaveCallback(instance, LPI2C_SLAVE_EVENT_RX_FULL, slave->callbackParam);
        }
    }

    if (slave->rxSize == 0U)
    {
        /* Still no room for data, record rx overrun event and dummy read data */
        slave->status = LPI2C_STATUS_SLAVE_RX_OVERRUN;
        (void)LPI2C_HAL_SlaveGetData(baseAddr);
    }
    else
    {
        slave->rxBuff[0U] = LPI2C_HAL_SlaveGetData(baseAddr);
        slave->rxBuff++;
        slave->rxSize--;
    }
}

/*! @endcond */

/*******************************************************************************
 * Code
 ******************************************************************************/

/*FUNCTION**********************************************************************
 *
 * Function Name : LPI2C_DRV_MasterInit
 * Description   : initialize the I2C master mode driver
 *
 * Implements : LPI2C_DRV_MasterInit_Activity
 *END**************************************************************************/
lpi2c_status_t LPI2C_DRV_MasterInit(uint32_t instance,
                                    const lpi2c_master_user_config_t * userConfigPtr,
                                    lpi2c_master_state_t * master)
{
    LPI2C_Type *baseAddr;
    lpi2c_status_t retVal = LPI2C_STATUS_SUCCESS;
    clock_manager_error_code_t clkErr;
    uint32_t inputClock;

    if ((master == NULL) || (userConfigPtr == NULL) || (instance >= LPI2C_INSTANCE_COUNT))
    {
        return LPI2C_STATUS_INVALID_PARAMETER;
    }

    /* Check to see if the LPI2C master instance is already initialized */
    if (g_lpi2cMasterStatePtr[instance] != NULL)
    {
        return LPI2C_STATUS_INITIALIZED;
    }

    /* Check the protocol clock frequency */
    clkErr = CLOCK_SYS_GetFreq(g_lpi2cClock[instance], &inputClock);
    if ((clkErr != CLOCK_MANAGER_SUCCESS) || (inputClock == 0U))
    {
        /* No clock configured for this module */
        return LPI2C_STATUS_FAIL;
    }

    baseAddr = g_lpi2cBase[instance];
    g_lpi2cMasterStatePtr[instance] = master;

    /* Initialize driver status structure */
    master->rxBuff = NULL;
    master->rxSize = 0U;
    master->txBuff = NULL;
    master->txSize = 0U;
    master->status = LPI2C_STATUS_SUCCESS;
    master->i2cIdle = true;
    master->masterCode = userConfigPtr->masterCode;
    master->slaveAddress = userConfigPtr->slaveAddress;
    master->is10bitAddr = userConfigPtr->is10bitAddr;
    master->transferType = userConfigPtr->transferType;
    master->masterCallback = userConfigPtr->masterCallback;
    master->callbackParam = userConfigPtr->callbackParam;
    master->highSpeedInProgress = false;
    master->blocking = false;

    /* Initialize the semaphore */
    if (OSIF_SemaCreate(&(master->idleSemaphore), 0) == OSIF_STATUS_FAIL)
    {
        return LPI2C_STATUS_FAIL;
    }

    LPI2C_DRV_MasterResetQueue(master);

    /* Enable lpi2c interrupt */
    INT_SYS_EnableIRQ(g_lpi2cMasterIrqId[instance]);

    /* Initialize module */
    LPI2C_HAL_Init(baseAddr);

    /* Set baud rate */
    retVal = LPI2C_DRV_MasterSetBaudRate(instance, userConfigPtr->operatingMode, userConfigPtr->baudRate, userConfigPtr->baudRateHS);
    if(retVal != LPI2C_STATUS_SUCCESS)
    {
        /* Setting the baud rate failed */
        (void)OSIF_SemaDestroy(&(master->idleSemaphore));
        return LPI2C_STATUS_FAIL;
    }

    /* Set slave address */
    LPI2C_DRV_MasterSetSlaveAddr(instance, userConfigPtr->slaveAddress, userConfigPtr->is10bitAddr);

    /* Enable LPI2C master */
    LPI2C_HAL_MasterSetEnable(baseAddr, true);

    return retVal;
}


/*FUNCTION**********************************************************************
 *
 * Function Name : LPI2C_DRV_MasterDeinit
 * Description   : deinitialize the I2C master mode driver
 *
 * Implements : LPI2C_DRV_MasterDeinit_Activity
 *END**************************************************************************/
lpi2c_status_t LPI2C_DRV_MasterDeinit(uint32_t instance)
{
    LPI2C_Type *baseAddr;
    const lpi2c_master_state_t *master;

    if (instance >= LPI2C_INSTANCE_COUNT)
    {
        return LPI2C_STATUS_INVALID_PARAMETER;
    }

    baseAddr = g_lpi2cBase[instance];
    master = g_lpi2cMasterStatePtr[instance];
    if (master == NULL)
    {
        return LPI2C_STATUS_FAIL;
    }

    /* Destroy the semaphore */
    (void)OSIF_SemaDestroy(&(master->idleSemaphore));

    g_lpi2cMasterStatePtr[instance] = NULL;

    /* Disable master */
    LPI2C_HAL_MasterSetEnable(baseAddr, false);

    /* Disable i2c interrupt */
    INT_SYS_DisableIRQ(g_lpi2cMasterIrqId[instance]);

    return LPI2C_STATUS_SUCCESS;
}


/*FUNCTION**********************************************************************
 *
 * Function Name : LPI2C_DRV_MasterGetBaudRate
 * Description   : returns the currently configured baud rate
 *
 * Implements : LPI2C_DRV_MasterGetBaudRate_Activity
 *END**************************************************************************/
lpi2c_status_t LPI2C_DRV_MasterGetBaudRate(uint32_t instance, uint32_t *baudRate, uint32_t *baudRateHS)
{
    const LPI2C_Type *baseAddr;
    const lpi2c_master_state_t *master;
    clock_manager_error_code_t clkErr;
    uint32_t prescaler;
    uint32_t clkLo;
    uint32_t clkHi;
    uint32_t inputClock;

    if (instance >= LPI2C_INSTANCE_COUNT)
    {
        return LPI2C_STATUS_INVALID_PARAMETER;
    }

    baseAddr = g_lpi2cBase[instance];
    master = g_lpi2cMasterStatePtr[instance];
    if (master == NULL)
    {
        return LPI2C_STATUS_FAIL;
    }

    /* Get the protocol clock frequency */
    clkErr = CLOCK_SYS_GetFreq(g_lpi2cClock[instance], &inputClock);
    if ((clkErr != CLOCK_MANAGER_SUCCESS) || (inputClock == 0U))
    {
        /* No clock configured for this module */
        return LPI2C_STATUS_FAIL;
    }

    /* Ignoring the glitch filter, the baud rate formula is:
            SCL_freq = Input_freq / (2^PRESCALER * (CLKLO + CLKHI + 2))
    */
    prescaler = LPI2C_HAL_MasterGetPrescaler(baseAddr);
    clkHi = LPI2C_HAL_MasterGetClockHighPeriod(baseAddr);
    clkLo = LPI2C_HAL_MasterGetClockLowPeriod(baseAddr);

    *baudRate = inputClock / (((uint32_t)1U << prescaler) * (clkLo + clkHi + (uint32_t)2U));

    if (master->operatingMode == LPI2C_HIGHSPEED_MODE)
    {
        clkHi = LPI2C_HAL_MasterGetClockHighPeriodHS(baseAddr);
        clkLo = LPI2C_HAL_MasterGetClockLowPeriodHS(baseAddr);

        *baudRateHS = inputClock / (((uint32_t)1U << prescaler) * (clkLo + clkHi + (uint32_t)2U));
    }

    return LPI2C_STATUS_SUCCESS;
}


/*FUNCTION**********************************************************************
 *
 * Function Name : LPI2C_DRV_MasterSetBaudRate
 * Description   : set the baud rate for any subsequent I2C communication
 *
 * Implements : LPI2C_DRV_MasterSetBaudRate_Activity
 *END**************************************************************************/
lpi2c_status_t LPI2C_DRV_MasterSetBaudRate(uint32_t instance,
                                           const lpi2c_mode_t operatingMode,
                                           const uint32_t baudRate,
                                           const uint32_t baudRateHS)
{
    LPI2C_Type *baseAddr;
    const lpi2c_master_state_t * master;
    clock_manager_error_code_t clkErr;
    uint32_t inputClock;
    uint32_t minPrescaler;
    uint32_t prescaler;
    uint32_t clkTotal;
    uint32_t clkLo;
    uint32_t clkHi;
    uint32_t setHold;
    uint32_t dataVd;

    if (instance >= LPI2C_INSTANCE_COUNT)
    {
        return LPI2C_STATUS_INVALID_PARAMETER;
    }

    baseAddr = g_lpi2cBase[instance];
    master = g_lpi2cMasterStatePtr[instance];
    if (master == NULL)
    {
        return LPI2C_STATUS_FAIL;
    }

    /* Check if driver is busy */
    if (!master->i2cIdle)
    {
        return LPI2C_STATUS_BUSY;
    }

    /* Get the protocol clock frequency */
    clkErr = CLOCK_SYS_GetFreq(g_lpi2cClock[instance], &inputClock);
    if ((clkErr != CLOCK_MANAGER_SUCCESS) || (inputClock == 0U))
    {
        /* No clock configured for this module */
        return LPI2C_STATUS_FAIL;
    }

    /* Disable master */
    LPI2C_HAL_MasterSetEnable(baseAddr, false);

    /* Ignoring the glitch filter, the baud rate formula is:
            SCL_freq = Input_freq / (2^PRESCALER * (CLKLO + CLKHI + 2))
            Assume CLKLO = 2*CLKHI, SETHOLD = CLKHI, DATAVD = CLKHI/2
    */
    /* Compute minimum prescaler for which CLKLO and CLKHI values are in valid range. Always round up. */
    minPrescaler = ((inputClock - 1U) / (baudRate * (CLKHI_MAX_VALUE + CLKLO_MAX_VALUE + 2U))) + (uint32_t)1U;
    for (prescaler = 0U; prescaler < 7U; prescaler++)
    {
        if (((uint32_t)1U << prescaler) >= minPrescaler)
        {
            break;
        }
    }

    /* Compute CLKLO and CLKHI values for this prescaler. Round to nearest integer. */
    clkTotal = (inputClock + ((baudRate << prescaler) >> 1U)) / (baudRate << prescaler);
    if (clkTotal > (CLKHI_MAX_VALUE + CLKLO_MAX_VALUE + 2U))
    {
        clkTotal = (CLKHI_MAX_VALUE + CLKLO_MAX_VALUE + 2U);
    }

    /*
     * If we try to compute clk high and low values using clkTotal equal with 0
     * (this is the case when the baudrate is 0), we will get negative values for
     * them, so we set them to 0 for this case.
     */
    if (clkTotal == 0U)
    {
        clkHi = 0U;
        clkLo = 0U;
    }
    else
    {
        clkHi = (clkTotal - 2U) / 3U;
        clkLo = clkTotal - 2U - clkHi;
    }

    if (clkHi < CLKHI_MIN_VALUE)
    {
        clkHi = CLKHI_MIN_VALUE;
    }
    if (clkLo < CLKLO_MIN_VALUE)
    {
        clkLo = CLKLO_MIN_VALUE;
    }

    /* Compute DATAVD and SETHOLD */
    setHold = clkHi;
    dataVd = clkHi >> 1U;
    if (setHold < SETHOLD_MIN_VALUE)
    {
        setHold = SETHOLD_MIN_VALUE;
    }
    if (dataVd < DATAVD_MIN_VALUE)
    {
        dataVd = DATAVD_MIN_VALUE;
    }

    /* Apply settings */
    LPI2C_HAL_MasterSetPrescaler(baseAddr, (lpi2c_master_prescaler_t)prescaler);
    LPI2C_HAL_MasterSetDataValidDelay(baseAddr, dataVd);
    LPI2C_HAL_MasterSetSetupHoldDelay(baseAddr, setHold);
    LPI2C_HAL_MasterSetClockHighPeriod(baseAddr, clkHi);
    LPI2C_HAL_MasterSetClockLowPeriod(baseAddr, clkLo);

    if (operatingMode == LPI2C_HIGHSPEED_MODE)
    {
        /* Compute settings for High-speed baud rate */
        /* Compute High-speed CLKLO and CLKHI values for the same prescaler. Round to nearest integer. */
        clkTotal = (inputClock + ((baudRateHS << prescaler) >> 1U)) / (baudRateHS << prescaler);
        if (clkTotal > (CLKHI_MAX_VALUE + CLKLO_MAX_VALUE + 2U))
        {
            clkTotal = (CLKHI_MAX_VALUE + CLKLO_MAX_VALUE + 2U);
        }

        clkHi = (clkTotal - 2U) / 3U;
        clkLo = clkTotal - 2U - clkHi;
        if (clkHi < CLKHI_MIN_VALUE)
        {
            clkHi = CLKHI_MIN_VALUE;
        }
        if (clkLo < CLKLO_MIN_VALUE)
        {
            clkLo = CLKLO_MIN_VALUE;
        }

        /* Compute High-speed DATAVD and SETHOLD */
        setHold = clkHi;
        dataVd = clkHi >> 1U;
        if (setHold < SETHOLD_MIN_VALUE)
        {
            setHold = SETHOLD_MIN_VALUE;
        }
        if (dataVd < DATAVD_MIN_VALUE)
        {
            dataVd = DATAVD_MIN_VALUE;
        }

        /* Apply High-speed settings */
        LPI2C_HAL_MasterSetDataValidDelayHS(baseAddr, dataVd);
        LPI2C_HAL_MasterSetSetupHoldDelayHS(baseAddr, setHold);
        LPI2C_HAL_MasterSetClockHighPeriodHS(baseAddr, clkHi);
        LPI2C_HAL_MasterSetClockLowPeriodHS(baseAddr, clkLo);
    }

    /* Perform other settings related to the chosen operating mode */
    LPI2C_DRV_MasterSetOperatingMode(instance, operatingMode);

    /* Re-enable master */
    LPI2C_HAL_MasterSetEnable(baseAddr, true);

    return LPI2C_STATUS_SUCCESS;
}


/*FUNCTION**********************************************************************
 *
 * Function Name : LPI2C_DRV_MasterSetSlaveAddr
 * Description   : set the slave address for any subsequent I2C communication
 *
 * Implements : LPI2C_DRV_MasterSetSlaveAddr_Activity
 *END**************************************************************************/
void LPI2C_DRV_MasterSetSlaveAddr(uint32_t instance, const uint16_t address, const bool is10bitAddr)
{
    lpi2c_master_state_t * master;

#ifdef DEV_ERROR_DETECT
    DEV_ASSERT(instance < LPI2C_INSTANCE_COUNT);
#endif

    master = g_lpi2cMasterStatePtr[instance];
#ifdef DEV_ERROR_DETECT
    DEV_ASSERT(master != NULL);
#endif

    master->slaveAddress = address;
    master->is10bitAddr = is10bitAddr;
}


/*FUNCTION**********************************************************************
 *
 * Function Name : LPI2C_DRV_MasterSendData
 * Description   : perform a non-blocking send transaction on the I2C bus
 *
 * Implements : LPI2C_DRV_MasterSendData_Activity
 *END**************************************************************************/
lpi2c_status_t LPI2C_DRV_MasterSendData(uint32_t instance,
                                            const uint8_t * txBuff,
                                            uint32_t txSize,
                                            bool sendStop)
{
    LPI2C_Type *baseAddr;
    lpi2c_master_state_t *master;
    lpi2c_status_t status;

#ifdef DEV_ERROR_DETECT
    DEV_ASSERT(instance < LPI2C_INSTANCE_COUNT);
#endif

    if ((txBuff == NULL) || (txSize == (uint32_t)0U))
    {
        return LPI2C_STATUS_INVALID_PARAMETER;
    }

    baseAddr = g_lpi2cBase[instance];
    master = g_lpi2cMasterStatePtr[instance];
    if (master == NULL)
    {
        return LPI2C_STATUS_FAIL;
    }

    /* Check if driver is busy */
    if (!master->i2cIdle)
    {
        return LPI2C_STATUS_BUSY;
    }

    /* Copy parameters to driver state structure */
    status = master->status;
    master->txBuff = txBuff;
    master->txSize = txSize;
    master->sendStop = sendStop;
    master->i2cIdle = false;
    master->status = LPI2C_STATUS_BUSY;

    if (master->transferType == LPI2C_USING_DMA)
    {
        uint8_t dmaChnTx;
        edma_status_t error;

        /* Request Tx DMA channel. */
        error = EDMA_DRV_RequestChannel((uint8_t)EDMA_ANY_CHANNEL, lpi2c_txDmaSource[instance],
                                        &(master->dmaChannel), &dmaChnTx);
        if (error != EDMA_STATUS_SUCCESS)
        {
            master->status = status;
            master->i2cIdle = true;
            return LPI2C_STATUS_FAIL;
        }

        LPI2C_HAL_MasterSetInt(baseAddr, LPI2C_HAL_MASTER_FIFO_ERROR_INT |
                                         LPI2C_HAL_MASTER_ARBITRATION_LOST_INT |
                                         LPI2C_HAL_MASTER_NACK_DETECT_INT,
                               true);

        LPI2C_DRV_MasterStartDmaTransfer(instance);
    }
    else
    {
        /* Initiate communication */
        LPI2C_DRV_MasterSendAddress(baseAddr, master, false);

        /* Queue data bytes to fill tx fifo */
        LPI2C_DRV_MasterQueueData(baseAddr, master);

        /* Set tx FIFO watermark */
        LPI2C_HAL_MasterSetTxFIFOWatermark(baseAddr, 0U);

        /* Enable relevant events */
        if (master->operatingMode == LPI2C_ULTRAFAST_MODE)
        {
            /* Do not enable NACK event reporting in ultra-fast mode */
            LPI2C_HAL_MasterSetInt(baseAddr, LPI2C_HAL_MASTER_FIFO_ERROR_INT |
                                             LPI2C_HAL_MASTER_ARBITRATION_LOST_INT |
                                             LPI2C_HAL_MASTER_TRANSMIT_DATA_INT,
                                   true);
        }
        else
        {
            LPI2C_HAL_MasterSetInt(baseAddr, LPI2C_HAL_MASTER_FIFO_ERROR_INT |
                                             LPI2C_HAL_MASTER_ARBITRATION_LOST_INT |
                                             LPI2C_HAL_MASTER_NACK_DETECT_INT |
                                             LPI2C_HAL_MASTER_TRANSMIT_DATA_INT,
                                   true);
        }
    }

    return LPI2C_STATUS_SUCCESS;
}


/*FUNCTION**********************************************************************
 *
 * Function Name : LPI2C_DRV_MasterSendDataBlocking
 * Description   : perform a blocking send transaction on the I2C bus
 *
 * Implements : LPI2C_DRV_MasterSendDataBlocking_Activity
 *END**************************************************************************/
lpi2c_status_t LPI2C_DRV_MasterSendDataBlocking(uint32_t instance,
                                    const uint8_t * txBuff,
                                    uint32_t txSize,
                                    bool sendStop,
                                    uint32_t timeout)
{
    lpi2c_status_t retVal = LPI2C_STATUS_SUCCESS;

#ifdef DEV_ERROR_DETECT
    DEV_ASSERT(instance < LPI2C_INSTANCE_COUNT);
#endif

    if ((txBuff == NULL) || (txSize == (uint32_t)0U))
    {
        return LPI2C_STATUS_INVALID_PARAMETER;
    }

    lpi2c_master_state_t *master = g_lpi2cMasterStatePtr[instance];
    if (master == NULL)
    {
        return LPI2C_STATUS_FAIL;
    }

    /* mark transfer as blocking */
    master->blocking = true;

    retVal = LPI2C_DRV_MasterSendData(instance, txBuff, txSize, sendStop);
    if (retVal != LPI2C_STATUS_SUCCESS)
    {
        master->blocking = false;
        return retVal;
    }

    /* Wait for transfer to end */
    return LPI2C_DRV_MasterWaitTransferEnd(instance, timeout);
}


/*FUNCTION**********************************************************************
 *
 * Function Name : LPI2C_DRV_MasterAbortTransferData
 * Description   : abort a non-blocking I2C Master transmission or reception
 *
 * Implements : LPI2C_DRV_MasterAbortTransferData_Activity
 *END**************************************************************************/
lpi2c_status_t LPI2C_DRV_MasterAbortTransferData(uint32_t instance)
{
    LPI2C_Type *baseAddr;
    lpi2c_master_state_t * master;

    if (instance >= LPI2C_INSTANCE_COUNT)
    {
        return LPI2C_STATUS_INVALID_PARAMETER;
    }

    baseAddr = g_lpi2cBase[instance];
    master = g_lpi2cMasterStatePtr[instance];
    if (master == NULL)
    {
        return LPI2C_STATUS_FAIL;
    }

    if (master->i2cIdle == true)
    {
        /* No transfer in progress */
        return LPI2C_STATUS_NO_TRANSFER;
    }

    if (master->rxBuff != NULL)
    {
        /* Aborting a reception not supported because hardware will continue the
           current command even if the FIFO is reset and this could last indefinitely */
        return LPI2C_STATUS_FAIL;
    }

    /* End transfer: force stop generation, reset FIFOs */
    master->status = LPI2C_STATUS_ABORTED;
    LPI2C_DRV_MasterEndTransfer(baseAddr, master, true, true);

    return LPI2C_STATUS_SUCCESS;
}


/*FUNCTION**********************************************************************
 *
 * Function Name : LPI2C_DRV_MasterReceiveData
 * Description   : perform a non-blocking receive transaction on the I2C bus
 *
 * Implements : LPI2C_DRV_MasterReceiveData_Activity
 *END**************************************************************************/
lpi2c_status_t LPI2C_DRV_MasterReceiveData(uint32_t  instance,
                                       uint8_t * rxBuff,
                                       uint32_t rxSize,
                                       bool sendStop)
{
    LPI2C_Type *baseAddr;
    lpi2c_master_state_t * master;
    uint8_t rxBytes;
    lpi2c_status_t status;

#ifdef DEV_ERROR_DETECT
    DEV_ASSERT(instance < LPI2C_INSTANCE_COUNT);
#endif

    if ((rxBuff == NULL) || (rxSize == 0U))
    {
        return LPI2C_STATUS_INVALID_PARAMETER;
    }

    baseAddr = g_lpi2cBase[instance];
    master = g_lpi2cMasterStatePtr[instance];
    if (master == NULL)
    {
        return LPI2C_STATUS_FAIL;
    }

    /* Check if driver is busy */
    if (!master->i2cIdle)
    {
        return LPI2C_STATUS_BUSY;
    }

    if (master->operatingMode == LPI2C_ULTRAFAST_MODE)
    {
        /* No reception possible in ultra-fast mode */
        return LPI2C_STATUS_FAIL;
    }

    /* Copy parameters to driver state structure */
    status = master->status;
    master->rxSize = rxSize;
    master->i2cIdle = false;
    master->sendStop = sendStop;
    master->rxBuff = rxBuff;
    master->status = LPI2C_STATUS_BUSY;

    if (master->transferType == LPI2C_USING_DMA)
    {
        uint8_t dmaChnRx;
        edma_status_t error;

        /* Request Rx DMA channel. */
        error = EDMA_DRV_RequestChannel((uint8_t)EDMA_ANY_CHANNEL, lpi2c_rxDmaSource[instance],
                                        &(master->dmaChannel), &dmaChnRx);
        if (error != EDMA_STATUS_SUCCESS)
        {
            master->status = status;
            master->i2cIdle = true;
            return LPI2C_STATUS_FAIL;
        }

        LPI2C_HAL_MasterSetInt(baseAddr, LPI2C_HAL_MASTER_FIFO_ERROR_INT |
                                         LPI2C_HAL_MASTER_ARBITRATION_LOST_INT |
                                         LPI2C_HAL_MASTER_NACK_DETECT_INT,
                               true);

        LPI2C_DRV_MasterStartDmaTransfer(instance);
    }
    else
    {
        /* Initiate communication */
        LPI2C_DRV_MasterSendAddress(baseAddr, master, true);
        /* Queue receive command for rxSize bytes */
        LPI2C_DRV_MasterQueueCmd(baseAddr, master, LPI2C_MASTER_COMMAND_RECEIVE, rxSize - 1U);

        /* Set rx FIFO watermark */
        rxBytes = LPI2C_HAL_MasterGetRxFIFOSize(baseAddr);
        if (rxBytes > rxSize)
        {
            rxBytes = rxSize;
        }
        LPI2C_HAL_MasterSetRxFIFOWatermark(baseAddr, rxBytes - 1U);

        /* Enable relevant events */
        if (!LPI2C_DRV_MasterCmdQueueEmpty(master))
        {
            /* Enable tx event too if there are commands in the software FIFO */
            LPI2C_HAL_MasterSetInt(baseAddr, LPI2C_HAL_MASTER_FIFO_ERROR_INT |
                                             LPI2C_HAL_MASTER_ARBITRATION_LOST_INT |
                                             LPI2C_HAL_MASTER_NACK_DETECT_INT |
                                             LPI2C_HAL_MASTER_TRANSMIT_DATA_INT |
                                             LPI2C_HAL_MASTER_RECEIVE_DATA_INT,
                                   true);
        }
        else
        {
            LPI2C_HAL_MasterSetInt(baseAddr, LPI2C_HAL_MASTER_FIFO_ERROR_INT |
                                             LPI2C_HAL_MASTER_ARBITRATION_LOST_INT |
                                             LPI2C_HAL_MASTER_NACK_DETECT_INT |
                                             LPI2C_HAL_MASTER_RECEIVE_DATA_INT,
                                   true);
        }
    }

    return LPI2C_STATUS_SUCCESS;
}


/*FUNCTION**********************************************************************
 *
 * Function Name : LPI2C_DRV_MasterReceiveDataBlocking
 * Description   : perform a blocking receive transaction on the I2C bus
 *
 * Implements : LPI2C_DRV_MasterReceiveDataBlocking_Activity
 *END**************************************************************************/
lpi2c_status_t LPI2C_DRV_MasterReceiveDataBlocking(uint32_t instance,
                                           uint8_t * rxBuff,
                                           uint32_t rxSize,
                                           bool sendStop,
                                           uint32_t timeout)
{
    lpi2c_status_t retVal = LPI2C_STATUS_SUCCESS;


#ifdef DEV_ERROR_DETECT
    DEV_ASSERT(instance < LPI2C_INSTANCE_COUNT);
#endif

    if ((rxBuff == NULL) || (rxSize == 0U))
    {
        return LPI2C_STATUS_INVALID_PARAMETER;
    }

    lpi2c_master_state_t *master = g_lpi2cMasterStatePtr[instance];
    if (master == NULL)
    {
        return LPI2C_STATUS_FAIL;
    }

    /* mark transfer as blocking */
    master->blocking = true;

    retVal = LPI2C_DRV_MasterReceiveData(instance, rxBuff, rxSize, sendStop);
    if (retVal != LPI2C_STATUS_SUCCESS)
    {
        master->blocking = false;
        return retVal;
    }

    /* Wait for transfer to end */
    return LPI2C_DRV_MasterWaitTransferEnd(instance, timeout);
}


/*FUNCTION**********************************************************************
 *
 * Function Name : LPI2C_DRV_MasterGetTransferStatus
 * Description   : return the current status of the I2C master transfer
 *
 * When performing an a-sync (non-blocking) transfer, the user can call this function
 * to ascertain the state of the current transfer. In addition, if the transfer is still
 * in progress, the user can get the number of words that should be receive.
 *
 * Implements : LPI2C_DRV_MasterGetTransferStatus_Activity
 *END**************************************************************************/
lpi2c_status_t LPI2C_DRV_MasterGetTransferStatus(uint32_t instance,
                                                uint32_t *bytesRemaining)
{
    const LPI2C_Type *baseAddr;
    const lpi2c_master_state_t * master;

    if (instance >= LPI2C_INSTANCE_COUNT)
    {
        return LPI2C_STATUS_INVALID_PARAMETER;
    }

    baseAddr = g_lpi2cBase[instance];
    master = g_lpi2cMasterStatePtr[instance];
    if (master == NULL)
    {
        return LPI2C_STATUS_FAIL;
    }

    if ((bytesRemaining != NULL) && (master->transferType == LPI2C_USING_INTERRUPTS))
    {
        if (master->txSize > 0U)
        {
            /* Send data */
            /* Remaining bytes = bytes in buffer + bytes in tx FIFO */
            *bytesRemaining = master->txSize + LPI2C_HAL_MasterGetTxFIFOCount(baseAddr);
        }
        else if (master->rxSize > 0U)
        {
            /* Receive data */
            /* Remaining bytes = free space in buffer - bytes in rx FIFO */
            *bytesRemaining = master->rxSize - LPI2C_HAL_MasterGetRxFIFOCount(baseAddr);
        }
        else
        {
            *bytesRemaining = 0U;
        }
    }

    return master->status;
}


/*FUNCTION**********************************************************************
 *
 * Function Name : LPI2C_DRV_MasterIRQHandler
 * Description   : handle non-blocking master operation when I2C interrupt occurs
 *
 *END**************************************************************************/
void LPI2C_DRV_MasterIRQHandler(uint32_t instance)
{
    LPI2C_Type *baseAddr;
    lpi2c_master_state_t * master;

#ifdef DEV_ERROR_DETECT
    DEV_ASSERT(instance < LPI2C_INSTANCE_COUNT);
#endif

    baseAddr = g_lpi2cBase[instance];
    master = g_lpi2cMasterStatePtr[instance];
#ifdef DEV_ERROR_DETECT
    DEV_ASSERT(master != NULL);
#endif

    /* Check which event caused the interrupt */
    if (LPI2C_HAL_MasterGetTransmitDataRequestEvent(baseAddr))
    {
        LPI2C_DRV_MasterHandleTransmitDataRequest(instance, baseAddr, master);
    }

    if (LPI2C_HAL_MasterGetReceiveDataReadyEvent(baseAddr))
    {
        LPI2C_DRV_MasterHandleReceiveDataReadyEvent(instance, baseAddr, master);
    }

    if (LPI2C_HAL_MasterGetFIFOErrorEvent(baseAddr))
    {
        /* FIFO error */
        LPI2C_HAL_MasterClearFIFOErrorEvent(baseAddr);

        /* High-speed transfers end at STOP condition */
        master->highSpeedInProgress = false;
        master->status = LPI2C_STATUS_FAIL;

        /* End transfer: no stop generation (the module will handle that by itself
           if needed), reset FIFOs */
        LPI2C_DRV_MasterEndTransfer(baseAddr, master, false, true);

        /* Signal transfer end for blocking transfers */
        if (master->blocking == true)
        {
            (void)OSIF_SemaPost(&(master->idleSemaphore));
        }

        if (master->masterCallback != NULL)
        {
            master->masterCallback(instance, LPI2C_MASTER_EVENT_FIFO_ERROR, master->callbackParam);
        }
    }

    if (LPI2C_HAL_MasterGetArbitrationLostEvent(baseAddr))
    {
        /* Arbitration lost */
        LPI2C_HAL_MasterClearArbitrationLostEvent(baseAddr);

        /* End transfer: no stop generation (the module will handle that by itself
           if needed), reset FIFOs */
        LPI2C_DRV_MasterEndTransfer(baseAddr, master, false, true);

        /* Signal transfer end for blocking transfers */
        if (master->blocking == true)
        {
            (void)OSIF_SemaPost(&(master->idleSemaphore));
        }

        master->status = LPI2C_STATUS_ARIBTRATION_LOST;

        if (master->masterCallback != NULL)
        {
            master->masterCallback(instance, LPI2C_MASTER_EVENT_ARBITRATION_LOST, master->callbackParam);
        }
    }

    if (LPI2C_HAL_MasterGetNACKDetectEvent(baseAddr))
    {
        /* Received NACK */
        LPI2C_HAL_MasterClearNACKDetectEvent(baseAddr);

        /* Ignore NACK in Ultra Fast mode */
        if (master->operatingMode != LPI2C_ULTRAFAST_MODE)
        {
            /* Signal transfer end for blocking transfers */
            if (master->blocking == true)
            {
                (void)OSIF_SemaPost(&(master->idleSemaphore));
            }

            /* High-speed transfers end at STOP condition */
            master->highSpeedInProgress = false;
            master->status = LPI2C_STATUS_RECEIVED_NACK;

            /* End transfer: no stop generation (the module will handle that by itself
               if needed), reset FIFOs */
            LPI2C_DRV_MasterEndTransfer(baseAddr, master, false, true);

            if (master->masterCallback != NULL)
            {
                master->masterCallback(instance, LPI2C_MASTER_EVENT_NACK, master->callbackParam);
            }
        }
    }
}


/*FUNCTION**********************************************************************
 *
 * Function Name : LPI2C_DRV_SlaveInit
 * Description   : initialize the I2C slave mode driver
 *
 * Implements : LPI2C_DRV_SlaveInit_Activity
 *END**************************************************************************/
lpi2c_status_t LPI2C_DRV_SlaveInit(uint32_t instance,
                                   const lpi2c_slave_user_config_t * userConfigPtr,
                                   lpi2c_slave_state_t * slave)
{
    LPI2C_Type *baseAddr;
    clock_manager_error_code_t clkErr;
    uint32_t inputClock;

    if ((userConfigPtr == NULL) || (slave == NULL) || (instance >= LPI2C_INSTANCE_COUNT))
    {
        return LPI2C_STATUS_INVALID_PARAMETER;
    }

    if (g_lpi2cSlaveStatePtr[instance] != NULL)
    {
        return LPI2C_STATUS_INITIALIZED;
    }

    /*
     * Check the protocol clock frequency.
     * LPI2C slave remains operational, even when the LPI2C functional
     * clock is disabled, so we don't need to check if inputClock is 0.
     */
    clkErr = CLOCK_SYS_GetFreq(g_lpi2cClock[instance], &inputClock);
    if (clkErr != CLOCK_MANAGER_SUCCESS)
    {
        /* No clock configured for this module */
        return LPI2C_STATUS_FAIL;
    }

    baseAddr = g_lpi2cBase[instance];
    g_lpi2cSlaveStatePtr[instance] = slave;

    /* Initialize driver status structure */
    slave->status = LPI2C_STATUS_SUCCESS;
    slave->slaveListening = userConfigPtr->slaveListening;
    slave->slaveCallback = userConfigPtr->slaveCallback;
    slave->callbackParam = userConfigPtr->callbackParam;
    slave->txBuff = NULL;
    slave->rxBuff = NULL;
    slave->txSize = 0U;
    slave->rxSize = 0U;
    slave->transferType = userConfigPtr->transferType;
    slave->isTransferInProgress = false;
    slave->blocking = false;

    /* Initialize the semaphore */
    if (OSIF_SemaCreate(&(slave->idleSemaphore), 0) == OSIF_STATUS_FAIL)
    {
        return LPI2C_STATUS_FAIL;
    }

    /* Enable lpi2c interrupt */
    INT_SYS_EnableIRQ(g_lpi2cSlaveIrqId[instance]);

    /* Initialize module */
    LPI2C_HAL_Init(baseAddr);

    /* Configure slave address */
    LPI2C_HAL_SlaveSetAddr0(baseAddr, userConfigPtr->slaveAddress);
    if (userConfigPtr->is10bitAddr)
    {
        LPI2C_HAL_SlaveSetAddrConfig(baseAddr, LPI2C_SLAVE_ADDR_MATCH_0_10BIT);
    }
    else
    {
        LPI2C_HAL_SlaveSetAddrConfig(baseAddr, LPI2C_SLAVE_ADDR_MATCH_0_7BIT);
    }

    /* Configure operating mode */
    LPI2C_DRV_SlaveSetOperatingMode(instance, userConfigPtr->operatingMode);

    if (userConfigPtr->slaveListening)
    {
        if (slave->transferType == LPI2C_USING_DMA)
        {
            uint8_t dmaChnRx, dmaChnTx;
            edma_status_t error;

            /* Request Rx DMA channel. */
            error = EDMA_DRV_RequestChannel((uint8_t)EDMA_ANY_CHANNEL, lpi2c_rxDmaSource[instance],
                                            &(slave->rxDMAChannel), &dmaChnRx);
            if (error != EDMA_STATUS_SUCCESS)
            {
                /* There are no free channels available for DMA transfer */
                (void)OSIF_SemaDestroy(&(slave->idleSemaphore));
                return LPI2C_STATUS_FAIL;
            }

            /* Request Tx DMA channel. */
            error = EDMA_DRV_RequestChannel((uint8_t)EDMA_ANY_CHANNEL, lpi2c_txDmaSource[instance],
                                            &(slave->txDMAChannel), &dmaChnTx);
            if (error != EDMA_STATUS_SUCCESS)
            {
                /* There are no free channels available for DMA transfer */
                (void)EDMA_DRV_ReleaseChannel(&(slave->rxDMAChannel));
                (void)OSIF_SemaDestroy(&(slave->idleSemaphore));
                return LPI2C_STATUS_FAIL;
            }

            /* Activate events */
            LPI2C_HAL_SlaveSetInt(baseAddr, LPI2C_HAL_SLAVE_BIT_ERROR_INT |
                                            LPI2C_HAL_SLAVE_FIFO_ERROR_INT |
                                            LPI2C_HAL_SLAVE_STOP_DETECT_INT |
                                            LPI2C_HAL_SLAVE_REPEATED_START_INT |
                                            LPI2C_HAL_SLAVE_ADDRESS_VALID_INT,
                                  true);
        }
        if (slave->transferType == LPI2C_USING_INTERRUPTS)
        {
            /* Activate events */
            LPI2C_HAL_SlaveSetInt(baseAddr, LPI2C_HAL_SLAVE_BIT_ERROR_INT |
                                            LPI2C_HAL_SLAVE_FIFO_ERROR_INT |
                                            LPI2C_HAL_SLAVE_STOP_DETECT_INT |
                                            LPI2C_HAL_SLAVE_REPEATED_START_INT |
                                            LPI2C_HAL_SLAVE_ADDRESS_VALID_INT |
                                            LPI2C_HAL_SLAVE_RECEIVE_DATA_INT |
                                            LPI2C_HAL_SLAVE_TRANSMIT_DATA_INT,
                                  true);
        }

        /* Enable LPI2C slave */
        LPI2C_HAL_SlaveSetEnable(baseAddr, true);
    }

    return LPI2C_STATUS_SUCCESS;
}


/*FUNCTION**********************************************************************
 *
 * Function Name : LPI2C_DRV_SlaveDeinit
 * Description   : de-initialize the I2C slave mode driver
 *
 * Implements : LPI2C_DRV_SlaveDeinit_Activity
 *END**************************************************************************/
lpi2c_status_t LPI2C_DRV_SlaveDeinit(uint32_t instance)
{
    LPI2C_Type *baseAddr;

    if (instance >= LPI2C_INSTANCE_COUNT)
    {
        return LPI2C_STATUS_INVALID_PARAMETER;
    }

    baseAddr = g_lpi2cBase[instance];
    lpi2c_slave_state_t *slave = g_lpi2cSlaveStatePtr[instance];
    if (slave == NULL)
    {
        return LPI2C_STATUS_FAIL;
    }

    /* Destroy the semaphore */
    (void)OSIF_SemaDestroy(&(slave->idleSemaphore));

    if ((slave->transferType == LPI2C_USING_DMA) && slave->slaveListening)
    {
        /*
         * Disable LPI2C DMA requests.
         * Release DMA channels.
         */
        (void)LPI2C_HAL_SlaveSetRxDMA(baseAddr, false);
        (void)LPI2C_HAL_SlaveSetTxDMA(baseAddr, false);

        (void)EDMA_DRV_ReleaseChannel(&(slave->rxDMAChannel));
        (void)EDMA_DRV_ReleaseChannel(&(slave->txDMAChannel));
    }

    g_lpi2cSlaveStatePtr[instance] = NULL;

    /* Disable LPI2C slave */
    LPI2C_HAL_SlaveSetEnable(baseAddr, false);

    /* Disable i2c interrupt */
    INT_SYS_DisableIRQ(g_lpi2cSlaveIrqId[instance]);

    return LPI2C_STATUS_SUCCESS;
}


/*FUNCTION**********************************************************************
 *
 * Function Name : LPI2C_DRV_SlaveSetTxBuffer
 * Description   : Provide a buffer for transmitting data.
 *
 * Implements : LPI2C_DRV_SlaveSetTxBuffer_Activity
 *END**************************************************************************/
lpi2c_status_t LPI2C_DRV_SlaveSetTxBuffer(uint32_t instance,
                                                 const uint8_t * txBuff,
                                                 uint32_t txSize)
{
    lpi2c_slave_state_t * slave;

    if ((txBuff == NULL) || (txSize <= 0U) || (instance >= LPI2C_INSTANCE_COUNT))
    {
        return LPI2C_STATUS_INVALID_PARAMETER;
    }

    slave = g_lpi2cSlaveStatePtr[instance];
    if (slave == NULL)
    {
        return LPI2C_STATUS_FAIL;
    }

    slave->txBuff = txBuff;
    slave->txSize = txSize;

    return LPI2C_STATUS_SUCCESS;
}


/*FUNCTION**********************************************************************
 *
 * Function Name : LPI2C_DRV_SlaveSetRxBuffer
 * Description   : Provide a buffer for receiving data.
 *
 * Implements : LPI2C_DRV_SlaveSetRxBuffer_Activity
 *END**************************************************************************/
lpi2c_status_t LPI2C_DRV_SlaveSetRxBuffer(uint32_t instance,
                                                 uint8_t * rxBuff,
                                                 uint32_t  rxSize)
{
    lpi2c_slave_state_t * slave;

    if ((rxBuff == NULL) || (rxSize <= 0U) || (instance >= LPI2C_INSTANCE_COUNT))
    {
        return LPI2C_STATUS_INVALID_PARAMETER;
    }

    slave = g_lpi2cSlaveStatePtr[instance];
    if (slave == NULL)
    {
        return LPI2C_STATUS_FAIL;
    }

    slave->rxBuff = rxBuff;
    slave->rxSize = rxSize;

    return LPI2C_STATUS_SUCCESS;
}


/*FUNCTION**********************************************************************
 *
 * Function Name : LPI2C_DRV_SlaveSendData
 * Description   : perform a non-blocking send transaction on the I2C bus
 *
 * Implements : LPI2C_DRV_SlaveSendData_Activity
 *END**************************************************************************/
lpi2c_status_t LPI2C_DRV_SlaveSendData(uint32_t instance,
                                   const uint8_t * txBuff,
                                   uint32_t txSize)
{
    LPI2C_Type *baseAddr;
    lpi2c_slave_state_t * slave;
    lpi2c_status_t status;


#ifdef DEV_ERROR_DETECT
    DEV_ASSERT(instance < LPI2C_INSTANCE_COUNT);
#endif

    if ((txBuff == NULL) || (txSize == 0U))
    {
        return LPI2C_STATUS_INVALID_PARAMETER;
    }

    baseAddr = g_lpi2cBase[instance];
    slave = g_lpi2cSlaveStatePtr[instance];
    if (slave == NULL)
    {
        return LPI2C_STATUS_FAIL;
    }

    if (!slave->slaveListening)
    {
         if (slave->isTransferInProgress == true)
        {
            /* Slave is busy */
            return LPI2C_STATUS_BUSY;
        }

        status = slave->status;
        slave->txBuff = txBuff;
        slave->txSize = txSize;
        slave->status = LPI2C_STATUS_BUSY;

        if (slave->transferType == LPI2C_USING_DMA)
        {
            uint8_t dmaChnTx;
            edma_status_t error;

            /* Request Tx DMA channel. */
            error = EDMA_DRV_RequestChannel((uint8_t)EDMA_ANY_CHANNEL, lpi2c_txDmaSource[instance],
                                            &(slave->txDMAChannel), &dmaChnTx);
            if (error != EDMA_STATUS_SUCCESS)
            {
                slave->status = status;
                slave->isTransferInProgress = false;
                return LPI2C_STATUS_FAIL;
            }

            /* Activate events */
            LPI2C_HAL_SlaveSetInt(baseAddr, LPI2C_HAL_SLAVE_BIT_ERROR_INT |
                                            LPI2C_HAL_SLAVE_FIFO_ERROR_INT |
                                            LPI2C_HAL_SLAVE_STOP_DETECT_INT |
                                            LPI2C_HAL_SLAVE_REPEATED_START_INT |
                                            LPI2C_HAL_SLAVE_ADDRESS_VALID_INT,
                                  true);

            /* Enable LPI2C slave */
            LPI2C_HAL_SlaveSetEnable(baseAddr, true);

            slave->isTransferInProgress = true;

            LPI2C_DRV_SlaveStartDmaTransfer(instance);
        }
        else
        {
            /* Activate events */
            LPI2C_HAL_SlaveSetInt(baseAddr, LPI2C_HAL_SLAVE_BIT_ERROR_INT |
                                            LPI2C_HAL_SLAVE_FIFO_ERROR_INT |
                                            LPI2C_HAL_SLAVE_STOP_DETECT_INT |
                                            LPI2C_HAL_SLAVE_REPEATED_START_INT |
                                            LPI2C_HAL_SLAVE_ADDRESS_VALID_INT |
                                            LPI2C_HAL_SLAVE_TRANSMIT_DATA_INT,
                                  true);

            /* Enable LPI2C slave */
            LPI2C_HAL_SlaveSetEnable(baseAddr, true);

            slave->isTransferInProgress = true;
        }
    }
    else
    {
        // If the slave is in listening mode the user should not use this function or the blocking counterpart.
        return LPI2C_STATUS_FAIL;
    }

    return LPI2C_STATUS_SUCCESS;
}


/*FUNCTION**********************************************************************
 *
 * Function Name : LPI2C_DRV_SlaveSendDataBlocking
 * Description   : perform a blocking send transaction on the I2C bus
 *
 * Implements : LPI2C_DRV_SlaveSendDataBlocking_Activity
 *END**************************************************************************/
lpi2c_status_t LPI2C_DRV_SlaveSendDataBlocking(uint32_t    instance,
                                           const uint8_t *  txBuff,
                                           uint32_t txSize,
                                           uint32_t timeout)
{
    lpi2c_status_t retVal = LPI2C_STATUS_SUCCESS;

#ifdef DEV_ERROR_DETECT
    DEV_ASSERT(instance < LPI2C_INSTANCE_COUNT);
#endif

    if ((txBuff == NULL) || (txSize == 0U))
    {
        return LPI2C_STATUS_INVALID_PARAMETER;
    }

    lpi2c_slave_state_t *slave = g_lpi2cSlaveStatePtr[instance];
    if (slave == NULL)
    {
        return LPI2C_STATUS_FAIL;
    }

    /* mark transfer as blocking */
    slave->blocking = true;

    retVal = LPI2C_DRV_SlaveSendData(instance, txBuff, txSize);
    if (retVal != LPI2C_STATUS_SUCCESS)
    {
        return retVal;
    }

    /* Wait for transfer to end */
    return LPI2C_DRV_SlaveWaitTransferEnd(instance, timeout);
}


/*FUNCTION**********************************************************************
 *
 * Function Name : LPI2C_DRV_SlaveReceiveData
 * Description   : perform a non-blocking receive transaction on the I2C bus
 *
 * Implements : LPI2C_DRV_SlaveReceiveData_Activity
 *END**************************************************************************/
lpi2c_status_t LPI2C_DRV_SlaveReceiveData(uint32_t   instance,
                                       uint8_t * rxBuff,
                                       uint32_t  rxSize)
{
    LPI2C_Type *baseAddr;
    lpi2c_slave_state_t * slave;
    lpi2c_status_t status;

#ifdef DEV_ERROR_DETECT
    DEV_ASSERT(instance < LPI2C_INSTANCE_COUNT);
#endif

    if ((rxBuff == NULL) || (rxSize == 0U))
    {
        return LPI2C_STATUS_INVALID_PARAMETER;
    }

    baseAddr = g_lpi2cBase[instance];
    slave = g_lpi2cSlaveStatePtr[instance];
    if (slave == NULL)
    {
        return LPI2C_STATUS_FAIL;
    }

    if (!slave->slaveListening)
    {
        if (slave->isTransferInProgress == true)
        {
            /* Slave is busy */
            return LPI2C_STATUS_BUSY;
        }

        status = slave->status;
        slave->rxBuff = rxBuff;
        slave->rxSize = rxSize;
        slave->status = LPI2C_STATUS_BUSY;

        if (slave->transferType == LPI2C_USING_DMA)
        {
            uint8_t dmaChnRx;
            edma_status_t error;

            /* Request Rx DMA channel. */
            error = EDMA_DRV_RequestChannel((uint8_t)EDMA_ANY_CHANNEL, lpi2c_rxDmaSource[instance],
                                            &(slave->rxDMAChannel), &dmaChnRx);
            if (error != EDMA_STATUS_SUCCESS)
            {
                slave->status = status;
                slave->isTransferInProgress = false;
                return LPI2C_STATUS_FAIL;
            }

            /* Activate events */
            LPI2C_HAL_SlaveSetInt(baseAddr, LPI2C_HAL_SLAVE_BIT_ERROR_INT |
                                            LPI2C_HAL_SLAVE_FIFO_ERROR_INT |
                                            LPI2C_HAL_SLAVE_STOP_DETECT_INT |
                                            LPI2C_HAL_SLAVE_REPEATED_START_INT |
                                            LPI2C_HAL_SLAVE_ADDRESS_VALID_INT,
                                  true);

            /* Enable LPI2C slave */
            LPI2C_HAL_SlaveSetEnable(baseAddr, true);

            slave->isTransferInProgress = true;

            LPI2C_DRV_SlaveStartDmaTransfer(instance);
        }
        else
        {
            slave->isTransferInProgress = true;

            /* Activate events */
            LPI2C_HAL_SlaveSetInt(baseAddr, LPI2C_HAL_SLAVE_BIT_ERROR_INT |
                                            LPI2C_HAL_SLAVE_FIFO_ERROR_INT |
                                            LPI2C_HAL_SLAVE_STOP_DETECT_INT |
                                            LPI2C_HAL_SLAVE_REPEATED_START_INT |
                                            LPI2C_HAL_SLAVE_ADDRESS_VALID_INT |
                                            LPI2C_HAL_SLAVE_RECEIVE_DATA_INT,
                                  true);

            /* Enable LPI2C slave */
            LPI2C_HAL_SlaveSetEnable(baseAddr, true);
        }
    }
    else
    {
        // If the slave is in listening mode the user should not use this function or the blocking counterpart.
        return LPI2C_STATUS_FAIL;
    }

    return LPI2C_STATUS_SUCCESS;
}


/*FUNCTION**********************************************************************
 *
 * Function Name : LPI2C_DRV_SlaveReceiveDataBlocking
 * Description   : perform a blocking receive transaction on the I2C bus
 *
 * Implements : LPI2C_DRV_SlaveReceiveDataBlocking_Activity
 *END**************************************************************************/
lpi2c_status_t LPI2C_DRV_SlaveReceiveDataBlocking(uint32_t instance,
                                               uint8_t  *rxBuff,
                                               uint32_t rxSize,
                                               uint32_t timeout)
{
    lpi2c_status_t retVal = LPI2C_STATUS_SUCCESS;

#ifdef DEV_ERROR_DETECT
    DEV_ASSERT(instance < LPI2C_INSTANCE_COUNT);
#endif

    if ((rxBuff == NULL) || (rxSize == 0U))
    {
        return LPI2C_STATUS_INVALID_PARAMETER;
    }

    lpi2c_slave_state_t *slave = g_lpi2cSlaveStatePtr[instance];
    if (slave == NULL)
    {
        return LPI2C_STATUS_FAIL;
    }

    /* mark transfer as blocking */
    slave->blocking = true;

    retVal = LPI2C_DRV_SlaveReceiveData(instance, rxBuff, rxSize);
    if (retVal != LPI2C_STATUS_SUCCESS)
    {
        return retVal;
    }

    /* Wait for transfer to end */
    return LPI2C_DRV_SlaveWaitTransferEnd(instance, timeout);
}


/*FUNCTION**********************************************************************
 *
 * Function Name : LPI2C_DRV_SlaveGetTransferStatus
 * Description   : return the current status of the I2C slave transfer
 *
 * When performing an a-sync (non-blocking) transfer, the user can call this function
 * to ascertain the state of the current transfer. In addition, if the transfer is still
 * in progress, the user can get the number of words that should be receive.
 *
 * Implements : LPI2C_DRV_SlaveGetTransferStatus_Activity
 *END**************************************************************************/
lpi2c_status_t LPI2C_DRV_SlaveGetTransferStatus(uint32_t instance,
                                               uint32_t *bytesRemaining)
{
    const lpi2c_slave_state_t *slave;

    if (instance >= LPI2C_INSTANCE_COUNT)
    {
        return LPI2C_STATUS_INVALID_PARAMETER;
    }

    slave = g_lpi2cSlaveStatePtr[instance];
    if (slave == NULL)
    {
        return LPI2C_STATUS_FAIL;
    }

    if ((bytesRemaining != NULL) && (slave->transferType == LPI2C_USING_INTERRUPTS))
    {
        if (slave->txSize > 0U)
        {
            /* Send data */
            *bytesRemaining = slave->txSize;
        }
        else if (slave->rxSize > 0U)
        {
            /* Receive data */
            *bytesRemaining = slave->rxSize;
        }
        else
        {
            *bytesRemaining = 0U;
        }
    }

    return slave->status;
}


/*FUNCTION**********************************************************************
 *
 * Function Name : LPI2C_DRV_SlaveAbortTransferData
 * Description   : abort a non-blocking I2C Master transmission or reception
 *
 * Implements : LPI2C_DRV_SlaveAbortTransferData_Activity
 *END**************************************************************************/
lpi2c_status_t LPI2C_DRV_SlaveAbortTransferData(uint32_t instance)
{
    lpi2c_slave_state_t * slave;
    LPI2C_Type *baseAddr;

    if (instance >= LPI2C_INSTANCE_COUNT)
    {
        return LPI2C_STATUS_INVALID_PARAMETER;
    }

    baseAddr = g_lpi2cBase[instance];
    slave = g_lpi2cSlaveStatePtr[instance];
    if (slave == NULL)
    {
        return LPI2C_STATUS_FAIL;
    }

    if (!slave->slaveListening)
    {
        if (slave->isTransferInProgress == false)
        {
            return LPI2C_STATUS_NO_TRANSFER;
        }

        slave->status = LPI2C_STATUS_ABORTED;
        LPI2C_DRV_SlaveEndTransfer(baseAddr, slave);
    }

    return LPI2C_STATUS_SUCCESS;
}


/*FUNCTION**********************************************************************
 *
 * Function Name : LPI2C_DRV_SlaveIRQHandler
 * Description   : handle non-blocking slave operation when I2C interrupt occurs
 *
 *END**************************************************************************/
void LPI2C_DRV_SlaveIRQHandler(uint32_t instance)
{
    LPI2C_Type *baseAddr;
    lpi2c_slave_state_t * slave;
    bool stopDetect = false, repeatStartDetect = false;

#ifdef DEV_ERROR_DETECT
    DEV_ASSERT(instance < LPI2C_INSTANCE_COUNT);
#endif

    baseAddr = g_lpi2cBase[instance];
    slave = g_lpi2cSlaveStatePtr[instance];
#ifdef DEV_ERROR_DETECT
    DEV_ASSERT(slave != NULL);
#endif

    /* Check which event caused the interrupt */
    if (LPI2C_HAL_SlaveGetAddressValidEvent(baseAddr))
    {
        LPI2C_DRV_SlaveHandleAddressValidEvent(instance, baseAddr, slave);
    }

    if (LPI2C_HAL_SlaveGetTransmitDataEvent(baseAddr))
    {
        if (LPI2C_HAL_SlaveGetInt(baseAddr, LPI2C_HAL_SLAVE_TRANSMIT_DATA_INT))
        {
            LPI2C_DRV_SlaveHandleTransmitDataEvent(instance, baseAddr, slave);
        }
    }

    if (LPI2C_HAL_SlaveGetReceiveDataEvent(baseAddr))
    {
        if (LPI2C_HAL_SlaveGetInt(baseAddr, LPI2C_HAL_SLAVE_RECEIVE_DATA_INT))
        {
            LPI2C_DRV_SlaveHandleReceiveDataEvent(instance, baseAddr, slave);
        }
    }

    stopDetect = LPI2C_HAL_SlaveGetSTOPDetectEvent(baseAddr);
    repeatStartDetect = LPI2C_HAL_SlaveGetRepeatedStartEvent(baseAddr);
    if ((stopDetect == true) || (repeatStartDetect == true))
    {
        /* Either STOP or repeated START have the same meaning here: the current transfer is over */
        LPI2C_HAL_SlaveClearSTOPDetectEvent(baseAddr);
        LPI2C_HAL_SlaveClearRepeatedStartEvent(baseAddr);

        if (slave->status == LPI2C_STATUS_BUSY)
        {
            /* Report success if no error was recorded */
            slave->status = LPI2C_STATUS_SUCCESS;
        }

        if (!slave->slaveListening)
        {
            LPI2C_DRV_SlaveEndTransfer(baseAddr, slave);

            /* Signal transfer end for blocking transfers */
            if (slave->blocking == true)
            {
                (void)OSIF_SemaPost(&(slave->idleSemaphore));
            }
        }
    }

    if (LPI2C_HAL_SlaveGetBitErrorEvent(baseAddr))
    {
        slave->status = LPI2C_STATUS_FAIL;
        LPI2C_HAL_SlaveClearBitErrorEvent(baseAddr);

        /* Signal transfer end for blocking transfers */
        if (slave->blocking == true)
        {
            (void)OSIF_SemaPost(&(slave->idleSemaphore));
        }
    }

    if (LPI2C_HAL_SlaveGetFIFOErrorEvent(baseAddr))
    {
        /* In Ultra-Fast mode clock stretching is disabled, so it is possible to get
           this event if the slave can't keep up */
        slave->status = LPI2C_STATUS_SLAVE_RX_OVERRUN;
        LPI2C_HAL_SlaveClearFIFOErrorEvent(baseAddr);

        /* Signal transfer end for blocking transfers */
        if (slave->blocking == true)
        {
            (void)OSIF_SemaPost(&(slave->idleSemaphore));
        }
    }
}

/*******************************************************************************
 * EOF
 ******************************************************************************/
