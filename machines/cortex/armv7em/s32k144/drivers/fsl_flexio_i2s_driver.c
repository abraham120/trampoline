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

#include "fsl_flexio_i2s_driver.h"
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

    /* Shifters/Timers used for I2S simulation The parameter x represents the 
       resourceIndex value for the current driver instance */
#define TX_SHIFTER(x)     (x)
#define RX_SHIFTER(x)     ((x)+1U)
#define SCK_TIMER(x)      (x)
#define WS_TIMER(x)       ((x)+1U)

    /* Table to map flexio_i2s transfer sizes to EDMA transfer sizes */
static const edma_transfer_size_t dmaTransferSize[4U] = 
        {EDMA_TRANSFER_SIZE_1B, EDMA_TRANSFER_SIZE_2B, EDMA_TRANSFER_SIZE_4B, EDMA_TRANSFER_SIZE_4B};

/*******************************************************************************
 * Private Functions
 ******************************************************************************/

/*FUNCTION**********************************************************************
 *
 * Function Name : FLEXIO_I2S_DRV_ComputeByteWidth
 * Description   : Computes the baud rate divider for a target baud rate
 *
 *END**************************************************************************/
static uint8_t FLEXIO_I2S_DRV_ComputeByteWidth(uint8_t bitsWidth)
{
    uint8_t byteWidth;

    if (bitsWidth <= 8U)
    {
        byteWidth = 1U;
    }
    else if (bitsWidth <= 16U)
    {
        byteWidth = 2U;
    }
    else
    {
        byteWidth = 4U;
    }

    return byteWidth;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : FLEXIO_I2S_DRV_MasterComputeBaudRateDivider
 * Description   : Computes the baud rate divider for a target baud rate
 *
 *END**************************************************************************/
static flexio_status_t FLEXIO_I2S_DRV_MasterComputeBaudRateDivider(flexio_i2s_master_state_t *master, 
                                                                   uint32_t baudRate, 
                                                                   uint16_t *divider,
                                                                   uint32_t inputClock)
{
    uint32_t tmpDiv;

    (void)master;
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
 * Function Name : FLEXIO_I2S_DRV_MasterConfigure
 * Description   : configures the FLEXIO module as I2S master
 *
 *END**************************************************************************/
static flexio_status_t FLEXIO_I2S_DRV_MasterConfigure(flexio_i2s_master_state_t *master,
                                                      const flexio_i2s_master_user_config_t * userConfigPtr,
                                                      uint32_t inputClock)
{
    FLEXIO_Type *baseAddr;
    uint16_t divider;
    uint16_t bits;
    uint8_t resourceIndex;    /* Index of first used internal resource instance (shifter and timer) */

    baseAddr = g_flexioBase[master->flexioCommon.instance];
    resourceIndex = master->flexioCommon.resourceIndex;

    /* Compute divider. */
    FLEXIO_I2S_DRV_MasterComputeBaudRateDivider(master, userConfigPtr->baudRate, &divider, inputClock);
    bits = userConfigPtr->bitsWidth;

    /* Configure tx shifter */
    FLEXIO_HAL_SetShifterConfig(baseAddr,
                                TX_SHIFTER(resourceIndex),
                                FLEXIO_SHIFTER_START_BIT_DISABLED_SH,
                                FLEXIO_SHIFTER_STOP_BIT_DISABLED,
                                FLEXIO_SHIFTER_SOURCE_PIN);
    FLEXIO_HAL_SetShifterControl(baseAddr,
                                 TX_SHIFTER(resourceIndex),
                                 FLEXIO_SHIFTER_MODE_TRANSMIT,
                                 userConfigPtr->txPin,             /* output on tx pin */
                                 FLEXIO_PIN_POLARITY_HIGH,
                                 FLEXIO_PIN_CONFIG_OUTPUT,
                                 SCK_TIMER(resourceIndex),         /* use clock timer to drive the shifter */
                                 FLEXIO_TIMER_POLARITY_POSEDGE);

    /* Configure rx shifter */
    FLEXIO_HAL_SetShifterConfig(baseAddr,
                                RX_SHIFTER(resourceIndex),
                                FLEXIO_SHIFTER_START_BIT_DISABLED,
                                FLEXIO_SHIFTER_STOP_BIT_DISABLED,
                                FLEXIO_SHIFTER_SOURCE_PIN);
    FLEXIO_HAL_SetShifterControl(baseAddr,
                                 RX_SHIFTER(resourceIndex),
                                 FLEXIO_SHIFTER_MODE_RECEIVE,
                                 userConfigPtr->rxPin,                    /* output to rx pin */       
                                 FLEXIO_PIN_POLARITY_HIGH,
                                 FLEXIO_PIN_CONFIG_DISABLED,
                                 SCK_TIMER(resourceIndex),     /* use control timer to drive the shifter */
                                 FLEXIO_TIMER_POLARITY_NEGEDGE);

    /* Configure SCK timer */
    FLEXIO_HAL_SetTimerCompare(baseAddr, SCK_TIMER(resourceIndex), (((bits << 1U) - 1U) << 8U) + divider);
    FLEXIO_HAL_SetTimerConfig(baseAddr, 
                              SCK_TIMER(resourceIndex),
                              FLEXIO_TIMER_START_BIT_ENABLED,
                              FLEXIO_TIMER_STOP_BIT_DISABLED,
                              FLEXIO_TIMER_ENABLE_TRG_HIGH,         /* enable when Tx data is available */
                              FLEXIO_TIMER_DISABLE_NEVER,
                              FLEXIO_TIMER_RESET_NEVER,
                              FLEXIO_TIMER_DECREMENT_CLK_SHIFT_TMR, /* decrement on FlexIO clock */
                              FLEXIO_TIMER_INITOUT_ONE);
    FLEXIO_HAL_SetTimerControl(baseAddr,
                               SCK_TIMER(resourceIndex),
                               (TX_SHIFTER(resourceIndex) << 2U)+1U,  /* trigger on tx shifter status flag */
                               FLEXIO_TRIGGER_POLARITY_LOW,
                               FLEXIO_TRIGGER_SOURCE_INTERNAL,
                               userConfigPtr->sckPin,                 /* output on SCK pin */
                               FLEXIO_PIN_POLARITY_HIGH,
                               FLEXIO_PIN_CONFIG_OUTPUT,              /* enable output */
                               FLEXIO_TIMER_MODE_8BIT_BAUD);

    /* Configure WS timer */

    FLEXIO_HAL_SetTimerCompare(baseAddr, WS_TIMER(resourceIndex), bits * ((divider + 1U) * 2U) - 1U);
    FLEXIO_HAL_SetTimerConfig(baseAddr, 
                              WS_TIMER(resourceIndex),
                              FLEXIO_TIMER_START_BIT_DISABLED,
                              FLEXIO_TIMER_STOP_BIT_DISABLED,
                              FLEXIO_TIMER_ENABLE_TIM_ENABLE,       /* enable when SCK timer is enabled */
                              FLEXIO_TIMER_DISABLE_NEVER,
                              FLEXIO_TIMER_RESET_NEVER,
                              FLEXIO_TIMER_DECREMENT_CLK_SHIFT_TMR, /* decrement on FlexIO clock */
                              FLEXIO_TIMER_INITOUT_ONE);
    FLEXIO_HAL_SetTimerControl(baseAddr,
                               WS_TIMER(resourceIndex),
                               0U,                                  /* trigger not used */
                               FLEXIO_TRIGGER_POLARITY_HIGH,
                               FLEXIO_TRIGGER_SOURCE_EXTERNAL,
                               userConfigPtr->wsPin,                /* output on WS pin */
                               FLEXIO_PIN_POLARITY_LOW,
                               FLEXIO_PIN_CONFIG_OUTPUT,            /* enable output */
                               FLEXIO_TIMER_MODE_16BIT);

    return FLEXIO_STATUS_SUCCESS;
}


/*FUNCTION**********************************************************************
 *
 * Function Name : FLEXIO_I2S_DRV_SlaveConfigure
 * Description   : configures the FLEXIO module as I2S slave
 *
 *END**************************************************************************/
static flexio_status_t FLEXIO_I2S_DRV_SlaveConfigure(flexio_i2s_slave_state_t *slave,
                                                      const flexio_i2s_slave_user_config_t * userConfigPtr)
{
    FLEXIO_Type *baseAddr;
    uint16_t bits;
    uint8_t resourceIndex;    /* Index of first used internal resource instance (shifter and timer) */

    baseAddr = g_flexioBase[slave->flexioCommon.instance];
    resourceIndex = slave->flexioCommon.resourceIndex;
    bits = userConfigPtr->bitsWidth;


    /* Configure tx shifter */
    FLEXIO_HAL_SetShifterConfig(baseAddr,
                                TX_SHIFTER(resourceIndex),
                                FLEXIO_SHIFTER_START_BIT_DISABLED,
                                FLEXIO_SHIFTER_STOP_BIT_DISABLED,
                                FLEXIO_SHIFTER_SOURCE_PIN);
    FLEXIO_HAL_SetShifterControl(baseAddr,
                                 TX_SHIFTER(resourceIndex),
                                 FLEXIO_SHIFTER_MODE_TRANSMIT,
                                 userConfigPtr->txPin,             /* output on tx pin */
                                 FLEXIO_PIN_POLARITY_HIGH,
                                 FLEXIO_PIN_CONFIG_OUTPUT,
                                 WS_TIMER(resourceIndex),         /* use clock timer to drive the shifter */
                                 FLEXIO_TIMER_POLARITY_POSEDGE);

    /* Configure rx shifter */
    FLEXIO_HAL_SetShifterConfig(baseAddr,
                                RX_SHIFTER(resourceIndex),
                                FLEXIO_SHIFTER_START_BIT_DISABLED,
                                FLEXIO_SHIFTER_STOP_BIT_DISABLED,
                                FLEXIO_SHIFTER_SOURCE_PIN);
    FLEXIO_HAL_SetShifterControl(baseAddr,
                                 RX_SHIFTER(resourceIndex),
                                 FLEXIO_SHIFTER_MODE_RECEIVE,
                                 userConfigPtr->rxPin,                    /* output to rx pin */       
                                 FLEXIO_PIN_POLARITY_HIGH,
                                 FLEXIO_PIN_CONFIG_DISABLED,
                                 WS_TIMER(resourceIndex),     /* use control timer to drive the shifter */
                                 FLEXIO_TIMER_POLARITY_NEGEDGE);

    /* Configure SCK timer */
    FLEXIO_HAL_SetTimerCompare(baseAddr, SCK_TIMER(resourceIndex), (bits << 2U) - 3U);
    FLEXIO_HAL_SetTimerConfig(baseAddr, 
                              SCK_TIMER(resourceIndex),
                              FLEXIO_TIMER_START_BIT_DISABLED,
                              FLEXIO_TIMER_STOP_BIT_DISABLED,
                              FLEXIO_TIMER_ENABLE_PIN_POSEDGE,      /* enable on WS positive edge */
                              FLEXIO_TIMER_DISABLE_TIM_CMP,
                              FLEXIO_TIMER_RESET_NEVER,
                              FLEXIO_TIMER_DECREMENT_TRG_SHIFT_TRG, /* decrement on SCK input  */
                              FLEXIO_TIMER_INITOUT_ONE);
    FLEXIO_HAL_SetTimerControl(baseAddr,
                               SCK_TIMER(resourceIndex),
                               (userConfigPtr->sckPin << 1U),          /* trigger on SCK pin */
                               FLEXIO_TRIGGER_POLARITY_HIGH,
                               FLEXIO_TRIGGER_SOURCE_INTERNAL,
                               userConfigPtr->wsPin,                 /* use WS input pin */
                               FLEXIO_PIN_POLARITY_LOW,
                               FLEXIO_PIN_CONFIG_DISABLED,
                               FLEXIO_TIMER_MODE_16BIT);

    /* Configure WS timer */
    FLEXIO_HAL_SetTimerCompare(baseAddr, WS_TIMER(resourceIndex), (bits << 1U) - 1U);
    FLEXIO_HAL_SetTimerConfig(baseAddr, 
                              WS_TIMER(resourceIndex),
                              FLEXIO_TIMER_START_BIT_DISABLED,
                              FLEXIO_TIMER_STOP_BIT_DISABLED,
                              FLEXIO_TIMER_ENABLE_PIN_POSEDGE_TRG_HIGH,
                              FLEXIO_TIMER_DISABLE_TIM_CMP_TRG_LOW,
                              FLEXIO_TIMER_RESET_NEVER,
                              FLEXIO_TIMER_DECREMENT_PIN_SHIFT_PIN,
                              FLEXIO_TIMER_INITOUT_ONE);
    FLEXIO_HAL_SetTimerControl(baseAddr,
                               WS_TIMER(resourceIndex),
                               (SCK_TIMER(resourceIndex) << 2U)+3U,   /* SCK timer trigger output */
                               FLEXIO_TRIGGER_POLARITY_HIGH,
                               FLEXIO_TRIGGER_SOURCE_INTERNAL,
                               userConfigPtr->sckPin,               /* SCK input pin */
                               FLEXIO_PIN_POLARITY_HIGH,
                               FLEXIO_PIN_CONFIG_DISABLED,          /* enable output */
                               FLEXIO_TIMER_MODE_16BIT);

    return FLEXIO_STATUS_SUCCESS;
}


/*FUNCTION**********************************************************************
 *
 * Function Name : FLEXIO_I2S_DRV_MasterEndTransfer
 * Description   : End the current transfer
 *
 *END**************************************************************************/
static void FLEXIO_I2S_DRV_MasterEndTransfer(flexio_i2s_master_state_t *master)
{
    FLEXIO_Type *baseAddr;
    uint8_t resourceIndex;    /* Index of first used internal resource instance (shifter and timer) */

    baseAddr = g_flexioBase[master->flexioCommon.instance];
    resourceIndex = master->flexioCommon.resourceIndex;

    /* Disable transfer engine */
    switch (master->driverType)
    {
        case FLEXIO_DRIVER_TYPE_INTERRUPTS:
            /* Disable interrupts for Rx and Tx shifters */
            FLEXIO_HAL_SetShifterInterrupt(baseAddr, 
                                 (1U << TX_SHIFTER(resourceIndex)) | (1U << RX_SHIFTER(resourceIndex)), 
                                 false);
            FLEXIO_HAL_SetShifterErrorInterrupt(baseAddr, 
                                 (1U << TX_SHIFTER(resourceIndex)) | (1U << RX_SHIFTER(resourceIndex)), 
                                 false);
            break;
        case FLEXIO_DRIVER_TYPE_POLLING:
            /* Nothing to do here */
            break;
        case FLEXIO_DRIVER_TYPE_DMA:
            /* Disable FlexIO Tx and Rx DMA requests */
            FLEXIO_HAL_SetShifterDMARequest(baseAddr, (1U << TX_SHIFTER(resourceIndex)) | (1U << RX_SHIFTER(resourceIndex)), false);
            /* Release the DMA channels */
            (void)EDMA_DRV_ReleaseChannel(&(master->txDMAChannel));
            (void)EDMA_DRV_ReleaseChannel(&(master->rxDMAChannel));
            break;
        default: 
            /* Impossible type - do nothing */
            break;
    }

    master->driverIdle = true;
    master->txData = NULL;
    master->rxData = NULL;
    master->txRemainingBytes = 0U;
    master->rxRemainingBytes = 0U;
}


/*FUNCTION**********************************************************************
 *
 * Function Name : FLEXIO_I2S_DRV_MasterStopTransfer
 * Description   : Forcefully stops the current transfer
 *
 *END**************************************************************************/
static void FLEXIO_I2S_DRV_MasterStopTransfer(flexio_i2s_master_state_t *master)
{
    FLEXIO_Type *baseAddr;
    uint8_t resourceIndex;    /* Index of first used internal resource instance (shifter and timer) */


    resourceIndex = master->flexioCommon.resourceIndex;
    baseAddr = g_flexioBase[master->flexioCommon.instance];

    /* disable and re-enable timers and shifters to reset them */
    FLEXIO_HAL_SetShifterMode(baseAddr, TX_SHIFTER(resourceIndex), FLEXIO_SHIFTER_MODE_DISABLED);
    FLEXIO_HAL_SetShifterMode(baseAddr, RX_SHIFTER(resourceIndex), FLEXIO_SHIFTER_MODE_DISABLED);
    FLEXIO_HAL_SetTimerMode(baseAddr, SCK_TIMER(resourceIndex), FLEXIO_TIMER_MODE_DISABLED);
    FLEXIO_HAL_SetTimerMode(baseAddr, WS_TIMER(resourceIndex), FLEXIO_TIMER_MODE_DISABLED);

    /* clear any leftover error flags */
    FLEXIO_HAL_ClearShifterErrorStatus(baseAddr, TX_SHIFTER(resourceIndex));
    FLEXIO_HAL_ClearShifterErrorStatus(baseAddr, RX_SHIFTER(resourceIndex));

    /* end the transfer */
    FLEXIO_I2S_DRV_MasterEndTransfer(master);

    /* re-enable timers and shifters */
    FLEXIO_HAL_SetShifterMode(baseAddr, TX_SHIFTER(resourceIndex), FLEXIO_SHIFTER_MODE_TRANSMIT);
    FLEXIO_HAL_SetShifterMode(baseAddr, RX_SHIFTER(resourceIndex), FLEXIO_SHIFTER_MODE_RECEIVE);
    if (master->master)
    {
        FLEXIO_HAL_SetTimerMode(baseAddr, SCK_TIMER(resourceIndex), FLEXIO_TIMER_MODE_8BIT_BAUD);
    }
    else
    {
        FLEXIO_HAL_SetTimerMode(baseAddr, SCK_TIMER(resourceIndex), FLEXIO_TIMER_MODE_16BIT);
    }
    FLEXIO_HAL_SetTimerMode(baseAddr, WS_TIMER(resourceIndex), FLEXIO_TIMER_MODE_16BIT);
}


/*FUNCTION**********************************************************************
 *
 * Function Name : FLEXIO_I2S_DRV_MasterWaitTransferEnd
 * Description   : waits for the end of a blocking transfer
 *
 *END**************************************************************************/
static flexio_status_t FLEXIO_I2S_DRV_MasterWaitTransferEnd(flexio_i2s_master_state_t *master, uint32_t timeout)
{
    osif_status_t osifError = OSIF_STATUS_SUCCESS;

    switch (master->driverType)
    {
        case FLEXIO_DRIVER_TYPE_INTERRUPTS:
            /* Wait for transfer to be completed by the IRQ */
            osifError = OSIF_SemaWait(&(master->idleSemaphore), timeout);
            break;
        case FLEXIO_DRIVER_TYPE_POLLING:
            /* Call FLEXIO_I2S_DRV_MasterGetStatus() to do the transfer */
            while (FLEXIO_I2S_DRV_MasterGetStatus(master, NULL) == FLEXIO_STATUS_BUSY);
            break;
        case FLEXIO_DRIVER_TYPE_DMA:
            osifError = OSIF_SemaWait(&(master->idleSemaphore), timeout);
            break;
        default:
            /* Impossible type - do nothing */
            break;
    }
    if (osifError == OSIF_STATUS_TIMEOUT)
    {
        /* abort current transfer */
        master->status = FLEXIO_STATUS_TIMEOUT;
        FLEXIO_I2S_DRV_MasterStopTransfer(master);
    }

    /* blocking transfer is over */
    master->blocking = false;
    return master->status;
}


/*FUNCTION**********************************************************************
 *
 * Function Name : FLEXIO_I2S_DRV_ReadData
 * Description   : reads data received by the module
 *
 *END**************************************************************************/
static void FLEXIO_I2S_DRV_ReadData(flexio_i2s_master_state_t *master)
{
    FLEXIO_Type *baseAddr;
    uint32_t data;
    uint8_t resourceIndex;    /* Index of first used internal resource instance (shifter and timer) */

    baseAddr = g_flexioBase[master->flexioCommon.instance];
    resourceIndex = master->flexioCommon.resourceIndex;

    /* Read data from shifter buffer */
    data = FLEXIO_HAL_ReadShifterBuffer(baseAddr, RX_SHIFTER(resourceIndex), FLEXIO_SHIFTER_RW_MODE_BIT_SWAP);

    if (master->rxRemainingBytes > 0U)
    {
        if (master->rxData != NULL)
        {
            switch (master->byteWidth)
            {
            case 1U:
                *(uint8_t *)master->rxData = (uint8_t)data;
                break;
            case 2U:
                *(uint16_t *)master->rxData = (uint16_t)data;
                break;
            default:
                *(uint32_t *)master->rxData = (uint32_t)data;
                break;
            }
            /* Update rx buffer pointer */
            master->rxData += (uint32_t)(master->byteWidth);
        }
        /* Update remaining bytes count even if buffer is null */
        master->rxRemainingBytes -= (uint32_t)(master->byteWidth);
    }
    else
    {
        /* No data to receive, just ignore the read data */
    }
}


/*FUNCTION**********************************************************************
 *
 * Function Name : FLEXIO_I2S_DRV_WriteData
 * Description   : writes data to be transmitted by the module
 *
 *END**************************************************************************/
static void FLEXIO_I2S_DRV_WriteData(flexio_i2s_master_state_t *master)
{
    FLEXIO_Type *baseAddr;
    uint32_t data;
    uint8_t resourceIndex;    /* Index of first used internal resource instance (shifter and timer) */

    baseAddr = g_flexioBase[master->flexioCommon.instance];
    resourceIndex = master->flexioCommon.resourceIndex;

    if (master->txRemainingBytes == 0U)
    {
        /* Done transmitting */
        return;
    }

    /* Read data from user buffer and update tx buffer pointer and remaining bytes count */
    switch (master->byteWidth)
    {
    case 1U:
        data = (uint32_t)(*(uint8_t *)master->txData);
        break;
    case 2U:
        data = (uint32_t)(*(uint16_t *)master->txData);
        break;
    default:
        data = (uint32_t)(*(uint32_t *)master->txData);
        break;
    }
    master->txData += master->byteWidth;
    master->txRemainingBytes -= master->byteWidth;

    /* Write data to shifter buffer */
    /* Shift data before bit-swapping it to get the relevant bits in the lower part of the shifter */
    data <<= 32U - (uint32_t)(master->bitsWidth);
    FLEXIO_HAL_WriteShifterBuffer(baseAddr, TX_SHIFTER(resourceIndex), data, FLEXIO_SHIFTER_RW_MODE_BIT_SWAP);
}


/*FUNCTION**********************************************************************
 *
 * Function Name : FLEXIO_I2S_DRV_MasterCheckStatus
 * Description   : Check status of the I2S transfer. This function can be 
 *                 called either in an interrupt routine or directly in polling 
 *                 mode to advance the I2S transfer.
 *
 *END**************************************************************************/
static void FLEXIO_I2S_DRV_MasterCheckStatus(void *stateStruct)
{
    FLEXIO_Type *baseAddr;
    uint8_t resourceIndex;    /* Index of first used internal resource instance (shifter and timer) */
    flexio_i2s_master_state_t *master;

#ifdef DEV_ERROR_DETECT
    DEV_ASSERT(stateStruct != NULL);
#endif

    master = (flexio_i2s_master_state_t *)stateStruct;
    baseAddr = g_flexioBase[master->flexioCommon.instance];
    resourceIndex = master->flexioCommon.resourceIndex;

    /* Check for errors */
    if ((master->txData != NULL) && FLEXIO_HAL_GetShifterErrorStatus(baseAddr, TX_SHIFTER(resourceIndex)))
    {
        master->status = FLEXIO_STATUS_TX_UNDERFLOW;
        FLEXIO_HAL_ClearShifterErrorStatus(baseAddr, TX_SHIFTER(resourceIndex));
        /* Continue processing events */
    }
    if ((master->rxData != NULL) && FLEXIO_HAL_GetShifterErrorStatus(baseAddr, RX_SHIFTER(resourceIndex)))
    {
        master->status = FLEXIO_STATUS_RX_OVERFLOW;
        FLEXIO_HAL_ClearShifterErrorStatus(baseAddr, RX_SHIFTER(resourceIndex));
        /* Continue processing events */
    }
    /* Check if data was received */
    if (FLEXIO_HAL_GetShifterStatus(baseAddr, RX_SHIFTER(resourceIndex)))
    {
        FLEXIO_I2S_DRV_ReadData(master);
        if ((master->rxData != NULL) && (master->rxRemainingBytes == 0U))
        {
            /* Out of rx space, call callback to allow user to provide a new buffer */
            if (master->callback != NULL)
            {
                master->callback(master, FLEXIO_EVENT_RX_FULL, master->callbackParam);
            }
        }
    }
    /* Check if transmitter needs more data */
    if ((master->txData != NULL) && FLEXIO_HAL_GetShifterStatus(baseAddr, TX_SHIFTER(resourceIndex)))
    {
        FLEXIO_I2S_DRV_WriteData(master);
        if (master->txRemainingBytes == 0U)
        {
            /* Out of data, call callback to allow user to provide a new buffer */
            if (master->callback != NULL)
            {
                master->callback(master, FLEXIO_EVENT_TX_EMPTY, master->callbackParam);
            }
            if (master->txRemainingBytes == 0U)
            {
                /* Still no more data to transmit, transmission will stop */
                if (master->driverType == FLEXIO_DRIVER_TYPE_INTERRUPTS)
                {
                    /* disable tx interrupts */
                    FLEXIO_HAL_SetShifterInterrupt(baseAddr, (1U << TX_SHIFTER(resourceIndex)), false);
                    FLEXIO_HAL_SetShifterErrorInterrupt(baseAddr, (1U << TX_SHIFTER(resourceIndex)), false);
                    /* Enable rx interrupt to signal end of transfer */
                    FLEXIO_HAL_SetShifterInterrupt(baseAddr, (1U << RX_SHIFTER(resourceIndex)), true);
                }
                master->txData = NULL;
            }
        }
    }

    /* Check if transfer is over */
    if (master->rxRemainingBytes == 0U)
    {
        /* Record success if there was no error */
        if (master->status == FLEXIO_STATUS_BUSY)
        {
            master->status = FLEXIO_STATUS_SUCCESS;
        }
        /* End transfer. Use forced stop because timers are set to never disable. */
        FLEXIO_I2S_DRV_MasterStopTransfer(master);
        /* Call callback to announce the event to the user */
        if (master->callback != NULL)
        {
            master->callback(master, FLEXIO_EVENT_END_TRANSFER, master->callbackParam);
        }
        /* Signal transfer end for blocking transfers */
        if (master->blocking == true)
        {
            (void)OSIF_SemaPost(&(master->idleSemaphore));
        }
    }
}


/*FUNCTION**********************************************************************
 *
 * Function Name : FLEXIO_I2S_DRV_MasterEndDmaTxTransfer
 * Description   : function called at the end of a DMA Tx transfer
 *
 *END**************************************************************************/
static void FLEXIO_I2S_DRV_MasterEndDmaTxTransfer(void *stateStruct, edma_chn_status_t status)
{
    flexio_i2s_master_state_t *master;
    uint8_t dmaChn;
    DMA_Type *edmaBase;

#ifdef DEV_ERROR_DETECT
    DEV_ASSERT(stateStruct != NULL);
#endif

    (void)status;
    master = (flexio_i2s_master_state_t *)stateStruct;

    /* Call callback to allow user to provide a new buffer */
    if (master->callback != NULL)
    {
        master->callback(master, FLEXIO_EVENT_TX_EMPTY, master->callbackParam);
    }
    if (master->txRemainingBytes == 0U)
    {
        /* No more data to transmit, transmission will stop */
        master->txData = NULL;
    }
    else
    {
        /* There is more data to transfer, restart DMA channel */
        /* Update buffer address and size */
        dmaChn = master->txDMAChannel.channel;
        edmaBase = g_edmaBase[0U];
        EDMA_HAL_TCDSetSrcAddr(edmaBase, dmaChn, (uint32_t)(master->txData));
        EDMA_HAL_TCDSetMajorCount(edmaBase, dmaChn, master->txRemainingBytes / master->byteWidth);
        /* Now that this tx is set up, clear remaining bytes count */
        master->txRemainingBytes = 0U;
        /* Start the channel */
        (void)EDMA_DRV_StartChannel(&(master->txDMAChannel));
    }
}


/*FUNCTION**********************************************************************
 *
 * Function Name : FLEXIO_I2S_DRV_MasterEndDmaRxTransfer
 * Description   : function called at the end of a DMA Tx transfer
 *
 *END**************************************************************************/
static void FLEXIO_I2S_DRV_MasterEndDmaRxTransfer(void *stateStruct, edma_chn_status_t status)
{
    flexio_i2s_master_state_t *master;
    uint8_t dmaChn;
    DMA_Type *edmaBase;
    uint32_t addr;

#ifdef DEV_ERROR_DETECT
    DEV_ASSERT(stateStruct != NULL);
#endif

    (void)status;
    master = (flexio_i2s_master_state_t *)stateStruct;

    /* If this was a reception, call callback to allow user to provide a new buffer */
    if ((master->rxData != NULL) && (master->callback != NULL))
    {
        master->callback(master, FLEXIO_EVENT_RX_FULL, master->callbackParam);
    }
    if (master->rxRemainingBytes == 0U)
    {
        /* No more data, end transfer */
        master->status = FLEXIO_STATUS_SUCCESS;
        /* End transfer. Use forced stop because timers are set to never disable. */
        FLEXIO_I2S_DRV_MasterStopTransfer(master);
        /* Call callback to announce the event to the user */
        if (master->callback != NULL)
        {
            master->callback(master, FLEXIO_EVENT_END_TRANSFER, master->callbackParam);
        }
        /* Signal transfer end for blocking transfers */
        if (master->blocking == true)
        {
            (void)OSIF_SemaPost(&(master->idleSemaphore));
        }
    }
    else
    {
        /* There is more data to transfer, restart DMA channel */
        /* Update buffer address and size */
        dmaChn = master->rxDMAChannel.channel;
        edmaBase = g_edmaBase[0U];
        if (master->rxData != NULL)
        {
            addr = (uint32_t)(master->rxData);
        }
        else
        {
            /* if there is no data to receive, use dummy data as destination for DMA transfer */
            addr = (uint32_t)(&(master->dummyDmaData));
        }
        EDMA_HAL_TCDSetDestAddr(edmaBase, dmaChn, addr);
        EDMA_HAL_TCDSetMajorCount(edmaBase, dmaChn, master->rxRemainingBytes / master->byteWidth);
        /* Now that this rx is set up, clear remaining bytes count */
        master->rxRemainingBytes = 0U;
        /* Start the channel */
        (void)EDMA_DRV_StartChannel(&(master->rxDMAChannel));
    }
}


/*FUNCTION**********************************************************************
 *
 * Function Name : FLEXIO_I2S_DRV_MasterComputeTxRegAddr
 * Description   : Computes the address of the register used for DMA tx transfer
 *
 *END**************************************************************************/
static uint32_t FLEXIO_I2S_DRV_MasterComputeTxRegAddr(flexio_i2s_master_state_t *master)
{
    uint32_t addr;
    FLEXIO_Type *baseAddr;
    uint8_t shifter;

    baseAddr = g_flexioBase[master->flexioCommon.instance];
    shifter = TX_SHIFTER(master->flexioCommon.resourceIndex);
    addr = (uint32_t)(&(baseAddr->SHIFTBUFBIS[shifter])) + (sizeof(uint32_t) - master->byteWidth);
    return addr;
}


/*FUNCTION**********************************************************************
 *
 * Function Name : FLEXIO_I2S_DRV_MasterComputeRxRegAddr
 * Description   : Computes the address of the register used for DMA rx transfer
 *
 *END**************************************************************************/
static uint32_t FLEXIO_I2S_DRV_MasterComputeRxRegAddr(flexio_i2s_master_state_t *master)
{
    uint32_t addr;
    FLEXIO_Type *baseAddr;
    uint8_t shifter;

    baseAddr = g_flexioBase[master->flexioCommon.instance];
    shifter = RX_SHIFTER(master->flexioCommon.resourceIndex);
    addr = (uint32_t)(&(baseAddr->SHIFTBUFBIS[shifter]));
    return addr;
}


/*FUNCTION**********************************************************************
 *
 * Function Name : FLEXIO_I2S_DRV_MasterStartDmaTransfer
 * Description   : Starts a DMA transfer
 *
 *END**************************************************************************/
static flexio_status_t FLEXIO_I2S_DRV_MasterStartDmaTransfer(flexio_i2s_master_state_t *master)
{
    dma_request_source_t dmaReq;
    uint32_t instance;
    uint8_t resourceIndex;    /* Index of first used internal resource instance (shifter and timer) */
    uint8_t dmaChn;
    DMA_Type *edmaBase = g_edmaBase[0U];
    FLEXIO_Type *baseAddr;
    uint32_t addr;
    edma_status_t dmaStatus;
    uint8_t requestMask = 0U;

    baseAddr = g_flexioBase[master->flexioCommon.instance];
    resourceIndex = master->flexioCommon.resourceIndex;
    instance = master->flexioCommon.instance;

    /* Configure Tx channel if this is a transmission */
    if (master->txData != NULL)
    {
        /* Request a DMA channel for transmission */
        dmaReq = g_flexioDMASrc[instance][TX_SHIFTER(resourceIndex)];
        dmaStatus = EDMA_DRV_RequestChannel((uint8_t)EDMA_ANY_CHANNEL, dmaReq, &(master->txDMAChannel), &dmaChn);
        if (dmaStatus == EDMA_STATUS_FAIL)
        {
            return FLEXIO_STATUS_FAIL;
        }
        /* Configure the transfer control descriptor for the previously allocated channel */
        (void)EDMA_DRV_ConfigSingleBlockTransfer(&(master->txDMAChannel), 
                                                 EDMA_TRANSFER_MEM2PERIPH, 
                                                 (uint32_t)(master->txData),
                                                 FLEXIO_I2S_DRV_MasterComputeTxRegAddr(master), 
                                                 dmaTransferSize[master->byteWidth - 1U], 
                                                 master->byteWidth);
        EDMA_HAL_TCDSetMajorCount(edmaBase, dmaChn, master->txRemainingBytes / master->byteWidth);
        EDMA_HAL_TCDSetDisableDmaRequestAfterTCDDoneCmd(edmaBase, dmaChn, true);
        /* Now that this tx is set up, clear remaining bytes count */
        master->txRemainingBytes = 0U;

        /* Setup callback for DMA tx transfer end */
        (void)EDMA_DRV_InstallCallback(&(master->txDMAChannel),
                                       (edma_callback_t)(FLEXIO_I2S_DRV_MasterEndDmaTxTransfer),
                                       (void*)(master));
        /* Start tx DMA channel */
        (void)EDMA_DRV_StartChannel(&(master->txDMAChannel));
        requestMask = 1U << TX_SHIFTER(resourceIndex);
    }
    /* Configure Rx channel; if this is a transmission we still use Rx for the "end transfer" event */
    if (master->rxData != NULL)
    {
        addr = (uint32_t)(master->rxData);
    }
    else
    {
        /* if there is no data to receive, use dummy data as destination for DMA transfer */
        addr = (uint32_t)(&(master->dummyDmaData));
    }
    /* Request a DMA channel for reception */
    dmaReq = g_flexioDMASrc[instance][RX_SHIFTER(resourceIndex)];
    dmaStatus = EDMA_DRV_RequestChannel((uint8_t)EDMA_ANY_CHANNEL, dmaReq, &(master->rxDMAChannel), &dmaChn);
    if (dmaStatus == EDMA_STATUS_FAIL)
    {
        /* Release Tx channel if it was allocated */
        if (master->txData != NULL)
        {
            (void)EDMA_DRV_ReleaseChannel(&(master->txDMAChannel));
        }
        return FLEXIO_STATUS_FAIL;
    }
    /* Configure the transfer control descriptor for the previously allocated channel */
    (void)EDMA_DRV_ConfigSingleBlockTransfer(&(master->rxDMAChannel), 
                                             EDMA_TRANSFER_PERIPH2MEM, 
                                             FLEXIO_I2S_DRV_MasterComputeRxRegAddr(master), 
                                             addr,
                                             dmaTransferSize[master->byteWidth - 1U], 
                                             master->byteWidth);
    if (master->rxData == NULL)
    {
        /* if there is no data to receive, don't increment destination offset */
        EDMA_HAL_TCDSetDestOffset(edmaBase, dmaChn, 0);
    }
    EDMA_HAL_TCDSetMajorCount(edmaBase, dmaChn, master->rxRemainingBytes / master->byteWidth);
    EDMA_HAL_TCDSetDisableDmaRequestAfterTCDDoneCmd(edmaBase, dmaChn, true);
    /* Now that this rx is set up, clear remaining bytes count */
    master->rxRemainingBytes = 0U;

    /* Setup callback for DMA rx transfer end */
    (void)EDMA_DRV_InstallCallback(&(master->rxDMAChannel),
                                   (edma_callback_t)(FLEXIO_I2S_DRV_MasterEndDmaRxTransfer),
                                   (void*)(master));
    /* Start rx DMA channel */
    (void)EDMA_DRV_StartChannel(&(master->rxDMAChannel));
    requestMask |= 1U << RX_SHIFTER(resourceIndex);

    /* Enable FlexIO DMA requests */
    FLEXIO_HAL_SetShifterDMARequest(baseAddr, requestMask, true);

    return FLEXIO_STATUS_SUCCESS;
}

/*! @endcond */

/*******************************************************************************
 * Code
 ******************************************************************************/

/*FUNCTION**********************************************************************
 *
 * Function Name : FLEXIO_I2S_DRV_MasterInit
 * Description   : Initialize the FLEXIO_I2S master mode driver
 *
 *END**************************************************************************/
flexio_status_t FLEXIO_I2S_DRV_MasterInit(uint32_t instance,
                                          const flexio_i2s_master_user_config_t * userConfigPtr,
                                          flexio_i2s_master_state_t * master)
{
    uint32_t inputClock;
    clock_manager_error_code_t clkErr;
    flexio_status_t retCode;

#ifdef DEV_ERROR_DETECT
    DEV_ASSERT(master != NULL);
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
    if (OSIF_SemaCreate(&(master->idleSemaphore), 0U) == OSIF_STATUS_FAIL)
    {
        return FLEXIO_STATUS_FAIL;
    }

    /* Instruct the resource allocator that we need two shifters/timers */
    master->flexioCommon.resourceCount = 2U;
    /* Common FlexIO driver initialization */
    retCode = FLEXIO_DRV_InitDriver(instance, (flexio_common_state_t *)master);
    if (retCode != FLEXIO_STATUS_SUCCESS)
    {   /* Initialization failed, not enough resources */
        (void)OSIF_SemaDestroy(&(master->idleSemaphore));
        return retCode;
    }

    /* Initialize driver-specific context structure */
    master->driverType = userConfigPtr->driverType;
    master->bitsWidth = userConfigPtr->bitsWidth;
    master->byteWidth = FLEXIO_I2S_DRV_ComputeByteWidth(userConfigPtr->bitsWidth);
    master->driverIdle = true;
    master->callback = userConfigPtr->callback;
    master->callbackParam = userConfigPtr->callbackParam;
    master->blocking = false;
    master->txData = NULL;
    master->txRemainingBytes = 0U;
    master->rxData = NULL;
    master->rxRemainingBytes = 0U;
    master->master = true;
    master->status = FLEXIO_STATUS_SUCCESS;

    /* Configure device for I2S mode */
    FLEXIO_I2S_DRV_MasterConfigure(master, userConfigPtr, inputClock);

    /* Set up transfer engine */
    switch (master->driverType)
    {
        case FLEXIO_DRIVER_TYPE_INTERRUPTS:
            master->flexioCommon.isr = FLEXIO_I2S_DRV_MasterCheckStatus;
            break;
        case FLEXIO_DRIVER_TYPE_POLLING:
            /* Nothing to do here, FLEXIO_I2S_DRV_MasterGetStatus() will handle the transfer */
            break;
        case FLEXIO_DRIVER_TYPE_DMA:
            /* Nothing to do here, DMA channels are allocated when a transfer is initiated */
            break;
        default: 
            /* Impossible type - do nothing */
            break;
    }

    return FLEXIO_STATUS_SUCCESS;
}


/*FUNCTION**********************************************************************
 *
 * Function Name : FLEXIO_I2S_DRV_MasterDeinit
 * Description   : De-initialize the FLEXIO_I2S master mode driver
 *
 *END**************************************************************************/
flexio_status_t FLEXIO_I2S_DRV_MasterDeinit(flexio_i2s_master_state_t *master)
{
#ifdef DEV_ERROR_DETECT
    DEV_ASSERT(master != NULL);
#endif

    /* Check if driver is busy */
    if (!master->driverIdle)
    {
        return FLEXIO_STATUS_BUSY;
    }

    /* Destroy the semaphore */
    (void)OSIF_SemaDestroy(&(master->idleSemaphore));

    return FLEXIO_DRV_DeinitDriver((flexio_common_state_t *)master);
}


/*FUNCTION**********************************************************************
 *
 * Function Name : FLEXIO_I2S_DRV_MasterSetConfig
 * Description   : Set the baud rate and bit width for any subsequent I2S transfer
 *
 *END**************************************************************************/
flexio_status_t FLEXIO_I2S_DRV_MasterSetConfig(flexio_i2s_master_state_t * master, 
                                               uint32_t baudRate, 
                                               uint8_t bitsWidth)
{
    FLEXIO_Type *baseAddr;
    uint16_t divider;
    uint32_t inputClock;
    clock_manager_error_code_t clkErr;
    uint8_t resourceIndex;    /* Index of first used internal resource instance (shifter and timer) */

#ifdef DEV_ERROR_DETECT
    DEV_ASSERT(master != NULL);
    DEV_ASSERT(bitsWidth <= 32U);
    /* for DMA transfers bitsWidth must 8, 16, or 32 */
    DEV_ASSERT(!((master->driverType == FLEXIO_DRIVER_TYPE_DMA) && 
                 (bitsWidth != 8U) && (bitsWidth != 16U) && (bitsWidth != 32U)
                ));
#endif

    baseAddr = g_flexioBase[master->flexioCommon.instance];
    resourceIndex = master->flexioCommon.resourceIndex;

    /* Check if driver is busy */
    if (!master->driverIdle)
    {
        return FLEXIO_STATUS_BUSY;
    }
    /* Get the protocol clock frequency */
    clkErr = CLOCK_SYS_GetFreq(g_flexioClock[master->flexioCommon.instance], &inputClock);
    if ((clkErr != CLOCK_MANAGER_SUCCESS) || (inputClock == 0U))
    {
        /* No clock configured for this module */
        return FLEXIO_STATUS_FAIL;
    }

    /* Compute divider */
    FLEXIO_I2S_DRV_MasterComputeBaudRateDivider(master, baudRate, &divider, inputClock);

    /* Configure SCK timer */
    FLEXIO_HAL_SetTimerCompare(baseAddr, SCK_TIMER(resourceIndex), (((bitsWidth << 1U) - 1U) << 8U) + divider);
    /* Configure WS timer */
    FLEXIO_HAL_SetTimerCompare(baseAddr, WS_TIMER(resourceIndex), bitsWidth * ((divider + 1U) * 2U) - 1U);

    master->bitsWidth = bitsWidth;
    master->byteWidth = FLEXIO_I2S_DRV_ComputeByteWidth(bitsWidth);

    return FLEXIO_STATUS_SUCCESS;
}


/*FUNCTION**********************************************************************
 *
 * Function Name : FLEXIO_I2S_DRV_MasterGetBaudRate
 * Description   : Get the currently configured baud rate
 *
 *END**************************************************************************/
flexio_status_t FLEXIO_I2S_DRV_MasterGetBaudRate(flexio_i2s_master_state_t *master, uint32_t *baudRate)
{
    FLEXIO_Type *baseAddr;
    uint32_t inputClock;
    uint32_t divider;
    uint16_t timerCmp;
    clock_manager_error_code_t clkErr;
    uint8_t resourceIndex;    /* Index of first used internal resource instance (shifter and timer) */

#ifdef DEV_ERROR_DETECT
    DEV_ASSERT(master != NULL);
#endif

    baseAddr = g_flexioBase[master->flexioCommon.instance];
    resourceIndex = master->flexioCommon.resourceIndex;

    /* Get the protocol clock frequency */
    clkErr = CLOCK_SYS_GetFreq(g_flexioClock[master->flexioCommon.instance], &inputClock);
    if ((clkErr != CLOCK_MANAGER_SUCCESS) || (inputClock == 0U))
    {
        /* No clock configured for this module */
        return FLEXIO_STATUS_FAIL;
    }

    /* Get the currently configured divider */
    timerCmp = FLEXIO_HAL_GetTimerCompare(baseAddr, SCK_TIMER(resourceIndex));
    divider = (uint32_t)(timerCmp & 0x00FFU);

    /* Compute baud rate: input_clock / (2 * (divider + 1)). Round to nearest integer */
    *baudRate = (inputClock + divider + 1U) / (2U * (divider + 1U));

    return FLEXIO_STATUS_SUCCESS;
}



/*FUNCTION**********************************************************************
 *
 * Function Name : FLEXIO_I2S_DRV_MasterSendData
 * Description   : Perform a non-blocking send transaction on the I2S bus
 *
 *END**************************************************************************/
flexio_status_t FLEXIO_I2S_DRV_MasterSendData(flexio_i2s_master_state_t *master,
                                              const uint8_t * txBuff,
                                              uint32_t txSize)
{
    FLEXIO_Type *baseAddr;
    uint8_t resourceIndex;    /* Index of first used internal resource instance (shifter and timer) */
    flexio_status_t retCode;
    flexio_status_t status;

#ifdef DEV_ERROR_DETECT
    DEV_ASSERT(master != NULL);
    DEV_ASSERT(txBuff != NULL);
    DEV_ASSERT(txSize > 0U);
#endif

    baseAddr = g_flexioBase[master->flexioCommon.instance];
    resourceIndex = master->flexioCommon.resourceIndex;

    /* Check if driver is busy */
    if (!master->driverIdle)
    {
        return FLEXIO_STATUS_BUSY;
    }
                                     
    status = master->status;
    master->txData = txBuff;
    master->txRemainingBytes = txSize;
    /* also count received data, it tells us when tx is actually completed */
    master->rxRemainingBytes = txSize;
    master->rxData = NULL;
    master->status = FLEXIO_STATUS_BUSY;
    master->driverIdle = false;

    /* Enable transfer engine */
    switch (master->driverType)
    {
        case FLEXIO_DRIVER_TYPE_INTERRUPTS:
            /* Enable interrupts for Tx shifter */
            FLEXIO_HAL_SetShifterInterrupt(baseAddr, 
                                 (1U << TX_SHIFTER(resourceIndex)), 
                                 true);
            FLEXIO_HAL_SetShifterErrorInterrupt(baseAddr, 
                                 (1U << TX_SHIFTER(resourceIndex)), 
                                 true);
            break;
        case FLEXIO_DRIVER_TYPE_POLLING:
            /* Call FLEXIO_I2S_DRV_MasterCheckStatus once to send the first byte */
            FLEXIO_I2S_DRV_MasterCheckStatus(master);
            break;
        case FLEXIO_DRIVER_TYPE_DMA:
            retCode = FLEXIO_I2S_DRV_MasterStartDmaTransfer(master);
            if (retCode != FLEXIO_STATUS_SUCCESS)
            {
                /* Restore status and idle indicator */
                master->status = status;
                master->driverIdle = true;
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
 * Function Name : FLEXIO_I2S_DRV_MasterSendDataBlocking
 * Description   : Perform a blocking send transaction on the I2S bus
 *
 *END**************************************************************************/
flexio_status_t FLEXIO_I2S_DRV_MasterSendDataBlocking(flexio_i2s_master_state_t *master,
                                                          const uint8_t * txBuff,
                                                          uint32_t txSize,
                                                          uint32_t timeout)
{
    flexio_status_t status;

    /* mark transfer as blocking */
    if (master->driverType != FLEXIO_DRIVER_TYPE_POLLING)
    {
        master->blocking = true;
    }
    /* Call FLEXIO_I2S_DRV_MasterSendData to start transfer */
    status = FLEXIO_I2S_DRV_MasterSendData(master, txBuff, txSize);
    if (status != FLEXIO_STATUS_SUCCESS)
    {
        /* Transfer could not be started */
        master->blocking = false;
        return status; 
    }

    /* Wait for transfer to end */
    return FLEXIO_I2S_DRV_MasterWaitTransferEnd(master, timeout);
}


/*FUNCTION**********************************************************************
 *
 * Function Name : FLEXIO_I2S_DRV_MasterReceiveData
 * Description   : Perform a non-blocking receive transaction on the I2S bus
 *
 *END**************************************************************************/
flexio_status_t FLEXIO_I2S_DRV_MasterReceiveData(flexio_i2s_master_state_t *master,
                                                     uint8_t * rxBuff,
                                                     uint32_t rxSize)
{
    FLEXIO_Type *baseAddr;
    uint8_t resourceIndex;    /* Index of first used internal resource instance (shifter and timer) */
    flexio_status_t retCode;
    flexio_status_t status;

#ifdef DEV_ERROR_DETECT
    DEV_ASSERT(master != NULL);
    DEV_ASSERT(rxBuff != NULL);
    DEV_ASSERT(rxSize > 0U);
#endif

    baseAddr = g_flexioBase[master->flexioCommon.instance];
    resourceIndex = master->flexioCommon.resourceIndex;

    /* Check if driver is busy */
    if (!master->driverIdle)
    {
        return FLEXIO_STATUS_BUSY;
    }

    status = master->status;
    master->rxData = rxBuff;
    master->rxRemainingBytes = rxSize;
    master->txData = NULL;
    master->txRemainingBytes = 0U;
    master->status = FLEXIO_STATUS_BUSY;
    master->driverIdle = false;

    /* Enable transfer engine */
    switch (master->driverType)
    {
        case FLEXIO_DRIVER_TYPE_INTERRUPTS:
            /* Enable interrupts for Rx shifter */
            FLEXIO_HAL_SetShifterInterrupt(baseAddr, 
                                 (1U << RX_SHIFTER(resourceIndex)), 
                                 true);
            FLEXIO_HAL_SetShifterErrorInterrupt(baseAddr, 
                                 (1U << RX_SHIFTER(resourceIndex)), 
                                 true);
            break;
        case FLEXIO_DRIVER_TYPE_POLLING:
            /* Call FLEXIO_I2S_DRV_MasterCheckStatus once to send the first byte */
            FLEXIO_I2S_DRV_MasterCheckStatus(master);
            break;
        case FLEXIO_DRIVER_TYPE_DMA:
            retCode = FLEXIO_I2S_DRV_MasterStartDmaTransfer(master);
            if (retCode != FLEXIO_STATUS_SUCCESS)
            {
                /* Restore status and idle indicator */
                master->status = status;
                master->driverIdle = true;
                return retCode;
            }
            break;
        default: 
            /* Impossible type - do nothing */
            break;
    }

    /* For master we need to send a dummy char to start the clock. 
       For slave just put a 0 in the buffer to keep the tx line clear while receiving. */
    FLEXIO_HAL_WriteShifterBuffer(baseAddr, TX_SHIFTER(resourceIndex), 0x0, FLEXIO_SHIFTER_RW_MODE_BIT_SWAP);

    return FLEXIO_STATUS_SUCCESS;
}


/*FUNCTION**********************************************************************
 *
 * Function Name : FLEXIO_I2S_DRV_MasterReceiveDataBlocking
 * Description   : Perform a blocking receive transaction on the I2S bus
 *
 *END**************************************************************************/
flexio_status_t FLEXIO_I2S_DRV_MasterReceiveDataBlocking(flexio_i2s_master_state_t *master,
                                                        uint8_t * rxBuff,
                                                        uint32_t rxSize,
                                                        uint32_t timeout)
{
    flexio_status_t status;

    /* mark transfer as blocking */
    if (master->driverType != FLEXIO_DRIVER_TYPE_POLLING)
    {
        master->blocking = true;
    }
    /* Call FLEXIO_I2S_DRV_MasterReceiveData to start transfer */
    status = FLEXIO_I2S_DRV_MasterReceiveData(master, rxBuff, rxSize);
    if (status != FLEXIO_STATUS_SUCCESS)
    {
        /* Transfer could not be started */
        master->blocking = false;
        return status; 
    }

    /* Wait for transfer to end */
    return FLEXIO_I2S_DRV_MasterWaitTransferEnd(master, timeout);
}


/*FUNCTION**********************************************************************
 *
 * Function Name : FLEXIO_I2S_DRV_MasterTransferAbort
 * Description   : Aborts a non-blocking I2S master transaction
 *
 *END**************************************************************************/
flexio_status_t FLEXIO_I2S_DRV_MasterTransferAbort(flexio_i2s_master_state_t *master)
{
#ifdef DEV_ERROR_DETECT
    DEV_ASSERT(master != NULL);
#endif

    /* Check if driver is busy */
    if (master->driverIdle)
    {
        return FLEXIO_STATUS_FAIL;
    }

    master->status = FLEXIO_STATUS_ABORTED;
    FLEXIO_I2S_DRV_MasterStopTransfer(master);

    return FLEXIO_STATUS_SUCCESS;
}


/*FUNCTION**********************************************************************
 *
 * Function Name : FLEXIO_I2S_DRV_MasterGetStatus
 * Description   : Get the status of the current non-blocking I2S master transaction
 *
 *END**************************************************************************/
flexio_status_t FLEXIO_I2S_DRV_MasterGetStatus(flexio_i2s_master_state_t *master, uint32_t *bytesRemaining)
{
#ifdef DEV_ERROR_DETECT
    DEV_ASSERT(master != NULL);
#endif

    if ((!master->driverIdle) && (master->driverType == FLEXIO_DRIVER_TYPE_POLLING))
    {
        /* In polling mode advance the I2S transfer here */
        FLEXIO_I2S_DRV_MasterCheckStatus(master);
    }

    if (bytesRemaining != NULL)
    {
        /* Use rxRemainingBytes even for transmit; byte is not transmitted 
           until rx shifter reports a receive event */
        *bytesRemaining = master->rxRemainingBytes;
    }

    if (!master->driverIdle)
    {
        return FLEXIO_STATUS_BUSY;
    }
    else
    {
        return master->status;
    }
}


/*FUNCTION**********************************************************************
 *
 * Function Name : FLEXIO_I2S_DRV_MasterSetRxBuffer
 * Description   : Provide a buffer for receiving data.
 *
 *END**************************************************************************/
flexio_status_t FLEXIO_I2S_DRV_MasterSetRxBuffer(flexio_i2s_master_state_t *master,
                                                 uint8_t * rxBuff,
                                                 uint32_t rxSize)
{
#ifdef DEV_ERROR_DETECT
    DEV_ASSERT(master != NULL);
    DEV_ASSERT(rxBuff != NULL);
    DEV_ASSERT(rxSize > 0U);
#endif
    master->rxData = rxBuff;
    master->rxRemainingBytes = rxSize;

    return FLEXIO_STATUS_SUCCESS;
}


/*FUNCTION**********************************************************************
 *
 * Function Name : FLEXIO_I2S_DRV_MasterSetTxBuffer
 * Description   : Provide a buffer for transmitting data.
 *
 *END**************************************************************************/
flexio_status_t FLEXIO_I2S_DRV_MasterSetTxBuffer(flexio_i2s_master_state_t *master,
                                                 uint8_t * txBuff,
                                                 uint32_t txSize)
{
#ifdef DEV_ERROR_DETECT
    DEV_ASSERT(master != NULL);
    DEV_ASSERT(txBuff != NULL);
    DEV_ASSERT(txSize > 0U);
#endif
    master->txData = txBuff;
    master->txRemainingBytes = txSize;
    /* for transmit we also count received bytes for end condition */
    master->rxRemainingBytes += txSize;

    return FLEXIO_STATUS_SUCCESS;
}


/*FUNCTION**********************************************************************
 *
 * Function Name : FLEXIO_I2S_DRV_SlaveInit
 * Description   : Initialize the FLEXIO_I2S slave mode driver
 *
 *END**************************************************************************/
flexio_status_t FLEXIO_I2S_DRV_SlaveInit(uint32_t instance,
                                          const flexio_i2s_slave_user_config_t * userConfigPtr,
                                          flexio_i2s_slave_state_t * slave)
{
    flexio_status_t retCode;

#ifdef DEV_ERROR_DETECT
    DEV_ASSERT(slave != NULL);
    DEV_ASSERT(instance < FLEXIO_INSTANCE_COUNT);
    /* Check that device was initialized */
    DEV_ASSERT(g_flexioDeviceStatePtr[instance] != NULL);
#endif

    /* Initialize the semaphore */
    if (OSIF_SemaCreate(&(slave->idleSemaphore), 0U) == OSIF_STATUS_FAIL)
    {
        return FLEXIO_STATUS_FAIL;
    }

    /* Instruct the resource allocator that we need two shifters/timers */
    slave->flexioCommon.resourceCount = 2U;
    /* Common FlexIO driver initialization */
    retCode = FLEXIO_DRV_InitDriver(instance, (flexio_common_state_t *)slave);
    if (retCode != FLEXIO_STATUS_SUCCESS)
    {   /* Initialization failed, not enough resources */
        (void)OSIF_SemaDestroy(&(slave->idleSemaphore));
        return retCode;
    }

    /* Initialize driver-specific context structure */
    slave->driverType = userConfigPtr->driverType;
    slave->bitsWidth = userConfigPtr->bitsWidth;
    slave->byteWidth = FLEXIO_I2S_DRV_ComputeByteWidth(userConfigPtr->bitsWidth);
    slave->driverIdle = true;
    slave->callback = userConfigPtr->callback;
    slave->callbackParam = userConfigPtr->callbackParam;
    slave->blocking = false;
    slave->txData = NULL;
    slave->txRemainingBytes = 0U;
    slave->rxData = NULL;
    slave->rxRemainingBytes = 0U;
    slave->master = false;
    slave->status = FLEXIO_STATUS_SUCCESS;

    /* Configure device for I2S mode */
    FLEXIO_I2S_DRV_SlaveConfigure(slave, userConfigPtr);

    /* Set up transfer engine */
    switch (slave->driverType)
    {
        case FLEXIO_DRIVER_TYPE_INTERRUPTS:
            slave->flexioCommon.isr = FLEXIO_I2S_DRV_MasterCheckStatus;
            break;
        case FLEXIO_DRIVER_TYPE_POLLING:
            /* Nothing to do here, FLEXIO_I2S_DRV_MasterGetStatus() will handle the transfer */
            break;
        case FLEXIO_DRIVER_TYPE_DMA:
            /* Nothing to do here, DMA channels are allocated when a transfer is initiated */
            break;
        default: 
            /* Impossible type - do nothing */
            break;
    }

    return FLEXIO_STATUS_SUCCESS;
}


/*FUNCTION**********************************************************************
 *
 * Function Name : FLEXIO_I2S_DRV_SlaveSetConfig
 * Description   : Set the baud rate and bit width for any subsequent I2S transfer
 *
 *END**************************************************************************/
flexio_status_t FLEXIO_I2S_DRV_SlaveSetConfig(flexio_i2s_slave_state_t * slave, 
                                               uint8_t bitsWidth)
{
    FLEXIO_Type *baseAddr;
    uint8_t resourceIndex;    /* Index of first used internal resource instance (shifter and timer) */

#ifdef DEV_ERROR_DETECT
    DEV_ASSERT(slave != NULL);
    DEV_ASSERT(bitsWidth <= 32U);
    /* for DMA transfers bitsWidth must 8, 16, or 32 */
    DEV_ASSERT(!((slave->driverType == FLEXIO_DRIVER_TYPE_DMA) && 
                 (bitsWidth != 8U) && (bitsWidth != 16U) && (bitsWidth != 32U)
                ));
#endif

    baseAddr = g_flexioBase[slave->flexioCommon.instance];
    resourceIndex = slave->flexioCommon.resourceIndex;

    /* Check if driver is busy */
    if (!slave->driverIdle)
    {
        return FLEXIO_STATUS_BUSY;
    }

    slave->bitsWidth = bitsWidth;
    slave->byteWidth = FLEXIO_I2S_DRV_ComputeByteWidth(bitsWidth);

    /* Configure SCK timer */
    FLEXIO_HAL_SetTimerCompare(baseAddr, SCK_TIMER(resourceIndex), (bitsWidth << 2U) - 3U);
    /* Configure WS timer */
    FLEXIO_HAL_SetTimerCompare(baseAddr, WS_TIMER(resourceIndex), (bitsWidth << 1U) - 1U);

    return FLEXIO_STATUS_SUCCESS;
}


/*******************************************************************************
 * EOF
 ******************************************************************************/
