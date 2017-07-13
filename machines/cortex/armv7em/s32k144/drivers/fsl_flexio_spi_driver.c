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

#include "fsl_flexio_spi_driver.h"
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
 *
 * @section [global]
 * Violates MISRA 2012 Required Rule 1.3, Taking address of near auto variable
 * The code is not dynamically linked. An absolute stack address is obtained
 * when taking the address of the near auto variable. A source of error in
 * writing dynamic code is that the stack segment may be different from the data
 * segment.
 *
 * @section [global]
 * Violates MISRA 2012 Required Rule 11.3, Cast performed between a pointer to object type 
 * and a pointer to a different object type.
 * This is needed for callbacks from other modules, which cannot know the actual argument type (flexio_common, dma).
 * Also used in multi-byte transfers for ensuring efficient and endianess-independent data read/write.
 *
 * @section [global]
 * Violates MISRA 2012 Advisory Rule 11.4, Conversion between a pointer and
 * integer type.
 * This is required for working with the dma. Addresses are configured as uint32_t in the dma driver.
 *
 * @section [global]
 * Violates MISRA 2012 Advisory Rule 11.5, Conversion from pointer to void to pointer to other type
 * This is needed for callbacks from other modules, which cannot know the actual argument type (flexio_common, dma)
 *
 * @section [global]
 * Violates MISRA 2012 Required Rule 11.6, Cast from pointer to unsigned long.
 * This is required for working with the dma. Addresses are configured as uint32_t in the dma driver.
 *
 * @section [global]
 * Violates MISRA 2012 Advisory Rule 15.5, Return statement before end of function.
 * The return statement before end of function is used for simpler code 
 * structure and better readability.
 *
 * @section [global]
 * Violates MISRA 2012 Advisory Rule 8.13, Pointer parameter 'master' could be declared as pointing to const
 * This is a pointer to the driver context structure which is for internal use only, and the application
 * must make no assumption about the content of this structure. Therefore it is irrelevant for the application
 * whether the structure is changed in the function or not. The fact that in a particular implementation of some 
 * functions there is no write in the context structure is an implementation detail and there is no reason to 
 * propagate it in the interface. That would compromise the stability of the interface, if this implementation 
 * detail is changed in later releases or on other platforms.
 *
 * @section [global]
 * Violates MISRA 2012 Advisory Rule 8.7, External could be made static.
 * Function is defined for usage by application code.
 */

/*******************************************************************************
 * Variables
 ******************************************************************************/

/*! @cond DRIVER_INTERNAL_USE_ONLY */

    /* Constraints used for baud rate computation */
#define DIVIDER_MIN_VALUE 1U
#define DIVIDER_MAX_VALUE 0xFFU

    /* Shifters/Timers used for SPI simulation. The parameter x represents the 
       resourceIndex value for the current driver instance */
#define TX_SHIFTER(x)     (x)
#define RX_SHIFTER(x)     ((x)+1U)
#define SCK_TIMER(x)      (x)
#define SS_TIMER(x)       ((x)+1U)

    /* Dummy data to send when user provides no data */
#define FLEXIO_SPI_DUMMYDATA (0xFFFFFFFFU)

/*******************************************************************************
 * Private Functions
 ******************************************************************************/

/*FUNCTION**********************************************************************
 *
 * Function Name : FLEXIO_SPI_DRV_MasterComputeBaudRateDivider
 * Description   : Computes the baud rate divider for a target baud rate
 *
 *END**************************************************************************/
static void FLEXIO_SPI_DRV_MasterComputeBaudRateDivider(uint32_t baudRate, 
                                                        uint16_t *divider,
                                                        uint32_t inputClock)
{
    uint32_t tmpDiv;

    /* Compute divider: ((input_clock / baud_rate) / 2) - 1. Round to nearest integer */
    tmpDiv = ((inputClock + baudRate) / (2U * baudRate)) - 1U;
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
}


/*FUNCTION**********************************************************************
 *
 * Function Name : FLEXIO_SPI_DRV_MasterConfigure
 * Description   : configures the FLEXIO module as SPI master
 *
 *END**************************************************************************/
static void FLEXIO_SPI_DRV_MasterConfigure(const flexio_spi_master_state_t *master, 
                                           const flexio_spi_master_user_config_t * userConfigPtr,
                                           uint32_t inputClock)
{
    FLEXIO_Type *baseAddr;
    flexio_pin_polarity_t clockPolarity;        /* Polarity of clock signal */
    flexio_timer_polarity_t txShifterPolarity;  /* Polarity of MOSI shifter */
    flexio_timer_polarity_t rxShifterPolarity;  /* Polarity of MISO shifter */
    uint16_t divider;
    flexio_shifter_stop_t stopBit;
    flexio_shifter_start_t startBit;
    uint8_t resourceIndex;    /* Index of first used internal resource instance (shifter and timer) */

    baseAddr = g_flexioBase[master->flexioCommon.instance];
    resourceIndex = master->flexioCommon.resourceIndex;
    /* Compute divider.*/
    FLEXIO_SPI_DRV_MasterComputeBaudRateDivider(userConfigPtr->baudRate, &divider, inputClock);
    /* add number of bits in the upper 8 bits of the divider. Formula is: TIMCMP[15:8] = (number of bits x 2) - 1 */
    divider += (uint16_t)((((uint16_t)(userConfigPtr->transferSize) * 8U * 2U) - 1U) << 8U);

    if (userConfigPtr->clockPolarity == 0U)
    {
        /* CPOL = 0 */
        clockPolarity = FLEXIO_PIN_POLARITY_HIGH;
    }
    else
    {
        /* CPOL = 1 */
        clockPolarity = FLEXIO_PIN_POLARITY_LOW;
    }

    if (userConfigPtr->clockPhase == 0U)
    {
        /* CPHA = 0 */
        txShifterPolarity = FLEXIO_TIMER_POLARITY_NEGEDGE;
        rxShifterPolarity = FLEXIO_TIMER_POLARITY_POSEDGE;
        stopBit = FLEXIO_SHIFTER_STOP_BIT_DISABLED;
        startBit = FLEXIO_SHIFTER_START_BIT_DISABLED;
    }
    else
    {
        /* CPHA = 1 */
        txShifterPolarity = FLEXIO_TIMER_POLARITY_POSEDGE;
        rxShifterPolarity = FLEXIO_TIMER_POLARITY_NEGEDGE;
        stopBit = FLEXIO_SHIFTER_STOP_BIT_0;
        startBit = FLEXIO_SHIFTER_START_BIT_DISABLED_SH;
    }

    /* Configure Tx shifter (MOSI) */
    FLEXIO_HAL_SetShifterControl(baseAddr,
                                 TX_SHIFTER(resourceIndex),
                                 FLEXIO_SHIFTER_MODE_TRANSMIT,
                                 userConfigPtr->mosiPin,
                                 FLEXIO_PIN_POLARITY_HIGH,
                                 FLEXIO_PIN_CONFIG_OUTPUT,
                                 SCK_TIMER(resourceIndex),
                                 txShifterPolarity);
    FLEXIO_HAL_SetShifterConfig(baseAddr,
                                TX_SHIFTER(resourceIndex),
                                startBit,
                                stopBit,
                                FLEXIO_SHIFTER_SOURCE_PIN);

    /* Configure Rx shifter (MISO) */
    FLEXIO_HAL_SetShifterControl(baseAddr,
                                 RX_SHIFTER(resourceIndex),
                                 FLEXIO_SHIFTER_MODE_RECEIVE,
                                 userConfigPtr->misoPin,
                                 FLEXIO_PIN_POLARITY_HIGH,
                                 FLEXIO_PIN_CONFIG_DISABLED,
                                 SCK_TIMER(resourceIndex),
                                 rxShifterPolarity);
    FLEXIO_HAL_SetShifterConfig(baseAddr,
                                RX_SHIFTER(resourceIndex),
                                FLEXIO_SHIFTER_START_BIT_DISABLED,
                                FLEXIO_SHIFTER_STOP_BIT_DISABLED,
                                FLEXIO_SHIFTER_SOURCE_PIN);

    /* Configure sck timer */
    FLEXIO_HAL_SetTimerCompare(baseAddr, SCK_TIMER(resourceIndex), divider); /* set baud rate, and number of bits */
    FLEXIO_HAL_SetTimerConfig(baseAddr,
                              SCK_TIMER(resourceIndex),
                              FLEXIO_TIMER_START_BIT_ENABLED,
                              FLEXIO_TIMER_STOP_BIT_TIM_DIS,
                              FLEXIO_TIMER_ENABLE_TRG_HIGH,
                              FLEXIO_TIMER_DISABLE_TIM_CMP,
                              FLEXIO_TIMER_RESET_NEVER,
                              FLEXIO_TIMER_DECREMENT_CLK_SHIFT_TMR,
                              FLEXIO_TIMER_INITOUT_ZERO);
    FLEXIO_HAL_SetTimerControl(baseAddr,
                               SCK_TIMER(resourceIndex),
                               (TX_SHIFTER(resourceIndex) << 2U) + 1U,            /* trigger on tx shifter status flag */
                               FLEXIO_TRIGGER_POLARITY_LOW,
                               FLEXIO_TRIGGER_SOURCE_INTERNAL,
                               userConfigPtr->sckPin,                     /* output on clock pin */
                               clockPolarity,                      /* used configured polarity */
                               FLEXIO_PIN_CONFIG_OUTPUT,
                               FLEXIO_TIMER_MODE_8BIT_BAUD);

    /* Configure SS timer */
    FLEXIO_HAL_SetTimerCompare(baseAddr, SS_TIMER(resourceIndex), 0xFFFFU);
    FLEXIO_HAL_SetTimerConfig(baseAddr,
                              SS_TIMER(resourceIndex),
                              FLEXIO_TIMER_START_BIT_DISABLED,
                              FLEXIO_TIMER_STOP_BIT_DISABLED,
                              FLEXIO_TIMER_ENABLE_TIM_ENABLE,    /* enable when SCK timer is enabled */
                              FLEXIO_TIMER_DISABLE_TIM_DISABLE,  /* disable when SCK timer is disabled */
                              FLEXIO_TIMER_RESET_NEVER,
                              FLEXIO_TIMER_DECREMENT_CLK_SHIFT_TMR,
                              FLEXIO_TIMER_INITOUT_ONE);
    FLEXIO_HAL_SetTimerControl(baseAddr,
                               SS_TIMER(resourceIndex),
                               0U,                 /* trigger not used, using defaults */
                               FLEXIO_TRIGGER_POLARITY_HIGH,
                               FLEXIO_TRIGGER_SOURCE_EXTERNAL,
                               userConfigPtr->ssPin,
                               FLEXIO_PIN_POLARITY_LOW,
                               FLEXIO_PIN_CONFIG_OUTPUT,
                               FLEXIO_TIMER_MODE_16BIT);
}


/*FUNCTION**********************************************************************
 *
 * Function Name : FLEXIO_SPI_DRV_MasterEndTransfer
 * Description   : end a transfer
 *
 *END**************************************************************************/
static void FLEXIO_SPI_DRV_MasterEndTransfer(flexio_spi_master_state_t *master)
{
    FLEXIO_Type *baseAddr;
    uint8_t resourceIndex;    /* Index of first used internal resource instance (shifter and timer) */

#ifdef DEV_ERROR_DETECT
    DEV_ASSERT(master != NULL);
#endif

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
            /* Disable FlexIO DMA requests for both shifters */
            FLEXIO_HAL_SetShifterDMARequest(baseAddr, 
                     ((1U << TX_SHIFTER(resourceIndex)) | (1U << RX_SHIFTER(resourceIndex))),
                     false);

            /* Release the DMA channels */
            (void)EDMA_DRV_ReleaseChannel(&(master->txDMAChannel));
            (void)EDMA_DRV_ReleaseChannel(&(master->rxDMAChannel));
            break;
        default: 
            /* Impossible type - do nothing */
            break;
    }

    master->driverIdle = true;
    master->txRemainingBytes = 0U;
    master->rxRemainingBytes = 0U;

    /* Call callback to announce the event to the user */
    if (master->callback != NULL)
    {
        master->callback(master, FLEXIO_EVENT_END_TRANSFER, master->callbackParam);
    }
}


/*FUNCTION**********************************************************************
 *
 * Function Name : FLEXIO_SPI_DRV_MasterStopTransfer
 * Description   : Forcefully stops the current transfer
 *
 *END**************************************************************************/
static void FLEXIO_SPI_DRV_MasterStopTransfer(flexio_spi_master_state_t *master)
{
    FLEXIO_Type *baseAddr;
    uint8_t resourceIndex;    /* Index of first used internal resource instance (shifter and timer) */


    resourceIndex = master->flexioCommon.resourceIndex;
    baseAddr = g_flexioBase[master->flexioCommon.instance];

    /* disable and re-enable timers and shifters to reset them */
    FLEXIO_HAL_SetShifterMode(baseAddr, TX_SHIFTER(resourceIndex), FLEXIO_SHIFTER_MODE_DISABLED);
    FLEXIO_HAL_SetShifterMode(baseAddr, RX_SHIFTER(resourceIndex), FLEXIO_SHIFTER_MODE_DISABLED);
    FLEXIO_HAL_SetTimerMode(baseAddr, SCK_TIMER(resourceIndex), FLEXIO_TIMER_MODE_DISABLED);
    FLEXIO_HAL_SetTimerMode(baseAddr, SS_TIMER(resourceIndex), FLEXIO_TIMER_MODE_DISABLED);

    /* clear any leftover error flags */
    FLEXIO_HAL_ClearShifterErrorStatus(baseAddr, TX_SHIFTER(resourceIndex));
    FLEXIO_HAL_ClearShifterErrorStatus(baseAddr, RX_SHIFTER(resourceIndex));

    /* end the transfer */
    FLEXIO_SPI_DRV_MasterEndTransfer(master);

    /* re-enable timers and shifters */
    FLEXIO_HAL_SetShifterMode(baseAddr, TX_SHIFTER(resourceIndex), FLEXIO_SHIFTER_MODE_TRANSMIT);
    FLEXIO_HAL_SetShifterMode(baseAddr, RX_SHIFTER(resourceIndex), FLEXIO_SHIFTER_MODE_RECEIVE);
    if (master->master)
    {
        FLEXIO_HAL_SetTimerMode(baseAddr, SCK_TIMER(resourceIndex), FLEXIO_TIMER_MODE_8BIT_BAUD);
        FLEXIO_HAL_SetTimerMode(baseAddr, SS_TIMER(resourceIndex), FLEXIO_TIMER_MODE_16BIT);
    }
    else
    {
        FLEXIO_HAL_SetTimerMode(baseAddr, SCK_TIMER(resourceIndex), FLEXIO_TIMER_MODE_16BIT);
    }
}


/*FUNCTION**********************************************************************
 *
 * Function Name : FLEXIO_SPI_DRV_MasterWaitTransferEnd
 * Description   : waits for the end of a blocking transfer
 *
 *END**************************************************************************/
static flexio_status_t FLEXIO_SPI_DRV_MasterWaitTransferEnd(flexio_spi_master_state_t *master, uint32_t timeout)
{
    osif_status_t osifError = OSIF_STATUS_SUCCESS;
#ifdef DEV_ERROR_DETECT
    DEV_ASSERT(master != NULL);
#endif

    switch (master->driverType)
    {
        case FLEXIO_DRIVER_TYPE_INTERRUPTS:
            /* Wait for transfer to be completed by the IRQ */
            osifError = OSIF_SemaWait(&(master->idleSemaphore), timeout);
            break;
        case FLEXIO_DRIVER_TYPE_POLLING:
            /* Call FLEXIO_SPI_DRV_MasterGetStatus() to do the transfer */
            while (FLEXIO_SPI_DRV_MasterGetStatus(master, NULL) == FLEXIO_STATUS_BUSY)
            {
            }
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
        FLEXIO_SPI_DRV_MasterStopTransfer(master);
    }

    /* blocking transfer is over */
    master->blocking = false;
    return master->status;
}


/*FUNCTION**********************************************************************
 *
 * Function Name : FLEXIO_SPI_DRV_MasterEndDmaTransfer
 * Description   : end a DMA transfer
 *
 *END**************************************************************************/
static void FLEXIO_SPI_DRV_MasterEndDmaTransfer(void *stateStruct, edma_chn_status_t status)
{
    flexio_spi_master_state_t *master;

#ifdef DEV_ERROR_DETECT
    DEV_ASSERT(stateStruct != NULL);
#endif

    master = (flexio_spi_master_state_t *)stateStruct;

    /* Record success if there was no error */
    if (status == EDMA_CHN_ERROR)
    {
        master->status = FLEXIO_STATUS_FAIL;
    }
    else
    {
        master->status = FLEXIO_STATUS_SUCCESS;
    }
    FLEXIO_SPI_DRV_MasterEndTransfer(master);
    /* Signal transfer end for blocking transfers */
    if (master->blocking == true)
    {
        (void)OSIF_SemaPost(&(master->idleSemaphore));
    }
}


/*FUNCTION**********************************************************************
 *
 * Function Name : FLEXIO_SPI_DRV_SlaveConfigure
 * Description   : configures the FLEXIO module as SPI slave
 *
 *END**************************************************************************/
static void FLEXIO_SPI_DRV_SlaveConfigure(const flexio_spi_slave_state_t *slave,
                                          const flexio_spi_slave_user_config_t * userConfigPtr)
{
    FLEXIO_Type *baseAddr;
    flexio_pin_polarity_t clockPolarity;        /* Polarity of clock signal */
    flexio_timer_polarity_t txShifterPolarity;  /* Polarity of MISO shifter */
    flexio_timer_polarity_t rxShifterPolarity;  /* Polarity of MOSI shifter */
    flexio_shifter_start_t startBit;
    uint8_t resourceIndex;    /* Index of first used internal resource instance (shifter and timer) */

    baseAddr = g_flexioBase[slave->flexioCommon.instance];
    resourceIndex = slave->flexioCommon.resourceIndex;

    if (userConfigPtr->clockPolarity == 0U)
    {
        /* CPOL = 0 */
        clockPolarity = FLEXIO_PIN_POLARITY_HIGH;
    }
    else
    {
        /* CPOL = 1 */
        clockPolarity = FLEXIO_PIN_POLARITY_LOW;
    }

    if (userConfigPtr->clockPhase == 0U)
    {
        /* CPHA = 0 */
        txShifterPolarity = FLEXIO_TIMER_POLARITY_NEGEDGE;
        rxShifterPolarity = FLEXIO_TIMER_POLARITY_POSEDGE;
        startBit = FLEXIO_SHIFTER_START_BIT_DISABLED;
    }
    else
    {
        /* CPHA = 1 */
        txShifterPolarity = FLEXIO_TIMER_POLARITY_POSEDGE;
        rxShifterPolarity = FLEXIO_TIMER_POLARITY_NEGEDGE;
        startBit = FLEXIO_SHIFTER_START_BIT_DISABLED_SH;
    }

    /* Configure Slave Tx shifter (MISO) */
    FLEXIO_HAL_SetShifterControl(baseAddr,
                                 TX_SHIFTER(resourceIndex),
                                 FLEXIO_SHIFTER_MODE_TRANSMIT,
                                 userConfigPtr->misoPin,
                                 FLEXIO_PIN_POLARITY_HIGH,
                                 FLEXIO_PIN_CONFIG_OUTPUT,
                                 SCK_TIMER(resourceIndex),
                                 txShifterPolarity);
    FLEXIO_HAL_SetShifterConfig(baseAddr,
                                TX_SHIFTER(resourceIndex),
                                startBit,
                                FLEXIO_SHIFTER_STOP_BIT_DISABLED,
                                FLEXIO_SHIFTER_SOURCE_PIN);

    /* Configure Slave Rx shifter (MOSI) */
    FLEXIO_HAL_SetShifterControl(baseAddr,
                                 RX_SHIFTER(resourceIndex),
                                 FLEXIO_SHIFTER_MODE_RECEIVE,
                                 userConfigPtr->mosiPin,
                                 FLEXIO_PIN_POLARITY_HIGH,
                                 FLEXIO_PIN_CONFIG_DISABLED,
                                 SCK_TIMER(resourceIndex),
                                 rxShifterPolarity);
    FLEXIO_HAL_SetShifterConfig(baseAddr,
                                RX_SHIFTER(resourceIndex),
                                FLEXIO_SHIFTER_START_BIT_DISABLED,
                                FLEXIO_SHIFTER_STOP_BIT_DISABLED,
                                FLEXIO_SHIFTER_SOURCE_PIN);

    /* Configure sck timer */
    FLEXIO_HAL_SetTimerCompare(baseAddr, SCK_TIMER(resourceIndex), 0xFFFFU);
    FLEXIO_HAL_SetTimerConfig(baseAddr,
                              SCK_TIMER(resourceIndex),
                              FLEXIO_TIMER_START_BIT_DISABLED,
                              FLEXIO_TIMER_STOP_BIT_DISABLED,
                              FLEXIO_TIMER_ENABLE_TRG_POSEDGE,        /* enable on SS pin rising edge */
                              FLEXIO_TIMER_DISABLE_TRG,               /* disable on SS pin falling edge */
                              FLEXIO_TIMER_RESET_NEVER,
                              FLEXIO_TIMER_DECREMENT_PIN_SHIFT_PIN,   /* decrement on input pin - SCK */
                              FLEXIO_TIMER_INITOUT_ZERO);
    FLEXIO_HAL_SetTimerControl(baseAddr,
                               SCK_TIMER(resourceIndex),
                               userConfigPtr->ssPin << 1U,          /* trigger on SS pin edge */
                               FLEXIO_TRIGGER_POLARITY_LOW,
                               FLEXIO_TRIGGER_SOURCE_INTERNAL,
                               userConfigPtr->sckPin,               /* use SCK pin as input pin */
                               clockPolarity,
                               FLEXIO_PIN_CONFIG_DISABLED,
                               FLEXIO_TIMER_MODE_16BIT);
}


/*FUNCTION**********************************************************************
 *
 * Function Name : FLEXIO_SPI_DRV_ReadData
 * Description   : reads data received by the module
 *
 *END**************************************************************************/
static void FLEXIO_SPI_DRV_ReadData(flexio_spi_master_state_t *master)
{
    const FLEXIO_Type *baseAddr;
    uint32_t data;
    uint8_t resourceIndex;    /* Index of first used internal resource instance (shifter and timer) */

    baseAddr = g_flexioBase[master->flexioCommon.instance];
    resourceIndex = master->flexioCommon.resourceIndex;

    /* Read data from shifter buffer */
    if (master->bitOrder == FLEXIO_SPI_TRANSFER_LSB_FIRST)
    {
        /* For data size < 4 bytes our data is in the upper part of the buffer and must be shifted */
        data = FLEXIO_HAL_ReadShifterBuffer(baseAddr, RX_SHIFTER(resourceIndex), FLEXIO_SHIFTER_RW_MODE_NORMAL);
        data >>= (32U - (8U * (uint32_t)(master->transferSize)));
    }
    else
    {
        data = FLEXIO_HAL_ReadShifterBuffer(baseAddr, RX_SHIFTER(resourceIndex), FLEXIO_SHIFTER_RW_MODE_BIT_SWAP);
    }

    if ((master->rxRemainingBytes > 0U) && (master->rxData != NULL))
    {
        switch (master->transferSize)
        {
            case FLEXIO_SPI_TRANSFER_1BYTE:
                *(uint8_t *)master->rxData = (uint8_t)data;
                break;
            case FLEXIO_SPI_TRANSFER_2BYTE:
                *(uint16_t *)master->rxData = (uint16_t)data;
                break;
            case FLEXIO_SPI_TRANSFER_4BYTE:
                *(uint32_t *)master->rxData = (uint32_t)data;
                break;
            default:
                /* Not possible */
                break;
        }
        /* Update rx buffer pointer and remaining bytes count */
        master->rxData = &master->rxData[(master->transferSize)];
        master->rxRemainingBytes -= (uint32_t)(master->transferSize);
    }
    else
    {
        /* No data to receive, just ignore the read data */
    }
}


/*FUNCTION**********************************************************************
 *
 * Function Name : FLEXIO_SPI_DRV_WriteData
 * Description   : writes data to be transmitted by the module
 *
 *END**************************************************************************/
static void FLEXIO_SPI_DRV_WriteData(flexio_spi_master_state_t *master)
{
    FLEXIO_Type *baseAddr;
    uint32_t data = FLEXIO_SPI_DUMMYDATA;
    uint8_t resourceIndex;    /* Index of first used internal resource instance (shifter and timer) */

    baseAddr = g_flexioBase[master->flexioCommon.instance];
    resourceIndex = master->flexioCommon.resourceIndex;

    if (master->txRemainingBytes == 0U)
    {
        /* Done transmitting */
        return;
    }

    if ((master->txRemainingBytes > 0U) && (master->txData != NULL))
    {
        /* Read data from user buffer */
        switch (master->transferSize)
        {
            case FLEXIO_SPI_TRANSFER_1BYTE:
                data = (uint32_t)(*(const uint8_t *)master->txData);
                break;
            case FLEXIO_SPI_TRANSFER_2BYTE:
                data = (uint32_t)(*(const uint16_t *)master->txData);
                break;
            case FLEXIO_SPI_TRANSFER_4BYTE:
                data = (uint32_t)(*(const uint32_t *)master->txData);
                break;
            default:
                /* Not possible */
                break;
        }
        /* Update tx buffer pointer and remaining bytes count */
        master->txData = &master->txData[master->transferSize];
        master->txRemainingBytes -= (uint32_t)(master->transferSize);
        /* Write data to shifter buffer */
        if (master->bitOrder == FLEXIO_SPI_TRANSFER_LSB_FIRST)
        {
            FLEXIO_HAL_WriteShifterBuffer(baseAddr, TX_SHIFTER(resourceIndex), data, FLEXIO_SHIFTER_RW_MODE_NORMAL);
        }
        else
        {
            /* Shift data before bit-swapping it to get the relevant bits in the lower part of the shifter */
            data <<= 32U - (8U * (uint32_t)(master->transferSize));
            FLEXIO_HAL_WriteShifterBuffer(baseAddr, TX_SHIFTER(resourceIndex), data, FLEXIO_SHIFTER_RW_MODE_BIT_SWAP);
        }
    }
    else
    {
        /* Nothing to send, write dummy data in buffer */
        FLEXIO_HAL_WriteShifterBuffer(baseAddr, TX_SHIFTER(resourceIndex), FLEXIO_SPI_DUMMYDATA, FLEXIO_SHIFTER_RW_MODE_NORMAL);
    }
}



/*FUNCTION**********************************************************************
 *
 * Function Name : FLEXIO_SPI_DRV_MasterCheckStatus
 * Description   : Check status of SPI master transfer. This function can be 
 *                 called either in an interrupt routine or directly in polling 
 *                 mode to advance the SPI transfer.
 *
 *END**************************************************************************/
static void FLEXIO_SPI_DRV_MasterCheckStatus(void *stateStruct)
{
    flexio_spi_master_state_t *master;
    FLEXIO_Type *baseAddr;
    uint8_t resourceIndex;    /* Index of first used internal resource instance (shifter and timer) */

#ifdef DEV_ERROR_DETECT
    DEV_ASSERT(stateStruct != NULL);
#endif

    master = (flexio_spi_master_state_t *)stateStruct;
    baseAddr = g_flexioBase[master->flexioCommon.instance];
    resourceIndex = master->flexioCommon.resourceIndex;

    /* Check for errors */
    if (FLEXIO_HAL_GetShifterErrorStatus(baseAddr, TX_SHIFTER(resourceIndex)))
    {
        master->status = FLEXIO_STATUS_TX_UNDERFLOW;
        FLEXIO_HAL_ClearShifterErrorStatus(baseAddr, TX_SHIFTER(resourceIndex));
        /* Force the transfer to stop */
        FLEXIO_SPI_DRV_MasterStopTransfer(master);
        /* Signal transfer end for blocking transfers */
        if (master->blocking == true)
        {
            (void)OSIF_SemaPost(&(master->idleSemaphore));
        }
        return;
    }
    if (FLEXIO_HAL_GetShifterErrorStatus(baseAddr, RX_SHIFTER(resourceIndex)))
    {
        master->status = FLEXIO_STATUS_RX_OVERFLOW;
        FLEXIO_HAL_ClearShifterErrorStatus(baseAddr, RX_SHIFTER(resourceIndex));
        /* Force the transfer to stop */
        FLEXIO_SPI_DRV_MasterStopTransfer(master);
        /* Signal transfer end for blocking transfers */
        if (master->blocking == true)
        {
            (void)OSIF_SemaPost(&(master->idleSemaphore));
        }
        return;
    }
    /* Check if data was received */
    if (FLEXIO_HAL_GetShifterStatus(baseAddr, RX_SHIFTER(resourceIndex)))
    {
        FLEXIO_SPI_DRV_ReadData(master);
    }
    /* Check if transmitter needs more data */
    if (FLEXIO_HAL_GetShifterStatus(baseAddr, TX_SHIFTER(resourceIndex)))
    {
        FLEXIO_SPI_DRV_WriteData(master);
        if (master->txRemainingBytes == 0U)
        {
            /* No more data to transmit, disable tx interrupts */
            FLEXIO_HAL_SetShifterInterrupt(baseAddr, (1U << TX_SHIFTER(resourceIndex)), false);
            FLEXIO_HAL_SetShifterErrorInterrupt(baseAddr, (1U << TX_SHIFTER(resourceIndex)), false);
        }
    }

    /* Check there is any data left */
    if ((master->txRemainingBytes == 0U) && (master->rxRemainingBytes == 0U))
    {
        /* Record success if there was no error */
        if (master->status == FLEXIO_STATUS_BUSY)
        {
            master->status = FLEXIO_STATUS_SUCCESS;
        }
        /* End transfer */
        FLEXIO_SPI_DRV_MasterEndTransfer(master);
        /* Signal transfer end for blocking transfers */
        if (master->blocking == true)
        {
            (void)OSIF_SemaPost(&(master->idleSemaphore));
        }
    }
}


/*FUNCTION**********************************************************************
 *
 * Function Name : FLEXIO_SPI_DRV_MasterComputeTxRegAddr
 * Description   : Computes the address of the register used for DMA tx transfer
 *
 *END**************************************************************************/
static uint32_t FLEXIO_SPI_DRV_MasterComputeTxRegAddr(const flexio_spi_master_state_t *master)
{
    uint32_t addr;
    const FLEXIO_Type *baseAddr;
    uint8_t shifter;

    baseAddr = g_flexioBase[master->flexioCommon.instance];
    shifter = TX_SHIFTER(master->flexioCommon.resourceIndex);
    if (master->bitOrder == FLEXIO_SPI_TRANSFER_LSB_FIRST)
    {
        addr = (uint32_t)(&(baseAddr->SHIFTBUF[shifter]));
    }
    else
    {
        addr = (uint32_t)(&(baseAddr->SHIFTBUFBIS[shifter])) + (sizeof(uint32_t) - (uint32_t)master->transferSize);
    }
    return addr;
}


/*FUNCTION**********************************************************************
 *
 * Function Name : FLEXIO_SPI_DRV_MasterComputeRxRegAddr
 * Description   : Computes the address of the register used for DMA rx transfer
 *
 *END**************************************************************************/
static uint32_t FLEXIO_SPI_DRV_MasterComputeRxRegAddr(const flexio_spi_master_state_t *master)
{
    uint32_t addr;
    const FLEXIO_Type *baseAddr;
    uint8_t shifter;

    baseAddr = g_flexioBase[master->flexioCommon.instance];
    shifter = RX_SHIFTER(master->flexioCommon.resourceIndex);
    if (master->bitOrder == FLEXIO_SPI_TRANSFER_LSB_FIRST)
    {
        addr = (uint32_t)(&(baseAddr->SHIFTBUF[shifter])) + (sizeof(uint32_t) - (uint32_t)master->transferSize);
    }
    else
    {
        addr = (uint32_t)(&(baseAddr->SHIFTBUFBIS[shifter]));
    }
    return addr;
}


/*FUNCTION**********************************************************************
 *
 * Function Name : FLEXIO_SPI_DRV_MasterStartDmaTransfer
 * Description   : Starts a DMA transfer
 *
 *END**************************************************************************/
static flexio_status_t FLEXIO_SPI_DRV_MasterStartDmaTransfer(flexio_spi_master_state_t *master)
{
    dma_request_source_t dmaReq;
    uint32_t instance;
    uint8_t resourceIndex;    /* Index of first used internal resource instance (shifter and timer) */
    uint8_t dmaChn;
    DMA_Type *edmaBase = g_edmaBase[0U];
    FLEXIO_Type *baseAddr;
    uint32_t addr;
    edma_status_t dmaStatus;
    /* Table to map flexio_spi transfer sizes to EDMA transfer sizes */
    static const edma_transfer_size_t dmaTransferSize[FLEXIO_SPI_TRANSFER_4BYTE] = 
            {EDMA_TRANSFER_SIZE_1B, EDMA_TRANSFER_SIZE_2B, EDMA_TRANSFER_SIZE_4B, EDMA_TRANSFER_SIZE_4B};


    baseAddr = g_flexioBase[master->flexioCommon.instance];
    resourceIndex = master->flexioCommon.resourceIndex;
    instance = master->flexioCommon.instance;

    if (master->txData != NULL)
    {
        addr = (uint32_t)(master->txData);
    }
    else
    {
        /* if there is no data to transmit, use dummy data as source for DMA transfer */
        master->dummyDmaData = FLEXIO_SPI_DUMMYDATA;
        addr = (uint32_t)(&(master->dummyDmaData));
    }
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
                                             addr,
                                             FLEXIO_SPI_DRV_MasterComputeTxRegAddr(master), 
                                             dmaTransferSize[(uint32_t)master->transferSize - 1U], 
                                             (uint32_t)master->transferSize);
    if (master->txData == NULL)
    {
        /* if there is no data to transmit, don't increment source offset */
        EDMA_HAL_TCDSetSrcOffset(edmaBase, dmaChn, 0);
    }
    EDMA_HAL_TCDSetMajorCount(edmaBase, dmaChn, master->txRemainingBytes / (uint32_t)master->transferSize);
    EDMA_HAL_TCDSetDisableDmaRequestAfterTCDDoneCmd(edmaBase, dmaChn, true);

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
        /* Release Tx channel */
        (void)EDMA_DRV_ReleaseChannel(&(master->txDMAChannel));
        return FLEXIO_STATUS_FAIL;
    }
    /* Configure the transfer control descriptor for the previously allocated channel */
    (void)EDMA_DRV_ConfigSingleBlockTransfer(&(master->rxDMAChannel), 
                                             EDMA_TRANSFER_PERIPH2MEM, 
                                             FLEXIO_SPI_DRV_MasterComputeRxRegAddr(master), 
                                             addr,
                                             dmaTransferSize[(uint32_t)master->transferSize - 1U], 
                                             (uint32_t)master->transferSize);
    if (master->rxData == NULL)
    {
        /* if there is no data to receive, don't increment destination offset */
        EDMA_HAL_TCDSetDestOffset(edmaBase, dmaChn, 0);
    }
    EDMA_HAL_TCDSetMajorCount(edmaBase, dmaChn, master->rxRemainingBytes / (uint32_t)master->transferSize);
    EDMA_HAL_TCDSetDisableDmaRequestAfterTCDDoneCmd(edmaBase, dmaChn, true);
    /* Setup callback for DMA transfer end */
    (void)EDMA_DRV_InstallCallback(&(master->rxDMAChannel),
                                   (edma_callback_t)(FLEXIO_SPI_DRV_MasterEndDmaTransfer),
                                   (void*)(master));

    /* Start both DMA channels */
    (void)EDMA_DRV_StartChannel(&(master->txDMAChannel));
    (void)EDMA_DRV_StartChannel(&(master->rxDMAChannel));

    /* Enable FlexIO DMA requests for both shifters */
    FLEXIO_HAL_SetShifterDMARequest(baseAddr, (1U << TX_SHIFTER(resourceIndex)) | (1U << RX_SHIFTER(resourceIndex)), true);

    return FLEXIO_STATUS_SUCCESS;
}


/*! @endcond */

/*******************************************************************************
 * Code
 ******************************************************************************/

/*FUNCTION**********************************************************************
 *
 * Function Name : FLEXIO_SPI_DRV_MasterInit
 * Description   : Initialize the FLEXIO_SPI master mode driver
 *
 * Implements : FLEXIO_SPI_DRV_MasterInit_Activity
 *END**************************************************************************/
flexio_status_t FLEXIO_SPI_DRV_MasterInit(uint32_t instance,
                                          const flexio_spi_master_user_config_t * userConfigPtr,
                                          flexio_spi_master_state_t * master)
{
    flexio_status_t retCode;
    uint32_t inputClock;
    clock_manager_error_code_t clkErr;

#ifdef DEV_ERROR_DETECT
    DEV_ASSERT(master != NULL);
    DEV_ASSERT(instance < FLEXIO_INSTANCE_COUNT);
    /* Check that device was initialized */
    DEV_ASSERT(g_flexioDeviceStatePtr[instance] != NULL);
    DEV_ASSERT((userConfigPtr->transferSize == FLEXIO_SPI_TRANSFER_1BYTE) ||
               (userConfigPtr->transferSize == FLEXIO_SPI_TRANSFER_2BYTE) ||
               (userConfigPtr->transferSize == FLEXIO_SPI_TRANSFER_4BYTE));
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
    retCode = FLEXIO_DRV_InitDriver(instance, &(master->flexioCommon));
    if (retCode != FLEXIO_STATUS_SUCCESS)
    {   /* Initialization failed, not enough resources */
        (void)OSIF_SemaDestroy(&(master->idleSemaphore));
        return retCode;
    }

    /* Initialize driver-specific context structure */
    master->driverType = userConfigPtr->driverType;
    master->bitOrder = userConfigPtr->bitOrder;
    master->transferSize = userConfigPtr->transferSize;
    master->callback = userConfigPtr->callback;
    master->callbackParam = userConfigPtr->callbackParam;
    master->blocking = false;
    master->driverIdle = true;
    master->master = true;
    master->status = FLEXIO_STATUS_SUCCESS;

    /* Configure device for SPI mode */
    FLEXIO_SPI_DRV_MasterConfigure(master, userConfigPtr, inputClock);

    /* Set up transfer engine */
    switch (userConfigPtr->driverType)
    {
        case FLEXIO_DRIVER_TYPE_INTERRUPTS:
            master->flexioCommon.isr = FLEXIO_SPI_DRV_MasterCheckStatus;
            break;
        case FLEXIO_DRIVER_TYPE_POLLING:
            /* Nothing to do here, FLEXIO_SPI_DRV_MasterGetStatus() will handle the transfer */
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
 * Function Name : FLEXIO_SPI_DRV_MasterDeinit
 * Description   : De-initialize the FLEXIO_SPI master mode driver
 *
 * Implements : FLEXIO_SPI_DRV_MasterDeinit_Activity
 *END**************************************************************************/

flexio_status_t FLEXIO_SPI_DRV_MasterDeinit(flexio_spi_master_state_t *master)
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

    return FLEXIO_DRV_DeinitDriver(&(master->flexioCommon));
}


/*FUNCTION**********************************************************************
 *
 * Function Name : FLEXIO_SPI_DRV_MasterSetBaudRate
 * Description   : Set the baud rate for any subsequent SPI communication
 *
 * Implements : FLEXIO_SPI_DRV_MasterSetBaudRate_Activity
 *END**************************************************************************/
flexio_status_t FLEXIO_SPI_DRV_MasterSetBaudRate(flexio_spi_master_state_t *master, uint32_t baudRate)
{
    FLEXIO_Type *baseAddr;
    uint16_t divider;
    uint16_t timerCmp;
    uint32_t inputClock;
    clock_manager_error_code_t clkErr;
    uint8_t resourceIndex;    /* Index of first used internal resource instance (shifter and timer) */

#ifdef DEV_ERROR_DETECT
    DEV_ASSERT(master != NULL);
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
    FLEXIO_SPI_DRV_MasterComputeBaudRateDivider(baudRate, &divider, inputClock);

    /* Configure timer divider in the lower 8 bits of TIMCMP[CMP] */
    timerCmp = FLEXIO_HAL_GetTimerCompare(baseAddr, SCK_TIMER(resourceIndex));
    timerCmp = (timerCmp & 0xFF00U) | divider;
    FLEXIO_HAL_SetTimerCompare(baseAddr, SCK_TIMER(resourceIndex), timerCmp);

    return FLEXIO_STATUS_SUCCESS;
}



/*FUNCTION**********************************************************************
 *
 * Function Name : FLEXIO_SPI_DRV_MasterGetBaudRate
 * Description   : Get the currently configured baud rate
 *
 * Implements : FLEXIO_SPI_DRV_MasterGetBaudRate_Activity
 *END**************************************************************************/
flexio_status_t FLEXIO_SPI_DRV_MasterGetBaudRate(flexio_spi_master_state_t *master, uint32_t *baudRate)
{
    const FLEXIO_Type *baseAddr;
    uint16_t divider;
    uint16_t timerCmp;
    uint32_t inputClock;
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
    divider = (timerCmp & 0x00FFU);

    /* Compute baud rate: input_clock / (2 * (divider + 1)). Round to nearest integer */
    *baudRate = (inputClock + divider + 1U) / (2U * ((uint32_t)divider + 1U));
    
    return FLEXIO_STATUS_SUCCESS;
}




/*FUNCTION**********************************************************************
 *
 * Function Name : FLEXIO_SPI_DRV_MasterTransfer
 * Description   : Perform an SPI master non-blocking transaction
 *
 * Implements : FLEXIO_SPI_DRV_MasterTransfer_Activity
 *END**************************************************************************/
flexio_status_t FLEXIO_SPI_DRV_MasterTransfer(flexio_spi_master_state_t *master,
                                              const uint8_t *txData,
                                              uint8_t *rxData,
                                              uint32_t dataSize)
{
    FLEXIO_Type *baseAddr;
    uint8_t resourceIndex;    /* Index of first used internal resource instance (shifter and timer) */
    flexio_status_t retCode;
    flexio_status_t status;

#ifdef DEV_ERROR_DETECT
    DEV_ASSERT(master != NULL);
    DEV_ASSERT(dataSize % (uint32_t)(master->transferSize) == 0U);
    DEV_ASSERT((txData != NULL) || (rxData != NULL));
#endif

    baseAddr = g_flexioBase[master->flexioCommon.instance];
    resourceIndex = master->flexioCommon.resourceIndex;

    /* Check if driver is busy */
    if (!master->driverIdle)
    {
        return FLEXIO_STATUS_BUSY;
    }
    status = master->status;
    /* Initialize transfer data */
    master->txData = txData;
    master->rxData = rxData;
    master->txRemainingBytes = dataSize;
    master->rxRemainingBytes = dataSize;
    master->driverIdle = false;
    master->status = FLEXIO_STATUS_BUSY;

    /* Enable transfer engine */
    switch (master->driverType)
    {
        case FLEXIO_DRIVER_TYPE_INTERRUPTS:
            /* Enable interrupts for Rx and Tx shifters */
            FLEXIO_HAL_SetShifterInterrupt(baseAddr, 
                                 (1U << TX_SHIFTER(resourceIndex)) | (1U << RX_SHIFTER(resourceIndex)), 
                                 true);
            FLEXIO_HAL_SetShifterErrorInterrupt(baseAddr, 
                                 (1U << TX_SHIFTER(resourceIndex)) | (1U << RX_SHIFTER(resourceIndex)), 
                                 true);
            break;
        case FLEXIO_DRIVER_TYPE_POLLING:
            /* Call FLEXIO_SPI_DRV_MasterCheckStatus once to send the first byte */
            FLEXIO_SPI_DRV_MasterCheckStatus(master);
            break;
        case FLEXIO_DRIVER_TYPE_DMA:
            retCode = FLEXIO_SPI_DRV_MasterStartDmaTransfer(master);
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
 * Function Name : FLEXIO_SPI_DRV_MasterTransferBlocking
 * Description   : Perform an SPI master blocking transaction
 *
 * Implements : FLEXIO_SPI_DRV_MasterTransferBlocking_Activity
 *END**************************************************************************/
flexio_status_t FLEXIO_SPI_DRV_MasterTransferBlocking(flexio_spi_master_state_t *master,
                                                      const uint8_t *txData,
                                                      uint8_t *rxData,
                                                      uint32_t dataSize,
                                                      uint32_t timeout)
{
    flexio_status_t status;

    /* mark transfer as blocking */
    if (master->driverType != FLEXIO_DRIVER_TYPE_POLLING)
    {
        master->blocking = true;
    }
    status = FLEXIO_SPI_DRV_MasterTransfer(master, txData, rxData, dataSize);
    if (status != FLEXIO_STATUS_SUCCESS)
    {
        /* Transfer could not be started */
        master->blocking = false;
        return status; 
    }

    /* Wait for transfer to end */
    return FLEXIO_SPI_DRV_MasterWaitTransferEnd(master, timeout);
}


/*FUNCTION**********************************************************************
 *
 * Function Name : FLEXIO_SPI_DRV_MasterTransferAbort
 * Description   : Aborts a non-blocking SPI master transaction
 *
 * Implements : FLEXIO_SPI_DRV_MasterTransferAbort_Activity
 *END**************************************************************************/
flexio_status_t FLEXIO_SPI_DRV_MasterTransferAbort(flexio_spi_master_state_t *master)
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
    FLEXIO_SPI_DRV_MasterStopTransfer(master);

    return FLEXIO_STATUS_SUCCESS;
}


/*FUNCTION**********************************************************************
 *
 * Function Name : FLEXIO_SPI_DRV_MasterGetStatus
 * Description   : Get the status of the current non-blocking SPI master transaction
 *
 * Implements : FLEXIO_SPI_DRV_MasterGetStatus_Activity
 *END**************************************************************************/
flexio_status_t FLEXIO_SPI_DRV_MasterGetStatus(flexio_spi_master_state_t *master, uint32_t *bytesRemaining)
{
#ifdef DEV_ERROR_DETECT
    DEV_ASSERT(master != NULL);
#endif

    if ((!master->driverIdle) && (master->driverType == FLEXIO_DRIVER_TYPE_POLLING))
    {
        /* In polling mode advance the SPI transfer here */
        FLEXIO_SPI_DRV_MasterCheckStatus(master);
    }

    if (bytesRemaining != NULL)
    {
        *bytesRemaining = master->txRemainingBytes;
    }

    return master->status;
}



/*FUNCTION**********************************************************************
 *
 * Function Name : FLEXIO_SPI_DRV_SlaveInit
 * Description   : Initialize the FLEXIO_SPI slave mode driver
 *
 * Implements : FLEXIO_SPI_DRV_SlaveInit_Activity
 *END**************************************************************************/
flexio_status_t FLEXIO_SPI_DRV_SlaveInit(uint32_t instance,
                                         const flexio_spi_slave_user_config_t * userConfigPtr,
                                         flexio_spi_slave_state_t * slave)
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
    retCode = FLEXIO_DRV_InitDriver(instance, &(slave->flexioCommon));
    if (retCode != FLEXIO_STATUS_SUCCESS)
    {   /* Initialization failed, not enough resources */
        (void)OSIF_SemaDestroy(&(slave->idleSemaphore));
        return retCode;
    }

    /* Initialize driver context structure */
    slave->driverType = userConfigPtr->driverType;
    slave->bitOrder = userConfigPtr->bitOrder;
    slave->transferSize = userConfigPtr->transferSize;
    slave->callback = userConfigPtr->callback;
    slave->callbackParam = userConfigPtr->callbackParam;
    slave->blocking = false;
    slave->driverIdle = true;
    slave->master = false;
    slave->status = FLEXIO_STATUS_SUCCESS;

    /* Configure device for SPI mode */
    FLEXIO_SPI_DRV_SlaveConfigure(slave, userConfigPtr);

    /* Set up transfer engine */
    switch (userConfigPtr->driverType)
    {
        case FLEXIO_DRIVER_TYPE_INTERRUPTS:
            slave->flexioCommon.isr = FLEXIO_SPI_DRV_MasterCheckStatus;
            break;
        case FLEXIO_DRIVER_TYPE_POLLING:
            /* Nothing to do here, FLEXIO_SPI_DRV_MasterGetStatus() will handle the transfer */
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



/*******************************************************************************
 * EOF
 ******************************************************************************/
