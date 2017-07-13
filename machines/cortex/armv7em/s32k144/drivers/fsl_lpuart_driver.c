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
 * @file fsl_lpuart_driver.c
 *
 * @page misra_violations MISRA-C:2012 violations
 *
 * @section [global]
 * Violates MISRA 2012 Required Rule 1.3, There shall be no occurrence of
 * undefined or critical unspecified behavior.
 * This is caused because the addresses of some uninitialized variables are
 * passed as parameters. Those variables are not initialized as they are used as
 * output parameters by the functions.
 *
 * @section [global]
 * Violates MISRA 2012 Required Rule 5.1, identifier clash
 * The supported compilers use more than 31 significant characters for identifiers.
 *
 * @section [global]
 * Violates MISRA 2012 Required Rule 5.2, identifier clash
 * The supported compilers use more than 31 significant characters for identifiers.
 *
 * @section [global]
 * Violates MISRA 2012 Required Rule 5.4, identifier clash
 * The supported compilers use more than 31 significant characters for identifiers.
 *
 * @section [global]
 * Violates MISRA 2012 Required Rule 5.5, identifier clash
 * The supported compilers use more than 31 significant characters for identifiers.
 *
 * @section [global]
 * Violates MISRA 2012 Advisory Rule 8.4, external symbol defined without a prior 
 * declaration.
 * The symbols are declared in the common/irq source files and are not a part of the
 * public API.
 *
 * @section [global]
 * Violates MISRA 2012 Advisory Rule 8.7, External could be made static.
 * Function is defined for usage by application code.
 *
 * @section [global]
 * Violates MISRA 2012 Required Rule 11.4, A conversion should not be 
 * performed between a pointer to object and an integer type.
 * The cast is required as source and destination addresses for DMA transfers must
 * be written to registers as 32-bit unsigned integers.
 * 
 * @section [global]
 * Violates MISRA 2012 Required Rule 11.6, A cast shall not be performed
 * between pointer to void and an arithmetic type.
 * The cast is required as source and destination addresses for DMA transfers must
 * be written to registers as 32-bit unsigned integers.
 * 
 * @section [global]
 * Violates MISRA 2012 Advisory Rule 15.5, Return statement before end of 
 * function .
 * The return statement before end of function is used for simpler code structure 
 * and better readability.
 */


#include "fsl_lpuart_driver.h"

/*******************************************************************************
 * Variables
 ******************************************************************************/

/* Pointer to lpuart runtime state structure */
lpuart_state_t * g_lpuartStatePtr[LPUART_INSTANCE_COUNT] = {NULL};

/*******************************************************************************
 * Private Functions
 ******************************************************************************/
static lpuart_status_t LPUART_DRV_StartSendDataUsingInt(uint32_t instance,
                                                        const uint8_t * txBuff,
                                                        uint32_t txSize);
#if FSL_FEATURE_LPUART_HAS_DMA_ENABLE
static lpuart_status_t LPUART_DRV_StartSendDataUsingDma(uint32_t instance,
                                                        const uint8_t * txBuff,
                                                        uint32_t txSize);
#endif
static void LPUART_DRV_CompleteSendDataUsingInt(uint32_t instance);
#if FSL_FEATURE_LPUART_HAS_DMA_ENABLE
static void LPUART_DRV_CompleteSendDataUsingDma(void * parameter, edma_chn_status_t status);
#endif
static lpuart_status_t LPUART_DRV_StartReceiveDataUsingInt(uint32_t instance,
                                                           uint8_t * rxBuff,
                                                           uint32_t rxSize);
#if FSL_FEATURE_LPUART_HAS_DMA_ENABLE
static lpuart_status_t LPUART_DRV_StartReceiveDataUsingDma(uint32_t instance,
                                                           uint8_t * rxBuff,
                                                           uint32_t rxSize);
#endif
static void LPUART_DRV_CompleteReceiveDataUsingInt(uint32_t instance);
#if FSL_FEATURE_LPUART_HAS_DMA_ENABLE
static void LPUART_DRV_CompleteReceiveDataUsingDma(void * parameter, edma_chn_status_t status);
#endif

/*******************************************************************************
 * Code
 ******************************************************************************/
/*FUNCTION**********************************************************************
 *
 * Function Name : LPUART_DRV_Init
 * Description   : This function initializes a LPUART instance for operation.
 * This function will initialize the run-time state structure to keep track of
 * the on-going transfers, ungate the clock to the LPUART module, initialize the
 * module to user defined settings and default settings, configure the IRQ state
 * structure and enable the module-level interrupt to the core, and enable the
 * LPUART module transmitter and receiver.
 * The following is an example of how to set up the lpuart_state_t and the
 * lpuart_user_config_t parameters and how to call the LPUART_DRV_Init function
 * by passing in these parameters:
 *    lpuart_user_config_t lpuartConfig;
 *    lpuartConfig.baudRate = 9600;
 *    lpuartConfig.bitCountPerChar = LPUART_8_BITS_PER_CHAR;
 *    lpuartConfig.parityMode = LPUART_PARITY_DISABLED;
 *    lpuartConfig.stopBitCount = LPUART_ONE_STOP_BIT;
 *    lpuartConfig.transferType = LPUART_USING_INTERRUPTS;
 *    lpuart_state_t lpuartState;
 *    LPUART_DRV_Init(instance, &lpuartState, &lpuartConfig);
 *
 * Implements    : LPUART_DRV_Init_Activity
 *END**************************************************************************/
lpuart_status_t LPUART_DRV_Init(uint32_t instance, lpuart_state_t * lpuartStatePtr,
                                const lpuart_user_config_t * lpuartUserConfig)
{
#ifdef DEV_ERROR_DETECT
    DEV_ASSERT(instance < LPUART_INSTANCE_COUNT);
#endif

    /* Check the validity of the input parameters */
    if ((lpuartStatePtr == NULL) || (lpuartUserConfig == NULL))
    {
        return LPUART_STAT_FAIL;
    }

    lpuart_status_t lpuartStatus;
    interrupt_manager_error_code_t intManStatus;
    uint32_t lpuartSourceClock;
    clock_names_t instanceClkName = g_lpuartClkNames[instance];
    LPUART_Type * base = g_lpuartBase[instance];
    uint32_t idx;

    /* Get the LPUART clock as configured in the clock manager */
    (void)CLOCK_SYS_GetFreq(instanceClkName, &lpuartSourceClock);

    /* Exit if current instance is clock gated off. */
    if (lpuartSourceClock == 0U)
    {
        return LPUART_STAT_CLOCK_GATED_OFF;
    }

    /* Exit if current instance is already initialized. */
    if (g_lpuartStatePtr[instance] != NULL)
    {
        return LPUART_STAT_INITIALIZED;
    }

#if FSL_FEATURE_LPUART_HAS_DMA_ENABLE
    /* In DMA mode, only 8-bits chars are supported */
    if ((lpuartUserConfig->transferType == LPUART_USING_DMA) &&
        (lpuartUserConfig->bitCountPerChar != LPUART_8_BITS_PER_CHAR))
    {
        return LPUART_STAT_ERRONEOUS_CONFIG;
    }
#endif

    /* Clear the state struct for this instance. */
    uint8_t *clearStructPtr = (uint8_t *)lpuartStatePtr;
    for (idx = 0; idx < sizeof(lpuart_state_t); idx++)
    {
        clearStructPtr[idx] = 0;
    }

    /* Save runtime structure pointer.*/
    g_lpuartStatePtr[instance] = lpuartStatePtr;
    lpuartStatePtr->transferType = lpuartUserConfig->transferType;

    /* initialize the LPUART instance */
    LPUART_HAL_Init(base);

    /* initialize the parameters of the LPUART config structure with desired data */
    lpuartStatus = LPUART_HAL_SetBaudRate(base, lpuartSourceClock, lpuartUserConfig->baudRate);
    if (lpuartStatus != LPUART_STAT_SUCCESS)
    {
        return LPUART_STAT_FAIL;
    }
    LPUART_HAL_SetBitCountPerChar(base, lpuartUserConfig->bitCountPerChar);
    LPUART_HAL_SetParityMode(base, lpuartUserConfig->parityMode);
    LPUART_HAL_SetStopBitCount(base, lpuartUserConfig->stopBitCount);

    /* finally, enable the LPUART transmitter and receiver */
    LPUART_HAL_SetTransmitterCmd(base, true);
    LPUART_HAL_SetReceiverCmd(base, true);

    /* Create the synchronization objects */
    (void)OSIF_SemaCreate(&lpuartStatePtr->rxComplete, 0);
    (void)OSIF_SemaCreate(&lpuartStatePtr->txComplete, 0);

    /* Install LPUART irq handler */
    intManStatus = INT_SYS_InstallHandler(g_lpuartRxTxIrqId[instance], g_lpuartIsr[instance], (isr_t*) 0);
    if(intManStatus != INTERRUPT_MANAGER_SUCCESS)
    {
        return LPUART_STAT_FAIL;
    }

    /* Enable LPUART interrupt. */
    INT_SYS_EnableIRQ(g_lpuartRxTxIrqId[instance]);

    return LPUART_STAT_SUCCESS;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : LPUART_DRV_Deinit
 * Description   : This function shuts down the UART by disabling interrupts and
 *                 transmitter/receiver.
 *
 * Implements    : LPUART_DRV_Deinit_Activity
 *END**************************************************************************/
lpuart_status_t LPUART_DRV_Deinit(uint32_t instance)
{
#ifdef DEV_ERROR_DETECT
    DEV_ASSERT(instance < LPUART_INSTANCE_COUNT);
#endif

    interrupt_manager_error_code_t intManStatus;
    clock_names_t instanceClkName = g_lpuartClkNames[instance];
    uint32_t lpuartSourceClock;
    LPUART_Type * base = g_lpuartBase[instance];
    const lpuart_state_t * lpuartState = (lpuart_state_t *)g_lpuartStatePtr[instance];

    (void)CLOCK_SYS_GetFreq(instanceClkName, &lpuartSourceClock);

    /* Exit if current instance is already de-initialized or is gated.*/
    if ((!g_lpuartStatePtr[instance]) || (lpuartSourceClock == 0U))
    {
        return LPUART_STAT_FAIL;
    }

    /* Wait until the data is completely shifted out of shift register */
    while (!LPUART_HAL_GetStatusFlag(base, LPUART_TX_COMPLETE)) {}

    /* Destroy the synchronization objects */
    (void)OSIF_SemaDestroy(&lpuartState->rxComplete);
    (void)OSIF_SemaDestroy(&lpuartState->txComplete);

    /* Disable LPUART interrupt. */
    INT_SYS_DisableIRQ(g_lpuartRxTxIrqId[instance]);

    /* Restore default handler. */
    intManStatus = INT_SYS_InstallHandler(g_lpuartRxTxIrqId[instance], DefaultISR, (isr_t*) 0);
    if (intManStatus != INTERRUPT_MANAGER_SUCCESS)
    {
        return LPUART_STAT_FAIL;
    }

    /* disable tx and rx */
    LPUART_HAL_SetTransmitterCmd(base, false);
    LPUART_HAL_SetReceiverCmd(base, false);

    /* Clear our saved pointer to the state structure */
    g_lpuartStatePtr[instance] = NULL;

    return LPUART_STAT_SUCCESS;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : LPUART_DRV_InstallRxCallback
 * Description   : Install receive data callback function.
 *
 * Implements    : LPUART_DRV_InstallRxCallback_Activity
 *END**************************************************************************/
lpuart_rx_callback_t LPUART_DRV_InstallRxCallback(uint32_t instance,
                                                  lpuart_rx_callback_t function,
                                                  uint8_t * rxBuff,
                                                  void * callbackParam,
                                                  bool alwaysEnableRxIrq)
{
#ifdef DEV_ERROR_DETECT
    DEV_ASSERT(instance < LPUART_INSTANCE_COUNT);
#endif

    LPUART_Type * base = g_lpuartBase[instance];
    lpuart_state_t * lpuartState = (lpuart_state_t *)g_lpuartStatePtr[instance];

    lpuart_rx_callback_t currentCallback = lpuartState->rxCallback;
    lpuartState->rxCallback = function;
    lpuartState->rxCallbackParam = callbackParam;
    lpuartState->rxBuff = rxBuff;

    /* Enable/Disable the receive data full interrupt */
    LPUART_HAL_SetIntMode(base, LPUART_INT_RX_DATA_REG_FULL, alwaysEnableRxIrq);

    return currentCallback;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : LPUART_DRV_InstallTxCallback
 * Description   : Install transmit data callback function, pass in NULL pointer
 * as callback will uninstall.
 *
 * Implements    : LPUART_DRV_InstallTxCallback_Activity
 *END**************************************************************************/
lpuart_tx_callback_t LPUART_DRV_InstallTxCallback(uint32_t instance,
                                                  lpuart_tx_callback_t function,
                                                  const uint8_t * txBuff,
                                                  void * callbackParam)
{
#ifdef DEV_ERROR_DETECT
    DEV_ASSERT(instance < LPUART_INSTANCE_COUNT);
#endif

    lpuart_state_t * lpuartState = (lpuart_state_t *)g_lpuartStatePtr[instance];

    lpuart_tx_callback_t currentCallback = lpuartState->txCallback;
    lpuartState->txCallback = function;
    lpuartState->txCallbackParam = callbackParam;
    lpuartState->txBuff = txBuff;

    return currentCallback;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : LPUART_DRV_SendDataBlocking
 * Description   : This function sends data out through the LPUART module using
 * blocking method. The function does not return until the transmit is complete.
 *
 * Implements    : LPUART_DRV_SendDataBlocking_Activity
 *END**************************************************************************/
lpuart_status_t LPUART_DRV_SendDataBlocking(uint32_t instance,
                                            const uint8_t * txBuff,
                                            uint32_t txSize,
                                            uint32_t timeout)
{
#ifdef DEV_ERROR_DETECT
    DEV_ASSERT(instance < LPUART_INSTANCE_COUNT);
#endif

    /* Check the validity of the input parameter */
    if (txBuff == NULL)
    {
        return LPUART_STAT_FAIL;
    }

    lpuart_state_t * lpuartState = (lpuart_state_t *)g_lpuartStatePtr[instance];
    lpuart_status_t retVal = LPUART_STAT_SUCCESS;
    osif_status_t syncStatus;

    /* Indicates this is a blocking transaction. */
    lpuartState->isTxBlocking = true;

    switch (lpuartState->transferType )
    {
        case LPUART_USING_INTERRUPTS:
            /* Start the transmission process using interrupts */
            retVal = LPUART_DRV_StartSendDataUsingInt(instance, txBuff, txSize);
            break;
#if FSL_FEATURE_LPUART_HAS_DMA_ENABLE
        case LPUART_USING_DMA:
            /* Start the transmission process using DMA */
            retVal = LPUART_DRV_StartSendDataUsingDma(instance, txBuff, txSize);
            break;
#endif
        default:
            retVal = LPUART_STAT_ERRONEOUS_CONFIG;
            break;
    }

    if (retVal == LPUART_STAT_SUCCESS)
    {
        /* Wait until the transmit is complete. */
        syncStatus = OSIF_SemaWait(&lpuartState->txComplete, timeout);

        /* Finish the transmission if timeout expired */
        if (syncStatus == OSIF_STATUS_TIMEOUT)
        {
            lpuartState->isTxBlocking = false;
            if (lpuartState->transferType == LPUART_USING_INTERRUPTS)
            {
                LPUART_DRV_CompleteSendDataUsingInt(instance);
            }
#if FSL_FEATURE_LPUART_HAS_DMA_ENABLE
            else
            {
                LPUART_DRV_CompleteSendDataUsingDma(((void *)instance), EDMA_CHN_NORMAL);
            }
#endif

            retVal = LPUART_STAT_TIMEOUT;
        }
    }

    return retVal;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : LPUART_DRV_SendData
 * Description   : This function sends data out through the LPUART module using
 * non-blocking method. The function will return immediately after calling this
 * function.
 *
 * Implements    : LPUART_DRV_SendData_Activity
 *END**************************************************************************/
lpuart_status_t LPUART_DRV_SendData(uint32_t instance,
                                    const uint8_t * txBuff,
                                    uint32_t txSize)
{
#ifdef DEV_ERROR_DETECT
    DEV_ASSERT(instance < LPUART_INSTANCE_COUNT);
#endif

    /* Check the validity of the input parameter */
    if (txBuff == NULL)
    {
        return LPUART_STAT_FAIL;
    }

    lpuart_status_t retVal = LPUART_STAT_SUCCESS;
    lpuart_state_t * lpuartState = (lpuart_state_t *)g_lpuartStatePtr[instance];

    /* Indicates this is a non-blocking transaction. */
    lpuartState->isTxBlocking = false;

    switch (lpuartState->transferType )
    {
        case LPUART_USING_INTERRUPTS:
            /* Start the transmission process using interrupts */
            retVal = LPUART_DRV_StartSendDataUsingInt(instance, txBuff, txSize);
            break;
#if FSL_FEATURE_LPUART_HAS_DMA_ENABLE
        case LPUART_USING_DMA:
            /* Start the transmission process using DMA */
            retVal = LPUART_DRV_StartSendDataUsingDma(instance, txBuff, txSize);
            break;
#endif
        default:
            retVal = LPUART_STAT_ERRONEOUS_CONFIG;
            break;
    }

    return retVal;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : LPUART_DRV_GetTransmitStatus
 * Description   : This function returns whether the previous LPUART transmit has
 * finished. When performing non-blocking transmit, the user can call this
 * function to ascertain the state of the current transmission:
 * in progress (or busy) or complete (success). In addition, if the transmission
 * is still in progress, the user can obtain the number of words that have been
 * currently transferred.
 *
 * Implements    : LPUART_DRV_GetTransmitStatus_Activity
 *END**************************************************************************/
lpuart_status_t LPUART_DRV_GetTransmitStatus(uint32_t instance, uint32_t * bytesRemaining)
{
#ifdef DEV_ERROR_DETECT
    DEV_ASSERT(instance < LPUART_INSTANCE_COUNT);
#endif

    /* Check the validity of the input parameter */
    if (bytesRemaining == NULL)
    {
        return LPUART_STAT_FAIL;
    }

    const lpuart_state_t * lpuartState = (lpuart_state_t *)g_lpuartStatePtr[instance];
    const DMA_Type * edmaBase = g_edmaBase[0U];
    lpuart_status_t retVal = LPUART_STAT_SUCCESS;

    if (lpuartState->isTxBusy)
    {
        /* Fill in the bytes not transferred yet. */
        if (lpuartState->transferType == LPUART_USING_INTERRUPTS)
        {
            /* In interrupt-based communication, the remaining bytes are retrieved
             * from the state structure
             */
            *bytesRemaining = lpuartState->txSize;;
        }
#if FSL_FEATURE_LPUART_HAS_DMA_ENABLE
        else
        {
            /* In DMA-based communication, the remaining bytes are retrieved
             * from the current DMA major loop count
             */
            *bytesRemaining = EDMA_HAL_TCDGetCurrentMajorCount(edmaBase, lpuartState->txDMAChannel.channel);
        }
#endif
        retVal = LPUART_STAT_TX_BUSY;
    }
    else
    {
        *bytesRemaining = 0;
    }

    return retVal;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : LPUART_DRV_AbortSendingData
 * Description   : This function terminates an non-blocking LPUART transmission
 * early. During a non-blocking LPUART transmission, the user has the option to
 * terminate the transmission early if the transmission is still in progress.
 *
 * Implements    : LPUART_DRV_AbortSendingData_Activity
 *END**************************************************************************/
lpuart_status_t LPUART_DRV_AbortSendingData(uint32_t instance)
{
#ifdef DEV_ERROR_DETECT
    DEV_ASSERT(instance < LPUART_INSTANCE_COUNT);
#endif

    const lpuart_state_t * lpuartState = (lpuart_state_t *)g_lpuartStatePtr[instance];

    /* Check if a transfer is running. */
    if (!lpuartState->isTxBusy)
    {
        return LPUART_STAT_NO_TRANSMIT_IN_PROGRESS;
    }

    /* Stop the running transfer. */
    if (lpuartState->transferType == LPUART_USING_INTERRUPTS)
    {
        LPUART_DRV_CompleteSendDataUsingInt(instance);
    }
#if FSL_FEATURE_LPUART_HAS_DMA_ENABLE
    else
    {
        LPUART_DRV_CompleteSendDataUsingDma(((void *)instance), EDMA_CHN_NORMAL);
    }
#endif

    return LPUART_STAT_SUCCESS;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : LPUART_DRV_ReceiveDataBlocking
 * Description   : This function receives data from LPUART module using blocking
 * method, the function does not return until the receive is complete.
 *
 * Implements    : LPUART_DRV_ReceiveDataBlocking_Activity
 *END**************************************************************************/
lpuart_status_t LPUART_DRV_ReceiveDataBlocking(uint32_t instance,
                                               uint8_t * rxBuff,
                                               uint32_t rxSize,
                                               uint32_t timeout)
{
#ifdef DEV_ERROR_DETECT
    DEV_ASSERT(instance < LPUART_INSTANCE_COUNT);
#endif

    /* Check the validity of the input parameter */
    if (rxBuff == NULL)
    {
        return LPUART_STAT_FAIL;
    }

    lpuart_state_t * lpuartState = (lpuart_state_t *)g_lpuartStatePtr[instance];
    lpuart_status_t retVal = LPUART_STAT_SUCCESS;
    osif_status_t syncStatus;

    /* Indicates this is a blocking transaction. */
    lpuartState->isRxBlocking = true;

    switch (lpuartState->transferType )
    {
        case LPUART_USING_INTERRUPTS:
            /* Start the reception process using interrupts */
            retVal = LPUART_DRV_StartReceiveDataUsingInt(instance, rxBuff, rxSize);
            break;
#if FSL_FEATURE_LPUART_HAS_DMA_ENABLE
        case LPUART_USING_DMA:
            /* Start the reception process using interrupts */
            retVal = LPUART_DRV_StartReceiveDataUsingDma(instance, rxBuff, rxSize);
            break;
#endif
        default:
            retVal = LPUART_STAT_ERRONEOUS_CONFIG;
            break;
    }

    if (retVal == LPUART_STAT_SUCCESS)
    {
        /* Wait until the receive is complete. */
        syncStatus = OSIF_SemaWait(&lpuartState->rxComplete, timeout);

        /* Finish the reception if timeout expired */
        if (syncStatus == OSIF_STATUS_TIMEOUT)
        {
            lpuartState->isRxBlocking = false;
            if (lpuartState->transferType == LPUART_USING_INTERRUPTS)
            {
                LPUART_DRV_CompleteReceiveDataUsingInt(instance);
            }
#if FSL_FEATURE_LPUART_HAS_DMA_ENABLE
            else
            {
                LPUART_DRV_CompleteReceiveDataUsingDma(((void *)instance), EDMA_CHN_NORMAL);
            }
#endif

            retVal = LPUART_STAT_TIMEOUT;
        }
    }

    return retVal;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : LPUART_DRV_ReceiveData
 * Description   : This function receives data from LPUART module using
 * non-blocking method.  This function returns immediately after initiating the
 * receive function. The application has to get the receive status to see when
 * the receive is complete. In other words, after calling non-blocking get
 * function, the application must get the receive status to check if receive
 * is completed or not.
 *
 * Implements    : LPUART_DRV_ReceiveData_Activity
 *END**************************************************************************/
lpuart_status_t LPUART_DRV_ReceiveData(uint32_t instance,
                                       uint8_t * rxBuff,
                                       uint32_t rxSize)
{
#ifdef DEV_ERROR_DETECT
    DEV_ASSERT(instance < LPUART_INSTANCE_COUNT);
#endif

    /* Check the validity of the input parameter */
    if (rxBuff == NULL)
    {
        return LPUART_STAT_FAIL;
    }

    lpuart_status_t retVal = LPUART_STAT_SUCCESS;
    lpuart_state_t * lpuartState = (lpuart_state_t *)g_lpuartStatePtr[instance];

    /* Indicates this is a non-blocking transaction. */
    lpuartState->isRxBlocking = false;

    switch (lpuartState->transferType)
    {
        case LPUART_USING_INTERRUPTS:
            /* Start the reception process using interrupts */
            retVal = LPUART_DRV_StartReceiveDataUsingInt(instance, rxBuff, rxSize);
            break;
#if FSL_FEATURE_LPUART_HAS_DMA_ENABLE
        case LPUART_USING_DMA:
            /* Start the reception process using DMA */
            retVal = LPUART_DRV_StartReceiveDataUsingDma(instance, rxBuff, rxSize);
            break;
#endif
        default:
            retVal = LPUART_STAT_ERRONEOUS_CONFIG;
            break;
    }

    return retVal;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : LPUART_DRV_GetReceiveStatus
 * Description   : This function returns whether the previous LPUART receive is
 * complete. When performing a non-blocking receive, the user can call this
 * function to ascertain the state of the current receive progress: in progress
 * or complete. In addition, if the receive is still in progress, the user can
 * obtain the number of words that have been currently received.
 *
 * Implements    : LPUART_DRV_GetReceiveStatus_Activity
 *END**************************************************************************/
lpuart_status_t LPUART_DRV_GetReceiveStatus(uint32_t instance,
                                            uint32_t * bytesRemaining)
{
#ifdef DEV_ERROR_DETECT
    DEV_ASSERT(instance < LPUART_INSTANCE_COUNT);
#endif

    /* Check the validity of the input parameter */
    if (bytesRemaining == NULL)
    {
        return LPUART_STAT_FAIL;
    }

    const lpuart_state_t * lpuartState = (lpuart_state_t *)g_lpuartStatePtr[instance];
    const DMA_Type * edmaBase = g_edmaBase[0U];
    lpuart_status_t retVal = LPUART_STAT_SUCCESS;

    if (lpuartState->isRxBusy)
    {
        /* Fill in the bytes transferred. */
        if (lpuartState->transferType == LPUART_USING_INTERRUPTS)
        {
            /* In interrupt-based communication, the remaining bytes are retrieved
             * from the state structure
             */
            *bytesRemaining = lpuartState->rxSize;
        }
#if FSL_FEATURE_LPUART_HAS_DMA_ENABLE
        else
        {
            /* In DMA-based communication, the remaining bytes are retrieved
             * from the current DMA major loop count
             */
            *bytesRemaining = EDMA_HAL_TCDGetCurrentMajorCount(edmaBase, lpuartState->rxDMAChannel.channel);
        }
#endif
        retVal = LPUART_STAT_RX_BUSY;
    }
    else
    {
        *bytesRemaining = 0;
    }

    return retVal;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : LPUART_DRV_AbortReceivingData
 * Description   : Terminates a non-blocking receive early.
 *
 * Implements    : LPUART_DRV_AbortReceivingData_Activity
 *END**************************************************************************/
lpuart_status_t LPUART_DRV_AbortReceivingData(uint32_t instance)
{
#ifdef DEV_ERROR_DETECT
    DEV_ASSERT(instance < LPUART_INSTANCE_COUNT);
#endif

    const lpuart_state_t * lpuartState = (lpuart_state_t *)g_lpuartStatePtr[instance];

    /* Check if a transfer is running. */
    if (!lpuartState->isRxBusy)
    {
        return LPUART_STAT_NO_RECEIVE_IN_PROGRESS;
    }

    /* Stop the running transfer. */
    if (lpuartState->transferType == LPUART_USING_INTERRUPTS)
    {
        LPUART_DRV_CompleteReceiveDataUsingInt(instance);
    }
#if FSL_FEATURE_LPUART_HAS_DMA_ENABLE
    else
    {
        LPUART_DRV_CompleteReceiveDataUsingDma(((void *)instance), EDMA_CHN_NORMAL);
    }
#endif

    return LPUART_STAT_SUCCESS;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : LPUART_DRV_IRQHandler
 * Description   : Interrupt handler for LPUART.
 * This handler uses the buffers stored in the lpuart_state_t structs to transfer
 * data. This is not a public API as it is called by IRQ whenever an interrupt
 * occurs.
 *
 *END**************************************************************************/
void LPUART_DRV_IRQHandler(uint32_t instance)
{
    lpuart_state_t * lpuartState = (lpuart_state_t *)g_lpuartStatePtr[instance];
    LPUART_Type * base = g_lpuartBase[instance];

    /* Exit the ISR if no transfer is happening for this instance. */
    if (!lpuartState->isTxBusy)
    {
        if (!lpuartState->isRxBusy)
        {
            return;
        }
    }

    /* Handle receive data full interrupt */
    if (LPUART_HAL_GetIntMode(base, LPUART_INT_RX_DATA_REG_FULL))
    {
        if (LPUART_HAL_GetStatusFlag(base, LPUART_RX_DATA_REG_FULL))
        {
            /* Get data and put in receive buffer  */
            LPUART_HAL_Getchar(base, lpuartState->rxBuff);
        
            /* Invoke callback if there is one */
            if (lpuartState->rxCallback != NULL)
            {
                lpuartState->rxCallback(instance, lpuartState);
            }
            else
            {
                ++lpuartState->rxBuff;
                --lpuartState->rxSize;
        
                /* Check and see if this was the last byte received */
                if (lpuartState->rxSize == 0U)
                {
                    LPUART_DRV_CompleteReceiveDataUsingInt(instance);
                }
            }
        }
    }

    /* Handle transmitter data register empty interrupt */
    if (LPUART_HAL_GetIntMode(base, LPUART_INT_TX_DATA_REG_EMPTY))
    {
        if (LPUART_HAL_GetStatusFlag(base, LPUART_TX_DATA_REG_EMPTY))
        {
            /* check to see if there are any more bytes to send */
            if (lpuartState->txSize > 0U)
            {
                /* Transmit the data */
                LPUART_HAL_Putchar(base, *(lpuartState->txBuff));
            
                /* Invoke callback if there is one */
                if (lpuartState->txCallback != NULL)
                {
                    /* The callback MUST set the txSize to 0 if the
                     * transmit is ended.*/
                    lpuartState->txCallback(instance, lpuartState);
                }
                else
                {
                    ++lpuartState->txBuff;
                    --lpuartState->txSize;
                }
            
                /* Check and see if this was the last byte */
                if (lpuartState->txSize == 0U)
                {
                    /* Complete transfer, will disable tx interrupt */
                    LPUART_DRV_CompleteSendDataUsingInt(instance);
                }
            }
        }
    }

    /* Handle receive overrun interrupt */
    if (LPUART_HAL_GetStatusFlag(base, LPUART_RX_OVERRUN))
    {
        /* Clear the flag, OR the rxDataRegFull will not be set any more */
        (void)LPUART_HAL_ClearStatusFlag(base, LPUART_RX_OVERRUN);
    }
}

/*FUNCTION**********************************************************************
 *
 * Function Name : LPUART_DRV_StartSendDataUsingInt
 * Description   : Initiate (start) a transmit by beginning the process of
 * sending data and enabling the interrupt.
 * This is not a public API as it is called from other driver functions.
 *
 *END**************************************************************************/
static lpuart_status_t LPUART_DRV_StartSendDataUsingInt(uint32_t instance,
                                                        const uint8_t * txBuff,
                                                        uint32_t txSize)
{
    LPUART_Type * base = g_lpuartBase[instance];
    lpuart_state_t * lpuartState = (lpuart_state_t *)g_lpuartStatePtr[instance];

    /* Check it's not busy transmitting data from a previous function call */
    if (lpuartState->isTxBusy)
    {
        return LPUART_STAT_TX_BUSY;
    }

    if (txSize == 0U)
    {
        return LPUART_STAT_NO_DATA_TO_DEAL;
    }

    /* initialize the module driver state structure */
    lpuartState->txBuff = txBuff;
    lpuartState->txSize = txSize;
    lpuartState->isTxBusy = true;

    /* enable transmission complete interrupt */
    LPUART_HAL_SetIntMode(base, LPUART_INT_TX_DATA_REG_EMPTY, true);

    return LPUART_STAT_SUCCESS;
}

#if FSL_FEATURE_LPUART_HAS_DMA_ENABLE
/*FUNCTION**********************************************************************
 *
 * Function Name : LPUART_DRV_StartSendDataUsingDma
 * Description   : Initiate (start) a transmit by beginning the process of
 * sending data using DMA transfers.
 * This is not a public API as it is called from other driver functions.
 *
 *END**************************************************************************/
static lpuart_status_t LPUART_DRV_StartSendDataUsingDma(uint32_t instance,
                                                        const uint8_t * txBuff,
                                                        uint32_t txSize)
{
    LPUART_Type * base = g_lpuartBase[instance];
    DMA_Type * edmaBase = g_edmaBase[0U];
    dma_request_source_t dmaReq = g_lpuartTxDMASrc[instance];
    lpuart_state_t * lpuartState = (lpuart_state_t *)g_lpuartStatePtr[instance];
    uint8_t dmaChn;
    edma_status_t status;

    /* Check it's not busy transmitting data from a previous function call */
    if (lpuartState->isTxBusy)
    {
        return LPUART_STAT_TX_BUSY;
    }

    if (txSize == 0U)
    {
        return LPUART_STAT_NO_DATA_TO_DEAL;
    }

    /* Update state structure */
    lpuartState->txBuff = txBuff;
    lpuartState->isTxBusy = true;

    /* Request a DMA channel for this transmission */
    status = EDMA_DRV_RequestChannel((uint8_t)EDMA_ANY_CHANNEL, dmaReq, &(lpuartState->txDMAChannel), &dmaChn);
    if (status == EDMA_STATUS_FAIL)
    {
        return LPUART_STAT_DMA_UNAVAILABLE;
    }

    /* Configure the transfer control descriptor for the previously allocated channel */
    (void)EDMA_DRV_ConfigSingleBlockTransfer(&(lpuartState->txDMAChannel), EDMA_TRANSFER_MEM2PERIPH, (uint32_t)txBuff,
                                             (uint32_t)(&(base->DATA)), EDMA_TRANSFER_SIZE_1B, 1U);
    EDMA_HAL_TCDSetMajorCount(edmaBase, dmaChn, txSize);
    EDMA_HAL_TCDSetDisableDmaRequestAfterTCDDoneCmd(edmaBase, lpuartState->txDMAChannel.channel, true);

    /* Call driver function to end the transmission when the DMA transfer is done */
    (void)EDMA_DRV_InstallCallback(&(lpuartState->txDMAChannel),
                                   (edma_callback_t)(LPUART_DRV_CompleteSendDataUsingDma),
                                   (void*)(instance));

    /* Start the DMA channel */
    (void)EDMA_DRV_StartChannel(&(lpuartState->txDMAChannel));

    /* Enable tx DMA requests for the current instance */
    LPUART_HAL_SetTxDmaCmd(base, true);

    return LPUART_STAT_SUCCESS;
}
#endif

/*FUNCTION**********************************************************************
 *
 * Function Name : LPUART_DRV_CompleteSendDataUsingInt
 * Description   : Finish up a transmit by completing the process of sending
 * data and disabling the interrupt.
 * This is not a public API as it is called from other driver functions.
 *
 *END**************************************************************************/
static void LPUART_DRV_CompleteSendDataUsingInt(uint32_t instance)
{
    LPUART_Type * base = g_lpuartBase[instance];
    lpuart_state_t * lpuartState = (lpuart_state_t *)g_lpuartStatePtr[instance];

    /* Disable transmission complete interrupt */
    LPUART_HAL_SetIntMode(base, LPUART_INT_TX_DATA_REG_EMPTY, false);

    /* Signal the synchronous completion object. */
    if (lpuartState->isTxBlocking)
    {
        (void)OSIF_SemaPost(&lpuartState->txComplete);
    }

    /* Update the information of the module driver state */
    lpuartState->isTxBusy = false;
}

#if FSL_FEATURE_LPUART_HAS_DMA_ENABLE
/*FUNCTION**********************************************************************
 *
 * Function Name : LPUART_DRV_CompleteSendDataUsingDma
 * Description   : Finish up a transmit by completing the process of sending
 * data and disabling the DMA requests. This is a callback for DMA major loop
 * completion, so it must match the DMA callback signature.
 * This is not a public API as it is called from other driver functions.
 *
 *END**************************************************************************/
static void LPUART_DRV_CompleteSendDataUsingDma(void * parameter, edma_chn_status_t status)
{
    if (status != EDMA_CHN_NORMAL)
    {
        return;
    }

    uint32_t instance = ((uint32_t)parameter);
    LPUART_Type * base = g_lpuartBase[instance];
    lpuart_state_t * lpuartState = (lpuart_state_t *)g_lpuartStatePtr[instance];

    /* Disable tx DMA requests for the current instance */
    LPUART_HAL_SetTxDmaCmd(base, false);

    /* Release the DMA channel */
    (void)EDMA_DRV_StopChannel(&(lpuartState->txDMAChannel));
    (void)EDMA_DRV_ReleaseChannel(&(lpuartState->txDMAChannel));

    /* Invoke callback if there is one */
    if (lpuartState->txCallback != NULL)
    {
        /* Pass the state structure as parameter for internal information retrieval */
        lpuartState->txCallback(instance, lpuartState->txCallbackParam);
    }

    /* Signal the synchronous completion object. */
    if (lpuartState->isTxBlocking)
    {
        (void)OSIF_SemaPost(&lpuartState->txComplete);
    }

    /* Update the information of the module driver state */
    lpuartState->isTxBusy = false;
}
#endif

/*FUNCTION**********************************************************************
 *
 * Function Name : LPUART_DRV_StartReceiveDataUsingInt
 * Description   : Initiate (start) a receive by beginning the process of
 * receiving data and enabling the interrupt.
 * This is not a public API as it is called from other driver functions.
 *
 *END**************************************************************************/
static lpuart_status_t LPUART_DRV_StartReceiveDataUsingInt(uint32_t instance,
                                                           uint8_t * rxBuff,
                                                           uint32_t rxSize)
{
    lpuart_state_t * lpuartState = (lpuart_state_t *)g_lpuartStatePtr[instance];
    LPUART_Type * base = g_lpuartBase[instance];

    /* Check it's not busy receiving data from a previous function call */
    if ((lpuartState->isRxBusy) && (!lpuartState->rxCallback))
    {
        return LPUART_STAT_RX_BUSY;
    }

    if (rxSize == 0U)
    {
        return LPUART_STAT_NO_DATA_TO_DEAL;
    }

    /* Initialize the module driver state struct to indicate transfer in progress
     * and with the buffer and byte count data. */
    lpuartState->isRxBusy = true;
    lpuartState->rxBuff = rxBuff;
    lpuartState->rxSize = rxSize;

    /* Enable the receive data overrun interrupt */
    LPUART_HAL_SetIntMode(base, LPUART_INT_RX_OVERRUN, true);

    /* Enable receive data full interrupt */
    LPUART_HAL_SetIntMode(base, LPUART_INT_RX_DATA_REG_FULL, true);

    return LPUART_STAT_SUCCESS;
}

#if FSL_FEATURE_LPUART_HAS_DMA_ENABLE
/*FUNCTION**********************************************************************
 *
 * Function Name : LPUART_DRV_StartReceiveDataUsingDma
 * Description   : Initiate (start) a receive by beginning the process of
 * receiving data using DMA transfers.
 * This is not a public API as it is called from other driver functions.
 *
 *END**************************************************************************/
static lpuart_status_t LPUART_DRV_StartReceiveDataUsingDma(uint32_t instance,
                                                           uint8_t * rxBuff,
                                                           uint32_t rxSize)
{
    LPUART_Type * base = g_lpuartBase[instance];
    DMA_Type * edmaBase = g_edmaBase[0U];
    dma_request_source_t dmaReq = g_lpuartRxDMASrc[instance];
    lpuart_state_t * lpuartState = (lpuart_state_t *)g_lpuartStatePtr[instance];
    uint8_t dmaChn;
    edma_status_t status;

    /* Check it's not busy transmitting data from a previous function call */
    if (lpuartState->isRxBusy)
    {
        return LPUART_STAT_RX_BUSY;
    }

    if (rxSize == 0U)
    {
        return LPUART_STAT_NO_DATA_TO_DEAL;
    }

    /* Request a DMA channel for this transmission */
    status = EDMA_DRV_RequestChannel((uint8_t)EDMA_ANY_CHANNEL, dmaReq, &(lpuartState->rxDMAChannel), &dmaChn);
    if (status == EDMA_STATUS_FAIL)
    {
        return LPUART_STAT_DMA_UNAVAILABLE;
    }

    /* Configure the transfer control descriptor for the previously allocated channel */
    (void)EDMA_DRV_ConfigSingleBlockTransfer(&(lpuartState->rxDMAChannel), EDMA_TRANSFER_PERIPH2MEM, 
                                             (uint32_t)(&(base->DATA)), (uint32_t)rxBuff, EDMA_TRANSFER_SIZE_1B, 1U);
    EDMA_HAL_TCDSetMajorCount(edmaBase, dmaChn, rxSize);
    EDMA_HAL_TCDSetDisableDmaRequestAfterTCDDoneCmd(edmaBase, lpuartState->rxDMAChannel.channel, true);

    /* Call driver function to end the reception when the DMA transfer is done */
    (void)EDMA_DRV_InstallCallback(&(lpuartState->rxDMAChannel),
                                   (edma_callback_t)(LPUART_DRV_CompleteReceiveDataUsingDma),
                                   (void*)(instance));

    /* Start the DMA channel */
    (void)EDMA_DRV_StartChannel(&(lpuartState->rxDMAChannel));

    /* Enable rx DMA requests for the current instance */
    LPUART_HAL_SetRxDmaCmd(base, true);

    /* Update the state structure */
    lpuartState->rxBuff = rxBuff;
    lpuartState->isRxBusy = true;

    /* Enable rx overrun interrupt, so the irq handler can clear the flag;
     * otherwise the receiver state machine may freeze on overrun condition
     */
    LPUART_HAL_SetIntMode(base, LPUART_INT_RX_OVERRUN, true);

    return LPUART_STAT_SUCCESS;
}
#endif

/*FUNCTION**********************************************************************
 *
 * Function Name : LPUART_DRV_CompleteReceiveDataUsingInt
 * Description   : Finish up a receive by completing the process of receiving data
 * and disabling the interrupt.
 * This is not a public API as it is called from other driver functions.
 *
 *END**************************************************************************/
static void LPUART_DRV_CompleteReceiveDataUsingInt(uint32_t instance)
{
    lpuart_state_t * lpuartState = (lpuart_state_t *)g_lpuartStatePtr[instance];
    LPUART_Type * base = g_lpuartBase[instance];

    /* disable receive data full and rx overrun interrupt. */
    LPUART_HAL_SetIntMode(base, LPUART_INT_RX_DATA_REG_FULL, false);
    LPUART_HAL_SetIntMode(base, LPUART_INT_RX_OVERRUN, false);

    /* Signal the synchronous completion object. */
    if (lpuartState->isRxBlocking)
    {
        (void)OSIF_SemaPost(&lpuartState->rxComplete);
    }

    /* Update the information of the module driver state */
    lpuartState->isRxBusy = false;
}

#if FSL_FEATURE_LPUART_HAS_DMA_ENABLE
/*FUNCTION**********************************************************************
 *
 * Function Name : LPUART_DRV_CompleteReceiveDataUsingDma
 * Description   : Finish up a receive by completing the process of receiving data
 * and disabling the DMA requests. This is a callback for DMA major loop
 * completion, so it must match the DMA callback signature.
 * This is not a public API as it is called from other driver functions.
 *
 *END**************************************************************************/
static void LPUART_DRV_CompleteReceiveDataUsingDma(void * parameter, edma_chn_status_t status)
{
    if (status != EDMA_CHN_NORMAL)
    {
        return;
    }

    uint32_t instance = ((uint32_t)parameter);
    LPUART_Type * base = g_lpuartBase[instance];
    lpuart_state_t * lpuartState = (lpuart_state_t *)g_lpuartStatePtr[instance];

    /* Disable rx DMA requests for the current instance */
    LPUART_HAL_SetRxDmaCmd(base, false);

    /* Release the DMA channel */
    (void)EDMA_DRV_StopChannel(&(lpuartState->rxDMAChannel));
    (void)EDMA_DRV_ReleaseChannel(&(lpuartState->rxDMAChannel));
    LPUART_HAL_SetIntMode(base, LPUART_INT_RX_OVERRUN, false);

    /* Invoke callback if there is one */
    if (lpuartState->rxCallback != NULL)
    {
        lpuartState->rxCallback(instance, lpuartState->rxCallbackParam);
    }

    /* Signal the synchronous completion object. */
    if (lpuartState->isRxBlocking)
    {
        (void)OSIF_SemaPost(&lpuartState->rxComplete);
    }

    /* Update the information of the module driver state */
    lpuartState->isRxBusy = false;
}
#endif

/*******************************************************************************
 * EOF
 ******************************************************************************/
