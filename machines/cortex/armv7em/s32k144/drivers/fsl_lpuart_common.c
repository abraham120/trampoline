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
 * @file fsl_lpuart_common.c
 *
 * @page misra_violations MISRA-C:2012 violations
 *
 * @section [global]
 * Violates MISRA 2012 Required Rule 11.4, A conversion should not be 
 * performed between a pointer to object and an integer type.
 * The cast is required as LPUART instance base addresses are defined as unsigned
 * integers in the header file, but the registers are accessed via pointers to 
 * structures.
 * 
 * @section [global]
 * Violates MISRA 2012 Advisory Rule 8.4, external symbol defined without a prior 
 * declaration.
 * The symbols are declared in the driver header as external; the header is not included 
 * by this file.
 *
 * @section [global]
 * Violates MISRA 2012 Required Rule 11.6, A conversion should not be 
 * performed between a pointer to object and an integer type.
 * The cast is required as LPUART instance base addresses are defined as unsigned
 * integers in the header file, but the registers are accessed via pointers to 
 * structures.
 */


#include "fsl_device_registers.h"
#include "fsl_clock_manager.h"
#include "fsl_interrupt_manager.h"
#if FSL_FEATURE_LPUART_HAS_DMA_ENABLE
#include "fsl_edma_driver.h"
#endif

/*******************************************************************************
 *  Default interrupt handlers signatures
 ******************************************************************************/

#if (LPUART_INSTANCE_COUNT > 0U)
/*! @brief LPUART0 interrupt handler. */
extern void LPUART0_IrqHandler(void);
#endif

#if (LPUART_INSTANCE_COUNT > 1U)
/*! @brief LPUART1 interrupt handler. */
extern void LPUART1_IrqHandler(void);
#endif

#if (LPUART_INSTANCE_COUNT > 2U)
/*! @brief LPUART2 interrupt handler. */
extern void LPUART2_IrqHandler(void);
#endif

#if (LPUART_INSTANCE_COUNT > 3U)
/*! @brief LPUART3 interrupt handler. */
extern void LPUART3_IrqHandler(void);
#endif

/*******************************************************************************
 * Variables
 ******************************************************************************/

/* Table of base addresses for lpuart instances. */
LPUART_Type * const g_lpuartBase[LPUART_INSTANCE_COUNT] = LPUART_BASE_PTRS;

/* Table to save LPUART enum numbers defined in CMSIS files. */
const IRQn_Type g_lpuartRxTxIrqId[LPUART_INSTANCE_COUNT] = LPUART_RX_TX_IRQS;

/* Table to save LPUART clock names as defined in clock manager. */
const clock_names_t g_lpuartClkNames[LPUART_INSTANCE_COUNT] = {PCC_LPUART0_CLOCK, PCC_LPUART1_CLOCK,
                                                               PCC_LPUART2_CLOCK};

/* Table to save LPUART ISRs - to be used for interrupt service routing
 * at runtime, parameter for INT_SYS_InstallHandler */
const isr_t g_lpuartIsr[LPUART_INSTANCE_COUNT] = {LPUART0_IrqHandler, LPUART1_IrqHandler, LPUART2_IrqHandler};

#if FSL_FEATURE_LPUART_HAS_DMA_ENABLE
/* Table to save LPUART rx DMA sources */
const dma_request_source_t g_lpuartRxDMASrc[LPUART_INSTANCE_COUNT] = {EDMA_REQ_LPUART0_RX, EDMA_REQ_LPUART1_RX,
                                                                      EDMA_REQ_LPUART2_RX};

/* Table to save LPUART tx DMA sources */
const dma_request_source_t g_lpuartTxDMASrc[LPUART_INSTANCE_COUNT] = {EDMA_REQ_LPUART0_TX, EDMA_REQ_LPUART1_TX,
                                                                      EDMA_REQ_LPUART2_TX};
#endif

/*******************************************************************************
 * EOF
 ******************************************************************************/
