/*
 * Copyright (c) 2015, Freescale Semiconductor, Inc.
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
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES,
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 * ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/*!
 * @file fsl_lin_irq.c
 *
 * @page misra_violations MISRA-C:2012 violations
 *
 * @section [global]
 * Violates MISRA 2012 Required Rule 8.4, A compatible declaration shall be
 * visible when an object or function with external linkage is defined.
 * Folder structure has only C source file for interrupt routine.
 *
 */

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "fsl_lin_driver.h"

/*******************************************************************************
 * Code
 ******************************************************************************/
#if (LPUART_INSTANCE_COUNT > 0U)
static void LIN_LPUART0_RxTx_IRQHandler(void)
{
    LIN_DRV_IRQHandler(0U);
}

static void LIN_LPUART0_ERR_IRQHandler(void)
{
    LIN_DRV_IRQHandler(0U);
}
#endif

#if (LPUART_INSTANCE_COUNT > 1U)
static void LIN_LPUART1_RxTx_IRQHandler(void)
{
    LIN_DRV_IRQHandler(1U);
}

static void LIN_LPUART1_ERR_IRQHandler(void)
{
    LIN_DRV_IRQHandler(1U);
}
#endif

#if (LPUART_INSTANCE_COUNT > 2U)
static void LIN_LPUART2_RxTx_IRQHandler(void)
{
    LIN_DRV_IRQHandler(2U);
}

static void LIN_LPUART2_ERR_IRQHandler(void)
{
    LIN_DRV_IRQHandler(2U);
}
#endif

isr_t g_linLpuartIsrs[LPUART_INSTANCE_COUNT] = {
#if (LPUART_INSTANCE_COUNT > 0U)
    LIN_LPUART0_RxTx_IRQHandler,
#endif
#if (LPUART_INSTANCE_COUNT > 1U)
    LIN_LPUART1_RxTx_IRQHandler,
#endif
#if (LPUART_INSTANCE_COUNT > 2U)
    LIN_LPUART2_RxTx_IRQHandler,
#endif
};

isr_t g_linLpuartErrIsrs[LPUART_INSTANCE_COUNT] = {
#if (LPUART_INSTANCE_COUNT > 0U)
    LIN_LPUART0_ERR_IRQHandler,
#endif
#if (LPUART_INSTANCE_COUNT > 1U)
    LIN_LPUART1_ERR_IRQHandler,
#endif
#if (LPUART_INSTANCE_COUNT > 2U)
    LIN_LPUART2_ERR_IRQHandler,
#endif
};

/******************************************************************************/
/* EOF */
/******************************************************************************/
