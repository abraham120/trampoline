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
#if !defined(FSL_INTERRUPT_MANAGER_H)
#define FSL_INTERRUPT_MANAGER_H

#include "fsl_device_registers.h"

/*! @addtogroup interrupt_manager*/
/*! @{*/


/**
 * @page misra_violations MISRA-C:2012 violations
 *
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
 */

 /*! @file fsl_interrupt_manager.h */


/*******************************************************************************
 * Definitions
 ******************************************************************************/
/*!
 * @brief Error code definition for the interrupt manager APIs
 * Implements: interrupt_manager_error_code_t_Class
 */
typedef enum {
    INTERRUPT_MANAGER_SUCCESS,         /*!< Success         */
    INTERRUPT_MANAGER_ERROR,           /*!< Error occurred. */
} interrupt_manager_error_code_t;

/*! @brief Interrupt handler type */
typedef void (* isr_t)(void);

/*******************************************************************************
 * Default interrupt handler - implemented in startup.s
 ******************************************************************************/
/*! @brief Default ISR. */
void DefaultISR(void);

/*******************************************************************************
 * API
 ******************************************************************************/

#if defined(__cplusplus)
extern "C" {
#endif /* __cplusplus*/

/*! @name Interrupt manager APIs*/
/*@{*/

/*!
 * @brief Installs an interrupt handler routine for a given IRQ number. 
 *
 * This function lets the application  register/replace the interrupt 
 * handler for a specified IRQ number. The IRQ number is different than the vector
 * number. IRQ 0  starts from the vector 16 address. See a chip-specific reference
 * manual for details and the  startup_<SoC>.s file for each chip
 * family to find out the default interrupt handler for each device. This
 * function converts the IRQ number to the vector number by adding 16 to
 * it. 
 *
 * @note This method is applicable only if interrupt vector is copied in RAM,
 *       __flash_vector_table__ symbol is used to control this from linker options.
 *
 * @param irqNumber   IRQ number
 * @param newHandler  New interrupt handler routine address pointer
 * @param oldHandler  Pointer to a location to store current interrupt handler
 *
 * @return Error code.
 */
interrupt_manager_error_code_t INT_SYS_InstallHandler(IRQn_Type irqNumber,
                                                      const isr_t newHandler,
                                                      isr_t* const oldHandler);
/*!
 * @brief Enables an interrupt for a given IRQ number. 
 *
 * This function  enables the individual interrupt for a specified IRQ
 * number. It calls the system NVIC API to access the interrupt control
 * register. The input IRQ number does not include the core interrupt, only
 * the peripheral interrupt, from 0 to a maximum supported IRQ.
 * Implements INT_SYS_EnableIRQ_Activity
 *
 * @param irqNumber IRQ number
 */
static inline void INT_SYS_EnableIRQ(IRQn_Type irqNumber)
{
    /* Check IRQ number */
#ifdef DEV_ERROR_DETECT
    DEV_ASSERT(0 <= irqNumber);
    DEV_ASSERT(irqNumber <= FSL_FEATURE_INTERRUPT_IRQ_MAX);
#endif

    /* Enable interrupt */
    FSL_NVIC->ISER[(uint32_t)(irqNumber) >> 5U] = (uint32_t)(1UL << ((uint32_t)(irqNumber) & (uint32_t)0x1FU));
}

/*!
 * @brief Disables an interrupt for a given IRQ number. 
 *
 * This function  enables the individual interrupt for a specified IRQ
 * number. It  calls the system NVIC API to access the interrupt control
 * register.
 * Implements INT_SYS_DisableIRQ_Activity
 *
 * @param irqNumber IRQ number
 */
static inline void INT_SYS_DisableIRQ(IRQn_Type irqNumber)
{
    /* Check IRQ number */
#ifdef DEV_ERROR_DETECT
    DEV_ASSERT(0 <= irqNumber);
    DEV_ASSERT(irqNumber <= FSL_FEATURE_INTERRUPT_IRQ_MAX);
#endif

    /* Disable interrupt */
    FSL_NVIC->ICER[((uint32_t)(irqNumber) >> 5U)] = (uint32_t)(1UL << ((uint32_t)(irqNumber) & (uint32_t)0x1FU));
}

/*!
 * @brief Enables system interrupt.
 *
 * This function  enables the global interrupt by calling the core API.
 *
 */
void INT_SYS_EnableIRQGlobal(void);

/*!
 * @brief Disable system interrupt. 
 *
 * This function  disables the global interrupt by calling the core API.
 *
 */
void INT_SYS_DisableIRQGlobal(void);

/*! @brief  Set Interrupt Priority
 *
 *   The function sets the priority of an interrupt.
 *
 *   Note: The priority cannot be set for every core interrupt.
 *   Implements INT_SYS_SetPriority_Activity
 *
 *   @param  irqNumber  Interrupt number.
 *   @param   priority  Priority to set.
 */
static inline void INT_SYS_SetPriority(IRQn_Type irqNumber, uint8_t priority)
{
    /* Check IRQ number and priority. */
#ifdef DEV_ERROR_DETECT
    DEV_ASSERT(irqNumber <= FSL_FEATURE_INTERRUPT_IRQ_MAX);
    DEV_ASSERT(priority < (1<<FSL_FEATURE_NVIC_PRIO_BITS));
#endif

    uint8_t shift = (uint8_t) (8U - FSL_FEATURE_NVIC_PRIO_BITS);

    if((int32_t)irqNumber < 0)
    {
        /* Compute pointer to SHPR register - avoid MISRA violation. */
        volatile uint8_t * shpr_reg_ptr = (volatile uint8_t*)&(FSL_SCB->SHPR1);
        /* Set Priority for Cortex-M  System Interrupts */
        shpr_reg_ptr[((uint32_t)(irqNumber) & 0xFU)-4U] = (uint8_t)(((((uint32_t)priority) << shift)) & 0xffUL);
    }
    else
    {
        /* Set Priority for device specific Interrupts */
        FSL_NVIC->IP[(uint32_t)(irqNumber)] =  (uint8_t)(((((uint32_t)priority) << shift)) & 0xFFUL);
    }
}

/*! @brief  Get Interrupt Priority
 *
 *   The function gets the priority of an interrupt.
 *
 *   Note: The priority cannot be obtained for every core interrupt.
 *   Implements INT_SYS_GetPriority_Activity
 *
 *   @param  irqNumber  Interrupt number.
 *   @return priority   Priority of the interrupt.
 */
static inline uint8_t INT_SYS_GetPriority(IRQn_Type irqNumber)
{
    /* Check IRQ number. */
#ifdef DEV_ERROR_DETECT
    DEV_ASSERT(irqNumber <= FSL_FEATURE_INTERRUPT_IRQ_MAX);
#endif

    uint8_t priority = 0U;
    uint8_t shift = (uint8_t) (8U - FSL_FEATURE_NVIC_PRIO_BITS);

    if((int32_t)irqNumber < 0)
    {
        /* Compute pointer to SHPR register - avoid MISRA violation. */
        volatile const uint8_t * const shpr_reg_ptr = (volatile uint8_t*)&(FSL_SCB->SHPR1);
        /* Get Priority from Cortex-M  System Interrupts */
        priority = (uint8_t)(shpr_reg_ptr[((uint32_t)(irqNumber) & 0xFU)-4U] >> (shift));
    }
    else
    {
        /* Get Priority for device specific Interrupts  */
        priority = (uint8_t)(FSL_NVIC->IP[(uint32_t)(irqNumber)] >> shift);
    }

    return priority;
}

/*!
 * @brief Clear Pending Interrupt
 *
 * The function clears the pending bit of an external interrupt.
 * Implements INT_SYS_ClearPending_Activity
 *
 * @param irqNumber IRQ number
 */
static inline void INT_SYS_ClearPending(IRQn_Type irqNumber)
{
    /* Check IRQ number */
#ifdef DEV_ERROR_DETECT
    DEV_ASSERT(0 <= irqNumber);
    DEV_ASSERT(irqNumber <= FSL_FEATURE_INTERRUPT_IRQ_MAX);
#endif

    /* Clear Pending Interrupt */
    FSL_NVIC->ICPR[(uint32_t)(irqNumber) >> 5U] = (uint32_t)(1UL << ((uint32_t)(irqNumber) & (uint32_t)0x1FU));
}

/*!
 * @brief Set Pending Interrupt
 *
 * The function configures the pending bit of an external interrupt.
 * Implements INT_SYS_SetPending_Activity
 *
 * @param irqNumber IRQ number
 */
static inline void INT_SYS_SetPending(IRQn_Type irqNumber)
{
    /* Check IRQ number */
#ifdef DEV_ERROR_DETECT
    DEV_ASSERT(0 <= irqNumber);
    DEV_ASSERT(irqNumber <= FSL_FEATURE_INTERRUPT_IRQ_MAX);
#endif

    /* Set Pending Interrupt */
    FSL_NVIC->ISPR[(uint32_t)(irqNumber) >> 5U] = (uint32_t)(1UL << ((uint32_t)(irqNumber) & (uint32_t)0x1FU));
}

/*!
 * @brief Get Pending Interrupt
 *
 * The function gets the pending bit of an external interrupt.
 * Implements INT_SYS_GetPending_Activity
 *
 * @param irqNumber IRQ number
 */
static inline uint32_t INT_SYS_GetPending(IRQn_Type irqNumber)
{
    /* Check IRQ number */
#ifdef DEV_ERROR_DETECT
    DEV_ASSERT(0 <= irqNumber);
    DEV_ASSERT(irqNumber <= FSL_FEATURE_INTERRUPT_IRQ_MAX);
#endif

    /* Get Pending Interrupt */
  return((uint32_t)(((FSL_NVIC->ISPR[(((uint32_t)(int32_t)irqNumber) >> 5UL)] & (1UL << (((uint32_t)(int32_t)irqNumber) & 0x1FUL))) != 0UL) ? 1UL : 0UL));
}

/*!
 * @brief Get Active Interrupt
 *
 * The function gets the active state of an external interrupt.
 * Implements INT_SYS_GetActive_Activity
 *
 * @param irqNumber IRQ number
 */
static inline uint32_t INT_SYS_GetActive(IRQn_Type irqNumber)
{
    /* Check IRQ number */
#ifdef DEV_ERROR_DETECT
    DEV_ASSERT(0 <= irqNumber);
    DEV_ASSERT(irqNumber <= FSL_FEATURE_INTERRUPT_IRQ_MAX);
#endif

    /* Get Active Interrupt */
  return((uint32_t)(((FSL_NVIC->IABR[(((uint32_t)(int32_t)irqNumber) >> 5UL)] & (1UL << (((uint32_t)(int32_t)irqNumber) & 0x1FUL))) != 0UL) ? 1UL : 0UL));
}


/*@}*/

#if defined(__cplusplus)
}
#endif /* __cplusplus*/

/*! @}*/

#endif /* FSL_INTERRUPT_MANAGER_H */
/*******************************************************************************
 * EOF
 ******************************************************************************/

