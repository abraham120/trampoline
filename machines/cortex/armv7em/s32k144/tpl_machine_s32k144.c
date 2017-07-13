/**
 * @file tpl_machine_s32k144.c
 *
 * @section descr File description
 *
 * Trampoline processor dependant memory protection for MPC551x mpu.
 *
 * @section copyright Copyright
 *
 * Trampoline RTOS
 *
 * Trampoline is copyright (c)
 * CNRS, University of Nantes, Ecole Centrale de Nantes
 * Trampoline is protected by the French intellectual property law.
 *
 * This software is distributed under the GNU Public Licence V2.
 * Check the LICENSE file in the root directory of Trampoline
 *
 * @section infos File informations
 *
 * $Date$
 * $Rev$
 * $Author$
 * $URL$
 */

#include "tpl_compiler.h"
#include "tpl_os_std_types.h"
#include "tpl_os_definitions.h"
#include "tpl_machine.h"
#include "tpl_cortex_definitions.h"
#include "fsl_interrupt_manager.h"

#define OS_START_SEC_CODE
#include "tpl_memmap.h"

extern uint32_t SystemCoreClock;

static FUNC(uint32, OS_CODE) tpl_configure_systick(
  CONST(uint32, AUTOMATIC) ticks,
  CONST(uint32, AUTOMATIC) priority)
{
  /* Reload value impossible */
  if ((ticks - 1) > FSL_SysTick_RVR_RELOAD_MASK) return (E_OS_NOFUNC);
  /* priority impossible */
  if (priority > (1 << PRIO_BITS_IN_NVIC) - 1) return (E_OS_VALUE);
  /* Reload value ok, set reload register */
  FSL_SysTick->RVR = ticks - 1;
  /* Set priority */
  INT_SYS_SetPriority(SysTick_IRQn, priority);
  /* Load the SysTick Counter Value */
  FSL_SysTick->CVR = 0;
  /* Enable SysTick IRQ and SysTick Timer */
  FSL_SysTick->CSR = FSL_SysTick_CSR_ENABLE_MASK | 
                  FSL_SysTick_CSR_TICKINT_MASK |
                  FSL_SysTick_CSR_CLKSOURCE_MASK; 

  return (E_OK);
}


FUNC(void, OS_CODE) tpl_set_systick_timer()
{
  if (tpl_configure_systick(80000000 / 1000, OS_ISR_PRIO_UNSHIFTED) != E_OK)
  {
    /* Failed to initialize Systick, fall back in an infinite loop */
    while(1);
  }
}
#define OS_STOP_SEC_CODE
#include "tpl_memmap.h"
