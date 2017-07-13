/**
 * @file cm4_irq.c
 *
 * @section descr File description
 *
 * STM32F4 platform specific routines and variables
 *
 * @section copyright Copyright
 *
 * Trampoline OS
 *
 * Trampoline is copyright (c) IRCCyN 2005+
 * Copyright DG for function and data structures documentation and THUMB2 port
 * Trampoline is protected by the French intellectual property law.
 *
 * This software is distributed under the Lesser GNU Public Licence
 *
 * @section infos File informations
 *
 * $Date$
 * $Rev$
 * $Author$
 * $URL$
 */

#include "tpl_os.h"
#include "tpl_os_interrupt_kernel.h"
#include "S32K144.h"
#include "tpl_compiler.h"

extern void tpl_primary_syscall_handler(void);

/******************************************************************************
 * Exception handlers.
 ******************************************************************************/
FUNC(void, OS_CODE)NMI_Handler(void)
{
	__asm volatile("BKPT #01");
	while (1)
    {
    }
}

FUNC(void, OS_CODE)HardFault_Handler(void)
{
	uint32 lregSCB_HFSR;
	uint32 lregSCB_CFSR;
	uint32 lregSCB_BFAR;
	uint32 lregSCB_DFSR;
	uint32 lregSCB_AFSR;
	uint32 lregSCB_MMFR;

	lregSCB_HFSR = (uint32)0;
	lregSCB_CFSR = (uint32)0;
	lregSCB_BFAR = (uint32)0;
	lregSCB_DFSR = (uint32)0;
	lregSCB_AFSR = (uint32)0;
	lregSCB_MMFR = (uint32)0;

	lregSCB_HFSR = (uint32)(FSL_SCB->HFSR);
	if ((FSL_SCB->HFSR & (1 << 30)) != 0)
	{
		//printErrorMsg("Forced Hard Fault\n");
		//sprintf(msg, "FSL_SCB->CFSR = 0x%08x\n", FSL_SCB->CFSR );
		//printErrorMsg(msg);
		if((FSL_SCB->CFSR & 0xFFFF0000) != 0)
		{
			lregSCB_CFSR = (uint32)FSL_SCB->CFSR;
			//printUsageErrorMsg(FSL_SCB->CFSR);
		}
		lregSCB_BFAR = (uint32)FSL_SCB->BFAR;
		lregSCB_DFSR = (uint32)FSL_SCB->DFSR;
		lregSCB_AFSR = (uint32)FSL_SCB->AFSR;
		lregSCB_MMFR = (uint32)FSL_SCB->MMFAR;
	}

  	while (1)
    {
    }
}

FUNC(void, OS_CODE)MemManage_Handler(void)
{
	__asm volatile("BKPT #03");
	while (1)
    {
    }
}

FUNC(void, OS_CODE)BusFault_Handler(void)
{
	__asm volatile("BKPT #04");
	while (1)
    {
    }
}

FUNC(void, OS_CODE)UsageFault_Handler(void)
{
	__asm volatile("BKPT #05");
	while (1)
    {
    }
}

//FUNC(void, OS_CODE)SVCall_Handler(void)
//{
//  tpl_primary_syscall_handler();
//}

FUNC(void, OS_CODE)DebugMonitor_Handler(void)
{
  while (1)
    {
    }
}

FUNC(void, OS_CODE)PendSV_Handler(void)
{
  while (1)
    {
    }
}

/******************************************************************************
 * IRQ acknowledge functions.
 ******************************************************************************/
void SysTick_ClearFlag(void)
{
}

