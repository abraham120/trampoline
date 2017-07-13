#include "tp.h"

#define OS_START_SEC_VAR_32BIT
#include "tpl_memmap.h"
extern volatile VAR(uint32, OS_VAR) tpl_time_counter;
#define OS_STOP_SEC_VAR_32BIT
#include "tpl_memmap.h"
#include "S32K144.h"
#include "clocks_and_modes.h"
#include "fsl_wdog_driver.h"

#ifdef __cplusplus
extern "C"{
#endif // __cplusplus


#define OS_START_SEC_CODE
#include "tpl_memmap.h"

#define PTE8 8 //HeadLight
#define PTE9 9 //HeadLight
#define PTC14 14
#define PTC15 15
#define PTC16 16
#define PTC17 17

FUNC(void, OS_CODE) STATUS_LED_OFF(int number) {
	switch (number) {
	case 1:
		PTC->PSOR |= 1 << PTC14;
		break;
	case 2:
		PTC->PSOR |= 1 << PTC15;
		break;
	case 3:
		PTC->PSOR |= 1 << PTC16;
		break;
	case 4:
		PTC->PSOR |= 1 << PTC17;
		break;
	}
}

FUNC(void, OS_CODE) STATUS_LED_ON(int number) {
	switch (number) {
	case 1:
		PTC->PCOR |= 1 << PTC14;
		break;
	case 2:
		PTC->PCOR |= 1 << PTC15;
		break;
	case 3:
		PTC->PCOR |= 1 << PTC16;
		break;
	case 4:
		PTC->PCOR |= 1 << PTC17;
		break;
	}
}

FUNC(void, OS_CODE) STATUS_LED_TOGGLE(int number) {
	switch (number) {
	case 1:
		PTC->PTOR |= 1 << PTC14;
		break;
	case 2:
		PTC->PTOR |= 1 << PTC15;
		break;
	case 3:
		PTC->PTOR |= 1 << PTC16;
		break;
	case 4:
		PTC->PTOR |= 1 << PTC17;
		break;
	}
}


static FUNC(void, OS_CODE) PORT_init(void) {
	//PORTE
	PCC->PCCn[PCC_PORTE_INDEX] |= PCC_PCCn_CGC_MASK; /* Enable clock for PORTE */
	//PWM
	PORTE->PCR[2] |= PORT_PCR_MUX(4); /* Port E2: MUX = ALT4, FTM3CH6, PWM */
	//CAN
	PORTE->PCR[4] |= PORT_PCR_MUX(5); /* Port E4: MUX = ALT5, CAN0_RX */
	PORTE->PCR[5] |= PORT_PCR_MUX(5); /* Port E5: MUX = ALT5, CAN0_TX */
	//GPIO
	PTE->PDDR |= 1 << PTE8; /* Port E8: Data Direction= output */
	PTE->PDDR |= 1 << PTE9; /* Port E9: Data Direction= output */
	PORTE->PCR[PTE8] = 0x00000100; /* Port E8: MUX = GPIO */
	PORTE->PCR[PTE9] = 0x00000100; /* Port E9: MUX = GPIO */

	//PORTD
	PCC->PCCn[PCC_PORTD_INDEX] |= PCC_PCCn_CGC_MASK; /* Enable clock for PORTD */
	PORTD->PCR[0] |= PORT_PCR_MUX(3); /* Port D0: MUX = ALT3, LPSPI1_SCK */
	PORTD->PCR[1] |= PORT_PCR_MUX(3); /* Port D1: MUX = ALT3, LPSPI1_SIN */
	PORTD->PCR[2] |= PORT_PCR_MUX(3); /* Port D2: MUX = ALT3, LPSPI1_SOUT */
	PORTD->PCR[3] = PORT_PCR_MUX(3); /* Port D3: MUX = ALT3, LPSPI1_PCS0 */

	//PORTC
	PCC->PCCn[PCC_PORTC_INDEX] |= PCC_PCCn_CGC_MASK; /* Enable clock for PORTC */
	//UART
	PORTC->PCR[6] |= PORT_PCR_MUX(2); /* Port C6: MUX = ALT2,UART1 TX */
	PORTC->PCR[7] |= PORT_PCR_MUX(2); /* Port C7: MUX = ALT2,UART1 RX */
	//GPIO
	PTC->PDDR |= 1 << PTC14; /* Port C14: Data Direction= output */
	PTC->PDDR |= 1 << PTC15; /* Port C15: Data Direction= output */
	PTC->PDDR |= 1 << PTC16; /* Port C16: Data Direction= output */
	PTC->PDDR |= 1 << PTC17; /* Port C17: Data Direction= output */
	PORTC->PCR[PTC14] = 0x00000100; /* Port C14: MUX = GPIO */
	PORTC->PCR[PTC15] = 0x00000100; /* Port C15: MUX = GPIO */
	PORTC->PCR[PTC16] = 0x00000100; /* Port C16: MUX = GPIO */
	PORTC->PCR[PTC17] = 0x00000100; /* Port C17: MUX = GPIO */
}

/*
 * Initialise la carte avec les ports d'E/S configurés pour allumer les LED
 * et lire le poussoir en mode GPIO ou IT
 */
FUNC(void, OS_CODE) initBoard(void)
{
  WDOG_HAL_Disable(WDOG);
  SOSC_init_8MHz();
  SPLL_init_160MHz();
  NormalRUNmode_80MHz();
  SystemCoreClockUpdate();
  PORT_init();
}

/*
 * delay comme sur l'arduino
 */
FUNC(void, OS_CODE) delay(CONST(uint32, AUTOMATIC) howMuch)
{
  CONST(uint32, AUTOMATIC) start = tpl_time_counter;
  while (tpl_time_counter - start < howMuch);
}

#ifdef __cplusplus
}
#endif // __cplusplus

#define OS_STOP_SEC_CODE
#include "tpl_memmap.h"
