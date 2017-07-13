/* FTM.c              (c) 2016 NXP Semiconductor, Inc.
 * Descriptions: FTM example code.
 * 2016 Jun 03  Osvaldo Romero: Initial version
 * 2016 Oct 31 SM: Updated for new header symbols for PCCn
 */

#include "S32K144.h" /* include peripheral declarations S32K144 */
#include "FTM.h"

uint16_t CurrentCaptureVal = 0;
uint16_t PriorCaptureVal = 0;
uint16_t DeltaCapture = 0;

void FTM3_init(void) {
	PCC->PCCn[PCC_FTM3_INDEX] &= ~PCC_PCCn_CGC_MASK; /* Ensure clk disabled for config */
	PCC->PCCn[PCC_FTM3_INDEX] |= PCC_PCCn_PCS(0b001) /* Clock Src=1, 8 MHz SOSCDIV1_CLK */
	| PCC_PCCn_CGC_MASK; /* Enable clock for FTM regs */
	FTM3->MODE |= FTM_MODE_WPDIS_MASK; /* Write protect to registers disabled (default) */
	FTM3->SC = 0x00400027; /* Enable PWM channel 0 output*/
	/* Enable PWM channel 1 output*/
	/* TOIE (Timer Overflow Interrupt Ena) = 0 (default) */
	/* CPWMS (Center aligned PWM Select) = 0 (default, up count) */
	/* CLKS (Clock source) = 0 (default, no clock; FTM disabled) */
	/* PS (Prescaler factor) = 7. Prescaler = 128 */
	FTM3->COMBINE = 0x00000000;/* FTM mode settings used: DECAPENx, MCOMBINEx, COMBINEx=0  */
	FTM3->POL = 0x00000000; /* Polarity for all channels is active high (default) */
	FTM3->MOD = 490; /* FTM1 counter final value (used for PWM mode) */
	/* FTM1 Period = MOD-CNTIN+0x0001 ~= 62500 ctr clks  */
	/* 8MHz /128 = 62.5kHz ->  ticks -> 1Hz */
}

void FTM3_CH6_PWM_init(void) {
	FTM3->CONTROLS[6].CnSC = 0x00000028; /* FTM3 ch6: edge-aligned PWM, low true pulses */
	/* CHIE (Chan Interrupt Ena) = 0 (default) */
	/* MSB:MSA (chan Mode Select)=0b10, Edge Align PWM*/
	/* ELSB:ELSA (chan Edge/Level Select)=0b10, low true */
	FTM3->CONTROLS[6].CnV = 0; /* FTM0 ch1 compare value (~75% duty cycle) */
}

void start_FTM3_counter(void) {
	FTM3->SC |= FTM_SC_CLKS(3);
	/* Start FTM0 counter with clk source = external clock (SOSCDIV1_CLK)*/
}
