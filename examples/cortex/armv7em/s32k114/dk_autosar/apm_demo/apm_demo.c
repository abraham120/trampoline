#include "tp.h"
#include "tpl_os.h"
#include "ADC.h"
#include "FlexCAN.h"
#include "LPSPI.h"
#include "FTM.h"
#include "LPSPI.h"
#include "LPUART.h"

#define SPEED_CAN_ID      (0xAA)
#define LIGHT_CAN_ID      (0x55)
#define SPEED_ADC_CH      (0x1)
#define LIGHT_ADC_CH      (0x8)
#define SPEED_STAT_LED    (0x2)
#define LIGHT_STAT_LED    (0x1)

#define APP_Task_apm_demo_START_SEC_CODE
#include "tpl_memmap.h"
FUNC(int, OS_APPL_CODE) main(void)
{
  initBoard();
  STATUS_LED_OFF(2);
  STATUS_LED_OFF(3);
  STATUS_LED_OFF(4);

  /* initialize */
  FLEXCAN0_init();
	FTM3_init(); /* Init FTM3 using 8 MHz SOSCDIV1_CLK */
	FTM3_CH6_PWM_init();/* Init FTM3 CH6, Motor PWM */
	start_FTM3_counter();/* Start FTM3 counter */
	ADC_init(); /* Init ADC resolution 12 bit*/
	LPSPI1_init_master(); /* Initialize LPSPI1 for communication with MC33903 */
	LPSPI1_init_MC33903(); /* Configure SBC via SPI for CAN transceiver operation */
	LPUART1_init(); /* Initialize LPUART @ 9600*/

  /* start */
  StartOS(OSDEFAULTAPPMODE);
  return 0;
}

TASK(apm_demo)
{
  STATUS_LED_TOGGLE(4);
  TerminateTask();
}
#define APP_Task_apm_demo_STOP_SEC_CODE
#include "tpl_memmap.h"

#define APP_Task_light_monitor_START_SEC_CODE
#include "tpl_memmap.h"
TASK(light_monitor)
{
  uint32_t adcResultInMv = 0;
#if 1
  convertAdcChan(LIGHT_ADC_CH);
  while (adc_complete() == 0);
  adcResultInMv = read_adc_chx();
  if (adcResultInMv > 4000) {
    // Light On
    PTE->PSOR |= 1 << 8;  // PTE8
    PTE->PSOR |= 1 << 9;  // PTE9
  } else {
    // Light Off
    PTE->PCOR |= 1 << 8;  // PTE8
    PTE->PCOR |= 1 << 9;  // PTE9
  }
  STATUS_LED_ON(LIGHT_STAT_LED);
  FLEXCAN0_transmit_msg(LIGHT_CAN_ID, 0, adcResultInMv);
  delay(2);
  STATUS_LED_OFF(LIGHT_STAT_LED);
    
#endif
  TerminateTask();
}
#define APP_Task_light_monitor_STOP_SEC_CODE
#include "tpl_memmap.h"

#define APP_Task_speed_monitor_START_SEC_CODE
#include "tpl_memmap.h"
TASK(speed_monitor)
{
  uint32_t adcResultInMv = 0;
#if 1
  convertAdcChan(SPEED_ADC_CH);
  while (adc_complete() == 0);
  adcResultInMv = read_adc_chx() / 10;

  FTM3->CONTROLS[6].CnV = adcResultInMv;

  STATUS_LED_ON(SPEED_STAT_LED);
  FLEXCAN0_transmit_msg(SPEED_CAN_ID, 0, adcResultInMv);
  delay(2);
  STATUS_LED_OFF(SPEED_STAT_LED);
#endif 
  TerminateTask();
}
#define APP_Task_speed_monitor_STOP_SEC_CODE
#include "tpl_memmap.h"

#define OS_START_SEC_CODE
#include "tpl_memmap.h"
/*
 *  * This is necessary for ST libraries
 *   */
FUNC(void, OS_CODE) assert_failed(uint8_t* file, uint32_t line)
{
}

FUNC(void, OS_CODE) PreTaskHook()
{
  TaskType task_id = 0;
  GetTaskID(&task_id);
  if (task_id == apm_demo) {
//    STATUS_LED_ON(2);
  }
}

FUNC(void, OS_CODE) PostTaskHook()
{
  TaskType task_id = 0;
  GetTaskID(&task_id);
  if (task_id == apm_demo) {
//    STATUS_LED_OFF(3);
  }
}
#define OS_STOP_SEC_CODE
#include "tpl_memmap.h"

