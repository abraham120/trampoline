/**
 * setup pour les TP
 */

#ifndef __tp_h__
#define __tp_h__

#include "tpl_compiler.h"
//#include "stm32f4xx.h"
//#include "stm32f4_discovery.h"
#include "tpl_os_std_types.h"

#ifdef __cplusplus
extern "C"{
#endif // __cplusplus

/*
 * Initialise la carte avec les ports d'E/S configurer pour allumer les LED
 * et lire le poussoir
 */
FUNC(void, OS_CODE) initBoard(void);

FUNC(void, OS_CODE) STATUS_LED_ON(int);
FUNC(void, OS_CODE) STATUS_LED_OFF(int);
FUNC(void, OS_CODE) STATUS_LED_TOGGLE(int);

/*
 * delay comme sur l'arduino
 */
FUNC(void, OS_CODE) delay(CONST(uint32, AUTOMATIC) howMuch);

#ifdef __cplusplus
}
#endif // __cplusplus


/* End of file tp.h */
#endif
