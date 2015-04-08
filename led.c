/*
 * led.c
 *
 *  Created on: 26 янв. 2015 г.
 *      Author: RLeonov
 */
#include "LPC11Uxx.h"
#include "gpio.h"
#include "led.h"


void LedB_On()      { LPC_GPIO->CLR[0] |= TEST_LED1;            }
void LedG_On()      { LPC_GPIO->CLR[0] |= TEST_LED2;            }
void LedB_Off()     { LPC_GPIO->SET[0] |=  TEST_LED1;           }
void LedG_Off()     { LPC_GPIO->SET[0] |=  TEST_LED2;           }
void LedB_Toggle()  { LPC_GPIO->NOT[0] |=  TEST_LED1;           }
void LedG_Toggle()  { LPC_GPIO->NOT[0] |=  TEST_LED2;           }

void Led_Init(void) {
    LPC_GPIO->DIR[0] = TEST_LED1 | TEST_LED2;
    LedB_Off();
    LedG_Off();
}






