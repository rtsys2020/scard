/*
 * led.c
 *
 *  Created on: 26 янв. 2015 г.
 *      Author: RLeonov
 */
#include "LPC11Uxx.h"
#include "gpio.h"
#include "led.h"


void led_init(void) { LPC_GPIO->DIR[0] = TEST_LED1 | TEST_LED2; }
void led1_on()      { LPC_GPIO->CLR[0] |= TEST_LED1;             }
void led2_on()      { LPC_GPIO->CLR[0] |= TEST_LED2;             }
void led1_off()     { LPC_GPIO->SET[0] |=  TEST_LED1;           }
void led2_off()     { LPC_GPIO->SET[0] |=  TEST_LED2;           }
void led1_toggle()  { LPC_GPIO->NOT[0] |=  TEST_LED1;           }
void led2_toggle()  { LPC_GPIO->NOT[0] |=  TEST_LED2;           }




