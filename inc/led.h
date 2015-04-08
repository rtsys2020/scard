/*
 * led.h
 *
 *  Created on: 26 янв. 2015 г.
 *      Author: RLeonov
 */

#ifndef INC_LED_H_
#define INC_LED_H_


#define TEST_LED1 (1<<23)
#define TEST_LED2 (1<<22)



void LedB_On();
void LedG_On();
void LedB_Off();
void LedG_Off();
void LedB_Toggle();
void LedG_Toggle();

void Led_Init();



#endif /* INC_LED_H_ */
