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



void led1_on();
void led2_on();
void led1_off();
void led2_off();
void led1_toggle();
void led2_toggle();

void led_init();



#endif /* INC_LED_H_ */
