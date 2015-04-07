/*
 * pwm_timer.h
 *
 *  Created on: 06 апр. 2015 г.
 *      Author: RLeonov
 */

#ifndef INC_PWM_TIMER_H_
#define INC_PWM_TIMER_H_

#include "LPC11Uxx.h"


void Pwm_Init();
void Pwm_Set(uint8_t Percent);

#endif /* INC_PWM_TIMER_H_ */
