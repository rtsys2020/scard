/*
 * delay.h
 *
 *  Created on: 06 апр. 2015 г.
 *      Author: RLeonov
 */

#ifndef INC_DELAY_H_
#define INC_DELAY_H_

#include "LPC11Uxx.h"
#include "system_LPC11Uxx.h"
#include "rj_lib_LPC11Uxx.h"

void Delay_Init();
void Delay_ms (uint32_t Ams);
// Timer-driven delays
bool Delay_Elapsed(uint32_t *AVar, const uint32_t ADelay);

#endif /* INC_DELAY_H_ */
