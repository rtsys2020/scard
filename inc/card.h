/*
 * card.h
 *
 *  Created on: 07 апр. 2015 г.
 *      Author: RLeonov
 */

#ifndef INC_CARD_H_
#define INC_CARD_H_


#include "LPC11Uxx.h"
#include "rj_lib_LPC11Uxx.h"

#define CARD_PWR_PORT   1
#define CARD_PWR_PIN    22
#define CARD_IO_PORT    0
#define CARD_IO_PIN     19

#define CARD_CLK_TIMER  LPC_CT16B1
#define CARD_CLK_PORT   1
#define CARD_CLK_PIN    23

#define PWR_ON()    PinClear(CARD_PWR_PORT, CARD_PWR_PIN)
#define PWR_OFF()   PinSet(CARD_PWR_PORT, CARD_PWR_PIN)

void Card_Init();


#endif /* INC_CARD_H_ */
