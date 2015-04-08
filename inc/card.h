/*
 * card.h
 *
 *  Created on: 07 ���. 2015 �.
 *      Author: RLeonov
 */

#ifndef INC_CARD_H_
#define INC_CARD_H_


#include "LPC11Uxx.h"
#include "rj_lib_LPC11Uxx.h"

#define CARD_PWR_PORT       1
#define CARD_PWR_PIN        22
#define PWR_ON()            PinClear(CARD_PWR_PORT, CARD_PWR_PIN)
#define PWR_OFF()           PinSet(CARD_PWR_PORT, CARD_PWR_PIN)

#define CARD_IO_PORT        0
#define CARD_IO_PIN         19
#define CARD_UART           LPC_USART
#define CARD_UART_IO_CON    LPC_IOCON->PIO0_19
#define CARD_UART_PEND      0x01
#define CARD_UART_RDA       0x02
#define CARD_UART_IR_RDA    0x01

#define CARD_RX_ON_IRQ()    CARD_UART->IER = CARD_UART_IR_RDA

#define CARD_CLK_PORT       0
#define CARD_CLK_PIN        17
#define CARD_CLK_IO_CON     LPC_IOCON->PIO0_17

#define CARD_RST_PORT       1
#define CARD_RST_PIN        24
#define RST_HI()            PinSet(CARD_RST_PORT, CARD_RST_PIN)
#define RST_LO()            PinClear(CARD_RST_PORT, CARD_RST_PIN)

void Card_Init();


#endif /* INC_CARD_H_ */
