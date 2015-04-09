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
#include "sw_cmd_uart.h"

#define CARD_PWR_PORT       1 // 38
#define CARD_PWR_PIN        22
#define PWR_ON()            PinClear(CARD_PWR_PORT, CARD_PWR_PIN)
#define PWR_OFF()           PinSet(CARD_PWR_PORT, CARD_PWR_PIN)

#define CARD_IO_PORT        0
#define CARD_IO_PIN         19
#define CARD_UART           LPC_USART
#define CARD_UART_IO_CON    LPC_IOCON->PIO0_19

#define IER_RDA     (1)
#define IER_RLS     (1 << 3)

#define IIR_PEND    0x01
#define IIR_RLS     0x03
#define IIR_RDA     0x02
#define IIR_CTI     0x06
#define IIR_THRE    0x01

#define LSR_RDR     0x01
#define LSR_OE      0x02
#define LSR_PE      0x04
#define LSR_FE      0x08
#define LSR_BI      0x10
#define LSR_THRE    0x20
#define LSR_TEMT    0x40
#define LSR_RXFE    0x80

#define CARD_RX_ON_IRQ()    CARD_UART->IER = CARD_UART_IR_RDA

#define CARD_CLK_TMR        LPC_CT16B1
#define CARD_CLK_PORT       0
#define CARD_CLK_PIN        22
#define CARD_CLK_IO_CON     LPC_IOCON->PIO0_22
#define CLK_ON()            CARD_CLK_TMR->TCR = 1;
#define CLK_OFF()           CARD_CLK_TMR->TCR = 0;

//#define CARD_CLK_PORT       0
//#define CARD_CLK_PIN        17
//#define CARD_CLK_IO_CON     LPC_IOCON->PIO0_17

#define CARD_RST_PORT       1
#define CARD_RST_PIN        24
#define RST_HI()            PinSet(CARD_RST_PORT, CARD_RST_PIN)
#define RST_LO()            PinClear(CARD_RST_PORT, CARD_RST_PIN)

typedef enum {
    CRD_NoCard, CRD_Error, CRD_Off, CRD_Active
} CardState_t;

extern uint8_t ReadByte(uint8_t *AByte);
void Card_Init();


#endif /* INC_CARD_H_ */
