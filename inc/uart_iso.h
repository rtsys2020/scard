/*
 * uart_iso.h
 *
 *  Created on: 16 апр. 2015 г.
 *      Author: RLeonov
 */

#ifndef INC_UART_ISO_H_
#define INC_UART_ISO_H_

#include <string.h>
#include "LPC11Uxx.h"
#include "rj_lib_LPC11Uxx.h"
#include "card_ll.h"

#define CARD_PWR_PIN        PIO1_22
//#define PWR_ON()            gpio_reset_pin(CARD_PWR_PIN);
//#define PWR_OFF()           gpio_set_pin(CARD_PWR_PIN);
#define PWR_ON()            PinClear(1, 22);
#define PWR_OFF()           PinSet(1, 22);

#define CARD_IO_PORT        0
#define CARD_IO_PIN         19
#define CARD_UART           LPC_USART
#define CARD_UART_IO_CON    LPC_IOCON->PIO0_19

#define IER_RBR             0x01
#define IER_THRE            0x02
#define IER_RLS             0x04

#define IIR_PEND            0x01
#define IIR_RLS             0x03
#define IIR_RDA             0x02
#define IIR_CTI             0x06
#define IIR_THRE            0x01

#define LSR_RDR             0x01
#define LSR_OE              0x02
#define LSR_PE              0x04
#define LSR_FE              0x08
#define LSR_BI              0x10
#define LSR_THRE            0x20
#define LSR_TEMT            0x40
#define LSR_RXFE            0x80

#define ENABLE_RX_IRQ()     CARD_UART->IER |= IER_RBR;   // Enable RX interrupt
#define ENABLE_TX_IRQ()     CARD_UART->IER |= IER_THRE;   // Enable TX interrupt
#define DISABLE_RX_IRQ()    CARD_UART->IER &= ~IER_RBR;   // Disable RX interrupt
#define DISABLE_TX_IRQ()    CARD_UART->IER &= ~IER_THRE;   // Disable TX interrupt

#define CARD_CLK_TMR        LPC_CT16B1
#define CARD_CLK_PORT       0
#define CARD_CLK_PIN        22
#define CARD_CLK_IO_CON     LPC_IOCON->PIO0_22
#define CLK_ON()            CARD_CLK_TMR->TCR = 1;
#define CLK_OFF()           CARD_CLK_TMR->TCR = 0;

#define IO_RESET()          LPC_IOCON->PIO0_19 &= ~0x07;
#define IO_ENABLE()         LPC_IOCON->PIO0_19 |= 0x01; // Port 0 Pin 19 is UART TXD
#define IO_DISABLE()        LPC_IOCON->PIO0_19 &= ~0x01;

#define CARD_RST_PIN        PIO1_24
//#define RST_HI()            gpio_set_pin(CARD_RST_PIN);
//#define RST_LO()            gpio_reset_pin(CARD_RST_PIN);
#define RST_HI()            PinSet(1, 24);
#define RST_LO()            PinClear(1, 24);

typedef enum {
    cmd_TxIdle, cmd_TxSendPCB, cmd_TxSendLEN, cmd_TxSendINFO, cmd_TxSendLRC, cmd_TxOff
} cmd_TxState_t;

typedef enum {
    cmd_Idle, cmd_sendPCB, cmd_sendLEN, cmd_sendINFO, cmd_sendLRC, cmd_Off
} cmd_RxState_t;

void card_lld_init(ISO7816_SC* scard);
uint32_t card_lld_data_synch(uint8_t* pData, uint8_t inLen, uint8_t OutLength);
uint32_t card_lld_data_exchange();


#endif /* INC_UART_ISO_H_ */
