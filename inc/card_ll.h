/*
 * card_ll.h
 *
 *  Created on: 10 апр. 2015 г.
 *      Author: RLeonov
 */

#ifndef CARD_LL_H_
#define CARD_LL_H_

#include <string.h>
#include "LPC11Uxx.h"
#include "rj_lib_LPC11Uxx.h"

#define MAX_ATR_SIZE        32

#define CARD_PWR_PIN        PIO1_22
//#define PWR_ON()            gpio_reset_pin(CARD_PWR_PIN);
//#define PWR_OFF()           gpio_set_pin(CARD_PWR_PIN);
#define PWR_ON()            PinClear(1, 22);
#define PWR_OFF()           PinSet(1, 22);

#define CARD_IO_PORT        0
#define CARD_IO_PIN         19
#define CARD_UART           LPC_USART
#define CARD_UART_IO_CON    LPC_IOCON->PIO0_19

#define IER_RBR             (1)
#define IER_RLS             (1 << 3)

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

#define CARD_RX_ON_IRQ()    CARD_UART->IER = CARD_UART_IR_RDA

#define CARD_CLK_TMR        LPC_CT16B1
#define CARD_CLK_PORT       0
#define CARD_CLK_PIN        22
#define CARD_CLK_IO_CON     LPC_IOCON->PIO0_22
#define CLK_ON()            CARD_CLK_TMR->TCR = 1;
#define CLK_OFF()           CARD_CLK_TMR->TCR = 0;

#define CARD_UART_CLK_PORT       0
#define CARD_UART_CLK_PIN        17
#define CARD_UART_CLK_IO_CON     LPC_IOCON->PIO0_17

#define CARD_RST_PIN        PIO1_24
//#define RST_HI()            gpio_set_pin(CARD_RST_PIN);
//#define RST_LO()            gpio_reset_pin(CARD_RST_PIN);
#define RST_HI()            PinSet(1, 24);
#define RST_LO()            PinClear(1, 24);

typedef enum {
    scs_Error, scs_Off, scs_Idle, scs_Busy
} CardState_t;

typedef struct {
    uint8_t ATR[MAX_ATR_SIZE];
    uint8_t ATRLength;
    CardState_t State;
} ISO7816_SC;


bool scard_init(ISO7816_SC* scard);
bool scard_power_on(ISO7816_SC* scard);
void scard_power_off(ISO7816_SC* scard);
int scard_execute_cmd(ISO7816_SC* scard, const uint8_t* pInBuf, unsigned int inLength, uint8_t* pOutBuf, unsigned int pOutLength);
//void scard_pps_req(ISO7816_SC* scard, const uint8_t* pInBuf, uint32_t inLength);

#endif /* CARD_LL_H_ */
