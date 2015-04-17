/*
 * cmd_uart.h
 *
 *  Created on: 27 џэт. 2015 у.
 *      Author: RLeonov
 */

#ifndef RJ_LIB_CMD_UART_H_
#define RJ_LIB_CMD_UART_H_

#include "LPC11Uxx.h"
#include "sprintf.h"

#define UART_TXBUF_SIZE     64

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

typedef struct {
    uint8_t TXBuf[UART_TXBUF_SIZE];
    uint8_t *PWrite, *PRead;
    uint32_t IFullSlotsCount, ITransSize;

} DbgUart_t;

void Uart_Printf(const char *S, ...);
void Uart_Init(uint32_t ABaudrate);


#endif /* RJ_LIB_CMD_UART_H_ */
