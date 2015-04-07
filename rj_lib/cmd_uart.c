/*
 * cmd_uart.c
 *
 *  Created on: 26 янв. 2015 г.
 *      Author: RLeonov
 */


#include "sprintf.h"
#include "cmd_uart.h"


uint32_t    IFullSlotsCount = 0;
uint8_t     TXBuf[UART_TXBUF_SIZE];
uint8_t*    PWrite = TXBuf;


static inline void FPutChar(char c) {
    *PWrite++ = c;
    if(PWrite >= &TXBuf[UART_TXBUF_SIZE]) return;   // Circulate buffer
}

void Uart_Printf(const char *format, ...) {
    uint32_t MaxLength = UART_TXBUF_SIZE;
    va_list args;
    va_start(args, format);
    IFullSlotsCount += vsprintf(FPutChar, MaxLength, format, args);
    va_end(args);

    // Start transmission if Idle
    Uart_Send(TXBuf, IFullSlotsCount);
    IFullSlotsCount = 0;
    PWrite = TXBuf;
}
