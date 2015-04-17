/*
 * cmd_uart.cpp
 *
 *  Created on: 27 џэт. 2015 у.
 *      Author: RLeonov
 */


#include "cmd_uart.h"

DbgUart_t Uart;

void Uart_Init(uint32_t ABaudrate) {
    Uart.PWrite = Uart.TXBuf;
    Uart.PRead = Uart.TXBuf;
    Uart.IFullSlotsCount = 0;

    uint32_t Fdiv;
    uint32_t regVal;

    LPC_IOCON->PIO0_19 &= ~0x07;
    LPC_IOCON->PIO0_19 |= 0x01;     /* UART TXD */
    /* Enable UART clock */
    LPC_SYSCON->SYSAHBCLKCTRL |= (1<<12);
    LPC_SYSCON->UARTCLKDIV = 0x1;     /* divided by 1 */

    LPC_USART->LCR = 0x83;             /* 8 bits, no Parity, 1 Stop bit */
    regVal = LPC_SYSCON->UARTCLKDIV;
    Fdiv = (((SystemCoreClock/LPC_SYSCON->SYSAHBCLKDIV)/regVal)/16)/ABaudrate ;    /*baud rate */

    LPC_USART->DLM = Fdiv / 256;
    LPC_USART->DLL = Fdiv % 256;
    LPC_USART->LCR = 0x03;        /* DLAB = 0 */
    LPC_USART->FCR = 0x07;        /* Enable and reset TX and RX FIFO. */

    regVal = LPC_USART->LSR; /* Read to clear the line status. */

    /* Ensure a clean start, no data in either TX or RX FIFO. */
  // CodeRed - added parentheses around comparison in operand of &
    while (( LPC_USART->LSR & (LSR_THRE|LSR_TEMT)) != (LSR_THRE|LSR_TEMT) );
    while ( LPC_USART->LSR & LSR_RDR ) {
        regVal = LPC_USART->RBR;    /* Dump data from RX FIFO */
    }
}

static inline void FPutChar(uint8_t c) {
    *Uart.PWrite++ = c;
        if(Uart.PWrite >= &Uart.TXBuf[UART_TXBUF_SIZE]) Uart.PWrite = Uart.TXBuf;   // Circulate buffer
}

void SendBufSynch(uint8_t *BufferPtr, uint32_t Length) {
    while (Length-- != 0) { // while length > 0
      while ( !(LPC_USART->LSR & LSR_THRE) ); // wait tx
      LPC_USART->THR = *BufferPtr++; // send Byte
    }
}

void Uart_Printf(const char *format, ...) {
    uint32_t MaxLength = UART_TXBUF_SIZE;
    va_list args;
    va_start(args, format);
    Uart.IFullSlotsCount += vsprintf(FPutChar, MaxLength, format, args);
    va_end(args);

    // Start transmission if Idle
    uint32_t PartSz = (Uart.TXBuf + UART_TXBUF_SIZE) - Uart.PRead;    // Char count from PRead to buffer end
    Uart.ITransSize = (Uart.IFullSlotsCount > PartSz)? PartSz : Uart.IFullSlotsCount;  // How many to transmit now
//    Send from Pread To Pread+ITransSize
    SendBufSynch(Uart.PRead, Uart.ITransSize);
    Uart.IFullSlotsCount -= Uart.ITransSize;
    Uart.PRead += Uart.ITransSize;
    if(Uart.PRead >= (Uart.TXBuf + UART_TXBUF_SIZE)) Uart.PRead = Uart.TXBuf; // Circulate pointer
}


