/*
 * card.c
 *
 *  Created on: 07 апр. 2015 г.
 *      Author: RLeonov
 */


#include "card.h"

uint8_t CardBuf[256];
uint32_t Cnt;

void PwrRst_Init() {
    PinSetupOut(CARD_PWR_PORT, CARD_PWR_PIN);
    PWR_OFF();
    PinSetupOut(CARD_RST_PORT, CARD_RST_PIN);
    RST_LO();
}

void IO_Init() {
    NVIC_DisableIRQ(UART_IRQn);
    CARD_UART_IO_CON &= ~0x07;  // Clean
    CARD_UART_IO_CON |= 0x01;   // Port 0 Pin 19 is UART IO
    CARD_CLK_IO_CON &= ~0x07;   // Clean
    CARD_CLK_IO_CON |=  0x03;    // Port 0 Pin 17 is an UART Clock

    LPC_SYSCON->UARTCLKDIV = 3;             // divided by 3
    CARD_UART->OSR = (uint32_t)(371 << 14); // Oversampling by 372
    CARD_UART->DLL = 0x01; // }
    CARD_UART->DLM = 0x00; // } pass the USART clock through without division
    CARD_UART->LCR = 0x03; // 8 bit
    CARD_UART->LCR |= (1 << 3); // Parity Enable
    CARD_UART->LCR |= (0x01 << 5); // Even Parity
    CARD_UART->SCICTRL = 0x01; // Enable SC mode
    LPC_SYSCON->SYSAHBCLKCTRL |= (1<<12);   // Enable Uart Clock

    CARD_UART->IER = CARD_UART_IR_RDA; // Enable Rx Irq

    NVIC_EnableIRQ(UART_IRQn);
    UartSW_Printf("Card Init\r");
}

void Card_Init() {
    PwrRst_Init();
    IO_Init();
    // ColdReset
    // Get ATR
}

void Card_IrqRx() {
    UartSW_Printf("i\r");
    uint8_t IIRValue = CARD_UART->IIR;
    IIRValue >>= 1;
    if (IIRValue == CARD_UART_RDA)
        CardBuf[Cnt++] = LPC_USART->RBR;
}

void Uart_IRQHandler(void) {
    Card_IrqRx();
}
