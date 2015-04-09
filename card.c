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

void ClockInit() {
    LPC_SYSCON->SYSAHBCLKCTRL |= (1 << 8); // Enable the CT16B1 timer peripherals
    CARD_CLK_TMR->TCR = 2;
    CARD_CLK_TMR->PR = 2; // PreScaler value, at 48000000Hz, a PR value of 2 gives us a 24000000Hz clock
    CARD_CLK_TMR->PWMC = 0x0002;     // Enable PWM mode for Match 1.
    CARD_CLK_TMR->MCR = (1 << 10); // Reset on MR3
    CARD_CLK_TMR->MR3 = 3;     // Set the period, the timer will be reset to zero once it reaches this value
    CARD_CLK_TMR->MR1 = 2;     // Match Register 0 set to 50 counts, giving us 50% duty // Here output 4 MHz
    CARD_CLK_IO_CON &= ~0x07;
    CARD_CLK_IO_CON = (2 | (1 << 3)); // Port 0 pin 22
    CARD_CLK_TMR->TCR = 0;     // Take timer out of reset
}


void IO_Init() {
    NVIC_DisableIRQ(UART_IRQn);
    CARD_UART_IO_CON &= ~0x07;  // Clean
    CARD_UART_IO_CON |= 0x01;   // Port 0 Pin 19 is UART IO

    LPC_SYSCON->UARTCLKDIV = 1;             // divided
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
    ClockInit();
    CLK_ON();
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
