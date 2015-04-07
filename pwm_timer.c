/*
 * pwm_timer.c
 *
 *  Created on: 06 апр. 2015 г.
 *      Author: RLeonov
 */


#include "pwm_timer.h"
#include "cmd_uart.h"


void Pwm_Init() {
    LPC_SYSCON->SYSAHBCLKCTRL |= (1 << 8); // Enable the CT16B1 timer peripherals
    LPC_CT16B1->TCR = 2;
    LPC_CT16B1->PR = 479; // PreScaler value, at 48MHz, a PR value of 959999 gives us a 50Hz clock
    LPC_CT16B1->CCR = 0; // No Capture
    LPC_CT16B1->EMR = 0;     // EMR not requared
    LPC_CT16B1->CTCR = 0; // No special Counter mode
    LPC_CT16B1->PWMC = 0x0002;     // Enable PWM mode for Match 1.
    LPC_CT16B1->MCR = (1 << 10); // Reset on MR3
    LPC_CT16B1->MR3 = 99;     // Set the period, the timer will be reset to zero once it reaches this value
    LPC_CT16B1->MR1 = 0;     // Match Register 0 set to 50 counts, giving us 50% duty
    LPC_IOCON->PIO0_22 &= ~0x07;
    LPC_IOCON->PIO0_22 = (2 | (1 << 3)); // Port 0 pin 22
    LPC_CT16B1->TCR = 0;     // Take timer out of reset
    LPC_CT16B1->TCR = 1; // Enable
    Uart_Printf("Pwm Init\r");
}

void Pwm_Set(uint8_t Percent) {
    LPC_CT16B1->TCR = 2;
    LPC_CT16B1->MR1 = (Percent >= 99)? 99 : Percent;     // Match Register 0 set to 50 counts, giving us 50% duty
    LPC_CT16B1->TCR = 1;
}
