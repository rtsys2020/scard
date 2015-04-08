/*
===============================================================================
 Name        : main.c
 Author      : Roman Leonov
 Version     :
 Copyright   : Copyright (C)
 Description : main definition
===============================================================================
*/

#include "LPC11Uxx.h"
#include "led.h"
#include "pwm_timer.h"
#include "delay.h"
#include "sw_cmd_uart.h"
#include "card.h"

uint32_t TimeVal;

int main (void) {
    SystemInit();
    SystemCoreClockUpdate();
    Delay_Init();

    Card_Init();

    UartSW_Init(4800);
    UartSW_Printf("smart_card %uMHz\r", SystemCoreClock/1000000);

    uint32_t Timer;
    while (1) {
        if(Delay_Elapsed(&Timer, 999)) {
            UartSW_PrintChar('a');
        }
    }
}

