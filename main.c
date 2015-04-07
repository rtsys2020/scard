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
#include "uart.h"
#include "led.h"
#include "cmd_uart.h"
#include "pwm_timer.h"
#include "delay.h"
#include "sw_cmd_uart.h"

uint32_t TimeVal;

int main (void) {
    SystemInit();
    SystemCoreClockUpdate();
    Delay_Init();

    UartSW_Init(4800);
    UartSW_Printf("SmartCard %u\r", SystemCoreClock/1000000);

    Pwm_Init();

    uint8_t PwmValue = 0;
    bool CountUp = true;

    while (1) {
        if(Delay_Elapsed(&TimeVal, 29)) {
            Pwm_Set(PwmValue);
            if(CountUp) {
                PwmValue++;
                if(PwmValue == 60) CountUp = false;
            }
            else {
                PwmValue--;
                if(PwmValue == 0) CountUp = true;
            }
        }
    }
}

