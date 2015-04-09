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
#include "usbd.h"

uint32_t TimeVal;


int main (void) {
    SystemInit();
    SystemCoreClockUpdate();
    Delay_Init();
    Led_Init();
    UartSW_Init(57600);
    UartSW_Printf("SC AHB:%uMHz\r", SystemCoreClock/1000000);

    Card_Init();
//    Usb_Init();
    uint32_t Timer;
    while (1) {
        if(Delay_Elapsed(&Timer, 999)) {
            LedB_Toggle();
        }
    }
}
