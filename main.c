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
//#include "sw_cmd_uart.h"
#include "cmd_uart.h"
#include "usbd.h"
#include "card_ll.h"

uint32_t TimeVal;


int main (void) {
    SystemInit();
    SystemCoreClockUpdate();
    Delay_Init();
    Led_Init();
    Uart_Init(115200);
    Uart_Printf("SC AHB:%uMHz\r", SystemCoreClock/1000000);

//    ISO7816_SC scard;
//    scard_init(&scard);
//    scard_power_on(&scard);

    Usb_Init();

    uint32_t Timer;
    while (1) {
        if(Delay_Elapsed(&Timer, 999)) {
            LedB_Toggle();
        }
    }
}
