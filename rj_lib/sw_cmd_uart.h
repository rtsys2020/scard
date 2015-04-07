/*
 * sw_cmd_uart.h
 *
 *  Created on: 07 апр. 2015 г.
 *      Author: RLeonov
 */

#ifndef INC_SW_CMD_UART_H_
#define INC_SW_CMD_UART_H_

#include "LPC11Uxx.h"  /* LPC11xx definitions */
#include "system_LPC11Uxx.h"
#include "gpio.h"

/*********************************************************
** Pin Definitions                                      **
*********************************************************/
#define PORT_SW_TX  1   //P1.27
#define PIN_SW_TX   27

#define SW_UART_TMR LPC_CT32B0

/*********************************************************
** Software UART configurable parameters                **
*********************************************************/
#define TEST_TIMER_NUM  0   /* 0 or 1 for 32-bit timers only */
#define TXBUFF_LEN      16
#define SW_OUT_BIF_SZ   128

#define STOP_BIT_SAMPLE (9*BIT_LENGTH)
/*********************************************************
** Exported Functions                                   **
*********************************************************/
void UartSW_Init(uint32_t Baudrate);
void UartSW_PrintStr(char* Str);                     //Transmit a string
void UartSW_PrintChar(char);          //Transmit a single character

void UartSW_Printf(const char *S, ...);
#endif /* INC_SW_CMD_UART_H_ */
