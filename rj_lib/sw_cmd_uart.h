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

#define TIMER32_0

#ifdef TIMER32_0
#define PORT_SW_TX      1   //P1.27
#define PIN_SW_TX       27
#define SW_UART_TMR     LPC_CT32B0
#define PIO_SW          PIO1_27
#define IRQ_Handler()   TIMER32_0_IRQHandler()
#define IRQ_Channel     TIMER_32_0_IRQn
#endif

#ifdef TIMER32_1
#define PORT_SW_TX      0   //P1.27
#define PIN_SW_TX       16
#define SW_UART_TMR     LPC_CT32B1
#define PIO_SW          PIO0_16
#define IRQ_Handler()   TIMER32_1_IRQHandler()
#define IRQ_Channel     TIMER_32_1_IRQn
#endif

#define TXBUFF_LEN      16
#define SW_OUT_BIF_SZ   128

#define STOP_BIT_SAMPLE (9*BIT_LENGTH)

void UartSW_Init(uint32_t Baudrate);
void UartSW_PrintStr(char* Str);                     //Transmit a string
void UartSW_PrintChar(char);          //Transmit a single character

void UartSW_Printf(const char *S, ...);

#endif /* INC_SW_CMD_UART_H_ */
