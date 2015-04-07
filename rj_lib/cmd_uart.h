/*
 * cmd_uart.h
 *
 *  Created on: 26 янв. 2015 г.
 *      Author: RLeonov
 */

#ifndef RJ_LIB_CMD_UART_H_
#define RJ_LIB_CMD_UART_H_

#define UART_TXBUF_SIZE 128

#include "uart.h"

void Uart_Printf(const char *S, ...);

#endif /* RJ_LIB_CMD_UART_H_ */
