/*
 * ccid.c
 *
 *  Created on: 08 ���. 2015 �.
 *      Author: RLeonov
 */

#include "ccid.h"
#include "cmd_uart.h"

bool isConfigured;

void ccid_init() {
  /* Update memory variables */
  isConfigured = true;
  Uart_Printf("ccid init\r");
}
