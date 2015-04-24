/*
 * card_ll.h
 *
 *  Created on: 10 апр. 2015 г.
 *      Author: RLeonov
 */

#ifndef CARD_LL_H_
#define CARD_LL_H_

#include <string.h>
#include <stdbool.h>
#include <stdint.h>
#include "uart_iso.h"

#define ATR_VERBOSE
#define ATR_CHANGE

#define TAi     0x10
#define TBi     0x20
#define TCi     0x40
#define TDi     0x80

bool scard_init(ISO7816_SC* pScard);
bool scard_power_on(ISO7816_SC* pScard);
void scard_power_off(ISO7816_SC* pScard);
int scard_execute_cmd(ISO7816_SC* pScard, const uint8_t* pInBuf, unsigned int inLength, uint8_t* pOutBuf);
int scard_apdu(ISO7816_SC* pScard, const uint8_t* pInBuf, unsigned int inLength, uint8_t* pOutBuf);
//void scard_pps_req(ISO7816_SC* scard, const uint8_t* pInBuf, uint32_t inLength);

#endif /* CARD_LL_H_ */
