/*
 * ccid.h
 *
 *  Created on: 08 апр. 2015 г.
 *      Author: RLeonov
 */

#ifndef INC_CCID_H_
#define INC_CCID_H_


#include "LPC11Uxx.h"
#include "rj_lib_LPC11Uxx.h"
#include "usbd.h"

#define CCID_HEADER_SZ      10
#define CCID_DATA_SZ        251

void Ccid_Init(USBD_HANDLE_T hUsb, USB_INTERFACE_DESCRIPTOR const *const pControlIntfDesc);


#endif /* INC_CCID_H_ */
