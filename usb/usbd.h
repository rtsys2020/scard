/**************************************************************************/
/*!
    @file     usbd.h
    @author   Thach Ha (tinyusb.net)

    @section LICENSE

    Software License Agreement (BSD License)

    Copyright (c) 2012, K. Townsend (microBuilder.eu)
    All rights reserved.

    Redistribution and use in source and binary forms, with or without
    modification, are permitted provided that the following conditions are met:
    1. Redistributions of source code must retain the above copyright
    notice, this list of conditions and the following disclaimer.
    2. Redistributions in binary form must reproduce the above copyright
    notice, this list of conditions and the following disclaimer in the
    documentation and/or other materials provided with the distribution.
    3. Neither the name of the copyright holders nor the
    names of its contributors may be used to endorse or promote products
    derived from this software without specific prior written permission.

    THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS ''AS IS'' AND ANY
    EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
    WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
    DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER BE LIABLE FOR ANY
    DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
    (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
    LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
    ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
    (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
    SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/
/**************************************************************************/
#ifndef __USBD_H__
#define __USBD_H__

#include "LPC11Uxx.h"
#include "power_api.h"
#include "mw_usbd_rom_api.h"
#include "descriptors.h"
#include "rj_lib_LPC11Uxx.h"


#define USB_VBUS_PIO        PIO0_3
#define USB_PULLUP          PIO0_6

#define USB_POWER_ON()      PinSet(0, 3)
#define USB_POWER_OFF()     PinClear(0, 3)
#define USB_CONNECT()       PinClear(0, 6)
#define USB_DISCONNECT()    PinSet(0, 6)

typedef struct {
    bool isConfigured;
} usb_t;

void usb_init(void);
bool usb_isConfigured(void);

extern USBD_HANDLE_T g_hUsb;

#endif
