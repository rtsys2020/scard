/**************************************************************************/
/*!
    @file     usbd.c
    @author   Thach Ha (tinyusb.net)

    @section DESCRIPTION

    Core USB functions

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
#include <string.h>
#include "usbd.h"
#include "cmd_uart.h"

usb_t usb;
uint8_t *usb_buffer = (uint8_t*)0x20004800;
USBD_HANDLE_T g_hUsb;

bool usb_isConfigured(void) {
  return usb.isConfigured;
}

static inline void usb_gpio_init() {
    PinSetupOut(0, 3); // USB_VBUS_PIO
    USB_POWER_ON();
    PinSetupOut(0, 6); // USB_PULLUP
    USB_DISCONNECT();
}

void usb_init(void) {
    usb.isConfigured = false;
    usb_gpio_init();
    LPC_SYSCON->SYSAHBCLKCTRL |= ((0x1<<14) | (0x1<<27));   // Enable AHB clock to the USB block and USB RAM.
    LPC_IOCON->USB_VBUS_PIO   &= ~0x1F;
    LPC_IOCON->USB_VBUS_PIO   |= (0x01<<0);            // VBUS
    LPC_IOCON->USB_PULLUP     &= ~0x07;
    LPC_IOCON->USB_PULLUP     |= (0x01<<0);            // SoftConn

//  Ccid_Init(g_hUsb, &USB_FsConfigDescriptor.CCID_Interface);
//  NVIC_EnableIRQ(USB_IRQn);
  // Perform USB soft connect
  USB_CONNECT();
}

void USB_IRQHandler(void) {
    Uart_Printf("i\r");
}
