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
#include "ccid.h"
#include "sw_cmd_uart.h"

#define USB_ROM_SIZE (1024*2)

volatile static bool isConfigured = false;
uint8_t *usb_RomDriver_buffer = (uint8_t*)0x20004800;
USBD_HANDLE_T g_hUsb;

bool Usb_isConfigured(void) {
  return isConfigured;
}

ErrorCode_t USB_Configure_Event (USBD_HANDLE_T hUsb) {
  USB_CORE_CTRL_T* pCtrl = (USB_CORE_CTRL_T*)hUsb;
  if (pCtrl->config_value) {
//    usb_cdc_configured(hUsb);
  }
  isConfigured = true;

  return LPC_OK;
}

ErrorCode_t USB_Reset_Event(USBD_HANDLE_T hUsb) {
  isConfigured = false;
  return LPC_OK;
}

ErrorCode_t Usb_Init(void) {
  uint32_t i;
  /* Enable AHB clock to the USB block and USB RAM. */
  LPC_SYSCON->SYSAHBCLKCTRL |= ((0x1<<14) | (0x1<<27));
  LPC_IOCON->PIO0_3   &= ~0x1F;
  LPC_IOCON->PIO0_3   |= (0x01<<0);            /* Secondary function VBUS */
  LPC_IOCON->PIO0_6   &= ~0x07;
  LPC_IOCON->PIO0_6   |= (0x01<<0);            /* Secondary function SoftConn */

  for (i=0; i < strlen(CFG_USB_STRING_MANUFACTURER); i++)
    USB_StringDescriptor.strManufacturer[i] = CFG_USB_STRING_MANUFACTURER[i];

  for (i=0; i < strlen(CFG_USB_STRING_PRODUCT); i++)
    USB_StringDescriptor.strProduct[i] = CFG_USB_STRING_PRODUCT[i];

  //iapReadUID(uid);  /* 1st byte is LSB, 4th byte is MSB */
  for (i = USB_STRING_SERIAL_LEN-1; i > 0; i--)
  {
    USB_StringDescriptor.strSerial[i] = ((uint8_t*)USB_StringDescriptor.strSerial)[i];
    ((uint8_t*)USB_StringDescriptor.strSerial)[i] = 0;
  }

  USBD_API_INIT_PARAM_T usb_param =
    {
    .usb_reg_base        = LPC_USB_BASE,
    .max_num_ep          = USB_MAX_EP_NUM,
    .mem_base            = (uint32_t) usb_RomDriver_buffer,
    .mem_size            = USB_ROM_SIZE,

    .USB_Configure_Event = USB_Configure_Event,
    .USB_Reset_Event     = USB_Reset_Event
  };

  USB_CORE_DESCS_T DeviceDes =
  {
    .device_desc      = (uint8_t*) &USB_DeviceDescriptor,
    .string_desc      = (uint8_t*) &USB_StringDescriptor,
    .full_speed_desc  = (uint8_t*) &USB_FsConfigDescriptor,
    .high_speed_desc  = (uint8_t*) &USB_FsConfigDescriptor,
    .device_qualifier = NULL
  };

  /* Start USB hardware initialisation */
  USBD_API->hw->Init(&g_hUsb, &DeviceDes, &usb_param);

  /* Initialise the class driver(s) */
  Ccid_Init(g_hUsb, &USB_FsConfigDescriptor.CCID_Interface);

  /* Enable the USB interrupt */
  NVIC_EnableIRQ(USB_IRQn);

  /* Perform USB soft connect */
  USBD_API->hw->Connect(g_hUsb, 1);

  return LPC_OK;
}

/**************************************************************************/
/*!
    @brief Redirect the USB IRQ handler to the ROM handler
*/
/**************************************************************************/
void USB_IRQHandler(void) {
  USBD_API->hw->ISR(g_hUsb);
}
