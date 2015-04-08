/*
 * ccid.c
 *
 *  Created on: 08 апр. 2015 г.
 *      Author: RLeonov
 */

#include "ccid.h"
#include "sw_cmd_uart.h"

bool isConfigured;

ErrorCode_t InHanlder(USBD_HANDLE_T hUsb, void* data, uint32_t event) {
    UartSW_Printf("I\r");
//  if (USB_EVT_IN == event) {
//    uint16_t count;
//    count = fifo_readArray(&ff_cdc_tx, buffer, CDC_DATA_EP_MAXPACKET_SIZE);
//    USBD_API->hw->WriteEP(hUsb, CCID_EP_IN, buffer, count); // write data to EP
//    isConfigured = true;
//  }
  return LPC_OK;
}

ErrorCode_t OutHandler(USBD_HANDLE_T hUsb, void* data, uint32_t event) {
    UartSW_Printf("O\r");
    if (USB_EVT_OUT == event) {
//        count = USBD_API->hw->ReadEP(hUsb, EP_DIR_OUT, Setup);
//        for (i=0; i<count; i++) {
//            fifo_write(&ff_cdc_rx, buffer+i);
//        }
  }
  return LPC_OK;
}

void Ccid_Init(USBD_HANDLE_T hUsb, USB_INTERFACE_DESCRIPTOR const *const pControlIntfDesc) {
   /* register Bulk IN & OUT endpoint interrupt handler */
  USBD_API->core->RegisterEpHandler(hUsb , ((EP_DIR_IN & 0x0F) << 1) + 1 , InHanlder, NULL);
  USBD_API->core->RegisterEpHandler(hUsb , (EP_DIR_OUT & 0x0F) << 1     , OutHandler , NULL);

  /* Update memory variables */
  isConfigured = false;
  UartSW_Printf("CCID Init\r");
}
