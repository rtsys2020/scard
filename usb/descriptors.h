/**************************************************************************/
/*!
    @file     descriptors.h
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
#ifndef _DESCRIPTORS_H_
#define _DESCRIPTORS_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "app_usbd_cfg.h"
#include "mw_usbd_rom_api.h"
#include "usbd.h"
#include "ccid.h"

#define CFG_ENABLE_USB
#define CFG_USB_STRING_MANUFACTURER       "Aladdin"
#define CFG_USB_STRING_PRODUCT            "Token"
#define CFG_USB_VENDORID                  (0x1FC9)

/* USB Serial uses the MCUs unique 128-bit chip ID via an IAP call = 32 hex chars */
#define USB_STRING_SERIAL_LEN     32

#define USB_STRING_LEN(n) (2 + ((n)<<1))

typedef PRE_PACK struct POST_PACK _USB_STR_DESCRIPTOR
{
  USB_COMMON_DESCRIPTOR LangID;
  uint16_t strLangID[1];

  USB_COMMON_DESCRIPTOR Manufacturer;
  uint16_t strManufacturer[sizeof(CFG_USB_STRING_MANUFACTURER)-1]; // exclude null-character

  USB_COMMON_DESCRIPTOR Product;
  uint16_t strProduct[sizeof(CFG_USB_STRING_PRODUCT)-1]; // exclude null-character

  USB_COMMON_DESCRIPTOR Serial;
  uint16_t strSerial[USB_STRING_SERIAL_LEN];
} USB_STR_DESCRIPTOR;

// USB Interface Assosication Descriptor
#define  USB_DEVICE_CLASS_IAD        USB_DEVICE_CLASS_MISCELLANEOUS
#define  USB_DEVICE_SUBCLASS_IAD     0x02
#define  USB_DEVICE_PROTOCOL_IAD     0x01

// USB Interface Association Descriptor
typedef PRE_PACK struct POST_PACK _USB_INTERFACE_ASSOCIATION_DESCRIPTOR
{
  uint8_t bLength;           /**< Size of descriptor*/
  uint8_t bDescriptorType;   /**< Other_speed_Configuration Type*/

  uint8_t bFirstInterface;   /**< Index of the first associated interface. */
  uint8_t bInterfaceCount;   /**< Total number of associated interfaces. */

  uint8_t bFunctionClass;    /**< Interface class ID. */
  uint8_t bFunctionSubClass; /**< Interface subclass ID. */
  uint8_t bFunctionProtocol; /**< Interface protocol ID. */

  uint8_t iFunction;         /**< Index of the string descriptor describing the interface association. */
} USB_INTERFACE_ASSOCIATION_DESCRIPTOR;

#ifndef USB_PRODUCT_ID
#define USB_PRODUCT_ID                  (0x2000 | 1)
#endif
typedef struct
{
  USB_CONFIGURATION_DESCRIPTOR                Config;
  USB_INTERFACE_DESCRIPTOR                    CCID_Interface;
  USB_FUNCTIONAL_DESCRIPTOR                   CCID_Functional;
  USB_ENDPOINT_DESCRIPTOR                     CCID_Interrupt;
  USB_ENDPOINT_DESCRIPTOR                     CCID_BulkIN;
  USB_ENDPOINT_DESCRIPTOR                     CCID_BulkOUT;
  unsigned char                               ConfigDescTermination;
} USB_FS_CONFIGURATION_DESCRIPTOR;

extern const USB_DEVICE_DESCRIPTOR USB_DeviceDescriptor;
extern const USB_FS_CONFIGURATION_DESCRIPTOR USB_FsConfigDescriptor;
extern USB_STR_DESCRIPTOR USB_StringDescriptor;


#ifdef __cplusplus
}
#endif

#endif
