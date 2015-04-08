/**************************************************************************/
/*!
    @file     descriptors.c
    @author   Thach Ha (tinyusb.net)

    @section DESCRIPTION

    Descriptors for USB initialisation and identification

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
#include "descriptors.h"

/* USB Standard Device Descriptor */
ALIGNED(4) const USB_DEVICE_DESCRIPTOR USB_DeviceDescriptor =
{
  .bLength            = sizeof(USB_DEVICE_DESCRIPTOR),
  .bDescriptorType    = USB_DEVICE_DESCRIPTOR_TYPE,
  .bcdUSB             = 0x0200,
  .bDeviceClass       = 0x00,
  .bDeviceSubClass    = 0x00,
  .bDeviceProtocol    = 0x00,

  .bMaxPacketSize0    = USB_MAX_PACKET0,

  .idVendor           = CFG_USB_VENDORID,
  .idProduct          = USB_PRODUCT_ID,
  .bcdDevice          = 0x00,
  .iManufacturer      = 0x01,
  .iProduct           = 0x02,
  .iSerialNumber      = 0x03,
  .bNumConfigurations = 0x01
};

ALIGNED(4) const USB_FS_CONFIGURATION_DESCRIPTOR USB_FsConfigDescriptor =
{
    .Config =
    {
        .bLength             = sizeof(USB_CONFIGURATION_DESCRIPTOR),
        .bDescriptorType     = USB_CONFIGURATION_DESCRIPTOR_TYPE,

        .wTotalLength        = sizeof(USB_FS_CONFIGURATION_DESCRIPTOR) - 1, // exclude termination
        .bNumInterfaces      = 1,

        .bConfigurationValue = 0,
        .iConfiguration      = 0x00,
        .bmAttributes        = USB_CONFIG_BUS_POWERED,
        .bMaxPower           = USB_CONFIG_POWER_MA(500)
    },

    .CCID_Interface =
    {
        .bLength            = sizeof(USB_INTERFACE_DESCRIPTOR),
        .bDescriptorType    = USB_INTERFACE_DESCRIPTOR_TYPE,
        .bInterfaceNumber   = 0,
        .bAlternateSetting  = 0x00,
        .bNumEndpoints      = 3,
        .bInterfaceClass    = 0x0B,
        .bInterfaceSubClass = 0x00,
        .bInterfaceProtocol = 0x00,
        .iInterface         = 0x00
    },

    .CCID_Functional = {
        bLength:                sizeof(USB_FUNCTIONAL_DESCRIPTOR),
        bDescriptorType:        0x21,
        bcdCCID:                0x0110,
        bMaxSlotIndex:          0,
        bVoltageSupport:        0x02,       // 3.0 V
        dwProtocols:            0x00000003, // Indicates the supported protocol types: 00000001h T=0, 00000002h T=1
        dwDefaultClock:         4000,       // 4MHz, not relevant, fixed for legacy reason
        dwMaximumClock:         4000,       // 4MHz, not relevant, fixed for legacy reason
        bNumClockSupported:     0,          // Default clock, not relevant, fixed for legacy reason
        dwDataRate:             9600,       // 9600 bps, not relevant, fixed for legacy reason
        dwMaxDataRate:          9600,       // 9600 bps, not relevant, fixed for legacy reason
        bNumDataRatesSupported: 0,          // Default data rate, not relevant, fixed for legacy reason
        dwMaxIFSD:              CCID_DATA_SZ, // Maximum size of a block accepted from card by the reader. Indicates the maximum IFSD supported by the USB-ICC for protocol T=1. For T=0 any value may be given.
        dwSynchProtocols:       0,          // ISO7816-3, not relevant, fixed to for legacy reason
        dwMechanical:           0,          // No special characteristic, not relevant, fixed to for legacy reason
        //dwFeatures: 0x00020842, = 2+40+800+20000
        //dwFeatures:             (0x02+0x08 + 0x10+0x20+0x40 + 0x020000), // Page 19
        dwFeatures: (0x02+0x04+0x08+0x20+0x40+0x020000),
        //dwFeatures:             (0x02+0x08 + 0x10+0x20+0x80 + 0x020000), // Page 19
        /* For extended APDU level the value shall be between 261 + 10 (header)
         * and 65544 +10, otherwise the minimum value is the wMaxPacketSize
         * of the Bulk-OUT endpoint */
        dwMaxCCIDMessageLength: (CCID_HEADER_SZ + CCID_DATA_SZ), // The value shall be between (dwMaxIFSD + 10) and (65544 +10)
        bClassGetResponse:      0xFF,       // Echoes the class of the APDU
        bClassEnvelope:         0xFF,       // Echoes the class of the APDU
        wLcdLayout:             0,          // No LCD, not relevant, fixed for legacy reason
        bPinSupport:            0,          // No PIN pad, not relevant, fixed for legacy reason
        bMaxCCIDBusySlots:      1           // One slot is busy, the USB-ICC is regarded as a single slot CCID
    },

    .CCID_Interrupt =
    {
        .bLength          = sizeof(USB_ENDPOINT_DESCRIPTOR),
        .bDescriptorType  = USB_ENDPOINT_DESCRIPTOR_TYPE,
        .bEndpointAddress = EP_DIR_IN | 1,
        .bmAttributes     = USB_ENDPOINT_TYPE_INTERRUPT,
        .wMaxPacketSize   = 8,
        .bInterval        = 0x10,
    },

    .CCID_BulkIN =
    {
        .bLength          = sizeof(USB_ENDPOINT_DESCRIPTOR),
        .bDescriptorType  = USB_ENDPOINT_DESCRIPTOR_TYPE,
        .bEndpointAddress = EP_DIR_IN | 3,
        .bmAttributes     = USB_ENDPOINT_TYPE_BULK,
        .wMaxPacketSize   = 64,
    },

    .CCID_BulkOUT =
    {
        .bLength          = sizeof(USB_ENDPOINT_DESCRIPTOR),
        .bDescriptorType  = USB_ENDPOINT_DESCRIPTOR_TYPE,
        .bEndpointAddress = EP_DIR_OUT | 2,
        .bmAttributes     = USB_ENDPOINT_TYPE_BULK,
        .wMaxPacketSize   = 64,
    },

    .ConfigDescTermination = 0,
};

ALIGNED(4) USB_STR_DESCRIPTOR USB_StringDescriptor =
{
    .LangID = { .bLength = 0x04, .bDescriptorType = USB_STRING_DESCRIPTOR_TYPE },
    .strLangID= {0x0409}, // US English
    .Manufacturer = { .bLength = USB_STRING_LEN(sizeof(CFG_USB_STRING_MANUFACTURER)-1), .bDescriptorType = USB_STRING_DESCRIPTOR_TYPE },
    .Product = { .bLength = USB_STRING_LEN(sizeof(CFG_USB_STRING_PRODUCT)-1), .bDescriptorType = USB_STRING_DESCRIPTOR_TYPE },
    .Serial = { .bLength = USB_STRING_LEN(USB_STRING_SERIAL_LEN), .bDescriptorType = USB_STRING_DESCRIPTOR_TYPE },
};
