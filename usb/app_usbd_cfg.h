/**************************************************************************/
/*!
    @file     app_usbd_cfg.h
    @author   Thach Ha (tinyusb.net)

    @section DESCRIPTION

    Common USB daemon config settings

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
#ifndef __APP_USBD_CFG_H__
#define __APP_USBD_CFG_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "romdriver/mw_usbd.h"

#define USB_MAX_IF_NUM                      (8)
#define USB_MAX_EP_NUM                      (5)

#define USB_FS_MAX_BULK_PACKET              (64)
#define USB_HS_MAX_BULK_PACKET              (USB_FS_MAX_BULK_PACKET) /* Full speed device only */

// Control Endpoint
#define USB_MAX_PACKET0                     (64)

#define EP_DIR_IN           0x80
#define EP_DIR_OUT          0x00

// Descriptor-type endpoint codes
#define EP_TYPE_ISOCHRONOUS 0x01
#define EP_TYPE_BULK        0x02
#define EP_TYPE_INTERRUPT   0x03

#ifdef __cplusplus
}
#endif

#endif  /* __USBCFG_H__ */
