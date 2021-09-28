/* SPDX-License-Identifier: GPL-2.0-only OR BSD-3-Clause */
/******************************************************************************
 *
 * This file is provided under a dual license.  When you use or
 * distribute this software, you may choose to be licensed under
 * version 2 of the GNU General Public License ("GPLv2 License")
 * or BSD License.
 *
 * GPLv2 License
 *
 * Copyright(C) 2019 MediaTek Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of version 2 of the GNU General Public License as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See http://www.gnu.org/licenses/gpl-2.0.html for more details.
 *
 * BSD LICENSE
 *
 * Copyright(C) 2019 MediaTek Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 *  * Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *  * Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 *  * Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 * A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 * HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 *****************************************************************************/

//------------------------------------------------------------------------------
// FILE
//      ms_usbmain.h
//
// DESCRIPTION
//      This header file define the exported main functions for MStar USB low level controls
//
// HISTORY
//      2008.8.1        Baker Chang     Initial Version
//
//------------------------------------------------------------------------------

#ifndef __MS_USBMAIN_H__
#define __MS_USBMAIN_H__

#ifdef __cplusplus
extern "C" {
#endif  //__cplusplus

//------------------------------------------------------------------------------
//  Include Files
//------------------------------------------------------------------------------
#include <asm/io.h>
#include "ms_drc.h"
#include "ms_cpu.h"

//------------------------------------------------------------------------------
//  Macros
//------------------------------------------------------------------------------
#define SET_CONTROL_LINE_STATE  0x22

//UTMI power control register
//
#define UTMI_REG_IREF_PDN       0x4000
#define UTMI_REG_SUSPEND_PDN    0xFF04

// Descriptor Types
#define DEVICE          0x01
#define CONFIGURATION   0x02
#define STRING          0x03
#define INTERFACE       0x04
#define ENDPOINT        0x05

#define CFGLEN 32
#ifdef EBOOT
#define USB_DMA_BUF_ADDR		0xA0040000
#else // !EBOOT
#define USB_DMA_BUF_ADDR		0xA0040000
#endif //EBOOT
#define USB_DMA_BUF_SIZE		(64*1024)
#define USB_EP0_MAX_PACKET_SIZE 64
#define USB_EP2_MAX_PACKET_SIZE 512
#define USB_DMA_MODE0   0
#define USB_DMA_MODE1   1

// Endpoint number
typedef enum{
    USB_EP_CONTROL = 0,
    USB_EP_TX,
    USB_EP_RX,
} IMAGE_TYPE_et;

//------------------------------------------------------------------------------
//  Variables
//------------------------------------------------------------------------------
extern USB_INFO_st g_USBInfo;

//------------------------------------------------------------------------------
//  Functions
//------------------------------------------------------------------------------
PUSB_INFO_st USBInit(u8 u8DeviceClass, u8 u8DeviceCap);
bool USBPollInterrupt(void);
bool USBPollTx(void);
bool USBPollRx(u8 *pData, u16 *size);
bool USBPollRxDMA(u8 *pData, u32 *pu32Size);

void USBKITLWaitForConnect(void);
void USB_ParseDRCIntrUsb(u16 intrusb, USB_INFO_st *pUsbInfo);

#ifdef __cplusplus
}
#endif  //__cplusplus

#endif
