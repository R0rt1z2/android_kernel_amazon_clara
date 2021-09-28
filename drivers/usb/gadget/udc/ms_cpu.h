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
//      ms_cpu.h
//
// DESCRIPTION
//
// HISTORY
//
//------------------------------------------------------------------------------
#ifndef _MS_CPU_H_
#define _MS_CPU_H_

#ifdef __cplusplus
extern "C" {
#endif  //__cplusplus
//#include "mach/platform.h"

#define OTG_BIT0                    0x00000001
#define OTG_BIT1                    0x00000002
#define OTG_BIT2                    0x00000004
#define OTG_BIT3                    0x00000008
#define OTG_BIT4                    0x00000010
#define OTG_BIT5                    0x00000020
#define OTG_BIT6                    0x00000040
#define OTG_BIT7                    0x00000080
#define OTG_BIT8                    0x00000100
#define OTG_BIT9                    0x00000200
#define OTG_BIT10                   0x00000400
#define OTG_BIT11                   0x00000800
#define OTG_BIT12                   0x00001000
#define OTG_BIT13                   0x00002000
#define OTG_BIT14                   0x00004000
#define OTG_BIT15                   0x00008000
#define OTG_BIT16                   0x00010000
#define OTG_BIT17                   0x00020000
#define OTG_BIT18                   0x00040000
#define OTG_BIT19                   0x00080000
#define OTG_BIT20                   0x00100000
#define OTG_BIT21                   0x00200000
#define OTG_BIT22                   0x00400000
#define OTG_BIT23                   0x00800000
#define OTG_BIT24                   0x01000000
#define OTG_BIT25                   0x02000000
#define OTG_BIT26                   0x04000000
#define OTG_BIT27                   0x08000000
#define OTG_BIT28                   0x10000000
#define OTG_BIT29                   0x20000000
#define OTG_BIT30                   0x40000000
#define OTG_BIT31                   0x80000000

#define USB_REG_READ8(r)                readb((void *)(OTG0_BASE_ADDR + (r)))
#define USB_REG_READ16(r)               readw((void *)(OTG0_BASE_ADDR + (r)))
#define USB_REG_WRITE8(r, v)             writeb(v, (void *)(OTG0_BASE_ADDR + r))//OUTREG8(otgRegAddress + r, v)
#define USB_REG_WRITE16(r, v)            writew(v, (void *)(OTG0_BASE_ADDR + r))//OUTREG16(otgRegAddress + r, v)

#define UTMI_REG_READ8(r)               readb((void *)(UTMI_BASE_ADDR + (r)))
#define UTMI_REG_READ16(r)              readw((void *)(UTMI_BASE_ADDR + (r)))
#define UTMI_REG_WRITE8(r, v)            writeb(v, (void *)(UTMI_BASE_ADDR + r))//OUTREG8(utmiRegAddress + r, v)
#define UTMI_REG_WRITE16(r, v)           writew(v, (void *)(UTMI_BASE_ADDR + r))//OUTREG16(utmiRegAddress + r, v)

#define USBC_REG_READ8(r)               readb((void *)(USBC_BASE_ADDR + (r)))
#define USBC_REG_READ16(r)              readw((void *)(USBC_BASE_ADDR + (r)))
#define USBC_REG_WRITE8(r, v)            writeb(v, (void *)(USBC_BASE_ADDR + r))//OUTREG8(usbcRegAddress + r, v)
#define USBC_REG_WRITE16(r, v)           writew(v, (void *)(USBC_BASE_ADDR + r))//OUTREG16(usbcRegAddress + r, v)

#define FIFO_ADDRESS(e) (OTG0_BASE_ADDR + (e<<3) + M_FIFO_EP0)
#define FIFO_DATA_PORT  (OTG0_BASE_ADDR + M_REG_FIFO_DATA_PORT)


#ifdef BIG_ENDIAN
#define SWOP(X) ((X) = (((X)<<8)+((X)>>8)))
#define SWAP4(X) ((X) = ((X)<<24) + ((X)>>24) + (((X)>>8)&0x0000FF00) + (((X)<<8)&0x00FF0000))
#else
#define SWAP4(X) (X = X)
#define SWOP(X)  (X = X)
#endif

#define RETAILMSG(a, b) printk(b)
#define _T(a) (KERN_INFO a)
#define TEXT
#define TRUE	1
#define FALSE	0

#ifdef __cplusplus
}
#endif  //__cplusplus

#endif  /* _MS_CPU_H_ */

