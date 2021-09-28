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
//      ms_config.h
//
// DESCRIPTION
//
// HISTORY
//
//------------------------------------------------------------------------------
#ifndef _MS_CONFIG_H_
#define _MS_CONFIG_H_

//#include <mach/hardware.h>
#include "msb250x_udc_reg.h"

#ifdef __cplusplus
extern "C" {
#endif  //__cplusplus

#if defined(CONFIG_ARCH_CEDRIC)
#define MSB250X_BASE_REG_RIU_PA		0xfd200000
#define usbcRegAddress   			(MSB250X_BASE_REG_RIU_PA+0x0700*2)
#define otgRegAddress   			(MSB250X_BASE_REG_RIU_PA+0x51000*2)
#define utmiRegAddress   			(MSB250X_BASE_REG_RIU_PA+0x3A80*2)
#endif
#if 0//defined(CONFIG_ARCH_CHICAGO)
#define MSB250X_BASE_REG_RIU_PA		0xfd000000
#define usbcRegAddress   			(MSB250X_BASE_REG_RIU_PA+0x2500*2)
#define otgRegAddress   			(MSB250X_BASE_REG_RIU_PA+0x2600*2)
#define utmiRegAddress   			(MSB250X_BASE_REG_RIU_PA+0x1F00*2)
#endif
#if defined(CONFIG_ARCH_INFINITY) || defined(CONFIG_ARCH_INFINITY3)
#define MSB250X_BASE_REG_RIU_PA		0xfd200000
#define usbcRegAddress   			(MSB250X_BASE_REG_RIU_PA+0x42300*2)
#define otgRegAddress   			(MSB250X_BASE_REG_RIU_PA+0x42500*2)
#define utmiRegAddress   			(MSB250X_BASE_REG_RIU_PA+0x42100*2)
#endif
#if defined(CONFIG_ARCH_CLEVELAND)
#define MSB250X_BASE_REG_RIU_PA		0xfd000000
#define usbcRegAddress   			(MSB250X_BASE_REG_RIU_PA+0x2500*2)
#define otgRegAddress   			(MSB250X_BASE_REG_RIU_PA+0x2600*2)
#define utmiRegAddress   			(MSB250X_BASE_REG_RIU_PA+0x10200*2)
#endif
#define OffShift        					1
#define otgNumEPDefs    				4
#define SCSI_BLOCK_NUM  			5000
#define SCSI_BLOCK_SIZE 			512
#define EVB_Board
#define MASS_BUFFER_SIZE 			(4 * 1024)
#define MAX_DMA_CHANNEL  			1

#define MASS_BUFFER_SIZE2 			0x10000
#define MASS_TRN_LEN 				8
#define Enable_Read_Write_Test
#define Force_Host_Mode
#define UVC_BULK_MODE

#ifdef __cplusplus
}
#endif  //__cplusplus

#endif  /* _MS_CONFIG_H_ */

