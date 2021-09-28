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


#ifndef _HAL_RTC_H_
#define _HAL_RTC_H_


#include "mdrv_types.h"

typedef enum
{
    E_RTC_0,
    E_RTC_2,
}E_MS_RTC;

void MHAL_RTC_Reading(E_MS_RTC eRtc, bool bEnable); // Value to load into RTC counter
void MHAL_RTC_Loading (E_MS_RTC eRtc, bool bEnable); // Load enable for loading value into RTC counter
void MHAL_RTC_RESET(E_MS_RTC eRtc, bool bEnable); // RTC software reset (low active)
void MHAL_RTC_Counter(E_MS_RTC eRtc, bool bEnable); //Enable RTC Counter

void MHAL_RTC_Init(E_MS_RTC eRtc,U32 u32Xtal);
void MHAL_RTC_SetCounter(E_MS_RTC eRtc,U32 u32RtcSetCounter);
U32 MHAL_RTC_GetCounter(E_MS_RTC eRtc);
void MHAL_RTC_SetMatchCounter(E_MS_RTC eRtc,U32 u32RtcSetCounter);
U32 MHAL_RTC_GetMatchCounter(E_MS_RTC eRtc);
U16 MHAL_RTC_GetInterrupt_Status(E_MS_RTC eRtc);
void MHAL_RTC_ClearInterrupt_Status(E_MS_RTC eRtc);
int MHal_PM_RTC_Interrupt_Init(void);
void MHal_RTC_Enable_Interrupt(void);
void MHal_RTC_Disable_Interrupt(void);
U16 MHAL_SLEEP_SW_Dummy_REG_Read(void);
int MHal_RTC_Request_IRQ(irq_handler_t pCallback, void *dev_id);

#endif

