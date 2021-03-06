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

///////////////////////////////////////////////////////////////////////////////////////////////////
///
/// @file   mdrv_utopia2k_str_io.h
/// @brief  UTOPIA2K STR interface
/// @author MStar Semiconductor Inc.
///
///////////////////////////////////////////////////////////////////////////////////////////////////

#ifndef __DRV_UTOPIA2K_STR_IO_H__
#define __DRV_UTOPIA2K_STR_IO_H__

////////////////////////////////////////////////////////////////////////////////
// Header Files
////////////////////////////////////////////////////////////////////////////////

#include "MsTypes.h"

////////////////////////////////////////////////////////////////////////////////
// Define & data type
////////////////////////////////////////////////////////////////////////////////

enum utopia2k_str_power_mode {
    UTOPIA2K_STR_POWER_SUSPEND = 1,
    UTOPIA2K_STR_POWER_RESUME = 2,
    UTOPIA2K_STR_POWER_MAX,
};

typedef int (*FUtopiaSTR)(int u32PowerState, void* pModule);

////////////////////////////////////////////////////////////////////////////////
// Extern Function
////////////////////////////////////////////////////////////////////////////////

int mdrv_utopia2k_str_setup_function_ptr(void* pModuleTmp, FUtopiaSTR fpSTR);
int mdrv_utopia2k_str_wait_condition(const char* name, MS_U32 mode, MS_U32 stage);
int mdrv_utopia2k_str_send_condition(const char* name, MS_U32 mode, MS_U32 stage);

/*
 * ioctl operations
 */
#include <linux/ioctl.h>

#define UTOPIA2K_STR_IOC_MAGIC   'U'

#define UTOPIA2K_STR_IOC_SET_WKUP_SRC    _IOW(UTOPIA2K_STR_IOC_MAGIC, 0x00, unsigned short)
#define UTOPIA2K_STR_IOC_GET_WKUP_SRC    _IOR(UTOPIA2K_STR_IOC_MAGIC, 0x01, unsigned short)
#define UTOPIA2K_STR_IOC_REPORT_EVENT    _IO(UTOPIA2K_STR_IOC_MAGIC, 0x02)
#endif

