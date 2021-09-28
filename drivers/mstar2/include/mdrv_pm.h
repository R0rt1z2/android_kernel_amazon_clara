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
/// @file   Mdrv_mtlb.h
/// @brief  MTLB Driver Interface
/// @author MStar Semiconductor Inc.
///
///////////////////////////////////////////////////////////////////////////////////////////////////

// -----------------------------------------------------------------------------
// Linux Mhal_mtlb.h define start
// -----------------------------------------------------------------------------
#ifndef __DRV_PM_H__
#define __DRV_PM_H__

//-------------------------------------------------------------------------------------------------
//  Structure and Enum
//-------------------------------------------------------------------------------------------------
typedef enum
{
    E_PM_FAIL       = 0,      // fail
    E_PM_OK         = 1,      // success
    E_PM_TIMEOUT    = 2,      // timeout

} PM_Result;

typedef enum
{
    E_PM_STATE_STORE_INFO,
    E_PM_STATE_SUSPEND_PREPARE,
    E_PM_STATE_POWER_OFF_AC,
    E_PM_STATE_POWER_OFF_DC,
    E_PM_STATE_POWER_ON_DC,
} PM_STATE;

typedef struct _MSTAR_PM_DEV{
    unsigned long flag;
}MSTAR_PM_DEV;

#define CRC_KERNEL_BUF            (3)
#define MAX_BUF_WAKE_IR           (32)
#define MAX_BUF_WAKE_IR2          (16)
#define MAX_BUF_WAKE_MAC_ADDRESS  (6)
#define PM_MAX_BUF_WAKE_IR_WAVEFORM_ADDR    (4)
#define PM_MAX_BUF_RESERVERD                (4)

#define PM_STR_MODE         0x02
#define PM_STANDBY_MODE     0x01

#define PM_KEY_DEFAULT                  (0x0000)
#define PM_KEY_POWER_DOWN               (0x0001)
#define PM_KEY_SAR                      (0x0002)
#define PM_KEY_LED                      (0x0003)
#define PM_KEY_IR                       (0x0004)
#define PM_KEY_BT                       (0x0012)
#define PM_KEY_EWBS                     (0x0013)

///Define PM Power Down Mode
#define E_PM_STANDBY    0       /// PM Power Down Mode is Standby
#define E_PM_SLEEP      1       /// PM Power Down Mode is Sleep
#define E_PM_DEEP_SLEEP 2       /// PM Power Down Mode is Deep Sleep
#define E_PM_NORMAL     3

#define E_PM_LAST_TWOSTAGE_POWERDOWN    3

//Define PM IR Version
#define IR_INI_VERSION_NUM          0x20
#define IR_NO_INI_VERSION_NUM       0x10

/// Define PM wake-up parameter
typedef struct
{
    u8 bPmWakeEnableIR         : 1;
    u8 bPmWakeEnableSAR        : 1;
    u8 bPmWakeEnableGPIO0      : 1;
    u8 bPmWakeEnableGPIO1      : 1;
    u8 bPmWakeEnableUART1      : 1;
    u8 bPmWakeEnableSYNC       : 1;
    u8 bPmWakeEnableESYNC      : 1;
    u8 bPmWakeEnableRTC0       : 1;

    u8 bPmWakeEnableRTC1       : 1;
    u8 bPmWakeEnableDVI0       : 1;
    u8 bPmWakeEnableDVI2       : 1;
    u8 bPmWakeEnableCEC        : 1;
    u8 bPmWakeEnableAVLINK     : 1;
    u8 bPmWakeEnableMHL        : 1;
    u8 bPmWakeEnableWOL        : 1;
    u8 bPmWakeEnableCM4        : 1;

    u8 u8PmWakeIR[MAX_BUF_WAKE_IR];        //For PM IR Wake-up key define
    u8 u8PmWakeIR2[MAX_BUF_WAKE_IR2];      //For PM IR Wake-up key 2 define
    u8 u8PmWakeMACAddress[MAX_BUF_WAKE_MAC_ADDRESS];    //For PM WOL Wake-up MAC Addr define

    u8 u8PmStrMode;                       //For STR Power Mode, defined in EN_PM_STR_MODE
    u8 bLxCRCMiu[CRC_KERNEL_BUF];         //For STR CRC check, TRUE for MIU1, FALSE for MIU0
    u32 u32LxCRCAddress[CRC_KERNEL_BUF];  //For STR CRC check, memory address
    u32 u32LxCRCSize[CRC_KERNEL_BUF];     //For STR CRC check, memory size
    u8 u8PmWakeEnableWOWLAN;
    u8 u8PmWakeWOWLANPol;
    u8 u8PmWakeKeyShotCountAddr[PM_MAX_BUF_WAKE_IR_WAVEFORM_ADDR]; //For PM IR Wake-up key waveform define
    u8 u8HdmiByPass;
    u8 u8Reserved[PM_MAX_BUF_RESERVERD];
}PM_WakeCfg_t;

typedef struct
{
    u8 u8GpioNum;   /* Disable: 0xFF. */
    u8 u8Polarity;
} PM_WoBT_WakeCfg;

typedef struct
{
    u8 u8GpioNum;   /* Disable: 0xFF. */
    u8 u8Polarity;
} PM_WoEWBS_WakeCfg;

typedef struct
{
    u8 u8UpBnd;   //upper bound
    u8 u8LoBnd;  //low bound
} SAR_BndCfg;

/// define SAR Kpd Configuration
typedef struct
{
    u8 u8SARChID;
    SAR_BndCfg tSARChBnd;
    u8 u8KeyLevelNum;
    u8 u8KeyThreshold[8];
    u8 u8KeyCode[8];
} SAR_RegCfg;

/// Define PM mode
typedef struct
{
    /// Power Down Mode
    u8 u8PowerDownMode;
    u8 u8WakeAddress;

} PM_PowerDownCfg_t ;

/// Define PM configuration
typedef struct
{

    PM_WakeCfg_t stPMWakeCfg;
    PM_PowerDownCfg_t stPMPowerDownCfg;

} PM_Cfg_t;

#define DRAM_MMAP_SZ                (0x00010000UL)      // 64KB

//-------------------------------------------------------------------------------------------------
//  Function and Variable
//-------------------------------------------------------------------------------------------------
/* Function via file system. */
ssize_t MDrv_PM_Read_Key(u16 u16Key, const char *buf);
ssize_t MDrv_PM_Write_Key(u16 u16Key, const char *buf, size_t size);

u8 MDrv_PM_GetPowerOnKey(void);
PM_Result MDrv_PM_CopyBin2Sram(void);
PM_Result MDrv_PM_CopyBin2Dram(void);
PM_Result MDrv_PM_Suspend(PM_STATE state);
PM_Result MDrv_PM_Resume(PM_STATE state);
PM_Result MDrv_PM_SetSRAMOffsetForMCU(void);
void MDrv_SetDram(u32 u32Addr, u32 u32Size);
void MDrv_SetData(u32 u32Addr, u32 u32Size);
PM_Result MDrv_PM_GetCfg(PM_Cfg_t* pstPMCfg);
PM_Result MDrv_PM_SetCfg(PM_Cfg_t* pstPMCfg);
#ifdef CONFIG_MSTAR_SYSFS_BACKLIGHT
PM_Result MDrv_PM_TurnoffBacklight(void);
#endif
PM_Result MDrv_PM_SetPMCfg(u8 u8PmStrMode);
PM_Result MDrv_PM_SetSRAMOffsetForMCU_DC(void);
void MDrv_PM_Show_PM_Info(void);
ssize_t MDrv_PM_Show_PM_WOL_EN(const char *buf);
void MDrv_PM_SetPM_WOL_EN(const char *buf);
PM_Result MDrv_PM_PMCfg_Init(void);
void MDrv_PM_Set_PowerOffLed(u8 u8LedPad);
u8 MDrv_PM_Get_PowerOffLed(void);
ssize_t MDrv_PM_Set_IRCfg(const u8 *buf, size_t size);
ssize_t MDrv_PM_Set_IR2Cfg(const u8 *buf, size_t size);
void MDrv_PM_WakeIrqMask(u8 mask);
#endif
