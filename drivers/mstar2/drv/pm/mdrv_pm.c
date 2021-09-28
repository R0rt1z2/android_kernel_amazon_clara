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
/// file    mdrv_mpool.c
/// @brief  Memory Pool Control Interface
/// @author MStar Semiconductor Inc.
///////////////////////////////////////////////////////////////////////////////////////////////////

//-------------------------------------------------------------------------------------------------
//  Include Files
//-------------------------------------------------------------------------------------------------
#include <generated/autoconf.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/init.h>
#include <linux/sched.h>
#include <linux/kernel.h>
#include <linux/fs.h>
#include <linux/errno.h>
#include <linux/delay.h>
#include <linux/kdev_t.h>
#include <linux/slab.h>
#include <linux/mm.h>
#include <linux/ioport.h>
#include <linux/interrupt.h>
#include <linux/workqueue.h>
#include <linux/poll.h>
#include <linux/wait.h>
#include <linux/cdev.h>
#include <linux/time.h>  //added
#include <linux/timer.h> //added
#include <linux/device.h>
#include <linux/version.h>
#include <linux/vmalloc.h>
#include <linux/binfmts.h>
#include <asm/io.h>
#include <asm/types.h>
#include <asm/cacheflush.h>

#if defined(CONFIG_COMPAT)
#include <linux/compat.h>
#endif

#include "power.h"

#include "mdrv_mstypes.h"
#include "mdrv_pm.h"
#include "mhal_pm.h"
#include "mdrv_system.h"
#include "mdrv_gpio.h"
#include "PM.h"
#include "RT_PM.h"

extern void SerPrintf(char *fmt, ...);
extern char *idme_get_product_name(void);

//--------------------------------------------------------------------------------------------------
//  Forward deabc123tion
//--------------------------------------------------------------------------------------------------
#define MDRV_PM_DEBUG(fmt, args...)  printk(KERN_ERR "[%s][%d] " fmt, __func__, __LINE__, ## args)

#define RTPM_BUFF_SZ 128

#define INVALID_GPIO_NUM        (0xFF)
#define DEFAULT_BT_GPIO_NUM     (0x05)

/* MMAP lay out. */
#define PM_OFFSET_BT            (0x0070)
#define PM_OFFSET_EWBS          (0x0075)
#define PM_OFFSET_POWER_DOWN    (0x00A0)
#define PM_OFFSET_SAR0          (0x00C0)
#define PM_OFFSET_SAR1          (0x00E0)
#define PM_OFFSET_LED           (0x0140)
#define PM_OFFSET_IR_VER        (0x0150)

#define PM_SUPPORT_SAR_KEYS_MAX     5

//--------------------------------------------------------------------------------------------------
//  Local variable
//--------------------------------------------------------------------------------------------------

//-------------------------------------------------------------------------------------------------
//  Golbal variable
//-------------------------------------------------------------------------------------------------
unsigned long gPM_DramAddr = 0x10000;
unsigned long gPM_DramSize = DRAM_MMAP_SZ;
unsigned long gPM_DataAddr = 0;
unsigned long gPM_DataSize = 0x1000;
static char pm_path[CORENAME_MAX_SIZE]={0};
static char pm_path_exist = 0;
static char gPM_Buf[0x10000];
static atomic_t PM_WOL_EN = ATOMIC_INIT(0);

static unsigned int     gRTPM_Enable = 0;
static unsigned int     gRTPM_Live = 1;
static unsigned long    gRTPM_Addr = 0;
static char             gRTPM_Path[RTPM_BUFF_SZ] = {0};

static PM_Cfg_t gstPMCfg;
static PM_WoBT_WakeCfg      gstWoBTCfg = {0};
static PM_WoEWBS_WakeCfg    gstWoEWBSCfg = {0};
static SAR_RegCfg gstSARCfg;
static SAR_RegCfg gstSARCfg1;
static u8 u8LedPara[10];
static u8 u8PowerOffLed = 0xFF;
static u8 u8IRVersion = IR_NO_INI_VERSION_NUM;
static u8 u8IRCfg[32] = {0};
static u8 u8IR2Cfg[16] = {0};
static phys_addr_t *remap_PM_addr;
static phys_addr_t *remap_config_addr;


//-------------------------------------------------------------------------------------------------
//  Data structure
//-------------------------------------------------------------------------------------------------

//-------------------------------------------------------------------------------------------------
//  Local function
//-------------------------------------------------------------------------------------------------
static unsigned long _MDrv_PM_Ba2Pa(unsigned long ba)
{
    unsigned long pa = 0;

    if ((ba > ARM_MIU0_BUS_BASE) && (ba <= ARM_MIU1_BUS_BASE))
        pa = ba - ARM_MIU0_BUS_BASE + ARM_MIU0_BASE_ADDR;
    else if ((ba > ARM_MIU1_BUS_BASE) && (ba <= ARM_MIU2_BUS_BASE))
        pa = ba - ARM_MIU1_BUS_BASE + ARM_MIU1_BASE_ADDR;
    else
        MDRV_PM_DEBUG("ba=0x%tX, pa=0x%tX.\n", (size_t)ba, (size_t)pa);

    return pa;
}

static unsigned long _MDrv_PM_Pa2Ba(unsigned long pa)
{
    unsigned long ba = 0;

    if ((pa > ARM_MIU0_BASE_ADDR) && (pa <= ARM_MIU1_BASE_ADDR))
        ba = pa + ARM_MIU0_BUS_BASE - ARM_MIU0_BASE_ADDR;
    else if ((pa > ARM_MIU1_BASE_ADDR) && (pa <= ARM_MIU2_BASE_ADDR))
        ba = pa + ARM_MIU1_BUS_BASE - ARM_MIU1_BASE_ADDR;
    else
        MDRV_PM_DEBUG("pa=0x%tX, ba=0x%tX.\n", (size_t)pa, (size_t)ba);

    return ba;
}

static PM_Result _MDrv_PM_Get_RTPM_Path(char *path)
{
    int ret = E_PM_OK;
    struct file *fp = NULL;
    mm_segment_t fs;
    loff_t pos = 0;
    int blk = 1;
    char buff[RTPM_BUFF_SZ] = {0};

    /* Find /dev/block/platform/mstar_mci.0/by-name/RTPM exist. */
    snprintf(buff, RTPM_BUFF_SZ, "/dev/block/platform/mstar_mci.0/by-name/RTPM");
    fp = filp_open(buff, O_RDONLY, 0);
    if (!IS_ERR(fp))
    {
        strncpy(path, buff, RTPM_BUFF_SZ);
        filp_close(fp, NULL);
        return E_PM_OK;
    }

    /* Store fs. */
    fs = get_fs();
    set_fs(KERNEL_DS);

    /* Find /dev/mmcblk0p#. */
    do
    {
        memset(buff, '\0', RTPM_BUFF_SZ);
        snprintf(buff, RTPM_BUFF_SZ, "/sys/block/mmcblk0/mmcblk0p%d/uevent", blk);
        fp = filp_open(buff, O_RDONLY, 0);
        if (IS_ERR(fp))
        {
            MDRV_PM_DEBUG("RTPM path not found.\n");
            ret = E_PM_FAIL;
            goto _MDrv_PM_Get_RTPM_Path_End;
        }
        memset(buff, '\0', RTPM_BUFF_SZ);
        vfs_read(fp, buff, RTPM_BUFF_SZ, &pos);
        filp_close(fp, NULL);

        /* Check found. */
        if (strnstr(buff, "PARTNAME=RTPM", RTPM_BUFF_SZ))
        {
            memset(buff, '\0', RTPM_BUFF_SZ);
            snprintf(buff, RTPM_BUFF_SZ, "/dev/mmcblk0p%d", blk);
            fp = filp_open(buff, O_RDONLY, 0);
            if (!IS_ERR(fp))
            {
                snprintf(path, RTPM_BUFF_SZ, "/dev/mmcblk0p%d", blk);
                filp_close(fp, NULL);
            }
            else
            {
                snprintf(path, RTPM_BUFF_SZ, "/dev/mmcblk%d", blk);
            }
            break;
        }
        else
        {
            pos = 0;
            blk++;
        }
    } while (1);

_MDrv_PM_Get_RTPM_Path_End:
    /* Restore fs. */
    set_fs(fs);

    return ret;
}

static PM_Result _MDrv_PM_Copy_RTPM_Bin(unsigned long dst_addr, char *src_path)
{
#if 0 // SOPHIA-3836: Use RT_PM.h instead of RT_PM.bin
    struct file *fp = NULL;
    mm_segment_t fs;
    loff_t pos = 0;
    phys_addr_t *va = 0;

    /* Check data init. */
    if ((strlen(src_path) == 0) || (dst_addr == 0))
    {
        MDRV_PM_DEBUG("RTPM data error: path=%s, addr=0x%lX.\n", src_path, dst_addr);
        return E_PM_FAIL;
    }

    /*
     * Prepare flow:
     * 1. open fd.
     * 2. ioremap_wc address.
     * 3. store fs.
     */
    fp = filp_open(src_path, O_RDONLY, 0);
    if (IS_ERR(fp))
    {
        MDRV_PM_DEBUG("filp_open() fail.\n");
        return E_PM_FAIL;
    }
    va = ioremap_wc(dst_addr, DRAM_MMAP_SZ);
    if (va == NULL)
    {
		MDRV_PM_DEBUG("ioremap_wc() fail.\n");
        return E_PM_FAIL;
    }
    fs = get_fs();
    set_fs(KERNEL_DS);

    /* Copy form RTPM partition to DRAM address. */
    vfs_read(fp, (char *)va, DRAM_MMAP_SZ, &pos);

    /*
     * Complete to do:
     * 1. restore fs.
     * 2. un-ioremap_wc address.
     * 3. close fd.
     */
    set_fs(fs);
    iounmap(va);
    filp_close(fp, NULL);
#else
	phys_addr_t *va = 0;

	/* Check data init. */
	if (dst_addr == 0) {
		MDRV_PM_DEBUG("RTPM data error: addr=0x%lX.\n", dst_addr);
		return E_PM_FAIL;
	}

	va = ioremap_wc (dst_addr, DRAM_MMAP_SZ);
	if (va == NULL) {
		MDRV_PM_DEBUG("ioremap_wc() fail.\n");
		return E_PM_FAIL;
	}

	if (RT_PM_bin_len <= 0) {
		MDRV_PM_DEBUG("RTPM lenth abnormal. Len: %d\n", RT_PM_bin_len);
		return E_PM_FAIL;
	}

	if (RT_PM_bin_len > DRAM_MMAP_SZ) {
		MDRV_PM_DEBUG("RTPM later than buffer. Len: %d\n", RT_PM_bin_len);
		return E_PM_FAIL;
	}

	/* Copy form RTPM partition to DRAM address. */
	memcpy((void *)va, RT_PM_bin, DRAM_MMAP_SZ);

	/*
	* Complete to do:
	*/
	iounmap(va);
#endif
	return E_PM_OK;
}

static PM_Result _MDrv_PM_Set_Data(u16 u16Key, void *pSrc, u8 u8Size)
{
    PM_Result eRet = E_PM_OK;
    phys_addr_t *pVa = NULL;

    /*
     * [byte-0]      : Command index for check.
     * [byte-1 ~ ...]: Original info.
     */
    switch (u16Key)
    {
        case PM_KEY_BT:
			pVa = (remap_config_addr + PM_OFFSET_BT);
            break;

        case PM_KEY_EWBS:
			pVa = (remap_config_addr + PM_OFFSET_EWBS);
            break;

        default:
            MDRV_PM_DEBUG("u16Key=0x%X not support.\n", u16Key);
            eRet = E_PM_FAIL;
            break;
    }

    /* Check mapping result. */
    if (eRet == E_PM_OK)
    {
		if (pVa == NULL)
        {
            MDRV_PM_DEBUG("ioremap_wc() fail.\n");
            eRet = E_PM_FAIL;
        }
        else
        {
			*((u8 *)pVa + 0) = (u8)u16Key;
			memcpy((void *)((u8 *)pVa + 1), pSrc, u8Size);
			//iounmap(pVa);
        }
    }

    return eRet;
}

//-------------------------------------------------------------------------------------------------
//  Golbal function
//-------------------------------------------------------------------------------------------------
ssize_t MDrv_PM_Read_Key(u16 u16Key, const char *buf)
{
    ssize_t tSize = 0;
    void *pPtr = NULL;
    u32 u32Idx = 0;

    switch (u16Key)
    {
        case PM_KEY_BT:
            tSize = sizeof(PM_WoBT_WakeCfg);
            pPtr = &gstWoBTCfg;
            break;

        case PM_KEY_POWER_DOWN:
            tSize = sizeof(PM_PowerDownCfg_t);
            pPtr = &gstPMCfg.stPMPowerDownCfg;
            break;

        case PM_KEY_SAR:
            tSize = sizeof(SAR_RegCfg);
            pPtr = &gstSARCfg;
            break;

        case PM_KEY_LED:
            tSize = sizeof(u8LedPara);
            pPtr = &u8LedPara;
            break;

        case PM_KEY_IR:
            tSize += scnprintf((char *)buf, 31, "IR Version: 0x%02x\nIR Data(Hex):", u8IRVersion);
            for (u32Idx = 0; u32Idx < sizeof(u8IRCfg); u32Idx++)
                tSize += scnprintf((char *)(buf + tSize), 5, "  %02x", u8IRCfg[u32Idx]);
            tSize += scnprintf((char *)(buf + tSize), 2, "\n");
            goto MDrv_PM_Read_Key_End;
            break;

        default:
            tSize = sizeof(PM_WakeCfg_t);
            pPtr = &gstPMCfg.stPMWakeCfg;
            break;
    }

    memcpy((void *)buf, (const void *)pPtr, tSize);

MDrv_PM_Read_Key_End:
    return tSize;
}

ssize_t MDrv_PM_Write_Key(u16 u16Key, const char *buf, size_t size)
{
    ssize_t tSize = 0;
    void *pPtr = NULL;

    switch (u16Key)
    {
        case PM_KEY_BT:
            tSize = sizeof(PM_WoBT_WakeCfg);
            pPtr = &gstWoBTCfg;
            break;

        case PM_KEY_POWER_DOWN:
            tSize = sizeof(PM_PowerDownCfg_t);
            pPtr = &gstPMCfg.stPMPowerDownCfg;
            break;

        case PM_KEY_SAR:
            tSize = sizeof(SAR_RegCfg);
            pPtr = &gstSARCfg;
            break;

        case PM_KEY_LED:
            tSize = sizeof(u8LedPara);
            pPtr = &u8LedPara;
            break;

        case PM_KEY_IR:
            tSize = sizeof(u8IRVersion);
            pPtr = &u8IRVersion;
            break;

        default:
            tSize = sizeof(PM_WakeCfg_t);
            pPtr = &gstPMCfg.stPMWakeCfg;
            break;
    }

    /* Check data size align. */
    if (size != tSize)
        MDRV_PM_DEBUG("Data(0x%X) un-align: input_size=0x%X, internal_size=0x%X\n", u16Key, size, tSize);
    else
        memcpy((void *)pPtr, (const void *)buf, tSize);

    return tSize;
}

u8 MDrv_PM_GetPowerOnKey(void)
{
    return MHal_PM_GetPowerOnKey();
}

void MDrv_PM_Set_PowerOffLed(u8 u8LedPad)
{
    MDRV_PM_DEBUG("LedPadNum = %d\n",u8LedPad);
    u8PowerOffLed = u8LedPad;
    return;
}

u8 MDrv_PM_Get_PowerOffLed(void)
{
    return u8PowerOffLed;
}

ssize_t MDrv_PM_Set_IRCfg(const u8 *buf, size_t size)
{
    u8IRVersion = IR_INI_VERSION_NUM;
    memcpy((void *)u8IRCfg, buf, sizeof(u8IRCfg));

    return sizeof(u8IRCfg);
}

EXPORT_SYMBOL(MDrv_PM_Set_IRCfg);

ssize_t MDrv_PM_Set_IR2Cfg(const u8 *buf, size_t size)
{
	u8IRVersion = IR_INI_VERSION_NUM;
	memcpy((void *)u8IR2Cfg, buf, sizeof(u8IR2Cfg));

	return sizeof(u8IR2Cfg);
}


EXPORT_SYMBOL(MDrv_PM_Set_IR2Cfg);

static u8 MDrv_PM_Get_IRVersion(void)
{
    return u8IRVersion;
}

static void MDrv_PM_Copy_IRCfg(PM_Cfg_t* pstPMCfg)
{
#if  0//for debug used
    u8 i = 0;
#endif
    if (MDrv_PM_Get_IRVersion() == IR_INI_VERSION_NUM)
    {
        MDRV_PM_DEBUG("IRVersion = 0x20,copy u8IRCfg to u8PmWakeIR!\n");
        memcpy((void*)&(pstPMCfg->stPMWakeCfg.u8PmWakeIR[PM_SUPPORT_SAR_KEYS_MAX]), \
        (void*)&(u8IRCfg[PM_SUPPORT_SAR_KEYS_MAX]), (sizeof(u8IRCfg) -PM_SUPPORT_SAR_KEYS_MAX));
	memcpy((void *)&(pstPMCfg->stPMWakeCfg.u8PmWakeIR2[0]), \
	(void *)&(u8IR2Cfg[0]), (sizeof(u8IR2Cfg)));
#if 0 // for debug used
        for(i=0;i<MAX_BUF_WAKE_IR;i++)
        {
            printk( "[Kernel_pm]   IR List  = 0x%x\n", pstPMCfg->stPMWakeCfg.u8PmWakeIR[i]);
        }
	for (i = 0; i < 16; i++) {
		MDRV_PM_DEBUG("[Kernel_pm]   IR2 List  = 0x%x\n", pstPMCfg->stPMWakeCfg.u8PmWakeIR2[i]);
	}
#endif
    }
    else
    {
        MDRV_PM_DEBUG("IRVersion = 0x10,use origin IRCfg!\n");
    }
}

void MDrv_PM_Show_PM_Info(void)
{
    u8 i = 0;

    // stPMWakeCfg
    printk("======== stPMWakeCfg start ========\n");
    printk("EnableIR     = %x\n", gstPMCfg.stPMWakeCfg.bPmWakeEnableIR);
    printk("EnableSAR    = %x\n", gstPMCfg.stPMWakeCfg.bPmWakeEnableSAR);
    printk("EnableGPIO0  = %x\n", gstPMCfg.stPMWakeCfg.bPmWakeEnableGPIO0);
    printk("EnableGPIO1  = %x\n", gstPMCfg.stPMWakeCfg.bPmWakeEnableGPIO1);
    printk("EnableUART1  = %x\n", gstPMCfg.stPMWakeCfg.bPmWakeEnableUART1);
    printk("EnableSYNC   = %x\n", gstPMCfg.stPMWakeCfg.bPmWakeEnableSYNC);
    printk("EnableRTC0   = %x\n", gstPMCfg.stPMWakeCfg.bPmWakeEnableRTC0);
    printk("EnableRTC1   = %x\n", gstPMCfg.stPMWakeCfg.bPmWakeEnableRTC1);
    printk("EnableDVI0   = %x\n", gstPMCfg.stPMWakeCfg.bPmWakeEnableDVI0);
    printk("EnableDVI2   = %x\n", gstPMCfg.stPMWakeCfg.bPmWakeEnableDVI2);
    printk("EnableCEC    = %x\n", gstPMCfg.stPMWakeCfg.bPmWakeEnableCEC);
    printk("EnableAVLINK = %x\n", gstPMCfg.stPMWakeCfg.bPmWakeEnableAVLINK);
    printk("EnableMHL    = %x\n", gstPMCfg.stPMWakeCfg.bPmWakeEnableMHL);
    printk("EnableWOL    = %x\n", gstPMCfg.stPMWakeCfg.bPmWakeEnableWOL);
    printk("EnableCM4    = %x\n", gstPMCfg.stPMWakeCfg.bPmWakeEnableCM4);

    printk("PmStrMode = %x\n", gstPMCfg.stPMWakeCfg.u8PmStrMode);

    for(i = 0; i<8; i++)
        printk("IR List =  0x%x\n", gstPMCfg.stPMWakeCfg.u8PmWakeIR[i]);

    for(i = 0; i<8; i++)
        printk("IR List2 =  0x%x\n", gstPMCfg.stPMWakeCfg.u8PmWakeIR2[i]);

    for(i = 0; i<MAX_BUF_WAKE_MAC_ADDRESS; i++)
        printk("MAC = 0x%x\n", gstPMCfg.stPMWakeCfg.u8PmWakeMACAddress[i]);
    printk("======== stPMWakeCfg end   ========\n");

    // stPMPowerDownCfg
    printk("======== stPMPowerDownCfg start ======== \n");
    printk("PowerDownMode = 0x%x\n", gstPMCfg.stPMPowerDownCfg.u8PowerDownMode);
    printk("WakeAddress = 0x%x\n", gstPMCfg.stPMPowerDownCfg.u8WakeAddress);
    printk("======== stPMPowerDownCfg end   ======== \n");

    //SAR_RegCfg1
    printk("======== MDrv_SAR_CfgChInfo start ========= \n");
    printk("u8SARChID=0x%02X\n", gstSARCfg.u8SARChID);
    printk("tSARChBnd.u8UpBnd=0x%02x\n", gstSARCfg.tSARChBnd.u8UpBnd);
    printk("tSARChBnd.u8LoBnd=0x%02x\n", gstSARCfg.tSARChBnd.u8LoBnd);
    printk("u8KeyLevelNum=0x%02x\n", gstSARCfg.u8KeyLevelNum);
    for(i=0; i < gstSARCfg.u8KeyLevelNum; i++)
        printk("u8KeyThreshold[%d]=0x%02x\n", i, gstSARCfg.u8KeyThreshold[i]);
    for(i=0; i < gstSARCfg.u8KeyLevelNum; i++)
        printk("u8KeyCode[%d]=0x%02x\n", i, gstSARCfg.u8KeyCode[i]);

    printk("======== MDrv_SAR_CfgChInfo end   ========= \n");

    //LED_RegCfg
    printk("======== MDrv_LED_CfgChInfo start ========= \n");
    for(i=0; i < 10; i++)
        printk("u8LedPara[%d]=0x%02x\n",i,u8LedPara[i]);
    printk("======== MDrv_LED_CfgChInfo end   ========= \n");
}

ssize_t MDrv_PM_Show_PM_WOL_EN(const char *buf)
{
	char WOL_EN_VALUE = 0;
	WOL_EN_VALUE = atomic_read(&PM_WOL_EN);
	printk("Current Parameter of WOL_EN=%d \n", WOL_EN_VALUE);
	return scnprintf((char *)buf, 32, "%d\n", WOL_EN_VALUE);
}

void MDrv_PM_SetPM_WOL_EN(const char *buf)
{
	unsigned int WOL_EN = 0;
	int readCount = 0;

	readCount = sscanf(buf, "%d", &WOL_EN);
	if (readCount != 1) {
		printk("ERROR cannot read WOL_EN from [%s] \n", buf);
		return;
	}
	if (WOL_EN > 0x01) {
		printk("ERROR Parameter WOL_EN=%d \n", WOL_EN);
		return;
	}

	printk("Set Parameter WOL_EN=%d success\n", WOL_EN);
	atomic_set(&PM_WOL_EN, WOL_EN);
}
PM_Result MDrv_PM_SetSRAMOffsetForMCU(void)
{
    PM_Result result;
    u8 u8LedPad = 0;

    result = MHal_PM_CopyBin2Sram(gPM_DramAddr);
    if (result != E_PM_OK)
    {
        return result;
    }

    MHal_PM_SetDram2Register(gPM_DataAddr);

    #if(CONFIG_MSTAR_GPIO)
    u8LedPad = MDrv_PM_Get_PowerOffLed();
    MDRV_PM_DEBUG("Get LedPadNum = %d\n",u8LedPad);
    if(u8LedPad != 0xFF)
    {
        MDRV_PM_DEBUG("======= set red led on ========\n");
        // set red gpio on
        MDrv_GPIO_Set_Low(u8LedPad);
    }
    #endif

    result = MHal_PM_SetSRAMOffsetForMCU();

    return result;
}

PM_Result MDrv_PM_SetSRAMOffsetForMCU_DC(void)
{
    PM_Result result;
    u8 u8LedPad = 0;

    MDRV_PM_DEBUG("start MDrv_PM_SetSRAMOffsetForMCU_DC \n");
    result = MHal_PM_CopyBin2Sram(gPM_DramAddr);
    if (result != E_PM_OK)
    {
        return result;
    }

    MHal_PM_SetDram2Register(gPM_DataAddr);

    #if(CONFIG_MSTAR_GPIO)
    u8LedPad = MDrv_PM_Get_PowerOffLed();
    MDRV_PM_DEBUG("Get LedPadNum = %d\n",u8LedPad);
    if(u8LedPad != 0xFF)
    {
        MDRV_PM_DEBUG("======= set red led on ========\n");
        // set red gpio on
        MDrv_GPIO_Set_Low(u8LedPad);
    }
    #endif

    result = MHal_PM_SetSRAMOffsetForMCU_DC();
    MDRV_PM_DEBUG("finish MDrv_PM_SetSRAMOffsetForMCU_DC \n");

    return result;
}

PM_Result MDrv_PM_CopyBin2Sram(void)
{
    MHal_PM_RunTimePM_Disable_PassWord();

    pr_info("start MDrv_PM_CopyBin2Sram \n");
	SerPrintf("start MDrv_PM_CopyBin2Sram \n");
    if (remap_PM_addr == NULL)
    {
		pr_err("ioremap_nocache failed\n");
        return E_PM_FAIL;
    }

    memcpy(remap_PM_addr, gPM_Buf, gPM_DramSize);

    //iounmap(remap_PM_addr);

    return E_PM_OK;
}

PM_Result MDrv_PM_CopyBin2Dram(void)
{
#if 0  // SOPHIA-3836: Use PM.h instead of PM.bin
    struct file *fp = NULL;
    mm_segment_t fs;
    loff_t pos;

    pr_info("start MDrv_PM_CopyBin2Dram \n");

    if (pm_path_exist) {
        fp = filp_open(pm_path, O_RDONLY, 0);
        if (IS_ERR(fp)) {
		pr_err("Only for Fusion Project: power off and recovery!!!\n");
            fp = filp_open("/mnt/vendor/tvservice/glibc/bin/PM.bin", O_RDONLY, 0);
        }
    }
    else
        fp = filp_open("/config/PM.bin", O_RDONLY, 0);

    if (IS_ERR(fp)){
		pr_err("MDrv_PM_CopyBin2Dram filp_open failed\n");
        return E_PM_FAIL;
    }
    fs = get_fs();
    set_fs(KERNEL_DS);
    pos = 0;
    memset(gPM_Buf, 0, gPM_DramSize);
    vfs_read(fp, gPM_Buf, gPM_DramSize, &pos);

    filp_close(fp, NULL);
    set_fs(fs);
#else
	pr_info("[S] start MDrv_PM_CopyBin2Dram\n");
	if (gPM_Buf == NULL || PM_bin == NULL) {
		pr_err("address is NULL\n");
		return E_PM_FAIL;
	}

	if (PM_bin_len <= 0) {
		pr_err("PM len abnormal! Len: %d\n", PM_bin_len);
		return E_PM_FAIL;
	}

	if (PM_bin_len > gPM_DramSize) {
		pr_err("PM len large than buffer size! Len: %d\n", PM_bin_len);
		return E_PM_FAIL;
	}

	memset(gPM_Buf, 0, gPM_DramSize);
	memcpy(gPM_Buf, PM_bin, gPM_DramSize);
#endif
	return E_PM_OK;
}

PM_Result MDrv_PM_Suspend(PM_STATE state)
{
    PM_Result result = E_PM_OK;
    PM_Cfg_t stPMCfg;

    switch (state)
    {
        /* System can read mmc partition, but can be wake up (can't disable). */
        case E_PM_STATE_STORE_INFO:
            /* Get RTPM bin path. */
            if (gRTPM_Enable == 1)
            {
                /* Check path exist. */
                if (strlen(gRTPM_Path) == 0)
                {
                    result = _MDrv_PM_Get_RTPM_Path(gRTPM_Path);
                }

		if (result != E_PM_OK) {
			pr_err("_MDrv_PM_Get_RTPM_Path error \n");
			return result;
		}
            }

            /* Copy PM bin from mmc partition to DRAM. */
            if (!is_mstar_str())
            {
                result = MDrv_PM_CopyBin2Dram();
            }
            break;

        /* System can't be wake up, but also can't read mmc partition. */
        case E_PM_STATE_SUSPEND_PREPARE:
            /* Copy PM bin from DRAM to SRAM (copy bin after disable). */
            if (!is_mstar_str())
            {
			result = MDrv_PM_CopyBin2Sram();
			if (result != E_PM_OK) {
				pr_err("MDrv_PM_CopyBin2Sram error \n");
				return result;
			}
		}
            gRTPM_Live = 0;
            break;

        /* System IP can't work. */
        case E_PM_STATE_POWER_OFF_AC:
            memset(&stPMCfg, 0, sizeof(stPMCfg));
            MDrv_PM_GetCfg(&stPMCfg);
            stPMCfg.stPMWakeCfg.u8PmStrMode = 1;
            MDrv_PM_SetCfg(&stPMCfg);
        case E_PM_STATE_POWER_OFF_DC:
            memset(&stPMCfg, 0, sizeof(stPMCfg));
		result = MDrv_PM_GetCfg(&stPMCfg);
		if (result != E_PM_OK) {
			pr_err("MDrv_PM_GetCfg Error \n");
			break;
		}
		stPMCfg.stPMWakeCfg.bPmWakeEnableCM4 = gstPMCfg.stPMWakeCfg.bPmWakeEnableCM4;
		MDrv_PM_Copy_IRCfg(&stPMCfg);
		stPMCfg.stPMWakeCfg.bPmWakeEnableWOL = (u8) atomic_read(&PM_WOL_EN); // The WOL_EN value is controlled by device node.
		result = MDrv_PM_SetCfg(&stPMCfg);
		if (result != E_PM_OK) {
			pr_err("MDrv_PM_SetCfg Error \n");
			break;
		}
		if (!strcmp(idme_get_product_name(), "sophia") || !strcmp(idme_get_product_name(), "abc123")) {
			MDRV_PM_DEBUG("Product is %s\n", idme_get_product_name());
			MHal_PM_WriteReg16(0x1040, 0x01);//Sophia, abc123 IR Header
		} else if (!strcmp(idme_get_product_name(), "greta") || !strcmp(idme_get_product_name(), "abc123")) {
			MDRV_PM_DEBUG("Product is %s\n", idme_get_product_name());
			MHal_PM_WriteReg16(0x1040, 0x00);//Greta, abc123 IR Header
		} else {
			MDRV_PM_DEBUG("Unknown Product\n");
		}

		result = _MDrv_PM_Set_Data(PM_KEY_BT,    (void *)&gstWoBTCfg, sizeof(PM_WoBT_WakeCfg));
		result = _MDrv_PM_Set_Data(PM_KEY_EWBS,  (void *)&gstWoEWBSCfg, sizeof(PM_WoEWBS_WakeCfg));
		if (result != E_PM_OK) {
			pr_err("_MDrv_PM_Set_Data Error \n");
			break;
		}
			result = MDrv_PM_SetSRAMOffsetForMCU();
            break;

        default:
            result = E_PM_FAIL;
            break;
    }

    return result;
}

PM_Result MDrv_PM_Resume(PM_STATE state)
{
    PM_Result result = E_PM_OK;

    switch (state)
    {
        case E_PM_STATE_POWER_ON_DC:
            /* Copy RTPM bin and Enable RTPM. */
            if (gRTPM_Enable == 1)
            {
                /* Check data. */
                if ((strlen(gRTPM_Path) == 0) || (gRTPM_Addr == 0))
                {
                    result = E_PM_FAIL;
                    break;
                }

                /* Check PM live. */
                if (gRTPM_Live == 0)
                {
                    MHal_PM_Disable_8051();
                    result = _MDrv_PM_Copy_RTPM_Bin(gRTPM_Addr, gRTPM_Path);
                    MHal_PM_SetDRAMOffsetForMCU(_MDrv_PM_Ba2Pa(gRTPM_Addr));
                    gRTPM_Live = 1;
                }
            }
            break;
        default:
            result = E_PM_FAIL;
            break;
    }

    return result;
}

PM_Result MDrv_PM_SetPMCfg(u8 u8PmStrMode)
{
    if (remap_config_addr == NULL)
    {
		pr_err("ioremap_wc failed\n");
        return E_PM_FAIL;
    }
    printk("gPM_DramAddr = 0x%x, gPM_DramSize = 0x%x\n",gPM_DramAddr,gPM_DramSize);
    printk("gPM_DataAddr = 0x%x, gPM_DataSize = 0x%x\n",gPM_DataAddr,gPM_DataSize);
    MDrv_PM_Copy_IRCfg(&gstPMCfg);
    gstPMCfg.stPMWakeCfg.u8PmStrMode = u8PmStrMode;
    memcpy((void *)remap_config_addr, (void *)&(gstPMCfg.stPMWakeCfg), sizeof(PM_WakeCfg_t));
    memcpy((void *)remap_config_addr + PM_OFFSET_POWER_DOWN, (void *)&(gstPMCfg.stPMPowerDownCfg), sizeof(PM_PowerDownCfg_t));
    memcpy((void *)remap_config_addr + PM_OFFSET_SAR0, (void *)&(gstSARCfg), sizeof(SAR_RegCfg));
    memcpy((void *)remap_config_addr + PM_OFFSET_SAR1, (void *)&(gstSARCfg1), sizeof(SAR_RegCfg));
    memcpy((void *)remap_config_addr + PM_OFFSET_LED, (void *)u8LedPara, sizeof(u8LedPara));
    memcpy((void *)remap_config_addr + PM_OFFSET_IR_VER, (void *)&u8IRVersion, sizeof(u8IRVersion));

    //iounmap(remap_config_addr);
    _MDrv_PM_Set_Data(PM_KEY_BT,    (void *)&gstWoBTCfg, sizeof(PM_WoBT_WakeCfg));
    _MDrv_PM_Set_Data(PM_KEY_EWBS,  (void *)&gstWoEWBSCfg, sizeof(PM_WoEWBS_WakeCfg));
    return E_PM_OK;
}

PM_Result MDrv_PM_PMCfg_Init(void)
{
    u8 idx = 0;

    MDRV_PM_DEBUG("start MDrv_PM_PMCfg_Init \n");


    memset(&gstPMCfg, 0, sizeof(gstPMCfg));
    gstPMCfg.stPMWakeCfg.u8PmWakeEnableWOWLAN = INVALID_GPIO_NUM;
    memset(&gstWoBTCfg, 0, sizeof(gstWoBTCfg));
    gstWoBTCfg.u8GpioNum = DEFAULT_BT_GPIO_NUM;
    memset(&gstWoEWBSCfg, 0, sizeof(gstWoEWBSCfg));
    gstWoEWBSCfg.u8GpioNum = INVALID_GPIO_NUM;
    memset(&gstSARCfg, 0, sizeof(gstSARCfg));
    memset(&gstSARCfg1, 0, sizeof(gstSARCfg1));

    //WakeUpCfg
    gstPMCfg.stPMWakeCfg.bPmWakeEnableIR = 1;
    gstPMCfg.stPMWakeCfg.bPmWakeEnableSAR = 1;
    gstPMCfg.stPMWakeCfg.bPmWakeEnableRTC0 = 1;
    gstPMCfg.stPMWakeCfg.bPmWakeEnableRTC1 = 1;
    gstPMCfg.stPMWakeCfg.bPmWakeEnableCEC = 1;
    gstPMCfg.stPMWakeCfg.bPmWakeEnableGPIO0 = 1;

    for(idx=0; idx<MAX_BUF_WAKE_IR; idx++)
    {
        gstPMCfg.stPMWakeCfg.u8PmWakeIR[idx] = 0xff;
    }
    gstPMCfg.stPMWakeCfg.u8PmWakeIR[0] = 0x46;

    for(idx=0; idx<MAX_BUF_WAKE_IR2; idx++)
    {
        gstPMCfg.stPMWakeCfg.u8PmWakeIR2[idx] = 0xff;
    }
    gstPMCfg.stPMWakeCfg.u8PmWakeIR2[0] = 0x46;
    gstPMCfg.stPMWakeCfg.u8PmStrMode = PM_STANDBY_MODE; //standby

    //PowerDownCfg
    gstPMCfg.stPMPowerDownCfg.u8PowerDownMode = E_PM_SLEEP;
    gstPMCfg.stPMPowerDownCfg.u8WakeAddress = E_PM_LAST_TWOSTAGE_POWERDOWN; //two stage

    //SarCfg
    gstSARCfg.u8SARChID = 0x00;
    gstSARCfg.tSARChBnd.u8UpBnd = 0xff;
    gstSARCfg.tSARChBnd.u8LoBnd = 0xf0;
    gstSARCfg.u8KeyLevelNum = 0x08;

    gstSARCfg.u8KeyThreshold[0] = 0x10;
    gstSARCfg.u8KeyThreshold[1] = 0x2f;
    gstSARCfg.u8KeyThreshold[2] = 0x4d;
    gstSARCfg.u8KeyThreshold[3] = 0x71;
    gstSARCfg.u8KeyThreshold[4] = 0x92;
    gstSARCfg.u8KeyThreshold[5] = 0xab;
    gstSARCfg.u8KeyThreshold[6] = 0xc3;
    gstSARCfg.u8KeyThreshold[7] = 0xe7;

    gstSARCfg.u8KeyCode[0] = 0x46;
    gstSARCfg.u8KeyCode[1] = 0xa8;
    gstSARCfg.u8KeyCode[2] = 0xa4;
    gstSARCfg.u8KeyCode[3] = 0xa2;

    MDRV_PM_DEBUG("finish MDrv_PM_PMCfg_Init \n");
    return E_PM_OK;
}

void MDrv_PM_WakeIrqMask(u8 mask)
{
#ifdef CONFIG_KEYBOARD_MTK
    MHal_PM_WakeIrqMask(mask);
#endif
}
EXPORT_SYMBOL(MDrv_PM_WakeIrqMask);

//-------------------------------------------------------------------------------------------------
void MDrv_SetDram(u32 u32Addr, u32 u32Size)
{
    gPM_DramAddr = u32Addr;
    gPM_DramSize = u32Size;
	if (remap_PM_addr != NULL) {
		iounmap(remap_PM_addr);
	}
	remap_PM_addr = (phys_addr_t *)ioremap_wc(gPM_DramAddr + ARM_MIU0_BUS_BASE, gPM_DramSize);
}
EXPORT_SYMBOL(MDrv_SetDram);
//-------------------------------------------------------------------------------------------------
void MDrv_SetData(u32 u32Addr, u32 u32Size)
{
    gPM_DataAddr = u32Addr;
    gPM_DataSize = u32Size;
	if (remap_config_addr != NULL) {
		iounmap(remap_config_addr);
	}
	remap_config_addr = (phys_addr_t *)ioremap_wc(gPM_DataAddr + ARM_MIU0_BUS_BASE, gPM_DataSize);
}
EXPORT_SYMBOL(MDrv_SetData);
PM_Result MDrv_PM_GetCfg(PM_Cfg_t* pstPMCfg)
{
    if (remap_config_addr == NULL)
    {
		pr_err("ioremap_wc failed\n");
        return E_PM_FAIL;
    }

    memcpy((void *)&(pstPMCfg->stPMWakeCfg), (void *)remap_config_addr, sizeof(PM_WakeCfg_t));
    memcpy((void *)&(pstPMCfg->stPMPowerDownCfg), (void *)remap_config_addr + PM_OFFSET_POWER_DOWN, sizeof(PM_PowerDownCfg_t));

    //iounmap(remap_addr);
    return E_PM_OK;
}

PM_Result MDrv_PM_SetCfg(PM_Cfg_t* pstPMCfg)
{
    if (remap_config_addr == NULL)
    {
		pr_err("ioremap_wc failed\n");
        return E_PM_FAIL;
    }

    memcpy((void *)remap_config_addr, (void *)&(pstPMCfg->stPMWakeCfg), sizeof(PM_WakeCfg_t));
    memcpy((void *)remap_config_addr + PM_OFFSET_POWER_DOWN, (void *)&(pstPMCfg->stPMPowerDownCfg), sizeof(PM_PowerDownCfg_t));
    memcpy((void *)remap_config_addr + PM_OFFSET_IR_VER, (void *)&u8IRVersion, sizeof(u8IRVersion));

    //iounmap(remap_config_addr);
    return E_PM_OK;
}

#ifdef CONFIG_MSTAR_SYSFS_BACKLIGHT
PM_Result MDrv_PM_TurnoffBacklight(void)
{
    struct file *fp = NULL;
    char BACKLIGHT_BUFFER[] = "0";
    mm_segment_t fs;
    loff_t pos;

    MDRV_PM_DEBUG("start MDrv_PM_TurnoffBacklight\n");

    fp = filp_open("/sys/class/backlight/backlight/brightness", O_RDWR, 0);

    if (IS_ERR(fp)) {
        MDRV_PM_DEBUG("filp_open failed\n");
        return E_PM_FAIL;
    }
    fs = get_fs();
    set_fs(KERNEL_DS);
    pos = 0;
    vfs_write(fp, BACKLIGHT_BUFFER, strlen(BACKLIGHT_BUFFER), &pos);

    filp_close(fp, NULL);
    set_fs(fs);

    return E_PM_OK;
}
#endif

static int __init MDrv_PM_SetPMPath(char *str)
{
    if( str != NULL)
    {
        strncpy(pm_path, str, (CORENAME_MAX_SIZE-1));
        pm_path[sizeof(pm_path) - 1] = '\0';
        pm_path_exist = 1;
    }
    return 0;
}

static int __init MDrv_PM_Set_RTPM_Info(char *str)
{
    if (str != NULL)
    {
        sscanf(str, "%u, %lx", &gRTPM_Enable, &gRTPM_Addr);
    }
    return 0;
}

static int __init MDrv_PM_Init_Addr(void)
{
    printk("Init STR PM addr first\n");
    remap_config_addr = (phys_addr_t *)ioremap_wc(gPM_DataAddr + ARM_MIU0_BUS_BASE, gPM_DataSize);
    remap_PM_addr = (phys_addr_t *)ioremap_wc(gPM_DramAddr + ARM_MIU0_BUS_BASE, gPM_DramSize);
    return 0;
}


EXPORT_SYMBOL(MDrv_PM_SetPMPath);

module_init(MDrv_PM_Init_Addr);
early_param("pm_path", MDrv_PM_SetPMPath);
early_param("RTPM_INFO", MDrv_PM_Set_RTPM_Info);
