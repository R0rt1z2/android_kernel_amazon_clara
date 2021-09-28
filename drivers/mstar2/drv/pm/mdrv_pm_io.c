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
/// file    mdrv_pm_io.c
/// @brief  PM I/O Control Interface
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
#include <linux/time.h>
#include <linux/timer.h>
#include <linux/platform_device.h>
#include <linux/proc_fs.h>
#include <linux/seq_file.h>
#include <linux/namei.h>
#include <linux/of.h>
#include <linux/of_device.h>

#include <asm/io.h>
#if defined(CONFIG_COMPAT)
#include <linux/compat.h>
#endif

#include "mdrv_pm_io.h"
#include "mdrv_pm.h"
#include "mst_devid.h"



//--------------------------------------------------------------------------------------------------
// Forward declaration
//--------------------------------------------------------------------------------------------------
static int _MDrv_PM_io_open(struct inode *inode, struct file *filp);
static int _MDrv_PM_io_release(struct inode *inode, struct file *filp);

#define     MDRV_PM_DEVICE_COUNT            1
#define     MDRV_PM_NAME                    "PM"


#ifdef HAVE_UNLOCKED_IOCTL
long _MDrv_PM_io_ioctl(struct file *filp, unsigned int cmd, unsigned long arg);
#else
int  _MDrv_PM_io_ioctl(struct inode *inode, struct file *filp, unsigned int cmd, unsigned long arg);
#endif
#if defined(CONFIG_COMPAT)
static long _Compat_MDrv_PM_io_ioctlOCtl(struct file *filp, unsigned int cmd, unsigned long arg);
#endif

//-------------------------------------------------------------------------------------------------
// Macros
//-------------------------------------------------------------------------------------------------

//-------------------------------------------------------------------------------------------------
//  Local Defines
//-------------------------------------------------------------------------------------------------

//-------------------------------------------------------------------------------------------------
//  Local Structurs
//-------------------------------------------------------------------------------------------------
typedef struct
{
    u32 Address;
    u32 Size;
} PM_DRAM_INFO;


typedef struct
{
    int s32Major;
    int s32Minor;
    struct cdev cdev;
    struct file_operations fops;
} PM_DEV;

PM_DEV _PMDev=
{
    .s32Major=               MDRV_MAJOR_PM,
    .s32Minor=               MDRV_MINOR_PM,
    .cdev=
    {
        .kobj=                  {.name= MDRV_NAME_PM, },
        .owner  =               THIS_MODULE,
    },
    .fops=
    {
        .open=                  _MDrv_PM_io_open,
        .release=               _MDrv_PM_io_release,
        #ifdef HAVE_UNLOCKED_IOCTL
        .unlocked_ioctl =       _MDrv_PM_io_ioctl,
        #else
        .ioctl =                _MDrv_PM_io_ioctl,
        #endif
        #if defined(CONFIG_COMPAT)
        .compat_ioctl = _Compat_MDrv_PM_io_ioctlOCtl,
        #endif
    },
};

//-------------------------------------------------------------------------------------------------
//  Global Variables
//-------------------------------------------------------------------------------------------------

//-------------------------------------------------------------------------------------------------
// Local Variables
//-------------------------------------------------------------------------------------------------

static struct class *pm_class;

//-------------------------------------------------------------------------------------------------
//  Debug Functions
//-------------------------------------------------------------------------------------------------

//-------------------------------------------------------------------------------------------------
//  Local Functions
//-------------------------------------------------------------------------------------------------


//-------------------------------------------------------------------------------------------------
//  Global Functions
//-------------------------------------------------------------------------------------------------
int _MDrv_PM_SetCodeDramAddr(struct file *filp, unsigned long arg)
{
    PM_DRAM_INFO stPM_temp;
    if(copy_from_user(&stPM_temp, (PM_DRAM_INFO __user *)arg, sizeof(PM_DRAM_INFO)))
    {
        return EFAULT;
    }
    if (stPM_temp.Size > DRAM_MMAP_SZ) {
	printk("\033[32m [%s, %s, %d] size: %x is larger than DRAM_MMAP_SZ. Set it as DRAM_MMAP_SZ(%x).\033[0m \n", __FILE__, __FUNCTION__, __LINE__, stPM_temp.Size, DRAM_MMAP_SZ);

	stPM_temp.Size = DRAM_MMAP_SZ;
    }
    MDrv_SetDram(stPM_temp.Address, stPM_temp.Size);

    printk("\033[32m [%s, %s, %d] address: %x size: %x \033[0m \n", __FILE__, __FUNCTION__, __LINE__, stPM_temp.Address, stPM_temp.Size);
    return 0;
}

//-------------------------------------------------------------------------------------------------
int _MDrv_PM_SetDataDramAddr(struct file *filp, unsigned long arg)
{
    PM_DRAM_INFO stPM_temp;
    if(copy_from_user(&stPM_temp, (PM_DRAM_INFO __user *)arg, sizeof(PM_DRAM_INFO)))
    {
        return EFAULT;
    }
    MDrv_SetData(stPM_temp.Address, stPM_temp.Size);
    printk("\033[32m [%s, %s, %d] address: %x size: %x \033[0m \n", __FILE__, __FUNCTION__, __LINE__, stPM_temp.Address, stPM_temp.Size);
    return 0;
}

int _MDrv_PM_EnableVoiceWakeUp(struct file *filp, unsigned long arg)
{
    int enVoiceWakeup = (int)arg;
    PM_WakeCfg_t buf;
    ssize_t ret = 0;
    memset(&buf,0,sizeof(PM_WakeCfg_t));
    ret = MDrv_PM_Read_Key(PM_KEY_DEFAULT, (char *)&buf);
    buf.bPmWakeEnableCM4 = enVoiceWakeup;
    ret = MDrv_PM_Write_Key(PM_KEY_DEFAULT, (char *)&buf, sizeof(PM_WakeCfg_t));
    return 0;
}

//-------------------------------------------------------------------------------------------------
// IOCtrl Driver interface functions
//-------------------------------------------------------------------------------------------------
int _MDrv_PM_io_open(struct inode *inode, struct file *filp)
{
    printk("Inside open \n");
    return 0;
}

//-------------------------------------------------------------------------------------------------
static int _MDrv_PM_io_release(struct inode *inode, struct file *filp)
{
    printk("Inside close \n");
    return 0;
}

//-------------------------------------------------------------------------------------------------
#ifdef HAVE_UNLOCKED_IOCTL
long _MDrv_PM_io_ioctl(struct file *filp, U32 u32Cmd, unsigned long u32Arg)
#else
int _MDrv_PM_io_ioctl(struct inode *inode, struct file *filp, unsigned long u32Cmd, unsigned long u32Arg)
#endif
{
    int retval;
    switch (u32Cmd)
    {
        case IOCTL_PM_SET_DRAM_CODE_ADDRESS:
            retval = _MDrv_PM_SetCodeDramAddr(filp, u32Arg);
            break;

        case IOCTL_PM_SET_DRAM_DATA_ADDRESS:
            retval = _MDrv_PM_SetDataDramAddr(filp, u32Arg);
            break;
        case IOCTL_PM_ENABLE_VOICE_WAKEUP:
            retval = _MDrv_PM_EnableVoiceWakeUp(filp, u32Arg);
            break;
        default:
            return -ENOTTY;

    }

    return 0;
}

//-------------------------------------------------------------------------------------------------
#if defined(CONFIG_COMPAT)
static long _Compat_MDrv_PM_io_ioctlOCtl(struct file *filp, unsigned int cmd, unsigned long arg)
{
    printk("\033[32m [%s, %s, %d] \033[0m \n", __FILE__, __FUNCTION__, __LINE__);
    switch (cmd)
    {
        case IOCTL_PM_SET_DRAM_CODE_ADDRESS:
        case IOCTL_PM_SET_DRAM_DATA_ADDRESS:
        case IOCTL_PM_ENABLE_VOICE_WAKEUP:
            return filp->f_op->unlocked_ioctl(filp, cmd,(unsigned long)compat_ptr(arg));
        default:
            return -ENOTTY;
    }

    return 0;
}
#endif

//-------------------------------------------------------------------------------------------------
static MSTAR_PM_DEV _st_pmdev={0};
static int _mstar_drv_pm_suspend(struct platform_device *dev, pm_message_t state)
{
    return 0;
}

//-------------------------------------------------------------------------------------------------
static int _mstar_drv_pm_resume(struct platform_device *dev)
{
    return 0;
}

//-------------------------------------------------------------------------------------------------
static struct of_device_id mstar_pm_of_device_ids[] = {
         {.compatible = "mstar-pm"},
         {},
};

//-------------------------------------------------------------------------------------------------
static int _mstar_drv_pm_probe(struct platform_device *pdev)
{
    int retval = 0;
    U8 u8LedPad = 0xFF;
    struct device_node  *node = pdev->dev.of_node;
    const struct of_device_id *match = of_match_device(of_match_ptr(mstar_pm_of_device_ids), &pdev->dev);

    if ( !(pdev->name) || strcmp(pdev->name,"Mstar-pm")
        || pdev->id!=0)
    {
        retval = -ENXIO;
    }

    if(match != NULL)//for dtb.bin config platform_device
    {
        pdev->dev.platform_data=&_st_pmdev;
        if(of_property_read_u8(node, "poweroff_led", &u8LedPad) >= 0)
        {
            if(u8LedPad != 0xFF)
            {
                MDrv_PM_Set_PowerOffLed(u8LedPad);
            }
        }
    }
    else //for chip_arch.c config  platform_device
    {
        u8* data = (u8*)dev_get_platdata(&pdev->dev);
        if (!data)
        {
            dev_err(&pdev->dev, "could not get resource\n");
        }
        else
        {
            u8LedPad = *data;
            printk("get platform_data u8LedPad = %d\n",u8LedPad);
            if(u8LedPad != 0xFF)
            {
                MDrv_PM_Set_PowerOffLed(u8LedPad);
            }
        }
    }
    return retval;
}

//-------------------------------------------------------------------------------------------------
static int _mstar_drv_pm_remove(struct platform_device *pdev)
{
    pdev->dev.platform_data=NULL;
    return 0;
}

//-------------------------------------------------------------------------------------------------
static struct platform_driver Mstar_pm_driver = {
    .probe      = _mstar_drv_pm_probe,
    .remove     = _mstar_drv_pm_remove,
    .suspend    = _mstar_drv_pm_suspend,
    .resume     = _mstar_drv_pm_resume,

    .driver = {
#if defined(CONFIG_OF)
    .of_match_table = mstar_pm_of_device_ids,
#endif
        .name   = "Mstar-pm",
        .owner  = THIS_MODULE,
    }
};

//-------------------------------------------------------------------------------------------------
// Module functions
//-------------------------------------------------------------------------------------------------
static ssize_t mstar_pm_show_WakeCfg(struct device * device, struct device_attribute * mattr, char * buf)
{
    printk("===========Mstar PM WakeCfg Show========\n");
    return MDrv_PM_Read_Key(PM_KEY_DEFAULT, buf);
}

static ssize_t mstar_pm_store_WakeCfg(struct device * device, struct device_attribute * mattr, const char * data, size_t len)
{
    printk("===========Mstar PM WakeCfg Store========\n");
    return MDrv_PM_Write_Key(PM_KEY_DEFAULT, data, len);
}

static ssize_t mstar_pm_show_WoBTCfg(struct device * device, struct device_attribute * mattr, char * buf)
{
    printk("===========Mstar PM WoBTCfg Show========\n");
    return MDrv_PM_Read_Key(PM_KEY_BT, buf);
}

static ssize_t mstar_pm_store_WoBTCfg(struct device * device, struct device_attribute * mattr, const char * data, size_t len)
{
    printk("===========Mstar PM WoBTCfg Store========\n");
    return MDrv_PM_Write_Key(PM_KEY_BT, data, len);
}

static ssize_t mstar_pm_show_WoEWBSCfg(struct device * device, struct device_attribute * mattr, char * buf)
{
    printk("===========Mstar PM WoEWBSCfg Show========\n");
    return MDrv_PM_Read_Key(PM_KEY_EWBS, buf);
}

static ssize_t mstar_pm_store_WoEWBSCfg(struct device * device, struct device_attribute * mattr, const char * data, size_t len)
{
    printk("===========Mstar PM WoEWBSCfg Store========\n");
    return MDrv_PM_Write_Key(PM_KEY_EWBS, data, len);
}

static ssize_t mstar_pm_show_PowerDownCfg(struct device * device, struct device_attribute * mattr, char * buf)
{
    printk("===========Mstar PM PowerDownCfg Show========\n");
    return MDrv_PM_Read_Key(PM_KEY_POWER_DOWN, buf);
}

static ssize_t mstar_pm_store_PowerDownCfg(struct device * device, struct device_attribute * mattr, const char * data, size_t len)
{
    printk("===========Mstar PM PowerDownCfg Store========\n");
    return MDrv_PM_Write_Key(PM_KEY_POWER_DOWN, data, len);
}

static ssize_t mstar_pm_show_SARCfg(struct device * device, struct device_attribute * mattr, char * buf)
{
    printk("===========Mstar PM SARCfg Show========\n");
    return MDrv_PM_Read_Key(PM_KEY_SAR, buf);
}

static ssize_t mstar_pm_store_SARCfg(struct device * device, struct device_attribute * mattr, const char * data, size_t len)
{
    printk("===========Mstar PM SARCfg Store========\n");
    return MDrv_PM_Write_Key(PM_KEY_SAR, data, len);
}

static ssize_t mstar_pm_show_LedCfg(struct device * device, struct device_attribute * mattr, char * buf)
{
    printk("===========Mstar PM LEDCfg Show========\n");
    return MDrv_PM_Read_Key(PM_KEY_LED, buf);
}

static ssize_t mstar_pm_store_LedCfg(struct device * device, struct device_attribute * mattr, const char * data, size_t len)
{
    printk("===========Mstar PM LEDCfg Store========\n");
    return MDrv_PM_Write_Key(PM_KEY_LED, data, len);
}

static ssize_t mstar_pm_show_IRCfg(struct device * device, struct device_attribute * mattr,char * buf)
{
    printk("===========Mstar PM IRCfg Show========\n");
    return MDrv_PM_Read_Key(PM_KEY_IR, buf);
}

static ssize_t mstar_pm_store_IRCfg(struct device * device, struct device_attribute * mattr, const char * data, size_t len)
{
    printk("===========Mstar PM IRCfg Store========\n");
    return MDrv_PM_Write_Key(PM_KEY_IR, data, len);
}

static ssize_t mstar_pm_show_PM_Info(struct device * device, struct device_attribute * mattr, char *buf)
{
    printk("===========Mstar PM Info Show========\n");
    MDrv_PM_Show_PM_Info();

    return 0;
}

static ssize_t mstar_pm_store_PM_Info(struct device * device, struct device_attribute * mattr, const char * data, size_t len)
{
    printk("===========Mstar PM Info Store========\n");
    return MDrv_PM_SetPMCfg(*data);
}

static ssize_t mstar_pm_show_PM_WOL_EN(struct device *device, struct device_attribute *mattr, char *buf)
{
	printk("===========Mstar PM WOL EN Show========\n");
	return MDrv_PM_Show_PM_WOL_EN(buf);
}

static ssize_t mstar_pm_set_PM_WOL_EN(struct device *device, struct device_attribute *mattr, const char *data, size_t len)
{
	printk("===========Mstar PM Info Set WOL EN========\n");
	MDrv_PM_SetPM_WOL_EN(data);
	return len;
}


static DEVICE_ATTR(WakeCfg,         S_IRUGO | S_IWUSR, mstar_pm_show_WakeCfg,       mstar_pm_store_WakeCfg);
static DEVICE_ATTR(WoBTCfg,         S_IRUGO | S_IWUSR, mstar_pm_show_WoBTCfg,       mstar_pm_store_WoBTCfg);
static DEVICE_ATTR(WoEWBSCfg,       S_IRUGO | S_IWUSR, mstar_pm_show_WoEWBSCfg,     mstar_pm_store_WoEWBSCfg);
static DEVICE_ATTR(PowerDownCfg,    S_IRUGO | S_IWUSR, mstar_pm_show_PowerDownCfg,  mstar_pm_store_PowerDownCfg);
static DEVICE_ATTR(SARCfg,          S_IRUGO | S_IWUSR, mstar_pm_show_SARCfg,        mstar_pm_store_SARCfg);
static DEVICE_ATTR(LEDCfg,          S_IRUGO | S_IWUSR, mstar_pm_show_LedCfg,        mstar_pm_store_LedCfg);
static DEVICE_ATTR(IRCfg,           S_IRUGO | S_IWUSR, mstar_pm_show_IRCfg,         mstar_pm_store_IRCfg);
static DEVICE_ATTR(PM_Info,         S_IRUGO | S_IWUSR, mstar_pm_show_PM_Info,       mstar_pm_store_PM_Info);
static DEVICE_ATTR(PM_WOL_EN,       S_IRUGO | S_IWUSR, mstar_pm_show_PM_WOL_EN,     mstar_pm_set_PM_WOL_EN);


MSYSTEM_STATIC int _MDrv_PMIO_ModuleInit(void)
{
    int s32Ret;
    dev_t  dev;
    struct device *pm_dev;

    pm_class = class_create(THIS_MODULE, MDRV_NAME_PM);
    if (IS_ERR(pm_class))
    {
        return PTR_ERR(pm_class);
    }

    if(_PMDev.s32Major)
    {
        dev = MKDEV(_PMDev.s32Major, _PMDev.s32Minor);
        s32Ret = register_chrdev_region(dev, MDRV_PM_DEVICE_COUNT, MDRV_PM_NAME);
    }
    else
    {
        s32Ret = alloc_chrdev_region(&dev, _PMDev.s32Minor, MDRV_PM_DEVICE_COUNT, MDRV_PM_NAME);
        _PMDev.s32Major = MAJOR(dev);
    }

    if (0 > s32Ret)
    {
        printk("Unable to get major %d\n", _PMDev.s32Major);
        class_destroy(pm_class);
        return s32Ret;
    }

    cdev_init(&_PMDev.cdev, &_PMDev.fops);
    if (0 != (s32Ret= cdev_add(&_PMDev.cdev, dev, MDRV_PM_DEVICE_COUNT)))
    {
        printk("Unable add a character device\n");
        unregister_chrdev_region(dev, MDRV_PM_DEVICE_COUNT);
        class_destroy(pm_class);
        return s32Ret;
    }

    /* initial the whole MBX Driver */
/*
    if(E_PM_OK != MDrv_MBX_Startup())
    {
        MBXIO_KDBG("Startup MBX Driver Failed! %d\n", _devMBX.s32Major);
        cdev_del(&_devMBX.cdev);
        unregister_chrdev_region(dev, MDRV_MBX_DEVICE_COUNT);
        return -ENOMEM;
    }

    DRV_MBX_LockIOCTL_Init();
*/
    pm_dev = device_create(pm_class, NULL, dev, NULL, MDRV_NAME_PM);
    platform_driver_register(&Mstar_pm_driver);

    device_create_file(pm_dev, &dev_attr_WakeCfg);
    device_create_file(pm_dev, &dev_attr_WoBTCfg);
    device_create_file(pm_dev, &dev_attr_WoEWBSCfg);
    device_create_file(pm_dev, &dev_attr_PowerDownCfg);
    device_create_file(pm_dev, &dev_attr_SARCfg);
    device_create_file(pm_dev, &dev_attr_LEDCfg);
    device_create_file(pm_dev, &dev_attr_IRCfg);
    device_create_file(pm_dev, &dev_attr_PM_Info);
	device_create_file(pm_dev, &dev_attr_PM_WOL_EN);

    MDrv_PM_PMCfg_Init();  //set default pm cfg
    return 0;
}


MSYSTEM_STATIC void _MDrv_PMIO_ModuleExit(void)
{
    /*de-initial the who MBX Driver */
 /*   MDrv_MBX_Exit(); */

    cdev_del(&_PMDev.cdev);
    unregister_chrdev_region(MKDEV(_PMDev.s32Major, _PMDev.s32Minor), MDRV_PM_DEVICE_COUNT);
    platform_driver_unregister(&Mstar_pm_driver);
    device_destroy(pm_class, MKDEV(_PMDev.s32Major, _PMDev.s32Minor));
    class_destroy(pm_class);
}

#if defined(CONFIG_MSTAR_MSYSTEM) || defined(CONFIG_MSTAR_MSYSTEM_MODULE)
#else//#if defined(CONFIG_MSTAR_MSYSTEM) || defined(CONFIG_MSTAR_MSYSTEM_MODULE)
module_init(_MDrv_PMIO_ModuleInit);
module_exit(_MDrv_PMIO_ModuleExit);

MODULE_AUTHOR("MSTAR");
MODULE_DESCRIPTION("PM ioctrl driver");
MODULE_LICENSE("GPL");
#endif
