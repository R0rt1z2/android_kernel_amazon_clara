///////////////////////////////////////////////////////////////////////////////////////////////////
//
// * Copyright (c) 2006 - 2017 MStar Semiconductor, Inc.
// This program is free software.
// You can redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software Foundation;
// either version 2 of the License, or (at your option) any later version.
//
// This program is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY;
// without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
// See the GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License along with this program;
// if not, write to the Free Software Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA 02111-1307, USA.
//
///////////////////////////////////////////////////////////////////////////////////////////////////
///
/// @file   mdrv_util_io.c
/// @brief  mdrv_util Driver Interface
/// @author MStar Semiconductor Inc.
///
///////////////////////////////////////////////////////////////////////////////////////////////////

//-------------------------------------------------------------------------------------------------
//  Include Files
//-------------------------------------------------------------------------------------------------
#include <linux/cdev.h>
#include <linux/proc_fs.h>
#include <linux/platform_device.h>
#include <linux/module.h>
#include "mdrv_util.h"
#include "mst_devid.h"

//-------------------------------------------------------------------------------------------------
//  Driver Compiler Options
//-------------------------------------------------------------------------------------------------

//-------------------------------------------------------------------------------------------------
//  Local Defines
//-------------------------------------------------------------------------------------------------
#define MOD_UTIL_DEVICE_COUNT   1

#define MOD_RUTIL_PRI_ADJ       "priority_adjust"

DEFINE_MUTEX(mstar_util_lock);

typedef struct
{
    int s32Major;
    int s32Minor;
    struct cdev cdev;
    struct file_operations fops;
} UTIL_DEV;

//-------------------------------------------------------------------------------------------------
//  Local Variables
//-------------------------------------------------------------------------------------------------
static struct proc_dir_entry *proc_util_dir;

//-------------------------------------------------------------------------------------------------
//  Debug Functions
//-------------------------------------------------------------------------------------------------

//-------------------------------------------------------------------------------------------------
//  Local Functions
//-------------------------------------------------------------------------------------------------
static long mstar_util_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
    int ret = 0;
    mutex_lock(&mstar_util_lock);
    switch (cmd)
    {
        case 0:
            break;
        case IOCTL_MDRV_UTIL_SAVE_CURR_SCHED:
            MDrv_UTIL_SaveCurrentScheduler();
            ret = 0;
            break;
        case IOCTL_MDRV_UTIL_SET_SCHED:
            if (_IOC_SIZE(cmd) != sizeof(struct tune_table))
            {
                printk(KERN_WARNING "[mstarutil] Wrong size of parameter\n");
                return -EINVAL;
            }
            ret = MDrv_UTIL_SetScheduler(arg);
            break;
        case IOCTL_MDRV_UTIL_RESTORE_SCHED:
            ret = MDrv_UTIL_RestoreScheduler();
            break;
        default:
            printk(KERN_WARNING "[mstarutil] Not support command\n");
            ret = -EINVAL;
            break;
    }
    mutex_unlock(&mstar_util_lock);
    return ret;
}

#if defined(CONFIG_COMPAT)
static long mstarutil_compat_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
    return mstar_util_ioctl(filp,cmd,arg);
}
#endif

static UTIL_DEV util_dev =
{
    .s32Major = MDRV_MAJOR_UTIL,
    .s32Minor = MDRV_MINOR_UTIL,
    .cdev =
    {
        .kobj = {.name= MDRV_NAME_UTIL, },
        .owner = THIS_MODULE,
    },
    .fops =
    {
        .owner = THIS_MODULE,
        #ifdef HAVE_UNLOCKED_IOCTL
        .unlocked_ioctl = mstar_util_ioctl,
        #else
        .ioctl = mstar_util_ioctl,
        #endif

        #if defined(CONFIG_COMPAT)
        .compat_ioctl = mstarutil_compat_ioctl,
        #endif
    }
};

static int mstar_util_drv_probe(struct platform_device *pdev)
{
    pdev->dev.platform_data=NULL;
    return 0;
}

static int mstar_util_drv_remove(struct platform_device *pdev)
{
    pdev->dev.platform_data=NULL;
    return 0;
}

#if defined (CONFIG_ARM64)
static struct of_device_id mstarutil_of_device_ids[] =
{
         {.compatible = MDRV_NAME_UTIL},
         {},
};
#endif

static struct platform_driver mstar_util_driver =
{
    .probe      = mstar_util_drv_probe,
    .remove     = mstar_util_drv_remove,
    .driver = {
#if defined(CONFIG_ARM64)
                .of_match_table = mstarutil_of_device_ids,
#endif
                .name   = MDRV_NAME_UTIL,
                .owner  = THIS_MODULE,
    }
};

static int __init mstar_util_drv_init_module(void)
{
    int s32Ret;
    dev_t dev;
    struct proc_dir_entry *entry;

    if (util_dev.s32Major)
    {
        dev = MKDEV(util_dev.s32Major, util_dev.s32Minor);
        s32Ret = register_chrdev_region(dev, MOD_UTIL_DEVICE_COUNT, MDRV_NAME_UTIL);
    }
    else
    {
        s32Ret = alloc_chrdev_region(&dev, util_dev.s32Minor, MOD_UTIL_DEVICE_COUNT, MDRV_NAME_UTIL);
        util_dev.s32Major = MAJOR(dev);
    }

    cdev_init(&util_dev.cdev, &util_dev.fops);
    if (0 != (s32Ret = cdev_add(&util_dev.cdev, dev, MOD_UTIL_DEVICE_COUNT)))
        goto fail;

    if (!(proc_util_dir = proc_mkdir(MDRV_NAME_UTIL, NULL)))
        goto fail;
    if (!(entry = proc_create(MOD_RUTIL_PRI_ADJ, S_IRUSR | S_IWUSR, proc_util_dir, &util_dev.fops)))
        goto fail;

    return platform_driver_register(&mstar_util_driver);

fail:
    unregister_chrdev_region(dev, MOD_UTIL_DEVICE_COUNT);
    return -ENODEV;
}

static void __exit mstar_util_drv_exit_module(void)
{
    if (proc_util_dir)
    {
        remove_proc_entry(MOD_RUTIL_PRI_ADJ, proc_util_dir);
        remove_proc_entry(MDRV_NAME_UTIL, NULL);
        proc_util_dir = NULL;
    }

    cdev_del(&util_dev.cdev);
    unregister_chrdev_region(MKDEV(util_dev.s32Major, util_dev.s32Minor), MOD_UTIL_DEVICE_COUNT);
    platform_driver_unregister(&mstar_util_driver);
}

//-------------------------------------------------------------------------------------------------
//  Global Functions
//-------------------------------------------------------------------------------------------------
module_init(mstar_util_drv_init_module);
module_exit(mstar_util_drv_exit_module);
MODULE_LICENSE("GPL");

