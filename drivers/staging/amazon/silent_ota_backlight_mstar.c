/**
* Copyright (c) 2017 Amazon, Inc.
* This program is free software. You can redistribute it and/or modify it under the terms of
* the GNU General Public License as published by the Free Software Foundation;
* either version 2 of the License, or (at your option) any later version.
*
* This program is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY;
* without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
* See the GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License along with this program;
*/

#include <linux/types.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/string.h>
#include <linux/ctype.h>
#include <linux/silent_ota_backlight.h>
#include <linux/sign_of_life.h>
#include <linux/seq_file.h>
#include <linux/proc_fs.h>
#include <linux/uaccess.h>
#include <linux/of.h>
#include <linux/input-event-codes.h>

#include "mdrv_gpio.h"
#include "mhal_gpio.h"
#if defined(CONFIG_MSTAR_PM)
#include "mdrv_pm.h"
#endif

extern char *idme_get_product_name(void);
extern char *idme_get_memc(void);
extern S32 MDrv_HW_IIC_WriteBytes(U8 u8Port, U8 u8SlaveIdIIC, U8 u8AddrSizeIIC, U8 *pu8AddrIIC, U32 u32BufSizeIIC, U8 *pu8BufIIC);


static DEFINE_MUTEX(backlight_lock);

#define REG_ADDR(addr)                           (*((volatile unsigned short int*)(mstar_pm_base + (addr << 1))))
#define PM_ADDR_OFFSET				 0x060a

/* read 2 byte */
#define REG_RR(_reg_)                            ({REG_ADDR(_reg_); })

/* write 2 byte */
#define REG_W2B(_reg_, _val_)    \
		do { REG_ADDR(_reg_) = (_val_); } while (0)

/*=============================================================================
 * Global Variables
 *============================================================================= */
static unsigned int backlight_status; /* 1: on, 0: off */
static unsigned int ursa_mute_status; /* 1: mute, 0: un-mute */
static unsigned int boot_complete;
static unsigned int backlight_inverse;
static unsigned int backlight_gpio;
static unsigned int is_OLED_tv;

#define T22_MFC_I2C_PORT 0x03
#define T22_MFC_I2C_ADDR 0x40

void update_Ursa_Mute_State(bool mute_state)
{
	ursa_mute_status = (unsigned int)mute_state;
	pr_info("[%s] mute: %d, OLED: %d\n", __FUNCTION__, ursa_mute_status, is_OLED_tv);
}
EXPORT_SYMBOL(update_Ursa_Mute_State);

static void set_Ursa_Mute(unsigned char on)
{
	u8 cmd[] = {0x03, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00};
	if (on) {
		cmd[0] = 0x43;
		cmd[2] = 0x01;
	} else {
		cmd[0] = 0x33;
		cmd[2] = 0x00;
	}
	ursa_mute_status = on;
	pr_info("set_Ursa_Mute %d\n", on);
	MDrv_HW_IIC_WriteBytes(T22_MFC_I2C_PORT, T22_MFC_I2C_ADDR, 0, NULL, 0x7, cmd);
}
static void set_backlight(unsigned char on)
{
	if (on) {
		pr_info("Turn on backlight from kernel \n");
		if (is_OLED_tv == TRUE) {
			set_Ursa_Mute(FALSE);
		} else {
			if (backlight_inverse)
				MDrv_GPIO_Set_Low(backlight_gpio);
			else
				MDrv_GPIO_Set_High(backlight_gpio);
		}
		/* Record the backlight status in dummy register, which may be used in bootloader */
		REG_W2B(PM_ADDR_OFFSET, REG_RR(PM_ADDR_OFFSET) | PM_SPARE_SCREEN_STATE);
		backlight_status = 1;
	} else {
		pr_info("Turn off backlight from kernel \n");
		if (is_OLED_tv == TRUE) {
			set_Ursa_Mute(TRUE);
		} else {
			if (backlight_inverse)
				MDrv_GPIO_Set_High(backlight_gpio);
			else
				MDrv_GPIO_Set_Low(backlight_gpio);
		}
		/* Record the backlight status in dummy register */
		REG_W2B(PM_ADDR_OFFSET, REG_RR(PM_ADDR_OFFSET) & ~PM_SPARE_SCREEN_STATE);
		backlight_status = 0;
	}
	return;
}

static u32 get_backlight(void)
{
	if (backlight_inverse) {
		pr_info("inverse backlight \n");
		return !MDrv_GPIO_Pad_Read(backlight_gpio);
	} else {
		pr_info("Do not inverse backlight \n");
		return MDrv_GPIO_Pad_Read(backlight_gpio);
	}
}

/*
This fucntion will toggle the backlight when user pressing IR power key before boot completed,
and it will turn on the backlight when user press "Netflix", "Amazon Video" and "Amazon Music"
keys.

*/
unsigned int toggle_backlight(unsigned int keycode)
{
	if (unlikely(!boot_complete)) {
		pr_info("toggle_backlight keycode 0x%x\n", keycode);
		mutex_lock(&backlight_lock);
		if (keycode == KEY_POWER) {
			set_backlight(!backlight_status);
		} else if ((keycode == KEY_HOME) ||
				(keycode == KEY_APP1) ||
				(keycode == KEY_APP2) ||
				(keycode == KEY_APP3) ||
				(keycode == KEY_APP4) ||
				(keycode == KEY_WAKEUP)) {
			if (!backlight_status)
				set_backlight(1);
		} else if (keycode == KEY_SLEEP) {
			if (backlight_status)
				set_backlight(0);
		} else {
			pr_info("toggle_backlight ignore\n");
		}
		mutex_unlock(&backlight_lock);
	}
	return 0;
}

static int backlight_status_proc_show(struct seq_file *m, void *v)
{
	if (is_OLED_tv == TRUE) {
		backlight_status = (!ursa_mute_status & get_backlight());
	} else {
		backlight_status = get_backlight();
	}
	seq_printf(m, "%d\n", backlight_status);
	return 0;
}
/* This fucntion will be called when user pressing the power key on TV keypad */
ssize_t backlight_status_proc_write(struct file *file, const char __user *userbuf, size_t count, loff_t *ppos)
{
	char buf[64];
	unsigned int set;
	count = min_t(size_t, count, (sizeof(buf)-1));
	if (copy_from_user(buf, userbuf, count))
		return -EFAULT;

	buf[count] = '\0';

	if (strict_strtol(buf, 0, &set) != 0) {
		pr_err("value is not correct for backlight_status_proc_write ,buf %s\n", buf);
		return -EINVAL;
	}

	pr_info("backlight_status_proc_write set %d \n", set);
	set_backlight(set);
	return -1;
}

static int backlight_status_proc_open(struct inode *inode, struct file *file)
{
	return single_open(file, backlight_status_proc_show, NULL);
}

static const struct file_operations backlight_status_proc_fops = {
	.open		= backlight_status_proc_open,
	.read		= seq_read,
	.write		= backlight_status_proc_write,
	.llseek		= seq_lseek,
	.release	= single_release,
};

/* This fucntion will be called when checking if system is in silent OTA mode */

static int silent_ota_status_proc_show(struct seq_file *m, void *v)
{
	seq_printf(m, "%d\n", mstar_get_silent_ota_flag());
	return 0;
}

static int silent_ota_status_proc_open(struct inode *inode, struct file *file)
{
	return single_open(file, silent_ota_status_proc_show, NULL);
}

static const struct file_operations silent_ota_status_proc_fops = {
	.open           = silent_ota_status_proc_open,
	.read           = seq_read,
	.llseek         = seq_lseek,
	.release        = single_release,
};

static int boot_complete_proc_show(struct seq_file *m, void *v)
{
	seq_printf(m, "%d\n", boot_complete);
	return 0;
}

static int boot_complete_proc_open(struct inode *inode, struct file *file)
{
	return single_open(file, boot_complete_proc_show, NULL);
}

/* This functioni will be called init.maxim.rc when boot complete */
ssize_t boot_complete_proc_write(struct file *file, const char __user *buf, size_t count, loff_t *ppos)
{
	char buffer[2];

	if (count != 1)
		return count;

	if (copy_from_user(buffer, buf, count))
		return -EFAULT;

	buffer[1] = '\0';
	pr_info("write boot_completed  %s %d \n", buffer, buffer[0]);
	sscanf(buffer, "%d", &boot_complete);
	if (boot_complete == 1)
		mstar_clear_silent_ota_flag();

	return count;
}

static const struct file_operations boot_complete_fops = {
	.open = boot_complete_proc_open,
	.read = seq_read,
	.write = boot_complete_proc_write,
	.llseek = seq_lseek,
	.release = single_release,
	.owner = THIS_MODULE,
};

static int __init ota_backlight_init(void)
{
	int ret = 0;
	struct device_node *np;
	u32 prop;
/* Get backlight configuration from device tree */
	np = of_find_node_by_name(NULL, "backlight_gpio");
	if (np == NULL) {
		pr_err("silentota: backlight_gpio is not defined in dts \n");
		goto err;
	}

	if (!of_property_read_u32(np, "bl-gpio", &prop)) {
		backlight_gpio = prop;
		pr_info("silentota: backlight-gpio is %d \n", backlight_gpio);
	} else {
		pr_err("silentota: backlight-gpio is not defined \n");
		goto err;
	}

	if (!of_property_read_u32(np, "backlight-inverse", &prop)) {
		backlight_inverse = prop;
		pr_info("silentota: backlight-inverse is %d \n", backlight_inverse);
	} else {
		pr_err("silentota: backlight-inverse is not defined \n");
		goto err;
	}

	MDrv_GPIO_Init();

/* detect current backlight status */
	backlight_status = get_backlight();
	pr_info("Inital backlight_status is %d \n", backlight_status);
	proc_create("boot_completed", 0664, NULL, &boot_complete_fops);
	proc_create("backlight_status", 0664, NULL, &backlight_status_proc_fops);
	proc_create("silent_ota_status", 0444, NULL, &silent_ota_status_proc_fops);
	return 0;
err:
	pr_err("silentota:Init failed, trigger a bug on \n");
	BUG();
	return ret;
}

static void __exit ota_backlight_exit(void)
{
	return;
}

static int __init IsOledTv(char *str)
{
	if (str != NULL) {
		sscanf(str, "%u", &is_OLED_tv);
		printk("Is OLED PAENL = %d \n", is_OLED_tv);
	}
	return 0;
}

static int __init IsUrsaMute(char *str)
{
	if (str != NULL) {
		sscanf(str, "%u", &ursa_mute_status);
		printk("Is Ursa Mute = %d \n", ursa_mute_status);
	}
	return 0;
}

early_initcall(ota_backlight_init);
module_exit(ota_backlight_exit);
early_param("OLED_TV", IsOledTv);
early_param("URSA_MUTE", IsUrsaMute);

MODULE_AUTHOR("Amazon.");
MODULE_DESCRIPTION("OTA backlight driver");
MODULE_LICENSE("GPL");
MODULE_ALIAS("ota-backlight");
