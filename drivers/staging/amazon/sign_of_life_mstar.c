/*
 * sign_of_life_mstar.c
 *
 * MStar platform implementation
 *
 * Copyright (C) Amazon Technologies Inc. All rights reserved.
 * Yang Liu (yangliu@lab126.com)
 * TODO: Add additional contributor's names.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/platform_device.h>
#include <linux/vmalloc.h>
#include <linux/proc_fs.h>
#include <linux/string.h>
#include <linux/sched.h>
#include <linux/uaccess.h>
#include <linux/io.h>
#include <linux/delay.h>
#include <linux/uaccess.h>
#include <linux/sign_of_life.h>

/* PM Base Spare Register Definition */

/*
 * PM_SPARE: bit0~16 : reserved bits for boot reasons
 */
extern ptrdiff_t mstar_pm_base;
#define REG_ADDR(addr)                           (*((volatile unsigned short int*)(mstar_pm_base + (addr << 1))))

#define PM_ADDR_OFFSET				 0x060a

/* read 2 byte */
#define REG_RR(_reg_)                            ({REG_ADDR(_reg_); })

/* write 2 byte */
#define REG_W2B(_reg_, _val_)    \
	do { REG_ADDR(_reg_) = (_val_); } while (0)


#define PM_SPARE_MASK                            0xffff

static struct mutex lock;

static int (mstar_read_boot_reason)(life_cycle_boot_reason *boot_reason)
{
	u16 pm_breason;

	mutex_lock(&lock);
	pm_breason = REG_RR(PM_ADDR_OFFSET);
	mutex_unlock(&lock);

	pr_err("%s: pm_breason is 0x%x\n", __func__, (u32)(pm_breason));

	if (pm_breason & PM_SPARE_WARM_BOOT_KERNEL_PANIC)
		*boot_reason = WARMBOOT_BY_KERNEL_PANIC;
	else if (pm_breason & PM_SPARE_WARM_BOOT_KERNEL_WDOG)
		*boot_reason = WARMBOOT_BY_KERNEL_WATCHDOG;
	else if (pm_breason & PM_SPARE_WARM_BOOT_SW)
		*boot_reason = WARMBOOT_BY_SW;
	else if (pm_breason == PM_SPARE_COLD_BOOT_POWER_SUPPLY)
		*boot_reason = COLDBOOT_BY_POWER_SUPPLY;
	else if (pm_breason & PM_SPARE_BOOT_LONG_PWR_KEY_PRESS)
		*boot_reason = WARMBOOT_BY_LONG_PWR_KEY;
	else if (pm_breason & PM_SPARE_THERMAL_BOOT_SOC)
		*boot_reason = WARMBOOT_BY_SOC_OVER_TEMP;
	else if (pm_breason & PM_SPARE_WARM_BOOT_HW_WDOG)
		*boot_reason = WARMBOOT_BY_HW_WATCHDOG;
	else if (pm_breason & PM_SPARE_SHUTDOWN_SW)
		*boot_reason = COLDBOOT_BY_POWER_KEY;
	else {
		pr_err("%s: No valid boot reason found: 0x%x\n", __func__, (u32)(pm_breason));
		return -1;
	}

	return 0;
}

static int (mstar_write_boot_reason)(life_cycle_boot_reason boot_reason)
{
	u16 pm_breason;

	mutex_lock(&lock);
	pm_breason = REG_RR(PM_ADDR_OFFSET);
	mutex_unlock(&lock);

	pr_info("%s: current pm_reason 0x%x boot_reason 0x%x\n", __func__, pm_breason, boot_reason);

	if (boot_reason == WARMBOOT_BY_KERNEL_PANIC)
		pm_breason = pm_breason | PM_SPARE_WARM_BOOT_KERNEL_PANIC;
	else if (boot_reason == WARMBOOT_BY_KERNEL_WATCHDOG)
		pm_breason = pm_breason | PM_SPARE_WARM_BOOT_KERNEL_WDOG;
	else if (boot_reason == WARMBOOT_BY_HW_WATCHDOG)
		pm_breason = pm_breason | PM_SPARE_WARM_BOOT_HW_WDOG;
	else if (boot_reason == WARMBOOT_BY_SW)
		pm_breason = pm_breason | PM_SPARE_WARM_BOOT_SW;
	else if (boot_reason == COLDBOOT_BY_POWER_SUPPLY)
		pm_breason = pm_breason | PM_SPARE_COLD_BOOT_POWER_SUPPLY;
	else if (boot_reason == WARMBOOT_BY_LONG_PWR_KEY)
		pm_breason = pm_breason | PM_SPARE_BOOT_LONG_PWR_KEY_PRESS;
	else if (boot_reason == WARMBOOT_BY_SOC_OVER_TEMP)
		pm_breason = pm_breason | PM_SPARE_THERMAL_BOOT_SOC;
	pr_err("%s: set boot reason 0x%x\n", __func__, (u32)(pm_breason));

	mutex_lock(&lock);
	REG_W2B(PM_ADDR_OFFSET, pm_breason);
	mutex_unlock(&lock);

	return 0;
}

static int (mstar_read_shutdown_reason)(life_cycle_shutdown_reason *shutdown_reason)
{
	u16 pm_shutdown_reason;

	mutex_lock(&lock);
	pm_shutdown_reason = REG_RR(PM_ADDR_OFFSET);
	mutex_unlock(&lock);
	if  (pm_shutdown_reason & PM_SPARE_SHUTDOWN_SW) {
		*shutdown_reason = SHUTDOWN_BY_SW;
		pr_info("%s, shutdown reason 0x%x\n", __func__, pm_shutdown_reason);
	} else {
		pr_info("%s, No valid shutdown reason found \n",  __func__);
		return -1;
	}
	return 0;
}

static int (mstar_write_shutdown_reason)(life_cycle_shutdown_reason shutdown_reason)
{
	u16 pm_shutdown_reason;
	mutex_lock(&lock);
	pm_shutdown_reason = REG_RR(PM_ADDR_OFFSET);
	mutex_unlock(&lock);
	if (shutdown_reason == SHUTDOWN_BY_SW) {
		pm_shutdown_reason = pm_shutdown_reason | PM_SPARE_SHUTDOWN_SW;
		pr_err("%s: mark shutdown_reason 0x%x\n", __func__, pm_shutdown_reason);
	}
	mutex_lock(&lock);
	REG_W2B(PM_ADDR_OFFSET, pm_shutdown_reason);
	mutex_unlock(&lock);
	return 0;
}

static int (mstar_read_thermal_shutdown_reason)(life_cycle_thermal_shutdown_reason *thermal_shutdown_reason)
{
	u16 pm_thermal_shutdown_reason;

	mutex_lock(&lock);
	pm_thermal_shutdown_reason = REG_RR(PM_ADDR_OFFSET);
	mutex_unlock(&lock);

	pr_info("%s: thermal shutdown reason 0x%x\n", __func__, pm_thermal_shutdown_reason);

	if (pm_thermal_shutdown_reason & PM_SPARE_THERMAL_SHUTDOWN_SOC)
		*thermal_shutdown_reason = THERMAL_SHUTDOWN_REASON_SOC;
	else {
		pr_err("%s: No valid thermal shutdown boot reason found 0x%x\n", __func__, pm_thermal_shutdown_reason);
		return -1;
	}

	return 0;
}

static int (mstar_write_thermal_shutdown_reason)(life_cycle_thermal_shutdown_reason thermal_shutdown_reason)
{
	u16 pm_thermal_shutdown_reason;

	mutex_lock(&lock);
	pm_thermal_shutdown_reason = REG_RR(PM_ADDR_OFFSET);
	mutex_unlock(&lock);

	pr_info("%s: shutdown_reason 0x%0x\n", __func__, pm_thermal_shutdown_reason);

	if (thermal_shutdown_reason == THERMAL_SHUTDOWN_REASON_SOC)
		pm_thermal_shutdown_reason = pm_thermal_shutdown_reason | PM_SPARE_THERMAL_SHUTDOWN_SOC;

	pr_err("%s: mark shutdown_reason 0x%x\n", __func__, pm_thermal_shutdown_reason);

	mutex_lock(&lock);
	REG_W2B(PM_ADDR_OFFSET, pm_thermal_shutdown_reason);
	mutex_unlock(&lock);

	return 0;
}

static int (mstar_read_special_mode)(life_cycle_special_mode *special_mode)
{
	u16 pm_smode;

	mutex_lock(&lock);
	pm_smode = REG_RR(PM_ADDR_OFFSET);
	mutex_unlock(&lock);

	pr_info("%s: special mode is 0x%x\n", __func__, pm_smode);

	if (pm_smode & PM_SPARE_SPECIAL_MODE_OTA)
		*special_mode = LIFE_CYCLE_SMODE_OTA;
	else if (pm_smode & PM_SPARE_SPECIAL_MODE_FACTORY_RESET)
		*special_mode = LIFE_CYCLE_SMODE_FACTORY_RESET;
	else {
		pr_err("%s: No valid special mode boot reason: 0x%x\n", __func__, pm_smode);
		return -1;
	}

	return 0;
}

static int (mstar_write_special_mode)(life_cycle_special_mode special_mode)
{
	u16 pm_smode;

	mutex_lock(&lock);
	pm_smode = REG_RR(PM_ADDR_OFFSET);
	mutex_unlock(&lock);

	pr_info("%s: special_mode 0x%x\n", __func__, pm_smode);

	if (special_mode == LIFE_CYCLE_SMODE_OTA)
		pm_smode = pm_smode | PM_SPARE_SPECIAL_MODE_OTA;
	else if (special_mode == LIFE_CYCLE_SMODE_FACTORY_RESET)
		pm_smode = pm_smode | PM_SPARE_SPECIAL_MODE_FACTORY_RESET;

	pr_err("%s: set special_mode 0x%x\n", __func__, pm_smode);

	mutex_lock(&lock);
	REG_W2B(PM_ADDR_OFFSET, pm_smode);
	mutex_unlock(&lock);

	return 0;
}

int mstar_lcr_reset(void)
{
	u16 data = 0;
	mutex_lock(&lock);
	data = REG_RR(PM_ADDR_OFFSET)&PM_SPARE_SECIAL_MODE_SILENT_OTA ?
			PM_SPARE_SECIAL_MODE_SILENT_OTA : 0;
	if (REG_RR(PM_ADDR_OFFSET)&PM_SPARE_SCREEN_STATE)
		data |= PM_SPARE_SCREEN_STATE;
	printk(" write 0x%x to PM spare register \n", data);
	/* clean up the PM spare register execpt silent ota and screen state flag */
	REG_W2B(PM_ADDR_OFFSET, data);
	mutex_unlock(&lock);

	return 0;
}

void mstar_set_screen_flag(void)
{
	mutex_lock(&lock);
	REG_W2B(PM_ADDR_OFFSET, REG_RR(PM_ADDR_OFFSET)|PM_SPARE_SCREEN_STATE);
	mutex_unlock(&lock);
	return;
}

void mstar_clear_screen_flag(void)
{
	mutex_lock(&lock);
	REG_W2B(PM_ADDR_OFFSET, REG_RR(PM_ADDR_OFFSET)&~PM_SPARE_SCREEN_STATE);
	mutex_unlock(&lock);
	return;
}

void mstar_clear_silent_ota_flag(void)
{
	mutex_lock(&lock);
	REG_W2B(PM_ADDR_OFFSET, REG_RR(PM_ADDR_OFFSET)&~PM_SPARE_SECIAL_MODE_SILENT_OTA);
	mutex_unlock(&lock);
	return;
}

int mstar_get_silent_ota_flag(void)
{
	int status;
	mutex_lock(&lock);
	status = (REG_RR(PM_ADDR_OFFSET)&PM_SPARE_SECIAL_MODE_SILENT_OTA) ? 1 : 0;
	mutex_unlock(&lock);
	return status;
}

int life_cycle_platform_init(sign_of_life_ops *sol_ops)
{
	pr_err("%s: Support MStar platform\n", __func__);
	sol_ops->read_boot_reason = mstar_read_boot_reason;
	sol_ops->write_boot_reason = mstar_write_boot_reason;
	sol_ops->read_shutdown_reason = mstar_read_shutdown_reason;
	sol_ops->write_shutdown_reason = mstar_write_shutdown_reason;
	sol_ops->read_thermal_shutdown_reason = mstar_read_thermal_shutdown_reason;
	sol_ops->write_thermal_shutdown_reason = mstar_write_thermal_shutdown_reason;
	sol_ops->read_special_mode = mstar_read_special_mode;
	sol_ops->write_special_mode = mstar_write_special_mode;
	sol_ops->lcr_reset = mstar_lcr_reset;

	mutex_init(&lock);

	return 0;
}

