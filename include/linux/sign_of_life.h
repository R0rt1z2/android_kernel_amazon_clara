/*
 * sign_of_life.h
 *
 * Device sign of life header file
 *
 * Copyright (C) Amazon Technologies Inc. All rights reserved.
 * Yang Liu (yangliu@lab126.com)
 * TODO: Add additional contributor's names.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef __SIGN_OF_LIFE_H
#define __SIGN_OF_LIFE_H

#define PM_SPARE_COLD_BOOT_POWER_SUPPLY          0
#define PM_SPARE_WARM_BOOT_HW_WDOG               (1U << 1)
#define PM_SPARE_WARM_BOOT_KERNEL_WDOG	         (1U << 2)
#define PM_SPARE_WARM_BOOT_SW                    (1U << 3)
#define PM_SPARE_WARM_BOOT_KERNEL_PANIC          (1U << 4)
#define PM_SPARE_BOOT_LONG_PWR_KEY_PRESS     	 (1U << 5)
#define PM_SPARE_THERMAL_BOOT_SOC            	 (1U << 6)

/* thermal shutdown/reboot reason */
#define PM_SPARE_THERMAL_SHUTDOWN_SOC            (1U << 6)
#define PM_SPARE_SHUTDOWN_SW                     (1U << 7)

#define PM_SPARE_SECIAL_MODE_SILENT_OTA		 (1U << 8)
#define PM_SPARE_SCREEN_STATE			 (1U << 10)
/* special mode */
#define PM_SPARE_SPECIAL_MODE_OTA                (1U << 11)
#define PM_SPARE_SPECIAL_MODE_FACTORY_RESET      (1U << 12)
/* Device Boot Reason */
typedef enum {
	WARMBOOT_BY_KERNEL_PANIC     = 0x100,
	WARMBOOT_BY_KERNEL_WATCHDOG  = 0x101,
	WARMBOOT_BY_HW_WATCHDOG      = 0x102,
	WARMBOOT_BY_SW               = 0x103,
	COLDBOOT_BY_USB              = 0x104,
	COLDBOOT_BY_POWER_KEY        = 0x105,
	COLDBOOT_BY_POWER_SUPPLY     = 0x106,

	WARMBOOT_BY_LONG_PWR_KEY     = 0x107,
	WARMBOOT_BY_SOC_OVER_TEMP    = 0x108,
} life_cycle_boot_reason;

/* Device Shutdown Reason */
typedef enum {
	SHUTDOWN_BY_LONG_PWR_KEY_PRESS = 0x201,
	SHUTDOWN_BY_SW                 = 0x202,
	SHUTDOWN_BY_PWR_KEY            = 0x203,
	SHUTDOWN_BY_SUDDEN_POWER_LOSS  = 0x204,
	SHUTDOWN_BY_UNKNOWN_REASONS    = 0x205,
} life_cycle_shutdown_reason;

/* THERMAL SHUTDOWN REASON */
typedef enum {
	THERMAL_SHUTDOWN_REASON_BATTERY = 0x301,
	THERMAL_SHUTDOWN_REASON_PMIC    = 0x302,
	THERMAL_SHUTDOWN_REASON_SOC     = 0x303,
	THERMAL_SHUTDOWN_REASON_MODEM   = 0x304,
	THERMAL_SHUTDOWN_REASON_WIFI    = 0x305,
	THERMAL_SHUTDOWN_REASON_PCB     = 0x306,
} life_cycle_thermal_shutdown_reason;

/* LIFE CYCLE Special Mode */
typedef enum {
	LIFE_CYCLE_SMODE_NONE                     = 0x400,
	LIFE_CYCLE_SMODE_LOW_BATTERY              = 0x401,
	LIFE_CYCLE_SMODE_WARM_BOOT_USB_CONNECTED  = 0x402,
	LIFE_CYCLE_SMODE_OTA                      = 0x403,
	LIFE_CYCLE_SMODE_FACTORY_RESET            = 0x404,
} life_cycle_special_mode;

/* sign of life operations */
typedef struct sign_of_life_ops {
	int (*read_boot_reason)(life_cycle_boot_reason *boot_reason);
	int (*write_boot_reason)(life_cycle_boot_reason boot_reason);
	int (*read_shutdown_reason)(life_cycle_shutdown_reason *shutdown_reason);
	int (*write_shutdown_reason)(life_cycle_shutdown_reason shutdown_reason);
	int (*read_thermal_shutdown_reason)(life_cycle_thermal_shutdown_reason *thermal_shutdown_reason);
	int (*write_thermal_shutdown_reason)(life_cycle_thermal_shutdown_reason thermal_shutdown_reason);
	int (*read_special_mode)(life_cycle_special_mode *special_mode);
	int (*write_special_mode)(life_cycle_special_mode special_mode);
	int (*lcr_reset)(void);
} sign_of_life_ops;


/*
 * life_cycle_set_boot_reason
 * Description: this function will set the boot reason which causing device booting
 */
int life_cycle_set_boot_reason(life_cycle_boot_reason boot_reason);

/*
 * life_cycle_set_shutdown_reason
 * Description: this function will set the Shutdown reason which causing device shutdown
 */
int life_cycle_set_shutdown_reason(life_cycle_shutdown_reason shutdown_reason);

/*
 * life_cycle_set_thermal_shutdown_reason
 * Description: this function will set the Thermal Shutdown reason which causing device shutdown
 */
int life_cycle_set_thermal_shutdown_reason(life_cycle_thermal_shutdown_reason thermal_shutdown_reason);

/*
 * life_cycle_set_special_mode
 * Description: this function will set the special mode which causing device life cycle change
 */
int life_cycle_set_special_mode(life_cycle_special_mode life_cycle_special_mode);

void mstar_set_screen_flag(void);

void mstar_clear_screen_flag(void);

void mstar_clear_silent_ota_flag(void);

int mstar_get_silent_ota_flag(void);

#endif
