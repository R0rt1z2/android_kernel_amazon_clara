/*
 * mstar_power.c
 *
 * ac power supply driver for Mstar T22
 *
 * Copyright (C) Amazon Technologies Inc. All rights reserved.
 * Ke Li	 (liken@amazon.com)
 * TODO: Add additional contributor's names.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */


#include <linux/module.h>
#include <linux/power_supply.h>
#include <linux/platform_device.h>

static enum power_supply_property ac_props[] = {
	POWER_SUPPLY_PROP_ONLINE,
};

static int ac_get_property(struct power_supply *psy,
			   enum power_supply_property psp, union power_supply_propval *val)
{
	int ret = 0;

	switch (psp) {
	case POWER_SUPPLY_PROP_ONLINE:
		val->intval = 1;
		break;
	default:
		ret = -EINVAL;
		break;
	}

	return ret;
}

static struct power_supply *ac_main;

/* ac_main initialization */
static struct power_supply_desc ac_main_desc = {
	.name = "ac",
	.type = POWER_SUPPLY_TYPE_MAINS,
	.properties = ac_props,
	.num_properties = ARRAY_SIZE(ac_props),
	.get_property = ac_get_property,
};

static int __init mstar_power_init(void)
{
	int ret = 0;

	pr_info("%s\n", __func__);

	/* Integrate with Android Battery Service */
	ac_main = power_supply_register(NULL, &ac_main_desc, NULL);
	if (IS_ERR(ac_main)) {
		pr_err("%s: power_supply_register AC Fail !\n", __func__);
		ret = PTR_ERR(ac_main);
	}

	return ret;
}
module_init(mstar_power_init);

static void __exit mstar_power_exit(void)
{
	pr_info("%s: unregister drivers\n", __func__);
	power_supply_unregister(ac_main);
}
module_exit(mstar_power_exit);

MODULE_AUTHOR("Ke Li");
MODULE_DESCRIPTION("MStar Power Device Driver");
MODULE_LICENSE("GPL");
