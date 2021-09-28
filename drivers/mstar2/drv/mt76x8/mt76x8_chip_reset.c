#include <generated/autoconf.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/init.h>
#include <linux/device.h>
#include <linux/errno.h>
#include <linux/platform_device.h>
#include <linux/of.h>
#include <linux/of_device.h>

#include "../gpio/mdrv_gpio.h"

static int bt_reset_gpio_pin = -1;
static unsigned int board_type;
static unsigned int board_rev;
extern unsigned int idme_get_board_rev(void);
extern unsigned int idme_get_board_type(void);

#define GPIO_MAX_RESERVED_PIN_NUMBER 20

#define RST_WLAN_DEBUG(fmt, args...) \
	printk(KERN_ERR "[%s][%d] " fmt, __func__, __LINE__, ##args)

#define RST_ERROR_DEBUG(fmt, args...) \
	printk(KERN_ERR "[%s][%d] " fmt, __func__, __LINE__, ##args)

#define RST_INFO_DEBUG(fmt, args...) \
	printk(KERN_ERR "[%s][%d] " fmt, __func__, __LINE__, ##args)

static int btmtk_get_reset_gpio_pin(struct platform_device *pdev)
{
	/* Default as Sophia */
	int reset_pin = 13;

	board_type = idme_get_board_type();
	board_rev = idme_get_board_rev();

	RST_WLAN_DEBUG("Enter - board_type = 0x%04x board_rev = 0x%4x\n",
	board_type, board_rev);

	/* TODO - add logic to get reset_pin number based on device ID */

	RST_WLAN_DEBUG("Leave - board_type = 0x%04x board_rev = 0x%4x, reset_pin=%d\n",
	board_type, board_rev, reset_pin);

	return reset_pin;
}

/* Set the Chip Reset pin state (0 - low, 1 - high) */
void btmtk_set_reset_pin_state(struct device * dev, int state)
{
	struct platform_device *pdev = to_platform_device(dev->parent);

	if (bt_reset_gpio_pin < 0) {
		RST_WLAN_DEBUG("[%s][%d] return error %d\n", \
				__FUNCTION__, __LINE__, bt_reset_gpio_pin);

		return;
	} else {
		RST_WLAN_DEBUG("reset_pin=%d reset state = %d\n", bt_reset_gpio_pin, state);
	}

	if (state == 0)
		MDrv_GPIO_Set_Low(bt_reset_gpio_pin);
	else
		MDrv_GPIO_Set_High(bt_reset_gpio_pin);
}

#if defined(CONFIG_OF)
static struct of_device_id btmtk_chip_reset_of_device_ids[] = {
	{.compatible = "mstar,gpio-wifi-ctl"},
	{},
};
#endif

static int btmtk_chip_reset_drv_probe(struct platform_device *pdev)
{
	int gpio_pin = -1;

#if defined(CONFIG_OF)
	struct device_node *dn;
	int rsv = 0;
	const struct of_device_id *match;
	match = of_match_device(btmtk_chip_reset_of_device_ids, &pdev->dev);
	if (!match) {
		dev_err(&pdev->dev, "Failed to find gpio-wifi-ctl node\n");
		return -ENODEV;
	}

	/* obtain pads that are used by wifi */
	for_each_compatible_node(dn, NULL, "mstar,gpio-wifi-ctl") {
		if (rsv++ >= GPIO_MAX_RESERVED_PIN_NUMBER)
			break;

		if (0 != of_property_read_u32(dn, "wifi-ctl-gpio", (u32 *)&gpio_pin)) {
			RST_ERROR_DEBUG("dn %d failed to obtain btmtk chip reset pin = %d\n", rsv, gpio_pin);
			gpio_pin = -1;
			continue;
		}

		RST_INFO_DEBUG("dn %d obtained btmtk chip reset pin = %d\n", rsv, gpio_pin);
		break;
	}
#endif

	RST_INFO_DEBUG("%s: gpio_pin = %d\n", pdev->name, gpio_pin);

	/* If we failed to get it through OF/DTS, try the default board ID based method */
	if (gpio_pin == -1) {
		gpio_pin = btmtk_get_reset_gpio_pin(pdev);
		RST_INFO_DEBUG("%s: fallback gpio_pin = %d\n", pdev->name, gpio_pin);
	}

	bt_reset_gpio_pin = gpio_pin;

	return 0;
}

static struct platform_driver btmtk_chip_reset_driver = {
	.probe  = btmtk_chip_reset_drv_probe,
	.driver = {
#if defined(CONFIG_OF)
	.of_match_table = of_match_ptr(btmtk_chip_reset_of_device_ids),
#endif
	.name = "btmtk-chip-reset",
	.owner  = THIS_MODULE,
	}
};

static int __init btmtk_chip_reset_drv_init_module(void)
{
	return platform_driver_register(&btmtk_chip_reset_driver);
}

static void __exit btmtk_chip_reset_drv_exit_module(void)
{
	platform_driver_unregister(&btmtk_chip_reset_driver);
}

postcore_initcall(btmtk_chip_reset_drv_init_module);
module_exit(btmtk_chip_reset_drv_exit_module);

MODULE_AUTHOR("coryxie@amazon.com");
MODULE_DESCRIPTION("BTMTK Chip Reset driver");
MODULE_LICENSE("GPL");
