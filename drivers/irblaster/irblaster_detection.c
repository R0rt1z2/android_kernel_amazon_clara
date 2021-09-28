/*
 *  drivers/irblaster/irblaster_detection.c
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
*/

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/workqueue.h>
#include <linux/kthread.h>
#include <linux/kdev_t.h>
#include <linux/of.h>
#include <linux/kernel.h>

#ifdef CONFIG_AMAZON_METRICS_LOG
#include <linux/metricslog.h>
#include <linux/vmalloc.h>
#ifndef BLASTER_METRICS_STR_LEN
#define BLASTER_METRICS_STR_LEN 128
#endif
#endif

#include "mdrv_gpio.h"
#include "mhal_gpio.h"
#include "mhal_gpio_reg.h"


#define DEVICE_NAME		"irblaster"

static struct task_struct *irblaster_poll_tsk;
static struct device *irblaster_dev;
struct class *irblaster_class;
static int headphone_state;

enum IRBLASTER_STATUS {
	IRBLASTER_UNPLUGGED		= 0,
	IRBLASTER_PLUG_IN		= 1,
};


enum SWITCH_DETECTION_CTRL {
	SWITCH_HP_PLUG_IN   = 0,
	SWITCH_HP_PLUG_OUT  = 1,
	SWITCH_IRB_PLUG_IN  = 2,
	SWITCH_IRB_PLUG_OUT = 3,
};

struct irblaster_detection_dev {
	struct device *dev;
	struct work_struct work;
	u32 irdetect_gpio;
	u32 irdetect_inverse;
	int ir_ctrl;
	int state;
	int shared;
	int switch_ctrl;
	bool is_suspend;
};


static struct platform_device irblaster_detection_device = {
	.name	= "irblaster-detection",
};

static ssize_t show_plug(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct irblaster_detection_dev *irdev = dev_get_drvdata(dev);
	sprintf(buf, "%d\n", irdev->state);
	return strlen(buf);
}
static DEVICE_ATTR(plug, S_IRUGO, show_plug, NULL);

static ssize_t enable_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct irblaster_detection_dev *irdev = dev_get_drvdata(dev);
	if (irdev->ir_ctrl == -1) {
		sprintf(buf, "%d\n", -1);
	} else {
		sprintf(buf, "%d\n", MDrv_GPIO_Pad_Read(irdev->ir_ctrl));
	}
	return strlen(buf);
}

static ssize_t enable_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t len)
{
	int ctrl;
	struct irblaster_detection_dev *irdev = dev_get_drvdata(dev);

	if (irdev->ir_ctrl == -1) {
		return -1;
	}
	kstrtoint(buf, 10, &ctrl);
	if (ctrl == 1) {
		MDrv_GPIO_Set_High(irdev->ir_ctrl);
	} else {
		MDrv_GPIO_Set_Low(irdev->ir_ctrl);
	}

	return -1;
}
static DEVICE_ATTR(enable, S_IRUGO|S_IWUSR|S_IWGRP, enable_show, enable_store);

int get_headphone_state(void)
{
	return headphone_state;
}
EXPORT_SYMBOL_GPL(get_headphone_state);

static void set_state(struct irblaster_detection_dev *dev, int state)
{
#ifdef CONFIG_AMAZON_METRICS_LOG
	char *blaster_metric_prefix = "blaster:def:monitor=1;CT;1";
	char mbuf[BLASTER_METRICS_STR_LEN + 1];
#endif
	char event_string[10];
	char *envp[] = { event_string, NULL };
	if ((dev->shared == 0) || (dev->switch_ctrl >= SWITCH_IRB_PLUG_IN)) {

		snprintf(event_string, sizeof(event_string), "plug=%d", state);
		pr_info("irblaster: generate IR detect uevent %s\n", envp[0]);
		kobject_uevent_env(&dev->dev->kobj, KOBJ_CHANGE, envp);
#ifdef CONFIG_AMAZON_METRICS_LOG
		snprintf(mbuf, BLASTER_METRICS_STR_LEN,
			"%s,irjack_dtected_%d;CT;",
			blaster_metric_prefix, state);
		log_to_metrics(ANDROID_LOG_INFO, "BlasterEvent", mbuf);
		log_counter_to_vitals(ANDROID_LOG_INFO, "Kernel", "Kernel",
			"BLASTER", "plug", (u32)state,
			"count", NULL, VITALS_NORMAL);

#endif
	} else {
		pr_info("Pass Headphone state to HP driver \n");
		headphone_state = dev->state;
	}
}
static ssize_t switch_ctrl_show(struct device *dev, struct device_attribute *attr,
		char *buf)
{
	struct irblaster_detection_dev *irdev = dev_get_drvdata(dev);
	sprintf(buf, "%d\n", irdev->switch_ctrl);

	return strlen(buf);
}


static ssize_t switch_ctrl_store(struct device *dev, struct device_attribute *attr,
		char *buf)
{
	int ctrl;
	char *prop_buf;
	char name_buf[120];
	char state_buf[120];
	char *envp[3];
	int length;
	int env_offset = 0;
	struct irblaster_detection_dev *irdev = dev_get_drvdata(dev);

	kstrtoint(buf, 10, &ctrl);
	if (ctrl < SWITCH_HP_PLUG_IN || ctrl > SWITCH_IRB_PLUG_OUT) {
		printk(KERN_ERR "switch: invalid enable value %s\n", buf);
		return -1;
	}
	irdev->switch_ctrl = ctrl;

	if (ctrl == SWITCH_HP_PLUG_IN || ctrl == SWITCH_HP_PLUG_OUT) {
		pr_info("HP is plug %s \n", irdev->state ? "in" : "out");
		headphone_state = irdev->state;
	} else if (ctrl == SWITCH_IRB_PLUG_IN || ctrl == SWITCH_IRB_PLUG_OUT) {
		pr_info("IR blaster is plug %s \n", irdev->state ? "in" : "out");
		set_state(irdev, irdev->state);
	} else {
		pr_err("Error, wrong switch ctontrol action \n");
		return -1;
	}

	return 1;
}
static DEVICE_ATTR(switch_ctrl, S_IRUGO|S_IWUSR, switch_ctrl_show, switch_ctrl_store);

static void switch_send_notification(struct irblaster_detection_dev *irdev, int state)
{
	char name_buf[120];
	char state_buf[120];
	char *prop_buf;
	char *envp[3];
	int env_offset = 0;
	int length;

	prop_buf = (char *)get_zeroed_page(GFP_KERNEL);
	if (prop_buf) {
		length = sprintf(prop_buf, "%s\n", "switch_detect");
		if (length > 0) {
			if (prop_buf[length - 1] == '\n')
				prop_buf[length - 1] = 0;
			snprintf(name_buf, sizeof(name_buf),
					"SWITCH_NAME=%s", prop_buf);
			envp[env_offset++] = name_buf;
		}
		length = sprintf(prop_buf, "%d\n", irdev->state);
		if (length > 0) {
			if (prop_buf[length - 1] == '\n')
				prop_buf[length - 1] = 0;
			snprintf(state_buf, sizeof(state_buf),
				"SWITCH_STATE=%s", prop_buf);
			envp[env_offset++] = state_buf;
		}
		envp[env_offset] = NULL;
		pr_info("send uevent %s, %s\n", envp[0], envp[1]);
		kobject_uevent_env(&irdev->dev->kobj, KOBJ_CHANGE, envp);
		free_page((unsigned long)prop_buf);
	} else {
		printk(KERN_ERR "out of memory in switch_set_state\n");
		kobject_uevent(&irdev->dev->kobj, KOBJ_CHANGE);
	}

}

static int irblaster_poll(void *arg)
{
	struct irblaster_detection_dev *dev =
		(struct irblaster_detection_dev *)arg;
	int state = 0;

	while (1) {
		schedule_timeout_interruptible(msecs_to_jiffies(100));
		if (!dev->is_suspend) {
			if (dev->irdetect_inverse == 1)
				state = !(MDrv_GPIO_Pad_Read(dev->irdetect_gpio));
			else
				state = (MDrv_GPIO_Pad_Read(dev->irdetect_gpio));
			if (dev->state != state) {
				if (dev->shared == 0) {
					dev->state = state;
					set_state(dev, state);
				} else {
					pr_info("Send notification to UI, plug %s \n", state ? "in" : "out");
					dev->state = state;
					switch_send_notification(dev, state);
					schedule_timeout_interruptible(msecs_to_jiffies(100));
				}
			}
		}
	}
	return 0;
}

extern char *idme_get_product_name(void);
static int irblaster_detection_probe(struct platform_device *pdev)
{
	struct irblaster_detection_dev *dev;
	struct device_node *np;
	u32 prop;
	int ret = 0;

	dev = kzalloc(sizeof(struct irblaster_detection_dev), GFP_KERNEL);
	if (!dev) {
		pr_err("irblaster: irblaster_detection_dev allocation fail");
		return -ENOMEM;
	}

	np = of_find_node_by_name(NULL, "irblaster_gpio");
	if (np == NULL) {
		pr_err("irblaster: irblaster_detection is not defined in dts\n");
		goto err_class_create;
	}

	if (!of_property_read_u32(np, "irblaster-gpio", &prop)) {
		dev->irdetect_gpio = prop;
		pr_info("irblaster: irblaster_detection gpio is %d \n", prop);
	} else {
		pr_err("irblaster: irblaster_detection gpio is not defined \n");
		goto err_class_create;
	}

	if (!of_property_read_u32(np, "irblaster-inverse", &prop)) {
		dev->irdetect_inverse = prop;
		pr_info("irblaster: irblaster_detection gpio inverse is %d \n", prop);
	} else {
		pr_err("irblaster: irblaster_detection gpio inverse is not defined \n");
		goto err_class_create;
	}

	if (!of_property_read_u32(np, "irblaster-ctrl", &prop)) {
		dev->ir_ctrl = prop;
		pr_info("irblaster: irblaster control gpio  is %d \n", prop);
	} else {
		pr_err("irblaster: irblaster control  gpio is not defined \n");
		dev->ir_ctrl = -1;
	}


	if (!of_property_read_u32(np, "irblaster-shared", &prop)) {
		dev->shared = prop;
		pr_info("irblaster: irblaster detection %s shared \n", prop ? "is" : " is not");
	} else {
		pr_err("irblaster: irblaster shared is not defined  \n");
		dev->shared = 0;
	}

	platform_set_drvdata(pdev, dev);
	MDrv_GPIO_Init();

	irblaster_poll_tsk = kthread_create(irblaster_poll, dev, "Irblaster poll Task");
	if (IS_ERR(irblaster_poll_tsk)) {
		pr_err("irblaster: create kthread for GPIO poll Task fail\n");
		ret = PTR_ERR(irblaster_poll_tsk);
		goto err_kthread_create;
	} else
		wake_up_process(irblaster_poll_tsk);

	irblaster_class = class_create(THIS_MODULE, DEVICE_NAME);
	if (IS_ERR(irblaster_class)) {
		pr_err("irblaster: create class fail\n");
		ret = PTR_ERR(irblaster_class);
		goto err_class_create;
	}
	irblaster_dev = device_create(irblaster_class, NULL,
			MKDEV(0, 0), dev, "irblaster%d", 1);
	dev->dev = irblaster_dev;

	ret = device_create_file(irblaster_dev, &dev_attr_plug);
	if (ret < 0)
		goto err_create_file;

	ret = device_create_file(irblaster_dev, &dev_attr_enable);
	if (ret < 0)
		goto err_create_file;

	ret = device_create_file(irblaster_dev, &dev_attr_switch_ctrl);
	if (ret < 0)
		goto err_create_file;

	if (dev->irdetect_inverse == 1)
		set_state(dev, !MDrv_GPIO_Pad_Read(dev->irdetect_gpio));
	else
		set_state(dev, MDrv_GPIO_Pad_Read(dev->irdetect_gpio));
	return 0;

err_create_file:
	device_destroy(irblaster_class, MKDEV(0, 0));
	class_destroy(irblaster_class);
err_class_create:
err_kthread_create:
	kfree(dev);
	return ret;
}

static int irblaster_detection_remove(struct platform_device *pdev)
{

	device_remove_file(irblaster_dev, &dev_attr_plug);
	device_remove_file(irblaster_dev, &dev_attr_enable);
	device_destroy(irblaster_class, MKDEV(0, 0));
	class_destroy(irblaster_class);
	return 0;
}

static int irblaster_detection_suspend(struct platform_device *pdev, pm_message_t state)
{
	struct irblaster_detection_dev *dev = platform_get_drvdata(pdev);
	if (dev)
		dev->is_suspend = true;
	return 0;
}

static int irblaster_detection_resume(struct platform_device *pdev)
{
	struct irblaster_detection_dev *dev = platform_get_drvdata(pdev);

	if (dev)
		dev->is_suspend = false;
	return 0;
}

static struct platform_driver irblaster_detection_driver = {
	.probe		= irblaster_detection_probe,
	.remove		= irblaster_detection_remove,
	.suspend	= irblaster_detection_suspend,
	.resume		= irblaster_detection_resume,
	.driver		= {
		.name	= "irblaster-detection",
		.owner	= THIS_MODULE,
	},
};

static int __init irblaster_detection_init(void)
{
	int ret;
	ret = platform_device_register(&irblaster_detection_device);
	if (ret < 0) {
		pr_err("failed to register irblaster device\n");
		return ret;
	}
	ret = platform_driver_register(&irblaster_detection_driver);
	if (ret < 0) {
		pr_err("failed to register irblaster driver\n");
		platform_device_unregister(&irblaster_detection_device);
	}
	return ret;
}

static void __exit irblaster_detection_exit(void)
{
	platform_driver_unregister(&irblaster_detection_driver);
	platform_device_unregister(&irblaster_detection_device);
}

module_init(irblaster_detection_init);
module_exit(irblaster_detection_exit);

MODULE_DESCRIPTION("IRblaster detection");
MODULE_LICENSE("GPL");

