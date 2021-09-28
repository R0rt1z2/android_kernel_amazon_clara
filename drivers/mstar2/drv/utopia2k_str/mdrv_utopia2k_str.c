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
/// @file   mdrv_utopia2k_str.c
/// @brief  Utopia2K STR interface
/// @author MStar Semiconductor Inc.
///
///////////////////////////////////////////////////////////////////////////////////////////////////
#include <linux/version.h>
#include <linux/timer.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/module.h>
#include <generated/autoconf.h>
#include <linux/async.h>
#include <linux/slab.h>
#include <linux/sched.h>
#include <linux/input.h>
#include <linux/fs.h>
#include <linux/proc_fs.h>
#include <linux/uaccess.h>
#if LINUX_VERSION_CODE >= KERNEL_VERSION(4, 9, 44)
#include <trace/events/power.h>
#endif
#if LINUX_VERSION_CODE >= KERNEL_VERSION(4,14,53)
#include <linux/sched/debug.h>
#endif
#include <power.h>

#include "mdrv_utopia2k_str_io.h"
#include <linux/debugfs.h>

#define UTOPIA2K_STR_SELF_TEST 0
#define WAKUP_ANDROID_FROM_KERNEL 0

#define UTOPIA2K_STR_NAME "Mstar-utopia2k-str"
#define POWERON_REASON_NAME "Mstar-poweron-reason"


#define WKUP_SRC_NONE    0x00
#define WKUP_SRC_IR    0x01
#define WKUP_SRC_DVI    0x02
#define WKUP_SRC_DVI2    0x03
#define WKUP_SRC_CEC    0x04
#define WKUP_SRC_SAR    0x05
#define WKUP_SRC_ESYNC    0x06
#define WKUP_SRC_SYNC    0x07
#define WKUP_SRC_RTC    0x08
#define WKUP_SRC_RTC2    0x09
#define WKUP_SRC_AVLINK    0x0a
#define WKUP_SRC_UART    0x0b
#define WKUP_SRC_GPIO    0x0c
#define WKUP_SRC_MHL    0x0d
#define WKUP_SRC_WOL    0x0e
#define WKUP_SRC_MAX    0x0f

// Define unused wakeup source as bluetooth and wifi, but you need
// cooperation with PM51 owner, here's some examples:
#define WKUP_SRC_BT    WKUP_SRC_DVI
#define WKUP_SRC_WIFI    WKUP_SRC_DVI2

extern ptrdiff_t mstar_pm_base;
#define PM_REG_BASE             (0x0700 * 2)
#define REG_PM_DUMMY_WAKEUP_SOURCE             ((PM_REG_BASE + 0x39 * 2))
#define WAKEUP_SOURCE *((volatile unsigned short*)(mstar_pm_base + ((REG_PM_DUMMY_WAKEUP_SOURCE)<<1)))

#define MCU_REG_BASE                            (0x1000UL)
#define REG_MCU_DUMMY_POWERON_KEY             ((MCU_REG_BASE + (0x53 * 2)))
#define POWERON_KEY  \
	(*((unsigned char *)(mstar_pm_base + ((REG_MCU_DUMMY_POWERON_KEY)<<1)+1)))

// From PM51
static unsigned short pm_wakeup_source;
static unsigned short pm_wakeup_key;
static char power_on_src_name[32] = {};
static DEFINE_MUTEX(event_lock);

static void get_wakeup_reason(void)
{
	pm_wakeup_source = WAKEUP_SOURCE;
	pm_wakeup_key = POWERON_KEY;
	strcpy(power_on_src_name, "unknonw");
	switch (pm_wakeup_source & 0x0F) {
	case WKUP_SRC_IR:
		switch (pm_wakeup_key) {
		case 0x0C:
			pr_info("System is woken up by Power button \n");
		case 0x29:
			pr_info("System is waken up by Home button \n");
		case 0x2B:
			pr_info("System is woken up by Enter button \n");
		case 0x3F:
			pr_info("System is woken up by Voice button \n");

			sprintf(power_on_src_name, "%s", "power");
			break;
		case 0x39:
			pr_info("System is woken up by Netflix key \n");
			sprintf(power_on_src_name, "%s", "app_1");
			break;
		case 0x05:
			pr_info("System is woken up by Amazon Video  key \n");
			sprintf(power_on_src_name, "%s", "app_2");
			break;
		case 0x25:
			pr_info("System is woken up by CUSTOM_3  key \n");
			sprintf(power_on_src_name, "%s", "app_3");
			break;
		case 0x15:
			pr_info("System is woken up by CUSTOM_4 key \n");
			sprintf(power_on_src_name, "%s", "app_4");
			break;
		}
		break;
	case WKUP_SRC_SAR:
		pr_info("System is woken up by Keypad on TV \n");
		sprintf(power_on_src_name, "%s", "power");
		break;
	default:
		pr_info("Error System is woken up by unknown reason \n");
		sprintf(power_on_src_name, "%s", "unknown_source");
		break;
	}
	return;
}

unsigned short get_wakeup_source(void)
{
	pm_wakeup_source = WAKEUP_SOURCE;
	return pm_wakeup_source;
}
EXPORT_SYMBOL(get_wakeup_source);

#define utopia2k_str_dbg(fmt, ...)              \
do {                                    \
    if (utopia2k_str_debug_enabled)  \
        pr_info(fmt, ##__VA_ARGS__);                \
} while (0)

#if LINUX_VERSION_CODE >= KERNEL_VERSION(4, 9, 44)
#define TRACE_POINT_UTPA_START(name, ck, event) \
	trace_utpa_pm_callback_start(name, ck, event);
#define TRACE_POINT_UTPA_END(name, ck, err)	\
	trace_utpa_pm_callback_end(name, ck, err);
#else
#define TRACE_POINT_UTPA_START(name, ck, pm)        do {} while(0)
#define TRACE_POINT_UTPA_END(name, ck, err)   do {} while(0)
#endif

#ifdef CONFIG_MP_MSTAR_STR_OF_ORDER
enum stage_nr {
    STR_STAGE_NONE,
    STR_STAGE_1,
    STR_STAGE_2,
};
static enum stage_nr current_stage;
#endif

static ASYNC_DOMAIN_EXCLUSIVE(condition_domain);
static spinlock_t lock;
static spinlock_t data_lock;
static unsigned int current_mode;
static pm_message_t pm_transition;

static unsigned int rtc_str_wake;

struct utopia2k_str_module {
    struct list_head list;
    struct task_struct *p;
    struct timer_list timer;
    FUtopiaSTR fpSTR;
    int selftest;
    int mode;
#ifdef CONFIG_MP_MSTAR_STR_OF_ORDER
    enum stage_nr stage;
#endif
    void *data;
};

/* data from one module to another */
struct utopia2k_str_data {
    struct list_head list;
    void *key;
    void *value;
};

struct dts_post_condition {
    struct list_head list;
    char name[];
};

struct list_head utopia2k_str_head;
struct list_head utopia2k_str_data_head;
struct list_head dts_post_condition_head;

#if WAKUP_ANDROID_FROM_KERNEL
static struct input_dev *input = NULL;
static struct wakeup_source *ws = NULL;
#endif

#ifdef CONFIG_OF
static const char *power_mode[] = {
    "",
    "suspend",
    "resume"
};
#endif

bool utopia2k_str_debug_enabled = 0;
bool utopia2k_str_selftest = 0;

static int __init utopia2k_str_debug_enable(char *str)
{
    utopia2k_str_debug_enabled = 1;
    return 1;
}
__setup("utopia2k_str_debug", utopia2k_str_debug_enable);

static void utopia2k_str_module_timeout(unsigned long data)
{
    struct utopia2k_str_module *um = (struct utopia2k_str_module *)data;
    show_stack(um->p, NULL);
}

static ktime_t utopia2k_str_time_debug_start(struct utopia2k_str_module *module)
{
    ktime_t calltime = ktime_set(0, 0);

    utopia2k_str_dbg("[module %i] calling [%pf].\n", task_pid_nr(current), module->fpSTR);
    calltime = ktime_get();

    return calltime;
}

#if WAKUP_ANDROID_FROM_KERNEL
static void utopia2k_str_report_wakeup_event(void)
{
    if (NULL != input) {
        mutex_lock(&event_lock);
        input_event(input, EV_KEY, KEY_POWER, 1);
        input_sync(input);
        input_event(input, EV_KEY, KEY_POWER, 0);
        input_sync(input);
        mutex_unlock(&event_lock);
    }
}

static int utopia2k_str_should_report_wakeup_event(void)
{
    int ret = 0;
    pm_wakeup_source = WAKEUP_SOURCE;

    switch(pm_wakeup_source) {
        case WKUP_SRC_IR:
        case WKUP_SRC_SAR:
        case WKUP_SRC_CEC:
        case WKUP_SRC_WOL:
            ret = 1;
            break;
        case WKUP_SRC_GPIO:
        case WKUP_SRC_BT:
        case WKUP_SRC_WIFI:
            //need customer policy
            ret = 1;
            break;
        case WKUP_SRC_RTC:
		ret = rtc_str_wake ? 1 : 0;
            break;
        default:
            break;
    }
    printk("Wakeup source %d %s report wakeup event.\n", pm_wakeup_source, ret ? "will":"won't");
    return ret;
}
#endif

static void utopia2k_str_time_debug_report(ktime_t calltime, struct utopia2k_str_module *module)
{
    ktime_t delta, rettime;

    rettime = ktime_get();
    delta = ktime_sub(rettime, calltime);
    utopia2k_str_dbg("[module %i] call [%pf] return after [%Ld] usecs.\n",
            task_pid_nr(current), module->fpSTR,
            (unsigned long long)ktime_to_ns(delta) >> 10);
}

#ifdef CONFIG_MP_MSTAR_STR_OF_ORDER
static enum stage_nr check_stage(const char *name)
{
    int i, rc;
    struct device_node *nproot, *np;
    const char *strings[32];

    nproot = of_find_node_by_name(NULL, UTOPIA2K_STR_NAME);
    if (!nproot) {
        pr_err("Utopia2k-STR: cannot find [%s] node in dts\n", UTOPIA2K_STR_NAME);
        return STR_STAGE_1;
    }

    np = of_find_node_by_name(nproot, "str-stage2");
    if (!np) {
        pr_err("Utopia2k-STR: cannot find [str-stage2] node in dts\n");
        return STR_STAGE_1;
    }

    rc = of_property_read_string_array(np, "functions", strings, ARRAY_SIZE(strings));

    /* This is a ugly way to register STR stage 2 */
    for (i = 0; i < rc; ++i) {
        /* find it! register to stage2 */
        if (!strncmp(name, strings[i], strlen(strings[i]))) {
            pr_info("Utopia2k-STR: register %s to STR stage 2\n", name);
            return STR_STAGE_2;
        }
    }

    return STR_STAGE_1;
}
#endif

int mdrv_utopia2k_str_setup_function_ptr(void* pModuleTmp, FUtopiaSTR fpSTR)
{
    struct utopia2k_str_module *um;
    char name[32];

    um = kmalloc(sizeof(struct utopia2k_str_module), GFP_KERNEL);
    if (!um)
        return -ENOMEM;

    INIT_LIST_HEAD(&um->list);
    list_add(&um->list, &utopia2k_str_head);
    um->fpSTR = fpSTR;
    um->data = pModuleTmp;
    um->selftest = 0;
    um->p = NULL;
#ifdef CONFIG_MP_MSTAR_STR_OF_ORDER
    snprintf(name, sizeof(name), "%pf", fpSTR);
    /* for now, just support 2 stage */
    um->stage = check_stage(name);
#endif
    return 0;
}
EXPORT_SYMBOL(mdrv_utopia2k_str_setup_function_ptr);

int mdrv_utopia2k_str_wait_condition(const char* name, MS_U32 mode, MS_U32 stage)
{
    int ret = 0;
#ifdef CONFIG_OF
    char node_path[128] = {0};
    struct device_node *stage_node;
    struct property *prop;

    int i = 0;
    size_t l = 0, total = 0;
    const char *p;

    struct dts_post_condition *pc;
    int cnt = 0;

    int prop_cnt;

    if (utopia2k_str_selftest) {
        printk("self test mode, don't wait!");
        return 0;
    }

    if (mode <= 0 || mode >= UTOPIA2K_STR_POWER_MAX) {
        printk("error!! no such mode [%d].\n", mode);
        return -1;
    }

    snprintf(node_path, sizeof(node_path), "/%s/%s/%s/stage%d", UTOPIA2K_STR_NAME, name, power_mode[mode], stage);

    utopia2k_str_dbg("module %i in wait %s start...\n", task_pid_nr(current), node_path);

    stage_node = of_find_node_by_path(node_path);
    if (!stage_node) {
        return -1;
    }

    prop = of_find_property(stage_node, "pre-condition", NULL);

    prop_cnt = of_property_count_strings(stage_node, "pre-condition");

    if (prop_cnt <= 0) {
        ret = -1;
        goto wait_out_np;
    }

    p = prop->value;

    while(true) {
        for (i = 0, p = prop->value, total = 0; total < prop->length; total += l, p += l, i++) {
            l = strlen(p) + 1;
            // for the following case:
            // pre-condition = "xc_suspend_stage0", "", "oo_suspend_stage0";
            if (1 == l) {
                cnt++;
                continue;
            }
            spin_lock(&lock);
            list_for_each_entry(pc, &dts_post_condition_head, list) {
                if (pc){
                    if (pc->name) {
                        if (!strncmp(pc->name, p, l)) {
                            cnt++;
                            break;
                        }
                    }
                }
            }
            spin_unlock(&lock);
        }
        if (cnt == prop_cnt) {
            break;
        }
        if (mode != current_mode) {
            printk("suspend or resume wait in another thread, and have not finished yet, don't wait any more.\n");
            printk("driver should handle this case.\n");
            ret = -1;
            break;
        }
        cnt = 0;
        schedule_timeout_interruptible(HZ / 1000);
    }

wait_out_np:

    of_node_put(stage_node);

    utopia2k_str_dbg("module %i in wait %s end.\n", task_pid_nr(current), node_path);
#endif
    return ret;
}
EXPORT_SYMBOL(mdrv_utopia2k_str_wait_condition);

int mdrv_utopia2k_str_send_condition(const char* name, MS_U32 mode, MS_U32 stage)
{
    int ret = 0;
#ifdef CONFIG_OF
    char node_path[128] = {0};
    struct device_node *stage_node;
    struct property *prop;

    int i = 0;
    size_t l = 0, total = 0;
    const char *p;

    struct dts_post_condition *pc;

    if (mode <= 0 || mode >= UTOPIA2K_STR_POWER_MAX) {
        printk("error!! no such mode [%d].\n", mode);
        return -1;
    }

    snprintf(node_path, sizeof(node_path), "/%s/%s/%s/stage%d", UTOPIA2K_STR_NAME, name, power_mode[mode], stage);

    utopia2k_str_dbg("module %i in send %s start...\n", task_pid_nr(current), node_path);

    stage_node = of_find_node_by_path(node_path);
    if (!stage_node) {
        return -1;
    }

    prop = of_find_property(stage_node, "post-condition", NULL);

    if (!prop)
        goto send_out_np;
    if (!prop->value)
        goto send_out_np;
    if (strnlen(prop->value, prop->length) >= prop->length)
        goto send_out_np;

    p = prop->value;

    for (i = 0; total < prop->length; total += l, p += l, i++) {
        l = strlen(p) + 1;
        // for the following case:
        // post-condition = "xc_suspend_stage0", "", "oo_suspend_stage0";
        if (1 == l) {
            continue;
        }
        pc = kmalloc(sizeof(struct dts_post_condition) + l, GFP_KERNEL);
        if (!pc) {
            ret = -ENOMEM;
            break;
        }
        strncpy(pc->name, p, l);
        INIT_LIST_HEAD(&pc->list);
        spin_lock(&lock);
        list_add(&pc->list, &dts_post_condition_head);
        spin_unlock(&lock);
    }

send_out_np:

    of_node_put(stage_node);

    utopia2k_str_dbg("module %i in send %s end.\n", task_pid_nr(current), node_path);
#endif
    return ret;
}
EXPORT_SYMBOL(mdrv_utopia2k_str_send_condition);

int mdrv_utopia2k_str_set_data(char *key, char *value)
{
    struct utopia2k_str_data *data, *m;
    bool found = false;

    if ((key == NULL) || strcmp(key, "") == 0) {
        return -EINVAL;
    }

    list_for_each_entry_safe(data, m, &utopia2k_str_data_head, list) {
        if (strcmp(data->key, key) == 0) {
            found = true;
            if ((value == NULL) || strcmp(value, "") == 0) {
                spin_lock(&data_lock);
                list_del(&data->list);
                kfree(data->key);
                kfree(data->value);
                spin_unlock(&data_lock);
                utopia2k_str_dbg("remove key=%s\n", key);
            } else {
                if (strcmp(data->value, value) == 0) {
                    utopia2k_str_dbg("key = %s, value = %s exists\n", key, value);
                } else {
                    spin_lock(&data_lock);
                    kfree(data->value);
                    data->value = kmalloc(strlen(value)+1, GFP_KERNEL);
                    if (!data->value) {
                        spin_unlock(&data_lock);
                        return -ENOMEM;
                    }
                    memcpy(data->value, value, strlen(value)+1);
                    spin_unlock(&data_lock);
                }
            }
        }
    }

    if (found == false) {
        if ((value == NULL) || strcmp(value, "") == 0) {
            return -EINVAL;
        }

        data = kmalloc(sizeof(struct utopia2k_str_data), GFP_KERNEL);
        if (!data) {
            return -ENOMEM;
        }
        data->key = kmalloc(strlen(key)+1, GFP_KERNEL);
        if (!data->key) {
            kfree(data);
            return -ENOMEM;
        }
        data->value = kmalloc(strlen(value)+1, GFP_KERNEL);
        if (!data->value) {
            kfree(data->key);
            kfree(data);
            return -ENOMEM;
        }
        memcpy(data->key, key, strlen(key)+1);
        memcpy(data->value, value, strlen(value)+1);
        INIT_LIST_HEAD(&data->list);
        spin_lock(&data_lock);
        list_add(&data->list, &utopia2k_str_data_head);
        spin_unlock(&data_lock);
    }

    return 0;
}
EXPORT_SYMBOL(mdrv_utopia2k_str_set_data);

int mdrv_utopia2k_str_get_data(char *key, char *value)
{
    struct utopia2k_str_data *data;

    if ((key == NULL) || strcmp(key, "") == 0) {
        return -EINVAL;
    }

    spin_lock(&data_lock);
    list_for_each_entry(data, &utopia2k_str_data_head, list) {
        if (strcmp(data->key, key) == 0) {
            memcpy(value, data->value, strlen(data->value)+1);
            break;
        }
    }
    spin_unlock(&data_lock);
    return 0;
}
EXPORT_SYMBOL(mdrv_utopia2k_str_get_data);

#if UTOPIA2K_STR_SELF_TEST
int vdecStr(int u32PowerState, void* pModule)
{
    mdrv_utopia2k_str_wait_condition("vdec", u32PowerState, 0);
    schedule_timeout_interruptible(HZ);
    mdrv_utopia2k_str_send_condition("vdec", u32PowerState, 0);
    return 0;
}

int xcStr(int u32PowerState, void* pModule)
{
    mdrv_utopia2k_str_wait_condition("xc", u32PowerState, 0);
    schedule_timeout_interruptible(HZ/2);
    mdrv_utopia2k_str_send_condition("xc", u32PowerState, 0);
    return 0;
}

int ooStr(int u32PowerState, void* pModule)
{
    mdrv_utopia2k_str_wait_condition("oo", u32PowerState, 0);
    schedule_timeout_interruptible(HZ);
    mdrv_utopia2k_str_send_condition("oo", u32PowerState, 0);
    return 0;
}
#endif

static void mdrv_utopia2k_str_async_scheduler(void *data, async_cookie_t cookie)
{
    struct utopia2k_str_module *module = (struct utopia2k_str_module *)data;
    ktime_t starttime;
    char name[32];
    int err;

    if (utopia2k_str_selftest && module) {
        // only suspend/resume selected module in selftest mode
        if (!module->selftest)
            return;
    }

    if (module && module->fpSTR != NULL) {
        snprintf(name, sizeof(name), "%pf", module->fpSTR);
        /* 2s timeout */
        module->p = current;
        init_timer(&module->timer);
        module->timer.function = utopia2k_str_module_timeout;
        module->timer.data = (unsigned long)module;
        mod_timer(&module->timer, jiffies + 2 * HZ);
        /* debug module time start */
        starttime = utopia2k_str_time_debug_start(module);
        /* module entry */
        TRACE_POINT_UTPA_START(name, (int)cookie, pm_transition.event);
        err = module->fpSTR(module->mode, module->data);
        TRACE_POINT_UTPA_END(name, (int)cookie, err);
        /* debug module time end */
        utopia2k_str_time_debug_report(starttime, module);
        del_timer_sync(&module->timer);
    }
}

int mdrv_utopia2k_str_module_selftest(void* pModuleTmp,char* sModuleName)
{
    struct utopia2k_str_module *module;

    utopia2k_str_selftest = 1;
    list_for_each_entry(module, &utopia2k_str_head, list) {
        if (module->fpSTR != NULL && module->data == pModuleTmp) {
            // find the module & set its selftest
            module->selftest = 1;
        }
    }
    list_for_each_entry(module, &utopia2k_str_head, list) {
        if (module->fpSTR != NULL && module->data==pModuleTmp) {
            module->mode = UTOPIA2K_STR_POWER_SUSPEND;
            async_schedule_domain(mdrv_utopia2k_str_async_scheduler, module, &condition_domain);
        }
    }
    async_synchronize_full_domain(&condition_domain);
    printk("%s suspend finished \n",sModuleName);

    list_for_each_entry(module, &utopia2k_str_head, list) {
        if (module->fpSTR != NULL && module->data==pModuleTmp) {
            module->mode = UTOPIA2K_STR_POWER_RESUME;
            async_schedule_domain(mdrv_utopia2k_str_async_scheduler, module, &condition_domain);
        }
    }
    async_synchronize_full_domain(&condition_domain);
    printk("%s resume finished \n",sModuleName);

    return 0;
}
EXPORT_SYMBOL(mdrv_utopia2k_str_module_selftest);

#ifdef CONFIG_MP_MSTAR_STR_OF_ORDER
static int mstar_utopia2k_str_drv_suspend(struct platform_device *dev, pm_message_t state);
static int mstar_utopia2k_str_drv_resume(struct platform_device *dev);
static struct dev_pm_ops mstar_utopia2k_str_pm_ops;
static struct str_waitfor_dev waitfor;

static int of_utopia2k_str_suspend_stage1(struct device *dev)
{
    struct platform_device *pdev = to_platform_device(dev);

    if (WARN_ON(!pdev))
        return -ENODEV;

    if (waitfor.stage1_s_wait)
        wait_for_completion(&(waitfor.stage1_s_wait->power.completion));

    current_stage = STR_STAGE_1;

    return mstar_utopia2k_str_drv_suspend(pdev, dev->power.power_state);
}

static int of_utopia2k_str_suspend_stage2(struct device *dev)
{
    struct platform_device *pdev = to_platform_device(dev);

    if (WARN_ON(!pdev))
        return -ENODEV;

    if (waitfor.stage2_s_wait)
        wait_for_completion(&(waitfor.stage2_s_wait->power.completion));

    current_stage = STR_STAGE_2;

    return mstar_utopia2k_str_drv_suspend(pdev, dev->power.power_state);
}

static int of_utopia2k_str_resume_stage1(struct device *dev)
{
    struct platform_device *pdev = to_platform_device(dev);

    if (WARN_ON(!pdev))
        return -ENODEV;

    if (waitfor.stage1_r_wait)
        wait_for_completion(&(waitfor.stage1_r_wait->power.completion));

    current_stage = STR_STAGE_1;

    return mstar_utopia2k_str_drv_resume(pdev);
}

static int of_utopia2k_str_resume_stage2(struct device *dev)
{
    struct platform_device *pdev = to_platform_device(dev);

    if (WARN_ON(!pdev))
        return -ENODEV;

    if (waitfor.stage2_r_wait)
        wait_for_completion(&(waitfor.stage2_r_wait->power.completion));

    current_stage = STR_STAGE_2;

    return mstar_utopia2k_str_drv_resume(pdev);
}
#endif

static int mstar_utopia2k_str_drv_suspend(struct platform_device *dev, pm_message_t state)
{
    struct utopia2k_str_module *module;
    struct dts_post_condition *pc, *n;

    current_mode = UTOPIA2K_STR_POWER_SUSPEND;
    pm_transition = state;

    list_for_each_entry_safe(pc, n, &dts_post_condition_head, list) {
        list_del(&pc->list);
        kfree(pc);
    }

    list_for_each_entry(module, &utopia2k_str_head, list) {
#ifdef CONFIG_MP_MSTAR_STR_OF_ORDER
        /* Ignore the callback which on the different stage */
        if (module->stage != current_stage)
            continue;
#endif
        if (module->fpSTR != NULL) {
            module->mode = UTOPIA2K_STR_POWER_SUSPEND;
            async_schedule_domain(mdrv_utopia2k_str_async_scheduler, module, &condition_domain);
        }
    }
    async_synchronize_full_domain(&condition_domain);

    printk("-----------utopia2k str suspend-----------\n");

    return 0;
}
static int mstar_utopia2k_str_drv_resume(struct platform_device *dev)
{
    struct utopia2k_str_module *module;
    struct dts_post_condition *pc, *n;

    current_mode = UTOPIA2K_STR_POWER_RESUME;
    pm_transition.event = PM_EVENT_RESUME;

    list_for_each_entry_safe(pc, n, &dts_post_condition_head, list) {
        list_del(&pc->list);
        kfree(pc);
    }

    list_for_each_entry(module, &utopia2k_str_head, list) {
#ifdef CONFIG_MP_MSTAR_STR_OF_ORDER
        /* Ignore the callback which on the different stage */
        if (module->stage != current_stage)
            continue;
#endif
        if (module->fpSTR != NULL) {
            module->mode = UTOPIA2K_STR_POWER_RESUME;
            async_schedule_domain(mdrv_utopia2k_str_async_scheduler, module, &condition_domain);
        }
    }
    async_synchronize_full_domain(&condition_domain);

#if WAKUP_ANDROID_FROM_KERNEL
    /* Make sure resuming from power off, rather than break from suspend*/
    if ((STENT_RESUME_FROM_SUSPEND == get_state_value())
            && utopia2k_str_should_report_wakeup_event()) {
        /* Hold wake lock for a while until Android really waking up*/
        __pm_wakeup_event(ws, 5 * MSEC_PER_SEC);
        utopia2k_str_report_wakeup_event();
    }
#endif
    printk("-----------utopia2k str resume-----------\n");
    return 0;
}

static int mstar_utopia2k_str_drv_probe(struct platform_device *pdev)
{
    int error = 0;
#if WAKUP_ANDROID_FROM_KERNEL
    struct device *dev = &pdev->dev;

    ws = wakeup_source_register(UTOPIA2K_STR_NAME);

    input = input_allocate_device();
    if (!input) {
        error = -ENOMEM;
        dev_err(dev, "probe failed, error: %d\n", error);
        goto out;
    }
    input->name = "MStar Utopia2k STR Virtual Wakeup Event";
    input->phys = NULL;
    input->dev.parent = dev;

    input->id.bustype = BUS_VIRTUAL;
    input->id.vendor = 0x3697UL;
    input->id.product = 0x0001UL;
    input->id.version = 0x0001UL;

    input_set_capability(input, EV_KEY, KEY_POWER);
    error = input_register_device(input);
    if (error) {
        dev_err(dev, "input_register_device failed, error: %d\n", error);
        input_free_device(input);
        goto out;
    }
#endif
#ifdef CONFIG_MP_MSTAR_STR_OF_ORDER
    of_mstar_str(UTOPIA2K_STR_NAME, &pdev->dev,
		&mstar_utopia2k_str_pm_ops, &waitfor,
		&of_utopia2k_str_suspend_stage1,
		&of_utopia2k_str_resume_stage1,
		&of_utopia2k_str_suspend_stage2,
		&of_utopia2k_str_resume_stage2);
#endif
out:
	get_wakeup_reason();
    return error;
}

static int mstar_utopia2k_str_drv_remove(struct platform_device *pdev)
{
#if WAKUP_ANDROID_FROM_KERNEL
    wakeup_source_unregister(ws);
    input_free_device(input);
#endif
    return 0;
}

static ssize_t mstar_utopia2k_str_read(struct file *filp, char __user *buf, size_t count, loff_t *ppos)
{
    return 0;
}

static ssize_t mstar_utopia2k_str_write(struct file *filp, const char __user *buf, size_t count, loff_t *ppos)
{
    return 0;
}
static long mstar_utopia2k_str_ioctl(struct file *filp, u_int cmd, u_long arg)
{
    int ret = 0;

    switch(cmd) {
#if WAKUP_ANDROID_FROM_KERNEL
        case UTOPIA2K_STR_IOC_GET_WKUP_SRC:
            ret = put_user(pm_wakeup_source, (unsigned int *)arg);
            break;
        case UTOPIA2K_STR_IOC_SET_WKUP_SRC:
            ret = get_user(pm_wakeup_source, (unsigned int *)arg);
            break;
        case UTOPIA2K_STR_IOC_REPORT_EVENT:
            utopia2k_str_report_wakeup_event();
            break;
#endif
        default:
            ret = -EINVAL;
            break;
    }
    return ret;
}

static int mstar_utopia2k_str_release(struct inode * inode, struct file * filp)
{
    return 0;
}

static int mstar_utopia2k_str_open(struct inode * inode, struct file * filp)
{
    return 0;
}

static const struct file_operations mstar_utopia2k_str_fops = {
    .owner      = THIS_MODULE,
    .read       = mstar_utopia2k_str_read,
    .write      = mstar_utopia2k_str_write,
    .unlocked_ioctl = mstar_utopia2k_str_ioctl,
#if defined(CONFIG_COMPAT)
    .compat_ioctl = mstar_utopia2k_str_ioctl,
#endif
    .open       = mstar_utopia2k_str_open,
    .release    = mstar_utopia2k_str_release,
};

static int show_power_on_reason(struct seq_file *m, void *v)
{
	pr_info("system is powered on by %s \n", power_on_src_name);
	seq_printf(m, "%s", power_on_src_name);
}

static int mstar_poweron_reason_open(struct inode *inode, struct file *filp)
{
	return single_open(filp, show_power_on_reason, NULL);
}
static const struct file_operations mstar_poweron_reason_fops = {
	.read = seq_read,
	.open = mstar_poweron_reason_open,
	.release = single_release,
};

#if defined (CONFIG_OF)
static struct of_device_id mstar_utopia2k_str_of_device_ids[] = {
         {.compatible = UTOPIA2K_STR_NAME},
         {},
};
#endif

static struct platform_driver Mstar_utopia2k_str_driver = {
    .probe      = mstar_utopia2k_str_drv_probe,
    .remove     = mstar_utopia2k_str_drv_remove,
#ifndef CONFIG_MP_MSTAR_STR_OF_ORDER
    .suspend    = mstar_utopia2k_str_drv_suspend,
    .resume     = mstar_utopia2k_str_drv_resume,
#endif

    .driver = {
#if defined(CONFIG_OF)
        .of_match_table = mstar_utopia2k_str_of_device_ids,
#endif
#ifdef CONFIG_MP_MSTAR_STR_OF_ORDER
        .pm     = &mstar_utopia2k_str_pm_ops,
#endif
        .name   = UTOPIA2K_STR_NAME,
        .owner  = THIS_MODULE,
    }
};

static int __init mstar_utopia2k_str_init(void)
{
    platform_driver_register(&Mstar_utopia2k_str_driver);

    INIT_LIST_HEAD(&dts_post_condition_head);
    INIT_LIST_HEAD(&utopia2k_str_head);
    INIT_LIST_HEAD(&utopia2k_str_data_head);
    spin_lock_init(&lock);
    spin_lock_init(&data_lock);

#ifdef CONFIG_PROC_FS
    proc_create(UTOPIA2K_STR_NAME, S_IRUGO | S_IWUGO, NULL, &mstar_utopia2k_str_fops);
	proc_create(POWERON_REASON_NAME, S_IRUSR, NULL, &mstar_poweron_reason_fops);
#endif

#if UTOPIA2K_STR_SELF_TEST
    mdrv_utopia2k_str_setup_function_ptr(NULL, ooStr);
    mdrv_utopia2k_str_setup_function_ptr(NULL, xcStr);
    mdrv_utopia2k_str_setup_function_ptr(NULL, vdecStr);
#endif

    return 0;
}

static void __exit mstar_utopia2k_str_exit(void)
{
    struct utopia2k_str_module *module, *n;
    list_for_each_entry_safe(module, n, &utopia2k_str_head, list) {
        list_del(&module->list);
        kfree(module);
    }
    struct utopia2k_str_data *data, *m;
    list_for_each_entry_safe(data, m, &utopia2k_str_data_head, list) {
        list_del(&data->list);
        kfree(data->key);
        kfree(data->value);
        kfree(data);
    }
    platform_driver_unregister(&Mstar_utopia2k_str_driver);
}

static int rtc_str_wake_debug_show(struct seq_file *s, void *data)
{
	seq_printf(s, "%d\n", rtc_str_wake);
}

static int rtc_str_wake_debug_open(struct inode *inode, struct file *file)
{
	return single_open(file, rtc_str_wake_debug_show, NULL);
}

static int rtc_str_wake_debug_write(struct file *file, const char __user *userbuf, size_t count, loff_t *ppos)
{
	char buf[64];
	unsigned int set;
	count = min_t(size_t, count, (sizeof(buf)-1));
	if (copy_from_user(buf, userbuf, count))
		return -EFAULT;

	buf[count] = '\0';

	if (strict_strtol(buf, 0, &set) != 0)
		return -EINVAL;
	pr_info("RTC Alarm %s turn on the screen \n", set ? "will" : "won't");
	rtc_str_wake = set;
	return count;
}

static const struct file_operations rtc_str_wake_debug_fops = {
	.open           = rtc_str_wake_debug_open,
	.read           = seq_read,
	.write          = rtc_str_wake_debug_write,
	.llseek         = seq_lseek,
	.release        = single_release,
};

static int __init rtc_str_wake_debug_init(void)
{
	struct deentry *d;

	d = debugfs_create_file("rtc_str_wake", 0664, NULL, NULL,
		&rtc_str_wake_debug_fops);
	if (!d) {
		pr_err("Failed to create rtc_str_wake debug file \n");
		return -ENOMEM;
	}

	return 0;
}

late_initcall(rtc_str_wake_debug_init);
module_init(mstar_utopia2k_str_init);
module_exit(mstar_utopia2k_str_exit);

MODULE_DESCRIPTION("Mstar utopia2k STR Device Driver");
MODULE_LICENSE("GPL");

