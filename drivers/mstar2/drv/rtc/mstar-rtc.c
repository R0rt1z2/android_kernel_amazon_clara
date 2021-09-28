#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/time.h>
#include <linux/platform_device.h>
#include <linux/rtc.h>
#include <linux/delay.h>
#include <linux/debugfs.h>

#include "chip_int.h"
#include "mdrv_types.h"
#include "mdrv_rtc.h"
#include "mdrv_pm.h"
#if defined(CONFIG_OF)
#include <linux/of.h>
#endif

struct mstar_rtc_plat_data {
    u32 index;
    u32 xtal;
    u32 freq;
};

#if !defined(CONFIG_OF)
static struct mstar_rtc_plat_data mstar_rtc_pdata = {
        .index = 0,
        .xtal = 12000000,  // external clock reference @ 12MHz
        .freq = 1,         // RTC update frequency in second
};

struct platform_device mstar_rtc_pdev = {
        .name = "mstar-rtc",
        .dev = {
            .platform_data = &mstar_rtc_pdata,
        }
};
MODULE_DEVICE_TABLE(platform, mstar_rtc_pdev);
#endif

struct mstar_rtc_dev {
	struct platform_device *pdev;
	struct rtc_device *rtc;
	PM_RtcParam mstar_rtc_param;
	spinlock_t mstar_rtc_lock;
	struct mutex mutex_lock;
};

#if defined (CONFIG_ANDROID)
static u8 rtc_int_is_enabled = 0;
#endif

static unsigned int rtc_str_time;
static int mstar_rtc_get_time(struct device *dev, struct rtc_time *tm)
{
    struct timeval time;
    PM_RtcParam pmRtcParam;

    pmRtcParam.u8PmRtcIndex = 0;
    time.tv_sec = (__kernel_time_t)MDrv_RTC_GetCount(&pmRtcParam);

    rtc_time_to_tm(time.tv_sec, tm);
    return rtc_valid_tm(tm);
}

static int mstar_rtc_set_time(struct device *dev, struct rtc_time *tm)
{
    unsigned long time;
    PM_RtcParam pmRtcParam;

    rtc_tm_to_time(tm, &time);
    pmRtcParam.u8PmRtcIndex = 0;
    pmRtcParam.u32RtcSetCounter = (u32)time;
    MDrv_RTC_SetCount(&pmRtcParam);
    return 0;
}

static irqreturn_t mstar_rtc_irq_handler(int irq, void *data)
{
	struct device *dev = data;
	struct mstar_rtc_dev *mstar_rtc = dev_get_drvdata(dev);
	unsigned long events = 0;

	events |= RTC_IRQF | RTC_AF;
	rtc_update_irq(mstar_rtc->rtc, 1, events);

	return IRQ_HANDLED;
}

static int mstar_rtc_alarm_irq_enable(struct device *dev, unsigned int enable)
{
	struct mstar_rtc_dev *mstar_rtc = dev_get_drvdata(dev);
	unsigned long irq_flags;

#if defined (CONFIG_ANDROID)
	mutex_lock(&mstar_rtc->mutex_lock);
	if (enable)
	{
		spin_lock_irqsave(&mstar_rtc->mstar_rtc_lock, irq_flags);
		if (!rtc_int_is_enabled)
		{
			//enable_irq(E_IRQ_PM_SLEEP);
			MDrv_RTC_Enable_Interrupt();
			rtc_int_is_enabled = 1;
		}
		spin_unlock_irqrestore(&mstar_rtc->mstar_rtc_lock, irq_flags);
	}
	else
	{
		if (rtc_int_is_enabled)
		{
			//disable_irq(E_IRQ_PM_SLEEP);
			MDrv_RTC_Disable_Interrupt();
			rtc_int_is_enabled = 0;
		}
	}
	mutex_unlock(&mstar_rtc->mutex_lock);
#endif

	return 0;
}

static int mstar_rtc_set_alarm(struct device *dev, struct rtc_wkalrm *alarm)
{
	PM_RtcParam pmRtcParam = {0};
	unsigned long sec;

	printk(KERN_ERR "mstar-rtc %s\n", __func__);

	if (alarm->enabled)
		rtc_tm_to_time(&alarm->time, &sec);
	else
		sec = 0;

	if (rtc_str_time > 0) {
		struct timeval tv;
		pmRtcParam.u8PmRtcIndex = 0;
		tv.tv_sec = (__kernel_time_t)MDrv_RTC_GetCount(&pmRtcParam);
		sec = rtc_str_time + tv.tv_sec;
		pr_info("will sleep %d second \n", rtc_str_time);
	}

	pmRtcParam.u8PmRtcIndex = 0;
	pmRtcParam.u32RtcSetMatchCounter = (u32)sec;
	MDrv_RTC_SetMatchCount((PM_RtcParam*)&pmRtcParam);

	if (sec) {
		mstar_rtc_alarm_irq_enable(dev, 1);
		dev_vdbg(dev, "alarm set as %lu. %d/%d/%d %d:%02u:%02u\n",
			sec,
			alarm->time.tm_mon+1,
			alarm->time.tm_mday,
			alarm->time.tm_year+1900,
			alarm->time.tm_hour,
			alarm->time.tm_min,
			alarm->time.tm_sec);
	} else {
		/* disable alarm */
		dev_vdbg(dev, "alarm disabled\n");
		mstar_rtc_alarm_irq_enable(dev, 0);
	}

	return 0;
}

static int mstar_rtc_read_alarm(struct device *dev, struct rtc_wkalrm *alarm)
{
	unsigned long sec;
	PM_RtcParam pmRtcParam;
	pmRtcParam.u8PmRtcIndex = 0;
	sec = MDrv_RTC_GetMatchCount(&pmRtcParam);

	if (sec == 0xFFFFFFFF) {
		alarm->enabled = 0;
	} else {
		alarm->enabled = 1;
		rtc_time_to_tm(sec, &alarm->time);
	}

    return 0;
}

static int mstar_rtc_proc(struct device *dev, struct seq_file *seq)
{
    printk(KERN_INFO "mstar-rtc %s: currently not supported\n", __func__);

    return 0;
}

static const struct rtc_class_ops mstar_rtc_ops = {
	.proc = mstar_rtc_proc,
	.read_time = mstar_rtc_get_time,
	.set_time = mstar_rtc_set_time,
	.read_alarm = mstar_rtc_read_alarm,
	.set_alarm = mstar_rtc_set_alarm,
	.alarm_irq_enable = mstar_rtc_alarm_irq_enable,
};

#if defined(CONFIG_OF)
static const struct of_device_id mstar_rtc_dt_match[] = {
	{ .compatible = "mstar-rtc", },
	{}
};
MODULE_DEVICE_TABLE(of, mstar_rtc_dt_match);
#endif


#if defined(CONFIG_OF)
static int mstar_rtc_parse_dt(struct device_node *dn, struct mstar_rtc_dev *mstar_rtc)
{
    u32 rtc_index = 0;
    u32 freq = 0;
    u32 xtal = 0;

    if ((of_property_read_u32(dn, "index", &rtc_index) != 0) ||
        (of_property_read_u32(dn, "xtal", &xtal) != 0)       ||
        (of_property_read_u32(dn, "freq", &freq) != 0)) {
        printk(KERN_ERR "Mstar-rtc parse DT error\n");
        return -1;
    }

    if (freq < 0)
        return -1;

    mstar_rtc->mstar_rtc_param.u8PmRtcIndex = rtc_index;
    mstar_rtc->mstar_rtc_param.u32RtcCtrlWord = xtal / freq;
#if 0
    printk("rtc index = %d\n", rtc_index);
    printk("rtc xtal = %d\n", xtal);
    printk("rtc update freq = %d\n", freq);
    printk("rtc cw = %d\n", mstar_rtc->mstar_rtc_param.u32RtcCtrlWord);
#endif
    return 0;
}
#endif

static int __init mstar_rtc_probe(struct platform_device *pdev)
{
    struct mstar_rtc_dev *mstar_rtc;
    PM_RtcParam pmRtcParam;
    int ret = 0;
#if defined(CONFIG_OF)
    struct device_node * dn;
#else
    struct mstar_rtc_plat_data *mstar_pdata;
#endif

    printk(KERN_INFO "mstar-rtc probe\n");

    if (!device_can_wakeup(&pdev->dev))
        device_init_wakeup(&pdev->dev, 1);

    mstar_rtc = devm_kzalloc(&pdev->dev, sizeof(struct mstar_rtc_dev), GFP_KERNEL);
    if (!mstar_rtc) {
        dev_err(&pdev->dev, "unable to allocate memory for mstar-rtc\n");
        return -ENOMEM;
    }

#if defined(CONFIG_OF)
    printk(KERN_INFO "mstar-rtc: DTS\n");
    dn = pdev->dev.of_node;
    if (dn) {
        ret = mstar_rtc_parse_dt(dn, mstar_rtc);
        if (ret < 0) {
            dev_err(&pdev->dev, "failed to parse RTC device tree\n");
            return -1;
        }
    }
#else // platform device
    printk(KERN_INFO "mstar-rtc: platform_device\n");
    mstar_pdata = (struct mstar_rtc_plat_data *)pdev->dev.platform_data;
    mstar_rtc->mstar_rtc_param.u8PmRtcIndex = mstar_pdata->index;
    // clock freq = xtal / cw
    mstar_rtc->mstar_rtc_param.u32RtcCtrlWord = mstar_pdata->xtal / mstar_pdata->freq;
#endif

    spin_lock_init(&mstar_rtc->mstar_rtc_lock);
    mutex_init(&mstar_rtc->mutex_lock);

    platform_set_drvdata(pdev, mstar_rtc);

    MDrv_RTC_Init(&mstar_rtc->mstar_rtc_param);

    // provide an initial counter value for RTC to avoid hctosys return error.
    mstar_rtc->mstar_rtc_param.u32RtcSetCounter = 946684800;
    MDrv_RTC_SetCount(&mstar_rtc->mstar_rtc_param);

    mstar_rtc->rtc = rtc_device_register("mstar-rtc", &pdev->dev, &mstar_rtc_ops, THIS_MODULE);
    if (IS_ERR(mstar_rtc->rtc))
    {
        printk(KERN_ERR "mstar-rtc: fail to register RTC device: %ld\n", PTR_ERR(mstar_rtc->rtc));
        return PTR_ERR(mstar_rtc->rtc);
    }

#if defined (CONFIG_ANDROID)
    ret = Request_RTC_IRQ(mstar_rtc_irq_handler, &pdev->dev);
    if (ret)
    {
        printk(KERN_ERR "mstar-rtc: fail to request irq\n");
        return ret;
    }

	//disable_irq(E_IRQ_PM_SLEEP);
#endif

    printk(KERN_INFO "mstar-rtc: rtc%d registered\n", mstar_rtc->rtc->id);
    printk(KERN_DEBUG "mstar-rtc: control word = %d\n", mstar_rtc->mstar_rtc_param.u32RtcCtrlWord);

    return 0;
}

static int __exit mstar_rtc_remove(struct platform_device *pdev)
{
	struct rtc_device *rtc = platform_get_drvdata(pdev);

	rtc_device_unregister(rtc);

	return 0;
}

#ifdef CONFIG_PM_SLEEP
static int mstar_rtc_suspend(struct device *pdev)
{
    struct mstar_rtc_dev *mstar_rtc = dev_get_drvdata(pdev);
#ifdef CONFIG_MSTAR_PM
    PM_WakeCfg_t buf;

    memset(&buf, 0, sizeof(PM_WakeCfg_t));
    MDrv_PM_Read_Key(PM_KEY_DEFAULT, (char *)&buf);
    buf.bPmWakeEnableRTC0 = 0x1;
    MDrv_PM_Write_Key(PM_KEY_DEFAULT, (char *)&buf, sizeof(PM_WakeCfg_t));
#endif

    printk("%s\n", __func__);
    return 0;
}

static int mstar_rtc_resume(struct device *dev)
{
    printk("%s\n", __func__);
    return 0;
}
#endif

static SIMPLE_DEV_PM_OPS(mstar_rtc_pm_ops, mstar_rtc_suspend, mstar_rtc_resume);

static struct platform_driver mstar_rtc_driver = {
    .probe = mstar_rtc_probe,
	.driver = {
		.name = "mstar-rtc",
		.owner = THIS_MODULE,
#if defined(CONFIG_OF)
		.of_match_table = mstar_rtc_dt_match,
#endif
        .pm	= &mstar_rtc_pm_ops,
	},
	.remove = __exit_p(mstar_rtc_remove),
};

static int __init mstar_rtc_init(void)
{
    int ret;

    ret = platform_driver_register(&mstar_rtc_driver);
    if (ret < 0) {
        printk(KERN_ERR "mstar-rtc: fail to register RTC platform driver\n");
        return ret;
    }

#if !defined(CONFIG_OF)
    printk(KERN_DEBUG "mstar-rtc: RTC platform driver\n");
    ret = platform_device_register(&mstar_rtc_pdev);
    if (ret < 0) {
        printk(KERN_ERR "mstar-rtc: fail to register RTC platform device\n");
        platform_driver_unregister(&mstar_rtc_driver);
        return ret;
    }
#endif

    printk(KERN_DEBUG "mstar-rtc: init done\n");

	return 0;
}

static void __exit mstar_rtc_fini(void)
{
	platform_driver_unregister(&mstar_rtc_driver);
}

static int rtc_str_time_debug_show(struct seq_file *s, void *data)
{
	seq_printf(s, "%d\n", rtc_str_time);
}

static int rtc_str_time_debug_open(struct inode *inode, struct file *file)
{
	return single_open(file, rtc_str_time_debug_show, NULL);
}

static int rtc_str_time_debug_write(struct file *file, const char __user *userbuf, size_t count, loff_t *ppos)
{
	char buf[64];
	unsigned int set;
	count = min_t(size_t, count, (sizeof(buf)-1));
	if (copy_from_user(buf, userbuf, count))
		return -EFAULT;

	buf[count] = '\0';

	if (strict_strtol(buf, 0, &set) != 0)
		return -EINVAL;

	pr_info("set rtc_str_time to %d s\n", set);
	rtc_str_time = set;

	return count;
}

static const struct file_operations rtc_str_time_debug_fops = {
	.open		= rtc_str_time_debug_open,
	.read		= seq_read,
	.write		= rtc_str_time_debug_write,
	.llseek		= seq_lseek,
	.release	= single_release,
};

static int __init rtc_str_time_debug_init(void)
{
	struct deentry *d;

	d = debugfs_create_file("rtc_str_time", 0664, NULL, NULL,
		&rtc_str_time_debug_fops);
	if (!d) {
		pr_err("Failed to create rtc_str_time debug file \n");
		return -ENOMEM;
	}

	return 0;
}

module_init(mstar_rtc_init);
module_exit(mstar_rtc_fini);
late_initcall(rtc_str_time_debug_init);

MODULE_AUTHOR("Mstarsemi");
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("mstar RTC driver");
MODULE_ALIAS("platform:mstar-rtc");
