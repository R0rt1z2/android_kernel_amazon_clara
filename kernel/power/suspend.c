/*
 * kernel/power/suspend.c - Suspend to RAM and standby functionality.
 *
 * Copyright (c) 2003 Patrick Mochel
 * Copyright (c) 2003 Open Source Development Lab
 * Copyright (c) 2009 Rafael J. Wysocki <rjw@sisk.pl>, Novell Inc.
 *
 * This file is released under the GPLv2.
 */

#include <linux/string.h>
#include <linux/delay.h>
#include <linux/errno.h>
#include <linux/init.h>
#include <linux/console.h>
#include <linux/cpu.h>
#include <linux/cpuidle.h>
#include <linux/syscalls.h>
#include <linux/gfp.h>
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/list.h>
#include <linux/mm.h>
#include <linux/slab.h>
#include <linux/export.h>
#include <linux/suspend.h>
#include <linux/syscore_ops.h>
#include <linux/ftrace.h>
#include <linux/rtc.h>
#include <trace/events/power.h>
#include <linux/compiler.h>
#include <linux/moduleparam.h>
#include <linux/wakeup_reason.h>
#include <linux/pm_wakeup.h>
#include <linux/workqueue.h>
#include <linux/debugfs.h>
extern int mstr_cnt;

#include "power.h"
#ifdef CONFIG_MP_CMA_PATCH_MBOOT_STR_USE_CMA
#include <mdrv_miu.h>
#include <mdrv_cma_pool.h>
#endif

#if defined(CONFIG_MSTAR_PM)
#include <mdrv_pm.h>
#endif

#if defined(CONFIG_MSTAR_FRC_R2)
#include "mdrv_r2.h"
#endif

#ifdef CONFIG_MSTAR_IPAPOOL
#include <mdrv_ipa_pool.h>
#endif

#ifdef CONFIG_MP_R2_STR_ENABLE
#include "../../drivers/mstar2/include/mdrv_types.h"
#include "../../drivers/mstar2/include/mdrv_mstypes.h"
#include "../../drivers/mstar2/drv/mbx/mdrv_mbx.h"
#include "../../drivers/mstar2/drv/mbx/mapi_mbx.h"
extern unsigned long get_str_handshake_addr(void);
#endif

#if defined(CONFIG_CPU_FREQ_DEFAULT_GOV_ONDEMAND) || defined(CONFIG_CPU_FREQ_DEFAULT_GOV_INTERACTIVE) || defined(CONFIG_CPU_FREQ_DEFAULT_GOV_SCHEDUTIL)
#if defined(CONFIG_MSTAR_CPU_CLUSTER_CALIBRATING)
extern atomic_t ac_str_cpufreq;
extern void change_cpus_timer(char *caller, unsigned int target_freq, unsigned int cpu_id);
extern atomic_t disable_dvfs;
extern void Mdrv_CpuFreq_All_Lock(char *caller);
extern void Mdrv_CpuFreq_All_UnLock(char *caller);
#endif
#endif

static unsigned long str_start_time;

#if (MP_USB_STR_PATCH==1)
typedef enum
{
	E_STR_NONE,
	E_STR_IN_SUSPEND,
	E_STR_IN_RESUME
}EN_STR_STATUS;

static EN_STR_STATUS enStrStatus = E_STR_NONE;

bool is_suspending(void)
{
	return (enStrStatus == E_STR_IN_SUSPEND);
}
EXPORT_SYMBOL_GPL(is_suspending);
#endif

void str_time_start(void)
{
	str_start_time = jiffies;
}
EXPORT_SYMBOL(str_time_start);

void str_time_check(const char *label, const char *module, const char *function)//, unsigned int *str_time)
{
	long nj = jiffies - str_start_time;
	unsigned int msec;

	msec = jiffies_to_msecs(abs(nj));
	pr_emerg("\033[31m[STR %s Monitor][%s][%s] took %d milliseconds \033[m\n", label, module, function, msec);
}
EXPORT_SYMBOL(str_time_check);

const char *pm_labels[] = { "mem", "standby", "freeze", NULL };
const char *pm_states[PM_SUSPEND_MAX];

unsigned int pm_suspend_global_flags;
EXPORT_SYMBOL_GPL(pm_suspend_global_flags);

static const struct platform_suspend_ops *suspend_ops;
static const struct platform_freeze_ops *freeze_ops;
static DECLARE_WAIT_QUEUE_HEAD(suspend_freeze_wait_head);

enum freeze_state __read_mostly suspend_freeze_state;
static DEFINE_SPINLOCK(suspend_freeze_lock);

void freeze_set_ops(const struct platform_freeze_ops *ops)
{
	lock_system_sleep();
	freeze_ops = ops;
	unlock_system_sleep();
}

static void freeze_begin(void)
{
	suspend_freeze_state = FREEZE_STATE_NONE;
}

static void freeze_enter(void)
{
	spin_lock_irq(&suspend_freeze_lock);
	if (pm_wakeup_pending())
		goto out;

	suspend_freeze_state = FREEZE_STATE_ENTER;
	spin_unlock_irq(&suspend_freeze_lock);

	get_online_cpus();
	cpuidle_resume();

	/* Push all the CPUs into the idle loop. */
	wake_up_all_idle_cpus();
	pr_debug("PM: suspend-to-idle\n");
	/* Make the current CPU wait so it can enter the idle loop too. */
	wait_event(suspend_freeze_wait_head,
		   suspend_freeze_state == FREEZE_STATE_WAKE);
	pr_debug("PM: resume from suspend-to-idle\n");

	cpuidle_pause();
	put_online_cpus();

	spin_lock_irq(&suspend_freeze_lock);

 out:
	suspend_freeze_state = FREEZE_STATE_NONE;
	spin_unlock_irq(&suspend_freeze_lock);
}

void freeze_wake(void)
{
	unsigned long flags;

	spin_lock_irqsave(&suspend_freeze_lock, flags);
	if (suspend_freeze_state > FREEZE_STATE_NONE) {
		suspend_freeze_state = FREEZE_STATE_WAKE;
		wake_up(&suspend_freeze_wait_head);
	}
	spin_unlock_irqrestore(&suspend_freeze_lock, flags);
}
EXPORT_SYMBOL_GPL(freeze_wake);

static bool valid_state(suspend_state_t state)
{
	/*
	 * PM_SUSPEND_STANDBY and PM_SUSPEND_MEM states need low level
	 * support and need to be valid to the low level
	 * implementation, no valid callback implies that none are valid.
	 */
	return suspend_ops && suspend_ops->valid && suspend_ops->valid(state);
}

/*
 * If this is set, the "mem" label always corresponds to the deepest sleep state
 * available, the "standby" label corresponds to the second deepest sleep state
 * available (if any), and the "freeze" label corresponds to the remaining
 * available sleep state (if there is one).
 */
static bool relative_states;

void __init pm_states_init(void)
{
	/*
	 * freeze state should be supported even without any suspend_ops,
	 * initialize pm_states accordingly here
	 */
	pm_states[PM_SUSPEND_FREEZE] = pm_labels[relative_states ? 0 : 2];
}

static int __init sleep_states_setup(char *str)
{
	relative_states = !strncmp(str, "1", 1);
	return 1;
}

__setup("relative_sleep_states=", sleep_states_setup);

/**
 * suspend_set_ops - Set the global suspend method table.
 * @ops: Suspend operations to use.
 */
void suspend_set_ops(const struct platform_suspend_ops *ops)
{
	suspend_state_t i;
	int j = 0;

	lock_system_sleep();

	suspend_ops = ops;
	for (i = PM_SUSPEND_MEM; i >= PM_SUSPEND_STANDBY; i--)
		if (valid_state(i)) {
			pm_states[i] = pm_labels[j++];
		} else if (!relative_states) {
			pm_states[i] = NULL;
			j++;
		}

	pm_states[PM_SUSPEND_FREEZE] = pm_labels[j];

	unlock_system_sleep();
}
EXPORT_SYMBOL_GPL(suspend_set_ops);

/**
 * suspend_valid_only_mem - Generic memory-only valid callback.
 *
 * Platform drivers that implement mem suspend only and only need to check for
 * that in their .valid() callback can use this instead of rolling their own
 * .valid() callback.
 */
int suspend_valid_only_mem(suspend_state_t state)
{
	return state == PM_SUSPEND_MEM;
}
EXPORT_SYMBOL_GPL(suspend_valid_only_mem);

static bool sleep_state_supported(suspend_state_t state)
{
	return state == PM_SUSPEND_FREEZE || (suspend_ops && suspend_ops->enter);
}

static int platform_suspend_prepare(suspend_state_t state)
{
	return state != PM_SUSPEND_FREEZE && suspend_ops->prepare ?
		suspend_ops->prepare() : 0;
}

static int platform_suspend_prepare_late(suspend_state_t state)
{
	return state == PM_SUSPEND_FREEZE && freeze_ops && freeze_ops->prepare ?
		freeze_ops->prepare() : 0;
}

static int platform_suspend_prepare_noirq(suspend_state_t state)
{
	return state != PM_SUSPEND_FREEZE && suspend_ops->prepare_late ?
		suspend_ops->prepare_late() : 0;
}

static void platform_resume_noirq(suspend_state_t state)
{
	if (state != PM_SUSPEND_FREEZE && suspend_ops->wake)
		suspend_ops->wake();
}

static void platform_resume_early(suspend_state_t state)
{
	if (state == PM_SUSPEND_FREEZE && freeze_ops && freeze_ops->restore)
		freeze_ops->restore();
}

static void platform_resume_finish(suspend_state_t state)
{
	if (state != PM_SUSPEND_FREEZE && suspend_ops->finish)
		suspend_ops->finish();
}

static int platform_suspend_begin(suspend_state_t state)
{
	if (state == PM_SUSPEND_FREEZE && freeze_ops && freeze_ops->begin)
		return freeze_ops->begin();
	else if (suspend_ops && suspend_ops->begin)
		return suspend_ops->begin(state);
	else
		return 0;
}

#ifdef CONFIG_MP_R2_STR_ENABLE
u32 kernel_read_phys(u64 phys_addr)
{
	u32 phys_addr_page = phys_addr & 0xFFFFE000;
	u32 phys_offset    = phys_addr & 0x00001FFF;
	u32 map_size       = phys_offset + sizeof(u32);
	u32 ret		   = 0xDEADBEEF;
	void *mem_mapped = ioremap_nocache(phys_addr_page, map_size);
	if (NULL != mem_mapped) {
		ret = (u32)ioread32(((u8 *)mem_mapped) + phys_offset);
		iounmap(mem_mapped);
	}

	return ret;
}
#endif

static void platform_resume_end(suspend_state_t state)
{
	if (state == PM_SUSPEND_FREEZE && freeze_ops && freeze_ops->end)
		freeze_ops->end();
	else if (suspend_ops && suspend_ops->end)
		suspend_ops->end();
}

static void platform_recover(suspend_state_t state)
{
	if (state != PM_SUSPEND_FREEZE && suspend_ops->recover)
		suspend_ops->recover();
}

static bool platform_suspend_again(suspend_state_t state)
{
	return state != PM_SUSPEND_FREEZE && suspend_ops->suspend_again ?
		suspend_ops->suspend_again() : false;
}

#ifdef CONFIG_PM_DEBUG
static unsigned int pm_test_delay = 5;
module_param(pm_test_delay, uint, 0644);
MODULE_PARM_DESC(pm_test_delay,
		 "Number of seconds to wait before resuming from suspend test");
#endif

static int suspend_test(int level)
{
#ifdef CONFIG_PM_DEBUG
	if (pm_test_level == level) {
		pr_info("suspend debug: Waiting for %d second(s).\n",
				pm_test_delay);
		mdelay(pm_test_delay * 1000);
		return 1;
	}
#endif /* !CONFIG_PM_DEBUG */
	return 0;
}

/**
 * suspend_prepare - Prepare for entering system sleep state.
 *
 * Common code run for every system sleep state that can be entered (except for
 * hibernation).  Run suspend notifiers, allocate the "suspend" console and
 * freeze processes.
 */
static int suspend_prepare(suspend_state_t state)
{
	int error, nr_calls = 0;

	if (!sleep_state_supported(state))
		return -EPERM;

	pm_prepare_console();

	error = __pm_notifier_call_chain(PM_SUSPEND_PREPARE, -1, &nr_calls);
	if (error) {
		nr_calls--;
		goto Finish;
	}

#if defined(CONFIG_CPU_FREQ_DEFAULT_GOV_ONDEMAND) || defined(CONFIG_CPU_FREQ_DEFAULT_GOV_INTERACTIVE) || defined(CONFIG_CPU_FREQ_DEFAULT_GOV_SCHEDUTIL)
#if defined(CONFIG_MSTAR_CPU_CLUSTER_CALIBRATING)
	int i = 0;
	/* Disable DVFS before suspend */
	Mdrv_CpuFreq_All_Lock((char *)__func__);
	atomic_set(&disable_dvfs, 1);
	pr_info("\033[0;32;31m%s %d Disable DVFS in STR\033[m\n", __func__, __LINE__);
	Mdrv_CpuFreq_All_UnLock((char *)__func__);

	/* Reset cpufreq and voltage to default setting */
	pr_err("\033[36mFunction = %s, Line = %d,  setting cpufreq\033[m\n", __PRETTY_FUNCTION__, __LINE__);
	for (i = 0; i < CONFIG_NR_CPUS; i ++)
		change_cpus_timer((char *)__FUNCTION__, 54472, i);
	mdelay(100);
#endif
#endif
	trace_suspend_resume(TPS("freeze_processes"), 0, true);
	error = suspend_freeze_processes();
	trace_suspend_resume(TPS("freeze_processes"), 0, false);
	if (!error) {
#ifdef CONFIG_MP_CMA_PATCH_MBOOT_STR_USE_CMA
#ifdef CONFIG_MSTAR_CMAPOOL
#ifdef CONFIG_MSTAR_IPAPOOL
		pr_err("\033[0;32;31m%s %d error,"
			"should never enable both CONFIG_MSTAR_CMAPOOL and CONFIG_MSTAR_IPAPOOL !\033[m\n",
			__func__, __LINE__);
		BUG();
#endif
#endif
		/*
		 * allocate all freed cma_memory from a mboot co-buffer cma_region,
		 * to prevent the kernel data is still @ the mboot co-buffer cma_region,
		 * and thus, the kernel data will be corrupted by mboot
		 */
#ifdef CONFIG_MSTAR_CMAPOOL
		str_reserve_mboot_cma_buffer();
#else
#ifdef CONFIG_MSTAR_IPAPOOL
		str_reserve_mboot_ipa_str_pool_buffer();
#endif
#endif
#endif
		return 0;
	}

	suspend_stats.failed_freeze++;
	dpm_save_failed_step(SUSPEND_FREEZE);
 Finish:
	__pm_notifier_call_chain(PM_POST_SUSPEND, nr_calls, NULL);

#if defined(CONFIG_CPU_FREQ_DEFAULT_GOV_ONDEMAND) || defined(CONFIG_CPU_FREQ_DEFAULT_GOV_INTERACTIVE) || defined(CONFIG_CPU_FREQ_DEFAULT_GOV_SCHEDUTIL)
#if defined(CONFIG_MSTAR_CPU_CLUSTER_CALIBRATING)
	/* Enable DVFS after resume, this is error case */
	Mdrv_CpuFreq_All_Lock((char *)__func__);
	atomic_set(&disable_dvfs, 0);
	pr_err("\033[0;32;31m%s %d Enable DVFS in STR (Error case)\033[m\n", __func__, __LINE__);
	Mdrv_CpuFreq_All_UnLock((char *)__func__);
#endif
#endif
	pm_restore_console();

	return error;
}

/* default implementation */
void __weak arch_suspend_disable_irqs(void)
{
	local_irq_disable();
}

/* default implementation */
void __weak arch_suspend_enable_irqs(void)
{
	local_irq_enable();
}

/**
 * suspend_enter - Make the system enter the given sleep state.
 * @state: System sleep state to enter.
 * @wakeup: Returns information that the sleep state should not be re-entered.
 *
 * This function should be called after devices have been suspended.
 */
static int suspend_enter(suspend_state_t state, bool *wakeup)
{
	char suspend_abort[MAX_SUSPEND_ABORT_LEN];
	int error, last_dev;

	error = platform_suspend_prepare(state);
	if (error) {
#if (MP_USB_STR_PATCH == 1)
		enStrStatus = E_STR_IN_RESUME;
#endif
		goto Platform_finish;
	}

	error = dpm_suspend_late(PMSG_SUSPEND);
	if (error) {
		last_dev = suspend_stats.last_failed_dev + REC_FAILED_NUM - 1;
		last_dev %= REC_FAILED_NUM;
		pr_err("PM: late suspend of devices failed\n");
		log_suspend_abort_reason("%s device failed to power down",
			suspend_stats.failed_devs[last_dev]);
#if (MP_USB_STR_PATCH == 1)
		enStrStatus = E_STR_IN_RESUME;
#endif
		goto Platform_finish;
	}

	error = platform_suspend_prepare_late(state);
	if (error) {
#if (MP_USB_STR_PATCH == 1)
		enStrStatus = E_STR_IN_RESUME;
#endif
		goto Devices_early_resume;
	}

	error = dpm_suspend_noirq(PMSG_SUSPEND);
	if (error) {
		last_dev = suspend_stats.last_failed_dev + REC_FAILED_NUM - 1;
		last_dev %= REC_FAILED_NUM;
		pr_err("PM: noirq suspend of devices failed\n");
		log_suspend_abort_reason("noirq suspend of %s device failed",
			suspend_stats.failed_devs[last_dev]);
#if (MP_USB_STR_PATCH == 1)
		enStrStatus = E_STR_IN_RESUME;
#endif
		goto Platform_early_resume;
	}

	error = platform_suspend_prepare_noirq(state);
	if (error) {
#if (MP_USB_STR_PATCH == 1)
		enStrStatus = E_STR_IN_RESUME;
#endif
		goto Platform_wake;
	}

	if (suspend_test(TEST_PLATFORM)) {
#if (MP_USB_STR_PATCH == 1)
		enStrStatus = E_STR_IN_RESUME;
#endif
		goto Platform_wake;
	}

	/*
	 * PM_SUSPEND_FREEZE equals
	 * frozen processes + suspended devices + idle processors.
	 * Thus we should invoke freeze_enter() soon after
	 * all the devices are suspended.
	 */
	if (state == PM_SUSPEND_FREEZE) {
		trace_suspend_resume(TPS("machine_suspend"), state, true);
		freeze_enter();
		trace_suspend_resume(TPS("machine_suspend"), state, false);
#if (MP_USB_STR_PATCH == 1)
		enStrStatus = E_STR_IN_RESUME;
#endif
		goto Platform_wake;
	}

	error = disable_nonboot_cpus();
	if (error || suspend_test(TEST_CPUS)) {
		log_suspend_abort_reason("Disabling non-boot cpus failed");
#if (MP_USB_STR_PATCH == 1)
		enStrStatus = E_STR_IN_RESUME;
#endif
		goto Enable_cpus;
	}

	arch_suspend_disable_irqs();
	BUG_ON(!irqs_disabled());

	error = syscore_suspend();
	if (!error) {
#if defined(CONFIG_MP_MSTAR_STR_BASE)
		if (is_mstar_str()) {
			*wakeup = false;
			} else
#endif
			*wakeup = pm_wakeup_pending();
		if (!(suspend_test(TEST_CORE) || *wakeup)) {
			trace_suspend_resume(TPS("machine_suspend"),
				state, true);
#if defined(CONFIG_MSTAR_PM)
			if (MDrv_PM_Suspend(E_PM_STATE_SUSPEND_PREPARE) != E_PM_OK) {
				pr_info("MDrv_PM_Suspend failed\n");
				error = -EFAULT;
			}
#endif
			error = suspend_ops->enter(state);
			trace_suspend_resume(TPS("machine_suspend"),
				state, false);
			events_check_enabled = false;
#if defined(CONFIG_MP_MSTAR_STR_BASE)
			set_state_value(STENT_RESUME_FROM_SUSPEND);
#endif
		} else if (*wakeup) {
			pm_get_active_wakeup_sources(suspend_abort,
				MAX_SUSPEND_ABORT_LEN);
			log_suspend_abort_reason(suspend_abort);
			error = -EBUSY;
		}
#if (MP_USB_STR_PATCH == 1)
		enStrStatus = E_STR_IN_RESUME;
#endif
		syscore_resume();
	} else {
#if (MP_USB_STR_PATCH == 1)
		enStrStatus = E_STR_IN_RESUME;
#endif
	}

	arch_suspend_enable_irqs();
	BUG_ON(irqs_disabled());

 Enable_cpus:
	str_time_start();
	pr_emerg("\033[31m[STR Resume Monitor][Kernel][%s] ##### START ##### \033[m\n", __FUNCTION__);
	enable_nonboot_cpus();

 Platform_wake:
	platform_resume_noirq(state);
	dpm_resume_noirq(PMSG_RESUME);

 Platform_early_resume:
	platform_resume_early(state);

 Devices_early_resume:
	dpm_resume_early(PMSG_RESUME);

 Platform_finish:
	platform_resume_finish(state);
	return error;
}

/**
 * suspend_devices_and_enter - Suspend devices and enter system sleep state.
 * @state: System sleep state to enter.
 */
#define UTOPIA2K_STR_NAME "Mstar-utopia2k-str"
#define MIK_NAME "mik.0"
int suspend_devices_and_enter(suspend_state_t state)
{
	int error;
	bool wakeup = false;

#ifdef CONFIG_MSTAR_UTOPIA2K_STR
	struct device *dev_utopia2k = NULL;
	struct device *dev_mik = NULL;
#endif

	if (!sleep_state_supported(state))
		return -ENOSYS;

	error = platform_suspend_begin(state);
	if (error) {
#if (MP_USB_STR_PATCH == 1)
		enStrStatus = E_STR_IN_RESUME;
#endif
		goto Close;
	}
#ifdef CONFIG_MSTAR_UTOPIA2K_STR
	extern struct device* dpm_get_dev(const char* name);
	extern void device_pm_add(struct device *dev);
	extern void device_pm_remove(struct device *dev);

	dev_utopia2k = dpm_get_dev(UTOPIA2K_STR_NAME);
	if (dev_utopia2k) {
		device_pm_remove (dev_utopia2k);
		device_pm_add (dev_utopia2k);
	}
	dev_mik = dpm_get_dev(MIK_NAME);
	if (dev_mik) {
		device_pm_remove (dev_mik);
		device_pm_add (dev_mik);
	}
#endif
	suspend_console();
	suspend_test_start();
	error = dpm_suspend_start(PMSG_SUSPEND);
	if (error) {
		pr_err("PM: Some devices failed to suspend, or early wake event detected\n");
		log_suspend_abort_reason("Some devices failed to suspend, or early wake event detected");
#if (MP_USB_STR_PATCH == 1)
		enStrStatus = E_STR_IN_RESUME;
#endif
		goto Recover_platform;
	}
	suspend_test_finish("suspend devices");
	if (suspend_test(TEST_DEVICES)) {
#if (MP_USB_STR_PATCH == 1)
		enStrStatus = E_STR_IN_RESUME;
#endif
		goto Recover_platform;
	}

	do {
		error = suspend_enter(state, &wakeup);
	} while (!error && !wakeup && platform_suspend_again(state));

 Resume_devices:
	suspend_test_start();
	dpm_resume_end(PMSG_RESUME);
	suspend_test_finish("resume devices");
	trace_suspend_resume(TPS("resume_console"), state, true);
	resume_console();
	trace_suspend_resume(TPS("resume_console"), state, false);
#if defined(CONFIG_MSTAR_FRC_R2)
	MDrv_FRC_R2_Resume(E_R2_STATE_POWER_ON_DC);
#endif

 Close:
	platform_resume_end(state);
	return error;

 Recover_platform:
	platform_recover(state);
	goto Resume_devices;
}

/**
 * suspend_finish - Clean up before finishing the suspend sequence.
 *
 * Call platform code to clean up, restart processes, and free the console that
 * we've allocated. This routine is not called for hibernation.
 */
static void suspend_finish(void)
{
#if defined(CONFIG_CPU_FREQ_DEFAULT_GOV_ONDEMAND) || defined(CONFIG_CPU_FREQ_DEFAULT_GOV_INTERACTIVE) || defined(CONFIG_CPU_FREQ_DEFAULT_GOV_SCHEDUTIL)
#if (defined CONFIG_MSTAR_CPU_CLUSTER_CALIBRATING)
	/* Enable DVFS after resume, this is error case */
	Mdrv_CpuFreq_All_Lock((char *)__func__);
	atomic_set(&disable_dvfs, 0);
	pr_info("\033[0;32;31m%s %d Enable DVFS in STR\033[m\n", __func__, __LINE__);
	Mdrv_CpuFreq_All_UnLock((char *)__func__);
#endif
#endif

#ifdef CONFIG_MP_CMA_PATCH_MBOOT_STR_USE_CMA
#ifdef CONFIG_MSTAR_CMAPOOL
#ifdef CONFIG_MSTAR_IPAPOOL
	pr_err("\033[0;32;31m%s %d error, should never enable both CONFIG_MSTAR_CMAPOOL and CONFIG_MSTAR_IPAPOOL !\033[m\n", __func__, __LINE__)
	BUG();
#endif
#endif
	/* free all pre-allocated cma_memory from a mboot co-buffer cma_region */
#ifdef CONFIG_MSTAR_CMAPOOL
	str_release_mboot_cma_buffer();
#else
#ifdef CONFIG_MSTAR_IPAPOOL
	str_release_mboot_ipa_str_pool_buffer();
#endif
#endif
#endif
	suspend_thaw_processes();
	pm_notifier_call_chain(PM_POST_SUSPEND);
	pm_restore_console();
}

#if defined(CONFIG_MSTAR_PM)
static struct wakeup_source *pm_wakeup_source;
static struct workqueue_struct *pm_wakeup_workqueue;

static void pm_resume(void)
{
	MDrv_PM_Resume(E_PM_STATE_POWER_ON_DC);

	if(pm_wakeup_source)
		__pm_relax(pm_wakeup_source);
}

static DECLARE_WORK(pm_wakeup_wq, pm_resume);
#endif

/**
 * enter_state - Do common work needed to enter system sleep state.
 * @state: System sleep state to enter.
 *
 * Make sure that no one else is trying to put the system into a sleep state.
 * Fail if that's not the case.  Otherwise, prepare for system suspend, make the
 * system enter the given sleep state and clean up after wakeup.
 */
static int enter_state(suspend_state_t state)
{
	int error;
#ifdef CONFIG_MP_R2_STR_ENABLE
	volatile unsigned long u64TEESTRBOOTFLAG;
#endif
#if defined(CONFIG_MP_MSTAR_STR_BASE)
	int bresumefromsuspend = 0;
#endif

	trace_suspend_resume(TPS("suspend_enter"), state, true);
	if (state == PM_SUSPEND_FREEZE) {
#ifdef CONFIG_PM_DEBUG
		if (pm_test_level != TEST_NONE && pm_test_level <= TEST_CPUS) {
			pr_warn("PM: Unsupported test mode for suspend to idle, please choose none/freezer/devices/platform.\n");
			return -EAGAIN;
		}
#endif
	} else if (!valid_state(state)) {
		return -EINVAL;
	}
	if (!mutex_trylock(&pm_mutex))
		return -EBUSY;

#if defined(CONFIG_MP_MSTAR_STR_BASE)
#if defined(CONFIG_MSTAR_PM)
	MDrv_PM_Suspend(E_PM_STATE_STORE_INFO);
#endif
#if defined(CONFIG_MSTAR_FRC_R2)
	MDrv_FRC_R2_Suspend(E_R2_STATE_STORE_INFO);
#endif

	set_state_entering();
#if (MP_USB_STR_PATCH==1)
	enStrStatus = E_STR_IN_SUSPEND;
#endif
try_again:
#endif
	if (state == PM_SUSPEND_FREEZE)
		freeze_begin();

#ifndef CONFIG_SUSPEND_SKIP_SYNC
	trace_suspend_resume(TPS("sync_filesystems"), 0, true);
	pr_info("PM: Syncing filesystems ... ");
	sys_sync();
	pr_cont("done.\n");
	trace_suspend_resume(TPS("sync_filesystems"), 0, false);
#endif

	pr_debug("PM: Preparing system for sleep (%s)\n", pm_states[state]);
	pm_suspend_clear_flags();
	error = suspend_prepare(state);
	if (error)
		goto Unlock;

#ifdef CONFIG_MP_R2_STR_ENABLE
#define MIU0_BASE               0x20000000
#define STR_FLAG_SUSPEND_FINISH 0xFFFF8888
	if(TEEINFO_TYPTE == SECURITY_TEEINFO_OSTYPE_NUTTX) {
		printk(KERN_INFO "TEE mode: Nuttx\n");
		u64TEESTRBOOTFLAG = get_str_handshake_addr();
		if(u64TEESTRBOOTFLAG != 0) {
			pr_info("PM: Send MBX to TEE for STR_Suspend  ... \n");

			//1. Setup Suspend Flag to 0
			pr_info("PM: u64TEESTRBOOTFLAG => Addr = 0x%x  !!!!\n", u64TEESTRBOOTFLAG);
			pr_info("PM: u64TEESTRBOOTFLAG => Value = 0x%x !!!! \n", kernel_read_phys(u64TEESTRBOOTFLAG));

			//2. Send Mailbox to TEE (PA!!!)
			MApi_MBX_NotifyTeetoSuspend(u64TEESTRBOOTFLAG - MIU0_BASE);

			//3. Waiting TEE to finish susepnd jobs
			while (kernel_read_phys(u64TEESTRBOOTFLAG) != STR_FLAG_SUSPEND_FINISH) {
				mdelay(400);
				pr_info("PM: Waiting TEE suspend done signal!!! 0x%x \n", kernel_read_phys(u64TEESTRBOOTFLAG));
			}
		} else {
			pr_info("Normal STR flow\n");
		}
	}
#endif
	if (suspend_test(TEST_FREEZER))
		goto Finish;

	trace_suspend_resume(TPS("suspend_enter"), state, false);
	pr_debug("PM: Suspending system (%s)\n", pm_states[state]);
	pm_restrict_gfp_mask();
	error = suspend_devices_and_enter(state);
	pm_restore_gfp_mask();
#if defined(CONFIG_MSTAR_PM)
	//RTPM start resume here
	if (pm_wakeup_source)
		__pm_stay_awake(pm_wakeup_source);

	queue_work(pm_wakeup_workqueue, &pm_wakeup_wq);
#endif

 Finish:
	pr_debug("PM: Finishing wakeup.\n");
#if defined(CONFIG_MP_MSTAR_STR_BASE)
	if (STENT_RESUME_FROM_SUSPEND == get_state_value()) {
		clear_state_entering();
		bresumefromsuspend = 1;
	}
#endif
	suspend_finish();
 Unlock:
#if defined(CONFIG_MP_MSTAR_STR_BASE)
#if defined(CONFIG_MSTAR_STR_ACOFF_ON_ERR)
	if (error) {
		extern void mstar_str_notifypmerror_off(void);
		mstar_str_notifypmerror_off(); //it won't return, wait pm to power off
	}
#endif
	if (is_mstar_str() && bresumefromsuspend == 0) {
		schedule_timeout_interruptible(HZ);
		goto try_again;
	}
#if (MP_USB_STR_PATCH==1)
	enStrStatus = E_STR_NONE;
#endif
#endif
	mutex_unlock(&pm_mutex);
	return error;
}

static void pm_suspend_marker(char *annotation)
{
	struct timespec ts;
	struct rtc_time tm;

	getnstimeofday(&ts);
	rtc_time_to_tm(ts.tv_sec, &tm);
	pr_info("PM: suspend %s %d-%02d-%02d %02d:%02d:%02d.%09lu UTC\n",
		annotation, tm.tm_year + 1900, tm.tm_mon + 1, tm.tm_mday,
		tm.tm_hour, tm.tm_min, tm.tm_sec, ts.tv_nsec);
}

/**
 * pm_suspend - Externally visible function for suspending the system.
 * @state: System sleep state to enter.
 *
 * Check if the value of @state represents one of the supported states,
 * execute enter_state() and update system suspend statistics.
 */
int pm_suspend(suspend_state_t state)
{
	int error;

	if (state <= PM_SUSPEND_ON || state >= PM_SUSPEND_MAX)
		return -EINVAL;

	pm_suspend_marker("entry");
	error = enter_state(state);
	if (error) {
		suspend_stats.fail++;
		dpm_save_failed_errno(error);
	} else {
		suspend_stats.success++;
	}
	pm_suspend_marker("exit");
	return error;
}
EXPORT_SYMBOL(pm_suspend);

#if defined(CONFIG_MP_MSTAR_STR_BASE) && defined(CONFIG_DEBUG_FS)
static int str_count_debug_show(struct seq_file *s, void *data)
{
	seq_printf(s, "%d\n", mstr_cnt);
	return 0;
}

static int str_count_debug_open(struct inode *inode, struct file *file)
{
	return single_open(file, str_count_debug_show, NULL);
}

static const struct file_operations str_count_debug_fops = {
	.open		= str_count_debug_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= single_release,
};

static int __init str_count_debug_init(void)
{
	struct dentry *d;

	d = debugfs_create_file("str_count", S_IFREG | S_IRUGO, NULL, NULL,
		&str_count_debug_fops);
	if (!d) {
		pr_err("Failed to create str_count debug file\n");
		return -ENOMEM;
	}

	return 0;
}

late_initcall(str_count_debug_init);
#endif

#if defined(CONFIG_MSTAR_PM)
static int __init pm_workqueue_init(void)
{
	pm_wakeup_source = wakeup_source_register("pm_ws");
	if (!pm_wakeup_source)
		return -ENOMEM;

	pm_wakeup_workqueue = create_singlethread_workqueue("pm_wq");
	if (pm_wakeup_workqueue)
		return 0;

	wakeup_source_unregister(pm_wakeup_source);

	return -ENOMEM;
}
__initcall(pm_workqueue_init);
#endif
