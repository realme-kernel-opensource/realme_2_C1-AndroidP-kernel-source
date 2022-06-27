/* Copyright (c) 2012-2018, The Linux Foundation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/io.h>
#include <linux/delay.h>
#include <linux/slab.h>
#include <linux/jiffies.h>
#include <linux/kthread.h>
#include <linux/mutex.h>
#include <linux/sched.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/percpu.h>
#include <linux/of.h>
#include <linux/cpu.h>
#include <linux/cpu_pm.h>
#include <linux/platform_device.h>
#include <linux/wait.h>
#include <soc/qcom/scm.h>
#include <soc/qcom/memory_dump.h>
#include <soc/qcom/minidump.h>
#include <soc/qcom/watchdog.h>
#include <linux/dma-mapping.h>

#ifdef VENDOR_EDIT
#include "oppo_watchdog_util.h"
#endif

#ifdef ODM_WT_EDIT
#include <linux/wt_system_monitor.h>
#endif
#define MODULE_NAME "msm_watchdog"
#define WDT0_ACCSCSSNBARK_INT 0
#define TCSR_WDT_CFG	0x30
#define WDT0_RST	0x04
#define WDT0_EN		0x08
#define WDT0_STS	0x0C
#define WDT0_BARK_TIME	0x10
#define WDT0_BITE_TIME	0x14

#define WDOG_ABSENT	0

#define EN		0
#define UNMASKED_INT_EN 1

#define MASK_SIZE		32
#define SCM_SET_REGSAVE_CMD	0x2
#define SCM_SVC_SEC_WDOG_DIS	0x7
#define MAX_CPU_CTX_SIZE	2048
#define MAX_CPU_SCANDUMP_SIZE	0x10100

static struct msm_watchdog_data *wdog_data;

int cpu_idle_pc_state[NR_CPUS];

/*
 * user_pet_enable:
 *	Require userspace to write to a sysfs file every pet_time milliseconds.
 *	Disabled by default on boot.
 */
struct msm_watchdog_data {
	unsigned int __iomem phys_base;
	size_t size;
	void __iomem *base;
	void __iomem *wdog_absent_base;
	struct device *dev;
	unsigned int pet_time;
	unsigned int bark_time;
	unsigned int bark_irq;
	unsigned int bite_irq;
	bool do_ipi_ping;
	bool wakeup_irq_enable;
	unsigned long long last_pet;
	unsigned int min_slack_ticks;
	unsigned long long min_slack_ns;
	void *scm_regsave;
	cpumask_t alive_mask;
	struct mutex disable_lock;
	bool irq_ppi;
	struct msm_watchdog_data __percpu **wdog_cpu_dd;
	struct notifier_block panic_blk;

	bool enabled;
	bool user_pet_enabled;

	struct task_struct *watchdog_task;
	struct timer_list pet_timer;
	wait_queue_head_t pet_complete;

	bool timer_expired;
	bool user_pet_complete;
	unsigned int scandump_size;
};

/*
 * On the kernel command line specify
 * watchdog_v2.enable=1 to enable the watchdog
 * By default watchdog is turned on
 */
static int enable = 1;
module_param(enable, int, 0);

/*
 * On the kernel command line specify
 * watchdog_v2.WDT_HZ=<clock val in HZ> to set Watchdog
 * ticks. By default it is set to 32765.
 */
static long WDT_HZ = 32765;
module_param(WDT_HZ, long, 0);

/*
 * Watchdog ipi optimization:
 * Does not ping cores in low power mode at pet time to save power.
 * This feature is enabled by default.
 *
 * On the kernel command line specify
 * watchdog_v2.ipi_en=1 to disable this optimization.
 * Or, can be turned off, by enabling CONFIG_QCOM_WDOG_IPI_ENABLE.
 */
#ifdef CONFIG_QCOM_WDOG_IPI_ENABLE
#define IPI_CORES_IN_LPM 1
#else
#define IPI_CORES_IN_LPM 0
#endif

static int ipi_en = IPI_CORES_IN_LPM;
module_param(ipi_en, int, 0444);

static void dump_cpu_alive_mask(struct msm_watchdog_data *wdog_dd)
{
	static char alive_mask_buf[MASK_SIZE];

	scnprintf(alive_mask_buf, MASK_SIZE, "%*pb1", cpumask_pr_args(
				&wdog_dd->alive_mask));
	dev_info(wdog_dd->dev, "cpu alive mask from last pet %s\n",
				alive_mask_buf);
}

static int msm_watchdog_suspend(struct device *dev)
{
	struct msm_watchdog_data *wdog_dd =
			(struct msm_watchdog_data *)dev_get_drvdata(dev);
	if (!enable)
		return 0;
	__raw_writel(1, wdog_dd->base + WDT0_RST);
	if (wdog_dd->wakeup_irq_enable) {
		/* Make sure register write is complete before proceeding */
		mb();
		wdog_dd->last_pet = sched_clock();
		return 0;
	}
	__raw_writel(0, wdog_dd->base + WDT0_EN);
	/* Make sure watchdog is suspended before setting enable */
	mb();
	wdog_dd->enabled = false;
	wdog_dd->last_pet = sched_clock();
	return 0;
}

static int msm_watchdog_resume(struct device *dev)
{
	struct msm_watchdog_data *wdog_dd =
			(struct msm_watchdog_data *)dev_get_drvdata(dev);
	if (!enable)
		return 0;
	if (wdog_dd->wakeup_irq_enable) {
		__raw_writel(1, wdog_dd->base + WDT0_RST);
		/* Make sure register write is complete before proceeding */
		mb();
		wdog_dd->last_pet = sched_clock();
		return 0;
	}
	__raw_writel(1, wdog_dd->base + WDT0_EN);
	__raw_writel(1, wdog_dd->base + WDT0_RST);
	/* Make sure watchdog is reset before setting enable */
	mb();
	wdog_dd->enabled = true;
	wdog_dd->last_pet = sched_clock();
	return 0;
}

static int panic_wdog_handler(struct notifier_block *this,
			      unsigned long event, void *ptr)
{
	struct msm_watchdog_data *wdog_dd = container_of(this,
				struct msm_watchdog_data, panic_blk);
	if (panic_timeout == 0) {
		__raw_writel(0, wdog_dd->base + WDT0_EN);
		/* Make sure watchdog is enabled before notifying the caller */
		mb();
	} else {
		__raw_writel(WDT_HZ * (panic_timeout + 10),
				wdog_dd->base + WDT0_BARK_TIME);
		__raw_writel(WDT_HZ * (panic_timeout + 10),
				wdog_dd->base + WDT0_BITE_TIME);
		__raw_writel(1, wdog_dd->base + WDT0_RST);
	}
	return NOTIFY_DONE;
}

static void wdog_disable(struct msm_watchdog_data *wdog_dd)
{
	__raw_writel(0, wdog_dd->base + WDT0_EN);
	/* Make sure watchdog is disabled before proceeding */
	mb();
	if (wdog_dd->irq_ppi) {
		disable_percpu_irq(wdog_dd->bark_irq);
		free_percpu_irq(wdog_dd->bark_irq, wdog_dd->wdog_cpu_dd);
	} else
		devm_free_irq(wdog_dd->dev, wdog_dd->bark_irq, wdog_dd);
	enable = 0;
	/*Ensure all cpus see update to enable*/
	smp_mb();
	atomic_notifier_chain_unregister(&panic_notifier_list,
						&wdog_dd->panic_blk);
	del_timer_sync(&wdog_dd->pet_timer);
	/* may be suspended after the first write above */
	__raw_writel(0, wdog_dd->base + WDT0_EN);
	/* Make sure watchdog is disabled before setting enable */
	mb();
	wdog_dd->enabled = false;
	pr_info("MSM Apps Watchdog deactivated.\n");
}

static ssize_t wdog_disable_get(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	int ret;
	struct msm_watchdog_data *wdog_dd = dev_get_drvdata(dev);

	mutex_lock(&wdog_dd->disable_lock);
	ret = snprintf(buf, PAGE_SIZE, "%d\n", enable == 0 ? 1 : 0);
	mutex_unlock(&wdog_dd->disable_lock);
	return ret;
}

static ssize_t wdog_disable_set(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t count)
{
	int ret;
	u8 disable;
	struct msm_watchdog_data *wdog_dd = dev_get_drvdata(dev);

	ret = kstrtou8(buf, 10, &disable);
	if (ret) {
		dev_err(wdog_dd->dev, "invalid user input\n");
		return ret;
	}
	if (disable == 1) {
		mutex_lock(&wdog_dd->disable_lock);
		if (enable == 0) {
			pr_info("MSM Apps Watchdog already disabled\n");
			mutex_unlock(&wdog_dd->disable_lock);
			return count;
		}
		disable = 1;
		if (!is_scm_armv8()) {
			ret = scm_call(SCM_SVC_BOOT, SCM_SVC_SEC_WDOG_DIS,
				       &disable, sizeof(disable), NULL, 0);
		} else {
			struct scm_desc desc = {0};

			desc.args[0] = 1;
			desc.arginfo = SCM_ARGS(1);
			ret = scm_call2(SCM_SIP_FNID(SCM_SVC_BOOT,
					SCM_SVC_SEC_WDOG_DIS), &desc);
		}
		if (ret) {
			dev_err(wdog_dd->dev,
					"Failed to deactivate secure wdog\n");
			mutex_unlock(&wdog_dd->disable_lock);
			return -EIO;
		}
		wdog_disable(wdog_dd);
		mutex_unlock(&wdog_dd->disable_lock);
	} else {
		pr_err("invalid operation, only disable = 1 supported\n");
		return -EINVAL;
	}
	return count;
}

static DEVICE_ATTR(disable, S_IWUSR | S_IRUSR, wdog_disable_get,
							wdog_disable_set);

/*
 * Userspace Watchdog Support:
 * Write 1 to the "user_pet_enabled" file to enable hw support for a
 * userspace watchdog.
 * Userspace is required to pet the watchdog by continuing to write 1
 * to this file in the expected interval.
 * Userspace may disable this requirement by writing 0 to this same
 * file.
 */
static void __wdog_user_pet(struct msm_watchdog_data *wdog_dd)
{
	wdog_dd->user_pet_complete = true;
	wake_up(&wdog_dd->pet_complete);
}

static ssize_t wdog_user_pet_enabled_get(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	int ret;
	struct msm_watchdog_data *wdog_dd = dev_get_drvdata(dev);

	ret = snprintf(buf, PAGE_SIZE, "%d\n",
			wdog_dd->user_pet_enabled);
	return ret;
}

static ssize_t wdog_user_pet_enabled_set(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t count)
{
	int ret;
	struct msm_watchdog_data *wdog_dd = dev_get_drvdata(dev);

	ret = strtobool(buf, &wdog_dd->user_pet_enabled);
	if (ret) {
		dev_err(wdog_dd->dev, "invalid user input\n");
		return ret;
	}

	__wdog_user_pet(wdog_dd);

	return count;
}

static DEVICE_ATTR(user_pet_enabled, S_IWUSR | S_IRUSR,
		wdog_user_pet_enabled_get, wdog_user_pet_enabled_set);

static ssize_t wdog_pet_time_get(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	int ret;
	struct msm_watchdog_data *wdog_dd = dev_get_drvdata(dev);

	ret = snprintf(buf, PAGE_SIZE, "%d\n", wdog_dd->pet_time);
	return ret;
}

static DEVICE_ATTR(pet_time, S_IRUSR, wdog_pet_time_get, NULL);

static void pet_watchdog(struct msm_watchdog_data *wdog_dd)
{
	int slack, i, count, prev_count = 0;
	unsigned long long time_ns;
	unsigned long long slack_ns;
	unsigned long long bark_time_ns = wdog_dd->bark_time * 1000000ULL;

	for (i = 0; i < 2; i++) {
		count = (__raw_readl(wdog_dd->base + WDT0_STS) >> 1) & 0xFFFFF;
		if (count != prev_count) {
			prev_count = count;
			i = 0;
		}
	}
	slack = ((wdog_dd->bark_time * WDT_HZ) / 1000) - count;
	if (slack < wdog_dd->min_slack_ticks)
		wdog_dd->min_slack_ticks = slack;
	__raw_writel(1, wdog_dd->base + WDT0_RST);
	time_ns = sched_clock();
	slack_ns = (wdog_dd->last_pet + bark_time_ns) - time_ns;
	if (slack_ns < wdog_dd->min_slack_ns)
		wdog_dd->min_slack_ns = slack_ns;
	wdog_dd->last_pet = time_ns;
}

static void keep_alive_response(void *info)
{
	int cpu = smp_processor_id();
	struct msm_watchdog_data *wdog_dd = (struct msm_watchdog_data *)info;

	cpumask_set_cpu(cpu, &wdog_dd->alive_mask);
	/* Make sure alive mask is cleared and set in order */
	smp_mb();
}

/*
 * If this function does not return, it implies one of the
 * other cpu's is not responsive.
 */
static void ping_other_cpus(struct msm_watchdog_data *wdog_dd)
{
	int cpu;
#ifdef VENDOR_EDIT
	cpumask_t mask;
	get_cpu_ping_mask(&mask);
#endif /*VENDOR_EDIT*/
	cpumask_clear(&wdog_dd->alive_mask);
	/* Make sure alive mask is cleared and set in order */
	smp_mb();

#ifdef VENDOR_EDIT
	for_each_cpu(cpu, &mask) {
#else
	for_each_cpu(cpu, cpu_online_mask) {
		if (!cpu_idle_pc_state[cpu] && !cpu_isolated(cpu))
#endif /*VENDOR_EDIT*/
			smp_call_function_single(cpu, keep_alive_response,
						 wdog_dd, 1);
	}
}

static void pet_task_wakeup(unsigned long data)
{
	struct msm_watchdog_data *wdog_dd =
		(struct msm_watchdog_data *)data;
	wdog_dd->timer_expired = true;
	wake_up(&wdog_dd->pet_complete);
}

static __ref int watchdog_kthread(void *arg)
{
	struct msm_watchdog_data *wdog_dd =
		(struct msm_watchdog_data *)arg;
	unsigned long delay_time = 0;
	struct sched_param param = {.sched_priority = MAX_RT_PRIO-1};

	sched_setscheduler(current, SCHED_FIFO, &param);
	while (!kthread_should_stop()) {
		while (wait_event_interruptible(
			wdog_dd->pet_complete,
			wdog_dd->timer_expired) != 0)
			;

		if (wdog_dd->do_ipi_ping)
			ping_other_cpus(wdog_dd);

		while (wait_event_interruptible(
			wdog_dd->pet_complete,
			wdog_dd->user_pet_complete) != 0)
			;

		wdog_dd->timer_expired = false;
		wdog_dd->user_pet_complete = !wdog_dd->user_pet_enabled;

		if (enable) {
			delay_time = msecs_to_jiffies(wdog_dd->pet_time);
			pet_watchdog(wdog_dd);
		}

#ifdef VENDOR_EDIT
		reset_recovery_tried();
#endif
		/* Check again before scheduling
		 * Could have been changed on other cpu
		 */
		mod_timer(&wdog_dd->pet_timer, jiffies + delay_time);
	}
	return 0;
}


#ifdef VENDOR_EDIT
#ifdef HANG_OPPO_ALL

#ifdef CONFIG_OPPO_DAILY_BUILD    // user version, not release
#define CHECK_BOOT_TIME 480
#else
#define CHECK_BOOT_TIME 240
#endif

struct task_struct *boot_complete_timer;
typedef void *fl_owner_t;

extern struct file *filp_open(const char *, int, umode_t);
extern int filp_close(struct file *, fl_owner_t id);
extern void log_boot(char *str);
extern void phx_monit(const char *monitor_command);
extern void kernel_restart(char *cmd);
extern int write_to_reserve1(struct pon_struct *pon_info, int fatal_error);
extern int need_recovery(struct pon_struct *pon_info);
extern int creds_change_dac(void);
extern int creds_change_id(void);
#ifndef O_RDONLY
#define O_RDONLY  00
#endif

int hang_oppo_main_on = 1;   //default on
int hang_oppo_recovery_method = RESTART_AND_RECOVERY;
static int hang_oppo_kernel_time = CHECK_BOOT_TIME;

static int __init HangOppoMainOn(char *str)
{
    get_option(&str,&hang_oppo_main_on);

    pr_info("hang_oppo_main_on %d\n", hang_oppo_main_on);

    return 1;
}
__setup("phx_rus_conf.main_on=", HangOppoMainOn);

static int __init HangOppoRecoveryMethod(char *str)
{
    get_option(&str,&hang_oppo_recovery_method);

    pr_info("hang_oppo_recovery_method %d\n", hang_oppo_recovery_method);

    return 1;
}
__setup("phx_rus_conf.recovery_method=", HangOppoRecoveryMethod);

static int __init HangOppoKernelTime(char *str)
{
    get_option(&str,&hang_oppo_kernel_time);

    pr_info("hang_oppo_kernel_time %d\n", hang_oppo_kernel_time);

    return 1;
}
__setup("phx_rus_conf.kernel_time=", HangOppoKernelTime);

int op_bootcompleted(void)
{
    struct file *opfile;
    ssize_t size;
    loff_t offsize;
    char data_info[6] = {'\0'};
    mm_segment_t old_fs;

    opfile = filp_open("/proc/opbootcomplete", O_RDONLY, 0444);
    if (IS_ERR(opfile)) {
        pr_err("open /proc/opbootcomplete error:\n");
        return -1;
    }

    offsize = 0;
    old_fs = get_fs();
    set_fs(KERNEL_DS);
    size = vfs_read(opfile,data_info,sizeof(data_info),&offsize);
    if (size < 0) {
        pr_err("data_info %s size %ld", data_info, size);
        set_fs(old_fs);
        return -1;
    }
    set_fs(old_fs);
    filp_close(opfile,NULL);

    if (strncmp(data_info, "true", 4) == 0) {
        return 1;
    }

    return 0;
}

int op_isLongTimeBoot(void)
{
    struct file *opfile;
    ssize_t size;
    loff_t offsize;
    char data_info[16] = {'\0'};
    mm_segment_t old_fs;

    opfile = filp_open("/proc/opbootfrom", O_RDONLY, 0444);
    if (IS_ERR(opfile)) {
        pr_err("open /proc/opbootfrom error:\n");
        return -1;
    }

    offsize = 0;
    old_fs = get_fs();
    set_fs(KERNEL_DS);
    size = vfs_read(opfile,data_info,sizeof(data_info),&offsize);
    if (size < 0) {
        pr_err("data_info %s size %ld", data_info, size);
        set_fs(old_fs);
        return -1;
    }
    set_fs(old_fs);
    filp_close(opfile,NULL);

    if (strncmp(data_info, "normal", 6) == 0) {
        return 0;
    }

    return 1;
}

int data_need_wipe(void)
{
    struct file *opfile;
    int rc;

    opfile = filp_open("/data/oppo/log/opporeserve/recovery_info", O_CREAT | O_RDWR, 0764);

    if (IS_ERR(opfile)) {
        pr_err("data_need_wipe open /data/oppo/log/opporeserve/recovery_info error: (%ld)\n", PTR_ERR(opfile));
        if((PTR_ERR(opfile) == (-EROFS)) || (PTR_ERR(opfile) == (-ENOENT)) || (PTR_ERR(opfile) == (-ENOSPC))) {
            return 1;
        } else {
            return 0;
        }
    }

    rc = vfs_fsync(opfile, 1);
    if (rc) {
        pr_err("sync returns %d\n", rc);
    }

    if(opfile) {
        filp_close(opfile,NULL);
    }

    return 0;
}

int set_fatal_err_to_recovery(void)
{
    int rc;
    ssize_t size;
    loff_t offsize = 0;
    struct file *opfile;
    mm_segment_t old_fs;
    char data_info[24] = {'\0'};

    creds_change_id();

    opfile = filp_open("/cache/recovery/command", O_CREAT | O_RDWR, 0764);

    if (IS_ERR(opfile)) {
        pr_err("open /cache/recovery/command error: (%ld)\n", PTR_ERR(opfile));
        return -1;
    }

    old_fs = get_fs();
    set_fs(KERNEL_DS);

    if(data_need_wipe())
    {
        strcpy(data_info, OPPO_FATAL_ERR_TO_WIPEDATA);
    } else {
        strcpy(data_info, OPPO_FATAL_ERR_TO_RECOVERY);
    }

    pr_info("data_info %s\n", data_info);

    size = vfs_write(opfile, data_info, sizeof(data_info), &offsize);

    if (size < 0) {
         pr_err("vfs_write data_info %s size %ld \n", data_info, size);
         set_fs(old_fs);
         return -1;
    }
    rc = vfs_fsync(opfile, 1);
    if (rc) {
        pr_err("sync returns %d\n", rc);
    }

    set_fs(old_fs);

    if(opfile) {
        filp_close(opfile,NULL);
    }

    return 1;
}

static int op_start_check_bootup(void)
{
    struct pon_struct pon_info;
    int hang_oppo_8_mins = 0;

#ifdef CONFIG_OPPO_DAILY_BUILD    // user version, not release
    hang_oppo_8_mins = 1;
    schedule_timeout_interruptible(hang_oppo_kernel_time * HZ);
#else
    if(op_isLongTimeBoot()) {
        hang_oppo_8_mins = 1;
        schedule_timeout_interruptible(hang_oppo_kernel_time * HZ);
    }
#endif

    pr_err("op_start_check_bootup:\n");

    if (0 == op_bootcompleted()) {
        struct timespec ts;
        struct rtc_time tm;
        char err_info[60] = {0};

        getnstimeofday(&ts);
        rtc_time_to_tm(ts.tv_sec, &tm);

        if(hang_oppo_8_mins) {
            sprintf(err_info, "SET_BOOTERROR@ERROR_HANG_OPPO_OVER_8_MINS@%d-%d-%d %d:%d:%d",
                tm.tm_year + 1900, tm.tm_mon + 1, tm.tm_mday, tm.tm_hour, tm.tm_min, tm.tm_sec);
            pr_err("SET_BOOTERROR@ERROR_HANG_OPPO_OVER_8_MINS@%d-%d-%d %d:%d:%d\n",
                tm.tm_year + 1900, tm.tm_mon + 1, tm.tm_mday, tm.tm_hour, tm.tm_min, tm.tm_sec);
        } else {
            sprintf(err_info, "SET_BOOTERROR@ERROR_HANG_OPPO_OVER_4_MINS@%d-%d-%d %d:%d:%d",
                tm.tm_year + 1900, tm.tm_mon + 1, tm.tm_mday, tm.tm_hour, tm.tm_min, tm.tm_sec);
            pr_err("SET_BOOTERROR@ERROR_HANG_OPPO_OVER_4_MINS@%d-%d-%d %d:%d:%d\n",
                tm.tm_year + 1900, tm.tm_mon + 1, tm.tm_mday, tm.tm_hour, tm.tm_min, tm.tm_sec);
        }

        log_boot(err_info);
        phx_monit(err_info);
        // wait to collect hang oppo log finished
        schedule_timeout_interruptible(20 * HZ);
        if(need_recovery(&pon_info)) {
            set_fatal_err_to_recovery();
            kernel_restart("recovery");
        } else {
            write_to_reserve1(&pon_info,FATAL_FLAG);
            if(hang_oppo_recovery_method) {
                kernel_restart("boot didn't complete in 480S!");
            }
        }
    } else {
        memset(&pon_info, 0, SIZE_OF_PON_STRUCT);
        write_to_reserve1(&pon_info,0);
    }

    return 0;

}

//start with watchdog, kick a 240/480s timer for bootcomplete
static int opmonitor_boot_kthread(void *dummy)
{
    set_user_nice(current, 0);
    pr_err("opmonitor_boot_kthread:\n");

    if(hang_oppo_kernel_time) {
        schedule_timeout_interruptible(hang_oppo_kernel_time * HZ);
    } else {
        schedule_timeout_interruptible(CHECK_BOOT_TIME * HZ);
    }
    op_start_check_bootup();
    return 0;
}
#endif
#endif /* VENDOR_EDIT */
static int wdog_cpu_pm_notify(struct notifier_block *self,
			      unsigned long action, void *v)
{
	int cpu;

	cpu = raw_smp_processor_id();

	switch (action) {
	case CPU_PM_ENTER:
		cpu_idle_pc_state[cpu] = 1;
		break;
	case CPU_PM_ENTER_FAILED:
	case CPU_PM_EXIT:
		cpu_idle_pc_state[cpu] = 0;
		break;
	}

	return NOTIFY_OK;
}

static struct notifier_block wdog_cpu_pm_nb = {
	.notifier_call = wdog_cpu_pm_notify,
};

static int msm_watchdog_remove(struct platform_device *pdev)
{
	struct msm_watchdog_data *wdog_dd =
			(struct msm_watchdog_data *)platform_get_drvdata(pdev);

	if (!ipi_en)
		cpu_pm_unregister_notifier(&wdog_cpu_pm_nb);

	mutex_lock(&wdog_dd->disable_lock);
	if (enable)
		wdog_disable(wdog_dd);

	mutex_unlock(&wdog_dd->disable_lock);
	device_remove_file(wdog_dd->dev, &dev_attr_disable);
	if (wdog_dd->irq_ppi)
		free_percpu(wdog_dd->wdog_cpu_dd);
	dev_info(wdog_dd->dev, "MSM Watchdog Exit - Deactivated\n");
	del_timer_sync(&wdog_dd->pet_timer);
	kthread_stop(wdog_dd->watchdog_task);
	kfree(wdog_dd);
	return 0;
}

void msm_trigger_wdog_bite(void)
{
	if (!wdog_data)
		return;
	pr_info("Causing a watchdog bite!");
	__raw_writel(1, wdog_data->base + WDT0_BITE_TIME);
	/* Mke sure bite time is written before we reset */
	mb();
	__raw_writel(1, wdog_data->base + WDT0_RST);
	/* Make sure we wait only after reset */
	mb();
	/* Delay to make sure bite occurs */
	mdelay(10000);
	pr_err("Wdog - STS: 0x%x, CTL: 0x%x, BARK TIME: 0x%x, BITE TIME: 0x%x",
		__raw_readl(wdog_data->base + WDT0_STS),
		__raw_readl(wdog_data->base + WDT0_EN),
		__raw_readl(wdog_data->base + WDT0_BARK_TIME),
		__raw_readl(wdog_data->base + WDT0_BITE_TIME));
}

static irqreturn_t wdog_bark_handler(int irq, void *dev_id)
{
	struct msm_watchdog_data *wdog_dd = (struct msm_watchdog_data *)dev_id;
	unsigned long nanosec_rem;
	unsigned long long t = sched_clock();

	nanosec_rem = do_div(t, 1000000000);
	dev_info(wdog_dd->dev, "Watchdog bark! Now = %lu.%06lu\n",
			(unsigned long) t, nanosec_rem / 1000);
#ifdef ODM_WT_EDIT
#ifdef WT_BOOT_REASON
	save_panic_key_log("Watchdog bark! Now = %lu.%06lu\n",
			(unsigned long) t, nanosec_rem / 1000);
#endif
#endif

	nanosec_rem = do_div(wdog_dd->last_pet, 1000000000);
	dev_info(wdog_dd->dev, "Watchdog last pet at %lu.%06lu\n",
			(unsigned long) wdog_dd->last_pet, nanosec_rem / 1000);
#ifdef ODM_WT_EDIT
#ifdef WT_BOOT_REASON
        save_panic_key_log("Watchdog last pet at %lu.%06lu\n",
                        (unsigned long) wdog_dd->last_pet, nanosec_rem / 1000);
#endif
#endif

        if (wdog_dd->do_ipi_ping)
                dump_cpu_alive_mask(wdog_dd);
#ifdef ODM_WT_EDIT
#ifdef WT_BOOT_REASON
        set_reset_magic(RESET_MAGIC_WDT_BARK);
#endif
#endif

	if (wdog_dd->do_ipi_ping) {
		dump_cpu_alive_mask(wdog_dd);
#ifdef VENDOR_EDIT
		dump_cpu_online_mask();
#endif
	}
#ifdef VENDOR_EDIT
	if (try_to_recover_pending(wdog_dd->watchdog_task)) {
		pet_watchdog(wdog_dd);
		return IRQ_HANDLED;
	}

	print_smp_call_cpu();
	dump_wdog_cpu(wdog_dd->watchdog_task);
#endif
#ifdef VENDOR_EDIT
	panic("Handle a watchdog bite! - Falling back to kernel panic!");
#else
	msm_trigger_wdog_bite();
	panic("Failed to cause a watchdog bite! - Falling back to kernel panic!");
#endif 
	return IRQ_HANDLED;
}

static irqreturn_t wdog_ppi_bark(int irq, void *dev_id)
{
	struct msm_watchdog_data *wdog_dd =
			*(struct msm_watchdog_data **)(dev_id);
	return wdog_bark_handler(irq, wdog_dd);
}

static void configure_bark_dump(struct msm_watchdog_data *wdog_dd)
{
	int ret;
	struct msm_dump_entry dump_entry;
	struct msm_dump_data *cpu_data;
	int cpu;
	void *cpu_buf;

	cpu_data = kzalloc(sizeof(struct msm_dump_data) *
			   num_present_cpus(), GFP_KERNEL);
	if (!cpu_data)
		goto out0;

	cpu_buf = kzalloc(MAX_CPU_CTX_SIZE * num_present_cpus(),
			  GFP_KERNEL);
	if (!cpu_buf)
		goto out1;

	for_each_cpu(cpu, cpu_present_mask) {
		cpu_data[cpu].addr = virt_to_phys(cpu_buf +
						cpu * MAX_CPU_CTX_SIZE);
		cpu_data[cpu].len = MAX_CPU_CTX_SIZE;
		snprintf(cpu_data[cpu].name, sizeof(cpu_data[cpu].name),
			"KCPU_CTX%d", cpu);
		dump_entry.id = MSM_DUMP_DATA_CPU_CTX + cpu;
		dump_entry.addr = virt_to_phys(&cpu_data[cpu]);
		ret = msm_dump_data_register(MSM_DUMP_TABLE_APPS,
					     &dump_entry);
		/*
		 * Don't free the buffers in case of error since
		 * registration may have succeeded for some cpus.
		 */
		if (ret)
			pr_err("cpu %d reg dump setup failed\n", cpu);
	}

	return;
out1:
	kfree(cpu_data);
out0:
	return;
}

static void register_scan_dump(struct msm_watchdog_data *wdog_dd)
{
	static void *dump_addr;
	int ret;
	struct msm_dump_entry dump_entry;
	struct msm_dump_data *dump_data;

	if (!wdog_dd->scandump_size)
		return;

	dump_data = kzalloc(sizeof(struct msm_dump_data), GFP_KERNEL);
	if (!dump_data)
		return;
	dump_addr = kzalloc(wdog_dd->scandump_size, GFP_KERNEL);
	if (!dump_addr)
		goto err0;

	dump_data->addr = virt_to_phys(dump_addr);
	dump_data->len = wdog_dd->scandump_size;
	strlcpy(dump_data->name, "KSCANDUMP", sizeof(dump_data->name));

	dump_entry.id = MSM_DUMP_DATA_SCANDUMP;
	dump_entry.addr = virt_to_phys(dump_data);
	ret = msm_dump_data_register(MSM_DUMP_TABLE_APPS, &dump_entry);
	if (ret) {
		pr_err("Registering scandump region failed\n");
		goto err1;
	}
	return;
err1:
	kfree(dump_addr);
err0:
	kfree(dump_data);
}

static void configure_scandump(struct msm_watchdog_data *wdog_dd)
{
	int ret;
	struct msm_dump_entry dump_entry;
	struct msm_dump_data *cpu_data;
	int cpu;
	static dma_addr_t dump_addr;
	static void *dump_vaddr;

	for_each_cpu(cpu, cpu_present_mask) {
		cpu_data = devm_kzalloc(wdog_dd->dev,
					sizeof(struct msm_dump_data),
					GFP_KERNEL);
		if (!cpu_data)
			continue;

		dump_vaddr = (void *) dma_alloc_coherent(wdog_dd->dev,
							 MAX_CPU_SCANDUMP_SIZE,
							 &dump_addr,
							 GFP_KERNEL);
		if (!dump_vaddr) {
			dev_err(wdog_dd->dev, "Couldn't get memory for dump\n");
			continue;
		}
		memset(dump_vaddr, 0x0, MAX_CPU_SCANDUMP_SIZE);

		cpu_data->addr = dump_addr;
		cpu_data->len = MAX_CPU_SCANDUMP_SIZE;
		snprintf(cpu_data->name, sizeof(cpu_data->name),
			"KSCANDUMP%d", cpu);
		dump_entry.id = MSM_DUMP_DATA_SCANDUMP_PER_CPU + cpu;
		dump_entry.addr = virt_to_phys(cpu_data);
		ret = msm_dump_data_register(MSM_DUMP_TABLE_APPS,
					     &dump_entry);
		if (ret) {
			dev_err(wdog_dd->dev, "Dump setup failed, id = %d\n",
				MSM_DUMP_DATA_SCANDUMP_PER_CPU + cpu);
			dma_free_coherent(wdog_dd->dev, MAX_CPU_SCANDUMP_SIZE,
					  dump_vaddr,
					  dump_addr);
			devm_kfree(wdog_dd->dev, cpu_data);
		}
	}

	register_scan_dump(wdog_dd);
}

static int init_watchdog_sysfs(struct msm_watchdog_data *wdog_dd)
{
	int error = 0;

	error |= device_create_file(wdog_dd->dev, &dev_attr_disable);

	if (of_property_read_bool(wdog_dd->dev->of_node,
					"qcom,userspace-watchdog")) {
		error |= device_create_file(wdog_dd->dev, &dev_attr_pet_time);
		error |= device_create_file(wdog_dd->dev,
					    &dev_attr_user_pet_enabled);
	}

	if (error)
		dev_err(wdog_dd->dev, "cannot create sysfs attribute\n");

	return error;
}

static void init_watchdog_data(struct msm_watchdog_data *wdog_dd)
{
	unsigned long delay_time;
	uint32_t val;
	u64 timeout;
	int ret;

	/*
	 * Disable the watchdog for cluster 1 so that cluster 0 watchdog will
	 * be mapped to the entire sub-system.
	 */
	if (wdog_dd->wdog_absent_base)
		__raw_writel(2, wdog_dd->wdog_absent_base + WDOG_ABSENT);

	if (wdog_dd->irq_ppi) {
		wdog_dd->wdog_cpu_dd = alloc_percpu(struct msm_watchdog_data *);
		if (!wdog_dd->wdog_cpu_dd) {
			dev_err(wdog_dd->dev, "fail to allocate cpu data\n");
			return;
		}
		*raw_cpu_ptr(wdog_dd->wdog_cpu_dd) = wdog_dd;
		ret = request_percpu_irq(wdog_dd->bark_irq, wdog_ppi_bark,
					"apps_wdog_bark",
					wdog_dd->wdog_cpu_dd);
		if (ret) {
			dev_err(wdog_dd->dev, "failed to request bark irq\n");
			free_percpu(wdog_dd->wdog_cpu_dd);
			return;
		}
	} else {
		ret = devm_request_irq(wdog_dd->dev, wdog_dd->bark_irq,
				wdog_bark_handler, IRQF_TRIGGER_RISING,
						"apps_wdog_bark", wdog_dd);
		if (ret) {
			dev_err(wdog_dd->dev, "failed to request bark irq\n");
			return;
		}
	}
	delay_time = msecs_to_jiffies(wdog_dd->pet_time);
	wdog_dd->min_slack_ticks = UINT_MAX;
	wdog_dd->min_slack_ns = ULLONG_MAX;
	configure_scandump(wdog_dd);
	configure_bark_dump(wdog_dd);
	timeout = (wdog_dd->bark_time * WDT_HZ)/1000;
	__raw_writel(timeout, wdog_dd->base + WDT0_BARK_TIME);
	__raw_writel(timeout + 3*WDT_HZ, wdog_dd->base + WDT0_BITE_TIME);

	wdog_dd->panic_blk.notifier_call = panic_wdog_handler;
	atomic_notifier_chain_register(&panic_notifier_list,
				       &wdog_dd->panic_blk);
	mutex_init(&wdog_dd->disable_lock);
	init_waitqueue_head(&wdog_dd->pet_complete);
	wdog_dd->timer_expired = false;
	wdog_dd->user_pet_complete = true;
	wdog_dd->user_pet_enabled = false;
	wake_up_process(wdog_dd->watchdog_task);
	init_timer(&wdog_dd->pet_timer);
	wdog_dd->pet_timer.data = (unsigned long)wdog_dd;
	wdog_dd->pet_timer.function = pet_task_wakeup;
	wdog_dd->pet_timer.expires = jiffies + delay_time;
	add_timer(&wdog_dd->pet_timer);

	val = BIT(EN);
	if (wdog_dd->wakeup_irq_enable)
		val |= BIT(UNMASKED_INT_EN);
	__raw_writel(val, wdog_dd->base + WDT0_EN);
	__raw_writel(1, wdog_dd->base + WDT0_RST);
	wdog_dd->last_pet = sched_clock();
	wdog_dd->enabled = true;

	init_watchdog_sysfs(wdog_dd);

	if (wdog_dd->irq_ppi)
		enable_percpu_irq(wdog_dd->bark_irq, 0);
	if (!ipi_en)
		cpu_pm_register_notifier(&wdog_cpu_pm_nb);
	dev_info(wdog_dd->dev, "MSM Watchdog Initialized\n");
}

static const struct of_device_id msm_wdog_match_table[] = {
	{ .compatible = "qcom,msm-watchdog" },
	{}
};

static void dump_pdata(struct msm_watchdog_data *pdata)
{
	dev_dbg(pdata->dev, "wdog bark_time %d", pdata->bark_time);
	dev_dbg(pdata->dev, "wdog pet_time %d", pdata->pet_time);
	dev_dbg(pdata->dev, "wdog perform ipi ping %d", pdata->do_ipi_ping);
	dev_dbg(pdata->dev, "wdog base address is 0x%lx\n", (unsigned long)
								pdata->base);
}

static int msm_wdog_dt_to_pdata(struct platform_device *pdev,
					struct msm_watchdog_data *pdata)
{
	struct device_node *node = pdev->dev.of_node;
	struct resource *res;
	int ret;

	res = platform_get_resource_byname(pdev, IORESOURCE_MEM, "wdt-base");
	if (!res)
		return -ENODEV;
	pdata->size = resource_size(res);
	pdata->phys_base = res->start;
	if (unlikely(!(devm_request_mem_region(&pdev->dev, pdata->phys_base,
					       pdata->size, "msm-watchdog")))) {

		dev_err(&pdev->dev, "%s cannot reserve watchdog region\n",
								__func__);
		return -ENXIO;
	}
	pdata->base  = devm_ioremap(&pdev->dev, pdata->phys_base,
							pdata->size);
	if (!pdata->base) {
		dev_err(&pdev->dev, "%s cannot map wdog register space\n",
				__func__);
		return -ENXIO;
	}

	res = platform_get_resource_byname(pdev, IORESOURCE_MEM,
					   "wdt-absent-base");
	if (res) {
		pdata->wdog_absent_base  = devm_ioremap(&pdev->dev, res->start,
							 resource_size(res));
		if (!pdata->wdog_absent_base) {
			dev_err(&pdev->dev,
				"cannot map wdog absent register space\n");
			return -ENXIO;
		}
	} else {
		dev_info(&pdev->dev, "wdog absent resource not present\n");
	}

	pdata->bark_irq = platform_get_irq(pdev, 0);
	pdata->bite_irq = platform_get_irq(pdev, 1);
	ret = of_property_read_u32(node, "qcom,bark-time", &pdata->bark_time);
	if (ret) {
		dev_err(&pdev->dev, "reading bark time failed\n");
		return -ENXIO;
	}
	ret = of_property_read_u32(node, "qcom,pet-time", &pdata->pet_time);
	if (ret) {
		dev_err(&pdev->dev, "reading pet time failed\n");
		return -ENXIO;
	}
	pdata->do_ipi_ping = of_property_read_bool(node, "qcom,ipi-ping");
	if (!pdata->bark_time) {
		dev_err(&pdev->dev, "%s watchdog bark time not setup\n",
								__func__);
		return -ENXIO;
	}
	if (!pdata->pet_time) {
		dev_err(&pdev->dev, "%s watchdog pet time not setup\n",
								__func__);
		return -ENXIO;
	}
	pdata->wakeup_irq_enable = of_property_read_bool(node,
							 "qcom,wakeup-enable");

	if (of_property_read_u32(node, "qcom,scandump-size",
				 &pdata->scandump_size))
		dev_info(&pdev->dev,
			 "No need to allocate memory for scandumps\n");

	pdata->irq_ppi = irq_is_percpu(pdata->bark_irq);
	dump_pdata(pdata);
	return 0;
}

static int msm_watchdog_probe(struct platform_device *pdev)
{
	int ret;
	struct msm_watchdog_data *wdog_dd;
	struct md_region md_entry;

	if (!pdev->dev.of_node || !enable)
		return -ENODEV;
	wdog_dd = kzalloc(sizeof(struct msm_watchdog_data), GFP_KERNEL);
	if (!wdog_dd)
		return -EIO;
	ret = msm_wdog_dt_to_pdata(pdev, wdog_dd);
	if (ret)
		goto err;

	wdog_data = wdog_dd;
	wdog_dd->dev = &pdev->dev;
	platform_set_drvdata(pdev, wdog_dd);
	cpumask_clear(&wdog_dd->alive_mask);
	wdog_dd->watchdog_task = kthread_create(watchdog_kthread, wdog_dd,
			"msm_watchdog");
	if (IS_ERR(wdog_dd->watchdog_task)) {
		ret = PTR_ERR(wdog_dd->watchdog_task);
		goto err;
	}
	init_watchdog_data(wdog_dd);

	/* Add wdog info to minidump table */
	strlcpy(md_entry.name, "KWDOGDATA", sizeof(md_entry.name));
	md_entry.virt_addr = (uintptr_t)wdog_dd;
	md_entry.phys_addr = virt_to_phys(wdog_dd);
	md_entry.size = sizeof(*wdog_dd);
	if (msm_minidump_add_region(&md_entry))
		pr_info("Failed to add Watchdog data in Minidump\n");
#ifdef VENDOR_EDIT
        ret = init_oppo_watchlog();
        if (ret < 0) {
                pr_info("Failed to init oppo watchlog");
        }
#endif

	return 0;
err:
	kzfree(wdog_dd);
	return ret;
}

static const struct dev_pm_ops msm_watchdog_dev_pm_ops = {
	.suspend_noirq = msm_watchdog_suspend,
	.resume_noirq = msm_watchdog_resume,
};

static struct platform_driver msm_watchdog_driver = {
	.probe = msm_watchdog_probe,
	.remove = msm_watchdog_remove,
	.driver = {
		.name = MODULE_NAME,
		.owner = THIS_MODULE,
		.pm = &msm_watchdog_dev_pm_ops,
		.of_match_table = msm_wdog_match_table,
	},
};

static int init_watchdog(void)
{
	return platform_driver_register(&msm_watchdog_driver);
}

pure_initcall(init_watchdog);
MODULE_DESCRIPTION("MSM Watchdog Driver");
MODULE_LICENSE("GPL v2");
