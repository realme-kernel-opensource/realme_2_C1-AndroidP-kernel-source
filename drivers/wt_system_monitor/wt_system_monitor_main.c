/******************************************************************
 * ** Copyright (C), 2008-2016, OPPO Mobile Comm Corp., Ltd.
 * ** ODM_WT_EDIT
 * ** File: - wt_system_monitor_main.c
 * ** Description: Add for crash log saving machanism
 * ** Version: 1.0
 * ** Date : 2018/05/19
 * **
 * ** ------------------------------- Revision History: -------------------------------
 * **    <author>       <data>         <version >      <desc>
 * **  Wuyun.Zhao       2018/05/19     1.0             build this module
 * ****************************************************************/

#include <linux/wt_system_monitor.h>

#ifdef WT_SYSTEM_MONITOR

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/wt_boot_reason.h>
#include <linux/wt_bootloader_log_save.h>

static int __init system_monitor_init(void)
{
	int ret;

	ret = wt_bootloader_log_init();
	if (ret) {
		printk(KERN_ERR"wt_bootloader_log_init error!\n");
		return -1;
	}

	ret = wt_boot_reason_init();
	if (ret) {
		printk(KERN_ERR"wt_boot_reason_init error!\n");
		return -1;
	}

	ret = wt_bootloader_log_handle();
	if (ret) {
		printk(KERN_ERR"wt_bootloader_log_handle error!\n");
		return -1;
	}

	return ret;
}

static void __exit system_monitor_exit(void)
{
	wt_bootloader_log_exit();
	wt_boot_reason_exit();
}

module_init(system_monitor_init);
module_exit(system_monitor_exit);
MODULE_LICENSE("GPL");

#endif
