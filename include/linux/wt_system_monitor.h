/******************************************************************
 * ** Copyright (C), 2008-2016, OPPO Mobile Comm Corp., Ltd.
 * ** ODM_WT_EDIT
 * ** File: - wt_system_monitor.h
 * ** Description: Add for crash log saving machanism
 * ** Version: 1.0
 * ** Date : 2018/05/19
 * **
 * ** ------------------------------- Revision History: -------------------------------
 * **    <author>       <data>         <version >      <desc>
 * **  Wuyun.Zhao       2018/05/19     1.0             build this module
 * ****************************************************************/

#ifndef _WT_SYSTEM_MONITOR_H
#define _WT_SYSTEM_MONITOR_H

/*
#ifndef WT_FINAL_RELEASE
#define WT_SYSTEM_MONITOR
#endif
*/

#ifdef WT_SYSTEM_MONITOR
#define WT_BOOT_REASON
#define WT_BOOTLOADER_LOG
#include "wt_boot_reason.h"
#include "wt_bootloader_log_save.h"
#endif

#endif
