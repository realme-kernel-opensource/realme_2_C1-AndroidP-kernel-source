/******************************************************************
 * ** Copyright (C), 2008-2016, OPPO Mobile Comm Corp., Ltd.
 * ** ODM_WT_EDIT
 * ** File: - wt_bootloader_log_save.h
 * ** Description: Add for crash log saving machanism
 * ** Version: 1.0
 * ** Date : 2018/05/19
 * **
 * ** ------------------------------- Revision History: -------------------------------
 * **    <author>       <data>         <version >      <desc>
 * **  Wuyun.Zhao       2018/05/19     1.0             build this module
 * ****************************************************************/

#ifndef _WT_BOOTLOADER_LOG_SAVE_H
#define _WT_BOOTLOADER_LOG_SAVE_H

#include <linux/types.h>

#define WT_BOOTLOADER_LOG_ADDR            0x93000000
#define WT_BOOTLOADER_LOG_SIZE            0x00100000
#define WT_BOOTLOADER_LOG_HALF_SIZE       0x00060000
#define WT_PANIC_KEY_LOG_ADDR             0x930C0000
#define WT_BOOTLOADER_LOG_MAGIC           0x474C4C42  /* B L L G */
#define WT_PANIC_KEY_LOG_MAGIC            0x474C4B50  /* P K L G */

struct wt_panic_key_log {
	uint32_t magic;
	uint32_t panic_key_log_size;
	uint8_t reserved[24];
}__attribute__ ((aligned(8)));

/* please must align in 8 bytes */
struct wt_logbuf_info {
	uint32_t magic;
	uint32_t bootloader_log_size;
	uint64_t bootloader_log_addr;
	uint64_t panic_key_log_addr;
	uint8_t  boot_reason_str[32];
	uint32_t boot_reason_copies;
	uint32_t bootloader_started;
	uint32_t kernel_started;
	uint32_t padding;
	uint64_t kernel_log_addr;
	uint32_t kernel_log_size;
	uint32_t checksum;
	uint8_t  reserved[40];
}__attribute__((aligned(8)));

extern struct wt_logbuf_info *logbuf_head;

int wt_bootloader_log_init(void);
int wt_bootloader_log_handle(void);
void wt_bootloader_log_exit(void);

#endif
