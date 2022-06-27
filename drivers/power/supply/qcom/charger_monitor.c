/*******************************************************************************
 *  Copyright (C), 2008-2018, OPPO Mobile Comm Corp., Ltd.
 *  ODM_WT_EDIT
 *  FILE: - charger_monitor.c
 *  Description : Add charger monitor function
 *  Version: 1.0
 *  Date : 2018/6/20
 *
 *  -------------------------Revision History:----------------------------------
 *   <author>	 <data> 	<version >			<desc>
 *  Bin2.Zhang	2018/6/20	1.0				Add charger monitor function
 ******************************************************************************/
#ifdef ODM_WT_EDIT
//#define DEBUG
#define pr_fmt(fmt) "CHG-CHK: " fmt

#include <linux/debugfs.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/iio/consumer.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/of_irq.h>
#include <linux/platform_device.h>
#include <linux/power_supply.h>
#include <linux/regmap.h>
#include <linux/regulator/driver.h>
#include <linux/regulator/machine.h>
#include <linux/regulator/of_regulator.h>
#include <linux/qpnp/qpnp-revid.h>
#include "smb5-reg.h"
#include "smb5-lib.h"
#include "storm-watch.h"
#include <linux/pmic-voter.h>

#include <linux/fs.h>
#include <linux/proc_fs.h>
#include <asm/uaccess.h>

#include <linux/notifier.h>
#include <linux/fb.h>
#include <linux/kthread.h>
#include <linux/reboot.h>

#include <linux/ktime.h>
#define MAX_CHARGER_TIMEOUT (10*3600) // 10H

#define USB_ICL_LCD_ON_ABOVE_350 (900000)
#define USB_ICL_LCD_ON  (1200000)
#define USB_ICL_LCD_OFF (2000000)

#define CHARGER_MONITOR_DELAY_MS (5000)

#define BAT_TEMP_TOO_HOT_PARAMS (530)
#define BAT_TEMP_TOO_COLD_PARAMS (-30)
#define BAT_TEMP_DISCONNECT_PARAMS (-190)
#define BAT_VOL_HIGH_PARAMS (4500)

#define CHG_VOL_HIGH_PARAMS (5800)
#define CHG_VOL_LOW_PARAMS (4300)

#define BAT_MIN_FV	(3600000)

enum notify_code {
    CHG_VOL_LOW                 = 0,
    CHG_VOL_HIGH                = 1,
    REVERSE_0                   = 2,
    BAT_TEMP_HIGH               = 3,
    BAT_TEMP_LOW                = 4,
    BAT_TEMP_DISCONNECT         = 5,
    BAT_VOL_HIGH                = 6,
    CHARGER_FULL                = 7,
    REVERSE_1                   = 8,
    CHARGER_OVERTIME            = 9,
    CHARGER_FULL_HIGHTEMP       = 10,
    CHARGER_FULL_LOWTEMP2       = 11,
    REVERSE_2                   = 14,
    CHARGER_UNKNOW              = 20,
    CHARGER_LOW_BATTERY         = 21,
    CHARGER_LOW_BATTERY2        = 22,
    CHARGER_LOW_BATTERY3        = 23,
    CHARGER_POC                 = 24,
    LOW_BATTERY                 = 26,
    MAX_NOTIFY_CODE             = 31,
};

struct smb_step_chg {
	int low_threshold;
	int high_threshold;
	int fcc;
	int fv;
	int rechr_v;
	int chg_fv;
	int above_chg_fv_cnt;
};

struct smb_chg_para {
	int init_first;

	int lcd_is_on;
	int lcd_on_chg_curr_proc;

	int batt_temp;
	int	batt_vol;
	//int	batt_curr;
	int	batt_soc;
	int	batt_status;
	int	batt_healthd;
	int	batt_healthd_save;
	int	batt_present;
	int batt_auth;

	int sum_cnt_warm;
	int sum_cnt_good;
	int sum_cnt_cool;
	int sum_cnt_good_down_fv;
	int sum_cnt_up_fv;
	int batt_fv_up_status;

	int need_do_icl;
	int	chg_vol;
	int chg_vol_low;
	//int	chg_curr;
	int	chg_present;
	int chg_present_save;
	unsigned long chg_times;
	long chg_time_out;

	int need_stop_usb_chg;
	int need_stop_bat_chg;
	int need_output_log;

	int chg_ranges_len;
	struct smb_step_chg *p_chg_ranges;
};

struct smb_charger *g_chg = NULL;

struct smb_step_chg chg_ranges[] = {
	{-30,  -1,  350000, 3983, 3700, 3993, 0},
	{  0,  49,  600000, 4373, 4150, 4383, 0},
	{ 50, 119, 1000000, 4373, 4270, 4383, 0},
	{120, 219, 1800000, 4373, 4270, 4383, 0},
	{220, 449, 2000000, 4373, 4270, 4383, 0},
	{450, 530, 1000000, 4083, 3980, 4093, 0},
};
struct smb_chg_para g_charger_para;

static bool charger_monitor_work_init = false;
static int charger_need_reset = 0;

extern int return_step_chg_jeita_fcc_low_threshold(void);

void reset_battery_float_voltage(struct smb_charger *chg);
void update_chg_monitor_statu(struct smb_charger *chg);

#define USBIN_AICL_EN_BIT	BIT(2)
#define USBIN_25MA			25000
#define USB_ICL_POINT_HIGH		(4550)
#define USB_ICL_POINT_LOW		(4500)
/* Add of 1200mA back, avoid can't charge with 1200mA when lcd was on. 0719*/
int usb_icl_step[] = {500, 900, 1200, 1500, 2000}; // Remove 1200mA, avoid 1000mA dcp charged with 1200mA current
/* Must be sure to call this funtion,it will cause charger current to be a low level !!! */
int do_charger_icl(struct smb_charger *chg)
{
	int usb_current_now = 0;
	int current_temp = 0;
	int usb_icl_step_lens = sizeof(usb_icl_step) / sizeof(int);
	int i = 0;
	int rc = 0;
	int usb_icl_point = 0;
	union power_supply_propval val = {0, };

	g_charger_para.need_do_icl = 0;
	if ((g_charger_para.chg_present != 1) || (chg->real_charger_type == POWER_SUPPLY_TYPE_USB) || (chg->real_charger_type == POWER_SUPPLY_TYPE_USB_CDP)
			|| (chg->real_charger_type == POWER_SUPPLY_TYPE_UNKNOWN) || (g_charger_para.batt_status == POWER_SUPPLY_STATUS_FULL)
			|| (get_client_vote(chg->usb_icl_votable, CHG_CHK_VOTER) == 0)) {
		pr_debug("USB don't need do charger ICL.\n");
		return 0;
	}

	power_supply_get_property(chg->batt_psy, POWER_SUPPLY_PROP_VOLTAGE_NOW, &val);
	if (val.intval / 1000 < 4100) {
		usb_icl_point = USB_ICL_POINT_LOW;
	} else {
		usb_icl_point = USB_ICL_POINT_HIGH;
	}

	/* clean last setting first */
	vote(chg->usb_icl_votable, CHG_CHK_VOTER, false, 0);
	//usb_current_now = get_effective_result(chg->usb_icl_votable);
	smblib_get_charge_param(chg, &chg->param.usb_icl, &usb_current_now);

	/* Disable AICL first */
	rc = smblib_masked_write(chg, USBIN_AICL_OPTIONS_CFG_REG, USBIN_AICL_EN_BIT, 0);
	if (rc < 0)
		pr_info("Clear USBIN_AICL_OPTIONS_CFG_REG error!");

	for (i = 1; i < usb_icl_step_lens; i++) {
		if (usb_current_now < usb_icl_step[i - 1] * 1000) {
			i--;
			break;
		}
		//vote(chg->usb_icl_votable, CHG_CHK_VOTER, true, usb_icl_step[i] * 1000);
		smblib_set_charge_param(chg, &chg->param.usb_icl, usb_icl_step[i] * 1000);
		msleep(90);

		if ((get_client_vote(chg->usb_icl_votable, BOOST_BACK_VOTER) == 0)
				&& (get_effective_result(chg->usb_icl_votable) < USBIN_25MA)) {
			i--;
			break;
		}

		power_supply_get_property(chg->usb_psy, POWER_SUPPLY_PROP_VOLTAGE_NOW, &val);
		if (val.intval / 1000 < usb_icl_point) {
			i--;
			break;
		}
	}
	current_temp = get_effective_result(chg->usb_icl_votable);
	smblib_set_charge_param(chg, &chg->param.usb_icl, current_temp);
	if (usb_icl_step_lens == i) {
		i = usb_icl_step_lens - 1;
	}
	if (usb_icl_step[i] == 1200) {
		i--;
	}
	g_charger_para.need_output_log = 1;
	vote(chg->usb_icl_votable, CHG_CHK_VOTER, true, usb_icl_step[i] * 1000);
	pr_info("USB do charger ICL %d->(Temp:%d)->%d,USB type:%d, Vbus:%d\n", usb_current_now / 1000, current_temp / 1000, usb_icl_step[i], chg->real_charger_type, val.intval);
	msleep(120);

	/* Enable AICL */
	rc = smblib_masked_write(chg, USBIN_AICL_OPTIONS_CFG_REG, USBIN_AICL_EN_BIT, USBIN_AICL_EN_BIT);
	if (rc < 0) {
		pr_info("Set USBIN_AICL_OPTIONS_CFG_REG error!");
	}

	return 0;
}

static bool full_not_chg_time_out_range(struct smb_charger *chg)
{
	static unsigned long not_chg_start_times = 0;
	unsigned long not_chg_end_times = 0;
	bool rc = true;

	if ((chg->prop_status == POWER_SUPPLY_STATUS_FULL) && (g_charger_para.chg_present == 1)
			&& (g_charger_para.batt_status == POWER_SUPPLY_STATUS_NOT_CHARGING)) {
		if (not_chg_start_times == 0) {
			get_rtc_time(&not_chg_start_times);
		}
		get_rtc_time(&not_chg_end_times);
	} else {
		not_chg_start_times = 0;
		not_chg_end_times = 0;
	}
	rc = (not_chg_end_times > not_chg_start_times + 1)? true : false;
	if ((!rc) && (not_chg_start_times != 0)) {
		pr_debug("Keep FUll & NOT CHARGING: %ld->%ld\n", not_chg_start_times, not_chg_end_times);
	}

	return rc;
}

int get_battery_status_modify(struct smb_charger *chg)
{
	int tmp_temp = return_step_chg_jeita_fcc_low_threshold();
	int batt_healthd = POWER_SUPPLY_HEALTH_GOOD;
	union power_supply_propval val = {0, };
	int rc = 0;

	if (charger_monitor_work_init == false) {
		smblib_get_prop_batt_status(chg, &val);
		return val.intval;
	}

	if (tmp_temp == g_charger_para.p_chg_ranges[0].low_threshold) { /* -30 ~ -1 */
		batt_healthd = POWER_SUPPLY_HEALTH_COOL;
	} else if (tmp_temp == g_charger_para.p_chg_ranges[5].low_threshold) { /* 450 ~ 530 */
		batt_healthd = POWER_SUPPLY_HEALTH_WARM;
	} else if ((tmp_temp > g_charger_para.p_chg_ranges[0].low_threshold)
		&& (tmp_temp < g_charger_para.p_chg_ranges[5].low_threshold)) { /* 0 ~ 449 */
		batt_healthd = POWER_SUPPLY_HEALTH_GOOD;
	}

	update_chg_monitor_statu(chg);
	full_not_chg_time_out_range(chg);

	/* Hold full after had already charged to full! Change to NOT_CHARGING when plug-out. */
	if ((chg->prop_status == POWER_SUPPLY_STATUS_FULL) && (batt_healthd == g_charger_para.batt_healthd_save)
				&& (g_charger_para.chg_present == 1)) {
		if (g_charger_para.batt_status != POWER_SUPPLY_STATUS_NOT_CHARGING) {
			/* Add recharger base on vol, Qcom auto recharger had disable. */
			if ((g_charger_para.batt_vol < chg->auto_recharge_mv) && (g_charger_para.batt_status == POWER_SUPPLY_STATUS_FULL)) {
				pr_info("Battery Recharger %dmV\n", chg->auto_recharge_mv);
				rc = smblib_masked_write(chg, CHARGING_ENABLE_CMD_REG, CHARGING_ENABLE_CMD_BIT, 0);
				if (rc < 0) {
					pr_info("Couldn't set CHARGING_ENABLE_CMD_REG 0 rc=%d\n", rc);
				}
				//msleep(50);
				rc = smblib_masked_write(chg, CHARGING_ENABLE_CMD_REG, CHARGING_ENABLE_CMD_BIT, CHARGING_ENABLE_CMD_BIT);
				if (rc < 0) {
					pr_info("Couldn't set CHARGING_ENABLE_CMD_REG 1 rc=%d\n", rc);
				}
			}
			return POWER_SUPPLY_STATUS_FULL;
		} else {
			if (!full_not_chg_time_out_range(chg)) {
				return POWER_SUPPLY_STATUS_FULL;
			}
		}
	}
	chg->prop_status = g_charger_para.batt_status;

	if ((g_charger_para.chg_present == 1) && (g_charger_para.batt_status == POWER_SUPPLY_STATUS_FULL)) {
		if (g_charger_para.batt_healthd == POWER_SUPPLY_HEALTH_GOOD) {
			if (chg->ui_soc >= 100) {
				chg->prop_status = POWER_SUPPLY_STATUS_FULL;
			} else {
				chg->prop_status = POWER_SUPPLY_STATUS_CHARGING;
			}
		} else {
			chg->prop_status = POWER_SUPPLY_STATUS_FULL;
		}
	}

	if (g_charger_para.batt_status == POWER_SUPPLY_STATUS_DISCHARGING) {
		chg->prop_status = POWER_SUPPLY_STATUS_NOT_CHARGING;
	}

	return chg->prop_status;
}

static bool soc_reduce_slow_when_1(struct smb_charger *chg)
{
	static int reduce_count = 0;
	int reduce_count_limit = 0;

	if (g_charger_para.chg_present == 1) {
		reduce_count_limit = 12;
	} else {
		reduce_count_limit = 4;
	}
	if (g_charger_para.batt_vol < 3410) {
		g_charger_para.need_output_log = 1;
		reduce_count++;
	} else {
		reduce_count = 0;
	}

	if (reduce_count > reduce_count_limit) {
		reduce_count = reduce_count_limit + 1;
		return true;
	} else {
		return false;
	}
}

#define SOC_SYNC_DOWN_RATE_300S (60) // 300/5
#define SOC_SYNC_DOWN_RATE_150S (30) // 150/5
#define SOC_SYNC_DOWN_RATE_90S  (18) //  90/5
#define SOC_SYNC_DOWN_RATE_60S  (12) //  60/5
#define SOC_SYNC_DOWN_RATE_40S  ( 8) //  40/5
#define SOC_SYNC_DOWN_RATE_15S  ( 3) //  15/5

#define SOC_SYNC_UP_RATE_60S  (12) //  60/5
#define SOC_SYNC_UP_RATE_10S  ( 2) //  10/5

#define TEN_MINUTES (10 * 60)

static int soc_down_count = 0;
static int soc_up_count = 0;
int update_battery_ui_soc(struct smb_charger *chg)
{
	int soc_down_limit = 0;
	int soc_up_limit = 0;
	unsigned long soc_reduce_margin = 0;
	bool vbatt_too_low = false;

	if ((chg->ui_soc < 0) || (chg->ui_soc > 100)) {
		pr_info("Don't need sync ui_soc:%d, msoc:%d.\n", chg->ui_soc, g_charger_para.batt_soc);
		chg->ui_soc = g_charger_para.batt_soc;
		return -1;
	}

	if (chg->ui_soc == 100) {
		soc_down_limit = SOC_SYNC_DOWN_RATE_300S;
	} else if (chg->ui_soc >= 95) {
		soc_down_limit = SOC_SYNC_DOWN_RATE_150S;
	} else if (chg->ui_soc >= 60) {
		soc_down_limit = SOC_SYNC_DOWN_RATE_60S;
	} else if ((g_charger_para.chg_present == 1) && (chg->ui_soc == 1)) {
		soc_down_limit = SOC_SYNC_DOWN_RATE_90S;
	} else {
		soc_down_limit = SOC_SYNC_DOWN_RATE_40S;
	}
	if ((g_charger_para.batt_vol < 3300) && (g_charger_para.batt_vol > 2500)) {
		soc_down_limit = SOC_SYNC_DOWN_RATE_15S;
		vbatt_too_low = true;
		pr_info("batt_volt:%d, vbatt_too_low:%d\n", g_charger_para.batt_vol, vbatt_too_low);
	}

	if (g_charger_para.batt_status == POWER_SUPPLY_STATUS_FULL) {
		soc_up_limit = SOC_SYNC_UP_RATE_60S;
	} else {
		soc_up_limit = SOC_SYNC_UP_RATE_10S;
	}

	if ((g_charger_para.chg_present == 1) && (g_charger_para.batt_status == POWER_SUPPLY_STATUS_FULL)) {
		chg->sleep_tm_sec = 0;
		if (g_charger_para.batt_healthd == POWER_SUPPLY_HEALTH_GOOD) {
			soc_down_count = 0;
			soc_up_count++;
			if (soc_up_count >= soc_up_limit) {
				soc_up_count = 0;
				chg->ui_soc++;
				if (chg->ui_soc <= 100) {
					g_charger_para.need_output_log = 1;
					pr_debug("full ui_soc:%d soc:%d up_limit:%d\n", chg->ui_soc, g_charger_para.batt_soc, soc_up_limit * 5);
				}
			}
			if (chg->ui_soc >= 100) {
				chg->ui_soc = 100;
				chg->prop_status = POWER_SUPPLY_STATUS_FULL;
			} else {
				g_charger_para.need_output_log = 1;
				pr_debug("full ui_soc:%d soc:%d up_limit:%d up_count:%d\n", chg->ui_soc, g_charger_para.batt_soc, soc_up_limit * 5, soc_up_count);
				chg->prop_status = POWER_SUPPLY_STATUS_CHARGING;
			}
		} else {
			chg->prop_status = POWER_SUPPLY_STATUS_FULL;
		}
	} else if ((g_charger_para.chg_present == 1) && (g_charger_para.batt_status == POWER_SUPPLY_STATUS_CHARGING)) {
		chg->sleep_tm_sec = 0;
		if (chg->prop_status != POWER_SUPPLY_STATUS_FULL) {
			chg->prop_status = POWER_SUPPLY_STATUS_CHARGING;
		}
		if (g_charger_para.batt_soc == chg->ui_soc) {
			soc_down_count = 0;
			soc_up_count = 0;
		} else if (g_charger_para.batt_soc > chg->ui_soc) {
			soc_down_count = 0;
			soc_up_count++;
			if (soc_up_count >= soc_up_limit) {
				soc_up_count = 0;
				chg->ui_soc++;
			}
		} else if (g_charger_para.batt_soc < chg->ui_soc) {
			soc_up_count = 0;
			soc_down_count++;
			if (soc_down_count >= soc_down_limit) {
				soc_down_count = 0;
				chg->ui_soc--;
			}
		}
		if (chg->ui_soc != g_charger_para.batt_soc) {
			g_charger_para.need_output_log = 1;
			pr_debug("charging ui_soc:%d soc:%d down_limit:%d down_count:%d up_limit:%d up_count:%d\n", chg->ui_soc, g_charger_para.batt_soc,
						soc_down_limit * 5, soc_down_count, soc_up_limit * 5, soc_up_count);
		}
	} else {
		if ((g_charger_para.chg_present == 1) && (g_charger_para.batt_status == POWER_SUPPLY_STATUS_NOT_CHARGING)
					&& (chg->prop_status == POWER_SUPPLY_STATUS_FULL) && (!full_not_chg_time_out_range(chg))) {
				g_charger_para.need_output_log = 1;
				chg->prop_status = POWER_SUPPLY_STATUS_FULL;
				return 1;
		} else {
			chg->prop_status = POWER_SUPPLY_STATUS_NOT_CHARGING;
		}
		soc_up_count = 0;
		if (g_charger_para.batt_soc <= chg->ui_soc || vbatt_too_low) {
			if (soc_down_count > soc_down_limit) {
				soc_down_count = soc_down_limit + 1;
			} else {
				soc_down_count++;
			}
			if (chg->sleep_tm_sec > 0) {
				soc_reduce_margin = chg->sleep_tm_sec / TEN_MINUTES;
				if (soc_reduce_margin == 0) {
					if ((chg->ui_soc - g_charger_para.batt_soc) > 2) {
						chg->ui_soc--;
						soc_down_count = 0;
						chg->sleep_tm_sec = 0;
					}
				} else if (soc_reduce_margin < (chg->ui_soc - g_charger_para.batt_soc)) {
					chg->ui_soc -= soc_reduce_margin;
					soc_down_count = 0;
					chg->sleep_tm_sec = 0;
				} else if (soc_reduce_margin >= (chg->ui_soc - g_charger_para.batt_soc)) {
					chg->ui_soc = g_charger_para.batt_soc;
					soc_down_count = 0;
					chg->sleep_tm_sec = 0;
				}
			}
			if (soc_down_count >= soc_down_limit && (g_charger_para.batt_soc < chg->ui_soc || vbatt_too_low)) {
				chg->sleep_tm_sec = 0;
				soc_down_count = 0;
				chg->ui_soc--;
			}
		}
		if (chg->ui_soc != g_charger_para.batt_soc) {
			g_charger_para.need_output_log = 1;
			pr_debug("discharging ui_soc:%d soc:%d down_limit:%d soc_down_count:%d sleep time:%ld\n", chg->ui_soc, g_charger_para.batt_soc,
						soc_down_limit * 5, soc_down_count, chg->sleep_tm_sec);
		}
	}

	if (chg->ui_soc < 2) {
		if (soc_reduce_slow_when_1(chg) == true) {
			chg->ui_soc = 0;
		} else {
			chg->ui_soc = 1;
		}
	}

	return 0;
}

int get_charger_timeout(struct smb_charger *chg)
{
	return (chg->notify_code & (1 << CHARGER_OVERTIME)) ? 1 : 0;
}

void reset_charge_modify_setting(struct smb_charger *chg, int chg_triggle)
{
	if (chg_triggle == 1) {
		chg->prop_status = POWER_SUPPLY_STATUS_NOT_CHARGING;
	}

	if (charger_monitor_work_init == false) {
		charger_need_reset = 1;
		return ;
	}

	g_charger_para.sum_cnt_warm = 0;
	g_charger_para.sum_cnt_good = 0;
	g_charger_para.sum_cnt_cool = 0;
	g_charger_para.sum_cnt_good_down_fv = 0;
	g_charger_para.sum_cnt_up_fv = 0;
	g_charger_para.batt_fv_up_status = 0;
	g_charger_para.chg_vol_low = 0;
	g_charger_para.need_output_log = 1;
	g_charger_para.need_do_icl = 0;
	reset_battery_float_voltage(chg);

	vote(chg->usb_icl_votable, CHG_CHK_VOTER, false, 0);
	//update_chg_monitor_statu(chg);
	pr_info("Resest charge modify setting.\n");
}

void update_chg_monitor_statu(struct smb_charger *chg)
{
	union power_supply_propval val = {0, };

	power_supply_get_property(chg->batt_psy, POWER_SUPPLY_PROP_TEMP, &val);
	g_charger_para.batt_temp = val.intval;
	power_supply_get_property(chg->batt_psy, POWER_SUPPLY_PROP_VOLTAGE_NOW, &val);
	g_charger_para.batt_vol = val.intval / 1000;
	//power_supply_get_property(chg->batt_psy, POWER_SUPPLY_PROP_CAPACITY, &val);
	smblib_get_prop_batt_capacity(chg, &val);
	g_charger_para.batt_soc = val.intval;
	//power_supply_get_property(chg->batt_psy, POWER_SUPPLY_PROP_STATUS, &val);
	smblib_get_prop_batt_status(chg, &val);
	g_charger_para.batt_status = val.intval;
	power_supply_get_property(chg->batt_psy, POWER_SUPPLY_PROP_PRESENT, &val);
	g_charger_para.batt_present = val.intval;
	power_supply_get_property(chg->batt_psy, POWER_SUPPLY_PROP_AUTHENTICATE, &val);
	g_charger_para.batt_auth = val.intval;

	power_supply_get_property(chg->usb_psy, POWER_SUPPLY_PROP_PRESENT, &val);
	g_charger_para.chg_present = val.intval;
	power_supply_get_property(chg->usb_psy, POWER_SUPPLY_PROP_VOLTAGE_NOW, &val);
	g_charger_para.chg_vol = val.intval / 1000;

	if (g_charger_para.chg_present == 1) {
		//g_charger_para.chg_times = (u32)ktime_get_seconds(); // Ktime will be error when system suspend
		if (g_charger_para.batt_status != POWER_SUPPLY_STATUS_CHARGING) {
			chg->charger_times = 0;
			g_charger_para.chg_times = 0;
		} else if (get_rtc_time(&g_charger_para.chg_times) < 0) {
			g_charger_para.chg_times = 0;
		} else if (chg->charger_times == 0) {
			chg->charger_times = g_charger_para.chg_times;
		}
		//pr_info("SMB charger notify from %ldS to %ldS.\n", chg->charger_times, g_charger_para.chg_times);
	} else {
		g_charger_para.chg_times = 0;
		chg->charger_times = 0;
	}
}

static void update_battery_healthd(struct smb_charger *chg)
{
	//int i = 0;
	int tmp_temp = return_step_chg_jeita_fcc_low_threshold();

	g_charger_para.batt_healthd_save = g_charger_para.batt_healthd;
	if (tmp_temp == g_charger_para.p_chg_ranges[0].low_threshold) { /* -30 ~ -1 */
		g_charger_para.batt_healthd = POWER_SUPPLY_HEALTH_COOL;
	} else if (tmp_temp == g_charger_para.p_chg_ranges[5].low_threshold) { /* 450 ~ 530 */
		g_charger_para.batt_healthd = POWER_SUPPLY_HEALTH_WARM;
	} else if ((tmp_temp > g_charger_para.p_chg_ranges[0].low_threshold)
		&& (tmp_temp < g_charger_para.p_chg_ranges[5].low_threshold)) { /* 0 ~ 449 */
		g_charger_para.batt_healthd = POWER_SUPPLY_HEALTH_GOOD;
	} else { /* other */
		if (g_charger_para.batt_temp < BAT_TEMP_DISCONNECT_PARAMS)
			g_charger_para.batt_healthd = POWER_SUPPLY_HEALTH_DEAD;
		else if (g_charger_para.batt_temp < g_charger_para.p_chg_ranges[0].low_threshold)
			g_charger_para.batt_healthd = POWER_SUPPLY_HEALTH_COLD;
		else if (g_charger_para.batt_temp < g_charger_para.p_chg_ranges[1].low_threshold)
			g_charger_para.batt_healthd = POWER_SUPPLY_HEALTH_COOL;
		else if (g_charger_para.batt_temp > g_charger_para.p_chg_ranges[5].high_threshold)
			g_charger_para.batt_healthd = POWER_SUPPLY_HEALTH_OVERHEAT;
		else if (g_charger_para.batt_temp > g_charger_para.p_chg_ranges[5].low_threshold)
			g_charger_para.batt_healthd = POWER_SUPPLY_HEALTH_WARM;
		else
			g_charger_para.batt_healthd = POWER_SUPPLY_HEALTH_GOOD;
	}
}

int get_chg_battery_healthd(void)
{
	return g_charger_para.batt_healthd;
}

void reset_battery_float_voltage(struct smb_charger *chg)
{
	#ifndef CONFIG_DISABLE_TEMP_PROTECT
	if (g_charger_para.batt_healthd == POWER_SUPPLY_HEALTH_GOOD) {
		vote(chg->fv_votable, BATT_PROFILE_VOTER, true, g_charger_para.p_chg_ranges[3].fv * 1000);
	} else if (g_charger_para.batt_healthd == POWER_SUPPLY_HEALTH_WARM) {
		vote(chg->fv_votable, BATT_PROFILE_VOTER, true, g_charger_para.p_chg_ranges[5].fv * 1000);
	} else if (g_charger_para.batt_healthd == POWER_SUPPLY_HEALTH_COOL) {
		vote(chg->fv_votable, BATT_PROFILE_VOTER, true, g_charger_para.p_chg_ranges[0].fv * 1000);
	} else {
		/* Something happen,set to good FV first */
		vote(chg->fv_votable, BATT_PROFILE_VOTER, true, g_charger_para.p_chg_ranges[3].fv * 1000);
	}
	#endif /* CONFIG_DISABLE_TEMP_PROTECT */
}

static void check_up_battery_float_voltage(struct smb_charger *chg)
{
	int set_fv = 0;

	if (g_charger_para.chg_present != 1) {
		/* Modify FV at charging only */
		return ;
	}

	if (g_charger_para.sum_cnt_good_down_fv != 0) {
		return;
	}
	if ((g_charger_para.chg_present == 1) && (g_charger_para.batt_fv_up_status == 0) && (g_charger_para.sum_cnt_good_down_fv == 0)
			&& (g_charger_para.batt_healthd == POWER_SUPPLY_HEALTH_GOOD) && (g_charger_para.batt_soc > 85)) {
		g_charger_para.need_output_log = 1;
		set_fv = get_client_vote(chg->fv_votable, BATT_PROFILE_VOTER);
		pr_info("Set Battery FV [%d->4353].\n", set_fv / 1000);
		if (set_fv > 4353000) {
			vote(chg->fv_votable, BATT_PROFILE_VOTER, true, 4353000);
			//g_charger_para.sum_cnt_good_down_fv += 1;
			g_charger_para.batt_fv_up_status = 1;
		}
		return;
	}

	set_fv = get_client_vote(chg->fv_votable, BATT_PROFILE_VOTER);
	if ((g_charger_para.batt_fv_up_status == 1) && (g_charger_para.batt_healthd == POWER_SUPPLY_HEALTH_GOOD)
			&& (g_charger_para.batt_vol < 4380) && (set_fv < 4373000)) {
		g_charger_para.need_output_log = 1;
		g_charger_para.sum_cnt_up_fv += 1;
		if (g_charger_para.sum_cnt_up_fv >= 3) {
			g_charger_para.sum_cnt_up_fv = 0;
			if (set_fv >= BAT_MIN_FV) {
				set_fv = (set_fv > 4363000) ? 4363000 : set_fv; //Max value can't out of 4.373V after add 0.01V
				vote(chg->fv_votable, BATT_PROFILE_VOTER, true, set_fv + 10000);
			}
			pr_info("Battery FV now:%dmV, + 10mV. Up good!\n", set_fv / 1000);
		}
	} else {
		g_charger_para.sum_cnt_up_fv = 0;
	}
}

void set_icl_flags(struct smb_charger *chg, int val)
{
	g_charger_para.need_do_icl = val;
}

void update_battery_float_voltage(struct smb_charger *chg)
{
	int set_fv = 0;

	if (g_charger_para.init_first == 1) {
		g_charger_para.need_output_log = 1;
		reset_battery_float_voltage(chg);
	}

	if (g_charger_para.batt_healthd_save != g_charger_para.batt_healthd) {
		g_charger_para.need_output_log = 1;
		reset_battery_float_voltage(chg);
		g_charger_para.sum_cnt_good_down_fv = 0;
	}

	if ((g_charger_para.chg_present != 1) || (g_charger_para.batt_status == POWER_SUPPLY_STATUS_FULL)) {
		/* Modify FV at charging only */
		return ;
	}

	if ((g_charger_para.batt_healthd_save != g_charger_para.batt_healthd) || (g_charger_para.need_do_icl != 0)) {
		do_charger_icl(chg);
	}

	if ((g_charger_para.batt_healthd == POWER_SUPPLY_HEALTH_GOOD)
			&& (g_charger_para.batt_vol > g_charger_para.p_chg_ranges[3].chg_fv)) {
		g_charger_para.need_output_log = 1;
		g_charger_para.sum_cnt_good += 1;
		if (g_charger_para.sum_cnt_good >= 3) {
			g_charger_para.sum_cnt_good = 0;
			set_fv = get_client_vote(chg->fv_votable, BATT_PROFILE_VOTER);
			pr_info("Battery FV now:%dmV, - 10mV. Down good!\n", set_fv / 1000);
			if (set_fv >= BAT_MIN_FV) {
				vote(chg->fv_votable, BATT_PROFILE_VOTER, true, set_fv - 10000);
				g_charger_para.sum_cnt_good_down_fv += 1;
			}
		}
	} else {
		g_charger_para.sum_cnt_good = 0;
	}

	if ((g_charger_para.batt_healthd == POWER_SUPPLY_HEALTH_WARM)
			&& (g_charger_para.batt_vol > g_charger_para.p_chg_ranges[5].chg_fv)) {
		g_charger_para.need_output_log = 1;
		g_charger_para.sum_cnt_warm += 1;
		if (g_charger_para.sum_cnt_warm >= 3) {
			g_charger_para.sum_cnt_warm = 0;
			set_fv = get_client_vote(chg->fv_votable, BATT_PROFILE_VOTER);
			pr_info("Battery FV now:%dmV, - 10mV. Down warm!\n", set_fv / 1000);
			if (set_fv >= BAT_MIN_FV) {
				vote(chg->fv_votable, BATT_PROFILE_VOTER, true, set_fv - 10000);
			}
		}
	} else {
		g_charger_para.sum_cnt_warm = 0;
	}

	if ((g_charger_para.batt_healthd == POWER_SUPPLY_HEALTH_COOL)
			&& (g_charger_para.batt_vol > g_charger_para.p_chg_ranges[0].chg_fv)) {
		g_charger_para.need_output_log = 1;
		g_charger_para.sum_cnt_cool += 1;
		if (g_charger_para.sum_cnt_cool >= 3) {
			g_charger_para.sum_cnt_cool = 0;
			set_fv = get_client_vote(chg->fv_votable, BATT_PROFILE_VOTER);
			pr_info("Battery FV now:%dmV, - 10mV. Down cool!\n", set_fv / 1000);
			if (set_fv >= BAT_MIN_FV) {
				vote(chg->fv_votable, BATT_PROFILE_VOTER, true, set_fv - 10000);
			}
		}
	} else {
		g_charger_para.sum_cnt_cool = 0;
	}
}

void update_charger_status(struct smb_charger *chg)
{
	int need_stop_usb_chg_tmp = 0;
	int need_stop_bat_chg_tmp = 0;
	static int chg_vol_low_out = 0;
	int chg_vol_low_rechr = 0;
	int rc = 0;
	union power_supply_propval val = {0, };

	if (g_charger_para.chg_present != 1) {
		return ;
	}

	if (g_charger_para.chg_vol > CHG_VOL_HIGH_PARAMS) {
		need_stop_usb_chg_tmp = 1;
		need_stop_bat_chg_tmp = 1;
	}
	/* Don't need stop charger when Vbus was below 4.3V */
	chg_vol_low_rechr = CHG_VOL_LOW_PARAMS > g_charger_para.batt_vol ? CHG_VOL_LOW_PARAMS : g_charger_para.batt_vol;
	if ((g_charger_para.chg_vol < CHG_VOL_LOW_PARAMS) && (g_charger_para.chg_vol_low != 1)) {
		msleep(10);
		power_supply_get_property(chg->usb_psy, POWER_SUPPLY_PROP_VOLTAGE_NOW, &val);
		if (val.intval / 1000 < CHG_VOL_LOW_PARAMS) {
			g_charger_para.chg_vol_low = 1;
			pr_info("VBUS was still low after read the seconds times.%d\n", val.intval);
		}
	} else if ((g_charger_para.chg_vol > chg_vol_low_rechr + 350) && (g_charger_para.chg_vol_low == 1)) {
		chg_vol_low_out += 1;
	} else {
		chg_vol_low_out = 0;
	}
	if (g_charger_para.chg_vol_low == 1) {
		g_charger_para.need_output_log = 1;
	}

	if ((chg->charger_times > 0) && (g_charger_para.chg_times >= chg->charger_times)
			&& (g_charger_para.chg_times - chg->charger_times >= g_charger_para.chg_time_out)
			&& (g_charger_para.chg_time_out >= 0)) {
		need_stop_usb_chg_tmp = 1;
		need_stop_bat_chg_tmp = 1;
	}
	if (g_charger_para.batt_vol > BAT_VOL_HIGH_PARAMS) {
		need_stop_bat_chg_tmp = 1;
	}
	if (g_charger_para.batt_temp > BAT_TEMP_TOO_HOT_PARAMS) {
		need_stop_bat_chg_tmp = 1;
	}
	if (g_charger_para.batt_temp < BAT_TEMP_TOO_COLD_PARAMS) {
		need_stop_bat_chg_tmp = 1;
	}

	if (g_charger_para.lcd_is_on == 1) {
		if (g_charger_para.batt_temp >= 350) {
		vote(chg->usb_icl_votable, FB_BLANK_VOTER, true, min(g_charger_para.lcd_on_chg_curr_proc, USB_ICL_LCD_ON_ABOVE_350));
		} else {
			vote(chg->usb_icl_votable, FB_BLANK_VOTER, true, min(g_charger_para.lcd_on_chg_curr_proc, USB_ICL_LCD_ON));
		}
	} else if (g_charger_para.lcd_is_on == 0) {
		vote(chg->usb_icl_votable, FB_BLANK_VOTER, false, 0);
	}

	if (need_stop_usb_chg_tmp != g_charger_para.need_stop_usb_chg) {
		g_charger_para.need_output_log = 1;
		g_charger_para.need_stop_usb_chg = need_stop_usb_chg_tmp;
		if (g_charger_para.need_stop_usb_chg == 1) {
			//vote(chg->usb_icl_votable, CHG_CHK_VOTER, true, 0);
			smblib_set_usb_suspend(chg, true);
		} else {
			//vote(chg->usb_icl_votable, CHG_CHK_VOTER, false, 0);
			smblib_set_usb_suspend(chg, false);
		}
	}
	if (need_stop_bat_chg_tmp != g_charger_para.need_stop_bat_chg) {
		g_charger_para.need_output_log = 1;
		g_charger_para.need_stop_bat_chg = need_stop_bat_chg_tmp;
		if (g_charger_para.need_stop_bat_chg == 1) {
			vote(chg->chg_disable_votable, CHG_CHK_VOTER, true, 0);
		} else {
			vote(chg->chg_disable_votable, CHG_CHK_VOTER, false, 0);
		}
	}

	if (/*need_stop_bat_chg_tmp || */need_stop_usb_chg_tmp) {
		return ;
	}

	/*
	if ((g_charger_para.chg_present == 1) && (g_charger_para.chg_present_save != g_charger_para.chg_present)) {
		do_charger_icl(chg);
	}
	*/

	/* Need do charging in FULL sometime, case about full status changed! */
	if ((g_charger_para.chg_vol_low == 1) && (chg_vol_low_out >= 2)/* && (chg->prop_status != POWER_SUPPLY_STATUS_FULL)*/) {
		g_charger_para.need_output_log = 1;
		g_charger_para.chg_vol_low = 0;
		pr_info("Reset from 4.3V VBUS\n");
		vote(chg->usb_icl_votable, WEAK_CHARGER_VOTER, false, 0);

		rc = smblib_masked_write(chg, USBIN_CMD_IL_REG, USBIN_SUSPEND_BIT, 1);
		if (rc < 0) {
			pr_info("Couldn't set USBIN_SUSPEND_BIT 1 rc=%d\n", rc);
		}
		//msleep(50);
		rc = smblib_masked_write(chg, USBIN_CMD_IL_REG, USBIN_SUSPEND_BIT, 0);
		if (rc < 0) {
			pr_info("Couldn't set USBIN_SUSPEND_BIT 0 rc=%d\n", rc);
		}
		msleep(5);
		//do_charger_icl(chg);
		rc = smblib_masked_write(chg, USBIN_AICL_OPTIONS_CFG_REG, USBIN_AICL_EN_BIT, 0);
		if (rc < 0) {
			pr_info("Couldn't set USBIN_AICL_OPTIONS_CFG_REG 0 rc=%d\n", rc);
		}
		msleep(5);
		rc = smblib_masked_write(chg, USBIN_AICL_OPTIONS_CFG_REG, USBIN_AICL_EN_BIT, USBIN_AICL_EN_BIT);
		if (rc < 0) {
			pr_info("Couldn't set USBIN_AICL_OPTIONS_CFG_REG 1 rc=%d\n", rc);
		}
		msleep(5);
		rc = smblib_masked_write(chg, AICL_CMD_REG, BIT(1), BIT(1));
		if (rc < 0) {
			pr_info("Couldn't set AICL_CMD_REG 1 rc=%d\n", rc);
		}
	}
}

void update_charger_notify_code(struct smb_charger *chg)
{
	int tmp_notify_code = 0;

	if (g_charger_para.chg_present != 1) {
		chg->notify_code = 0;
		return ;
	}

	if (g_charger_para.chg_vol > CHG_VOL_HIGH_PARAMS) {
		tmp_notify_code |= (1 << CHG_VOL_HIGH);
	} else {
		tmp_notify_code &= ~(1 << CHG_VOL_HIGH);
	}
	if /*(g_charger_para.batt_healthd == POWER_SUPPLY_HEALTH_OVERHEAT)*/(g_charger_para.batt_temp > BAT_TEMP_TOO_HOT_PARAMS) {
		tmp_notify_code |= (1 << BAT_TEMP_HIGH);
	} else {
		tmp_notify_code &= ~(1 << BAT_TEMP_HIGH);
	}
	if ((g_charger_para.batt_temp >= BAT_TEMP_DISCONNECT_PARAMS) && (g_charger_para.batt_temp < BAT_TEMP_TOO_COLD_PARAMS)) {
		/*(g_charger_para.batt_healthd == POWER_SUPPLY_HEALTH_COLD)*/
		tmp_notify_code |= (1 << BAT_TEMP_LOW);
	} else {
		tmp_notify_code &= ~(1 << BAT_TEMP_LOW);
	}
	if ((g_charger_para.batt_temp < BAT_TEMP_DISCONNECT_PARAMS) || (g_charger_para.batt_auth != 1))/*(batt_present != 1)*/ {
		tmp_notify_code |= (1 << BAT_TEMP_DISCONNECT);
	} else {
		tmp_notify_code &= ~(1 << BAT_TEMP_DISCONNECT);
	}
	if (g_charger_para.batt_vol > BAT_VOL_HIGH_PARAMS) {
		tmp_notify_code |= (1 << BAT_VOL_HIGH);
	} else {
		tmp_notify_code &= ~(1 << BAT_VOL_HIGH);
	}
	if ((g_charger_para.batt_healthd  == POWER_SUPPLY_HEALTH_GOOD) && (/*g_charger_para.batt_status*/chg->prop_status == POWER_SUPPLY_STATUS_FULL)) {
		tmp_notify_code |= (1 << CHARGER_FULL);
	} else {
		tmp_notify_code &= ~(1 << CHARGER_FULL);
	}
	if ((chg->charger_times > 0) && (g_charger_para.chg_times >= chg->charger_times)
			&& (g_charger_para.chg_times - chg->charger_times >= g_charger_para.chg_time_out)
			&& (g_charger_para.chg_time_out >= 0) && (g_charger_para.chg_present == 1)) {
		tmp_notify_code |= (1 << CHARGER_OVERTIME);
	} else {
		tmp_notify_code &= ~(1 << CHARGER_OVERTIME);
	}
	if ((g_charger_para.batt_healthd == POWER_SUPPLY_HEALTH_WARM) && (/*g_charger_para.batt_status*/chg->prop_status == POWER_SUPPLY_STATUS_FULL)
			&& (g_charger_para.chg_present == 1)) {
		tmp_notify_code |= (1 << CHARGER_FULL_HIGHTEMP);
	} else {
		tmp_notify_code &= ~(1 << CHARGER_FULL_HIGHTEMP);
	}
	if ((g_charger_para.batt_healthd == POWER_SUPPLY_HEALTH_COOL) && (/*g_charger_para.batt_status*/chg->prop_status == POWER_SUPPLY_STATUS_FULL)
			&& (g_charger_para.chg_present == 1)) {
		tmp_notify_code |= (1 << CHARGER_FULL_LOWTEMP2);
	} else {
		tmp_notify_code &= ~(1 << CHARGER_FULL_LOWTEMP2);
	}

	if (chg->notify_code != tmp_notify_code) {
		pr_info("SMB charger notify code from 0x%x to 0x%x,charger time %ldS (%ldS %ldS).\n", chg->notify_code, tmp_notify_code,
					(g_charger_para.chg_times ? (g_charger_para.chg_times - chg->charger_times) : 0), g_charger_para.chg_times, chg->charger_times);
		g_charger_para.need_output_log = 1;
		chg->notify_code = tmp_notify_code;
		//power_supply_changed(chg->batt_psy);
	}
	//pr_info("SMB charger notify code 0x%x with charge time %ldS.\n", chg->notify_code, chg_times - chg->charger_times);
}

void update_charger_output_log(struct smb_charger *chg)
{
	unsigned long c_time =
		(((g_charger_para.chg_times > 0) && (g_charger_para.chg_times >= chg->charger_times)) ? (g_charger_para.chg_times - chg->charger_times) : 0);

	if ((chg->prop_status != g_charger_para.batt_status) && (g_charger_para.batt_status != POWER_SUPPLY_STATUS_DISCHARGING)) {
		g_charger_para.need_output_log = 1;
	}

	if (g_charger_para.need_output_log == 0) {
		return ;
	}

	pr_info("Battery:[UI%d/%d/%dmV/T%d/S O(%d)->N(%d)/H O(%d)->N(%d)]; Vbus:[P%d/T%d/%dmV/%ldS]; Notify:[0x%x]; Sleep:[%ld].\n",
				chg->ui_soc, g_charger_para.batt_soc, g_charger_para.batt_vol, g_charger_para.batt_temp,
				g_charger_para.batt_status, chg->prop_status, g_charger_para.batt_healthd_save, g_charger_para.batt_healthd,
				g_charger_para.chg_present, chg->real_charger_type, g_charger_para.chg_vol, c_time, chg->notify_code, chg->sleep_tm_sec);

	return ;
}

static void smblib_charger_monitor_work(struct work_struct *work)
{
	struct smb_charger *chg = container_of(work, struct smb_charger,
						charger_monitor_work.work);

	charger_monitor_work_init = true;
	if (charger_need_reset == 1) {
		charger_need_reset = 0;
		reset_charge_modify_setting(chg, 0);
	}

	update_chg_monitor_statu(chg);

	update_battery_healthd(chg);
	update_battery_ui_soc(chg);
	update_battery_float_voltage(chg);
	check_up_battery_float_voltage(chg);

	update_charger_status(chg);
	update_charger_notify_code(chg);

	update_charger_output_log(chg);

	g_charger_para.init_first = 0;
	g_charger_para.chg_present_save = g_charger_para.chg_present;

	power_supply_changed(chg->batt_psy);
	schedule_delayed_work(&chg->charger_monitor_work, msecs_to_jiffies(CHARGER_MONITOR_DELAY_MS));

	g_charger_para.need_output_log = 0;
}

static int fb_notifier_callback(struct notifier_block *self, unsigned long event, void *data)
{
	struct fb_event *evdata = data;
	int *blank;
	struct smb_charger *chg= container_of(self, struct smb_charger, fb_notif);

	if (evdata && evdata->data && event == FB_EVENT_BLANK ) {
		blank = evdata->data;
		switch (*blank) {
		case FB_BLANK_UNBLANK:
			g_charger_para.lcd_is_on = 1;
			set_icl_flags(chg, 2);
			break;
		case FB_BLANK_POWERDOWN:
		case FB_BLANK_HSYNC_SUSPEND:
		case FB_BLANK_VSYNC_SUSPEND:
		case FB_BLANK_NORMAL:
                        vote(chg->usb_icl_votable, FB_BLANK_VOTER, true, USB_ICL_LCD_OFF);
			g_charger_para.lcd_is_on = 0;
			set_icl_flags(chg, 2);
			break;
		default :
			pr_info("Something error when run lcd set usb icl!\n");
			break;
		}
	}

    return 0;
}

static ssize_t charger_cycle_write(struct file *filp, const char __user *buff, size_t len, loff_t *data)
{
	char buf[8] = {0};

	if (copy_from_user(buf, buff, (len > 8) ? 8 : len)) {
		pr_err("proc write error.\n");
		return -EFAULT;
	}
	if (!strncmp(buf, "en808", strlen("en808"))) {
		vote(g_chg->usb_icl_votable, USER_VOTER, false, 0);
	}
	if (!strncmp(buf, "dis808", strlen("dis808"))) {
		vote(g_chg->usb_icl_votable, USER_VOTER, true, 0);
	}

	return len;
}
#if 0
static ssize_t charger_cycle_read(struct file *filp, char __user *buff, size_t len, loff_t *data)
{
	char value[2] = {0};
	snprintf(value, sizeof(value), "%d", flash_mode);
	return simple_read_from_buffer(buff, len, data, value,1);
}
#endif
static const struct file_operations charger_cycle_fops = {
	.owner		= THIS_MODULE,
	//.read		= NULL,
	.write		= charger_cycle_write,
};

static ssize_t charging_limit_current_write(struct file *filp, const char __user *buff, size_t len, loff_t *data)
{
	char buf[8] = {0};
	int tmp = 0;

	if (copy_from_user(buf, buff, (len > 8) ? 8 : len)) {
		pr_err("proc write error.\n");
		return -EFAULT;
	}
	tmp = simple_strtoul(buf, NULL, 10);
	g_charger_para.lcd_on_chg_curr_proc = tmp > 1200 ? 1200000 : tmp * 1000;

	if ((g_charger_para.lcd_is_on == 1) && (g_charger_para.batt_temp >= 350)) {
		vote(g_chg->usb_icl_votable, FB_BLANK_VOTER, true, min(g_charger_para.lcd_on_chg_curr_proc, USB_ICL_LCD_ON_ABOVE_350));
	} else if (g_charger_para.lcd_is_on == 1) {
		vote(g_chg->usb_icl_votable, FB_BLANK_VOTER, true, min(g_charger_para.lcd_on_chg_curr_proc, USB_ICL_LCD_ON));
	}

	return len;
}
static ssize_t charging_limit_current_read(struct file *filp, char __user *buff, size_t len, loff_t *data)
{
	char value[8] = {0};
	int str_len = 0;

	str_len = snprintf(value, sizeof(value), "%d", g_charger_para.lcd_on_chg_curr_proc);

	return simple_read_from_buffer(buff, len, data, value, str_len);
}

static const struct file_operations charging_limit_current_fops = {
	.owner		= THIS_MODULE,
	.read		= charging_limit_current_read,
	.write		= charging_limit_current_write,
};

static ssize_t charging_limit_time_write(struct file *filp, const char __user *buff, size_t len, loff_t *data)
{
	char buf[16] = {0};

	if (copy_from_user(buf, buff, (len > 16) ? 16 : len)) {
		pr_err("proc write error.\n");
		return -EFAULT;
	}
	g_charger_para.chg_time_out = simple_strtol(buf, NULL, 10);
	pr_info("Modify charger time out to %ldS.\n", g_charger_para.chg_time_out);

	return len;
}
static ssize_t charging_limit_time_read(struct file *filp, char __user *buff, size_t len, loff_t *data)
{
	char value[8] = {0};
	int str_len = 0;

	str_len = snprintf(value, sizeof(value), "%ld", g_charger_para.chg_time_out);

	return simple_read_from_buffer(buff, len, data, value, str_len);
}

static const struct file_operations charging_limit_time_fops = {
	.owner		= THIS_MODULE,
	.read		= charging_limit_time_read,
	.write		= charging_limit_time_write,
};

void wake_chg_monitor_work(struct smb_charger *chg, int ms)
{
	if (charger_monitor_work_init == false) {
		return ;
	}
	g_charger_para.need_output_log = 1;
	ms = ms > 0 ? ms : 0;
	cancel_delayed_work(&chg->charger_monitor_work);
	schedule_delayed_work(&chg->charger_monitor_work, msecs_to_jiffies(ms));
}

int tbatt_pwroff_enable = 1;
#define OPPO_TBATT_HIGH_PWROFF_COUNT            (18)
#define OPPO_TBATT_EMERGENCY_PWROFF_COUNT        (6)
#define OPCHG_PWROFF_HIGH_BATT_TEMP		   770
#define OPCHG_PWROFF_EMERGENCY_BATT_TEMP      850
DECLARE_WAIT_QUEUE_HEAD(oppo_tbatt_pwroff_wq);
void oppo_tbatt_power_off_task_wakeup(void);

static ssize_t  proc_tbatt_pwroff_write(struct file *filp, const char __user *buf, size_t len, loff_t *data)
{
	char buffer[2] = {0};

	if (len > 2) {
		return -EFAULT;
	}

	if (copy_from_user(buffer, buf, 2)) {
		printk("%s:  error.\n", __func__);
		return -EFAULT;
	}

	if (buffer[0] == '0') {
		tbatt_pwroff_enable = 0;
	} else {
		tbatt_pwroff_enable = 1;
		oppo_tbatt_power_off_task_wakeup();
	}
	printk("%s:tbatt_pwroff_enable = %d.\n", __func__, tbatt_pwroff_enable);

	return len;
}

static ssize_t proc_tbatt_pwroff_read(struct file *filp, char __user *buff, size_t count, loff_t *off)
{
	char page[256] = {0};
	char read_data[3] = {0};
	int len = 0;

	if (tbatt_pwroff_enable == 1) {
		read_data[0] = '1';
	} else {
	read_data[0] = '0';
	}
	read_data[1] = '\0';
	len = sprintf(page, "%s", read_data);
	if (len > *off) {
		len -= *off;
	} else {
		len = 0;
	}

	if (copy_to_user(buff, page, (len < count ? len : count))) {
		return -EFAULT;
	}
	*off += len < count ? len : count;

	return (len < count ? len : count);
}

static const struct file_operations tbatt_pwroff_proc_fops = {
	.write = proc_tbatt_pwroff_write,
	.read = proc_tbatt_pwroff_read,
};

static int init_proc_tbatt_pwroff(void)
{
	struct proc_dir_entry *p = NULL;

	p = proc_create("tbatt_pwroff", 0664, NULL, &tbatt_pwroff_proc_fops);
	if (!p) {
		printk("proc_create  fail!\n");
	}
	return 0;
}

static int oppo_tbatt_power_off_kthread(void *arg)
{
	int over_temp_count = 0, emergency_count = 0;
	int batt_temp = 0;
	//struct oppo_chg_chip *chip = (struct oppo_chg_chip *)arg;
	struct sched_param param = {.sched_priority = MAX_RT_PRIO-1};
	union power_supply_propval val = {0, };

	power_supply_get_property(g_chg->batt_psy, POWER_SUPPLY_PROP_TEMP, &val);
	g_charger_para.batt_temp = val.intval;


	sched_setscheduler(current, SCHED_FIFO, &param);
	tbatt_pwroff_enable = 1;

	while (!kthread_should_stop()) {
		schedule_timeout_interruptible(round_jiffies_relative(msecs_to_jiffies(5*1000)));
		//printk(" tbatt_pwroff_enable:%d over_temp_count[%d] start\n", tbatt_pwroff_enable, over_temp_count);
		if (!tbatt_pwroff_enable) {
			emergency_count = 0;
			over_temp_count = 0;
			wait_event_interruptible(oppo_tbatt_pwroff_wq, tbatt_pwroff_enable == 1);
		}

		power_supply_get_property(g_chg->batt_psy, POWER_SUPPLY_PROP_TEMP, &val);
		batt_temp = val.intval;

		if (batt_temp > OPCHG_PWROFF_EMERGENCY_BATT_TEMP) {
			emergency_count++;
			printk(" emergency_count:%d \n", emergency_count);
		} else {
			emergency_count = 0;
		}
		if (batt_temp > OPCHG_PWROFF_HIGH_BATT_TEMP) {
			over_temp_count++;
			printk("over_temp_count[%d] \n", over_temp_count);
		} else {
			over_temp_count = 0;
		}
		//printk("over_temp_count[%d], chip->temperature[%d]\n", over_temp_count, batt_temp);

		if (over_temp_count >= OPPO_TBATT_HIGH_PWROFF_COUNT
				|| emergency_count >= OPPO_TBATT_EMERGENCY_PWROFF_COUNT) {
			printk("ERROR: battery temperature is too high, goto power off\n");
			///msleep(1000);
			machine_power_off();
		}
	}
	return 0;
}

int oppo_tbatt_power_off_task_init(struct smb_charger *chg)
{
	if (!chg) {
		return -1;
	}

	chg->tbatt_pwroff_task = kthread_create(oppo_tbatt_power_off_kthread, chg, "tbatt_pwroff");
	if (chg->tbatt_pwroff_task) {
	    wake_up_process(chg->tbatt_pwroff_task);
	} else {
		printk("ERROR: chg->tbatt_pwroff_task creat fail\n");
		return -1;
	}

	return 0;
}

void oppo_tbatt_power_off_task_wakeup(void)
{
	wake_up_interruptible(&oppo_tbatt_pwroff_wq);
	return;
}



int init_chg_monitor_work(struct smb_charger *chg)
{
	struct proc_dir_entry *proc_entry_charger_cycle;
	struct proc_dir_entry *proc_entry_charging_limit_current;
	struct proc_dir_entry *proc_entry_charging_limit_time;

	g_chg = chg;

	memset(&g_charger_para, 0, sizeof(g_charger_para));
	g_charger_para.init_first = 1;
	g_charger_para.p_chg_ranges = chg_ranges;
	g_charger_para.chg_ranges_len = sizeof(chg_ranges) / sizeof(struct smb_step_chg);
	g_charger_para.batt_healthd = POWER_SUPPLY_HEALTH_GOOD;
	g_charger_para.batt_healthd_save = POWER_SUPPLY_HEALTH_GOOD;
	g_charger_para.chg_time_out = MAX_CHARGER_TIMEOUT;

	g_charger_para.lcd_is_on = 0;
	g_charger_para.lcd_on_chg_curr_proc = 2000000; //Set to max charger current
	chg->fb_notif.notifier_call = fb_notifier_callback;
	fb_register_client(&chg->fb_notif);

	chg->notify_code = 0;
	//chg->charger_times = 0;// Can't clear here,maybe clear the boot-up value

	proc_entry_charger_cycle = proc_create_data("charger_cycle", 0444, NULL,&charger_cycle_fops, NULL);
	if (proc_entry_charger_cycle == NULL) {
		pr_err("[%s]: Error! Couldn't create proc entry charger_cycle!\n", __func__);
	}
	proc_entry_charging_limit_current = proc_create_data("charging_limit_current", 0666, NULL, &charging_limit_current_fops, NULL);
	if (proc_entry_charging_limit_current == NULL) {
		pr_err("[%s]: Error! Couldn't create proc entry charging_limit_current!\n", __func__);
	}
	proc_entry_charging_limit_time = proc_create_data("charging_limit_time", 0666, NULL, &charging_limit_time_fops, NULL);
	if (proc_entry_charging_limit_time == NULL) {
		pr_err("[%s]: Error! Couldn't create proc entry charging_limit_time!\n", __func__);
	}

	INIT_DELAYED_WORK(&chg->charger_monitor_work, smblib_charger_monitor_work);
	schedule_delayed_work(&chg->charger_monitor_work, msecs_to_jiffies(5000));

	init_proc_tbatt_pwroff();
	oppo_tbatt_power_off_task_init(chg);

	return 0;
}

int deinit_chg_monitor_work(struct smb_charger *chg)
{
	fb_unregister_client(&chg->fb_notif);

	cancel_delayed_work_sync(&chg->charger_monitor_work);

	return 0;
}
#endif
