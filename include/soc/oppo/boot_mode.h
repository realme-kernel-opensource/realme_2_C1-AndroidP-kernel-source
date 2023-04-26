/************************************************************************************
** File: - android\kernel\arch\arm\mach-msm\include\mach\oppo_boot.h
** VENDOR_EDIT
** Copyright (C), 2008-2012, OPPO Mobile Comm Corp., Ltd
** 
** Description:  
**     change define of boot_mode here for other place to use it
** Version: 1.0 
** --------------------------- Revision History: --------------------------------
** 	<author>	<data>			<desc>
************************************************************************************/
#ifndef _OPPO_BOOT_H
#define _OPPO_BOOT_H
enum{
	MSM_BOOT_MODE__NORMAL,
	MSM_BOOT_MODE__FASTBOOT,
	MSM_BOOT_MODE__RECOVERY,
	MSM_BOOT_MODE__FACTORY,
	MSM_BOOT_MODE__RF,
	MSM_BOOT_MODE__WLAN,
	MSM_BOOT_MODE__MOS,
	MSM_BOOT_MODE__CHARGE,
	MSM_BOOT_MODE__SILENCE,
	MSM_BOOT_MODE__SAU,
   
};

extern int get_boot_mode(void);
#ifdef VENDOR_EDIT
extern bool qpnp_is_power_off_charging(void);
#endif
#ifdef VENDOR_EDIT
extern bool qpnp_is_charger_reboot(void);
#endif /*VENDOR_EDIT*/
#ifdef VENDOR_EDIT
enum{
        VERIFIED_BOOT_STATE__GREEN,
        VERIFIED_BOOT_STATE__ORANGE,
        VERIFIED_BOOT_STATE__YELLOW,
        VERIFIED_BOOT_STATE__RED,
};

extern bool is_bootloader_unlocked(void);
#endif /*VENDOR_EDIT*/
#endif