/*******************************************************************************
** Copyright (C), 2008-2016, OPPO Mobile Comm Corp., Ltd.
** ODM_WT_EDIT
** FILE: - hiamx_ic_HX86102.h
** Description : This program is for hiamx driver
** Version: 1.0
** Date : 2018/5/22
**
** -------------------------Revision History:----------------------------------
**  <author>	 <data> 	<version >			<desc>
**
**
*******************************************************************************/

#include "himax_platform.h"
#include "himax_common.h"
#include "himax_ic_core.h"
#include <linux/slab.h>

#define hx83102ab_fw_addr_sorting_mode_en		0x100007FC
#define hx83102ab_fw_addr_selftest_addr_en		0x100007F8
#define hx83102ab_data_adc_cfg_1			0x10007B00

#define hx83102d_fw_addr_raw_out_sel		0x800204f4//0x10007fdc
//#define hx83102d_fw_addr_fw_edge_limit 0x10007F3C
//#define hx83102d_fw_addr_fw_no_jiter_en 0x10007FE0

#define hx83102d_zf_data_adc_cfg_1 0x10007B00
#define hx83102d_zf_data_adc_cfg_2 0x10006A00
#define hx83102d_zf_data_adc_cfg_3 0x10007500
#define hx83102d_zf_data_bor_prevent_info 0x10007268
#define hx83102d_zf_data_notch_info 0x10007300
#define hx83102d_zf_func_info_en 0x10007FD0
#define hx83102d_zf_po_sub_func 0x10005A00

#define hx83102d_zf_data_sram_start_addr 0x20000000


#ifdef HX_ESD_RECOVERY
	extern u8 HX_ESD_RESET_ACTIVATE;
#endif

