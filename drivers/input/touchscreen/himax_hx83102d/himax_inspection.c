/*******************************************************************************
** Copyright (C), 2008-2016, OPPO Mobile Comm Corp., Ltd.
** ODM_WT_EDIT
** FILE: - hiamx_inspection.c
** Description : This program is for hiamx driver
** Version: 1.0
** Date : 2018/5/22
**
** -------------------------Revision History:----------------------------------
**  <author>	 <data> 	<version >			<desc>
**
**
*******************************************************************************/
#include "himax_inspection.h"

extern struct himax_core_fp g_core_fp;
extern struct himax_ts_data *private_ts;
extern struct himax_ic_data *ic_data;
extern int hx_ctpmodule;
extern struct fw_operation *pfw_op;
#ifdef HX_ZERO_FLASH
extern struct zf_operation *pzf_op;
#endif

void himax_inspection_init(void);

#if defined(HX_ZERO_FLASH)
char *sort_fw_name = "himax_firmware_sort.bin";
char *sort_fw_name_tcl = "himax_firmware_sort_tcl.bin";
#endif
int himax_chip_lpwug_self_test(void);

void (*fp_himax_baseline_test_init)(void) = himax_inspection_init;

#ifdef HX_ESD_RECOVERY
	extern u8 HX_ESD_RESET_ACTIVATE;
#endif

#ifdef HX_INSPECT_LPWUG_TEST
extern bool FAKE_POWER_KEY_SEND;
void himax_press_powerkey(bool key_status)
{
	/*if(FAKE_POWER_KEY_SEND == key_status)
	{
		if(key_status == false)
		{
			I("Already suspend!\n");
		}
		else
		{
			I("Already resume!\n");
		}
		return ;
	}*/

	I(" %s POWER KEY event %x press\n",__func__,KEY_POWER);
    input_report_key(private_ts->input_dev, KEY_POWER, 1);
    input_sync(private_ts->input_dev);

    I(" %s POWER KEY event %x release\n",__func__,KEY_POWER);
    input_report_key(private_ts->input_dev, KEY_POWER, 0);
    input_sync(private_ts->input_dev);

	//FAKE_POWER_KEY_SEND = key_status;
}
#endif
static uint8_t	NOISEMAX;

int himax_switch_mode_inspection(int mode)
{
	uint8_t tmp_addr[4];
	uint8_t tmp_data[4];
	I("%s: Entering\n", __func__);

	/*Stop Handshaking*/
	tmp_addr[3] = 0x10; tmp_addr[2] = 0x00; tmp_addr[1] = 0x00; tmp_addr[0] = 0x00;
	tmp_data[3] = 0x00; tmp_data[2] = 0x00; tmp_data[1] = 0x00; tmp_data[0] = 0x00;
	g_core_fp.fp_flash_write_burst_lenth(tmp_addr, tmp_data, 4);

	/*Swtich Mode*/
	switch (mode) {
	case HIMAX_INSPECTION_SORTING:
		tmp_data[3] = 0x00; tmp_data[2] = 0x00; tmp_data[1] = PWD_SORTING_START; tmp_data[0] = PWD_SORTING_START;
		break;
	case HIMAX_INSPECTION_OPEN:
		tmp_data[3] = 0x00; tmp_data[2] = 0x00; tmp_data[1] = PWD_OPEN_START; tmp_data[0] = PWD_OPEN_START;
		break;
	case HIMAX_INSPECTION_MICRO_OPEN:
		tmp_data[3] = 0x00; tmp_data[2] = 0x00; tmp_data[1] = PWD_OPEN_START; tmp_data[0] = PWD_OPEN_START;
		break;
	case HIMAX_INSPECTION_SHORT:
		tmp_data[3] = 0x00; tmp_data[2] = 0x00; tmp_data[1] = PWD_SHORT_START; tmp_data[0] = PWD_SHORT_START;
		break;
	case HIMAX_INSPECTION_GAPTEST_RAW:
	case HIMAX_INSPECTION_RAWDATA:
		tmp_data[3] = 0x00; tmp_data[2] = 0x00; tmp_data[1] = PWD_RAWDATA_START; tmp_data[0] = PWD_RAWDATA_START;
		break;
	case HIMAX_INSPECTION_NOISE:
		tmp_data[3] = 0x00; tmp_data[2] = 0x00; tmp_data[1] = PWD_NOISE_START; tmp_data[0] = PWD_NOISE_START;
		break;
#ifdef HX_DOZE_TEST
	case HIMAX_INSPECTION_DOZE_RAWDATA:
	case HIMAX_INSPECTION_DOZE_NOISE:
		tmp_data[3] = 0x00; tmp_data[2] = 0x00; tmp_data[1] = PWD_DOZE_START; tmp_data[0] = PWD_DOZE_START;
		break;
#endif
#ifdef HX_INSPECT_LPWUG_TEST
	case HIMAX_INSPECTION_LPWUG_RAWDATA:
	case HIMAX_INSPECTION_LPWUG_NOISE:
		tmp_data[3] = 0x00; tmp_data[2] = 0x00; tmp_data[1] = PWD_LPWUG_START; tmp_data[0] = PWD_LPWUG_START;
		break;
	case HIMAX_INSPECTION_LPWUG_IDLE_RAWDATA:
	case HIMAX_INSPECTION_LPWUG_IDLE_NOISE:
		tmp_data[3] = 0x00; tmp_data[2] = 0x00; tmp_data[1] = PWD_LPWUG_IDLE_START; tmp_data[0] = PWD_LPWUG_IDLE_START;
		break;
#endif
	default:
		I("%s,Nothing to be done!\n", __func__);
		break;
	}

	if (g_core_fp.fp_assign_sorting_mode != NULL)
		g_core_fp.fp_assign_sorting_mode(tmp_data);
	I("%s: End of setting!\n", __func__);

	return 0;

}

int himax_get_rawdata(uint32_t RAW[], uint32_t datalen)
{
	uint8_t tmp_addr[4];
	uint8_t tmp_data[4];
	uint8_t *tmp_rawdata;
	uint8_t retry = 0;
	uint16_t checksum_cal;
	uint32_t i = 0;

	uint8_t max_i2c_size = 128;
	int address = 0;
	int total_read_times = 0;
	int total_size = datalen * 2 + 4;
	int total_size_temp;
#if 1
	uint32_t j = 0;
	uint32_t index = 0;
	uint32_t Min_DATA = 0xFFFFFFFF;
	uint32_t Max_DATA = 0x00000000;
#endif

	tmp_rawdata = kzalloc(sizeof(uint8_t)*(datalen*2), GFP_KERNEL);

	/*1 Set Data Ready PWD*/
	while (retry < 200) {
		tmp_addr[3] = 0x10; tmp_addr[2] = 0x00; tmp_addr[1] = 0x00; tmp_addr[0] = 0x00;
		tmp_data[3] = 0x00; tmp_data[2] = 0x00;
		tmp_data[1] = Data_PWD1;
		tmp_data[0] = Data_PWD0;
		g_core_fp.fp_flash_write_burst_lenth(tmp_addr, tmp_data, 4);

		g_core_fp.fp_register_read(tmp_addr, 4, tmp_data, false);
		if ((tmp_data[0] == Data_PWD0 && tmp_data[1] == Data_PWD1) ||
			(tmp_data[0] == Data_PWD1 && tmp_data[1] == Data_PWD0)) {
			break;
		}

		retry++;
		msleep(1);
	}

	if (retry >= 200) {
		kfree(tmp_rawdata);
		return 1;
	} else {
		retry = 0;
	}

	while (retry < 200) {
		if (tmp_data[0] == Data_PWD1 && tmp_data[1] == Data_PWD0) {
			break;
		}

		retry++;
		msleep(1);
		g_core_fp.fp_register_read(tmp_addr, 4, tmp_data, false);
	}

	if (retry >= 200) {
		kfree(tmp_rawdata);
		return 1;
	} else {
		retry = 0;
	}

	/*2 Read Data from SRAM*/
	while (retry < 10) {
		checksum_cal = 0;
		total_size_temp = total_size;
		tmp_addr[3] = 0x10;	tmp_addr[2] = 0x00;	tmp_addr[1] = 0x00;	tmp_addr[0] = 0x00;

		if (total_size % max_i2c_size == 0) {
			total_read_times = total_size / max_i2c_size;
		} else {
			total_read_times = total_size / max_i2c_size + 1;
		}

		for (i = 0; i < (total_read_times); i++) {
			if (total_size_temp >= max_i2c_size) {
				g_core_fp.fp_register_read(tmp_addr, max_i2c_size, &tmp_rawdata[i*max_i2c_size], false);
				total_size_temp = total_size_temp - max_i2c_size;
			} else {
				/*I("last total_size_temp=%d\n", total_size_temp);*/
				g_core_fp.fp_register_read(tmp_addr, total_size_temp % max_i2c_size, &tmp_rawdata[i*max_i2c_size], false);
			}

			address = ((i+1)*max_i2c_size);
			tmp_addr[1] = (uint8_t)((address>>8)&0x00FF);
			tmp_addr[0] = (uint8_t)((address)&0x00FF);
		}

		/*3 Check Checksum*/
		for (i = 2; i < datalen * 2 + 4; i = i + 2) {
			checksum_cal += tmp_rawdata[i + 1] * 256 + tmp_rawdata[i];
		}

		if (checksum_cal == 0) {
			break;
		}

		retry++;
	}

	if (checksum_cal != 0) {
		E("%s: Get rawdata checksum fail!\n", __func__);
		kfree(tmp_rawdata);
		return HX_CHKSUM_FAIL;
	}

	/*4 Copy Data*/
	for (i = 0; i < ic_data->HX_TX_NUM*ic_data->HX_RX_NUM; i++) {
		RAW[i] = tmp_rawdata[(i * 2) + 1 + 4] * 256 + tmp_rawdata[(i * 2) + 4];
	}

#if 1
	for (j = 0; j < ic_data->HX_RX_NUM; j++) {
		if (j == 0) {
			printk("      RX%2d", j + 1);
		} else {
			printk("  RX%2d", j + 1);
		}
	}
	printk("\n");

	for (i = 0; i < ic_data->HX_TX_NUM; i++) {
		printk("TX%2d", i + 1);
		for (j = 0; j < ic_data->HX_RX_NUM; j++) {
			printk("%5d ", RAW[index]);
			if (RAW[index] > Max_DATA) {
				Max_DATA = RAW[index];
			}
			if (RAW[index] < Min_DATA) {
				Min_DATA = RAW[index];
			}
			index++;
		}
		printk("\n");
	}
	I("Max = %5d, Min = %5d \n", Max_DATA, Min_DATA);
#endif

	kfree(tmp_rawdata);
	return HX_INSPECT_OK;
}

void himax_switch_data_type(uint8_t checktype)
{
	uint8_t datatype = 0x00;

	switch (checktype) {
	case HIMAX_INSPECTION_SORTING:
		datatype = DATA_SORTING;
		break;
	case HIMAX_INSPECTION_OPEN:
		datatype = DATA_OPEN;
		break;
	case HIMAX_INSPECTION_MICRO_OPEN:
		datatype = DATA_MICRO_OPEN;
		break;
	case HIMAX_INSPECTION_SHORT:
		datatype = DATA_SHORT;
		break;
	case HIMAX_INSPECTION_RAWDATA:
		datatype = DATA_RAWDATA;
		break;
	case HIMAX_INSPECTION_NOISE:
		datatype = DATA_NOISE;
		break;
	case HIMAX_INSPECTION_BACK_NORMAL:
		datatype = DATA_BACK_NORMAL;
		break;
#ifdef HX_GAP_TEST
	case HIMAX_INSPECTION_GAPTEST_RAW:
		datatype = DATA_RAWDATA;
		break;
#endif
#ifdef HX_DOZE_TEST
	case HIMAX_INSPECTION_DOZE_RAWDATA:
		datatype = DATA_DOZE_RAWDATA;
		break;
	case HIMAX_INSPECTION_DOZE_NOISE:
		datatype = DATA_DOZE_NOISE;
		break;
#endif
#ifdef HX_INSPECT_LPWUG_TEST
	case HIMAX_INSPECTION_LPWUG_RAWDATA:
		datatype = DATA_LPWUG_RAWDATA;
		break;
	case HIMAX_INSPECTION_LPWUG_NOISE:
		datatype = DATA_LPWUG_NOISE;
		break;
	case HIMAX_INSPECTION_LPWUG_IDLE_RAWDATA:
		datatype = DATA_LPWUG_IDLE_RAWDATA;
		break;
	case HIMAX_INSPECTION_LPWUG_IDLE_NOISE:
		datatype = DATA_LPWUG_IDLE_NOISE;
		break;
#endif
	default:
		E("Wrong type=%d\n", checktype);
		break;
	}
	g_core_fp.fp_diag_register_set(datatype, 0x00);
}

void himax_set_N_frame(uint16_t Nframe, uint8_t checktype)
{
	uint8_t tmp_addr[4];
	uint8_t tmp_data[4];

	/*IIR MAX*/
	tmp_addr[3] = 0x10; tmp_addr[2] = 0x00; tmp_addr[1] = 0x72; tmp_addr[0] = 0x94;
	tmp_data[3] = 0x00; tmp_data[2] = 0x00;
	tmp_data[1] = (uint8_t)((Nframe & 0xFF00) >> 8);
	tmp_data[0] = (uint8_t)(Nframe & 0x00FF);
	g_core_fp.fp_flash_write_burst_lenth(tmp_addr, tmp_data, 4);

	/*skip frame*/
	tmp_addr[3] = 0x10; tmp_addr[2] = 0x00; tmp_addr[1] = 0x70; tmp_addr[0] = 0xF4;
	g_core_fp.fp_register_read(tmp_addr, 4, tmp_data, false);

	switch (checktype) {
#ifdef HX_DOZE_TEST
	case HIMAX_INSPECTION_DOZE_RAWDATA:
	case HIMAX_INSPECTION_DOZE_NOISE:
		tmp_data[0] = BS_DOZE;
		break;
#endif
#ifdef HX_INSPECT_LPWUG_TEST
	case HIMAX_INSPECTION_LPWUG_RAWDATA:
	case HIMAX_INSPECTION_LPWUG_NOISE:
		tmp_data[0] = BS_LPWUG;
		break;
	case HIMAX_INSPECTION_LPWUG_IDLE_RAWDATA:
	case HIMAX_INSPECTION_LPWUG_IDLE_NOISE:
		tmp_data[0] = BS_LPWUG_dile;
		break;
#endif
	case HIMAX_INSPECTION_RAWDATA:
	case HIMAX_INSPECTION_NOISE:
		tmp_data[0] = BS_RAWDATANOISE;
		break;
	default:
		tmp_data[0] = BS_OPENSHORT;
		break;
	}
	g_core_fp.fp_flash_write_burst_lenth(tmp_addr, tmp_data, 4);
}

void himax_get_noise_base(void)
{
	uint8_t tmp_addr[4];
	uint8_t tmp_data[4];

	tmp_addr[3] = 0x10; tmp_addr[2] = 0x00; tmp_addr[1] = 0x70; tmp_addr[0] = 0x8C;
	g_core_fp.fp_register_read(tmp_addr, 4, tmp_data, false);

	g_inspection_criteria[IDX_NOISEMAX] = tmp_data[3];
	I("%s: g_inspection_criteria[IDX_NOISEMAX]=%d\n", __func__, g_inspection_criteria[IDX_NOISEMAX]);
}

uint32_t himax_check_mode(uint8_t checktype)
{
	uint8_t tmp_data[4] = {0};
	uint8_t wait_pwd[2] = {0};

	switch (checktype) {
	case HIMAX_INSPECTION_SORTING:
		wait_pwd[0] = PWD_SORTING_END;
		wait_pwd[1] = PWD_SORTING_END;
		break;
	case HIMAX_INSPECTION_OPEN:
		wait_pwd[0] = PWD_OPEN_END;
		wait_pwd[1] = PWD_OPEN_END;
		break;
	case HIMAX_INSPECTION_MICRO_OPEN:
		wait_pwd[0] = PWD_OPEN_END;
		wait_pwd[1] = PWD_OPEN_END;
		break;
	case HIMAX_INSPECTION_SHORT:
		wait_pwd[0] = PWD_SHORT_END;
		wait_pwd[1] = PWD_SHORT_END;
		break;
	case HIMAX_INSPECTION_RAWDATA:
		wait_pwd[0] = PWD_RAWDATA_END;
		wait_pwd[1] = PWD_RAWDATA_END;
		break;
	case HIMAX_INSPECTION_NOISE:
		wait_pwd[0] = PWD_NOISE_END;
		wait_pwd[1] = PWD_NOISE_END;
		break;
#ifdef HX_DOZE
	case HIMAX_INSPECTION_DOZE_RAWDATA:
	case HIMAX_INSPECTION_DOZE_NOISE:
		wait_pwd[0] = PWD_DOZE_END;
		wait_pwd[1] = PWD_DOZE_END;
		break;
#endif
#ifdef HX_INSPECT_LPWUG_TEST
	case HIMAX_INSPECTION_LPWUG_RAWDATA:
	case HIMAX_INSPECTION_LPWUG_NOISE:
		wait_pwd[0] = PWD_LPWUG_END;
		wait_pwd[1] = PWD_LPWUG_END;
		break;
	case HIMAX_INSPECTION_LPWUG_IDLE_RAWDATA:
	case HIMAX_INSPECTION_LPWUG_IDLE_NOISE:
		wait_pwd[0] = PWD_LPWUG_IDLE_END;
		wait_pwd[1] = PWD_LPWUG_IDLE_END;
		break;
#endif
	default:
		E("Wrong type=%d\n", checktype);
		break;
	}

	if (g_core_fp.fp_check_sorting_mode != NULL)
		g_core_fp.fp_check_sorting_mode(tmp_data);

	if ((wait_pwd[0] == tmp_data[0]) && (wait_pwd[1] == tmp_data[1])) {
		I("Change to mode=%s\n", g_himax_inspection_mode[checktype]);
		return 0;
	} else {
		return 1;
	}
}

uint32_t himax_wait_sorting_mode(uint8_t checktype)
{
	uint8_t tmp_addr[4] = {0};
	uint8_t tmp_data[4] = {0};
	uint8_t wait_pwd[2] = {0};
	int count = 0;

	switch (checktype) {
	case HIMAX_INSPECTION_SORTING:
		wait_pwd[0] = PWD_SORTING_END;
		wait_pwd[1] = PWD_SORTING_END;
		break;
	case HIMAX_INSPECTION_OPEN:
		wait_pwd[0] = PWD_OPEN_END;
		wait_pwd[1] = PWD_OPEN_END;
		break;
	case HIMAX_INSPECTION_MICRO_OPEN:
		wait_pwd[0] = PWD_OPEN_END;
		wait_pwd[1] = PWD_OPEN_END;
		break;
	case HIMAX_INSPECTION_SHORT:
		wait_pwd[0] = PWD_SHORT_END;
		wait_pwd[1] = PWD_SHORT_END;
		break;
	case HIMAX_INSPECTION_RAWDATA:
		wait_pwd[0] = PWD_RAWDATA_END;
		wait_pwd[1] = PWD_RAWDATA_END;
		break;
	case HIMAX_INSPECTION_NOISE:
		wait_pwd[0] = PWD_NOISE_END;
		wait_pwd[1] = PWD_NOISE_END;
		break;
	case HIMAX_INSPECTION_GAPTEST_RAW:
		wait_pwd[0] = PWD_RAWDATA_END;
		wait_pwd[1] = PWD_RAWDATA_END;
		break;
#ifdef HX_DOZE_TEST
	case HIMAX_INSPECTION_DOZE_RAWDATA:
	case HIMAX_INSPECTION_DOZE_NOISE:
		wait_pwd[0] = PWD_DOZE_END;
		wait_pwd[1] = PWD_DOZE_END;
		break;
#endif
#ifdef HX_INSPECT_LPWUG_TEST
	case HIMAX_INSPECTION_LPWUG_RAWDATA:
	case HIMAX_INSPECTION_LPWUG_NOISE:
		wait_pwd[0] = PWD_LPWUG_END;
		wait_pwd[1] = PWD_LPWUG_END;
		break;
	case HIMAX_INSPECTION_LPWUG_IDLE_RAWDATA:
	case HIMAX_INSPECTION_LPWUG_IDLE_NOISE:
		wait_pwd[0] = PWD_LPWUG_IDLE_END;
		wait_pwd[1] = PWD_LPWUG_IDLE_END;
		break;
#endif
	default:
		I("No Change Mode and now type=%d\n", checktype);
		break;
	}

	do {
		if (g_core_fp.fp_check_sorting_mode != NULL)
			g_core_fp.fp_check_sorting_mode(tmp_data);
		if ((wait_pwd[0] == tmp_data[0]) && (wait_pwd[1] == tmp_data[1])) {
			return 0;
		}

		tmp_addr[3] = 0x90; tmp_addr[2] = 0x00; tmp_addr[1] = 0x00; tmp_addr[0] = 0xA8;
		g_core_fp.fp_register_read(tmp_addr, 4, tmp_data, false);
		I("%s: 0x900000A8, tmp_data[0]=%x,tmp_data[1]=%x,tmp_data[2]=%x,tmp_data[3]=%x \n", __func__, tmp_data[0], tmp_data[1], tmp_data[2], tmp_data[3]);

		tmp_addr[3] = 0x90; tmp_addr[2] = 0x00; tmp_addr[1] = 0x00; tmp_addr[0] = 0xE4;
		g_core_fp.fp_register_read(tmp_addr, 4, tmp_data, false);
		I("%s: 0x900000E4, tmp_data[0]=%x,tmp_data[1]=%x,tmp_data[2]=%x,tmp_data[3]=%x \n", __func__, tmp_data[0], tmp_data[1], tmp_data[2], tmp_data[3]);

		tmp_addr[3] = 0x10; tmp_addr[2] = 0x00; tmp_addr[1] = 0x7F; tmp_addr[0] = 0x40;
		g_core_fp.fp_register_read(tmp_addr, 4, tmp_data, false);
		I("%s: 0x10007F40,tmp_data[0]=%x,tmp_data[1]=%x,tmp_data[2]=%x,tmp_data[3]=%x \n", __func__, tmp_data[0], tmp_data[1], tmp_data[2], tmp_data[3]);
		I("Now retry %d times!\n", count++);
		msleep(50);
	} while (count < 100);

	return 1;
}

int himax_check_notch(int index)
{
	int rx_num = ic_data->HX_RX_NUM;
	int i = 0;
	int j = 0;

	for (i = NOTCH_RX_START;i <= NOTCH_RX_END; i++) {
		for (j = NOTCH_TX_START;j <= NOTCH_TX_END; j++) {
			if (((j - 1) * rx_num + i - 1) == index) {
				//I("Bypass -> index = %d, i = %d, j = %d\n", index, i, j);
				return 1;
			} else {
				//I("index = %d, i = %d, j = %d\n", index, i, j);
				continue;
			}
				
			//return (((j - 1) * rx_num + i - 1) == index)?1:0;
		}
	}

	return 0;
}

uint32_t mpTestFunc(uint8_t checktype, uint32_t datalen)
{
	uint32_t i/*, j*/, ret = 0;
	/*wdd 18/07/25 uninit*/
	uint32_t RAW[datalen];
	/*uint16_t* pInspectGridData = &gInspectGridData[0];*/
	/*uint16_t* pInspectNoiseData = &gInspectNoiseData[0];*/
	memset(&RAW[0],0x00,datalen*sizeof(uint32_t));
	if (himax_check_mode(checktype)) {
		I("Need Change Mode ,target=%s\n", g_himax_inspection_mode[checktype]);

		g_core_fp.fp_sense_off(true);

#ifndef HX_ZERO_FLASH
		if (g_core_fp.fp_reload_disable != NULL)
			g_core_fp.fp_reload_disable(1);
#endif

		himax_switch_mode_inspection(checktype);

		if (checktype == HIMAX_INSPECTION_NOISE) {
			himax_set_N_frame(NOISEFRAME, checktype);
			himax_get_noise_base();
#ifdef HX_DOZE_TEST
		} else if (checktype == HIMAX_INSPECTION_DOZE_RAWDATA || checktype == HIMAX_INSPECTION_DOZE_NOISE) {
			I("N frame = %d\n", 10);
			himax_set_N_frame(10, checktype);
#endif
#ifdef HX_INSPECT_LPWUG_TEST
		} else if (checktype >= HIMAX_INSPECTION_LPWUG_RAWDATA) {
			I("N frame = %d\n", 1);
			himax_set_N_frame(1, checktype);
#endif
		} else {
			himax_set_N_frame(2, checktype);
		}

		g_core_fp.fp_sense_on(1);

		ret = himax_wait_sorting_mode(checktype);
		if (ret) {
			E("%s: himax_wait_sorting_mode FAIL\n", __func__);
			return ret;
		}
	}

	himax_switch_data_type(checktype);

	ret = himax_get_rawdata(RAW, datalen);
	if (ret) {
		E("%s: himax_get_rawdata FAIL\n", __func__);
		return ret;
	}

	/* back to normal */
	himax_switch_data_type(HIMAX_INSPECTION_BACK_NORMAL);

	/*Check Data*/
	switch (checktype) {
	case HIMAX_INSPECTION_SORTING:
		for (i = 0; i < (ic_data->HX_TX_NUM*ic_data->HX_RX_NUM); i++) {
			if (himax_check_notch(i)) {
				continue;
			}
			if ((int)RAW[i] < g_inspection_criteria[IDX_SORTMIN]) {
				E("%s: sorting mode open test FAIL\n", __func__);
				return HX_INSPECT_EOPEN;
			}
		}
		I("%s: sorting mode open test PASS\n", __func__);
		break;
	case HIMAX_INSPECTION_OPEN:
		for (i = 0; i < (ic_data->HX_TX_NUM*ic_data->HX_RX_NUM); i++) {
			if (himax_check_notch(i)) {
				continue;
			}
			if ((int)RAW[i] > g_inspection_criteria[IDX_OPENMAX] || (int)RAW[i] < g_inspection_criteria[IDX_OPENMIN]) {
				E("%s: open test FAIL\n", __func__);
				return HX_INSPECT_EOPEN;
			}
		}
		I("%s: open test PASS\n", __func__);
		break;

	case HIMAX_INSPECTION_MICRO_OPEN:
		for (i = 0; i < (ic_data->HX_TX_NUM*ic_data->HX_RX_NUM); i++) {
			if (himax_check_notch(i)) {
				continue;
			}
			if ((int)RAW[i] > g_inspection_criteria[IDX_M_OPENMAX] || (int)RAW[i] < g_inspection_criteria[IDX_M_OPENMIN]) {
				E("%s: open test FAIL\n", __func__);
				return HX_INSPECT_EMOPEN;
			}
		}
		I("%s: open test PASS\n", __func__);
		break;

	case HIMAX_INSPECTION_SHORT:
		for (i = 0; i < (ic_data->HX_TX_NUM*ic_data->HX_RX_NUM); i++) {
			if (himax_check_notch(i)) {
				continue;
			}
			if ((int)RAW[i] > g_inspection_criteria[IDX_SHORTMAX] || (int)RAW[i] < g_inspection_criteria[IDX_SHORTMIN]) {
				E("%s: short test FAIL\n", __func__);
					return HX_INSPECT_ESHORT;
			}
		}
		I("%s: short test PASS\n", __func__);
		break;

	case HIMAX_INSPECTION_RAWDATA:
		for (i = 0; i < (ic_data->HX_TX_NUM*ic_data->HX_RX_NUM); i++) {
			if (himax_check_notch(i)) {
				continue;
			}
			if ((int)RAW[i] > g_inspection_criteria[IDX_RAWMAX] || (int)RAW[i] < g_inspection_criteria[IDX_RAWMIN]) {
				E("%s: rawdata test FAIL\n", __func__);
					return HX_INSPECT_ERAW;
			}
		}
		I("%s: rawdata test PASS\n", __func__);
		break;

	case HIMAX_INSPECTION_NOISE:
		I("NOISEMAX=%d\n", NOISEMAX);
		for (i = 0; i < (ic_data->HX_TX_NUM * ic_data->HX_RX_NUM); i++) {
			if (himax_check_notch(i)) {
				continue;
			}
			if ((int)RAW[i] > g_inspection_criteria[IDX_NOISEMAX]) {
				E("%s: noise test FAIL\n", __func__);
					return HX_INSPECT_ENOISE;
			}
		}
		I("%s: noise test PASS\n", __func__);
		break;
#ifdef HX_GAP_TEST
	case HIMAX_INSPECTION_GAPTEST_RAW:
		if (himax_gap_test_vertical_raw(HIMAX_INSPECTION_GAPTEST_RAW, RAW) != NO_ERR) {
			E("%s: HIMAX_INSPECTION_GAPTEST_RAW FAIL\n", __func__);
			return HX_INSPECT_EGAP_RAW;
		}
		if (himax_gap_test_honrizontal_raw(HIMAX_INSPECTION_GAPTEST_RAW, RAW) != NO_ERR) {
			E("%s: HIMAX_INSPECTION_GAPTEST_RAW FAIL\n", __func__);
			return HX_INSPECT_EGAP_RAW;
		}
		break;
#endif
#ifdef HX_DOZE_TEST
	case HIMAX_INSPECTION_DOZE_RAWDATA:
		for (i = 0; i < (ic_data->HX_TX_NUM*ic_data->HX_RX_NUM); i++) {
			if (himax_check_notch(i)) {
				continue;
			}
			if ((int)RAW[i] > g_inspection_criteria[IDX_DOZE_RAWDATA_MAX] || (int)RAW[i] < g_inspection_criteria[IDX_DOZE_RAWDATA_MIN]) {
				E("%s: HIMAX_INSPECTION_DOZE_RAWDATA FAIL\n", __func__);
					return HX_INSPECT_EDOZE_RAW;
			}
		}
		I("%s: HIMAX_INSPECTION_DOZE_RAWDATA PASS\n", __func__);
		break;
	case HIMAX_INSPECTION_DOZE_NOISE:
		for (i = 0; i < (ic_data->HX_TX_NUM*ic_data->HX_RX_NUM); i++) {
			if (himax_check_notch(i)) {
				continue;
			}
			if ((int)RAW[i] > g_inspection_criteria[IDX_DOZE_NOISE_MAX] || (int)RAW[i] < g_inspection_criteria[IDX_DOZE_NOISE_MIN]) {
				E("%s: HIMAX_INSPECTION_DOZE_NOISE FAIL\n", __func__);
					return HX_INSPECT_EDOZE_NOISE;
			}
		}
		I("%s: HIMAX_INSPECTION_DOZE_NOISE PASS\n", __func__);
		break;
#endif
#ifdef HX_INSPECT_LPWUG_TEST
	case HIMAX_INSPECTION_LPWUG_RAWDATA:
		for (i = 0; i < (ic_data->HX_TX_NUM*ic_data->HX_RX_NUM); i++) {
			if (himax_check_notch(i)) {
				continue;
			}
			if ((int)RAW[i] > g_inspection_criteria[IDX_LPWUG_RAWDATA_MAX] || (int)RAW[i] < g_inspection_criteria[IDX_LPWUG_RAWDATA_MIN]) {
				E("%s: HIMAX_INSPECTION_LPWUG_RAWDATA FAIL\n", __func__);
					return HX_INSPECT_ELPWUG_RAW;
			}
		}
		I("%s: HIMAX_INSPECTION_LPWUG_RAWDATA PASS\n", __func__);
		break;
	case HIMAX_INSPECTION_LPWUG_NOISE:
		for (i = 0; i < (ic_data->HX_TX_NUM * ic_data->HX_RX_NUM); i++) {
			if (himax_check_notch(i)) {
				continue;
			}
			if ((int)RAW[i] > g_inspection_criteria[IDX_LPWUG_NOISE_MAX] || (int)RAW[i] < g_inspection_criteria[IDX_LPWUG_NOISE_MIN]) {
				E("%s: HIMAX_INSPECTION_LPWUG_NOISE FAIL\n", __func__);
					return HX_INSPECT_ELPWUG_NOISE;
			}
		}
		I("%s: HIMAX_INSPECTION_LPWUG_NOISE PASS\n", __func__);
		break;
	case HIMAX_INSPECTION_LPWUG_IDLE_RAWDATA:
		for (i = 0; i < (ic_data->HX_TX_NUM*ic_data->HX_RX_NUM); i++) {
			if (himax_check_notch(i)) {
				continue;
			}
			if ((int)RAW[i] > g_inspection_criteria[IDX_LPWUG_IDLE_RAWDATA_MAX] || (int)RAW[i] < g_inspection_criteria[IDX_LPWUG_IDLE_RAWDATA_MIN]) {
				E("%s: HIMAX_INSPECTION_LPWUG_IDLE_RAWDATA FAIL\n", __func__);
					return HX_INSPECT_ELPWUG_IDLE_RAW;
			}
		}
		I("%s: HIMAX_INSPECTION_LPWUG_IDLE_RAWDATA PASS\n", __func__);
		break;
	case HIMAX_INSPECTION_LPWUG_IDLE_NOISE:
		for (i = 0; i < (ic_data->HX_TX_NUM*ic_data->HX_RX_NUM); i++) {
			if (himax_check_notch(i)) {
				continue;
			}
			if ((int)RAW[i] > g_inspection_criteria[IDX_LPWUG_IDLE_NOISE_MAX] || (int)RAW[i] < g_inspection_criteria[IDX_LPWUG_IDLE_NOISE_MIN]) {
				E("%s: HIMAX_INSPECTION_LPWUG_IDLE_NOISE FAIL\n", __func__);
					return HX_INSPECT_ELPWUG_IDLE_NOISE;
			}
		}
		I("%s: HIMAX_INSPECTION_LPWUG_IDLE_NOISE PASS\n", __func__);
		break;
#endif
	default:
		E("Wrong type=%d\n", checktype);
		break;
	}

	return HX_INSPECT_OK;
}

/* parsing Criteria start */
int himax_get_criteria_size(void)
{
	int result = 0;

	result = HX_CRITERIA_SIZE;

	return result;
}

/* claculate 10's power function */
int himax_power_cal(int pow, int number)
{
	int i = 0;
	int result = 1;

	for (i = 0; i < pow; i++)
		result *= 10;
	result = result * number;

	return result;

}

/* String to int */
int hiamx_parse_str2int(char *str)
{
	int i = 0;
	int temp_cal = 0;
	int result = 0;
	int str_len = strlen(str);
	int negtive_flag = 0;
	for (i = 0; i < strlen(str); i++) {
		if (str[i] == '-') {
			negtive_flag = 1;
			continue;
		}
		temp_cal = str[i] - '0';
		result += himax_power_cal(str_len-i-1, temp_cal); /* str's the lowest char is the number's the highest number
															So we should reverse this number before using the power function
															-1: starting number is from 0 ex:10^0 = 1,10^1=10*/
	}

	if (negtive_flag == 1) {
		result = 0 - result;
	}

	return result;
}

int himax_count_comma(const struct firmware *file_entry)
{
	int i = 0;
	int result = 0;
	for (i = 0; i < file_entry->size; i++) {
		if (file_entry->data[i] == ASCII_COMMA)
			result++;
	}
	return result;
}

/* Get sub-string from original string by using some charaters */
int himax_saperate_comma(const struct firmware *file_entry, char **result, int str_size)
{
	int count = 0;
	int str_count = 0; /* now string*/
	int char_count = 0; /* now char count in string*/

	do {
		switch (file_entry->data[count]) {
		case ASCII_COMMA:
		case ACSII_SPACE:
		case ASCII_CR:
		case ASCII_LF:
			count++;
			/* If end of line as above condifiton, differencing the count of char.
				If char_count != 0 it's meaning this string is parsing over .
				The Next char is belong to next string */
			if (char_count != 0) {
				char_count = 0;
				str_count++;
			}
			break;
		default:
			result[str_count][char_count++] = file_entry->data[count];
			count++;
			break;
		}
	} while (count < file_entry->size && str_count < str_size);

	return 0;
}

void himax_parse_criteria_file(void)
{
	int err = NO_ERR;
	int tmp = 0;
	const struct firmware *file_entry = NULL;

	char **result;
	int i = 0;
	int data_size = 0; /* The maximum of number Data*/
	int test_array_size = 0;
	char *file_name = NULL;

	I("%s,Entering \n", __func__);
	I("file name = %s\n", file_name);
	if ( hx_ctpmodule == 0 ) {
		file_name = "hx_criteria.csv";
	} else {
		file_name = "hx_criteria_tcl.csv";

	}

	/* default path is /system/etc/firmware */
	err = request_firmware(&file_entry, file_name, private_ts->dev);
	if (err < 0) {
		E("%s,fail in line%d error code=%d\n", __func__, __LINE__, err);
		return ;
	}

	/* size of criteria include name string */
	data_size = himax_get_criteria_size();

	/* init the array which store original criteria and include name string*/
	result = kzalloc(data_size*sizeof(char *), GFP_KERNEL);
	for (i = 0 ; i < data_size; i++)
		result[i] = kzalloc(128*sizeof(char), GFP_KERNEL);

	/* dbg */
	I("first 4 bytes 0x%2X,0x%2X,0x%2X,0x%2X !\n", file_entry->data[0], file_entry->data[1], file_entry->data[2], file_entry->data[3]);


	himax_saperate_comma(file_entry, result, data_size);

	test_array_size = himax_get_criteria_size();
	I("Test print Size=%d\n", test_array_size);

	for (i = 0; i < HX_CRITERIA_ITEM; i++) {
		g_inspection_criteria[i] = hiamx_parse_str2int(result[(i*2)+1]);
		I("g_inspection_criteria[%d]=%d\n", i, g_inspection_criteria[i]);
		tmp = g_inspection_criteria[i] < 0 ? (0 - g_inspection_criteria[i]) : g_inspection_criteria[i];
		if(tmp > 99999)/* prevent abnormal case, rawdata above 65535 means abnormal */
		{
			break;
		}
	}

	if(i < HX_CRITERIA_ITEM){
		E("Loading csv fail, use default value:1, pls check csv file!\n!");
		for (i = 0; i < HX_CRITERIA_ITEM; i++) {
			g_inspection_criteria[i] = 1;
			I("g_inspection_criteria[%d]=%d\n", i, g_inspection_criteria[i]);
		}
	}

	/* for dbg*/
	for (i = 0; i < test_array_size; i++)
		I("[%d]%s\n", i, result[i]);

	release_firmware(file_entry);
	for (i = 0 ; i < data_size; i++)
		kfree(result[i]);
	kfree(result);
	I("%s,END \n", __func__);
}
/* parsing Criteria end */

void himax_self_test_data_init(void)
{
	g_inspection_criteria = kzalloc(sizeof(int)*HX_CRITERIA_ITEM, GFP_KERNEL);
	himax_parse_criteria_file();
}

void himax_self_test_data_deinit(void)
{
	int i = 0;

	/*dbg*/
	for (i = 0; i < HX_CRITERIA_ITEM; i++)
		I("%s:[%d]%d\n", __func__, i, g_inspection_criteria[i]);

	kfree(g_inspection_criteria);
}

int himax_chip_self_test(void)
{
	struct himax_ts_data *ts = private_ts;
	uint32_t ret = HX_INSPECT_OK;
	I("%s:IN\n", __func__);

	himax_self_test_data_init();
	ts->in_baseline_test = 1;
	
	/*Write selftets bin into SRAM for factory test*/
	//g_core_fp.fp_0f_op_file_dirly(sort_fw_name);
	g_core_fp.fp_0f_operation_dirly();
	msleep(5);
	g_core_fp.fp_reload_disable(0);
	msleep(10);
	g_core_fp.fp_read_FW_ver();
	g_core_fp.fp_sense_on(0x00);

	/* Clear N_frame*/
	g_core_fp.fp_register_write(pzf_op->data_mode_switch,
		sizeof(pfw_op->data_clear), pfw_op->data_clear, false);

	if (ts->suspended == false) {
		I("[SCREEN ON SELF TEST!]\n");
		if (hx_ctpmodule == 1) { /*HX83112-A*/
			/*1. Open Test*/
			I("[MP_OPEN_TEST_RAW]\n");
			ret += mpTestFunc(HIMAX_INSPECTION_OPEN, (ic_data->HX_TX_NUM*ic_data->HX_RX_NUM) + ic_data->HX_TX_NUM + ic_data->HX_RX_NUM);
			I("1. Open Test: End %d\n\n\n", ret);
 

			/*2. Micro-Open Test*/
			I("[MP_MICRO_OPEN_TEST_RAW]\n");
			ret += mpTestFunc(HIMAX_INSPECTION_MICRO_OPEN, (ic_data->HX_TX_NUM*ic_data->HX_RX_NUM) + ic_data->HX_TX_NUM + ic_data->HX_RX_NUM);
			I("2. Micro Open Test: End %d\n\n\n", ret);
		} else { /*HX83102-D*/
			I("[MP_SORTING_TEST_RAW]\n");
			ret += mpTestFunc(HIMAX_INSPECTION_SORTING, (ic_data->HX_TX_NUM*ic_data->HX_RX_NUM) + ic_data->HX_TX_NUM + ic_data->HX_RX_NUM);
			I("1. 2. Sorting Test: End %d\n\n\n", ret);
		}

	/*3. Short Test*/
	I("[MP_SHORT_TEST_RAW]\n");
	ret += mpTestFunc(HIMAX_INSPECTION_SHORT, (ic_data->HX_TX_NUM*ic_data->HX_RX_NUM) + ic_data->HX_TX_NUM + ic_data->HX_RX_NUM);
	I("3. Short Test: End %d\n\n\n", ret);

	/*4. RawData Test*/
	I("==========================================\n");
	I("[MP_RAW_TEST_RAW]\n");
	ret += mpTestFunc(HIMAX_INSPECTION_RAWDATA, (ic_data->HX_TX_NUM*ic_data->HX_RX_NUM)+ic_data->HX_TX_NUM+ic_data->HX_RX_NUM);
	I("4. Short Test: End %d\n\n\n", ret);

	/*5. Noise Test*/
	I("[MP_NOISE_TEST_RAW]\n");
	ret += mpTestFunc(HIMAX_INSPECTION_NOISE, (ic_data->HX_TX_NUM*ic_data->HX_RX_NUM) + ic_data->HX_TX_NUM + ic_data->HX_RX_NUM);
	I("5. Noise Test: End %d\n\n\n", ret);

#ifdef HX_GAP_TEST
	/*6. GAP Test*/
	I("[MP_GAP_TEST_RAW]\n");
	ret += mpTestFunc(HIMAX_INSPECTION_GAPTEST_RAW, (ic_data->HX_TX_NUM*ic_data->HX_RX_NUM) + ic_data->HX_TX_NUM + ic_data->HX_RX_NUM);
	I("6. MP_GAP_TEST_RAW: End %d\n\n\n", ret);
#endif

#ifdef HX_DOZE_TEST
	/*7. DOZE RAWDATA*/
	I("[MP_DOZE_TEST_RAW]\n");
	ret += mpTestFunc(HIMAX_INSPECTION_DOZE_RAWDATA, (ic_data->HX_TX_NUM*ic_data->HX_RX_NUM) + ic_data->HX_TX_NUM + ic_data->HX_RX_NUM);
	I("7. MP_DOZE_TEST_RAW: End %d\n\n\n", ret);

	/*8. DOZE NOISE*/
	I("[MP_DOZE_TEST_NOISE]\n");
	ret += mpTestFunc(HIMAX_INSPECTION_DOZE_NOISE, (ic_data->HX_TX_NUM*ic_data->HX_RX_NUM) + ic_data->HX_TX_NUM + ic_data->HX_RX_NUM);
	I("8. MP_DOZE_TEST_NOISE: End %d\n\n\n", ret);
#endif
	}

	g_core_fp.fp_sense_off(true);

	/*Prepare for LPWUG TEST*/
	if (ts->SMWP_enable == 0)
		g_core_fp.fp_set_SMWP_enable(1, false);

	himax_switch_mode_inspection(HIMAX_INSPECTION_LPWUG_RAWDATA);

	if (ret == 0)
		ret = 0xAA;

	I("%s:OUT\n", __func__);
	return ret;
}
int himax_black_chip_self_test(void)
{
	struct himax_ts_data *ts = private_ts;
	uint32_t ret = HX_INSPECT_OK;
	I("%s:IN\n", __func__);

#ifdef HX_INSPECT_LPWUG_TEST
	//himax_press_powerkey(false);
#ifdef HX_SMART_WAKEUP
	/*himax_press_powerkey(false);*/
	if (ts->suspended == true && ts->in_baseline_test == 1) {
	I("[SCREEN OFF SELF TEST!]\n");
	/*9. LPWUG RAWDATA*/
	I("[MP_LPWUG_TEST_RAW]\n");
	ret += mpTestFunc(HIMAX_INSPECTION_LPWUG_RAWDATA, (ic_data->HX_TX_NUM*ic_data->HX_RX_NUM) + ic_data->HX_TX_NUM + ic_data->HX_RX_NUM);
	I("9. MP_LPWUG_TEST_RAW: End %d\n\n\n", ret);

	/*10. LPWUG NOISE*/
	I("[MP_LPWUG_TEST_NOISE]\n");
	ret += mpTestFunc(HIMAX_INSPECTION_LPWUG_NOISE, (ic_data->HX_TX_NUM*ic_data->HX_RX_NUM) + ic_data->HX_TX_NUM + ic_data->HX_RX_NUM);
	I("10. MP_LPWUG_TEST_NOISE: End %d\n\n\n", ret);

	/*11. LPWUG IDLE RAWDATA*/
	I("[MP_LPWUG_IDLE_TEST_RAW]\n");
	ret += mpTestFunc(HIMAX_INSPECTION_LPWUG_IDLE_RAWDATA, (ic_data->HX_TX_NUM*ic_data->HX_RX_NUM) + ic_data->HX_TX_NUM + ic_data->HX_RX_NUM);
	I("11. MP_LPWUG_IDLE_TEST_RAW: End %d\n\n\n", ret);

	/*12. LPWUG IDLE RAWDATA*/
	I("[MP_LPWUG_IDLE_TEST_NOISE]\n");
	ret += mpTestFunc(HIMAX_INSPECTION_LPWUG_IDLE_NOISE, (ic_data->HX_TX_NUM*ic_data->HX_RX_NUM) + ic_data->HX_TX_NUM + ic_data->HX_RX_NUM);
	I("12. MP_LPWUG_IDLE_TEST_NOISE: End %d\n\n\n", ret);

	//himax_press_powerkey(true);
	himax_self_test_data_deinit();
	}
#endif
	/*himax_press_powerkey(true);*/
#endif
	ts->in_baseline_test = 0;
	g_core_fp.fp_sense_off(false);
	//himax_set_N_frame(1, HIMAX_INSPECTION_NOISE);
#ifndef HX_ZERO_FLASH
	if (g_core_fp.fp_reload_disable != NULL)
		g_core_fp.fp_reload_disable(0);
#endif
	g_core_fp.fp_sense_on(0);

  g_core_fp.fp_0f_operation_dirly();
	msleep(5);
	g_core_fp.fp_reload_disable(0);
	msleep(5);
	g_core_fp.fp_read_FW_ver();
	g_core_fp.fp_sense_on(0);

	if (ret == 0)
		ret = 0xAA;

	I("%s:OUT\n", __func__);
	return ret;
}

void himax_inspection_init(void)
{
	I("%s: enter, %d \n", __func__, __LINE__);

	g_core_fp.fp_chip_baseline_test = himax_chip_self_test;

	return;
}
