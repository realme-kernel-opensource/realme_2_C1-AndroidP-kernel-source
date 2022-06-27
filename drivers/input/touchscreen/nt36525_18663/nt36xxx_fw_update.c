/*******************************************************************************
** Copyright (C), 2008-2016, OPPO Mobile Comm Corp., Ltd.
** ODM_WT_EDIT
** FILE: - nt36xxx_fw_update.c
** Description : This program is for nt36xxx driver
** Version: 1.0
** Date : 2018/5/8
**
** -------------------------Revision History:----------------------------------
**  <author>	 <data> 	<version >			<desc>
**
**
*******************************************************************************/
#include <linux/proc_fs.h>
#include <linux/seq_file.h>
#include <linux/delay.h>
#include <linux/slab.h>
#include <asm/uaccess.h>

#include <linux/delay.h>
#include <linux/firmware.h>
#include <linux/gpio.h>

#include "nt36xxx.h"
//#include "NT36525_AUO_0622_HDp_OPPO_PID5F12_V03_20180626.h"
//#include "NT36525_AUO_0622_HDp_OPPO_PID5F12_V05_20180723.h"
#include "NT36525_AUO_0622_HDp_OPPO_PID5F12_V08_20180817.h"

#if BOOT_UPDATE_FIRMWARE

#define FW_BIN_SIZE_116KB		(118784)
#define FW_BIN_SIZE FW_BIN_SIZE_116KB
#define FW_BIN_VER_OFFSET		(0x1A000)
#define FW_BIN_VER_BAR_OFFSET	(0x1A001)
#define FW_BIN_TYPE_OFFSET		(0x1A00D)
extern void nvt_resume_tmp(void);
extern int request_firmware_select(const struct firmware **firmware_p, const char *name,struct device *device);

#define NVT_DUMP_SRAM   (0)
#ifdef ODM_WT_EDIT
extern int ctpmodule;
#endif
struct timeval start, end;
const struct firmware *fw_entry = NULL;

const struct firmware *fw_entry_normal = NULL;
const struct firmware *fw_entry_mp = NULL;
static uint8_t request_and_download_normal_complete = false;
static uint8_t request_and_download_mp_complete = false;

static uint8_t *fwbuf = NULL;

struct nvt_ts_bin_map {
	char name[12];
	uint32_t BIN_addr;
	uint32_t SRAM_addr;
	uint32_t size;
	uint32_t crc;
};

static struct nvt_ts_bin_map *bin_map;

/*******************************************************
Description:
	Novatek touchscreen init variable and allocate buffer
for download firmware function.

return:
	n.a.
*******************************************************/
static int32_t nvt_download_init(void)
{
	/* allocate buffer for transfer firmware */
	//NVT_LOG("NVT_TANSFER_LEN = %ld\n", NVT_TANSFER_LEN);

	if (fwbuf == NULL) {
		fwbuf = (uint8_t *)kzalloc((NVT_TANSFER_LEN+1), GFP_KERNEL);
		if(fwbuf == NULL) {
			NVT_ERR("kzalloc for fwbuf failed!\n");
			return -ENOMEM;
		}
	}

	return 0;
}

/*******************************************************
Description:
	Novatek touchscreen checksum function. Calculate bin
file checksum for comparison.

return:
	n.a.
*******************************************************/
static uint32_t CheckSum(const u8 *data, size_t len)
{
	uint32_t i = 0;
	uint32_t checksum = 0;

	for (i = 0 ; i < len+1 ; i++)
		checksum += data[i];

	checksum += len;
	checksum = ~checksum +1;

	return checksum;
}

static uint32_t byte_to_word(const uint8_t *data)
{
	return data[0] + (data[1] << 8) + (data[2] << 16) + (data[3] << 24);
}

/*******************************************************
Description:
	Novatek touchscreen parsing bin header function.

return:
	n.a.
*******************************************************/
static uint32_t partition = 0;
static uint8_t ilm_dlm_num = 2;
static int32_t nvt_bin_header_parser(const u8 *fwdata, size_t fwsize)
{
	uint32_t list = 0;
	uint32_t pos = 0x00;
	uint32_t end = 0x00;
	uint8_t info_sec_num = 0;
	uint8_t ovly_sec_num = 0;
	uint8_t ovly_info = 0;

	/* Find the header size */
	end = fwdata[0] + (fwdata[1] << 8) + (fwdata[2] << 16) + (fwdata[3] << 24);
	pos = 0x30;	// info section start at 0x30 offset
	while (pos < end) {
		info_sec_num ++;
		pos += 0x10;	/* each header info is 16 bytes */
	}

	/*
	 * Find the DLM OVLY section
	 * [0:3] Overlay Section Number
	 * [4]   Overlay Info
	 */
	ovly_info = (fwdata[0x28] & 0x10) >> 4;
	ovly_sec_num = (ovly_info) ? (fwdata[0x28] & 0x0F) : 0;

	/*
	 * calculate all partition number
	 * ilm_dlm_num (ILM & DLM) + ovly_sec_num + info_sec_num
	 */
	partition = ilm_dlm_num + ovly_sec_num + info_sec_num;
	NVT_LOG("ovly_info = %d, ilm_dlm_num = %d, ovly_sec_num = %d, info_sec_num = %d, partition = %d\n",
			ovly_info, ilm_dlm_num, ovly_sec_num, info_sec_num, partition);

	/* allocated memory for header info */
	bin_map = (struct nvt_ts_bin_map *)kzalloc((partition+1) * sizeof(struct nvt_ts_bin_map), GFP_KERNEL);
	if(bin_map == NULL) {
		NVT_ERR("kzalloc for bin_map failed!\n");
		return -ENOMEM;
	}

	for (list = 0; list < partition; list++) {
		/*
		 * [1] parsing ILM & DLM header info
		 * BIN_addr : SRAM_addr : size (12-bytes)
		 * crc located at 0x18 & 0x1C
		 */
		if (list < ilm_dlm_num) {
			bin_map[list].BIN_addr = byte_to_word(&fwdata[0 + list*12]);
			bin_map[list].SRAM_addr = byte_to_word(&fwdata[4 + list*12]);
			bin_map[list].size = byte_to_word(&fwdata[8 + list*12]);
			if ((bin_map[list].BIN_addr + bin_map[list].size) < fwsize)
				bin_map[list].crc = CheckSum(&fwdata[bin_map[list].BIN_addr], bin_map[list].size);
			else {
				NVT_ERR("access range (0x%08X to 0x%08X) is larger than bin size!\n",
						bin_map[list].BIN_addr, bin_map[list].BIN_addr + bin_map[list].size);
				return -EINVAL;
			}
			if (list == 0)
				sprintf(bin_map[list].name, "ILM");
			else if (list == 1)
				sprintf(bin_map[list].name, "DLM");
		}

		/*
		 * [2] parsing others header info
		 * SRAM_addr : size : BIN_addr : crc (16-bytes)
		 */
		if ((list >= ilm_dlm_num) && (list < (ilm_dlm_num + info_sec_num))) {
			/* others partition located at 0x30 offset */
			pos = 0x30 + (0x10 * (list - ilm_dlm_num));

			bin_map[list].SRAM_addr = byte_to_word(&fwdata[pos]);
			bin_map[list].size = byte_to_word(&fwdata[pos+4]);
			bin_map[list].BIN_addr = byte_to_word(&fwdata[pos+8]);
			if ((bin_map[list].BIN_addr + bin_map[list].size) < fwsize)
				bin_map[list].crc = CheckSum(&fwdata[bin_map[list].BIN_addr], bin_map[list].size);
			else {
				NVT_ERR("access range (0x%08X to 0x%08X) is larger than bin size!\n",
						bin_map[list].BIN_addr, bin_map[list].BIN_addr + bin_map[list].size);
				return -EINVAL;
			}
			/* detect header end to protect parser function */
			if ((bin_map[list].BIN_addr == 0) && (bin_map[list].size != 0)) {
				sprintf(bin_map[list].name, "Header");
			} else {
				sprintf(bin_map[list].name, "Info-%d", (list - ilm_dlm_num));
			}
		}

		/*
		 * [3] parsing overlay section header info
		 * SRAM_addr : size : BIN_addr : crc (16-bytes)
		 */
		if (list >= (ilm_dlm_num + info_sec_num)) {
			/* overlay info located at DLM (list = 1) start addr */
			pos = bin_map[1].BIN_addr + (0x10 * (list- ilm_dlm_num - info_sec_num));

			bin_map[list].SRAM_addr = byte_to_word(&fwdata[pos]);
			bin_map[list].size = byte_to_word(&fwdata[pos+4]);
			bin_map[list].BIN_addr = byte_to_word(&fwdata[pos+8]);
			if ((bin_map[list].BIN_addr + bin_map[list].size) < fwsize)
				bin_map[list].crc = CheckSum(&fwdata[bin_map[list].BIN_addr], bin_map[list].size);
			else {
				NVT_ERR("access range (0x%08X to 0x%08X) is larger than bin size!\n",
						bin_map[list].BIN_addr, bin_map[list].BIN_addr + bin_map[list].size);
				return -EINVAL;
			}
			sprintf(bin_map[list].name, "Overlay-%d", (list- ilm_dlm_num - info_sec_num));
		}

//		NVT_LOG("[%d][%s] SRAM (0x%08X), SIZE (0x%08X), BIN (0x%08X), CRC (0x%08X)\n",
//				list, bin_map[list].name,
//				bin_map[list].SRAM_addr, bin_map[list].size,  bin_map[list].BIN_addr, bin_map[list].crc);
	}

	return 0;
}

/*******************************************************
Description:
	Novatek touchscreen release update firmware function.

return:
	n.a.
*******************************************************/
struct firmware *request_fw_headfile = NULL;
static void update_firmware_release(void)
{
	if ((request_and_download_normal_complete == true) &&
			(request_and_download_mp_complete == true)) {
		NVT_LOG("ignore release firmware\n");
		return;
	}

	if (request_and_download_normal_complete == false) {
		if (!IS_ERR_OR_NULL(request_fw_headfile)) {
			kfree(request_fw_headfile);
			request_fw_headfile = NULL;
			fw_entry = NULL;
		}

		if (!IS_ERR_OR_NULL(fw_entry_normal)) {
			release_firmware(fw_entry_normal);
			fw_entry_normal = NULL;
		}
	}

	if (request_and_download_mp_complete == false) {
		if (!IS_ERR_OR_NULL(fw_entry_mp)) {
			release_firmware(fw_entry_mp);
			fw_entry_mp = NULL;
		}
	}

}

/*******************************************************
Description:
	Novatek touchscreen request update firmware function.

return:
	Executive outcomes. 0---succeed. -1,-22---failed.
*******************************************************/
static int32_t update_firmware_request(char *filename)
{
	uint8_t retry = 0;
	int32_t ret = 0;

	if (NULL == filename) {
		return -1;
	}

	while (1) {
		if ( ctpmodule == 0 ){
			if ((strcmp(filename, BOOT_UPDATE_FIRMWARE_NAME_TM)) &&
				(strcmp(filename, MP_UPDATE_FIRMWARE_NAME_TM))) {
				NVT_ERR("filename %s not support\n", filename);
				goto request_fail;
			}
		} else {
			if ((strcmp(filename, BOOT_UPDATE_FIRMWARE_NAME)) &&
				(strcmp(filename, MP_UPDATE_FIRMWARE_NAME))) {
				NVT_ERR("filename %s not support\n", filename);
				goto request_fail;
			}
		}
		if (ts->recovery_flag == 0) {
			NVT_LOG("filename is %s\n", filename);
			if(ts->oppo_update_fw_flag ==1) {
				request_and_download_normal_complete = false;
			}
			if ((request_and_download_normal_complete == false) &&
				(!strcmp(filename, BOOT_UPDATE_FIRMWARE_NAME))) {
				if(ts->oppo_update_fw_flag ==1) {
					NVT_LOG("request request_firmware_select firmware\n");
					ret = request_firmware_select(&fw_entry_normal, filename,&ts->client->dev);
				} else {
					NVT_LOG("request normal firmware\n");
					ret = request_firmware(&fw_entry_normal, filename, &ts->client->dev);
				}

				if (ret) {
					NVT_ERR("normal firmware load failed,get from headfile ret=%d\n", ret);
					request_fw_headfile = kzalloc(sizeof(struct firmware), GFP_KERNEL);

					if(request_fw_headfile == NULL) {
						NVT_ERR("firmware load failed and get from headfile failed ret=%d\n", ret);
						ret = -1;
						goto request_fail;
					}
					request_fw_headfile->size = sizeof(FIRMWARE_DATA_AUO);
					request_fw_headfile->data = FIRMWARE_DATA_AUO;
					fw_entry_normal = request_fw_headfile;
				}
			}
		} else {
			NVT_LOG("is recovery boot mode, get from headfile\n");
			request_fw_headfile = kzalloc(sizeof(struct firmware), GFP_KERNEL);
			if(request_fw_headfile == NULL) {
				NVT_LOG("request_fw_headfile kzalloc failed!\n");
				ret = -1;
				goto request_fail;
			}

			request_fw_headfile->size = sizeof(FIRMWARE_DATA_AUO);
			request_fw_headfile->data = FIRMWARE_DATA_AUO;

			fw_entry_normal = request_fw_headfile;
		}

		/* request MP firmware on the first time. */
		if ((request_and_download_mp_complete == false) &&
			(!strcmp(filename, MP_UPDATE_FIRMWARE_NAME))) {
			NVT_LOG("request mp firmware\n");
			ret = request_firmware(&fw_entry_mp, filename, &ts->client->dev);
			if (ret) {
				NVT_LOG("request mp firmware failed\n");
				goto request_fail;
			}
		}
		/* choice backup firmware data */
		if (!strcmp(filename, MP_UPDATE_FIRMWARE_NAME)) {
			fw_entry = fw_entry_mp;
			if (request_and_download_mp_complete) {
				NVT_LOG("use backup mp fw\n");
			}
		} else {
			fw_entry = fw_entry_normal;
			if (request_and_download_normal_complete) {
				NVT_LOG("use backup normal fw\n");
			}
		}
		// check bin file size (116kb)
		if (fw_entry->size != FW_BIN_SIZE) {
			NVT_ERR("bin file size not match. (%zu)\n", fw_entry->size);
			ret = -1;
			goto invalid;
		}

		// check if FW version add FW version bar equals 0xFF
		if (*(fw_entry->data + FW_BIN_VER_OFFSET) + *(fw_entry->data + FW_BIN_VER_BAR_OFFSET) != 0xFF) {
			NVT_ERR("bin file FW_VER + FW_VER_BAR should be 0xFF!\n");
			NVT_ERR("FW_VER=0x%02X, FW_VER_BAR=0x%02X\n", *(fw_entry->data+FW_BIN_VER_OFFSET), *(fw_entry->data+FW_BIN_VER_BAR_OFFSET));
			ret = -1;
			goto invalid;
		}

		NVT_LOG("FW type is 0x%02X\n", *(fw_entry->data + FW_BIN_TYPE_OFFSET));

		/* BIN Header Parser */
		ret = nvt_bin_header_parser(fw_entry->data, fw_entry->size);
		if (ret) {
			NVT_ERR("bin header parser failed\n");
			goto invalid;
		} else {
			break;
		}

invalid:
		update_firmware_release();

request_fail:
		retry++;
		if(unlikely(retry > 2)) {
			NVT_ERR("error, retry=%d\n", retry);
			break;
		}
	}

	return ret;
}

#if NVT_DUMP_SRAM
/*******************************************************
Description:
	Novatek touchscreen dump flash partition function.

return:
	n.a.
*******************************************************/
loff_t file_offset = 0;
static void nvt_read_ram_test(uint32_t addr, uint16_t len, char *name)
{
	char file[256] = "";
	uint8_t *fbufp = NULL;
	int32_t ret = 0;
	struct file *fp = NULL;
	mm_segment_t org_fs;

	sprintf(file, "/data/local/tmp/dump_%s.bin", name);
	NVT_LOG("Dump [%s] from 0x%08X to 0x%08X\n", file, addr, addr+len);

	fbufp = (uint8_t *)kzalloc(len+1, GFP_KERNEL);
	if(fbufp == NULL) {
		NVT_ERR("kzalloc for fbufp failed!\n");
		return;
	}

	org_fs = get_fs();
	set_fs(KERNEL_DS);
	fp = filp_open(file, O_RDWR | O_CREAT, 0644);
	if (fp == NULL || IS_ERR(fp)) {
		NVT_ERR("open file failed\n");
		goto open_file_fail;
	}

	/* SPI read */
	//---set xdata index to addr---
	nvt_set_page(addr);

	fbufp[0] = addr & 0x7F;	//offset
	CTP_SPI_READ(ts->client, fbufp, len+1);

	/* Write to file */
	ret = vfs_write(fp, (char __user *)fbufp+1, len, &file_offset);
	if (ret <= 0) {
		NVT_ERR("write file failed\n");
		goto open_file_fail;
	}

open_file_fail:
	set_fs(org_fs);
	if (!IS_ERR_OR_NULL(fp)) {
		filp_close(fp, NULL);
		fp = NULL;
	}

	if (!IS_ERR_OR_NULL(fbufp)) {
		kfree(fbufp);
		fbufp = NULL;
	}

	return;
}
#endif

/*******************************************************
Description:
	Novatek touchscreen nvt_write_firmware function to write
firmware into each partition.

return:
	n.a.
*******************************************************/
static int32_t nvt_write_firmware(const u8 *fwdata, size_t fwsize)
{
	uint32_t list = 0;
	char *name;
	uint32_t BIN_addr, SRAM_addr, size;
	uint32_t i = 0;
	uint16_t len = 0;
	int32_t count = 0;
	int32_t ret = 0;

	memset(fwbuf, 0, (NVT_TANSFER_LEN+1));

	for (list = 0; list < partition; list++) {
		/* initialize variable */
		SRAM_addr = bin_map[list].SRAM_addr;
		size = bin_map[list].size;
		BIN_addr = bin_map[list].BIN_addr;
		name = bin_map[list].name;

//		NVT_LOG("[%d][%s] SRAM (0x%08X), SIZE (0x%08X), BIN (0x%08X)\n",
//				list, name, SRAM_addr, size, BIN_addr);

		/* Check data size */
		if ((BIN_addr + size) > fwsize) {
			NVT_ERR("access range (0x%08X to 0x%08X) is larger than bin size!\n",
					BIN_addr, BIN_addr + size);
			ret = -1;
			goto out;
		}

		/* ignore reserved partition (Reserved Partition size is zero) */
		if (!size)
			continue;
		else
			size = size +1;

		/* write data to SRAM */
		if (size % NVT_TANSFER_LEN)
			count = (size / NVT_TANSFER_LEN) + 1;
		else
			count = (size / NVT_TANSFER_LEN);

		for (i = 0 ; i < count ; i++) {
			len = (size < NVT_TANSFER_LEN) ? size : NVT_TANSFER_LEN;

			//---set xdata index to start address of SRAM---
			nvt_set_page(SRAM_addr);

			//---write data into SRAM---
			fwbuf[0] = SRAM_addr & 0x7F;	//offset
			memcpy(fwbuf+1, &fwdata[BIN_addr], len);	//payload
			CTP_SPI_WRITE(ts->client, fwbuf, len+1);

#if NVT_DUMP_SRAM
			/* dump for debug download firmware */
			nvt_read_ram_test(SRAM_addr, len, name);
#endif
			SRAM_addr += NVT_TANSFER_LEN;
			BIN_addr += NVT_TANSFER_LEN;
			size -= NVT_TANSFER_LEN;
		}

#if NVT_DUMP_SRAM
		file_offset = 0;
#endif
	}

out:
	return ret;
}

/*******************************************************
Description:
	Novatek touchscreen check checksum function.
This function will compare file checksum and fw checksum.

return:
	n.a.
*******************************************************/
static int32_t nvt_check_fw_checksum(void)
{
	uint32_t fw_checksum = 0;
	uint32_t len = partition*4;
	uint32_t list = 0;
	int32_t ret = 0;

	memset(fwbuf, 0, (len+1));

	//---set xdata index to checksum---
	nvt_set_page(ts->mmap->R_ILM_CHECKSUM_ADDR);

	/* read checksum */
	fwbuf[0] = (ts->mmap->R_ILM_CHECKSUM_ADDR) & 0x7F;
	ret = CTP_SPI_READ(ts->client, fwbuf, len+1);
	if (ret) {
		NVT_ERR("Read fw checksum failed\n");
		return ret;
	}

	/*
	 * Compare each checksum from fw
	 * ILM + DLM + Overlay + Info
	 * ilm_dlm_num (ILM & DLM) + ovly_sec_num + info_sec_num
	 */
	for (list = 0; list < partition; list++) {
		fw_checksum = byte_to_word(&fwbuf[1+list*4]);

		/* ignore reserved partition (Reserved Partition size is zero) */
		if(!bin_map[list].size)
			continue;

		if (bin_map[list].crc != fw_checksum) {
			NVT_ERR("[%d] BIN_checksum=0x%08X, FW_checksum=0x%08X\n",
					list, bin_map[list].crc, fw_checksum);

			NVT_ERR("firmware checksum not match!!\n");
			ret = -EIO;
			break;
		}
	}

	//---set xdata index to EVENT BUF ADDR---
	nvt_set_page(ts->mmap->EVENT_BUF_ADDR);

	return ret;
}


/*******************************************************
Description:
	Novatek touchscreen Download_Firmware function. It's
complete download firmware flow.

return:
	n.a.
*******************************************************/
static int32_t nvt_download_firmware(void)
{
	uint8_t retry = 0;
	int32_t ret = 0;

	do_gettimeofday(&start);

	while (1) {
		/*
		 * Send eng reset cmd before download FW
		 * Keep TP_RESX low when send eng reset cmd
		 */
#if NVT_TOUCH_SUPPORT_HW_RST
		gpio_set_value(ts->reset_gpio, 0);
		mdelay(1);	//wait 1ms
#endif
#ifdef ODM_WT_EDIT
		nvt_eng_reset();
#endif
#if NVT_TOUCH_SUPPORT_HW_RST
		gpio_set_value(ts->reset_gpio, 1);
		mdelay(10);	//wait tRT2BRST after TP_RST
#endif
		nvt_bootloader_reset();

		/* clear fw reset status */
		nvt_write_addr(ts->mmap->EVENT_BUF_ADDR | EVENT_MAP_RESET_COMPLETE, 0x00);

		/* Start to write firmware process */
		ret = nvt_write_firmware(fw_entry->data, fw_entry->size);
		if (ret) {
			NVT_ERR("Write_Firmware failed. (%d)\n", ret);
			goto fail;
		}

		/* Set Boot Ready Bit */
		nvt_boot_ready();

		ret = nvt_check_fw_reset_state(RESET_STATE_INIT);
		if (ret) {
			NVT_ERR("nvt_check_fw_reset_state failed. (%d)\n", ret);
			goto fail;
		}

		/* check fw checksum result */
		ret = nvt_check_fw_checksum();
		if (ret) {
			NVT_LOG("check checksum failed, retry=%d\n", retry);
			goto fail;
		} else {
			break;
		}

fail:
		retry++;
		if(unlikely(retry > 2)) {
			NVT_ERR("error, retry=%d\n", retry);
			break;
		}
	}

	do_gettimeofday(&end);

	return ret;
}
/*******************************************************
Description:
	Novatek touchscreen update firmware main function.

return:
	n.a.
*******************************************************/
void nvt_update_firmware(char *firmware_name)
{
	int8_t ret = 0;

	// request bin file in "/etc/firmware"
	ret = update_firmware_request(firmware_name);
	if (ret) {
		NVT_ERR("update_firmware_request failed. (%d)\n", ret);
		goto request_firmware_fail;
	}

	/* initial buffer and variable */
	ret = nvt_download_init();
	if (ret) {
		NVT_ERR("Download Init failed. (%d)\n", ret);
		goto init_fail;
	}

	/* download firmware process */
	ret = nvt_download_firmware();
	if (ret) {
		NVT_ERR("Download Firmware failed. (%d)\n", ret);
		goto download_fail;
	}

	NVT_LOG("Update firmware success! <%ld us>\n",
			(end.tv_sec - start.tv_sec)*1000000L + (end.tv_usec - start.tv_usec));

	/* Get FW Info */
	ret = nvt_get_fw_info();
	if (ret) {
		NVT_ERR("nvt_get_fw_info failed. (%d)\n", ret);
		goto download_fail;
	}
	if (!strcmp(firmware_name, BOOT_UPDATE_FIRMWARE_NAME)) {
		request_and_download_normal_complete = true;
	} else if (!strcmp(firmware_name, MP_UPDATE_FIRMWARE_NAME)) {
		request_and_download_mp_complete = true;
	}
download_fail:
	kfree(bin_map);
init_fail:
	update_firmware_release();
request_firmware_fail:

	return;
}

/*******************************************************
Description:
	Novatek touchscreen update firmware when booting
	function.

return:
	n.a.
*******************************************************/
void Boot_Update_Firmware(struct work_struct *work)
{
	mutex_lock(&ts->lock);
#ifdef ODM_WT_EDIT
	if ( ctpmodule == 0 ){
		printk("%s:nvt:this is tm module\n",__func__);
		nvt_update_firmware(BOOT_UPDATE_FIRMWARE_NAME_TM);
	}
	else {
		printk("%s:nvt:this is xinli module\n",__func__);
		nvt_update_firmware(BOOT_UPDATE_FIRMWARE_NAME);
	}
#endif
    mutex_unlock(&ts->lock);
}

void resume_Update_Firmware(struct work_struct *work)
{
	mutex_lock(&ts->lock);
	ts->sleep_flag = 0;
	ts->nvt_fw_updating = 1;
#if NVT_TOUCH_SUPPORT_HW_RST
	gpio_set_value(ts->reset_gpio, 1);
#endif
#ifdef ODM_WT_EDIT
	if ( ctpmodule == 0 ){
		printk("%s:nvt:this is tm module\n",__func__);
		nvt_update_firmware(BOOT_UPDATE_FIRMWARE_NAME_TM);
	}
	else {
		printk("%s:nvt:this is xinli module\n",__func__);
		nvt_update_firmware(BOOT_UPDATE_FIRMWARE_NAME);
	}
#endif
	nvt_check_fw_reset_state(RESET_STATE_REK);
	nvt_resume_tmp();
	mutex_unlock(&ts->lock);
	return;

}

#endif /* BOOT_UPDATE_FIRMWARE */
