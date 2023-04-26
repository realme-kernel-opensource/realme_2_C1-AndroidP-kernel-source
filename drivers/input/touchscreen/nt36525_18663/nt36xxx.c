/*******************************************************************************
** Copyright (C), 2008-2016, OPPO Mobile Comm Corp., Ltd.
** ODM_WT_EDIT
** FILE: - nt36xxx.c
** Description : This program is for nt36xxx driver
** Version: 1.0
** Date : 2018/5/8
**
** -------------------------Revision History:----------------------------------
**  <author>	 <data> 	<version >			<desc>
**
**
*******************************************************************************/
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/delay.h>
#include <linux/proc_fs.h>
#include <asm/uaccess.h>
#include <linux/input/mt.h>
//#include <linux/wakelock.h>
#include <linux/pm_wakeup.h>

#include <linux/of_gpio.h>
#include <linux/of_irq.h>

#if defined(CONFIG_FB)
#include <linux/notifier.h>
#include <linux/fb.h>
#elif defined(CONFIG_HAS_EARLYSUSPEND)
#include <linux/earlysuspend.h>
#endif

#include "nt36xxx.h"
#if NVT_TOUCH_ESD_PROTECT
#include <linux/jiffies.h>
#endif /* #if NVT_TOUCH_ESD_PROTECT */


#ifdef ODM_WT_EDIT
//#include <linux/hardware_info.h>
#include <linux/regulator/consumer.h>
//extern char Ctp_name[HARDWARE_MAX_ITEM_LONGTH];
extern int get_boot_mode(void);
extern int usb_state_ctp;
int novatek_tp = 0;
int ctpmodule = -1;
#endif
#ifdef ODM_WT_EDIT
extern void devinfo_info_tp_set(char *version, char *manufacture, char *fw_path);
#endif

#ifdef ODM_WT_EDIT
	static char version[20] = {"0"};
#endif


#if NVT_TOUCH_ESD_PROTECT
static struct delayed_work nvt_esd_check_work;
static struct workqueue_struct *nvt_esd_check_wq;
static unsigned long irq_timer = 0;
uint8_t esd_check = false;
uint8_t esd_retry = 0;
#endif /* #if NVT_TOUCH_ESD_PROTECT */

#if NVT_TOUCH_EXT_PROC
extern int32_t nvt_extra_proc_init(void);
#endif

#if NVT_TOUCH_MP
extern int32_t nvt_mp_proc_init(void);
#endif

struct nvt_ts_data *ts;

static struct workqueue_struct *nvt_wq;
static struct workqueue_struct *nvt_wq_use_check;


#if BOOT_UPDATE_FIRMWARE
static struct workqueue_struct *nvt_fwu_wq;
//static struct workqueue_struct *nvt_resume_wq;

extern void Boot_Update_Firmware(struct work_struct *work);
extern void resume_Update_Firmware(struct work_struct *work);

#endif

#if defined(CONFIG_FB)
static int fb_notifier_callback(struct notifier_block *self, unsigned long event, void *data);
#elif defined(CONFIG_HAS_EARLYSUSPEND)
static void nvt_ts_early_suspend(struct early_suspend *h);
static void nvt_ts_late_resume(struct early_suspend *h);
#endif

uint32_t ENG_RST_ADDR  = 0x7FFF80;
uint32_t SWRST_N8_ADDR = 0; //read from dtsi

#if TOUCH_KEY_NUM > 0
const uint16_t touch_key_array[TOUCH_KEY_NUM] = {
	KEY_BACK,
	KEY_HOME,
	KEY_MENU
};
#endif

#ifdef CONFIG_MTK_SPI
const struct mt_chip_conf spi_ctrdata = {
	.setuptime = 25,
	.holdtime = 25,
	.high_time = 5,	/* 10MHz (SPI_SPEED=100M / (high_time+low_time(10ns)))*/
	.low_time = 5,
	.cs_idletime = 2,
	.ulthgh_thrsh = 0,
	.cpol = 0,
	.cpha = 0,
	.rx_mlsb = 1,
	.tx_mlsb = 1,
	.tx_endian = 0,
	.rx_endian = 0,
	.com_mod = DMA_TRANSFER,
	.pause = 0,
	.finish_intr = 1,
	.deassert = 0,
	.ulthigh = 0,
	.tckdly = 0,
};
#endif

static uint8_t bTouchIsAwake = 0;

/*******************************************************
Description:
	Novatek touchscreen spi read/write core function.

return:
	Executive outcomes. 0---succeed.
*******************************************************/
#ifdef ODM_WT_EDIT
//static uint8_t rbuf[NVT_TANSFER_LEN+1] = {0};
static uint8_t *rbuf = NULL;
static inline void rbuf_init(void)
{
	rbuf = (uint8_t *)kzalloc((NVT_TANSFER_LEN+1),GFP_KERNEL);
	if ( rbuf == NULL ) {
		return;
	}
}
#endif
static inline int32_t spi_read_write(struct spi_device *client, uint8_t *buf, size_t len , NVT_SPI_RW rw)
{
	struct spi_message m;
	struct spi_transfer t = {
		.len    = len,
	};

	switch (rw) {
		case NVTREAD:
			t.tx_buf = &buf[0];
			t.rx_buf = rbuf;
			t.len    = (len + DUMMY_BYTES);
			break;

		case NVTWRITE:
			t.tx_buf = buf;
			break;
	}

	spi_message_init(&m);
	spi_message_add_tail(&t, &m);
	return spi_sync(client, &m);
}

/*******************************************************
Description:
	Novatek touchscreen spi read function.

return:
	Executive outcomes. 2---succeed. -5---I/O error
*******************************************************/
int32_t CTP_SPI_READ(struct spi_device *client, uint8_t *buf, uint16_t len)
{
	int32_t ret = -1;
	int32_t retries = 0;

	buf[0] = SPI_READ_MASK(buf[0]);

	while (retries < 5) {
		ret = spi_read_write(client, buf, len, NVTREAD);
		if (ret == 0) break;
		retries++;
	}

	if (unlikely(retries == 5)) {
		NVT_ERR("read error, ret=%d\n", ret);
		ret = -EIO;
	} else {
		memcpy((buf+1), (rbuf+2), (len-1));
	}

	return ret;
}

/*******************************************************
Description:
	Novatek touchscreen spi write function.

return:
	Executive outcomes. 1---succeed. -5---I/O error
*******************************************************/
int32_t CTP_SPI_WRITE(struct spi_device *client, uint8_t *buf, uint16_t len)
{
	int32_t ret = -1;
	int32_t retries = 0;

	buf[0] = SPI_WRITE_MASK(buf[0]);

	while (retries < 5) {
		ret = spi_read_write(client, buf, len, NVTWRITE);
		if (ret == 0)	break;
		retries++;
	}

	if (unlikely(retries == 5)) {
		NVT_ERR("error, ret=%d\n", ret);
		ret = -EIO;
	}

	return ret;
}

/*******************************************************
Description:
	Novatek touchscreen set index/page/addr address.

return:
	Executive outcomes. 0---succeed. -5---access fail.
*******************************************************/
int32_t nvt_set_page(uint32_t addr)
{
	uint8_t buf[4] = {0};

	buf[0] = 0xFF;	//set index/page/addr command
	buf[1] = (addr >> 15) & 0xFF;
	buf[2] = (addr >> 7) & 0xFF;

	return CTP_SPI_WRITE(ts->client, buf, 3);
}

/*******************************************************
Description:
	Novatek touchscreen write data to specify address.

return:
	Executive outcomes. 0---succeed. -5---access fail.
*******************************************************/
int32_t nvt_write_addr(uint32_t addr, uint8_t data)
{
	int32_t ret = 0;
	uint8_t buf[4] = {0};

	//---set xdata index---
	buf[0] = 0xFF;	//set index/page/addr command
	buf[1] = (addr >> 15) & 0xFF;
	buf[2] = (addr >> 7) & 0xFF;
	ret = CTP_SPI_WRITE(ts->client, buf, 3);
	if (ret) {
		NVT_ERR("set page 0x%06X failed, ret = %d\n", addr, ret);
		return ret;
	}

	//---write data to index---
	buf[0] = addr & (0x7F);
	buf[1] = data;
	ret = CTP_SPI_WRITE(ts->client, buf, 2);
	if (ret) {
		NVT_ERR("write data to 0x%06X failed, ret = %d\n", addr, ret);
		return ret;
	}

	return ret;
}

/*******************************************************
Description:
	Novatek touchscreen set boot ready function.

return:
	N/A.
*******************************************************/
void nvt_boot_ready(void)
{
	//---write BOOT_RDY status cmds---
	nvt_write_addr(ts->mmap->BOOT_RDY_ADDR, 1);

	mdelay(5);

	//---clear BOOT_RDY status cmds---
	nvt_write_addr(ts->mmap->BOOT_RDY_ADDR, 0);

	//---write POR_CD cmds---
	nvt_write_addr(ts->mmap->POR_CD_ADDR, 0xA0);
}
#ifdef ODM_WT_EDIT
/*******************************************************
Description:
	Novatek touchscreen eng reset cmd
    function.

return:
	n.a.
*******************************************************/
void nvt_eng_reset(void)
{
	//---eng reset cmds to ENG_RST_ADDR---
	nvt_write_addr(ENG_RST_ADDR, 0x5A);

	mdelay(1);	//wait tMCU_Idle2TP_REX_Hi after TP_RST
}
#endif
/*******************************************************
Description:
	Novatek touchscreen reset MCU
    function.

return:
	n.a.
*******************************************************/
void nvt_sw_reset(void)
{
	//---software reset cmds to SWRST_N8_ADDR---
	nvt_write_addr(SWRST_N8_ADDR, 0x55);

	msleep(10);
}

/*******************************************************
Description:
	Novatek touchscreen reset MCU then into idle mode
    function.

return:
	n.a.
*******************************************************/
void nvt_sw_reset_idle(void)
{
	//---MCU idle cmds to SWRST_N8_ADDR---
	nvt_write_addr(SWRST_N8_ADDR, 0xAA);

	msleep(15);
}

/*******************************************************
Description:
	Novatek touchscreen reset MCU (boot) function.

return:
	n.a.
*******************************************************/
void nvt_bootloader_reset(void)
{
	//---reset cmds to SWRST_N8_ADDR---
	nvt_write_addr(SWRST_N8_ADDR, 0x69);

	mdelay(5);	//wait tBRST2FR after Bootload RST
}

/*******************************************************
Description:
	Novatek touchscreen clear FW status function.

return:
	Executive outcomes. 0---succeed. -1---fail.
*******************************************************/
int32_t nvt_clear_fw_status(void)
{
	uint8_t buf[8] = {0};
	int32_t i = 0;
	const int32_t retry = 20;

	for (i = 0; i < retry; i++) {
		//---set xdata index to EVENT BUF ADDR---
		nvt_set_page(ts->mmap->EVENT_BUF_ADDR | EVENT_MAP_HANDSHAKING_or_SUB_CMD_BYTE);

		//---clear fw status---
		buf[0] = EVENT_MAP_HANDSHAKING_or_SUB_CMD_BYTE;
		buf[1] = 0x00;
		CTP_SPI_WRITE(ts->client, buf, 2);

		//---read fw status---
		buf[0] = EVENT_MAP_HANDSHAKING_or_SUB_CMD_BYTE;
		buf[1] = 0xFF;
		CTP_SPI_READ(ts->client, buf, 2);

		if (buf[1] == 0x00)
			break;

		msleep(10);
	}

	if (i >= retry) {
		NVT_ERR("failed, i=%d, buf[1]=0x%02X\n", i, buf[1]);
		return -1;
	} else {
		return 0;
	}
}


void nvt_irq_enable(bool enable)
{
	unsigned long nIrqFlag;
	spin_lock_irqsave(&ts->spinlock_int, nIrqFlag);

	if (enable == 1 && ts->irq_enable_flag == 0) {
		enable_irq(ts->client->irq);
		ts->irq_enable_time++;
		ts->irq_enable_flag = 1;
	} else if (enable == 0 && ts->irq_enable_flag == 1) {
		disable_irq_nosync(ts->client->irq);
		ts->irq_enable_time--;
		ts->irq_enable_flag = 0;
	}

	//NVT_LOG("ts->irq_enable_time = %d,ts->irq_enable_flag = %d\n", ts->irq_enable_time,ts->irq_enable_flag);
	spin_unlock_irqrestore(&ts->spinlock_int, nIrqFlag);
}



/*******************************************************
Description:
	Novatek touchscreen check FW status function.

return:
	Executive outcomes. 0---succeed. -1---failed.
*******************************************************/
int32_t nvt_check_fw_status(void)
{
	uint8_t buf[8] = {0};
	int32_t i = 0;
	const int32_t retry = 50;

	for (i = 0; i < retry; i++) {
		//---set xdata index to EVENT BUF ADDR---
		nvt_set_page(ts->mmap->EVENT_BUF_ADDR | EVENT_MAP_HANDSHAKING_or_SUB_CMD_BYTE);

		//---read fw status---
		buf[0] = EVENT_MAP_HANDSHAKING_or_SUB_CMD_BYTE;
		buf[1] = 0x00;
		CTP_SPI_READ(ts->client, buf, 2);

		if ((buf[1] & 0xF0) == 0xA0)
			break;

		msleep(10);
	}

	if (i >= retry) {
		NVT_ERR("failed, i=%d, buf[1]=0x%02X\n", i, buf[1]);
		return -1;
	} else {
		return 0;
	}
}

/*******************************************************
Description:
	Novatek touchscreen check FW reset state function.

return:
	Executive outcomes. 0---succeed. -1---failed.
*******************************************************/
int32_t nvt_check_fw_reset_state(RST_COMPLETE_STATE check_reset_state)
{
	uint8_t buf[8] = {0};
	int32_t ret = 0;
	int32_t retry = 0;
	int32_t retry_max = check_reset_state == RESET_STATE_INIT ? 10 : 50;

	//---set xdata index to EVENT BUF ADDR---
	nvt_set_page(ts->mmap->EVENT_BUF_ADDR | EVENT_MAP_RESET_COMPLETE);

	while (1) {
		//---read reset state---
		buf[0] = EVENT_MAP_RESET_COMPLETE;
		buf[1] = 0x00;
		CTP_SPI_READ(ts->client, buf, 6);

		if ((buf[1] >= check_reset_state) && (buf[1] <= RESET_STATE_MAX)) {
			ret = 0;
			break;
		}

		retry++;
		if(unlikely(retry > retry_max)) {
			NVT_ERR("error, retry=%d, buf[1]=0x%02X, 0x%02X, 0x%02X, 0x%02X, 0x%02X\n", retry, buf[1], buf[2], buf[3], buf[4], buf[5]);
			ret = -1;
			break;
		}

		msleep(10);
	}

	return ret;
}

/*******************************************************
Description:
	Novatek touchscreen get novatek project id information
	function.

return:
	Executive outcomes. 0---success. -1---fail.
*******************************************************/
int32_t nvt_read_pid(void)
{
	uint8_t buf[3] = {0};
	int32_t ret = 0;

	//---set xdata index to EVENT BUF ADDR---
	nvt_set_page(ts->mmap->EVENT_BUF_ADDR | EVENT_MAP_PROJECTID);

	//---read project id---
	buf[0] = EVENT_MAP_PROJECTID;
	buf[1] = 0x00;
	buf[2] = 0x00;
	CTP_SPI_READ(ts->client, buf, 3);

	ts->nvt_pid = (buf[2] << 8) + buf[1];

	//---set xdata index to EVENT BUF ADDR---
	nvt_set_page(ts->mmap->EVENT_BUF_ADDR);

	NVT_LOG("PID=%04X\n", ts->nvt_pid);

	return ret;
}

/*******************************************************
Description:
	Novatek touchscreen get firmware related information
	function.

return:
	Executive outcomes. 0---success. -1---fail.
*******************************************************/
int32_t nvt_get_fw_info(void)
{
	uint8_t buf[64] = {0};
	uint32_t retry_count = 0;
	int32_t ret = 0;
info_retry:
	//---set xdata index to EVENT BUF ADDR---
	nvt_set_page(ts->mmap->EVENT_BUF_ADDR | EVENT_MAP_FWINFO);

	//---read fw info---
	buf[0] = EVENT_MAP_FWINFO;
	CTP_SPI_READ(ts->client, buf, 17);
	ts->fw_ver = buf[1];
	ts->x_num = buf[3];
	ts->y_num = buf[4];
	ts->abs_x_max = (uint16_t)((buf[5] << 8) | buf[6]);
	ts->abs_y_max = (uint16_t)((buf[7] << 8) | buf[8]);
	ts->max_button_num = buf[11];

	//---clear x_num, y_num if fw info is broken---
	if ((buf[1] + buf[2]) != 0xFF) {
		NVT_ERR("FW info is broken! fw_ver=0x%02X, ~fw_ver=0x%02X\n", buf[1], buf[2]);
		ts->fw_ver = 0;
		ts->x_num = 18;
		ts->y_num = 32;
		ts->abs_x_max = TOUCH_DEFAULT_MAX_WIDTH;
		ts->abs_y_max = TOUCH_DEFAULT_MAX_HEIGHT;
		ts->max_button_num = TOUCH_KEY_NUM;

		if(retry_count < 3) {
			retry_count++;
			NVT_ERR("retry_count=%d\n", retry_count);
			goto info_retry;
		} else {
			NVT_ERR("Set default fw_ver=%d, x_num=%d, y_num=%d, \
					abs_x_max=%d, abs_y_max=%d, max_button_num=%d!\n",
					ts->fw_ver, ts->x_num, ts->y_num,
					ts->abs_x_max, ts->abs_y_max, ts->max_button_num);
			ret = -1;
		}
	} else {
		NVT_LOG("fw_ver=%02X\n", ts->fw_ver);
		ret = 0;
	}
#ifdef ODM_WT_EDIT
	if ( ctpmodule == 0 ){
//		sprintf(Ctp_name,"NT36525,TianMa,FW:0x%02x\n",ts->fw_ver);
	}
	else{
//		sprintf(Ctp_name,"NT36525,XinLi,FW:0x%02x\n",ts->fw_ver);
	}
#endif
#ifdef ODM_WT_EDIT
	if ( ctpmodule == 0 ){
	    sprintf(version,"TM_nt525_0x%02x",ts->fw_ver);
	    devinfo_info_tp_set(version, "TIANMA",OPPO_BOOT_UPDATE_FIRMWARE_NAME_TM);
	} else {
        sprintf(version,"XL_nt525_0x%02x",ts->fw_ver);
	    devinfo_info_tp_set(version, "XINLI",OPPO_BOOT_UPDATE_FIRMWARE_NAME);
	}
#endif
	//---Get Novatek PID---
	nvt_read_pid();

	return ret;
}

/*******************************************************
  Create Device Node (Proc Entry)
*******************************************************/
#if NVT_TOUCH_PROC
static struct proc_dir_entry *NVT_proc_entry;
#define DEVICE_NAME	"NVTSPI"

/*******************************************************
Description:
	Novatek touchscreen /proc/NVTSPI read function.

return:
	Executive outcomes. 2---succeed. -5,-14---failed.
*******************************************************/
static ssize_t nvt_flash_read(struct file *file, char __user *buff, size_t count, loff_t *offp)
{
	uint8_t *str = NULL;
	int32_t ret = 0;
	int32_t retries = 0;
	int8_t spi_wr = 0;

	if (count > NVT_TANSFER_LEN) {
		NVT_ERR("invalid transfer len!\n");
		return -EFAULT;
	}

	/* allocate buffer for spi transfer */
	str = (uint8_t *)kzalloc((count), GFP_KERNEL);
	if(str == NULL) {
		NVT_ERR("kzalloc for buf failed!\n");
		ret = -ENOMEM;
		goto kzalloc_failed;
	}

	if (copy_from_user(str, buff, count)) {
		NVT_ERR("copy from user error\n");
		ret = -EFAULT;
		goto out;
	}

#if NVT_TOUCH_ESD_PROTECT
	/*
	 * stop esd check work to avoid case that 0x77 report righ after here to enable esd check again
	 * finally lead to trigger esd recovery bootloader reset
	 */
	cancel_delayed_work_sync(&nvt_esd_check_work);
	nvt_esd_check_enable(false);
#endif /* #if NVT_TOUCH_ESD_PROTECT */

	spi_wr = str[0] >> 7;

	if (spi_wr == NVTWRITE) {	//SPI write
		while (retries < 20) {
			ret = CTP_SPI_WRITE(ts->client, &str[2], ((str[0] & 0x7F) << 8) | str[1]);
			if (!ret)
				break;
			else
				NVT_ERR("error, retries=%d, ret=%d\n", retries, ret);

			retries++;
		}

		if (unlikely(retries == 20)) {
			NVT_ERR("error, ret = %d\n", ret);
			ret = -EIO;
			goto out;
		}
	} else if (spi_wr == NVTREAD) {	//SPI read
		while (retries < 20) {
			ret = CTP_SPI_READ(ts->client, &str[2], ((str[0] & 0x7F) << 8) | str[1]);
			if (!ret)
				break;
			else
				NVT_ERR("error, retries=%d, ret=%d\n", retries, ret);

			retries++;
		}

		// copy buff to user if spi transfer
		if (retries < 20) {
			if (copy_to_user(buff, str, count)) {
				ret = -EFAULT;
				goto out;
			}
		}

		if (unlikely(retries == 20)) {
			NVT_ERR("error, ret = %d\n", ret);
			ret = -EIO;
			goto out;
		}
	} else {
		NVT_ERR("Call error, str[0]=%d\n", str[0]);
		ret = -EFAULT;
		goto out;
	}

out:
	kfree(str);
kzalloc_failed:
	return ret;
}

/*******************************************************
Description:
	Novatek touchscreen /proc/NVTSPI open function.

return:
	Executive outcomes. 0---succeed. -12---failed.
*******************************************************/
static int32_t nvt_flash_open(struct inode *inode, struct file *file)
{
	struct nvt_flash_data *dev;

	dev = kmalloc(sizeof(struct nvt_flash_data), GFP_KERNEL);
	if (dev == NULL) {
		NVT_ERR("Failed to allocate memory for nvt flash data\n");
		return -ENOMEM;
	}

	rwlock_init(&dev->lock);
	file->private_data = dev;

	return 0;
}

/*******************************************************
Description:
	Novatek touchscreen /proc/NVTSPI close function.

return:
	Executive outcomes. 0---succeed.
*******************************************************/
static int32_t nvt_flash_close(struct inode *inode, struct file *file)
{
	struct nvt_flash_data *dev = file->private_data;

	if (dev)
		kfree(dev);

	return 0;
}

static const struct file_operations nvt_flash_fops = {
	.owner = THIS_MODULE,
	.open = nvt_flash_open,
	.release = nvt_flash_close,
	.read = nvt_flash_read,
};

/*******************************************************
Description:
	Novatek touchscreen /proc/NVTSPI initial function.

return:
	Executive outcomes. 0---succeed. -12---failed.
*******************************************************/
static int32_t nvt_flash_proc_init(void)
{
	NVT_proc_entry = proc_create(DEVICE_NAME, 0444, NULL,&nvt_flash_fops);
	if (NVT_proc_entry == NULL) {
		NVT_ERR("Failed!\n");
		return -ENOMEM;
	} else {
		NVT_LOG("Succeeded!\n");
	}

	NVT_LOG("============================================================\n");
	NVT_LOG("Create /proc/NVTSPI\n");
	NVT_LOG("============================================================\n");

	return 0;
}
#endif

#if WAKEUP_GESTURE
#ifdef ODM_WT_EDIT
enum {	/* oppo gesture type */
	UnkownGesture = 0,
	DouTap        = 1,
	UpVee,
	DownVee,
	LeftVee,	//>
	RightVee,	//<
	Circle,
	DouSwip,
	Right2LeftSwip,
	Left2RightSwip,
	Up2DownSwip,
	Down2UpSwip,
	Mgestrue,
	Wgestrue,
};

#define W_DETECT                13
#define UP_VEE_DETECT           14
#define DTAP_DETECT             15
#define M_DETECT                17
#define CIRCLE_DETECT           18
#define UP_SLIDE_DETECT         21
#define DOWN_SLIDE_DETECT       22
#define LEFT_SLIDE_DETECT       23
#define RIGHT_SLIDE_DETECT      24
#define LEFT_VEE_DETECT         31	//>
#define RIGHT_VEE_DETECT        32	//<
#define DOWN_VEE_DETECT         33
#define DOUSWIP_DETECT          34
#endif
/* customized gesture id */
#define DATA_PROTOCOL           30

/* function page definition */
#define FUNCPAGE_GESTURE         1

//static struct wake_lock gestrue_wakelock;

/*******************************************************
Description:
	Novatek touchscreen wake up gesture key report function.

return:
	n.a.
*******************************************************/
#ifdef ODM_WT_EDIT
void nvt_ts_wakeup_gesture_report(uint8_t gesture_id, uint8_t *data, struct gesture_info *gesture)
{
	uint32_t keycode = 0;
	uint8_t func_type = data[2];
	uint8_t func_id = data[3];

	/* support fw specifal data protocol */
	if ((gesture_id == DATA_PROTOCOL) && (func_type == FUNCPAGE_GESTURE)) {
		gesture_id = func_id;
	} else if (gesture_id > DATA_PROTOCOL) {
		NVT_ERR("gesture_id %d is invalid, func_type=%d, func_id=%d\n", gesture_id, func_type, func_id);
		return;
	}

	NVT_LOG("gesture_id = %d\n", gesture_id);
	gesture->clockwise = 1;	//set default clockwise is 1.

	switch (gesture_id) {
		case RIGHT_SLIDE_DETECT :
			//gesture->gesture_type  = Left2RightSwip;
			gesture->gesture_type  = Right2LeftSwip;
			gesture->Point_start.x = (data[4] & 0xFF) | (data[5] & 0x0F) << 8;
			gesture->Point_start.y = (data[6] & 0xFF) | (data[7] & 0x0F) << 8;
			gesture->Point_end.x   = (data[8] & 0xFF) | (data[9] & 0x0F) << 8;
			gesture->Point_end.y   = (data[10] & 0xFF) | (data[11] & 0x0F) << 8;
			break;

		case LEFT_SLIDE_DETECT :
			//gesture->gesture_type  = Right2LeftSwip;
			gesture->gesture_type  = Left2RightSwip;
			gesture->Point_start.x = (data[4] & 0xFF) | (data[5] & 0x0F) << 8;
			gesture->Point_start.y = (data[6] & 0xFF) | (data[7] & 0x0F) << 8;
			gesture->Point_end.x   = (data[8] & 0xFF) | (data[9] & 0x0F) << 8;
			gesture->Point_end.y   = (data[10] & 0xFF) | (data[11] & 0x0F) << 8;
			break;

		case DOWN_SLIDE_DETECT  :
			gesture->gesture_type  = Up2DownSwip;
			gesture->Point_start.x = (data[4] & 0xFF) | (data[5] & 0x0F) << 8;
			gesture->Point_start.y = (data[6] & 0xFF) | (data[7] & 0x0F) << 8;
			gesture->Point_end.x   = (data[8] & 0xFF) | (data[9] & 0x0F) << 8;
			gesture->Point_end.y   = (data[10] & 0xFF) | (data[11] & 0x0F) << 8;
			break;

		case UP_SLIDE_DETECT :
			gesture->gesture_type  = Down2UpSwip;
			gesture->Point_start.x = (data[4] & 0xFF) | (data[5] & 0x0F) << 8;
			gesture->Point_start.y = (data[6] & 0xFF) | (data[7] & 0x0F) << 8;
			gesture->Point_end.x   = (data[8] & 0xFF) | (data[9] & 0x0F) << 8;
			gesture->Point_end.y   = (data[10] & 0xFF) | (data[11] & 0x0F) << 8;
			break;

		case DTAP_DETECT:
			gesture->gesture_type  = DouTap;
			gesture->Point_start.x = (data[4] & 0xFF) | (data[5] & 0x0F) << 8;
			gesture->Point_start.y = (data[6] & 0xFF) | (data[7] & 0x0F) << 8;
			gesture->Point_end     = gesture->Point_start;
			break;

		case UP_VEE_DETECT :
			gesture->gesture_type  = UpVee;
			gesture->Point_start.x = (data[4] & 0xFF) | (data[5] & 0x0F) << 8;
			gesture->Point_start.y = (data[6] & 0xFF) | (data[7] & 0x0F) << 8;
			gesture->Point_end.x   = (data[12] & 0xFF) | (data[13] & 0x0F) << 8;
			gesture->Point_end.y   = (data[14] & 0xFF) | (data[15] & 0x0F) << 8;
			gesture->Point_1st.x   = (data[8] & 0xFF) | (data[9] & 0x0F) << 8;
			gesture->Point_1st.y   = (data[10] & 0xFF) | (data[11] & 0x0F) << 8;
			break;

		case DOWN_VEE_DETECT :
			gesture->gesture_type  = DownVee;
			gesture->Point_start.x = (data[4] & 0xFF) | (data[5] & 0x0F) << 8;
			gesture->Point_start.y = (data[6] & 0xFF) | (data[7] & 0x0F) << 8;
			gesture->Point_end.x   = (data[12] & 0xFF) | (data[13] & 0x0F) << 8;
			gesture->Point_end.y   = (data[14] & 0xFF) | (data[15] & 0x0F) << 8;
			gesture->Point_1st.x   = (data[8] & 0xFF) | (data[9] & 0x0F) << 8;
			gesture->Point_1st.y   = (data[10] & 0xFF) | (data[11] & 0x0F) << 8;
			break;

		case LEFT_VEE_DETECT:
			gesture->gesture_type = LeftVee;
			gesture->Point_start.x = (data[4] & 0xFF) | (data[5] & 0x0F) << 8;
			gesture->Point_start.y = (data[6] & 0xFF) | (data[7] & 0x0F) << 8;
			gesture->Point_end.x   = (data[12] & 0xFF) | (data[13] & 0x0F) << 8;
			gesture->Point_end.y   = (data[14] & 0xFF) | (data[15] & 0x0F) << 8;
			gesture->Point_1st.x   = (data[8] & 0xFF) | (data[9] & 0x0F) << 8;
			gesture->Point_1st.y   = (data[10] & 0xFF) | (data[11] & 0x0F) << 8;
			break;

		case RIGHT_VEE_DETECT:
			gesture->gesture_type  = RightVee;
			gesture->Point_start.x = (data[4] & 0xFF) | (data[5] & 0x0F) << 8;
			gesture->Point_start.y = (data[6] & 0xFF) | (data[7] & 0x0F) << 8;
			gesture->Point_end.x   = (data[12] & 0xFF) | (data[13] & 0x0F) << 8;
			gesture->Point_end.y   = (data[14] & 0xFF) | (data[15] & 0x0F) << 8;
			gesture->Point_1st.x   = (data[8] & 0xFF) | (data[9] & 0x0F) << 8;
			gesture->Point_1st.y   = (data[10] & 0xFF) | (data[11] & 0x0F) << 8;
			break;

		case CIRCLE_DETECT:
			gesture->gesture_type = Circle;
			gesture->clockwise = (data[43] == 0x20) ? 1 : 0;
			gesture->Point_start.x = (data[4] & 0xFF) | (data[5] & 0x0F) << 8;
			gesture->Point_start.y = (data[6] & 0xFF) | (data[7] & 0x0F) << 8;
			gesture->Point_1st.x   = (data[8] & 0xFF) | (data[9] & 0x0F) << 8;    //ymin
			gesture->Point_1st.y   = (data[10] & 0xFF) | (data[11] & 0x0F) << 8;
			gesture->Point_2nd.x   = (data[12] & 0xFF) | (data[13] & 0x0F) << 8;  //xmin
			gesture->Point_2nd.y   = (data[14] & 0xFF) | (data[15] & 0x0F) << 8;
			gesture->Point_3rd.x   = (data[16] & 0xFF) | (data[17] & 0x0F) << 8;  //ymax
			gesture->Point_3rd.y   = (data[18] & 0xFF) | (data[19] & 0x0F) << 8;
			gesture->Point_4th.x   = (data[20] & 0xFF) | (data[21] & 0x0F) << 8;  //xmax
			gesture->Point_4th.y   = (data[22] & 0xFF) | (data[23] & 0x0F) << 8;
			gesture->Point_end.x   = (data[24] & 0xFF) | (data[25] & 0x0F) << 8;
			gesture->Point_end.y   = (data[26] & 0xFF) | (data[27] & 0x0F) << 8;
			break;

		case DOUSWIP_DETECT:
			gesture->gesture_type  = DouSwip;
			gesture->Point_start.x = (data[4] & 0xFF) | (data[5] & 0x0F) << 8;
			gesture->Point_start.y = (data[6] & 0xFF) | (data[7] & 0x0F) << 8;
			gesture->Point_end.x   = (data[12] & 0xFF) | (data[13] & 0x0F) << 8;
			gesture->Point_end.y   = (data[14] & 0xFF) | (data[15] & 0x0F) << 8;
			gesture->Point_1st.x   = (data[8] & 0xFF) | (data[9] & 0x0F) << 8;
			gesture->Point_1st.y   = (data[10] & 0xFF) | (data[11] & 0x0F) << 8;
			gesture->Point_2nd.x   = (data[16] & 0xFF) | (data[17] & 0x0F) << 8;
			gesture->Point_2nd.y   = (data[18] & 0xFF) | (data[19] & 0x0F) << 8;
			break;

		case M_DETECT:
			gesture->gesture_type  = Mgestrue;
			gesture->Point_start.x = (data[4] & 0xFF) | (data[5] & 0x0F) << 8;
			gesture->Point_start.y = (data[6] & 0xFF) | (data[7] & 0x0F) << 8;
			gesture->Point_1st.x   = (data[8] & 0xFF) | (data[9] & 0x0F) << 8;
			gesture->Point_1st.y   = (data[10] & 0xFF) | (data[11] & 0x0F) << 8;
			gesture->Point_2nd.x   = (data[12] & 0xFF) | (data[13] & 0x0F) << 8;
			gesture->Point_2nd.y   = (data[14] & 0xFF) | (data[15] & 0x0F) << 8;
			gesture->Point_3rd.x   = (data[16] & 0xFF) | (data[17] & 0x0F) << 8;
			gesture->Point_3rd.y   = (data[18] & 0xFF) | (data[19] & 0x0F) << 8;
			gesture->Point_end.x   = (data[20] & 0xFF) | (data[21] & 0x0F) << 8;
			gesture->Point_end.y   = (data[22] & 0xFF) | (data[23] & 0x0F) << 8;
			break;

		case W_DETECT:
			gesture->gesture_type  = Wgestrue;
			gesture->Point_start.x = (data[4] & 0xFF) | (data[5] & 0x0F) << 8;
			gesture->Point_start.y = (data[6] & 0xFF) | (data[7] & 0x0F) << 8;
			gesture->Point_1st.x   = (data[8] & 0xFF) | (data[9] & 0x0F) << 8;
			gesture->Point_1st.y   = (data[10] & 0xFF) | (data[11] & 0x0F) << 8;
			gesture->Point_2nd.x   = (data[12] & 0xFF) | (data[13] & 0x0F) << 8;
			gesture->Point_2nd.y   = (data[14] & 0xFF) | (data[15] & 0x0F) << 8;
			gesture->Point_3rd.x   = (data[16] & 0xFF) | (data[17] & 0x0F) << 8;
			gesture->Point_3rd.y   = (data[18] & 0xFF) | (data[19] & 0x0F) << 8;
			gesture->Point_end.x   = (data[20] & 0xFF) | (data[21] & 0x0F) << 8;
			gesture->Point_end.y   = (data[22] & 0xFF) | (data[23] & 0x0F) << 8;
			break;

		default:
			gesture->gesture_type = UnkownGesture;
			break;
	}

	NVT_LOG("gesture_id: 0x%x, func_type: 0x%x, gesture_type: %d, clockwise: %d, points: (%d, %d)(%d, %d) (%d, %d)(%d, %d)(%d, %d)(%d, %d)\n",
			gesture_id, func_type, gesture->gesture_type, gesture->clockwise,
			gesture->Point_start.x, gesture->Point_start.y,
			gesture->Point_end.x, gesture->Point_end.y,
			gesture->Point_1st.x, gesture->Point_1st.y,
			gesture->Point_2nd.x, gesture->Point_2nd.y,
			gesture->Point_3rd.x, gesture->Point_3rd.y,
			gesture->Point_4th.x, gesture->Point_4th.y);

	if (gesture->gesture_type != UnkownGesture) {
		keycode = KEY_F4;
#endif
		input_report_key(ts->input_dev, keycode, 1);
		input_sync(ts->input_dev);
		input_report_key(ts->input_dev, keycode, 0);
		input_sync(ts->input_dev);
	}
}
#endif
/*******************************************************
Description:
	Novatek touchscreen parse device tree function.

return:
	n.a.
*******************************************************/
#ifdef CONFIG_OF
static int32_t nvt_parse_dt(struct device *dev)
{
	struct device_node *np = dev->of_node;
	int32_t ret = 0;
	ts->lcd_reset_gpio = of_get_named_gpio_flags(np, "share,lcd_reset-gpio", 0, &ts->lcd_reset_flags);
	NVT_LOG("novatek,lcd reset-gpio=%d\n", ts->lcd_reset_gpio);
	ts->cs_gpio = of_get_named_gpio_flags(np, "share,cs-gpio", 0, &ts->cs_flags);
	NVT_LOG("share,cs-gpio=%d\n", ts->cs_flags);

#if NVT_TOUCH_SUPPORT_HW_RST
	ts->reset_gpio = of_get_named_gpio_flags(np, "novatek,reset-gpio", 0, &ts->reset_flags);
	NVT_LOG("novatek,reset-gpio=%d\n", ts->reset_gpio);
#endif
	ts->irq_gpio = of_get_named_gpio_flags(np, "novatek,irq-gpio", 0, &ts->irq_flags);
	NVT_LOG("novatek,irq-gpio=%d\n", ts->irq_gpio);

	ret = of_property_read_u32(np, "novatek,swrst-n8-addr", &SWRST_N8_ADDR);
	if (ret) {
		NVT_ERR("error reading novatek,swrst-n8-addr. ret=%d\n", ret);
		return ret;
	} else {
		NVT_LOG("SWRST_N8_ADDR=0x%06X\n", SWRST_N8_ADDR);
	}

	return ret;
}
#else
static int32_t nvt_parse_dt(struct device *dev)
{
#if NVT_TOUCH_SUPPORT_HW_RST
	ts->reset_gpio = NVTTOUCH_RST_PIN;
#endif
	ts->irq_gpio = NVTTOUCH_INT_PIN;
	return 0;
}
#endif

/*******************************************************
Description:
	Novatek touchscreen config and request gpio

return:
	Executive outcomes. 0---succeed. not 0---failed.
*******************************************************/
static int nvt_gpio_config(struct nvt_ts_data *ts)
{
	int32_t ret = 0;

#if NVT_TOUCH_SUPPORT_HW_RST
	/* request RST-pin (Output/High) */
	if (gpio_is_valid(ts->reset_gpio)) {
		ret = gpio_request_one(ts->reset_gpio, GPIOF_OUT_INIT_LOW, "NVT-tp-rst");
		if (ret) {
			NVT_ERR("Failed to request NVT-tp-rst GPIO\n");
			goto err_request_reset_gpio;
		}
	}
#endif

	/* request INT-pin (Input) */
	if (gpio_is_valid(ts->irq_gpio)) {
		ret = gpio_request_one(ts->irq_gpio, GPIOF_IN, "NVT-int");
		if (ret) {
			NVT_ERR("Failed to request NVT-int GPIO\n");
			goto err_request_irq_gpio;
		}
	}

	return ret;

err_request_irq_gpio:
#if NVT_TOUCH_SUPPORT_HW_RST
	gpio_free(ts->reset_gpio);
err_request_reset_gpio:
#endif
	return ret;
}

static int32_t nvt_cmd_store(uint8_t cmd)
{
    int32_t i, retry = 5;
    uint8_t buf[3] = {0};

    //---set xdata index to EVENT BUF ADDR---
	nvt_set_page(ts->mmap->EVENT_BUF_ADDR);

    for (i = 0; i < retry; i++) {
        //---set cmd status---
        buf[0] = EVENT_MAP_HOST_CMD;
        buf[1] = cmd;
        CTP_SPI_WRITE(ts->client, buf, 2);

        msleep(20);

        //---read cmd status---
        buf[0] = EVENT_MAP_HOST_CMD;
        buf[1] = 0xFF;
        CTP_SPI_READ(ts->client, buf, 2);
        if (buf[1] == 0x00)
            break;
    }

    if (i == retry) {
        NVT_ERR("send Cmd 0x%02X failed, buf[1]=0x%02X\n", cmd, buf[1]);
        return -1;
    } else {
        NVT_LOG("send Cmd 0x%02X success, tried %d times\n", cmd, i);
    }

    return 0;
}

static int32_t nvt_enable_edge_limit(bool enable)
{
    int32_t ret = -1;

    NVT_LOG("%s:enable = %d\n", __func__, enable);

    if (enable)
        ret = nvt_cmd_store(HOST_CMD_EDGE_LIMIT_ON);
    else
        ret = nvt_cmd_store(HOST_CMD_EDGE_LIMIT_OFF);

    return ret;
}

static int32_t nvt_enable_charge_mode(bool enable)
{
    int32_t ret = -1;

    NVT_LOG("%s:enable = %d\n", __func__, enable);

    if (enable)
        ret = nvt_cmd_store(HOST_CMD_PWR_PLUG_IN);
    else
        ret = nvt_cmd_store(HOST_CMD_PWR_PLUG_OUT);

    return ret;
}

static int32_t nvt_enable_jitter_mode(bool enable)
{
    int32_t ret = -1;

    NVT_LOG("%s:enable = %d\n", __func__, enable);

    if (enable)
        ret = nvt_cmd_store(HOST_CMD_JITTER_ON);
    else
        ret = nvt_cmd_store(HOST_CMD_JITTER_OFF);

    return ret;
}

int32_t nvt_mode_switch(NVT_CUSTOMIZED_MODE mode, uint8_t flag)
{
	int32_t ret = -1;

	switch(mode) {
		case MODE_EDGE:
			ret = nvt_enable_edge_limit(flag);
			if (ret < 0) {
				NVT_ERR("%s: nvt enable edg limit failed.\n", __func__);
				return ret;
			}
			break;

		case MODE_CHARGE:
			ret = nvt_enable_charge_mode(flag);
			if (ret < 0) {
				NVT_ERR("%s: enable charge mode : %d failed\n", __func__, flag);
			}
			break;

		case MODE_GAME:
			ret = nvt_enable_jitter_mode(flag);
			break;

		default:
			NVT_ERR("%s: Wrong mode %d.\n", __func__, mode);
	}

	return ret;
}

#if NVT_TOUCH_ESD_PROTECT
void nvt_esd_check_enable(uint8_t enable)
{
	/* update interrupt timer */
	irq_timer = jiffies;
	/* clear esd_retry counter, if protect function is enabled */
	esd_retry = enable ? 0 : esd_retry;
	/* enable/disable esd check flag */
	esd_check = enable;
}

static uint8_t nvt_fw_recovery(uint8_t *point_data)
{
	uint8_t i = 0;
	uint8_t detected = true;

	/* check pattern */
	for (i=1 ; i<7 ; i++) {
		if (point_data[i] != 0x77) {
			detected = false;
			break;
		}
	}

	return detected;
}

static void nvt_esd_check_func(struct work_struct *work)
{
	unsigned int timer = jiffies_to_msecs(jiffies - irq_timer);

	//NVT_LOG("esd_check = %d (retry %d)\n", esd_check, esd_retry);	//DEBUG

	if ((timer > NVT_TOUCH_ESD_CHECK_PERIOD) && esd_check) {
		mutex_lock(&ts->lock);
		NVT_ERR("do ESD recovery, timer = %d, retry = %d\n", timer, esd_retry);
		/* do esd recovery, reload fw */
#ifdef ODM_WT_EDIT
		if (  ctpmodule == 0 ){
			nvt_update_firmware(BOOT_UPDATE_FIRMWARE_NAME_TM);
		}
		else{
			nvt_update_firmware(BOOT_UPDATE_FIRMWARE_NAME);
		}
		 mutex_unlock(&ts->lock);
#endif
		/* update interrupt timer */
		irq_timer = jiffies;
		/* update esd_retry counter */
		esd_retry++;
	}

	queue_delayed_work(nvt_esd_check_wq, &nvt_esd_check_work,
			msecs_to_jiffies(NVT_TOUCH_ESD_CHECK_PERIOD));
}
#endif /* #if NVT_TOUCH_ESD_PROTECT */

static bool nvt_corner_point_process(int i)
{
    int j;
	int flag = 0;
	//printk("ts->edge_limit.limit_00 = %d\n",ts->edge_limit.limit_00);
	if (ts->edge_limit.limit_00 == 0) {
		//½Úµã/proc/touchpanel/oppo_tp_limit_enableµÄbit1Î»À´¿ØÖÆ¿ØÖÆ
		if ((ts->edge_limit.limit_lu) && (ts->nvt_point_info[i].x < ts->nvt_limit_area.area_xlu && ts->nvt_point_info[i].y < ts->nvt_limit_area.area_ylu)) {
			//printk("1  ts->nvt_point_info[i].x = %d,ts->nvt_point_info[i].y = %d\n",ts->nvt_point_info[i].x,ts->nvt_point_info[i].y);
		    ts->nvt_point_info[i].type  = NVT_AREA_CORNER;
			if (ts->nvt_limit_area.which_area == NVT_AREA_NORMAL)
				return true;
		    ts->nvt_coner_info[CORNER_TOPLEFT].id = i;
		    ts->nvt_coner_info[CORNER_TOPLEFT].point_info = ts->nvt_point_info[i];
		    ts->nvt_coner_info[CORNER_TOPLEFT].flag = true;
			ts->nvt_limit_area.which_area = ts->nvt_point_info[i].type;
			flag = 1;

        }
         //½Úµã/proc/touchpanel/oppo_tp_limit_enableµÄbit2Î»À´¿ØÖÆ¿ØÖÆ
        if ((ts->edge_limit.limit_ru)  && (ts->nvt_point_info[i].x > ts->nvt_limit_area.area_xru && ts->nvt_point_info[i].y < ts->nvt_limit_area.area_yru)) {
			//printk("2  ts->nvt_point_info[i].x = %d,ts->nvt_point_info[i].y = %d\n",ts->nvt_point_info[i].x,ts->nvt_point_info[i].y);
			ts->nvt_point_info[i].type  = NVT_AREA_CORNER;
			if (ts->nvt_limit_area.which_area == NVT_AREA_NORMAL)
				return true;
			ts->nvt_limit_area.which_area = ts->nvt_point_info[i].type;
			ts->nvt_coner_info[CORNER_TOPRIGHT].id = i;
			ts->nvt_coner_info[CORNER_TOPRIGHT].point_info = ts->nvt_point_info[i];
			ts->nvt_coner_info[CORNER_TOPRIGHT].flag = true;
			flag = 1;

        }
         //½Úµã/proc/touchpanel/oppo_tp_limit_enableµÄbit3Î»À´¿ØÖÆ¿ØÖÆ
       if ((ts->edge_limit.limit_lb) && (ts->nvt_point_info[i].x < ts->nvt_limit_area.area_xlb && ts->nvt_point_info[i].y > ts->nvt_limit_area.area_ylb)) {
			//printk("3  ts->nvt_point_info[i].x = %d,ts->nvt_point_info[i].y = %d\n",ts->nvt_point_info[i].x,ts->nvt_point_info[i].y);
            ts->nvt_point_info[i].type  = NVT_AREA_CORNER;
			if (ts->nvt_limit_area.which_area == NVT_AREA_NORMAL)
				return true;
  			ts->nvt_limit_area.which_area = ts->nvt_point_info[i].type;
   			ts->nvt_coner_info[CORNER_BOTTOMLEFT].id = i;
   			ts->nvt_coner_info[CORNER_BOTTOMLEFT].point_info = ts->nvt_point_info[i];
   			ts->nvt_coner_info[CORNER_BOTTOMLEFT].flag = true;
			flag = 1;

        }
         //½Úµã/proc/touchpanel/oppo_tp_limit_enableµÄbit4Î»À´¿ØÖÆ¿ØÖÆ
       if ((ts->edge_limit.limit_rb) && (ts->nvt_point_info[i].x > ts->nvt_limit_area.area_xrb && ts->nvt_point_info[i].y > ts->nvt_limit_area.area_yrb)) {
			//printk("4  ts->nvt_point_info[i].x = %d,ts->nvt_point_info[i].y = %d\n",ts->nvt_point_info[i].x,ts->nvt_point_info[i].y);
			ts->nvt_point_info[i].type  = NVT_AREA_CORNER;
			if (ts->nvt_limit_area.which_area == NVT_AREA_NORMAL)
				return true;

			ts->nvt_limit_area.which_area = ts->nvt_point_info[i].type;
			ts->nvt_coner_info[CORNER_BOTTOMRIGHT].id = i;
			ts->nvt_coner_info[CORNER_BOTTOMRIGHT].point_info = ts->nvt_point_info[i];
			ts->nvt_coner_info[CORNER_BOTTOMRIGHT].flag = true;
			flag = 1;

        }
        //×ø±êµãÎª·Ç±ß½ÇÇøÓòÊ±£¬µ¯ÆðÇ°Ãæ¼ÇÂ¼µÄ±ß½Ç×ø±êµã
        if (ts->nvt_point_info[i].type != NVT_AREA_CORNER) {
			//printk("ii i = %d,ts->nvt_point_info[i].type = %d\n",i,ts->nvt_point_info[i].type);
			//printk("NVT_AREA_CORNER =%d,NVT_AREA_NORMAL = %d\n",NVT_AREA_CORNER,NVT_AREA_NORMAL);
            if (ts->nvt_limit_area.which_area == NVT_AREA_CORNER) {
                for (j = 0; j < 4; j++) {
                    if (ts->nvt_coner_info[j].flag) {
						//printk("jj  j= %d, ts->nvt_coner_info[j].flag = %d\n",j,ts->nvt_coner_info[j].flag);
						//printk("j = %d,,ts->nvt_coner_info[j].id = %d\n",j,ts->nvt_coner_info[j].id);
#ifdef MT_PROTOCOL_B
                        input_mt_slot(ts->input_dev, ts->nvt_coner_info[j].id);
                        input_mt_report_slot_state(ts->input_dev, MT_TOOL_FINGER, 0);
#endif
                    }
                }
            }
        }
		if ( flag == 0 ) {
			//ts->nvt_point_info[i].type = NVT_AREA_NORMAL;
			ts->nvt_limit_area.which_area = NVT_AREA_NORMAL;
		} else {
			ts->nvt_limit_area.which_area = NVT_AREA_CORNER;
		}
    }

    return false;
}

#if NVT_TOUCH_WDT_RECOVERY
static uint8_t recovery_cnt = 0;
static uint8_t nvt_wdt_fw_recovery(uint8_t *point_data)
{
   uint32_t recovery_cnt_max = 10;
   uint8_t recovery_enable = false;
   uint8_t i = 0;

   recovery_cnt++;

   /* check pattern */
   for (i=1 ; i<7 ; i++) {
       if (point_data[i] != 0xFE) {
           recovery_cnt = 0;
           break;
       }
   }

   if (recovery_cnt > recovery_cnt_max){
       recovery_enable = true;
       recovery_cnt = 0;
   }

   return recovery_enable;
}
#endif	/* #if NVT_TOUCH_WDT_RECOVERY */

#define POINT_DATA_LEN 65
uint8_t last_st = 0;
/*******************************************************
Description:
	Novatek touchscreen work function.

return:
	n.a.
*******************************************************/
static void nvt_ts_work_func(struct work_struct *work)
{
	int32_t ret = -1;
	uint8_t point_data[POINT_DATA_LEN + 1] = {0};
	uint32_t position = 0;
	uint32_t input_x = 0;
	uint32_t input_y = 0;
	uint32_t input_w = 0;
	uint32_t input_p = 0;
	uint8_t input_id = 0;
#if MT_PROTOCOL_B
	uint8_t press_id[TOUCH_MAX_FINGER_NUM] = {0};
#endif /* MT_PROTOCOL_B */
	int32_t i = 0;
	int32_t finger_cnt = 0;
	uint32_t finger_last = 0;

	mutex_lock(&ts->lock);

	ret = CTP_SPI_READ(ts->client, point_data, POINT_DATA_LEN + 1);
	if (ret < 0) {
		NVT_ERR("CTP_SPI_READ failed.(%d)\n", ret);
		goto XFER_ERROR;
	}
/*
	//--- dump SPI buf ---
	for (i = 0; i < 10; i++) {
		printk("%02X %02X %02X %02X %02X %02X  ", point_data[1+i*6], point_data[2+i*6], point_data[3+i*6], point_data[4+i*6], point_data[5+i*6], point_data[6+i*6]);
	}
	printk("\n");
*/

#if NVT_TOUCH_WDT_RECOVERY
   /* ESD protect by WDT */
   if (nvt_wdt_fw_recovery(point_data)) {
       NVT_ERR("Recover for fw reset, %02X\n", point_data[1]);
#ifdef ODM_WT_EDIT
	   if (  ctpmodule == 0 ){
			nvt_update_firmware(BOOT_UPDATE_FIRMWARE_NAME_TM);
		}
	   else{
			nvt_update_firmware(BOOT_UPDATE_FIRMWARE_NAME);
	   }
#endif
       goto XFER_ERROR;
   }
#endif /* #if NVT_TOUCH_WDT_RECOVERY */

#if NVT_TOUCH_ESD_PROTECT
	/* ESD protect by FW handshake */
	if (nvt_fw_recovery(point_data)) {
		nvt_esd_check_enable(true);
		goto XFER_ERROR;
	}
#endif /* #if NVT_TOUCH_ESD_PROTECT */

#if WAKEUP_GESTURE
	if (bTouchIsAwake == 0) {
		input_id = (uint8_t)(point_data[1] >> 3);
		memset(&ts->gesture, 0, sizeof(struct gesture_info));
		nvt_ts_wakeup_gesture_report(input_id, point_data, &ts->gesture);
		//enable_irq(ts->client->irq);
		if ( ts->irq_enable_flag == 0) {
			enable_irq(ts->client->irq);
			ts->irq_enable_time++;
			ts->irq_enable_flag = 1;
		}

		mutex_unlock(&ts->lock);
		return;
	}
#endif

	finger_cnt = 0;
	ts->nvt_limit_area.which_area = NVT_AREA_DEFAULT;
	for (i = 0; i < ts->max_touch_num; i++) {
		position = 1 + 6 * i;
		input_id = (uint8_t)(point_data[position + 0] >> 3);
		if ((input_id == 0) || (input_id > ts->max_touch_num))
			continue;

		if (((point_data[position] & 0x07) == 0x01) || ((point_data[position] & 0x07) == 0x02)) {	//finger down (enter & moving)
#if NVT_TOUCH_ESD_PROTECT
			/* update interrupt timer */
			irq_timer = jiffies;
#endif /* #if NVT_TOUCH_ESD_PROTECT */
			input_x = (uint32_t)(point_data[position + 1] << 4) + (uint32_t) (point_data[position + 3] >> 4);
			input_y = (uint32_t)(point_data[position + 2] << 4) + (uint32_t) (point_data[position + 3] & 0x0F);
			if ((input_x < 0) || (input_y < 0))
				continue;
			if ((input_x > ts->abs_x_max) || (input_y > ts->abs_y_max))
				continue;
			input_w = (uint32_t)(point_data[position + 4]);
			if (input_w < 16)
				input_w = 5;
			if (input_w == 0)
				input_w = 1;
			if (i < 2) {
				input_p = (uint32_t)(point_data[position + 5]) + (uint32_t)(point_data[i + 63] << 8);
				if (input_p > TOUCH_FORCE_NUM)
					input_p = TOUCH_FORCE_NUM;
			} else {
				input_p = (uint32_t)(point_data[position + 5]);
			}
			if (input_p == 0)
				input_p = 1;

#if MT_PROTOCOL_B
			press_id[input_id - 1] = 1;
			input_mt_slot(ts->input_dev, input_id - 1);
			input_mt_report_slot_state(ts->input_dev, MT_TOOL_FINGER, true);
#else /* MT_PROTOCOL_B */
			input_report_abs(ts->input_dev, ABS_MT_TRACKING_ID, input_id - 1);
			input_report_key(ts->input_dev, BTN_TOUCH, 1);
#endif /* MT_PROTOCOL_B */

			input_report_abs(ts->input_dev, ABS_MT_POSITION_X, input_x);
			input_report_abs(ts->input_dev, ABS_MT_POSITION_Y, input_y);
			input_report_abs(ts->input_dev, ABS_MT_TOUCH_MAJOR, input_w);
			input_report_abs(ts->input_dev, ABS_MT_WIDTH_MAJOR, input_w);
			input_report_abs(ts->input_dev, ABS_MT_PRESSURE, input_p);

#if MT_PROTOCOL_B
#else /* MT_PROTOCOL_B */
			input_mt_sync(ts->input_dev);
#endif /* MT_PROTOCOL_B */

			finger_cnt++;

			/* backup oppo debug coordinate info */
			finger_last = (input_id > finger_last) ? input_id : finger_last;
			ts->oppo_debug_info.coordinate[i].x = (uint16_t) input_x;
			ts->oppo_debug_info.coordinate[i].y = (uint16_t) input_y;
			ts->nvt_point_info[i].x = (uint16_t) input_x;
			ts->nvt_point_info[i].y = (uint16_t) input_y;
			ts->nvt_point_info[i].type = NVT_AREA_NORMAL;
			if(ts->edge_limit.limit_lu == 1 || ts->edge_limit.limit_ru == 1 || ts->edge_limit.limit_lb == 1 || ts->edge_limit.limit_rb == 1) {
				nvt_corner_point_process(i);
			}

		}
	}

#if MT_PROTOCOL_B
	for (i = 0; i < ts->max_touch_num; i++) {
		if (press_id[i] != 1) {
			input_mt_slot(ts->input_dev, i);
			input_report_abs(ts->input_dev, ABS_MT_TOUCH_MAJOR, 0);
			input_report_abs(ts->input_dev, ABS_MT_WIDTH_MAJOR, 0);
			input_report_abs(ts->input_dev, ABS_MT_PRESSURE, 0);
			input_mt_report_slot_state(ts->input_dev, MT_TOOL_FINGER, false);
		}
	}

	input_report_key(ts->input_dev, BTN_TOUCH, (finger_cnt > 0));
#else /* MT_PROTOCOL_B */
	if (finger_cnt == 0) {
		input_report_key(ts->input_dev, BTN_TOUCH, 0);
		input_mt_sync(ts->input_dev);
	}
#endif /* MT_PROTOCOL_B */

#if TOUCH_KEY_NUM > 0
	if (point_data[61] == 0xF8) {
#if NVT_TOUCH_ESD_PROTECT
		/* update interrupt timer */
		ts->irq_timer = jiffies;
#endif /* #if NVT_TOUCH_ESD_PROTECT */
		for (i = 0; i < ts->max_button_num; i++) {
			input_report_key(ts->input_dev, touch_key_array[i], ((point_data[62] >> i) & 0x01));
		}
	} else {
		for (i = 0; i < ts->max_button_num; i++) {
			input_report_key(ts->input_dev, touch_key_array[i], 0);
		}
	}
#endif

	input_sync(ts->input_dev);

	for (i = 0; i < ts->max_touch_num; i++) {
		if ((ts->debug_level == 1) || (ts->debug_level == 2)) {
			/* debug touch coordinate info */
			if ((press_id[0] == 1) && (last_st == 0)) {	//finger down
				NVT_LOG("Touchpanel id %d :Down [%4d, %4d]\n",
						i,
						ts->oppo_debug_info.coordinate[0].x,
						ts->oppo_debug_info.coordinate[0].y);
			}
			//else if ((press_id[0] == 0) && (last_st == 1)) { //finger up
			else if ((finger_cnt == 0) && (i == finger_last)) { //finger up
				NVT_LOG("Touchpanel id %d :Up   [%4d, %4d]\n",
						i,
						ts->oppo_debug_info.coordinate[i].x,
						ts->oppo_debug_info.coordinate[i].y);
			}
		}

		if (ts->debug_level == 2) {
			if (press_id[i] == 1) {
				NVT_LOG("Touchpanel id %d :     [%4d, %4d]\n",
						i,
						ts->oppo_debug_info.coordinate[i].x,
						ts->oppo_debug_info.coordinate[i].y);
			}
		}

		if ((ts->debug_level == 1) || (ts->debug_level == 2)) {
			last_st = press_id[0];
		}
	}

XFER_ERROR:
	//enable_irq(ts->client->irq);
	if ( ts->irq_enable_flag == 0) {
		enable_irq(ts->client->irq);
		ts->irq_enable_time++;
		ts->irq_enable_flag = 1;
	}

	mutex_unlock(&ts->lock);
}

static void nvt_ts_usb_check_func(struct work_struct *work)
{

	int ret = -1;
	if (usb_state_ctp == 1) {
		ret = nvt_mode_switch(MODE_CHARGE, true);
		if (ret) {
			NVT_LOG("%s:plug in fail\n",__func__);
		}
	}
	if (usb_state_ctp == 0) {
		ret = nvt_mode_switch(MODE_CHARGE, false);
		if (ret) {
			NVT_LOG("%s:plug out fail\n",__func__);
		}
	}
	return;
}
void nvt_ts_usb_check_queue(void)
{

	queue_work(nvt_wq_use_check,&ts->nvt_usb_check_work);

	return;

}

/*******************************************************
Description:
	External interrupt service routine.

return:
	irq execute status.
*******************************************************/
static irqreturn_t nvt_ts_irq_handler(int32_t irq, void *dev_id)
{
	//disable_irq_nosync(ts->client->irq);
	unsigned long nIrqFlag;
	spin_lock_irqsave(&ts->spinlock_int, nIrqFlag);
	if (ts->irq_enable_flag == 1) {
		disable_irq_nosync(ts->client->irq);
		ts->irq_enable_time--;
		ts->irq_enable_flag = 0;
	}
	spin_unlock_irqrestore(&ts->spinlock_int, nIrqFlag);

#if WAKEUP_GESTURE
	if (bTouchIsAwake == 0) {
		//wake_lock_timeout(&gestrue_wakelock, msecs_to_jiffies(5000));
		pm_wakeup_event(&ts->input_dev->dev, 5000);
	}
#endif

	queue_work(nvt_wq, &ts->nvt_work);

	return IRQ_HANDLED;
}

/*******************************************************
Description:
	Novatek touchscreen check chip version trim function.

return:
	Executive outcomes. 0---NVT IC. -1---not NVT IC.
*******************************************************/
static int8_t nvt_ts_check_chip_ver_trim(void)
{
	uint8_t buf[8] = {0};
	int32_t retry = 0;
	int32_t list = 0;
	int32_t i = 0;
	int32_t found_nvt_chip = 0;
	int32_t ret = -1;

	//---Check for 5 times---
	for (retry = 5; retry > 0; retry--) {

		nvt_bootloader_reset();

		//---set xdata index to 0x1F600---
		nvt_set_page(0x1F600);

		buf[0] = 0x4E;
		buf[1] = 0x00;
		buf[2] = 0x00;
		buf[3] = 0x00;
		buf[4] = 0x00;
		buf[5] = 0x00;
		buf[6] = 0x00;
		CTP_SPI_READ(ts->client, buf, 7);
		NVT_LOG("buf[1]=0x%02X, buf[2]=0x%02X, buf[3]=0x%02X, buf[4]=0x%02X, buf[5]=0x%02X, buf[6]=0x%02X\n",
			buf[1], buf[2], buf[3], buf[4], buf[5], buf[6]);

		// compare read chip id on supported list
		for (list = 0; list < (sizeof(trim_id_table) / sizeof(struct nvt_ts_trim_id_table)); list++) {
			found_nvt_chip = 0;

			// compare each byte
			for (i = 0; i < NVT_ID_BYTE_MAX; i++) {
				if (trim_id_table[list].mask[i]) {
					if (buf[i + 1] != trim_id_table[list].id[i])
						break;
				}
			}

			if (i == NVT_ID_BYTE_MAX) {
				found_nvt_chip = 1;
			}

			if (found_nvt_chip) {
				NVT_LOG("This is NVT touch IC\n");
				ts->mmap = trim_id_table[list].mmap;
				ts->carrier_system = trim_id_table[list].carrier_system;
				ret = 0;
				goto out;
			} else {
				ts->mmap = NULL;
				ret = -1;
			}
		}

		msleep(10);
	}

out:
	return ret;
}
#if 0
static int  power_on(struct nvt_ts_data *ts, bool on)
{
    int rc = 0;
    if (!on) {
       printk("ctp power_off\n");
       goto power_off;
	}
	printk("ctp power_on start\n");
    rc = regulator_enable(ts->gpio_pwr);
    if ( rc != 0 ) {
        dev_err(&ts->client->dev,
                "%s Regulator ts->gpio_pwr enable failed rc=%d\n", __func__,rc);
        goto gpio_pwr_err;
    }
	rc = regulator_enable(ts->lab_pwr);
    if ( rc != 0 ) {
        dev_err(&ts->client->dev,
                "%s Regulator ts->lab_pwr enable failed rc=%d\n", __func__,rc);
        goto lab_pwr_err;
    }
    rc = regulator_enable(ts->ibb_pwr);
    if ( rc != 0 ) {
        dev_err(&ts->client->dev,
                "%s Regulator ts->ibb_pwr enable failed rc=%d\n", __func__,rc);
        goto ibb_pwr_err;
    }
	printk("ctp power_on end\n");

    return rc;

power_off:
ibb_pwr_err:
	if (ts->ibb_pwr) {
		regulator_disable(ts->ibb_pwr);
	}
lab_pwr_err:
	if (ts->lab_pwr) {
		regulator_disable(ts->lab_pwr);
	}
gpio_pwr_err:
	if (ts->gpio_pwr) {
		regulator_disable(ts->gpio_pwr);
	}

    return rc;
}

static int power_init(struct nvt_ts_data *ts, bool on)
{
    int rc = 0;
    if ( !on ) {
		printk("power_init is deny\n");
        goto pwr_deny;
	}
    ts->gpio_pwr = regulator_get(&ts->client->dev, "vdd");
	if ( IS_ERR(ts->gpio_pwr) ) {
        rc = PTR_ERR(ts->gpio_pwr);
        dev_err(&ts->client->dev,
                "%s Regulator get failed ts->gpio_pwr rc=%d\n", __func__,rc);

		goto gpio_pwr_err;
    }
	ts->lab_pwr = regulator_get(&ts->client->dev, "lab");
	if ( IS_ERR(ts->lab_pwr) ) {
        rc = PTR_ERR(ts->lab_pwr);
        dev_err(&ts->client->dev,
                "%s Regulator get failed ts->lab_pwr rc=%d\n", __func__,rc);

		goto lab_pwr_err;
    }
	ts->ibb_pwr = regulator_get(&ts->client->dev, "ibb");
	if ( IS_ERR(ts->ibb_pwr) ) {
        rc = PTR_ERR(ts->ibb_pwr);
       dev_err(&ts->client->dev,
                "%s Regulator get failed ts->ibb_pwr rc=%d\n", __func__,rc);

		goto ibb_pwr_err;
    }

    return rc;

pwr_deny:
ibb_pwr_err:
	if (ts->ibb_pwr) {
		regulator_put(ts->ibb_pwr);
		ts->ibb_pwr = NULL;
	}
lab_pwr_err:
	if (ts->lab_pwr) {
		regulator_put(ts->lab_pwr);
		ts->lab_pwr = NULL;
	}
gpio_pwr_err:
	if (ts->gpio_pwr) {
		regulator_put(ts->gpio_pwr);
		ts->gpio_pwr = NULL;
	}

    return rc;

}
#endif
/*******************************************************
Description:
	Novatek touchscreen driver probe function.

return:
	Executive outcomes. 0---succeed. negative---failed
*******************************************************/
static int32_t nvt_ts_probe(struct spi_device *client)
{
	int32_t ret = 0;
	int cs_pin = 0;
#if (TOUCH_KEY_NUM > 0)
	int32_t retry = 0;
#endif
#ifdef ODM_WT_EDIT
	char *temp = NULL;
	char * cmdline_tp = NULL;
	cmdline_tp = strstr(saved_command_line,"qcom,mdss_dsi_nt36525_");
	printk("cmdline_tp = %s\n",cmdline_tp);
	if ( cmdline_tp == NULL ){
		printk("get qcom,mdss_dsi_nt36525_tianma_hdp_video fail ");
		return -1;
	}
	temp = cmdline_tp + strlen("qcom,mdss_dsi_nt36525_");
	printk("temp = %s\n",temp);
	ctpmodule = strncmp(temp,"tianma",strlen("tianma"));
	if ( ctpmodule       == 0 ){
		printk("this is TM touchscreen\n");
	}
	else {
		printk("this is xinli touchscreen\n");
	}
#endif
	NVT_LOG("start\n");

	ts = kmalloc(sizeof(struct nvt_ts_data), GFP_KERNEL);
	ts->client = client;
	spi_set_drvdata(client, ts);
	if (ts == NULL) {
		NVT_ERR("failed to allocated memory for nvt ts data\n");
		return -ENOMEM;
	}
	ts->recovery_flag = 0;
	ts->boot_mode = -1;
	NVT_LOG("==========ts->boot_mode = %d\n",ts->boot_mode);
	//if ((ts->boot_mode == MSM_BOOT_MODE__FACTORY || ts->boot_mode == MSM_BOOT_MODE__RF || ts->boot_mode == MSM_BOOT_MODE__WLAN))
	//if ((ts->boot_mode == 3 || ts->boot_mode == 4 || ts->boot_mode == 5)) {
	//	NVT_LOG("boot_mode is FACTORY,RF and WLAN not need to add enable irq\n");
		//return -1;
	//} else if(ts->boot_mode == 2) {
		//ts->recovery_flag = 1;
		//NVT_LOG("boot_mode is recovery\n");
	//} else {
		//NVT_LOG("boot_mode is normol or others\n");
	//}
#if 0
    ret = power_init(ts, true);
	if (ret) {
        printk("nvt power init fail\n");
	}
	ret = power_on(ts, true);
	if (ret) {
        printk("nvt power on fail\n");
	}
#endif
	novatek_tp = 1;
	ts->irq_enable_time = 0;
	ts->irq_enable_flag = 0;
	ts->sleep_flag = 0;
	ts->gesture_test.flag = 0;
	ts->g_gesture_bak = 0;
	ts->nvt_fw_updating = 0;
	ts->oppo_update_fw_flag = 0;
#ifdef ODM_WT_EDIT
	rbuf_init();
#endif
	//---prepare for spi parameter---
	if (ts->client->master->flags & SPI_MASTER_HALF_DUPLEX) {
		NVT_ERR("Full duplex not supported by master\n");
		ret = -EIO;
		goto err_ckeck_full_duplex;
	}
	ts->client->bits_per_word = 8;
	ts->client->mode = SPI_MODE_0;
    ts->selft_test_flg = 0;
	ret = spi_setup(ts->client);
	if (ret < 0) {
		NVT_ERR("Failed to perform SPI setup\n");
		goto err_spi_setup;
	}

#ifdef CONFIG_MTK_SPI
	memcpy(&ts->spi_ctrl, &spi_ctrdata, sizeof(struct mt_chip_conf));
	ts->client->controller_data = (void *)&ts->spi_ctrl;
#endif

	NVT_LOG("mode=%d, max_speed_hz=%d\n", ts->client->mode, ts->client->max_speed_hz);

	//---parse dts---
	ret = nvt_parse_dt(&client->dev);
	if (ret) {
		NVT_ERR("parse dt error\n");
		goto err_spi_setup;
	}

	//---request and config GPIOs---
	ret = nvt_gpio_config(ts);
	if (ret) {
		NVT_ERR("gpio config error!\n");
		goto err_gpio_config_failed;
	}

	//---eng reset before TP_RESX high
	nvt_eng_reset();

#if NVT_TOUCH_SUPPORT_HW_RST
	gpio_set_value(ts->reset_gpio, 1);
#endif

	// need 10ms delay after POR(power on reset)
	msleep(10);

	//---check chip version trim---
	ret = nvt_ts_check_chip_ver_trim();
	if (ret) {
		NVT_ERR("chip is not identified\n");
		ret = -EINVAL;
		goto err_chipvertrim_failed;
	}

	mutex_init(&ts->lock);
	spin_lock_init(&ts->spinlock_int);

	ts->abs_x_max = TOUCH_DEFAULT_MAX_WIDTH;
	ts->abs_y_max = TOUCH_DEFAULT_MAX_HEIGHT;

	//---create workqueue---
	nvt_wq = create_workqueue("nvt_wq");
	if (!nvt_wq) {
		NVT_ERR("nvt_wq create workqueue failed\n");
		ret = -ENOMEM;
		goto err_create_nvt_wq_failed;
	}
	INIT_WORK(&ts->nvt_work, nvt_ts_work_func);

	nvt_wq_use_check = create_workqueue("nvt_wq_use_check");
	if (!nvt_wq_use_check) {
		NVT_ERR("nvt_wq_use_check create workqueue failed\n");
		ret = -ENOMEM;
		goto err_create_nvt_wq_failed;
	}
	INIT_WORK(&ts->nvt_usb_check_work, nvt_ts_usb_check_func);


	//---allocate input device---
	ts->input_dev = input_allocate_device();
	if (ts->input_dev == NULL) {
		NVT_ERR("allocate input device failed\n");
		ret = -ENOMEM;
		goto err_input_dev_alloc_failed;
	}

	ts->max_touch_num = TOUCH_MAX_FINGER_NUM;

#if TOUCH_KEY_NUM > 0
	ts->max_button_num = TOUCH_KEY_NUM;
#endif

	ts->int_trigger_type = INT_TRIGGER_TYPE;


	//---set input device info.---
	ts->input_dev->evbit[0] = BIT_MASK(EV_SYN) | BIT_MASK(EV_KEY) | BIT_MASK(EV_ABS) ;
	ts->input_dev->keybit[BIT_WORD(BTN_TOUCH)] = BIT_MASK(BTN_TOUCH);
	ts->input_dev->propbit[0] = BIT(INPUT_PROP_DIRECT);

#if MT_PROTOCOL_B
	input_mt_init_slots(ts->input_dev, ts->max_touch_num, 0);
#endif

	input_set_abs_params(ts->input_dev, ABS_MT_PRESSURE, 0, TOUCH_FORCE_NUM, 0, 0);    //pressure = TOUCH_FORCE_NUM

#if TOUCH_MAX_FINGER_NUM > 1
	input_set_abs_params(ts->input_dev, ABS_MT_TOUCH_MAJOR, 0, 255, 0, 0);    //area = 255
	input_set_abs_params(ts->input_dev, ABS_MT_WIDTH_MAJOR, 0, 255, 0, 0);

	input_set_abs_params(ts->input_dev, ABS_MT_POSITION_X, 0, ts->abs_x_max, 0, 0);
	input_set_abs_params(ts->input_dev, ABS_MT_POSITION_Y, 0, ts->abs_y_max, 0, 0);
#if MT_PROTOCOL_B
	// no need to set ABS_MT_TRACKING_ID, input_mt_init_slots() already set it
#else
	input_set_abs_params(ts->input_dev, ABS_MT_TRACKING_ID, 0, ts->max_touch_num, 0, 0);
#endif //MT_PROTOCOL_B
#endif //TOUCH_MAX_FINGER_NUM > 1

#if TOUCH_KEY_NUM > 0
	for (retry = 0; retry < ts->max_button_num; retry++) {
		input_set_capability(ts->input_dev, EV_KEY, touch_key_array[retry]);
	}
#endif

#if WAKEUP_GESTURE
	ts->gesture_enable = 0;
	memset(&ts->gesture, 0, sizeof(struct gesture_info));
	input_set_capability(ts->input_dev, EV_KEY, KEY_F4);
	//wake_lock_init(&gestrue_wakelock, WAKE_LOCK_SUSPEND, "poll-wake-lock");
	device_init_wakeup(&ts->input_dev->dev, 1);
#endif

	sprintf(ts->phys, "input/ts");
	ts->input_dev->name = NVT_TS_NAME;
	ts->input_dev->phys = ts->phys;
	ts->input_dev->id.bustype = BUS_SPI;

	//---register input device---
	ret = input_register_device(ts->input_dev);
	if (ret) {
		NVT_ERR("register input device (%s) failed. ret=%d\n", ts->input_dev->name, ret);
		goto err_input_register_device_failed;
	}

	//---set int-pin & request irq---
	client->irq = gpio_to_irq(ts->irq_gpio);
	if (client->irq) {
		NVT_LOG("int_trigger_type=%d\n", ts->int_trigger_type);

#if WAKEUP_GESTURE
		ret = request_irq(client->irq, nvt_ts_irq_handler,
				ts->int_trigger_type | IRQF_NO_SUSPEND, NVT_SPI_NAME, ts);
#else
		ret = request_irq(client->irq, nvt_ts_irq_handler,
				ts->int_trigger_type, NVT_SPI_NAME, ts);
#endif
		if (ret != 0) {
			NVT_ERR("request irq failed. ret=%d\n", ret);
			goto err_int_request_failed;
		} else {
			//disable_irq(client->irq);
			ts->irq_enable_flag = 1;
			nvt_irq_enable(0);
			NVT_LOG("request irq %d succeed\n", client->irq);
		}
	}

#if BOOT_UPDATE_FIRMWARE
	nvt_fwu_wq = create_singlethread_workqueue("nvt_fwu_wq");
	if (!nvt_fwu_wq) {
		NVT_ERR("nvt_fwu_wq create workqueue failed\n");
		ret = -ENOMEM;
		goto err_create_nvt_fwu_wq_failed;
	}
	INIT_DELAYED_WORK(&ts->nvt_fwu_work, Boot_Update_Firmware);
	// please make sure boot update start after display reset(RESX) sequence
	queue_delayed_work(nvt_fwu_wq, &ts->nvt_fwu_work, msecs_to_jiffies(14000));
#endif

	/*nvt_resume_wq = create_singlethread_workqueue("resume_fwu_wq");
	if (!nvt_resume_wq) {
		NVT_ERR("nvt_resume_wq create workqueue failed\n");
		ret = -ENOMEM;
		goto err_create_nvt_fwu_wq_failed;
	}*/
	INIT_WORK(&ts->nvt_resume_work, resume_Update_Firmware);

	NVT_LOG("NVT_TOUCH_ESD_PROTECT is %d\n", NVT_TOUCH_ESD_PROTECT);
#if NVT_TOUCH_ESD_PROTECT
	INIT_DELAYED_WORK(&nvt_esd_check_work, nvt_esd_check_func);
	nvt_esd_check_wq = create_workqueue("nvt_esd_check_wq");
	queue_delayed_work(nvt_esd_check_wq, &nvt_esd_check_work,
			msecs_to_jiffies(NVT_TOUCH_ESD_CHECK_PERIOD));
#endif /* #if NVT_TOUCH_ESD_PROTECT */

	//---set device node---
#if NVT_TOUCH_PROC
	ret = nvt_flash_proc_init();
	if (ret != 0) {
		NVT_ERR("nvt flash proc init failed. ret=%d\n", ret);
		goto err_init_NVT_ts;
	}
#endif

#if NVT_TOUCH_EXT_PROC
	ret = nvt_extra_proc_init();
	if (ret != 0) {
		NVT_ERR("nvt extra proc init failed. ret=%d\n", ret);
		goto err_init_NVT_ts;
	}
#endif

#if NVT_TOUCH_MP
	ret = nvt_mp_proc_init();
	if (ret != 0) {
		NVT_ERR("nvt mp proc init failed. ret=%d\n", ret);
		goto err_init_NVT_ts;
	}
#endif

#if defined(CONFIG_FB)
	ts->fb_notif.notifier_call = fb_notifier_callback;
	ret = fb_register_client(&ts->fb_notif);
	if(ret) {
		NVT_ERR("register fb_notifier failed. ret=%d\n", ret);
		goto err_register_fb_notif_failed;
	}
#elif defined(CONFIG_HAS_EARLYSUSPEND)
	ts->early_suspend.level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN + 1;
	ts->early_suspend.suspend = nvt_ts_early_suspend;
	ts->early_suspend.resume = nvt_ts_late_resume;
	ret = register_early_suspend(&ts->early_suspend);
	if(ret) {
		NVT_ERR("register early suspend failed. ret=%d\n", ret);
		goto err_register_early_suspend_failed;
	}
#endif

	bTouchIsAwake = 1;
	NVT_LOG("end\n");
	if ((ts->boot_mode == 3 || ts->boot_mode == 4 || ts->boot_mode == 5)) {
	
		NVT_LOG("boot_mode is FACTORY,RF and WLAN not need to add enable irq and pull up cs pin\n");
		if (gpio_is_valid(ts->cs_gpio)) {
			cs_pin = gpio_request(ts->cs_gpio, "cs-gpio");
			if (cs_pin) {
	            NVT_ERR("Failed to request cs_pin GPIO\n");
		    }
	        gpio_direction_output(ts->cs_gpio, 1);
			NVT_LOG("gpio_get_value(ts->cs_gpio)1  = %d\n",gpio_get_value(ts->cs_gpio));
			gpio_free(ts->cs_gpio);
    	}
	} else {
		//enable_irq(client->irq);
		nvt_irq_enable(1);
	}
	return 0;

#if defined(CONFIG_FB)
err_register_fb_notif_failed:
#elif defined(CONFIG_HAS_EARLYSUSPEND)
err_register_early_suspend_failed:
#endif
#if (NVT_TOUCH_PROC || NVT_TOUCH_EXT_PROC || NVT_TOUCH_MP)
err_init_NVT_ts:
#endif
	free_irq(client->irq, ts);
#if BOOT_UPDATE_FIRMWARE
err_create_nvt_fwu_wq_failed:
#endif
err_int_request_failed:
err_input_register_device_failed:
	input_free_device(ts->input_dev);
err_input_dev_alloc_failed:
err_create_nvt_wq_failed:
	mutex_destroy(&ts->lock);
err_chipvertrim_failed:
	gpio_free(ts->irq_gpio);
err_gpio_config_failed:
err_spi_setup:
err_ckeck_full_duplex:
	spi_set_drvdata(client, NULL);
	kfree(ts);
	return ret;
}

/*******************************************************
Description:
	Novatek touchscreen driver release function.

return:
	Executive outcomes. 0---succeed.
*******************************************************/
static int32_t nvt_ts_remove(struct spi_device *client)
{
#if defined(CONFIG_FB)
	if (fb_unregister_client(&ts->fb_notif))
		NVT_ERR("Error occurred while unregistering fb_notifier.\n");
#elif defined(CONFIG_HAS_EARLYSUSPEND)
	unregister_early_suspend(&ts->early_suspend);
#endif

	mutex_destroy(&ts->lock);

	NVT_LOG("Removing driver...\n");

	free_irq(client->irq, ts);
	input_unregister_device(ts->input_dev);
	spi_set_drvdata(client, NULL);
	kfree(ts);

	return 0;
}

/*******************************************************
Description:
	Novatek touchscreen driver suspend function.

return:
	Executive outcomes. 0---succeed.
*******************************************************/
static int32_t nvt_ts_suspend(struct device *dev)
{
	uint8_t buf[4] = {0};
	//int pwr = 0;
	//int lcd_flags = 0;
	int j = 0;
	int cs_pin = 0;
#if MT_PROTOCOL_B
	uint32_t i = 0;
#endif

	if (!bTouchIsAwake) {
		NVT_LOG("Touch is already suspend\n");
		return 0;
	}
	bTouchIsAwake = 0;
	ts->sleep_flag = 1;
	for(j = 0;j<80;j++) {
		if(ts->nvt_fw_updating == 1) {
			msleep(5);
		} else {
			break;
		}
	}

#if NVT_TOUCH_ESD_PROTECT
		cancel_delayed_work_sync(&nvt_esd_check_work);
		nvt_esd_check_enable(false);
#endif /* #if NVT_TOUCH_ESD_PROTECT */

	mutex_lock(&ts->lock);

	NVT_LOG("start\n");

	//bTouchIsAwake = 0;

//#if NVT_TOUCH_ESD_PROTECT
	//cancel_delayed_work_sync(&nvt_esd_check_work);
	//nvt_esd_check_enable(false);
//#endif /* #if NVT_TOUCH_ESD_PROTECT */

#if WAKEUP_GESTURE
	if (ts->gesture_enable) {
		NVT_LOG("ts->gesture_enable = %d:enter gesture mode\n",ts->gesture_enable);
		if (gpio_is_valid(ts->cs_gpio)) {
			cs_pin = gpio_request(ts->cs_gpio, "cs-gpio");
			if (cs_pin) {
	            NVT_ERR("Failed to request cs_pin GPIO\n");
		    }
	        gpio_direction_output(ts->cs_gpio, 1);
			NVT_LOG("gpio_get_value(ts->cs_gpio)1  = %d\n",gpio_get_value(ts->cs_gpio));
			gpio_free(ts->cs_gpio);
    	}
		//---write spi command to enter "wakeup gesture mode"---
		buf[0] = EVENT_MAP_HOST_CMD;
		buf[1] = 0x13;
		CTP_SPI_WRITE(ts->client, buf, 2);

	    enable_irq_wake(ts->client->irq);

		NVT_LOG("Enabled touch wakeup gesture\n");
	} else {
	    NVT_LOG("ts->gesture_enable = %d:close gesture mode\n",ts->gesture_enable);
		/*if (gpio_is_valid(ts->lcd_reset_gpio)) {
            lcd_flags = gpio_request(ts->lcd_reset_gpio, "NVT-lcd-rst");
			NVT_LOG("ts->lcd_reset_gpio = %d\n",ts->lcd_reset_gpio);
            if (lcd_flags) {
	            NVT_ERR("Failed to request NVT-lcd-rst GPIO\n");
		    }
			NVT_LOG("gpio_get_value(ts->lcd_reset_gpio)1  = %d\n",gpio_get_value(ts->lcd_reset_gpio));
            gpio_direction_output(ts->lcd_reset_gpio, 0);
			NVT_LOG("gpio_get_value(ts->lcd_reset_gpio)0  = %d\n",gpio_get_value(ts->lcd_reset_gpio));
			gpio_free(ts->lcd_reset_gpio);
	    }*/
	    #if 0
		if (gpio_is_valid(ts->reset_gpio)) {
			NVT_LOG("gpio_get_value(ts->reset_gpio)1  = %d\n",gpio_get_value(ts->reset_gpio));
	        gpio_set_value(ts->reset_gpio, 0);
			NVT_LOG("gpio_get_value(ts->ctp reset_gpio)0  = %d\n",gpio_get_value(ts->reset_gpio));
    	}
		#endif
		nvt_irq_enable(0);
		//---write spi command to enter "deep sleep mode"---
		buf[0] = EVENT_MAP_HOST_CMD;
		buf[1] = 0x11;
		CTP_SPI_WRITE(ts->client, buf, 2);
	    /*pwr = power_on(ts,false);
		if(pwr != 0) {
			NVT_ERR("%s:power off fail\n",__func__);
		}*/
		//disable_irq(ts->client->irq);
		NVT_LOG("gpio_get_value(ts->cs_gpio)0  = %d\n",gpio_get_value(ts->cs_gpio));
	}

#else // WAKEUP_GESTURE
    printk("ts->gesture_enable = %d:colse gesture mode\n",ts->gesture_enable);
	if (gpio_is_valid(ts->lcd_reset_gpio)) {
		lcd_flags = gpio_request(ts->lcd_reset_gpio, "NVT-lcd-rst");
		printk("ts->lcd_reset_gpio = %d\n",ts->lcd_reset_gpio);
		if (lcd_flags) {
			NVT_ERR("Failed to request NVT-lcd-rst GPIO\n");
		}
		printk("gpio_get_value(ts->lcd_reset_gpio)3  = %d\n",gpio_get_value(ts->lcd_reset_gpio));
		gpio_direction_output(ts->lcd_reset_gpio, 0);
		printk("gpio_get_value(ts->lcd_reset_gpio)4  = %d\n",gpio_get_value(ts->lcd_reset_gpio));
	}
	gpio_free(ts->lcd_reset_gpio);
	if (gpio_is_valid(ts->reset_gpio)) {
		printk("gpio_get_value(ts->reset_gpio)1  = %d\n",gpio_get_value(ts->reset_gpio));
        gpio_set_value(ts->reset_gpio, 0);
		printk("gpio_get_value(ts->lcd_reset_gpio)0  = %d\n",gpio_get_value(ts->reset_gpio));
	}
	//---write spi command to enter "deep sleep mode"---
	buf[0] = EVENT_MAP_HOST_CMD;
	buf[1] = 0x11;
	CTP_SPI_WRITE(ts->client, buf, 2);
	pwr = power_on(ts,false);
	if(pwr != 0) {
		printk("%s:power off fail\n",__func__);
	}
	//disable_irq(ts->client->irq);
	nvt_irq_enable(0);
	printk("gpio_get_value(ts->cs_gpio)0  = %d\n",gpio_get_value(ts->cs_gpio));
#endif // WAKEUP_GESTURE

	/* release all touches */
#if MT_PROTOCOL_B
	for (i = 0; i < ts->max_touch_num; i++) {
		input_mt_slot(ts->input_dev, i);
		input_report_abs(ts->input_dev, ABS_MT_TOUCH_MAJOR, 0);
		input_report_abs(ts->input_dev, ABS_MT_WIDTH_MAJOR, 0);
		input_report_abs(ts->input_dev, ABS_MT_PRESSURE, 0);
		input_mt_report_slot_state(ts->input_dev, MT_TOOL_FINGER, 0);
	}
#endif
	input_report_key(ts->input_dev, BTN_TOUCH, 0);
#if !MT_PROTOCOL_B
	input_mt_sync(ts->input_dev);
#endif
	input_sync(ts->input_dev);

	msleep(50);

	mutex_unlock(&ts->lock);

	NVT_LOG("end\n");

	return 0;
}
static void nova_report_all_leave_event(void)
{
#if MT_PROTOCOL_B
		int i;
		for (i = 0; i < ts->max_touch_num; i++) {
			input_mt_slot(ts->input_dev, i);
			input_report_abs(ts->input_dev, ABS_MT_TOUCH_MAJOR, 0);
			input_report_abs(ts->input_dev, ABS_MT_WIDTH_MAJOR, 0);
			input_report_abs(ts->input_dev, ABS_MT_PRESSURE, 0);
			input_mt_report_slot_state(ts->input_dev, MT_TOOL_FINGER, 0);
		}
#endif

}
void nvt_resume_tmp(void)
{
	int gpio = 0;

	nvt_ts_usb_check_queue();

#if !WAKEUP_GESTURE
		#if 0
		if (gpio_is_valid(ts->reset_gpio)) {
			gpio_set_value(ts->reset_gpio, 1);
			NVT_LOG("gpio_get_value(ts->lcd_reset_gpio)1  = %d\n",gpio_get_value(ts->reset_gpio));
		}
		NVT_LOG("gpio_get_value(ts->cs_gpio)1  = %d\n",gpio_get_value(ts->cs_gpio));
		pwr = power_on(ts,true);
		if(pwr != 0) {
			NVT_LOG("%s:power on fail\n",__func__);
		}
		#endif
		//enable_irq(ts->client->irq);
		nvt_irq_enable(1);
#else
		if ( ts->gesture_enable == 0 ) {
			#if 0
			if (gpio_is_valid(ts->reset_gpio)) {
				gpio_set_value(ts->reset_gpio, 1);
				NVT_LOG("gpio_get_value(ts->lcd_reset_gpio)1  = %d\n",gpio_get_value(ts->reset_gpio));
			}
			NVT_LOG("gpio_get_value(ts->cs_gpio)1  = %d\n",gpio_get_value(ts->cs_gpio));
			pwr = power_on(ts,true);
			if(pwr != 0) {
				NVT_LOG("%s:power off fail\n",__func__);
			}
			#endif
			//enable_irq(ts->client->irq);
			nvt_irq_enable(1);
		} else {
			gpio = gpio_get_value(ts->lcd_reset_gpio);
			NVT_LOG("ts->lcd_reset_gpio = %d",gpio);
		}
		//ts->sleep_flag = 0;
		ts->nvt_fw_updating = 0;
#endif

#if NVT_TOUCH_ESD_PROTECT
		queue_delayed_work(nvt_esd_check_wq, &nvt_esd_check_work,
				msecs_to_jiffies(NVT_TOUCH_ESD_CHECK_PERIOD));
#endif /* #if NVT_TOUCH_ESD_PROTECT */
		NVT_LOG("%s\n",version);
		//bTouchIsAwake = 1;
		nvt_irq_enable(1);
		nova_report_all_leave_event();
		return;

}
void lcd_resume_load_nvt_fw(void)
{
	schedule_work(&ts->nvt_resume_work);
	return;

}
/*******************************************************
Description:
	Novatek touchscreen driver resume function.

return:
	Executive outcomes. 0---succeed.
*******************************************************/
static int32_t nvt_ts_resume(struct device *dev)
{
    //int pwr = 0;
	//int gpio = 0;
	if (bTouchIsAwake) {
		NVT_LOG("Touch is already resume\n");
		return 0;
	}
	//ts->sleep_flag = 0;
	//mutex_lock(&ts->lock);

	NVT_LOG("start\n");
	bTouchIsAwake = 1;
	//ts->sleep_flag = 0;
	// please make sure display reset(RESX) sequence and mipi dsi cmds sent before this
#if 0

#if NVT_TOUCH_SUPPORT_HW_RST
	gpio_set_value(ts->reset_gpio, 1);
#endif
	//queue_delayed_work(nvt_resume_wq, &ts->nvt_resume_work, msecs_to_jiffies(10));
	//schedule_work(&ts->nvt_resume_work);
	NVT_LOG("%s\n",version);

#ifdef ODM_WT_EDIT
	if ( ctpmodule == 0 ){
		nvt_update_firmware(BOOT_UPDATE_FIRMWARE_NAME_TM);
	}
	else{
		nvt_update_firmware(BOOT_UPDATE_FIRMWARE_NAME);
	}
#endif
	nvt_check_fw_reset_state(RESET_STATE_REK);


#if !WAKEUP_GESTURE
	if (gpio_is_valid(ts->reset_gpio)) {
        gpio_set_value(ts->reset_gpio, 1);
		NVT_LOG("gpio_get_value(ts->lcd_reset_gpio)1  = %d\n",gpio_get_value(ts->reset_gpio));
	}
	NVT_LOG("gpio_get_value(ts->cs_gpio)1  = %d\n",gpio_get_value(ts->cs_gpio));
	/*pwr = power_on(ts,true);
	if(pwr != 0) {
		NVT_LOG("%s:power on fail\n",__func__);
	}*/
	//enable_irq(ts->client->irq);
	nvt_irq_enable(1);
#else
    if ( ts->gesture_enable == 0 ) {
		if (gpio_is_valid(ts->reset_gpio)) {
	        gpio_set_value(ts->reset_gpio, 1);
			NVT_LOG("gpio_get_value(ts->lcd_reset_gpio)1  = %d\n",gpio_get_value(ts->reset_gpio));
		}
		NVT_LOG("gpio_get_value(ts->cs_gpio)1  = %d\n",gpio_get_value(ts->cs_gpio));
	    /*pwr = power_on(ts,true);
		if(pwr != 0) {
			NVT_LOG("%s:power off fail\n",__func__);
		}*/
        //enable_irq(ts->client->irq);
        nvt_irq_enable(1);
	} else {
	    gpio = gpio_get_value(ts->lcd_reset_gpio);
		NVT_LOG("ts->lcd_reset_gpio = %d",gpio);
	}
	ts->sleep_flag = 0;
#endif

#if NVT_TOUCH_ESD_PROTECT
	queue_delayed_work(nvt_esd_check_wq, &nvt_esd_check_work,
			msecs_to_jiffies(NVT_TOUCH_ESD_CHECK_PERIOD));
#endif /* #if NVT_TOUCH_ESD_PROTECT */

	bTouchIsAwake = 1;
	nvt_irq_enable(1);
	mutex_unlock(&ts->lock);
	nova_report_all_leave_event();
#endif

	NVT_LOG("end\n");

	return 0;
}


#if defined(CONFIG_FB)
static int fb_notifier_callback(struct notifier_block *self, unsigned long event, void *data)
{
	struct fb_event *evdata = data;
	int *blank;
	struct nvt_ts_data *ts =
		container_of(self, struct nvt_ts_data, fb_notif);

	if (evdata && evdata->data && event == FB_EARLY_EVENT_BLANK) {
		blank = evdata->data;
		if (*blank == FB_BLANK_POWERDOWN) {
			nvt_ts_suspend(&ts->client->dev);
		}
	} else if (evdata && evdata->data && event == FB_EVENT_BLANK) {
		blank = evdata->data;
		if (*blank == FB_BLANK_UNBLANK) {
			nvt_ts_resume(&ts->client->dev);
		}
	}

	return 0;
}
#elif defined(CONFIG_HAS_EARLYSUSPEND)
/*******************************************************
Description:
	Novatek touchscreen driver early suspend function.

return:
	n.a.
*******************************************************/
static void nvt_ts_early_suspend(struct early_suspend *h)
{
	nvt_ts_suspend(ts->client, PMSG_SUSPEND);
}

/*******************************************************
Description:
	Novatek touchscreen driver late resume function.

return:
	n.a.
*******************************************************/
static void nvt_ts_late_resume(struct early_suspend *h)
{
	nvt_ts_resume(ts->client);
}
#endif

static const struct spi_device_id nvt_ts_id[] = {
	{ NVT_SPI_NAME, 0 },
	{ }
};

#ifdef CONFIG_OF
static struct of_device_id nvt_match_table[] = {
	{ .compatible = "novatek,NVT-ts-spi",},
	{ },
};
#endif

static struct spi_driver nvt_spi_driver = {
	.probe		= nvt_ts_probe,
	.remove		= nvt_ts_remove,
	.id_table	= nvt_ts_id,
	.driver = {
		.name	= NVT_SPI_NAME,
		.owner	= THIS_MODULE,
#ifdef CONFIG_OF
		.of_match_table = nvt_match_table,
#endif
	},
};

/*******************************************************
Description:
	Driver Install function.

return:
	Executive Outcomes. 0---succeed. not 0---failed.
********************************************************/
static int32_t __init nvt_driver_init(void)
{
	int32_t ret = 0;

	NVT_LOG("start\n");

	//---add spi driver---
	ret = spi_register_driver(&nvt_spi_driver);
	if (ret) {
		pr_err("%s: failed to add spi driver", __func__);
		goto err_driver;
	}

	NVT_LOG("%s: finished\n", __func__);

err_driver:
	return ret;
}

/*******************************************************
Description:
	Driver uninstall function.

return:
	n.a.
********************************************************/
static void __exit nvt_driver_exit(void)
{
	spi_unregister_driver(&nvt_spi_driver);

	if (nvt_wq)
		destroy_workqueue(nvt_wq);

#if BOOT_UPDATE_FIRMWARE
	if (nvt_fwu_wq)
		destroy_workqueue(nvt_fwu_wq);
#endif

#if NVT_TOUCH_ESD_PROTECT
	if (nvt_esd_check_wq)
		destroy_workqueue(nvt_esd_check_wq);
#endif /* #if NVT_TOUCH_ESD_PROTECT */
}

//late_initcall(nvt_driver_init);
module_init(nvt_driver_init);
module_exit(nvt_driver_exit);

MODULE_DESCRIPTION("Novatek Touchscreen Driver");
MODULE_LICENSE("GPL");
