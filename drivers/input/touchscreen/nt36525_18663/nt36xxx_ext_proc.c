/*******************************************************************************
** Copyright (C), 2008-2016, OPPO Mobile Comm Corp., Ltd.
** ODM_WT_EDIT
** FILE: - nt36xxx_ext_proc.c
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

#include "nt36xxx.h"
#include <asm/uaccess.h>

#if NVT_TOUCH_EXT_PROC
#define NVT_FW_VERSION "nvt_fw_version"
#define NVT_BASELINE "nvt_baseline"
#define NVT_RAW "nvt_raw"
#define NVT_DIFF "nvt_diff"
#ifdef ODM_WT_EDIT
#define OPPO_TOUCHPANEL_NAME "touchpanel"
#define OPPO_BASELINE_TEST "baseline_test"
#define OPPO_COORDINATE "coordinate"
#define OPPO_DEBUG_INFO "debug_info"
#define OPPO_DELTA "delta"
#define OPPO_BASELINE "baseline"
#define OPPO_MAIN_REGISTER "main_register"
#define OPPO_DEBUG_LEVEL "debug_level"
#define OPPO_GESTURE "double_tap_enable"
#define OPPO_IRQ_DEPATH "irq_depth"
#define OPPO_REGISTER_INFO "oppo_register_info"
#define OPPO_FW_UPDATE "tp_fw_update"
#define OPPO_DEVICE_TEST "i2c_device_test"
#define OPPO_BLACKSCREEN_TEST "black_screen_test"
#define OPPO_GAME_SWITCH "game_switch_enable"
#define OPPO_TP_LIMIT_ENABLE "oppo_tp_limit_enable"
#define OPPO_TP_LIMIT_AREA "oppo_tp_limit_area"


extern int oppo_nvt_blackscreen_test(void);
extern int g_gesture;
extern void nvt_irq_enable(bool enable);
extern int32_t nvt_selftest_open(struct inode *inode, struct file *file);
extern int8_t nvt_ts_check_chip_ver_trim(void);

static struct proc_dir_entry *oppo_baseline_test;
static struct proc_dir_entry *oppo_coordinate;
static struct proc_dir_entry *oppo_delta;
static struct proc_dir_entry *oppo_baseline;
static struct proc_dir_entry *oppo_main_register;
static struct proc_dir_entry *oppo_debug_level;
static struct proc_dir_entry *oppo_gesture;
static struct proc_dir_entry *oppo_irq_depath;
static struct proc_dir_entry *register_info_oppo;
static struct proc_dir_entry *oppo_fw_update;
static struct proc_dir_entry *oppo_device_test;
static struct proc_dir_entry *oppo_blackscreen_test;
static struct proc_dir_entry *oppo_game_switch;
static struct proc_dir_entry *oppo_tp_limit_enable;
static struct proc_dir_entry *oppo_tp_limit_area;

extern int ctpmodule;

#endif
#define SPI_TANSFER_LENGTH  256

#define NORMAL_MODE 0x00
#define TEST_MODE_1 0x21
#define TEST_MODE_2 0x22
#define HANDSHAKING_HOST_READY 0xBB

#define XDATA_SECTOR_SIZE   256

static uint8_t xdata_tmp[2048] = {0};
static int32_t xdata[2048] = {0};
static int32_t xdata_i[2048] = {0};
static int32_t xdata_q[2048] = {0};

static struct proc_dir_entry *NVT_proc_fw_version_entry;
static struct proc_dir_entry *NVT_proc_baseline_entry;
static struct proc_dir_entry *NVT_proc_raw_entry;
static struct proc_dir_entry *NVT_proc_diff_entry;

/*******************************************************
Description:
	Novatek touchscreen change mode function.

return:
	n.a.
*******************************************************/
void nvt_change_mode(uint8_t mode)
{
	uint8_t buf[8] = {0};

	//---set xdata index to EVENT BUF ADDR---
	nvt_set_page(ts->mmap->EVENT_BUF_ADDR | EVENT_MAP_HOST_CMD);

	//---set mode---
	buf[0] = EVENT_MAP_HOST_CMD;
	buf[1] = mode;
	CTP_SPI_WRITE(ts->client, buf, 2);

	if (mode == NORMAL_MODE) {
		buf[0] = EVENT_MAP_HANDSHAKING_or_SUB_CMD_BYTE;
		buf[1] = HANDSHAKING_HOST_READY;
		CTP_SPI_WRITE(ts->client, buf, 2);
		msleep(20);
	}
}

/*******************************************************
Description:
	Novatek touchscreen get firmware pipe function.

return:
	Executive outcomes. 0---pipe 0. 1---pipe 1.
*******************************************************/
uint8_t nvt_get_fw_pipe(void)
{
	uint8_t buf[8]= {0};

	//---set xdata index to EVENT BUF ADDR---
	nvt_set_page(ts->mmap->EVENT_BUF_ADDR | EVENT_MAP_HANDSHAKING_or_SUB_CMD_BYTE);

	//---read fw status---
	buf[0] = EVENT_MAP_HANDSHAKING_or_SUB_CMD_BYTE;
	buf[1] = 0x00;
	CTP_SPI_READ(ts->client, buf, 2);

	//NVT_LOG("FW pipe=%d, buf[1]=0x%02X\n", (buf[1]&0x01), buf[1]);

	return (buf[1] & 0x01);
}

/*******************************************************
Description:
	Novatek touchscreen read meta data function.

return:
	n.a.
*******************************************************/
void nvt_read_mdata(uint32_t xdata_addr, uint32_t xdata_btn_addr)
{
	int32_t i = 0;
	int32_t j = 0;
	int32_t k = 0;
	uint8_t buf[SPI_TANSFER_LENGTH + 1] = {0};
	uint32_t head_addr = 0;
	int32_t dummy_len = 0;
	int32_t data_len = 0;
	int32_t residual_len = 0;

	//---set xdata sector address & length---
	head_addr = xdata_addr - (xdata_addr % XDATA_SECTOR_SIZE);
	dummy_len = xdata_addr - head_addr;
	data_len = ts->x_num * ts->y_num * 2;
	residual_len = (head_addr + dummy_len + data_len) % XDATA_SECTOR_SIZE;

	//printk("head_addr=0x%05X, dummy_len=0x%05X, data_len=0x%05X, residual_len=0x%05X\n", head_addr, dummy_len, data_len, residual_len);

	//read xdata : step 1
	for (i = 0; i < ((dummy_len + data_len) / XDATA_SECTOR_SIZE); i++) {
		//---read xdata by SPI_TANSFER_LENGTH
		for (j = 0; j < (XDATA_SECTOR_SIZE / SPI_TANSFER_LENGTH); j++) {
			//---change xdata index---
			nvt_set_page(head_addr + (XDATA_SECTOR_SIZE * i) + (SPI_TANSFER_LENGTH * j));

			//---read data---
			buf[0] = SPI_TANSFER_LENGTH * j;
			CTP_SPI_READ(ts->client, buf, SPI_TANSFER_LENGTH + 1);

			//---copy buf to xdata_tmp---
			for (k = 0; k < SPI_TANSFER_LENGTH; k++) {
				xdata_tmp[XDATA_SECTOR_SIZE * i + SPI_TANSFER_LENGTH * j + k] = buf[k + 1];
				//printk("0x%02X, 0x%04X\n", buf[k+1], (XDATA_SECTOR_SIZE*i + SPI_TANSFER_LENGTH*j + k));
			}
		}
		//printk("addr=0x%05X\n", (head_addr+XDATA_SECTOR_SIZE*i));
	}

	//read xdata : step2
	if (residual_len != 0) {
		//---read xdata by SPI_TANSFER_LENGTH
		for (j = 0; j < (residual_len / SPI_TANSFER_LENGTH + 1); j++) {
			//---change xdata index---
			nvt_set_page(xdata_addr + data_len - residual_len + (SPI_TANSFER_LENGTH * j));

			//---read data---
			buf[0] = SPI_TANSFER_LENGTH * j;
			CTP_SPI_READ(ts->client, buf, SPI_TANSFER_LENGTH + 1);

			//---copy buf to xdata_tmp---
			for (k = 0; k < SPI_TANSFER_LENGTH; k++) {
				xdata_tmp[(dummy_len + data_len - residual_len) + SPI_TANSFER_LENGTH * j + k] = buf[k + 1];
				//printk("0x%02X, 0x%04x\n", buf[k+1], ((dummy_len+data_len-residual_len) + SPI_TANSFER_LENGTH*j + k));
			}
		}
		//printk("addr=0x%05X\n", (xdata_addr+data_len-residual_len));
	}

	//---remove dummy data and 2bytes-to-1data---
	for (i = 0; i < (data_len / 2); i++) {
		xdata[i] = (int16_t)(xdata_tmp[dummy_len + i * 2] + 256 * xdata_tmp[dummy_len + i * 2 + 1]);
	}

#if TOUCH_KEY_NUM > 0
	//read button xdata : step3
	//---change xdata index---
	nvt_set_page(xdata_btn_addr);
	//---read data---
	buf[0] = (xdata_btn_addr & 0xFF);
	CTP_SPI_READ(ts->client, buf, (TOUCH_KEY_NUM * 2 + 1));

	//---2bytes-to-1data---
	for (i = 0; i < TOUCH_KEY_NUM; i++) {
		xdata[ts->x_num * ts->y_num + i] = (int16_t)(buf[1 + i * 2] + 256 * buf[1 + i * 2 + 1]);
	}
#endif

	//---set xdata index to EVENT BUF ADDR---
	nvt_set_page(ts->mmap->EVENT_BUF_ADDR);
}

/*******************************************************
Description:
	Novatek touchscreen read meta data from IQ to rss function.

return:
	n.a.
*******************************************************/
void nvt_read_mdata_rss(uint32_t xdata_i_addr, uint32_t xdata_q_addr, uint32_t xdata_btn_i_addr, uint32_t xdata_btn_q_addr)
{
	int i = 0;

	nvt_read_mdata(xdata_i_addr, xdata_btn_i_addr);
	memcpy(xdata_i, xdata, ((ts->x_num * ts->y_num + TOUCH_KEY_NUM) * sizeof(int32_t)));

	nvt_read_mdata(xdata_q_addr, xdata_btn_q_addr);
	memcpy(xdata_q, xdata, ((ts->x_num * ts->y_num + TOUCH_KEY_NUM) * sizeof(int32_t)));

	for (i = 0; i < (ts->x_num * ts->y_num + TOUCH_KEY_NUM); i++) {
		xdata[i] = (int32_t)int_sqrt((unsigned long)(xdata_i[i] * xdata_i[i]) + (unsigned long)(xdata_q[i] * xdata_q[i]));
	}
}

/*******************************************************
Description:
    Novatek touchscreen get meta data function.

return:
    n.a.
*******************************************************/
void nvt_get_mdata(int32_t *buf, uint8_t *m_x_num, uint8_t *m_y_num)
{
    *m_x_num = ts->x_num;
    *m_y_num = ts->y_num;
    memcpy(buf, xdata, ((ts->x_num * ts->y_num + TOUCH_KEY_NUM) * sizeof(int32_t)));
}

/*******************************************************
Description:
	Novatek touchscreen firmware version show function.

return:
	Executive outcomes. 0---succeed.
*******************************************************/
static int32_t c_fw_version_show(struct seq_file *m, void *v)
{
	seq_printf(m, "fw_ver=%d, x_num=%d, y_num=%d, button_num=%d\n", ts->fw_ver, ts->x_num, ts->y_num, ts->max_button_num);
	return 0;
}

/*******************************************************
Description:
	Novatek touchscreen xdata sequence print show
	function.

return:
	Executive outcomes. 0---succeed.
*******************************************************/
static int32_t c_show(struct seq_file *m, void *v)
{
	int32_t i = 0;
	int32_t j = 0;

	for (i = 0; i < ts->y_num; i++) {
		for (j = 0; j < ts->x_num; j++) {
			seq_printf(m, "%5d, ", xdata[i * ts->x_num + j]);
		}
		seq_puts(m, "\n");
	}

#if TOUCH_KEY_NUM > 0
	for (i = 0; i < TOUCH_KEY_NUM; i++) {
		seq_printf(m, "%5d, ", xdata[ts->x_num * ts->y_num + i]);
	}
	seq_puts(m, "\n");
#endif

	seq_printf(m, "\n\n");
	return 0;
}

/*******************************************************
Description:
	Novatek touchscreen xdata sequence print start
	function.

return:
	Executive outcomes. 1---call next function.
	NULL---not call next function and sequence loop
	stop.
*******************************************************/
static void *c_start(struct seq_file *m, loff_t *pos)
{
	return *pos < 1 ? (void *)1 : NULL;
}

/*******************************************************
Description:
	Novatek touchscreen xdata sequence print next
	function.

return:
	Executive outcomes. NULL---no next and call sequence
	stop function.
*******************************************************/
static void *c_next(struct seq_file *m, void *v, loff_t *pos)
{
	++*pos;
	return NULL;
}

/*******************************************************
Description:
	Novatek touchscreen xdata sequence print stop
	function.

return:
	n.a.
*******************************************************/
static void c_stop(struct seq_file *m, void *v)
{
	return;
}

const struct seq_operations nvt_fw_version_seq_ops = {
	.start  = c_start,
	.next   = c_next,
	.stop   = c_stop,
	.show   = c_fw_version_show
};

const struct seq_operations nvt_seq_ops = {
	.start  = c_start,
	.next   = c_next,
	.stop   = c_stop,
	.show   = c_show
};

/*******************************************************
Description:
	Novatek touchscreen /proc/nvt_fw_version open
	function.

return:
	n.a.
*******************************************************/
static int32_t nvt_fw_version_open(struct inode *inode, struct file *file)
{
	if (mutex_lock_interruptible(&ts->lock)) {
		return -ERESTARTSYS;
	}

	NVT_LOG("++\n");

#if NVT_TOUCH_ESD_PROTECT
	nvt_esd_check_enable(false);
#endif /* #if NVT_TOUCH_ESD_PROTECT */

	if (nvt_get_fw_info()) {
		mutex_unlock(&ts->lock);
		return -EAGAIN;
	}

	mutex_unlock(&ts->lock);

	NVT_LOG("--\n");

	return seq_open(file, &nvt_fw_version_seq_ops);
}

static const struct file_operations nvt_fw_version_fops = {
	.owner = THIS_MODULE,
	.open = nvt_fw_version_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = seq_release,
};

/*******************************************************
Description:
	Novatek touchscreen /proc/nvt_baseline open function.

return:
	Executive outcomes. 0---succeed.
*******************************************************/
static int32_t nvt_baseline_open(struct inode *inode, struct file *file)
{
	if (mutex_lock_interruptible(&ts->lock)) {
		return -ERESTARTSYS;
	}

	NVT_LOG("++\n");

#if NVT_TOUCH_ESD_PROTECT
	nvt_esd_check_enable(false);
#endif /* #if NVT_TOUCH_ESD_PROTECT */

	if (nvt_clear_fw_status()) {
		mutex_unlock(&ts->lock);
		return -EAGAIN;
	}

	nvt_change_mode(TEST_MODE_2);

	if (nvt_check_fw_status()) {
		mutex_unlock(&ts->lock);
		return -EAGAIN;
	}

	if (nvt_get_fw_info()) {
		mutex_unlock(&ts->lock);
		return -EAGAIN;
	}

	if (ts->carrier_system) {
		nvt_read_mdata_rss(ts->mmap->BASELINE_ADDR, ts->mmap->BASELINE_Q_ADDR,
				ts->mmap->BASELINE_BTN_ADDR, ts->mmap->BASELINE_BTN_Q_ADDR);
	} else {
		nvt_read_mdata(ts->mmap->BASELINE_ADDR, ts->mmap->BASELINE_BTN_ADDR);
	}

	nvt_change_mode(NORMAL_MODE);

	mutex_unlock(&ts->lock);

	NVT_LOG("--\n");

	return seq_open(file, &nvt_seq_ops);
}

static const struct file_operations nvt_baseline_fops = {
	.owner = THIS_MODULE,
	.open = nvt_baseline_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = seq_release,
};

/*******************************************************
Description:
	Novatek touchscreen /proc/nvt_raw open function.

return:
	Executive outcomes. 0---succeed.
*******************************************************/
static int32_t nvt_raw_open(struct inode *inode, struct file *file)
{
	if (mutex_lock_interruptible(&ts->lock)) {
		return -ERESTARTSYS;
	}

	NVT_LOG("++\n");

#if NVT_TOUCH_ESD_PROTECT
	nvt_esd_check_enable(false);
#endif /* #if NVT_TOUCH_ESD_PROTECT */

	if (nvt_clear_fw_status()) {
		mutex_unlock(&ts->lock);
		return -EAGAIN;
	}

	nvt_change_mode(TEST_MODE_2);

	if (nvt_check_fw_status()) {
		mutex_unlock(&ts->lock);
		return -EAGAIN;
	}

	if (nvt_get_fw_info()) {
		mutex_unlock(&ts->lock);
		return -EAGAIN;
	}

	if (ts->carrier_system) {
		if (nvt_get_fw_pipe() == 0)
			nvt_read_mdata_rss(ts->mmap->RAW_PIPE0_ADDR, ts->mmap->RAW_PIPE0_Q_ADDR,
				ts->mmap->RAW_BTN_PIPE0_ADDR, ts->mmap->RAW_BTN_PIPE0_Q_ADDR);
		else
			nvt_read_mdata_rss(ts->mmap->RAW_PIPE1_ADDR, ts->mmap->RAW_PIPE1_Q_ADDR,
				ts->mmap->RAW_BTN_PIPE1_ADDR, ts->mmap->RAW_BTN_PIPE1_Q_ADDR);
	} else {
		if (nvt_get_fw_pipe() == 0)
			nvt_read_mdata(ts->mmap->RAW_PIPE0_ADDR, ts->mmap->RAW_BTN_PIPE0_ADDR);
		else
			nvt_read_mdata(ts->mmap->RAW_PIPE1_ADDR, ts->mmap->RAW_BTN_PIPE1_ADDR);
	}

	nvt_change_mode(NORMAL_MODE);

	mutex_unlock(&ts->lock);

	NVT_LOG("--\n");

	return seq_open(file, &nvt_seq_ops);
}

static const struct file_operations nvt_raw_fops = {
	.owner = THIS_MODULE,
	.open = nvt_raw_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = seq_release,
};

/*******************************************************
Description:
	Novatek touchscreen /proc/nvt_diff open function.

return:
	Executive outcomes. 0---succeed. negative---failed.
*******************************************************/
static int32_t nvt_diff_open(struct inode *inode, struct file *file)
{
	if (mutex_lock_interruptible(&ts->lock)) {
		return -ERESTARTSYS;
	}

	NVT_LOG("++\n");

#if NVT_TOUCH_ESD_PROTECT
	nvt_esd_check_enable(false);
#endif /* #if NVT_TOUCH_ESD_PROTECT */

	if (nvt_clear_fw_status()) {
		mutex_unlock(&ts->lock);
		return -EAGAIN;
	}

	nvt_change_mode(TEST_MODE_2);

	if (nvt_check_fw_status()) {
		mutex_unlock(&ts->lock);
		return -EAGAIN;
	}

	if (nvt_get_fw_info()) {
		mutex_unlock(&ts->lock);
		return -EAGAIN;
	}

	if (ts->carrier_system) {
		if (nvt_get_fw_pipe() == 0)
			nvt_read_mdata_rss(ts->mmap->DIFF_PIPE0_ADDR, ts->mmap->DIFF_PIPE0_Q_ADDR,
				ts->mmap->DIFF_BTN_PIPE0_ADDR, ts->mmap->DIFF_BTN_PIPE0_Q_ADDR);
		else
			nvt_read_mdata_rss(ts->mmap->DIFF_PIPE1_ADDR, ts->mmap->DIFF_PIPE1_Q_ADDR,
				ts->mmap->DIFF_BTN_PIPE1_ADDR, ts->mmap->DIFF_BTN_PIPE1_Q_ADDR);
	} else {
		if (nvt_get_fw_pipe() == 0)
			nvt_read_mdata(ts->mmap->DIFF_PIPE0_ADDR, ts->mmap->DIFF_BTN_PIPE0_ADDR);
		else
			nvt_read_mdata(ts->mmap->DIFF_PIPE1_ADDR, ts->mmap->DIFF_BTN_PIPE1_ADDR);
	}

	nvt_change_mode(NORMAL_MODE);

	mutex_unlock(&ts->lock);

	NVT_LOG("--\n");

	return seq_open(file, &nvt_seq_ops);
}

static const struct file_operations nvt_diff_fops = {
	.owner = THIS_MODULE,
	.open = nvt_diff_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = seq_release,
};

/*******************************************************
Description:
	Novatek touchscreen extra function proc. file node
	initial function.

return:
	Executive outcomes. 0---succeed. -12---failed.
*******************************************************/
#ifdef ODM_WT_EDIT
/* coordinate */
static int32_t c_oppo_coordinate_show(struct seq_file *m, void *v)
{
	struct gesture_info *gesture = &ts->gesture;
	char tmp[256] = {0};
	printk("c_oppo_coordinate_show\n");
	sprintf(tmp, "%d,%d:%d,%d:%d,%d:%d,%d:%d,%d:%d,%d:%d,%d",
		gesture->gesture_type,
		gesture->Point_start.x, gesture->Point_start.y,
		gesture->Point_end.x, gesture->Point_end.y,
		gesture->Point_1st.x, gesture->Point_1st.y,
		gesture->Point_2nd.x, gesture->Point_2nd.y,
		gesture->Point_3rd.x, gesture->Point_3rd.y,
		gesture->Point_4th.x, gesture->Point_4th.y,
		gesture->clockwise);

	/* oppo gesture formate */
	seq_printf(m, "%s\n", tmp);

	return 0;
}

const struct seq_operations oppo_coordinate_seq_ops = {
	.start  = c_start,
	.next   = c_next,
	.stop   = c_stop,
	.show   = c_oppo_coordinate_show
};

static int32_t oppo_coordinate_open(struct inode *inode, struct file *file)
{
	return seq_open(file, &oppo_coordinate_seq_ops);
}

static const struct file_operations oppo_coordinate_fops = {
	.owner = THIS_MODULE,
	.open = oppo_coordinate_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = seq_release,
};

/* main_register */
static int32_t c_main_register_show(struct seq_file *m, void *v)
{
    uint8_t buf[4] = {0};

	//---set xdata index to EVENT BUF ADDR---
	nvt_set_page(ts->mmap->EVENT_BUF_ADDR);

	//---read cmd status---
	buf[0] = 0x5E;
	buf[1] = 0xFF;
	CTP_SPI_READ(ts->client, buf, 2);

	NVT_LOG("PWR_FLAG:%d\n", (buf[1]>> PWR_FLAG) & 0x01);
	seq_printf(m, "PWR_FLAG:%d\n", (buf[1]>> PWR_FLAG) & 0x01);

	NVT_LOG("EDGE_REGECT:%d\n", (buf[1]>> EDGE_REGECT) & 0x01);
	seq_printf(m, "EDGE_REGECT:%d\n", (buf[1]>> EDGE_REGECT) & 0x01);

	NVT_LOG("JITTER_FLAG:%d\n", (buf[1]>> JITTER_FLAG) & 0x01);
	seq_printf(m, "JITTER_FLAG:%d\n", (buf[1]>> JITTER_FLAG) & 0x01);

	return 0;
}

const struct seq_operations oppo_main_register_seq_ops = {
	.start  = c_start,
	.next   = c_next,
	.stop   = c_stop,
	.show   = c_main_register_show
};

static int32_t nvt_main_register_open(struct inode *inode, struct file *file)
{
	return seq_open(file, &oppo_main_register_seq_ops);
}

static const struct file_operations oppo_main_register_fops = {
	.owner = THIS_MODULE,
	.open = nvt_main_register_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = seq_release,
};

/* debug_level */
static ssize_t oppo_debug_level_write(struct file *filp, const char __user *buf,
		size_t count, loff_t *ppos)
{
	unsigned int tmp = 0;
	char cmd[128] = {0};

	if(copy_from_user(cmd, buf, count)) {
		NVT_ERR("input value error\n");
		return -EINVAL;
	}

	if (kstrtouint(cmd, 0, &tmp)) {
		NVT_ERR("kstrtouint error\n");
		return -EINVAL;
	}

	ts->debug_level = tmp;

	NVT_LOG("debug_level is %d\n", ts->debug_level);

	if ((ts->debug_level != 0) && (ts->debug_level != 1) && (ts->debug_level != 2)) {
		NVT_ERR("debug level error %d\n", ts->debug_level);
		ts->debug_level = 0;
	}

	return count;
};

static const struct file_operations oppo_debug_level_fops =
{
	.write = oppo_debug_level_write,
	.owner = THIS_MODULE,
};

/* double_tap_enable */
#define GESTURE_ENABLE   (1)
#define GESTURE_DISABLE  (0)
static ssize_t oppo_gesture_write(struct file *filp, const char __user *buf,size_t count, loff_t *ppos)
{
	//unsigned int tmp = 0;
	//char cmd[5] = {0};
	//int len = 0;
	uint8_t buff[4] = {0};
	char *ptr = NULL;
	/*if (ts->sleep_flag !=0) {
		NVT_LOG("%s, is already suspend",__func__);
		return -1;
	}*/
	ptr = kzalloc(count,GFP_KERNEL);
	if (ptr == NULL){
		NVT_LOG("allocate memory fail\n");
		return -1;
	}
	if (copy_from_user(ptr, buf, count)) {
		NVT_LOG("input value error\n");
		return -EINVAL;
	}
	if (ptr[0] == '1') {
		if (ts->sleep_flag == 1) {
			mutex_lock(&ts->lock);
			NVT_LOG("far away psensor\n");
			ts->gesture_enable = 1;
			g_gesture = 1;
			if ( ctpmodule == 0 ){
				NVT_LOG("%s:nvt:this is tm module\n",__func__);
				nvt_update_firmware(BOOT_UPDATE_FIRMWARE_NAME_TM);
			} else {
				NVT_LOG("%s:nvt:this is xinli module\n",__func__);
				nvt_update_firmware(BOOT_UPDATE_FIRMWARE_NAME);
			}
			buff[0] = EVENT_MAP_HOST_CMD;
			buff[1] = 0x13;
			CTP_SPI_WRITE(ts->client, buff, 2);
			NVT_LOG("ts->gesture_enable =%d:need to download fw\n",ts->gesture_enable);
			mutex_unlock(&ts->lock);
		} else {
			ts->gesture_enable = 1;
			g_gesture = 1;
			NVT_LOG("normal to open gesture:Gesture %d\n",ts->gesture_enable);
		}
	} else if (ptr[0] == '0') {
		ts->gesture_enable = 0;
		g_gesture = 0;
		NVT_LOG("normal clsoe gesture:Gesture %d\n",ts->gesture_enable);

	} else if (ptr[0] == '2') {
		if(ts->sleep_flag == 1) {
			ts->gesture_enable = 0;
			NVT_LOG("need psensor close Gesture \n");
			//g_gesture = 0;
			buff[0] = EVENT_MAP_HOST_CMD;
			buff[1] = 0x11;
			CTP_SPI_WRITE(ts->client, buff, 2);
		}
	}
	else {
		NVT_LOG("can not get the correct gesture control\n");
	}
	/*if (kstrtouint(cmd, 0, &tmp)) {
		NVT_ERR("kstrtouint error\n");
		return -EINVAL;
	}

	ts->gesture_enable = tmp > 0 ? GESTURE_ENABLE : GESTURE_DISABLE;

	NVT_LOG("Gesture %s\n", ts->gesture_enable ? "enable" : "disable");
	*/
	kfree(ptr);
	return count;

}

static ssize_t oppo_gesture_read(struct file *file, char __user *buf,size_t count, loff_t *ppos)
{
	int ret = 0;
	int len;
	uint8_t *ptr = NULL;

    if(*ppos) {
        return 0;
    }

	ptr = kzalloc(count, GFP_KERNEL);
	if (ptr == NULL) {
		NVT_ERR("failed to allocate memory for ptr\n");
		return 0;
	}
	len = snprintf(ptr,count, "%d\n", ts->gesture_enable);
	ret = copy_to_user(buf, ptr, len);

	*ppos += len;
	kfree(ptr);
	return len;

}

static const struct file_operations oppo_gesture_fops =
{
	.write = oppo_gesture_write,
	.read = oppo_gesture_read,
	.owner = THIS_MODULE,
};

/* tp_fw_update */
static ssize_t oppo_fw_update_write(struct file *filp, const char __user *buf,
		size_t count, loff_t *ppos)
{
	unsigned int tmp = 0;
	uint8_t update_type = 0;
	char cmd[128] = {0};
	//int res = 0;
	if (mutex_lock_interruptible(&ts->lock)) {
		return -ERESTARTSYS;
	}

	NVT_LOG("++\n");

#if NVT_TOUCH_ESD_PROTECT
	nvt_esd_check_enable(false);
#endif /* #if NVT_TOUCH_ESD_PROTECT */

	if(copy_from_user(cmd, buf, count)) {
		NVT_ERR("input value error\n");
		return -EINVAL;
	}

	if (kstrtouint(cmd, 0, &tmp)) {
		NVT_ERR("kstrtouint error\n");
		return -EINVAL;
	}

	update_type = tmp;

	NVT_LOG("update_type is %d\n", update_type);
	switch (update_type) {
		case 0:	/* noflash: force update. flash: force update */
			if ( ctpmodule ==0 ) {
                nvt_update_firmware(BOOT_UPDATE_FIRMWARE_NAME_TM);
			} else {
                nvt_update_firmware(BOOT_UPDATE_FIRMWARE_NAME);
			}
			break;
		case 1: /* noflash: do nothing. flash: check fw version and update */
			NVT_LOG("update_type %d. Do update for oppo\n", update_type);
			break;
		default:
			ts->oppo_update_fw_flag = 1;
			if ( ctpmodule ==0 ) {
                nvt_update_firmware(BOOT_UPDATE_FIRMWARE_NAME_TM);
			} else {
                nvt_update_firmware(BOOT_UPDATE_FIRMWARE_NAME);
			}
			ts->oppo_update_fw_flag = 0;
			/*
			res = request_firmware_select(&fw_entry_oppo, BOOT_UPDATE_FIRMWARE_NAME,&ts->client->dev);
			if (res) {
				NVT_ERR("update_type %d. update oppo error\n", update_type);
			}*/
			NVT_LOG("update_type %d update oppo\n", update_type);
	}

	NVT_LOG("--\n");
	mutex_unlock(&ts->lock);

	return count;
};

static const struct file_operations oppo_fw_update_fops =
{
	.write = oppo_fw_update_write,
	.owner = THIS_MODULE,
};

/* irq_depth */
static int32_t c_irq_depath_show(struct seq_file *m, void *v)
{
	struct irq_desc *desc = irq_to_desc(gpio_to_irq(ts->irq_gpio));
	NVT_ERR("depth %d\n", desc->depth);

	seq_printf(m, "%d\n", desc->depth);

	return 0;
}

const struct seq_operations oppo_irq_depath_seq_ops = {
	.start  = c_start,
	.next   = c_next,
	.stop   = c_stop,
	.show   = c_irq_depath_show
};

static int32_t nvt_irq_depath_open(struct inode *inode, struct file *file)
{
	return seq_open(file, &oppo_irq_depath_seq_ops);
}

static const struct file_operations oppo_irq_depath_fops = {
	.owner = THIS_MODULE,
	.open = nvt_irq_depath_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = seq_release,
};

/* oppo_register_info */
struct oppo_register_info {
	uint32_t addr;
	uint32_t len;
} oppo_reg;

/*
 * Example data format: echo 11A60,2 > file_node
 */
static ssize_t oppo_register_info_write(struct file *filp, const char __user *buf,
		size_t count, loff_t *ppos)
{
	uint8_t tmp[5] = {0};
	char cmd[128] = {0};

	/* Error handler */
	if (count != 8) {
		NVT_ERR("count %ld error\n", count);
		return count;
	}

	if(copy_from_user(cmd, buf, count)) {
		NVT_ERR("input value error\n");
		return -EINVAL;
	}

	/* parsing address (Novatek address length: 5 bit) */
	sprintf(tmp, "%c%c%c%c%c", cmd[0], cmd[1], cmd[2], cmd[3], cmd[4]);

	if (kstrtouint(tmp, 16, &oppo_reg.addr)) {
		NVT_ERR("kstrtouint error\n");
		return -EINVAL;
	}

	NVT_LOG("address: 0x%05X\n", oppo_reg.addr);

	/* parsing length */
	sprintf(tmp, "%c", cmd[6]);
	if (kstrtouint(tmp, 10, &oppo_reg.len)) {
		NVT_ERR("kstrtouint error\n");
		return -EINVAL;
	}

	NVT_LOG("len %d\n", oppo_reg.len);

	return count;
}

static ssize_t oppo_register_info_read(struct file *file, char __user *buff,size_t count, loff_t *ppos)
{
	uint8_t *buf = NULL;
	uint8_t *ptr = NULL;
	uint8_t len = 0;
	uint8_t i = 0;
	int32_t ret = 0;

	if(*ppos) {
		return 0;	/* the end */
	}

	if (mutex_lock_interruptible(&ts->lock)) {
		return -ERESTARTSYS;
	}

	if (oppo_reg.len == 0) {
		NVT_ERR("len = %d\n", oppo_reg.len);
		goto fail;
	}

	buf = (uint8_t *)kzalloc(sizeof(uint8_t)*(oppo_reg.len), GFP_KERNEL);
	if (buf == NULL) {
		NVT_ERR("failed to allocate memory for buf\n");
		goto fail;
	}

	ptr = (uint8_t *)kzalloc(sizeof(uint8_t)*(oppo_reg.len)*3+1, GFP_KERNEL);
	if (ptr == NULL) {
		NVT_ERR("failed to allocate memory for ptr\n");
		goto fail;
	}

	/* read data */
	nvt_set_page(oppo_reg.addr);
	buf[0] = oppo_reg.addr & 0x7F;
	CTP_SPI_READ(ts->client, buf, oppo_reg.len + 1);

	/* set index to EVENT_BUF_ADDR */
	nvt_set_page(ts->mmap->EVENT_BUF_ADDR);

	/* copy hex data to string */
	for (i=0 ; i<oppo_reg.len ; i++) {
		len += sprintf(ptr+len, "%02X ", buf[i+1]);
		//NVT_ERR("[%d] buf %02X\n", i, buf[i+1]);
	}

	/* new line */
	len += sprintf(ptr+len, "\n");

	ret = copy_to_user(buff,ptr,len);

	*ppos += len;

fail:
	mutex_unlock(&ts->lock);

	return len;
}

static const struct file_operations oppo_register_info_fops =
{
	.write = oppo_register_info_write,
	.read = oppo_register_info_read,
	.owner = THIS_MODULE,
};
static int32_t oppo_nvt_selftest_open(struct inode *inode, struct file *file)
{
    int32_t ret = 0;
	ts->selft_test_flg = 1;
	ret = nvt_selftest_open(inode,file);
	printk("ts->selft_test_flg1 = %d\n",ts->selft_test_flg);

	return ret;


}
static const struct file_operations oppo_nvt_selftest_fops = {
	.owner = THIS_MODULE,
	.open = oppo_nvt_selftest_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = seq_release,
};

static int32_t oppo_devices_test_show(struct seq_file *m, void *v)
{
	uint8_t buf[4] = {0};
	int flag = 0;
	char *ptr = NULL;
	ptr = kzalloc(sizeof(char)*50, GFP_KERNEL);
	nvt_set_page(ts->mmap->EVENT_BUF_ADDR);
	//---read cmd status---
	buf[0] = 0x5E;
	buf[1] = 0xFF;
	CTP_SPI_READ(ts->client, buf, 2);
	flag = (buf[1]>> EDGE_REGECT) & 0x01;
	if ( flag ) {
        ptr = "Noatek Spi Device";
	} else {
        ptr = "Error!";
	}
    seq_printf(m, "%s\n",ptr);
	return 0;
}

const struct seq_operations oppo_devices_test_ops = {
	.start  = c_start,
	.next   = c_next,
	.stop   = c_stop,
	.show   = oppo_devices_test_show
};

static int32_t oppo_devices_test_open(struct inode *inode, struct file *file)
{
	return seq_open(file, &oppo_devices_test_ops);
}
static const struct file_operations oppo_devices_fops = {
	.owner = THIS_MODULE,
	.open = oppo_devices_test_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = seq_release,
};

static ssize_t oppo_blackscreen_test_read(struct file *file, char __user *buf, size_t count, loff_t *ppos)
{
    int ret = 0;
    int retry = 20;
	int len = 0;
	char *ptr = NULL;
    NVT_LOG("%s %ld %lld\n", __func__, count, *ppos);
	if ( *ppos ) {
    	printk("is already read the file\n");
    	return 0;
	}
    *ppos += count;

    if (!ts->gesture_test.flag) {
		printk("ts->gesture_test.flag ==== %d\n",ts->gesture_test.flag);
		//return 0;
	}

    ts->gesture_test.message = kzalloc(256, GFP_KERNEL);
    if (ts->gesture_test.message==NULL) {
        NVT_LOG("failed to alloc gesture_test.message memory\n");
        return 0;
    }
    ptr = kzalloc(256, GFP_KERNEL);
    if (ptr == NULL) {
        NVT_LOG("failed to alloc ptr memory\n");
        return 0;
    }

    //wait until tp is in sleep, then sleep 500ms to make sure tp is in gesture mode
    do {
        if (ts->sleep_flag) {
            msleep(500);
            break;
        }
        msleep(200);
    } while(--retry);

    NVT_LOG("%s retry times %d\n", __func__, retry);
    if (retry == 0 && !ts->sleep_flag) {
		len = snprintf(ts->gesture_test.message,count,"%s\n", "1 errors: not in sleep");
        goto OUT;
    }

    //mutex_lock(&ts->lock);
    ret = oppo_nvt_blackscreen_test();
	if (ret){
		NVT_LOG("can not complete blackscreen test\n");
	}
    //mutex_unlock(&ts->lock);
	len = snprintf(ptr, count,"%s\n",ts->gesture_test.message);
OUT:
    ts->gesture_test.flag = 0;
	g_gesture = ts->g_gesture_bak;
    ts->gesture_enable = ts->gesture_test.gesture_backup;
	if (ts->gesture_test.gesture_backup == 0) {
		nvt_irq_enable(0);
	}
	ret=copy_to_user(buf,ptr,len);

    kfree(ts->gesture_test.message);
	kfree(ptr);
    return len;
}

static ssize_t oppo_blackscreen_test_write(struct file *file, const char __user *userbuf, size_t count, loff_t *ppos)
{
    int value = 0;
	//char buf[4] = {0};
	char *ptr = NULL;
	ptr = kzalloc(count,GFP_KERNEL);
	if ( ptr == NULL ) {
		return -1;
	}
	if ( copy_from_user(ptr, userbuf, count) ) {
		NVT_LOG("%s: copy from user error.", __func__);
		return -1;
	}
	sscanf(ptr, "%d", &value);

	printk("value ============%d\n",value);

    ts->gesture_test.gesture_backup = ts->gesture_enable;
	ts->g_gesture_bak = g_gesture;
    ts->gesture_enable = 1;
	g_gesture = 1;
    ts->gesture_test.flag = !!value;
	kfree(ptr);
    return count;
}

static const struct file_operations black_screen_test_fops = {
    .owner = THIS_MODULE,
    .read  = oppo_blackscreen_test_read,
    .write = oppo_blackscreen_test_write,
};



#endif

/* game_switch_enable */
static ssize_t oppo_game_switch_write(struct file *filp, const char __user *buf,
		size_t count, loff_t *ppos)
{
	unsigned int tmp = 0;
	char cmd[128] = {0};

	if(copy_from_user(cmd, buf, count)) {
		NVT_ERR("input value error\n");
		return -EINVAL;
	}

	if (kstrtouint(cmd, 0, &tmp)) {
		NVT_ERR("kstrtouint error\n");
		return -EINVAL;
	}
	NVT_LOG("game switch enable is %d\n", tmp);
	tmp = !!tmp;

	if(nvt_mode_switch(MODE_GAME, tmp)) {
		NVT_ERR("game switch enable fail!\n");
	}

	return count;
};

static const struct file_operations oppo_game_switch_fops =
{
	.write = oppo_game_switch_write,
	.owner = THIS_MODULE,
};

/* oppo_tp_limit_enable */
static ssize_t oppo_tp_limit_write(struct file *filp, const char __user *buf,
		size_t count, loff_t *ppos)
{
	unsigned int tmp = 0;
	char cmd[128] = {0};

	if(copy_from_user(cmd, buf, count)) {
		NVT_ERR("input value error\n");
		return -EINVAL;
	}
	sscanf(buf, "%x", &tmp);
	ts->edge_limit.limit_control = tmp;
	ts->edge_limit.limit_00 = tmp & 0x01;
	ts->edge_limit.limit_lu = (tmp >> 1) & 0x01;
	ts->edge_limit.limit_ru = (tmp >> 2) & 0x01;
	ts->edge_limit.limit_lb = (tmp >> 3) & 0x01;
	ts->edge_limit.limit_rb = (tmp >> 4) & 0x01;
	printk("ts->edge_limit.limit_00 == %d,ts->edge_limit.limit_lu ==%d,ru =%d,lb=%d,rb=%d\n",ts->edge_limit.limit_00,
		ts->edge_limit.limit_lu,ts->edge_limit.limit_ru,ts->edge_limit.limit_lb,ts->edge_limit.limit_rb);
	/*if (kstrtouint(cmd, 0, &tmp)) {
		NVT_ERR("kstrtouint error\n");
		return -EINVAL;
	}

	NVT_LOG("edge reject enable is %d\n", tmp);
	tmp = !!tmp;
	*/

	if(nvt_mode_switch(MODE_EDGE, ts->edge_limit.limit_00)) {
		NVT_ERR("edge reject enable fail!\n");
	}

	return count;
};
static ssize_t oppo_tp_limit_read(struct file *file, char __user *buf, size_t count, loff_t *ppos)
{
	ssize_t ret = 0;
	char *ptr = NULL;
	int lens = 0;
	if (*ppos){
		printk("is not at the page start \n");
		return 0;
	}
	*ppos +=count;
	ptr = kzalloc(256,GFP_KERNEL);
	lens += snprintf(ptr + lens,count-lens,"limit_control = %d\n",ts->edge_limit.limit_control);
	lens += snprintf(ptr + lens,count-lens,"limit_00 = %d\n",ts->edge_limit.limit_00);
	lens += snprintf(ptr + lens,count-lens,"limit_lu = %d\n",ts->edge_limit.limit_lu);
	lens += snprintf(ptr + lens,count-lens,"limit_ru = %d\n",ts->edge_limit.limit_ru);
	lens += snprintf(ptr + lens,count-lens,"limit_lb = %d\n",ts->edge_limit.limit_lb);
	lens += snprintf(ptr + lens,count-lens,"limit_rb = %d\n",ts->edge_limit.limit_rb);

	ret = copy_to_user(buf,ptr,lens);
	if (ret)
		printk("copy_to_user fail \n");

	return lens;


}
static ssize_t oppo_tp_area_read(struct file *file, char __user *buf, size_t count, loff_t *ppos)
{

	int ret = 0;
	int len = 0;
	char *ptr = NULL;
	if ( *ppos ) {
    	printk("is already read the file\n");
    	return 0;
	}
	ptr = (char*)kzalloc(count,GFP_KERNEL);
	len += snprintf(ptr+len, count-len,"left_u(x1,y1)=(%d,%d)    ",ts->nvt_limit_area.area_xlu,ts->nvt_limit_area.area_ylu);
	len += snprintf(ptr+len, count-len,"right_u(x2,y2)=(%d,%d)\n",ts->nvt_limit_area.area_xru,ts->nvt_limit_area.area_yru);
	len += snprintf(ptr+len, count-len,"left_b(x1,y1)=(%d,%d)    ",ts->nvt_limit_area.area_xlb,ts->nvt_limit_area.area_ylb);
	len += snprintf(ptr+len, count-len,"right_b(x2,y2)=(%d,%d)\n",ts->nvt_limit_area.area_xrb,ts->nvt_limit_area.area_yrb);
	ret = copy_to_user(buf,ptr,len);
	*ppos = len;
	kfree(ptr);
	return len;


}
static ssize_t oppo_tp_area_write(struct file *filp, const char __user *buffer,size_t count, loff_t *ppos)
{
    char buf[8] = {0};
    int  temp;
	if (buffer != NULL)
	{
	    if (copy_from_user(buf, buffer, count)) {
	        printk("%s: read proc input error.\n", __func__);
	        return count;
	    }
	}
    sscanf(buf, "%x", &temp);

    if (temp < 0 || temp > 10)
        return count;
	ts->nvt_limit_area.limit_area = temp;
    ts->nvt_limit_area.area_xlu   = (ts->nvt_limit_area.limit_area*1000)/100;
    ts->nvt_limit_area.area_xru   = TOUCH_DEFAULT_MAX_WIDTH - ts->nvt_limit_area.area_xlu;
    ts->nvt_limit_area.area_xlb    = 2 * ts->nvt_limit_area.area_xlu;
    ts->nvt_limit_area.area_xrb   = TOUCH_DEFAULT_MAX_WIDTH - (2 * ts->nvt_limit_area.area_xlu);

    NVT_LOG("limit_area = %d; left_x1 = %d; right_x1 = %d; left_x2 = %d; right_x2 = %d\n",
           ts->nvt_limit_area.limit_area, ts->nvt_limit_area.area_xlu, ts->nvt_limit_area.area_xru,ts->nvt_limit_area.area_xlb, ts->nvt_limit_area.area_xrb);

    ts->nvt_limit_area.area_ylu   = (ts->nvt_limit_area.limit_area*1000)/100;
    ts->nvt_limit_area.area_yru   =  ts->nvt_limit_area.area_ylu;
    ts->nvt_limit_area.area_ylb   = TOUCH_DEFAULT_MAX_HEIGHT-2 * ts->nvt_limit_area.area_ylu;
    ts->nvt_limit_area.area_yrb   = TOUCH_DEFAULT_MAX_HEIGHT - (2 * ts->nvt_limit_area.area_ylu);

    return count;

}

static const struct file_operations oppo_tp_limit_fops =
{
	.write = oppo_tp_limit_write,
	.read = oppo_tp_limit_read,
	.owner = THIS_MODULE,
};

static const struct file_operations oppo_tp_area_fops =
{
	.write = oppo_tp_area_write,
	.read = oppo_tp_area_read,
	.owner = THIS_MODULE,
};


int32_t nvt_extra_proc_init(void)
{
#ifdef ODM_WT_EDIT
	struct proc_dir_entry *oppo_touchpanel_proc = NULL;
	struct proc_dir_entry *debug_info = NULL;
	oppo_touchpanel_proc = proc_mkdir(OPPO_TOUCHPANEL_NAME, NULL);
	if( oppo_touchpanel_proc == NULL ) {
		NVT_ERR("create oppo_touchpanel_proc fail\n");
		return -1;
	}
	debug_info = proc_mkdir(OPPO_DEBUG_INFO, oppo_touchpanel_proc);
	if( debug_info == NULL ) {
		NVT_ERR("create debug_info fail\n");
		return -1;
	}
	oppo_baseline_test = proc_create(OPPO_BASELINE_TEST,0666,oppo_touchpanel_proc,&oppo_nvt_selftest_fops);
	if ( oppo_baseline_test == NULL ) {
		NVT_ERR("create proc/touchpanel/baseline_test Failed!\n");
		return -1;
	}
	oppo_coordinate= proc_create(OPPO_COORDINATE,0444,oppo_touchpanel_proc,&oppo_coordinate_fops);
	if ( oppo_coordinate == NULL ) {
		NVT_ERR("create proc/touchpanel/coordinate Failed!\n");
		return -1;
	}
	oppo_delta= proc_create(OPPO_DELTA,0664,debug_info,&nvt_diff_fops);
	if ( oppo_delta == NULL ) {
		NVT_ERR("create proc/touchpanel/debug_info/delta Failed!\n");
		return -1;
	}
	oppo_baseline = proc_create(OPPO_BASELINE,0664,debug_info,&nvt_baseline_fops);
	if ( oppo_baseline == NULL ) {
		NVT_ERR("create proc/touchpanel/debug_info/baseline Failed!\n");
		return -1;
	}
	oppo_main_register= proc_create(OPPO_MAIN_REGISTER,0664,debug_info,&oppo_main_register_fops);
	if ( oppo_main_register == NULL ) {
		NVT_ERR("create proc/touchpanel/debug_info/main_register Failed!\n");
		return -1;
	}
	oppo_debug_level= proc_create(OPPO_DEBUG_LEVEL,0644,oppo_touchpanel_proc,&oppo_debug_level_fops);
	if ( oppo_debug_level == NULL ) {
		NVT_ERR("create proc/touchpanel/debug_level Failed!\n");
		return -1;
	}
	oppo_gesture= proc_create(OPPO_GESTURE,0666,oppo_touchpanel_proc,&oppo_gesture_fops);
	if ( oppo_gesture == NULL ) {
		NVT_ERR("create proc/touchpanel/double_tap_enable Failed!\n");
		return -1;
	}

	oppo_irq_depath = proc_create(OPPO_IRQ_DEPATH, 0666, oppo_touchpanel_proc, &oppo_irq_depath_fops);
	if (oppo_irq_depath == NULL) {
        NVT_ERR("create proc/touchpanel/irq_depath Failed!\n");
		return -ENOMEM;
	}

	register_info_oppo = proc_create(OPPO_REGISTER_INFO,0664, oppo_touchpanel_proc, &oppo_register_info_fops);
	if (register_info_oppo == NULL) {
        NVT_ERR("create proc/touchpanel/oppo_register_info Failed!\n");
		return -ENOMEM;
	}

	oppo_fw_update = proc_create(OPPO_FW_UPDATE,0664, oppo_touchpanel_proc, &oppo_fw_update_fops);
	if (oppo_fw_update == NULL) {
        NVT_ERR("create proc/touchpanel/tp_fw_update Failed!\n");
		return -ENOMEM;
	}

	oppo_device_test = proc_create(OPPO_DEVICE_TEST,0664, oppo_touchpanel_proc, &oppo_devices_fops);
	if (oppo_device_test == NULL) {
        NVT_ERR("create proc/touchpanel/oppo_device_test Failed!\n");
		return -ENOMEM;
	}
	oppo_blackscreen_test = proc_create(OPPO_BLACKSCREEN_TEST,0666, oppo_touchpanel_proc, &black_screen_test_fops);
	if (oppo_blackscreen_test == NULL) {
        NVT_ERR("create proc/touchpanel/blackscreen_test Failed!\n");
		return -ENOMEM;
	}
	oppo_game_switch = proc_create(OPPO_GAME_SWITCH,0664, oppo_touchpanel_proc, &oppo_game_switch_fops);
	if (oppo_game_switch == NULL) {
        NVT_ERR("create proc/touchpanel/oppo_game_switch_enable Failed!\n");
		return -ENOMEM;
	}

	oppo_tp_limit_enable = proc_create(OPPO_TP_LIMIT_ENABLE,0664, oppo_touchpanel_proc, &oppo_tp_limit_fops);
	if (oppo_tp_limit_enable == NULL) {
        NVT_ERR("create proc/touchpanel/oppo_tp_limit_enable Failed!\n");
		return -ENOMEM;
	}
	oppo_tp_limit_area = proc_create(OPPO_TP_LIMIT_AREA,0664, oppo_touchpanel_proc, &oppo_tp_area_fops);
	if (oppo_tp_limit_area == NULL) {
        NVT_ERR("create proc/touchpanel/oppo_tp_limit_area Failed!\n");
		return -ENOMEM;
	}

#endif
	NVT_proc_fw_version_entry = proc_create(NVT_FW_VERSION, 0444, NULL,&nvt_fw_version_fops);
	if (NVT_proc_fw_version_entry == NULL) {
		NVT_ERR("create proc/nvt_fw_version Failed!\n");
		return -ENOMEM;
	} else {
		NVT_LOG("create proc/nvt_fw_version Succeeded!\n");
	}

	NVT_proc_baseline_entry = proc_create(NVT_BASELINE, 0444, NULL,&nvt_baseline_fops);
	if (NVT_proc_baseline_entry == NULL) {
		NVT_ERR("create proc/nvt_baseline Failed!\n");
		return -ENOMEM;
	} else {
		NVT_LOG("create proc/nvt_baseline Succeeded!\n");
	}

	NVT_proc_raw_entry = proc_create(NVT_RAW, 0444, NULL,&nvt_raw_fops);
	if (NVT_proc_raw_entry == NULL) {
		NVT_ERR("create proc/nvt_raw Failed!\n");
		return -ENOMEM;
	} else {
		NVT_LOG("create proc/nvt_raw Succeeded!\n");
	}

	NVT_proc_diff_entry = proc_create(NVT_DIFF, 0444, NULL,&nvt_diff_fops);
	if (NVT_proc_diff_entry == NULL) {
		NVT_ERR("create proc/nvt_diff Failed!\n");
		return -ENOMEM;
	} else {
		NVT_LOG("create proc/nvt_diff Succeeded!\n");
	}

	return 0;
}
#endif
