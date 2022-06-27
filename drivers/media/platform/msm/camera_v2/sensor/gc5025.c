/* Copyright (c) 2013-2014, The Linux Foundation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */
#include "msm_sensor.h"
#include "msm_cci.h"
#include "msm_camera_io_util.h"
#include "msm_camera_i2c_mux.h"
#include<linux/kernel.h>


#define GC5025_SENSOR_NAME "gc5025"
DEFINE_MSM_MUTEX(gc5025_mut);

//#undef CDBG
#define CDBG(fmt, args...) pr_debug(fmt, ##args)
#define CINFO(fmt, args...) pr_info(fmt, ##args)
#define CERR(fmt, args...) pr_err(fmt, ##args)
//#define CDBG(fmt, args...) do { } while (0)

#define IMAGE_NORMAL_MIRROR
//#define IMAGE_H_MIRROR
//#define IMAGE_V_MIRROR
//#define IMAGE_HV_MIRROR

#define DD_PARAM_QTY 		200
#define WINDOW_WIDTH  		0x0a30 //2608 max effective pixels
#define WINDOW_HEIGHT 		0x079c //1948
#define RG_TYPICAL    		0x0400
#define BG_TYPICAL			0x0400
#define INFO_ROM_START		0x01
#define INFO_WIDTH       	0x08
#define WB_ROM_START      	0x11
#define WB_WIDTH          	0x05
#define GOLDEN_ROM_START  	0x1c
#define GOLDEN_WIDTH      	0x05
#define REG_ROM_START       0x62
#define BASE_ADDR_DR        0x26

typedef struct otp_gc5025
{
	uint16_t module_id;
	uint16_t lens_id;
	uint16_t vcm_id;
	uint16_t vcm_driver_id;
	uint16_t year;
	uint16_t month;
	uint16_t day;
	uint16_t rg_gain;
	uint16_t bg_gain;
	uint16_t wb_flag;
	uint16_t golden_flag;
	uint16_t dd_param_x[DD_PARAM_QTY];
	uint16_t dd_param_y[DD_PARAM_QTY];
	uint16_t dd_param_type[DD_PARAM_QTY];
	uint16_t dd_cnt;
	uint16_t dd_flag;
	uint16_t golden_rg;
	uint16_t golden_bg;
	uint16_t reg_addr[10];
	uint16_t reg_value[10];
	uint16_t reg_num;
}gc5025_otp;

static gc5025_otp gc5025_otp_info;

typedef enum{
	otp_page0=0,
	otp_page1,
}otp_page;

typedef enum{
	otp_close=0,
	otp_open,
}otp_state;

static uint16_t gc5025_Sensor_ReadReg(
	struct msm_sensor_ctrl_t *s_ctrl,uint8_t reg_addr)
{
	uint16_t reg_value = 0;

	s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_read(
				s_ctrl->sensor_i2c_client,
				reg_addr,
				&reg_value, MSM_CAMERA_I2C_BYTE_DATA);
	return reg_value ;
}

static void gc5025_Sensor_WriteReg(
	struct msm_sensor_ctrl_t *s_ctrl,uint8_t reg_addr,uint8_t reg_value)
{

	s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_write(
		s_ctrl->sensor_i2c_client,reg_addr,reg_value, MSM_CAMERA_I2C_BYTE_DATA);
}


static uint8_t gc5025_read_otp(struct msm_sensor_ctrl_t *s_ctrl,uint8_t addr)
{
	uint8_t value;
	uint8_t regd4;
	uint16_t realaddr = addr * 8;
	regd4 = gc5025_Sensor_ReadReg(s_ctrl,0xd4);

	gc5025_Sensor_WriteReg(s_ctrl,0xfe,0x00);
	gc5025_Sensor_WriteReg(s_ctrl,0xd4,(regd4&0xfc)+((realaddr>>8)&0x03));
	gc5025_Sensor_WriteReg(s_ctrl,0xd5,realaddr&0xff);
	gc5025_Sensor_WriteReg(s_ctrl,0xf3,0x20);
	value = gc5025_Sensor_ReadReg(s_ctrl,0xd7);

	return value;
}

static void gc5025_read_otp_group(struct msm_sensor_ctrl_t *s_ctrl,uint8_t addr,uint8_t* buff,int size)
{
	uint8_t i;
	uint8_t regd4;
	uint16_t realaddr = addr * 8;
	regd4 = gc5025_Sensor_ReadReg(s_ctrl,0xd4);

	gc5025_Sensor_WriteReg(s_ctrl,0xfe,0x00);
	gc5025_Sensor_WriteReg(s_ctrl,0xd4,(regd4&0xfc)+((realaddr>>8)&0x03));
	gc5025_Sensor_WriteReg(s_ctrl,0xd5,realaddr);
	gc5025_Sensor_WriteReg(s_ctrl,0xf3,0x20);
	gc5025_Sensor_WriteReg(s_ctrl,0xf3,0x88);

	for(i=0;i<size;i++)
	{
		buff[i] = gc5025_Sensor_ReadReg(s_ctrl,0xd7);
	}
}

static void gc5025_select_page_otp(struct msm_sensor_ctrl_t *s_ctrl,otp_page otp_select_page)
{
	uint8_t page;

	gc5025_Sensor_WriteReg(s_ctrl, 0xfe,0x00);
	page = gc5025_Sensor_ReadReg(s_ctrl, 0xd4);

	switch(otp_select_page)
	{
	case otp_page0:
		page = page & 0xfb;
		break;
	case otp_page1:
		page = page | 0x04;
		break;
	default:
		break;
	}

	msleep(5);
	gc5025_Sensor_WriteReg(s_ctrl, 0xd4,page);

}

static void gc5025_gcore_read_otp_info(struct msm_sensor_ctrl_t *s_ctrl)
{
	uint8_t flagdd,flag1,flag_golden,flag_chipversion;
	uint8_t i, j, cnt = 0;
	uint16_t check;
	uint8_t total_number=0;
	uint8_t ddtempbuff[50*4];
	uint8_t ddchecksum;
    //uint8_t index = 0;
	//uint8_t info[8];
	//uint8_t wb[5];
	//uint8_t golden[5];
	memset(&gc5025_otp_info,0,sizeof(gc5025_otp));

	/*TODO*/
	gc5025_select_page_otp(s_ctrl,otp_page0);
	flagdd = gc5025_read_otp(s_ctrl,0x00);
	CINFO("GC5025_OTP_DD : flag_dd = 0x%x\n",flagdd);
	CINFO("GC5025_OTP_DD : 0x80= 0x%x\n",gc5025_Sensor_ReadReg(s_ctrl,0x80));
	flag_chipversion = gc5025_read_otp(s_ctrl,0x7f);

	//DD
	switch(flagdd&0x03)
	{
	case 0x00:
		CERR("GC5025_OTP_DD is Empty !!\n");
		gc5025_otp_info.dd_flag = 0x00;
		break;
	case 0x01:
		CINFO("GC5025_OTP_DD is Valid!!\n");
		total_number = gc5025_read_otp(s_ctrl,0x01) + gc5025_read_otp(s_ctrl,0x02);
		CINFO("GC5025_OTP : total_number = %d\n",total_number);

		if (total_number <= 31)
		{
			gc5025_read_otp_group(s_ctrl,0x03, &ddtempbuff[0], total_number * 4);
		}
		else
		{
			gc5025_read_otp_group(s_ctrl,0x03, &ddtempbuff[0], 31 * 4);
			gc5025_select_page_otp(s_ctrl,otp_page1);
			gc5025_read_otp_group(s_ctrl,0x29, &ddtempbuff[31 * 4], (total_number - 31) * 4);
		}

		/*DD check sum*/
		gc5025_select_page_otp(s_ctrl,otp_page1);
		ddchecksum = gc5025_read_otp(s_ctrl,0x61);
		check = total_number;
		for(i = 0; i < 4 * total_number; i++)
		{
			check += ddtempbuff[i];
		}
		if((check % 255 + 1) == ddchecksum)
		{
			CINFO("GC5025_OTP_DD : DD check sum correct! checksum = 0x%x\n", ddchecksum);
		}
		else
		{
			CERR("GC5025_OTP_DD : DD check sum error! otpchecksum = 0x%x, sum = 0x%x\n", ddchecksum, (check % 255 + 1));
		}

		for(i=0; i<total_number; i++)
		{
			CDBG("GC5025_OTP_DD:index = %d, data[0] = %x , data[1] = %x , data[2] = %x ,data[3] = %x \n",\
				i, ddtempbuff[4 * i], ddtempbuff[4 * i + 1], ddtempbuff[4 * i + 2], ddtempbuff[4 * i + 3]);

			if (ddtempbuff[4 * i + 3] & 0x10)
			{
				switch (ddtempbuff[4 * i + 3] & 0x0f)
				{
				case 3:
					for (j = 0; j < 4; j++)
					{
						gc5025_otp_info.dd_param_x[cnt] = (((uint16_t)ddtempbuff[4 * i + 1] & 0x0f) << 8) + ddtempbuff[4 * i];
						gc5025_otp_info.dd_param_y[cnt] = ((uint16_t)ddtempbuff[4 * i + 2] << 4) + ((ddtempbuff[4 * i + 1] & 0xf0) >> 4) + j;
						gc5025_otp_info.dd_param_type[cnt++] = 2;
					}
					break;
				case 4:
					for (j = 0; j < 2; j++)
					{
						gc5025_otp_info.dd_param_x[cnt] = (((uint16_t)ddtempbuff[4 * i + 1] & 0x0f) << 8) + ddtempbuff[4 * i];
						gc5025_otp_info.dd_param_y[cnt] = ((uint16_t)ddtempbuff[4 * i + 2] << 4) + ((ddtempbuff[4 * i + 1] & 0xf0) >> 4) + j;
						gc5025_otp_info.dd_param_type[cnt++] = 2;
					}
					break;
				default:
					gc5025_otp_info.dd_param_x[cnt] = (((uint16_t)ddtempbuff[4 * i + 1] & 0x0f) << 8) + ddtempbuff[4 * i];
					gc5025_otp_info.dd_param_y[cnt] = ((uint16_t)ddtempbuff[4 * i + 2] << 4) + ((ddtempbuff[4 * i + 1] & 0xf0) >> 4);
					gc5025_otp_info.dd_param_type[cnt++] = ddtempbuff[4 * i + 3] & 0x0f;
					break;
				}
			}
			else
			{
				CERR("GC5025_OTP_DD:check_id[%d] = %x,checkid error!!\n", i, ddtempbuff[4 * i + 3] & 0xf0);
			}
		}
		gc5025_otp_info.dd_cnt = cnt;
		gc5025_otp_info.dd_flag = 0x01;
		break;
	case 0x02:
	case 0x03:
		printk("GC5025_OTP_DD is Invalid !!\n");
		gc5025_otp_info.dd_flag = 0x02;
		break;
	default :
		break;
	}

	gc5025_select_page_otp(s_ctrl,otp_page1);
	flag1 = gc5025_read_otp(s_ctrl,0x00);
	flag_golden = gc5025_read_otp(s_ctrl,0x1b);
	CINFO("GC5025_OTP : flag1 = 0x%x , flag_golden = 0x%x\n",flag1,flag_golden);

//INFO&WB
/*
	for(index=0;index<2;index++)
	{
		switch((flag1>>(4 + 2 * index))&0x03)
		{
		case 0x00:
			printk("GC5025_OTP_INFO group %d is Empty !!\n", index + 1);
			break;
		case 0x01:
			printk("GC5025_OTP_INFO group %d is Valid !!\n", index + 1);
			check = 0;
			gc5025_read_otp_group(s_ctrl,INFO_ROM_START + index * INFO_WIDTH, &info[0], INFO_WIDTH);
			for (i = 0; i < INFO_WIDTH - 1; i++)
			{
				check += info[i];
			}
			if ((check % 255 + 1) == info[INFO_WIDTH-1])
			{
				gc5025_otp_info.module_id = info[0];
				gc5025_otp_info.lens_id = info[1];
				gc5025_otp_info.vcm_driver_id = info[2];
				gc5025_otp_info.vcm_id = info[3];
				gc5025_otp_info.year = info[4];
				gc5025_otp_info.month = info[5];
				gc5025_otp_info.day = info[6];
			}
			else
			{
				printk("GC5025_OTP_INFO Check sum %d Error !!\n", index + 1);
			}
			break;
		case 0x02:
		case 0x03:
			printk("GC5025_OTP_INFO group %d is Invalid !!\n", index + 1);
			break;
		default :
			break;
		}

		switch((flag1>>(2 * index))&0x03)
		{
		case 0x00:
			printk("GC5025_OTP_WB group %d is Empty !!\n", index + 1);
			gc5025_otp_info.wb_flag = gc5025_otp_info.wb_flag|0x00;
			break;
		case 0x01:
			printk("GC5025_OTP_WB group %d is Valid !!\n", index + 1);
			check = 0;
			gc5025_read_otp_group(s_ctrl,WB_ROM_START + index * WB_WIDTH, &wb[0], WB_WIDTH);

			for (i = 0; i < WB_WIDTH - 1; i++)
			{
				check += wb[i];
			}

			if ((check % 255 + 1) == wb[WB_WIDTH - 1])
			{
				gc5025_otp_info.rg_gain = (((wb[0]<<8)&0xff00)|wb[1]) > 0 ? (((wb[0]<<8)&0xff00)|wb[1]) : 0x400;
				gc5025_otp_info.bg_gain = (((wb[2]<<8)&0xff00)|wb[3]) > 0 ? (((wb[2]<<8)&0xff00)|wb[3]) : 0x400;
				gc5025_otp_info.wb_flag = gc5025_otp_info.wb_flag|0x01;
				printk("GC5025_OTP_WB wb[0]=0x%x,wb[1]=0x%x,wb[2]=0x%x,wb[3]=0x%x\n", wb[0],wb[1],wb[2],wb[3]);
			}
			else
			{
				printk("GC5025_OTP_WB Check sum %d Error !!\n", index + 1);
			}
			break;
		case 0x02:
		case 0x03:
			printk("GC5025_OTP_WB group %d is Invalid !!\n", index + 1);
			gc5025_otp_info.wb_flag = gc5025_otp_info.wb_flag|0x02;
			break;
		default :
			break;
		}

		switch((flag_golden>>(2 * index))&0x03)
		{
		case 0x00:
			printk("GC5025_OTP_GOLDEN group %d is Empty !!\n", index + 1);
			gc5025_otp_info.golden_flag = gc5025_otp_info.golden_flag|0x00;
			break;
		case 0x01:
			printk("GC5025_OTP_GOLDEN group %d is Valid !!\n", index + 1);
			check = 0;
			gc5025_read_otp_group(s_ctrl,GOLDEN_ROM_START + index * GOLDEN_WIDTH, &golden[0], GOLDEN_WIDTH);
			for (i = 0; i < GOLDEN_WIDTH - 1; i++)
			{
				check += golden[i];
			}
			if ((check % 255 + 1) == golden[GOLDEN_WIDTH - 1])
			{
				gc5025_otp_info.golden_rg = (((golden[0]<<8)&0xff00)|golden[1]) > 0 ? (((golden[0]<<8)&0xff00)|golden[1]) : RG_TYPICAL;
				gc5025_otp_info.golden_bg = (((golden[2]<<8)&0xff00)|golden[3]) > 0 ? (((golden[2]<<8)&0xff00)|golden[3]) : BG_TYPICAL;
				gc5025_otp_info.golden_flag = gc5025_otp_info.golden_flag|0x01;
			}
			else
			{
				printk("GC5025_OTP_GOLDEN Check sum %d Error !!\n", index + 1);
			}
			break;
		case 0x02:
		case 0x03:
			printk("GC5025_OTP_GOLDEN group %d is Invalid !!\n", index + 1);
			gc5025_otp_info.golden_flag = gc5025_otp_info.golden_flag|0x02;
			break;
		default :
			break;
		}
	}
*/
/*For Chip Version*/
	CINFO("GC5025_OTP_CHIPVESION : flag_chipversion = 0x%x\n",flag_chipversion);

	switch((flag_chipversion>>4)&0x03)
	{
	case 0x00:
		CERR("GC5025_OTP_CHIPVERSION is Empty !!\n");
		break;
	case 0x01:
		CINFO("GC5025_OTP_CHIPVERSION is Valid !!\n");
		i = 0;
		do{
			gc5025_otp_info.reg_addr[i] = gc5025_read_otp(s_ctrl,REG_ROM_START + i*2 ) ;
			gc5025_otp_info.reg_value[i] = gc5025_read_otp(s_ctrl,REG_ROM_START + i*2 + 1 ) ;
			CDBG("GC5025_OTP_CHIPVERSION reg_addr[%d] = 0x%x,reg_value[%d] = 0x%x\n",i,gc5025_otp_info.reg_addr[i],i,gc5025_otp_info.reg_value[i]);
			i++;
		}while((gc5025_otp_info.reg_addr[i-1]!=0)&&(i<10));
		gc5025_otp_info.reg_num = i - 1;
		break;
	case 0x02:
		CERR("GC5025_OTP_CHIPVERSION is Invalid !!\n");
		break;
	default :
		break;
	}

    /* check the sensor is gc5025 or gc5025h */
    if (gc5025_read_otp(s_ctrl, BASE_ADDR_DR) == 1 ) {
        CINFO("%s:%d check_gc_dr=1, the sensor is gc5025h in S88051AA1!", __func__, __LINE__);
    } else {
        CERR("%s:%d the sensor is gc5025 (not gc5025h) in S88051AA1, pls check with the GC/Vendor FAE!!!!", __func__, __LINE__);
    }
	/*print otp information*/
    /*
	printk("GC5025_OTP_INFO:module_id=0x%x\n",gc5025_otp_info.module_id);
	printk("GC5025_OTP_INFO:lens_id=0x%x\n",gc5025_otp_info.lens_id);
	printk("GC5025_OTP_INFO:vcm_id=0x%x\n",gc5025_otp_info.vcm_id);
	printk("GC5025_OTP_INFO:vcm_driver_id=0x%x\n",gc5025_otp_info.vcm_driver_id);
	printk("GC5025_OTP_INFO:data=%d-%d-%d\n",gc5025_otp_info.year,gc5025_otp_info.month,gc5025_otp_info.day);
	printk("GC5025_OTP_WB:r/g=0x%x\n",gc5025_otp_info.rg_gain);
	printk("GC5025_OTP_WB:b/g=0x%x\n",gc5025_otp_info.bg_gain);
	printk("GC5025_OTP_GOLDEN:golden_rg=0x%x\n",gc5025_otp_info.golden_rg);
	printk("GC5025_OTP_GOLDEN:golden_bg=0x%x\n",gc5025_otp_info.golden_bg);
	*/
}



static void gc5025_gcore_update_dd(struct msm_sensor_ctrl_t *s_ctrl)
{
	uint16_t i=0,j=0,n=0,m=0,s=0,e=0;
	uint16_t temp_x=0,temp_y=0;
	uint8_t flag=0;
	uint8_t temp_type=0;
	uint8_t temp_val0,temp_val1,temp_val2;
	/*TODO*/

	if(0x01 ==gc5025_otp_info.dd_flag)
	{
#if defined(IMAGE_NORMAL_MIRROR)
	//do nothing
#elif defined(IMAGE_H_MIRROR)
	for(i=0; i<gc5025_otp_info.dd_cnt; i++)
	{
		if(gc5025_otp_info.dd_param_type[i]==0)
		{	gc5025_otp_info.dd_param_x[i]= WINDOW_WIDTH - gc5025_otp_info.dd_param_x[i]+1;	}
		else if(gc5025_otp_info.dd_param_type[i]==1)
		{	gc5025_otp_info.dd_param_x[i]= WINDOW_WIDTH - gc5025_otp_info.dd_param_x[i]-1;	}
		else
		{	gc5025_otp_info.dd_param_x[i]= WINDOW_WIDTH - gc5025_otp_info.dd_param_x[i] ;	}
	}
#elif defined(IMAGE_V_MIRROR)
		for(i=0; i<gc5025_otp_info.dd_cnt; i++)
		{	gc5025_otp_info.dd_param_y[i]= WINDOW_HEIGHT - gc5025_otp_info.dd_param_y[i] + 1;	}

#elif defined(IMAGE_HV_MIRROR)
	for(i=0; i<gc5025_otp_info.dd_cnt; i++)
		{
			if(gc5025_otp_info.dd_param_type[i]==0)
			{
				gc5025_otp_info.dd_param_x[i]= WINDOW_WIDTH - gc5025_otp_info.dd_param_x[i]+1;
				gc5025_otp_info.dd_param_y[i]= WINDOW_HEIGHT - gc5025_otp_info.dd_param_y[i]+1;
			}
			else if(gc5025_otp_info.dd_param_type[i]==1)
			{
				gc5025_otp_info.dd_param_x[i]= WINDOW_WIDTH - gc5025_otp_info.dd_param_x[i]-1;
				gc5025_otp_info.dd_param_y[i]= WINDOW_HEIGHT - gc5025_otp_info.dd_param_y[i]+1;
			}
			else
			{
				gc5025_otp_info.dd_param_x[i]= WINDOW_WIDTH - gc5025_otp_info.dd_param_x[i] ;
				gc5025_otp_info.dd_param_y[i]= WINDOW_HEIGHT - gc5025_otp_info.dd_param_y[i] + 1;
			}
		}
#endif

		//y
		for(i=0 ; i< gc5025_otp_info.dd_cnt-1; i++)
		{
			for(j = i+1; j < gc5025_otp_info.dd_cnt; j++)
			{
				if(gc5025_otp_info.dd_param_y[i] > gc5025_otp_info.dd_param_y[j])
				{
					temp_x = gc5025_otp_info.dd_param_x[i] ; gc5025_otp_info.dd_param_x[i] = gc5025_otp_info.dd_param_x[j] ;  gc5025_otp_info.dd_param_x[j] = temp_x;
					temp_y = gc5025_otp_info.dd_param_y[i] ; gc5025_otp_info.dd_param_y[i] = gc5025_otp_info.dd_param_y[j] ;  gc5025_otp_info.dd_param_y[j] = temp_y;
					temp_type = gc5025_otp_info.dd_param_type[i] ; gc5025_otp_info.dd_param_type[i] = gc5025_otp_info.dd_param_type[j]; gc5025_otp_info.dd_param_type[j]= temp_type;
				}
			}

		}

		//x
		for(i=0; i<gc5025_otp_info.dd_cnt; i++)
		{
			if(gc5025_otp_info.dd_param_y[i]==gc5025_otp_info.dd_param_y[i+1])
			{
				s=i++;
				while((gc5025_otp_info.dd_param_y[s] == gc5025_otp_info.dd_param_y[i+1])&&(i<gc5025_otp_info.dd_cnt-1))
					i++;
				e=i;

				for(n=s; n<e; n++)
				{
					for(m=n+1; m<e+1; m++)
					{
						if(gc5025_otp_info.dd_param_x[n] > gc5025_otp_info.dd_param_x[m])
						{
							temp_x = gc5025_otp_info.dd_param_x[n] ; gc5025_otp_info.dd_param_x[n] = gc5025_otp_info.dd_param_x[m] ;  gc5025_otp_info.dd_param_x[m] = temp_x;
							temp_y = gc5025_otp_info.dd_param_y[n] ; gc5025_otp_info.dd_param_y[n] = gc5025_otp_info.dd_param_y[m] ;  gc5025_otp_info.dd_param_y[m] = temp_y;
							temp_type = gc5025_otp_info.dd_param_type[n] ; gc5025_otp_info.dd_param_type[n] = gc5025_otp_info.dd_param_type[m]; gc5025_otp_info.dd_param_type[m]= temp_type;
						}
					}
				}

			}

		}


		//write SRAM
		gc5025_Sensor_WriteReg(s_ctrl,0xfe, 0x01);
		gc5025_Sensor_WriteReg(s_ctrl,0xa8, 0x00);
		gc5025_Sensor_WriteReg(s_ctrl,0x9d, 0x04);
		gc5025_Sensor_WriteReg(s_ctrl,0xbe, 0x00);
		gc5025_Sensor_WriteReg(s_ctrl,0xa9, 0x01);

		for(i=0; i<gc5025_otp_info.dd_cnt; i++)
		{
			temp_val0 = gc5025_otp_info.dd_param_x[i]& 0x00ff;
			temp_val1 = (((gc5025_otp_info.dd_param_y[i])<<4)& 0x00f0) + ((gc5025_otp_info.dd_param_x[i]>>8)&0X000f);
			temp_val2 = (gc5025_otp_info.dd_param_y[i]>>4) & 0xff;
			gc5025_Sensor_WriteReg(s_ctrl,0xaa,i);
			gc5025_Sensor_WriteReg(s_ctrl,0xac,temp_val0);
			gc5025_Sensor_WriteReg(s_ctrl,0xac,temp_val1);
			gc5025_Sensor_WriteReg(s_ctrl,0xac,temp_val2);
			while((i < gc5025_otp_info.dd_cnt - 1) && (gc5025_otp_info.dd_param_x[i]==gc5025_otp_info.dd_param_x[i+1]) && (gc5025_otp_info.dd_param_y[i]==gc5025_otp_info.dd_param_y[i+1]))
				{
					flag = 1;
					i++;
				}
			if(flag)
				gc5025_Sensor_WriteReg(s_ctrl,0xac,0x02);
			else
				gc5025_Sensor_WriteReg(s_ctrl,0xac,gc5025_otp_info.dd_param_type[i]);

			CDBG("GC5025_OTP_GC val0 = 0x%x , val1 = 0x%x , val2 = 0x%x \n",temp_val0,temp_val1,temp_val2);
			CDBG("GC5025_OTP_GC x = %d , y = %d \n",((temp_val1&0x0f)<<8) + temp_val0,(temp_val2<<4) + ((temp_val1&0xf0)>>4));
		}

		gc5025_Sensor_WriteReg(s_ctrl,0xbe,0x01);
		gc5025_Sensor_WriteReg(s_ctrl,0xfe,0x00);
	}

}

#if 0
static void gc5025_gcore_update_wb(struct msm_sensor_ctrl_t *s_ctrl)
{
	uint16_t r_gain_current = 0 , g_gain_current = 0 , b_gain_current = 0 , base_gain = 0;
	uint16_t r_gain = 1024 , g_gain = 1024 , b_gain = 1024 ;
	uint16_t rg_typical,bg_typical;

	if(0x02==gc5025_otp_info.golden_flag)
	{
		return;
	}
	if(0x00==(gc5025_otp_info.golden_flag&0x01))
	{
		rg_typical=RG_TYPICAL;
		bg_typical=BG_TYPICAL;
	}
	if(0x01==(gc5025_otp_info.golden_flag&0x01))
	{
		rg_typical=gc5025_otp_info.golden_rg;
		bg_typical=gc5025_otp_info.golden_bg;
		CDBG("GC5025_OTP_UPDATE_AWB:rg_typical = 0x%x , bg_typical = 0x%x\n",rg_typical,bg_typical);
	}

	if(0x01==(gc5025_otp_info.wb_flag&0x01))
	{
		r_gain_current = 1024 * rg_typical/gc5025_otp_info.rg_gain;
		b_gain_current = 1024 * bg_typical/gc5025_otp_info.bg_gain;
		g_gain_current = 1024;

		base_gain = (r_gain_current<b_gain_current) ? r_gain_current : b_gain_current;
		base_gain = (base_gain<g_gain_current) ? base_gain : g_gain_current;
		CDBG("GC5025_OTP_UPDATE_AWB:r_gain_current = 0x%x , b_gain_current = 0x%x , base_gain = 0x%x \n",r_gain_current,b_gain_current,base_gain);

		r_gain = 0x400 * r_gain_current / base_gain;
		g_gain = 0x400 * g_gain_current / base_gain;
		b_gain = 0x400 * b_gain_current / base_gain;
		CDBG("GC5025_OTP_UPDATE_AWB:r_gain = 0x%x , g_gain = 0x%x , b_gain = 0x%x \n",r_gain,g_gain,b_gain);
		CDBG("GC5025_OTP_DD : 0x80= 0x%x\n",gc5025_Sensor_ReadReg(s_ctrl,0x80));

		/*TODO*/
		gc5025_Sensor_WriteReg(s_ctrl,0xfe,0x00);
		gc5025_Sensor_WriteReg(s_ctrl,0xc6,g_gain>>3);
		gc5025_Sensor_WriteReg(s_ctrl,0xc7,r_gain>>3);
		gc5025_Sensor_WriteReg(s_ctrl,0xc8,b_gain>>3);
		gc5025_Sensor_WriteReg(s_ctrl,0xc9,g_gain>>3);
		gc5025_Sensor_WriteReg(s_ctrl,0xc4,((g_gain&0x07) << 4) + (r_gain&0x07));
		gc5025_Sensor_WriteReg(s_ctrl,0xc5,((b_gain&0x07) << 4) + (g_gain&0x07));

	}

}
#endif

void gc5025_gcore_update_otp(struct msm_sensor_ctrl_t *s_ctrl)
{

	CINFO("GC5025_OTP: Update!!\n");
	gc5025_gcore_update_dd(s_ctrl);
	/*gc5025_gcore_update_wb(s_ctrl);	*/
}


static void gc5025_gcore_enable_otp(struct msm_sensor_ctrl_t *s_ctrl,otp_state state)
{
	uint8_t otp_clk,otp_en;
	otp_clk = gc5025_Sensor_ReadReg(s_ctrl, 0xfa);
	otp_en= gc5025_Sensor_ReadReg(s_ctrl, 0xd4);
	if(state)
	{
		otp_clk = otp_clk | 0x10;
		otp_en = otp_en | 0x80;
		mdelay(5);
		gc5025_Sensor_WriteReg(s_ctrl, 0xfa,otp_clk);	// 0xfa[6]:OTP_CLK_en
		gc5025_Sensor_WriteReg(s_ctrl, 0xd4,otp_en);	// 0xd4[7]:OTP_en

		CINFO("GC5025_OTP: Enable OTP!\n");
	}
	else
	{
		otp_en = otp_en & 0x7f;
		otp_clk = otp_clk & 0xef;
		mdelay(5);
		gc5025_Sensor_WriteReg(s_ctrl, 0xd4,otp_en);
		gc5025_Sensor_WriteReg(s_ctrl, 0xfa,otp_clk);

		CINFO("GC5025_OTP: Disable OTP!\n");
	}

}

void gc5025_gcore_identify_otp(struct msm_sensor_ctrl_t *s_ctrl)
{
	gc5025_Sensor_WriteReg(s_ctrl,0xfe,0x00);
	gc5025_Sensor_WriteReg(s_ctrl,0xfe,0x00);
	gc5025_Sensor_WriteReg(s_ctrl,0xfe,0x00);
	gc5025_Sensor_WriteReg(s_ctrl,0xf7,0x01);
	gc5025_Sensor_WriteReg(s_ctrl,0xf8,0x11);
	gc5025_Sensor_WriteReg(s_ctrl,0xf9,0x00);
	gc5025_Sensor_WriteReg(s_ctrl,0xfa,0xa0);
	gc5025_Sensor_WriteReg(s_ctrl,0xfc,0x2e);

	gc5025_gcore_enable_otp(s_ctrl,otp_open);
	gc5025_gcore_read_otp_info(s_ctrl);
	gc5025_gcore_enable_otp(s_ctrl,otp_close);

	gc5025_gcore_update_otp(s_ctrl);

}
// otp end