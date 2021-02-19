/*
 * Copyright (C) 2015 MediaTek Inc.
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 */



#include <linux/videodev2.h>
#include <linux/i2c.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/cdev.h>
#include <linux/uaccess.h>
#include <linux/fs.h>
#include <asm/atomic.h>
#include <linux/slab.h>

//#include "kd_camera_hw.h"
#include "kd_imgsensor.h"
#include "kd_imgsensor_define.h"
#include "kd_imgsensor_errcode.h"
#include "s5k4h7mipiraw_Sensor.h"

#define USHORT             unsigned short
#define BYTE               unsigned char
#define S5K4H7SUB_OTP_LSCFLAG_ADDR	0x0A3D
#define S5K4H7SUB_OTP_FLAGFLAG_ADDR	0x0A04
#define S5K4H7SUB_OTP_AWBFLAG_ADDR	0x0A24
#define S5K4H7SUB_LSC_PAGE		0
#define S5K4H7SUB_FLAG_PAGE		21
#define S5K4H7SUB_AWB_PAGE		21

typedef struct {
	unsigned short	infoflag;
	unsigned short	lsc_infoflag;
	unsigned short	flag_infoflag;
	unsigned short	flag_module_integrator_id;
	int		awb_offset;
	int		flag_offset;
	int		lsc_offset;
	int		lsc_group;
	int		flag_group;
	int		group;
	unsigned short	frgcur;
	unsigned short	fbgcur;
	unsigned int	nr_gain;
	unsigned int	ng_gain;
	unsigned int	nb_gain;
	unsigned int	ngrcur;
	unsigned int	ngbcur;
	unsigned int	ngcur;
	unsigned int	nrcur;
	unsigned int	nbcur;
	unsigned int	nggolden;
	unsigned int	nrgolden;
	unsigned int	nbgolden;
	unsigned int	ngrgolden;
	unsigned int	ngbgolden;
	unsigned int	frggolden;
	unsigned int	fbggolden;
	unsigned int	awb_flag_sum;
	unsigned int	lsc_sum;
	unsigned int	lsc_check_flag;
} OTP;

OTP otp_4h7_data_info = {0};
unsigned int otp_group = 0;

/**********************************************************
 * get_4h7_page_data
 * get page data
 * return true or false
 * ***********************************************************/
void get_4h7_page_data(int pageidx, unsigned char *pdata)
{
	unsigned short get_byte = 0;
	unsigned int addr = 0x0A04;
	int i = 0;

	otp_4h7_write_cmos_sensor_8(0x0A02, pageidx);
	otp_4h7_write_cmos_sensor_8(0x0A00, 0x01);

	do {
		mdelay(1);
		get_byte = otp_4h7_read_cmos_sensor(0x0A01);
	} while ((get_byte & 0x01) != 1);

	for (i = 0; i < 64; i++) {
		pdata[i] = otp_4h7_read_cmos_sensor(addr);
		addr++;
	}

	otp_4h7_write_cmos_sensor_8(0x0A00, 0x00);
}

unsigned short selective_4h7_read_region(int pageidx, unsigned int addr)
{
	unsigned short get_byte = 0;

	otp_4h7_write_cmos_sensor_8(0x0A02, pageidx);
	otp_4h7_write_cmos_sensor_8(0x0A00, 0x01);
	do {
		mdelay(1);
		get_byte = otp_4h7_read_cmos_sensor(0x0A01);
	} while ((get_byte & 0x01) != 1);

	get_byte = otp_4h7_read_cmos_sensor(addr);
	otp_4h7_write_cmos_sensor_8(0x0A00, 0x00);

	return get_byte;
}

unsigned int selective_4h7_read_region_16(int pageidx, unsigned int addr)
{
	unsigned int get_byte = 0;
	static int old_pageidx;

	if (pageidx != old_pageidx) {
		otp_4h7_write_cmos_sensor_8(0x0A00, 0x00);
		otp_4h7_write_cmos_sensor_8(0x0A02, pageidx);
		otp_4h7_write_cmos_sensor_8(0x0A00, 0x01);
		do {
			mdelay(1);
			get_byte = otp_4h7_read_cmos_sensor(0x0A01);
		} while ((get_byte & 0x01) != 1);
	}

	get_byte = ((otp_4h7_read_cmos_sensor(addr) << 8) | otp_4h7_read_cmos_sensor(addr+1));
	old_pageidx = pageidx;
	return get_byte;
}
/**********************************************************
 * apply_4h7_otp_awb
 * apply otp
 * *******************************************************/
void apply_4h7_otp_awb(void)
{
	int R_gain,B_gain,Gb_gain,Gr_gain,Base_gain;
	int R_ratio, B_ratio, GR_ratio, GB_ratio;

	R_ratio = otp_4h7_data_info.nrcur*1000/otp_4h7_data_info.nrgolden;
	B_ratio = otp_4h7_data_info.nbcur*1000/otp_4h7_data_info.nbgolden;
	GR_ratio = otp_4h7_data_info.ngrcur*1000/otp_4h7_data_info.ngrgolden;
	GB_ratio = otp_4h7_data_info.ngbcur*1000/otp_4h7_data_info.ngbgolden;

	Base_gain = R_ratio;
	if(Base_gain>B_ratio) Base_gain=B_ratio;
	if(Base_gain>GR_ratio) Base_gain=GR_ratio;
	if(Base_gain>GB_ratio) Base_gain=GB_ratio;
	
	R_gain = 0x100 * R_ratio / Base_gain;
	B_gain = 0x100 * B_ratio / Base_gain;
	Gb_gain = 0x100 * GR_ratio / Base_gain;
	Gr_gain = 0x100 * GB_ratio / Base_gain;
	
	printk("apply_4h7_otp_awb R_gain:%d,B_gain:%d,Gb_gain:%d,Gr_gain:%d,Base_gain:%d,R_ratio:%d, B_ratio:%d, GR_ratio:%d, GB_ratio:%d\n",
		R_gain,B_gain,Gb_gain,Gr_gain,Base_gain,R_ratio, B_ratio, GR_ratio, GB_ratio);

	otp_4h7_write_cmos_sensor_8(0x3C0F, 0x00);

	if(Gr_gain>0x100) {
		otp_4h7_write_cmos_sensor_8(0x020E,Gr_gain>>8);
		otp_4h7_write_cmos_sensor_8(0x020F,Gr_gain&0xff);
	}
	
	if(R_gain>0x100) {
		otp_4h7_write_cmos_sensor_8(0x0210,R_gain>>8);
		otp_4h7_write_cmos_sensor_8(0x0211,R_gain&0xff);
	}
	
	if(B_gain>0x100) {
		otp_4h7_write_cmos_sensor_8(0x0212,B_gain>>8);
		otp_4h7_write_cmos_sensor_8(0x0213,B_gain&0xff);
	}
	
	if(Gb_gain>0x100) {
		otp_4h7_write_cmos_sensor_8(0x0214,Gb_gain>>8);
		otp_4h7_write_cmos_sensor_8(0x0215,Gb_gain&0xff);
	}
	printk("OTP apply_4h7_otp_awb\n");
	
	printk("4h7 OTP read otp_group_info_4h7 read back r_gain_h:%x,r_gain_l:%x,gr_gain_h:%x,gr_gain_l:%x\n",
		otp_4h7_read_cmos_sensor(0x0210),otp_4h7_read_cmos_sensor(0x0211),otp_4h7_read_cmos_sensor(0x020E),otp_4h7_read_cmos_sensor(0x020F));
	printk("4h7 OTP read otp_group_info_4h7 read back gb_gain_h:%x,gb_gain_l:%x,b_gain_h:%x,b_gain_l:%x,0x3C0F:%x\n",
		otp_4h7_read_cmos_sensor(0x0214),otp_4h7_read_cmos_sensor(0x0215),otp_4h7_read_cmos_sensor(0x0212),otp_4h7_read_cmos_sensor(0x0213),otp_4h7_read_cmos_sensor(0x3C0F));
}

/*********************************************************
 *apply_4h7_otp_lsc
 * ******************************************************/

void apply_4h7_otp_enb_lsc(void)
{
	printk("OTP enable lsc\n");
	otp_4h7_write_cmos_sensor_8(0x0B00, 0x01);
}

/*********************************************************
 * otp_group_info_4h7
 * ******************************************************/
int otp_group_info_4h7(void)
{
	memset(&otp_4h7_data_info, 0, sizeof(OTP));

	otp_4h7_data_info.lsc_infoflag =
		selective_4h7_read_region(S5K4H7SUB_LSC_PAGE, S5K4H7SUB_OTP_LSCFLAG_ADDR);

	if (otp_4h7_data_info.lsc_infoflag == 0x01) {
		otp_4h7_data_info.lsc_offset = 0;
		otp_4h7_data_info.lsc_group = 1;
		otp_4h7_data_info.lsc_sum = selective_4h7_read_region(S5K4H7SUB_AWB_PAGE, 0x0A1B);
	} else if (otp_4h7_data_info.lsc_infoflag == 0x03) {
		otp_4h7_data_info.lsc_offset = 1;
		otp_4h7_data_info.lsc_group = 2;
		otp_4h7_data_info.lsc_sum = selective_4h7_read_region(S5K4H7SUB_AWB_PAGE, 0x0A3B);
	} else {
		printk("4H7 OTP read data fail lsc empty!!!\n");
		goto error;
	}

	otp_4h7_data_info.flag_infoflag =
		selective_4h7_read_region(S5K4H7SUB_FLAG_PAGE, S5K4H7SUB_OTP_AWBFLAG_ADDR);

	if(otp_4h7_data_info.flag_infoflag == 0x01) {
		otp_4h7_data_info.flag_offset = 1;
		otp_4h7_data_info.flag_group = 2;
		otp_4h7_data_info.group = 2;
		otp_group = 2;
	} else {
		otp_4h7_data_info.flag_infoflag =
			selective_4h7_read_region(S5K4H7SUB_FLAG_PAGE, S5K4H7SUB_OTP_FLAGFLAG_ADDR);

		if(otp_4h7_data_info.flag_infoflag == 0x01) {
			otp_4h7_data_info.flag_offset = 1;
			otp_4h7_data_info.flag_group = 1;
			otp_4h7_data_info.group = 1;
			otp_group = 1;
		}
	}

	if((otp_4h7_data_info.group != 1)&&(otp_4h7_data_info.group != 2)){
		printk("4H7 OTP read data fail awb empty!!!\n");
		goto error;
	}

	printk("4H7 OTP read awb current group is %d\n", otp_4h7_data_info.group);

	if(otp_4h7_data_info.group == 2) {
		otp_4h7_data_info.flag_module_integrator_id =
			otp_4h7_read_cmos_sensor(S5K4H7SUB_OTP_AWBFLAG_ADDR + otp_4h7_data_info.flag_offset);
		
		otp_4h7_data_info.nrcur = selective_4h7_read_region(S5K4H7SUB_AWB_PAGE, 0x0A2E);
		otp_4h7_data_info.nbcur = selective_4h7_read_region(S5K4H7SUB_AWB_PAGE, 0x0A31);
		otp_4h7_data_info.ngrcur = selective_4h7_read_region(S5K4H7SUB_AWB_PAGE, 0x0A2F);
		otp_4h7_data_info.ngbcur = selective_4h7_read_region(S5K4H7SUB_AWB_PAGE, 0x0A30);

		otp_4h7_data_info.nrgolden = selective_4h7_read_region(S5K4H7SUB_AWB_PAGE, 0x0A34);
		otp_4h7_data_info.nbgolden = selective_4h7_read_region(S5K4H7SUB_AWB_PAGE, 0x0A37);
		otp_4h7_data_info.ngrgolden = selective_4h7_read_region(S5K4H7SUB_AWB_PAGE, 0x0A35);
		otp_4h7_data_info.ngbgolden = selective_4h7_read_region(S5K4H7SUB_AWB_PAGE, 0x0A36);
		otp_4h7_data_info.awb_flag_sum = selective_4h7_read_region(S5K4H7SUB_AWB_PAGE, 0x0A25); 	
	} else {
		otp_4h7_data_info.flag_module_integrator_id =
			otp_4h7_read_cmos_sensor(S5K4H7SUB_OTP_FLAGFLAG_ADDR + otp_4h7_data_info.flag_offset + 1);
		
		otp_4h7_data_info.nrcur = selective_4h7_read_region(S5K4H7SUB_AWB_PAGE, 0x0A0E);
		otp_4h7_data_info.nbcur = selective_4h7_read_region(S5K4H7SUB_AWB_PAGE, 0x0A11);
		otp_4h7_data_info.ngrcur = selective_4h7_read_region(S5K4H7SUB_AWB_PAGE, 0x0A0F);
		otp_4h7_data_info.ngbcur = selective_4h7_read_region(S5K4H7SUB_AWB_PAGE, 0x0A10);

		otp_4h7_data_info.nrgolden = selective_4h7_read_region(S5K4H7SUB_AWB_PAGE, 0x0A14);
		otp_4h7_data_info.nbgolden = selective_4h7_read_region(S5K4H7SUB_AWB_PAGE, 0x0A17);
		otp_4h7_data_info.ngrgolden = selective_4h7_read_region(S5K4H7SUB_AWB_PAGE, 0x0A15);
		otp_4h7_data_info.ngbgolden = selective_4h7_read_region(S5K4H7SUB_AWB_PAGE, 0x0A16);
		otp_4h7_data_info.awb_flag_sum = selective_4h7_read_region(S5K4H7SUB_AWB_PAGE, 0x0A05);
	}
	
	printk("4h7 OTP read otp_group_info_4h7 flag_module_integrator_id:0x%x,awb_flag_sum:0x%x\n",otp_4h7_data_info.flag_module_integrator_id,otp_4h7_data_info.awb_flag_sum);
	printk("4h7 OTP read otp_group_info_4h7 awb r:%x,b:%x,gr:%x,gb:%x\n",otp_4h7_data_info.nrcur,otp_4h7_data_info.nbcur,otp_4h7_data_info.ngrcur,otp_4h7_data_info.ngbcur);
	printk("4h7 OTP read otp_group_info_4h7 awb golden r:%x,b:%x,gr:%x,gb:%x\n",otp_4h7_data_info.nrgolden,otp_4h7_data_info.nbgolden,otp_4h7_data_info.ngrgolden,otp_4h7_data_info.ngbgolden);
	return  0;
error:
	return  -1;
}
/*********************************************************
 * read_4h7_page
 * read_Page1~Page21 of data
 * return true or false
 ********************************************************/
bool read_4h7_page(int page_start, int page_end, unsigned char *pdata)
{
	bool bresult = true;
	int st_page_start = page_start;

	if (page_start <= 0 || page_end > 21) {
		bresult = false;
		printk(" OTP page_end is large!");
		return bresult;
	}
	for (; st_page_start <= page_end; st_page_start++) {
		get_4h7_page_data(st_page_start, pdata);
	}
	return bresult;
}

unsigned int sum_4h7_awb_flag_lsc(unsigned int sum_start, unsigned int sum_end, unsigned char *pdata)
{
	int i = 0;
	unsigned int start;
	unsigned int re_sum = 0;

	for (start = 0x0A04; i < 64; i++, start++) {
		if ((start >= sum_start) && (start <= sum_end)) {
			re_sum += pdata[i];
		}
	}
	return  re_sum;
}

bool check_4h7_sum_flag_awb(void)
{
	int page_start = 21, page_end = 21;
	unsigned char data_p[21][64] = {};
	bool bresult = true;
	unsigned int  sum_awbfg = 0;

	bresult &= read_4h7_page(page_start, page_end, data_p[page_start-1]);

	if (otp_4h7_data_info.group == 1) {
		sum_awbfg = sum_4h7_awb_flag_lsc(0x0A06, 0X0A0C, data_p[page_start-1]);
		sum_awbfg += sum_4h7_awb_flag_lsc(0x0A0E, 0X0A11, data_p[page_start-1]);
	} else if (otp_4h7_data_info.group == 2) {
		sum_awbfg = sum_4h7_awb_flag_lsc(0x0A26, 0X0A2C, data_p[page_start-1]);
		sum_awbfg += sum_4h7_awb_flag_lsc(0x0A2E, 0X0A31, data_p[page_start-1]);
	}

	sum_awbfg = sum_awbfg%0xff + 1;
	printk("4H7 sum_awbfg2:0x%x   awb_flag_sum:0x%x!!!\n",sum_awbfg,otp_4h7_data_info.awb_flag_sum);
	apply_4h7_otp_awb();
	return  bresult;
}

bool  check_4h7_sum_flag_lsc(void)
{
	int page_start = 21, page_end = 21;

	unsigned char data_p[21][64] = {};
	bool bresult = true;
	unsigned int  sum_slc = 0;

	if (otp_4h7_data_info.lsc_group == 1) {
		for (page_start = 1, page_end = 6; page_start <= page_end; page_start++) {
			bresult &= read_4h7_page(page_start, page_start, data_p[page_start-1]);
			if (page_start == 6) {
				sum_slc += sum_4h7_awb_flag_lsc(0x0A04, 0x0A2B, data_p[page_start-1]);
				continue;
			}
			sum_slc += sum_4h7_awb_flag_lsc(0x0A04, 0X0A43, data_p[page_start-1]);
		}
	} else if (otp_4h7_data_info.lsc_group == 2) {
		for (page_start = 6, page_end = 12; page_start <= page_end; page_start++) {
			bresult &= read_4h7_page(page_start, page_start, data_p[page_start-1]);
			if (page_start == 6) {
				sum_slc += sum_4h7_awb_flag_lsc(0x0A2C, 0x0A43, data_p[page_start-1]);
				continue;
			} else if (page_start < 12) {
				sum_slc += sum_4h7_awb_flag_lsc(0x0A04, 0X0A43, data_p[page_start-1]);
			} else {
				sum_slc += sum_4h7_awb_flag_lsc(0x0A04, 0X0A13, data_p[page_start-1]);
			}
		}
	}

	sum_slc = sum_slc%0xff + 1;
	printk("4H7 sum_slc:0x%x   lsc_sum:0x%x!!!\n",sum_slc,otp_4h7_data_info.lsc_sum);
	apply_4h7_otp_enb_lsc();
	otp_4h7_data_info.lsc_check_flag = 1;
	return  bresult;
}

bool update_4h7_otp(void)
{
	int result = 1;
	if(otp_group_info_4h7() == -1){
		printk("OTP read data fail  empty!!!\n");
		result &= 0;
	}
	else {
		if(check_4h7_sum_flag_awb() == 0 && otp_4h7_data_info.lsc_check_flag){
			printk("OTP 4h7 check sum fail!!!\n");
			result &= 0;
		}
		else {
			printk("OTP 4h7 check ok\n");
		}
	}
	return  result;
}
