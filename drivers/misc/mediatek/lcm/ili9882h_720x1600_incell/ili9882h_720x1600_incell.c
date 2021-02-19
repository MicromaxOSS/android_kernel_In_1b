/* Copyright Statement:
*
* This software/firmware and related documentation ("MediaTek Software") are
* protected under relevant copyright laws. The information contained herein
* is confidential and proprietary to MediaTek Inc. and/or its licensors.
* Without the prior written permission of MediaTek inc. and/or its licensors,
* any reproduction, modification, use or disclosure of MediaTek Software,
* and information contained herein, in whole or in part, shall be strictly prohibited.
*/
/* MediaTek Inc. (C) 2015. All rights reserved.
*
* BY OPENING THIS FILE, RECEIVER HEREBY UNEQUIVOCALLY ACKNOWLEDGES AND AGREES
* THAT THE SOFTWARE/FIRMWARE AND ITS DOCUMENTATIONS ("MEDIATEK SOFTWARE")
* RECEIVED FROM MEDIATEK AND/OR ITS REPRESENTATIVES ARE PROVIDED TO RECEIVER ON
* AN "AS-IS" BASIS ONLY. MEDIATEK EXPRESSLY DISCLAIMS ANY AND ALL WARRANTIES,
* EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED WARRANTIES OF
* MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE OR NONINFRINGEMENT.
* NEITHER DOES MEDIATEK PROVIDE ANY WARRANTY WHATSOEVER WITH RESPECT TO THE
* SOFTWARE OF ANY THIRD PARTY WHICH MAY BE USED BY, INCORPORATED IN, OR
* SUPPLIED WITH THE MEDIATEK SOFTWARE, AND RECEIVER AGREES TO LOOK ONLY TO SUCH
* THIRD PARTY FOR ANY WARRANTY CLAIM RELATING THERETO. RECEIVER EXPRESSLY ACKNOWLEDGES
* THAT IT IS RECEIVER'S SOLE RESPONSIBILITY TO OBTAIN FROM ANY THIRD PARTY ALL PROPER LICENSES
* CONTAINED IN MEDIATEK SOFTWARE. MEDIATEK SHALL ALSO NOT BE RESPONSIBLE FOR ANY MEDIATEK
* SOFTWARE RELEASES MADE TO RECEIVER'S SPECIFICATION OR TO CONFORM TO A PARTICULAR
* STANDARD OR OPEN FORUM. RECEIVER'S SOLE AND EXCLUSIVE REMEDY AND MEDIATEK'S ENTIRE AND
* CUMULATIVE LIABILITY WITH RESPECT TO THE MEDIATEK SOFTWARE RELEASED HEREUNDER WILL BE,
* AT MEDIATEK'S OPTION, TO REVISE OR REPLACE THE MEDIATEK SOFTWARE AT ISSUE,
* OR REFUND ANY SOFTWARE LICENSE FEES OR SERVICE CHARGE PAID BY RECEIVER TO
* MEDIATEK FOR SUCH MEDIATEK SOFTWARE AT ISSUE.
*/

#define LOG_TAG "LCM"

#ifndef BUILD_LK
#include <linux/string.h>
#include <linux/kernel.h>
#endif

#include "lcm_drv.h"

#ifdef BUILD_LK
#include <platform/upmu_common.h>
#include <platform/mt_gpio.h>
#include <platform/mt_i2c.h>
#include <platform/mt_pmic.h>
#include <string.h>
#ifndef MACH_FPGA
#include <lcm_pmic.h>
#endif
#endif

#ifdef BUILD_LK
#define LCM_LOGI(string, args...)  dprintf(ALWAYS, "[LK/"LOG_TAG"]"string, ##args)
#define LCM_LOGD(string, args...)  dprintf(INFO, "[LK/"LOG_TAG"]"string, ##args)
#else
#define LCM_LOGI(fmt, args...)  pr_notice("[KERNEL/"LOG_TAG"]"fmt, ##args)
#define LCM_LOGD(fmt, args...)  pr_debug("[KERNEL/"LOG_TAG"]"fmt, ##args)
#endif


static struct LCM_UTIL_FUNCS lcm_util;
#define SET_RESET_PIN(v)    (lcm_util.set_reset_pin((v)))
#ifdef BUILD_LK

#else
#define SET_LCM_ENP(v)	(lcm_util.set_gpio_lcd_enp_bias((v)))
#define SET_LCM_ENN(v)	(lcm_util.set_gpio_lcd_enn_bias((v)))
#endif
#define MDELAY(n)       (lcm_util.mdelay(n))
#define UDELAY(n)       (lcm_util.udelay(n))
/* --------------------------------------------------------------------------- */
/* Local Functions */
/* --------------------------------------------------------------------------- */

#define dsi_set_cmdq_V2(cmd, count, ppara, force_update) \
    lcm_util.dsi_set_cmdq_V2(cmd, count, ppara, force_update)
#define dsi_set_cmdq(pdata, queue_size, force_update) \
        lcm_util.dsi_set_cmdq(pdata, queue_size, force_update)
#define wrtie_cmd(cmd) lcm_util.dsi_write_cmd(cmd)
#define write_regs(addr, pdata, byte_nums) \
        lcm_util.dsi_write_regs(addr, pdata, byte_nums)
#define read_reg(cmd) \
      lcm_util.dsi_dcs_read_lcm_reg(cmd)
#define read_reg_v2(cmd, buffer, buffer_size) \
        lcm_util.dsi_dcs_read_lcm_reg_v2(cmd, buffer, buffer_size)

#ifndef BUILD_LK
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/fs.h>
#include <linux/slab.h>
#include <linux/init.h>
#include <linux/list.h>
#include <linux/i2c.h>
#include <linux/irq.h>
#include <linux/uaccess.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/platform_device.h>
#endif

typedef enum{
	ILI9882_VENDOR_HUAXIAN = 0
	,ILI9882_VENDOR_GUOXIAN
	,ILI9882_VENDOR_UNKNOWN
}ILI9882_VENDOR_ENUM;
/* --------------------------------------------------------------------------- */
/* Local Constants */
/* --------------------------------------------------------------------------- */
#define FRAME_WIDTH                                     (720)
#define FRAME_HEIGHT                                    (1600)
#define PHYSICAL_WIDTH                                  (65)
#define PHYSICAL_HEIGHT                                 (145)
#define REGFLAG_DELAY       0xFFFC
#define REGFLAG_UDELAY  0xFFFB
#define REGFLAG_END_OF_TABLE    0xFFFD

static int ili9882_vendor = ILI9882_VENDOR_UNKNOWN;

extern int tpd_load_status;

extern char ilitek_get_vendor_id(void);

struct LCM_setting_table {
	unsigned int cmd;
	unsigned char count;
	unsigned char para_list[64];
};

static struct LCM_setting_table lcm_suspend_setting[] = {
	{0xFF, 0x03, {0x98, 0x82, 0x00} },
	{0x28, 0, {} },
	{REGFLAG_DELAY, 20, {} },
	{0x10, 0, {} },
	{REGFLAG_DELAY, 120, {} }
};

#define ILI9882_GESTURE_WAKEUP_SUPPORT 0

#ifdef BUILD_LK
static unsigned int GPIO_LCD_ENN = 66;
static unsigned int GPIO_LCD_ENP = 65;
#define sgm3804_SLAVE_ADDR_WRITE  0x7C
static struct mt_i2c_t sgm3804_i2c;

static int sgm3804_write_byte(kal_uint8 addr, kal_uint8 value)
{
	kal_uint32 ret_code = I2C_OK;
	kal_uint8 write_data[2];
	kal_uint16 len;

	write_data[0] = addr;
	write_data[1] = value;

	sgm3804_i2c.id = 3; /* I2C1; */
	/* Since i2c will left shift 1 bit, we need to set FAN5405 I2C address to >>1 */
	sgm3804_i2c.addr = (sgm3804_SLAVE_ADDR_WRITE >> 1);
	sgm3804_i2c.mode = ST_MODE;
	sgm3804_i2c.speed = 100;
	len = 2;

	ret_code = i2c_write(&sgm3804_i2c, write_data, len);
	printf("%s: sgm3804 i2c_write addr:%x ret_code: %d\n", __func__, addr, ret_code);

	return ret_code;
}
#endif

#ifdef BUILD_LK
static void tp_reset_ctl(int onoff)
{
	static unsigned int GPIO_TP_RESET = 174;
	mt_set_gpio_mode(GPIO_TP_RESET, GPIO_MODE_00);
	mt_set_gpio_dir(GPIO_TP_RESET, GPIO_DIR_OUT);
	
	if(onoff)
		mt_set_gpio_out(GPIO_TP_RESET, GPIO_OUT_ONE);
	else
		mt_set_gpio_out(GPIO_TP_RESET, GPIO_OUT_ZERO);
}
#else
extern void ili_reset_pin_ctl(int onoff);

static void tp_reset_ctl(int onoff)
{
	ili_reset_pin_ctl(onoff);
}
#endif

extern int ili_get_gesture_status(void);
static int ili_curr_avddavee_status = 1; 

static void lcm_power_onoff(int onoff)
{
	if(onoff)
	{
		#if ILI9882_GESTURE_WAKEUP_SUPPORT
		if(ili_curr_avddavee_status == 0)
		#endif
		{
			//printk("wxs lcm_power_onoff open avdd\n");
			SET_LCM_ENP(1);
			MDELAY(3);
			SET_LCM_ENN(1);
			MDELAY(20);
			ili_curr_avddavee_status = 1;
		}

		SET_RESET_PIN(0);
		MDELAY(10);
		SET_RESET_PIN(1);
		MDELAY(5);
		tp_reset_ctl(0);
		MDELAY(10);
		tp_reset_ctl(1);
		MDELAY(30);
	}
	else
	{
		#if ILI9882_GESTURE_WAKEUP_SUPPORT
		if(ili_get_gesture_status() == 0)
		#endif
		{
			printk("wxs lcm_power_onoff shutdown avdd\n");
			SET_LCM_ENP(0);
			MDELAY(3);
			SET_LCM_ENN(0);
			MDELAY(10);
			ili_curr_avddavee_status = 0;
		}
	}
}

void ili9882_shutdown_power(void)
{
	unsigned int version_id = 0;

	version_id = ilitek_get_vendor_id( );

	if ((version_id == 0x11)||(version_id == 0x21))
	{
		printk("wxs ili9882_shutdown_power\n");
		tp_reset_ctl(0);
		MDELAY(1);
		SET_RESET_PIN(0);
		MDELAY(15);
		SET_LCM_ENN(0);
		MDELAY(3);
		SET_LCM_ENP(0);
		MDELAY(3);
	}
}

static struct LCM_setting_table init_setting_huaxian[] = {
	{0xFF, 3, {0x98, 0x82, 0x01}},
	{0x00, 1, {0x42}},  //STV A rise
	{0x01, 1, {0x33}},  //STV A overlap
	{0x02, 1, {0x01}},
	{0x03, 1, {0x00}},
	{0x04, 1, {0x02}},  //STV B rise
	{0x05, 1, {0x31}},  //STV B overlap
	{0x07, 1, {0x00}},
	{0x08, 1, {0x80}},  //CLK A rise
	{0x09, 1, {0x81}},  //CLK A fall
	{0x0a, 1, {0x71}},  //CLK A overlap
	{0x0b, 1, {0x00}},
	{0x0c, 1, {0x01}},
	{0x0d, 1, {0x01}},
	{0x0e, 1, {0x00}},
	{0x0f, 1, {0x00}},
	{0x28, 1, {0x88}},
	{0x29, 1, {0x88}},
	{0x2a, 1, {0x00}},
	{0x2b, 1, {0x00}},
	{0x31, 1, {0x07}},
	{0x32, 1, {0x0c}},
	{0x33, 1, {0x22}},
	{0x34, 1, {0x23}}, //GOUT_4R_FW
	{0x35, 1, {0x07}},
	{0x36, 1, {0x08}},
	{0x37, 1, {0x16}},
	{0x38, 1, {0x14}},
	{0x39, 1, {0x12}},
	{0x3a, 1, {0x10}},
	{0x3b, 1, {0x21}},
	{0x3c, 1, {0x07}}, //GOUT_12R_FW
	{0x3d, 1, {0x07}},
	{0x3e, 1, {0x07}},
	{0x3f, 1, {0x07}},
	{0x40, 1, {0x07}},
	{0x41, 1, {0x07}},
	{0x42, 1, {0x07}},
	{0x43, 1, {0x07}},
	{0x44, 1, {0x07}},
	{0x45, 1, {0x07}},
	{0x46, 1, {0x07}},
	{0x47, 1, {0x07}},
	{0x48, 1, {0x0d}},
	{0x49, 1, {0x22}},
	{0x4a, 1, {0x23}}, //GOUT_4L_FW
	{0x4b, 1, {0x07}},
	{0x4c, 1, {0x09}},
	{0x4d, 1, {0x17}},
	{0x4e, 1, {0x15}},
	{0x4f, 1, {0x13}},
	{0x50, 1, {0x11}},
	{0x51, 1, {0x21}},
	{0x52, 1, {0x07}}, //GOUT_12L_FW
	{0x53, 1, {0x07}},
	{0x54, 1, {0x07}},
	{0x55, 1, {0x07}},
	{0x56, 1, {0x07}},
	{0x57, 1, {0x07}},
	{0x58, 1, {0x07}},
	{0x59, 1, {0x07}},
	{0x5a, 1, {0x07}},
	{0x5b, 1, {0x07}},
	{0x5c, 1, {0x07}}, 
	{0x61, 1, {0x07}},
	{0x62, 1, {0x0d}},
	{0x63, 1, {0x22}},
	{0x64, 1, {0x23}}, //GOUT_4R_BW
	{0x65, 1, {0x07}},
	{0x66, 1, {0x09}},
	{0x67, 1, {0x11}},
	{0x68, 1, {0x13}},
	{0x69, 1, {0x15}},
	{0x6a, 1, {0x17}},
	{0x6b, 1, {0x21}},
	{0x6c, 1, {0x07}}, //GOUT_12R_BW
	{0x6d, 1, {0x07}},
	{0x6e, 1, {0x07}},
	{0x6f, 1, {0x07}},
	{0x70, 1, {0x07}},
	{0x71, 1, {0x07}},
	{0x72, 1, {0x07}},
	{0x73, 1, {0x07}},
	{0x74, 1, {0x07}},
	{0x75, 1, {0x07}},
	{0x76, 1, {0x07}}, 
	{0x77, 1, {0x07}},
	{0x78, 1, {0x0c}},
	{0x79, 1, {0x22}},
	{0x7a, 1, {0x23}}, //GOUT_4L_BW
	{0x7b, 1, {0x07}},
	{0x7c, 1, {0x08}},
	{0x7d, 1, {0x10}},
	{0x7e, 1, {0x12}},
	{0x7f, 1, {0x14}},
	{0x80, 1, {0x16}},
	{0x81, 1, {0x21}},
	{0x82, 1, {0x07}}, //GOUT_12L_BW
	{0x83, 1, {0x07}},
	{0x84, 1, {0x07}},
	{0x85, 1, {0x07}},
	{0x86, 1, {0x07}},
	{0x87, 1, {0x07}},
	{0x88, 1, {0x07}},
	{0x89, 1, {0x07}},
	{0x8a, 1, {0x07}},
	{0x8b, 1, {0x07}},
	{0x8c, 1, {0x07}},
	{0xb0, 1, {0x33}},
	{0xba, 1, {0x06}},
	{0xc0, 1, {0x07}},
	{0xc1, 1, {0x00}},
	{0xc2, 1, {0x00}},
	{0xc3, 1, {0x00}},
	{0xc4, 1, {0x00}},
	{0xca, 1, {0x44}},
	{0xd0, 1, {0x01}},
	{0xd1, 1, {0x22}},
	{0xd3, 1, {0x40}},
	{0xd5, 1, {0x51}},
	{0xd6, 1, {0x20}},
	{0xd7, 1, {0x01}},
	{0xd8, 1, {0x00}},
	{0xdc, 1, {0xc2}},
	{0xdd, 1, {0x10}},
	{0xdf, 1, {0xb6}},
	{0xe0, 1, {0x3E}},
	{0xe2, 1, {0x47}}, //SRC power off GND
	{0xe7, 1, {0x54}},                                       
	{0xe6, 1, {0x22}},
	{0xee, 1, {0x15}},
	{0xf1, 1, {0x00}},
	{0xf2, 1, {0x00}},
	{0xfa, 1, {0xdf}},
	{0xFF, 3, {0x98, 0x82, 0x02}},
	{0xF1, 1, {0x1C}},     // Tcon ESD option
	{0x40, 1, {0x4B}},     //Data_in=7us
	{0x4B, 1, {0x5A}},     // line_chopper
	{0x50, 1, {0xCA}},     // line_chopper
	{0x51, 1, {0x50}},      // line_chopper
	{0x06, 1, {0x8D}},     // Internal Line Time (RTN)
	{0x0B, 1, {0xA0}},     // Internal VFP[9]
	{0x0C, 1, {0x80}},     // Internal VFP[8]
	{0x0D, 1, {0x1A}},     // Internal VBP
	{0x0E, 1, {0x04}},     // Internal VFP
	{0x4D, 1, {0xCE}},     // Power Saving Off
	{0x73, 1, {0x04}},     // notch tuning    
	{0x70, 1, {0x32}},     // notch tuning    
	{0xFF, 3, {0x98, 0x82, 0x05}},
	{0xA8, 1, {0x62}},
	{0x63, 1, {0x73}},     // GVDDN = -4.8V
	{0x64, 1, {0x73}},     // GVDDP = 4.8V
	{0x68, 1, {0x65}},     // VGHO = 12V
	{0x69, 1, {0x6B}},     // VGH = 13V
	{0x6A, 1, {0xA1}},     // VGLO = -12V
	{0x6B, 1, {0x93}},     // VGL = -13V
	{0x00, 1, {0x01}},     // Panda Enable
	{0x46, 1, {0x00}},     // LVD HVREG option 
	{0x45, 1, {0x01}},     // VGHO/DCHG1/DCHG2 DELAY OPTION Default
	{0x17, 1, {0x50}},     // LVD rise debounce
	{0x18, 1, {0x01}},     // Keep LVD state
	{0x85, 1, {0x37}},     // HW RESET option
	{0x86, 1, {0x0F}},     // FOR PANDA GIP DCHG1/2ON EN
	{0xFF, 3, {0x98, 0x82, 0x06}},
	{0xD9, 1, {0x1F}},     // 4Lane
	{0xC0, 1, {0x40}},     // NL = 1600
	{0xC1, 1, {0x16}},     // NL = 1600
	{0x07, 1, {0xF7}},     // ABNORMAL RESET OPTION
	//{0xD6, 1, {0x55}},   //TFE=THSD
	//{0xDD, 1, {0x61}},   //TFE=THSD
	//{0xDC, 1, {0x68}},   //TFE=THSD
	{0xFF, 3, {0x98, 0x82, 0x07}},
	{0x00, 1, {0x0C}}, 
	{0xFF, 3, {0x98, 0x82, 0x08}},
	{0xE0,27, {0x40, 0x24, 0x98, 0xD1, 0x10, 0x55, 0x40, 0x64, 0x8D, 0xAD, 0xA9, 0xDF, 0x05, 0x25, 0x45, 0xAA, 0x65, 0x8D, 0xA7, 0xCA, 0xFE, 0xE9, 0x12, 0x44, 0x6B, 0x03, 0xEC}},
	{0xE1,27, {0x40, 0x24, 0x98, 0xD1, 0x10, 0x55, 0x40, 0x64, 0x8D, 0xAD, 0xA9, 0xDF, 0x05, 0x25, 0x45, 0xAA, 0x65, 0x8D, 0xA7, 0xCA, 0xFE, 0xE9, 0x12, 0x44, 0x6B, 0x03, 0xEC}},
	{0xFF, 3, {0x98, 0x82, 0x0B}},
	{0x9A, 1, {0x44}},
	{0x9B, 1, {0x88}},
	{0x9C, 1, {0x03}},
	{0x9D, 1, {0x03}},
	{0x9E, 1, {0x71}},
	{0x9F, 1, {0x71}},
	{0xAB, 1, {0xE0}},     // AutoTrimType
	{0xFF, 3, {0x98, 0x82, 0x0E}},
	{0x02, 1, {0x09}},
	{0x11, 1, {0x50}},      // TSVD Rise Poisition
	{0x13, 1, {0x10}},     // TSHD Rise Poisition
	{0x00, 1, {0xA0}},     // LV mode
	{0xFF, 3, {0x98, 0x82, 0x00}},
	{0x35, 1, {0x00}},
	{0x11, 1, {0x00}},
	{REGFLAG_DELAY, 120, {}},
	{0x29, 1, {0x00}},
	{REGFLAG_DELAY, 20, {}}
};

static struct LCM_setting_table init_setting_guoxian[] = {
	{0xFF, 3, {0x98, 0x82, 0x01}}, 
	{0x00, 1, {0x42}},   //STV A rise
	{0x01, 1, {0x33}},   //STV A overlap
	{0x02, 1, {0x01}}, 
	{0x03, 1, {0x00}}, 
	{0x04, 1, {0x02}},   //STV B rise
	{0x05, 1, {0x31}},   //STV B overlap
	{0x07, 1, {0x00}}, 
	{0x08, 1, {0x80}},   //CLK A rise
	{0x09, 1, {0x81}},   //CLK A fall
	{0x0a, 1, {0x71}},   //CLK A overlap
	{0x0b, 1, {0x00}}, 
	{0x0c, 1, {0x01}}, 
	{0x0d, 1, {0x01}}, 
	{0x0e, 1, {0x00}}, 
	{0x0f, 1, {0x00}}, 
	{0x28, 1, {0x88}}, 
	{0x29, 1, {0x88}}, 
	{0x2a, 1, {0x00}}, 
	{0x2b, 1, {0x00}}, 
	{0x31, 1, {0x07}}, 
	{0x32, 1, {0x0c}}, 
	{0x33, 1, {0x22}}, 
	{0x34, 1, {0x23}},  //GOUT_4R_FW
	{0x35, 1, {0x07}}, 
	{0x36, 1, {0x08}}, 
	{0x37, 1, {0x16}}, 
	{0x38, 1, {0x14}}, 
	{0x39, 1, {0x12}}, 
	{0x3a, 1, {0x10}}, 
	{0x3b, 1, {0x21}}, 
	{0x3c, 1, {0x07}},  //GOUT_12R_FW
	{0x3d, 1, {0x07}}, 
	{0x3e, 1, {0x07}}, 
	{0x3f, 1, {0x07}}, 
	{0x40, 1, {0x07}}, 
	{0x41, 1, {0x07}}, 
	{0x42, 1, {0x07}}, 
	{0x43, 1, {0x07}}, 
	{0x44, 1, {0x07}}, 
	{0x45, 1, {0x07}}, 
	{0x46, 1, {0x07}}, 
	{0x47, 1, {0x07}}, 
	{0x48, 1, {0x0d}}, 
	{0x49, 1, {0x22}}, 
	{0x4a, 1, {0x23}},  //GOUT_4L_FW
	{0x4b, 1, {0x07}}, 
	{0x4c, 1, {0x09}}, 
	{0x4d, 1, {0x17}}, 
	{0x4e, 1, {0x15}}, 
	{0x4f, 1, {0x13}}, 
	{0x50, 1, {0x11}}, 
	{0x51, 1, {0x21}}, 
	{0x52, 1, {0x07}},  //GOUT_12L_FW
	{0x53, 1, {0x07}}, 
	{0x54, 1, {0x07}}, 
	{0x55, 1, {0x07}}, 
	{0x56, 1, {0x07}}, 
	{0x57, 1, {0x07}}, 
	{0x58, 1, {0x07}}, 
	{0x59, 1, {0x07}}, 
	{0x5a, 1, {0x07}}, 
	{0x5b, 1, {0x07}}, 
	{0x5c, 1, {0x07}},  
	{0x61, 1, {0x07}}, 
	{0x62, 1, {0x0d}}, 
	{0x63, 1, {0x22}}, 
	{0x64, 1, {0x23}},  //GOUT_4R_BW
	{0x65, 1, {0x07}}, 
	{0x66, 1, {0x09}}, 
	{0x67, 1, {0x11}}, 
	{0x68, 1, {0x13}}, 
	{0x69, 1, {0x15}}, 
	{0x6a, 1, {0x17}}, 
	{0x6b, 1, {0x21}}, 
	{0x6c, 1, {0x07}},  //GOUT_12R_BW
	{0x6d, 1, {0x07}}, 
	{0x6e, 1, {0x07}}, 
	{0x6f, 1, {0x07}}, 
	{0x70, 1, {0x07}}, 
	{0x71, 1, {0x07}}, 
	{0x72, 1, {0x07}}, 
	{0x73, 1, {0x07}}, 
	{0x74, 1, {0x07}}, 
	{0x75, 1, {0x07}}, 
	{0x76, 1, {0x07}},  
	{0x77, 1, {0x07}}, 
	{0x78, 1, {0x0c}}, 
	{0x79, 1, {0x22}}, 
	{0x7a, 1, {0x23}},  //GOUT_4L_BW
	{0x7b, 1, {0x07}}, 
	{0x7c, 1, {0x08}}, 
	{0x7d, 1, {0x10}}, 
	{0x7e, 1, {0x12}}, 
	{0x7f, 1, {0x14}}, 
	{0x80, 1, {0x16}}, 
	{0x81, 1, {0x21}}, 
	{0x82, 1, {0x07}},  //GOUT_12L_BW
	{0x83, 1, {0x07}}, 
	{0x84, 1, {0x07}}, 
	{0x85, 1, {0x07}}, 
	{0x86, 1, {0x07}}, 
	{0x87, 1, {0x07}}, 
	{0x88, 1, {0x07}}, 
	{0x89, 1, {0x07}}, 
	{0x8a, 1, {0x07}}, 
	{0x8b, 1, {0x07}}, 
	{0x8c, 1, {0x07}}, 
	{0xb0, 1, {0x33}}, 
	{0xba, 1, {0x06}}, 
	{0xc0, 1, {0x07}}, 
	{0xc1, 1, {0x00}}, 
	{0xc2, 1, {0x00}}, 
	{0xc3, 1, {0x00}}, 
	{0xc4, 1, {0x00}}, 
	{0xca, 1, {0x44}}, 
	{0xd0, 1, {0x01}}, 
	{0xd1, 1, {0x22}}, 
	{0xd3, 1, {0x40}}, 
	{0xd5, 1, {0x51}}, 
	{0xd6, 1, {0x20}}, 
	{0xd7, 1, {0x01}}, 
	{0xd8, 1, {0x00}}, 
	{0xdc, 1, {0xc2}}, 
	{0xdd, 1, {0x10}}, 
	{0xdf, 1, {0xb6}}, 
	{0xe0, 1, {0x3E}}, 
	{0xe2, 1, {0x47}},  //SRC power off GND
	{0xe7, 1, {0x54}},                                        
	{0xe6, 1, {0x22}}, 
	{0xee, 1, {0x15}}, 
	{0xf1, 1, {0x00}}, 
	{0xf2, 1, {0x00}}, 
	{0xfa, 1, {0xdf}}, 
	{0xFF, 3, {0x98, 0x82, 0x02}}, 
	{0xF1, 1, {0x1C}},      // Tcon ESD option
	{0x40, 1, {0x4B}},      // Data_in=7us
	{0x4B, 1, {0x5A}},      // line_chopper
	{0x50, 1, {0xCA}},      // line_chopper
	{0x51, 1, {0x50}},      // line_chopper
	{0x06, 1, {0x8D}},      // Internal Line Time (RTN)
	{0x0B, 1, {0xA0}},      // Internal VFP[9]
	{0x0C, 1, {0x80}},      // Internal VFP[8]
	{0x0D, 1, {0x1A}},      // Internal VBP
	{0x0E, 1, {0x04}},      // Internal VFP
	{0x4D, 1, {0xCE}},      // Power Saving Off
	{0x73, 1, {0x04}},      // notch tuning    
	{0x70, 1, {0x32}},      // notch tuning     
	{0xFF, 3, {0x98, 0x82, 0x05}}, 
	{0xA8, 1, {0x62}}, 
	{0x03, 1, {0x00}},      // VCOM
	{0x04, 1, {0xAF}},      // VCOM
	{0x63, 1, {0x73}},      // GVDDN = -4.8V
	{0x64, 1, {0x73}},      // GVDDP = 4.8V
	{0x68, 1, {0x65}},      // VGHO = 12V
	{0x69, 1, {0x6B}},      // VGH = 13V
	{0x6A, 1, {0xA1}},      // VGLO = -12V
	{0x6B, 1, {0x93}},      // VGL = -13V
	{0x00, 1, {0x01}},      // Panda Enable
	{0x46, 1, {0x00}},      // LVD HVREG option 
	{0x45, 1, {0x01}},      // VGHO/DCHG1/DCHG2 DELAY OPTION Default
	{0x17, 1, {0x50}},      // LVD rise debounce
	{0x18, 1, {0x01}},      // Keep LVD state
	{0x85, 1, {0x37}},      // HW RESET option
	{0x86, 1, {0x0F}},      // FOR PANDA GIP DCHG1/2ON EN
	{0xFF, 3, {0x98, 0x82, 0x06}}, 
	{0xD9, 1, {0x1F}},      // 4Lane
	{0xC0, 1, {0x40}},      // NL = 1600
	{0xC1, 1, {0x16}},      // NL = 1600
	{0x07, 1, {0xF7}},      // ABNORMAL RESET OPTION
	{0xFF, 3, {0x98, 0x82, 0x08}}, 
	{0xE0,27, {0x40, 0x24, 0x97, 0xD1, 0x10, 0x55, 0x40, 0x64, 0x8D, 0xAD, 0xA9, 0xDE, 0x03, 0x24, 0x43, 0xAA, 0x63, 0x8C, 0xA8, 0xCC, 0xFE, 0xEC, 0x18, 0x50, 0x7D, 0x03, 0xEC}}, 
	{0xE1,27, {0x40, 0x24, 0x97, 0xD1, 0x10, 0x55, 0x40, 0x64, 0x8D, 0xAD, 0xA9, 0xDE, 0x03, 0x24, 0x43, 0xAA, 0x63, 0x8C, 0xA8, 0xCC, 0xFE, 0xEC, 0x18, 0x50, 0x7D, 0x03, 0xEC}}, 
	{0xFF, 3, {0x98, 0x82, 0x0B}}, 
	{0x9A, 1, {0x44}}, 
	{0x9B, 1, {0x88}}, 
	{0x9C, 1, {0x03}}, 
	{0x9D, 1, {0x03}}, 
	{0x9E, 1, {0x71}}, 
	{0x9F, 1, {0x71}}, 
	{0xAB, 1, {0xE0}},      // AutoTrimType
	{0xFF, 3, {0x98, 0x82, 0x0E}}, 
	{0x02, 1, {0x09}}, 
	{0x11, 1, {0x50}},       // TSVD Rise Poisition
	{0x13, 1, {0x10}},      // TSHD Rise Poisition
	{0x00, 1, {0xA0}},      // LV mode
	{0xFF, 3, {0x98, 0x82, 0x00}},
	{0x35, 1, {0x00}},
	{0x11, 1, {0x00}},
	{REGFLAG_DELAY, 120, {}},
	{0x29, 1, {0x00}},
	{REGFLAG_DELAY, 20, {}}
};

static void push_table(struct LCM_setting_table *table, unsigned int count, unsigned char force_update)
{
	unsigned int i;

	for (i = 0; i < count; i++) {
		unsigned cmd;
		cmd = table[i].cmd;

		switch (cmd) {

			case REGFLAG_DELAY:
				if (table[i].count <= 10)
					MDELAY(table[i].count);
				else
					MDELAY(table[i].count);
				break;

			case REGFLAG_UDELAY:
				UDELAY(table[i].count);
				break;

			case REGFLAG_END_OF_TABLE:
				break;

			default:
				dsi_set_cmdq_V2(cmd, table[i].count, table[i].para_list, force_update);
		}
	}
}

/* --------------------------------------------------------------------------- */
/* LCM Driver Implementations */
/* --------------------------------------------------------------------------- */

static void lcm_set_util_funcs(const struct LCM_UTIL_FUNCS *util)
{
	memcpy(&lcm_util, util, sizeof(struct LCM_UTIL_FUNCS));
}


static void lcm_get_params(struct LCM_PARAMS *params)
{
	memset(params, 0, sizeof(struct LCM_PARAMS));

	params->type = LCM_TYPE_DSI;

	params->width = FRAME_WIDTH;
	params->height = FRAME_HEIGHT;

	params->physical_width = PHYSICAL_WIDTH;
	params->physical_height = PHYSICAL_HEIGHT;

	params->dsi.switch_mode_enable = 0;
	params->dsi.mode = SYNC_PULSE_VDO_MODE;
	params->dsi.switch_mode = CMD_MODE;

	/* DSI */
	/* Command mode setting */
	params->dsi.LANE_NUM = LCM_FOUR_LANE;
	/* The following defined the fomat for data coming from LCD engine. */
	params->dsi.data_format.color_order = LCM_COLOR_ORDER_RGB;
	params->dsi.data_format.trans_seq = LCM_DSI_TRANS_SEQ_MSB_FIRST;
	params->dsi.data_format.padding = LCM_DSI_PADDING_ON_LSB;
	params->dsi.data_format.format = LCM_DSI_FORMAT_RGB888;

	/* Highly depends on LCD driver capability. */
	params->dsi.packet_size = 256;
	/* video mode timing */

	params->dsi.PS = LCM_PACKED_PS_24BIT_RGB888;

	params->dsi.vertical_sync_active = 2;
	params->dsi.vertical_backporch = 16;
	params->dsi.vertical_frontporch = 240;
	params->dsi.vertical_active_line = FRAME_HEIGHT;

	params->dsi.horizontal_sync_active = 20;
	params->dsi.horizontal_backporch = 20;
	params->dsi.horizontal_frontporch = 20;
	params->dsi.horizontal_active_pixel = FRAME_WIDTH;
	params->dsi.PLL_CLOCK = 275;	/* this value must be in MTK suggested table */
	params->dsi.clk_lp_per_line_enable = 0;
	params->dsi.esd_check_enable = 1;
	params->dsi.customization_esd_check_enable = 1;
	params->dsi.lcm_esd_check_table[0].cmd = 0x0A;
	params->dsi.lcm_esd_check_table[0].count = 1;
	params->dsi.lcm_esd_check_table[0].para_list[0] = 0x9C;
}

static void lcm_init(void)
{
	unsigned int version_id = 0;

	lcm_power_onoff(1);
	
	version_id = ilitek_get_vendor_id( );

	if (version_id == 0x11)
		ili9882_vendor = ILI9882_VENDOR_HUAXIAN;
	else if (version_id == 0x21)
		ili9882_vendor = ILI9882_VENDOR_GUOXIAN;
		
	//printk("%s,ili9882 0xda version_id=0x%x,ili9882_vendor:%d\n", __func__, version_id,ili9882_vendor);

	if(ili9882_vendor == ILI9882_VENDOR_HUAXIAN)
		push_table(init_setting_huaxian, sizeof(init_setting_huaxian) / sizeof(struct LCM_setting_table), 1);
	else 	if(ili9882_vendor == ILI9882_VENDOR_GUOXIAN)
		push_table(init_setting_guoxian, sizeof(init_setting_guoxian) / sizeof(struct LCM_setting_table), 1);
}

int ili9882_get_vendor(void)
{
	return ili9882_vendor;
}

#define TP_DEEP_SLEEP_BEFORE_LCD 1
extern int ili_sleep_handler(int mode);

static void lcm_suspend(void)
{
	if(tpd_load_status)
	{
		ili_sleep_handler(TP_DEEP_SLEEP_BEFORE_LCD);
		push_table(lcm_suspend_setting, sizeof(lcm_suspend_setting) / sizeof(struct LCM_setting_table), 1);
		MDELAY(100);
		lcm_power_onoff(0);
	}
	else
	{
		lcm_power_onoff(0);
	}
}

static void lcm_resume(void)
{
	lcm_init();
	MDELAY(30);
}

static unsigned int lcm_compare_id(void)
{
	return 1;
}

static unsigned int lcm_ata_check(unsigned char *buffer)
{
	unsigned int version_id = 0;
	unsigned char ata_check_buffer[2];
	unsigned int array[16];
	struct LCM_setting_table switch_table_page0[] = {
		{ 0xFF, 0x03, {0x98, 0x82, 0x00} }
	};

	push_table(switch_table_page0, sizeof(switch_table_page0) / sizeof(struct LCM_setting_table), 1);

	array[0] = 0x00013700;	/* read id return two byte,version and id */
	dsi_set_cmdq(array, 1, 1);

	read_reg_v2(0xda, ata_check_buffer, 1);
	version_id = ata_check_buffer[0]; 	/* we only need ID */

	LCM_LOGI("%s,ili9882 0xda version_id=0x%x\n", __func__, version_id);

	if ((version_id == 0x11)||(version_id == 0x21))
		return 1;
	else
		return 0;
}


struct LCM_DRIVER ili9882h_720x1600_incell_drv = {
	.name = "ili9882h_720x1600_incell",
	.set_util_funcs = lcm_set_util_funcs,
	.get_params = lcm_get_params,
	.init = lcm_init,
	.suspend = lcm_suspend,
	.resume = lcm_resume,
	.compare_id = lcm_compare_id,
	.ata_check = lcm_ata_check,
};
