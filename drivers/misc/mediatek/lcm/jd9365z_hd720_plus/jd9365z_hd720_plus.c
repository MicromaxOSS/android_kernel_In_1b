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
#elif defined(BUILD_UBOOT)
#include <asm/arch/mt_gpio.h>
#else
#include "disp_dts_gpio.h"
#endif

#ifdef BUILD_LK
#define LCM_LOGI(string, args...)  dprintf(0, "[LK/"LOG_TAG"]"string, ##args)
#define LCM_LOGD(string, args...)  dprintf(1, "[LK/"LOG_TAG"]"string, ##args)
#else
#define LCM_LOGI(fmt, args...)  pr_debug("[KERNEL/"LOG_TAG"]"fmt, ##args)
#define LCM_LOGD(fmt, args...)  pr_debug("[KERNEL/"LOG_TAG"]"fmt, ##args)
#endif

#define LCM_ID (0x98)

static const unsigned int BL_MIN_LEVEL = 20;
static struct LCM_UTIL_FUNCS lcm_util;

#define SET_RESET_PIN(v)	(lcm_util.set_reset_pin((v)))
#define SET_LCM_ENP(v)	(lcm_util.set_gpio_lcd_enp_bias((v)))
#define SET_LCM_ENN(v)	(lcm_util.set_gpio_lcd_enn_bias((v)))
#define MDELAY(n)		(lcm_util.mdelay(n))
#define UDELAY(n)		(lcm_util.udelay(n))


#define dsi_set_cmdq_V22(cmdq, cmd, count, ppara, force_update) \
	lcm_util.dsi_set_cmdq_V22(cmdq, cmd, count, ppara, force_update)
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
/* #include <linux/jiffies.h> */
/* #include <linux/delay.h> */
#include <linux/uaccess.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/platform_device.h>
#endif

/* static unsigned char lcd_id_pins_value = 0xFF; */
static const unsigned char LCD_MODULE_ID = 0x01;
#define LCM_DSI_CMD_MODE	0
#define FRAME_WIDTH		(720)
#define FRAME_HEIGHT                                    (1520)

#define LCM_PHYSICAL_WIDTH		(68040)
#define LCM_PHYSICAL_HEIGHT		(143640)

#define REGFLAG_DELAY		0xFFFC
#define REGFLAG_UDELAY	0xFFFB
#define REGFLAG_END_OF_TABLE	0xFFFD
#define REGFLAG_RESET_LOW	0xFFFE
#define REGFLAG_RESET_HIGH	0xFFFF

#ifndef TRUE
#define TRUE 1
#endif

#ifndef FALSE
#define FALSE 0
#endif

struct LCM_setting_table {
	unsigned int cmd;
	unsigned char count;
	unsigned char para_list[64];
};

static struct LCM_setting_table lcm_suspend_setting[] = {
	{0x28, 0, {} },
	{REGFLAG_DELAY, 20, {} },
	{0x10, 0, {} },
	{REGFLAG_DELAY, 120, {} }
};


static struct LCM_setting_table init_setting_vdo[] = {
	{0xDF,3,{0x93,0x65,0xF8}},
	{0xB0,7,{0x10,0x15,0x06,0x00,0x64,0x06,0x01}},
	{0xB2,2,{0x00,0x3D}},
	{0xB3,2,{0x00,0x3D}},
	{0xB5,2,{0x64,0x84}},
	{0xB7,6,{0x00,0xBF,0x01,0x00,0xBF,0x01}},
	{0xB9,4,{0x00,0x84,0x13,0x02}},
	{0xBB,11,{0x0F,0x01,0x24,0x00,0x2F,0x13,0x28,0x04,0xDD,0xDD,0xDD}},
	{0xBC,2,{0x0F,0x04}},
	{0xBE,2,{0x1E,0xF2}},
	{0xC0,2,{0x26,0x03}},
	{0xC1,2,{0x00,0x12}},
	{0xC3,7,{0x04,0x02,0x02,0x66,0x01,0x80,0x80}},
	{0xC4,9,{0x24,0xF8,0xB4,0x81,0x12,0x0F,0x16,0x00,0x00}},
	{0xC8,38,{0x5F,0x44,0x36,0x2A,0x28,0x1A,0x20,0x0B,0x25,0x24,0x25,0x44,0x33,0x3E,0x30,0x2C,0x20,0x12,0x08,
			  0x5F,0x44,0x36,0x2A,0x28,0x1A,0x20,0x0B,0x25,0x24,0x25,0x44,0x33,0x3E,0x30,0x2C,0x20,0x12,0x08}},
	{0xD0,22,{0x1E,0x1F,0x57,0x58,0x44,0x46,0x48,0x4A,0x40,0x1F,0x1F,0x1F,0x1F,0x1F,0x1F,0x50,0x1F,0x1F,0x1F,0x1F,0x1F,0x1F}},
	{0xD1,22,{0x1E,0x1F,0x57,0x58,0x45,0x47,0x49,0x4B,0x41,0x1F,0x1F,0x1F,0x1F,0x1F,0x1F,0x51,0x1F,0x1F,0x1F,0x1F,0x1F,0x1F}},
	{0xD2,22,{0x1F,0x1E,0x17,0x18,0x0B,0x09,0x07,0x05,0x11,0x1F,0x1F,0x1F,0x1F,0x1F,0x1F,0x01,0x1F,0x1F,0x1F,0x1F,0x1F,0x1F}},
	{0xD3,22,{0x1F,0x1E,0x17,0x18,0x0A,0x08,0x06,0x04,0x10,0x1F,0x1F,0x1F,0x1F,0x1F,0x1F,0x00,0x1F,0x1F,0x1F,0x1F,0x1F,0x1F}},
	{0xD4,22,{0x00,0x00,0x00,0x04,0x0B,0x30,0x01,0x02,0x00,0x58,0x55,0xFF,0x30,0x03,0x04,0x1C,0x74,0x73,0x0D,0x00,0x58,0x00}},
	{0xD5,17,{0x00,0x08,0x00,0x08,0x30,0x00,0x00,0x00,0x00,0x00,0xBC,0x50,0x00,0x06,0x12,0x18,0x60}},
	{0xDD,3,{0x2C,0xA3,0x00}},
	{0xDE,1,{0x02}},

	{0xC6,1,{0x02}},
	{0xD7,1,{0x12}},
	{0xE9,1,{0x06}},

	{0xB2,2,{0x32,0x1C}},
	{0xB7,4,{0x1F,0x00,0x00,0x04}},
	{0xC1,1,{0x11}},
	{0xCF,1,{0x00}},
	{0xBB,6,{0x21,0x05,0x23,0x24,0x34,0x35}},
//	{0xC2,4,{0x20,0x38,0x1E,0x84}},
	{0xDE,1,{0x00}},
	{0x35,1,{0x00}},
//	{0x36,1,{0x08}},
//	{0xC2,1,{0x15}},
	{0x11,1,{0x00}},
	{REGFLAG_DELAY, 120, {}},
	{0x29,1, {0x00}},
	{REGFLAG_DELAY, 20, {}}
};

#if 0
static struct LCM_setting_table lcm_set_window[] = {
	{0x2A, 4, {0x00, 0x00, (FRAME_WIDTH >> 8), (FRAME_WIDTH & 0xFF)} },
	{0x2B, 4, {0x00, 0x00, (FRAME_HEIGHT >> 8), (FRAME_HEIGHT & 0xFF)} },
	{REGFLAG_END_OF_TABLE, 0x00, {} }
};
#endif
#if 0
static struct LCM_setting_table lcm_sleep_out_setting[] = {
	/* Sleep Out */
	{0x11, 1, {0x00} },
	{REGFLAG_DELAY, 120, {} },

	/* Display ON */
	{0x29, 1, {0x00} },
	{REGFLAG_DELAY, 20, {} },
	{REGFLAG_END_OF_TABLE, 0x00, {} }
};

static struct LCM_setting_table lcm_deep_sleep_mode_in_setting[] = {
	/* Display off sequence */
	{0x28, 1, {0x00} },
	{REGFLAG_DELAY, 20, {} },

	/* Sleep Mode On */
	{0x10, 1, {0x00} },
	{REGFLAG_DELAY, 120, {} },
	{REGFLAG_END_OF_TABLE, 0x00, {} }
};
#endif
static struct LCM_setting_table bl_level[] = {
	{0x51, 1, {0xFF} },
	{REGFLAG_END_OF_TABLE, 0x00, {} }
};

static void push_table(void *cmdq, struct LCM_setting_table *table,
	unsigned int count, unsigned char force_update)
{
	unsigned int i;
	unsigned int cmd;

	for (i = 0; i < count; i++) {
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
			dsi_set_cmdq_V22(cmdq, cmd,
				table[i].count,
				table[i].para_list,
				force_update);
		}
	}
}


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
	
	params->physical_width = LCM_PHYSICAL_WIDTH/1000;
	params->physical_height = LCM_PHYSICAL_HEIGHT/1000;
	params->physical_width_um = LCM_PHYSICAL_WIDTH;
	params->physical_height_um = LCM_PHYSICAL_HEIGHT;

	params->dsi.mode = SYNC_PULSE_VDO_MODE;
	params->dsi.switch_mode = CMD_MODE;

	params->dsi.switch_mode_enable = 0;

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

	params->dsi.vertical_sync_active = 4;
	params->dsi.vertical_backporch = 12;
	params->dsi.vertical_frontporch = 18;
	//params->dsi.vertical_frontporch_for_low_power = 540;
	params->dsi.vertical_active_line = FRAME_HEIGHT;

	params->dsi.horizontal_sync_active = 8;
	params->dsi.horizontal_backporch = 42;
	params->dsi.horizontal_frontporch = 42;
	params->dsi.horizontal_active_pixel = FRAME_WIDTH;
	params->dsi.ssc_disable = 1;

	/* this value must be in MTK suggested table */
	params->dsi.PLL_CLOCK = 255;//228

	//params->dsi.PLL_CK_CMD = 220;
	//params->dsi.PLL_CK_VDO = 255;
	params->dsi.cont_clock = 0;
	params->dsi.clk_lp_per_line_enable = 1;
	params->dsi.esd_check_enable = 1;
	params->dsi.customization_esd_check_enable = 1;
	params->dsi.lcm_esd_check_table[0].cmd = 0x0A;
	params->dsi.lcm_esd_check_table[0].count = 1;
	params->dsi.lcm_esd_check_table[0].para_list[0] = 0x1C;

	params->dsi.lcm_esd_check_table[1].cmd = 0x0e;
	params->dsi.lcm_esd_check_table[1].count = 1;
	params->dsi.lcm_esd_check_table[1].para_list[0] = 0x80;

}

static void lcm_init_power(void)
{
	/*LCM_LOGD("lcm_init_power\n");*/

#ifdef BUILD_LK
	mt_set_gpio_mode(GPIO2_LCM_1V8_EN, GPIO_MODE_00);
	mt_set_gpio_dir(GPIO2_LCM_1V8_EN, GPIO_DIR_OUT);
	mt_set_gpio_out(GPIO2_LCM_1V8_EN, GPIO_OUT_ONE);
	MDELAY(2);

	mt_set_gpio_mode(GPIO25_LCM_PO_EN, GPIO_MODE_00);
	mt_set_gpio_dir(GPIO25_LCM_PO_EN, GPIO_DIR_OUT);
	mt_set_gpio_out(GPIO25_LCM_PO_EN, GPIO_OUT_ONE);
	MDELAY(2);

	mt_set_gpio_mode(GPIO26_LCM_NO_EN, GPIO_MODE_00);
	mt_set_gpio_dir(GPIO26_LCM_NO_EN, GPIO_DIR_OUT);
	mt_set_gpio_out(GPIO26_LCM_NO_EN, GPIO_OUT_ONE);

	MDELAY(20);
#else
	display_bias_enable();

#endif
}

static void lcm_suspend_power(void)
{
	/*LCM_LOGD("lcm_suspend_power\n");*/

#ifdef BUILD_LK
	mt_set_gpio_out(GPIO2_LCM_1V8_EN, GPIO_OUT_ZERO);
	MDELAY(2);

	mt_set_gpio_out(GPIO26_LCM_NO_EN, GPIO_OUT_ZERO);
	MDELAY(2);

	mt_set_gpio_out(GPIO25_LCM_PO_EN, GPIO_OUT_ZERO);
#else
	display_bias_disable();

#endif
}

static void lcm_resume_power(void)
{
	/*LCM_LOGD("lcm_resume_power\n");*/

	/*lcm_init_power();*/
//	SET_RESET_PIN(0);
	display_bias_enable();
}

static void lcm_init(void)
{
	int size;

	SET_LCM_ENP(1);
	MDELAY(5);
	SET_LCM_ENN(1);
	MDELAY(10);
	/*LCM_LOGD("lcm_init\n");*/

#ifdef BUILD_LK
	mt_set_gpio_mode(GPIO45_LCM_RESET, GPIO_MODE_00);
	mt_set_gpio_dir(GPIO45_LCM_RESET, GPIO_DIR_OUT);

	mt_set_gpio_out(GPIO45_LCM_RESET, GPIO_OUT_ONE);
	MDELAY(1);
	mt_set_gpio_out(GPIO45_LCM_RESET, GPIO_OUT_ZERO);
	MDELAY(1);
	mt_set_gpio_out(GPIO45_LCM_RESET, GPIO_OUT_ONE);
	MDELAY(20);
#else
	SET_RESET_PIN(0);
	MDELAY(15);
	SET_RESET_PIN(1);
	MDELAY(10);
	SET_RESET_PIN(0);
	MDELAY(10);

	SET_RESET_PIN(1);
	MDELAY(60);

	size = sizeof(init_setting_vdo) / sizeof(struct LCM_setting_table);
	push_table(NULL, init_setting_vdo, size, 1);

#endif

}

static void lcm_suspend(void)
{
	/*LCM_LOGD("lcm_suspend\n");*/

	push_table(NULL, lcm_suspend_setting,
		sizeof(lcm_suspend_setting) / sizeof(struct LCM_setting_table),1);
	MDELAY(10);
}

static void lcm_resume(void)
{
	/*LCM_LOGD("lcm_resume\n");*/

	lcm_init();
}

void disable_power(void)
{
	SET_LCM_ENN(0);
	MDELAY(5);
	SET_LCM_ENP(0);
}

static void lcm_update(unsigned int x, unsigned int y,
	unsigned int width, unsigned int height)
{
	unsigned int x0 = x;
	unsigned int y0 = y;
	unsigned int x1 = x0 + width - 1;
	unsigned int y1 = y0 + height - 1;

	unsigned char x0_MSB = ((x0 >> 8) & 0xFF);
	unsigned char x0_LSB = (x0 & 0xFF);
	unsigned char x1_MSB = ((x1 >> 8) & 0xFF);
	unsigned char x1_LSB = (x1 & 0xFF);
	unsigned char y0_MSB = ((y0 >> 8) & 0xFF);
	unsigned char y0_LSB = (y0 & 0xFF);
	unsigned char y1_MSB = ((y1 >> 8) & 0xFF);
	unsigned char y1_LSB = (y1 & 0xFF);

	unsigned int data_array[16];

	data_array[0] = 0x00053902;
	data_array[1] = (x1_MSB << 24) | (x0_LSB << 16) | (x0_MSB << 8) | 0x2a;
	data_array[2] = (x1_LSB);
	dsi_set_cmdq(data_array, 3, 1);

	data_array[0] = 0x00053902;
	data_array[1] = (y1_MSB << 24) | (y0_LSB << 16) | (y0_MSB << 8) | 0x2b;
	data_array[2] = (y1_LSB);
	dsi_set_cmdq(data_array, 3, 1);

	data_array[0] = 0x002c3909;
	dsi_set_cmdq(data_array, 1, 0);
}

static unsigned int lcm_compare_id(void)
{
	return 1;
}


/* return TRUE: need recovery */
/* return FALSE: No need recovery */
static unsigned int lcm_esd_check(void)
{
#ifndef BUILD_LK
	char buffer[3];
	int array[4];

	array[0] = 0x00013700;
	dsi_set_cmdq(array, 1, 1);

	read_reg_v2(0x0A, buffer, 1);

	if (buffer[0] != 0x9D) {
		LCM_LOGI("[LCM ERROR] [0xA]=0x%02x\n", buffer[0]);
		return TRUE;
	}
	LCM_LOGI("[LCM NORMAL] [0xA]=0x%02x\n", buffer[0]);
	return FALSE;
#else
	return FALSE;
#endif

}

static unsigned int lcm_ata_check(unsigned char *buffer)
{
#ifndef BUILD_LK
	int array[4];
	char lcm_buffer[3];
	
	array[0] = 0x00023700;
	dsi_set_cmdq(array, 1, 1);
	read_reg_v2(0x04, lcm_buffer, 2);
	printk("jd9365z id = %x:%x:%x\n", lcm_buffer[0], lcm_buffer[1], lcm_buffer[2]);
	
	if(0x93 == lcm_buffer[0])
		return 1;
	else
		return 0;
#else
	return 1;
#endif
}

static void lcm_setbacklight_cmdq(void *handle, unsigned int level)
{

	LCM_LOGI("%s,ili9881c backlight: level = %d\n", __func__, level);

	bl_level[0].para_list[0] = level;

	push_table(handle,
		bl_level,
		sizeof(bl_level) / sizeof(struct LCM_setting_table),
		1);
}

struct LCM_DRIVER jd9365z_hd720_plus_drv = {
	.name = "jd9365z_hd720_plus",
	.set_util_funcs = lcm_set_util_funcs,
	.get_params = lcm_get_params,
	.init = lcm_init,
	.suspend = lcm_suspend,
	.resume = lcm_resume,
	.compare_id = lcm_compare_id,
	.init_power = lcm_init_power,
	.resume_power = lcm_resume_power,
	.suspend_power = lcm_suspend_power,
	.esd_check = lcm_esd_check,
	.set_backlight_cmdq = lcm_setbacklight_cmdq,
	.ata_check = lcm_ata_check,
	.update = lcm_update,

};
