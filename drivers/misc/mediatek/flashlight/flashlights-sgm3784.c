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

#define pr_fmt(fmt) KBUILD_MODNAME ": %s: " fmt, __func__

#include <linux/types.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/device.h>
#include <linux/platform_device.h>
#include <linux/hrtimer.h>
#include <linux/ktime.h>
#include <linux/workqueue.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/list.h>
#include <linux/delay.h>
#include <linux/i2c.h>
#include <linux/slab.h>
#include <linux/pinctrl/consumer.h>

#include "flashlight-core.h"
#include "flashlight-dt.h"

/* device tree should be defined in flashlight-dt.h */
#ifndef SGM3784_DTNAME
#define SGM3784_DTNAME "mediatek,flashlights_sgm3784"
#endif
#ifndef SGM3784_DTNAME_I2C
#define SGM3784_DTNAME_I2C "mediatek,strobe_main"
#endif

#define SGM3784_NAME "flashlights-sgm3784"

/* define registers */
#define SGM3784_REG_ENABLE	(0x0F)
#define SGM3784_MASK_ENABLE_LED1 (0x01)
#define SGM3784_MASK_ENABLE_LED2 (0x02)
#define SGM3784_DISABLE (0x00)
#define SGM3784_ENABLE_LED1 (0x01)
#define SGM3784_ENABLE_LED1_TORCH (0x09)
#define SGM3784_ENABLE_LED1_FLASH (0x0D)
#define SGM3784_ENABLE_LED2 (0x02)
#define SGM3784_ENABLE_LED2_TORCH (0x0A)
#define SGM3784_ENABLE_LED2_FLASH (0x0E)

#define SGM3784_REG_TORCH_LEVEL_LED1 (0x08)
#define SGM3784_REG_FLASH_LEVEL_LED1 (0x06)
#define SGM3784_REG_TORCH_LEVEL_LED2 (0x0B)
#define SGM3784_REG_FLASH_LEVEL_LED2 (0x09)

#define SGM3784_REG_TIMING_CONF (0x08)
#define SGM3784_TORCH_RAMP_TIME (0x00)
#define SGM3784_FLASH_TIMEOUT   (0x0F)

/* define channel, level */
#define SGM3784_CHANNEL_NUM 2
#define SGM3784_CHANNEL_CH1 0
#define SGM3784_CHANNEL_CH2 1

#define SGM3784_LEVEL_NUM 26
#define SGM3784_LEVEL_TORCH 15

#define SGM3784_HW_TIMEOUT 1600 /* ms */

/* define mutex and work queue */
static DEFINE_MUTEX(sgm3784_mutex);
static struct work_struct sgm3784_work_ch1;
static struct work_struct sgm3784_work_ch2;

/* define pinctrl */
#define SGM3784_PINCTRL_PIN_HWEN 0
#define SGM3784_PINCTRL_PIN_HWSTROBE 1
#define SGM3784_PINCTRL_PIN_HWGPIO 2
#define SGM3784_PINCTRL_PINSTATE_LOW 0
#define SGM3784_PINCTRL_PINSTATE_HIGH 1
#define SGM3784_PINCTRL_STATE_HWEN_HIGH "sgm3784_en_high"
#define SGM3784_PINCTRL_STATE_HWEN_LOW  "sgm3784_en_low"
#define SGM3784_PINCTRL_STATE_HWSTROBE_HIGH "sgm3784_strobe_high"
#define SGM3784_PINCTRL_STATE_HWSTROBE_LOW  "sgm3784_strobe_low"
#define SGM3784_PINCTRL_STATE_HWGPIO_HIGH "sgm3784_gpio_high"
#define SGM3784_PINCTRL_STATE_HWGPIO_LOW  "sgm3784_gpio_low"
static struct pinctrl *sgm3784_pinctrl;
static struct pinctrl_state *sgm3784_hwen_high;
static struct pinctrl_state *sgm3784_hwen_low;
static struct pinctrl_state *sgm3784_hwstrobe_high;
static struct pinctrl_state *sgm3784_hwstrobe_low;
static struct pinctrl_state *sgm3784_hwgpio_high;
static struct pinctrl_state *sgm3784_hwgpio_low;

int LED1Closeflag = 0;
int LED2Closeflag = 0;

/* define usage count */
static int use_count;

/* define i2c */
static struct i2c_client *sgm3784_i2c_client;

/* platform data */
struct sgm3784_platform_data {
	int channel_num;
	struct flashlight_device_id *dev_id;
};

/* sgm3784 chip data */
struct sgm3784_chip_data {
	struct i2c_client *client;
	struct sgm3784_platform_data *pdata;
	struct mutex lock;
};


/******************************************************************************
 * Pinctrl configuration
 *****************************************************************************/
static int sgm3784_pinctrl_init(struct platform_device *pdev)
{
	int ret = 0;

	/* get pinctrl */
	sgm3784_pinctrl = devm_pinctrl_get(&pdev->dev);
	if (IS_ERR(sgm3784_pinctrl)) {
		pr_err("Failed to get flashlight pinctrl.\n");
		ret = PTR_ERR(sgm3784_pinctrl);
	}

	/* Flashlight HWEN pin initialization */
	sgm3784_hwen_high = pinctrl_lookup_state(
			sgm3784_pinctrl, SGM3784_PINCTRL_STATE_HWEN_HIGH);
	if (IS_ERR(sgm3784_hwen_high)) {
		pr_err("Failed to init (%s)\n", SGM3784_PINCTRL_STATE_HWEN_HIGH);
		ret = PTR_ERR(sgm3784_hwen_high);
	}
	sgm3784_hwen_low = pinctrl_lookup_state(
			sgm3784_pinctrl, SGM3784_PINCTRL_STATE_HWEN_LOW);
	if (IS_ERR(sgm3784_hwen_low)) {
		pr_err("Failed to init (%s)\n", SGM3784_PINCTRL_STATE_HWEN_LOW);
		ret = PTR_ERR(sgm3784_hwen_low);
	}
	/* Flashlight HWSTROBE pin initialization */
	sgm3784_hwstrobe_high = pinctrl_lookup_state(
			sgm3784_pinctrl, SGM3784_PINCTRL_STATE_HWSTROBE_HIGH);
	if (IS_ERR(sgm3784_hwstrobe_high)) {
		pr_err("Failed to init (%s)\n", SGM3784_PINCTRL_STATE_HWSTROBE_HIGH);
		ret = PTR_ERR(sgm3784_hwstrobe_high);
	}
	sgm3784_hwstrobe_low = pinctrl_lookup_state(
			sgm3784_pinctrl, SGM3784_PINCTRL_STATE_HWSTROBE_LOW);
	if (IS_ERR(sgm3784_hwstrobe_low)) {
		pr_err("Failed to init (%s)\n", SGM3784_PINCTRL_STATE_HWSTROBE_LOW);
		ret = PTR_ERR(sgm3784_hwstrobe_low);
	}
	/* Flashlight HWGPIO pin initialization */
	sgm3784_hwgpio_high = pinctrl_lookup_state(
			sgm3784_pinctrl, SGM3784_PINCTRL_STATE_HWGPIO_HIGH);
	if (IS_ERR(sgm3784_hwgpio_high)) {
		pr_err("Failed to init (%s)\n", SGM3784_PINCTRL_STATE_HWGPIO_HIGH);
		ret = PTR_ERR(sgm3784_hwgpio_high);
	}
	sgm3784_hwgpio_low = pinctrl_lookup_state(
			sgm3784_pinctrl, SGM3784_PINCTRL_STATE_HWGPIO_LOW);
	if (IS_ERR(sgm3784_hwgpio_low)) {
		pr_err("Failed to init (%s)\n", SGM3784_PINCTRL_STATE_HWGPIO_LOW);
		ret = PTR_ERR(sgm3784_hwgpio_low);
	}

	return ret;
}

static int sgm3784_pinctrl_set(int pin, int state)
{
	int ret = 0;

	if (IS_ERR(sgm3784_pinctrl)) {
		pr_err("pinctrl is not available\n");
		return -1;
	}

	switch (pin) {
	case SGM3784_PINCTRL_PIN_HWEN:
		if (state == SGM3784_PINCTRL_PINSTATE_LOW &&
				!IS_ERR(sgm3784_hwen_low))
			pinctrl_select_state(sgm3784_pinctrl, sgm3784_hwen_low);
		else if (state == SGM3784_PINCTRL_PINSTATE_HIGH &&
				!IS_ERR(sgm3784_hwen_high))
			pinctrl_select_state(sgm3784_pinctrl, sgm3784_hwen_high);
		else
			pr_err("set err, pin(%d) state(%d)\n", pin, state);
		break;
	case SGM3784_PINCTRL_PIN_HWSTROBE:
		if (state == SGM3784_PINCTRL_PINSTATE_LOW &&
				!IS_ERR(sgm3784_hwstrobe_low))
			pinctrl_select_state(sgm3784_pinctrl, sgm3784_hwstrobe_low);
		else if (state == SGM3784_PINCTRL_PINSTATE_HIGH &&
				!IS_ERR(sgm3784_hwstrobe_high))
			pinctrl_select_state(sgm3784_pinctrl, sgm3784_hwstrobe_high);
		else
			pr_err("set err, pin(%d) state(%d)\n", pin, state);
		break;
	case SGM3784_PINCTRL_PIN_HWGPIO:
		if (state == SGM3784_PINCTRL_PINSTATE_LOW &&
				!IS_ERR(sgm3784_hwgpio_low))
			pinctrl_select_state(sgm3784_pinctrl, sgm3784_hwgpio_low);
		else if (state == SGM3784_PINCTRL_PINSTATE_HIGH &&
				!IS_ERR(sgm3784_hwgpio_high))
			pinctrl_select_state(sgm3784_pinctrl, sgm3784_hwgpio_high);
		else
			pr_err("set err, pin(%d) state(%d)\n", pin, state);
		break;
	default:
		pr_err("set err, pin(%d) state(%d)\n", pin, state);
		break;
	}
	printk("[jiangguijuan]pin(%d) state(%d)\n", pin, state);

	return ret;
}


/******************************************************************************
 * sgm3784 operations
 *****************************************************************************/
static const int sgm3784_current[SGM3784_LEVEL_NUM] = {
	 0,  37,  74,  111,  148, 185, 222, 259, 296, 333,
	370, 407, 444, 481,  518, 555, 592, 629, 666, 703,
	740, 777, 814, 851, 888, 925
};

static const unsigned char sgm3784_torch_level[SGM3784_LEVEL_NUM] = {
	0x00, 0x02, 0x04, 0x06, 0x08, 0x0a, 0x0c, 0x0e, 0x10, 0x12,
	0x14, 0x16, 0x18, 0x1a, 0x1c, 0x1e, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00
};

static const unsigned char sgm3784_flash_level[SGM3784_LEVEL_NUM] = {
	0x00, 0x02, 0x04, 0x06, 0x08, 0x0a, 0x0c, 0xe, 0x10, 0x12,
	0x14, 0x16, 0x18, 0x1a, 0x1c, 0x1e, 0x20, 0x22, 0x24, 0x26,
	0x28, 0x2a, 0x2c, 0x2e, 0x30, 0x32
};

//static unsigned char sgm3784_reg_enable;
static int sgm3784_level_ch1 = -1;
static int sgm3784_level_ch2 = -1;


static int sgm3784_is_torch(void)
{
	if ((sgm3784_level_ch1 >= SGM3784_LEVEL_TORCH)||(sgm3784_level_ch2>=SGM3784_LEVEL_TORCH))
		return -1;

	return 0;
}

static int sgm3784_verify_level(int level)
{
	if (level < -1)
		level = -1;
	else if (level >= (SGM3784_LEVEL_NUM-1))
		level = SGM3784_LEVEL_NUM - 2;

	return level;
}

/* i2c wrapper function */
static int sgm3784_write_reg(struct i2c_client *client, u8 reg, u8 val)
{
	int ret;
	struct sgm3784_chip_data *chip = i2c_get_clientdata(client);

	mutex_lock(&chip->lock);
	ret = i2c_smbus_write_byte_data(client, reg, val);
	mutex_unlock(&chip->lock);
	printk("jiang add sgm3784 write reg 0x%x,value 0x%x\n",reg,val);
	if (ret < 0)
		pr_err("failed writing at 0x%02x\n", reg);

	return ret;
}
/*flashlight duty set*/
static int sgm3784_write_led1_torch_duty(void)
{
	unsigned char reg, val;
	reg = SGM3784_REG_TORCH_LEVEL_LED1;
	val = sgm3784_torch_level[sgm3784_level_ch1+1];
	sgm3784_write_reg(sgm3784_i2c_client, reg, val);
	
	return 0;
}

static int sgm3784_write_led1_flash_duty(void)
{
	unsigned char reg, val;
	reg = SGM3784_REG_FLASH_LEVEL_LED1;
	val = sgm3784_flash_level[sgm3784_level_ch1+1];
	sgm3784_write_reg(sgm3784_i2c_client, reg, val);
	return 0;
}

static int sgm3784_write_led2_torch_duty(void)
{
	unsigned char reg, val;
	reg = SGM3784_REG_TORCH_LEVEL_LED2;
	val = sgm3784_torch_level[sgm3784_level_ch2+1];
	sgm3784_write_reg(sgm3784_i2c_client, reg, val);
	
	return 0;
}

static int sgm3784_write_led2_flash_duty(void)
{
	unsigned char reg, val;
	reg = SGM3784_REG_FLASH_LEVEL_LED2;
	val = sgm3784_flash_level[sgm3784_level_ch2+1];
	sgm3784_write_reg(sgm3784_i2c_client, reg, val);
	return 0;
}


/* flashlight enable function */
static int sgm3784_enable_ch1(void)
{	
	printk("[jiangguijuan] %s\n",__func__);
	LED1Closeflag = 0;
	return 0;
}

static int sgm3784_enable_ch2(void)
{
	printk("[jiangguijuan] %s\n",__func__);
	LED2Closeflag = 0;
	
	if((LED1Closeflag == 1) && (LED2Closeflag == 1))
	{
		sgm3784_write_reg(sgm3784_i2c_client, SGM3784_REG_ENABLE, 0x00);
	}
	else if(LED1Closeflag == 1)//led1 close
	{
		if(!sgm3784_is_torch())//assist light mode
		{
			sgm3784_pinctrl_set(SGM3784_PINCTRL_PIN_HWSTROBE, SGM3784_PINCTRL_PINSTATE_LOW);
			sgm3784_write_reg(sgm3784_i2c_client, 0x01, 0xf8);
			sgm3784_write_reg(sgm3784_i2c_client, 0x02, 0xff);
			sgm3784_write_reg(sgm3784_i2c_client, 0x03, 0x48);
			sgm3784_write_reg(sgm3784_i2c_client, 0x06, 0x00);
			sgm3784_write_reg(sgm3784_i2c_client, 0x08, 0x00);
			sgm3784_write_led2_torch_duty();
			sgm3784_write_reg(sgm3784_i2c_client, SGM3784_REG_ENABLE, 0x03);
			sgm3784_pinctrl_set(SGM3784_PINCTRL_PIN_HWGPIO, SGM3784_PINCTRL_PINSTATE_HIGH);
			
		}
		else //flash mode
		{
			sgm3784_pinctrl_set(SGM3784_PINCTRL_PIN_HWGPIO, SGM3784_PINCTRL_PINSTATE_LOW);
			sgm3784_write_reg(sgm3784_i2c_client, 0x01, 0xfb);
			sgm3784_write_reg(sgm3784_i2c_client, 0x02, 0xcf);
			sgm3784_write_reg(sgm3784_i2c_client, 0x03, 0x48);
			sgm3784_write_reg(sgm3784_i2c_client, 0x06, 0x00);
			sgm3784_write_reg(sgm3784_i2c_client, 0x08, 0x00);
			sgm3784_write_led2_flash_duty();
			sgm3784_write_reg(sgm3784_i2c_client, SGM3784_REG_ENABLE, 0x03);
			sgm3784_pinctrl_set(SGM3784_PINCTRL_PIN_HWSTROBE, SGM3784_PINCTRL_PINSTATE_HIGH);
		}
	}
	else if(LED2Closeflag == 1)
	{
		if(!sgm3784_is_torch())//assist light mode
		{
			sgm3784_pinctrl_set(SGM3784_PINCTRL_PIN_HWSTROBE, SGM3784_PINCTRL_PINSTATE_LOW);
			sgm3784_write_reg(sgm3784_i2c_client, 0x01, 0xf8);
			sgm3784_write_reg(sgm3784_i2c_client, 0x02, 0xff);
			sgm3784_write_reg(sgm3784_i2c_client, 0x03, 0x48);
			sgm3784_write_reg(sgm3784_i2c_client, 0x09, 0x00);
			sgm3784_write_reg(sgm3784_i2c_client, 0x0b, 0x00);
			sgm3784_write_led1_torch_duty();
			sgm3784_write_reg(sgm3784_i2c_client, SGM3784_REG_ENABLE, 0x03);
			sgm3784_pinctrl_set(SGM3784_PINCTRL_PIN_HWGPIO, SGM3784_PINCTRL_PINSTATE_HIGH);
		}
		else
		{
			sgm3784_pinctrl_set(SGM3784_PINCTRL_PIN_HWGPIO, SGM3784_PINCTRL_PINSTATE_LOW);
			sgm3784_write_reg(sgm3784_i2c_client, 0x01, 0xfb);
			sgm3784_write_reg(sgm3784_i2c_client, 0x02, 0xcf);
			sgm3784_write_reg(sgm3784_i2c_client, 0x03, 0x48);
			sgm3784_write_reg(sgm3784_i2c_client, 0x09, 0x00);
			sgm3784_write_reg(sgm3784_i2c_client, 0x0b, 0x00);
			sgm3784_write_led1_flash_duty();
			sgm3784_write_reg(sgm3784_i2c_client, SGM3784_REG_ENABLE, 0x03);
			sgm3784_pinctrl_set(SGM3784_PINCTRL_PIN_HWSTROBE, SGM3784_PINCTRL_PINSTATE_HIGH);			
			
		}
	}
	else
	{
		if(!sgm3784_is_torch())
		{
			sgm3784_pinctrl_set(SGM3784_PINCTRL_PIN_HWSTROBE, SGM3784_PINCTRL_PINSTATE_LOW);
			sgm3784_write_reg(sgm3784_i2c_client, 0x01, 0xf8);
			sgm3784_write_reg(sgm3784_i2c_client, 0x02, 0xff);
			sgm3784_write_reg(sgm3784_i2c_client, 0x03, 0x48);
			sgm3784_write_led1_torch_duty();
			sgm3784_write_led2_torch_duty();
			sgm3784_write_reg(sgm3784_i2c_client, SGM3784_REG_ENABLE, 0x03);
			sgm3784_pinctrl_set(SGM3784_PINCTRL_PIN_HWGPIO, SGM3784_PINCTRL_PINSTATE_HIGH);			
			
		}
		else
		{
			sgm3784_pinctrl_set(SGM3784_PINCTRL_PIN_HWGPIO, SGM3784_PINCTRL_PINSTATE_LOW);
			sgm3784_write_reg(sgm3784_i2c_client, 0x01, 0xfb);
			sgm3784_write_reg(sgm3784_i2c_client, 0x02, 0xcf);
			sgm3784_write_reg(sgm3784_i2c_client, 0x03, 0x48);
			sgm3784_write_led1_flash_duty();
			sgm3784_write_led2_flash_duty();
			sgm3784_write_reg(sgm3784_i2c_client, SGM3784_REG_ENABLE, 0x03);	
			sgm3784_pinctrl_set(SGM3784_PINCTRL_PIN_HWSTROBE, SGM3784_PINCTRL_PINSTATE_HIGH);	
		}
		
	}	
	
	return 0;

}

int sgm3784_open(void)
{
	sgm3784_pinctrl_set(
			SGM3784_PINCTRL_PIN_HWEN, SGM3784_PINCTRL_PINSTATE_LOW);
	sgm3784_pinctrl_set(
			SGM3784_PINCTRL_PIN_HWSTROBE, SGM3784_PINCTRL_PINSTATE_LOW);
	sgm3784_pinctrl_set(
			SGM3784_PINCTRL_PIN_HWGPIO, SGM3784_PINCTRL_PINSTATE_LOW);
	return 0;
}

/* flashlight init */
int sgm3784_init(void)
{
	int ret;
	unsigned char reg, val;
	printk("[jiangguijuan] %s\n",__func__);
	sgm3784_pinctrl_set(
			SGM3784_PINCTRL_PIN_HWEN, SGM3784_PINCTRL_PINSTATE_LOW);
	msleep(5);
	sgm3784_pinctrl_set(
			SGM3784_PINCTRL_PIN_HWEN, SGM3784_PINCTRL_PINSTATE_HIGH);
	sgm3784_pinctrl_set(
			SGM3784_PINCTRL_PIN_HWSTROBE, SGM3784_PINCTRL_PINSTATE_LOW);
	sgm3784_pinctrl_set(
			SGM3784_PINCTRL_PIN_HWGPIO, SGM3784_PINCTRL_PINSTATE_LOW);
	msleep(20);

	/* clear enable register */
	reg = SGM3784_REG_ENABLE;
	val = SGM3784_DISABLE;
	ret = sgm3784_write_reg(sgm3784_i2c_client, reg, val);

	/* set torch current ramp time and flash timeout */
	return 0;
}

static int sgm3784_enable(int channel)
{
//	sgm3784_init();
	if (channel == SGM3784_CHANNEL_CH1)
		sgm3784_enable_ch1();
	else if (channel == SGM3784_CHANNEL_CH2)
		sgm3784_enable_ch2();
	else {
		pr_err("Error channel\n");
		return -1;
	}

	return 0;
}

/* flashlight disable function */
static int sgm3784_disable_ch1(void)
{
	printk("[jiangguijuan] %s\n",__func__);
	LED1Closeflag = 1;
	return 0;
}

static int sgm3784_disable_ch2(void)
{
	printk("[jiangguijuan] %s\n",__func__);
	LED2Closeflag = 1;
	printk("[jiangguijuan] %s,flag1 = %d,flag2 = %d\n",__func__,LED1Closeflag,LED2Closeflag);
	if((LED1Closeflag == 1) && (LED2Closeflag == 1))
	{
		sgm3784_write_reg(sgm3784_i2c_client, SGM3784_REG_ENABLE, 0x00);
	}
	else if(LED1Closeflag == 1)//led1 close
	{
		if(!sgm3784_is_torch())//assist light mode
		{
			sgm3784_pinctrl_set(SGM3784_PINCTRL_PIN_HWSTROBE, SGM3784_PINCTRL_PINSTATE_LOW);
			sgm3784_write_reg(sgm3784_i2c_client, 0x01, 0xf8);
			sgm3784_write_reg(sgm3784_i2c_client, 0x02, 0xff);
			sgm3784_write_reg(sgm3784_i2c_client, 0x03, 0x48);
			sgm3784_write_reg(sgm3784_i2c_client, 0x06, 0x00);
			sgm3784_write_reg(sgm3784_i2c_client, 0x08, 0x00);
			sgm3784_write_led2_torch_duty();
			sgm3784_write_reg(sgm3784_i2c_client, SGM3784_REG_ENABLE, 0x03);
			sgm3784_pinctrl_set(SGM3784_PINCTRL_PIN_HWGPIO, SGM3784_PINCTRL_PINSTATE_HIGH);			
		}
		else //flash mode
		{
			sgm3784_pinctrl_set(SGM3784_PINCTRL_PIN_HWGPIO, SGM3784_PINCTRL_PINSTATE_LOW);
			sgm3784_write_reg(sgm3784_i2c_client, 0x01, 0xfb);
			sgm3784_write_reg(sgm3784_i2c_client, 0x02, 0xcf);
			sgm3784_write_reg(sgm3784_i2c_client, 0x03, 0x48);
			sgm3784_write_reg(sgm3784_i2c_client, 0x06, 0x00);
			sgm3784_write_reg(sgm3784_i2c_client, 0x08, 0x00);
			sgm3784_write_led2_flash_duty();
			sgm3784_write_reg(sgm3784_i2c_client, SGM3784_REG_ENABLE, 0x03);
			sgm3784_pinctrl_set(SGM3784_PINCTRL_PIN_HWSTROBE, SGM3784_PINCTRL_PINSTATE_HIGH);
		}
	}
	else if(LED2Closeflag == 1)
	{
		if(!sgm3784_is_torch())//assist light mode
		{
			sgm3784_pinctrl_set(SGM3784_PINCTRL_PIN_HWSTROBE, SGM3784_PINCTRL_PINSTATE_LOW);
			sgm3784_write_reg(sgm3784_i2c_client, 0x01, 0xf8);
			sgm3784_write_reg(sgm3784_i2c_client, 0x02, 0xff);
			sgm3784_write_reg(sgm3784_i2c_client, 0x03, 0x48);
			sgm3784_write_reg(sgm3784_i2c_client, 0x09, 0x00);
			sgm3784_write_reg(sgm3784_i2c_client, 0x0b, 0x00);
			sgm3784_write_led1_torch_duty();
			sgm3784_write_reg(sgm3784_i2c_client, SGM3784_REG_ENABLE, 0x03);
			sgm3784_pinctrl_set(SGM3784_PINCTRL_PIN_HWGPIO, SGM3784_PINCTRL_PINSTATE_HIGH);
		}
		else
		{
			sgm3784_pinctrl_set(SGM3784_PINCTRL_PIN_HWGPIO, SGM3784_PINCTRL_PINSTATE_LOW);
			sgm3784_write_reg(sgm3784_i2c_client, 0x01, 0xfb);
			sgm3784_write_reg(sgm3784_i2c_client, 0x02, 0xcf);
			sgm3784_write_reg(sgm3784_i2c_client, 0x03, 0x48);
			sgm3784_write_reg(sgm3784_i2c_client, 0x09, 0x00);
			sgm3784_write_reg(sgm3784_i2c_client, 0x0b, 0x00);
			sgm3784_write_led1_flash_duty();
			sgm3784_write_reg(sgm3784_i2c_client, SGM3784_REG_ENABLE, 0x03);
			sgm3784_pinctrl_set(SGM3784_PINCTRL_PIN_HWSTROBE, SGM3784_PINCTRL_PINSTATE_HIGH);						
		}
	}
	else
	{
		if(!sgm3784_is_torch())
		{
			sgm3784_pinctrl_set(SGM3784_PINCTRL_PIN_HWSTROBE, SGM3784_PINCTRL_PINSTATE_LOW);
			sgm3784_write_reg(sgm3784_i2c_client, 0x01, 0xf8);
			sgm3784_write_reg(sgm3784_i2c_client, 0x02, 0xff);
			sgm3784_write_reg(sgm3784_i2c_client, 0x03, 0x48);
			sgm3784_write_led1_torch_duty();
			sgm3784_write_led2_torch_duty();
			sgm3784_write_reg(sgm3784_i2c_client, SGM3784_REG_ENABLE, 0x03);
			sgm3784_pinctrl_set(SGM3784_PINCTRL_PIN_HWGPIO, SGM3784_PINCTRL_PINSTATE_HIGH);			
			
		}
		else
		{
			sgm3784_pinctrl_set(SGM3784_PINCTRL_PIN_HWGPIO, SGM3784_PINCTRL_PINSTATE_LOW);
			sgm3784_write_reg(sgm3784_i2c_client, 0x01, 0xfb);
			sgm3784_write_reg(sgm3784_i2c_client, 0x02, 0xcf);
			sgm3784_write_reg(sgm3784_i2c_client, 0x03, 0x48);
			sgm3784_write_led1_flash_duty();
			sgm3784_write_led2_flash_duty();
			sgm3784_write_reg(sgm3784_i2c_client, SGM3784_REG_ENABLE, 0x03);	
			sgm3784_pinctrl_set(SGM3784_PINCTRL_PIN_HWSTROBE, SGM3784_PINCTRL_PINSTATE_HIGH);	
		}
		
	}
	return 0;
}

static int sgm3784_disable(int channel)
{
	if (channel == SGM3784_CHANNEL_CH1)
		sgm3784_disable_ch1();
	else if (channel == SGM3784_CHANNEL_CH2)
		sgm3784_disable_ch2();
	else {
		pr_err("Error channel\n");
		return -1;
	}	
	return 0;
}

/* set flashlight level */
static int sgm3784_set_level_ch1(int level)
{
	int ret = 0;
#if 0
	unsigned char reg, val;

	
	sgm3784_level_ch1 = level;
	
	if(!sgm3784_is_torch(level+1))
	{/* set torch brightness level */
		
	}
	else
	{/* set flash brightness level */
		reg = SGM3784_REG_FLASH_LEVEL_LED1;
		val = sgm3784_flash_level[level+1];
		ret = sgm3784_write_reg(sgm3784_i2c_client, reg, val);
	}
#endif
	level = sgm3784_verify_level(level);
	sgm3784_level_ch1 = level;
	return ret;
}

static int sgm3784_set_level_ch2(int level)
{
	int ret = 0;
#if 0
	unsigned char reg, val;

	
	sgm3784_level_ch2 = level;
	
	if(!sgm3784_is_torch())
	{/* set torch brightness level */
		reg = SGM3784_REG_TORCH_LEVEL_LED2;
		val = sgm3784_torch_level[level+1];
		ret = sgm3784_write_reg(sgm3784_i2c_client, reg, val);
		
	}
	else
	{/* set flash brightness level */
		reg = SGM3784_REG_FLASH_LEVEL_LED2;
		val = sgm3784_flash_level[level+1];
		ret = sgm3784_write_reg(sgm3784_i2c_client, reg, val);
		
	}
#endif
	level = sgm3784_verify_level(level);
	sgm3784_level_ch2 = level;
	return ret;
}

static int sgm3784_set_level(int channel, int level)
{
	if (channel == SGM3784_CHANNEL_CH1)
		sgm3784_set_level_ch1(level);
	else if (channel == SGM3784_CHANNEL_CH2)
		sgm3784_set_level_ch2(level);
	else {
		pr_err("Error channel\n");
		return -1;
	}
	return 0;
}		

/* flashlight uninit */
int sgm3784_uninit(void)
{
	printk("[jiangguijuan] %s\n",__func__);
	sgm3784_disable(SGM3784_CHANNEL_CH1);
	sgm3784_disable(SGM3784_CHANNEL_CH2);
	sgm3784_pinctrl_set(
			SGM3784_PINCTRL_PIN_HWEN, SGM3784_PINCTRL_PINSTATE_LOW);
	sgm3784_pinctrl_set(
			SGM3784_PINCTRL_PIN_HWSTROBE, SGM3784_PINCTRL_PINSTATE_LOW);
	sgm3784_pinctrl_set(
			SGM3784_PINCTRL_PIN_HWGPIO, SGM3784_PINCTRL_PINSTATE_LOW);

	return 0;
}


/******************************************************************************
 * Timer and work queue
 *****************************************************************************/
static struct hrtimer sgm3784_timer_ch1;
static struct hrtimer sgm3784_timer_ch2;
static unsigned int sgm3784_timeout_ms[SGM3784_CHANNEL_NUM];

static void sgm3784_work_disable_ch1(struct work_struct *data)
{
	printk("ht work queue callback\n");
	sgm3784_disable_ch1();
}

static void sgm3784_work_disable_ch2(struct work_struct *data)
{
	printk("lt work queue callback\n");
	sgm3784_disable_ch2();
}

static enum hrtimer_restart sgm3784_timer_func_ch1(struct hrtimer *timer)
{
	schedule_work(&sgm3784_work_ch1);
	return HRTIMER_NORESTART;
}

static enum hrtimer_restart sgm3784_timer_func_ch2(struct hrtimer *timer)
{
	schedule_work(&sgm3784_work_ch2);
	return HRTIMER_NORESTART;
}

static int sgm3784_timer_start(int channel, ktime_t ktime)
{
	if (channel == SGM3784_CHANNEL_CH1)
		hrtimer_start(&sgm3784_timer_ch1, ktime, HRTIMER_MODE_REL);
	else if (channel == SGM3784_CHANNEL_CH2)
		hrtimer_start(&sgm3784_timer_ch2, ktime, HRTIMER_MODE_REL);
	else {
		pr_err("Error channel\n");
		return -1;
	}

	return 0;
}

static int sgm3784_timer_cancel(int channel)
{
	if (channel == SGM3784_CHANNEL_CH1)
		hrtimer_cancel(&sgm3784_timer_ch1);
	else if (channel == SGM3784_CHANNEL_CH2)
		hrtimer_cancel(&sgm3784_timer_ch2);
	else {
		pr_err("Error channel\n");
		return -1;
	}

	return 0;
}


/******************************************************************************
 * Flashlight operations
 *****************************************************************************/
static int sgm3784_ioctl(unsigned int cmd, unsigned long arg)
{
	struct flashlight_dev_arg *fl_arg;
	int channel;
	ktime_t ktime;
	unsigned int s;
	unsigned int ns;

	fl_arg = (struct flashlight_dev_arg *)arg;
	channel = fl_arg->channel;

	/* verify channel */
	if (channel < 0 || channel >= SGM3784_CHANNEL_NUM) {
		pr_err("Failed with error channel\n");
		return -EINVAL;
	}

	switch (cmd) {
	case FLASH_IOC_SET_TIME_OUT_TIME_MS:
		printk("sgm3784 FLASH_IOC_SET_TIME_OUT_TIME_MS(%d): %d\n",
				channel, (int)fl_arg->arg);
		sgm3784_timeout_ms[channel] = fl_arg->arg;
		break;

	case FLASH_IOC_SET_DUTY:
		printk("sgm3784 FLASH_IOC_SET_DUTY(%d): %d\n",
				channel, (int)fl_arg->arg);
		sgm3784_set_level(channel, fl_arg->arg);
		break;

	case FLASH_IOC_SET_ONOFF:
		printk("sgm3784 FLASH_IOC_SET_ONOFF(%d): %d\n",
				channel, (int)fl_arg->arg);
		if (fl_arg->arg == 1) {
			if (sgm3784_timeout_ms[channel]) {
				s = sgm3784_timeout_ms[channel] / 1000;
				ns = sgm3784_timeout_ms[channel] % 1000
					* 1000000;
				ktime = ktime_set(s, ns);
				sgm3784_timer_start(channel, ktime);
			}
			sgm3784_enable(channel);
		} else {
			sgm3784_disable(channel);
			sgm3784_timer_cancel(channel);
		}
		break;

	case FLASH_IOC_GET_DUTY_NUMBER:
		printk("sgm3784 FLASH_IOC_GET_DUTY_NUMBER(%d)\n", channel);
		fl_arg->arg = SGM3784_LEVEL_NUM;
		break;

	case FLASH_IOC_GET_MAX_TORCH_DUTY:
		printk("sgm3784 FLASH_IOC_GET_MAX_TORCH_DUTY(%d)\n", channel);
		fl_arg->arg = SGM3784_LEVEL_TORCH - 1;
		break;

	case FLASH_IOC_GET_DUTY_CURRENT:
		fl_arg->arg = sgm3784_verify_level(fl_arg->arg);
		printk("sgm3784 FLASH_IOC_GET_DUTY_CURRENT(%d): %d\n",
				channel, (int)fl_arg->arg);
		fl_arg->arg = sgm3784_current[fl_arg->arg];
		break;

	case FLASH_IOC_GET_HW_TIMEOUT:
		printk("sgm3784 FLASH_IOC_GET_HW_TIMEOUT(%d)\n", channel);
		fl_arg->arg = SGM3784_HW_TIMEOUT;
		break;
		
	default:
		printk("No such command and arg(%d): (%d, %d)\n",
				channel, _IOC_NR(cmd), (int)fl_arg->arg);
		return -ENOTTY;
	}

	return 0;
}


static int sgm3784_release(void)
{
	/* Move to set driver for saving power */
	sgm3784_pinctrl_set(SGM3784_PINCTRL_PIN_HWEN, SGM3784_PINCTRL_PINSTATE_LOW);
	sgm3784_pinctrl_set(SGM3784_PINCTRL_PIN_HWSTROBE, SGM3784_PINCTRL_PINSTATE_LOW);
	sgm3784_pinctrl_set(SGM3784_PINCTRL_PIN_HWGPIO, SGM3784_PINCTRL_PINSTATE_LOW);
	return 0;
}

static int sgm3784_set_driver(int set)
{
	int ret = 0;
	printk("[jiangguijuan] %s,set = %d\n",__func__,set);
	/* set chip and usage count */
	mutex_lock(&sgm3784_mutex);
	if (set) {
		if (!use_count)
			ret = sgm3784_init();
		use_count++;
		pr_debug("Set driver: %d\n", use_count);
	} else {
		use_count--;
		if (!use_count)
			ret = sgm3784_uninit();
		if (use_count < 0)
			use_count = 0;
		pr_debug("Unset driver: %d\n", use_count);
	}
	mutex_unlock(&sgm3784_mutex);

	return ret;
}

static ssize_t sgm3784_strobe_store(struct flashlight_arg arg)
{
	printk("[jiangguijuan]%s\n",__func__);
	#if 0
	
	sgm3784_set_driver(1);
	sgm3784_set_level(arg.channel, arg.level);
	sgm3784_timeout_ms[arg.channel] = 0;
	sgm3784_enable(arg.channel);
	msleep(arg.dur);
	sgm3784_disable(arg.channel);
	sgm3784_set_driver(0);
	#endif
	return 0;
}

static struct flashlight_operations sgm3784_ops = {
	sgm3784_open,
	sgm3784_release,
	sgm3784_ioctl,
	sgm3784_strobe_store,
	sgm3784_set_driver
};


/******************************************************************************
 * I2C device and driver
 *****************************************************************************/
static int sgm3784_chip_init(struct sgm3784_chip_data *chip)
{
	/* NOTE: Chip initialication move to "set driver" for power saving.
	 * sgm3784_init();
	 */

	return 0;
}

static int sgm3784_parse_dt(struct device *dev,
		struct sgm3784_platform_data *pdata)
{
	//struct device_node *np, *cnp;
	struct device_node *np;
	u32 decouple = 0;
	int i = 0;

	if (!dev || !dev->of_node || !pdata)
		return -ENODEV;

	np = dev->of_node;

	of_property_read_u32(np, "decouple", &decouple);
		printk("decouple,%d\n",decouple);
#if 0
	pdata->channel_num = of_get_child_count(np);
	printk("jiang add channel_num = %d\n",pdata->channel_num);
	if (!pdata->channel_num) {
		pr_info("Parse no dt, node.\n");
		return 0;
	}
	printk("Channel number(%d).\n", pdata->channel_num);

	if (of_property_read_u32(np, "decouple", &decouple))
		printk("Parse no dt, decouple.\n");

	pdata->dev_id = devm_kzalloc(dev,
			pdata->channel_num *
			sizeof(struct flashlight_device_id),
			GFP_KERNEL);
	if (!pdata->dev_id)
		return -ENOMEM;

	for_each_child_of_node(np, cnp) {
		if (of_property_read_u32(cnp, "type", &pdata->dev_id[i].type))
			goto err_node_put;
		if (of_property_read_u32(cnp, "ct", &pdata->dev_id[i].ct))
			goto err_node_put;
		if (of_property_read_u32(cnp, "part", &pdata->dev_id[i].part))
			goto err_node_put;
		snprintf(pdata->dev_id[i].name, FLASHLIGHT_NAME_SIZE,
				SGM3784_NAME);
		pdata->dev_id[i].channel = i;
		pdata->dev_id[i].decouple = decouple;

		printk("Parse dt (type,ct,part,name,channel,decouple)=(%d,%d,%d,%s,%d,%d).\n",
				pdata->dev_id[i].type, pdata->dev_id[i].ct,
				pdata->dev_id[i].part, pdata->dev_id[i].name,
				pdata->dev_id[i].channel,
				pdata->dev_id[i].decouple);
		i++;
	}
	
#else 
	pdata->channel_num = 2;		
	decouple = 0;
	
	pdata->dev_id = devm_kzalloc(dev,
			pdata->channel_num *
			sizeof(struct flashlight_device_id),
			GFP_KERNEL);
	if (!pdata->dev_id)
		return -ENOMEM;
	
	pdata->dev_id[0].type = 0;
	pdata->dev_id[0].ct = 0;
	pdata->dev_id[0].part = 0;
	snprintf(pdata->dev_id[0].name, FLASHLIGHT_NAME_SIZE,
				SGM3784_NAME);
	pdata->dev_id[0].channel = 0;
	pdata->dev_id[0].decouple = decouple;
	
	pdata->dev_id[1].type = 0;
	pdata->dev_id[1].ct = 1;
	pdata->dev_id[1].part = 0;
	snprintf(pdata->dev_id[1].name, FLASHLIGHT_NAME_SIZE,
				SGM3784_NAME);
	pdata->dev_id[1].channel = 1;
	pdata->dev_id[1].decouple = decouple;
	
	for(i = 0;i<2 ;i++)
	{
		printk("Parse dt (type,ct,part,name,channel,decouple)=(%d,%d,%d,%s,%d,%d).\n",
				pdata->dev_id[i].type, pdata->dev_id[i].ct,
				pdata->dev_id[i].part, pdata->dev_id[i].name,
				pdata->dev_id[i].channel,
				pdata->dev_id[i].decouple);
				
	}	
	
#endif

	return 0;

//err_node_put:
	//of_node_put(cnp);
	return -EINVAL;
}

static int sgm3784_i2c_probe(
		struct i2c_client *client, const struct i2c_device_id *id)
{
	struct sgm3784_platform_data *pdata = dev_get_platdata(&client->dev);
	struct sgm3784_chip_data *chip;
	int err;
	int i;

	printk("%s\n",__func__);

	/* check i2c */
	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		pr_err("Failed to check i2c functionality.\n");
		err = -ENODEV;
		goto err_out;
	}

	/* init chip private data */
	chip = kzalloc(sizeof(struct sgm3784_chip_data), GFP_KERNEL);
	if (!chip) {
		err = -ENOMEM;
		goto err_out;
	}
	chip->client = client;

	/* init platform data */
	if (!pdata) {
		pdata = devm_kzalloc(&client->dev, sizeof(*pdata), GFP_KERNEL);
		if (!pdata) {
			err = -ENOMEM;
			goto err_free;
		}
		client->dev.platform_data = pdata;
		err = sgm3784_parse_dt(&client->dev, pdata);
		if (err)
			goto err_free;
	}
	chip->pdata = pdata;
	i2c_set_clientdata(client, chip);
	sgm3784_i2c_client = client;

	/* init mutex and spinlock */
	mutex_init(&chip->lock);

	/* init work queue */
	INIT_WORK(&sgm3784_work_ch1, sgm3784_work_disable_ch1);
	INIT_WORK(&sgm3784_work_ch2, sgm3784_work_disable_ch2);

	/* init timer */
	hrtimer_init(&sgm3784_timer_ch1, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	sgm3784_timer_ch1.function = sgm3784_timer_func_ch1;
	hrtimer_init(&sgm3784_timer_ch2, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	sgm3784_timer_ch2.function = sgm3784_timer_func_ch2;
	sgm3784_timeout_ms[SGM3784_CHANNEL_CH1] = 100;
	sgm3784_timeout_ms[SGM3784_CHANNEL_CH2] = 100;

	/* init chip hw */
	sgm3784_chip_init(chip);

	/* clear usage count */
	use_count = 0;

	/* register flashlight device */
	if (pdata->channel_num) {
		printk("[jiangguijuan] channel_num = %d\n",pdata->channel_num);
		for (i = 0; i < pdata->channel_num; i++)
			if (flashlight_dev_register_by_device_id(
						&pdata->dev_id[i],
						&sgm3784_ops)) {
				err = -EFAULT;
				goto err_free;
			}
	} else {
		if (flashlight_dev_register(SGM3784_NAME, &sgm3784_ops)) {
			err = -EFAULT;
			goto err_free;
		}
	}

	pr_debug("Probe done.\n");

	return 0;

err_free:
	i2c_set_clientdata(client, NULL);
	kfree(chip);
err_out:
	return err;
}

static int sgm3784_i2c_remove(struct i2c_client *client)
{
	struct sgm3784_platform_data *pdata = dev_get_platdata(&client->dev);
	struct sgm3784_chip_data *chip = i2c_get_clientdata(client);
	int i;

	pr_debug("Remove start.\n");

	client->dev.platform_data = NULL;

	/* unregister flashlight device */
	if (pdata && pdata->channel_num)
		for (i = 0; i < pdata->channel_num; i++)
			flashlight_dev_unregister_by_device_id(
					&pdata->dev_id[i]);
	else
		flashlight_dev_unregister(SGM3784_NAME);

	/* flush work queue */
	flush_work(&sgm3784_work_ch1);
	flush_work(&sgm3784_work_ch2);

	/* free resource */
	kfree(chip);

	pr_debug("Remove done.\n");

	return 0;
}

static const struct i2c_device_id sgm3784_i2c_id[] = {
	{SGM3784_NAME, 0},
	{}
};

#ifdef CONFIG_OF
static const struct of_device_id sgm3784_i2c_of_match[] = {
	{.compatible = SGM3784_DTNAME_I2C},
	{},
};
#endif

static struct i2c_driver sgm3784_i2c_driver = {
	.driver = {
		.name = SGM3784_NAME,
#ifdef CONFIG_OF
		.of_match_table = sgm3784_i2c_of_match,
#endif
	},
	.probe = sgm3784_i2c_probe,
	.remove = sgm3784_i2c_remove,
	.id_table = sgm3784_i2c_id,
};


/******************************************************************************
 * Platform device and driver
 *****************************************************************************/
static int sgm3784_probe(struct platform_device *dev)
{
	printk("sgm3784_probe Probe start.\n");

	/* init pinctrl */
	if (sgm3784_pinctrl_init(dev)) {
		pr_debug("Failed to init pinctrl.\n");
		return -1;
	}

	if (i2c_add_driver(&sgm3784_i2c_driver)) {
		pr_debug("Failed to add i2c driver.\n");
		return -1;
	}

	pr_debug("Probe done.\n");

	return 0;
}

static int sgm3784_remove(struct platform_device *dev)
{
	pr_debug("Remove start.\n");

	i2c_del_driver(&sgm3784_i2c_driver);

	pr_debug("Remove done.\n");

	return 0;
}

#ifdef CONFIG_OF
static const struct of_device_id sgm3784_of_match[] = {
	{.compatible = SGM3784_DTNAME},
	{},
};
MODULE_DEVICE_TABLE(of, sgm3784_of_match);
#else
static struct platform_device sgm3784_platform_device[] = {
	{
		.name = SGM3784_NAME,
		.id = 0,
		.dev = {}
	},
	{}
};
MODULE_DEVICE_TABLE(platform, sgm3784_platform_device);
#endif

static struct platform_driver sgm3784_platform_driver = {
	.probe = sgm3784_probe,
	.remove = sgm3784_remove,
	.driver = {
		.name = SGM3784_NAME,
		.owner = THIS_MODULE,
#ifdef CONFIG_OF
		.of_match_table = sgm3784_of_match,
#endif
	},
};

static int __init flashlight_sgm3784_init(void)
{
	int ret;

	pr_debug("Init start.\n");

#ifndef CONFIG_OF
	ret = platform_device_register(&sgm3784_platform_device);
	if (ret) {
		pr_err("Failed to register platform device\n");
		return ret;
	}
#endif

	ret = platform_driver_register(&sgm3784_platform_driver);
	if (ret) {
		pr_err("Failed to register platform driver\n");
		return ret;
	}

	pr_debug("Init done.\n");

	return 0;
}

static void __exit flashlight_sgm3784_exit(void)
{
	pr_debug("Exit start.\n");

	platform_driver_unregister(&sgm3784_platform_driver);

	pr_debug("Exit done.\n");
}

module_init(flashlight_sgm3784_init);
module_exit(flashlight_sgm3784_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Simon Wang <Simon-TCH.Wang@mediatek.com>");
MODULE_DESCRIPTION("MTK Flashlight SGM3784 Driver");

