/*
 * Copyright (C) 2019 MediaTek Inc.
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
#include <linux/pinctrl/consumer.h>

#include "flashlight-core.h"
#include "flashlight-dt.h"

#define TAG_NAME "[flashligh_aw3641e_drv]"
#define PK_DBG_NONE(fmt, arg...)  do {} while (0)
#define PK_DBG_FUNC(fmt, arg...)  pr_debug(TAG_NAME "%s: " fmt, __func__, ##arg)
#define PK_ERR(fmt, arg...)       pr_info(TAG_NAME "%s: " fmt, __func__, ##arg)

#define DEBUG_LEDS_STROBE
#ifdef DEBUG_LEDS_STROBE
#define PK_INF(fmt, arg...)       pr_info(TAG_NAME "%s is called.\n", __func__)
#define PK_DBG                    PK_DBG_FUNC
#else
#define PK_INF(fmt, arg...)       do {} while (0)
#define PK_DBG(a, ...)
#endif


/* define device tree */
#ifndef aw3641e_DTNAME
#define aw3641e_DTNAME "mediatek,flashlights_aw3641e"
#endif

#define aw3641e_NAME "flashlights-aw3641e"

/* define registers */

/* define mutex and work queue */
static DEFINE_MUTEX(aw3641e_mutex);
static struct work_struct aw3641e_work;

/* define pinctrl */
#define AW3641E_PINCTRL_STATE_FLASH_HIGH "flash_high"
#define AW3641E_PINCTRL_STATE_FLASH_LOW  "flash_low"
#define AW3641E_PINCTRL_STATE_EN_HIGH "en_high"
#define AW3641E_PINCTRL_STATE_EN_LOW  "en_low"

static struct pinctrl *aw3641e_pinctrl;
static struct pinctrl_state *aw3641e_flash_high;
static struct pinctrl_state *aw3641e_flash_low;
static struct pinctrl_state *aw3641e_en_high;
static struct pinctrl_state *aw3641e_en_low;

/* define usage count */
static int use_count;
static int g_flash_duty = -1;

/* platform data */
struct aw3641e_platform_data {
	int channel_num;
	struct flashlight_device_id *dev_id;
};
/******************************************************************************
 * Pinctrl configuration
 *****************************************************************************/
static int AW3641E_PINCTRL_init(struct platform_device *pdev)
{
	int ret = 0;

	/* get pinctrl */
	aw3641e_pinctrl = devm_pinctrl_get(&pdev->dev);
	if (IS_ERR(aw3641e_pinctrl)) {
		PK_ERR("Failed to get flashlight pinctrl.\n");
		ret = PTR_ERR(aw3641e_pinctrl);
	}

	/*  Flashlight pin initialization */
	aw3641e_flash_high = pinctrl_lookup_state(
			aw3641e_pinctrl, AW3641E_PINCTRL_STATE_FLASH_HIGH);
	if (IS_ERR(aw3641e_flash_high)) {
		PK_ERR("Failed to init (%s)\n",
			AW3641E_PINCTRL_STATE_FLASH_HIGH);
		ret = PTR_ERR(aw3641e_flash_high);
	}
	aw3641e_flash_low = pinctrl_lookup_state(
			aw3641e_pinctrl, AW3641E_PINCTRL_STATE_FLASH_LOW);
	if (IS_ERR(aw3641e_flash_low)) {
		PK_ERR("Failed to init (%s)\n", AW3641E_PINCTRL_STATE_FLASH_LOW);
		ret = PTR_ERR(aw3641e_flash_low);
	}

	aw3641e_en_high = pinctrl_lookup_state(
			aw3641e_pinctrl, AW3641E_PINCTRL_STATE_EN_HIGH);
	if (IS_ERR(aw3641e_en_high)) {
		PK_ERR("Failed to init (%s)\n",
			AW3641E_PINCTRL_STATE_EN_HIGH);
		ret = PTR_ERR(aw3641e_en_high);
	}
	aw3641e_en_low = pinctrl_lookup_state(
			aw3641e_pinctrl, AW3641E_PINCTRL_STATE_EN_LOW);
	if (IS_ERR(aw3641e_en_low)) {
		PK_ERR("Failed to init (%s)\n", AW3641E_PINCTRL_STATE_EN_LOW);
		ret = PTR_ERR(aw3641e_en_low);
	}

	return ret;
}
/******************************************************************************
 * aw3641e operations
 *****************************************************************************/
static void aw3641e_ctrl_pulse(int num)
{
	if((num <= 0)&&(num > 16))//error
		return;
		
	pinctrl_select_state(aw3641e_pinctrl, aw3641e_flash_high);
	udelay(1);
	while(--num)
	{
		pinctrl_select_state(aw3641e_pinctrl, aw3641e_flash_low);
		udelay(1);
		pinctrl_select_state(aw3641e_pinctrl, aw3641e_flash_high);
		udelay(1);
	}
}

/* flashlight enable function */
static int aw3641e_enable(void)
{
	printk("wxs aw3641e_enable g_flash_duty:%d\n",g_flash_duty);
	if(g_flash_duty == 1)
	{
		pinctrl_select_state(aw3641e_pinctrl, aw3641e_flash_low);
		mdelay(1);
		pinctrl_select_state(aw3641e_pinctrl, aw3641e_en_high);
		mdelay(3);
		aw3641e_ctrl_pulse(9);
	}
	else
	{
		pinctrl_select_state(aw3641e_pinctrl, aw3641e_en_high);
		pinctrl_select_state(aw3641e_pinctrl, aw3641e_flash_low);
		printk("wxs aw3641e_enable enter touch mode\n");
	}

	return 0;
}

/* flashlight disable function */
static int aw3641e_disable(void)
{
	printk("wxs aw3641e_disable\n");
	pinctrl_select_state(aw3641e_pinctrl, aw3641e_flash_low);
	pinctrl_select_state(aw3641e_pinctrl, aw3641e_en_low);
	mdelay(1);
	return 0;
}

/* set flashlight level */
static int aw3641e_set_level(int level)
{
	printk("wxs aw3641e_set_level g_flash_duty:%d\n",level);
	g_flash_duty = level;
	return 0;
}

/* flashlight init */
static int aw3641e_init(void)
{
	printk("wxs aw3641e_init\n");
	pinctrl_select_state(aw3641e_pinctrl, aw3641e_flash_low);
	pinctrl_select_state(aw3641e_pinctrl, aw3641e_en_low);
	return 0;
}

/* flashlight uninit */
static int aw3641e_uninit(void)
{
	aw3641e_disable();
	return 0;
}

/******************************************************************************
 * Timer and work queue
 *****************************************************************************/
static struct hrtimer aw3641e_timer;
static unsigned int aw3641e_timeout_ms;

static void aw3641e_work_disable(struct work_struct *data)
{
	PK_DBG("work queue callback\n");
	//aw3641e_disable();
}

static enum hrtimer_restart aw3641e_timer_func(struct hrtimer *timer)
{
	schedule_work(&aw3641e_work);
	return HRTIMER_NORESTART;
}


/******************************************************************************
 * Flashlight operations
 *****************************************************************************/
static int aw3641e_ioctl(unsigned int cmd, unsigned long arg)
{
	struct flashlight_dev_arg *fl_arg;
	int channel;
	ktime_t ktime;
	unsigned int s;
	unsigned int ns;

	fl_arg = (struct flashlight_dev_arg *)arg;
	channel = fl_arg->channel;
	if(channel != 0)
		return 0;

	switch (cmd) {
	case FLASH_IOC_SET_TIME_OUT_TIME_MS:
		printk("FLASH_IOC_SET_TIME_OUT_TIME_MS(%d): %d\n",
				channel, (int)fl_arg->arg);
		aw3641e_timeout_ms = fl_arg->arg;
		break;

	case FLASH_IOC_SET_DUTY:
		printk("FLASH_IOC_SET_DUTY(%d): %d\n",
				channel, (int)fl_arg->arg);
		aw3641e_set_level(fl_arg->arg);
		break;

	case FLASH_IOC_SET_ONOFF:
		printk("FLASH_IOC_SET_ONOFF channel(%d): fl_arg->arg :%d ,aw3641e_timeout_ms:%d\n",
				channel, (int)fl_arg->arg, aw3641e_timeout_ms);
		if (fl_arg->arg == 1) {
			if (aw3641e_timeout_ms) {
				s = aw3641e_timeout_ms / 1000;
				ns = aw3641e_timeout_ms % 1000 * 1000000;
				ktime = ktime_set(s, ns);
				hrtimer_start(&aw3641e_timer, ktime,
						HRTIMER_MODE_REL);
			}
			aw3641e_enable();
		} else {
			aw3641e_disable();
			hrtimer_cancel(&aw3641e_timer);
		}
		break;
	default:
		PK_INF("No such command and arg(%d): (%d, %d)\n",
				channel, _IOC_NR(cmd), (int)fl_arg->arg);
		return -ENOTTY;
	}

	return 0;
}

static int aw3641e_open(void)
{
	/* Move to set driver for saving power */
	return 0;
}

static int aw3641e_release(void)
{
	/* Move to set driver for saving power */
	return 0;
}

static int aw3641e_set_driver(int set)
{
	int ret = 0;

	/* set chip and usage count */
	mutex_lock(&aw3641e_mutex);
	if (set) {
		if (!use_count)
			ret = aw3641e_init();
		use_count++;
		PK_DBG("Set driver: %d\n", use_count);
	} else {
		use_count--;
		if (!use_count)
			ret = aw3641e_uninit();
		if (use_count < 0)
			use_count = 0;
		PK_DBG("Unset driver: %d\n", use_count);
	}
	mutex_unlock(&aw3641e_mutex);

	return ret;
}

static ssize_t aw3641e_strobe_store(struct flashlight_arg arg)
{
	aw3641e_set_driver(1);
	aw3641e_set_level(arg.level);
	aw3641e_timeout_ms = 0;
	aw3641e_enable();
	msleep(arg.dur);
	aw3641e_disable();
	aw3641e_set_driver(0);

	return 0;
}

static struct flashlight_operations aw3641e_ops = {
	aw3641e_open,
	aw3641e_release,
	aw3641e_ioctl,
	aw3641e_strobe_store,
	aw3641e_set_driver
};


/******************************************************************************
 * Platform device and driver
 *****************************************************************************/
static int aw3641e_chip_init(void)
{
	/* NOTE: Chip initialication move to "set driver" for power saving.
	 * aw3641e_init();
	 */

	return 0;
}

static int aw3641e_parse_dt(struct device *dev,
		struct aw3641e_platform_data *pdata)
{
	struct device_node *np, *cnp;
	u32 decouple = 0;
	int i = 0;

	if (!dev || !dev->of_node || !pdata)
		return -ENODEV;

	np = dev->of_node;

	pdata->channel_num = of_get_child_count(np);
	if (!pdata->channel_num) {
		PK_INF("Parse no dt, node.\n");
		return 0;
	}
	PK_INF("Channel number(%d).\n", pdata->channel_num);

	if (of_property_read_u32(np, "decouple", &decouple))
		PK_INF("Parse no dt, decouple.\n");

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
				aw3641e_NAME);
		pdata->dev_id[i].channel = i;
		pdata->dev_id[i].decouple = decouple;

		pr_info("Parse dt (type,ct,part,name,channel,decouple)=(%d,%d,%d,%s,%d,%d).\n",
				pdata->dev_id[i].type, pdata->dev_id[i].ct,
				pdata->dev_id[i].part, pdata->dev_id[i].name,
				pdata->dev_id[i].channel,
				pdata->dev_id[i].decouple);
		i++;
	}

	return 0;

err_node_put:
	of_node_put(cnp);
	return -EINVAL;
}

static int aw3641e_probe(struct platform_device *pdev)
{
	struct aw3641e_platform_data *pdata = dev_get_platdata(&pdev->dev);
	int err;
	int i;

	PK_DBG("Probe start.\n");

	/* init pinctrl */
	if (AW3641E_PINCTRL_init(pdev)) {
		PK_DBG("Failed to init pinctrl.\n");
		err = -EFAULT;
		goto err;
	}

	/* init platform data */
	if (!pdata) {
		pdata = devm_kzalloc(&pdev->dev, sizeof(*pdata), GFP_KERNEL);
		if (!pdata) {
			err = -ENOMEM;
			goto err;
		}
		pdev->dev.platform_data = pdata;
		err = aw3641e_parse_dt(&pdev->dev, pdata);
		if (err)
			goto err;
	}

	/* init work queue */
	INIT_WORK(&aw3641e_work, aw3641e_work_disable);

	/* init timer */
	hrtimer_init(&aw3641e_timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	aw3641e_timer.function = aw3641e_timer_func;
	aw3641e_timeout_ms = 100;

	/* init chip hw */
	aw3641e_chip_init();

	/* clear usage count */
	use_count = 0;

	/* register flashlight device */
	if (pdata->channel_num) {
		for (i = 0; i < pdata->channel_num; i++)
			if (flashlight_dev_register_by_device_id(
						&pdata->dev_id[i],
						&aw3641e_ops)) {
				err = -EFAULT;
				goto err;
			}
	} else {
		if (flashlight_dev_register(aw3641e_NAME, &aw3641e_ops)) {
			err = -EFAULT;
			goto err;
		}
	}

	PK_DBG("Probe done.\n");
	aw3641e_init();
	return 0;
err:
	return err;
}

static int aw3641e_remove(struct platform_device *pdev)
{
	struct aw3641e_platform_data *pdata = dev_get_platdata(&pdev->dev);
	int i;

	PK_DBG("Remove start.\n");

	pdev->dev.platform_data = NULL;

	/* unregister flashlight device */
	if (pdata && pdata->channel_num)
		for (i = 0; i < pdata->channel_num; i++)
			flashlight_dev_unregister_by_device_id(
					&pdata->dev_id[i]);
	else
		flashlight_dev_unregister(aw3641e_NAME);

	/* flush work queue */
	flush_work(&aw3641e_work);

	PK_DBG("Remove done.\n");

	return 0;
}

static const struct of_device_id aw3641e_gpio_of_match[] = {
	{.compatible = aw3641e_DTNAME},
	{},
};
MODULE_DEVICE_TABLE(of, aw3641e_gpio_of_match);

static struct platform_driver aw3641e_platform_driver = {
	.probe = aw3641e_probe,
	.remove = aw3641e_remove,
	.driver = {
		.name = aw3641e_NAME,
		.owner = THIS_MODULE,
#ifdef CONFIG_OF
		.of_match_table = aw3641e_gpio_of_match,
#endif
	},
};

static int __init flashlight_aw3641e_init(void)
{
	int ret;

	PK_DBG("Init start.\n");

#ifndef CONFIG_OF
	ret = platform_device_register(&aw3641e_gpio_platform_device);
	if (ret) {
		PK_ERR("Failed to register platform device\n");
		return ret;
	}
#endif

	ret = platform_driver_register(&aw3641e_platform_driver);
	if (ret) {
		PK_ERR("Failed to register platform driver\n");
		return ret;
	}

	PK_DBG("Init done.\n");

	return 0;
}

static void __exit flashlight_aw3641e_exit(void)
{
	PK_DBG("Exit start.\n");

	platform_driver_unregister(&aw3641e_platform_driver);

	PK_DBG("Exit done.\n");
}

module_init(flashlight_aw3641e_init);
module_exit(flashlight_aw3641e_exit);

MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("Dongchun Zhu <dongchun.zhu@mediatek.com>");
MODULE_DESCRIPTION("MTK Flashlight aw3641e GPIO Driver");

