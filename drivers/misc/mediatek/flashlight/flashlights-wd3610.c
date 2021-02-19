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

#define TAG_NAME "[flashligh_wd3610_drv]"
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
#ifndef wd3610_DTNAME
#define wd3610_DTNAME "mediatek,flashlights_wd3610"
#endif

#define wd3610_NAME "flashlights-wd3610"

/* define registers */

/* define mutex and work queue */
static DEFINE_MUTEX(wd3610_mutex);
static struct work_struct wd3610_work;

/* define pinctrl */
#define wd3610_PINCTRL_PIN_TORCH 0
#define wd3610_PINCTRL_PIN_FLASH 1
#define wd3610_PINCTRL_PINSTATE_LOW 0
#define wd3610_PINCTRL_PINSTATE_HIGH 1
#define wd3610_PINCTRL_STATE_FLASH_HIGH "flash_high"
#define wd3610_PINCTRL_STATE_FLASH_LOW  "flash_low"
#define wd3610_PINCTRL_STATE_TORCH_HIGH "torch_high"
#define wd3610_PINCTRL_STATE_TORCH_LOW  "torch_low"

static struct pinctrl *wd3610_pinctrl;
static struct pinctrl_state *wd3610_flash_high;
static struct pinctrl_state *wd3610_flash_low;
static struct pinctrl_state *wd3610_torch_high;
static struct pinctrl_state *wd3610_torch_low;

/* define usage count */
static int use_count;

static int g_flash_duty = -1;

/* platform data */
struct wd3610_platform_data {
	int channel_num;
	struct flashlight_device_id *dev_id;
};


/******************************************************************************
 * Pinctrl configuration
 *****************************************************************************/
static int wd3610_pinctrl_init(struct platform_device *pdev)
{
	int ret = 0;

	/* get pinctrl */
	wd3610_pinctrl = devm_pinctrl_get(&pdev->dev);
	if (IS_ERR(wd3610_pinctrl)) {
		PK_ERR("Failed to get flashlight pinctrl.\n");
		ret = PTR_ERR(wd3610_pinctrl);
	}

	/*  Flashlight pin initialization */
	wd3610_flash_high = pinctrl_lookup_state(
			wd3610_pinctrl, wd3610_PINCTRL_STATE_FLASH_HIGH);
	if (IS_ERR(wd3610_flash_high)) {
		PK_ERR("Failed to init (%s)\n",
			wd3610_PINCTRL_STATE_FLASH_HIGH);
		ret = PTR_ERR(wd3610_flash_high);
	}
	wd3610_flash_low = pinctrl_lookup_state(
			wd3610_pinctrl, wd3610_PINCTRL_STATE_FLASH_LOW);
	if (IS_ERR(wd3610_flash_low)) {
		PK_ERR("Failed to init (%s)\n", wd3610_PINCTRL_STATE_FLASH_LOW);
		ret = PTR_ERR(wd3610_flash_low);
	}

	wd3610_torch_high = pinctrl_lookup_state(
			wd3610_pinctrl, wd3610_PINCTRL_STATE_TORCH_HIGH);
	if (IS_ERR(wd3610_torch_high)) {
		PK_ERR("Failed to init (%s)\n",
			wd3610_PINCTRL_STATE_TORCH_HIGH);
		ret = PTR_ERR(wd3610_torch_high);
	}
	wd3610_torch_low = pinctrl_lookup_state(
			wd3610_pinctrl, wd3610_PINCTRL_STATE_TORCH_LOW);
	if (IS_ERR(wd3610_torch_low)) {
		PK_ERR("Failed to init (%s)\n", wd3610_PINCTRL_STATE_TORCH_LOW);
		ret = PTR_ERR(wd3610_torch_low);
	}

	return ret;
}

static int wd3610_pinctrl_set(int pin, int state)
{
	int ret = 0;

	if (IS_ERR(wd3610_pinctrl)) {
		PK_ERR("pinctrl is not available\n");
		return -1;
	}

	switch (pin) {
	case wd3610_PINCTRL_PIN_FLASH:
		if (state == wd3610_PINCTRL_PINSTATE_LOW &&
				!IS_ERR(wd3610_flash_low))
			ret = pinctrl_select_state(
					wd3610_pinctrl, wd3610_flash_low);
		else if (state == wd3610_PINCTRL_PINSTATE_HIGH &&
				!IS_ERR(wd3610_flash_high))
			ret = pinctrl_select_state(
					wd3610_pinctrl, wd3610_flash_high);
		else
			PK_ERR("set err, pin(%d) state(%d)\n", pin, state);
		break;
	case wd3610_PINCTRL_PIN_TORCH:
		if (state == wd3610_PINCTRL_PINSTATE_LOW &&
				!IS_ERR(wd3610_torch_low))
			ret = pinctrl_select_state(
					wd3610_pinctrl, wd3610_torch_low);
		else if (state == wd3610_PINCTRL_PINSTATE_HIGH &&
				!IS_ERR(wd3610_torch_high))
			ret = pinctrl_select_state(
					wd3610_pinctrl, wd3610_torch_high);
		else
			PK_ERR("set err, pin(%d) state(%d)\n", pin, state);
		break;
	default:
		PK_ERR("set err, pin(%d) state(%d)\n", pin, state);
		break;
	}

	PK_DBG("pin(%d) state(%d), ret:%d\n", pin, state, ret);

	return ret;
}


/******************************************************************************
 * wd3610 operations
 *****************************************************************************/
/* flashlight enable function */
static int wd3610_enable(void)
{
	int pin_flash = wd3610_PINCTRL_PIN_FLASH;
	int pin_torch = wd3610_PINCTRL_PIN_TORCH;

	PK_DBG("g_flash_duty %d\n", g_flash_duty);

	if (g_flash_duty == 1)   /* flash mode */
		wd3610_pinctrl_set(pin_flash, wd3610_PINCTRL_PINSTATE_HIGH);
	else                     /* torch mode */
		wd3610_pinctrl_set(pin_torch, wd3610_PINCTRL_PINSTATE_HIGH);

	return 0;
}

/* flashlight disable function */
static int wd3610_disable(void)
{
	int pin_flash = wd3610_PINCTRL_PIN_FLASH;
	int pin_torch = wd3610_PINCTRL_PIN_TORCH;
	int state = wd3610_PINCTRL_PINSTATE_LOW;

	wd3610_pinctrl_set(pin_torch, state);
	wd3610_pinctrl_set(pin_flash, state);

	return 0;
}

/* set flashlight level */
static int wd3610_set_level(int level)
{
	g_flash_duty = level;
	return 0;
}

/* flashlight init */
static int wd3610_init(void)
{
	int pin_flash = wd3610_PINCTRL_PIN_FLASH;
	int pin_torch = wd3610_PINCTRL_PIN_TORCH;
	int state = wd3610_PINCTRL_PINSTATE_LOW;

	wd3610_pinctrl_set(pin_torch, state);
	wd3610_pinctrl_set(pin_flash, state);

	return 0;
}

/* flashlight uninit */
static int wd3610_uninit(void)
{
	wd3610_disable();
	return 0;
}

/******************************************************************************
 * Timer and work queue
 *****************************************************************************/
static struct hrtimer wd3610_timer;
static unsigned int wd3610_timeout_ms;

static void wd3610_work_disable(struct work_struct *data)
{
	PK_DBG("work queue callback\n");
	wd3610_disable();
}

static enum hrtimer_restart wd3610_timer_func(struct hrtimer *timer)
{
	schedule_work(&wd3610_work);
	return HRTIMER_NORESTART;
}


/******************************************************************************
 * Flashlight operations
 *****************************************************************************/
static int wd3610_ioctl(unsigned int cmd, unsigned long arg)
{
	struct flashlight_dev_arg *fl_arg;
	int channel;
	ktime_t ktime;
	unsigned int s;
	unsigned int ns;

	fl_arg = (struct flashlight_dev_arg *)arg;
	channel = fl_arg->channel;

	switch (cmd) {
	case FLASH_IOC_SET_TIME_OUT_TIME_MS:
		PK_DBG("FLASH_IOC_SET_TIME_OUT_TIME_MS(%d): %d\n",
				channel, (int)fl_arg->arg);
		wd3610_timeout_ms = fl_arg->arg;
		break;

	case FLASH_IOC_SET_DUTY:
		PK_DBG("FLASH_IOC_SET_DUTY(%d): %d\n",
				channel, (int)fl_arg->arg);
		wd3610_set_level(fl_arg->arg);
		break;

	case FLASH_IOC_SET_ONOFF:
		PK_DBG("FLASH_IOC_SET_ONOFF(%d): %d\n",
				channel, (int)fl_arg->arg);
		if (fl_arg->arg == 1) {
			if (wd3610_timeout_ms) {
				s = wd3610_timeout_ms / 1000;
				ns = wd3610_timeout_ms % 1000 * 1000000;
				ktime = ktime_set(s, ns);
				hrtimer_start(&wd3610_timer, ktime,
						HRTIMER_MODE_REL);
			}
			wd3610_enable();
		} else {
			wd3610_disable();
			hrtimer_cancel(&wd3610_timer);
		}
		break;
	default:
		PK_INF("No such command and arg(%d): (%d, %d)\n",
				channel, _IOC_NR(cmd), (int)fl_arg->arg);
		return -ENOTTY;
	}

	return 0;
}

static int wd3610_open(void)
{
	/* Move to set driver for saving power */
	return 0;
}

static int wd3610_release(void)
{
	/* Move to set driver for saving power */
	return 0;
}

static int wd3610_set_driver(int set)
{
	int ret = 0;

	/* set chip and usage count */
	mutex_lock(&wd3610_mutex);
	if (set) {
		if (!use_count)
			ret = wd3610_init();
		use_count++;
		PK_DBG("Set driver: %d\n", use_count);
	} else {
		use_count--;
		if (!use_count)
			ret = wd3610_uninit();
		if (use_count < 0)
			use_count = 0;
		PK_DBG("Unset driver: %d\n", use_count);
	}
	mutex_unlock(&wd3610_mutex);

	return ret;
}

static ssize_t wd3610_strobe_store(struct flashlight_arg arg)
{
	wd3610_set_driver(1);
	wd3610_set_level(arg.level);
	wd3610_timeout_ms = 0;
	wd3610_enable();
	msleep(arg.dur);
	wd3610_disable();
	wd3610_set_driver(0);

	return 0;
}

static struct flashlight_operations wd3610_ops = {
	wd3610_open,
	wd3610_release,
	wd3610_ioctl,
	wd3610_strobe_store,
	wd3610_set_driver
};


/******************************************************************************
 * Platform device and driver
 *****************************************************************************/
static int wd3610_chip_init(void)
{
	/* NOTE: Chip initialication move to "set driver" for power saving.
	 * wd3610_init();
	 */

	return 0;
}

static int wd3610_parse_dt(struct device *dev,
		struct wd3610_platform_data *pdata)
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
				wd3610_NAME);
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

static int wd3610_probe(struct platform_device *pdev)
{
	struct wd3610_platform_data *pdata = dev_get_platdata(&pdev->dev);
	int err;
	int i;

	PK_DBG("Probe start.\n");

	/* init pinctrl */
	if (wd3610_pinctrl_init(pdev)) {
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
		err = wd3610_parse_dt(&pdev->dev, pdata);
		if (err)
			goto err;
	}

	/* init work queue */
	INIT_WORK(&wd3610_work, wd3610_work_disable);

	/* init timer */
	hrtimer_init(&wd3610_timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	wd3610_timer.function = wd3610_timer_func;
	wd3610_timeout_ms = 100;

	/* init chip hw */
	wd3610_chip_init();

	/* clear usage count */
	use_count = 0;

	/* register flashlight device */
	if (pdata->channel_num) {
		for (i = 0; i < pdata->channel_num; i++)
			if (flashlight_dev_register_by_device_id(
						&pdata->dev_id[i],
						&wd3610_ops)) {
				err = -EFAULT;
				goto err;
			}
	} else {
		if (flashlight_dev_register(wd3610_NAME, &wd3610_ops)) {
			err = -EFAULT;
			goto err;
		}
	}

	PK_DBG("Probe done.\n");

	return 0;
err:
	return err;
}

static int wd3610_remove(struct platform_device *pdev)
{
	struct wd3610_platform_data *pdata = dev_get_platdata(&pdev->dev);
	int i;

	PK_DBG("Remove start.\n");

	pdev->dev.platform_data = NULL;

	/* unregister flashlight device */
	if (pdata && pdata->channel_num)
		for (i = 0; i < pdata->channel_num; i++)
			flashlight_dev_unregister_by_device_id(
					&pdata->dev_id[i]);
	else
		flashlight_dev_unregister(wd3610_NAME);

	/* flush work queue */
	flush_work(&wd3610_work);

	PK_DBG("Remove done.\n");

	return 0;
}

#ifdef CONFIG_OF
static const struct of_device_id wd3610_gpio_of_match[] = {
	{.compatible = wd3610_DTNAME},
	{},
};
MODULE_DEVICE_TABLE(of, wd3610_gpio_of_match);
#else
static struct platform_device wd3610_gpio_platform_device[] = {
	{
		.name = wd3610_NAME,
		.id = 0,
		.dev = {}
	},
	{}
};
MODULE_DEVICE_TABLE(platform, wd3610_gpio_platform_device);
#endif

static struct platform_driver wd3610_platform_driver = {
	.probe = wd3610_probe,
	.remove = wd3610_remove,
	.driver = {
		.name = wd3610_NAME,
		.owner = THIS_MODULE,
#ifdef CONFIG_OF
		.of_match_table = wd3610_gpio_of_match,
#endif
	},
};

static int __init flashlight_wd3610_init(void)
{
	int ret;

	PK_DBG("Init start.\n");

#ifndef CONFIG_OF
	ret = platform_device_register(&wd3610_gpio_platform_device);
	if (ret) {
		PK_ERR("Failed to register platform device\n");
		return ret;
	}
#endif

	ret = platform_driver_register(&wd3610_platform_driver);
	if (ret) {
		PK_ERR("Failed to register platform driver\n");
		return ret;
	}

	PK_DBG("Init done.\n");

	return 0;
}

static void __exit flashlight_wd3610_exit(void)
{
	PK_DBG("Exit start.\n");

	platform_driver_unregister(&wd3610_platform_driver);

	PK_DBG("Exit done.\n");
}

module_init(flashlight_wd3610_init);
module_exit(flashlight_wd3610_exit);

MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("Dongchun Zhu <dongchun.zhu@mediatek.com>");
MODULE_DESCRIPTION("MTK Flashlight wd3610 GPIO Driver");

