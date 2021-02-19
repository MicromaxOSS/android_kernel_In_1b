/*
* Copyright (C) 2016 MediaTek Inc.
*
* This program is free software; you can redistribute it and/or modify
* it under the terms of the GNU General Public License version 2 as
* published by the Free Software Foundation.
*
* This program is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
* See http://www.gnu.org/licenses/gpl-2.0.html for more details.
*/

#include <linux/init.h>		/* For init/exit macros */
#include <linux/module.h>	/* For MODULE_ marcros*/
#include <linux/fs.h>
#include <linux/device.h>
#include <linux/interrupt.h>
#include <linux/spinlock.h>
#include <linux/platform_device.h>
#include <linux/device.h>
#include <linux/kdev_t.h>
#include <linux/fs.h>
#include <linux/cdev.h>
#include <linux/delay.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/types.h>
#include <linux/wait.h>
#include <linux/slab.h>
#include <linux/fs.h>
#include <linux/sched.h>
#include <linux/poll.h>
#include <linux/power_supply.h>
#include <linux/time.h>
#include <linux/mutex.h>
#include <linux/kthread.h>
#include <linux/proc_fs.h>
#include <linux/platform_device.h>
#include <linux/seq_file.h>
#include <linux/scatterlist.h>
#include <linux/suspend.h>
#include <linux/version.h>
#include <linux/i2c.h>

#ifdef CONFIG_OF
#include <linux/of.h>
#include <linux/of_irq.h>
#include <linux/of_address.h>
#include <linux/of_device.h>
#endif
#include "upmu_common.h"
#include "psc5415e.h"
#include "mtk_charger_intf.h"

const unsigned int VBAT_CVTH[] = {
	3500000, 3520000, 3540000, 3560000,
	3580000, 3600000, 3620000, 3640000,
	3660000, 3680000, 3700000, 3720000,
	3740000, 3760000, 3780000, 3800000,
	3820000, 3840000, 3860000, 3880000,
	3900000, 3920000, 3940000, 3960000,
	3980000, 4000000, 4020000, 4040000,
	4060000, 4080000, 4100000, 4120000,
	4140000, 4160000, 4180000, 4200000,
	4220000, 4240000, 4260000, 4280000,
	4300000, 4320000, 4340000, 4360000,
	4380000, 4400000, 4420000, 4440000
};

const unsigned int CSTH[] = {
		800000, 1000000, 1200000, 1400000,
	1600000, 1800000, 2000000, 2600000
};
/*
const unsigned int CSTH[] = {
	425000, 785000, 946000, 1160000,
	1339000, 1464000, 1696000, 1875000
};
*/

/*psc5415e REG00 IINLIM[5:0]*/
const unsigned int INPUT_CSTH[] = {
	100000, 500000, 800000, 5000000
};

/* psc5415e REG0A BOOST_LIM[2:0], mA */
const unsigned int BOOST_CURRENT_LIMIT[] = {
	500, 750, 1200, 1400, 1650, 1875, 2150,
};

#ifdef CONFIG_OF
#else
#define PSC5415E_SLAVE_ADDR_WRITE	0xD4
#define PSC5415E_SLAVE_ADDR_Read	0xD5
#ifdef  I2C_SWITHING_CHARGER_CHANNEL
#define PSC5415E_BUSNUM I2C_SWITHING_CHARGER_CHANNEL
#else
#define PSC5415E_BUSNUM 1
#endif
#endif

static struct psc5415e_info {
	struct charger_device *chg_dev;
	struct power_supply *psy;
	struct charger_properties chg_props;
	struct device *dev;
	struct gtimer otg_kthread_gtimer;
	struct workqueue_struct *otg_boost_workq;
	struct work_struct kick_work;
	unsigned int polling_interval;
	bool polling_enabled;
	const char *chg_dev_name;
	const char *eint_name;
	enum charger_type chg_type;
	int irq;
} *g_psc5415e_info;

static struct pinctrl *g_pinctrl;
//static struct pinctrl_state *g_pins_default;
static struct pinctrl_state *g_psc_chg_en_low;
static struct pinctrl_state *g_psc_chg_en_high;
static const char *g_chg_dev_name = "primary_chg";
static const char *g_alias_name = "psc5415e";
static struct i2c_client *new_client;
static const struct i2c_device_id psc5415e_i2c_id[] = { {"psc5415e", 1}, {} };

static void enable_boost_polling(bool poll_en);
static void usbotg_boost_kick_work(struct work_struct *work);
static int usbotg_gtimer_func(struct gtimer *data);

unsigned int charging_value_to_parameter(const unsigned int *parameter, const unsigned int array_size,
					const unsigned int val)
{
	if (val < array_size)
		return parameter[val];
	pr_err("Can't find the parameter\n");
	return parameter[0];
}

unsigned int charging_parameter_to_value(const unsigned int *parameter, const unsigned int array_size,
					const unsigned int val)
{
	unsigned int i;

	pr_err_ratelimited("array_size = %d\n", array_size);

	for (i = 0; i < array_size; i++) {
		if (val == *(parameter + i))
			return i;
	}

	pr_err("NO register value match\n");
	/* TODO: ASSERT(0);	// not find the value */
	return 0;
}

static unsigned int bmt_find_closest_level(const unsigned int *pList, unsigned int number,
					 unsigned int level)
{
	unsigned int i;
	unsigned int max_value_in_last_element;

	if (pList[0] < pList[1])
		max_value_in_last_element = 1;
	else
		max_value_in_last_element = 0;

	if (max_value_in_last_element == 1) {
		for (i = (number - 1); i != 0; i--) {	/* max value in the last element */
			if (pList[i] <= level) {
				pr_err_ratelimited("zzf_%d<=%d, i=%d\n", pList[i], level, i);
				return pList[i];
			}
		}
		pr_err("Can't find closest level\n");
		return pList[0];
		/* return 000; */
	} else {
		for (i = 0; i < number; i++) {	/* max value in the first element */
			if (pList[i] <= level)
				return pList[i];
		}
		pr_err("Can't find closest level\n");
		return pList[number - 1];
		/* return 000; */
	}
}

unsigned char psc5415e_reg[PSC5415E_REG_NUM] = { 0 };
static DEFINE_MUTEX(psc5415e_i2c_access);
static DEFINE_MUTEX(psc5415e_access_lock);

static int psc5415e_read_byte(u8 reg_addr, u8 *rd_buf, int rd_len)
{
	int ret = 0;
	struct i2c_adapter *adap = new_client->adapter;
	struct i2c_msg msg[2];
	u8 *w_buf = NULL;
	u8 *r_buf = NULL;

	memset(msg, 0, 2 * sizeof(struct i2c_msg));

	w_buf = kzalloc(1, GFP_KERNEL);
	if (w_buf == NULL)
		return -1;
	r_buf = kzalloc(rd_len, GFP_KERNEL);
	if (r_buf == NULL)
		return -1;

	*w_buf = reg_addr;

	msg[0].addr = new_client->addr;
	msg[0].flags = 0;
	msg[0].len = 1;
	msg[0].buf = w_buf;

	msg[1].addr = new_client->addr;
	msg[1].flags = 1;
	msg[1].len = rd_len;
	msg[1].buf = r_buf;

	ret = i2c_transfer(adap, msg, 2);

	memcpy(rd_buf, r_buf, rd_len);

	kfree(w_buf);
	kfree(r_buf);
	return ret;
}

int psc5415e_write_byte(unsigned char reg_num, u8 *wr_buf, int wr_len)
{
	int ret = 0;
	struct i2c_adapter *adap = new_client->adapter;
	struct i2c_msg msg;
	u8 *w_buf = NULL;

	memset(&msg, 0, sizeof(struct i2c_msg));

	w_buf = kzalloc(wr_len, GFP_KERNEL);
	if (w_buf == NULL)
		return -1;

	w_buf[0] = reg_num;
	memcpy(w_buf + 1, wr_buf, wr_len);

	msg.addr = new_client->addr;
	msg.flags = 0;
	msg.len = wr_len;
	msg.buf = w_buf;

	ret = i2c_transfer(adap, &msg, 1);

	kfree(w_buf);
	return ret;
}

unsigned int psc5415e_read_interface(unsigned char reg_num, unsigned char *val, unsigned char MASK,
				unsigned char SHIFT)
{
	unsigned char psc5415e_reg = 0;
	unsigned int ret = 0;

	ret = psc5415e_read_byte(reg_num, &psc5415e_reg, 1);
	pr_err_ratelimited("[psc5415e_read_interface] Reg[%x]=0x%x\n", reg_num, psc5415e_reg);
	psc5415e_reg &= (MASK << SHIFT);
	*val = (psc5415e_reg >> SHIFT);
	pr_err_ratelimited("[psc5415e_read_interface] val=0x%x\n", *val);

	return ret;
}

unsigned int psc5415e_config_interface(unsigned char reg_num, unsigned char val, unsigned char MASK,
					unsigned char SHIFT)
{
	unsigned char psc5415e_reg = 0;
	unsigned char psc5415e_reg_ori = 0;
	unsigned int ret = 0;

	mutex_lock(&psc5415e_access_lock);
	ret = psc5415e_read_byte(reg_num, &psc5415e_reg, 1);
	
	if(ret<0)
		return ret;
	
	psc5415e_reg_ori = psc5415e_reg;
	psc5415e_reg &= ~(MASK << SHIFT);
	psc5415e_reg |= (val << SHIFT);
	if (reg_num == PSC5415E_CON4)
		psc5415e_reg &= ~(1 << CON4_RESET_SHIFT);

	ret = psc5415e_write_byte(reg_num, &psc5415e_reg, 2);
	mutex_unlock(&psc5415e_access_lock);
	pr_err_ratelimited("[psc5415e_config_interface] write Reg[%x]=0x%x from 0x%x\n", reg_num,
			psc5415e_reg, psc5415e_reg_ori);
	/* Check */
	/* psc5415e_read_byte(reg_num, &psc5415e_reg, 1); */
	/* printk("[psc5415e_config_interface] Check Reg[%x]=0x%x\n", reg_num, psc5415e_reg); */

	return ret;
}

/* write one register directly */
unsigned int psc5415e_reg_config_interface(unsigned char reg_num, unsigned char val)
{
	unsigned char psc5415e_reg = val;

	return psc5415e_write_byte(reg_num, &psc5415e_reg, 2);
}

void psc5415e_set_tmr_rst(unsigned int val)
{
	psc5415e_config_interface((unsigned char)(PSC5415E_CON0),
				(unsigned char)(val),
				(unsigned char)(CON0_TMR_RST_MASK),
				(unsigned char)(CON0_TMR_RST_SHIFT)
				);
}

unsigned int psc5415e_get_otg_status(void)
{
	unsigned char val = 0;

	psc5415e_read_interface((unsigned char)(PSC5415E_CON0),
				(unsigned char *)(&val),
				(unsigned char)(CON0_OTG_MASK),
				(unsigned char)(CON0_OTG_SHIFT)
				);
	return val;
}

void psc5415e_set_en_stat(unsigned int val)
{
	psc5415e_config_interface((unsigned char)(PSC5415E_CON0),
				(unsigned char)(val),
				(unsigned char)(CON0_EN_STAT_MASK),
				(unsigned char)(CON0_EN_STAT_SHIFT)
				);
}

unsigned int psc5415e_get_chip_status(void)
{
	unsigned char val = 0;

	psc5415e_read_interface((unsigned char)(PSC5415E_CON0),
				(unsigned char *)(&val),
				(unsigned char)(CON0_STAT_MASK),
				(unsigned char)(CON0_STAT_SHIFT)
				);
	return val;
}

unsigned int psc5415e_get_boost_status(void)
{
	unsigned char val = 0;

	psc5415e_read_interface((unsigned char)(PSC5415E_CON0),
				(unsigned char *)(&val),
				(unsigned char)(CON0_BOOST_MASK),
				(unsigned char)(CON0_BOOST_SHIFT)
				);
	return val;

}

unsigned int psc5415e_get_fault_status(void)
{
	unsigned char val = 0;

	psc5415e_read_interface((unsigned char)(PSC5415E_CON0),
				(unsigned char *)(&val),
				(unsigned char)(CON0_FAULT_MASK),
				(unsigned char)(CON0_FAULT_SHIFT)
				);
	return val;
}

void psc5415e_set_input_charging_current(unsigned int val)
{
	psc5415e_config_interface((unsigned char)(PSC5415E_CON1),
				(unsigned char)(val),
				(unsigned char)(CON1_LIN_LIMIT_MASK),
				(unsigned char)(CON1_LIN_LIMIT_SHIFT)
				);
}

unsigned int psc5415e_get_input_charging_current(void)
{
	unsigned char val = 0;

	psc5415e_read_interface((unsigned char)(PSC5415E_CON1),
				(unsigned char *)(&val),
				(unsigned char)(CON1_LIN_LIMIT_MASK),
				(unsigned char)(CON1_LIN_LIMIT_SHIFT)
				);

	return val;
}

void psc5415e_set_v_low(unsigned int val)
{

	psc5415e_config_interface((unsigned char)(PSC5415E_CON1),
				(unsigned char)(val),
				(unsigned char)(CON1_LOW_V_MASK),
				(unsigned char)(CON1_LOW_V_SHIFT)
				);
}

void psc5415e_set_te(unsigned int val)
{
	psc5415e_config_interface((unsigned char)(PSC5415E_CON1),
				(unsigned char)(val),
				(unsigned char)(CON1_TE_MASK),
				(unsigned char)(CON1_TE_SHIFT)
				);
}

void psc5415e_set_ce(unsigned int val)
{
	psc5415e_config_interface((unsigned char)(PSC5415E_CON1),
				(unsigned char)(val),
				(unsigned char)(CON1_CE_MASK),
				(unsigned char)(CON1_CE_SHIFT)
				);
}

void psc5415e_set_hz_mode(unsigned int val)
{
	psc5415e_config_interface((unsigned char)(PSC5415E_CON1),
				(unsigned char)(val),
				(unsigned char)(CON1_HZ_MODE_MASK),
				(unsigned char)(CON1_HZ_MODE_SHIFT)
				);
}

void psc5415e_set_opa_mode(unsigned int val)
{
	psc5415e_config_interface((unsigned char)(PSC5415E_CON1),
				(unsigned char)(val),
				(unsigned char)(CON1_OPA_MODE_MASK),
				(unsigned char)(CON1_OPA_MODE_SHIFT)
				);
}

void psc5415e_set_oreg(unsigned int val)
{
	psc5415e_config_interface((unsigned char)(PSC5415E_CON2),
				(unsigned char)(val),
				(unsigned char)(CON2_OREG_MASK),
				(unsigned char)(CON2_OREG_SHIFT)
				);
}
void psc5415e_set_otg_pl(unsigned int val)
{
	psc5415e_config_interface((unsigned char)(PSC5415E_CON2),
				(unsigned char)(val),
				(unsigned char)(CON2_OTG_PL_MASK),
				(unsigned char)(CON2_OTG_PL_SHIFT)
				);
}
void psc5415e_set_otg_en(unsigned int val)
{
	psc5415e_config_interface((unsigned char)(PSC5415E_CON2),
				(unsigned char)(val),
				(unsigned char)(CON2_OTG_EN_MASK),
				(unsigned char)(CON2_OTG_EN_SHIFT)
				);
}

unsigned int psc5415e_get_vender_code(void)
{
	unsigned char val = 0;

	psc5415e_read_interface((unsigned char)(PSC5415E_CON3),
				(unsigned char *)(&val),
				(unsigned char)(CON3_VENDER_CODE_MASK),
				(unsigned char)(CON3_VENDER_CODE_SHIFT)
				);
	return val;
}
unsigned int psc5415e_get_pn(void)
{
	unsigned char val = 0;

	psc5415e_read_interface((unsigned char)(PSC5415E_CON3),
				(unsigned char *)(&val),
				(unsigned char)(CON3_PIN_MASK),
				(unsigned char)(CON3_PIN_SHIFT)
				);
	return val;
}

unsigned int psc5415e_get_revision(void)
{
	unsigned char val = 0;

	psc5415e_read_interface((unsigned char)(PSC5415E_CON3),
				(unsigned char *)(&val),
				(unsigned char)(CON3_REVISION_MASK),
				(unsigned char)(CON3_REVISION_SHIFT)
				);
	return val;
}

void psc5415e_set_reset(unsigned int val)
{
	psc5415e_config_interface((unsigned char)(PSC5415E_CON4),
				(unsigned char)(val),
				(unsigned char)(CON4_RESET_MASK),
				(unsigned char)(CON4_RESET_SHIFT)
				);
}

void psc5415e_set_iocharge(unsigned int val)
{
	psc5415e_config_interface((unsigned char)(PSC5415E_CON4),
				(unsigned char)(val),
				(unsigned char)(CON4_I_CHR_MASK),
				(unsigned char)(CON4_I_CHR_SHIFT)
				);
}

void psc5415e_set_iterm(unsigned int val)
{
	psc5415e_config_interface((unsigned char)(PSC5415E_CON4),
				(unsigned char)(val),
				(unsigned char)(CON4_I_TERM_MASK),
				(unsigned char)(CON4_I_TERM_SHIFT)
				);
}

void psc5415e_set_dis_vreg(unsigned int val)
{
	psc5415e_config_interface((unsigned char)(PSC5415E_CON5),
				(unsigned char)(val),
				(unsigned char)(CON5_DIS_VREG_MASK),
				(unsigned char)(CON5_DIS_VREG_SHIFT)
				);
}

void psc5415e_set_io_level(unsigned int val)
{
	psc5415e_config_interface((unsigned char)(PSC5415E_CON5),
				(unsigned char)(val),
				(unsigned char)(CON5_IO_LEVEL_MASK),
				(unsigned char)(CON5_IO_LEVEL_SHIFT)
				);
}

unsigned int psc5415e_get_sp_status(void)
{
	unsigned char val = 0;

	psc5415e_read_interface((unsigned char)(PSC5415E_CON5),
				(unsigned char *)(&val),
				(unsigned char)(CON5_SP_STATUS_MASK),
				(unsigned char)(CON5_SP_STATUS_SHIFT)
				);
	return val;
}

unsigned int psc5415e_get_en_level(void)
{
	unsigned char val = 0;

	psc5415e_read_interface((unsigned char)(PSC5415E_CON5),
				(unsigned char *)(&val),
				(unsigned char)(CON5_EN_LEVEL_MASK),
				(unsigned char)(CON5_EN_LEVEL_SHIFT)
				);
	return val;
}

void psc5415e_set_vsp(unsigned int val)
{
	psc5415e_config_interface((unsigned char)(PSC5415E_CON5),
				(unsigned char)(val),
				(unsigned char)(CON5_VSP_MASK),
				(unsigned char)(CON5_VSP_SHIFT)
				);
}

void psc5415e_set_i_safe(unsigned int val)
{
	psc5415e_config_interface((unsigned char)(PSC5415E_CON6),
				(unsigned char)(val),
				(unsigned char)(CON6_ISAFE_MASK),
				(unsigned char)(CON6_ISAFE_SHIFT)
				);
}

void psc5415e_set_v_safe(unsigned int val)
{
	psc5415e_config_interface((unsigned char)(PSC5415E_CON6),
				(unsigned char)(val),
				(unsigned char)(CON6_VSAFE_MASK),
				(unsigned char)(CON6_VSAFE_SHIFT)
				);
}

static int psc5415e_dump_register(struct charger_device *chg_dev)
{
	int i;

	for (i = 0; i < PSC5415E_REG_NUM; i++) {
		psc5415e_read_byte(i, &psc5415e_reg[i], 1);
		pr_err("psc5415e cz [0x%x]=0x%x ", i, psc5415e_reg[i]);
	}
	pr_err("\n");

	return 0;
}

static int psc5415e_do_event(struct charger_device *chg_dev, unsigned int event, unsigned int args)
{
	if (chg_dev == NULL)
		return -EINVAL;

	pr_err("%s: event = %d\n", __func__, event);

	switch (event) {
	case EVENT_EOC:
		charger_dev_notify(chg_dev, CHARGER_DEV_NOTIFY_EOC);
		break;
	case EVENT_RECHARGE:
		charger_dev_notify(chg_dev, CHARGER_DEV_NOTIFY_RECHG);
		break;
	default:
		break;
	}

	return 0;
}

static int psc5415e_enable_charging(struct charger_device *chg_dev, bool en)
{
	unsigned int status = 0;

	pr_err("psc5415e_enable_charging en:%d\n", en);
	if (true == en) {
		psc5415e_set_ce(0);
		pinctrl_select_state(g_pinctrl, g_psc_chg_en_low);
		pr_err("psc5415e calm enable charging\n");
	} else {
		//psc5415e_set_ce(1);
		//disable charging: psc_chg_en_high
		pinctrl_select_state(g_pinctrl, g_psc_chg_en_high);
		pr_err("psc5415e calm disable charging\n");
	}

	return status;
}

static int psc5415e_set_cv_voltage(struct charger_device *chg_dev, u32 cv)
{
	int status = 0;
	unsigned short int array_size;
	unsigned int set_cv_voltage;
	unsigned short int register_value;
	/*static kal_int16 pre_register_value; */
	array_size = ARRAY_SIZE(VBAT_CVTH);
	/*pre_register_value = -1; */
	set_cv_voltage = bmt_find_closest_level(VBAT_CVTH, array_size, cv);

	register_value =
	charging_parameter_to_value(VBAT_CVTH, array_size, set_cv_voltage);
	pr_err("charging_set_cv_voltage register_value=0x%x %d %d\n",
	 register_value, cv, set_cv_voltage);
	psc5415e_set_oreg(register_value);

	return status;
}

static int psc5415e_get_current(struct charger_device *chg_dev, u32 *ichg)
{
	int status = 0;
	unsigned int array_size;
	unsigned char reg_value;

	array_size = ARRAY_SIZE(CSTH);
	psc5415e_read_interface(0x1, &reg_value, 0x3, 0x6);	/* IINLIM */
	*ichg = charging_value_to_parameter(CSTH, array_size, reg_value);

	return status;
}

static int psc5415e_set_current(struct charger_device *chg_dev, u32 current_value)
{
	unsigned int status = 0;
	unsigned int set_chr_current;
	unsigned int array_size;
	unsigned int register_value;

	if (current_value <= 350000) {
		psc5415e_set_io_level(1);
	} else {
		psc5415e_set_io_level(0);
		array_size = ARRAY_SIZE(CSTH);
		set_chr_current = bmt_find_closest_level(CSTH, array_size, current_value);
		register_value = charging_parameter_to_value(CSTH, array_size, set_chr_current);
		psc5415e_set_iocharge(register_value);
	}
      psc5415e_reg_config_interface(0x51, 0x01);
      psc5415e_set_te(1);
	return status;
}


static int psc5415e_get_input_current(struct charger_device *chg_dev, u32 *aicr)
{
	unsigned int status = 0;
	unsigned int array_size;
	unsigned int register_value;

	array_size = ARRAY_SIZE(INPUT_CSTH);
	register_value = psc5415e_get_input_charging_current();
	*aicr = charging_parameter_to_value(INPUT_CSTH, array_size, register_value);

	return status;
}

static int psc5415e_set_input_current(struct charger_device *chg_dev, u32 current_value)
{
	unsigned int status = 0;
	unsigned int set_chr_current;
	unsigned int array_size;
	unsigned int register_value;

	if (current_value > 500000) {
		register_value = 0x3;
	} else {
		array_size = ARRAY_SIZE(INPUT_CSTH);
		set_chr_current = bmt_find_closest_level(INPUT_CSTH, array_size, current_value);
		register_value =
	 charging_parameter_to_value(INPUT_CSTH, array_size, set_chr_current);
	}

   if(register_value == 0x0)
   register_value = 0x3;
	psc5415e_set_input_charging_current(register_value);

	return status;
}

static int psc5415e_get_charging_status(struct charger_device *chg_dev, bool *is_done)
{
	unsigned int status = 0;
	unsigned int ret_val;

	ret_val = psc5415e_get_chip_status();

	if (ret_val == 0x2)
		*is_done = true;
	else
		*is_done = false;

	return status;
}

static int psc5415e_reset_watch_dog_timer(struct charger_device *chg_dev)
{
	psc5415e_set_tmr_rst(1);
	return 0;
}

static int psc5415e_charger_enable_otg(struct charger_device *chg_dev, bool en)
{
	if(en)
		psc5415e_enable_charging(chg_dev,1);

	psc5415e_set_opa_mode(en);
	enable_boost_polling(en);
	return 0;
}

static void enable_boost_polling(bool poll_en)
{
	if (g_psc5415e_info) {
		if (poll_en) {
			gtimer_start(&g_psc5415e_info->otg_kthread_gtimer,
				     g_psc5415e_info->polling_interval);
			g_psc5415e_info->polling_enabled = true;
		} else {
			g_psc5415e_info->polling_enabled = false;
			gtimer_stop(&g_psc5415e_info->otg_kthread_gtimer);
		}
	}
}

static void usbotg_boost_kick_work(struct work_struct *work)
{

	struct psc5415e_info *boost_manager =
		container_of(work, struct psc5415e_info, kick_work);

	pr_debug_ratelimited("usbotg_boost_kick_work\n");

	psc5415e_set_tmr_rst(1);

	if (boost_manager->polling_enabled == true)
		gtimer_start(&boost_manager->otg_kthread_gtimer,
			     boost_manager->polling_interval);
}

static int usbotg_gtimer_func(struct gtimer *data)
{
	struct psc5415e_info *boost_manager =
		container_of(data, struct psc5415e_info,
			     otg_kthread_gtimer);

	queue_work(boost_manager->otg_boost_workq,
		   &boost_manager->kick_work);

	return 0;
}

void psc5415e_charger_enable_otg_ext(bool en)
{
	pr_err("psc5415e_charger_enable_otg is called by tcpc:%d\n",en);
	if(en)
	{
		psc5415e_set_ce(0);
		pinctrl_select_state(g_pinctrl, g_psc_chg_en_low);
	}

	psc5415e_set_opa_mode(en);
	enable_boost_polling(en);
}

static struct charger_ops psc5415e_chg_ops = {

	/* Normal charging */
	.dump_registers = psc5415e_dump_register,
	.enable = psc5415e_enable_charging,
	.get_charging_current = psc5415e_get_current,
	.set_charging_current = psc5415e_set_current,
	.get_input_current = psc5415e_get_input_current,
	.set_input_current = psc5415e_set_input_current,
	/*.get_constant_voltage = psc5415e_get_battery_voreg,*/
	.set_constant_voltage = psc5415e_set_cv_voltage,
	.kick_wdt = psc5415e_reset_watch_dog_timer,
	.is_charging_done = psc5415e_get_charging_status,
	.enable_otg = psc5415e_charger_enable_otg,
	.event = psc5415e_do_event,
};

static int psc5415e_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	int ret = 0;
	int count = 0;
	struct psc5415e_info *info = NULL;

	pr_err("%s entry\n", __func__);
	info = devm_kzalloc(&client->dev, sizeof(struct psc5415e_info), GFP_KERNEL);

	if (!info)
		return -ENOMEM;

	client->addr = 0x6A;
	new_client = client;
	info->dev = &client->dev;

	if (ret < 0)
		return ret;

	/* Register charger device */
	info->chg_dev_name = g_chg_dev_name;
	info->chg_props.alias_name = "psc5415e";
	pr_err("%s: psc5415e_i2c_probe g_dev_name:%s\n", __func__, info->chg_dev_name);
	info->chg_dev = charger_device_register(info->chg_dev_name,
		&client->dev, info, &psc5415e_chg_ops, &info->chg_props);

	if (IS_ERR_OR_NULL(info->chg_dev)) {
		pr_err("%s: register charger device failed\n", __func__);
		ret = PTR_ERR(info->chg_dev);
		return ret;
	}

	while(ret != 2 && count < 5)
	{
		ret = psc5415e_get_vender_code();
		count++;
		if (ret != 7) {
			pr_err("%s: psc5415e_get_vender_code failed %d\n", __func__,count);
			}
		else
			break;
	}
	
	if(count>5)
	{
		return -ENODEV;
		pr_err("%s: get vendor id failed\n", __func__);
	}


	psc5415e_reg_config_interface(0x51, 0x01);
	psc5415e_reg_config_interface(0x00, 0xC0);	/* kick chip watch dog */
	psc5415e_reg_config_interface(0x01, 0xb8);	/* TE=1, CE=0, HZ_MODE=0, OPA_MODE=0 */
	psc5415e_reg_config_interface(0x05, 0x02);
	psc5415e_reg_config_interface(0x04, 0x11);	/* 100mA */
	psc5415e_reg_config_interface(0x01, 0xB0);
	psc5415e_dump_register(info->chg_dev);
	
	gtimer_init(&info->otg_kthread_gtimer, info->dev, "otg_boost");
	info->otg_kthread_gtimer.callback = usbotg_gtimer_func;

	info->otg_boost_workq = create_singlethread_workqueue("otg_boost_workq");
	INIT_WORK(&info->kick_work, usbotg_boost_kick_work);
	info->polling_interval = 20;
	g_psc5415e_info = info;

	return 0;
}

static int psc5415e_i2c_remove(struct i2c_client *client)
{
	psc5415e_set_reset(1);
	return 0;
}

static void psc5415e_shutdown(struct i2c_client *i2c)
{
	psc5415e_set_reset(1);
	pr_info("%s\n", __func__);
}

static const struct of_device_id psc5415e_of_match[] = {
	{.compatible = "mediatek,switching_charger"},
	{},
};

const struct of_device_id psc_of_match[] = {
	{ .compatible = "mediatek,psc5415e", },
	{},
};

static struct i2c_driver psc5415e_driver = {
	.driver = {
		.name = "psc5415e",
		.owner = THIS_MODULE,
#ifdef CONFIG_OF
		.of_match_table = psc5415e_of_match,
#endif
		},
	.probe = psc5415e_i2c_probe,
	.remove = psc5415e_i2c_remove,
	.shutdown = psc5415e_shutdown,
	.id_table = psc5415e_i2c_id,
};

static int psc_probe(struct platform_device *pdev)
{
	int ret = 0;
	struct device_node *node1 = NULL;

	pr_err("entry psc_probe.\n");
	g_pinctrl = devm_pinctrl_get(&pdev->dev);
	if (IS_ERR(g_pinctrl)) {
		ret = PTR_ERR(g_pinctrl);
		pr_err("fwq Cannot find g_pinctrl!\n");
		return ret;
	}
	/*
	g_pins_default = pinctrl_lookup_state(g_pinctrl, "default");
	if (IS_ERR(g_pins_default)) {
		ret = PTR_ERR(g_pins_default);
		pr_err("Cannot find pinctrl default %d!\n", ret);
		return ret;
	}*/
	g_psc_chg_en_low = pinctrl_lookup_state(g_pinctrl, "psc_chg_en_low");
	if (IS_ERR(g_psc_chg_en_low)) {
		ret = PTR_ERR(g_psc_chg_en_low);
		pr_err("Cannot find pinctrl g_psc_chg_en_low!\n");
		return ret;
	}
	g_psc_chg_en_high = pinctrl_lookup_state(g_pinctrl, "psc_chg_en_high");
	if (IS_ERR(g_psc_chg_en_high)) {
		ret = PTR_ERR(g_psc_chg_en_high);
		pr_err("Cannot find pinctrl psc_chg_en_high!\n");
		return ret;
	}

	node1 = of_find_matching_node(node1, psc_of_match);
	if (node1) {
		if (of_property_read_string(node1, "charger_name", &g_chg_dev_name) < 0) {
			g_chg_dev_name = "primary_chg";
			pr_err("%s: no charger name\n", __func__);
		}

		if (of_property_read_string(node1, "alias_name", &g_alias_name) < 0) {
			g_alias_name = "psc5415e";
			pr_err("%s: no alias name\n", __func__);
		}
	}
	
	return 0;
}

static int psc_remove(struct platform_device *pdev)
{
	return 0;
}

static struct platform_driver psc_driver = {
	.remove = psc_remove,
	.shutdown = NULL,
	.probe = psc_probe,
	.driver = {
			.name = "psc5415e",
			.owner = THIS_MODULE,
			.of_match_table = psc_of_match,
	},
};

static int __init psc5415e_init(void)
{
	if (platform_driver_register(&psc_driver) != 0)
		pr_err("unable to psc5415e driver.\n");

	pr_err("%s entry.\n",__func__);
	if (i2c_add_driver(&psc5415e_driver) != 0)
		pr_err("Failed to register psc5415e i2c driver.\n");
	else
		pr_info("Success to register psc5415e i2c driver.\n");

	return 0;
}

static void __exit psc5415e_exit(void)
{
	psc5415e_set_reset(1);
	i2c_del_driver(&psc5415e_driver);
}

module_init(psc5415e_init);
module_exit(psc5415e_exit);
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("I2C psc5415e Driver");
MODULE_AUTHOR("Henry Chen<henryc.chen@mediatek.com>");
