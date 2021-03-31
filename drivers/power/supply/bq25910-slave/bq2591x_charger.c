/*
 * BQ2589x battery charging driver
 *
 * Copyright (C) 2013 Texas Instruments
 *
 * This package is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.

 * THIS PACKAGE IS PROVIDED ``AS IS'' AND WITHOUT ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, WITHOUT LIMITATION, THE IMPLIED
 * WARRANTIES OF MERCHANTIBILITY AND FITNESS FOR A PARTICULAR PURPOSE.
 */
#define pr_fmt(fmt) KBUILD_MODNAME ": " fmt

#include <linux/gpio.h>
#include <linux/i2c.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/power_supply.h>
#include <linux/slab.h>
#include <linux/kernel.h>
#include <linux/sched.h>
#include <linux/kthread.h>
#include <linux/delay.h>
#include <linux/of.h>
#include <linux/of_gpio.h>
#include <linux/of_device.h>
#include <linux/debugfs.h>
#include <linux/bitops.h>
#include "bq25910_reg.h"

enum bq2591x_part_no {
	BQ25910 = 0x01,
};

enum reason {
	USER	= BIT(0),
	THERMAL = BIT(1),
	CURRENT = BIT(2),
	SOC	= BIT(3),
};

#define  SUSPEND_CURRENT_MA 2

struct bq2591x_config {
	int	chg_mv;
	int	chg_ma;

	int	ivl_mv;
	int	icl_ma;

	int	iterm_ma;
	int	batlow_mv;

	bool	enable_term;
};

struct bq2591x {
	struct device		*dev;
	struct i2c_client	*client;
	enum bq2591x_part_no	part_no;
	int			revision;

	struct bq2591x_config	cfg;
	struct delayed_work	monitor_work;
	struct delayed_work	icl_softstart_work;
	struct delayed_work	fcc_softstart_work;

	bool			iindpm;
	bool			vindpm;

	bool			in_therm_regulation;

	int			chg_mv;
	int			chg_ma;
	int			ivl_mv;
	int			icl_ma;

	int			vfloat_mv;
	int			usb_psy_ma;
	int			fast_cc_ma;

	int			charge_state;
	int			fault_status;

	int			prev_stat_flag;
	int			prev_fault_flag;

	int			c_health;
	int			max_fcc;

	int			reg_stat;
	int			reg_fault;
	int			reg_stat_flag;
	int			reg_fault_flag;

	struct mutex		i2c_rw_lock;

	struct power_supply	*usb_psy;
	struct power_supply	*parallel_psy;
	struct power_supply_desc parallel_psy_d;

	struct dentry		*debug_root;
	int			skip_reads;
	int			skip_writes;
};

static int __bq2591x_read_reg(struct bq2591x *bq, u8 reg, u8 *data)
{
	s32 ret;

	pm_stay_awake(bq->dev);

	ret = i2c_smbus_read_byte_data(bq->client, reg);
	if (ret < 0) {
		dev_err(bq->dev, "i2c: failed to read from reg 0x%02X\n",
			reg);
		pm_relax(bq->dev);
		return ret;
	}

	*data = (u8)ret;

	pm_relax(bq->dev);

	return 0;
}

static int __bq2591x_write_reg(struct bq2591x *bq, int reg, u8 val)
{
	s32 ret;

	pm_stay_awake(bq->dev);

	ret = i2c_smbus_write_byte_data(bq->client, reg, val);
	if (ret < 0) {
		dev_err(bq->dev,
			"i2c: failed to write 0x%02X to reg 0x%02X: %d\n",
		       val, reg, ret);
		pm_relax(bq->dev);
		return ret;
	}

	pm_relax(bq->dev);

	return 0;
}

static int bq2591x_read_byte(struct bq2591x *bq, u8 *data, u8 reg)
{
	int ret;

	if (bq->skip_reads) {
		*data = 0;
		return 0;
	}

	mutex_lock(&bq->i2c_rw_lock);
	ret = __bq2591x_read_reg(bq, reg, data);
	mutex_unlock(&bq->i2c_rw_lock);

	return ret;
}

static int bq2591x_write_byte(struct bq2591x *bq, u8 reg, u8 data)
{
	int ret;

	if (bq->skip_writes)
		return 0;

	mutex_lock(&bq->i2c_rw_lock);
	ret = __bq2591x_write_reg(bq, reg, data);
	mutex_unlock(&bq->i2c_rw_lock);

	return ret;
}

static int bq2591x_update_reg(struct bq2591x *bq, u8 reg, u8 mask, u8 data)
{
	int ret;
	u8 val;

	if (bq->skip_reads || bq->skip_writes)
		return 0;

	mutex_lock(&bq->i2c_rw_lock);

	ret = __bq2591x_read_reg(bq, reg, &val);
	if (ret)
		goto out;

	val &= ~mask;
	val |= data & mask;

	ret = __bq2591x_write_reg(bq, reg, val);

out:
	mutex_unlock(&bq->i2c_rw_lock);

	return ret;
}

#define _BQ2591X_READ_REG_M(BQ, REG_NUM, REG_NAME, OUT, RET, VAL) ({	\
	int RET;							\
	u8 VAL;								\
	RET = bq2591x_read_byte(BQ, &VAL, REG_NUM);			\
	if (RET < 0)							\
		dev_err((BQ)->dev, "failed to read %s register "	\
				   "(rc=%d)\n", #REG_NAME, RET);	\
	else								\
		(OUT) = (VAL & BQ2591X_##REG_NAME##_MASK);		\
	RET; })

#define BQ2591X_READ_REG_M(BQ, REG_NUM, REG_NAME, OUT)			\
	_BQ2591X_READ_REG_M(BQ, REG_NUM, REG_NAME, OUT,			\
			    __UNIQUE_ID(ret), __UNIQUE_ID(val))

#define _BQ2591X_READ_REG_MS(BQ, REG_NUM, REG_NAME, OUT, RET, VAL) ({	\
	int RET, VAL;							\
	RET = BQ2591X_READ_REG_M(BQ, REG_NUM, REG_NAME, VAL);		\
	if (!RET)							\
		(OUT) = VAL >> BQ2591X_##REG_NAME##_SHIFT;		\
	RET; })

#define BQ2591X_READ_REG_MS(BQ, REG_NUM, REG_NAME, OUT)			\
	_BQ2591X_READ_REG_MS(BQ, REG_NUM, REG_NAME, OUT,		\
			     __UNIQUE_ID(ret), __UNIQUE_ID(val))

#define _BQ2591X_READ_REG_MSLB(BQ, REG_NUM, REG_NAME, OUT, RET, VAL) ({	\
	int RET, VAL;							\
	RET = BQ2591X_READ_REG_MS(BQ, REG_NUM, REG_NAME, VAL);		\
	if (!RET) {							\
		VAL *= BQ2591X_##REG_NAME##_LSB;			\
		VAL += BQ2591X_##REG_NAME##_BASE;			\
		(OUT) = VAL;						\
	}								\
	RET; })

#define BQ2591X_READ_REG_MSLB(BQ, REG_NUM, REG_NAME, OUT)		\
	_BQ2591X_READ_REG_MSLB(BQ, REG_NUM, REG_NAME, OUT,		\
			       __UNIQUE_ID(ret), __UNIQUE_ID(val))

#define _BQ2591X_UPDATE_REG_S(BQ, REG_NUM, REG_NAME, IN, RET, VAL) ({	\
	u8 VAL = (IN) << BQ2591X_##REG_NAME##_SHIFT;			\
	int RET;							\
	RET = bq2591x_update_reg(BQ, REG_NUM,				\
				 BQ2591X_##REG_NAME##_MASK, VAL);	\
	if (RET < 0)							\
		dev_err((BQ)->dev, "failed to update %s register "	\
				   "(rc=%d)\n", #REG_NAME, RET);	\
	RET; })

#define BQ2591X_UPDATE_REG_S(BQ, REG_NUM, REG_NAME, IN)			\
	_BQ2591X_UPDATE_REG_S(BQ, REG_NUM, REG_NAME, IN,		\
			      __UNIQUE_ID(ret), __UNIQUE_ID(val))

#define _BQ2591X_UPDATE_REG_BLS(BQ, REG_NUM, REG_NAME, IN, RET, VAL)	\
	({								\
	 u8 VAL = ((IN) - BQ2591X_##REG_NAME##_BASE);			\
	 int RET;							\
	 VAL /= BQ2591X_##REG_NAME##_LSB;				\
	 RET = BQ2591X_UPDATE_REG_S(BQ, REG_NUM, REG_NAME, VAL);	\
	 RET; })

#define BQ2591X_UPDATE_REG_BLS(BQ, REG_NUM, REG_NAME, IN)		\
	_BQ2591X_UPDATE_REG_BLS(BQ, REG_NUM, REG_NAME, IN,		\
				__UNIQUE_ID(ret), __UNIQUE_ID(val))

static int bq2591x_enable_charger(struct bq2591x *bq, bool enable)
{
	u8 val = enable ? BQ2591X_CHG_ENABLE : BQ2591X_CHG_DISABLE;

	return BQ2591X_UPDATE_REG_S(bq, BQ2591X_REG_06, EN_CHG, val);
}

static int bq2591x_enable_term(struct bq2591x *bq, bool enable)
{
	u8 val = enable ? BQ2591X_TERM_ENABLE : BQ2591X_TERM_DISABLE;

	return BQ2591X_UPDATE_REG_S(bq, BQ2591X_REG_05, EN_TERM, val);
}

static int bq2591x_set_chargecurrent(struct bq2591x *bq, int curr)
{
	return BQ2591X_UPDATE_REG_BLS(bq, BQ2591X_REG_01, ICHG, curr);
}

static int bq2591x_get_chargecurrent(struct bq2591x *bq, int *curr)
{
	return BQ2591X_READ_REG_MSLB(bq, BQ2591X_REG_01, ICHG, *curr);
}

static int bq2591x_set_chargevoltage(struct bq2591x *bq, int volt)
{
	return BQ2591X_UPDATE_REG_BLS(bq, BQ2591X_REG_00, VREG, volt);
}

static int bq2591x_get_chargevoltage(struct bq2591x *bq, int *volt)
{
	return BQ2591X_READ_REG_MSLB(bq, BQ2591X_REG_00, VREG, *volt);
}

static int bq2591x_set_input_volt_limit(struct bq2591x *bq, int volt)
{
	return BQ2591X_UPDATE_REG_BLS(bq, BQ2591X_REG_02, VINDPM, volt);
}

static int bq2591x_set_input_current_limit(struct bq2591x *bq, int curr)
{
	return BQ2591X_UPDATE_REG_BLS(bq, BQ2591X_REG_03, IINLIM, curr);
}

static int bq2591x_get_input_current_limit(struct bq2591x *bq, int *curr)
{
	return BQ2591X_READ_REG_MSLB(bq, BQ2591X_REG_03, IINLIM, *curr);
}

static int bq2591x_set_watchdog_timer(struct bq2591x *bq, u8 timeout)
{
	return BQ2591X_UPDATE_REG_BLS(bq, BQ2591X_REG_05, WDT, timeout);
}

static int bq2591x_disable_watchdog_timer(struct bq2591x *bq)
{
	return bq2591x_set_watchdog_timer(bq, 0);
}

static int bq2591x_reset_watchdog_timer(struct bq2591x *bq)
{
	return BQ2591X_UPDATE_REG_S(bq, BQ2591X_REG_05, WDT_RESET, BQ2591X_WDT_RESET);
}

static int bq2591x_reset_chip(struct bq2591x *bq)
{
	return BQ2591X_UPDATE_REG_S(bq, BQ2591X_REG_0D, RESET, BQ2591X_RESET);
}

static int bq2591x_set_vbatlow_volt(struct bq2591x *bq, int volt)
{
	switch (volt) {
	case 2600: volt = BQ2591X_VBATLOWV_2600MV; break;
	case 2900: volt = BQ2591X_VBATLOWV_2900MV; break;
	case 3200: volt = BQ2591X_VBATLOWV_3200MV; break;
	case 3500: volt = BQ2591X_VBATLOWV_3500MV; break;
	default: volt = BQ2591X_VBATLOWV_3500MV;
	}

	return BQ2591X_UPDATE_REG_S(bq, BQ2591X_REG_06, VBATLOWV, volt);
}

static ssize_t bq2591x_show_registers(struct device *dev,
				      struct device_attribute *attr,
				      char *buf)
{
	struct bq2591x *bq = dev_get_drvdata(dev);
	u8 tmpbuf[200];
	int len, ret;
	u8 addr, val;
	int idx = 0;

	idx = snprintf(buf, PAGE_SIZE, "%s:\n", "bq2591x Reg");
	for (addr = 0x0; addr <= 0x0D; addr++) {
		ret = bq2591x_read_byte(bq, &val, addr);
		if (ret == 0) {
			len = snprintf(tmpbuf, PAGE_SIZE - idx,
				       "Reg[%02X] = 0x%02X\n",	addr, val);
			memcpy(&buf[idx], tmpbuf, len);
			idx += len;
		}
	}

	return idx;
}

static ssize_t bq2591x_store_registers(struct device *dev,
				       struct device_attribute *attr,
				       const char *buf, size_t count)
{
	struct bq2591x *bq = dev_get_drvdata(dev);
	unsigned int reg, val;
	int ret;

	ret = sscanf(buf, "%x %x", &reg, &val);
	if (ret == 2 && reg <= 0x0D)
		bq2591x_write_byte(bq, (u8)reg, (u8)val);

	return count;
}

static DEVICE_ATTR(registers, 0644,
		   bq2591x_show_registers,
		   bq2591x_store_registers);

static struct attribute *bq2591x_attributes[] = {
	&dev_attr_registers.attr,
	NULL,
};

static const struct attribute_group bq2591x_attr_group = {
	.attrs = bq2591x_attributes,
};

static int show_registers(struct seq_file *m, void *data)
{
	struct bq2591x *bq = m->private;
	int addr;
	int ret;
	u8 val;

	for (addr = 0x0; addr <= 0x0D; addr++) {
		ret = bq2591x_read_byte(bq, &val, addr);
		if (!ret)
			seq_printf(m, "Reg[%02X] = 0x%02X\n", addr, val);
	}

	return 0;
}

static int reg_debugfs_open(struct inode *inode, struct file *file)
{
	struct bq2591x *bq = inode->i_private;

	return single_open(file, show_registers, bq);
}

static const struct file_operations reg_debugfs_ops = {
	.owner		= THIS_MODULE,
	.open		= reg_debugfs_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= single_release,
};

static void create_debugfs_entries(struct bq2591x *bq)
{
	bq->debug_root = debugfs_create_dir("bq2591x", NULL);
	if (!bq->debug_root)
		dev_err(bq->dev, "failed to create debug dir\n");

	if (bq->debug_root) {
		debugfs_create_file("registers", 0444, bq->debug_root,
				    bq, &reg_debugfs_ops);

		debugfs_create_x32("skip_writes", 0444, bq->debug_root,
				   &bq->skip_writes);

		debugfs_create_x32("skip_reads", 0444, bq->debug_root,
				   &bq->skip_reads);

		debugfs_create_x32("fault_status", 0444, bq->debug_root,
				   &bq->fault_status);

		debugfs_create_x32("charge_state", 0444, bq->debug_root,
				   &bq->charge_state);
	}
}

static int bq2591x_usb_suspend(struct bq2591x *bq, bool suspend)
{
	int rc = 0;

	rc = bq2591x_enable_charger(bq, suspend);
	if (rc) {
		dev_err(bq->dev, "failed to %s (rc=%d)\n",
			suspend ? "suspend" : "resume",  rc);
		return rc;
	}

	return rc;
}

int bq2591x_get_charging_status(struct bq2591x *bq)
{
	int ret, val;

	ret = BQ2591X_READ_REG_MS(bq, BQ2591X_REG_07, CHRG_STAT, val);
	if (ret < 0)
		return ret;

	switch (val) {
	case BQ2591X_CHRG_STAT_NCHG:
		return POWER_SUPPLY_STATUS_NOT_CHARGING;
	case BQ2591X_CHRG_STAT_FCHG:
		return POWER_SUPPLY_STATUS_CHARGING;
	case BQ2591X_CHRG_STAT_TCHG:
		return POWER_SUPPLY_STATUS_CHARGING;
	}

	return POWER_SUPPLY_STATUS_UNKNOWN;
}

static int bq2591x_get_prop_charge_type(struct bq2591x *bq)
{
	int ret, val;

	ret = BQ2591X_READ_REG_MS(bq, BQ2591X_REG_07, CHRG_STAT, val);
	if (ret)
		return POWER_SUPPLY_CHARGE_TYPE_UNKNOWN;

	dev_dbg(bq->dev, "Status Reg = 0x%02X\n", val);

	switch (val) {
	case BQ2591X_CHRG_STAT_NCHG:
		return POWER_SUPPLY_CHARGE_TYPE_NONE;
	case BQ2591X_CHRG_STAT_FCHG:
		return POWER_SUPPLY_CHARGE_TYPE_FAST;
	case BQ2591X_CHRG_STAT_TCHG:
		return POWER_SUPPLY_CHARGE_TYPE_TAPER;
	}

	return  POWER_SUPPLY_CHARGE_TYPE_NONE;
}

static bool bq2591x_is_usb_present(struct bq2591x *bq)
{
	union power_supply_propval val = { 0, };
	int rc;

	if (!bq->usb_psy)
		bq->usb_psy = power_supply_get_by_name("usb");

	if (!bq->usb_psy) {
		dev_err(bq->dev, "USB psy not found\n");
		return false;
	}

	rc = power_supply_get_property(bq->usb_psy, POWER_SUPPLY_PROP_ONLINE,
				       &val);

	if (rc < 0) {
		dev_err(bq->dev, "failed to get present property (rc=%d)\n",
			rc);
		return false;
	}

	if (val.intval)
		return true;

	return false;
}

static int bq2591x_is_input_current_limited(struct bq2591x *bq)
{
	u8 status;

	if (BQ2591X_READ_REG_M(bq, BQ2591X_REG_07, IINDPM_STAT, status))
		return 0;

	return status ? 1 : 0;
}

static void bq2591x_icl_softstart_workfunc(struct work_struct *work)
{
	struct bq2591x *bq = container_of(work, struct bq2591x,
					  icl_softstart_work.work);
	int icl_set, ret;

	ret = bq2591x_get_input_current_limit(bq, &icl_set);
	if (!ret) {
		if (bq->usb_psy_ma < icl_set) {
			bq2591x_set_input_current_limit(bq, bq->usb_psy_ma);
		} else if (bq->usb_psy_ma - icl_set < BQ2591X_IINLIM_LSB) {
			dev_dbg(bq->dev, "icl softstart done!\n");
			return;/*softstart done*/
		} else {
			icl_set = icl_set + BQ2591X_IINLIM_LSB;
			dev_dbg(bq->dev, "icl softstart set:%d\n", icl_set);
			bq2591x_set_input_current_limit(bq, icl_set);
			schedule_delayed_work(&bq->icl_softstart_work, HZ / 10);
		}
	} else {
		schedule_delayed_work(&bq->icl_softstart_work, HZ / 10);
	}
}

static int bq2591x_set_usb_chg_current(struct bq2591x *bq, int current_ma)
{
	int rc = 0;

	if (bq->revision == 0) /*PG1.0*/
		schedule_delayed_work(&bq->icl_softstart_work, 0);
	else
		rc = bq2591x_set_input_current_limit(bq, current_ma);

	if (rc) {
		dev_err(bq->dev, "failed to set input current limit:%d\n", rc);
		return rc;
	}

	return rc;
}

static void bq2591x_fcc_softstart_workfunc(struct work_struct *work)
{
	struct bq2591x *bq = container_of(work, struct bq2591x,
					  fcc_softstart_work.work);
	int fcc_set;
	int ret;

	ret = bq2591x_get_chargecurrent(bq, &fcc_set);
	if (!ret) {
		if (bq->fast_cc_ma < fcc_set) {
			bq2591x_set_chargecurrent(bq, bq->fast_cc_ma);
		} else if (bq->fast_cc_ma - fcc_set < BQ2591X_ICHG_LSB) {
			dev_dbg(bq->dev, "fcc softstart done!\n");
			return;/*softstart done*/
		} else {
			fcc_set = fcc_set + BQ2591X_ICHG_LSB;
			dev_dbg(bq->dev, "fcc softstart set fcc:%d\n", fcc_set);
			bq2591x_set_chargecurrent(bq, fcc_set);
			schedule_delayed_work(&bq->fcc_softstart_work, HZ / 10);
		}
	} else {
		schedule_delayed_work(&bq->fcc_softstart_work, HZ / 10);
	}
}

static int bq2591x_set_fast_chg_current(struct bq2591x *bq, int current_ma)
{
	int ret = 0;

	if (bq->revision == 0) {
		bq2591x_set_chargecurrent(bq, 1000);
		schedule_delayed_work(&bq->fcc_softstart_work, 0);
	} else {
		ret = bq2591x_set_chargecurrent(bq, current_ma);
	}

	return ret;
}

static enum power_supply_property bq2591x_charger_properties[] = {
	POWER_SUPPLY_PROP_CHARGE_TYPE,
	POWER_SUPPLY_PROP_ONLINE,
	POWER_SUPPLY_PROP_CHARGING_ENABLED,
	POWER_SUPPLY_PROP_PIN_ENABLED,
	POWER_SUPPLY_PROP_INPUT_SUSPEND,
	POWER_SUPPLY_PROP_CHARGER_TEMP,
	POWER_SUPPLY_PROP_CHARGER_TEMP_MAX,
	POWER_SUPPLY_PROP_VOLTAGE_MAX,
	POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT_MAX,
	POWER_SUPPLY_PROP_MODEL_NAME,
	POWER_SUPPLY_PROP_PARALLEL_MODE,
	POWER_SUPPLY_PROP_CONNECTOR_HEALTH,
	POWER_SUPPLY_PROP_PARALLEL_BATFET_MODE,
	POWER_SUPPLY_PROP_PARALLEL_FCC_MAX,
	POWER_SUPPLY_PROP_INPUT_CURRENT_LIMITED,
	POWER_SUPPLY_PROP_MIN_ICL,
	POWER_SUPPLY_PROP_CURRENT_MAX,
	POWER_SUPPLY_PROP_SET_SHIP_MODE,
	POWER_SUPPLY_PROP_DIE_HEALTH,
};

static int bq2591x_charger_set_property(struct power_supply *psy,
					enum power_supply_property prop,
					const union power_supply_propval *val)
{
	struct bq2591x *bq = power_supply_get_drvdata(psy);
	int rc = 0;

	switch (prop) {
	case POWER_SUPPLY_PROP_INPUT_SUSPEND:
		dev_dbg(bq->dev, "POWER_SUPPLY_PROP_INPUT_SUSPEND: %d\n",
			 val->intval);

		rc = bq2591x_usb_suspend(bq, val->intval);
		break;

	case POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT_MAX:
		dev_dbg(bq->dev,
			"POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT_MAX: %d\n",
			val->intval);

		bq->fast_cc_ma = val->intval / 1000;
		rc = bq2591x_set_fast_chg_current(bq, bq->fast_cc_ma);
		break;

	case POWER_SUPPLY_PROP_CURRENT_MAX:
		dev_dbg(bq->dev, "POWER_SUPPLY_PROP_CURRENT_MAX: %d\n",
			val->intval);

		bq->usb_psy_ma = val->intval / 1000;
		rc = bq2591x_set_usb_chg_current(bq, bq->usb_psy_ma);
		break;

	case POWER_SUPPLY_PROP_VOLTAGE_MAX:
		dev_dbg(bq->dev, "POWER_SUPPLY_PROP_VOLTAGE_MAX: %d\n",
			val->intval);

		bq->vfloat_mv = val->intval / 1000;
		rc = bq2591x_set_chargevoltage(bq, bq->vfloat_mv);
		break;

	case POWER_SUPPLY_PROP_CONNECTOR_HEALTH:
		bq->c_health = val->intval;
		power_supply_changed(bq->parallel_psy);
		break;

	case POWER_SUPPLY_PROP_SET_SHIP_MODE:
		rc = 0;
		break;

	default:
		dev_err(bq->dev, "unsupported prop: %d", prop);
		return -EINVAL;
	}

	return rc;
}

static int bq2591x_charger_is_writeable(struct power_supply *psy,
					enum power_supply_property prop)
{
	int rc;

	switch (prop) {
	case POWER_SUPPLY_PROP_CONNECTOR_HEALTH:
		rc = 1;
		break;
	default:
		rc = 0;
		break;
	}
	return rc;
}

#define MIN_PARALLEL_ICL_UA 250000

static int bq2591x_charger_get_property(struct power_supply *psy,
					enum power_supply_property prop,
					union power_supply_propval *val)
{
	struct bq2591x *bq = power_supply_get_drvdata(psy);
	int rc, itemp = 0;

	switch (prop) {
	case POWER_SUPPLY_PROP_ONLINE:
		rc = BQ2591X_READ_REG_M(bq, BQ2591X_REG_07, PG_STAT, itemp);
		if (rc >= 0)
			val->intval = !!itemp;

		dev_dbg(bq->dev, "POWER_SUPPLY_PROP_ONLINE: %d\n",
			val->intval);
		break;

	case POWER_SUPPLY_PROP_CHARGING_ENABLED:
		/* assume it is always enabled, using SUSPEND to control
		 * charging */
		val->intval = 1;
		dev_dbg(bq->dev, "POWER_SUPPLY_PROP_CHARGING_ENABLED: %d\n",
			val->intval);
		break;

	case POWER_SUPPLY_PROP_INPUT_SUSPEND:
		rc = BQ2591X_READ_REG_M(bq, BQ2591X_REG_06, EN_CHG, itemp);
		if (rc >= 0)
			val->intval = !itemp;

		dev_dbg(bq->dev, "POWER_SUPPLY_PROP_INPUT_SUSPEND: %d\n",
			val->intval);

		break;

	case POWER_SUPPLY_PROP_CURRENT_MAX:
		rc = bq2591x_get_input_current_limit(bq, &itemp);
		if (rc >= 0)
			val->intval = itemp * 1000;

		dev_dbg(bq->dev, "POWER_SUPPLY_PROP_CURRENT_MAX: %d\n",
			val->intval);
		break;

	case POWER_SUPPLY_PROP_VOLTAGE_MAX:
		rc = bq2591x_get_chargevoltage(bq, &itemp);
		if (rc >= 0)
			val->intval = itemp * 1000;

		dev_dbg(bq->dev, "POWER_SUPPLY_PROP_VOLTAGE_MAX: %d\n",
			val->intval);
		break;

	case POWER_SUPPLY_PROP_CHARGE_TYPE:
		val->intval = POWER_SUPPLY_CHARGE_TYPE_NONE;

		if (bq2591x_is_usb_present(bq)) {
			val->intval = bq2591x_get_prop_charge_type(bq);
			if (val->intval == POWER_SUPPLY_CHARGE_TYPE_UNKNOWN) {
				dev_dbg(bq->dev,
					"failed to get charge type, charger may be absent\n");
				return -ENODEV;
			}
		}

		dev_dbg(bq->dev, "POWER_SUPPLY_PROP_CHARGE_TYPE:%d\n",
			val->intval);
		break;

	case POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT_MAX:
		rc = bq2591x_get_chargecurrent(bq, &itemp);
		if (rc >= 0)
			val->intval = itemp * 1000;

		dev_dbg(bq->dev,
			"POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT_MAX: %d\n",
			val->intval);
		break;

	case POWER_SUPPLY_PROP_STATUS:
		val->intval = bq2591x_get_charging_status(bq);

		dev_dbg(bq->dev, "POWER_SUPPLY_PROP_STATUS: %d\n",
			val->intval);
		break;

	case POWER_SUPPLY_PROP_INPUT_CURRENT_LIMITED:
		val->intval = bq2591x_is_input_current_limited(bq);

		dev_dbg(bq->dev,
			"POWER_SUPPLY_PROP_INPUT_CURRENT_LIMITED: %d\n",
			val->intval);
		break;

	case POWER_SUPPLY_PROP_PARALLEL_MODE:
		val->intval = POWER_SUPPLY_PL_USBIN_USBIN;

		dev_dbg(bq->dev, "POWER_SUPPLY_PROP_PARALLEL_MODE: %d\n",
			val->intval);
		break;

	case POWER_SUPPLY_PROP_MODEL_NAME:
		val->strval = "smbq2591x";
		break;

	case POWER_SUPPLY_PROP_PARALLEL_BATFET_MODE:
		/* used for 910 output to connect to vphpwr */
		val->intval = POWER_SUPPLY_PL_STACKED_BATFET;
		break;

	case POWER_SUPPLY_PROP_PIN_ENABLED:
		val->intval = 0;
		break;

	case POWER_SUPPLY_PROP_CONNECTOR_HEALTH:
		val->intval = POWER_SUPPLY_HEALTH_GOOD;
		break;

	case POWER_SUPPLY_PROP_CHARGER_TEMP:
		val->intval = 20000;
		break;

	case POWER_SUPPLY_PROP_CHARGER_TEMP_MAX:
		val->intval = 80000;
		break;

	case POWER_SUPPLY_PROP_SET_SHIP_MODE:
		val->intval = 0;
		break;

	case POWER_SUPPLY_PROP_MIN_ICL:
		val->intval = MIN_PARALLEL_ICL_UA;
		break;

	case POWER_SUPPLY_PROP_PARALLEL_FCC_MAX:
		val->intval = bq->max_fcc;
		break;

	default:
		dev_err(bq->dev, "unsupported prop: %d\n", prop);
		return -EINVAL;
	}

	return 0;
}

static int bq2591x_parse_dt(struct device *dev, struct bq2591x *bq)
{
	struct device_node *np = dev->of_node;
	int ret;

	bq->cfg.enable_term = of_property_read_bool(np,
						    "ti,bq2591x,enable-term");

	ret = of_property_read_u32(np, "ti,bq2591x,charge-voltage",
				   &bq->cfg.chg_mv);
	if (ret)
		return ret;

	bq->vfloat_mv = bq->cfg.chg_mv;

	ret = of_property_read_u32(np, "ti,bq2591x,charge-current",
				   &bq->cfg.chg_ma);
	if (ret)
		return ret;

	bq->fast_cc_ma = bq->cfg.chg_ma;

	ret = of_property_read_u32(np, "ti,bq2591x,input-current-limit",
				   &bq->cfg.icl_ma);
	if (ret)
		return ret;

	ret = of_property_read_u32(np, "ti,bq2591x,input-voltage-limit",
				   &bq->cfg.ivl_mv);
	if (ret)
		return ret;

	ret = of_property_read_u32(np, "ti,bq2591x,vbatlow-volt",
				   &bq->cfg.batlow_mv);

	return ret;
}

static int bq2591x_detect_device(struct bq2591x *bq)
{
	int ret;
	u8 data;

	ret = bq2591x_read_byte(bq, &data, BQ2591X_REG_0D);
	if (ret == 0) {
		bq->part_no = (data & BQ2591X_PN_MASK) >> BQ2591X_PN_SHIFT;
		bq->revision = (data & BQ2591X_DEV_REV_MASK) >>
			BQ2591X_DEV_REV_SHIFT;
	}

	return ret;
}

static int bq2591x_set_charge_profile(struct bq2591x *bq)
{
	int ret;

	ret = bq2591x_set_chargevoltage(bq, bq->cfg.chg_mv);
	if (ret < 0) {
		dev_err(bq->dev, "failed to set charge voltage (rc=%d)\n",
			ret);
		return ret;
	}

	ret = bq2591x_set_chargecurrent(bq, bq->cfg.chg_ma);
	if (ret < 0) {
		dev_err(bq->dev, "failed to set charge current (rc=%d)\n",
			ret);
		return ret;
	}

	ret = bq2591x_set_input_current_limit(bq, bq->cfg.icl_ma);
	if (ret < 0) {
		dev_err(bq->dev, "failed to set input current limit (rc=%d)\n",
			ret);
		return ret;
	}

	ret = bq2591x_set_input_volt_limit(bq, bq->cfg.ivl_mv);
	if (ret < 0) {
		dev_err(bq->dev, "failed to set input voltage limit (rc=%d)\n",
			ret);
		return ret;
	}
	return 0;
}

static int bq2591x_init_device(struct bq2591x *bq)
{
	int ret;

	bq->chg_mv = bq->cfg.chg_mv;
	bq->chg_ma = bq->cfg.chg_ma;
	bq->ivl_mv = bq->cfg.ivl_mv;
	bq->icl_ma = bq->cfg.icl_ma;

	ret = bq2591x_disable_watchdog_timer(bq);
	if (ret < 0)
		return ret;

	/* as slave charger, disable it by default */
	bq2591x_usb_suspend(bq, true);

	ret = bq2591x_enable_term(bq, bq->cfg.enable_term);
	if (ret < 0)
		return ret;

	ret = bq2591x_set_vbatlow_volt(bq, bq->cfg.batlow_mv);
	if (ret < 0)
		return ret;

	bq2591x_set_charge_profile(bq);

	return ret;
}

static void bq2591x_dump_regs(struct bq2591x *bq)
{
	u8 addr, val;
	int ret;

	for (addr = 0x00; addr <= 0x0D; addr++) {
		msleep(2);
		ret = bq2591x_read_byte(bq, &val, addr);
		if (!ret)
			dev_err(bq->dev, "Reg[%02X] = 0x%02X\n", addr, val);
	}
}

static void bq2591x_stat_handler(struct bq2591x *bq)
{
	if (bq->prev_stat_flag == bq->reg_stat_flag)
		return;

	bq->prev_stat_flag = bq->reg_stat_flag;

	dev_info(bq->dev, "power %s\n", (bq->reg_stat & BQ2591X_PG_STAT_MASK) ?
		 "good" : "poor");

	if (bq->reg_stat & BQ2591X_IINDPM_STAT_MASK)
		dev_info(bq->dev, "IINDPM triggered\n");

	if (bq->reg_stat & BQ2591X_VINDPM_STAT_MASK)
		dev_info(bq->dev, "VINDPM triggered\n");

	if (bq->reg_stat & BQ2591X_TREG_STAT_MASK)
		dev_info(bq->dev, "TREG triggered\n");

	if (bq->reg_stat & BQ2591X_WD_STAT_MASK)
		dev_err(bq->dev, "watchdog overflow\n");

	bq->charge_state = (bq->reg_stat & BQ2591X_CHRG_STAT_MASK)
		>> BQ2591X_CHRG_STAT_SHIFT;

	if (bq->charge_state == BQ2591X_CHRG_STAT_NCHG)
		dev_info(bq->dev, "Not Charging\n");
	else if (bq->charge_state == BQ2591X_CHRG_STAT_FCHG)
		dev_info(bq->dev, "Fast Charging\n");
	else if (bq->charge_state == BQ2591X_CHRG_STAT_TCHG)
		dev_info(bq->dev, "Taper Charging\n");
}

static void bq2591x_fault_handler(struct bq2591x *bq)
{
	if (bq->prev_fault_flag == bq->reg_fault_flag)
		return;

	bq->prev_fault_flag = bq->reg_fault_flag;

	if (bq->reg_fault_flag & BQ2591X_VBUS_OVP_FLAG_MASK)
		dev_info(bq->dev,
			 "VBus OVP fault occurred, current stat: %d",
			 bq->reg_fault & BQ2591X_VBUS_OVP_STAT_MASK);

	if (bq->reg_fault_flag & BQ2591X_TSHUT_FLAG_MASK)
		dev_info(bq->dev,
			 "Thermal shutdown occurred, current stat: %d",
			 bq->reg_fault & BQ2591X_TSHUT_STAT_MASK);

	if (bq->reg_fault_flag & BQ2591X_BATOVP_FLAG_MASK)
		dev_info(bq->dev,
			 "Battery OVP fault occurred, current stat: %d",
			 bq->reg_fault & BQ2591X_BATOVP_STAT_MASK);

	if (bq->reg_fault_flag & BQ2591X_CFLY_FLAG_MASK)
		dev_info(bq->dev,
			 "CFLY fault occurred, current stat: %d",
			 bq->reg_fault & BQ2591X_CFLY_STAT_MASK);

	if (bq->reg_fault_flag & BQ2591X_TMR_FLAG_MASK)
		dev_info(bq->dev,
			 "Charge safety timer fault, current stat: %d",
			 bq->reg_fault & BQ2591X_TMR_STAT_MASK);

	if (bq->reg_fault_flag & BQ2591X_CAP_COND_FLAG_MASK)
		dev_info(bq->dev,
			 "CAP conditon fault occurred, current stat: %d",
			 bq->reg_fault & BQ2591X_CAP_COND_STAT_MASK);
}

static irqreturn_t bq2591x_charger_interrupt(int irq, void *data)
{
	struct bq2591x *bq = data;
	int ret;
	u8  val;

	ret = bq2591x_read_byte(bq, &val, BQ2591X_REG_07);
	if (ret)
		return IRQ_HANDLED;
	bq->reg_stat = val;

	ret = bq2591x_read_byte(bq, &val, BQ2591X_REG_08);
	if (ret)
		return IRQ_HANDLED;
	bq->reg_fault = val;

	ret = bq2591x_read_byte(bq, &val, BQ2591X_REG_09);
	if (ret)
		return IRQ_HANDLED;
	bq->reg_stat_flag = val;

	ret = bq2591x_read_byte(bq, &val, BQ2591X_REG_0A);
	if (ret)
		return IRQ_HANDLED;
	bq->reg_fault_flag = val;

	bq2591x_stat_handler(bq);
	bq2591x_fault_handler(bq);

	bq2591x_dump_regs(bq);

	return IRQ_HANDLED;
}

static void bq2591x_monitor_workfunc(struct work_struct *work)
{
	struct bq2591x *bq = container_of(work, struct bq2591x,
					  monitor_work.work);

	bq2591x_dump_regs(bq);

	schedule_delayed_work(&bq->monitor_work, 5 * HZ);
}

static int bq2591x_charger_probe(struct i2c_client *client,
				 const struct i2c_device_id *id)
{
	struct power_supply_config parallel_psy_cfg = {};
	struct bq2591x *bq = NULL;
	int ret;

	bq = devm_kzalloc(&client->dev, sizeof(struct bq2591x), GFP_KERNEL);
	if (!bq)
		return -ENOMEM;

	bq->dev = &client->dev;
	bq->client = client;

	i2c_set_clientdata(client, bq);

	mutex_init(&bq->i2c_rw_lock);

	ret = bq2591x_detect_device(bq);
	if (!ret && bq->part_no == BQ25910) {
		dev_info(bq->dev,
			 "charger device bq25910 detected, revision: %d\n",
			 bq->revision);
	} else {
		dev_info(bq->dev, "no bq25910 charger device found (rc=%d)\n",
			 ret);
		return -ENODEV;
	}

	if (client->dev.of_node)
		bq2591x_parse_dt(&client->dev, bq);

	ret = bq2591x_init_device(bq);
	if (ret) {
		dev_err(bq->dev, "failed to init device (rc=%d)\n", ret);
		goto err_0;
	}

	bq->max_fcc = INT_MAX;
	bq->c_health = -EINVAL;

	INIT_DELAYED_WORK(&bq->monitor_work,
			  bq2591x_monitor_workfunc);
	INIT_DELAYED_WORK(&bq->icl_softstart_work,
			  bq2591x_icl_softstart_workfunc);
	INIT_DELAYED_WORK(&bq->fcc_softstart_work,
			  bq2591x_fcc_softstart_workfunc);

	bq->parallel_psy_d.name	= "parallel";
	bq->parallel_psy_d.type	= POWER_SUPPLY_TYPE_PARALLEL;
	bq->parallel_psy_d.get_property = bq2591x_charger_get_property;
	bq->parallel_psy_d.set_property = bq2591x_charger_set_property;
	bq->parallel_psy_d.properties   = bq2591x_charger_properties;
	bq->parallel_psy_d.property_is_writeable = bq2591x_charger_is_writeable;
	bq->parallel_psy_d.num_properties =
		ARRAY_SIZE(bq2591x_charger_properties);

	parallel_psy_cfg.drv_data = bq;
	parallel_psy_cfg.num_supplicants = 0;
	bq->parallel_psy = devm_power_supply_register(bq->dev,
						      &bq->parallel_psy_d,
						      &parallel_psy_cfg);
	if (IS_ERR(bq->parallel_psy)) {
		dev_err(bq->dev, "failed to register parallel psy (rc=%ld)\n",
			PTR_ERR(bq->parallel_psy));
		ret = PTR_ERR(bq->parallel_psy);
		return ret;
	}

	if (client->irq) {
		ret = devm_request_threaded_irq(&client->dev, client->irq, NULL,
						bq2591x_charger_interrupt,
						IRQF_TRIGGER_FALLING |
						IRQF_ONESHOT,
						"bq2591x charger irq", bq);
		if (ret < 0) {
			dev_err(bq->dev,
				"request irq for irq=%d failed (rc =%d)\n",
				client->irq, ret);
			goto err_0;
		}
	}

	schedule_delayed_work(&bq->monitor_work, HZ);

	create_debugfs_entries(bq);

	ret = sysfs_create_group(&bq->dev->kobj, &bq2591x_attr_group);
	if (ret)
		dev_err(bq->dev, "failed to register sysfs (rc=%d)\n", ret);

	dev_info(bq->dev,
		 "BQ2591X PARALLEL charger driver probe successfully\n");

	return 0;

err_0:
	power_supply_unregister(bq->parallel_psy);
	return ret;
}

static int bq2591x_charger_remove(struct i2c_client *client)
{
	struct bq2591x *bq = i2c_get_clientdata(client);

	cancel_delayed_work_sync(&bq->monitor_work);

	power_supply_unregister(bq->parallel_psy);

	mutex_destroy(&bq->i2c_rw_lock);

	debugfs_remove_recursive(bq->debug_root);
	sysfs_remove_group(&bq->dev->kobj, &bq2591x_attr_group);

	return 0;
}

static void bq2591x_charger_shutdown(struct i2c_client *client)
{
	pr_info("shutdown\n");
}

static const struct of_device_id bq2591x_charger_match_table[] = {
	{ .compatible = "ti,smbq2591x" },
	{ },
};

static const struct i2c_device_id bq2591x_charger_id[] = {
	{ "bq2591x", BQ25910 },
	{ },
};

MODULE_DEVICE_TABLE(i2c, bq2591x_charger_id);

static struct i2c_driver bq2591x_charger_driver = {
	.driver = {
		.name	= "smbq2591x",
		.of_match_table = bq2591x_charger_match_table,
	},
	.id_table = bq2591x_charger_id,
	.probe = bq2591x_charger_probe,
	.remove = bq2591x_charger_remove,
	.shutdown = bq2591x_charger_shutdown,
};

module_i2c_driver(bq2591x_charger_driver);

MODULE_DESCRIPTION("TI BQ2591x Charger Driver");
MODULE_LICENSE("GPL v2");
