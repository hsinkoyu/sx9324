/*
 * Semtech SX9324 - a capacitive Specific Absorption Rate (SAR) controller
 *
 * Copyright (C) 2020 FIH Mobile Limited
 *
 * Author: Hsinko Yu <hsinkoyu@fih-foxconn.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#define pr_fmt(fmt) KBUILD_MODNAME ": " fmt
#define DEBUG

#include <linux/delay.h>
#include <linux/gpio/consumer.h>
#include <linux/i2c.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/regmap.h>
#include <linux/regulator/consumer.h>

#define DRIVER_NAME "sx9324"

/* Interrupt and Status */
#define SX9324_IRQ_SRC		0x00
#define  SX9324_RESETIRQ		BIT(7)
#define  SX9324_CLOSEANYIRQ		BIT(6)
#define  SX9324_FARANYIRQ		BIT(5)
#define  SX9324_COMPDONEIRQ		BIT(4)
#define  SX9324_CONVDONEIRQ		BIT(3)
#define  SX9324_PROG2IRQ		BIT(2)
#define  SX9324_PROG1IRQ		BIT(1)
#define  SX9324_PROG0IRQ		BIT(0)
#define SX9324_STAT_0		0x01
#define SX9324_STAT_1		0x02
#define SX9324_STAT_2		0x03
#define SX9324_STAT_3		0x04
#define SX9324_IRQ_MSK		0x05
#define  SX9324_CLOSEANYIRQEN	BIT(6)
#define  SX9324_FARANYIRQEN		BIT(5)
#define  SX9324_COMPDONEIRQEN	BIT(4)
#define  SX9324_CONVDONEIRQEN	BIT(3)
#define  SX9324_PROG2IRQEN		BIT(2)
#define  SX9324_PROG1IRQEN		BIT(1)
#define  SX9324_PROG0IRQEN		BIT(0)
#define SX9324_IRQ_CFG_0	0x06
#define SX9324_IRQ_CFG_1	0x07
#define SX9324_IRQ_CFG_2	0x08

/* General Control */
#define SX9324_GNRL_CTRL_0	0x10
#define  SX9324_PAUSEIRQEN		BIT(7)
#define  SX9324_DOZEPERIOD		GENMASK(6, 5)
#define  SX9324_SCANPERIOD		GENMASK(4, 0)
#define SX9324_GNRL_CTRL_1	0x11
#define  SX9324_PHEN			GENMASK(3, 0)
#define SX9324_I2C_ADDR		0x14
#define SX9324_CLK_SPRD		0x15

/* Analog‐Front‐End (AFE) Control */
#define SX9324_AFE_CTRL_0	0x20
#define SX9324_AFE_CTRL_1	0x21
#define SX9324_AFE_CTRL_2	0x22
#define SX9324_AFE_CTRL_3	0x23
#define SX9324_AFE_CTRL_4	0x24
#define SX9324_AFE_CTRL_5	0x25
#define SX9324_AFE_CTRL_6	0x26
#define SX9324_AFE_CTRL_7	0x27
#define SX9324_AFE_PH_0		0x28
#define SX9324_AFE_PH_1		0x29
#define SX9324_AFE_PH_2		0x2a
#define SX9324_AFE_PH_3		0x2b
#define SX9324_AFE_CTRL_8	0x2c
#define SX9324_AFE_CTRL_9	0x2d

/* Main Digital Processing (Prox) Control */
#define SX9324_PROX_CTRL_0	0x30
#define SX9324_PROX_CTRL_1	0x31
#define SX9324_PROX_CTRL_2	0x32
#define SX9324_PROX_CTRL_3	0x33
#define SX9324_PROX_CTRL_4	0x34
#define SX9324_PROX_CTRL_5	0x35
#define SX9324_PROX_CTRL_6	0x36
#define SX9324_PROX_CTRL_7	0x37

/* Advanced Digital Processing Control */
#define SX9324_ADV_CTRL_0	0x40
#define SX9324_ADV_CTRL_1	0x41
#define SX9324_ADV_CTRL_2	0x42
#define SX9324_ADV_CTRL_3	0x43
#define SX9324_ADV_CTRL_4	0x44
#define SX9324_ADV_CTRL_5	0x45
#define SX9324_ADV_CTRL_6	0x46
#define SX9324_ADV_CTRL_7	0x47
#define SX9324_ADV_CTRL_8	0x48
#define SX9324_ADV_CTRL_9	0x49
#define SX9324_ADV_CTRL_10	0x4a
#define SX9324_ADV_CTRL_11	0x4b
#define SX9324_ADV_CTRL_12	0x4c
#define SX9324_ADV_CTRL_13	0x4d
#define SX9324_ADV_CTRL_14	0x4e
#define SX9324_ADV_CTRL_15	0x4f
#define SX9324_ADV_CTRL_16	0x50
#define SX9324_ADV_CTRL_17	0x51
#define SX9324_ADV_CTRL_18	0x52
#define SX9324_ADV_CTRL_19	0x53
#define SX9324_ADV_CTRL_20	0x54

/* Phase Data Readback */
#define SX9324_PHASE_SEL	0x60
#define SX9324_USE_MSB		0x61
#define SX9324_USE_LSB		0x62
#define SX9324_AVG_MSB		0x63
#define SX9324_AVG_LSB		0x64
#define SX9324_DIFF_MSB		0x65
#define SX9324_DIFF_LSB		0x66
#define SX9324_OFFSET_MSB	0x67
#define SX9324_OFFSET_LSB	0x68
#define SX9324_SAR_MSB		0x69
#define SX9324_SAR_LSB		0x6a

/* Miscellaneous */
#define SX9324_RESET		0x9f
#define SX9324_WHO_AM_I		0xfa
#define SX9324_REV			0xfe

#define to_client(dev) container_of(dev, struct i2c_client, dev)

/* lock to read phase data without interruption */
static DEFINE_MUTEX(phdata_readback_lock);

enum sx9324_phase {
	PH0,
	PH1,
	PH2,
	PH3,
	SX9324_PHASES
};

struct sx9324_phase_stat {
	unsigned int steady : 1;
	unsigned int prox : 1;
	unsigned int table : 1;
	unsigned int body : 1;
	unsigned int fail : 1;
	unsigned int comp : 1;
};

struct sx9324_phase_data {
	bool is_valid; /* data is valid when phase is enabled */
	int proxuseful;
	int proxavg;
	int proxdiff;
	struct sx9324_phase_stat stat;
};

struct sx9324_data {
	struct i2c_client *client;
	struct regmap *regmap;
	struct gpio_desc *nirq_gpio;
	int nirq;
	struct workqueue_struct *workqueue;
	struct work_struct nirq_work;
	/* host dependent power control */
	struct regulator *pullup;
	bool pullup_enabled;
	struct regulator *vdd;
	bool vdd_enabled;
	/* phase data container */
	struct sx9324_phase_data phdata[SX9324_PHASES];

};

enum sx9324_reset_source {
	POWER_UP_RESET,
	SOFTWARE_RESET
};

enum sx9324_operational_mode {
	SX9324_ACTIVE,
	SX9324_DOZE,
	SX9324_SLEEP
};

static const struct reg_default sx9324_reg_defaults[] = {
	{ SX9324_IRQ_MSK,       SX9324_CLOSEANYIRQEN | SX9324_FARANYIRQEN },
	{ SX9324_GNRL_CTRL_0,   0x45 }, /* Tscan=10ms, Tdoze=8xTscan */
	/* phase 0: CS0 measured while CS1/2 are shielded */
	{ SX9324_AFE_PH_0,      0x29 },
	{ SX9324_GNRL_CTRL_1,   0x21 }, /* only phase 0 enabled */
};

static int sx9324_enable_pullup(struct device *dev, bool enable)
{
	int error = 0;
	struct sx9324_data *drv_data =
		(struct sx9324_data *)i2c_get_clientdata(to_client(dev));

	if (drv_data->pullup_enabled != enable) {
		if (enable)
			error = regulator_enable(drv_data->pullup);
		else
			error = regulator_disable(drv_data->pullup);
		if (!error)
			drv_data->pullup_enabled = enable;
	}
	return error;
}

static int sx9324_enable_vdd(struct device *dev, bool enable)
{
	int error = 0;
	struct sx9324_data *drv_data =
		(struct sx9324_data *)i2c_get_clientdata(to_client(dev));

	if (drv_data->vdd_enabled != enable) {
		if (enable)
			error = regulator_enable(drv_data->vdd);
		else
			error = regulator_disable(drv_data->vdd);
		if (!error)
			drv_data->vdd_enabled = enable;
	}
	return error;
}

static bool sx9324_get_software_default(struct device *dev, int reg,
	unsigned int *val)
{
	int i = 0;

	while (i < ARRAY_SIZE(sx9324_reg_defaults)) {
		if (sx9324_reg_defaults[i].reg == reg) {
			*val = sx9324_reg_defaults[i].def;
			return true;
		}
		i++;
	}
	return false;
}

static int sx9324_reset_software_default(struct device *dev)
{
	struct sx9324_data *drv_data =
		(struct sx9324_data *)i2c_get_clientdata(to_client(dev));
	int i;
	int error = 0;

	for (i = 0; i < ARRAY_SIZE(sx9324_reg_defaults); i++) {
		error = regmap_write(drv_data->regmap, sx9324_reg_defaults[i].reg,
			sx9324_reg_defaults[i].def);
		if (error) {
			pr_err("failed to write register '0x%02x' with software default "
				"value, err=%d\n", sx9324_reg_defaults[i].reg, error);
			break;
		}
	}
	return error;
}

static int sx9324_read_phdata(struct device *dev)
{
	struct sx9324_data *drv_data =
		(struct sx9324_data *)i2c_get_clientdata(to_client(dev));
	unsigned int val;
	unsigned int stat0, stat1, stat2;
	unsigned int msb, lsb;
	int error;
	int i;

	for (i = PH0; i < SX9324_PHASES; i++)
		drv_data->phdata[i].is_valid = false;

	error = regmap_read(drv_data->regmap, SX9324_GNRL_CTRL_1, &val);
	if (error)
		return error;

	mutex_lock(&phdata_readback_lock);
	if (val & SX9324_PHEN) {
		error = regmap_read(drv_data->regmap, SX9324_STAT_0, &stat0);
		error |= regmap_read(drv_data->regmap, SX9324_STAT_1, &stat1);
		error |= regmap_read(drv_data->regmap, SX9324_STAT_2, &stat2);
		for (i = PH0; i < SX9324_PHASES && !error; i++) {
			/* an enabled phase */
			if ((val >> i) & 0x01) {
				error = regmap_write(drv_data->regmap, SX9324_PHASE_SEL, i);
				if (error)
					break;

				error = regmap_read(drv_data->regmap, SX9324_USE_MSB, &msb);
				error |= regmap_read(drv_data->regmap, SX9324_USE_LSB, &lsb);
				if (error)
					break;
				/* must type-cast to short to be signed */
				drv_data->phdata[i].proxuseful = (short)((msb << 8) | lsb);

				error = regmap_read(drv_data->regmap, SX9324_AVG_MSB, &msb);
				error |= regmap_read(drv_data->regmap, SX9324_AVG_LSB, &lsb);
				if (error)
					break;
				drv_data->phdata[i].proxavg = (short)((msb << 8) | lsb);

				error = regmap_read(drv_data->regmap, SX9324_DIFF_MSB, &msb);
				error |= regmap_read(drv_data->regmap, SX9324_DIFF_LSB, &lsb);
				if (error)
					break;
				drv_data->phdata[i].proxdiff = (short)((msb << 8) | lsb);

				drv_data->phdata[i].stat.steady = (stat0 >> (i + 4)) & 0x01;
				drv_data->phdata[i].stat.prox = (stat0 >> i) & 0x01;
				drv_data->phdata[i].stat.table = (stat1 >> (i + 4)) & 0x01;
				drv_data->phdata[i].stat.body = (stat1 >> i) & 0x01;
				drv_data->phdata[i].stat.fail = (stat2 >> (i + 4)) & 0x01;
				drv_data->phdata[i].stat.comp = (stat2 >> i) & 0x01;

				drv_data->phdata[i].is_valid = true;
			}
		}
	}
	mutex_unlock(&phdata_readback_lock);

	return error;
}

static int sx9324_get_mode(struct device *dev,
	enum sx9324_operational_mode *mode)
{
	struct sx9324_data *drv_data =
		(struct sx9324_data *)i2c_get_clientdata(to_client(dev));
	int error;
	unsigned int val;

	error = regmap_read(drv_data->regmap, SX9324_GNRL_CTRL_1, &val);
	if (!error) {
		if (val & SX9324_PHEN) {
			error = regmap_read(drv_data->regmap, SX9324_GNRL_CTRL_0, &val);
			if (!error) {
				if (val & SX9324_DOZEPERIOD)
					*mode = SX9324_DOZE;
				else
					*mode = SX9324_ACTIVE;
			}
		} else {
			*mode = SX9324_SLEEP;
		}
	}

	return error;
}

static int sx9324_set_mode(struct device *dev,
	enum sx9324_operational_mode mode)
{
	struct sx9324_data *drv_data =
		(struct sx9324_data *)i2c_get_clientdata(to_client(dev));
	int error;
	unsigned int val;

	switch (mode) {
		case SX9324_ACTIVE:
		case SX9324_DOZE:
			if (!sx9324_get_software_default(dev, SX9324_GNRL_CTRL_1, &val))
				val = 0x0f; /* all phases enabled if no software default set */
			error = regmap_update_bits(drv_data->regmap, SX9324_GNRL_CTRL_1,
				SX9324_PHEN, val);
			if (mode == SX9324_ACTIVE) {
				error |= regmap_update_bits(drv_data->regmap, SX9324_GNRL_CTRL_0,
					SX9324_DOZEPERIOD, 0);
			} else {
				if (!sx9324_get_software_default(dev, SX9324_GNRL_CTRL_0, &val))
					val = 0x40; /* Tdoze=8xTscan if no software default set */
				error |= regmap_update_bits(drv_data->regmap, SX9324_GNRL_CTRL_0,
					SX9324_DOZEPERIOD, val);
			}
			break;
		case SX9324_SLEEP:
			error = regmap_update_bits(drv_data->regmap, SX9324_GNRL_CTRL_1,
				SX9324_PHEN, 0);
			break;
		default:
			error = -EINVAL;
			break;
	}

	return error;
}

static int sx9324_reset(struct device *dev, enum sx9324_reset_source src)
{
	struct sx9324_data *drv_data =
		(struct sx9324_data *)i2c_get_clientdata(to_client(dev));
	unsigned int val;
	int ret;

	if (src == POWER_UP_RESET) {
		/* maximum power-up time */
		udelay(1000);
	} else if (src == SOFTWARE_RESET) {
		ret = regmap_write(drv_data->regmap, SX9324_RESET, 0xde);
		if (ret) {
			pr_err("failed to perform a software reset, err=%d\n", ret);
			return ret;
		}
	}

	if (0 == gpiod_get_value(drv_data->nirq_gpio)) {
		/* clear NIRQ status and the chip is ready for operation */
		ret = regmap_read(drv_data->regmap, SX9324_IRQ_SRC, &val);
		if (ret) {
			pr_err("chip is not ready for operation, err=%d", ret);
			return ret;
		} else {
			if (1 != gpiod_get_value(drv_data->nirq_gpio)) {
				pr_err("failed to reset the chip\n");
				return -ENODEV;
			}
		}
	} else {
		pr_err("failed to initialize the chip on reset\n");
		return -ENODEV;
	}

	return 0;
}

static bool sx9324_writeable_reg(struct device *dev, unsigned int reg)
{
	switch (reg) {
		case SX9324_IRQ_MSK ... SX9324_IRQ_CFG_2:
		case SX9324_GNRL_CTRL_0:
		case SX9324_GNRL_CTRL_1:
		case SX9324_I2C_ADDR:
		case SX9324_CLK_SPRD:
		case SX9324_AFE_CTRL_0:
		case SX9324_AFE_CTRL_3:
		case SX9324_AFE_CTRL_4:
		case SX9324_AFE_CTRL_6 ... SX9324_AFE_CTRL_9:
		case SX9324_PROX_CTRL_0 ... SX9324_PROX_CTRL_7:
		case SX9324_ADV_CTRL_0 ... SX9324_ADV_CTRL_20:
		case SX9324_PHASE_SEL:
		case SX9324_OFFSET_MSB:
		case SX9324_OFFSET_LSB:
		case SX9324_RESET:
			return true;
		default:
			return false;
	}
}

static bool sx9324_readable_reg(struct device *dev, unsigned int reg)
{
	switch (reg) {
		case SX9324_IRQ_SRC ... SX9324_IRQ_CFG_2:
		case SX9324_GNRL_CTRL_0:
		case SX9324_GNRL_CTRL_1:
		case SX9324_I2C_ADDR:
		case SX9324_CLK_SPRD:
		case SX9324_AFE_CTRL_0:
		case SX9324_AFE_CTRL_3:
		case SX9324_AFE_CTRL_4:
		case SX9324_AFE_CTRL_6 ... SX9324_AFE_CTRL_9:
		case SX9324_PROX_CTRL_0 ... SX9324_PROX_CTRL_7:
		case SX9324_ADV_CTRL_0 ... SX9324_ADV_CTRL_20:
		case SX9324_PHASE_SEL ... SX9324_SAR_LSB:
		case SX9324_WHO_AM_I:
		case SX9324_REV:
			return true;
		default:
			return false;
	}
}

static const struct regmap_config sx9324_regmap_config = {
	.reg_bits = 8,
	.val_bits = 8,
	.writeable_reg = sx9324_writeable_reg,
	.readable_reg = sx9324_readable_reg,
	.cache_type	= REGCACHE_NONE,
};

#define MAX_DUMPING_REGISTERS 8
#define REGISTER_UNSET_VALUE 0xff
static unsigned int dumping_regs[MAX_DUMPING_REGISTERS];

static ssize_t sx9324_registers_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct sx9324_data *drv_data =
		(struct sx9324_data *)i2c_get_clientdata(to_client(dev));
	unsigned int val;
	int written = 0;
	int i;
	int error;

	for (i = 0; i < MAX_DUMPING_REGISTERS; i++) {
		written += sprintf(buf + written, "0x%02x: ", dumping_regs[i]);
		if (dumping_regs[i] != REGISTER_UNSET_VALUE) {
			error = regmap_read(drv_data->regmap, dumping_regs[i], &val);
			if (error)
				pr_err("failed reading register '0x%02x', err=%d\n",
					dumping_regs[i], error);
			else
				written += sprintf(buf + written, "0x%02x", val);
		}
		written += sprintf(buf + written, "\n");
	}

	return written;
}

static ssize_t sx9324_registers_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	struct sx9324_data *drv_data =
		(struct sx9324_data *)i2c_get_clientdata(to_client(dev));
	unsigned int val;
	int i = 0;
	int arg_c = -1;
	struct _arg_str {
		int s;
		int e;
	} arg_v[MAX_DUMPING_REGISTERS];
	int j;
	char reg_str[32];
	char *reg_val;
	unsigned int reg_offset, write_value;
	int error;
	bool to_write;
	bool new_arg = false;

	pr_info("usage: echo \"[reg=[value]]...\" > registers\n");

	if (buf && count != 0) {
		/* skip leading space */
		while (buf[i] == ' ')
			i++;

		/* split arguments */
		while (buf[i] != '\0') {
			if (buf[i] == ' ') {
				arg_v[arg_c].e = i - 1;
				new_arg = false;
				while (buf[i + 1] == ' ')
					i++;
			} else {
				if (!new_arg) {
					if (arg_c + 1 >= MAX_DUMPING_REGISTERS)
						break;
					arg_c++;
					arg_v[arg_c].s = i;
					new_arg = true;
				}
			}
			i++;
		}
		/* the last argument which ends with null termination */
		if (new_arg) {
			arg_v[arg_c].e = i - 1;
		}

		/* store dumping registers and write registers if requested */
		for (j = 0; j <= arg_c; j++) {
			memset(reg_str, 0, sizeof(reg_str));
			strncpy(reg_str, buf + arg_v[j].s,
				min(arg_v[j].e - arg_v[j].s + 1, (int)sizeof(reg_str)));

			to_write = false;
			reg_val = strstr(reg_str, "=");
			if (reg_val) {
				error = kstrtouint(reg_val + 1, 16, &write_value);
				if (error == 0)
					to_write = true;
				*reg_val = '\0';
			}

			error = kstrtouint(reg_str, 16, &reg_offset);
			if (error == 0) {
				dumping_regs[j] = reg_offset;
				pr_info("register 0x%02x is stored to be dumped\n",
					dumping_regs[j]);
				if (to_write) {
					error = regmap_write(drv_data->regmap, dumping_regs[j],
						write_value);
					if (error)
						pr_err("failed writing register 0x%02x with value "
							"0x%02x, err=%d\n", dumping_regs[j], write_value,
							error);
					else
						pr_info("successfully wrote register 0x%02x with "
							"value 0x%02x\n", dumping_regs[j], write_value);
				}
			}
		}
	}

	return count;
}

static ssize_t sx9324_reset_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	sx9324_reset(dev, SOFTWARE_RESET);
	return 0;
}

static ssize_t sx9324_phdata_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct sx9324_data *drv_data =
		(struct sx9324_data *)i2c_get_clientdata(to_client(dev));
	struct sx9324_phase_data *phdata;
	int written = 0;
	int i;
	int error;

	written += sprintf(buf + written, "PH Useful Avg Diff Steady Prox Table Body Fail Comp\n");
	written += sprintf(buf + written, "===================================================\n");
	error = sx9324_read_phdata(dev);
	if (!error) {
		for (i = PH0; i < SX9324_PHASES; i++) {
			phdata = &drv_data->phdata[i];
			if (phdata->is_valid) {
				written += sprintf(buf + written, "%d %d %d %d %d %d %d %d %d %d\n", i,
					phdata->proxuseful, phdata->proxavg, phdata->proxdiff,
					phdata->stat.steady, phdata->stat.prox, phdata->stat.table,
					phdata->stat.body, phdata->stat.fail, phdata->stat.comp);
			}
		}
	}
	return written;
}

static ssize_t sx9324_mode_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct sx9324_data *drv_data =
		(struct sx9324_data *)i2c_get_clientdata(to_client(dev));
	enum sx9324_operational_mode mode;
	int written = 0;
	int error;

	error = sx9324_get_mode(dev, &mode);
	if (!error) {
		written += sprintf(buf + written, "%c active\n",
			mode == SX9324_ACTIVE ? 'v' : ' ');
		written += sprintf(buf + written, "%c doze\n",
			mode == SX9324_DOZE ? 'v' : ' ');
		written += sprintf(buf + written, "%c sleep\n",
			mode == SX9324_SLEEP ? 'v' : ' ');
	} else {
		*buf = '\0';
	}

	return written;
}

static ssize_t sx9324_mode_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	if (buf && count != 0) {
		if (0 == strncmp(buf, "active", 6))
			sx9324_set_mode(dev, SX9324_ACTIVE);
		else if (0 == strncmp(buf, "doze", 4))
			sx9324_set_mode(dev, SX9324_DOZE);
		else if (0 == strncmp(buf, "sleep", 5))
			sx9324_set_mode(dev, SX9324_SLEEP);
	}

	return count;
}

static struct device_attribute sx9324_attrs[] =
{
	__ATTR(registers, S_IWUSR | S_IRUGO, sx9324_registers_show,
		sx9324_registers_store),
	__ATTR(reset, S_IRUSR, sx9324_reset_show, NULL),
	__ATTR(phdata, S_IRUGO, sx9324_phdata_show, NULL),
	__ATTR(mode, S_IWUSR | S_IRUGO, sx9324_mode_show, sx9324_mode_store),
};

static int sx9324_create_sysfs_attr(struct device *dev)
{
	int i;
	int error;

	for (i = 0; i < ARRAY_SIZE(sx9324_attrs); i++) {
		error = device_create_file(dev, &sx9324_attrs[i]);
		if (error) {
			pr_err("failed creating device attribute files, err=%d\n", error);
			break;
		}
	}

	if (i < ARRAY_SIZE(sx9324_attrs)) {
		for (i--; i >= 0; i--)
			device_remove_file(dev, &sx9324_attrs[i]);
	}

	return error;
}

static void sx9324_remove_sysfs_attr(struct device *dev)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(sx9324_attrs); i++)
		device_remove_file(dev, &sx9324_attrs[i]);
}

static void sx9324_nirq_worker(struct work_struct *work) {
	struct sx9324_data *drv_data = container_of(work, struct sx9324_data,
		nirq_work);
	unsigned int val;
	int err;

	err = regmap_read(drv_data->regmap, SX9324_IRQ_SRC, &val);
	if (!err) {
		pr_debug("IRQ SRC: 0x%02x; Reset=%d, Close=%d, Far=%d\n", val,
			val & SX9324_RESETIRQ ? 1 : 0,
			val & SX9324_CLOSEANYIRQ ? 1 : 0,
			val & SX9324_FARANYIRQ ? 1 : 0);
	} else {
		pr_err("failed to read register (SX9324_IRQ_SRC), err=%d\n", err);
	}
}

static irqreturn_t sx9324_nirq_handler(int irq, void *p)
{
	struct sx9324_data *drv_data =
		(struct sx9324_data *)i2c_get_clientdata(p);
	queue_work(drv_data->workqueue, &drv_data->nirq_work);
	return IRQ_HANDLED;
}

static int sx9324_probe(struct i2c_client *client,
	const struct i2c_device_id *id)
{
	struct sx9324_data *drv_data;
	int error;
	int i;

	pr_info("probed i2c client '%s', functionality=0x%x\n", client->name,
		i2c_get_functionality(client->adapter));

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		pr_err("no specified i2c functionality\n");
		return -ENODEV;
	}

	drv_data = devm_kzalloc(&client->dev, sizeof(*drv_data), GFP_KERNEL);
	if (!drv_data) {
		pr_err("failed memory allocation\n");
		return -ENOMEM;
	}

	drv_data->client = client;
	i2c_set_clientdata(client, drv_data);

	drv_data->workqueue = create_singlethread_workqueue("workqueue");
	if (!drv_data->workqueue) {
		pr_err("failed to create the workqueue\n");
		return -ENOMEM;
	}
	INIT_WORK(&drv_data->nirq_work, sx9324_nirq_worker);

	drv_data->regmap = devm_regmap_init_i2c(client, &sx9324_regmap_config);
	if (IS_ERR(drv_data->regmap)) {
		error = PTR_ERR(drv_data->regmap);
		pr_err("failed to initialize regmap, err=%d\n", error);
		goto error_exit;
	}

	drv_data->nirq_gpio = devm_gpiod_get(&client->dev, "nirq",
		GPIOD_IN);
	if (IS_ERR(drv_data->nirq_gpio)) {
		error = PTR_ERR(drv_data->nirq_gpio);
		pr_err("failed to obtain the gpio for NIRQ, err=%d\n", error);
		goto error_exit;
	}

	drv_data->nirq = gpiod_to_irq(drv_data->nirq_gpio);
	if (drv_data->nirq < 0) {
		error = drv_data->nirq;
		pr_err("failed to retrieve irq corresponding to gpio-%d, err=%d\n",
			desc_to_gpio(drv_data->nirq_gpio), error);
		goto error_exit;
	}

	error = devm_request_any_context_irq(&client->dev, drv_data->nirq,
		sx9324_nirq_handler, IRQ_TYPE_EDGE_FALLING, dev_name(&client->dev),
		client);
	if (error < 0) {
		pr_err("failed to claim irq for gpio-%d, err=%d\n",
			desc_to_gpio(drv_data->nirq_gpio), error);
		goto error_exit;
	}

	drv_data->vdd = devm_regulator_get(&client->dev, "vdd");
	if (IS_ERR(drv_data->vdd)) {
		error = PTR_ERR(drv_data->vdd);
		pr_err("failed to obtain the vdd regulator, err=%d\n", error);
		goto error_exit;
	}

	drv_data->pullup = devm_regulator_get(&client->dev, "pullup");
	if (IS_ERR(drv_data->pullup)) {
		error = ERR_PTR(drv_data->pullup);
		pr_err("failed to obtain the pull-up regulator, err=%d\n", error);
		goto error_exit;
	}

	error = sx9324_enable_pullup(&client->dev, true);
	error |= sx9324_enable_vdd(&client->dev, true);
	if (error) {
		sx9324_enable_vdd(&client->dev, false);
		sx9324_enable_pullup(&client->dev, false);
		pr_err("failed to enable the power supply, err=%d\n", error);
		goto error_exit;
	}

	error = sx9324_reset(&client->dev, POWER_UP_RESET);
	if (error) {
		pr_err("failed to reset the chip on power-up, err=%d", error);
		goto error_exit;
	}

	error = sx9324_reset_software_default(&client->dev);
	if (error) {
		pr_err("failed to reset registers to software default, err=%d\n", error);
		goto error_exit;
	}

	error = sx9324_create_sysfs_attr(&client->dev);
	if (error) {
		pr_err("failed to create sysfs device attributes, err=%d\n", error);
		goto error_exit;
	}

	for (i = 0; i < MAX_DUMPING_REGISTERS; i++)
		dumping_regs[i] = REGISTER_UNSET_VALUE;

	return 0;

error_exit:
	destroy_workqueue(drv_data->workqueue);
	return error;
}

static int sx9324_remove(struct i2c_client *client)
{
	struct sx9324_data *drv_data =
		(struct sx9324_data *)i2c_get_clientdata(client);
	sx9324_remove_sysfs_attr(&client->dev);
	sx9324_enable_vdd(&client->dev, false);
	sx9324_enable_pullup(&client->dev, false);
	destroy_workqueue(drv_data->workqueue);
	return 0;
}

static int sx9324_suspend(struct device *dev)
{
	return 0;
}

static int sx9324_resume(struct device *dev)
{
	return 0;
}

static SIMPLE_DEV_PM_OPS(sx9324_pm_ops, sx9324_suspend, sx9324_resume);

static const struct i2c_device_id sx9324_id[] = {
	{ "sx9324", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, sx9324_id);

static const struct of_device_id sx9324_of_match[] = {
	{ .compatible = "semtech,sx9324", },
	{ }
};
MODULE_DEVICE_TABLE(of, sx9324_of_match);

static struct i2c_driver sx9324_driver = {
	.probe = sx9324_probe,
	.remove = sx9324_remove,
	.driver = {
		.name = DRIVER_NAME,
		.of_match_table = of_match_ptr(sx9324_of_match),
		.pm = &sx9324_pm_ops,
	},
	.id_table = sx9324_id,
};
module_i2c_driver(sx9324_driver);

MODULE_AUTHOR("Hsinko Yu <hsinkoyu@fih-foxconn.com>");
MODULE_DESCRIPTION("Driver for Semtech SX9324");
MODULE_LICENSE("GPL v2");
