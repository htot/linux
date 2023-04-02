/*
 * Intel MRFLD Whiskey Cove PMIC I2C Master driver
 * Copyright (C) 2017 Hans de Goede <hdegoede@redhat.com>
 *
 * Based on various non upstream patches to support the MRFLD Whiskey Cove PMIC:
 * Copyright (C) 2011 - 2014 Intel Corporation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License version
 * 2 as published by the Free Software Foundation, or (at your option)
 * any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/completion.h>
#include <linux/delay.h>
#include <linux/i2c.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/irqdomain.h>
#include <linux/mfd/intel_soc_pmic.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/slab.h>

#define MRFLD_BC_I2C_CTRL		0x58
#define MRFLD_BC_I2C_CTRL_WR		BIT(0)
#define MRFLD_BC_I2C_CTRL_RD		BIT(1)
#define MRFLD_BC_I2C_CLIENT_ADDR	0x59
#define MRFLD_BC_I2C_REG_OFFSET		0x5a
#define MRFLD_BC_I2C_WRDATA		0x5b
#define MRFLD_BC_I2C_RDDATA		0x5c

#define MRFLD_BC_EXTCHGRIRQ_CLIENT_IRQ	BIT(0)
#define MRFLD_BC_EXTCHGRIRQ_WRITE_IRQ	BIT(1)
#define MRFLD_BC_EXTCHGRIRQ_READ_IRQ	BIT(2)
#define MRFLD_BC_EXTCHGRIRQ_NACK_IRQ	BIT(3)
#define MRFLD_BC_EXTCHGRIRQ_ADAP_IRQMASK	((u8)GENMASK(3, 1))

struct mrfld_bc_i2c_adap {
	struct i2c_adapter adapter;
	wait_queue_head_t wait;
	struct mutex irqchip_lock;
	struct regmap *regmap;
	struct i2c_client *client;
	u8 irq_mask;
	u8 old_irq_mask;
	bool nack;
	bool done;
};

static irqreturn_t mrfld_bc_i2c_adap_thread_handler(int id, void *data)
{
	struct mrfld_bc_i2c_adap *adap = data;
	int ret, reg;

	/* Read IRQs */
	ret = regmap_read(adap->regmap, CHT_BC_EXTCHGRIRQ, &reg);
	if (ret) {
		dev_err(&adap->adapter.dev, "Error reading extchgrirq reg\n");
		return IRQ_NONE;
	}

	reg &= ~adap->irq_mask;

	/*
	 * Immediately ack IRQs, so that if new IRQs arrives while we're
	 * handling the previous ones our irq will re-trigger when we're done.
	 */
	ret = regmap_write(adap->regmap, CHT_BC_EXTCHGRIRQ, reg);
	if (ret)
		dev_err(&adap->adapter.dev, "Error writing extchgrirq reg\n");

	regmap_update_bits(pmic->regmap, BCOVE_MIRQLVL1, BCOVE_LVL1_CHGR, 0);

	if (reg & MRFLD_BC_EXTCHGRIRQ_ADAP_IRQMASK) {
		adap->nack = !!(reg & MRFLD_BC_EXTCHGRIRQ_NACK_IRQ);
		adap->done = true;
		wake_up(&adap->wait);
	}

	return IRQ_HANDLED;
}

static u32 mrfld_bc_i2c_adap_master_func(struct i2c_adapter *adap)
{
	/* This i2c adapter only supports SMBUS byte transfers */
	return I2C_FUNC_SMBUS_BYTE_DATA;
}

static int mrfld_bc_i2c_adap_smbus_xfer(struct i2c_adapter *_adap, u16 addr,
				      unsigned short flags, char read_write,
				      u8 command, int size,
				      union i2c_smbus_data *data)
{
	struct mrfld_bc_i2c_adap *adap = i2c_get_adapdata(_adap);
	int ret, reg;

	adap->nack = false;
	adap->done = false;

	ret = regmap_write(adap->regmap, MRFLD_BC_I2C_CLIENT_ADDR, addr);
	if (ret)
		return ret;

	if (read_write == I2C_SMBUS_WRITE) {
		ret = regmap_write(adap->regmap, MRFLD_BC_I2C_WRDATA, data->byte);
		if (ret)
			return ret;
	}

	ret = regmap_write(adap->regmap, MRFLD_BC_I2C_REG_OFFSET, command);
	if (ret)
		return ret;

	ret = regmap_write(adap->regmap, MRFLD_BC_I2C_CTRL,
			   (read_write == I2C_SMBUS_WRITE) ?
			   MRFLD_BC_I2C_CTRL_WR : MRFLD_BC_I2C_CTRL_RD);
	if (ret)
		return ret;

	/* 3 second timeout, during cable plug the PMIC responds quite slow */
	ret = wait_event_timeout(adap->wait, adap->done, 3 * HZ);
	if (ret == 0)
		return -ETIMEDOUT;
	if (adap->nack)
		return -EIO;

	if (read_write == I2C_SMBUS_READ) {
		ret = regmap_read(adap->regmap, MRFLD_BC_I2C_RDDATA, &reg);
		if (ret)
			return ret;

		data->byte = reg;
	}

	return 0;
}

static const struct i2c_algorithm mrfld_bc_i2c_adap_algo = {
	.functionality = mrfld_bc_i2c_adap_master_func,
	.smbus_xfer = mrfld_bc_i2c_adap_smbus_xfer,
};

static const struct property_entry bq24261_props[] = {
	PROPERTY_ENTRY_STRING("extcon-name", "mrfld_bcove_pwrsrc"),
	PROPERTY_ENTRY_BOOL("omit-battery-class"),
	PROPERTY_ENTRY_BOOL("disable-reset"),
	{ }
};

static const struct software_node bq24261_node = {
	.properties = bq24261_props,
};

static int mrfld_bc_i2c_adap_i2c_probe(struct platform_device *pdev)
{
	struct intel_soc_pmic *pmic = dev_get_drvdata(pdev->dev.parent);
	struct mrfld_bc_i2c_adap *adap;
	struct i2c_board_info board_info = {
		.type = "bq24261",
		.addr = 0x6b,
		.swnode = &bq24261_node,
	};
	int ret, irq;

	irq = platform_get_irq(pdev, 0);
	if (irq < 0) {
		dev_err(&pdev->dev, "Error missing irq resource\n");
		return -EINVAL;
	}

	adap = devm_kzalloc(&pdev->dev, sizeof(*adap), GFP_KERNEL);
	if (!adap)
		return -ENOMEM;

	init_waitqueue_head(&adap->wait);
	mutex_init(&adap->irqchip_lock);
	adap->regmap = pmic->regmap;
	adap->adapter.owner = THIS_MODULE;
	adap->adapter.class = I2C_CLASS_HWMON;
	adap->adapter.algo = &mrfld_bc_i2c_adap_algo;
	strlcpy(adap->adapter.name, "PMIC I2C Adapter",
		sizeof(adap->adapter.name));
	adap->adapter.dev.parent = &pdev->dev;

	/* Clear and activate i2c-adapter interrupts, disable client IRQ */
	adap->old_irq_mask = adap->irq_mask = MRFLD_BC_EXTCHGRIRQ_ADAP_IRQMASK;
	regmap_update_bits(pmic->regmap, BCOVE_MIRQLVL1, BCOVE_LVL1_CHGR, 0);
	regmap_update_bits(pmic->regmap, BCOVE_MCHGRIRQ0, adap->irq_mask, 0);

	ret = devm_request_threaded_irq(&pdev->dev, irq, NULL,
					mrfld_bc_i2c_adap_thread_handler,
					IRQF_ONESHOT, "PMIC I2C Adapter", adap);
	if (ret)
		return ret;

	i2c_set_adapdata(&adap->adapter, adap);
	ret = i2c_add_adapter(&adap->adapter);
	if (ret)
		return ret;

	adap->client = i2c_new_client_device(&adap->adapter, &board_info);
	if (IS_ERR(adap->client)) {
		ret = PTR_ERR(adap->client);
		goto del_adapter;
	}

	platform_set_drvdata(pdev, adap);
	return 0;

del_adapter:
	i2c_del_adapter(&adap->adapter);
	return ret;
}

static int mrfld_bc_i2c_adap_i2c_remove(struct platform_device *pdev)
{
	struct mrfld_bc_i2c_adap *adap = platform_get_drvdata(pdev);

	i2c_unregister_device(adap->client);
	i2c_del_adapter(&adap->adapter);

	return 0;
}

static struct platform_device_id mrfld_bc_i2c_adap_id_table[] = {
	{ .name = "mrfld_bcove_charger" },
	{},
};
MODULE_DEVICE_TABLE(platform, mrfld_bc_i2c_adap_id_table);

struct platform_driver mrfld_bc_i2c_adap_driver = {
	.probe = mrfld_bc_i2c_adap_i2c_probe,
	.remove = mrfld_bc_i2c_adap_i2c_remove,
	.driver = {
		.name = "mrfld_bcove_charger",
	},
	.id_table = mrfld_bc_i2c_adap_id_table,
};
module_platform_driver(mrfld_bc_i2c_adap_driver);

MODULE_DESCRIPTION("Intel MRFLD Basin Cove PMIC I2C Master driver");
MODULE_AUTHOR("Ferry Toth <ftoth@exalondelft.nl>");
MODULE_LICENSE("GPL");
