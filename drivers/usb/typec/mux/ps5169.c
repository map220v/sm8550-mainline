// SPDX-License-Identifier: GPL-2.0-only
/*
 * Parade PS5169 Type-C driver
 *
 * Copyright (c) 2025 map220v <map220v300@gmail.com>
 */

#include <drm/bridge/aux-bridge.h>
#include <linux/i2c.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of_graph.h>
#include <linux/regmap.h>
#include <linux/regulator/consumer.h>
#include <linux/usb/typec_dp.h>
#include <linux/usb/typec_mux.h>
#include <linux/usb/typec_retimer.h>

#define PS5169_CHIP_ID_REG		0xac
#define PS5169_CHIP_ID			0x6987

struct ps5169 {
	struct i2c_client *client;
	struct regulator *dvdd_supply;
	struct gpio_desc *reset_gpio;
	struct regmap *regmap;
	struct typec_switch_dev *sw;
	struct typec_retimer *retimer;

	struct typec_switch *typec_switch;
	struct typec_mux *typec_mux;

	struct mutex lock; /* protect non-concurrent retimer & switch */

	enum typec_orientation orientation;
	unsigned long mode;
	unsigned int svid;
};

static int ps5169_set(struct ps5169 *ps5169)
{
	bool reverse = (ps5169->orientation == TYPEC_ORIENTATION_REVERSE);

	switch (ps5169->mode) {
	case TYPEC_STATE_SAFE:
		/* regmap_write(ps5169->regmap, 0x04, 0x00); */
		regmap_write(ps5169->regmap, 0x40, 0x80);
		regmap_write(ps5169->regmap, 0xa0, 0x02);
		regmap_write(ps5169->regmap, 0xa1, 0x00);

		return 0;

	case TYPEC_STATE_USB:
		/*
		 * Normal Orientation (CC1)
		 * A -> USB RX
		 * B -> USB TX
		 * C -> X
		 * D -> X
		 * Flipped Orientation (CC2)
		 * A -> X
		 * B -> X
		 * C -> USB TX
		 * D -> USB RX
		 */

		/* USB 3.2 Gen 2 only */
		if (!reverse)
			regmap_write(ps5169->regmap, 0x40, 0xc0);
		else
			regmap_write(ps5169->regmap, 0x40, 0xd0);

		return 0;

	default:
		if (ps5169->svid != USB_TYPEC_DP_SID)
			return -EINVAL;

		break;
	}

	/* DP Altmode Setup */
	switch (ps5169->mode) {
	case TYPEC_DP_STATE_C:
	case TYPEC_DP_STATE_E:
		/*
		 * Normal Orientation (CC1)
		 * A -> DP3
		 * B -> DP2
		 * C -> DP1
		 * D -> DP0
		 * Flipped Orientation (CC2)
		 * A -> DP0
		 * B -> DP1
		 * C -> DP2
		 * D -> DP3
		 */

		/* 4-lane DP */
		if (!reverse)
			regmap_write(ps5169->regmap, 0x40, 0xa0);
		else
			regmap_write(ps5169->regmap, 0x40, 0xb0);

		regmap_write(ps5169->regmap, 0xa0, 0x00);
		regmap_write(ps5169->regmap, 0xa1, 0x04);
		break;

	case TYPEC_DP_STATE_D:
	case TYPEC_DP_STATE_F: /* State F is deprecated */
		/*
		 * Normal Orientation (CC1)
		 * A -> USB RX
		 * B -> USB TX
		 * C -> DP1
		 * D -> DP0
		 * Flipped Orientation (CC2)
		 * A -> DP0
		 * B -> DP1
		 * C -> USB TX
		 * D -> USB RX
		 */

		/* USB 3.2 Gen 2 and 2-lane DP */
		if (!reverse)
			regmap_write(ps5169->regmap, 0x40, 0xe0);
		else
			regmap_write(ps5169->regmap, 0x40, 0xf0);

		regmap_write(ps5169->regmap, 0xa0, 0x00);
		regmap_write(ps5169->regmap, 0xa1, 0x04);
		break;

	default:
		return -EOPNOTSUPP;
	}

	return 0;
}

static int ps5169_sw_set(struct typec_switch_dev *sw, enum typec_orientation orientation)
{
	struct ps5169 *ps5169 = typec_switch_get_drvdata(sw);
	int ret;

	ret = typec_switch_set(ps5169->typec_switch, orientation);
	if (ret)
		return ret;

	mutex_lock(&ps5169->lock);

	if (ps5169->orientation != orientation) {
		ps5169->orientation = orientation;

		ret = ps5169_set(ps5169);
	}

	mutex_unlock(&ps5169->lock);

	return ret;
}

static int ps5169_retimer_set(struct typec_retimer *retimer, struct typec_retimer_state *state)
{
	struct ps5169 *ps5169 = typec_retimer_get_drvdata(retimer);
	struct typec_mux_state mux_state;
	int ret = 0;

	mutex_lock(&ps5169->lock);

	if (ps5169->mode != state->mode) {
		ps5169->mode = state->mode;

		if (state->alt)
			ps5169->svid = state->alt->svid;
		else
			ps5169->svid = 0;

		ret = ps5169_set(ps5169);
	}

	mutex_unlock(&ps5169->lock);

	if (ret)
		return ret;

	mux_state.alt = state->alt;
	mux_state.data = state->data;
	mux_state.mode = state->mode;

	return typec_mux_set(ps5169->typec_mux, &mux_state);
}

static int ps5169_detect(struct ps5169 *ps5169)
{
	struct device *dev = &ps5169->client->dev;
	u32 reg_val;
	int ret;

	ret = regmap_raw_read(ps5169->regmap, PS5169_CHIP_ID_REG, &reg_val, 2);
	if (ret < 0)
		return dev_err_probe(dev, ret, "failed to read chip ID\n");

	if (reg_val != PS5169_CHIP_ID)
		return dev_err_probe(dev, -ENODEV, "unexpected chip ID: (0x%04x)\n", reg_val);

	return 0;
}

static int ps5169_configure(struct ps5169 *ps5169)
{
	regmap_write(ps5169->regmap, 0x9d, 0x80);
	usleep_range(10000, 10100);
	regmap_write(ps5169->regmap, 0x9d, 0x00);
	regmap_write(ps5169->regmap, 0x40, 0x80);

	regmap_write(ps5169->regmap, 0xa0, 0x02);
	regmap_write(ps5169->regmap, 0x8d, 0x01);
	regmap_write(ps5169->regmap, 0x90, 0x01);

	regmap_write(ps5169->regmap, 0x51, 0x87);
	regmap_write(ps5169->regmap, 0x50, 0x20);
	regmap_write(ps5169->regmap, 0x54, 0x11);
	regmap_write(ps5169->regmap, 0x5d, 0x66);
	regmap_write(ps5169->regmap, 0x52, 0x50);
	regmap_write(ps5169->regmap, 0x55, 0x00);
	regmap_write(ps5169->regmap, 0x56, 0x00);
	regmap_write(ps5169->regmap, 0x57, 0x00);
	regmap_write(ps5169->regmap, 0x58, 0x00);
	regmap_write(ps5169->regmap, 0x59, 0x00);
	regmap_write(ps5169->regmap, 0x5a, 0x00);
	regmap_write(ps5169->regmap, 0x5b, 0x00);
	regmap_write(ps5169->regmap, 0x5e, 0x06);
	regmap_write(ps5169->regmap, 0x5f, 0x00);
	regmap_write(ps5169->regmap, 0x60, 0x00);
	regmap_write(ps5169->regmap, 0x61, 0x03);
	regmap_write(ps5169->regmap, 0x65, 0x40);
	regmap_write(ps5169->regmap, 0x66, 0x00);
	regmap_write(ps5169->regmap, 0x67, 0x03);
	regmap_write(ps5169->regmap, 0x75, 0x0c);
	regmap_write(ps5169->regmap, 0x77, 0x00);
	regmap_write(ps5169->regmap, 0x78, 0x7c);

	return 0;
}

static const struct regmap_config ps5169_regmap = {
	.reg_bits = 8,
	.val_bits = 8,
	.max_register = 0xff,
};

static int ps5169_probe(struct i2c_client *client)
{
	struct device *dev = &client->dev;
	struct typec_switch_desc sw_desc = { };
	struct typec_retimer_desc retimer_desc = { };
	struct ps5169 *ps5169;
	int ret;

	ps5169 = devm_kzalloc(dev, sizeof(*ps5169), GFP_KERNEL);
	if (!ps5169)
		return -ENOMEM;

	ps5169->client = client;

	mutex_init(&ps5169->lock);

	ps5169->regmap = devm_regmap_init_i2c(client, &ps5169_regmap);
	if (IS_ERR(ps5169->regmap))
		return dev_err_probe(dev, PTR_ERR(ps5169->regmap),
				     "failed to allocate register map\n");

	ps5169->dvdd_supply = devm_regulator_get(dev, "dvdd");
	if (IS_ERR(ps5169->dvdd_supply))
		return dev_err_probe(dev, PTR_ERR(ps5169->dvdd_supply),
				     "failed to get dvdd regulator\n");

	ps5169->reset_gpio = devm_gpiod_get(dev, "reset", GPIOD_OUT_LOW);
	if (IS_ERR(ps5169->reset_gpio))
		return dev_err_probe(dev, PTR_ERR(ps5169->reset_gpio),
				     "failed to get reset gpio\n");

	ps5169->typec_switch = typec_switch_get(dev);
	if (IS_ERR(ps5169->typec_switch))
		return dev_err_probe(dev, PTR_ERR(ps5169->typec_switch),
				     "failed to acquire orientation-switch\n");

	ps5169->typec_mux = typec_mux_get(dev);
	if (IS_ERR(ps5169->typec_mux)) {
		ret = dev_err_probe(dev, PTR_ERR(ps5169->typec_mux),
				    "failed to acquire mode-switch\n");
		goto err_switch_put;
	}

	ret = regulator_enable(ps5169->dvdd_supply);
	if (ret) {
		ret = dev_err_probe(dev, ret, "failed to enable dvdd regulator\n");
		goto err_mux_put;
	}

	gpiod_set_value(ps5169->reset_gpio, 0);

	msleep(20);

	ret = ps5169_detect(ps5169);
	if (ret)
		goto err_disable_regulator;

	ret = ps5169_configure(ps5169);
	if (ret)
		goto err_disable_regulator;

	ret = drm_aux_bridge_register(dev);
	if (ret)
		goto err_disable_regulator;

	sw_desc.drvdata = ps5169;
	sw_desc.fwnode = dev_fwnode(dev);
	sw_desc.set = ps5169_sw_set;

	ps5169->sw = typec_switch_register(dev, &sw_desc);
	if (IS_ERR(ps5169->sw)) {
		ret = dev_err_probe(dev, PTR_ERR(ps5169->sw),
				    "failed to register typec switch\n");
		goto err_disable_regulator;
	}

	retimer_desc.drvdata = ps5169;
	retimer_desc.fwnode = dev_fwnode(dev);
	retimer_desc.set = ps5169_retimer_set;

	ps5169->retimer = typec_retimer_register(dev, &retimer_desc);
	if (IS_ERR(ps5169->retimer)) {
		ret = dev_err_probe(dev, PTR_ERR(ps5169->retimer),
				    "failed to register typec retimer\n");
		goto err_switch_unregister;
	}

	return 0;

err_switch_unregister:
	typec_switch_unregister(ps5169->sw);
err_disable_regulator:
	gpiod_set_value(ps5169->reset_gpio, 1);
	regulator_disable(ps5169->dvdd_supply);
err_mux_put:
	typec_mux_put(ps5169->typec_mux);
err_switch_put:
	typec_switch_put(ps5169->typec_switch);

	return ret;
}

static void ps5169_remove(struct i2c_client *client)
{
	struct ps5169 *ps5169 = i2c_get_clientdata(client);

	typec_retimer_unregister(ps5169->retimer);
	typec_switch_unregister(ps5169->sw);

	gpiod_set_value(ps5169->reset_gpio, 1);

	regulator_disable(ps5169->dvdd_supply);

	typec_mux_put(ps5169->typec_mux);
	typec_switch_put(ps5169->typec_switch);
}

static const struct of_device_id ps5169_of_table[] = {
	{ .compatible = "parade,ps5169" },
	{ }
};
MODULE_DEVICE_TABLE(of, ps5169_of_table);

static struct i2c_driver ps5169_driver = {
	.driver = {
		.name = "ps5169",
		.of_match_table = ps5169_of_table,
	},
	.probe		= ps5169_probe,
	.remove		= ps5169_remove,
};
module_i2c_driver(ps5169_driver);

MODULE_AUTHOR("map220v <map220v300@gmail.com>");
MODULE_DESCRIPTION("Parade PS5169 Type-C driver");
MODULE_LICENSE("GPL v2");
