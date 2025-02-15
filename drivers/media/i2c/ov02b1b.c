// SPDX-License-Identifier: GPL-2.0-only
/*
 * Omnivision OV02B1B Camera Sensor driver
 *
 * Copyright (c) 2025 map220v <map220v300@gmail.com>
 */

#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/gpio/consumer.h>
#include <linux/i2c.h>
#include <linux/module.h>
#include <linux/pm_runtime.h>
#include <linux/regulator/consumer.h>
#include <linux/units.h>
#include <media/media-entity.h>

#include <media/v4l2-async.h>
#include <media/v4l2-cci.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-fwnode.h>
#include <media/v4l2-subdev.h>

#define OV02B1B_ID			0x2B

#define OV02B1B_REG_CHIP_ID		CCI_REG16(0x02)

#define OV02B1B_REG_EXPO		CCI_REG16(0x0e)
#define OV02B1B_EXPOSURE_MIN		4
#define OV02B1B_EXPOSURE_MAX_MARGIN	7
#define OV02B1B_EXPOSURE_DEFAULT	768

#define OV02B1B_REG_GAIN		CCI_REG8(0x22)
/* Min 1.0, Max 16.0, Mul 15.5 */
#define OV02B1B_ANA_GAIN_MIN		16
#define OV02B1B_ANA_GAIN_MAX		248
#define OV02B1B_ANA_GAIN_DEFAULT	124

#define OV02B1B_REG_VBLANK		CCI_REG16(0x14)
#define OV02B1B_VBLANK_MAX		0xffff

#define OV02B1B_REG_TEST_PATTERN	CCI_REG8(0x63)

#define OV02B1B_NATIVE_WIDTH		1608U
#define OV02B1B_NATIVE_HEIGHT		1208U
#define OV02B1B_PIXEL_ARRAY_LEFT	4U
#define OV02B1B_PIXEL_ARRAY_TOP		4U
#define OV02B1B_PIXEL_ARRAY_WIDTH	1600U
#define OV02B1B_PIXEL_ARRAY_HEIGHT	1200U

#define OV02B1B_DATA_LANES		1

#define OV02B1B_BITS_PER_SAMPLE		10

static const char * const ov02b1b_supply_names[] = {
	"vddio",	/* I/O power supply (1.8V) */
	"avdd",		/* Analog power supply (2.8V) */
};

struct ov02b1b_reg_list {
	u32 num_of_regs;
	const struct cci_reg_sequence *regs;
};

struct ov02b1b_mode {
	u32 width;
	u32 height;
	struct v4l2_rect crop;
	u32 line_length;
	u32 frame_length;
	u32 vblank_min;
	const struct ov02b1b_reg_list reg_list;
};

struct ov02b1b {
	u32 mclk_freq;

	struct device *dev;
	struct clk *mclk;
	struct gpio_desc *reset_gpio;
	struct regulator_bulk_data supplies[ARRAY_SIZE(ov02b1b_supply_names)];
	struct regmap *regmap;

	bool streaming;

	/*
	 * Serialize control access, get/set format, get selection
	 * and start streaming.
	 */
	struct mutex mutex;
	struct v4l2_subdev subdev;
	struct media_pad pad;
	struct v4l2_ctrl_handler ctrl_handler;

	struct v4l2_ctrl *vblank;
	struct v4l2_ctrl *hblank;
	struct v4l2_ctrl *exposure;

	const struct ov02b1b_mode *cur_mode;
};

static inline struct ov02b1b *to_ov02b1b(struct v4l2_subdev *sd)
{
	return container_of(sd, struct ov02b1b, subdev);
}

static const struct cci_reg_sequence ov02b1b_init_regs[] = {
	{ CCI_REG8(0xfd), 0x00 },
	{ CCI_REG8(0xfc), 0x01 },
	{ CCI_REG8(0xfd), 0x00 },
	{ CCI_REG8(0x24), 0x03 },
	{ CCI_REG8(0x25), 0x0c },
	{ CCI_REG8(0x29), 0x02 },
	{ CCI_REG8(0x2a), 0x31 },
	{ CCI_REG8(0x2b), 0x05 },
	{ CCI_REG8(0x1e), 0x17 },
	{ CCI_REG8(0x33), 0x07 },
	{ CCI_REG8(0x35), 0x07 },
	{ CCI_REG8(0x4a), 0x0c },
	{ CCI_REG8(0x3a), 0x05 },
	{ CCI_REG8(0x3b), 0x02 },
	{ CCI_REG8(0x3e), 0x00 },
	{ CCI_REG8(0x46), 0x01 },
	{ CCI_REG8(0x6d), 0x03 },
	{ CCI_REG8(0xfd), 0x01 },
	{ CCI_REG8(0x0e), 0x02 },
	{ CCI_REG8(0x0f), 0x1a },
	{ CCI_REG8(0x18), 0x00 },
	{ CCI_REG8(0x22), 0xff },
	{ CCI_REG8(0x23), 0x02 },
	{ CCI_REG8(0x17), 0x2c },
	{ CCI_REG8(0x19), 0x20 },
	{ CCI_REG8(0x1b), 0x06 },
	{ CCI_REG8(0x1c), 0x04 },
	{ CCI_REG8(0x20), 0x03 },
	{ CCI_REG8(0x30), 0x01 },
	{ CCI_REG8(0x33), 0x01 },
	{ CCI_REG8(0x31), 0x0a },
	{ CCI_REG8(0x32), 0x09 },
	{ CCI_REG8(0x38), 0x01 },
	{ CCI_REG8(0x39), 0x01 },
	{ CCI_REG8(0x3a), 0x01 },
	{ CCI_REG8(0x3b), 0x01 },
	{ CCI_REG8(0x4f), 0x04 },
	{ CCI_REG8(0x4e), 0x05 },
	{ CCI_REG8(0x50), 0x01 },
	{ CCI_REG8(0x35), 0x0c },
	{ CCI_REG8(0x45), 0x2a },
	{ CCI_REG8(0x46), 0x2a },
	{ CCI_REG8(0x47), 0x2a },
	{ CCI_REG8(0x48), 0x2a },
	{ CCI_REG8(0x4a), 0x2c },
	{ CCI_REG8(0x4b), 0x2c },
	{ CCI_REG8(0x4c), 0x2c },
	{ CCI_REG8(0x4d), 0x2c },
	{ CCI_REG8(0x56), 0x3a },
	{ CCI_REG8(0x57), 0x0a },
	{ CCI_REG8(0x58), 0x24 },
	{ CCI_REG8(0x59), 0x20 },
	{ CCI_REG8(0x5a), 0x0a },
	{ CCI_REG8(0x5b), 0xff },
	{ CCI_REG8(0x37), 0x0a },
	{ CCI_REG8(0x42), 0x0e },
	{ CCI_REG8(0x68), 0x90 },
	{ CCI_REG8(0x69), 0xcd },
	{ CCI_REG8(0x6a), 0x8f },
	{ CCI_REG8(0x7c), 0x0a },
	{ CCI_REG8(0x7d), 0x0a },
	{ CCI_REG8(0x7e), 0x0a },
	{ CCI_REG8(0x7f), 0x08 },
	{ CCI_REG8(0x83), 0x14 },
	{ CCI_REG8(0x84), 0x14 },
	{ CCI_REG8(0x86), 0x14 },
	{ CCI_REG8(0x87), 0x07 },
	{ CCI_REG8(0x88), 0x0f },
	{ CCI_REG8(0x94), 0x02 },
	{ CCI_REG8(0x98), 0xd1 },
	{ CCI_REG8(0xfe), 0x02 },
	{ CCI_REG8(0xfd), 0x03 },
	{ CCI_REG8(0x97), 0x6c },
	{ CCI_REG8(0x98), 0x60 },
	{ CCI_REG8(0x99), 0x60 },
	{ CCI_REG8(0x9a), 0x6c },
	{ CCI_REG8(0xa1), 0x40 },
	{ CCI_REG8(0xaf), 0x04 },
	{ CCI_REG8(0xb1), 0x40 },
	{ CCI_REG8(0xae), 0x0d },
	{ CCI_REG8(0x88), 0x5b },
	{ CCI_REG8(0x89), 0x7c },
	{ CCI_REG8(0xb4), 0x05 },
	{ CCI_REG8(0x8c), 0x40 },
	{ CCI_REG8(0x8e), 0x40 },
	{ CCI_REG8(0x90), 0x40 },
	{ CCI_REG8(0x92), 0x40 },
	{ CCI_REG8(0x9b), 0x46 },
	{ CCI_REG8(0xac), 0x40 },
	{ CCI_REG8(0xfd), 0x00 },
	{ CCI_REG8(0x5a), 0x15 },
	{ CCI_REG8(0x74), 0x01 },
};

static const struct cci_reg_sequence ov02b1b_1600x1200_regs[] = {
	{ CCI_REG8(0xfd), 0x00 },
	{ CCI_REG8(0x28), 0x00 },
	{ CCI_REG16(0x4f), 0x0640 },
	{ CCI_REG16(0x51), 0x04b0 },
	{ CCI_REG8(0xfd), 0x01 },
	{ CCI_REG16(0x02), 0x0070 },
	{ CCI_REG16(0x04), 0x0010 },
	{ CCI_REG16(0x06), 0x0320 },
	{ CCI_REG16(0x08), 0x04b0 },
	{ CCI_REG16(0x16), 0x002c },
	{ CCI_REG8(0x6c), 0x00 },
	{ CCI_REG8(0xfe), 0x02 },
	{ CCI_REG8(0xfd), 0x03 },
	{ CCI_REG8(0xfb), 0x01 },
};

static const struct cci_reg_sequence ov02b1b_streamon_regs[] = {
	{ CCI_REG8(0xfd), 0x03 },
	{ CCI_REG8(0xc2), 0x01 },
};

static const struct cci_reg_sequence ov02b1b_streamoff_regs[] = {
	{ CCI_REG8(0xfd), 0x03 },
	{ CCI_REG8(0xc2), 0x00 },
};

static const char * const ov02b1b_test_pattern_menu[] = {
	"Disabled",
	"Eight Vertical Colour Bars",
};

static const s64 link_freq_menu_items[] = {
	332800000,
};

static u64 to_pixel_rate(u32 f_index)
{
	u64 pixel_rate = link_freq_menu_items[f_index] * 2 * OV02B1B_DATA_LANES;

	do_div(pixel_rate, OV02B1B_BITS_PER_SAMPLE);

	return pixel_rate;
}

static const struct ov02b1b_mode supported_modes[] = {
	/* 1200p 4:3 30fps */
	{
		.width = 1600,
		.height = 1200,
		.crop = {
			.left = OV02B1B_PIXEL_ARRAY_LEFT,
			.top = OV02B1B_PIXEL_ARRAY_TOP,
			.width = 1600,
			.height = 1200,
		},
		.line_length = 448 * 4,
		.frame_length = 1238,
		.vblank_min = 20,
		.reg_list = {
			.num_of_regs = ARRAY_SIZE(ov02b1b_1600x1200_regs),
			.regs = ov02b1b_1600x1200_regs,
		},
	},
};

static int ov02b1b_check_hwcfg(struct device *dev, struct ov02b1b *ov02b1b)
{
	struct fwnode_handle *ep;
	struct fwnode_handle *fwnode = dev_fwnode(dev);
	struct v4l2_fwnode_endpoint bus_cfg = {
		.bus_type = V4L2_MBUS_CSI2_DPHY,
	};
	unsigned int i, j;
	int ret;

	if (!fwnode)
		return -EINVAL;

	ep = fwnode_graph_get_next_endpoint(fwnode, NULL);
	if (!ep)
		return -ENXIO;

	ret = v4l2_fwnode_endpoint_alloc_parse(ep, &bus_cfg);
	fwnode_handle_put(ep);
	if (ret)
		return ret;

	for (i = 0; i < ARRAY_SIZE(link_freq_menu_items); i++) {
		for (j = 0; j < bus_cfg.nr_of_link_frequencies; j++) {
			if (link_freq_menu_items[i] ==
				bus_cfg.link_frequencies[j])
				break;
		}

		if (j == bus_cfg.nr_of_link_frequencies) {
			dev_err(dev, "no link frequency %lld supported\n",
				link_freq_menu_items[i]);
			ret = -EINVAL;
			break;
		}
	}

	v4l2_fwnode_endpoint_free(&bus_cfg);

	return ret;
}

static int __ov02b1b_start_stream(struct ov02b1b *ov02b1b)
{
	const struct ov02b1b_reg_list *reg_list;
	int ret;

	/* Apply default values of current mode */
	reg_list = &ov02b1b->cur_mode->reg_list;

	ret = cci_multi_reg_write(ov02b1b->regmap, ov02b1b_init_regs, ARRAY_SIZE(ov02b1b_init_regs), NULL);
	if (ret)
		return ret;

	ret = cci_multi_reg_write(ov02b1b->regmap, reg_list->regs, reg_list->num_of_regs, NULL);
	if (ret)
		return ret;

	/* Apply customized values from user */
	ret = __v4l2_ctrl_handler_setup(ov02b1b->subdev.ctrl_handler);
	if (ret)
		return ret;

	ret = cci_multi_reg_write(ov02b1b->regmap, ov02b1b_streamon_regs, ARRAY_SIZE(ov02b1b_streamon_regs), NULL);
	if (ret)
		return ret;

	return 0;
}

static int __ov02b1b_stop_stream(struct ov02b1b *ov02b1b)
{
	int ret;

	ret = cci_multi_reg_write(ov02b1b->regmap, ov02b1b_streamoff_regs, ARRAY_SIZE(ov02b1b_streamoff_regs), NULL);
	if (ret)
		return ret;

	return 0;
}

static int ov02b1b_s_stream(struct v4l2_subdev *sd, int on)
{
	struct ov02b1b *ov02b1b = to_ov02b1b(sd);
	struct i2c_client *client = v4l2_get_subdevdata(&ov02b1b->subdev);
	int ret;

	mutex_lock(&ov02b1b->mutex);

	if (ov02b1b->streaming == on) {
		ret = 0;
		goto unlock_and_return;
	}

	if (on) {
		ret = pm_runtime_resume_and_get(&client->dev);
		if (ret < 0)
			goto unlock_and_return;

		ret = __ov02b1b_start_stream(ov02b1b);
		if (ret) {
			__ov02b1b_stop_stream(ov02b1b);
			ov02b1b->streaming = !on;
			goto err_rpm_put;
		}
	} else {
		__ov02b1b_stop_stream(ov02b1b);
		pm_runtime_put(&client->dev);
	}

	ov02b1b->streaming = on;
	mutex_unlock(&ov02b1b->mutex);

	return 0;

err_rpm_put:
	pm_runtime_put(&client->dev);
unlock_and_return:
	mutex_unlock(&ov02b1b->mutex);

	return ret;
}

static int ov02b1b_enum_mbus_code(struct v4l2_subdev *sd,
				  struct v4l2_subdev_state *sd_state,
				  struct v4l2_subdev_mbus_code_enum *code)
{
	if (code->index != 0)
		return -EINVAL;

	code->code = MEDIA_BUS_FMT_Y10_1X10;

	return 0;
}

static int ov02b1b_enum_frame_sizes(struct v4l2_subdev *sd,
				    struct v4l2_subdev_state *sd_state,
				    struct v4l2_subdev_frame_size_enum *fse)
{
	if (fse->index >= ARRAY_SIZE(supported_modes))
		return -EINVAL;

	fse->min_width  = supported_modes[fse->index].width;
	fse->max_width  = supported_modes[fse->index].width;
	fse->max_height = supported_modes[fse->index].height;
	fse->min_height = supported_modes[fse->index].height;

	return 0;
}

static void ov02b1b_fill_fmt(const struct ov02b1b_mode *mode,
			     struct v4l2_mbus_framefmt *fmt)
{
	fmt->width = mode->width;
	fmt->height = mode->height;
	fmt->code = MEDIA_BUS_FMT_Y10_1X10;
	fmt->field = V4L2_FIELD_NONE;
}

static int ov02b1b_get_fmt(struct v4l2_subdev *sd,
			   struct v4l2_subdev_state *sd_state,
			   struct v4l2_subdev_format *fmt)
{
	struct ov02b1b *ov02b1b = to_ov02b1b(sd);

	mutex_lock(&ov02b1b->mutex);

	if (fmt->which == V4L2_SUBDEV_FORMAT_TRY)
		fmt->format = *v4l2_subdev_state_get_format(sd_state, fmt->pad);
	else
		ov02b1b_fill_fmt(ov02b1b->cur_mode, &fmt->format);

	mutex_unlock(&ov02b1b->mutex);

	return 0;
}

static int ov02b1b_set_fmt(struct v4l2_subdev *sd,
			   struct v4l2_subdev_state *sd_state,
			   struct v4l2_subdev_format *fmt)
{
	struct ov02b1b *ov02b1b = to_ov02b1b(sd);
	const struct ov02b1b_mode *mode;
	struct v4l2_rect *crop;
	s32 vblank_def, hblank_def;
	int ret = 0;

	mode = v4l2_find_nearest_size(supported_modes,
				      ARRAY_SIZE(supported_modes), width,
				      height, fmt->format.width,
				      fmt->format.height);

	mutex_lock(&ov02b1b->mutex);

	if (ov02b1b->streaming && fmt->which == V4L2_SUBDEV_FORMAT_ACTIVE) {
		ret = -EBUSY;
		goto out_unlock;
	}

	ov02b1b_fill_fmt(mode, &fmt->format);

	if (fmt->which == V4L2_SUBDEV_FORMAT_TRY) {
		*v4l2_subdev_state_get_format(sd_state, 0) = fmt->format;
		crop = v4l2_subdev_state_get_crop(sd_state, 0);
		crop->left = mode->crop.left;
		crop->top = mode->crop.top;
		crop->width = mode->crop.width;
		crop->height = mode->crop.height;
	} else {
		ov02b1b->cur_mode = mode;

		vblank_def = mode->frame_length - mode->height;
		__v4l2_ctrl_modify_range(ov02b1b->vblank, vblank_def,
					 OV02B1B_VBLANK_MAX - mode->height, 1,
					 vblank_def);
		__v4l2_ctrl_s_ctrl(ov02b1b->vblank, vblank_def);

		hblank_def = mode->line_length - mode->width;
		__v4l2_ctrl_modify_range(ov02b1b->hblank, hblank_def, hblank_def, 1,
					 hblank_def);
	}

out_unlock:
	mutex_unlock(&ov02b1b->mutex);
	return ret;
}

static int ov02b1b_get_selection(struct v4l2_subdev *sd,
				 struct v4l2_subdev_state *sd_state,
				 struct v4l2_subdev_selection *sel)
{
	switch (sel->target) {
	case V4L2_SEL_TGT_CROP: {
		struct ov02b1b *ov02b1b = to_ov02b1b(sd);

		mutex_lock(&ov02b1b->mutex);
		switch (sel->which) {
		case V4L2_SUBDEV_FORMAT_TRY:
			sel->r = *v4l2_subdev_state_get_crop(sd_state, sel->pad);
			break;
		case V4L2_SUBDEV_FORMAT_ACTIVE:
			sel->r = ov02b1b->cur_mode->crop;
			break;
		}
		mutex_unlock(&ov02b1b->mutex);
		return 0;
	}
	case V4L2_SEL_TGT_NATIVE_SIZE:
		sel->r.left = 0;
		sel->r.top = 0;
		sel->r.width = OV02B1B_NATIVE_WIDTH;
		sel->r.height = OV02B1B_NATIVE_HEIGHT;
		return 0;
	case V4L2_SEL_TGT_CROP_DEFAULT:
	case V4L2_SEL_TGT_CROP_BOUNDS:
		sel->r.left = OV02B1B_PIXEL_ARRAY_LEFT;
		sel->r.top = OV02B1B_PIXEL_ARRAY_TOP;
		sel->r.width = OV02B1B_PIXEL_ARRAY_WIDTH;
		sel->r.height = OV02B1B_PIXEL_ARRAY_HEIGHT;
		return 0;
	}

	return -EINVAL;
}

static int ov02b1b_init_state(struct v4l2_subdev *sd,
			      struct v4l2_subdev_state *sd_state)
{
	struct v4l2_subdev_format fmt = { 0 };

	fmt.which = sd_state ? V4L2_SUBDEV_FORMAT_TRY : V4L2_SUBDEV_FORMAT_ACTIVE;

	ov02b1b_set_fmt(sd, sd_state, &fmt);

	return 0;
}

static int ov02b1b_set_exposure(struct ov02b1b *ov02b1b, int val)
{
	int ret;

	ret = cci_write(ov02b1b->regmap, CCI_REG8(0xfd), 0x01, NULL);
	if (ret)
		return ret;

	ret = cci_write(ov02b1b->regmap, OV02B1B_REG_EXPO, val, NULL);
	if (ret)
		return ret;

	return cci_write(ov02b1b->regmap, CCI_REG8(0xfe), 0x02, NULL);
}

static int ov02b1b_set_gain(struct ov02b1b *ov02b1b, int val)
{
	int ret;

	ret = cci_write(ov02b1b->regmap, CCI_REG8(0xfd), 0x01, NULL);
	if (ret)
		return ret;

	ret = cci_write(ov02b1b->regmap, OV02B1B_REG_GAIN, val, NULL);
	if (ret)
		return ret;

	return cci_write(ov02b1b->regmap, CCI_REG8(0xfe), 0x02, NULL);
}

static int ov02b1b_set_vblank(struct ov02b1b *ov02b1b, int val)
{
	int ret;

	ret = cci_write(ov02b1b->regmap, CCI_REG8(0xfd), 0x01, NULL);
	if (ret)
		return ret;

	ret = cci_write(ov02b1b->regmap, OV02B1B_REG_VBLANK, val - ov02b1b->cur_mode->vblank_min, NULL);
	if (ret)
		return ret;

	return cci_write(ov02b1b->regmap, CCI_REG8(0xfe), 0x02, NULL);
}

static int ov02b1b_set_test_pattern(struct ov02b1b *ov02b1b, int val)
{
	int ret;

	ret = cci_write(ov02b1b->regmap, CCI_REG8(0xfd), 0x00, NULL);
	if (ret)
		return ret;

	ret = cci_write(ov02b1b->regmap, OV02B1B_REG_TEST_PATTERN, val, NULL);
	if (ret)
		return ret;

	return cci_write(ov02b1b->regmap, CCI_REG8(0xfe), 0x02, NULL);
}

static int ov02b1b_set_ctrl(struct v4l2_ctrl *ctrl)
{
	struct ov02b1b *ov02b1b = container_of(ctrl->handler,
					       struct ov02b1b, ctrl_handler);
	struct i2c_client *client = v4l2_get_subdevdata(&ov02b1b->subdev);
	s32 max_expo;
	int ret;

	/* Propagate change of current control to all related controls */
	if (ctrl->id == V4L2_CID_VBLANK) {
		/* Update max exposure while meeting expected vblanking */
		max_expo = ov02b1b->cur_mode->height + ctrl->val -
			   OV02B1B_EXPOSURE_MAX_MARGIN;
		__v4l2_ctrl_modify_range(ov02b1b->exposure,
					 ov02b1b->exposure->minimum, max_expo,
					 ov02b1b->exposure->step,
					 ov02b1b->exposure->default_value);
	}

	/* V4L2 controls values will be applied only when power is already up */
	if (!pm_runtime_get_if_in_use(&client->dev))
		return 0;

	switch (ctrl->id) {
	case V4L2_CID_EXPOSURE:
		ret = ov02b1b_set_exposure(ov02b1b, ctrl->val);
		break;
	case V4L2_CID_ANALOGUE_GAIN:
		ret = ov02b1b_set_gain(ov02b1b, ctrl->val);
		break;
	case V4L2_CID_VBLANK:
		ret = ov02b1b_set_vblank(ov02b1b, ctrl->val);
		break;
	case V4L2_CID_TEST_PATTERN:
		ret = ov02b1b_set_test_pattern(ov02b1b, ctrl->val);
		break;
	default:
		ret = -EINVAL;
		break;
	}

	pm_runtime_put(&client->dev);

	return ret;
}

static const struct v4l2_subdev_video_ops ov02b1b_video_ops = {
	.s_stream = ov02b1b_s_stream,
};

static const struct v4l2_subdev_pad_ops ov02b1b_pad_ops = {
	.enum_mbus_code = ov02b1b_enum_mbus_code,
	.enum_frame_size = ov02b1b_enum_frame_sizes,
	.get_selection = ov02b1b_get_selection,
	.get_fmt = ov02b1b_get_fmt,
	.set_fmt = ov02b1b_set_fmt,
};

static const struct v4l2_subdev_ops ov02b1b_subdev_ops = {
	.video = &ov02b1b_video_ops,
	.pad = &ov02b1b_pad_ops,
};

static const struct media_entity_operations ov02b1b_subdev_entity_ops = {
	.link_validate = v4l2_subdev_link_validate,
};

static const struct v4l2_subdev_internal_ops ov02b1b_internal_ops = {
	.init_state = ov02b1b_init_state,
};

static const struct v4l2_ctrl_ops ov02b1b_ctrl_ops = {
	.s_ctrl = ov02b1b_set_ctrl,
};

static int ov02b1b_initialize_controls(struct ov02b1b *ov02b1b)
{
	struct i2c_client *client = v4l2_get_subdevdata(&ov02b1b->subdev);
	const struct ov02b1b_mode *mode;
	struct v4l2_ctrl_handler *handler;
	struct v4l2_ctrl *ctrl;
	struct v4l2_fwnode_device_properties props;
	s32 exposure_max;
	s32 vblank_def, hblank_def;
	s32 pixel_rate;
	int ret;

	handler = &ov02b1b->ctrl_handler;
	mode = ov02b1b->cur_mode;
	ret = v4l2_ctrl_handler_init(handler, 7);
	if (ret)
		return ret;

	handler->lock = &ov02b1b->mutex;

	ctrl = v4l2_ctrl_new_int_menu(handler, NULL, V4L2_CID_LINK_FREQ, 0, 0,
				      link_freq_menu_items);
	if (ctrl)
		ctrl->flags |= V4L2_CTRL_FLAG_READ_ONLY;

	pixel_rate = to_pixel_rate(0);
	v4l2_ctrl_new_std(handler, NULL, V4L2_CID_PIXEL_RATE, 0, pixel_rate, 1,
			  pixel_rate);

	hblank_def = mode->line_length - mode->width;
	ov02b1b->hblank = v4l2_ctrl_new_std(handler, NULL, V4L2_CID_HBLANK,
					    hblank_def, hblank_def, 1, hblank_def);

	vblank_def = mode->frame_length - mode->height;
	ov02b1b->vblank = v4l2_ctrl_new_std(handler, &ov02b1b_ctrl_ops, V4L2_CID_VBLANK, vblank_def,
			  		    OV02B1B_VBLANK_MAX - mode->height, 1, vblank_def);

	exposure_max = mode->frame_length - OV02B1B_EXPOSURE_MAX_MARGIN;
	ov02b1b->exposure = v4l2_ctrl_new_std(handler, &ov02b1b_ctrl_ops,
					      V4L2_CID_EXPOSURE,
					      OV02B1B_EXPOSURE_MIN,
					      exposure_max, 1,
					      OV02B1B_EXPOSURE_DEFAULT);

	v4l2_ctrl_new_std(handler, &ov02b1b_ctrl_ops,
			  V4L2_CID_ANALOGUE_GAIN, OV02B1B_ANA_GAIN_MIN,
			  OV02B1B_ANA_GAIN_MAX, 1,
			  OV02B1B_ANA_GAIN_DEFAULT);

	v4l2_ctrl_new_std_menu_items(handler, &ov02b1b_ctrl_ops,
				     V4L2_CID_TEST_PATTERN,
				     ARRAY_SIZE(ov02b1b_test_pattern_menu) - 1,
				     0, 0, ov02b1b_test_pattern_menu);

	if (handler->error) {
		ret = handler->error;
		dev_err(&client->dev, "failed to init controls(%d)\n", ret);
		goto err_free_handler;
	}

	ret = v4l2_fwnode_device_parse(&client->dev, &props);
	if (ret)
		goto err_free_handler;

	ret = v4l2_ctrl_new_fwnode_properties(handler, &ov02b1b_ctrl_ops, &props);
	if (ret)
		goto err_free_handler;

	ov02b1b->subdev.ctrl_handler = handler;

	return 0;

err_free_handler:
	v4l2_ctrl_handler_free(handler);

	return ret;
}

static int ov02b1b_check_sensor_id(struct ov02b1b *ov02b1b)
{
	u64 chip_id;
	int ret;

	/* Validate the chip ID */
	ret = cci_read(ov02b1b->regmap, OV02B1B_REG_CHIP_ID, &chip_id, NULL);
	if (ret < 0) {
		dev_err(ov02b1b->dev, "failed to read sensor information\n");
		return ret;
	}

	if (chip_id != OV02B1B_ID) {
		dev_err(ov02b1b->dev, "unexpected sensor id(0x%04llx)\n", chip_id);
		return -EINVAL;
	}

	return 0;
}

static int ov02b1b_power_on(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct ov02b1b *ov02b1b = to_ov02b1b(sd);
	int ret;

	gpiod_set_value_cansleep(ov02b1b->reset_gpio, 1);

	ret = regulator_bulk_enable(ARRAY_SIZE(ov02b1b_supply_names),
				    ov02b1b->supplies);
	if (ret < 0) {
		dev_err(dev, "failed to enable regulators\n");
		goto disable_clk;
	}
	usleep_range(4000, 5000);

	ret = clk_prepare_enable(ov02b1b->mclk);
	if (ret < 0) {
		dev_err(dev, "failed to enable mclk\n");
		return ret;
	}
	usleep_range(1000, 2000);

	gpiod_set_value_cansleep(ov02b1b->reset_gpio, 0);
	usleep_range(9000, 10000);

	ret = ov02b1b_check_sensor_id(ov02b1b);
	if (ret)
		goto disable_regulator;

	return 0;

disable_regulator:
	regulator_bulk_disable(ARRAY_SIZE(ov02b1b_supply_names),
			       ov02b1b->supplies);
disable_clk:
	clk_disable_unprepare(ov02b1b->mclk);

	return ret;
}

static int ov02b1b_power_off(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct ov02b1b *ov02b1b = to_ov02b1b(sd);

	gpiod_set_value_cansleep(ov02b1b->reset_gpio, 1);
	clk_disable_unprepare(ov02b1b->mclk);
	regulator_bulk_disable(ARRAY_SIZE(ov02b1b_supply_names),
			       ov02b1b->supplies);

	return 0;
}

static const struct dev_pm_ops ov02b1b_pm_ops = {
	SET_RUNTIME_PM_OPS(ov02b1b_power_off, ov02b1b_power_on, NULL)
};

static int ov02b1b_probe(struct i2c_client *client)
{
	struct ov02b1b *ov02b1b;
	unsigned int i;
	int ret;

	ov02b1b = devm_kzalloc(&client->dev, sizeof(*ov02b1b), GFP_KERNEL);
	if (!ov02b1b)
		return -ENOMEM;

	ov02b1b->dev = &client->dev;

	ret = ov02b1b_check_hwcfg(ov02b1b->dev, ov02b1b);
	if (ret)
		return dev_err_probe(ov02b1b->dev, ret, "failed to check HW configuration\n");

	ov02b1b->regmap = devm_cci_regmap_init_i2c(client, 8);
	if (IS_ERR(ov02b1b->regmap))
		return dev_err_probe(ov02b1b->dev, PTR_ERR(ov02b1b->regmap), "failed to init regmap\n");

	v4l2_i2c_subdev_init(&ov02b1b->subdev, client, &ov02b1b_subdev_ops);
	ov02b1b->subdev.internal_ops = &ov02b1b_internal_ops;

	ov02b1b->mclk = devm_clk_get(ov02b1b->dev, NULL);
	if (IS_ERR(ov02b1b->mclk))
		return dev_err_probe(ov02b1b->dev, PTR_ERR(ov02b1b->mclk), "failed to get mclk\n");

	ret = clk_set_rate(ov02b1b->mclk, 19200000);
	if (ret < 0)
		return dev_err_probe(ov02b1b->dev, ret, "failed to set mclk frequency\n");

	ov02b1b->reset_gpio = devm_gpiod_get(ov02b1b->dev, "reset", GPIOD_OUT_LOW);
	if (IS_ERR(ov02b1b->reset_gpio))
		return dev_err_probe(ov02b1b->dev, PTR_ERR(ov02b1b->reset_gpio),
				     "failed to get reset-gpios\n");

	for (i = 0; i < ARRAY_SIZE(ov02b1b_supply_names); i++)
		ov02b1b->supplies[i].supply = ov02b1b_supply_names[i];

	ret = devm_regulator_bulk_get(ov02b1b->dev, ARRAY_SIZE(ov02b1b_supply_names),
				      ov02b1b->supplies);
	if (ret)
		return dev_err_probe(ov02b1b->dev, ret, "failed to get regulators\n");

	mutex_init(&ov02b1b->mutex);

	/* Set default mode */
	ov02b1b->cur_mode = &supported_modes[0];

	ret = ov02b1b_initialize_controls(ov02b1b);
	if (ret) {
		dev_err_probe(ov02b1b->dev, ret, "failed to initialize controls\n");
		goto err_destroy_mutex;
	}

	/* Initialize subdev */
	ov02b1b->subdev.flags |= V4L2_SUBDEV_FL_HAS_DEVNODE;
	ov02b1b->subdev.entity.ops = &ov02b1b_subdev_entity_ops;
	ov02b1b->subdev.entity.function = MEDIA_ENT_F_CAM_SENSOR;
	ov02b1b->pad.flags = MEDIA_PAD_FL_SOURCE;

	ret = media_entity_pads_init(&ov02b1b->subdev.entity, 1, &ov02b1b->pad);
	if (ret < 0) {
		dev_err_probe(ov02b1b->dev, ret, "failed to initialize entity pads\n");
		goto err_free_handler;
	}

	pm_runtime_enable(ov02b1b->dev);
	if (!pm_runtime_enabled(ov02b1b->dev)) {
		ret = ov02b1b_power_on(ov02b1b->dev);
		if (ret < 0) {
			dev_err_probe(ov02b1b->dev, ret, "failed to power on\n");
			goto err_clean_entity;
		}
	}

	ret = v4l2_async_register_subdev_sensor(&ov02b1b->subdev);
	if (ret) {
		dev_err_probe(ov02b1b->dev, ret, "failed to register V4L2 subdev\n");
		goto err_power_off;
	}

	return 0;

err_power_off:
	if (pm_runtime_enabled(ov02b1b->dev))
		pm_runtime_disable(ov02b1b->dev);
	else
		ov02b1b_power_off(ov02b1b->dev);
err_clean_entity:
	media_entity_cleanup(&ov02b1b->subdev.entity);
err_free_handler:
	v4l2_ctrl_handler_free(ov02b1b->subdev.ctrl_handler);
err_destroy_mutex:
	mutex_destroy(&ov02b1b->mutex);

	return ret;
}

static void ov02b1b_remove(struct i2c_client *client)
{
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct ov02b1b *ov02b1b = to_ov02b1b(sd);

	v4l2_async_unregister_subdev(sd);
	media_entity_cleanup(&sd->entity);
	v4l2_ctrl_handler_free(sd->ctrl_handler);

	pm_runtime_disable(&client->dev);
	if (!pm_runtime_status_suspended(&client->dev))
		ov02b1b_power_off(&client->dev);

	pm_runtime_set_suspended(&client->dev);

	mutex_destroy(&ov02b1b->mutex);
}

static const struct of_device_id ov02b1b_of_match[] = {
	{ .compatible = "ovti,ov02b1b" },
	{}
};
MODULE_DEVICE_TABLE(of, ov02b1b_of_match);

static struct i2c_driver ov02b1b_i2c_driver = {
	.driver = {
		.name = "ov02b1b",
		.pm = &ov02b1b_pm_ops,
		.of_match_table = ov02b1b_of_match,
	},
	.probe = ov02b1b_probe,
	.remove = ov02b1b_remove,
};
module_i2c_driver(ov02b1b_i2c_driver);

MODULE_AUTHOR("map220v <map220v300@gmail.com>");
MODULE_DESCRIPTION("OmniVision OV02B1B camera sensor driver");
MODULE_LICENSE("GPL v2");
