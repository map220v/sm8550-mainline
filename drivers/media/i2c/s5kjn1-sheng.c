// SPDX-License-Identifier: GPL-2.0-only
/*
 * Samsung S5KJN1 Camera Sensor driver
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

/* S5KJN1 follows SMIA++ standard. */

#define S5KJN1_ID			0x38E1

#define S5KJN1_REG_CHIP_ID		CCI_REG16(0x0000)

#define S5KJN1_REG_EXPO			CCI_REG16(0x0202)
#define S5KJN1_EXPOSURE_MIN		6
#define S5KJN1_EXPOSURE_MAX_MARGIN	12
#define S5KJN1_EXPOSURE_DEFAULT		1024

#define S5KJN1_REG_GAIN			CCI_REG16(0x0204)
/* Min 1.0, Max 64.0, Mul 32 */
#define S5KJN1_ANA_GAIN_MIN		32
#define S5KJN1_ANA_GAIN_MAX		2048
#define S5KJN1_ANA_GAIN_DEFAULT		1024

#define S5KJN1_REG_FRAME_LENGTH		CCI_REG16(0x0340)
#define S5KJN1_FRAME_LENGTH_MAX		0xffff

#define S5KJN1_REG_TEST_PATTERN		CCI_REG16(0x0600)

#define S5KJN1_NATIVE_WIDTH		8192U
#define S5KJN1_NATIVE_HEIGHT		6176U
#define S5KJN1_PIXEL_ARRAY_LEFT		16U
#define S5KJN1_PIXEL_ARRAY_TOP		16U
#define S5KJN1_PIXEL_ARRAY_WIDTH	8160U
#define S5KJN1_PIXEL_ARRAY_HEIGHT	6144U

#define S5KJN1_DATA_LANES		4

#define S5KJN1_BITS_PER_SAMPLE		10

static const char * const s5kjn1_supply_names[] = {
	"vddio",	/* I/O power supply (1.8V) */
	"vddd",		/* Digital power supply (1.05V) */
	"vdda",		/* Analog power supply (2.8V) */
};

struct s5kjn1_reg_list {
	u32 num_of_regs;
	const struct cci_reg_sequence *regs;
};

struct s5kjn1_mode {
	u32 width;
	u32 height;
	struct v4l2_rect crop;
	u32 line_length;
	u32 frame_length;
	u32 max_again;
	u32 again;
	const struct s5kjn1_reg_list reg_list;
};

struct s5kjn1 {
	u32 mclk_freq;

	struct device *dev;
	struct clk *mclk;
	struct gpio_desc *reset_gpio;
	struct regulator_bulk_data supplies[ARRAY_SIZE(s5kjn1_supply_names)];
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
	struct v4l2_ctrl *again;

	const struct s5kjn1_mode *cur_mode;
};

static inline struct s5kjn1 *to_s5kjn1(struct v4l2_subdev *sd)
{
	return container_of(sd, struct s5kjn1, subdev);
}

static const struct cci_reg_sequence s5kjn1_init_regs[] = {
	{ CCI_REG16(0x6028), 0x2400 },
	{ CCI_REG16(0x602a), 0x1354 },
	{ CCI_REG16(0x6f12), 0x0100 },
	{ CCI_REG16(0x6f12), 0x7017 },
	{ CCI_REG16(0x602a), 0x13b2 },
	{ CCI_REG16(0x6f12), 0x0000 },
	{ CCI_REG16(0x602a), 0x1236 },
	{ CCI_REG16(0x6f12), 0x0000 },
	{ CCI_REG16(0x602a), 0x1a0a },
	{ CCI_REG16(0x6f12), 0x4c0a },
	{ CCI_REG16(0x602a), 0x2210 },
	{ CCI_REG16(0x6f12), 0x3401 },
	{ CCI_REG16(0x602a), 0x2176 },
	{ CCI_REG16(0x6f12), 0x6400 },
	{ CCI_REG16(0x602a), 0x222e },
	{ CCI_REG16(0x6f12), 0x0001 },
	{ CCI_REG16(0x602a), 0x06b6 },
	{ CCI_REG16(0x6f12), 0x0a00 },
	{ CCI_REG16(0x602a), 0x06bc },
	{ CCI_REG16(0x6f12), 0x1001 },
	{ CCI_REG16(0x602a), 0x2140 },
	{ CCI_REG16(0x6f12), 0x0101 },
	{ CCI_REG16(0x602a), 0x1a0e },
	{ CCI_REG16(0x6f12), 0x9600 },
	{ CCI_REG16(0x6028), 0x4000 },
	{ CCI_REG16(0xf44e), 0x0011 },
	{ CCI_REG16(0xf44c), 0x0b0b },
	{ CCI_REG16(0xf44a), 0x0006 },
	{ CCI_REG16(0x0118), 0x0002 },
	{ CCI_REG16(0x011a), 0x0001 },
	{ CCI_REG16(0x6028), 0x4000 },
	{ CCI_REG8(0x0106), 0x01 },
	{ CCI_REG16(0x0bcc), 0x0000 },
	{ CCI_REG16(0x6028), 0x2400 },
	{ CCI_REG16(0x602a), 0x2174 },
	{ CCI_REG16(0x6f12), 0x0400 },
	{ CCI_REG16(0x6028), 0x4000 },
};

static const struct cci_reg_sequence s5kjn1_8160x6120_regs[] = {
	{ CCI_REG16(0x6028), 0x2400 },
	{ CCI_REG16(0x602a), 0x1a28 },
	{ CCI_REG16(0x6f12), 0x4c00 },
	{ CCI_REG16(0x602a), 0x065a },
	{ CCI_REG16(0x6f12), 0x0000 },
	{ CCI_REG16(0x602a), 0x139e },
	{ CCI_REG16(0x6f12), 0x0400 },
	{ CCI_REG16(0x602a), 0x139c },
	{ CCI_REG16(0x6f12), 0x0100 },
	{ CCI_REG16(0x602a), 0x13a0 },
	{ CCI_REG16(0x6f12), 0x0500 },
	{ CCI_REG16(0x6f12), 0x0120 },
	{ CCI_REG16(0x602a), 0x2072 },
	{ CCI_REG16(0x6f12), 0x0101 },
	{ CCI_REG16(0x602a), 0x1a64 },
	{ CCI_REG16(0x6f12), 0x0001 },
	{ CCI_REG16(0x6f12), 0x0000 },
	{ CCI_REG16(0x602a), 0x19e6 },
	{ CCI_REG16(0x6f12), 0x0200 },
	{ CCI_REG16(0x602a), 0x1a30 },
	{ CCI_REG16(0x6f12), 0x3403 },
	{ CCI_REG16(0x602a), 0x19fc },
	{ CCI_REG16(0x6f12), 0x0700 },
	{ CCI_REG16(0x602a), 0x19f4 },
	{ CCI_REG16(0x6f12), 0x0707 },
	{ CCI_REG16(0x602a), 0x19f8 },
	{ CCI_REG16(0x6f12), 0x0b0b },
	{ CCI_REG16(0x602a), 0x1b26 },
	{ CCI_REG16(0x6f12), 0x6f80 },
	{ CCI_REG16(0x6f12), 0xa060 },
	{ CCI_REG16(0x602a), 0x1a3c },
	{ CCI_REG16(0x6f12), 0x8207 },
	{ CCI_REG16(0x602a), 0x1a48 },
	{ CCI_REG16(0x6f12), 0x8207 },
	{ CCI_REG16(0x602a), 0x1444 },
	{ CCI_REG16(0x6f12), 0x2000 },
	{ CCI_REG16(0x6f12), 0x2000 },
	{ CCI_REG16(0x602a), 0x144c },
	{ CCI_REG16(0x6f12), 0x3f00 },
	{ CCI_REG16(0x6f12), 0x3f00 },
	{ CCI_REG16(0x602a), 0x7f6c },
	{ CCI_REG16(0x6f12), 0x0100 },
	{ CCI_REG16(0x6f12), 0x2f00 },
	{ CCI_REG16(0x6f12), 0xfa00 },
	{ CCI_REG16(0x6f12), 0x2400 },
	{ CCI_REG16(0x6f12), 0xe500 },
	{ CCI_REG16(0x602a), 0x0650 },
	{ CCI_REG16(0x6f12), 0x0600 },
	{ CCI_REG16(0x602a), 0x0654 },
	{ CCI_REG16(0x6f12), 0x0000 },
	{ CCI_REG16(0x602a), 0x1a46 },
	{ CCI_REG16(0x6f12), 0x8500 },
	{ CCI_REG16(0x602a), 0x1a52 },
	{ CCI_REG16(0x6f12), 0x9800 },
	{ CCI_REG16(0x602a), 0x0674 },
	{ CCI_REG16(0x6f12), 0x0300 },
	{ CCI_REG16(0x6f12), 0x0300 },
	{ CCI_REG16(0x6f12), 0x0300 },
	{ CCI_REG16(0x6f12), 0x0300 },
	{ CCI_REG16(0x602a), 0x0668 },
	{ CCI_REG16(0x6f12), 0x0800 },
	{ CCI_REG16(0x6f12), 0x0800 },
	{ CCI_REG16(0x6f12), 0x0800 },
	{ CCI_REG16(0x6f12), 0x0800 },
	{ CCI_REG16(0x602a), 0x0684 },
	{ CCI_REG16(0x6f12), 0x4001 },
	{ CCI_REG16(0x602a), 0x0688 },
	{ CCI_REG16(0x6f12), 0x4001 },
	{ CCI_REG16(0x602a), 0x147c },
	{ CCI_REG16(0x6f12), 0x0400 },
	{ CCI_REG16(0x602a), 0x1480 },
	{ CCI_REG16(0x6f12), 0x0400 },
	{ CCI_REG16(0x602a), 0x19f6 },
	{ CCI_REG16(0x6f12), 0x0404 },
	{ CCI_REG16(0x602a), 0x0812 },
	{ CCI_REG16(0x6f12), 0x0000 },
	{ CCI_REG16(0x602a), 0x1a02 },
	{ CCI_REG16(0x6f12), 0x1800 },
	{ CCI_REG16(0x602a), 0x2148 },
	{ CCI_REG16(0x6f12), 0x0100 },
	{ CCI_REG16(0x602a), 0x2042 },
	{ CCI_REG16(0x6f12), 0x1a00 },
	{ CCI_REG16(0x602a), 0x0874 },
	{ CCI_REG16(0x6f12), 0x0106 },
	{ CCI_REG16(0x602a), 0x09c0 },
	{ CCI_REG16(0x6f12), 0x4000 },
	{ CCI_REG16(0x602a), 0x09c4 },
	{ CCI_REG16(0x6f12), 0x4000 },
	{ CCI_REG16(0x602a), 0x19fe },
	{ CCI_REG16(0x6f12), 0x0c1c },
	{ CCI_REG16(0x602a), 0x4d92 },
	{ CCI_REG16(0x6f12), 0x0000 },
	{ CCI_REG16(0x602a), 0x84c8 },
	{ CCI_REG16(0x6f12), 0x0000 },
	{ CCI_REG16(0x602a), 0x4d94 },
	{ CCI_REG16(0x6f12), 0x0000 },
	{ CCI_REG16(0x6f12), 0x0000 },
	{ CCI_REG16(0x6f12), 0x0000 },
	{ CCI_REG16(0x6f12), 0x0000 },
	{ CCI_REG16(0x6f12), 0x0000 },
	{ CCI_REG16(0x6f12), 0x0000 },
	{ CCI_REG16(0x6f12), 0x0000 },
	{ CCI_REG16(0x6f12), 0x0000 },
	{ CCI_REG16(0x6f12), 0x0000 },
	{ CCI_REG16(0x6f12), 0x0000 },
	{ CCI_REG16(0x6f12), 0x0000 },
	{ CCI_REG16(0x6f12), 0x0000 },
	{ CCI_REG16(0x602a), 0x3570 },
	{ CCI_REG16(0x6f12), 0x0000 },
	{ CCI_REG16(0x602a), 0x3574 },
	{ CCI_REG16(0x6f12), 0x7306 },
	{ CCI_REG16(0x602a), 0x21e4 },
	{ CCI_REG16(0x6f12), 0x0400 },
	{ CCI_REG16(0x602a), 0x21ec },
	{ CCI_REG16(0x6f12), 0x6902 },
	{ CCI_REG16(0x602a), 0x2080 },
	{ CCI_REG16(0x6f12), 0x0100 },
	{ CCI_REG16(0x6f12), 0xff00 },
	{ CCI_REG16(0x6f12), 0x0002 },
	{ CCI_REG16(0x6f12), 0x0001 },
	{ CCI_REG16(0x6f12), 0x0002 },
	{ CCI_REG16(0x6f12), 0xd244 },
	{ CCI_REG16(0x6f12), 0xd244 },
	{ CCI_REG16(0x6f12), 0x14f4 },
	{ CCI_REG16(0x6f12), 0x101c },
	{ CCI_REG16(0x6f12), 0x0d1c },
	{ CCI_REG16(0x6f12), 0x54f4 },
	{ CCI_REG16(0x602a), 0x20ba },
	{ CCI_REG16(0x6f12), 0x0000 },
	{ CCI_REG16(0x6f12), 0x0000 },
	{ CCI_REG16(0x6f12), 0x0000 },
	{ CCI_REG16(0x602a), 0x120e },
	{ CCI_REG16(0x6f12), 0x1000 },
	{ CCI_REG16(0x602a), 0x212e },
	{ CCI_REG16(0x6f12), 0x0200 },
	{ CCI_REG16(0x602a), 0x13ae },
	{ CCI_REG16(0x6f12), 0x0100 },
	{ CCI_REG16(0x602a), 0x0718 },
	{ CCI_REG16(0x6f12), 0x0000 },
	{ CCI_REG16(0x602a), 0x0710 },
	{ CCI_REG16(0x6f12), 0x0010 },
	{ CCI_REG16(0x6f12), 0x0201 },
	{ CCI_REG16(0x6f12), 0x0800 },
	{ CCI_REG16(0x602a), 0x1b5c },
	{ CCI_REG16(0x6f12), 0x0000 },
	{ CCI_REG16(0x602a), 0x0786 },
	{ CCI_REG16(0x6f12), 0x1401 },
	{ CCI_REG16(0x602a), 0x2022 },
	{ CCI_REG16(0x6f12), 0x0500 },
	{ CCI_REG16(0x6f12), 0x0500 },
	{ CCI_REG16(0x602a), 0x1360 },
	{ CCI_REG16(0x6f12), 0x0000 },
	{ CCI_REG16(0x602a), 0x1376 },
	{ CCI_REG16(0x6f12), 0x0000 },
	{ CCI_REG16(0x6f12), 0x6038 },
	{ CCI_REG16(0x6f12), 0x7038 },
	{ CCI_REG16(0x6f12), 0x8038 },
	{ CCI_REG16(0x602a), 0x1386 },
	{ CCI_REG16(0x6f12), 0x0b00 },
	{ CCI_REG16(0x602a), 0x06fa },
	{ CCI_REG16(0x6f12), 0x1000 },
	{ CCI_REG16(0x602a), 0x4a94 },
	{ CCI_REG16(0x6f12), 0x0400 },
	{ CCI_REG16(0x6f12), 0x0400 },
	{ CCI_REG16(0x6f12), 0x0400 },
	{ CCI_REG16(0x6f12), 0x0400 },
	{ CCI_REG16(0x6f12), 0x0800 },
	{ CCI_REG16(0x6f12), 0x0800 },
	{ CCI_REG16(0x6f12), 0x0800 },
	{ CCI_REG16(0x6f12), 0x0800 },
	{ CCI_REG16(0x6f12), 0x0400 },
	{ CCI_REG16(0x6f12), 0x0400 },
	{ CCI_REG16(0x6f12), 0x0400 },
	{ CCI_REG16(0x6f12), 0x0400 },
	{ CCI_REG16(0x6f12), 0x0800 },
	{ CCI_REG16(0x6f12), 0x0800 },
	{ CCI_REG16(0x6f12), 0x0800 },
	{ CCI_REG16(0x6f12), 0x0800 },
	{ CCI_REG16(0x602a), 0x0a76 },
	{ CCI_REG16(0x6f12), 0x1000 },
	{ CCI_REG16(0x602a), 0x0aee },
	{ CCI_REG16(0x6f12), 0x1000 },
	{ CCI_REG16(0x602a), 0x0b66 },
	{ CCI_REG16(0x6f12), 0x1000 },
	{ CCI_REG16(0x602a), 0x0bde },
	{ CCI_REG16(0x6f12), 0x1000 },
	{ CCI_REG16(0x602a), 0x0be8 },
	{ CCI_REG16(0x6f12), 0x5000 },
	{ CCI_REG16(0x6f12), 0x5000 },
	{ CCI_REG16(0x602a), 0x0c56 },
	{ CCI_REG16(0x6f12), 0x1000 },
	{ CCI_REG16(0x602a), 0x0c60 },
	{ CCI_REG16(0x6f12), 0x5000 },
	{ CCI_REG16(0x6f12), 0x5000 },
	{ CCI_REG16(0x602a), 0x0cb6 },
	{ CCI_REG16(0x6f12), 0x0000 },
	{ CCI_REG16(0x602a), 0x0cf2 },
	{ CCI_REG16(0x6f12), 0x0001 },
	{ CCI_REG16(0x602a), 0x0cf0 },
	{ CCI_REG16(0x6f12), 0x0101 },
	{ CCI_REG16(0x602a), 0x11b8 },
	{ CCI_REG16(0x6f12), 0x0000 },
	{ CCI_REG16(0x602a), 0x11f6 },
	{ CCI_REG16(0x6f12), 0x0010 },
	{ CCI_REG16(0x602a), 0x4a74 },
	{ CCI_REG16(0x6f12), 0x0000 },
	{ CCI_REG16(0x6f12), 0x0000 },
	{ CCI_REG16(0x6f12), 0x0000 },
	{ CCI_REG16(0x6f12), 0x0000 },
	{ CCI_REG16(0x6f12), 0x0000 },
	{ CCI_REG16(0x6f12), 0x0000 },
	{ CCI_REG16(0x6f12), 0x0000 },
	{ CCI_REG16(0x6f12), 0x0000 },
	{ CCI_REG16(0x6f12), 0x0000 },
	{ CCI_REG16(0x6f12), 0x0000 },
	{ CCI_REG16(0x6f12), 0x0000 },
	{ CCI_REG16(0x6f12), 0x0000 },
	{ CCI_REG16(0x6f12), 0x0000 },
	{ CCI_REG16(0x6f12), 0x0000 },
	{ CCI_REG16(0x6f12), 0x0000 },
	{ CCI_REG16(0x6f12), 0x0000 },
	{ CCI_REG16(0x602a), 0x218e },
	{ CCI_REG16(0x6f12), 0x0000 },
	{ CCI_REG16(0x602a), 0x2268 },
	{ CCI_REG16(0x6f12), 0xf279 },
	{ CCI_REG16(0x602a), 0x5006 },
	{ CCI_REG16(0x6f12), 0x0000 },
	{ CCI_REG16(0x602a), 0x500e },
	{ CCI_REG16(0x6f12), 0x0100 },
	{ CCI_REG16(0x602a), 0x4e70 },
	{ CCI_REG16(0x6f12), 0x2062 },
	{ CCI_REG16(0x6f12), 0x5501 },
	{ CCI_REG16(0x602a), 0x06dc },
	{ CCI_REG16(0x6f12), 0x0000 },
	{ CCI_REG16(0x6f12), 0x0000 },
	{ CCI_REG16(0x6f12), 0x0000 },
	{ CCI_REG16(0x6f12), 0x0000 },
	{ CCI_REG16(0x6028), 0x4000 },
	{ CCI_REG16(0xf46a), 0xae80 },
	{ CCI_REG16(0x0344), 0x0000 },
	{ CCI_REG16(0x0346), 0x000c },
	{ CCI_REG16(0x0348), 0x1fff },
	{ CCI_REG16(0x034a), 0x1813 },
	{ CCI_REG16(0x034c), 0x1fe0 },
	{ CCI_REG16(0x034e), 0x17e8 },
	{ CCI_REG16(0x0350), 0x0010 },
	{ CCI_REG16(0x0352), 0x0010 },
	{ CCI_REG16(0x0900), 0x0111 },
	{ CCI_REG16(0x0380), 0x0001 },
	{ CCI_REG16(0x0382), 0x0001 },
	{ CCI_REG16(0x0384), 0x0001 },
	{ CCI_REG16(0x0386), 0x0001 },
	{ CCI_REG16(0x0110), 0x1002 },
	{ CCI_REG16(0x0114), 0x0300 },
	{ CCI_REG16(0x0116), 0x3000 },
	{ CCI_REG16(0x0136), 0x1300 },
	{ CCI_REG16(0x013e), 0x00c8 },
	{ CCI_REG16(0x0300), 0x0006 },
	{ CCI_REG16(0x0302), 0x0001 },
	{ CCI_REG16(0x0304), 0x0003 },
	{ CCI_REG16(0x0306), 0x0083 },
	{ CCI_REG16(0x0308), 0x0008 },
	{ CCI_REG16(0x030a), 0x0001 },
	{ CCI_REG16(0x030c), 0x0000 },
	{ CCI_REG16(0x030e), 0x0003 },
	{ CCI_REG16(0x0310), 0x0086 },
	{ CCI_REG16(0x0312), 0x0000 },
	{ CCI_REG16(0x080e), 0x0000 },
	{ CCI_REG16(0x0340), 0x1900 },
	{ CCI_REG16(0x0342), 0x21f0 },
	{ CCI_REG16(0x0702), 0x0000 },
	{ CCI_REG16(0x0202), 0x0100 },
	{ CCI_REG16(0x0200), 0x0100 },
	{ CCI_REG16(0x0d00), 0x0100 },
	{ CCI_REG16(0x0d02), 0x0001 },
	{ CCI_REG16(0x0d04), 0x0002 },
	{ CCI_REG16(0x6226), 0x0000 },
};

static const struct cci_reg_sequence s5kjn1_4080x3060_regs[] = {
	{ CCI_REG16(0x6028), 0x2400 },
	{ CCI_REG16(0x602a), 0x1a28 },
	{ CCI_REG16(0x6f12), 0x4c00 },
	{ CCI_REG16(0x602a), 0x065a },
	{ CCI_REG16(0x6f12), 0x0000 },
	{ CCI_REG16(0x602a), 0x139e },
	{ CCI_REG16(0x6f12), 0x0100 },
	{ CCI_REG16(0x602a), 0x139c },
	{ CCI_REG16(0x6f12), 0x0000 },
	{ CCI_REG16(0x602a), 0x13a0 },
	{ CCI_REG16(0x6f12), 0x0a00 },
	{ CCI_REG16(0x6f12), 0x0120 },
	{ CCI_REG16(0x602a), 0x2072 },
	{ CCI_REG16(0x6f12), 0x0000 },
	{ CCI_REG16(0x602a), 0x1a64 },
	{ CCI_REG16(0x6f12), 0x0301 },
	{ CCI_REG16(0x6f12), 0xff00 },
	{ CCI_REG16(0x602a), 0x19e6 },
	{ CCI_REG16(0x6f12), 0x0200 },
	{ CCI_REG16(0x602a), 0x1a30 },
	{ CCI_REG16(0x6f12), 0x3401 },
	{ CCI_REG16(0x602a), 0x19fc },
	{ CCI_REG16(0x6f12), 0x0b00 },
	{ CCI_REG16(0x602a), 0x19f4 },
	{ CCI_REG16(0x6f12), 0x0606 },
	{ CCI_REG16(0x602a), 0x19f8 },
	{ CCI_REG16(0x6f12), 0x1010 },
	{ CCI_REG16(0x602a), 0x1b26 },
	{ CCI_REG16(0x6f12), 0x6f80 },
	{ CCI_REG16(0x6f12), 0xa060 },
	{ CCI_REG16(0x602a), 0x1a3c },
	{ CCI_REG16(0x6f12), 0x6207 },
	{ CCI_REG16(0x602a), 0x1a48 },
	{ CCI_REG16(0x6f12), 0x6207 },
	{ CCI_REG16(0x602a), 0x1444 },
	{ CCI_REG16(0x6f12), 0x2000 },
	{ CCI_REG16(0x6f12), 0x2000 },
	{ CCI_REG16(0x602a), 0x144c },
	{ CCI_REG16(0x6f12), 0x3f00 },
	{ CCI_REG16(0x6f12), 0x3f00 },
	{ CCI_REG16(0x602a), 0x7f6c },
	{ CCI_REG16(0x6f12), 0x0100 },
	{ CCI_REG16(0x6f12), 0x2f00 },
	{ CCI_REG16(0x6f12), 0xfa00 },
	{ CCI_REG16(0x6f12), 0x2400 },
	{ CCI_REG16(0x6f12), 0xe500 },
	{ CCI_REG16(0x602a), 0x0650 },
	{ CCI_REG16(0x6f12), 0x0600 },
	{ CCI_REG16(0x602a), 0x0654 },
	{ CCI_REG16(0x6f12), 0x0000 },
	{ CCI_REG16(0x602a), 0x1a46 },
	{ CCI_REG16(0x6f12), 0x8a00 },
	{ CCI_REG16(0x602a), 0x1a52 },
	{ CCI_REG16(0x6f12), 0xbf00 },
	{ CCI_REG16(0x602a), 0x0674 },
	{ CCI_REG16(0x6f12), 0x0300 },
	{ CCI_REG16(0x6f12), 0x0300 },
	{ CCI_REG16(0x6f12), 0x0300 },
	{ CCI_REG16(0x6f12), 0x0300 },
	{ CCI_REG16(0x602a), 0x0668 },
	{ CCI_REG16(0x6f12), 0x0800 },
	{ CCI_REG16(0x6f12), 0x0800 },
	{ CCI_REG16(0x6f12), 0x0800 },
	{ CCI_REG16(0x6f12), 0x0800 },
	{ CCI_REG16(0x602a), 0x0684 },
	{ CCI_REG16(0x6f12), 0x4001 },
	{ CCI_REG16(0x602a), 0x0688 },
	{ CCI_REG16(0x6f12), 0x4001 },
	{ CCI_REG16(0x602a), 0x147c },
	{ CCI_REG16(0x6f12), 0x1000 },
	{ CCI_REG16(0x602a), 0x1480 },
	{ CCI_REG16(0x6f12), 0x1000 },
	{ CCI_REG16(0x602a), 0x19f6 },
	{ CCI_REG16(0x6f12), 0x0904 },
	{ CCI_REG16(0x602a), 0x0812 },
	{ CCI_REG16(0x6f12), 0x0000 },
	{ CCI_REG16(0x602a), 0x1a02 },
	{ CCI_REG16(0x6f12), 0x1800 },
	{ CCI_REG16(0x602a), 0x2148 },
	{ CCI_REG16(0x6f12), 0x0100 },
	{ CCI_REG16(0x602a), 0x2042 },
	{ CCI_REG16(0x6f12), 0x1a00 },
	{ CCI_REG16(0x602a), 0x0874 },
	{ CCI_REG16(0x6f12), 0x0100 },
	{ CCI_REG16(0x602a), 0x09c0 },
	{ CCI_REG16(0x6f12), 0x2008 },
	{ CCI_REG16(0x602a), 0x09c4 },
	{ CCI_REG16(0x6f12), 0x2000 },
	{ CCI_REG16(0x602a), 0x19fe },
	{ CCI_REG16(0x6f12), 0x0e1c },
	{ CCI_REG16(0x602a), 0x4d92 },
	{ CCI_REG16(0x6f12), 0x0100 },
	{ CCI_REG16(0x602a), 0x84c8 },
	{ CCI_REG16(0x6f12), 0x0100 },
	{ CCI_REG16(0x602a), 0x4d94 },
	{ CCI_REG16(0x6f12), 0x0005 },
	{ CCI_REG16(0x6f12), 0x000a },
	{ CCI_REG16(0x6f12), 0x0010 },
	{ CCI_REG16(0x6f12), 0x0810 },
	{ CCI_REG16(0x6f12), 0x000a },
	{ CCI_REG16(0x6f12), 0x0040 },
	{ CCI_REG16(0x6f12), 0x0810 },
	{ CCI_REG16(0x6f12), 0x0810 },
	{ CCI_REG16(0x6f12), 0x8002 },
	{ CCI_REG16(0x6f12), 0xfd03 },
	{ CCI_REG16(0x6f12), 0x0010 },
	{ CCI_REG16(0x6f12), 0x1510 },
	{ CCI_REG16(0x602a), 0x3570 },
	{ CCI_REG16(0x6f12), 0x0000 },
	{ CCI_REG16(0x602a), 0x3574 },
	{ CCI_REG16(0x6f12), 0x1201 },
	{ CCI_REG16(0x602a), 0x21e4 },
	{ CCI_REG16(0x6f12), 0x0400 },
	{ CCI_REG16(0x602a), 0x21ec },
	{ CCI_REG16(0x6f12), 0x1f04 },
	{ CCI_REG16(0x602a), 0x2080 },
	{ CCI_REG16(0x6f12), 0x0101 },
	{ CCI_REG16(0x6f12), 0xff00 },
	{ CCI_REG16(0x6f12), 0x7f01 },
	{ CCI_REG16(0x6f12), 0x0001 },
	{ CCI_REG16(0x6f12), 0x8001 },
	{ CCI_REG16(0x6f12), 0xd244 },
	{ CCI_REG16(0x6f12), 0xd244 },
	{ CCI_REG16(0x6f12), 0x14f4 },
	{ CCI_REG16(0x6f12), 0x0000 },
	{ CCI_REG16(0x6f12), 0x0000 },
	{ CCI_REG16(0x6f12), 0x0000 },
	{ CCI_REG16(0x602a), 0x20ba },
	{ CCI_REG16(0x6f12), 0x121c },
	{ CCI_REG16(0x6f12), 0x111c },
	{ CCI_REG16(0x6f12), 0x54f4 },
	{ CCI_REG16(0x602a), 0x120e },
	{ CCI_REG16(0x6f12), 0x1000 },
	{ CCI_REG16(0x602a), 0x212e },
	{ CCI_REG16(0x6f12), 0x0200 },
	{ CCI_REG16(0x602a), 0x13ae },
	{ CCI_REG16(0x6f12), 0x0101 },
	{ CCI_REG16(0x602a), 0x0718 },
	{ CCI_REG16(0x6f12), 0x0001 },
	{ CCI_REG16(0x602a), 0x0710 },
	{ CCI_REG16(0x6f12), 0x0002 },
	{ CCI_REG16(0x6f12), 0x0804 },
	{ CCI_REG16(0x6f12), 0x0100 },
	{ CCI_REG16(0x602a), 0x1b5c },
	{ CCI_REG16(0x6f12), 0x0000 },
	{ CCI_REG16(0x602a), 0x0786 },
	{ CCI_REG16(0x6f12), 0x7701 },
	{ CCI_REG16(0x602a), 0x2022 },
	{ CCI_REG16(0x6f12), 0x0500 },
	{ CCI_REG16(0x6f12), 0x0500 },
	{ CCI_REG16(0x602a), 0x1360 },
	{ CCI_REG16(0x6f12), 0x0100 },
	{ CCI_REG16(0x602a), 0x1376 },
	{ CCI_REG16(0x6f12), 0x0100 },
	{ CCI_REG16(0x6f12), 0x6038 },
	{ CCI_REG16(0x6f12), 0x7038 },
	{ CCI_REG16(0x6f12), 0x8038 },
	{ CCI_REG16(0x602a), 0x1386 },
	{ CCI_REG16(0x6f12), 0x0b00 },
	{ CCI_REG16(0x602a), 0x06fa },
	{ CCI_REG16(0x6f12), 0x0000 },
	{ CCI_REG16(0x602a), 0x4a94 },
	{ CCI_REG16(0x6f12), 0x0900 },
	{ CCI_REG16(0x6f12), 0x0000 },
	{ CCI_REG16(0x6f12), 0x0300 },
	{ CCI_REG16(0x6f12), 0x0000 },
	{ CCI_REG16(0x6f12), 0x0000 },
	{ CCI_REG16(0x6f12), 0x0000 },
	{ CCI_REG16(0x6f12), 0x0000 },
	{ CCI_REG16(0x6f12), 0x0000 },
	{ CCI_REG16(0x6f12), 0x0300 },
	{ CCI_REG16(0x6f12), 0x0000 },
	{ CCI_REG16(0x6f12), 0x0900 },
	{ CCI_REG16(0x6f12), 0x0000 },
	{ CCI_REG16(0x6f12), 0x0000 },
	{ CCI_REG16(0x6f12), 0x0000 },
	{ CCI_REG16(0x6f12), 0x0000 },
	{ CCI_REG16(0x6f12), 0x0000 },
	{ CCI_REG16(0x602a), 0x0a76 },
	{ CCI_REG16(0x6f12), 0x1000 },
	{ CCI_REG16(0x602a), 0x0aee },
	{ CCI_REG16(0x6f12), 0x1000 },
	{ CCI_REG16(0x602a), 0x0b66 },
	{ CCI_REG16(0x6f12), 0x1000 },
	{ CCI_REG16(0x602a), 0x0bde },
	{ CCI_REG16(0x6f12), 0x1000 },
	{ CCI_REG16(0x602a), 0x0be8 },
	{ CCI_REG16(0x6f12), 0x3000 },
	{ CCI_REG16(0x6f12), 0x3000 },
	{ CCI_REG16(0x602a), 0x0c56 },
	{ CCI_REG16(0x6f12), 0x1000 },
	{ CCI_REG16(0x602a), 0x0c60 },
	{ CCI_REG16(0x6f12), 0x3000 },
	{ CCI_REG16(0x6f12), 0x3000 },
	{ CCI_REG16(0x602a), 0x0cb6 },
	{ CCI_REG16(0x6f12), 0x0100 },
	{ CCI_REG16(0x602a), 0x0cf2 },
	{ CCI_REG16(0x6f12), 0x0001 },
	{ CCI_REG16(0x602a), 0x0cf0 },
	{ CCI_REG16(0x6f12), 0x0101 },
	{ CCI_REG16(0x602a), 0x11b8 },
	{ CCI_REG16(0x6f12), 0x0100 },
	{ CCI_REG16(0x602a), 0x11f6 },
	{ CCI_REG16(0x6f12), 0x0020 },
	{ CCI_REG16(0x602a), 0x4a74 },
	{ CCI_REG16(0x6f12), 0x0000 },
	{ CCI_REG16(0x6f12), 0x0000 },
	{ CCI_REG16(0x6f12), 0xd8ff },
	{ CCI_REG16(0x6f12), 0x0000 },
	{ CCI_REG16(0x6f12), 0x0000 },
	{ CCI_REG16(0x6f12), 0x0000 },
	{ CCI_REG16(0x6f12), 0x0000 },
	{ CCI_REG16(0x6f12), 0x0000 },
	{ CCI_REG16(0x6f12), 0xd8ff },
	{ CCI_REG16(0x6f12), 0x0000 },
	{ CCI_REG16(0x6f12), 0x0000 },
	{ CCI_REG16(0x6f12), 0x0000 },
	{ CCI_REG16(0x6f12), 0x0000 },
	{ CCI_REG16(0x6f12), 0x0000 },
	{ CCI_REG16(0x6f12), 0x0000 },
	{ CCI_REG16(0x6f12), 0x0000 },
	{ CCI_REG16(0x602a), 0x218e },
	{ CCI_REG16(0x6f12), 0x0000 },
	{ CCI_REG16(0x602a), 0x2268 },
	{ CCI_REG16(0x6f12), 0xf279 },
	{ CCI_REG16(0x602a), 0x5006 },
	{ CCI_REG16(0x6f12), 0x0000 },
	{ CCI_REG16(0x602a), 0x500e },
	{ CCI_REG16(0x6f12), 0x0100 },
	{ CCI_REG16(0x602a), 0x4e70 },
	{ CCI_REG16(0x6f12), 0x2062 },
	{ CCI_REG16(0x6f12), 0x5501 },
	{ CCI_REG16(0x602a), 0x06dc },
	{ CCI_REG16(0x6f12), 0x0000 },
	{ CCI_REG16(0x6f12), 0x0000 },
	{ CCI_REG16(0x6f12), 0x0000 },
	{ CCI_REG16(0x6f12), 0x0000 },
	{ CCI_REG16(0x6028), 0x4000 },
	{ CCI_REG16(0xf46a), 0xae80 },
	{ CCI_REG16(0x0344), 0x0000 },
	{ CCI_REG16(0x0346), 0x000c },
	{ CCI_REG16(0x0348), 0x1fff },
	{ CCI_REG16(0x034a), 0x1813 },
	{ CCI_REG16(0x034c), 0x0ff0 },
	{ CCI_REG16(0x034e), 0x0bf4 },
	{ CCI_REG16(0x0350), 0x0008 },
	{ CCI_REG16(0x0352), 0x0008 },
	{ CCI_REG16(0x0900), 0x0122 },
	{ CCI_REG16(0x0380), 0x0002 },
	{ CCI_REG16(0x0382), 0x0002 },
	{ CCI_REG16(0x0384), 0x0002 },
	{ CCI_REG16(0x0386), 0x0002 },
	{ CCI_REG16(0x0110), 0x1002 },
	{ CCI_REG16(0x0114), 0x0301 },
	{ CCI_REG16(0x0116), 0x3000 },
	{ CCI_REG16(0x0136), 0x1300 },
	{ CCI_REG16(0x013e), 0x00c8 },
	{ CCI_REG16(0x0300), 0x0006 },
	{ CCI_REG16(0x0302), 0x0001 },
	{ CCI_REG16(0x0304), 0x0003 },
	{ CCI_REG16(0x0306), 0x0083 },
	{ CCI_REG16(0x0308), 0x0008 },
	{ CCI_REG16(0x030a), 0x0001 },
	{ CCI_REG16(0x030c), 0x0000 },
	{ CCI_REG16(0x030e), 0x0003 },
	{ CCI_REG16(0x0310), 0x0086 },
	{ CCI_REG16(0x0312), 0x0000 },
	{ CCI_REG16(0x080e), 0x0000 },
	{ CCI_REG16(0x0340), 0x0c5c },
	{ CCI_REG16(0x0342), 0x1700 },
	{ CCI_REG16(0x0702), 0x0000 },
	{ CCI_REG16(0x0202), 0x0100 },
	{ CCI_REG16(0x0200), 0x0100 },
	{ CCI_REG16(0x0d00), 0x0101 },
	{ CCI_REG16(0x0d02), 0x0101 },
	{ CCI_REG16(0x0d04), 0x0102 },
	{ CCI_REG16(0x6226), 0x0000 },
};

static const struct cci_reg_sequence s5kjn1_4080x2296_regs[] = {
	{ CCI_REG16(0x6028), 0x2400 },
	{ CCI_REG16(0x602a), 0x1a28 },
	{ CCI_REG16(0x6f12), 0x4c00 },
	{ CCI_REG16(0x602a), 0x065a },
	{ CCI_REG16(0x6f12), 0x0000 },
	{ CCI_REG16(0x602a), 0x139e },
	{ CCI_REG16(0x6f12), 0x0100 },
	{ CCI_REG16(0x602a), 0x139c },
	{ CCI_REG16(0x6f12), 0x0000 },
	{ CCI_REG16(0x602a), 0x13a0 },
	{ CCI_REG16(0x6f12), 0x0a00 },
	{ CCI_REG16(0x6f12), 0x0120 },
	{ CCI_REG16(0x602a), 0x2072 },
	{ CCI_REG16(0x6f12), 0x0000 },
	{ CCI_REG16(0x602a), 0x1a64 },
	{ CCI_REG16(0x6f12), 0x0301 },
	{ CCI_REG16(0x6f12), 0xff00 },
	{ CCI_REG16(0x602a), 0x19e6 },
	{ CCI_REG16(0x6f12), 0x0200 },
	{ CCI_REG16(0x602a), 0x1a30 },
	{ CCI_REG16(0x6f12), 0x3401 },
	{ CCI_REG16(0x602a), 0x19fc },
	{ CCI_REG16(0x6f12), 0x0b00 },
	{ CCI_REG16(0x602a), 0x19f4 },
	{ CCI_REG16(0x6f12), 0x0606 },
	{ CCI_REG16(0x602a), 0x19f8 },
	{ CCI_REG16(0x6f12), 0x1010 },
	{ CCI_REG16(0x602a), 0x1b26 },
	{ CCI_REG16(0x6f12), 0x6f80 },
	{ CCI_REG16(0x6f12), 0xa060 },
	{ CCI_REG16(0x602a), 0x1a3c },
	{ CCI_REG16(0x6f12), 0x6207 },
	{ CCI_REG16(0x602a), 0x1a48 },
	{ CCI_REG16(0x6f12), 0x6207 },
	{ CCI_REG16(0x602a), 0x1444 },
	{ CCI_REG16(0x6f12), 0x2000 },
	{ CCI_REG16(0x6f12), 0x2000 },
	{ CCI_REG16(0x602a), 0x144c },
	{ CCI_REG16(0x6f12), 0x3f00 },
	{ CCI_REG16(0x6f12), 0x3f00 },
	{ CCI_REG16(0x602a), 0x7f6c },
	{ CCI_REG16(0x6f12), 0x0100 },
	{ CCI_REG16(0x6f12), 0x2f00 },
	{ CCI_REG16(0x6f12), 0xfa00 },
	{ CCI_REG16(0x6f12), 0x2400 },
	{ CCI_REG16(0x6f12), 0xe500 },
	{ CCI_REG16(0x602a), 0x0650 },
	{ CCI_REG16(0x6f12), 0x0600 },
	{ CCI_REG16(0x602a), 0x0654 },
	{ CCI_REG16(0x6f12), 0x0000 },
	{ CCI_REG16(0x602a), 0x1a46 },
	{ CCI_REG16(0x6f12), 0x8a00 },
	{ CCI_REG16(0x602a), 0x1a52 },
	{ CCI_REG16(0x6f12), 0xbf00 },
	{ CCI_REG16(0x602a), 0x0674 },
	{ CCI_REG16(0x6f12), 0x0300 },
	{ CCI_REG16(0x6f12), 0x0300 },
	{ CCI_REG16(0x6f12), 0x0300 },
	{ CCI_REG16(0x6f12), 0x0300 },
	{ CCI_REG16(0x602a), 0x0668 },
	{ CCI_REG16(0x6f12), 0x0800 },
	{ CCI_REG16(0x6f12), 0x0800 },
	{ CCI_REG16(0x6f12), 0x0800 },
	{ CCI_REG16(0x6f12), 0x0800 },
	{ CCI_REG16(0x602a), 0x0684 },
	{ CCI_REG16(0x6f12), 0x4001 },
	{ CCI_REG16(0x602a), 0x0688 },
	{ CCI_REG16(0x6f12), 0x4001 },
	{ CCI_REG16(0x602a), 0x147c },
	{ CCI_REG16(0x6f12), 0x1000 },
	{ CCI_REG16(0x602a), 0x1480 },
	{ CCI_REG16(0x6f12), 0x1000 },
	{ CCI_REG16(0x602a), 0x19f6 },
	{ CCI_REG16(0x6f12), 0x0904 },
	{ CCI_REG16(0x602a), 0x0812 },
	{ CCI_REG16(0x6f12), 0x0000 },
	{ CCI_REG16(0x602a), 0x1a02 },
	{ CCI_REG16(0x6f12), 0x1800 },
	{ CCI_REG16(0x602a), 0x2148 },
	{ CCI_REG16(0x6f12), 0x0100 },
	{ CCI_REG16(0x602a), 0x2042 },
	{ CCI_REG16(0x6f12), 0x1a00 },
	{ CCI_REG16(0x602a), 0x0874 },
	{ CCI_REG16(0x6f12), 0x0100 },
	{ CCI_REG16(0x602a), 0x09c0 },
	{ CCI_REG16(0x6f12), 0x2008 },
	{ CCI_REG16(0x602a), 0x09c4 },
	{ CCI_REG16(0x6f12), 0x2000 },
	{ CCI_REG16(0x602a), 0x19fe },
	{ CCI_REG16(0x6f12), 0x0e1c },
	{ CCI_REG16(0x602a), 0x4d92 },
	{ CCI_REG16(0x6f12), 0x0100 },
	{ CCI_REG16(0x602a), 0x84c8 },
	{ CCI_REG16(0x6f12), 0x0100 },
	{ CCI_REG16(0x602a), 0x4d94 },
	{ CCI_REG16(0x6f12), 0x0005 },
	{ CCI_REG16(0x6f12), 0x000a },
	{ CCI_REG16(0x6f12), 0x0010 },
	{ CCI_REG16(0x6f12), 0x0810 },
	{ CCI_REG16(0x6f12), 0x000a },
	{ CCI_REG16(0x6f12), 0x0040 },
	{ CCI_REG16(0x6f12), 0x0810 },
	{ CCI_REG16(0x6f12), 0x0810 },
	{ CCI_REG16(0x6f12), 0x8002 },
	{ CCI_REG16(0x6f12), 0xfd03 },
	{ CCI_REG16(0x6f12), 0x0010 },
	{ CCI_REG16(0x6f12), 0x1510 },
	{ CCI_REG16(0x602a), 0x3570 },
	{ CCI_REG16(0x6f12), 0x0000 },
	{ CCI_REG16(0x602a), 0x3574 },
	{ CCI_REG16(0x6f12), 0x1201 },
	{ CCI_REG16(0x602a), 0x21e4 },
	{ CCI_REG16(0x6f12), 0x0400 },
	{ CCI_REG16(0x602a), 0x21ec },
	{ CCI_REG16(0x6f12), 0x1f04 },
	{ CCI_REG16(0x602a), 0x2080 },
	{ CCI_REG16(0x6f12), 0x0101 },
	{ CCI_REG16(0x6f12), 0xff00 },
	{ CCI_REG16(0x6f12), 0x7f01 },
	{ CCI_REG16(0x6f12), 0x0001 },
	{ CCI_REG16(0x6f12), 0x8001 },
	{ CCI_REG16(0x6f12), 0xd244 },
	{ CCI_REG16(0x6f12), 0xd244 },
	{ CCI_REG16(0x6f12), 0x14f4 },
	{ CCI_REG16(0x6f12), 0x0000 },
	{ CCI_REG16(0x6f12), 0x0000 },
	{ CCI_REG16(0x6f12), 0x0000 },
	{ CCI_REG16(0x602a), 0x20ba },
	{ CCI_REG16(0x6f12), 0x121c },
	{ CCI_REG16(0x6f12), 0x111c },
	{ CCI_REG16(0x6f12), 0x54f4 },
	{ CCI_REG16(0x602a), 0x120e },
	{ CCI_REG16(0x6f12), 0x1000 },
	{ CCI_REG16(0x602a), 0x212e },
	{ CCI_REG16(0x6f12), 0x0200 },
	{ CCI_REG16(0x602a), 0x13ae },
	{ CCI_REG16(0x6f12), 0x0101 },
	{ CCI_REG16(0x602a), 0x0718 },
	{ CCI_REG16(0x6f12), 0x0001 },
	{ CCI_REG16(0x602a), 0x0710 },
	{ CCI_REG16(0x6f12), 0x0002 },
	{ CCI_REG16(0x6f12), 0x0804 },
	{ CCI_REG16(0x6f12), 0x0100 },
	{ CCI_REG16(0x602a), 0x1b5c },
	{ CCI_REG16(0x6f12), 0x0000 },
	{ CCI_REG16(0x602a), 0x0786 },
	{ CCI_REG16(0x6f12), 0x7701 },
	{ CCI_REG16(0x602a), 0x2022 },
	{ CCI_REG16(0x6f12), 0x0500 },
	{ CCI_REG16(0x6f12), 0x0500 },
	{ CCI_REG16(0x602a), 0x1360 },
	{ CCI_REG16(0x6f12), 0x0100 },
	{ CCI_REG16(0x602a), 0x1376 },
	{ CCI_REG16(0x6f12), 0x0100 },
	{ CCI_REG16(0x6f12), 0x6038 },
	{ CCI_REG16(0x6f12), 0x7038 },
	{ CCI_REG16(0x6f12), 0x8038 },
	{ CCI_REG16(0x602a), 0x1386 },
	{ CCI_REG16(0x6f12), 0x0b00 },
	{ CCI_REG16(0x602a), 0x06fa },
	{ CCI_REG16(0x6f12), 0x0000 },
	{ CCI_REG16(0x602a), 0x4a94 },
	{ CCI_REG16(0x6f12), 0x0900 },
	{ CCI_REG16(0x6f12), 0x0000 },
	{ CCI_REG16(0x6f12), 0x0300 },
	{ CCI_REG16(0x6f12), 0x0000 },
	{ CCI_REG16(0x6f12), 0x0000 },
	{ CCI_REG16(0x6f12), 0x0000 },
	{ CCI_REG16(0x6f12), 0x0000 },
	{ CCI_REG16(0x6f12), 0x0000 },
	{ CCI_REG16(0x6f12), 0x0300 },
	{ CCI_REG16(0x6f12), 0x0000 },
	{ CCI_REG16(0x6f12), 0x0900 },
	{ CCI_REG16(0x6f12), 0x0000 },
	{ CCI_REG16(0x6f12), 0x0000 },
	{ CCI_REG16(0x6f12), 0x0000 },
	{ CCI_REG16(0x6f12), 0x0000 },
	{ CCI_REG16(0x6f12), 0x0000 },
	{ CCI_REG16(0x602a), 0x0a76 },
	{ CCI_REG16(0x6f12), 0x1000 },
	{ CCI_REG16(0x602a), 0x0aee },
	{ CCI_REG16(0x6f12), 0x1000 },
	{ CCI_REG16(0x602a), 0x0b66 },
	{ CCI_REG16(0x6f12), 0x1000 },
	{ CCI_REG16(0x602a), 0x0bde },
	{ CCI_REG16(0x6f12), 0x1000 },
	{ CCI_REG16(0x602a), 0x0be8 },
	{ CCI_REG16(0x6f12), 0x3000 },
	{ CCI_REG16(0x6f12), 0x3000 },
	{ CCI_REG16(0x602a), 0x0c56 },
	{ CCI_REG16(0x6f12), 0x1000 },
	{ CCI_REG16(0x602a), 0x0c60 },
	{ CCI_REG16(0x6f12), 0x3000 },
	{ CCI_REG16(0x6f12), 0x3000 },
	{ CCI_REG16(0x602a), 0x0cb6 },
	{ CCI_REG16(0x6f12), 0x0100 },
	{ CCI_REG16(0x602a), 0x0cf2 },
	{ CCI_REG16(0x6f12), 0x0001 },
	{ CCI_REG16(0x602a), 0x0cf0 },
	{ CCI_REG16(0x6f12), 0x0101 },
	{ CCI_REG16(0x602a), 0x11b8 },
	{ CCI_REG16(0x6f12), 0x0100 },
	{ CCI_REG16(0x602a), 0x11f6 },
	{ CCI_REG16(0x6f12), 0x0020 },
	{ CCI_REG16(0x602a), 0x4a74 },
	{ CCI_REG16(0x6f12), 0x0000 },
	{ CCI_REG16(0x6f12), 0x0000 },
	{ CCI_REG16(0x6f12), 0xd8ff },
	{ CCI_REG16(0x6f12), 0x0000 },
	{ CCI_REG16(0x6f12), 0x0000 },
	{ CCI_REG16(0x6f12), 0x0000 },
	{ CCI_REG16(0x6f12), 0x0000 },
	{ CCI_REG16(0x6f12), 0x0000 },
	{ CCI_REG16(0x6f12), 0xd8ff },
	{ CCI_REG16(0x6f12), 0x0000 },
	{ CCI_REG16(0x6f12), 0x0000 },
	{ CCI_REG16(0x6f12), 0x0000 },
	{ CCI_REG16(0x6f12), 0x0000 },
	{ CCI_REG16(0x6f12), 0x0000 },
	{ CCI_REG16(0x6f12), 0x0000 },
	{ CCI_REG16(0x6f12), 0x0000 },
	{ CCI_REG16(0x602a), 0x218e },
	{ CCI_REG16(0x6f12), 0x0000 },
	{ CCI_REG16(0x602a), 0x2268 },
	{ CCI_REG16(0x6f12), 0xf279 },
	{ CCI_REG16(0x602a), 0x5006 },
	{ CCI_REG16(0x6f12), 0x0000 },
	{ CCI_REG16(0x602a), 0x500e },
	{ CCI_REG16(0x6f12), 0x0100 },
	{ CCI_REG16(0x602a), 0x4e70 },
	{ CCI_REG16(0x6f12), 0x2062 },
	{ CCI_REG16(0x6f12), 0x5501 },
	{ CCI_REG16(0x602a), 0x06dc },
	{ CCI_REG16(0x6f12), 0x0000 },
	{ CCI_REG16(0x6f12), 0x0000 },
	{ CCI_REG16(0x6f12), 0x0000 },
	{ CCI_REG16(0x6f12), 0x0000 },
	{ CCI_REG16(0x6028), 0x4000 },
	{ CCI_REG16(0xf46a), 0xae80 },
	{ CCI_REG16(0x0344), 0x0000 },
	{ CCI_REG16(0x0346), 0x0308 },
	{ CCI_REG16(0x0348), 0x1fff },
	{ CCI_REG16(0x034a), 0x1517 },
	{ CCI_REG16(0x034c), 0x0ff0 },
	{ CCI_REG16(0x034e), 0x08f8 },
	{ CCI_REG16(0x0350), 0x0008 },
	{ CCI_REG16(0x0352), 0x0008 },
	{ CCI_REG16(0x0900), 0x0122 },
	{ CCI_REG16(0x0380), 0x0002 },
	{ CCI_REG16(0x0382), 0x0002 },
	{ CCI_REG16(0x0384), 0x0002 },
	{ CCI_REG16(0x0386), 0x0002 },
	{ CCI_REG16(0x0110), 0x1002 },
	{ CCI_REG16(0x0114), 0x0301 },
	{ CCI_REG16(0x0116), 0x3000 },
	{ CCI_REG16(0x0136), 0x1300 },
	{ CCI_REG16(0x013e), 0x00c8 },
	{ CCI_REG16(0x0300), 0x0006 },
	{ CCI_REG16(0x0302), 0x0001 },
	{ CCI_REG16(0x0304), 0x0003 },
	{ CCI_REG16(0x0306), 0x0083 },
	{ CCI_REG16(0x0308), 0x0008 },
	{ CCI_REG16(0x030a), 0x0001 },
	{ CCI_REG16(0x030c), 0x0000 },
	{ CCI_REG16(0x030e), 0x0003 },
	{ CCI_REG16(0x0310), 0x0086 },
	{ CCI_REG16(0x0312), 0x0000 },
	{ CCI_REG16(0x080e), 0x0000 },
	{ CCI_REG16(0x0340), 0x0c5c },
	{ CCI_REG16(0x0342), 0x1700 },
	{ CCI_REG16(0x0702), 0x0000 },
	{ CCI_REG16(0x0202), 0x0100 },
	{ CCI_REG16(0x0200), 0x0100 },
	{ CCI_REG16(0x0d00), 0x0101 },
	{ CCI_REG16(0x0d02), 0x0101 },
	{ CCI_REG16(0x0d04), 0x0102 },
	{ CCI_REG16(0x6226), 0x0000 },
};

static const struct cci_reg_sequence s5kjn1_3840x2160_regs[] = {
	{ CCI_REG16(0x6028), 0x2400 },
	{ CCI_REG16(0x602a), 0x1a28 },
	{ CCI_REG16(0x6f12), 0x4c00 },
	{ CCI_REG16(0x602a), 0x065a },
	{ CCI_REG16(0x6f12), 0x0000 },
	{ CCI_REG16(0x602a), 0x139e },
	{ CCI_REG16(0x6f12), 0x0100 },
	{ CCI_REG16(0x602a), 0x139c },
	{ CCI_REG16(0x6f12), 0x0000 },
	{ CCI_REG16(0x602a), 0x13a0 },
	{ CCI_REG16(0x6f12), 0x0a00 },
	{ CCI_REG16(0x6f12), 0x0120 },
	{ CCI_REG16(0x602a), 0x2072 },
	{ CCI_REG16(0x6f12), 0x0000 },
	{ CCI_REG16(0x602a), 0x1a64 },
	{ CCI_REG16(0x6f12), 0x0301 },
	{ CCI_REG16(0x6f12), 0xff00 },
	{ CCI_REG16(0x602a), 0x19e6 },
	{ CCI_REG16(0x6f12), 0x0200 },
	{ CCI_REG16(0x602a), 0x1a30 },
	{ CCI_REG16(0x6f12), 0x3401 },
	{ CCI_REG16(0x602a), 0x19fc },
	{ CCI_REG16(0x6f12), 0x0b00 },
	{ CCI_REG16(0x602a), 0x19f4 },
	{ CCI_REG16(0x6f12), 0x0606 },
	{ CCI_REG16(0x602a), 0x19f8 },
	{ CCI_REG16(0x6f12), 0x1010 },
	{ CCI_REG16(0x602a), 0x1b26 },
	{ CCI_REG16(0x6f12), 0x6f80 },
	{ CCI_REG16(0x6f12), 0xa060 },
	{ CCI_REG16(0x602a), 0x1a3c },
	{ CCI_REG16(0x6f12), 0x6207 },
	{ CCI_REG16(0x602a), 0x1a48 },
	{ CCI_REG16(0x6f12), 0x6207 },
	{ CCI_REG16(0x602a), 0x1444 },
	{ CCI_REG16(0x6f12), 0x2000 },
	{ CCI_REG16(0x6f12), 0x2000 },
	{ CCI_REG16(0x602a), 0x144c },
	{ CCI_REG16(0x6f12), 0x3f00 },
	{ CCI_REG16(0x6f12), 0x3f00 },
	{ CCI_REG16(0x602a), 0x7f6c },
	{ CCI_REG16(0x6f12), 0x0100 },
	{ CCI_REG16(0x6f12), 0x2f00 },
	{ CCI_REG16(0x6f12), 0xfa00 },
	{ CCI_REG16(0x6f12), 0x2400 },
	{ CCI_REG16(0x6f12), 0xe500 },
	{ CCI_REG16(0x602a), 0x0650 },
	{ CCI_REG16(0x6f12), 0x0600 },
	{ CCI_REG16(0x602a), 0x0654 },
	{ CCI_REG16(0x6f12), 0x0000 },
	{ CCI_REG16(0x602a), 0x1a46 },
	{ CCI_REG16(0x6f12), 0x8a00 },
	{ CCI_REG16(0x602a), 0x1a52 },
	{ CCI_REG16(0x6f12), 0xbf00 },
	{ CCI_REG16(0x602a), 0x0674 },
	{ CCI_REG16(0x6f12), 0x0300 },
	{ CCI_REG16(0x6f12), 0x0300 },
	{ CCI_REG16(0x6f12), 0x0300 },
	{ CCI_REG16(0x6f12), 0x0300 },
	{ CCI_REG16(0x602a), 0x0668 },
	{ CCI_REG16(0x6f12), 0x0800 },
	{ CCI_REG16(0x6f12), 0x0800 },
	{ CCI_REG16(0x6f12), 0x0800 },
	{ CCI_REG16(0x6f12), 0x0800 },
	{ CCI_REG16(0x602a), 0x0684 },
	{ CCI_REG16(0x6f12), 0x4001 },
	{ CCI_REG16(0x602a), 0x0688 },
	{ CCI_REG16(0x6f12), 0x4001 },
	{ CCI_REG16(0x602a), 0x147c },
	{ CCI_REG16(0x6f12), 0x1000 },
	{ CCI_REG16(0x602a), 0x1480 },
	{ CCI_REG16(0x6f12), 0x1000 },
	{ CCI_REG16(0x602a), 0x19f6 },
	{ CCI_REG16(0x6f12), 0x0904 },
	{ CCI_REG16(0x602a), 0x0812 },
	{ CCI_REG16(0x6f12), 0x0000 },
	{ CCI_REG16(0x602a), 0x1a02 },
	{ CCI_REG16(0x6f12), 0x1800 },
	{ CCI_REG16(0x602a), 0x2148 },
	{ CCI_REG16(0x6f12), 0x0100 },
	{ CCI_REG16(0x602a), 0x2042 },
	{ CCI_REG16(0x6f12), 0x1a00 },
	{ CCI_REG16(0x602a), 0x0874 },
	{ CCI_REG16(0x6f12), 0x0100 },
	{ CCI_REG16(0x602a), 0x09c0 },
	{ CCI_REG16(0x6f12), 0x2008 },
	{ CCI_REG16(0x602a), 0x09c4 },
	{ CCI_REG16(0x6f12), 0x9800 },
	{ CCI_REG16(0x602a), 0x19fe },
	{ CCI_REG16(0x6f12), 0x0e1c },
	{ CCI_REG16(0x602a), 0x4d92 },
	{ CCI_REG16(0x6f12), 0x0100 },
	{ CCI_REG16(0x602a), 0x84c8 },
	{ CCI_REG16(0x6f12), 0x0100 },
	{ CCI_REG16(0x602a), 0x4d94 },
	{ CCI_REG16(0x6f12), 0x0005 },
	{ CCI_REG16(0x6f12), 0x000a },
	{ CCI_REG16(0x6f12), 0x0010 },
	{ CCI_REG16(0x6f12), 0x0810 },
	{ CCI_REG16(0x6f12), 0x000a },
	{ CCI_REG16(0x6f12), 0x0040 },
	{ CCI_REG16(0x6f12), 0x0810 },
	{ CCI_REG16(0x6f12), 0x0810 },
	{ CCI_REG16(0x6f12), 0x8002 },
	{ CCI_REG16(0x6f12), 0xfd03 },
	{ CCI_REG16(0x6f12), 0x0010 },
	{ CCI_REG16(0x6f12), 0x1510 },
	{ CCI_REG16(0x602a), 0x3570 },
	{ CCI_REG16(0x6f12), 0x0000 },
	{ CCI_REG16(0x602a), 0x3574 },
	{ CCI_REG16(0x6f12), 0x6100 },
	{ CCI_REG16(0x602a), 0x21e4 },
	{ CCI_REG16(0x6f12), 0x0400 },
	{ CCI_REG16(0x602a), 0x21ec },
	{ CCI_REG16(0x6f12), 0xef03 },
	{ CCI_REG16(0x602a), 0x2080 },
	{ CCI_REG16(0x6f12), 0x0101 },
	{ CCI_REG16(0x6f12), 0xff00 },
	{ CCI_REG16(0x6f12), 0x7f01 },
	{ CCI_REG16(0x6f12), 0x0001 },
	{ CCI_REG16(0x6f12), 0x8001 },
	{ CCI_REG16(0x6f12), 0xd244 },
	{ CCI_REG16(0x6f12), 0xd244 },
	{ CCI_REG16(0x6f12), 0x14f4 },
	{ CCI_REG16(0x6f12), 0x0000 },
	{ CCI_REG16(0x6f12), 0x0000 },
	{ CCI_REG16(0x6f12), 0x0000 },
	{ CCI_REG16(0x602a), 0x20ba },
	{ CCI_REG16(0x6f12), 0x121c },
	{ CCI_REG16(0x6f12), 0x111c },
	{ CCI_REG16(0x6f12), 0x54f4 },
	{ CCI_REG16(0x602a), 0x120e },
	{ CCI_REG16(0x6f12), 0x1000 },
	{ CCI_REG16(0x602a), 0x212e },
	{ CCI_REG16(0x6f12), 0x0200 },
	{ CCI_REG16(0x602a), 0x13ae },
	{ CCI_REG16(0x6f12), 0x0101 },
	{ CCI_REG16(0x602a), 0x0718 },
	{ CCI_REG16(0x6f12), 0x0001 },
	{ CCI_REG16(0x602a), 0x0710 },
	{ CCI_REG16(0x6f12), 0x0002 },
	{ CCI_REG16(0x6f12), 0x0804 },
	{ CCI_REG16(0x6f12), 0x0100 },
	{ CCI_REG16(0x602a), 0x1b5c },
	{ CCI_REG16(0x6f12), 0x0000 },
	{ CCI_REG16(0x602a), 0x0786 },
	{ CCI_REG16(0x6f12), 0x7701 },
	{ CCI_REG16(0x602a), 0x2022 },
	{ CCI_REG16(0x6f12), 0x0500 },
	{ CCI_REG16(0x6f12), 0x0500 },
	{ CCI_REG16(0x602a), 0x1360 },
	{ CCI_REG16(0x6f12), 0x0100 },
	{ CCI_REG16(0x602a), 0x1376 },
	{ CCI_REG16(0x6f12), 0x0100 },
	{ CCI_REG16(0x6f12), 0x6038 },
	{ CCI_REG16(0x6f12), 0x7038 },
	{ CCI_REG16(0x6f12), 0x8038 },
	{ CCI_REG16(0x602a), 0x1386 },
	{ CCI_REG16(0x6f12), 0x0b00 },
	{ CCI_REG16(0x602a), 0x06fa },
	{ CCI_REG16(0x6f12), 0x1000 },
	{ CCI_REG16(0x602a), 0x4a94 },
	{ CCI_REG16(0x6f12), 0x0900 },
	{ CCI_REG16(0x6f12), 0x0000 },
	{ CCI_REG16(0x6f12), 0x0300 },
	{ CCI_REG16(0x6f12), 0x0000 },
	{ CCI_REG16(0x6f12), 0x0000 },
	{ CCI_REG16(0x6f12), 0x0000 },
	{ CCI_REG16(0x6f12), 0x0000 },
	{ CCI_REG16(0x6f12), 0x0000 },
	{ CCI_REG16(0x6f12), 0x0300 },
	{ CCI_REG16(0x6f12), 0x0000 },
	{ CCI_REG16(0x6f12), 0x0900 },
	{ CCI_REG16(0x6f12), 0x0000 },
	{ CCI_REG16(0x6f12), 0x0000 },
	{ CCI_REG16(0x6f12), 0x0000 },
	{ CCI_REG16(0x6f12), 0x0000 },
	{ CCI_REG16(0x6f12), 0x0000 },
	{ CCI_REG16(0x602a), 0x0a76 },
	{ CCI_REG16(0x6f12), 0x1000 },
	{ CCI_REG16(0x602a), 0x0aee },
	{ CCI_REG16(0x6f12), 0x1000 },
	{ CCI_REG16(0x602a), 0x0b66 },
	{ CCI_REG16(0x6f12), 0x1000 },
	{ CCI_REG16(0x602a), 0x0bde },
	{ CCI_REG16(0x6f12), 0x1000 },
	{ CCI_REG16(0x602a), 0x0be8 },
	{ CCI_REG16(0x6f12), 0x3000 },
	{ CCI_REG16(0x6f12), 0x3000 },
	{ CCI_REG16(0x602a), 0x0c56 },
	{ CCI_REG16(0x6f12), 0x1000 },
	{ CCI_REG16(0x602a), 0x0c60 },
	{ CCI_REG16(0x6f12), 0x3000 },
	{ CCI_REG16(0x6f12), 0x3000 },
	{ CCI_REG16(0x602a), 0x0cb6 },
	{ CCI_REG16(0x6f12), 0x0100 },
	{ CCI_REG16(0x602a), 0x0cf2 },
	{ CCI_REG16(0x6f12), 0x0001 },
	{ CCI_REG16(0x602a), 0x0cf0 },
	{ CCI_REG16(0x6f12), 0x0101 },
	{ CCI_REG16(0x602a), 0x11b8 },
	{ CCI_REG16(0x6f12), 0x0100 },
	{ CCI_REG16(0x602a), 0x11f6 },
	{ CCI_REG16(0x6f12), 0x0020 },
	{ CCI_REG16(0x602a), 0x4a74 },
	{ CCI_REG16(0x6f12), 0x0000 },
	{ CCI_REG16(0x6f12), 0x0000 },
	{ CCI_REG16(0x6f12), 0xd8ff },
	{ CCI_REG16(0x6f12), 0x0000 },
	{ CCI_REG16(0x6f12), 0x0000 },
	{ CCI_REG16(0x6f12), 0x0000 },
	{ CCI_REG16(0x6f12), 0x0000 },
	{ CCI_REG16(0x6f12), 0x0000 },
	{ CCI_REG16(0x6f12), 0xd8ff },
	{ CCI_REG16(0x6f12), 0x0000 },
	{ CCI_REG16(0x6f12), 0x0000 },
	{ CCI_REG16(0x6f12), 0x0000 },
	{ CCI_REG16(0x6f12), 0x0000 },
	{ CCI_REG16(0x6f12), 0x0000 },
	{ CCI_REG16(0x6f12), 0x0000 },
	{ CCI_REG16(0x6f12), 0x0000 },
	{ CCI_REG16(0x602a), 0x218e },
	{ CCI_REG16(0x6f12), 0x0000 },
	{ CCI_REG16(0x602a), 0x2268 },
	{ CCI_REG16(0x6f12), 0xf279 },
	{ CCI_REG16(0x602a), 0x5006 },
	{ CCI_REG16(0x6f12), 0x0000 },
	{ CCI_REG16(0x602a), 0x500e },
	{ CCI_REG16(0x6f12), 0x0100 },
	{ CCI_REG16(0x602a), 0x4e70 },
	{ CCI_REG16(0x6f12), 0x2062 },
	{ CCI_REG16(0x6f12), 0x5501 },
	{ CCI_REG16(0x602a), 0x06dc },
	{ CCI_REG16(0x6f12), 0x0000 },
	{ CCI_REG16(0x6f12), 0x0000 },
	{ CCI_REG16(0x6f12), 0x0000 },
	{ CCI_REG16(0x6f12), 0x0000 },
	{ CCI_REG16(0x6028), 0x4000 },
	{ CCI_REG16(0xf46a), 0xae80 },
	{ CCI_REG16(0x0344), 0x00f0 },
	{ CCI_REG16(0x0346), 0x0390 },
	{ CCI_REG16(0x0348), 0x1f0f },
	{ CCI_REG16(0x034a), 0x148f },
	{ CCI_REG16(0x034c), 0x0f00 },
	{ CCI_REG16(0x034e), 0x0870 },
	{ CCI_REG16(0x0350), 0x0008 },
	{ CCI_REG16(0x0352), 0x0008 },
	{ CCI_REG16(0x0900), 0x0122 },
	{ CCI_REG16(0x0380), 0x0002 },
	{ CCI_REG16(0x0382), 0x0002 },
	{ CCI_REG16(0x0384), 0x0002 },
	{ CCI_REG16(0x0386), 0x0002 },
	{ CCI_REG16(0x0110), 0x1002 },
	{ CCI_REG16(0x0114), 0x0301 },
	{ CCI_REG16(0x0116), 0x3000 },
	{ CCI_REG16(0x0136), 0x1300 },
	{ CCI_REG16(0x013e), 0x00c8 },
	{ CCI_REG16(0x0300), 0x0006 },
	{ CCI_REG16(0x0302), 0x0001 },
	{ CCI_REG16(0x0304), 0x0003 },
	{ CCI_REG16(0x0306), 0x0083 },
	{ CCI_REG16(0x0308), 0x0008 },
	{ CCI_REG16(0x030a), 0x0001 },
	{ CCI_REG16(0x030c), 0x0000 },
	{ CCI_REG16(0x030e), 0x0003 },
	{ CCI_REG16(0x0310), 0x0086 },
	{ CCI_REG16(0x0312), 0x0000 },
	{ CCI_REG16(0x080e), 0x0000 },
	{ CCI_REG16(0x0340), 0x08e2 },
	{ CCI_REG16(0x0342), 0x1000 },
	{ CCI_REG16(0x0702), 0x0000 },
	{ CCI_REG16(0x0202), 0x0100 },
	{ CCI_REG16(0x0200), 0x0100 },
	{ CCI_REG16(0x0d00), 0x0101 },
	{ CCI_REG16(0x0d02), 0x0101 },
	{ CCI_REG16(0x0d04), 0x0102 },
	{ CCI_REG16(0x6226), 0x0000 },
};

static const char * const s5kjn1_test_pattern_menu[] = {
	"Disabled",
	"Solid Colour",
	"Eight Vertical Colour Bars",
	"Colour Bars With Fade to Grey",
	"Pseudorandom Sequence (PN9)",
};

static const s64 link_freq_menu_items[] = {
	857600000,
};

static u64 to_pixel_rate(u32 f_index)
{
	u64 pixel_rate = link_freq_menu_items[f_index] * 2 * S5KJN1_DATA_LANES;

	do_div(pixel_rate, S5KJN1_BITS_PER_SAMPLE);

	return pixel_rate;
}

static const struct s5kjn1_mode supported_modes[] = {
	/* 4:3 8k 10fps */
	{
		.width = 8160,
		.height = 6120,
		.crop = {
			.left = S5KJN1_PIXEL_ARRAY_LEFT,
			.top = 12 + S5KJN1_PIXEL_ARRAY_TOP,
			.width = 8160,
			.height = 6120,
		},
		.line_length = 8688,
		.frame_length = 6400,
		.max_again = 512,
		.again = 256,
		.reg_list = {
			.num_of_regs = ARRAY_SIZE(s5kjn1_8160x6120_regs),
			.regs = s5kjn1_8160x6120_regs,
		},
	},
	/* 4:3 4k 30fps */
	{
		.width = 4080,
		.height = 3060,
		.crop = {
			.left = S5KJN1_PIXEL_ARRAY_LEFT,
			.top = 12 + S5KJN1_PIXEL_ARRAY_TOP,
			.width = 4080 * 2,
			.height = 3060 * 2,
		},
		.line_length = 5888,
		.frame_length = 3164,
		.max_again = S5KJN1_ANA_GAIN_MAX,
		.again = S5KJN1_ANA_GAIN_DEFAULT,
		.reg_list = {
			.num_of_regs = ARRAY_SIZE(s5kjn1_4080x3060_regs),
			.regs = s5kjn1_4080x3060_regs,
		},
	},
	/* 16:9 4k 30fps */
	{
		.width = 4080,
		.height = 2296,
		.crop = {
			.left = S5KJN1_PIXEL_ARRAY_LEFT,
			.top = 776 + S5KJN1_PIXEL_ARRAY_TOP,
			.width = 4080 * 2,
			.height = 2296 * 2,
		},
		.line_length = 5888,
		.frame_length = 3164,
		.max_again = S5KJN1_ANA_GAIN_MAX,
		.again = S5KJN1_ANA_GAIN_DEFAULT,
		.reg_list = {
			.num_of_regs = ARRAY_SIZE(s5kjn1_4080x2296_regs),
			.regs = s5kjn1_4080x2296_regs,
		},
	},
	/* 16:9 4k 60fps */
	{
		.width = 3840,
		.height = 2160,
		.crop = {
			.left = 240 + S5KJN1_PIXEL_ARRAY_LEFT,
			.top = 912 + S5KJN1_PIXEL_ARRAY_TOP,
			.width = 3840 * 2,
			.height = 2160 * 2,
		},
		.line_length = 4096,
		.frame_length = 2274,
		.max_again = S5KJN1_ANA_GAIN_MAX,
		.again = S5KJN1_ANA_GAIN_DEFAULT,
		.reg_list = {
			.num_of_regs = ARRAY_SIZE(s5kjn1_3840x2160_regs),
			.regs = s5kjn1_3840x2160_regs,
		},
	},
};

static int s5kjn1_check_hwcfg(struct device *dev, struct s5kjn1 *s5kjn1)
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

static int __s5kjn1_start_stream(struct s5kjn1 *s5kjn1)
{
	const struct s5kjn1_reg_list *reg_list;
	int ret;

	/* Apply default values of current mode */
	reg_list = &s5kjn1->cur_mode->reg_list;

	ret = cci_write(s5kjn1->regmap, CCI_REG16(0x6028), 0x4000, NULL);
	if (ret)
		return ret;

	ret = cci_write(s5kjn1->regmap, CCI_REG16(0x6010), 0x0001, NULL);
	if (ret)
		return ret;

	usleep_range(5000, 6000);

	ret = cci_write(s5kjn1->regmap, CCI_REG16(0x6226), 0x0001, NULL);
	if (ret)
		return ret;

	usleep_range(10000, 11000);

	ret = cci_multi_reg_write(s5kjn1->regmap, s5kjn1_init_regs, ARRAY_SIZE(s5kjn1_init_regs), NULL);
	if (ret)
		return ret;

	ret = cci_multi_reg_write(s5kjn1->regmap, reg_list->regs, reg_list->num_of_regs, NULL);
	if (ret)
		return ret;

	/* Apply customized values from user */
	ret = __v4l2_ctrl_handler_setup(s5kjn1->subdev.ctrl_handler);
	if (ret)
		return ret;

	ret = cci_write(s5kjn1->regmap, CCI_REG8(0x0100), 0x01, NULL);
	if (ret)
		return ret;

	return 0;
}

static int __s5kjn1_stop_stream(struct s5kjn1 *s5kjn1)
{
	int ret;

	ret = cci_write(s5kjn1->regmap, CCI_REG8(0x0100), 0x00, NULL);
	if (ret)
		return ret;

	return 0;
}

static int s5kjn1_s_stream(struct v4l2_subdev *sd, int on)
{
	struct s5kjn1 *s5kjn1 = to_s5kjn1(sd);
	struct i2c_client *client = v4l2_get_subdevdata(&s5kjn1->subdev);
	int ret;

	mutex_lock(&s5kjn1->mutex);

	if (s5kjn1->streaming == on) {
		ret = 0;
		goto unlock_and_return;
	}

	if (on) {
		ret = pm_runtime_resume_and_get(&client->dev);
		if (ret < 0)
			goto unlock_and_return;

		ret = __s5kjn1_start_stream(s5kjn1);
		if (ret) {
			__s5kjn1_stop_stream(s5kjn1);
			s5kjn1->streaming = !on;
			goto err_rpm_put;
		}
	} else {
		__s5kjn1_stop_stream(s5kjn1);
		pm_runtime_put(&client->dev);
	}

	s5kjn1->streaming = on;
	mutex_unlock(&s5kjn1->mutex);

	return 0;

err_rpm_put:
	pm_runtime_put(&client->dev);
unlock_and_return:
	mutex_unlock(&s5kjn1->mutex);

	return ret;
}

static int s5kjn1_enum_mbus_code(struct v4l2_subdev *sd,
				 struct v4l2_subdev_state *sd_state,
				 struct v4l2_subdev_mbus_code_enum *code)
{
	if (code->index != 0)
		return -EINVAL;

	code->code = MEDIA_BUS_FMT_SGRBG10_1X10;

	return 0;
}

static int s5kjn1_enum_frame_sizes(struct v4l2_subdev *sd,
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

static void s5kjn1_fill_fmt(const struct s5kjn1_mode *mode,
			    struct v4l2_mbus_framefmt *fmt)
{
	fmt->width = mode->width;
	fmt->height = mode->height;
	fmt->code = MEDIA_BUS_FMT_SGRBG10_1X10;
	fmt->field = V4L2_FIELD_NONE;
}

static int s5kjn1_get_fmt(struct v4l2_subdev *sd,
			  struct v4l2_subdev_state *sd_state,
			  struct v4l2_subdev_format *fmt)
{
	struct s5kjn1 *s5kjn1 = to_s5kjn1(sd);

	mutex_lock(&s5kjn1->mutex);

	if (fmt->which == V4L2_SUBDEV_FORMAT_TRY)
		fmt->format = *v4l2_subdev_state_get_format(sd_state, fmt->pad);
	else
		s5kjn1_fill_fmt(s5kjn1->cur_mode, &fmt->format);

	mutex_unlock(&s5kjn1->mutex);

	return 0;
}

static int s5kjn1_set_fmt(struct v4l2_subdev *sd,
			  struct v4l2_subdev_state *sd_state,
			  struct v4l2_subdev_format *fmt)
{
	struct s5kjn1 *s5kjn1 = to_s5kjn1(sd);
	const struct s5kjn1_mode *mode;
	struct v4l2_rect *crop;
	s32 vblank_def, hblank_def;
	int ret = 0;

	mode = v4l2_find_nearest_size(supported_modes,
				      ARRAY_SIZE(supported_modes), width,
				      height, fmt->format.width,
				      fmt->format.height);

	mutex_lock(&s5kjn1->mutex);

	if (s5kjn1->streaming && fmt->which == V4L2_SUBDEV_FORMAT_ACTIVE) {
		ret = -EBUSY;
		goto out_unlock;
	}

	s5kjn1_fill_fmt(mode, &fmt->format);

	if (fmt->which == V4L2_SUBDEV_FORMAT_TRY) {
		*v4l2_subdev_state_get_format(sd_state, 0) = fmt->format;
		crop = v4l2_subdev_state_get_crop(sd_state, 0);
		crop->left = mode->crop.left;
		crop->top = mode->crop.top;
		crop->width = mode->crop.width;
		crop->height = mode->crop.height;
	} else {
		s5kjn1->cur_mode = mode;

		__v4l2_ctrl_modify_range(s5kjn1->again, S5KJN1_ANA_GAIN_MIN,
					 mode->max_again, 1,
					 mode->again);

		vblank_def = mode->frame_length - mode->height;
		__v4l2_ctrl_modify_range(s5kjn1->vblank, vblank_def,
					 S5KJN1_FRAME_LENGTH_MAX - mode->height, 1,
					 vblank_def);
		__v4l2_ctrl_s_ctrl(s5kjn1->vblank, vblank_def);

		hblank_def = mode->line_length - mode->width;
		__v4l2_ctrl_modify_range(s5kjn1->hblank, hblank_def, hblank_def, 1,
					 hblank_def);
	}

out_unlock:
	mutex_unlock(&s5kjn1->mutex);
	return ret;
}

static int s5kjn1_get_selection(struct v4l2_subdev *sd,
				struct v4l2_subdev_state *sd_state,
				struct v4l2_subdev_selection *sel)
{
	switch (sel->target) {
	case V4L2_SEL_TGT_CROP: {
		struct s5kjn1 *s5kjn1 = to_s5kjn1(sd);

		mutex_lock(&s5kjn1->mutex);
		switch (sel->which) {
		case V4L2_SUBDEV_FORMAT_TRY:
			sel->r = *v4l2_subdev_state_get_crop(sd_state, sel->pad);
			break;
		case V4L2_SUBDEV_FORMAT_ACTIVE:
			sel->r = s5kjn1->cur_mode->crop;
			break;
		}
		mutex_unlock(&s5kjn1->mutex);
		return 0;
	}
	case V4L2_SEL_TGT_NATIVE_SIZE:
		sel->r.left = 0;
		sel->r.top = 0;
		sel->r.width = S5KJN1_NATIVE_WIDTH;
		sel->r.height = S5KJN1_NATIVE_HEIGHT;
		return 0;
	case V4L2_SEL_TGT_CROP_DEFAULT:
	case V4L2_SEL_TGT_CROP_BOUNDS:
		sel->r.left = S5KJN1_PIXEL_ARRAY_LEFT;
		sel->r.top = S5KJN1_PIXEL_ARRAY_TOP;
		sel->r.width = S5KJN1_PIXEL_ARRAY_WIDTH;
		sel->r.height = S5KJN1_PIXEL_ARRAY_HEIGHT;
		return 0;
	}

	return -EINVAL;
}

static int s5kjn1_init_state(struct v4l2_subdev *sd,
			     struct v4l2_subdev_state *sd_state)
{
	struct v4l2_subdev_format fmt = { 0 };

	fmt.which = sd_state ? V4L2_SUBDEV_FORMAT_TRY : V4L2_SUBDEV_FORMAT_ACTIVE;

	s5kjn1_set_fmt(sd, sd_state, &fmt);

	return 0;
}

static int s5kjn1_set_ctrl(struct v4l2_ctrl *ctrl)
{
	struct s5kjn1 *s5kjn1 = container_of(ctrl->handler,
					     struct s5kjn1, ctrl_handler);
	struct i2c_client *client = v4l2_get_subdevdata(&s5kjn1->subdev);
	s32 max_expo;
	int ret;

	/* Propagate change of current control to all related controls */
	if (ctrl->id == V4L2_CID_VBLANK) {
		/* Update max exposure while meeting expected vblanking */
		max_expo = s5kjn1->cur_mode->height + ctrl->val -
			   S5KJN1_EXPOSURE_MAX_MARGIN;
		__v4l2_ctrl_modify_range(s5kjn1->exposure,
					 s5kjn1->exposure->minimum, max_expo,
					 s5kjn1->exposure->step,
					 s5kjn1->exposure->default_value);
	}

	/* V4L2 controls values will be applied only when power is already up */
	if (!pm_runtime_get_if_in_use(&client->dev))
		return 0;

	switch (ctrl->id) {
	case V4L2_CID_EXPOSURE:
		ret = cci_write(s5kjn1->regmap, S5KJN1_REG_EXPO, ctrl->val, NULL);
		break;
	case V4L2_CID_ANALOGUE_GAIN:
		ret = cci_write(s5kjn1->regmap, S5KJN1_REG_GAIN, ctrl->val, NULL);
		break;
	case V4L2_CID_VBLANK:
		ret = cci_write(s5kjn1->regmap, S5KJN1_REG_FRAME_LENGTH,
				s5kjn1->cur_mode->height + ctrl->val, NULL);
		break;
	case V4L2_CID_TEST_PATTERN:
		ret = cci_write(s5kjn1->regmap, S5KJN1_REG_TEST_PATTERN, ctrl->val, NULL);
		break;
	default:
		ret = -EINVAL;
		break;
	}

	pm_runtime_put(&client->dev);

	return ret;
}

static const struct v4l2_subdev_video_ops s5kjn1_video_ops = {
	.s_stream = s5kjn1_s_stream,
};

static const struct v4l2_subdev_pad_ops s5kjn1_pad_ops = {
	.enum_mbus_code = s5kjn1_enum_mbus_code,
	.enum_frame_size = s5kjn1_enum_frame_sizes,
	.get_selection = s5kjn1_get_selection,
	.get_fmt = s5kjn1_get_fmt,
	.set_fmt = s5kjn1_set_fmt,
};

static const struct v4l2_subdev_ops s5kjn1_subdev_ops = {
	.video = &s5kjn1_video_ops,
	.pad = &s5kjn1_pad_ops,
};

static const struct media_entity_operations s5kjn1_subdev_entity_ops = {
	.link_validate = v4l2_subdev_link_validate,
};

static const struct v4l2_subdev_internal_ops s5kjn1_internal_ops = {
	.init_state = s5kjn1_init_state,
};

static const struct v4l2_ctrl_ops s5kjn1_ctrl_ops = {
	.s_ctrl = s5kjn1_set_ctrl,
};

static int s5kjn1_initialize_controls(struct s5kjn1 *s5kjn1)
{
	struct i2c_client *client = v4l2_get_subdevdata(&s5kjn1->subdev);
	const struct s5kjn1_mode *mode;
	struct v4l2_ctrl_handler *handler;
	struct v4l2_ctrl *ctrl;
	struct v4l2_fwnode_device_properties props;
	s32 exposure_max;
	s32 vblank_def, hblank_def;
	s32 pixel_rate;
	int ret;

	handler = &s5kjn1->ctrl_handler;
	mode = s5kjn1->cur_mode;
	ret = v4l2_ctrl_handler_init(handler, 7);
	if (ret)
		return ret;

	handler->lock = &s5kjn1->mutex;

	ctrl = v4l2_ctrl_new_int_menu(handler, NULL, V4L2_CID_LINK_FREQ, 0, 0,
				      link_freq_menu_items);
	if (ctrl)
		ctrl->flags |= V4L2_CTRL_FLAG_READ_ONLY;

	pixel_rate = to_pixel_rate(0);
	v4l2_ctrl_new_std(handler, NULL, V4L2_CID_PIXEL_RATE, 0, pixel_rate, 1,
			  pixel_rate);

	hblank_def = mode->line_length - mode->width;
	s5kjn1->hblank = v4l2_ctrl_new_std(handler, NULL, V4L2_CID_HBLANK,
					   hblank_def, hblank_def, 1, hblank_def);

	vblank_def = mode->frame_length - mode->height;
	s5kjn1->vblank = v4l2_ctrl_new_std(handler, &s5kjn1_ctrl_ops, V4L2_CID_VBLANK, vblank_def,
					   S5KJN1_FRAME_LENGTH_MAX - mode->height, 1, vblank_def);

	exposure_max = mode->frame_length - S5KJN1_EXPOSURE_MAX_MARGIN;
	s5kjn1->exposure = v4l2_ctrl_new_std(handler, &s5kjn1_ctrl_ops,
			  		     V4L2_CID_EXPOSURE,
			  		     S5KJN1_EXPOSURE_MIN,
			  		     exposure_max,
			  		     1,
			  		     S5KJN1_EXPOSURE_DEFAULT);

	s5kjn1->again = v4l2_ctrl_new_std(handler, &s5kjn1_ctrl_ops,
			  V4L2_CID_ANALOGUE_GAIN, S5KJN1_ANA_GAIN_MIN,
			  mode->max_again, 1,
			  mode->again);

	v4l2_ctrl_new_std_menu_items(handler, &s5kjn1_ctrl_ops,
				     V4L2_CID_TEST_PATTERN,
				     ARRAY_SIZE(s5kjn1_test_pattern_menu) - 1,
				     0, 0, s5kjn1_test_pattern_menu);

	if (handler->error) {
		ret = handler->error;
		dev_err(&client->dev, "failed to init controls(%d)\n", ret);
		goto err_free_handler;
	}

	ret = v4l2_fwnode_device_parse(&client->dev, &props);
	if (ret)
		goto err_free_handler;

	ret = v4l2_ctrl_new_fwnode_properties(handler, &s5kjn1_ctrl_ops, &props);
	if (ret)
		goto err_free_handler;

	s5kjn1->subdev.ctrl_handler = handler;

	return 0;

err_free_handler:
	v4l2_ctrl_handler_free(handler);

	return ret;
}

static int s5kjn1_check_sensor_id(struct s5kjn1 *s5kjn1)
{
	u64 chip_id;
	int ret;

	/* Validate the chip ID */
	ret = cci_read(s5kjn1->regmap, S5KJN1_REG_CHIP_ID, &chip_id, NULL);
	if (ret < 0) {
		dev_err(s5kjn1->dev, "failed to read sensor information\n");
		return ret;
	}

	if (chip_id != S5KJN1_ID) {
		dev_err(s5kjn1->dev, "unexpected sensor id(0x%04llx)\n", chip_id);
		return -EINVAL;
	}

	return 0;
}

static int s5kjn1_power_on(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct s5kjn1 *s5kjn1 = to_s5kjn1(sd);
	int ret;

	gpiod_set_value_cansleep(s5kjn1->reset_gpio, 1);

	ret = regulator_bulk_enable(ARRAY_SIZE(s5kjn1_supply_names),
				    s5kjn1->supplies);
	if (ret < 0) {
		dev_err(dev, "failed to enable regulators\n");
		goto disable_clk;
	}
	usleep_range(1000, 2000);

	ret = clk_prepare_enable(s5kjn1->mclk);
	if (ret < 0) {
		dev_err(dev, "failed to enable mclk\n");
		return ret;
	}
	usleep_range(1000, 2000);

	gpiod_set_value_cansleep(s5kjn1->reset_gpio, 0);
	usleep_range(12000, 13000);

	ret = s5kjn1_check_sensor_id(s5kjn1);
	if (ret)
		goto disable_regulator;

	return 0;

disable_regulator:
	regulator_bulk_disable(ARRAY_SIZE(s5kjn1_supply_names),
			       s5kjn1->supplies);
disable_clk:
	clk_disable_unprepare(s5kjn1->mclk);

	return ret;
}

static int s5kjn1_power_off(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct s5kjn1 *s5kjn1 = to_s5kjn1(sd);

	gpiod_set_value_cansleep(s5kjn1->reset_gpio, 1);
	clk_disable_unprepare(s5kjn1->mclk);
	regulator_bulk_disable(ARRAY_SIZE(s5kjn1_supply_names),
			       s5kjn1->supplies);

	return 0;
}

static const struct dev_pm_ops s5kjn1_pm_ops = {
	SET_RUNTIME_PM_OPS(s5kjn1_power_off, s5kjn1_power_on, NULL)
};

static int s5kjn1_probe(struct i2c_client *client)
{
	struct s5kjn1 *s5kjn1;
	unsigned int i;
	int ret;

	s5kjn1 = devm_kzalloc(&client->dev, sizeof(*s5kjn1), GFP_KERNEL);
	if (!s5kjn1)
		return -ENOMEM;

	s5kjn1->dev = &client->dev;

	ret = s5kjn1_check_hwcfg(s5kjn1->dev, s5kjn1);
	if (ret)
		return dev_err_probe(s5kjn1->dev, ret, "failed to check HW configuration\n");

	s5kjn1->regmap = devm_cci_regmap_init_i2c(client, 16);
	if (IS_ERR(s5kjn1->regmap))
		return dev_err_probe(s5kjn1->dev, PTR_ERR(s5kjn1->regmap), "failed to init regmap\n");

	v4l2_i2c_subdev_init(&s5kjn1->subdev, client, &s5kjn1_subdev_ops);
	s5kjn1->subdev.internal_ops = &s5kjn1_internal_ops;

	s5kjn1->mclk = devm_clk_get(s5kjn1->dev, NULL);
	if (IS_ERR(s5kjn1->mclk))
		return dev_err_probe(s5kjn1->dev, PTR_ERR(s5kjn1->mclk), "failed to get mclk\n");

	ret = clk_set_rate(s5kjn1->mclk, 19200000);
	if (ret < 0)
		return dev_err_probe(s5kjn1->dev, ret, "failed to set mclk frequency\n");

	s5kjn1->reset_gpio = devm_gpiod_get(s5kjn1->dev, "reset", GPIOD_OUT_LOW);
	if (IS_ERR(s5kjn1->reset_gpio))
		return dev_err_probe(s5kjn1->dev, PTR_ERR(s5kjn1->reset_gpio),
				     "failed to get reset-gpios\n");

	for (i = 0; i < ARRAY_SIZE(s5kjn1_supply_names); i++)
		s5kjn1->supplies[i].supply = s5kjn1_supply_names[i];

	ret = devm_regulator_bulk_get(s5kjn1->dev, ARRAY_SIZE(s5kjn1_supply_names),
				      s5kjn1->supplies);
	if (ret)
		return dev_err_probe(s5kjn1->dev, ret, "failed to get regulators\n");

	mutex_init(&s5kjn1->mutex);

	/* Set default mode */
	s5kjn1->cur_mode = &supported_modes[0];

	ret = s5kjn1_initialize_controls(s5kjn1);
	if (ret) {
		dev_err_probe(s5kjn1->dev, ret, "failed to initialize controls\n");
		goto err_destroy_mutex;
	}

	/* Initialize subdev */
	s5kjn1->subdev.flags |= V4L2_SUBDEV_FL_HAS_DEVNODE;
	s5kjn1->subdev.entity.ops = &s5kjn1_subdev_entity_ops;
	s5kjn1->subdev.entity.function = MEDIA_ENT_F_CAM_SENSOR;
	s5kjn1->pad.flags = MEDIA_PAD_FL_SOURCE;

	ret = media_entity_pads_init(&s5kjn1->subdev.entity, 1, &s5kjn1->pad);
	if (ret < 0) {
		dev_err_probe(s5kjn1->dev, ret, "failed to initialize entity pads\n");
		goto err_free_handler;
	}

	pm_runtime_enable(s5kjn1->dev);
	if (!pm_runtime_enabled(s5kjn1->dev)) {
		ret = s5kjn1_power_on(s5kjn1->dev);
		if (ret < 0) {
			dev_err_probe(s5kjn1->dev, ret, "failed to power on\n");
			goto err_clean_entity;
		}
	}

	ret = v4l2_async_register_subdev_sensor(&s5kjn1->subdev);
	if (ret) {
		dev_err_probe(s5kjn1->dev, ret, "failed to register V4L2 subdev\n");
		goto err_power_off;
	}

	return 0;

err_power_off:
	if (pm_runtime_enabled(s5kjn1->dev))
		pm_runtime_disable(s5kjn1->dev);
	else
		s5kjn1_power_off(s5kjn1->dev);
err_clean_entity:
	media_entity_cleanup(&s5kjn1->subdev.entity);
err_free_handler:
	v4l2_ctrl_handler_free(s5kjn1->subdev.ctrl_handler);
err_destroy_mutex:
	mutex_destroy(&s5kjn1->mutex);

	return ret;
}

static void s5kjn1_remove(struct i2c_client *client)
{
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct s5kjn1 *s5kjn1 = to_s5kjn1(sd);

	v4l2_async_unregister_subdev(sd);
	media_entity_cleanup(&sd->entity);
	v4l2_ctrl_handler_free(sd->ctrl_handler);

	pm_runtime_disable(&client->dev);
	if (!pm_runtime_status_suspended(&client->dev))
		s5kjn1_power_off(&client->dev);

	pm_runtime_set_suspended(&client->dev);

	mutex_destroy(&s5kjn1->mutex);
}

static const struct of_device_id s5kjn1_of_match[] = {
	{ .compatible = "samsung,s5kjn1" },
	{}
};
MODULE_DEVICE_TABLE(of, s5kjn1_of_match);

static struct i2c_driver s5kjn1_i2c_driver = {
	.driver = {
		.name = "s5kjn1",
		.pm = &s5kjn1_pm_ops,
		.of_match_table = s5kjn1_of_match,
	},
	.probe = s5kjn1_probe,
	.remove = s5kjn1_remove,
};
module_i2c_driver(s5kjn1_i2c_driver);

MODULE_AUTHOR("map220v <map220v300@gmail.com>");
MODULE_DESCRIPTION("Samsung S5KJN1 camera sensor driver");
MODULE_LICENSE("GPL v2");
