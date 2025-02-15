// SPDX-License-Identifier: GPL-2.0-only
/*
 * Omnivision OV32D40 Camera Sensor driver
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

#define OV32D40_ID			0x5632

#define OV32D40_REG_CHIP_ID		CCI_REG16(0x300A)

#define OV32D40_REG_EXPO		CCI_REG24(0x3500)
#define OV32D40_EXPOSURE_MIN		4
#define OV32D40_EXPOSURE_MAX_MARGIN		12
#define OV32D40_EXPOSURE_DEFAULT	1024

#define OV32D40_REG_GAIN		CCI_REG16(0x3508)
/* Min 1.125, Max 62.0, Mul 256 */
#define OV32D40_ANA_GAIN_MIN		288
#define OV32D40_ANA_GAIN_MAX		15872
#define OV32D40_ANA_GAIN_DEFAULT	7936

#define OV32D40_REG_FRAME_LENGTH		CCI_REG16(0x380E)
//#define OV32D40_REG_FRAME_LENGTH_HIGH	CCI_REG8(0x3840)
#define OV32D40_FRAME_LENGTH_MAX		0xffff

/* Most likely not actual test pattern reg */
#define OV32D40_REG_TEST_PATTERN	CCI_REG8(0x4308)

#define OV32D40_NATIVE_WIDTH		6592U
#define OV32D40_NATIVE_HEIGHT		4960U
#define OV32D40_PIXEL_ARRAY_LEFT	32U
#define OV32D40_PIXEL_ARRAY_TOP		32U
#define OV32D40_PIXEL_ARRAY_WIDTH	6528U
#define OV32D40_PIXEL_ARRAY_HEIGHT	4896U

#define OV32D40_DATA_LANES		4

#define OV32D40_BITS_PER_SAMPLE		10

static const char * const ov32d40_supply_names[] = {
	"vddio",	/* I/O power supply (1.8V) */
	"dvdd",		/* Digital power supply (1.2V) */
	"avdd",		/* Analog power supply (2.8V) */
};

struct ov32d40_reg_list {
	u32 num_of_regs;
	const struct cci_reg_sequence *regs;
};

struct ov32d40_mode {
	u32 width;
	u32 height;
	struct v4l2_rect crop;
	u32 line_length;
	u32 frame_length;
	const struct ov32d40_reg_list reg_list;
};

struct ov32d40 {
	u32 mclk_freq;

	struct device *dev;
	struct clk *mclk;
	struct gpio_desc *reset_gpio;
	struct regulator_bulk_data supplies[ARRAY_SIZE(ov32d40_supply_names)];
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

	const struct ov32d40_mode *cur_mode;
};

static inline struct ov32d40 *to_ov32d40(struct v4l2_subdev *sd)
{
	return container_of(sd, struct ov32d40, subdev);
}

static const struct cci_reg_sequence ov32d40_init_regs[] = {
	{ CCI_REG8(0x0103), 0x01 },
	{ CCI_REG8(0x0102), 0x01 },
	{ CCI_REG8(0x032e), 0x02 },
	{ CCI_REG8(0x0360), 0x01 },
	{ CCI_REG8(0x1003), 0x01 },
	{ CCI_REG8(0x1216), 0x00 },
	{ CCI_REG8(0x1217), 0x00 },
	{ CCI_REG8(0x1218), 0x00 },
	{ CCI_REG8(0x3016), 0x90 },
	{ CCI_REG8(0x3025), 0x89 },
	{ CCI_REG8(0x3026), 0x00 },
	{ CCI_REG8(0x3027), 0x01 },
	{ CCI_REG8(0x3029), 0x00 },
	{ CCI_REG8(0x3400), 0x1c },
	{ CCI_REG8(0x3401), 0x80 },
	{ CCI_REG8(0x3440), 0x56 },
	{ CCI_REG8(0x3441), 0x6a },
	{ CCI_REG8(0x3501), 0x01 },
	{ CCI_REG8(0x3502), 0x02 },
	{ CCI_REG8(0x3504), 0x4c },
	{ CCI_REG8(0x3542), 0x82 },
	{ CCI_REG8(0x3544), 0x4c },
	{ CCI_REG8(0x3600), 0x64 },
	{ CCI_REG8(0x3601), 0x24 },
	{ CCI_REG8(0x3602), 0x09 },
	{ CCI_REG8(0x3603), 0x24 },
	{ CCI_REG8(0x3608), 0x0c },
	{ CCI_REG8(0x3610), 0x78 },
	{ CCI_REG8(0x3626), 0x18 },
	{ CCI_REG8(0x3682), 0x80 },
	{ CCI_REG8(0x3684), 0x04 },
	{ CCI_REG8(0x3696), 0xd1 },
	{ CCI_REG8(0x3718), 0x10 },
	{ CCI_REG8(0x3729), 0x7c },
	{ CCI_REG8(0x37c0), 0x17 },
	{ CCI_REG8(0x3820), 0x06 },
	{ CCI_REG8(0x3821), 0x12 },
	{ CCI_REG8(0x3828), 0x00 },
	{ CCI_REG8(0x382a), 0x80 },
	{ CCI_REG8(0x383a), 0x01 },
	{ CCI_REG8(0x3901), 0xc0 },
	{ CCI_REG8(0x3903), 0x1f },
	{ CCI_REG8(0x3904), 0x1f },
	{ CCI_REG8(0x3905), 0x1f },
	{ CCI_REG8(0x3906), 0x1f },
	{ CCI_REG8(0x3907), 0x1f },
	{ CCI_REG8(0x3908), 0x1f },
	{ CCI_REG8(0x3909), 0x1f },
	{ CCI_REG8(0x390a), 0x14 },
	{ CCI_REG8(0x390b), 0x14 },
	{ CCI_REG8(0x390c), 0x14 },
	{ CCI_REG8(0x390d), 0x14 },
	{ CCI_REG8(0x390e), 0x14 },
	{ CCI_REG8(0x390f), 0x14 },
	{ CCI_REG8(0x3910), 0x14 },
	{ CCI_REG8(0x3911), 0x05 },
	{ CCI_REG8(0x3912), 0x05 },
	{ CCI_REG8(0x3913), 0x05 },
	{ CCI_REG8(0x3914), 0x05 },
	{ CCI_REG8(0x3915), 0x05 },
	{ CCI_REG8(0x3916), 0x05 },
	{ CCI_REG8(0x3917), 0x05 },
	{ CCI_REG8(0x3918), 0x44 },
	{ CCI_REG8(0x3919), 0x34 },
	{ CCI_REG8(0x391a), 0x33 },
	{ CCI_REG8(0x391d), 0x0f },
	{ CCI_REG8(0x3927), 0xc8 },
	{ CCI_REG8(0x3928), 0x01 },
	{ CCI_REG8(0x3929), 0x68 },
	{ CCI_REG8(0x3934), 0x2f },
	{ CCI_REG8(0x3935), 0x31 },
	{ CCI_REG8(0x3936), 0x2d },
	{ CCI_REG8(0x3937), 0x2f },
	{ CCI_REG8(0x3938), 0x2d },
	{ CCI_REG8(0x3939), 0x2d },
	{ CCI_REG8(0x393a), 0x2c },
	{ CCI_REG8(0x393b), 0x2d },
	{ CCI_REG8(0x393c), 0x2d },
	{ CCI_REG8(0x393d), 0x2d },
	{ CCI_REG8(0x393e), 0x31 },
	{ CCI_REG8(0x393f), 0x31 },
	{ CCI_REG8(0x3940), 0x33 },
	{ CCI_REG8(0x3941), 0x33 },
	{ CCI_REG8(0x3947), 0x1d },
	{ CCI_REG8(0x3948), 0x1d },
	{ CCI_REG8(0x3949), 0x1e },
	{ CCI_REG8(0x394a), 0x1c },
	{ CCI_REG8(0x394b), 0x1f },
	{ CCI_REG8(0x394c), 0x1e },
	{ CCI_REG8(0x394d), 0x1f },
	{ CCI_REG8(0x394e), 0x13 },
	{ CCI_REG8(0x394f), 0x26 },
	{ CCI_REG8(0x3950), 0x48 },
	{ CCI_REG8(0x3951), 0x8b },
	{ CCI_REG8(0x3952), 0x8c },
	{ CCI_REG8(0x3953), 0x8c },
	{ CCI_REG8(0x3954), 0x8a },
	{ CCI_REG8(0x3967), 0x17 },
	{ CCI_REG8(0x3968), 0x17 },
	{ CCI_REG8(0x3969), 0x0d },
	{ CCI_REG8(0x396a), 0x0d },
	{ CCI_REG8(0x396b), 0x0c },
	{ CCI_REG8(0x396c), 0x0c },
	{ CCI_REG8(0x396d), 0x13 },
	{ CCI_REG8(0x396e), 0x1c },
	{ CCI_REG8(0x396f), 0x0e },
	{ CCI_REG8(0x3970), 0x10 },
	{ CCI_REG8(0x3971), 0x0a },
	{ CCI_REG8(0x3972), 0x0b },
	{ CCI_REG8(0x3973), 0x07 },
	{ CCI_REG8(0x3974), 0x07 },
	{ CCI_REG8(0x3a1a), 0x0e },
	{ CCI_REG8(0x3a1f), 0x8c },
	{ CCI_REG8(0x3a28), 0xf0 },
	{ CCI_REG8(0x3a48), 0x0a },
	{ CCI_REG8(0x3a49), 0x8d },
	{ CCI_REG8(0x3a4a), 0xbb },
	{ CCI_REG8(0x3a4b), 0xf6 },
	{ CCI_REG8(0x3a61), 0x29 },
	{ CCI_REG8(0x3a6a), 0x8d },
	{ CCI_REG8(0x3a6c), 0xf1 },
	{ CCI_REG8(0x3a6d), 0xbb },
	{ CCI_REG8(0x3a6e), 0x8d },
	{ CCI_REG8(0x3a70), 0xf1 },
	{ CCI_REG8(0x3a71), 0xbb },
	{ CCI_REG8(0x3a86), 0xbb },
	{ CCI_REG8(0x3a8a), 0xbb },
	{ CCI_REG8(0x3a8e), 0x00 },
	{ CCI_REG8(0x3a8f), 0x00 },
	{ CCI_REG8(0x3aaf), 0x2f },
	{ CCI_REG8(0x3ab3), 0x37 },
	{ CCI_REG8(0x3ab7), 0x48 },
	{ CCI_REG8(0x3abb), 0x48 },
	{ CCI_REG8(0x3abf), 0x48 },
	{ CCI_REG8(0x3ac3), 0x48 },
	{ CCI_REG8(0x3ac7), 0x48 },
	{ CCI_REG8(0x3acb), 0x2f },
	{ CCI_REG8(0x3acf), 0x37 },
	{ CCI_REG8(0x3ae7), 0x2d },
	{ CCI_REG8(0x3aec), 0xc3 },
	{ CCI_REG8(0x3aed), 0xce },
	{ CCI_REG8(0x3b1a), 0x8b },
	{ CCI_REG8(0x3b1c), 0xe5 },
	{ CCI_REG8(0x3b22), 0x8b },
	{ CCI_REG8(0x3b24), 0xe4 },
	{ CCI_REG8(0x3b2a), 0x8b },
	{ CCI_REG8(0x3b2c), 0xe5 },
	{ CCI_REG8(0x3b32), 0x88 },
	{ CCI_REG8(0x3b34), 0xe0 },
	{ CCI_REG8(0x3b3a), 0x7f },
	{ CCI_REG8(0x3b3c), 0xd5 },
	{ CCI_REG8(0x3b42), 0x7f },
	{ CCI_REG8(0x3b44), 0xd5 },
	{ CCI_REG8(0x3b83), 0x8d },
	{ CCI_REG8(0x3b84), 0xbb },
	{ CCI_REG8(0x3bc2), 0x8b },
	{ CCI_REG8(0x3bc4), 0xe5 },
	{ CCI_REG8(0x3bca), 0x8b },
	{ CCI_REG8(0x3bcc), 0xe4 },
	{ CCI_REG8(0x3bd2), 0x8b },
	{ CCI_REG8(0x3bd4), 0xe5 },
	{ CCI_REG8(0x3bda), 0x88 },
	{ CCI_REG8(0x3bdc), 0xe0 },
	{ CCI_REG8(0x3be2), 0x7f },
	{ CCI_REG8(0x3be4), 0xd5 },
	{ CCI_REG8(0x3bea), 0x7f },
	{ CCI_REG8(0x3bec), 0xd5 },
	{ CCI_REG8(0x4010), 0xf5 },
	{ CCI_REG8(0x4015), 0x00 },
	{ CCI_REG8(0x4018), 0x0d },
	{ CCI_REG8(0x403f), 0x80 },
	{ CCI_REG8(0x4042), 0x00 },
	{ CCI_REG8(0x4045), 0x80 },
	{ CCI_REG8(0x4046), 0x00 },
	{ CCI_REG8(0x4047), 0x80 },
	{ CCI_REG8(0x40ea), 0x00 },
	{ CCI_REG8(0x40ed), 0x80 },
	{ CCI_REG8(0x40ee), 0x00 },
	{ CCI_REG8(0x4549), 0x00 },
	{ CCI_REG8(0x4680), 0xb1 },
	{ CCI_REG8(0x4684), 0x2b },
	{ CCI_REG8(0x4813), 0x6c },
	{ CCI_REG8(0x4883), 0x01 },
	{ CCI_REG8(0x4a3f), 0x80 },
	{ CCI_REG8(0x4a42), 0x00 },
	{ CCI_REG8(0x4a45), 0x80 },
	{ CCI_REG8(0x4a46), 0x00 },
	{ CCI_REG8(0x4a47), 0x80 },
	{ CCI_REG8(0x4aea), 0x00 },
	{ CCI_REG8(0x4aed), 0x80 },
	{ CCI_REG8(0x4aee), 0x00 },
	{ CCI_REG8(0x4c00), 0x42 },
	{ CCI_REG8(0x4c11), 0x02 },
	{ CCI_REG8(0x4c12), 0x06 },
	{ CCI_REG8(0x4c15), 0x02 },
	{ CCI_REG8(0x4c16), 0x06 },
	{ CCI_REG8(0x4c17), 0x06 },
	{ CCI_REG8(0x4c18), 0x01 },
	{ CCI_REG8(0x4c1b), 0x08 },
	{ CCI_REG8(0x4c31), 0x02 },
	{ CCI_REG8(0x4c32), 0x06 },
	{ CCI_REG8(0x4c35), 0x02 },
	{ CCI_REG8(0x4c36), 0x06 },
	{ CCI_REG8(0x4c37), 0x06 },
	{ CCI_REG8(0x4c38), 0x01 },
	{ CCI_REG8(0x4c3b), 0x08 },
	{ CCI_REG8(0x4c51), 0x02 },
	{ CCI_REG8(0x4c52), 0x06 },
	{ CCI_REG8(0x4c55), 0x02 },
	{ CCI_REG8(0x4c56), 0x06 },
	{ CCI_REG8(0x4c57), 0x06 },
	{ CCI_REG8(0x4c58), 0x01 },
	{ CCI_REG8(0x4c5b), 0x08 },
	{ CCI_REG8(0x4c61), 0x10 },
	{ CCI_REG8(0x4d00), 0x03 },
	{ CCI_REG8(0x4d01), 0xa9 },
	{ CCI_REG8(0x4d02), 0xbd },
	{ CCI_REG8(0x4d03), 0x31 },
	{ CCI_REG8(0x4d04), 0x5f },
	{ CCI_REG8(0x4d05), 0xdc },
	{ CCI_REG8(0x5000), 0x35 },
	{ CCI_REG8(0x5002), 0x0f },
	{ CCI_REG8(0x5055), 0x10 },
	{ CCI_REG8(0x5056), 0x0c },
	{ CCI_REG8(0x5059), 0x10 },
	{ CCI_REG8(0x505a), 0x09 },
	{ CCI_REG8(0x505b), 0x90 },
	{ CCI_REG8(0x505c), 0x08 },
	{ CCI_REG8(0x505d), 0x08 },
	{ CCI_REG8(0x505e), 0x02 },
	{ CCI_REG8(0x505f), 0x02 },
	{ CCI_REG8(0x5060), 0x06 },
	{ CCI_REG8(0x5061), 0x06 },
	{ CCI_REG8(0x5062), 0x02 },
	{ CCI_REG8(0x5063), 0x02 },
	{ CCI_REG8(0x5064), 0x06 },
	{ CCI_REG8(0x5065), 0x06 },
	{ CCI_REG8(0x5066), 0x10 },
	{ CCI_REG8(0x5068), 0x0a },
	{ CCI_REG8(0x5151), 0x14 },
	{ CCI_REG8(0x5152), 0x01 },
	{ CCI_REG8(0x5153), 0x53 },
	{ CCI_REG8(0x5194), 0x0c },
	{ CCI_REG8(0x5195), 0x0c },
	{ CCI_REG8(0x5196), 0x0c },
	{ CCI_REG8(0x5197), 0x0c },
	{ CCI_REG8(0x5198), 0x0c },
	{ CCI_REG8(0x5199), 0x0c },
	{ CCI_REG8(0x519a), 0x0c },
	{ CCI_REG8(0x519b), 0x0c },
	{ CCI_REG8(0x5201), 0x04 },
	{ CCI_REG8(0x5203), 0x04 },
	{ CCI_REG8(0x5244), 0x00 },
	{ CCI_REG8(0x5245), 0x50 },
	{ CCI_REG8(0x5246), 0x00 },
	{ CCI_REG8(0x5247), 0x50 },
	{ CCI_REG8(0x5248), 0x00 },
	{ CCI_REG8(0x5249), 0x50 },
	{ CCI_REG8(0x524a), 0x00 },
	{ CCI_REG8(0x524b), 0x50 },
	{ CCI_REG8(0x524c), 0x00 },
	{ CCI_REG8(0x524d), 0x50 },
	{ CCI_REG8(0x524e), 0x00 },
	{ CCI_REG8(0x524f), 0x50 },
	{ CCI_REG8(0x5250), 0x00 },
	{ CCI_REG8(0x5251), 0x50 },
	{ CCI_REG8(0x5252), 0x00 },
	{ CCI_REG8(0x5253), 0x50 },
	{ CCI_REG8(0x52a0), 0x02 },
	{ CCI_REG8(0x52a1), 0x08 },
	{ CCI_REG8(0x52a2), 0x07 },
	{ CCI_REG8(0x52a3), 0x09 },
	{ CCI_REG8(0x52a4), 0x0a },
	{ CCI_REG8(0x52a5), 0x01 },
	{ CCI_REG8(0x52a6), 0x04 },
	{ CCI_REG8(0x52a7), 0x03 },
	{ CCI_REG8(0x52a8), 0x05 },
	{ CCI_REG8(0x52a9), 0x06 },
	{ CCI_REG8(0x52c1), 0xe8 },
	{ CCI_REG8(0x5320), 0xf8 },
	{ CCI_REG8(0x5322), 0xff },
	{ CCI_REG8(0x5323), 0x0c },
	{ CCI_REG8(0x5651), 0x14 },
	{ CCI_REG8(0x5652), 0x01 },
	{ CCI_REG8(0x5653), 0x53 },
	{ CCI_REG8(0x5694), 0x0c },
	{ CCI_REG8(0x5695), 0x0c },
	{ CCI_REG8(0x5696), 0x0c },
	{ CCI_REG8(0x5697), 0x0c },
	{ CCI_REG8(0x5698), 0x0c },
	{ CCI_REG8(0x5699), 0x0c },
	{ CCI_REG8(0x569a), 0x0c },
	{ CCI_REG8(0x569b), 0x0c },
	{ CCI_REG8(0x5744), 0x00 },
	{ CCI_REG8(0x5745), 0x50 },
	{ CCI_REG8(0x5746), 0x00 },
	{ CCI_REG8(0x5747), 0x50 },
	{ CCI_REG8(0x5748), 0x00 },
	{ CCI_REG8(0x5749), 0x50 },
	{ CCI_REG8(0x574a), 0x00 },
	{ CCI_REG8(0x574b), 0x50 },
	{ CCI_REG8(0x574c), 0x00 },
	{ CCI_REG8(0x574d), 0x50 },
	{ CCI_REG8(0x574e), 0x00 },
	{ CCI_REG8(0x574f), 0x50 },
	{ CCI_REG8(0x5750), 0x00 },
	{ CCI_REG8(0x5751), 0x50 },
	{ CCI_REG8(0x5752), 0x00 },
	{ CCI_REG8(0x5753), 0x50 },
	{ CCI_REG8(0x57a0), 0x02 },
	{ CCI_REG8(0x57a1), 0x08 },
	{ CCI_REG8(0x57a2), 0x07 },
	{ CCI_REG8(0x57a3), 0x09 },
	{ CCI_REG8(0x57a4), 0x0a },
	{ CCI_REG8(0x57a5), 0x01 },
	{ CCI_REG8(0x57a6), 0x04 },
	{ CCI_REG8(0x57a7), 0x03 },
	{ CCI_REG8(0x57a8), 0x05 },
	{ CCI_REG8(0x57a9), 0x06 },
	{ CCI_REG8(0x57c1), 0xe8 },
	{ CCI_REG8(0x5820), 0xf8 },
	{ CCI_REG8(0x5822), 0xff },
	{ CCI_REG8(0x5823), 0x0c },
	{ CCI_REG8(0x6003), 0x00 },
	{ CCI_REG8(0x5a20), 0x40 },
	{ CCI_REG8(0x5a21), 0x40 },
	{ CCI_REG8(0x5a22), 0x40 },
	{ CCI_REG8(0x5a23), 0x40 },
	{ CCI_REG8(0x5a24), 0x40 },
	{ CCI_REG8(0x5a25), 0x40 },
	{ CCI_REG8(0x5a26), 0x40 },
	{ CCI_REG8(0x5a27), 0x40 },
	{ CCI_REG8(0x5a28), 0x40 },
	{ CCI_REG8(0x5a29), 0x40 },
	{ CCI_REG8(0x5a2a), 0x40 },
	{ CCI_REG8(0x5a2b), 0x40 },
	{ CCI_REG8(0x5a2c), 0x40 },
	{ CCI_REG8(0x5a2d), 0x40 },
	{ CCI_REG8(0x5a2e), 0x40 },
	{ CCI_REG8(0x5a2f), 0x40 },
	{ CCI_REG8(0x5a30), 0x40 },
	{ CCI_REG8(0x5a31), 0x40 },
	{ CCI_REG8(0x5a32), 0x40 },
	{ CCI_REG8(0x5a33), 0x40 },
	{ CCI_REG8(0x5a34), 0x40 },
	{ CCI_REG8(0x5a35), 0x40 },
	{ CCI_REG8(0x5a36), 0x40 },
	{ CCI_REG8(0x5a37), 0x40 },
	{ CCI_REG8(0x5a38), 0x40 },
	{ CCI_REG8(0x5a39), 0x40 },
	{ CCI_REG8(0x5a3a), 0x40 },
	{ CCI_REG8(0x5a3b), 0x40 },
	{ CCI_REG8(0x5a3c), 0x40 },
	{ CCI_REG8(0x5a3d), 0x40 },
	{ CCI_REG8(0x5a3e), 0x40 },
	{ CCI_REG8(0x5a3f), 0x40 },
	{ CCI_REG8(0x5a40), 0x40 },
	{ CCI_REG8(0x5a41), 0x40 },
	{ CCI_REG8(0x5a42), 0x40 },
	{ CCI_REG8(0x5a43), 0x40 },
	{ CCI_REG8(0x5a44), 0x40 },
	{ CCI_REG8(0x5a45), 0x40 },
	{ CCI_REG8(0x5a46), 0x40 },
	{ CCI_REG8(0x5a47), 0x40 },
	{ CCI_REG8(0x5a48), 0x40 },
	{ CCI_REG8(0x5a49), 0x40 },
	{ CCI_REG8(0x5a4a), 0x40 },
	{ CCI_REG8(0x5a4b), 0x40 },
	{ CCI_REG8(0x5a4c), 0x40 },
	{ CCI_REG8(0x5a4d), 0x40 },
	{ CCI_REG8(0x5a4e), 0x40 },
	{ CCI_REG8(0x5a4f), 0x40 },
	{ CCI_REG8(0x5a50), 0x40 },
	{ CCI_REG8(0x5a51), 0x40 },
	{ CCI_REG8(0x5a52), 0x40 },
	{ CCI_REG8(0x5a53), 0x40 },
	{ CCI_REG8(0x5a54), 0x40 },
	{ CCI_REG8(0x5a55), 0x40 },
	{ CCI_REG8(0x5a56), 0x40 },
	{ CCI_REG8(0x5a57), 0x40 },
	{ CCI_REG8(0x5a58), 0x40 },
	{ CCI_REG8(0x5a59), 0x40 },
	{ CCI_REG8(0x5a5a), 0x40 },
	{ CCI_REG8(0x5a5b), 0x40 },
	{ CCI_REG8(0x5a5c), 0x40 },
	{ CCI_REG8(0x5a5d), 0x40 },
	{ CCI_REG8(0x5a5e), 0x40 },
	{ CCI_REG8(0x5a5f), 0x40 },
	{ CCI_REG8(0x5a60), 0x40 },
	{ CCI_REG8(0x5a61), 0x40 },
	{ CCI_REG8(0x5a62), 0x40 },
	{ CCI_REG8(0x5a63), 0x40 },
	{ CCI_REG8(0x5a64), 0x40 },
	{ CCI_REG8(0x5a65), 0x40 },
	{ CCI_REG8(0x5a66), 0x40 },
	{ CCI_REG8(0x5a67), 0x40 },
	{ CCI_REG8(0x5a68), 0x40 },
	{ CCI_REG8(0x5a69), 0x40 },
	{ CCI_REG8(0x5a6a), 0x40 },
	{ CCI_REG8(0x5a6b), 0x40 },
	{ CCI_REG8(0x5a6c), 0x40 },
	{ CCI_REG8(0x5a6d), 0x40 },
	{ CCI_REG8(0x5a6e), 0x40 },
	{ CCI_REG8(0x5a6f), 0x40 },
	{ CCI_REG8(0x5a70), 0x40 },
	{ CCI_REG8(0x5a71), 0x40 },
	{ CCI_REG8(0x5a72), 0x40 },
	{ CCI_REG8(0x5a73), 0x40 },
	{ CCI_REG8(0x5a74), 0x40 },
	{ CCI_REG8(0x5a75), 0x40 },
	{ CCI_REG8(0x5a76), 0x40 },
	{ CCI_REG8(0x5a77), 0x40 },
	{ CCI_REG8(0x5a78), 0x40 },
	{ CCI_REG8(0x5a79), 0x40 },
	{ CCI_REG8(0x5a7a), 0xcd },
	{ CCI_REG8(0x5a7b), 0xcd },
	{ CCI_REG8(0x5a7c), 0xcd },
	{ CCI_REG8(0x5a7d), 0xcd },
	{ CCI_REG8(0x5a7e), 0xcd },
	{ CCI_REG8(0x5a7f), 0xcd },
	{ CCI_REG8(0x5a80), 0xcd },
	{ CCI_REG8(0x5a81), 0xcd },
	{ CCI_REG8(0x5a82), 0xcd },
	{ CCI_REG8(0x5a83), 0xcd },
	{ CCI_REG8(0x5a84), 0xcd },
	{ CCI_REG8(0x5a85), 0xcd },
	{ CCI_REG8(0x5a86), 0xcd },
	{ CCI_REG8(0x5a87), 0xcd },
	{ CCI_REG8(0x5a88), 0xcd },
	{ CCI_REG8(0x5a89), 0xcd },
	{ CCI_REG8(0x5a8a), 0xcd },
	{ CCI_REG8(0x5a8b), 0xcd },
	{ CCI_REG8(0x5a8c), 0xcd },
	{ CCI_REG8(0x5a8d), 0xcd },
	{ CCI_REG8(0x5a8e), 0xcd },
	{ CCI_REG8(0x5a8f), 0xcd },
	{ CCI_REG8(0x5a90), 0xcd },
	{ CCI_REG8(0x5a91), 0xcd },
	{ CCI_REG8(0x5a92), 0xcd },
	{ CCI_REG8(0x5a93), 0xcd },
	{ CCI_REG8(0x5a94), 0xcd },
	{ CCI_REG8(0x5a95), 0xcd },
	{ CCI_REG8(0x5a96), 0xcd },
	{ CCI_REG8(0x5a97), 0xcd },
	{ CCI_REG8(0x5a98), 0xcd },
	{ CCI_REG8(0x5a99), 0xcd },
	{ CCI_REG8(0x5a9a), 0xcd },
	{ CCI_REG8(0x5a9b), 0xcd },
	{ CCI_REG8(0x5a9c), 0xcd },
	{ CCI_REG8(0x5a9d), 0xcd },
	{ CCI_REG8(0x5a9e), 0xcd },
	{ CCI_REG8(0x5a9f), 0xcd },
	{ CCI_REG8(0x5aa0), 0xcd },
	{ CCI_REG8(0x5aa1), 0xcd },
	{ CCI_REG8(0x5aa2), 0xcd },
	{ CCI_REG8(0x5aa3), 0xcd },
	{ CCI_REG8(0x5aa4), 0xcd },
	{ CCI_REG8(0x5aa5), 0xcd },
	{ CCI_REG8(0x5aa6), 0xcd },
	{ CCI_REG8(0x5aa7), 0xcd },
	{ CCI_REG8(0x5aa8), 0xcd },
	{ CCI_REG8(0x5aa9), 0xcd },
	{ CCI_REG8(0x5aaa), 0xcd },
	{ CCI_REG8(0x5aab), 0xcd },
	{ CCI_REG8(0x5aac), 0xcd },
	{ CCI_REG8(0x5aad), 0xcd },
	{ CCI_REG8(0x5aae), 0xcd },
	{ CCI_REG8(0x5aaf), 0xcd },
	{ CCI_REG8(0x5ab0), 0xcd },
	{ CCI_REG8(0x5ab1), 0xcd },
	{ CCI_REG8(0x5ab2), 0xcd },
	{ CCI_REG8(0x5ab3), 0xcd },
	{ CCI_REG8(0x5ab4), 0xcd },
	{ CCI_REG8(0x5ab5), 0xcd },
	{ CCI_REG8(0x5ab6), 0xcd },
	{ CCI_REG8(0x5ab7), 0xcd },
	{ CCI_REG8(0x5ab8), 0xcd },
	{ CCI_REG8(0x5ab9), 0xcd },
	{ CCI_REG8(0x5aba), 0xcd },
	{ CCI_REG8(0x5abb), 0xcd },
	{ CCI_REG8(0x5abc), 0xcd },
	{ CCI_REG8(0x5abd), 0xcd },
	{ CCI_REG8(0x5abe), 0xcd },
	{ CCI_REG8(0x5abf), 0xcd },
	{ CCI_REG8(0x5ac0), 0xcd },
	{ CCI_REG8(0x5ac1), 0xcd },
	{ CCI_REG8(0x5ac2), 0xcd },
	{ CCI_REG8(0x5ac3), 0xcd },
	{ CCI_REG8(0x5ac4), 0xcd },
	{ CCI_REG8(0x5ac5), 0xcd },
	{ CCI_REG8(0x5ac6), 0xcd },
	{ CCI_REG8(0x5ac7), 0xcd },
	{ CCI_REG8(0x5ac8), 0xcd },
	{ CCI_REG8(0x5ac9), 0xcd },
	{ CCI_REG8(0x5aca), 0xcd },
	{ CCI_REG8(0x5acb), 0xcd },
	{ CCI_REG8(0x5acc), 0xcd },
	{ CCI_REG8(0x5acd), 0xcd },
	{ CCI_REG8(0x5ace), 0xcd },
	{ CCI_REG8(0x5acf), 0xcd },
	{ CCI_REG8(0x5ad0), 0xcd },
	{ CCI_REG8(0x5ad1), 0xcd },
	{ CCI_REG8(0x5ad2), 0xcd },
	{ CCI_REG8(0x5ad3), 0xcd },
	{ CCI_REG8(0x5ad4), 0xcd },
	{ CCI_REG8(0x5ad5), 0xcd },
	{ CCI_REG8(0x5ad6), 0xcd },
	{ CCI_REG8(0x5ad7), 0xcd },
	{ CCI_REG8(0x5ad8), 0xcd },
	{ CCI_REG8(0x5ad9), 0xcd },
	{ CCI_REG8(0x5ada), 0xcd },
	{ CCI_REG8(0x5adb), 0xcd },
	{ CCI_REG8(0x5adc), 0xcd },
	{ CCI_REG8(0x5add), 0xcd },
	{ CCI_REG8(0x5ade), 0xcd },
	{ CCI_REG8(0x5adf), 0xcd },
	{ CCI_REG8(0x5ae0), 0xcd },
	{ CCI_REG8(0x5ae1), 0xcd },
	{ CCI_REG8(0x5ae2), 0xcd },
	{ CCI_REG8(0x5ae3), 0xcd },
	{ CCI_REG8(0x5ae4), 0xcd },
	{ CCI_REG8(0x5ae5), 0xcd },
	{ CCI_REG8(0x5ae6), 0xcd },
	{ CCI_REG8(0x5ae7), 0xcd },
	{ CCI_REG8(0x5ae8), 0xcd },
	{ CCI_REG8(0x5ae9), 0xcd },
	{ CCI_REG8(0x5aea), 0xcd },
	{ CCI_REG8(0x5aeb), 0xcd },
	{ CCI_REG8(0x5aec), 0xcd },
	{ CCI_REG8(0x5aed), 0xcd },
	{ CCI_REG8(0x5aee), 0xcd },
	{ CCI_REG8(0x5aef), 0xcd },
	{ CCI_REG8(0x5af0), 0xcd },
	{ CCI_REG8(0x5af1), 0xcd },
	{ CCI_REG8(0x5af2), 0xcd },
	{ CCI_REG8(0x5af3), 0xcd },
	{ CCI_REG8(0x5af4), 0xcd },
	{ CCI_REG8(0x5af5), 0xcd },
	{ CCI_REG8(0x5af6), 0xcd },
	{ CCI_REG8(0x5af7), 0xcd },
	{ CCI_REG8(0x5af8), 0xcd },
	{ CCI_REG8(0x5af9), 0xcd },
	{ CCI_REG8(0x5afa), 0xcd },
	{ CCI_REG8(0x5afb), 0xcd },
	{ CCI_REG8(0x5afc), 0xcd },
	{ CCI_REG8(0x5afd), 0xcd },
	{ CCI_REG8(0x5afe), 0xcd },
	{ CCI_REG8(0x5aff), 0xcd },
	{ CCI_REG8(0x5b00), 0xcd },
	{ CCI_REG8(0x5b01), 0xcd },
	{ CCI_REG8(0x5b02), 0xcd },
	{ CCI_REG8(0x5b03), 0xcd },
	{ CCI_REG8(0x5b04), 0xcd },
	{ CCI_REG8(0x5b05), 0xcd },
	{ CCI_REG8(0x5b06), 0xcd },
	{ CCI_REG8(0x5b07), 0xcd },
	{ CCI_REG8(0x5b08), 0xcd },
	{ CCI_REG8(0x5b09), 0xcd },
	{ CCI_REG8(0x5b0a), 0xcd },
	{ CCI_REG8(0x5b0b), 0xcd },
	{ CCI_REG8(0x5b0c), 0xcd },
	{ CCI_REG8(0x5b0d), 0xcd },
	{ CCI_REG8(0x5b0e), 0xcd },
	{ CCI_REG8(0x5b0f), 0xcd },
	{ CCI_REG8(0x5b10), 0xcd },
	{ CCI_REG8(0x5b11), 0xcd },
	{ CCI_REG8(0x5b12), 0xcd },
	{ CCI_REG8(0x5b13), 0xcd },
	{ CCI_REG8(0x5b14), 0xcd },
	{ CCI_REG8(0x5b15), 0xcd },
	{ CCI_REG8(0x5b16), 0xcd },
	{ CCI_REG8(0x5b17), 0xcd },
	{ CCI_REG8(0x5b18), 0xcd },
	{ CCI_REG8(0x5b19), 0xcd },
	{ CCI_REG8(0x5b1a), 0xcd },
	{ CCI_REG8(0x5b1b), 0xcd },
	{ CCI_REG8(0x5b1c), 0xcd },
	{ CCI_REG8(0x5b1d), 0xcd },
	{ CCI_REG8(0x5b1e), 0xcd },
	{ CCI_REG8(0x5b1f), 0xcd },
	{ CCI_REG8(0x5b20), 0xcd },
	{ CCI_REG8(0x5b21), 0xcd },
	{ CCI_REG8(0x5b22), 0xcd },
	{ CCI_REG8(0x5b23), 0xcd },
	{ CCI_REG8(0x5b24), 0xcd },
	{ CCI_REG8(0x5b25), 0xcd },
	{ CCI_REG8(0x5b26), 0xcd },
	{ CCI_REG8(0x5b27), 0xcd },
	{ CCI_REG8(0x5b28), 0xcd },
	{ CCI_REG8(0x5b29), 0xcd },
	{ CCI_REG8(0x5b2a), 0xcd },
	{ CCI_REG8(0x5b2b), 0xcd },
	{ CCI_REG8(0x5b2c), 0xcd },
	{ CCI_REG8(0x5b2d), 0xcd },
	{ CCI_REG8(0x5b2e), 0xcd },
	{ CCI_REG8(0x5b2f), 0xcd },
	{ CCI_REG8(0x5b30), 0xcd },
	{ CCI_REG8(0x5b31), 0xcd },
	{ CCI_REG8(0x5b32), 0xcd },
	{ CCI_REG8(0x5b33), 0xcd },
	{ CCI_REG8(0x5b34), 0xcd },
	{ CCI_REG8(0x5b35), 0xcd },
	{ CCI_REG8(0x5b36), 0xcd },
	{ CCI_REG8(0x5b37), 0xcd },
	{ CCI_REG8(0x5b38), 0xcd },
	{ CCI_REG8(0x5b39), 0xcd },
	{ CCI_REG8(0x5b3a), 0xcd },
	{ CCI_REG8(0x5b3b), 0xcd },
	{ CCI_REG8(0x5b3c), 0xcd },
	{ CCI_REG8(0x5b3d), 0xcd },
	{ CCI_REG8(0x5b3e), 0xcd },
	{ CCI_REG8(0x5b3f), 0xcd },
	{ CCI_REG8(0x5b40), 0xcd },
	{ CCI_REG8(0x5b41), 0xcd },
	{ CCI_REG8(0x5b42), 0xcd },
	{ CCI_REG8(0x5b43), 0xcd },
	{ CCI_REG8(0x5b44), 0xcd },
	{ CCI_REG8(0x5b45), 0xcd },
	{ CCI_REG8(0x5b46), 0xcd },
	{ CCI_REG8(0x5b47), 0xcd },
	{ CCI_REG8(0x5b48), 0xcd },
	{ CCI_REG8(0x5b49), 0xcd },
	{ CCI_REG8(0x5b4a), 0xcd },
	{ CCI_REG8(0x5b4b), 0xcd },
	{ CCI_REG8(0x5b4c), 0xcd },
	{ CCI_REG8(0x5b4d), 0xcd },
	{ CCI_REG8(0x5b4e), 0xcd },
	{ CCI_REG8(0x5b4f), 0xcd },
	{ CCI_REG8(0x5b50), 0xcd },
	{ CCI_REG8(0x5b51), 0xcd },
	{ CCI_REG8(0x5b52), 0xcd },
	{ CCI_REG8(0x5b53), 0xcd },
	{ CCI_REG8(0x5b54), 0xcd },
	{ CCI_REG8(0x5b55), 0xcd },
	{ CCI_REG8(0x5b56), 0xcd },
	{ CCI_REG8(0x5b57), 0xcd },
	{ CCI_REG8(0x5b58), 0xcd },
	{ CCI_REG8(0x5b59), 0xcd },
	{ CCI_REG8(0x5b5a), 0xcd },
	{ CCI_REG8(0x5b5b), 0xcd },
	{ CCI_REG8(0x5b5c), 0xcd },
	{ CCI_REG8(0x5b5d), 0xcd },
	{ CCI_REG8(0x5b5e), 0xcd },
	{ CCI_REG8(0x5b5f), 0xcd },
	{ CCI_REG8(0x5b60), 0xcd },
	{ CCI_REG8(0x5b61), 0xcd },
	{ CCI_REG8(0x5b62), 0xcd },
	{ CCI_REG8(0x5b63), 0xcd },
	{ CCI_REG8(0x5b64), 0xcd },
	{ CCI_REG8(0x5b65), 0xcd },
	{ CCI_REG8(0x5b66), 0xcd },
	{ CCI_REG8(0x5b67), 0xcd },
	{ CCI_REG8(0x5b68), 0xcd },
	{ CCI_REG8(0x5b69), 0xcd },
	{ CCI_REG8(0x5b6a), 0xcd },
	{ CCI_REG8(0x5b6b), 0xcd },
	{ CCI_REG8(0x5b6c), 0xcd },
	{ CCI_REG8(0x5b6d), 0xcd },
	{ CCI_REG8(0x5b6e), 0xcd },
	{ CCI_REG8(0x5b6f), 0xcd },
	{ CCI_REG8(0x5b70), 0xcd },
	{ CCI_REG8(0x5b71), 0xcd },
	{ CCI_REG8(0x5b72), 0xcd },
	{ CCI_REG8(0x5b73), 0xcd },
	{ CCI_REG8(0x5b74), 0xcd },
	{ CCI_REG8(0x5b75), 0xcd },
	{ CCI_REG8(0x5b76), 0xcd },
	{ CCI_REG8(0x5b77), 0xcd },
	{ CCI_REG8(0x5b78), 0xcd },
	{ CCI_REG8(0x5b79), 0xcd },
	{ CCI_REG8(0x5b7a), 0xcd },
	{ CCI_REG8(0x5b7b), 0xcd },
	{ CCI_REG8(0x5b7c), 0xcd },
	{ CCI_REG8(0x5b7d), 0xcd },
	{ CCI_REG8(0x5b7e), 0xcd },
	{ CCI_REG8(0x5b7f), 0xcd },
	{ CCI_REG8(0x5b80), 0xcd },
	{ CCI_REG8(0x5b81), 0xcd },
	{ CCI_REG8(0x5b82), 0xcd },
	{ CCI_REG8(0x5b83), 0xcd },
	{ CCI_REG8(0x5b84), 0xcd },
	{ CCI_REG8(0x5b85), 0xcd },
	{ CCI_REG8(0x5b86), 0xcd },
	{ CCI_REG8(0x5b87), 0xcd },
	{ CCI_REG8(0x5b88), 0xcd },
	{ CCI_REG8(0x5b89), 0xcd },
	{ CCI_REG8(0x5b8a), 0xcd },
	{ CCI_REG8(0x5b8b), 0xcd },
	{ CCI_REG8(0x5b8c), 0xcd },
	{ CCI_REG8(0x5b8d), 0xcd },
	{ CCI_REG8(0x5b8e), 0xcd },
	{ CCI_REG8(0x5b8f), 0xcd },
	{ CCI_REG8(0x5b90), 0xcd },
	{ CCI_REG8(0x5b91), 0xcd },
	{ CCI_REG8(0x5b92), 0xcd },
	{ CCI_REG8(0x5b93), 0xcd },
	{ CCI_REG8(0x5b94), 0xcd },
	{ CCI_REG8(0x5b95), 0xcd },
	{ CCI_REG8(0x5b96), 0xcd },
	{ CCI_REG8(0x5b97), 0xcd },
	{ CCI_REG8(0x5b98), 0xcd },
	{ CCI_REG8(0x5b99), 0xcd },
	{ CCI_REG8(0x5b9a), 0xcd },
	{ CCI_REG8(0x5b9b), 0xcd },
	{ CCI_REG8(0x5b9c), 0xcd },
	{ CCI_REG8(0x5b9d), 0xcd },
	{ CCI_REG8(0x5b9e), 0xcd },
	{ CCI_REG8(0x5b9f), 0xcd },
	{ CCI_REG8(0x5ba0), 0xcd },
	{ CCI_REG8(0x5ba1), 0xcd },
	{ CCI_REG8(0x5ba2), 0xcd },
	{ CCI_REG8(0x5ba3), 0xcd },
	{ CCI_REG8(0x5ba4), 0xcd },
	{ CCI_REG8(0x5ba5), 0xcd },
	{ CCI_REG8(0x5ba6), 0xcd },
	{ CCI_REG8(0x5ba7), 0xcd },
	{ CCI_REG8(0x5ba8), 0xcd },
	{ CCI_REG8(0x5ba9), 0xcd },
	{ CCI_REG8(0x5baa), 0xcd },
	{ CCI_REG8(0x5bab), 0xcd },
	{ CCI_REG8(0x5bac), 0xcd },
	{ CCI_REG8(0x5bad), 0xcd },
	{ CCI_REG8(0x5bae), 0xcd },
	{ CCI_REG8(0x5baf), 0xcd },
	{ CCI_REG8(0x5bb0), 0xcd },
	{ CCI_REG8(0x5bb1), 0xcd },
	{ CCI_REG8(0x5bb2), 0xcd },
	{ CCI_REG8(0x5bb3), 0xcd },
	{ CCI_REG8(0x5bb4), 0xcd },
	{ CCI_REG8(0x5bb5), 0xcd },
	{ CCI_REG8(0x5bb6), 0xcd },
	{ CCI_REG8(0x5bb7), 0xcd },
	{ CCI_REG8(0x5bb8), 0xcd },
	{ CCI_REG8(0x5bb9), 0xcd },
	{ CCI_REG8(0x5bba), 0xcd },
	{ CCI_REG8(0x5bbb), 0xcd },
	{ CCI_REG8(0x5bbc), 0xcd },
	{ CCI_REG8(0x5bbd), 0xcd },
	{ CCI_REG8(0x5bbe), 0xcd },
	{ CCI_REG8(0x5bbf), 0xcd },
	{ CCI_REG8(0x5bc0), 0xcd },
	{ CCI_REG8(0x5bc1), 0xcd },
	{ CCI_REG8(0x5bc2), 0xcd },
	{ CCI_REG8(0x5bc3), 0xcd },
	{ CCI_REG8(0x5bc4), 0xcd },
	{ CCI_REG8(0x5bc5), 0xcd },
	{ CCI_REG8(0x5bc6), 0xcd },
	{ CCI_REG8(0x5bc7), 0xcd },
	{ CCI_REG8(0x5bc8), 0xcd },
	{ CCI_REG8(0x5bc9), 0xcd },
	{ CCI_REG8(0x5bca), 0xcd },
	{ CCI_REG8(0x5bcb), 0xcd },
	{ CCI_REG8(0x5bcc), 0xcd },
	{ CCI_REG8(0x5bcd), 0xcd },
	{ CCI_REG8(0x5bce), 0xcd },
	{ CCI_REG8(0x5bcf), 0xcd },
	{ CCI_REG8(0x5bd0), 0xcd },
	{ CCI_REG8(0x5bd1), 0xcd },
	{ CCI_REG8(0x5bd2), 0xcd },
	{ CCI_REG8(0x5bd3), 0xcd },
	{ CCI_REG8(0x5bd4), 0xcd },
	{ CCI_REG8(0x5bd5), 0xcd },
	{ CCI_REG8(0x5bd6), 0xcd },
	{ CCI_REG8(0x5bd7), 0xcd },
	{ CCI_REG8(0x5bd8), 0xcd },
	{ CCI_REG8(0x5bd9), 0xcd },
	{ CCI_REG8(0x5bda), 0xcd },
	{ CCI_REG8(0x5bdb), 0xcd },
	{ CCI_REG8(0x5bdc), 0xcd },
	{ CCI_REG8(0x5bdd), 0xcd },
	{ CCI_REG8(0x5bde), 0xcd },
	{ CCI_REG8(0x5bdf), 0xcd },
	{ CCI_REG8(0x5be0), 0xcd },
	{ CCI_REG8(0x5be1), 0xcd },
	{ CCI_REG8(0x0300), 0x02 },
	{ CCI_REG8(0x0301), 0xc8 },
	{ CCI_REG8(0x0302), 0x33 },
	{ CCI_REG8(0x0303), 0x02 },
	{ CCI_REG8(0x0304), 0x01 },
	{ CCI_REG8(0x0305), 0x5f },
	{ CCI_REG8(0x0306), 0x04 },
	{ CCI_REG8(0x0307), 0x00 },
	{ CCI_REG8(0x0308), 0x03 },
	{ CCI_REG8(0x0309), 0x12 },
	{ CCI_REG8(0x0310), 0x00 },
	{ CCI_REG8(0x0311), 0x00 },
	{ CCI_REG8(0x0312), 0x00 },
	{ CCI_REG8(0x0313), 0x00 },
	{ CCI_REG8(0x0314), 0x00 },
	{ CCI_REG8(0x0315), 0x00 },
	{ CCI_REG8(0x0320), 0x02 },
	{ CCI_REG8(0x0321), 0x03 },
	{ CCI_REG8(0x0322), 0x52 },
	{ CCI_REG8(0x0323), 0x04 },
	{ CCI_REG8(0x0324), 0x01 },
	{ CCI_REG8(0x0325), 0x77 },
	{ CCI_REG8(0x0326), 0x45 },
	{ CCI_REG8(0x0327), 0x04 },
	{ CCI_REG8(0x0328), 0x00 },
	{ CCI_REG8(0x0329), 0x00 },
	{ CCI_REG8(0x032a), 0x06 },
	{ CCI_REG8(0x032b), 0x00 },
	{ CCI_REG8(0x032c), 0x01 },
	{ CCI_REG8(0x032d), 0x01 },
	{ CCI_REG8(0x032f), 0xc1 },
	{ CCI_REG8(0x3012), 0x41 },
	{ CCI_REG8(0x0360), 0x01 },
	{ CCI_REG8(0x4860), 0x00 },
	{ CCI_REG8(0x0316), 0x31 },
	{ CCI_REG8(0x4837), 0x09 },
	{ CCI_REG8(0x4850), 0x47 },
};

static const struct cci_reg_sequence ov32d40_6528x4896_regs[] = {
	{ CCI_REG8(0x0100), 0x00 },
	{ CCI_REG8(0x3403), 0x16 },
	{ CCI_REG8(0x3405), 0x3a },
	{ CCI_REG8(0x3406), 0x2b },
	{ CCI_REG8(0x3408), 0x3a },
	{ CCI_REG8(0x3409), 0x2b },
	{ CCI_REG8(0x340b), 0x33 },
	{ CCI_REG8(0x3419), 0x13 },
	{ CCI_REG8(0x341a), 0xef },
	{ CCI_REG8(0x3761), 0x30 },
	{ CCI_REG8(0x3800), 0x00 },
	{ CCI_REG8(0x3801), 0x00 },
	{ CCI_REG8(0x3802), 0x00 },
	{ CCI_REG8(0x3803), 0x00 },
	{ CCI_REG8(0x3804), 0x19 },
	{ CCI_REG8(0x3805), 0xbf },
	{ CCI_REG8(0x3806), 0x13 },
	{ CCI_REG8(0x3807), 0x5f },
	{ CCI_REG8(0x3808), 0x19 },
	{ CCI_REG8(0x3809), 0x80 },
	{ CCI_REG8(0x380a), 0x13 },
	{ CCI_REG8(0x380b), 0x20 },
	{ CCI_REG8(0x380c), 0x0f },
	{ CCI_REG8(0x380d), 0x50 },
	{ CCI_REG8(0x380e), 0x09 },
	{ CCI_REG8(0x380f), 0xf7 },
	{ CCI_REG8(0x3811), 0x1c },
	{ CCI_REG8(0x3813), 0x1c },
	{ CCI_REG8(0x384c), 0x0f },
	{ CCI_REG8(0x384d), 0x50 },
	{ CCI_REG8(0x388b), 0x10 },
	{ CCI_REG8(0x388c), 0x0c },
	{ CCI_REG8(0x388d), 0xc0 },
	{ CCI_REG8(0x388e), 0x09 },
	{ CCI_REG8(0x388f), 0x90 },
	{ CCI_REG8(0x3a11), 0x1d },
	{ CCI_REG8(0x3a20), 0x3d },
	{ CCI_REG8(0x3a26), 0xa7 },
	{ CCI_REG8(0x3a29), 0x5c },
	{ CCI_REG8(0x3f00), 0x00 },
	{ CCI_REG8(0x4016), 0x2f },
	{ CCI_REG8(0x4509), 0x05 },
	{ CCI_REG8(0x4837), 0x09 },
	{ CCI_REG8(0x4c62), 0x0c },
	{ CCI_REG8(0x4c63), 0xc0 },
	{ CCI_REG8(0x4c65), 0x10 },
	{ CCI_REG8(0x4c66), 0x09 },
	{ CCI_REG8(0x4c67), 0x90 },
	{ CCI_REG8(0x4c68), 0x0c },
	{ CCI_REG8(0x4c69), 0xe0 },
	{ CCI_REG8(0x4c6a), 0x09 },
	{ CCI_REG8(0x4c6b), 0xb0 },
	{ CCI_REG8(0x5001), 0x40 },
	{ CCI_REG8(0x5a01), 0x44 },
};

static const struct cci_reg_sequence ov32d40_3264x2448_regs[] = {
	{ CCI_REG8(0x0100), 0x00 },
	{ CCI_REG8(0x3403), 0x4a },
	{ CCI_REG8(0x3405), 0xc0 },
	{ CCI_REG8(0x3406), 0x8f },
	{ CCI_REG8(0x3408), 0xc0 },
	{ CCI_REG8(0x3409), 0x8f },
	{ CCI_REG8(0x340b), 0x3b },
	{ CCI_REG8(0x3419), 0x16 },
	{ CCI_REG8(0x341a), 0x09 },
	{ CCI_REG8(0x3761), 0x3e },
	{ CCI_REG8(0x3800), 0x00 },
	{ CCI_REG8(0x3801), 0x00 },
	{ CCI_REG8(0x3802), 0x00 },
	{ CCI_REG8(0x3803), 0x00 },
	{ CCI_REG8(0x3804), 0x19 },
	{ CCI_REG8(0x3805), 0xbf },
	{ CCI_REG8(0x3806), 0x13 },
	{ CCI_REG8(0x3807), 0x5f },
	{ CCI_REG8(0x3808), 0x0c },
	{ CCI_REG8(0x3809), 0xc0 },
	{ CCI_REG8(0x380a), 0x09 },
	{ CCI_REG8(0x380b), 0x90 },
	{ CCI_REG8(0x380c), 0x04 },
	{ CCI_REG8(0x380d), 0x9e },
	{ CCI_REG8(0x380e), 0x0b },
	{ CCI_REG8(0x380f), 0x04 },
	{ CCI_REG8(0x3811), 0x10 },
	{ CCI_REG8(0x3813), 0x10 },
	{ CCI_REG8(0x384c), 0x04 },
	{ CCI_REG8(0x384d), 0x9e },
	{ CCI_REG8(0x388b), 0x10 },
	{ CCI_REG8(0x388c), 0x0c },
	{ CCI_REG8(0x388d), 0xc0 },
	{ CCI_REG8(0x388e), 0x09 },
	{ CCI_REG8(0x388f), 0x90 },
	{ CCI_REG8(0x3a11), 0x14 },
	{ CCI_REG8(0x3a20), 0x22 },
	{ CCI_REG8(0x3a26), 0x9f },
	{ CCI_REG8(0x3a29), 0x41 },
	{ CCI_REG8(0x3f00), 0x02 },
	{ CCI_REG8(0x4016), 0x3d },
	{ CCI_REG8(0x4509), 0x15 },
	{ CCI_REG8(0x4837), 0x09 },
	{ CCI_REG8(0x4c62), 0x0c },
	{ CCI_REG8(0x4c63), 0xc0 },
	{ CCI_REG8(0x4c65), 0x10 },
	{ CCI_REG8(0x4c66), 0x09 },
	{ CCI_REG8(0x4c67), 0x90 },
	{ CCI_REG8(0x4c68), 0x0c },
	{ CCI_REG8(0x4c69), 0xe0 },
	{ CCI_REG8(0x4c6a), 0x09 },
	{ CCI_REG8(0x4c6b), 0xb0 },
	{ CCI_REG8(0x5001), 0x00 },
	{ CCI_REG8(0x5a01), 0xdc },
};

static const struct cci_reg_sequence ov32d40_3264x1836_regs[] = {
	{ CCI_REG8(0x0100), 0x00 },
	{ CCI_REG8(0x3403), 0x4a },
	{ CCI_REG8(0x3405), 0xc0 },
	{ CCI_REG8(0x3406), 0x8f },
	{ CCI_REG8(0x3408), 0xc0 },
	{ CCI_REG8(0x3409), 0x8f },
	{ CCI_REG8(0x340b), 0x3b },
	{ CCI_REG8(0x3419), 0x16 },
	{ CCI_REG8(0x341a), 0x09 },
	{ CCI_REG8(0x3761), 0x3e },
	{ CCI_REG8(0x3800), 0x00 },
	{ CCI_REG8(0x3801), 0x00 },
	{ CCI_REG8(0x3802), 0x02 },
	{ CCI_REG8(0x3803), 0x64 },
	{ CCI_REG8(0x3804), 0x19 },
	{ CCI_REG8(0x3805), 0xbf },
	{ CCI_REG8(0x3806), 0x10 },
	{ CCI_REG8(0x3807), 0xfb },
	{ CCI_REG8(0x3808), 0x0c },
	{ CCI_REG8(0x3809), 0xc0 },
	{ CCI_REG8(0x380a), 0x07 },
	{ CCI_REG8(0x380b), 0x2c },
	{ CCI_REG8(0x380c), 0x04 },
	{ CCI_REG8(0x380d), 0x9e },
	{ CCI_REG8(0x380e), 0x0b },
	{ CCI_REG8(0x380f), 0x04 },
	{ CCI_REG8(0x3811), 0x10 },
	{ CCI_REG8(0x3813), 0x10 },
	{ CCI_REG8(0x384c), 0x04 },
	{ CCI_REG8(0x384d), 0x9e },
	{ CCI_REG8(0x388b), 0x16 },
	{ CCI_REG8(0x388c), 0x0c },
	{ CCI_REG8(0x388d), 0xc0 },
	{ CCI_REG8(0x388e), 0x07 },
	{ CCI_REG8(0x388f), 0x20 },
	{ CCI_REG8(0x3a11), 0x14 },
	{ CCI_REG8(0x3a20), 0x22 },
	{ CCI_REG8(0x3a26), 0x9f },
	{ CCI_REG8(0x3a29), 0x41 },
	{ CCI_REG8(0x3f00), 0x02 },
	{ CCI_REG8(0x4016), 0x3d },
	{ CCI_REG8(0x4509), 0x15 },
	{ CCI_REG8(0x4837), 0x09 },
	{ CCI_REG8(0x4c62), 0x0c },
	{ CCI_REG8(0x4c63), 0xc0 },
	{ CCI_REG8(0x4c65), 0x16 },
	{ CCI_REG8(0x4c66), 0x07 },
	{ CCI_REG8(0x4c67), 0x20 },
	{ CCI_REG8(0x4c68), 0x0c },
	{ CCI_REG8(0x4c69), 0xe0 },
	{ CCI_REG8(0x4c6a), 0x07 },
	{ CCI_REG8(0x4c6b), 0x4c },
	{ CCI_REG8(0x5001), 0x00 },
	{ CCI_REG8(0x5a01), 0xdc },
};

static const struct cci_reg_sequence ov32d40_streamon_regs[] = {
	{ CCI_REG8(0x3716), 0x00 },
	{ CCI_REG8(0x3717), 0x00 },
	{ CCI_REG8(0x3729), 0x7c },
	{ CCI_REG8(0x0100), 0x01 },
};

static const struct cci_reg_sequence ov32d40_streamoff_regs[] = {
	{ CCI_REG8(0x3716), 0x08 },
	{ CCI_REG8(0x3717), 0x20 },
	{ CCI_REG8(0x3729), 0x7e },
	{ CCI_REG8(0x0100), 0x00 },
};

static const char * const ov32d40_test_pattern_menu[] = {
	"Disabled",
	"Solid Colour",
	"Pseudorandom Sequence (PN9)",
};

static const s64 link_freq_menu_items[] = {
	842400000,
};

static u64 to_pixel_rate(u32 f_index)
{
	u64 pixel_rate = link_freq_menu_items[f_index] * 2 * OV32D40_DATA_LANES;

	do_div(pixel_rate, OV32D40_BITS_PER_SAMPLE);

	return pixel_rate;
}

static const struct ov32d40_mode supported_modes[] = {
	/* 6k 4:3 10fps */
	{
		.width = 6528,
		.height = 4896,
		.crop = {
			.left = OV32D40_PIXEL_ARRAY_LEFT,
			.top = OV32D40_PIXEL_ARRAY_TOP,
			.width = 6528,
			.height = 4896,
		},
		.line_length = 3920,
		.frame_length = 2551,
		.reg_list = {
			.num_of_regs = ARRAY_SIZE(ov32d40_6528x4896_regs),
			.regs = ov32d40_6528x4896_regs,
		},
	},
	/* 3k 4:3 30fps */
	{
		.width = 3264,
		.height = 2448,
		.crop = {
			.left = OV32D40_PIXEL_ARRAY_LEFT,
			.top = OV32D40_PIXEL_ARRAY_TOP,
			.width = 3264 * 2,
			.height = 2448 * 2,
		},
		.line_length = 1182,
		.frame_length = 2820,
		.reg_list = {
			.num_of_regs = ARRAY_SIZE(ov32d40_3264x2448_regs),
			.regs = ov32d40_3264x2448_regs,
		},
	},
	/* 3k 16:9 30fps */
	{
		.width = 3264,
		.height = 1836,
		.crop = {
			.left = OV32D40_PIXEL_ARRAY_LEFT,
			.top = 612 + OV32D40_PIXEL_ARRAY_TOP,
			.width = 3264 * 2,
			.height = 1836 * 2,
		},
		.line_length = 1182,
		.frame_length = 2820,
		.reg_list = {
			.num_of_regs = ARRAY_SIZE(ov32d40_3264x1836_regs),
			.regs = ov32d40_3264x1836_regs,
		},
	},
};

static int ov32d40_check_hwcfg(struct device *dev, struct ov32d40 *ov32d40)
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

static int __ov32d40_start_stream(struct ov32d40 *ov32d40)
{
	const struct ov32d40_reg_list *reg_list;
	int ret;

	/* Apply default values of current mode */
	reg_list = &ov32d40->cur_mode->reg_list;

	ret = cci_write(ov32d40->regmap, CCI_REG8(0x1001), 0x04, NULL);
	if (ret)
		return ret;

	usleep_range(500, 1000);

	ret = cci_multi_reg_write(ov32d40->regmap, ov32d40_init_regs, ARRAY_SIZE(ov32d40_init_regs), NULL);
	if (ret)
		return ret;

	ret = cci_multi_reg_write(ov32d40->regmap, reg_list->regs, reg_list->num_of_regs, NULL);
	if (ret)
		return ret;

	/* Apply customized values from user */
	ret = __v4l2_ctrl_handler_setup(ov32d40->subdev.ctrl_handler);
	if (ret)
		return ret;

	ret = cci_multi_reg_write(ov32d40->regmap, ov32d40_streamon_regs, ARRAY_SIZE(ov32d40_streamon_regs), NULL);
	if (ret)
		return ret;

	return 0;
}

static int __ov32d40_stop_stream(struct ov32d40 *ov32d40)
{
	int ret;

	ret = cci_multi_reg_write(ov32d40->regmap, ov32d40_streamoff_regs, ARRAY_SIZE(ov32d40_streamoff_regs), NULL);
	if (ret)
		return ret;

	return 0;
}

static int ov32d40_s_stream(struct v4l2_subdev *sd, int on)
{
	struct ov32d40 *ov32d40 = to_ov32d40(sd);
	struct i2c_client *client = v4l2_get_subdevdata(&ov32d40->subdev);
	int ret;

	mutex_lock(&ov32d40->mutex);

	if (ov32d40->streaming == on) {
		ret = 0;
		goto unlock_and_return;
	}

	if (on) {
		ret = pm_runtime_resume_and_get(&client->dev);
		if (ret < 0)
			goto unlock_and_return;

		ret = __ov32d40_start_stream(ov32d40);
		if (ret) {
			__ov32d40_stop_stream(ov32d40);
			ov32d40->streaming = !on;
			goto err_rpm_put;
		}
	} else {
		__ov32d40_stop_stream(ov32d40);
		pm_runtime_put(&client->dev);
	}

	ov32d40->streaming = on;
	mutex_unlock(&ov32d40->mutex);

	return 0;

err_rpm_put:
	pm_runtime_put(&client->dev);
unlock_and_return:
	mutex_unlock(&ov32d40->mutex);

	return ret;
}

static int ov32d40_enum_mbus_code(struct v4l2_subdev *sd,
				  struct v4l2_subdev_state *sd_state,
				  struct v4l2_subdev_mbus_code_enum *code)
{
	if (code->index != 0)
		return -EINVAL;

	code->code = MEDIA_BUS_FMT_SBGGR10_1X10;

	return 0;
}

static int ov32d40_enum_frame_sizes(struct v4l2_subdev *sd,
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

static void ov32d40_fill_fmt(const struct ov32d40_mode *mode,
			     struct v4l2_mbus_framefmt *fmt)
{
	fmt->width = mode->width;
	fmt->height = mode->height;
	fmt->code = MEDIA_BUS_FMT_SBGGR10_1X10;
	fmt->field = V4L2_FIELD_NONE;
}

static int ov32d40_get_fmt(struct v4l2_subdev *sd,
			   struct v4l2_subdev_state *sd_state,
			   struct v4l2_subdev_format *fmt)
{
	struct ov32d40 *ov32d40 = to_ov32d40(sd);

	mutex_lock(&ov32d40->mutex);

	if (fmt->which == V4L2_SUBDEV_FORMAT_TRY)
		fmt->format = *v4l2_subdev_state_get_format(sd_state, fmt->pad);
	else
		ov32d40_fill_fmt(ov32d40->cur_mode, &fmt->format);

	mutex_unlock(&ov32d40->mutex);

	return 0;
}

static int ov32d40_set_fmt(struct v4l2_subdev *sd,
			   struct v4l2_subdev_state *sd_state,
			   struct v4l2_subdev_format *fmt)
{
	struct ov32d40 *ov32d40 = to_ov32d40(sd);
	const struct ov32d40_mode *mode;
	struct v4l2_rect *crop;
	s32 vblank_def, hblank_def;
	int ret = 0;

	mode = v4l2_find_nearest_size(supported_modes,
				      ARRAY_SIZE(supported_modes), width,
				      height, fmt->format.width,
				      fmt->format.height);

	mutex_lock(&ov32d40->mutex);

	if (ov32d40->streaming && fmt->which == V4L2_SUBDEV_FORMAT_ACTIVE) {
		ret = -EBUSY;
		goto out_unlock;
	}

	ov32d40_fill_fmt(mode, &fmt->format);

	if (fmt->which == V4L2_SUBDEV_FORMAT_TRY) {
		*v4l2_subdev_state_get_format(sd_state, 0) = fmt->format;
		crop = v4l2_subdev_state_get_crop(sd_state, 0);
		crop->left = mode->crop.left;
		crop->top = mode->crop.top;
		crop->width = mode->crop.width;
		crop->height = mode->crop.height;
	} else {
		ov32d40->cur_mode = mode;

		vblank_def = mode->frame_length - mode->height;
		__v4l2_ctrl_modify_range(ov32d40->vblank, vblank_def,
					 OV32D40_FRAME_LENGTH_MAX - mode->height, 1,
					 vblank_def);
		__v4l2_ctrl_s_ctrl(ov32d40->vblank, vblank_def);

		hblank_def = mode->line_length - mode->width;
		__v4l2_ctrl_modify_range(ov32d40->hblank, hblank_def, hblank_def, 1,
					 hblank_def);
	}

out_unlock:
	mutex_unlock(&ov32d40->mutex);
	return ret;
}

static int ov32d40_get_selection(struct v4l2_subdev *sd,
				 struct v4l2_subdev_state *sd_state,
				 struct v4l2_subdev_selection *sel)
{
	switch (sel->target) {
	case V4L2_SEL_TGT_CROP: {
		struct ov32d40 *ov32d40 = to_ov32d40(sd);

		mutex_lock(&ov32d40->mutex);
		switch (sel->which) {
		case V4L2_SUBDEV_FORMAT_TRY:
			sel->r = *v4l2_subdev_state_get_crop(sd_state, sel->pad);
			break;
		case V4L2_SUBDEV_FORMAT_ACTIVE:
			sel->r = ov32d40->cur_mode->crop;
			break;
		}
		mutex_unlock(&ov32d40->mutex);
		return 0;
	}
	case V4L2_SEL_TGT_NATIVE_SIZE:
		sel->r.left = 0;
		sel->r.top = 0;
		sel->r.width = OV32D40_NATIVE_WIDTH;
		sel->r.height = OV32D40_NATIVE_HEIGHT;
		return 0;
	case V4L2_SEL_TGT_CROP_DEFAULT:
	case V4L2_SEL_TGT_CROP_BOUNDS:
		sel->r.left = OV32D40_PIXEL_ARRAY_LEFT;
		sel->r.top = OV32D40_PIXEL_ARRAY_TOP;
		sel->r.width = OV32D40_PIXEL_ARRAY_WIDTH;
		sel->r.height = OV32D40_PIXEL_ARRAY_HEIGHT;
		return 0;
	}

	return -EINVAL;
}

static int ov32d40_init_state(struct v4l2_subdev *sd,
			      struct v4l2_subdev_state *sd_state)
{
	struct v4l2_subdev_format fmt = { 0 };

	fmt.which = sd_state ? V4L2_SUBDEV_FORMAT_TRY : V4L2_SUBDEV_FORMAT_ACTIVE;

	ov32d40_set_fmt(sd, sd_state, &fmt);

	return 0;
}

static int ov32d40_set_ctrl(struct v4l2_ctrl *ctrl)
{
	struct ov32d40 *ov32d40 = container_of(ctrl->handler,
					       struct ov32d40, ctrl_handler);
	struct i2c_client *client = v4l2_get_subdevdata(&ov32d40->subdev);
	s32 max_expo;
	int ret;

	/* Propagate change of current control to all related controls */
	if (ctrl->id == V4L2_CID_VBLANK) {
		/* Update max exposure while meeting expected vblanking */
		max_expo = ov32d40->cur_mode->height + ctrl->val -
			   OV32D40_EXPOSURE_MAX_MARGIN;
		__v4l2_ctrl_modify_range(ov32d40->exposure,
					 ov32d40->exposure->minimum, max_expo,
					 ov32d40->exposure->step,
					 ov32d40->exposure->default_value);
	}

	/* V4L2 controls values will be applied only when power is already up */
	if (!pm_runtime_get_if_in_use(&client->dev))
		return 0;

	switch (ctrl->id) {
	case V4L2_CID_EXPOSURE:
		ret = cci_write(ov32d40->regmap, OV32D40_REG_EXPO, ctrl->val, NULL);
		break;
	case V4L2_CID_ANALOGUE_GAIN:
		ret = cci_write(ov32d40->regmap, OV32D40_REG_GAIN, ctrl->val, NULL);
		break;
	case V4L2_CID_VBLANK:
		ret = cci_write(ov32d40->regmap, OV32D40_REG_FRAME_LENGTH,
				ov32d40->cur_mode->height + ctrl->val, NULL);
		break;
	case V4L2_CID_TEST_PATTERN:
		if (ctrl->val)
			cci_write(ov32d40->regmap, CCI_REG8(0x3019), 0xf0, NULL);
		else
			cci_write(ov32d40->regmap, CCI_REG8(0x3019), 0xd2, NULL);
		ret = cci_write(ov32d40->regmap, OV32D40_REG_TEST_PATTERN, ctrl->val, NULL);
		break;
	default:
		ret = -EINVAL;
		break;
	}

	pm_runtime_put(&client->dev);

	return ret;
}

static const struct v4l2_subdev_video_ops ov32d40_video_ops = {
	.s_stream = ov32d40_s_stream,
};

static const struct v4l2_subdev_pad_ops ov32d40_pad_ops = {
	.enum_mbus_code = ov32d40_enum_mbus_code,
	.enum_frame_size = ov32d40_enum_frame_sizes,
	.get_selection = ov32d40_get_selection,
	.get_fmt = ov32d40_get_fmt,
	.set_fmt = ov32d40_set_fmt,
};

static const struct v4l2_subdev_ops ov32d40_subdev_ops = {
	.video = &ov32d40_video_ops,
	.pad = &ov32d40_pad_ops,
};

static const struct media_entity_operations ov32d40_subdev_entity_ops = {
	.link_validate = v4l2_subdev_link_validate,
};

static const struct v4l2_subdev_internal_ops ov32d40_internal_ops = {
	.init_state = ov32d40_init_state,
};

static const struct v4l2_ctrl_ops ov32d40_ctrl_ops = {
	.s_ctrl = ov32d40_set_ctrl,
};

static int ov32d40_initialize_controls(struct ov32d40 *ov32d40)
{
	struct i2c_client *client = v4l2_get_subdevdata(&ov32d40->subdev);
	const struct ov32d40_mode *mode;
	struct v4l2_ctrl_handler *handler;
	struct v4l2_ctrl *ctrl;
	struct v4l2_fwnode_device_properties props;
	s32 exposure_max;
	s32 vblank_def, hblank_def;
	s32 pixel_rate;
	int ret;

	handler = &ov32d40->ctrl_handler;
	mode = ov32d40->cur_mode;
	ret = v4l2_ctrl_handler_init(handler, 7);
	if (ret)
		return ret;

	handler->lock = &ov32d40->mutex;

	ctrl = v4l2_ctrl_new_int_menu(handler, NULL, V4L2_CID_LINK_FREQ, 0, 0,
				      link_freq_menu_items);
	if (ctrl)
		ctrl->flags |= V4L2_CTRL_FLAG_READ_ONLY;

	pixel_rate = to_pixel_rate(0);
	v4l2_ctrl_new_std(handler, NULL, V4L2_CID_PIXEL_RATE, 0, pixel_rate, 1,
			  pixel_rate);

	hblank_def = mode->line_length - mode->width;
	ov32d40->hblank = v4l2_ctrl_new_std(handler, NULL, V4L2_CID_HBLANK,
			  		    hblank_def, hblank_def, 1, hblank_def);

	vblank_def = mode->frame_length - mode->height;
	ov32d40->vblank = v4l2_ctrl_new_std(handler, &ov32d40_ctrl_ops, V4L2_CID_VBLANK, vblank_def,
			  		    OV32D40_FRAME_LENGTH_MAX - mode->height, 1, vblank_def);

	exposure_max = mode->frame_length - OV32D40_EXPOSURE_MAX_MARGIN;
	ov32d40->exposure = v4l2_ctrl_new_std(handler, &ov32d40_ctrl_ops,
			  		      V4L2_CID_EXPOSURE,
			  		      OV32D40_EXPOSURE_MIN,
			  		      exposure_max, 1,
			  		      OV32D40_EXPOSURE_DEFAULT);

	v4l2_ctrl_new_std(handler, &ov32d40_ctrl_ops,
			  V4L2_CID_ANALOGUE_GAIN, OV32D40_ANA_GAIN_MIN,
			  OV32D40_ANA_GAIN_MAX, 1,
			  OV32D40_ANA_GAIN_DEFAULT);

	v4l2_ctrl_new_std_menu_items(handler, &ov32d40_ctrl_ops,
				     V4L2_CID_TEST_PATTERN,
				     ARRAY_SIZE(ov32d40_test_pattern_menu) - 1,
				     0, 0, ov32d40_test_pattern_menu);

	if (handler->error) {
		ret = handler->error;
		dev_err(&client->dev, "failed to init controls(%d)\n", ret);
		goto err_free_handler;
	}

	ret = v4l2_fwnode_device_parse(&client->dev, &props);
	if (ret)
		goto err_free_handler;

	ret = v4l2_ctrl_new_fwnode_properties(handler, &ov32d40_ctrl_ops, &props);
	if (ret)
		goto err_free_handler;

	ov32d40->subdev.ctrl_handler = handler;

	return 0;

err_free_handler:
	v4l2_ctrl_handler_free(handler);

	return ret;
}

static int ov32d40_check_sensor_id(struct ov32d40 *ov32d40)
{
	u64 chip_id;
	int ret;

	/* Validate the chip ID */
	ret = cci_read(ov32d40->regmap, OV32D40_REG_CHIP_ID, &chip_id, NULL);
	if (ret < 0) {
		dev_err(ov32d40->dev, "failed to read sensor information\n");
		return ret;
	}

	if (chip_id != OV32D40_ID) {
		dev_err(ov32d40->dev, "unexpected sensor id(0x%04llx)\n", chip_id);
		return -EINVAL;
	}

	return 0;
}

static int ov32d40_power_on(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct ov32d40 *ov32d40 = to_ov32d40(sd);
	int ret;

	gpiod_set_value_cansleep(ov32d40->reset_gpio, 1);

	ret = regulator_bulk_enable(ARRAY_SIZE(ov32d40_supply_names),
				    ov32d40->supplies);
	if (ret < 0) {
		dev_err(dev, "failed to enable regulators\n");
		goto disable_clk;
	}
	usleep_range(1000, 2000);

	ret = clk_prepare_enable(ov32d40->mclk);
	if (ret < 0) {
		dev_err(dev, "failed to enable mclk\n");
		return ret;
	}
	usleep_range(1000, 2000);

	gpiod_set_value_cansleep(ov32d40->reset_gpio, 0);
	usleep_range(5000, 6000);

	ret = ov32d40_check_sensor_id(ov32d40);
	if (ret)
		goto disable_regulator;

	return 0;

disable_regulator:
	regulator_bulk_disable(ARRAY_SIZE(ov32d40_supply_names),
			       ov32d40->supplies);
disable_clk:
	clk_disable_unprepare(ov32d40->mclk);

	return ret;
}

static int ov32d40_power_off(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct ov32d40 *ov32d40 = to_ov32d40(sd);

	gpiod_set_value_cansleep(ov32d40->reset_gpio, 1);
	clk_disable_unprepare(ov32d40->mclk);
	regulator_bulk_disable(ARRAY_SIZE(ov32d40_supply_names),
			       ov32d40->supplies);

	return 0;
}

static const struct dev_pm_ops ov32d40_pm_ops = {
	SET_RUNTIME_PM_OPS(ov32d40_power_off, ov32d40_power_on, NULL)
};

static int ov32d40_probe(struct i2c_client *client)
{
	struct ov32d40 *ov32d40;
	unsigned int i;
	int ret;

	ov32d40 = devm_kzalloc(&client->dev, sizeof(*ov32d40), GFP_KERNEL);
	if (!ov32d40)
		return -ENOMEM;

	ov32d40->dev = &client->dev;

	ret = ov32d40_check_hwcfg(ov32d40->dev, ov32d40);
	if (ret)
		return dev_err_probe(ov32d40->dev, ret, "failed to check HW configuration\n");

	ov32d40->regmap = devm_cci_regmap_init_i2c(client, 16);
	if (IS_ERR(ov32d40->regmap))
		return dev_err_probe(ov32d40->dev, PTR_ERR(ov32d40->regmap), "failed to init regmap\n");

	v4l2_i2c_subdev_init(&ov32d40->subdev, client, &ov32d40_subdev_ops);
	ov32d40->subdev.internal_ops = &ov32d40_internal_ops;

	ov32d40->mclk = devm_clk_get(ov32d40->dev, NULL);
	if (IS_ERR(ov32d40->mclk))
		return dev_err_probe(ov32d40->dev, PTR_ERR(ov32d40->mclk), "failed to get mclk\n");

	ret = clk_set_rate(ov32d40->mclk, 19200000);
	if (ret < 0)
		return dev_err_probe(ov32d40->dev, ret, "failed to set mclk frequency\n");

	ov32d40->reset_gpio = devm_gpiod_get(ov32d40->dev, "reset", GPIOD_OUT_LOW);
	if (IS_ERR(ov32d40->reset_gpio))
		return dev_err_probe(ov32d40->dev, PTR_ERR(ov32d40->reset_gpio),
				     "failed to get reset-gpios\n");

	for (i = 0; i < ARRAY_SIZE(ov32d40_supply_names); i++)
		ov32d40->supplies[i].supply = ov32d40_supply_names[i];

	ret = devm_regulator_bulk_get(ov32d40->dev, ARRAY_SIZE(ov32d40_supply_names),
				      ov32d40->supplies);
	if (ret)
		return dev_err_probe(ov32d40->dev, ret, "failed to get regulators\n");

	mutex_init(&ov32d40->mutex);

	/* Set default mode */
	ov32d40->cur_mode = &supported_modes[0];

	ret = ov32d40_initialize_controls(ov32d40);
	if (ret) {
		dev_err_probe(ov32d40->dev, ret, "failed to initialize controls\n");
		goto err_destroy_mutex;
	}

	/* Initialize subdev */
	ov32d40->subdev.flags |= V4L2_SUBDEV_FL_HAS_DEVNODE;
	ov32d40->subdev.entity.ops = &ov32d40_subdev_entity_ops;
	ov32d40->subdev.entity.function = MEDIA_ENT_F_CAM_SENSOR;
	ov32d40->pad.flags = MEDIA_PAD_FL_SOURCE;

	ret = media_entity_pads_init(&ov32d40->subdev.entity, 1, &ov32d40->pad);
	if (ret < 0) {
		dev_err_probe(ov32d40->dev, ret, "failed to initialize entity pads\n");
		goto err_free_handler;
	}

	pm_runtime_enable(ov32d40->dev);
	if (!pm_runtime_enabled(ov32d40->dev)) {
		ret = ov32d40_power_on(ov32d40->dev);
		if (ret < 0) {
			dev_err_probe(ov32d40->dev, ret, "failed to power on\n");
			goto err_clean_entity;
		}
	}

	ret = v4l2_async_register_subdev_sensor(&ov32d40->subdev);
	if (ret) {
		dev_err_probe(ov32d40->dev, ret, "failed to register V4L2 subdev\n");
		goto err_power_off;
	}

	return 0;

err_power_off:
	if (pm_runtime_enabled(ov32d40->dev))
		pm_runtime_disable(ov32d40->dev);
	else
		ov32d40_power_off(ov32d40->dev);
err_clean_entity:
	media_entity_cleanup(&ov32d40->subdev.entity);
err_free_handler:
	v4l2_ctrl_handler_free(ov32d40->subdev.ctrl_handler);
err_destroy_mutex:
	mutex_destroy(&ov32d40->mutex);

	return ret;
}

static void ov32d40_remove(struct i2c_client *client)
{
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct ov32d40 *ov32d40 = to_ov32d40(sd);

	v4l2_async_unregister_subdev(sd);
	media_entity_cleanup(&sd->entity);
	v4l2_ctrl_handler_free(sd->ctrl_handler);

	pm_runtime_disable(&client->dev);
	if (!pm_runtime_status_suspended(&client->dev))
		ov32d40_power_off(&client->dev);

	pm_runtime_set_suspended(&client->dev);

	mutex_destroy(&ov32d40->mutex);
}

static const struct of_device_id ov32d40_of_match[] = {
	{ .compatible = "ovti,ov32d40" },
	{}
};
MODULE_DEVICE_TABLE(of, ov32d40_of_match);

static struct i2c_driver ov32d40_i2c_driver = {
	.driver = {
		.name = "ov32d40",
		.pm = &ov32d40_pm_ops,
		.of_match_table = ov32d40_of_match,
	},
	.probe = ov32d40_probe,
	.remove	= ov32d40_remove,
};
module_i2c_driver(ov32d40_i2c_driver);

MODULE_AUTHOR("map220v <map220v300@gmail.com>");
MODULE_DESCRIPTION("OmniVision OV32D40 camera sensor driver");
MODULE_LICENSE("GPL v2");
