// SPDX-License-Identifier: GPL-2.0-only
/*
 * HID driver for Nanosic WN8030 keyboard MCU
 *
 * Copyright (c) 2025 map220v <map220v300@gmail.com>
 */

#include <linux/delay.h>
#include <linux/firmware.h>
#include <linux/hid.h>
#include <linux/i2c.h>
#include <linux/interrupt.h>
#include <linux/kernel.h>
#include <linux/miscdevice.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/regmap.h>
#include <linux/workqueue.h>

#define WN8030_HEADER_ADDR 0x7FC0
#define WN8030_CODE_ADDR 0x8000

#define XM_WN8030_I2C_READ (68)
#define XM_WN8030_I2C_WRITE (66)

static const char * const nanosic_wn8030_supply_names[] = {
	"vddio",	/* I/O power supply (1.8V) */
	"dvdd",		/* Digital power supply (3.3V) */
};

struct nanosic_wn8030 {
	struct device *dev;
	struct i2c_client *client;
	struct hid_device *hid_keyboard;
	struct hid_device *hid_touchpad;

	struct gpio_desc *status_gpio;
	struct gpio_desc *sleep_gpio;
	struct gpio_desc *reset_gpio;
	struct regulator_bulk_data supplies[ARRAY_SIZE(nanosic_wn8030_supply_names)];

	struct regmap *regmap;
	struct mutex conn_mutex;
	struct delayed_work wake_worker;

	bool suspended;
	bool keyboard_attached;

	/* Authentication */
	struct miscdevice auth_misc;
	wait_queue_head_t auth_read_wq;
	struct completion auth_token_ready;
	bool auth_open;
	bool auth_request_pending;
	u8 auth_uid[16];
	u8 auth_challenge[16];
	u8 auth_token[16];
};

static const struct regmap_config nanosic_wn8030_regmap_config = {
	.reg_bits = 8,
	.val_bits = 8,
};

#pragma pack(push, 1)
struct wn8030_bin_header {
	u8 flag[7];
	u16 chip_id;
	u32 ro_data;
	u32 rame_code; //??
	u32 rame_code_size; //??
	u32 code_entry;
	u16 pid;
	u16 vid;
	u8 major_version;
	u8 minor_version;
	char version_string[17];
	u32 ram_code_size;
	u32 ram_code_checksum;
	u32 unk;
	u8 device_type;
	u8 unk1[3];
};
#pragma pack(pop)

#pragma pack(push, 1)
struct wn8030_header_cmd {
	u8 cmd; /* 0x06 */
	u8 flag[7];
	u16 chip_id;
	u32 ram_code_source;
	u32 ram_code_dest;
	u32 ram_code_size;
	u32 code_entry;
	u16 pid;
	u16 vid;
	u8 major_version;
	u8 minor_version;
	u8 unk[17]; /* version_string? */
	u32 image_size;
	u32 ram_code_checksum;
	u32 image_checksum;
	u8 pad[3];
	u8 header_checksum;
};
#pragma pack(pop)

#pragma pack(push, 1)
struct wn8030_send_cmd {
	u8 cmd; /* 0x07 */
	__be32 ram_dest;
	__be32 size;
	u8 data[503];
};
#pragma pack(pop)

static u8 hid_keyboard_descriptor[] = {
	0x05, 0x01,        // Usage Page (Generic Desktop Ctrls)
	0x09, 0x06,        // Usage (Keyboard)
	0xA1, 0x01,        // Collection (Application)
	0x85, 0x05,        //   Report ID (5)
	0x05, 0x07,        //   Usage Page (Kbrd/Keypad)
	0x19, 0xE0,        //   Usage Minimum (0xE0)
	0x29, 0xE7,        //   Usage Maximum (0xE7)
	0x15, 0x00,        //   Logical Minimum (0)
	0x25, 0x01,        //   Logical Maximum (1)
	0x75, 0x01,        //   Report Size (1)
	0x95, 0x08,        //   Report Count (8)
	0x81, 0x02,        //   Input (Data,Var,Abs,No Wrap,Linear,Preferred State,No Null Position)
	0x81, 0x03,        //   Input (Const,Var,Abs,No Wrap,Linear,Preferred State,No Null Position)
	0x95, 0x05,        //   Report Count (5)
	0x05, 0x08,        //   Usage Page (LEDs)
	0x19, 0x02,        //   Usage Minimum (Caps Lock)
	0x29, 0x02,        //   Usage Maximum (Caps Lock)
	0x91, 0x02,        //   Output (Data,Var,Abs,No Wrap,Linear,Preferred State,No Null Position,Non-volatile)
	0x95, 0x01,        //   Report Count (1)
	0x75, 0x03,        //   Report Size (3)
	0x91, 0x01,        //   Output (Const,Array,Abs,No Wrap,Linear,Preferred State,No Null Position,Non-volatile)
	0x95, 0x06,        //   Report Count (6)
	0x75, 0x08,        //   Report Size (8)
	0x15, 0x00,        //   Logical Minimum (0)
	0x26, 0xA4, 0x00,  //   Logical Maximum (164)
	0x05, 0x07,        //   Usage Page (Kbrd/Keypad)
	0x19, 0x00,        //   Usage Minimum (0x00)
	0x2A, 0xA4, 0x00,  //   Usage Maximum (0xA4)
	0x81, 0x00,        //   Input (Data,Array,Abs,No Wrap,Linear,Preferred State,No Null Position)
	0xC0,              // End Collection
};

static u8 hid_touchpad_descriptor[] = {
	0x05, 0x0D,        // Usage Page (Digitizer)
	0x09, 0x05,        // Usage (Touch Pad)
	0xA1, 0x01,        // Collection (Application)
	0x85, 0x19,        //   Report ID (25)
	0x15, 0x00,        //   Logical Minimum (0)
	0x25, 0x01,        //   Logical Maximum (1)
	0x35, 0x00,        //   Physical Minimum (0)
	0x45, 0x01,        //   Physical Maximum (1)
	0x75, 0x01,        //   Report Size (1)
	0x95, 0x02,        //   Report Count (2)
	0x05, 0x09,        //   Usage Page (Button)
	0x09, 0x01,        //   Usage (0x01)
	0x09, 0x02,        //   Usage (0x02)
	0x81, 0x02,        //   Input (Data,Var,Abs,No Wrap,Linear,Preferred State,No Null Position)
	0x95, 0x06,        //   Report Count (6)
	0x81, 0x01,        //   Input (Const,Array,Abs,No Wrap,Linear,Preferred State,No Null Position)
	0x05, 0x0D,        //   Usage Page (Digitizer)
	0x09, 0x22,        //   Usage (Finger)
	0xA1, 0x02,        //   Collection (Logical)
	0x09, 0x42,        //     Usage (Tip Switch)
	0x15, 0x00,        //     Logical Minimum (0)
	0x25, 0x01,        //     Logical Maximum (1)
	0x75, 0x01,        //     Report Size (1)
	0x95, 0x01,        //     Report Count (1)
	0x81, 0x02,        //     Input (Data,Var,Abs,No Wrap,Linear,Preferred State,No Null Position)
	0x09, 0x32,        //     Usage (In Range)
	0x81, 0x02,        //     Input (Data,Var,Abs,No Wrap,Linear,Preferred State,No Null Position)
	0x09, 0x47,        //     Usage (0x47)
	0x81, 0x02,        //     Input (Data,Var,Abs,No Wrap,Linear,Preferred State,No Null Position)
	0x95, 0x05,        //     Report Count (5)
	0x81, 0x03,        //     Input (Const,Var,Abs,No Wrap,Linear,Preferred State,No Null Position)
	0x75, 0x08,        //     Report Size (8)
	0x09, 0x51,        //     Usage (0x51)
	0x95, 0x01,        //     Report Count (1)
	0x81, 0x02,        //     Input (Data,Var,Abs,No Wrap,Linear,Preferred State,No Null Position)
	0x05, 0x01,        //     Usage Page (Generic Desktop Ctrls)
	0x15, 0x00,        //     Logical Minimum (0)
	0x26, 0xE7, 0x0B,  //     Logical Maximum (3047)
	0x75, 0x10,        //     Report Size (16)
	0x55, 0x0D,        //     Unit Exponent (-3)
	0x65, 0x13,        //     Unit (System: English Linear, Length: Centimeter)
	0x09, 0x30,        //     Usage (X)
	0x35, 0x00,        //     Physical Minimum (0)
	0x46, 0x00, 0x00,  //     Physical Maximum (0)
	0x81, 0x02,        //     Input (Data,Var,Abs,No Wrap,Linear,Preferred State,No Null Position)
	0x09, 0x31,        //     Usage (Y)
	0x26, 0xEF, 0x07,  //     Logical Maximum (2031)
	0x46, 0x00, 0x00,  //     Physical Maximum (0)
	0x81, 0x02,        //     Input (Data,Var,Abs,No Wrap,Linear,Preferred State,No Null Position)
	0xC0,              //   End Collection
	0xA1, 0x02,        //   Collection (Logical)
	0x05, 0x0D,        //     Usage Page (Digitizer)
	0x09, 0x42,        //     Usage (Tip Switch)
	0x15, 0x00,        //     Logical Minimum (0)
	0x25, 0x01,        //     Logical Maximum (1)
	0x75, 0x01,        //     Report Size (1)
	0x95, 0x01,        //     Report Count (1)
	0x81, 0x02,        //     Input (Data,Var,Abs,No Wrap,Linear,Preferred State,No Null Position)
	0x09, 0x32,        //     Usage (In Range)
	0x81, 0x02,        //     Input (Data,Var,Abs,No Wrap,Linear,Preferred State,No Null Position)
	0x09, 0x47,        //     Usage (0x47)
	0x81, 0x02,        //     Input (Data,Var,Abs,No Wrap,Linear,Preferred State,No Null Position)
	0x95, 0x05,        //     Report Count (5)
	0x81, 0x03,        //     Input (Const,Var,Abs,No Wrap,Linear,Preferred State,No Null Position)
	0x75, 0x08,        //     Report Size (8)
	0x09, 0x51,        //     Usage (0x51)
	0x95, 0x01,        //     Report Count (1)
	0x81, 0x02,        //     Input (Data,Var,Abs,No Wrap,Linear,Preferred State,No Null Position)
	0x05, 0x01,        //     Usage Page (Generic Desktop Ctrls)
	0x15, 0x00,        //     Logical Minimum (0)
	0x26, 0xE7, 0x0B,  //     Logical Maximum (3047)
	0x75, 0x10,        //     Report Size (16)
	0x55, 0x0D,        //     Unit Exponent (-3)
	0x65, 0x13,        //     Unit (System: English Linear, Length: Centimeter)
	0x09, 0x30,        //     Usage (X)
	0x35, 0x00,        //     Physical Minimum (0)
	0x46, 0x00, 0x00,  //     Physical Maximum (0)
	0x81, 0x02,        //     Input (Data,Var,Abs,No Wrap,Linear,Preferred State,No Null Position)
	0x09, 0x31,        //     Usage (Y)
	0x26, 0xEF, 0x07,  //     Logical Maximum (2031)
	0x46, 0x00, 0x00,  //     Physical Maximum (0)
	0x81, 0x02,        //     Input (Data,Var,Abs,No Wrap,Linear,Preferred State,No Null Position)
	0xC0,              //   End Collection
	0xA1, 0x02,        //   Collection (Logical)
	0x05, 0x0D,        //     Usage Page (Digitizer)
	0x09, 0x42,        //     Usage (Tip Switch)
	0x15, 0x00,        //     Logical Minimum (0)
	0x25, 0x01,        //     Logical Maximum (1)
	0x75, 0x01,        //     Report Size (1)
	0x95, 0x01,        //     Report Count (1)
	0x81, 0x02,        //     Input (Data,Var,Abs,No Wrap,Linear,Preferred State,No Null Position)
	0x09, 0x32,        //     Usage (In Range)
	0x81, 0x02,        //     Input (Data,Var,Abs,No Wrap,Linear,Preferred State,No Null Position)
	0x09, 0x47,        //     Usage (0x47)
	0x81, 0x02,        //     Input (Data,Var,Abs,No Wrap,Linear,Preferred State,No Null Position)
	0x95, 0x05,        //     Report Count (5)
	0x81, 0x03,        //     Input (Const,Var,Abs,No Wrap,Linear,Preferred State,No Null Position)
	0x75, 0x08,        //     Report Size (8)
	0x09, 0x51,        //     Usage (0x51)
	0x95, 0x01,        //     Report Count (1)
	0x81, 0x02,        //     Input (Data,Var,Abs,No Wrap,Linear,Preferred State,No Null Position)
	0x05, 0x01,        //     Usage Page (Generic Desktop Ctrls)
	0x15, 0x00,        //     Logical Minimum (0)
	0x26, 0xE7, 0x0B,  //     Logical Maximum (3047)
	0x75, 0x10,        //     Report Size (16)
	0x55, 0x0D,        //     Unit Exponent (-3)
	0x65, 0x13,        //     Unit (System: English Linear, Length: Centimeter)
	0x09, 0x30,        //     Usage (X)
	0x35, 0x00,        //     Physical Minimum (0)
	0x46, 0x00, 0x00,  //     Physical Maximum (0)
	0x81, 0x02,        //     Input (Data,Var,Abs,No Wrap,Linear,Preferred State,No Null Position)
	0x09, 0x31,        //     Usage (Y)
	0x26, 0xEF, 0x07,  //     Logical Maximum (2031)
	0x46, 0x00, 0x00,  //     Physical Maximum (0)
	0x81, 0x02,        //     Input (Data,Var,Abs,No Wrap,Linear,Preferred State,No Null Position)
	0xC0,              //   End Collection
	0x05, 0x0D,        //   Usage Page (Digitizer)
	0x09, 0x54,        //   Usage (0x54)
	0x95, 0x01,        //   Report Count (1)
	0x75, 0x08,        //   Report Size (8)
	0x15, 0x00,        //   Logical Minimum (0)
	0x25, 0x08,        //   Logical Maximum (8)
	0x81, 0x02,        //   Input (Data,Var,Abs,No Wrap,Linear,Preferred State,No Null Position)
	0x09, 0x55,        //   Usage (0x55)
	0xB1, 0x02,        //   Feature (Data,Var,Abs,No Wrap,Linear,Preferred State,No Null Position,Non-volatile)
	0x06, 0x00, 0xFF,  //   Usage Page (Vendor Defined 0xFF00)
	0x09, 0xC5,        //   Usage (0xC5)
	0x15, 0x00,        //   Logical Minimum (0)
	0x26, 0xFF, 0x00,  //   Logical Maximum (255)
	0x75, 0x08,        //   Report Size (8)
	0x96, 0x00, 0x01,  //   Report Count (256)
	0xB1, 0x02,        //   Feature (Data,Var,Abs,No Wrap,Linear,Preferred State,No Null Position,Non-volatile)
	0xC0,              // End Collection
};

static int nanosic_wn8030_hid_parse(struct hid_device *hid)
{
	if (hid->product == 0x00A3)
		return hid_parse_report(hid, hid_keyboard_descriptor, sizeof(hid_keyboard_descriptor));
	else if (hid->product == 0x00A1)
		return hid_parse_report(hid, hid_touchpad_descriptor, sizeof(hid_touchpad_descriptor));
	else
		return -ENODEV;
}

static int nanosic_wn8030_hid_start(struct hid_device *hid)
{
	return 0;
}

static void nanosic_wn8030_hid_stop(struct hid_device *hid)
{
	hid->claimed = 0;
}

static int nanosic_wn8030_hid_open(struct hid_device *hid)
{
	return 0;
}

static void nanosic_wn8030_hid_close(struct hid_device *hid)
{
}

static int nanosic_wn8030_raw_request(struct hid_device *hid,
				      unsigned char reportnum,
				      __u8 *buf, size_t len,
				      unsigned char rtype, int reqtype)
{
	return len;
}

static inline u32 nanosic_wn8030_checksum32(const u8 *data, int size)
{
	u32 checksum = 0;

	for (u32 i = 0; i < size; i += 4)
		checksum += data[i] + (data[i+1] << 8) + (data[i+2] << 16) + (data[i+3] << 24);

	return checksum;
}

static inline u8 nanosic_wn8030_checksum8(const u8 *data, int size)
{
	u8 checksum = 0;

	for (u32 i = 0; i < size; i++)
		checksum += data[i];

	return checksum;
}

static int nanosic_wn8030_set_caps_led(struct nanosic_wn8030 *nanosic, bool enable)
{
	u8 buf[XM_WN8030_I2C_WRITE] = { 0x32, 0x00, 0x4E, 0x31,
					0x80, 0x38, 0x2E, 0x01 };

	buf[8] = enable ? 0xFD : 0xFC;
	buf[9] = nanosic_wn8030_checksum8(&buf[2], 7);

	return regmap_bulk_write(nanosic->regmap, 0x5c, buf, sizeof(buf));
}

static int nanosic_wn8030_set_touchpad(struct nanosic_wn8030 *nanosic, bool enable)
{
	u8 buf[XM_WN8030_I2C_WRITE] = { 0x32, 0x00, 0x4E, 0x31,
					0x80, 0x38, 0x21, 0x01 };

	buf[8] = enable;
	buf[9] = nanosic_wn8030_checksum8(&buf[2], 7);

	return regmap_bulk_write(nanosic->regmap, 0x5c, buf, sizeof(buf));
}

static int nanosic_wn8030_set_kb_power(struct nanosic_wn8030 *nanosic, bool enable)
{
	u8 buf[XM_WN8030_I2C_WRITE] = { 0x32, 0x00, 0x4E, 0x31,
					0x80, 0x38, 0x25, 0x01 };

	buf[8] = enable;
	buf[9] = nanosic_wn8030_checksum8(&buf[2], 7);

	return regmap_bulk_write(nanosic->regmap, 0x5c, buf, sizeof(buf));
}

static int nanosic_wn8030_xm_auth_init(struct nanosic_wn8030 *nanosic)
{
	u8 buf[XM_WN8030_I2C_WRITE] = { 0x32, 0x00, 0x4F, 0x31,
					0x80, 0x38, 0x31, 0x06,
					0x4d, 0x49, 0x41, 0x55, 0x54, 0x48,
					0x00 };

	buf[14] = nanosic_wn8030_checksum8(&buf[2], 13);

	return regmap_bulk_write(nanosic->regmap, 0x5c, buf, sizeof(buf));
}

static int nanosic_wn8030_xm_auth_s3t1(struct nanosic_wn8030 *nanosic, u8 *key_meta, u8 *challenge)
{
	u8 buf[XM_WN8030_I2C_WRITE] = { 0x32, 0x00, 0x4f, 0x31,
					0x80, 0x38, 0x32, 0x14,
					0x00, 0x00, 0x00, 0x00,
					0x00, 0x00, 0x00, 0x00,
					0x00, 0x00, 0x00, 0x00,
					0x00, 0x00, 0x00, 0x00,
					0x00, 0x00, 0x00, 0x00,
					0x00 };

	memcpy(&buf[8], key_meta, 4);
	memcpy(&buf[12], challenge, 16);
	buf[28] = nanosic_wn8030_checksum8(&buf[2], 26);

	return regmap_bulk_write(nanosic->regmap, 0x5c, buf, sizeof(buf));
}

static int nanosic_wn8030_xm_auth_s5t1(struct nanosic_wn8030 *nanosic, u8 *token)
{
	u8 buf[XM_WN8030_I2C_WRITE] = { 0x32, 0x00, 0x4f, 0x31,
					0x80, 0x38, 0x33, 0x10,
					0x00, 0x00, 0x00, 0x00,
					0x00, 0x00, 0x00, 0x00,
					0x00, 0x00, 0x00, 0x00,
					0x00, 0x00, 0x00, 0x00,
					0x00 };

	memcpy(&buf[8], token, 16);
	buf[24] = nanosic_wn8030_checksum8(&buf[2], 22);

	return regmap_bulk_write(nanosic->regmap, 0x5c, buf, sizeof(buf));
}

static int nanosic_wn8030_output_report(struct hid_device *hid, u8 *buf, size_t count)
{
	struct nanosic_wn8030 *nanosic = hid->driver_data;

	//print_hex_dump(KERN_INFO, "output_report: ", DUMP_PREFIX_NONE, 16, 1, buf, count, false);

	if (!count)
		return -EINVAL;

	if (buf[0] == 0x5)
		nanosic_wn8030_set_caps_led(nanosic, buf[1]);

	return count;
}

static struct hid_ll_driver nanosic_wn8030_hid_ll_driver = {
	.parse = nanosic_wn8030_hid_parse,
	.start = nanosic_wn8030_hid_start,
	.stop = nanosic_wn8030_hid_stop,
	.open = nanosic_wn8030_hid_open,
	.close = nanosic_wn8030_hid_close,
	.raw_request = nanosic_wn8030_raw_request,
	.output_report = nanosic_wn8030_output_report,
};

static void nanosic_wn8030_add_keyboard_hid(struct nanosic_wn8030 *nanosic)
{
	struct hid_device *hid;

	hid = hid_allocate_device();
	if (IS_ERR(hid)) {
		dev_err(nanosic->dev, "failed allocate keyboard hid device");
		nanosic->hid_keyboard = NULL;
		return;
	}

	hid->driver_data = nanosic;
	hid->ll_driver = &nanosic_wn8030_hid_ll_driver;
	hid->bus = BUS_I2C;
	hid->dev.parent = nanosic->dev;
	hid->vendor = 0x15D9;
	hid->product = 0x00A3;
	sprintf(hid->name, "%s", "Xiaomi Keyboard");

	if (hid_add_device(hid)) {
		dev_err(nanosic->dev, "failed add keyboard hid device");
		hid_destroy_device(hid);
		nanosic->hid_keyboard = NULL;
		return;
	}

	nanosic->hid_keyboard = hid;
}

static void nanosic_wn8030_add_touchpad_hid(struct nanosic_wn8030 *nanosic)
{
	struct hid_device *hid;

	hid = hid_allocate_device();
	if (IS_ERR(hid)) {
		dev_err(nanosic->dev, "failed allocate touchpad hid device");
		nanosic->hid_touchpad = NULL;
		return;
	}

	hid->driver_data = nanosic;
	hid->ll_driver = &nanosic_wn8030_hid_ll_driver;
	hid->bus = BUS_I2C;
	hid->dev.parent = nanosic->dev;
	hid->vendor = 0x15D9;
	hid->product = 0x00A1;
	sprintf(hid->name, "%s", "Xiaomi Touchpad");

	if (hid_add_device(hid)) {
		dev_err(nanosic->dev, "failed add touchpad hid device");
		hid_destroy_device(hid);
		nanosic->hid_touchpad = NULL;
		return;
	}

	nanosic->hid_touchpad = hid;
}

static void nanosic_wn8030_handle_vendor(struct nanosic_wn8030 *nanosic, u8 *buf)
{
	/* WN8012(KB) -> HOST, kb detect/attach state info */
	if (buf[5] == 0x38 && buf[6] == 0x80 && buf[7] == 0xa2) {
		mutex_lock(&nanosic->conn_mutex);
		if (((buf[12] & 0x3) == 0x3) && !nanosic->keyboard_attached) {
			/* Caps LED state saved between reconnects by WN8030 */
			nanosic_wn8030_set_caps_led(nanosic, false);
			nanosic_wn8030_set_touchpad(nanosic, true);
			nanosic_wn8030_add_keyboard_hid(nanosic);
			nanosic_wn8030_add_touchpad_hid(nanosic);
			nanosic->keyboard_attached = true;
			schedule_delayed_work(&nanosic->wake_worker, msecs_to_jiffies(12000));
		} else if (((buf[12] & 0x3) == 0x0) && nanosic->keyboard_attached) {
			cancel_delayed_work_sync(&nanosic->wake_worker);
			if (nanosic->hid_keyboard) {
				hid_destroy_device(nanosic->hid_keyboard);
				nanosic->hid_keyboard = NULL;
			}
			if (nanosic->hid_touchpad) {
				hid_destroy_device(nanosic->hid_touchpad);
				nanosic->hid_touchpad = NULL;
			}
			nanosic->keyboard_attached = false;
		}
		mutex_unlock(&nanosic->conn_mutex);
	}
	/* WN8012(KB) -> HOST, kb auth */
	/* 0x0/0x1 - init auth, 0x64 - repeat request auth */
	else if (buf[5] == 0x38 && buf[6] == 0x80 && buf[7] == 0x24) {
		if (nanosic->auth_open)
			nanosic_wn8030_xm_auth_init(nanosic);
	}
	/* WN8012(KB) -> HOST, kb auth init */
	else if (buf[5] == 0x38 && buf[6] == 0x80 && buf[7] == 0x31) {
		/* Use offline auth */
		u8 key_meta[4] = { 0x00, 0x00, 0x00, 0x02 };
		u8 challenge[16] = { 0x80, 0x3d, 0x84, 0x36,
				     0x29, 0x82, 0xde, 0x6a,
				     0x0e, 0x68, 0x1d, 0x3d,
				     0x6b, 0x32, 0xa5, 0xa6 };

		reinit_completion(&nanosic->auth_token_ready);
		memcpy(nanosic->auth_uid, &buf[11], 16);

		nanosic_wn8030_xm_auth_s3t1(nanosic, key_meta, challenge);
	}
	/* WN8012(KB) -> HOST, kb auth s3t1 */
	else if (buf[5] == 0x38 && buf[6] == 0x80 && buf[7] == 0x32) {
		memcpy(nanosic->auth_challenge, &buf[25], 16);
		nanosic->auth_request_pending = true;
		wake_up_interruptible(&nanosic->auth_read_wq);
		if (wait_for_completion_interruptible_timeout(&nanosic->auth_token_ready,
							      msecs_to_jiffies(5000)) <= 0) {
			dev_err(nanosic->dev, "timeout waiting for keyboard auth token\n");
		} else {
			nanosic_wn8030_xm_auth_s5t1(nanosic, nanosic->auth_token);
		}
	}
}

static irqreturn_t nanosic_wn8030_handler(int irq, void *data)
{
	int ret;
	struct nanosic_wn8030 *nanosic = data;
	u8 buf[XM_WN8030_I2C_READ];

	/* After sleep pin deactivated, fw will trigger irq to notify about
	 * successful resume, second irq will tell about kb connection state
	 */
	if (nanosic->suspended) {
		nanosic->suspended = false;
		return IRQ_HANDLED;
	}

	ret = regmap_bulk_read(nanosic->regmap, 0x4c, buf, sizeof(buf));
	if (ret) {
		dev_warn(nanosic->dev, "failed to read data on irq %d\n", ret);
		return IRQ_HANDLED;
	}

	/* Bad data */
	if (buf[0] != 0x57 || buf[2] == 0x0)
		return IRQ_HANDLED;

	switch (buf[3]) {
	case 0x5:
		if (nanosic->hid_keyboard)
			hid_input_report(nanosic->hid_keyboard, HID_INPUT_REPORT,
					 &buf[3], 9, 0);
		break;
	case 0x19:
		if (nanosic->hid_touchpad)
			hid_input_report(nanosic->hid_touchpad, HID_INPUT_REPORT,
					 &buf[3], 21, 0);
		break;
	case 0x22:
	case 0x23:
	case 0x24:
		nanosic_wn8030_handle_vendor(nanosic, buf);
		break;
	}

	//print_hex_dump(KERN_INFO, "", DUMP_PREFIX_OFFSET, 16, 1, buf, sizeof(buf), false);

	return IRQ_HANDLED;
}

static int nanosic_wn8030_check_boot_id(struct nanosic_wn8030 *nanosic)
{
	int ret;
	unsigned int boot_id;

	ret = regmap_write(nanosic->regmap, 0x5c, 0x04);
	if (ret) {
		dev_err(nanosic->dev, "failed to set read address\n");
		return ret;
	}

	ret = regmap_read(nanosic->regmap, 0x5c, &boot_id);
	if (ret) {
		dev_err(nanosic->dev, "failed to read bootloader id\n");
		return ret;
	}

	if (boot_id != 0xc8) {
		dev_err(nanosic->dev, "unexpected bootloader id 0x%x\n", boot_id);
		return -EINVAL;
	}

	return 0;
}

static int nanosic_wn8030_get_boot_state(struct nanosic_wn8030 *nanosic)
{
	int ret;
	unsigned int boot_state;

	ret = regmap_write(nanosic->regmap, 0x5c, 0x05);
	if (ret) {
		dev_err(nanosic->dev, "failed to set read address\n");
		return ret;
	}

	ret = regmap_read(nanosic->regmap, 0x5c, &boot_state);
	if (ret) {
		dev_err(nanosic->dev, "failed to read bootloader state\n");
		return ret;
	}

	return boot_state;
}

static int nanosic_wn8030_send_header(struct nanosic_wn8030 *nanosic, const u8 *fw_buf, u32 *ram_code_size)
{
	int ret;
	struct wn8030_header_cmd hdr_cmd = {0};
	struct wn8030_bin_header *bin_hdr = (struct wn8030_bin_header *)&fw_buf[WN8030_HEADER_ADDR];

	hdr_cmd.cmd = 0x06; /* header send cmd */
	memcpy(hdr_cmd.flag, bin_hdr->flag, sizeof(hdr_cmd.flag));
	hdr_cmd.chip_id = 0x8140;
	hdr_cmd.ram_code_source = 0x2345;
	hdr_cmd.ram_code_dest = 0x40;
	hdr_cmd.ram_code_size = bin_hdr->ram_code_size;
	*ram_code_size = bin_hdr->ram_code_size;
	hdr_cmd.code_entry = bin_hdr->code_entry;
	hdr_cmd.code_entry = bin_hdr->code_entry;
	hdr_cmd.pid = bin_hdr->pid;
	hdr_cmd.vid = bin_hdr->vid;
	hdr_cmd.major_version = bin_hdr->major_version;
	hdr_cmd.minor_version = bin_hdr->minor_version;
	hdr_cmd.image_size = 0;
	hdr_cmd.ram_code_checksum = nanosic_wn8030_checksum32(&fw_buf[WN8030_CODE_ADDR], hdr_cmd.ram_code_size);
	hdr_cmd.image_checksum = 0;
	hdr_cmd.header_checksum = nanosic_wn8030_checksum8((u8 *)&hdr_cmd + 1, 64);

	ret = regmap_bulk_write(nanosic->regmap, 0x5c, &hdr_cmd, sizeof(hdr_cmd));
	if (ret) {
		dev_err(nanosic->dev, "failed to send header\n");
		return ret;
	}

	ret = nanosic_wn8030_get_boot_state(nanosic);
	if (ret < 0)
		return ret;

	if (ret != 0x1) {
		dev_err(nanosic->dev, "unexpected bootloader state after sending header %d\n", ret);
		return -EINVAL;
	}

	return 0;
}

static int nanosic_wn8030_send_data(struct nanosic_wn8030 *nanosic, const u8 *fw_buf, u32 ram_code_size)
{
	int ret;
	struct wn8030_send_cmd send_cmd;
	unsigned int offset = 0;

	while (ram_code_size > offset) {
		unsigned int send_size;

		memset(&send_cmd, 0, sizeof(send_cmd));
		send_cmd.cmd = 0x07;
		send_cmd.ram_dest = cpu_to_be32(offset);

		if (ram_code_size - offset < 503)
			send_size = ram_code_size - offset;
		else
			send_size = 503;

		send_cmd.size = cpu_to_be32(send_size);
		memcpy(send_cmd.data, &fw_buf[WN8030_CODE_ADDR+offset], send_size);

		ret = regmap_bulk_write(nanosic->regmap, 0x5c, &send_cmd, sizeof(send_cmd));
		if (ret) {
			dev_err(nanosic->dev, "failed to send data to 0x%x\n", offset);
			return ret;
		}
		offset += send_size;
	}

	return 0;
}

static int nanosic_wn8030_load_fw(struct nanosic_wn8030 *nanosic)
{
	int ret;
	const struct firmware *firmware;
	unsigned int ram_code_size;

	ret = request_firmware(&firmware, "nanosic/MCU_Upgrade.bin", nanosic->dev);
	if (ret) {
		dev_err(nanosic->dev, "failed to load MCU firmware\n");
		return ret;
	}

	if (firmware->size < 0xA000) {
		dev_err(nanosic->dev, "invalid firmware size\n");
		ret = -EINVAL;
		goto exit;
	}

	if (strncmp(&firmware->data[0x7FC0], "NANO IC", 7) != 0) {
		dev_err(nanosic->dev, "invalid firmware header\n");
		ret = -EINVAL;
		goto exit;
	}

	ret = nanosic_wn8030_send_header(nanosic, firmware->data, &ram_code_size);
	if (ret)
		goto exit;

	ret = nanosic_wn8030_send_data(nanosic, firmware->data, ram_code_size);
	if (ret)
		goto exit;

	ret = regmap_write(nanosic->regmap, 0x5c, 0x08);
	if (ret) {
		dev_err(nanosic->dev, "failed to start firmware verification\n");
		goto exit;
	}

	msleep(10);

	ret = nanosic_wn8030_get_boot_state(nanosic);
	if (ret < 0)
		goto exit;

	if (ret != 0x3) {
		dev_err(nanosic->dev, "bootloader failed to verify firmware %d\n", ret);
		ret = -EINVAL;
		goto exit;
	}

	ret = regmap_write(nanosic->regmap, 0x5c, 0x09);
	if (ret) {
		dev_err(nanosic->dev, "failed to boot firmware\n");
		goto exit;
	}

	msleep(50);

exit:
	release_firmware(firmware);

	return ret;
}

static int nanosic_wn8030_power_on(struct nanosic_wn8030 *nanosic)
{
	int ret;

	ret = regulator_bulk_enable(ARRAY_SIZE(nanosic_wn8030_supply_names),
				    nanosic->supplies);
	if (ret) {
		dev_err(nanosic->dev, "failed to enable regulators\n");
		return ret;
	}

	gpiod_set_value_cansleep(nanosic->reset_gpio, 0);
	msleep(20); /* On Pad 6S Pro we only wait for WN8030 bootrom start */
	gpiod_set_value_cansleep(nanosic->sleep_gpio, 0);

	return 0;
}

static void nanosic_wn8030_power_off(struct nanosic_wn8030 *nanosic)
{
	gpiod_set_value_cansleep(nanosic->sleep_gpio, 1);
	gpiod_set_value_cansleep(nanosic->reset_gpio, 1);
	msleep(10);
	regulator_bulk_disable(ARRAY_SIZE(nanosic_wn8030_supply_names), nanosic->supplies);
}

static void nanosic_wn8030_wake_worker(struct work_struct *data)
{
	struct nanosic_wn8030 *nanosic =
		container_of(data, struct nanosic_wn8030, wake_worker.work);

	nanosic_wn8030_set_kb_power(nanosic, true);
	schedule_delayed_work(&nanosic->wake_worker, msecs_to_jiffies(12000));
}

struct nanosic_auth_req {
	u8 uid[16];
	u8 challenge[16];
};

static int nanosic_auth_open(struct inode *inode, struct file *file)
{
	struct nanosic_wn8030 *nanosic =
		container_of(file->private_data, struct nanosic_wn8030, auth_misc);

	if (nanosic->auth_open)
		return -EBUSY;
	nanosic->auth_open = true;
	file->private_data = nanosic;
	return 0;
}

static int nanosic_auth_release(struct inode *inode, struct file *file)
{
	struct nanosic_wn8030 *nanosic = file->private_data;

	nanosic->auth_open = false;
	return 0;
}

static ssize_t nanosic_auth_read(struct file *file, char __user *buf,
				 size_t count, loff_t *off)
{
	struct nanosic_wn8030 *nanosic = file->private_data;
	struct nanosic_auth_req req;
	int ret;

	if (count != sizeof(req))
		return -EINVAL;

	ret = wait_event_interruptible(nanosic->auth_read_wq,
				       nanosic->auth_request_pending);
	if (ret)
		return ret;

	memcpy(req.uid, nanosic->auth_uid, sizeof(req.uid));
	memcpy(req.challenge, nanosic->auth_challenge, sizeof(req.challenge));
	nanosic->auth_request_pending = false;

	if (copy_to_user(buf, &req, sizeof(req)))
		return -EFAULT;

	return sizeof(req);
}

static ssize_t nanosic_auth_write(struct file *file, const char __user *buf,
				  size_t count, loff_t *off)
{
	struct nanosic_wn8030 *nanosic = file->private_data;
	u8 token[16];

	if (count != sizeof(token))
		return -EINVAL;

	if (copy_from_user(token, buf, sizeof(token)))
		return -EFAULT;

	memcpy(nanosic->auth_token, token, sizeof(token));
	complete(&nanosic->auth_token_ready);

	return sizeof(token);
}

static const struct file_operations nanosic_auth_fops = {
	.owner = THIS_MODULE,
	.open = nanosic_auth_open,
	.release = nanosic_auth_release,
	.read = nanosic_auth_read,
	.write = nanosic_auth_write,
};

static int nanosic_wn8030_probe(struct i2c_client *client)
{
	struct nanosic_wn8030 *nanosic;
	int ret;

	nanosic = devm_kzalloc(&client->dev, sizeof(*nanosic), GFP_KERNEL);
	if (!nanosic)
		return -ENOMEM;

	nanosic->dev = &client->dev;
	nanosic->client = client;

	nanosic->reset_gpio = devm_gpiod_get(nanosic->dev, "reset", GPIOD_OUT_LOW);
	if (IS_ERR(nanosic->reset_gpio))
		return dev_err_probe(nanosic->dev, PTR_ERR(nanosic->reset_gpio),
				     "failed to get reset gpio\n");

	nanosic->status_gpio = devm_gpiod_get(nanosic->dev, "status", GPIOD_IN);
	if (IS_ERR(nanosic->status_gpio))
		return dev_err_probe(nanosic->dev, PTR_ERR(nanosic->status_gpio),
				     "failed to get status gpio\n");

	nanosic->sleep_gpio = devm_gpiod_get(nanosic->dev, "sleep", GPIOD_OUT_LOW);
	if (IS_ERR(nanosic->sleep_gpio))
		return dev_err_probe(nanosic->dev, PTR_ERR(nanosic->sleep_gpio),
				     "failed to get sleep gpio\n");

	for (u32 i = 0; i < ARRAY_SIZE(nanosic_wn8030_supply_names); i++)
		nanosic->supplies[i].supply = nanosic_wn8030_supply_names[i];

	ret = devm_regulator_bulk_get(nanosic->dev, ARRAY_SIZE(nanosic_wn8030_supply_names),
				      nanosic->supplies);
	if (ret)
		return dev_err_probe(nanosic->dev, ret, "failed to get regulators\n");

	ret = nanosic_wn8030_power_on(nanosic);
	if (ret)
		return ret;

	nanosic->regmap = devm_regmap_init_i2c(client, &nanosic_wn8030_regmap_config);
	if (IS_ERR(nanosic->regmap)) {
		ret = dev_err_probe(nanosic->dev, ret, "failed to init regmap\n");
		goto err;
	}

	mutex_init(&nanosic->conn_mutex);

	dev_set_drvdata(nanosic->dev, nanosic);
	i2c_set_clientdata(client, nanosic);

	INIT_DELAYED_WORK(&nanosic->wake_worker, nanosic_wn8030_wake_worker);

	init_waitqueue_head(&nanosic->auth_read_wq);
	init_completion(&nanosic->auth_token_ready);

	nanosic->auth_open = false;
	nanosic->auth_misc.minor = MISC_DYNAMIC_MINOR;
	nanosic->auth_misc.name = "nanosic_auth";
	nanosic->auth_misc.fops = &nanosic_auth_fops;
	nanosic->auth_misc.parent = nanosic->dev;
	ret = misc_register(&nanosic->auth_misc);
	if (ret) {
		dev_err(nanosic->dev, "failed to register misc device\n");
		goto err;
	}

	ret = nanosic_wn8030_check_boot_id(nanosic);
	if (ret)
		goto err_misc;

	ret = nanosic_wn8030_get_boot_state(nanosic);
	if (ret < 0)
		goto err_misc;

	if (ret) {
		ret = dev_err_probe(nanosic->dev, -EINVAL, "unexpected initial bootloader state %d\n", ret);
		goto err_misc;
	}

	ret = nanosic_wn8030_load_fw(nanosic);
	if (ret)
		goto err_misc;

	ret = devm_request_threaded_irq(&client->dev, client->irq,
					NULL, nanosic_wn8030_handler,
					IRQF_ONESHOT | IRQF_TRIGGER_RISING,
					"nanosic_wn8030_irq", nanosic);
	if (ret) {
		ret = dev_err_probe(nanosic->dev, ret, "failed to request irq %d\n", client->irq);
		goto err_misc;
	}

	return 0;

err_misc:
	misc_deregister(&nanosic->auth_misc);
err:
	nanosic_wn8030_power_off(nanosic);

	return ret;
}

static void nanosic_wn8030_remove(struct i2c_client *client)
{
	struct nanosic_wn8030 *nanosic = i2c_get_clientdata(client);

	disable_irq(nanosic->client->irq);
	cancel_delayed_work_sync(&nanosic->wake_worker);

	misc_deregister(&nanosic->auth_misc);

	if (nanosic->hid_keyboard)
		hid_destroy_device(nanosic->hid_keyboard);
	if (nanosic->hid_touchpad)
		hid_destroy_device(nanosic->hid_touchpad);
	nanosic_wn8030_power_off(nanosic);
}

static int nanosic_wn8030_resume(struct device *dev)
{
	struct nanosic_wn8030 *nanosic = dev_get_drvdata(dev);

	enable_irq(nanosic->client->irq);

	gpiod_set_value_cansleep(nanosic->sleep_gpio, 0);
	msleep(25);

	if (nanosic->suspended) {
		dev_warn(dev, "timeout waiting for chip resume. Reseting chip...\n");
		gpiod_set_value_cansleep(nanosic->reset_gpio, 1);
		msleep(10);
		gpiod_set_value_cansleep(nanosic->reset_gpio, 0);
		msleep(20);
		nanosic->suspended = false;
		return nanosic_wn8030_load_fw(nanosic);
	}

	/* Wake up keyboard after MCU exits sleep */
	nanosic_wn8030_set_kb_power(nanosic, true);

	return 0;
}

static int nanosic_wn8030_suspend(struct device *dev)
{
	struct nanosic_wn8030 *nanosic = dev_get_drvdata(dev);

	disable_irq(nanosic->client->irq);
	cancel_delayed_work_sync(&nanosic->wake_worker);

	gpiod_set_value_cansleep(nanosic->sleep_gpio, 1);
	msleep(10);
	nanosic->suspended = true;

	return 0;
}

static DEFINE_SIMPLE_DEV_PM_OPS(nanosic_wn8030_pm_ops, nanosic_wn8030_suspend, nanosic_wn8030_resume);

static const struct of_device_id __maybe_unused nanosic_wn8030_of_match[] = {
	{ .compatible = "nanosic,wn8030-sheng", },
	{ },
};
MODULE_DEVICE_TABLE(of, nanosic_wn8030_of_match);

static struct i2c_driver nanosic_wn8030_driver = {
	.driver	= {
		.name = "nanosic_wn8030",
		.of_match_table = of_match_ptr(nanosic_wn8030_of_match),
		.pm = pm_sleep_ptr(&nanosic_wn8030_pm_ops),
	},
	.probe = nanosic_wn8030_probe,
	.remove = nanosic_wn8030_remove,
};

module_i2c_driver(nanosic_wn8030_driver);

MODULE_AUTHOR("map220v <map220v300@gmail.com>");
MODULE_DESCRIPTION("HID driver for Nanosic WN8030 keyboard MCU");
MODULE_LICENSE("GPL v2");
