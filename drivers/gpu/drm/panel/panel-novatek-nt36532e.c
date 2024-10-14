// SPDX-License-Identifier: GPL-2.0-only
/*
 * Novatek NT36532E DriverIC panels driver
 *
 * Copyright (c) 2024 map220v <map220v300@gmail.com>
 */

#include <linux/backlight.h>
#include <linux/delay.h>
#include <linux/gpio/consumer.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_graph.h>
#include <linux/regulator/consumer.h>

#include <video/mipi_display.h>

#include <drm/display/drm_dsc.h>
#include <drm/display/drm_dsc_helper.h>
#include <drm/drm_connector.h>
#include <drm/drm_crtc.h>
#include <drm/drm_mipi_dsi.h>
#include <drm/drm_modes.h>
#include <drm/drm_panel.h>

#define NT36532E_DSC

#define DSI_NUM_MIN 1

struct panel_info {
	struct drm_panel panel;
	struct mipi_dsi_device *dsi[2];
	struct panel_desc *desc;
	enum drm_panel_orientation orientation;

	struct gpio_desc *reset_gpio;
	struct regulator *vddio;
};

struct panel_desc {
	unsigned int width_mm;
	unsigned int height_mm;

	unsigned int bpc;
	unsigned int lanes;
	unsigned long mode_flags;
	enum mipi_dsi_pixel_format format;

	const struct drm_display_mode *modes;
	unsigned int num_modes;
	const struct mipi_dsi_device_info dsi_info;
	int (*init_sequence)(struct panel_info *pinfo);

	bool is_dual_dsi;

	struct drm_dsc_config dsc;
};

static inline struct panel_info *to_panel_info(struct drm_panel *panel)
{
	return container_of(panel, struct panel_info, panel);
}

static int sheng_tianma_init_sequence(struct panel_info *pinfo)
{
	struct mipi_dsi_device *dsi0 = pinfo->dsi[0];
	struct device *dev = &dsi0->dev;
	int ret;

	/* NT36532E accepts commands only on DSI0 */

	/* Timings Page 3 Cmd Set */
	mipi_dsi_dcs_write_seq(dsi0, 0xff, 0x26);
	mipi_dsi_dcs_write_seq(dsi0, 0xfb, 0x01);
	mipi_dsi_dcs_write_seq(dsi0, 0xcd, 0x3a, 0x46);
	mipi_dsi_dcs_write_seq(dsi0, 0xce, 0x39, 0x46);

	/* Timings Page 2 Cmd Set */
	mipi_dsi_dcs_write_seq(dsi0, 0xff, 0x25);
	mipi_dsi_dcs_write_seq(dsi0, 0xfb, 0x01);
	mipi_dsi_dcs_write_seq(dsi0, 0x05, 0x00);

	/* Timings Page 4 Cmd Set */
	mipi_dsi_dcs_write_seq(dsi0, 0xff, 0x27);
	mipi_dsi_dcs_write_seq(dsi0, 0xfb, 0x01);
	mipi_dsi_dcs_write_seq(dsi0, 0xd0, 0x31);
	mipi_dsi_dcs_write_seq(dsi0, 0xd1, 0x20);
	mipi_dsi_dcs_write_seq(dsi0, 0xd2, 0x38);
	mipi_dsi_dcs_write_seq(dsi0, 0xde, 0x43);
	mipi_dsi_dcs_write_seq(dsi0, 0xdf, 0x02);

	mipi_dsi_dcs_write_seq(dsi0, 0x13, 0x00);
	mipi_dsi_dcs_write_seq(dsi0, 0x14, 0x11);

	/* CABC Cmd Set */
	mipi_dsi_dcs_write_seq(dsi0, 0xff, 0x23);
	mipi_dsi_dcs_write_seq(dsi0, 0xfb, 0x01);
	mipi_dsi_dcs_write_seq(dsi0, 0x00, 0x80);
	mipi_dsi_dcs_write_seq(dsi0, 0x01, 0x84);
	mipi_dsi_dcs_write_seq(dsi0, 0x05, 0xf6);
	mipi_dsi_dcs_write_seq(dsi0, 0x06, 0x02);
	mipi_dsi_dcs_write_seq(dsi0, 0x11, 0x03);
	mipi_dsi_dcs_write_seq(dsi0, 0x12, 0xc7);
	mipi_dsi_dcs_write_seq(dsi0, 0x15, 0xae);
	mipi_dsi_dcs_write_seq(dsi0, 0x16, 0x16);
	/* CABC: UI mode */
	mipi_dsi_dcs_write_seq(dsi0, 0x29, 0x0a);
	mipi_dsi_dcs_write_seq(dsi0, 0x30, 0xff);
	mipi_dsi_dcs_write_seq(dsi0, 0x31, 0xfe);
	mipi_dsi_dcs_write_seq(dsi0, 0x32, 0xfd);
	mipi_dsi_dcs_write_seq(dsi0, 0x33, 0xfb);
	mipi_dsi_dcs_write_seq(dsi0, 0x34, 0xf8);
	mipi_dsi_dcs_write_seq(dsi0, 0x35, 0xf5);
	mipi_dsi_dcs_write_seq(dsi0, 0x36, 0xf3);
	mipi_dsi_dcs_write_seq(dsi0, 0x37, 0xf2);
	mipi_dsi_dcs_write_seq(dsi0, 0x38, 0xf2);
	mipi_dsi_dcs_write_seq(dsi0, 0x39, 0xf2);
	mipi_dsi_dcs_write_seq(dsi0, 0x3a, 0xef);
	mipi_dsi_dcs_write_seq(dsi0, 0x3b, 0xec);
	mipi_dsi_dcs_write_seq(dsi0, 0x3d, 0xe9);
	mipi_dsi_dcs_write_seq(dsi0, 0x3f, 0xe5);
	mipi_dsi_dcs_write_seq(dsi0, 0x40, 0xe5);
	mipi_dsi_dcs_write_seq(dsi0, 0x41, 0xe5);
	/* CABC: STILL mode */
	mipi_dsi_dcs_write_seq(dsi0, 0x2a, 0x13);
	mipi_dsi_dcs_write_seq(dsi0, 0x45, 0xff);
	mipi_dsi_dcs_write_seq(dsi0, 0x46, 0xf4);
	mipi_dsi_dcs_write_seq(dsi0, 0x47, 0xe7);
	mipi_dsi_dcs_write_seq(dsi0, 0x48, 0xda);
	mipi_dsi_dcs_write_seq(dsi0, 0x49, 0xcd);
	mipi_dsi_dcs_write_seq(dsi0, 0x4a, 0xc0);
	mipi_dsi_dcs_write_seq(dsi0, 0x4b, 0xb3);
	mipi_dsi_dcs_write_seq(dsi0, 0x4c, 0xb1);
	mipi_dsi_dcs_write_seq(dsi0, 0x4d, 0xb1);
	mipi_dsi_dcs_write_seq(dsi0, 0x4e, 0xb1);
	mipi_dsi_dcs_write_seq(dsi0, 0x4f, 0x95);
	mipi_dsi_dcs_write_seq(dsi0, 0x50, 0x79);
	mipi_dsi_dcs_write_seq(dsi0, 0x51, 0x5c);
	mipi_dsi_dcs_write_seq(dsi0, 0x52, 0x58);
	mipi_dsi_dcs_write_seq(dsi0, 0x53, 0x58);
	mipi_dsi_dcs_write_seq(dsi0, 0x54, 0x58);
	/* CABC: MOVING mode */
	mipi_dsi_dcs_write_seq(dsi0, 0x2b, 0x0e);
	mipi_dsi_dcs_write_seq(dsi0, 0x58, 0xff);
	mipi_dsi_dcs_write_seq(dsi0, 0x59, 0xfb);
	mipi_dsi_dcs_write_seq(dsi0, 0x5a, 0xf7);
	mipi_dsi_dcs_write_seq(dsi0, 0x5b, 0xf3);
	mipi_dsi_dcs_write_seq(dsi0, 0x5c, 0xef);
	mipi_dsi_dcs_write_seq(dsi0, 0x5d, 0xe3);
	mipi_dsi_dcs_write_seq(dsi0, 0x5e, 0xd8);
	mipi_dsi_dcs_write_seq(dsi0, 0x5f, 0xd6);
	mipi_dsi_dcs_write_seq(dsi0, 0x60, 0xd6);
	mipi_dsi_dcs_write_seq(dsi0, 0x61, 0xd6);
	mipi_dsi_dcs_write_seq(dsi0, 0x62, 0xc8);
	mipi_dsi_dcs_write_seq(dsi0, 0x63, 0xb7);
	mipi_dsi_dcs_write_seq(dsi0, 0x64, 0xaa);
	mipi_dsi_dcs_write_seq(dsi0, 0x65, 0xa8);
	mipi_dsi_dcs_write_seq(dsi0, 0x66, 0xa8);
	mipi_dsi_dcs_write_seq(dsi0, 0x67, 0xa8);

	/* IC Transfer Cmd Set */
	mipi_dsi_dcs_write_seq(dsi0, 0xff, 0xf0);
	mipi_dsi_dcs_write_seq(dsi0, 0xfb, 0x01);
	mipi_dsi_dcs_write_seq(dsi0, 0xfa, 0x05);
	mipi_dsi_dcs_write_seq(dsi0, 0x76, 0x16);

	/* User Cmd Set */
	mipi_dsi_dcs_write_seq(dsi0, 0xff, 0x10);
	mipi_dsi_dcs_write_seq(dsi0, 0xfb, 0x01);//Don't reload registers from MTP/Default values
	mipi_dsi_dcs_write_seq(dsi0, 0x35, 0x00);//Tear on
	mipi_dsi_dcs_write_seq(dsi0, 0x3b, 0x03, 0x8c, 0x1a, 0x04, 0x04, 0x00);
	mipi_dsi_dcs_write_seq(dsi0, 0x51, 0x0f, 0xff);//Set brightness(controls power output to leds or ktz8866?)
	mipi_dsi_dcs_write_seq(dsi0, 0x53, 0x24);//Enable backlight and no dimming

#ifdef NT36532E_DSC
	mipi_dsi_dcs_write_seq(dsi0, 0x90, 0x03);//Enable DSC
	mipi_dsi_dcs_write_seq(dsi0, 0x91,
			       0x89, 0x28, 0x00, 0x10, 0xd2, 0x00, 0x02, 0x9d,
			       0x01, 0xb1, 0x00, 0x0a, 0x06, 0xef, 0x04, 0x82);//Set PPS
	mipi_dsi_dcs_write_seq(dsi0, 0x92, 0x10, 0xf0);
#else
	mipi_dsi_dcs_write_seq(dsi0, 0x90, 0x00);
	mipi_dsi_dcs_write_seq(dsi0, 0x92, 0x10, 0xf0);
#endif

	mipi_dsi_dcs_write_seq(dsi0, 0x9d, 0x01);
#ifdef NT36532E_DSC
	mipi_dsi_dcs_write_seq(dsi0, 0xb2, 0x00);//Framerate ctrl
	mipi_dsi_dcs_write_seq(dsi0, 0xb3, 0x00);//Framerate ctrl2
#else
	mipi_dsi_dcs_write_seq(dsi0, 0xb2, 0x91);//Framerate ctrl
	mipi_dsi_dcs_write_seq(dsi0, 0xb3, 0x40);//Framerate ctrl2
#endif

	ret = mipi_dsi_dcs_exit_sleep_mode(dsi0);
	if (ret < 0) {
		dev_err(dev, "failed to exit sleep mode: %d\n", ret);
		return ret;
	}

	msleep(120);

	ret = mipi_dsi_dcs_set_display_on(dsi0);
	if (ret < 0) {
		dev_err(dev, "failed to set display on: %d\n", ret);
		return ret;
	}

	return 0;
}

static const struct drm_display_mode sheng_tianma_modes[] = {
#ifdef NT36532E_DSC
	{
		/* 144Hz */
		.clock = (3048 + 142 + 4 + 92) * (2032 + 26 + 2 + 138) * 144 / 1000,
		.hdisplay = 3048,
		.hsync_start = 3048 + 142,
		.hsync_end = 3048 + 142 + 4,
		.htotal = 3048 + 142 + 4 + 92,
		.vdisplay = 2032,
		.vsync_start = 2032 + 26,
		.vsync_end = 2032 + 26 + 2,
		.vtotal = 2032 + 26 + 2 + 138,
	}
#else
	{
		/* 60Hz */
		.clock = (3048 + 392 + 4 + 92) * (2032 + 26 + 2 + 138) * 60 / 1000,
		.hdisplay = 3048,
		.hsync_start = 3048 + 392,
		.hsync_end = 3048 + 392 + 4,
		.htotal = 3048 + 392 + 4 + 92,
		.vdisplay = 2032,
		.vsync_start = 2032 + 26,
		.vsync_end = 2032 + 26 + 2,
		.vtotal = 2032 + 26 + 2 + 138,
	}
#endif
};

static struct panel_desc sheng_tianma_desc = {
	.modes = sheng_tianma_modes,
	.num_modes = ARRAY_SIZE(sheng_tianma_modes),
	.dsi_info = {
		.type = "TIANMA-sheng",
		.channel = 0,
		.node = NULL,
	},
	.width_mm = 2632,
	.height_mm = 1754,
	.bpc = 8,
	.lanes = 4,
	.format = MIPI_DSI_FMT_RGB888,
	.mode_flags = MIPI_DSI_MODE_VIDEO | MIPI_DSI_MODE_VIDEO_BURST |
				MIPI_DSI_CLOCK_NON_CONTINUOUS | MIPI_DSI_MODE_LPM,
	.init_sequence = sheng_tianma_init_sequence,
	.is_dual_dsi = true,
	.dsc = {
		.dsc_version_major = 0x1,
		.dsc_version_minor = 0x1,
		.slice_height = 16,
		.slice_width = 762,
		.slice_count = 2,
		.bits_per_component = 8,
		.bits_per_pixel = 8 << 4,
		.block_pred_enable = true,
	},
};

static void nt36532e_reset(struct panel_info *pinfo)
{
	gpiod_set_value_cansleep(pinfo->reset_gpio, 1);
	usleep_range(10000, 11000);
	gpiod_set_value_cansleep(pinfo->reset_gpio, 0);
	usleep_range(3000, 4000);
	gpiod_set_value_cansleep(pinfo->reset_gpio, 1);
	usleep_range(3000, 4000);
	gpiod_set_value_cansleep(pinfo->reset_gpio, 0);
	usleep_range(15000, 16000);
}

static int nt36532e_prepare(struct drm_panel *panel)
{
	struct panel_info *pinfo = to_panel_info(panel);
	struct drm_dsc_picture_parameter_set pps;
	int ret;

	ret = regulator_enable(pinfo->vddio);
	if (ret) {
		dev_err(panel->dev, "failed to enable vddio regulator: %d\n", ret);
		return ret;
	}

	nt36532e_reset(pinfo);

	ret = pinfo->desc->init_sequence(pinfo);
	if (ret < 0) {
		regulator_disable(pinfo->vddio);
		dev_err(panel->dev, "failed to initialize panel: %d\n", ret);
		return ret;
	}

#ifdef NT36532E_DSC
	drm_dsc_pps_payload_pack(&pps, &pinfo->desc->dsc);

	ret = mipi_dsi_picture_parameter_set(pinfo->dsi[0], &pps);
	if (ret < 0) {
		dev_err(panel->dev, "failed to transmit PPS: %d\n", ret);
		return ret;
	}

	/* Not required, NT36532E has DSC always enabled. */
	ret = mipi_dsi_compression_mode(pinfo->dsi[0], true);
	if (ret < 0) {
		dev_err(panel->dev, "failed to enable compression mode: %d\n", ret);
		return ret;
	}
#endif

	return 0;
}

static int nt36532e_disable(struct drm_panel *panel)
{
	struct panel_info *pinfo = to_panel_info(panel);
	int ret;

	ret = mipi_dsi_dcs_set_display_off(pinfo->dsi[0]);
	if (ret < 0)
		dev_err(&pinfo->dsi[0]->dev, "failed to set display off: %d\n", ret);

	msleep(50);

	ret = mipi_dsi_dcs_enter_sleep_mode(pinfo->dsi[0]);
	if (ret < 0)
		dev_err(&pinfo->dsi[0]->dev, "failed to enter sleep mode: %d\n", ret);

	msleep(120);

	return 0;
}

static int nt36532e_unprepare(struct drm_panel *panel)
{
	struct panel_info *pinfo = to_panel_info(panel);

	gpiod_set_value_cansleep(pinfo->reset_gpio, 1);
	regulator_disable(pinfo->vddio);

	return 0;
}

static void nt36532e_remove(struct mipi_dsi_device *dsi)
{
	struct panel_info *pinfo = mipi_dsi_get_drvdata(dsi);
	int ret;

	ret = mipi_dsi_detach(pinfo->dsi[0]);
	if (ret < 0)
		dev_err(&dsi->dev, "failed to detach from DSI0 host: %d\n", ret);

	if (pinfo->desc->is_dual_dsi) {
		ret = mipi_dsi_detach(pinfo->dsi[1]);
		if (ret < 0)
			dev_err(&pinfo->dsi[1]->dev, "failed to detach from DSI1 host: %d\n", ret);
		mipi_dsi_device_unregister(pinfo->dsi[1]);
	}

	drm_panel_remove(&pinfo->panel);
}

static int nt36532e_get_modes(struct drm_panel *panel,
			       struct drm_connector *connector)
{
	struct panel_info *pinfo = to_panel_info(panel);
	int i;

	for (i = 0; i < pinfo->desc->num_modes; i++) {
		const struct drm_display_mode *m = &pinfo->desc->modes[i];
		struct drm_display_mode *mode;

		mode = drm_mode_duplicate(connector->dev, m);
		if (!mode) {
			dev_err(panel->dev, "failed to add mode %ux%u@%u\n",
				m->hdisplay, m->vdisplay, drm_mode_vrefresh(m));
			return -ENOMEM;
		}

		mode->type = DRM_MODE_TYPE_DRIVER;
		if (i == 0)
			mode->type |= DRM_MODE_TYPE_PREFERRED;

		drm_mode_set_name(mode);
		drm_mode_probed_add(connector, mode);
	}

	connector->display_info.width_mm = pinfo->desc->width_mm;
	connector->display_info.height_mm = pinfo->desc->height_mm;
	connector->display_info.bpc = pinfo->desc->bpc;

	return pinfo->desc->num_modes;
}

static enum drm_panel_orientation nt36532e_get_orientation(struct drm_panel *panel)
{
	struct panel_info *pinfo = to_panel_info(panel);

	return pinfo->orientation;
}

static const struct drm_panel_funcs nt36532e_panel_funcs = {
	.disable = nt36532e_disable,
	.prepare = nt36532e_prepare,
	.unprepare = nt36532e_unprepare,
	.get_modes = nt36532e_get_modes,
	.get_orientation = nt36532e_get_orientation,
};

static int nt36532e_probe(struct mipi_dsi_device *dsi)
{
	struct device *dev = &dsi->dev;
	struct device_node *dsi1;
	struct mipi_dsi_host *dsi1_host;
	struct panel_info *pinfo;
	const struct mipi_dsi_device_info *info;
	int i, ret;

	pinfo = devm_kzalloc(dev, sizeof(*pinfo), GFP_KERNEL);
	if (!pinfo)
		return -ENOMEM;

	pinfo->vddio = devm_regulator_get(dev, "vddio");
	if (IS_ERR(pinfo->vddio))
		return dev_err_probe(dev, PTR_ERR(pinfo->vddio), "failed to get vddio regulator\n");

	pinfo->reset_gpio = devm_gpiod_get(dev, "reset", GPIOD_OUT_HIGH);
	if (IS_ERR(pinfo->reset_gpio))
		return dev_err_probe(dev, PTR_ERR(pinfo->reset_gpio), "failed to get reset gpio\n");

	pinfo->desc = of_device_get_match_data(dev);
	if (!pinfo->desc)
		return -ENODEV;

	/* If the panel is dual dsi, register DSI1 */
	if (pinfo->desc->is_dual_dsi) {
		info = &pinfo->desc->dsi_info;

		dsi1 = of_graph_get_remote_node(dsi->dev.of_node, 1, -1);
		if (!dsi1) {
			dev_err(dev, "cannot get secondary DSI node.\n");
			return -ENODEV;
		}

		dsi1_host = of_find_mipi_dsi_host_by_node(dsi1);
		of_node_put(dsi1);
		if (!dsi1_host)
			return dev_err_probe(dev, -EPROBE_DEFER, "cannot get secondary DSI host\n");

		pinfo->dsi[1] = mipi_dsi_device_register_full(dsi1_host, info);
		if (IS_ERR(pinfo->dsi[1])) {
			dev_err(dev, "cannot get secondary DSI device\n");
			return PTR_ERR(pinfo->dsi[1]);
		}
	}

	pinfo->dsi[0] = dsi;
	mipi_dsi_set_drvdata(dsi, pinfo);
	drm_panel_init(&pinfo->panel, dev, &nt36532e_panel_funcs, DRM_MODE_CONNECTOR_DSI);

	ret = of_drm_get_panel_orientation(dev->of_node, &pinfo->orientation);
	if (ret < 0) {
		dev_err(dev, "%pOF: failed to get orientation %d\n", dev->of_node, ret);
		return ret;
	}

	pinfo->panel.prepare_prev_first = true;

	ret = drm_panel_of_backlight(&pinfo->panel);
	if (ret)
		return dev_err_probe(dev, ret, "Failed to get backlight\n");

	drm_panel_add(&pinfo->panel);

	for (i = 0; i < DSI_NUM_MIN + pinfo->desc->is_dual_dsi; i++) {
		pinfo->dsi[i]->lanes = pinfo->desc->lanes;
		pinfo->dsi[i]->format = pinfo->desc->format;
		pinfo->dsi[i]->mode_flags = pinfo->desc->mode_flags;
#ifdef NT36532E_DSC
		pinfo->dsi[i]->dsc = &pinfo->desc->dsc;
		pinfo->dsi[i]->dsc_slice_per_pkt = 2;
#endif

		ret = mipi_dsi_attach(pinfo->dsi[i]);
		if (ret < 0)
			return dev_err_probe(dev, ret, "cannot attach to DSI%d host.\n", i);
	}

	return 0;
}

static const struct of_device_id nt36532e_of_match[] = {
	{
		.compatible = "xiaomi,sheng-nt36532e",
		.data = &sheng_tianma_desc,
	},
	{},
};
MODULE_DEVICE_TABLE(of, nt36532e_of_match);

static struct mipi_dsi_driver nt36532e_driver = {
	.probe = nt36532e_probe,
	.remove = nt36532e_remove,
	.driver = {
		.name = "panel-novatek-nt36532e",
		.of_match_table = nt36532e_of_match,
	},
};
module_mipi_dsi_driver(nt36532e_driver);

MODULE_AUTHOR("map220v <map220v300@gmail.com>");
MODULE_DESCRIPTION("DRM driver for Novatek NT36532E based MIPI DSI panels");
MODULE_LICENSE("GPL");
