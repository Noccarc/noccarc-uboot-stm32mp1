// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright (C) 2019 STMicroelectronics - All Rights Reserved
 * Author(s): Yannick Fertre <yannick.fertre@st.com> for STMicroelectronics.
 *            Philippe Cornu <philippe.cornu@st.com> for STMicroelectronics.
 *
 * This otm8009a panel driver is inspired from the Linux Kernel driver
 * drivers/gpu/drm/panel/panel-orisetech-otm8009a.c.
 */
#define LOG_DEBUG
#define LOG_CATEGORY LOGC_DM

#include <common.h>
#include <backlight.h>
#include <dm.h>
#include <mipi_dsi.h>
#include <panel.h>
#include <asm/gpio.h>
#include <dm/device_compat.h>
#include <linux/delay.h>
#include <power/regulator.h>
#include <i2c.h>
#include <command.h>

#define OTM8009A_BACKLIGHT_DEFAULT	100
#define OTM8009A_BACKLIGHT_MAX		255

/* Manufacturer Command Set */


struct otm8009a_panel_priv {
	struct udevice *reg;
	struct gpio_desc enable;
};

static const struct display_timing default_timing = {
	.pixelclock.typ		= 66000000,
	.hactive.typ		= 1024,
	.hfront_porch.typ	= 155,
	.hback_porch.typ	= 155,
	.hsync_len.typ		= 10,
	.vactive.typ		= 768,
	.vfront_porch.typ	= 16,
	.vback_porch.typ	= 16,
	.vsync_len.typ		= 6,
};


static int otm8009a_init_sequence(struct udevice *dev)
{
	struct mipi_dsi_panel_plat *plat = dev_get_platdata(dev);
	struct mipi_dsi_device *device = plat->device;
	uchar ret;
	struct udevice *dev1;
	int ret1;

	ret1 = i2c_get_chip_for_busnum(3, 0x2c,
				      1, &dev1);
	if (ret1) {
		log_info("driver %s: Cannot find udev for a bus %d\n", __func__,
		       3);
		return ret1;
	}
	
	ret = dm_i2c_reg_read(dev1, 0xe5);
	//ret = i2c_reg_read(0x2c, 0xe5);

	log_info("driver: %d \n", ret);
	

	return 0;
}

static int otm8009a_panel_enable_backlight(struct udevice *dev)
{
	struct mipi_dsi_panel_plat *plat = dev_get_platdata(dev);
	struct mipi_dsi_device *device = plat->device;
	int ret;
	
	log_info("driver: Entered enable backlight \n");

	ret = mipi_dsi_attach(device);
	if (ret < 0)
		return ret;

	ret = otm8009a_init_sequence(dev);
	if (ret)
		return ret;
	

	return 0;
}

static int otm8009a_panel_get_display_timing(struct udevice *dev,
					     struct display_timing *timings)
{
	memcpy(timings, &default_timing, sizeof(*timings));

	return 0;
}

static int otm8009a_panel_ofdata_to_platdata(struct udevice *dev)
{
	struct otm8009a_panel_priv *priv = dev_get_priv(dev);
	int ret;

	if (IS_ENABLED(CONFIG_DM_REGULATOR)) {
		ret =  device_get_supply_regulator(dev, "power-supply",
						   &priv->reg);
		if (ret && ret != -ENOENT) {
			dev_err(dev, "Warning: cannot get power supply\n");
			return ret;
		}
	}

	ret = gpio_request_by_name(dev, "enable-gpios", 0, &priv->enable,
				   GPIOD_IS_OUT_ACTIVE);
	if (ret) {
		dev_err(dev, "warning: cannot get enable GPIO\n");
		log_info("driver: enable gpio not found\n");
		if (ret != -ENOENT)
			return ret;	
	}

	return 0;
}

static int otm8009a_panel_probe(struct udevice *dev)
{
	struct otm8009a_panel_priv *priv = dev_get_priv(dev);
	struct mipi_dsi_panel_plat *plat = dev_get_platdata(dev);
	int ret;
	
	log_info("driver: Entered probe \n");
	
	if (IS_ENABLED(CONFIG_DM_REGULATOR) && priv->reg) {
		dev_dbg(dev, "enable regulator '%s'\n", priv->reg->name);
		ret = regulator_set_enable(priv->reg, true);
		if (ret)
			return ret;
	}	

	/* enable panel */
	dm_gpio_set_value(&priv->enable, false);
	mdelay(10); /* >50us */
	dm_gpio_set_value(&priv->enable, true);
	mdelay(10); /* >5ms */

	/* fill characteristics of DSI data link */
	plat->lanes = 2;
	plat->format = MIPI_DSI_FMT_RGB888;
	plat->mode_flags = MIPI_DSI_MODE_VIDEO_BURST ;

	return 0;
}

static const struct panel_ops otm8009a_panel_ops = {
	.enable_backlight = otm8009a_panel_enable_backlight,
	.get_display_timing = otm8009a_panel_get_display_timing,
};

static const struct udevice_id otm8009a_panel_ids[] = {
	{ .compatible = "orisetech,otm8009a" },
	{ }
};

U_BOOT_DRIVER(otm8009a_panel) = {
	.name			  = "otm8009a_panel",
	.id			  = UCLASS_PANEL,
	.of_match		  = otm8009a_panel_ids,
	.ops			  = &otm8009a_panel_ops,
	.ofdata_to_platdata	  = otm8009a_panel_ofdata_to_platdata,
	.probe			  = otm8009a_panel_probe,
	.platdata_auto_alloc_size = sizeof(struct mipi_dsi_panel_plat),
	.priv_auto_alloc_size	= sizeof(struct otm8009a_panel_priv),
};
