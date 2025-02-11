// SPDX-License-Identifier: GPL-2.0+

#include <common.h>
#include <backlight.h>
#include <dm.h>
#include <mipi_dsi.h>
#include <panel.h>
#include <asm/gpio.h>
#include <dm/device_compat.h>
#include <linux/delay.h>
#include <power/regulator.h>

struct ili9881c_panel_priv {
	struct udevice *reg;
	struct udevice *backlight;
	struct gpio_desc reset;
};

static const struct display_timing default_timing = {
	.pixelclock = {.min = 54000000, .typ = 54000000, .max = 54000000,},
	.hactive = {.min = 720, .typ = 720, .max = 720,},
	.hfront_porch = {.min = 20, .typ = 20, .max = 20,},
	.hback_porch = {.min = 20, .typ = 20, .max = 20,},
	.hsync_len = {.min = 2, .typ = 2, .max = 2,},
	.vactive = {.min = 1280, .typ = 1280, .max = 1280,},
	.vfront_porch = {.min = 15, .typ = 15, .max = 15,},
	.vback_porch = {.min = 10, .typ = 10, .max = 10,},
	.vsync_len = {.min = 2, .typ = 2, .max = 2,},
	.flags = 0,
};

static int ili9881c_switch_page(struct udevice *dev, u8 page)
{
	u8 buf[4] = { 0xff, 0x98, 0x81, page };

        struct mipi_dsi_panel_plat *plat = dev_get_platdata(dev);
        struct mipi_dsi_device *device = plat->device;

        return mipi_dsi_dcs_write_buffer(device, buf, sizeof(buf));
}

static int ili9881c_send_cmd_data(struct udevice *dev, u8 cmd, u8 data)
{
	u8 buf[2] = { cmd, data };
	struct mipi_dsi_panel_plat *plat = dev_get_platdata(dev);
        struct mipi_dsi_device *device = plat->device;

        return mipi_dsi_dcs_write_buffer(device, buf, sizeof(buf));
}

struct ili9881c_instr {
	u8 cmd;
	u8 data;
};

#define LCD_ILI9881C_CMD(CMD, DATA)	{.cmd = CMD, .data = DATA}

static const struct ili9881c_instr ili9881c_init_data[] = {
    LCD_ILI9881C_CMD(0xFF, 0x03),
    LCD_ILI9881C_CMD(0x01, 0x00),
    LCD_ILI9881C_CMD(0x02, 0x00),
    LCD_ILI9881C_CMD(0x03, 0x55),
    LCD_ILI9881C_CMD(0x04, 0x13),
    LCD_ILI9881C_CMD(0x05, 0x00),
    LCD_ILI9881C_CMD(0x06, 0x06),
    LCD_ILI9881C_CMD(0x07, 0x01),
    LCD_ILI9881C_CMD(0x08, 0x00),
    LCD_ILI9881C_CMD(0x09, 0x01),
    LCD_ILI9881C_CMD(0x0A, 0x01),
    LCD_ILI9881C_CMD(0x0B, 0x00),
    LCD_ILI9881C_CMD(0x0C, 0x00),
    LCD_ILI9881C_CMD(0x0D, 0x00),
    LCD_ILI9881C_CMD(0x0E, 0x00),
    LCD_ILI9881C_CMD(0x0F, 0x18),
    LCD_ILI9881C_CMD(0x10, 0x18),
    LCD_ILI9881C_CMD(0x11, 0x00),
    LCD_ILI9881C_CMD(0x12, 0x00),
    LCD_ILI9881C_CMD(0x13, 0x00),
    LCD_ILI9881C_CMD(0x14, 0x00),
    LCD_ILI9881C_CMD(0x15, 0x00),
    LCD_ILI9881C_CMD(0x16, 0x00),
    LCD_ILI9881C_CMD(0x17, 0x00),
    LCD_ILI9881C_CMD(0x18, 0x00),
    LCD_ILI9881C_CMD(0x19, 0x00),
    LCD_ILI9881C_CMD(0x1A, 0x00),
    LCD_ILI9881C_CMD(0x1B, 0x00),
    LCD_ILI9881C_CMD(0x1C, 0x00),
    LCD_ILI9881C_CMD(0x1D, 0x00),
    LCD_ILI9881C_CMD(0x1E, 0x44),
    LCD_ILI9881C_CMD(0x1F, 0x80),
    LCD_ILI9881C_CMD(0x20, 0x02),
    LCD_ILI9881C_CMD(0x21, 0x03),
    LCD_ILI9881C_CMD(0x22, 0x00),
    LCD_ILI9881C_CMD(0x23, 0x00),
    LCD_ILI9881C_CMD(0x24, 0x00),
    LCD_ILI9881C_CMD(0x25, 0x00),
    LCD_ILI9881C_CMD(0x26, 0x00),
    LCD_ILI9881C_CMD(0x27, 0x00),
    LCD_ILI9881C_CMD(0x28, 0x33),
    LCD_ILI9881C_CMD(0x29, 0x03),
    LCD_ILI9881C_CMD(0x2A, 0x00),
    LCD_ILI9881C_CMD(0x2B, 0x00),
    LCD_ILI9881C_CMD(0x2C, 0x00),
    LCD_ILI9881C_CMD(0x2D, 0x00),
    LCD_ILI9881C_CMD(0x2E, 0x00),
    LCD_ILI9881C_CMD(0x2F, 0x00),
    LCD_ILI9881C_CMD(0x30, 0x00),
    LCD_ILI9881C_CMD(0x31, 0x00),
    LCD_ILI9881C_CMD(0x32, 0x00),
    LCD_ILI9881C_CMD(0x33, 0x00),
    LCD_ILI9881C_CMD(0x34, 0x04),
    LCD_ILI9881C_CMD(0x35, 0x00),
    LCD_ILI9881C_CMD(0x36, 0x00),
    LCD_ILI9881C_CMD(0x37, 0x00),
    LCD_ILI9881C_CMD(0x38, 0x01),
    LCD_ILI9881C_CMD(0x39, 0x00),
    LCD_ILI9881C_CMD(0x3A, 0x00),
    LCD_ILI9881C_CMD(0x3B, 0x00),
    LCD_ILI9881C_CMD(0x3C, 0x00),
    LCD_ILI9881C_CMD(0x3D, 0x00),
    LCD_ILI9881C_CMD(0x3E, 0x00),
    LCD_ILI9881C_CMD(0x3F, 0x00),
    LCD_ILI9881C_CMD(0x40, 0x00),
    LCD_ILI9881C_CMD(0x41, 0x00),
    LCD_ILI9881C_CMD(0x42, 0x00),
    LCD_ILI9881C_CMD(0x43, 0x00),
    LCD_ILI9881C_CMD(0x44, 0x00),
    LCD_ILI9881C_CMD(0x50, 0x01),
    LCD_ILI9881C_CMD(0x51, 0x23),
    LCD_ILI9881C_CMD(0x52, 0x45),
    LCD_ILI9881C_CMD(0x53, 0x67),
    LCD_ILI9881C_CMD(0x54, 0x89),
    LCD_ILI9881C_CMD(0x55, 0xAB),
    LCD_ILI9881C_CMD(0x56, 0x01),
    LCD_ILI9881C_CMD(0x57, 0x23),
    LCD_ILI9881C_CMD(0x58, 0x45),
    LCD_ILI9881C_CMD(0x59, 0x67),
    LCD_ILI9881C_CMD(0x5A, 0x89),
    LCD_ILI9881C_CMD(0x5B, 0xAB),
    LCD_ILI9881C_CMD(0x5C, 0xCD),
    LCD_ILI9881C_CMD(0x5D, 0xEF),
    LCD_ILI9881C_CMD(0x5E, 0x11),
    LCD_ILI9881C_CMD(0x5F, 0x14),
    LCD_ILI9881C_CMD(0x60, 0x15),
    LCD_ILI9881C_CMD(0x61, 0x0F),
    LCD_ILI9881C_CMD(0x62, 0x0D),
    LCD_ILI9881C_CMD(0x63, 0x0E),
    LCD_ILI9881C_CMD(0x64, 0x0C),
    LCD_ILI9881C_CMD(0x65, 0x06),
    LCD_ILI9881C_CMD(0x66, 0x02),
    LCD_ILI9881C_CMD(0x67, 0x02),
    LCD_ILI9881C_CMD(0x68, 0x02),
    LCD_ILI9881C_CMD(0x69, 0x02),
    LCD_ILI9881C_CMD(0x6A, 0x02),
    LCD_ILI9881C_CMD(0x6B, 0x02),
    LCD_ILI9881C_CMD(0x6C, 0x02),
    LCD_ILI9881C_CMD(0x6D, 0x02),
    LCD_ILI9881C_CMD(0x6E, 0x02),
    LCD_ILI9881C_CMD(0x6F, 0x02),
    LCD_ILI9881C_CMD(0x70, 0x02),
    LCD_ILI9881C_CMD(0x71, 0x00),
    LCD_ILI9881C_CMD(0x72, 0x01),
    LCD_ILI9881C_CMD(0x73, 0x08),
    LCD_ILI9881C_CMD(0x74, 0x02),
    LCD_ILI9881C_CMD(0x75, 0x14),
    LCD_ILI9881C_CMD(0x76, 0x15),
    LCD_ILI9881C_CMD(0x77, 0x0F),
    LCD_ILI9881C_CMD(0x78, 0x0D),
    LCD_ILI9881C_CMD(0x79, 0x0E),
    LCD_ILI9881C_CMD(0x7A, 0x0C),
    LCD_ILI9881C_CMD(0x7B, 0x08),
    LCD_ILI9881C_CMD(0x7C, 0x02),
    LCD_ILI9881C_CMD(0x7D, 0x02),
    LCD_ILI9881C_CMD(0x7E, 0x02),
    LCD_ILI9881C_CMD(0x7F, 0x02),
    LCD_ILI9881C_CMD(0x80, 0x02),
    LCD_ILI9881C_CMD(0x81, 0x02),
    LCD_ILI9881C_CMD(0x82, 0x02),
    LCD_ILI9881C_CMD(0x83, 0x02),
    LCD_ILI9881C_CMD(0x84, 0x02),
    LCD_ILI9881C_CMD(0x85, 0x02),
    LCD_ILI9881C_CMD(0x86, 0x02),
    LCD_ILI9881C_CMD(0x87, 0x00),
    LCD_ILI9881C_CMD(0x88, 0x01),
    LCD_ILI9881C_CMD(0x89, 0x06),
    LCD_ILI9881C_CMD(0x8A, 0x02),
    LCD_ILI9881C_CMD(0xFF, 0x04),
    LCD_ILI9881C_CMD(0x6C, 0x15),
    LCD_ILI9881C_CMD(0x6E, 0x2A),
    LCD_ILI9881C_CMD(0x6F, 0x33),
    LCD_ILI9881C_CMD(0x3A, 0x24),
    LCD_ILI9881C_CMD(0x8D, 0x14),
    LCD_ILI9881C_CMD(0x87, 0xBA),
    LCD_ILI9881C_CMD(0x26, 0x76),
    LCD_ILI9881C_CMD(0xB2, 0xD1),
    LCD_ILI9881C_CMD(0xB5, 0xD7),
    LCD_ILI9881C_CMD(0x35, 0x1F),
    LCD_ILI9881C_CMD(0xFF, 0x01),
    LCD_ILI9881C_CMD(0x22, 0x0A),
    LCD_ILI9881C_CMD(0x53, 0x72),
    LCD_ILI9881C_CMD(0x55, 0x77),
    LCD_ILI9881C_CMD(0x50, 0xA6),
    LCD_ILI9881C_CMD(0x51, 0xA6),
    LCD_ILI9881C_CMD(0x31, 0x00),
    LCD_ILI9881C_CMD(0x60, 0x20),
    LCD_ILI9881C_CMD(0xA0, 0x08),
    LCD_ILI9881C_CMD(0xA1, 0x1A),
    LCD_ILI9881C_CMD(0xA2, 0x2A),
    LCD_ILI9881C_CMD(0xA3, 0x14),
    LCD_ILI9881C_CMD(0xA4, 0x17),
    LCD_ILI9881C_CMD(0xA5, 0x2B),
    LCD_ILI9881C_CMD(0xA6, 0x1D),
    LCD_ILI9881C_CMD(0xA7, 0x20),
    LCD_ILI9881C_CMD(0xA8, 0x9D),
    LCD_ILI9881C_CMD(0xA9, 0x1C),
    LCD_ILI9881C_CMD(0xAA, 0x29),
    LCD_ILI9881C_CMD(0xAB, 0x8F),
    LCD_ILI9881C_CMD(0xAC, 0x20),
    LCD_ILI9881C_CMD(0xAD, 0x1F),
    LCD_ILI9881C_CMD(0xAE, 0x4F),
    LCD_ILI9881C_CMD(0xAF, 0x23),
    LCD_ILI9881C_CMD(0xB0, 0x29),
    LCD_ILI9881C_CMD(0xB1, 0x56),
    LCD_ILI9881C_CMD(0xB2, 0x66),
    LCD_ILI9881C_CMD(0xB3, 0x39),
    LCD_ILI9881C_CMD(0xC0, 0x08),
    LCD_ILI9881C_CMD(0xC1, 0x1A),
    LCD_ILI9881C_CMD(0xC2, 0x2A),
    LCD_ILI9881C_CMD(0xC3, 0x15),
    LCD_ILI9881C_CMD(0xC4, 0x17),
    LCD_ILI9881C_CMD(0xC5, 0x2B),
    LCD_ILI9881C_CMD(0xC6, 0x1D),
    LCD_ILI9881C_CMD(0xC7, 0x20),
    LCD_ILI9881C_CMD(0xC8, 0x9D),
    LCD_ILI9881C_CMD(0xC9, 0x1D),
    LCD_ILI9881C_CMD(0xCA, 0x29),
    LCD_ILI9881C_CMD(0xCB, 0x8F),
    LCD_ILI9881C_CMD(0xCC, 0x20),
    LCD_ILI9881C_CMD(0xCD, 0x1F),
    LCD_ILI9881C_CMD(0xCE, 0x4F),
    LCD_ILI9881C_CMD(0xCF, 0x24),
    LCD_ILI9881C_CMD(0xD0, 0x29),
    LCD_ILI9881C_CMD(0xD1, 0x56),
    LCD_ILI9881C_CMD(0xD2, 0x66),
    LCD_ILI9881C_CMD(0xD3, 0x39),
    LCD_ILI9881C_CMD(0xFF, 0x00),
    LCD_ILI9881C_CMD(0x11, 0x00),
};

static void ili9881c_init_sequence(struct udevice *dev)
{
	int i;
	int ret;
	u8 buf[128] = {0};

        struct mipi_dsi_panel_plat *plat = dev_get_platdata(dev);
        struct mipi_dsi_device *device = plat->device;

	printf("MIPI DSI LCD ILI9881C setup.\n");
	for (i = 0; i < ARRAY_SIZE(ili9881c_init_data); i++) {
		const struct ili9881c_instr *instr = &ili9881c_init_data[i];

		if (instr->cmd == 0xFF) {
			ret = ili9881c_switch_page(dev, instr->data);
		} else {
			ret = ili9881c_send_cmd_data(dev, instr->cmd, instr->data);
		}
		if (ret < 0){
			printf("MIPI DSI LCD ILI9881C setup failed with cmd: %08X.\n", instr->cmd);
			return;
		}
	}

	ili9881c_switch_page(dev, 0);
	buf[0] = MIPI_DCS_EXIT_SLEEP_MODE;
	buf[1] = 0;
        mipi_dsi_dcs_write_buffer(device, buf, 2);
	mdelay(120);
	buf[0] = MIPI_DCS_SET_DISPLAY_ON;
        mipi_dsi_dcs_write_buffer(device, buf, 2);

	return;
}

static int ili9881c_panel_enable_backlight(struct udevice *dev)
{
	struct mipi_dsi_panel_plat *plat = dev_get_platdata(dev);
	struct mipi_dsi_device *device = plat->device;
	struct ili9881c_panel_priv *priv = dev_get_priv(dev);
	int ret;

	ret = mipi_dsi_attach(device);
	if (ret < 0)
		return ret;

	ili9881c_init_sequence(dev);

	ret = mipi_dsi_dcs_exit_sleep_mode(device);
	if (ret)
		return ret;

	mdelay(125);

	ret = mipi_dsi_dcs_set_display_on(device);
	if (ret)
		return ret;

	mdelay(20);

	return 0;
}

static int ili9881c_panel_get_display_timing(struct udevice *dev,
                                             struct display_timing *timings)
{
	memcpy(timings, &default_timing, sizeof(*timings));
	return 0;
}

static int ili9881c_panel_ofdata_to_platdata(struct udevice *dev)
{
	struct ili9881c_panel_priv *priv = dev_get_priv(dev);
	int ret;

	if (IS_ENABLED(CONFIG_DM_REGULATOR)) {
		ret =  device_get_supply_regulator(dev, "power-supply",
						   &priv->reg);
		if (ret && ret != -ENOENT) {
			dev_err(dev, "Warning: cannot get power supply\n");
			return ret;
		}
	}

	ret = gpio_request_by_name(dev, "reset-gpios", 0, &priv->reset,
				   GPIOD_IS_OUT);
	if (ret) {
		dev_err(dev, "Warning: cannot get reset GPIO\n");
		if (ret != -ENOENT)
			return ret;
	}

	ret = uclass_get_device_by_phandle(UCLASS_PANEL_BACKLIGHT, dev,
					   "backlight", &priv->backlight);
	if (ret) {
		dev_err(dev, "Cannot get backlight: ret=%d\n", ret);
		return ret;
	}

	return 0;
}

static int ili9881c_panel_probe(struct udevice *dev)
{

	struct ili9881c_panel_priv *priv = dev_get_priv(dev);
	struct mipi_dsi_panel_plat *plat = dev_get_platdata(dev);
	int ret;

	if (IS_ENABLED(CONFIG_DM_REGULATOR) && priv->reg) {
		ret = regulator_set_enable(priv->reg, true);
		if (ret)
			return ret;
	}

	/* reset panel */
	dm_gpio_set_value(&priv->reset, true);
	mdelay(1);
	dm_gpio_set_value(&priv->reset, false);
	mdelay(10);

	plat->lanes = 2;
	plat->format = MIPI_DSI_FMT_RGB888;
	plat->mode_flags = MIPI_DSI_MODE_VIDEO |
			MIPI_DSI_MODE_VIDEO_SYNC_PULSE |
			MIPI_DSI_MODE_LPM;

	return 0;
}

static const struct panel_ops ili9881c_panel_ops = {
	.enable_backlight = ili9881c_panel_enable_backlight,
	.get_display_timing = ili9881c_panel_get_display_timing,
};

static const struct udevice_id ili9881c_panel_ids[] = {
	{ .compatible = "powertip,ph720128t003-zbc02" },
	{ }
};

U_BOOT_DRIVER(ili9881c_panel) = {
	.name			  = "ili9881c_panel",
	.id			  = UCLASS_PANEL,
	.of_match		  = ili9881c_panel_ids,
	.ops			  = &ili9881c_panel_ops,
	.ofdata_to_platdata	  = ili9881c_panel_ofdata_to_platdata,
	.probe			  = ili9881c_panel_probe,
	.platdata_auto_alloc_size = sizeof(struct mipi_dsi_panel_plat),
	.priv_auto_alloc_size	= sizeof(struct ili9881c_panel_priv),
};
