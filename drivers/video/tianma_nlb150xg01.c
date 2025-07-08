// SPDX-License-Identifier: GPL-2.0+
/*
 * Panel Driver with Full Debug Logs using printf
 */

#include <common.h>
#include <dm.h>
#include <panel.h>
#include <mipi_dsi.h>
#include <asm/gpio.h>
#include <linux/delay.h>
#include <power/regulator.h>
#include <i2c.h>
#include <pwm.h>

#define OTM8009A_BACKLIGHT_DEFAULT 100
#define OTM8009A_BACKLIGHT_MAX 255

// Panel private data
struct otm8009a_panel_priv {
	struct udevice *reg;
	struct gpio_desc enable;
	struct gpio_desc backlight_en;
	struct gpio_desc backlight_pwm;
	struct udevice *backlight;
	struct udevice *pwm;
};

// Display timing
static const struct display_timing default_timing = {
	.pixelclock.typ = 65000000,
	.hactive.typ = 1024,
	.hfront_porch.typ = 24,
	.hsync_len.typ = 136,
	.hback_porch.typ = 160,
	.vactive.typ = 768,
	.vfront_porch.typ = 3,
	.vsync_len.typ = 6,
	.vback_porch.typ = 29,
};

// Initialization sequence
static int otm8009a_init_sequence(struct udevice *dev)
{
	printf("[panel] Entering init sequence\n");
	struct udevice *dev1;
	u8 val;
	bool pll_en_flag = false;
	int ret, i;

	u32 hactive = 1024, vactive = 768;
	u32 hfp = 24, hsw = 136, hbp = 160;
	u32 vfp = 3, vsw = 6, vbp = 29;

	ret = i2c_get_chip_for_busnum(3, 0x2c, 1, &dev1);
	if (ret) {
		printf("[panel] I2C chip not found on bus 3\n");
		return ret;
	}

	printf("[panel] Programming bridge registers\n");
	dm_i2c_reg_write(dev1, 0x0d, 0x00);
	mdelay(1); // T1
	dm_i2c_reg_write(dev1, 0x0a, 0x05);
	dm_i2c_reg_write(dev1, 0x12, 0x48);
	dm_i2c_reg_write(dev1, 0x0b, 0x28);
	dm_i2c_reg_write(dev1, 0x0d, 0x00);
	dm_i2c_reg_write(dev1, 0x10, 0x30);
	dm_i2c_reg_write(dev1, 0x11, 0x00);
	dm_i2c_reg_write(dev1, 0x18, 0x78);
	dm_i2c_reg_write(dev1, 0x19, 0x00);
	dm_i2c_reg_write(dev1, 0x1a, 0x00);
	dm_i2c_reg_write(dev1, 0x1b, 0x00);

	dm_i2c_reg_write(dev1, 0x20, hactive & 0xff);
	dm_i2c_reg_write(dev1, 0x21, hactive >> 8);
	dm_i2c_reg_write(dev1, 0x24, vactive & 0xff);
	dm_i2c_reg_write(dev1, 0x25, vactive >> 8);

	dm_i2c_reg_write(dev1, 0x2c, hsw & 0xff);
	dm_i2c_reg_write(dev1, 0x2d, hsw >> 8);
	dm_i2c_reg_write(dev1, 0x30, vsw & 0xff);
	dm_i2c_reg_write(dev1, 0x31, vsw >> 8);
	dm_i2c_reg_write(dev1, 0x34, hbp);
	dm_i2c_reg_write(dev1, 0x36, vbp);
	dm_i2c_reg_write(dev1, 0x38, hfp);
	dm_i2c_reg_write(dev1, 0x3a, vfp);
	dm_i2c_reg_write(dev1, 0x3c, 0x00);
	dm_i2c_reg_write(dev1, 0x28, 0xff);
	dm_i2c_reg_write(dev1, 0x29, 0x00);

	printf("[panel] Enabling PLL and waiting for lock\n");
	dm_i2c_reg_write(dev1, 0x0d, 0x01);
	for (i = 0; i < 10; i++) {
		mdelay(1);
		val = dm_i2c_reg_read(dev1, 0x0a);
		if (val & 0x80) {
			pll_en_flag = true;
			break;
		}
	}
	if (!pll_en_flag) {
		printf("[panel] PLL lock failed\n");
		dm_i2c_reg_write(dev1, 0x0d, 0x00);
		return -EINVAL;
	}

	printf("[panel] PLL locked. Sending reset\n");
	dm_i2c_reg_write(dev1, 0x09, 0x01);
	mdelay(10); // T2
	val = dm_i2c_reg_read(dev1, 0xe5);
	dm_i2c_reg_write(dev1, 0xe5, val);
	printf("[panel] Init sequence complete\n");
	return 0;
}

// Enable backlight
static int otm8009a_panel_enable_backlight(struct udevice *dev)
{
	printf("[panel] Enabling backlight\n");
	struct mipi_dsi_panel_plat *plat = dev_get_platdata(dev);
	struct mipi_dsi_device *device = plat->device;
	struct otm8009a_panel_priv *priv = dev_get_priv(dev);
	int ret = 0;

	ret = mipi_dsi_attach(device);
	if (ret < 0) {
		printf("[panel] mipi_dsi_attach failed\n");
		return ret;
	}

	ret = otm8009a_init_sequence(dev);
	if (ret) {
		printf("[panel] Bridge init failed\n");
		return ret;
	}

	printf("[panel] Waiting 500ms for panel stabilization\n");
	mdelay(500); // T3

	printf("[panel] Enabling power and PWM GPIOs\n");
	dm_gpio_set_value(&priv->enable, true);
	mdelay(10);
	dm_gpio_set_value(&priv->backlight_pwm, true);
	mdelay(10);
	dm_gpio_set_value(&priv->backlight_en, true);
	printf("[panel] Backlight enabled\n");
	return 0;
}

// Parse GPIO and regulator info
static int otm8009a_panel_ofdata_to_platdata(struct udevice *dev)
{
	printf("[panel] Parsing GPIOs and regulators\n");
	struct otm8009a_panel_priv *priv = dev_get_priv(dev);
	ofnode node = ofnode_path("/config");
	if (!ofnode_valid(node)) return -EINVAL;

	gpio_request_by_name_nodev(node, "backlight-gpios", 0, &priv->backlight_en, GPIOD_IS_OUT);
	gpio_request_by_name_nodev(node, "lvds-gpios", 0, &priv->enable, GPIOD_IS_OUT);
	gpio_request_by_name_nodev(node, "pwm-gpios", 0, &priv->backlight_pwm, GPIOD_IS_OUT);

	if (IS_ENABLED(CONFIG_DM_REGULATOR)) {
		device_get_supply_regulator(dev, "power-supply", &priv->reg);
	}
	return 0;
}

// Set up DSI parameters
static int otm8009a_panel_probe(struct udevice *dev)
{
	printf("[panel] Panel probe started\n");
	struct mipi_dsi_panel_plat *plat = dev_get_platdata(dev);
	plat->lanes = 2;
	plat->format = MIPI_DSI_FMT_RGB888;
	plat->mode_flags = MIPI_DSI_MODE_VIDEO_BURST;
	printf("[panel] Panel probe done\n");
	return 0;
}

// Display timing callback
static int otm8009a_panel_get_display_timing(struct udevice *dev, struct display_timing *timings)
{
	memcpy(timings, &default_timing, sizeof(*timings));
	return 0;
}

// Driver structure
static const struct panel_ops otm8009a_panel_ops = {
	.enable_backlight = otm8009a_panel_enable_backlight,
	.get_display_timing = otm8009a_panel_get_display_timing,
};

static const struct udevice_id otm8009a_panel_ids[] = {
	{ .compatible = "tianma,nlb150xg01" },
	{ }
};

U_BOOT_DRIVER(otm8009a_panel) = {
	.name = "otm8009a_panel",
	.id = UCLASS_PANEL,
	.of_match = otm8009a_panel_ids,
	.ops = &otm8009a_panel_ops,
	.ofdata_to_platdata = otm8009a_panel_ofdata_to_platdata,
	.probe = otm8009a_panel_probe,
	.platdata_auto_alloc_size = sizeof(struct mipi_dsi_panel_plat),
	.priv_auto_alloc_size = sizeof(struct otm8009a_panel_priv),
};
