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
	#include <pwm.h>

	#include <common.h>
#include <adc.h>
#include <bootm.h>
#include <clk.h>
#include <config.h>
#include <dm.h>
#include <env.h>
#include <env_internal.h>
#include <fdt_support.h>
#include <g_dnl.h>
#include <generic-phy.h>
#include <hang.h>
#include <i2c.h>
#include <init.h>
#include <led.h>
#include <log.h>
#include <malloc.h>
#include <misc.h>
#include <mtd_node.h>
#include <net.h>
#include <netdev.h>
#include <phy.h>
#include <remoteproc.h>
#include <reset.h>
#include <syscon.h>
#include <usb.h>
#include <watchdog.h>
#include <asm/io.h>
#include <asm/gpio.h>
#include <asm/arch/stm32.h>
#include <asm/arch/sys_proto.h>
#include <jffs2/load_kernel.h>
#include <linux/bitops.h>
#include <linux/delay.h>
#include <linux/err.h>
#include <linux/iopoll.h>
#include <power/regulator.h>
#include <usb/dwc2_udc.h>


	#define OTM8009A_BACKLIGHT_DEFAULT	100
	#define OTM8009A_BACKLIGHT_MAX		255


	/* ID registers */
	#define REG_ID(n)				                (0x00 + (n))
	/* Reset and clock registers */
	#define REG_RC_RESET				            0x09
	#define  REG_RC_RESET_SOFT_RESET		        0x01
	#define REG_RC_LVDS_PLL				            0x0a
	#define  REG_RC_LVDS_PLL_PLL_EN_STAT		    0x80
	#define  REG_RC_LVDS_PLL_LVDS_CLK_RANGE(n)	    (((n) & 0x7) << 1)
	#define  REG_RC_LVDS_PLL_HS_CLK_SRC_DPHY	    0x01
	#define REG_RC_DSI_CLK				            0x0b
	#define  REG_RC_DSI_CLK_DSI_CLK_DIVIDER(n)	    (((n) & 0x1f) << 3)
	#define  REG_RC_DSI_CLK_REFCLK_MULTIPLIER(n)	((n) & 0x3)
	#define REG_RC_PLL_EN				            0x0d
	#define  REG_RC_PLL_EN_PLL_EN			        0x01
	/* DSI registers */
	#define REG_DSI_LANE				            0x10
	#define  REG_DSI_LANE_LEFT_RIGHT_PIXELS		    0x80	      /* DSI85-only */
	#define  REG_DSI_LANE_DSI_CHANNEL_MODE_DUAL	    0	          /* DSI85-only */
	#define  REG_DSI_LANE_DSI_CHANNEL_MODE_2SINGLE	0x40	      /* DSI85-only */
	#define  REG_DSI_LANE_DSI_CHANNEL_MODE_SINGLE	0x20
	#define  REG_DSI_LANE_CHA_DSI_LANES(n)		    (((n) & 0x3) << 3)
	#define  REG_DSI_LANE_CHB_DSI_LANES(n)		    (((n) & 0x3) << 1)
	#define  REG_DSI_LANE_SOT_ERR_TOL_DIS		    0x01
	#define REG_DSI_EQ				                0x11
	#define  REG_DSI_EQ_CHA_DSI_DATA_EQ(n)		    (((n) & 0x3) << 6)
	#define  REG_DSI_EQ_CHA_DSI_CLK_EQ(n)		    (((n) & 0x3) << 2)
	#define REG_DSI_CLK				                0x12
	#define  REG_DSI_CLK_CHA_DSI_CLK_RANGE(n)	    ((n) & 0xff)
	/* LVDS registers */
	#define REG_LVDS_FMT				            0x18
	#define  REG_LVDS_FMT_DE_NEG_POLARITY		    0x80
	#define  REG_LVDS_FMT_HS_NEG_POLARITY		    0x40
	#define  REG_LVDS_FMT_VS_NEG_POLARITY		    0x20
	#define  REG_LVDS_FMT_LVDS_LINK_CFG		        0x10	      /* 0:AB 1:A-only */
	#define  REG_LVDS_FMT_CHA_24BPP_MODE		    0x08
	#define  REG_LVDS_FMT_CHB_24BPP_MODE		    0x04
	#define  REG_LVDS_FMT_CHA_24BPP_FORMAT1		    0x02
	#define  REG_LVDS_FMT_CHB_24BPP_FORMAT1		    0x01
	#define REG_LVDS_VCOM				            0x19
	#define  REG_LVDS_VCOM_CHA_LVDS_VOCM		    0x40
	#define  REG_LVDS_VCOM_CHB_LVDS_VOCM		    0x10
	#define  REG_LVDS_VCOM_CHA_LVDS_VOD_SWING(n)	(((n) & 0x3) << 2)
	#define  REG_LVDS_VCOM_CHB_LVDS_VOD_SWING(n)	((n) & 0x3)
	#define REG_LVDS_LANE				            0x1a
	#define  REG_LVDS_LANE_EVEN_ODD_SWAP		    0x40
	#define  REG_LVDS_LANE_CHA_REVERSE_LVDS		    0x20
	#define  REG_LVDS_LANE_CHB_REVERSE_LVDS		    0x10
	#define  REG_LVDS_LANE_CHA_LVDS_TERM		    0x02
	#define  REG_LVDS_LANE_CHB_LVDS_TERM		    0x01
	#define REG_LVDS_CM				                0x1b
	#define  REG_LVDS_CM_CHA_LVDS_CM_ADJUST(n)	    (((n) & 0x3) << 4)
	#define  REG_LVDS_CM_CHB_LVDS_CM_ADJUST(n)	    ((n) & 0x3)
	/* Video registers */
	#define REG_VID_CHA_ACTIVE_LINE_LENGTH_LOW	    0x20
	#define REG_VID_CHA_ACTIVE_LINE_LENGTH_HIGH	    0x21
	#define REG_VID_CHA_VERTICAL_DISPLAY_SIZE_LOW	0x24
	#define REG_VID_CHA_VERTICAL_DISPLAY_SIZE_HIGH	0x25
	#define REG_VID_CHA_SYNC_DELAY_LOW		        0x28
	#define REG_VID_CHA_SYNC_DELAY_HIGH		        0x29
	#define REG_VID_CHA_HSYNC_PULSE_WIDTH_LOW	    0x2c
	#define REG_VID_CHA_HSYNC_PULSE_WIDTH_HIGH	    0x2d
	#define REG_VID_CHA_VSYNC_PULSE_WIDTH_LOW	    0x30
	#define REG_VID_CHA_VSYNC_PULSE_WIDTH_HIGH	    0x31
	#define REG_VID_CHA_HORIZONTAL_BACK_PORCH	    0x34
	#define REG_VID_CHA_VERTICAL_BACK_PORCH		    0x36
	#define REG_VID_CHA_HORIZONTAL_FRONT_PORCH	    0x38
	#define REG_VID_CHA_VERTICAL_FRONT_PORCH	    0x3a
	#define REG_VID_CHA_TEST_PATTERN		        0x3c
	/* IRQ registers */
	#define REG_IRQ_GLOBAL				            0xe0
	#define  REG_IRQ_GLOBAL_IRQ_EN			        0x01
	#define REG_IRQ_EN				                0xe1
	#define  REG_IRQ_EN_CHA_SYNCH_ERR_EN		    0x80
	#define  REG_IRQ_EN_CHA_CRC_ERR_EN		        0x40
	#define  REG_IRQ_EN_CHA_UNC_ECC_ERR_EN		    0x20
	#define  REG_IRQ_EN_CHA_COR_ECC_ERR_EN		    0x10
	#define  REG_IRQ_EN_CHA_LLP_ERR_EN		        0x08
	#define  REG_IRQ_EN_CHA_SOT_BIT_ERR_EN		    0x04
	#define  REG_IRQ_EN_CHA_PLL_UNLOCK_EN		    0x01
	#define REG_IRQ_STAT				            0xe5
	#define  REG_IRQ_STAT_CHA_SYNCH_ERR		        0x80
	#define  REG_IRQ_STAT_CHA_CRC_ERR		        0x40
	#define  REG_IRQ_STAT_CHA_UNC_ECC_ERR		    0x20
	#define  REG_IRQ_STAT_CHA_COR_ECC_ERR		    0x10
	#define  REG_IRQ_STAT_CHA_LLP_ERR		        0x08
	#define  REG_IRQ_STAT_CHA_SOT_BIT_ERR		    0x04
	#define  REG_IRQ_STAT_CHA_PLL_UNLOCK		    0x01

	#define SINGLE_LINK		                        1		
	#define DUAL_LINK		                        2

	/* Manufacturer Command Set */


	struct otm8009a_panel_priv {
		struct udevice *reg;
		struct gpio_desc enable;
        struct gpio_desc backlight_en;
		struct gpio_desc backlight_pwm;
		struct udevice *backlight;
		struct udevice *pwm;
	};

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


	static void sn65dsi83_dump_registers(struct udevice *dev1)
	{
		u8 val;
		int i;

		printf("\nSN65DSI83 Register Dump:\n");

		for (i = 0x00; i <= 0x3C; i++) {
			val = dm_i2c_reg_read(dev1, i);
			printf("Reg[0x%02X] = 0x%02X\n", i, val);
		}

		printf("---- End Dump ----\n");
	}

	// 
	
	static int otm8009a_init_sequence(struct udevice *dev)
	{
		struct mipi_dsi_panel_plat *plat = dev_get_platdata(dev);
		struct mipi_dsi_device *device = plat->device;
		struct udevice *dev1;
		int ret1;
		u8 val = 0;
		bool pll_en_flag = false;
		int i;

		u32 hback_porch = 160;
		u32 hsync_len   = 136;
		u32 vback_porch = 29;
		u32 vsync_len   = 6;
		u32 hfront_porch = 24;
		u32 hactive = 1024;
		u32 vfront_porch = 3;
		u32 vactive = 768;

		ret1 = i2c_get_chip_for_busnum(3, 0x2c, 1, &dev1);
		if (ret1) {
			log_info("Cannot find SN65DSI83 on bus\n");
			return ret1;
		}

		/* ------------------------------------------------ */
		/* 1. Reset bridge first (important) */
		/* ------------------------------------------------ */

		dm_i2c_reg_write(dev1, REG_RC_RESET, REG_RC_RESET_SOFT_RESET);
		mdelay(10);

		/* ------------------------------------------------ */
		/* 2. Disable PLL before configuration */
		/* ------------------------------------------------ */

		dm_i2c_reg_write(dev1, REG_RC_PLL_EN, 0x00);
		mdelay(1);

		/* ------------------------------------------------ */
		/* 3. Configure PLL and DSI clock */
		/* ------------------------------------------------ */

		dm_i2c_reg_write(dev1, REG_RC_LVDS_PLL, 0x05);
		dm_i2c_reg_write(dev1, REG_DSI_CLK, 0x50);
		dm_i2c_reg_write(dev1, REG_RC_DSI_CLK, 0x28);

		/* ------------------------------------------------ */
		/* 4. Configure DSI lanes */
		/* ------------------------------------------------ */

		dm_i2c_reg_write(dev1, REG_DSI_LANE, 0x30);
		dm_i2c_reg_write(dev1, REG_DSI_EQ, 0x00);

		/* ------------------------------------------------ */
		/* 5. Configure LVDS output */
		/* ------------------------------------------------ */

		dm_i2c_reg_write(dev1, REG_LVDS_FMT, 0x78);   /* 24-bit LVDS JEIDA */
		dm_i2c_reg_write(dev1, REG_LVDS_VCOM, 0x00);
		dm_i2c_reg_write(dev1, REG_LVDS_LANE, 0x00);
		dm_i2c_reg_write(dev1, REG_LVDS_CM, 0x00);

		/* ------------------------------------------------ */
		/* 6. Configure video timing */
		/* ------------------------------------------------ */

		dm_i2c_reg_write(dev1, REG_VID_CHA_ACTIVE_LINE_LENGTH_LOW, (u8)(hactive & 0xff));
		dm_i2c_reg_write(dev1, REG_VID_CHA_ACTIVE_LINE_LENGTH_HIGH, (u8)((hactive >> 8) & 0xff));

		dm_i2c_reg_write(dev1, REG_VID_CHA_VERTICAL_DISPLAY_SIZE_LOW, (u8)(vactive & 0xff));
		dm_i2c_reg_write(dev1, REG_VID_CHA_VERTICAL_DISPLAY_SIZE_HIGH, (u8)((vactive >> 8) & 0xff));

		dm_i2c_reg_write(dev1, REG_VID_CHA_SYNC_DELAY_LOW, 0xff);
		dm_i2c_reg_write(dev1, REG_VID_CHA_SYNC_DELAY_HIGH, 0x00);

		dm_i2c_reg_write(dev1, REG_VID_CHA_HSYNC_PULSE_WIDTH_LOW, (u8)(hsync_len & 0xff));
		dm_i2c_reg_write(dev1, REG_VID_CHA_HSYNC_PULSE_WIDTH_HIGH, (u8)((hsync_len >> 8) & 0xff));

		dm_i2c_reg_write(dev1, REG_VID_CHA_VSYNC_PULSE_WIDTH_LOW, (u8)(vsync_len & 0xff));
		dm_i2c_reg_write(dev1, REG_VID_CHA_VSYNC_PULSE_WIDTH_HIGH, (u8)((vsync_len >> 8) & 0xff));

		dm_i2c_reg_write(dev1, REG_VID_CHA_HORIZONTAL_BACK_PORCH, (u8)(hback_porch & 0xff));
		dm_i2c_reg_write(dev1, REG_VID_CHA_VERTICAL_BACK_PORCH, (u8)(vback_porch & 0xff));

		dm_i2c_reg_write(dev1, REG_VID_CHA_HORIZONTAL_FRONT_PORCH, (u8)(hfront_porch & 0xff));
		dm_i2c_reg_write(dev1, REG_VID_CHA_VERTICAL_FRONT_PORCH, (u8)(vfront_porch & 0xff));

		dm_i2c_reg_write(dev1, REG_VID_CHA_TEST_PATTERN, 0x00);

		/* ------------------------------------------------ */
		/* 7. Enable PLL */
		/* ------------------------------------------------ */

		dm_i2c_reg_write(dev1, REG_RC_PLL_EN, REG_RC_PLL_EN_PLL_EN);

		for (i = 0; i < 10; i++) {
			mdelay(1);

			val = dm_i2c_reg_read(dev1, REG_RC_LVDS_PLL);

			if ((val & REG_RC_LVDS_PLL_PLL_EN_STAT) ==
				REG_RC_LVDS_PLL_PLL_EN_STAT) {
				pll_en_flag = true;
				break;
			}
		}

		if (!pll_en_flag) {
			log_info("SN65DSI83: PLL lock failed\n");
			dm_i2c_reg_write(dev1, REG_RC_PLL_EN, 0x00);
			return -EINVAL;
		}

		/* ------------------------------------------------ */
		/* 8. Soft reset after configuration */
		/* ------------------------------------------------ */

		dm_i2c_reg_write(dev1, REG_RC_RESET, REG_RC_RESET_SOFT_RESET);
		mdelay(10);

		/* Clear IRQ flags */

		val = dm_i2c_reg_read(dev1, REG_IRQ_STAT);
		dm_i2c_reg_write(dev1, REG_IRQ_STAT, val);

		/* Debug dump */

		sn65dsi83_dump_registers(dev1);

		return 0;
	}

	static int otm8009a_panel_enable_backlight(struct udevice *dev)
	{
		struct mipi_dsi_panel_plat *plat = dev_get_platdata(dev);
		struct mipi_dsi_device *device = plat->device;
		struct otm8009a_panel_priv *priv = dev_get_priv(dev);
		int ret;

		log_info("driver: Entered enable backlight\n");

		ret = mipi_dsi_attach(device);
		if (ret < 0)
			return ret;

		ret = otm8009a_init_sequence(dev);
		if (ret)
			return ret;

		mdelay(100); // Wait after bridge setup

		dm_gpio_set_value(&priv->backlight_en, true);
		mdelay(10);
		dm_gpio_set_value(&priv->backlight_pwm, true);

		return 0;
	}

	static int otm8009a_panel_get_display_timing(struct udevice *dev,
							 struct display_timing *timings)
	{
		memcpy(timings, &default_timing, sizeof(*timings));

		return 0;
	}

	static int otm8009a_panel_ofdata_to_platdata(struct udevicotm8009a_panel_ofdata_to_platdatae *dev)
	{
		struct otm8009a_panel_priv *priv = dev_get_priv(dev);
		int ret;
		ofnode node;
	
		node = ofnode_path("/config");
		if (!ofnode_valid(node)) {
			debug("%s: no /config node?\n", __func__);
			return;
		}
		if (gpio_request_by_name_nodev(node, "backlight-gpios", 0,
					       &priv->backlight_en, GPIOD_IS_OUT)) {
			debug("%s: could not find a /config/backlight_en\n",
			      __func__);
				  log_info("driver: enable backlight-gpio not found\n");
		} 
		if (gpio_request_by_name_nodev(node, "lvds-gpios", 0,
					       &priv->enable, GPIOD_IS_OUT)) {
			debug("%s: could not find a /config/vds-gpios\n",
			      __func__);
				  log_info("driver: enable gpio not found\n");
		} 
		if (gpio_request_by_name_nodev(node, "pwm-gpios", 0,
					       &priv->backlight_pwm, GPIOD_IS_OUT)) {
			debug("%s: could not find a /config/pwm-gpios\n",
			      __func__);
				  log_info("driver: enable gpio not found\n");
		} 

		dm_gpio_set_value(&priv->enable, true);
		dm_gpio_set_value(&priv->backlight_pwm, true);
		dm_gpio_set_value(&priv->backlight_en, true);

		log_info("driver: Entered of to plat \n");
		
		if (IS_ENABLED(CONFIG_DM_REGULATOR)) {
			ret =  device_get_supply_regulator(dev, "power-supply",
							   &priv->reg);
			if (ret && ret != -ENOENT) {
				dev_err(dev, "Warning: cannot get power supply\n");
				return ret;
			}
		}

		// ret = gpio_request_by_name(dev, "enable-gpios", 0, &priv->enable,
		// 			   GPIOD_IS_OUT | GPIOD_IS_OUT_ACTIVE);
		// if (ret) {
		// 	dev_err(dev, "warning: cannot get enable GPIO\n");
		// 	log_info("driver: enable gpio not found\n");
		// 	if (ret != -ENOENT)
		// 		return ret;	
		// }

		/*ret = gpio_request_by_name(dev, "backlight-enable", 0, &priv->backlight_en,
					   GPIOD_IS_OUT | GPIOD_IS_OUT_ACTIVE);
		if (ret) {
			dev_err(dev, "warning: cannot get backlight-enable GPIO= %d\n",ret);
			log_info("driver: backlight-enable gpio not found = %d \n ",ret);
			if (ret != -ENOENT)
				return ret;	
		}

		ret = gpio_request_by_name(dev, "backlight-pwm", 0, &priv->backlight_pwm,
					   GPIOD_IS_OUT | GPIOD_IS_OUT_ACTIVE);
		if (ret) {
			dev_err(dev, "warning: cannot get backlight-pwm GPIO = %d\n ",ret);
			log_info("driver: backlight-pwm gpio not found=%d\n",ret);
			if (ret != -ENOENT)
				return ret;	
		}*/
		
		// ret = uclass_get_device_by_phandle(UCLASS_PANEL_BACKLIGHT, dev,
		// 				   "backlight", &priv->backlight);
		// if (ret) {
		// 	log_info("%s: Cannot get backlight: ret=%d\n", __func__, ret);
		// 	return ret;
		// }
		


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
		//dm_gpio_set_value(&priv->enable, false);
		//mdelay(10); /* >50us */
		//dm_gpio_set_value(&priv->enable, true);
		//mdelay(10); /* >5ms */
		// dm_gpio_set_value(&priv->backlight_gpio, true);

		// dm_gpio_set_value(&priv->backlight_en, true);

		// dm_gpio_set_value(&priv->backlight_pwm, true);


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
		{ .compatible = "tianma,nlb150xg01" },
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