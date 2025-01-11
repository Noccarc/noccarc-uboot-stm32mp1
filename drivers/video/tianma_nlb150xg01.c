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
		.pixelclock.typ		= 65000000,
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
		
		u8 val=0,i=0,k=0;
		bool pll_en_flag = false;
		u32 hback_porch, hsync_len, hfront_porch, hactive, htime1, htime2;
		u32 vback_porch, vsync_len, vfront_porch, vactive, vtime1, vtime2;

		ret1 = i2c_get_chip_for_busnum(3, 0x2c,
						  1, &dev1);
		if (ret1) {
			log_info("driver %s: Cannot find udev for a bus %d\n", __func__,
				   3);
			return ret1;
		}
		
		dm_i2c_reg_write(dev1, REG_RC_PLL_EN, 0x00);   //0d
		mdelay(1);
		/* Reference clock derived from DSI link clock. */
		dm_i2c_reg_write(dev1, REG_RC_LVDS_PLL, 0x05);  //0a
		dm_i2c_reg_write(dev1, REG_DSI_CLK, 0x48);      //12
		dm_i2c_reg_write(dev1, REG_RC_DSI_CLK, 0x28);   //0b
		dm_i2c_reg_write(dev1, REG_RC_PLL_EN, 0x00);    //0d
		/* Set number of DSI lanes and LVDS link config. */
		dm_i2c_reg_write(dev1, REG_DSI_LANE, 0x30);     //10
		/* No equalization. */
		dm_i2c_reg_write(dev1, REG_DSI_EQ, 0x00);       //11
		dm_i2c_reg_write(dev1, REG_LVDS_FMT, 0x78);       //18
		dm_i2c_reg_write(dev1, REG_LVDS_VCOM, 0x00);     //19
		dm_i2c_reg_write(dev1, REG_LVDS_LANE, 0x00);     //1a	
		dm_i2c_reg_write(dev1, REG_LVDS_CM, 0x00);       //1b
			
		
		hback_porch      = 155;  //default_timing->hback_porch.typ;
		hsync_len        = 10;   //default_timing->hsync_len.typ;
		vback_porch      = 16;   //default_timing->vback_porch.typ;
		vsync_len        = 6;    //default_timing->vsync_len.typ;
		hfront_porch     = 155;  //default_timing->hfront_porch.typ;	
		hactive          = 1024; //default_timing->hactive.typ;
		vfront_porch     = 16;   //default_timing->vfront_porch.typ;
		vactive          = 768;  //default_timing->vactive.typ;

		
		dm_i2c_reg_write(dev1, REG_VID_CHA_ACTIVE_LINE_LENGTH_LOW, (u8)(hactive&0xff));            //20		
		dm_i2c_reg_write(dev1, REG_VID_CHA_ACTIVE_LINE_LENGTH_HIGH, (u8)((hactive>>8)&0xff));      //21	
		dm_i2c_reg_write(dev1, REG_VID_CHA_VERTICAL_DISPLAY_SIZE_LOW, (u8)(vactive&0xff));         //24
		dm_i2c_reg_write(dev1, REG_VID_CHA_VERTICAL_DISPLAY_SIZE_HIGH, (u8)((vactive>>8)&0xff));   //25
		/* 32 + 1 pixel clock to ensure proper operation */
		dm_i2c_reg_write(dev1, REG_VID_CHA_SYNC_DELAY_LOW, 0xff);               //28
		dm_i2c_reg_write(dev1, REG_VID_CHA_SYNC_DELAY_HIGH, 0x00);              //29
		dm_i2c_reg_write(dev1, REG_VID_CHA_HSYNC_PULSE_WIDTH_LOW, (u8)(hsync_len&0xff));             //2c
		dm_i2c_reg_write(dev1, REG_VID_CHA_HSYNC_PULSE_WIDTH_HIGH, (u8)((hsync_len>>8)&0xff));       //2d
		dm_i2c_reg_write(dev1, REG_VID_CHA_VSYNC_PULSE_WIDTH_LOW, (u8)(vsync_len&0xff));             //30
			
		dm_i2c_reg_write(dev1, REG_VID_CHA_VSYNC_PULSE_WIDTH_HIGH, (u8)((vsync_len>>8)&0xff));       //31
		dm_i2c_reg_write(dev1, REG_VID_CHA_HORIZONTAL_BACK_PORCH, (u8)(hback_porch&0xff));           //34
		dm_i2c_reg_write(dev1, REG_VID_CHA_VERTICAL_BACK_PORCH, (u8)(vback_porch&0xff));             //36
		dm_i2c_reg_write(dev1, REG_VID_CHA_HORIZONTAL_FRONT_PORCH, (u8)(hfront_porch&0xff));         //38
		dm_i2c_reg_write(dev1, REG_VID_CHA_VERTICAL_FRONT_PORCH, (u8)(vfront_porch&0xff));           //3a
		dm_i2c_reg_write(dev1, REG_VID_CHA_TEST_PATTERN, 0x00);                 //3c
		
		/* Enable PLL */
		dm_i2c_reg_write(dev1, REG_RC_PLL_EN, 0x01);    
		
		for(i=0; i<10; i++)
		{
			mdelay(1);
			val=0;
			val = dm_i2c_reg_read(dev1, REG_RC_LVDS_PLL);
			if(val & 0x80 == 0x80)
			{
				pll_en_flag = true;
				break;
			}
		}
		
		if (pll_en_flag==false) {
			log_info("tianma: (attach) failed to lock PLL \n");
			/* On failure, disable PLL again and exit. */
			dm_i2c_reg_write(dev1, REG_RC_PLL_EN, 0x00);
			return -EINVAL;
		}
		/* Trigger reset after CSR register update. */
		dm_i2c_reg_write(dev1, REG_RC_RESET, 0x01);
		mdelay(10);
		
		/* Clear all errors that got asserted during initialization. */
		val=0;
		val = dm_i2c_reg_read(dev1, REG_IRQ_STAT);
		dm_i2c_reg_write(dev1, REG_IRQ_STAT, val);
		
		return 0;
	}

	static int otm8009a_panel_enable_backlight(struct udevice *dev)
	{
		struct mipi_dsi_panel_plat *plat = dev_get_platdata(dev);
		struct mipi_dsi_device *device = plat->device;
		//struct otm8009a_panel_priv *priv = dev_get_priv(dev);
		int ret = 0;
		
		log_info("driver: Entered enable backlight \n");
		
		ret = mipi_dsi_attach(device);
		if (ret < 0)
			return ret;

		ret = otm8009a_init_sequence(dev);
		if (ret)
			return ret;
		
		mdelay(200);

		// dm_gpio_set_value(&priv->backlight_en, true);

		// dm_gpio_set_value(&priv->backlight_pwm, true);
		ret = backlight_enable(dev);
		
		/*if (ret){
			log_info("driver: set enable failed \n");
			return ret;
		}*/
		
		// mdelay(100);
		// dm_gpio_set_value(&priv->backlight_gpio, true);
		

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

		log_info("driver: Entered of to plat \n");
		
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

		// ret = gpio_request_by_name(dev, "backlight-enable", 0, &priv->backlight_en,
		// 			   GPIOD_IS_OUT_ACTIVE);
		// if (ret) {
		// 	dev_err(dev, "warning: cannot get backlight-enable GPIO\n");
		// 	log_info("driver: backlight-enable gpio not found\n");
		// 	if (ret != -ENOENT)
		// 		return ret;	
		// }

		// ret = gpio_request_by_name(dev, "backlight-pwm", 0, &priv->backlight_pwm,
		// 			   GPIOD_IS_OUT_ACTIVE);
		// if (ret) {
		// 	dev_err(dev, "warning: cannot get backlight-pwm GPIO\n");
		// 	log_info("driver: backlight-pwm gpio not found\n");
		// 	if (ret != -ENOENT)
		// 		return ret;	
		// }
		
		ret = uclass_get_device_by_phandle(UCLASS_PANEL_BACKLIGHT, dev,
						   "backlight", &priv->backlight);
		if (ret) {
			log_info("%s: Cannot get backlight: ret=%d\n", __func__, ret);
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
