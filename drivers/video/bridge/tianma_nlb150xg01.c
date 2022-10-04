/* Heavy driver */

#define LOG_DEBUG

#include <common.h>
#include <dm.h>
#include <mipi_dsi.h>
#include <panel.h>
#include <asm/gpio.h>
#include <dm/device_compat.h>
#include <linux/delay.h>
#include <power/regulator.h>

#include <i2c.h>
#include <video_bridge.h>
#include <log.h>
#include <errno.h>

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

DECLARE_GLOBAL_DATA_PTR;

struct tianma_panel_priv {
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

static int tianma_write(struct udevice *dev, uint8_t addr,
						  uint8_t value)
{
	struct dm_i2c_chip *chip = dev_get_parent_platdata(dev);
	int ret;

	ret =  i2c_write(chip->chip_addr, addr, 1, value, 1);
	if (ret) {
		debug("%s: write failed, reg=%#x, value=%#x, ret=%d\n",
		      addr, value, ret);
		return ret;
	}

	return 0;
}
static int tianma_read(struct udevice *dev, uint8_t addr,
						  uint8_t *value)
{
	struct dm_i2c_chip *chip = dev_get_parent_platdata(dev);
	int ret;
	
	ret =  i2c_read(chip->chip_addr, addr, 1, value, 1);
	if (ret) {
		debug("%s: read failed, reg=%#x, value=%#x, ret=%d\n",
		      addr, value, ret);
		return ret;
	}

	return 0;
}

static int tianma_attach(struct udevice *dev)
{
	struct mipi_dsi_panel_plat *plat = dev_get_platdata(dev);
	struct mipi_dsi_device *device; 
	struct mipi_dsi_host *host_p;  	
	int ret, i, len;
	u8 val=0;
	bool pll_en_flag = false;
	u32 hback_porch, hsync_len, hfront_porch, hactive, htime1, htime2;
	u32 vback_porch, vsync_len, vfront_porch, vactive, vtime1, vtime2;

	device = plat->device;
	
	device->channel = 0;
	device->name = "heavy_driver";
	device->lanes = 2;
	device->format = MIPI_DSI_FMT_RGB888;
	device->mode_flags = MIPI_DSI_MODE_VIDEO_BURST;
	
	//add code to detect remote dsi node and add it to host node.
	host_p = fdt_getprop(gd->fdt_blob, dev_of_offset(dev), "host-dsi-node",
						&len);
	if(len == NULL)
	{
		log_info("tianma: (attach) Failed to get host node.\n");
		return -EINVAL;
	}
	device->host = host_p;
	
	ret = mipi_dsi_attach(device);
	if (ret < 0){
		log_info("tianma: (attach) Failed to attach mipi dsi device.\n");
		return ret;
	}
	
	log_info("tianma: (attach) mipi dsi device attached successfully.\n");
	
	/* Disable PLL */
	tianma_write(dev, REG_RC_PLL_EN, 0x00);   //0d
	mdelay(1);
	
	/* Reference clock derived from DSI link clock. */
	tianma_write(dev, REG_RC_LVDS_PLL, 0x05);  //0a
	tianma_write(dev, REG_DSI_CLK, 0x4E);      //12
	tianma_write(dev, REG_RC_DSI_CLK, 0x30);   //0b
	tianma_write(dev, REG_RC_PLL_EN, 0x00);    //0d
	
	/* Set number of DSI lanes and LVDS link config. */
	tianma_write(dev, REG_DSI_LANE, 0x30);     //10
	
	/* No equalization. */
	tianma_write(dev, REG_DSI_EQ, 0x00);       //11

	
	tianma_write(dev, REG_LVDS_FMT, 0x78);       //18
	tianma_write(dev, REG_LVDS_VCOM, 0x00);     //19
	tianma_write(dev, REG_LVDS_LANE, 0x00);     //1a
	tianma_write(dev, REG_LVDS_CM, 0x00);       //1b
	
	
	hback_porch      = 155;  //default_timing->hback_porch.typ;
	hsync_len        = 10;   //default_timing->hsync_len.typ;
	vback_porch      = 16;   //default_timing->vback_porch.typ;
	vsync_len        = 6;    //default_timing->vsync_len.typ;
	hfront_porch     = 155;  //default_timing->hfront_porch.typ;	
	hactive          = 1024; //default_timing->hactive.typ;
	vfront_porch     = 16;   //default_timing->vfront_porch.typ;
	vactive          = 768;  //default_timing->vactive.typ;

	
	tianma_write(dev, REG_VID_CHA_ACTIVE_LINE_LENGTH_LOW, (u8)(hactive&0xff));            //20
	tianma_write(dev, REG_VID_CHA_ACTIVE_LINE_LENGTH_HIGH, (u8)((hactive>>8)&0xff));      //21
	tianma_write(dev, REG_VID_CHA_VERTICAL_DISPLAY_SIZE_LOW, (u8)(vactive&0xff));         //24
	tianma_write(dev, REG_VID_CHA_VERTICAL_DISPLAY_SIZE_HIGH, (u8)((vactive>>8)&0xff));   //25
	
	
	/* 32 + 1 pixel clock to ensure proper operation */
	tianma_write(dev, REG_VID_CHA_SYNC_DELAY_LOW, 0xff);               //28
	tianma_write(dev, REG_VID_CHA_SYNC_DELAY_HIGH, 0x00);              //29
	tianma_write(dev, REG_VID_CHA_HSYNC_PULSE_WIDTH_LOW, (u8)(hsync_len&0xff));             //2c
	tianma_write(dev, REG_VID_CHA_HSYNC_PULSE_WIDTH_HIGH, (u8)((hsync_len>>8)&0xff));       //2d
	tianma_write(dev, REG_VID_CHA_VSYNC_PULSE_WIDTH_LOW, (u8)(vsync_len&0xff));             //30
	tianma_write(dev, REG_VID_CHA_VSYNC_PULSE_WIDTH_HIGH, (u8)((vsync_len>>8)&0xff));       //31
	tianma_write(dev, REG_VID_CHA_HORIZONTAL_BACK_PORCH, (u8)(hback_porch&0xff));           //34
	tianma_write(dev, REG_VID_CHA_VERTICAL_BACK_PORCH, (u8)(vback_porch&0xff));             //36
	tianma_write(dev, REG_VID_CHA_HORIZONTAL_FRONT_PORCH, (u8)(hfront_porch&0xff));         //38
	tianma_write(dev, REG_VID_CHA_VERTICAL_FRONT_PORCH, (u8)(vfront_porch&0xff));           //3a
	tianma_write(dev, REG_VID_CHA_TEST_PATTERN, 0x00);                 //3c
	
	/* Enable PLL */
	tianma_write(dev, REG_RC_PLL_EN, 0x01);    
	
	for(i=0; i<10; i++)
	{
		mdelay(1);
		val=0;
		tianma_read(dev, REG_RC_LVDS_PLL, &val);
		if(val & 0x80 == 0x80)
		{
			pll_en_flag = true;
			break;
		}
	}
	
	if (pll_en_flag==false) {
		log_info("tianma: (attach) failed to lock PLL \n");
		/* On failure, disable PLL again and exit. */
		tianma_write(dev, REG_RC_PLL_EN, 0x00);
		return;
	}
	/* Trigger reset after CSR register update. */
	tianma_write(dev, REG_RC_RESET, 0x01);
	mdelay(10);
	
	/* Clear all errors that got asserted during initialization. */
	val=0;
	tianma_read(dev, REG_IRQ_STAT, &val);
	tianma_write(dev, REG_IRQ_STAT, val);
	
}

static int tianma_ofdata_to_platdata(struct udevice *dev)
{
	struct tianma_panel_priv *priv = dev_get_priv(dev);
	int ret;

	if (IS_ENABLED(CONFIG_DM_REGULATOR)) {
		ret =  device_get_supply_regulator(dev, "power-supply",
						   &priv->reg);
		if (ret && ret != -ENOENT) {
			log_info("tianma: (ofdata) cannot get power supply\n");
			dev_err(dev, "Warning: cannot get power supply\n");
			return ret;
		}
		log_info("tianma: (ofdata) regulator found.\n");
	}

	ret = gpio_request_by_name(dev, "enable-gpios", 0, &priv->enable,
				   GPIOD_IS_OUT_ACTIVE);
	if (ret) {
		dev_err(dev, "warning: cannot get reset GPIO\n");
		log_info("tianma: (ofdata) cannot get enable GPIO.\n");
		if (ret != -ENOENT)
			return ret;	
	}
	
	log_info("tianma: (ofdata) of data parsed successfully.\n");

	return 0;
}
	
static int tianma_probe(struct udevice *dev)
{
	struct tianma_panel_priv *priv = dev_get_priv(dev);
	int ret;
	
	log_info("%s\n", __func__);
	if (device_get_uclass_id(dev->parent) != UCLASS_I2C)
	{
		log_info("tianma: (probe) Uclass i2c not found.\n");
		return -EPROTONOSUPPORT;
	}
	log_info("tianma: (probe) Uclass i2c found.\n");
	
	if (IS_ENABLED(CONFIG_DM_REGULATOR) && priv->reg) {
		log_info("tianma: (probe) regulator found.\n");
		dev_dbg(dev, "enable regulator '%s'\n", priv->reg->name);
		ret = regulator_set_enable(priv->reg, true);
		if (ret)
		{ 
			log_info("tianma: (probe) regulator not found.\n");
			return ret;
		}
	}
	
    log_info("tianma: Probing successful.\n");
	return 0;
	
}

struct video_bridge_ops tianma_ops = {
	.attach                = tianma_attach,
};

static const struct udevice_id tianma_ids[] = {
	{ .compatible = "tianma,nlb150xg01-01", },
	{ }
};

U_BOOT_DRIVER(tianma_nlb150xg01) = {
	.name	              = "tianma_nlb150xg01-01",
	.id	                  = UCLASS_VIDEO_BRIDGE,
	.of_match             = tianma_ids,
	.probe	              = tianma_probe,
	.ops	              = &tianma_ops,
	.ofdata_to_platdata	  = tianma_ofdata_to_platdata,
	.priv_auto_alloc_size	= sizeof(struct tianma_panel_priv),
};			
