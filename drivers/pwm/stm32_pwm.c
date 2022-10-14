// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright (C) 2018, STMicroelectronics - All Rights Reserved
 * Author(s): Patrice Chotard, <patrice.chotard@foss.st.com> for STMicroelectronics.
 */

#define LOG_CATEGORY UCLASS_TIMER

#include <common.h>
#include <clk.h>
#include <dm.h>
#include <fdtdec.h>
#include <pwm.h>
#include <dm/device_compat.h>
#include <linux/bitops.h>

#include <asm/io.h>

/* Timer control1 register  */
#define CR1_CEN			BIT(0)
#define CR1_ARPE		BIT(7)

/* Event Generation Register register  */
#define EGR_UG			BIT(0)

/* Auto reload register for free running config */
#define GPT_FREE_RUNNING	0xFFFFFFFF

//timer registers as mentioned in reference manual 
//they are in sequence of offsets 
struct stm32_pwm_regs {
	u32 cr1;
	u32 cr2;
	u32 smcr;
	u32 dier;
	u32 sr;
	u32 egr;
	u32 ccmr1;
	u32 ccmr2;
	u32 ccer;
	u32 cnt;
	u32 psc;
	u32 arr;
	u32 reserved;
	u32 ccr1;
	u32 ccr2;
	u32 ccr3;
	u32 ccr4;
	u32 reserved1;
	u32 dcr;
	u32 dmar;
	u32 tim2_5_or;
};

//holds the base register addresses of all registers 
struct stm32_pwm_priv {
	struct stm32_pwm_regs *base;
};


static u64 stm32_pwm_get_count(struct udevice *dev)
{
	//gets base registers address
	struct stm32_pwm_priv *priv = dev_get_priv(dev);
	
	//unecessary. just creats a local variable to store 
	//the base register addresses
	struct stm32_pwm_regs *regs = priv->base;

	//read the count value from CNT register
	return 0;
	//readl(&regs->cnt);
}

static int stm32_pwm_probe(struct udevice *dev)
{
	log_info("PWM: entered pwm probe \n");
	//gets the input clock for this timer block
	struct pwm_dev_priv *uc_priv = dev_get_uclass_priv(dev);
	
	//gets the base register addresses and assigns it to a regs 
	//local variable
	struct stm32_pwm_priv *priv = dev_get_priv(dev);
	struct stm32_pwm_regs *regs;
	struct clk clk;
	
	//physical register address struct
	fdt_addr_t addr;
	int ret;
	u32 rate, psc;
	
	//reads the reg property from the device tree
	addr = dev_read_addr(dev);
	if (addr == FDT_ADDR_T_NONE)
		return -EINVAL;
	
	//gets the base register address from struct.this is the base 
	//address of the timer block registers (CR1)
	priv->base = (struct stm32_pwm_regs *)addr;

	//important step. this gets the clock for the peripheral 
	//from the defined sequence of clocks in device tree.
	//returs the address to clk vaariable 
	ret = clk_get_by_index(dev, 0, &clk);
	if (ret < 0)
		return ret;

	//enables the clock source
	ret = clk_enable(&clk);
	if (ret) {
		dev_err(dev, "failed to enable clock\n");
		return ret;
	}

	regs = priv->base;

	/* Stop the timer */
	//clrbits_le32(&regs->cr1, CR1_CEN);

	/* get timer clock */
	//rate = clk_get_rate(&clk);

	/* we set timer prescaler to obtain a 1MHz timer counter frequency */
	//psc = (rate / CONFIG_SYS_HZ_CLOCK) - 1;
	//writel(psc, &regs->psc);

	/* Set timer frequency to 1MHz */
	//uc_priv->clock_rate = CONFIG_SYS_HZ_CLOCK;

	/* Configure timer for auto-reload */
	//setbits_le32(&regs->cr1, CR1_ARPE);

	/* load value for auto reload */
	//writel(GPT_FREE_RUNNING, &regs->arr);

	/* start timer */
	//setbits_le32(&regs->cr1, CR1_CEN);

	/* Update generation */
	//setbits_le32(&regs->egr, EGR_UG);

	return 0;
}
static int stm32_pwm_set_config(struct udevice *dev, uint channel, uint period_ns,
		   uint duty_ns)
{
		log_info("PWM: entered pwm set config \n");
		return 0;
}

static int stm32_pwm_set_enable(struct udevice *dev, uint channel, bool enable)
{
		log_info("PWM: entered pwm seet enable \n");
		return 0;
}

static const struct pwm_ops stm32_pwm_ops = {
	.set_config	= stm32_pwm_set_config,
	.set_enable	= stm32_pwm_set_enable,
};

static const struct udevice_id stm32_pwm_ids[] = {
	{ .compatible = "st,stm32-pwm" },
	{}
};

U_BOOT_DRIVER(stm32_pwm) = {
	.name = "stm32_pwm",
	.id = UCLASS_PWM,
	.of_match = stm32_pwm_ids,
	.priv_auto_alloc_size = sizeof(struct stm32_pwm_priv),
	.probe = stm32_pwm_probe,
	.ops = &stm32_pwm_ops,
};
