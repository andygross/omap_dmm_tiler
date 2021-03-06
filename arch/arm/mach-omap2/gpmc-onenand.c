/*
 * linux/arch/arm/mach-omap2/gpmc-onenand.c
 *
 * Copyright (C) 2006 - 2009 Nokia Corporation
 * Contacts:	Juha Yrjola
 *		Tony Lindgren
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/string.h>
#include <linux/kernel.h>
#include <linux/platform_device.h>
#include <linux/mtd/onenand_regs.h>
#include <linux/io.h>
#include <linux/platform_data/mtd-onenand-omap2.h>
#include <linux/err.h>

#include <asm/mach/flash.h>

#include "gpmc.h"
#include "soc.h"
#include "gpmc-onenand.h"

#define	ONENAND_IO_SIZE	SZ_128K

#define	ONENAND_FLAG_SYNCREAD	(1 << 0)
#define	ONENAND_FLAG_SYNCWRITE	(1 << 1)
#define	ONENAND_FLAG_HF		(1 << 2)
#define	ONENAND_FLAG_VHF	(1 << 3)

static unsigned onenand_flags;
static unsigned latency;
static int fclk_offset;

static struct omap_onenand_platform_data *gpmc_onenand_data;

static struct resource gpmc_onenand_resource = {
	.flags		= IORESOURCE_MEM,
};

static struct platform_device gpmc_onenand_device = {
	.name		= "omap2-onenand",
	.id		= -1,
	.num_resources	= 1,
	.resource	= &gpmc_onenand_resource,
};

static struct gpmc_timings omap2_onenand_calc_async_timings(void)
{
	struct gpmc_timings t;

	const int t_cer = 15;
	const int t_avdp = 12;
	const int t_aavdh = 7;
	const int t_ce = 76;
	const int t_aa = 76;
	const int t_oe = 20;
	const int t_cez = 20; /* max of t_cez, t_oez */
	const int t_ds = 30;
	const int t_wpl = 40;
	const int t_wph = 30;

	memset(&t, 0, sizeof(t));
	t.sync_clk = 0;
	t.cs_on = 0;
	t.adv_on = 0;

	/* Read */
	t.adv_rd_off = gpmc_round_ns_to_ticks(max_t(int, t_avdp, t_cer));
	t.oe_on  = t.adv_rd_off + gpmc_round_ns_to_ticks(t_aavdh);
	t.access = t.adv_on + gpmc_round_ns_to_ticks(t_aa);
	t.access = max_t(int, t.access, t.cs_on + gpmc_round_ns_to_ticks(t_ce));
	t.access = max_t(int, t.access, t.oe_on + gpmc_round_ns_to_ticks(t_oe));
	t.oe_off = t.access + gpmc_round_ns_to_ticks(1);
	t.cs_rd_off = t.oe_off;
	t.rd_cycle  = t.cs_rd_off + gpmc_round_ns_to_ticks(t_cez);

	/* Write */
	t.adv_wr_off = t.adv_rd_off;
	t.we_on  = t.oe_on;
	if (cpu_is_omap34xx()) {
		t.wr_data_mux_bus = t.we_on;
		t.wr_access = t.we_on + gpmc_round_ns_to_ticks(t_ds);
	}
	t.we_off = t.we_on + gpmc_round_ns_to_ticks(t_wpl);
	t.cs_wr_off = t.we_off + gpmc_round_ns_to_ticks(t_wph);
	t.wr_cycle  = t.cs_wr_off + gpmc_round_ns_to_ticks(t_cez);

	return t;
}

static int gpmc_set_async_mode(int cs, struct gpmc_timings *t)
{
	/* Configure GPMC for asynchronous read */
	gpmc_cs_write_reg(cs, GPMC_CS_CONFIG1,
			  GPMC_CONFIG1_DEVICESIZE_16 |
			  GPMC_CONFIG1_MUXADDDATA);

	return gpmc_cs_set_timings(cs, t);
}

static void omap2_onenand_set_async_mode(void __iomem *onenand_base)
{
	u32 reg;

	/* Ensure sync read and sync write are disabled */
	reg = readw(onenand_base + ONENAND_REG_SYS_CFG1);
	reg &= ~ONENAND_SYS_CFG1_SYNC_READ & ~ONENAND_SYS_CFG1_SYNC_WRITE;
	writew(reg, onenand_base + ONENAND_REG_SYS_CFG1);
}

static void set_onenand_cfg(void __iomem *onenand_base)
{
	u32 reg;

	reg = readw(onenand_base + ONENAND_REG_SYS_CFG1);
	reg &= ~((0x7 << ONENAND_SYS_CFG1_BRL_SHIFT) | (0x7 << 9));
	reg |=	(latency << ONENAND_SYS_CFG1_BRL_SHIFT) |
		ONENAND_SYS_CFG1_BL_16;
	if (onenand_flags & ONENAND_FLAG_SYNCREAD)
		reg |= ONENAND_SYS_CFG1_SYNC_READ;
	else
		reg &= ~ONENAND_SYS_CFG1_SYNC_READ;
	if (onenand_flags & ONENAND_FLAG_SYNCWRITE)
		reg |= ONENAND_SYS_CFG1_SYNC_WRITE;
	else
		reg &= ~ONENAND_SYS_CFG1_SYNC_WRITE;
	if (onenand_flags & ONENAND_FLAG_HF)
		reg |= ONENAND_SYS_CFG1_HF;
	else
		reg &= ~ONENAND_SYS_CFG1_HF;
	if (onenand_flags & ONENAND_FLAG_VHF)
		reg |= ONENAND_SYS_CFG1_VHF;
	else
		reg &= ~ONENAND_SYS_CFG1_VHF;
	writew(reg, onenand_base + ONENAND_REG_SYS_CFG1);
}

static int omap2_onenand_get_freq(struct omap_onenand_platform_data *cfg,
				  void __iomem *onenand_base)
{
	u16 ver = readw(onenand_base + ONENAND_REG_VERSION_ID);
	int freq;

	switch ((ver >> 4) & 0xf) {
	case 0:
		freq = 40;
		break;
	case 1:
		freq = 54;
		break;
	case 2:
		freq = 66;
		break;
	case 3:
		freq = 83;
		break;
	case 4:
		freq = 104;
		break;
	default:
		freq = 54;
		break;
	}

	return freq;
}

static struct gpmc_timings
omap2_onenand_calc_sync_timings(struct omap_onenand_platform_data *cfg,
				int freq)
{
	struct gpmc_timings t;
	const int t_cer  = 15;
	const int t_avdp = 12;
	const int t_cez  = 20; /* max of t_cez, t_oez */
	const int t_ds   = 30;
	const int t_wpl  = 40;
	const int t_wph  = 30;
	int min_gpmc_clk_period, t_ces, t_avds, t_avdh, t_ach, t_aavdh, t_rdyo;
	u32 reg;
	int div, fclk_offset_ns, gpmc_clk_ns;
	int ticks_cez;
	int cs = cfg->cs;

	if (cfg->flags & ONENAND_SYNC_READ)
		onenand_flags = ONENAND_FLAG_SYNCREAD;
	else if (cfg->flags & ONENAND_SYNC_READWRITE)
		onenand_flags = ONENAND_FLAG_SYNCREAD | ONENAND_FLAG_SYNCWRITE;

	switch (freq) {
	case 104:
		min_gpmc_clk_period = 9600; /* 104 MHz */
		t_ces   = 3;
		t_avds  = 4;
		t_avdh  = 2;
		t_ach   = 3;
		t_aavdh = 6;
		t_rdyo  = 6;
		break;
	case 83:
		min_gpmc_clk_period = 12000; /* 83 MHz */
		t_ces   = 5;
		t_avds  = 4;
		t_avdh  = 2;
		t_ach   = 6;
		t_aavdh = 6;
		t_rdyo  = 9;
		break;
	case 66:
		min_gpmc_clk_period = 15000; /* 66 MHz */
		t_ces   = 6;
		t_avds  = 5;
		t_avdh  = 2;
		t_ach   = 6;
		t_aavdh = 6;
		t_rdyo  = 11;
		break;
	default:
		min_gpmc_clk_period = 18500; /* 54 MHz */
		t_ces   = 7;
		t_avds  = 7;
		t_avdh  = 7;
		t_ach   = 9;
		t_aavdh = 7;
		t_rdyo  = 15;
		onenand_flags &= ~ONENAND_FLAG_SYNCWRITE;
		break;
	}

	div = gpmc_calc_divider(min_gpmc_clk_period);
	gpmc_clk_ns = gpmc_ticks_to_ns(div);
	if (gpmc_clk_ns < 15) /* >66Mhz */
		onenand_flags |= ONENAND_FLAG_HF;
	else
		onenand_flags &= ~ONENAND_FLAG_HF;
	if (gpmc_clk_ns < 12) /* >83Mhz */
		onenand_flags |= ONENAND_FLAG_VHF;
	else
		onenand_flags &= ~ONENAND_FLAG_VHF;
	if (onenand_flags & ONENAND_FLAG_VHF)
		latency = 8;
	else if (onenand_flags & ONENAND_FLAG_HF)
		latency = 6;
	else if (gpmc_clk_ns >= 25) /* 40 MHz*/
		latency = 3;
	else
		latency = 4;

	/* Set synchronous read timings */
	memset(&t, 0, sizeof(t));

	if (div == 1) {
		reg = gpmc_cs_read_reg(cs, GPMC_CS_CONFIG2);
		reg |= (1 << 7);
		gpmc_cs_write_reg(cs, GPMC_CS_CONFIG2, reg);
		reg = gpmc_cs_read_reg(cs, GPMC_CS_CONFIG3);
		reg |= (1 << 7);
		gpmc_cs_write_reg(cs, GPMC_CS_CONFIG3, reg);
		reg = gpmc_cs_read_reg(cs, GPMC_CS_CONFIG4);
		reg |= (1 << 7);
		reg |= (1 << 23);
		gpmc_cs_write_reg(cs, GPMC_CS_CONFIG4, reg);
	} else {
		reg = gpmc_cs_read_reg(cs, GPMC_CS_CONFIG2);
		reg &= ~(1 << 7);
		gpmc_cs_write_reg(cs, GPMC_CS_CONFIG2, reg);
		reg = gpmc_cs_read_reg(cs, GPMC_CS_CONFIG3);
		reg &= ~(1 << 7);
		gpmc_cs_write_reg(cs, GPMC_CS_CONFIG3, reg);
		reg = gpmc_cs_read_reg(cs, GPMC_CS_CONFIG4);
		reg &= ~(1 << 7);
		reg &= ~(1 << 23);
		gpmc_cs_write_reg(cs, GPMC_CS_CONFIG4, reg);
	}

	t.sync_clk = min_gpmc_clk_period;
	t.cs_on = 0;
	t.adv_on = 0;
	fclk_offset_ns = gpmc_round_ns_to_ticks(max_t(int, t_ces, t_avds));
	fclk_offset = gpmc_ns_to_ticks(fclk_offset_ns);
	t.page_burst_access = gpmc_clk_ns;

	/* Read */
	t.adv_rd_off = gpmc_ticks_to_ns(fclk_offset + gpmc_ns_to_ticks(t_avdh));
	t.oe_on = gpmc_ticks_to_ns(fclk_offset + gpmc_ns_to_ticks(t_ach));
	/* Force at least 1 clk between AVD High to OE Low */
	if (t.oe_on <= t.adv_rd_off)
		t.oe_on = t.adv_rd_off + gpmc_round_ns_to_ticks(1);
	t.access = gpmc_ticks_to_ns(fclk_offset + (latency + 1) * div);
	t.oe_off = t.access + gpmc_round_ns_to_ticks(1);
	t.cs_rd_off = t.oe_off;
	ticks_cez = ((gpmc_ns_to_ticks(t_cez) + div - 1) / div) * div;
	t.rd_cycle = gpmc_ticks_to_ns(fclk_offset + (latency + 1) * div +
		     ticks_cez);

	/* Write */
	if (onenand_flags & ONENAND_FLAG_SYNCWRITE) {
		t.adv_wr_off = t.adv_rd_off;
		t.we_on  = 0;
		t.we_off = t.cs_rd_off;
		t.cs_wr_off = t.cs_rd_off;
		t.wr_cycle  = t.rd_cycle;
		if (cpu_is_omap34xx()) {
			t.wr_data_mux_bus = gpmc_ticks_to_ns(fclk_offset +
					gpmc_ps_to_ticks(min_gpmc_clk_period +
					t_rdyo * 1000));
			t.wr_access = t.access;
		}
	} else {
		t.adv_wr_off = gpmc_round_ns_to_ticks(max_t(int,
							t_avdp, t_cer));
		t.we_on  = t.adv_wr_off + gpmc_round_ns_to_ticks(t_aavdh);
		t.we_off = t.we_on + gpmc_round_ns_to_ticks(t_wpl);
		t.cs_wr_off = t.we_off + gpmc_round_ns_to_ticks(t_wph);
		t.wr_cycle  = t.cs_wr_off + gpmc_round_ns_to_ticks(t_cez);
		if (cpu_is_omap34xx()) {
			t.wr_data_mux_bus = t.we_on;
			t.wr_access = t.we_on + gpmc_round_ns_to_ticks(t_ds);
		}
	}

	return t;
}

static int gpmc_set_sync_mode(int cs, struct gpmc_timings *t)
{
	unsigned sync_read = onenand_flags & ONENAND_FLAG_SYNCREAD;
	unsigned sync_write = onenand_flags & ONENAND_FLAG_SYNCWRITE;

	/* Configure GPMC for synchronous read */
	gpmc_cs_write_reg(cs, GPMC_CS_CONFIG1,
			  GPMC_CONFIG1_WRAPBURST_SUPP |
			  GPMC_CONFIG1_READMULTIPLE_SUPP |
			  (sync_read ? GPMC_CONFIG1_READTYPE_SYNC : 0) |
			  (sync_write ? GPMC_CONFIG1_WRITEMULTIPLE_SUPP : 0) |
			  (sync_write ? GPMC_CONFIG1_WRITETYPE_SYNC : 0) |
			  GPMC_CONFIG1_CLKACTIVATIONTIME(fclk_offset) |
			  GPMC_CONFIG1_PAGE_LEN(2) |
			  (cpu_is_omap34xx() ? 0 :
				(GPMC_CONFIG1_WAIT_READ_MON |
				 GPMC_CONFIG1_WAIT_PIN_SEL(0))) |
			  GPMC_CONFIG1_DEVICESIZE_16 |
			  GPMC_CONFIG1_DEVICETYPE_NOR |
			  GPMC_CONFIG1_MUXADDDATA);

	return gpmc_cs_set_timings(cs, t);
}

static int omap2_onenand_setup_async(void __iomem *onenand_base)
{
	struct gpmc_timings t;
	int ret;

	omap2_onenand_set_async_mode(onenand_base);

	t = omap2_onenand_calc_async_timings();

	ret = gpmc_set_async_mode(gpmc_onenand_data->cs, &t);
	if (IS_ERR_VALUE(ret))
		return ret;

	omap2_onenand_set_async_mode(onenand_base);

	return 0;
}

static int omap2_onenand_setup_sync(void __iomem *onenand_base, int *freq_ptr)
{
	int ret, freq = *freq_ptr;
	struct gpmc_timings t;

	if (!freq) {
		/* Very first call freq is not known */
		freq = omap2_onenand_get_freq(gpmc_onenand_data, onenand_base);
		set_onenand_cfg(onenand_base);
	}

	t = omap2_onenand_calc_sync_timings(gpmc_onenand_data, freq);

	ret = gpmc_set_sync_mode(gpmc_onenand_data->cs, &t);
	if (IS_ERR_VALUE(ret))
		return ret;

	set_onenand_cfg(onenand_base);

	*freq_ptr = freq;

	return 0;
}

static int gpmc_onenand_setup(void __iomem *onenand_base, int *freq_ptr)
{
	struct device *dev = &gpmc_onenand_device.dev;
	unsigned l = ONENAND_SYNC_READ | ONENAND_SYNC_READWRITE;
	int ret;

	ret = omap2_onenand_setup_async(onenand_base);
	if (ret) {
		dev_err(dev, "unable to set to async mode\n");
		return ret;
	}

	if (!(gpmc_onenand_data->flags & l))
		return 0;

	ret = omap2_onenand_setup_sync(onenand_base, freq_ptr);
	if (ret)
		dev_err(dev, "unable to set to sync mode\n");
	return ret;
}

void __init gpmc_onenand_init(struct omap_onenand_platform_data *_onenand_data)
{
	int err;

	gpmc_onenand_data = _onenand_data;
	gpmc_onenand_data->onenand_setup = gpmc_onenand_setup;
	gpmc_onenand_device.dev.platform_data = gpmc_onenand_data;

	if (cpu_is_omap24xx() &&
			(gpmc_onenand_data->flags & ONENAND_SYNC_READWRITE)) {
		printk(KERN_ERR "Onenand using only SYNC_READ on 24xx\n");
		gpmc_onenand_data->flags &= ~ONENAND_SYNC_READWRITE;
		gpmc_onenand_data->flags |= ONENAND_SYNC_READ;
	}

	if (cpu_is_omap34xx())
		gpmc_onenand_data->flags |= ONENAND_IN_OMAP34XX;
	else
		gpmc_onenand_data->flags &= ~ONENAND_IN_OMAP34XX;

	err = gpmc_cs_request(gpmc_onenand_data->cs, ONENAND_IO_SIZE,
				(unsigned long *)&gpmc_onenand_resource.start);
	if (err < 0) {
		pr_err("%s: Cannot request GPMC CS\n", __func__);
		return;
	}

	gpmc_onenand_resource.end = gpmc_onenand_resource.start +
							ONENAND_IO_SIZE - 1;

	if (platform_device_register(&gpmc_onenand_device) < 0) {
		pr_err("%s: Unable to register OneNAND device\n", __func__);
		gpmc_cs_free(gpmc_onenand_data->cs);
		return;
	}
}
