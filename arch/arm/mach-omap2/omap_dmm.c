/*
 * DMM driver support functions for TI OMAP processors.
 *
 * Copyright (C) 2011 Texas Instruments Incorporated - http://www.ti.com/
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation version 2.
 *
 * This program is distributed "as is" WITHOUT ANY WARRANTY of any
 * kind, whether express or implied; without even the implied warranty
 * of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */
#include <linux/io.h>
#include <linux/init.h>
#include <linux/module.h>
#include <mach/dmm.h>
#include <plat/omap_device.h>
#include <plat/omap_hwmod.h>
#include <linux/errno.h>
#include <linux/err.h>

static struct container containers[] = {
	[0] = {
		.base = (void *)0x60000000,
		.format = TILER_MODE_8BPP,
	},
	[1] = {
		.base = (void *)0x68000000,
		.format = TILER_MODE_16BPP,
	},
	[2] = {
		.base = (void *)0x70000000,
		.format = TILER_MODE_32BPP,
	},
	[3] = {
		.base = (void *)0x78000000,
		.format = TILER_MODE_PAGE,
	},
};

static struct omap_dmm_platform_data dmm_data = {
	.oh_name = "dmm",
	.num_engines = 2,
	.containers = containers,
	.num_containers = ARRAY_SIZE(containers),
};

static struct omap_device_pm_latency omap_dmm_latency[] = {
	[0] = {
		.deactivate_func = omap_device_idle_hwmods,
		.activate_func = omap_device_enable_hwmods,
		.flags = OMAP_DEVICE_LATENCY_AUTO_ADJUST,
	},
};


void __init omap_dmm_init(void)
{
	struct omap_hwmod *oh = NULL;
	struct omap_device *od = NULL;

	oh = omap_hwmod_lookup(dmm_data.oh_name);
	if (!oh)
		return;

	dmm_data.base = omap_hwmod_get_mpu_rt_va(oh);
	dmm_data.irq = oh->mpu_irqs[0].irq;

	if (!dmm_data.base) {
		pr_err("Failed to get DMM base initialized\n");
		return;
	}

	printk(KERN_ERR "create omap dmm device\n");

	od = omap_device_build(dmm_data.oh_name, -1, oh, &dmm_data,
				sizeof(dmm_data), omap_dmm_latency,
				ARRAY_SIZE(omap_dmm_latency), false);

	return;
}
