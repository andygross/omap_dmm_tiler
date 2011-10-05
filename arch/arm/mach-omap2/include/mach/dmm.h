/*
 * dmm.h
 *
 * DMM driver support functions for TI OMAP processors.
 *
 * Copyright (C) 2009-2010 Texas Instruments, Inc.
 *
 * This package is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * THIS PACKAGE IS PROVIDED ``AS IS'' AND WITHOUT ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, WITHOUT LIMITATION, THE IMPLIED
 * WARRANTIES OF MERCHANTIBILITY AND FITNESS FOR A PARTICULAR PURPOSE.
 */

#ifndef DMM_H
#define DMM_H

#define DMM_BASE 0x4E000000
#define DMM_SIZE 0x800

#define DMM_REVISION          0x000
#define DMM_HWINFO            0x004
#define DMM_LISA_HWINFO       0x008
#define DMM_DMM_SYSCONFIG     0x010
#define DMM_LISA_LOCK         0x01C
#define DMM_LISA_MAP__0       0x040
#define DMM_LISA_MAP__1       0x044
#define DMM_TILER_HWINFO      0x208
#define DMM_TILER_OR__0       0x220
#define DMM_TILER_OR__1       0x224
#define DMM_PAT_HWINFO        0x408
#define DMM_PAT_GEOMETRY      0x40C
#define DMM_PAT_CONFIG        0x410
#define DMM_PAT_VIEW__0       0x420
#define DMM_PAT_VIEW__1       0x424
#define DMM_PAT_VIEW_MAP__0   0x440
#define DMM_PAT_VIEW_MAP_BASE 0x460
#define DMM_PAT_IRQ_EOI       0x478
#define DMM_PAT_IRQSTATUS_RAW 0x480
#define DMM_PAT_IRQSTATUS     0x490
#define DMM_PAT_IRQENABLE_SET 0x4A0
#define DMM_PAT_IRQENABLE_CLR 0x4B0
#define DMM_PAT_STATUS__0     0x4C0
#define DMM_PAT_STATUS__1     0x4C4
#define DMM_PAT_DESCR__0      0x500
#define DMM_PAT_DESCR__1      0x510
#define DMM_PAT_AREA__0       0x504
#define DMM_PAT_AREA__1	      0x514
#define DMM_PAT_CTRL__0       0x508
#define DMM_PAT_CTRL__1       0x518
#define DMM_PAT_DATA__0       0x50C
#define DMM_PAT_DATA__1       0x51C
#define DMM_PEG_HWINFO        0x608
#define DMM_PEG_PRIO          0x620
#define DMM_PEG_PRIO_PAT      0x640

#define DMM_IRQSTAT_DST			1<<0
#define DMM_IRQSTAT_LST			1<<1
#define DMM_IRQSTAT_ERR_INV_DSC		1<<2
#define DMM_IRQSTAT_ERR_INV_DATA	1<<3
#define DMM_IRQSTAT_ERR_UPD_AREA	1<<4
#define DMM_IRQSTAT_ERR_UPD_CTRL	1<<5
#define DMM_IRQSTAT_ERR_UPD_DATA	1<<6
#define DMM_IRQSTAT_ERR_LUT_MISS	1<<7

#define DMM_IRQSTAT_ERR_MASK	(DMM_IRQ_STAT_ERR_INV_DSC | \
				DMM_IRQ_STAT_ERR_INV_DATA | \
				DMM_IRQ_STAT_ERR_UPD_AREA | \
				DMM_IRQ_STAT_ERR_UPD_CTRL | \
				DMM_IRQ_STAT_ERR_UPD_DATA | \
				DMM_IRQ_STAT_ERR_LUT_MISS )

enum tiler_mode {
	TILER_MODE_8BPP = 0,
	TILER_MODE_16BPP,
	TILER_MODE_32BPP,
	TILER_MODE_PAGE,
	TILER_MODE_INVALID
};

/**
 * PAT refill programming mode.
 */
enum pat_mode {
	MANUAL,
	AUTO
};

struct omap_dmm_iommu_drvdata {
	void __iomem *base;
	int irq;

};

/**
 * platform data
 */
struct omap_dmm_platform_data {
	const char *oh_name;
	void __iomem *base;
	int irq;
	int num_engines;
	struct omap_dmm_aperture *apertures;
};

void omap_dmm_init(void);

#endif
