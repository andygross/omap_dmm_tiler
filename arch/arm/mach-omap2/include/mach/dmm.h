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
#define DMM_PAT_STATUS__2     0x4C8
#define DMM_PAT_STATUS__3     0x4CC
#define DMM_PAT_DESCR__0      0x500
#define DMM_PAT_AREA__0       0x504
#define DMM_PAT_CTRL__0       0x508
#define DMM_PAT_DATA__0       0x50C
#define DMM_PEG_HWINFO        0x608
#define DMM_PEG_PRIO          0x620
#define DMM_PEG_PRIO_PAT      0x640

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

/**
 * Area definition for DMM physical address translator.
 */
struct pat_area {
	u32 x0:8;
	u32 y0:8;
	u32 x1:8;
	u32 y1:8;
};

/**
 * DMM physical address translator control.
 */
struct pat_ctrl {
	s32 start:4;
	s32 dir:4;
	s32 lut_id:8;
	s32 sync:12;
	s32 ini:4;
};

/**
 * PAT descriptor.
 */
struct pat {
	struct pat *next;
	struct pat_area area;
	struct pat_ctrl ctrl;
	u32 data;
};

struct omap_dmm_aperture {
	unsigned long base;
	size_t size;
	int fmt;
};

struct omap_dmm_iommu_drvdata {
	void __iomem *base;
	int irq;

	int num_apertures;

	/* dmm hw info */
	int num_refill_engines;
	int num_luts;
};

/**
 * platform data
 */
struct omap_dmm_platform_data {
	const char *oh_name;
	void __iomem *base;
	int irq;
	int num_apertures;
	struct omap_dmm_aperture *apertures;
};

void omap_dmm_init(void);

#ifdef CONFIG_OMAP_DMM_IOMMU
#define DMM_INIT()	omap_dmm_init()
#else
#define DMM_INIT()
#endif

#endif
