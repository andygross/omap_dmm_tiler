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

enum tiler_mode {
	TILER_MODE_8BPP = 0,
	TILER_MODE_16BPP,
	TILER_MODE_32BPP,
	TILER_MODE_PAGE,
	TILER_MODE_INVALID
};

struct pat_area {
        u32 x0:8;
        u32 y0:8;
        u32 x1:8;
        u32 y1:8;
};


/**
 * platform data
 */
struct container {
	void __iomem *base;
	int lut_id;
	int y_offset;
	enum tiler_mode format;
};

struct omap_dmm_platform_data {
	const char *oh_name;
	void __iomem *base;
	int irq;
	int num_engines;
	struct container *containers;
	int num_containers;
};

void omap_dmm_init(void);

#endif
