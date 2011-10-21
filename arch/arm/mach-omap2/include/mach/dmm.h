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

enum tiler_fmt {
	TILFMT_8BPP = 0,
	TILFMT_16BPP,
	TILFMT_32BPP,
	TILFMT_PAGE,
	TILFMT_NFORMATS
};

struct pat_area {
	u32 x0:8;
	u32 y0:8;
	u32 x1:8;
	u32 y1:8;
};

struct omap_dmm_platform_data {
	const char *oh_name;
	void __iomem *base;
	int irq;

	/* number of refill engines in DMM block */
	int num_engines;

	/* lookup table information */
	int num_lut;
	int lut_width;
	int lut_height;
};

void omap_dmm_init(void);

typedef struct tiler_block *tiler_handle_t;

/* pin/unpin */
int tiler_pin(tiler_handle_t handle, struct page **pages, bool wait);
int tiler_unpin(tiler_handle_t handle);

/* reserve/release */
tiler_handle_t tiler_reserve_2d(enum tiler_fmt fmt, uint16_t w, uint16_t h,
				uint16_t align);
tiler_handle_t tiler_reserve_1d(size_t size);
int tiler_release(tiler_handle_t handle);

/* utilities */
dma_addr_t tiler_ssptr(tiler_handle_t handle);

uint32_t tiler_stride(enum tiler_fmt fmt);

size_t tiler_size(enum tiler_fmt fmt, uint16_t w, uint16_t h);

size_t tiler_vsize(enum tiler_fmt fmt, uint16_t w, uint16_t h);

void tiler_align(enum tiler_fmt fmt, uint16_t *w, uint16_t *h);

void tiler_print_allocations(void);

static inline bool validfmt(enum tiler_fmt fmt)
{
	switch (fmt) {
	case TILFMT_8BPP:
	case TILFMT_16BPP:
	case TILFMT_32BPP:
	case TILFMT_PAGE:
		return true;
	default:
		return false;
	}
}

#endif
