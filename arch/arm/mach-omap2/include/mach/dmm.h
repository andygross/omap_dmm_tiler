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

struct tcm_area;

/* pin/unpin */
int omap_dmm_pin(enum tiler_mode fmt, struct tcm_area *area,
                struct page **pages, bool wait);
int omap_dmm_unpin(enum tiler_mode fmt, struct tcm_area *area);

/* reserve/release */
struct tcm_area * omap_dmm_reserve_2d(enum tiler_mode fmt,
                uint16_t w, uint16_t h, uint16_t align);
struct tcm_area * omap_dmm_reserve_1d(size_t size);
int omap_dmm_release(struct tcm_area *area);

/* utilities */
dma_addr_t omap_dmm_ssptr(enum tiler_mode fmt, struct tcm_area *area);
uint32_t omap_dmm_stride(enum tiler_mode fmt);
void omap_dmm_align(enum tiler_mode fmt, uint16_t *w, uint16_t *h);
size_t omap_dmm_size(enum tiler_mode fmt, uint16_t w, uint16_t h);
size_t omap_dmm_vsize(enum tiler_mode fmt, uint16_t w, uint16_t h);

static inline bool validfmt(enum tiler_mode fmt)
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
