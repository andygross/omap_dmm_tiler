/*
 * drivers/staging/omapdrm/omap_dmm.h
 *
 * Copyright (C) 2011 Texas Instruments
 * Author: Rob Clark <rob.clark@linaro.org>
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License version 2 as published by
 * the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef __OMAP_DMM_H__
#define __OMAP_DMM_H__

#include "omap_drv.h"

enum tiler_fmt {
	TILFMT_8BIT    = 0,
	TILFMT_16BIT   = 1,
	TILFMT_32BIT   = 2,
	TILFMT_PAGE    = 3,
	TILFMT_NFORMATS
};

struct tcm_area;

int omap_dmm_init(struct drm_device *dev);
void omap_dmm_deinit(struct drm_device *dev);

/* pin/unpin */
int omap_dmm_pin(enum tiler_fmt fmt, struct tcm_area *area,
		struct page **pages, bool wait);
int omap_dmm_unpin(enum tiler_fmt fmt, struct tcm_area *area);

/* reserve/release */
struct tcm_area * omap_dmm_reserve_2d(enum tiler_fmt fmt,
		uint16_t w, uint16_t h, uint16_t align);
struct tcm_area * omap_dmm_reserve_1d(size_t size);
int omap_dmm_release(struct tcm_area *area);

/* utilities */
dma_addr_t omap_dmm_ssptr(enum tiler_fmt fmt, struct tcm_area *area);
uint32_t omap_dmm_stride(enum tiler_fmt fmt);
void omap_dmm_align(enum tiler_fmt fmt, uint16_t *w, uint16_t *h);
size_t omap_dmm_size(enum tiler_fmt fmt, uint16_t w, uint16_t h);
size_t omap_dmm_vsize(enum tiler_fmt fmt, uint16_t w, uint16_t h);


/* GEM bo flags -> tiler fmt */
static inline enum tiler_fmt gem2fmt(uint32_t flags)
{
	switch (flags & OMAP_BO_TILED) {
	case OMAP_BO_TILED_8:
		return TILFMT_8BIT;
	case OMAP_BO_TILED_16:
		return TILFMT_16BIT;
	case OMAP_BO_TILED_32:
		return TILFMT_32BIT;
	default:
		return TILFMT_PAGE;
	}
}

static inline bool validfmt(enum tiler_fmt fmt)
{
	switch (fmt) {
	case TILFMT_8BIT:
	case TILFMT_16BIT:
	case TILFMT_32BIT:
	case TILFMT_PAGE:
		return true;
	default:
		return false;
	}
}

#endif /* __OMAP_DMM_H__ */
