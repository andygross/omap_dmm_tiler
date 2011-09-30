/*
 * drivers/staging/omapdrm/omap_dmm.c
 *
 * Copyright (C) 2011 Texas Instruments
 * Authors: Rob Clark <rob.clark@linaro.org>
 *          Lajos Molnar <molnar@ti.com>
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

#include "omap_drv.h"
#include "omap_dmm.h"

#include <plat/dmm.h>
#include "tcm/tcm.h"
#include "tcm/tcm-sita.h"


/* bits representing the same slot in DMM-TILER hw-block */
#define SLOT_WIDTH_BITS         6
#define SLOT_HEIGHT_BITS        6

/* bits reserved to describe coordinates in DMM-TILER hw-block */
#define CONT_WIDTH_BITS         14
#define CONT_HEIGHT_BITS        13

/* calculated constants */
#define TILER_PAGE              (1 << (SLOT_WIDTH_BITS + SLOT_HEIGHT_BITS))
#define TILER_WIDTH             (1 << (CONT_WIDTH_BITS - SLOT_WIDTH_BITS))
#define TILER_HEIGHT            (1 << (CONT_HEIGHT_BITS - SLOT_HEIGHT_BITS))

#define VIEW_SIZE               (1u << (CONT_WIDTH_BITS + CONT_HEIGHT_BITS))

/* location of the various tiler views in physical address space (ssptr) */
#define TILVIEW_8BIT    0x60000000u
#define TILVIEW_16BIT   (TILVIEW_8BIT  + VIEW_SIZE)
#define TILVIEW_32BIT   (TILVIEW_16BIT + VIEW_SIZE)
#define TILVIEW_PAGE    (TILVIEW_32BIT + VIEW_SIZE)
#define TILVIEW_END     (TILVIEW_PAGE  + VIEW_SIZE)


/* Geometry table */
#define GEOM(xshift, yshift, bytes_per_pixel) { \
		.x_shft = (xshift), \
		.y_shft = (yshift), \
		.cpp    = (bytes_per_pixel), \
		.slot_w = 1 << (SLOT_WIDTH_BITS - (xshift)), \
		.slot_h = 1 << (SLOT_HEIGHT_BITS - (yshift)), \
	}
static const struct {
	uint32_t x_shft;	/* unused X-bits (as part of bpp) */
	uint32_t y_shft;	/* unused Y-bits (as part of bpp) */
	uint32_t cpp;		/* bytes/chars per pixel */
	uint32_t slot_w;	/* width of each slot (in pixels) */
	uint32_t slot_h;	/* height of each slot (in pixels) */
} geom[TILFMT_NFORMATS] = {
		[TILFMT_8BIT]  = GEOM(0, 0, 1),
		[TILFMT_16BIT] = GEOM(0, 1, 2),
		[TILFMT_32BIT] = GEOM(1, 1, 4),
		[TILFMT_PAGE]  = GEOM(SLOT_WIDTH_BITS, SLOT_HEIGHT_BITS, 1),
};

/*
 * Instance Data
 */

static struct {
	struct tcm *tcm[TILFMT_NFORMATS];
	struct dmm *dmm[TILFMT_NFORMATS];
} *omap_dmm;

/*
 * DMM programming
 */

static int fill(enum tiler_fmt fmt, struct tcm_area *area,
		struct page **pages, bool wait)
{
	int ret = 0;
	struct tcm_area slice, area_s;
	struct dmm_txn *txn;

	BUG_ON(!validfmt(fmt));

	txn = dmm_txn_init(omap_dmm->dmm[fmt]);
	if (IS_ERR_OR_NULL(txn)) {
		return PTR_ERR(txn);
	}

	tcm_for_each_slice(slice, *area, area_s) {
		struct pat_area p_area = {
				.x0 = slice.p0.x,  .y0 = slice.p0.y,
				.x1 = slice.p1.x,  .y1 = slice.p1.y,
		};

		ret = dmm_txn_append(txn, &p_area, pages);
		if (ret)
			goto fail;

		if (pages)
			pages += tcm_sizeof(slice);
	}

	ret = dmm_txn_commit(txn, wait);

fail:
	return ret;
}

/*
 * Pin/unpin
 */

/* note: slots for which pages[i] == NULL are filled w/ dummy page.. this is
 * needed for usergart stuff where we partially fill an area
 */
int omap_dmm_pin(enum tiler_fmt fmt, struct tcm_area *area,
		struct page **pages, bool wait)
{
	int ret = fill(fmt, area, pages, wait);
	if (ret) {
		omap_dmm_unpin(fmt, area);
		return ret;
	}
	return 0;
}

int omap_dmm_unpin(enum tiler_fmt fmt, struct tcm_area *area)
{
	return fill(fmt, area, NULL, false);
}

/*
 * Reserve/release
 */

/* note: w/h/align must be slot aligned */
struct tcm_area * omap_dmm_reserve_2d(enum tiler_fmt fmt,
		uint16_t w, uint16_t h, uint16_t align)
{
	struct tcm_area *area = kzalloc(sizeof(*area), GFP_KERNEL);
	int ret;

	BUG_ON(!validfmt(fmt));

	/* convert width/height to slots */
	w /= geom[fmt].slot_w;
	h /= geom[fmt].slot_h;

	/* convert alignment to slots */
	align /= geom[fmt].slot_w * geom[fmt].cpp;

	ret = tcm_reserve_2d(omap_dmm->tcm[fmt], w, h, align, area);
	if (ret) {
		kfree(area);
		return ERR_PTR(ret);
	}
	return area;
}

/* note: size must be page aligned */
struct tcm_area * omap_dmm_reserve_1d(size_t size)
{
	struct tcm_area *area = kzalloc(sizeof(*area), GFP_KERNEL);
	int ret = tcm_reserve_1d(omap_dmm->tcm[TILFMT_8BIT],
			size >> PAGE_SHIFT, area);
	if (ret) {
		kfree(area);
		return ERR_PTR(ret);
	}
	return area;
}

/* note: if you have pin'd pages, you should have already unpin'd first! */
int omap_dmm_release(struct tcm_area *area)
{
	int ret = tcm_free(area);
	if (ret) {
		return ret;
	}
	kfree(area);
	return 0;
}

/*
 * Utilities
 */

/* tiler space addressing bitfields */
#define MASK_XY_FLIP		(1 << 31)
#define MASK_Y_INVERT		(1 << 30)
#define MASK_X_INVERT		(1 << 29)
#define SHIFT_ACC_MODE		27
#define MASK_ACC_MODE		3

#define MASK(bits) ((1 << (bits)) - 1)

/* create tsptr by adding view orientation and access mode */
#define TIL_ADDR(x, orient, a)\
	((u32) (x) | (orient) | ((a) << SHIFT_ACC_MODE))

/* calculate the tiler space address of a pixel in a view orientation */
static u32 tiler_get_address(u32 orient, enum tiler_fmt fmt, u32 x, u32 y)
{
	u32 x_bits, y_bits, tmp, x_mask, y_mask, alignment;

	x_bits = CONT_WIDTH_BITS - geom[fmt].x_shft;
	y_bits = CONT_HEIGHT_BITS - geom[fmt].y_shft;
	alignment = geom[fmt].x_shft + geom[fmt].y_shft;

	/* validate coordinate */
	x_mask = MASK(x_bits);
	y_mask = MASK(y_bits);
DBG("%dx%d, %dx%d", x, y, x_mask, y_mask);
	if (x < 0 || x > x_mask || y < 0 || y > y_mask)
		return 0;

	/* account for mirroring */
	if (orient & MASK_X_INVERT)
		x ^= x_mask;
	if (orient & MASK_Y_INVERT)
		y ^= y_mask;

	/* get coordinate address */
	if (orient & MASK_XY_FLIP)
		tmp = ((x << y_bits) + y);
	else
		tmp = ((y << x_bits) + x);

	return TIL_ADDR((tmp << alignment), orient, fmt);
}

dma_addr_t omap_dmm_ssptr(enum tiler_fmt fmt, struct tcm_area *area)
{
	BUG_ON(!validfmt(fmt));

	return TILVIEW_8BIT + tiler_get_address(0, fmt,
			area->p0.x * geom[fmt].slot_w,
			area->p0.y * geom[fmt].slot_h);
}

uint32_t omap_dmm_stride(enum tiler_fmt fmt)
{
	BUG_ON(!validfmt(fmt));

	return 1 << (CONT_WIDTH_BITS + geom[fmt].y_shft);
}

void omap_dmm_align(enum tiler_fmt fmt, uint16_t *w, uint16_t *h)
{
	BUG_ON(!validfmt(fmt));
	/* note: slot width/height are always powers of 2 */
	*w = round_up(*w, geom[fmt].slot_w);
	*h = round_up(*h, geom[fmt].slot_h);
}

/* note: w/h should be aligned, see omap_dmm_align() */
size_t omap_dmm_size(enum tiler_fmt fmt, uint16_t w, uint16_t h)
{
	BUG_ON(!validfmt(fmt));
	return geom[fmt].cpp * w * h;
}

/* same as omap_dmm_size(), but calculates virtual size based on
 * rounding up pitch to next PAGE_SIZE boundary
 */
size_t omap_dmm_vsize(enum tiler_fmt fmt, uint16_t w, uint16_t h)
{
	BUG_ON(!validfmt(fmt));
	return round_up(geom[fmt].cpp * w, PAGE_SIZE) * h;
}

/*
 * Init/cleanup
 */

void omap_dmm_deinit(struct drm_device *dev)
{
	/* TODO */
}

int omap_dmm_init(struct drm_device *dev)
{
	struct tcm *tcm;
	struct dmm *dmm;
	struct dmm_txn *txn;
	struct pat_area area;
	int ret;

	if (!has_dmm())
		return -EINVAL;

	omap_dmm = kzalloc(sizeof(*omap_dmm), GFP_KERNEL);
	if (!omap_dmm) {
		dev_err(dev->dev, "could not allocate DMM\n");
		ret = -ENOMEM;
		goto fail;
	}

	/* Allocate tiler container manager (we share 1 on OMAP4) */
	tcm = sita_init(TILER_WIDTH, TILER_HEIGHT, NULL);
	if (!tcm) {
		dev_err(dev->dev, "could not initialize container manager\n");
		ret = -ENOMEM;
		goto fail;
	}

	omap_dmm->tcm[TILFMT_8BIT]  = tcm;
	omap_dmm->tcm[TILFMT_16BIT] = tcm;
	omap_dmm->tcm[TILFMT_32BIT] = tcm;
	omap_dmm->tcm[TILFMT_PAGE]  = tcm;

	/* Allocate DMM (we share 1 on OMAP4) */
	dmm = dmm_pat_init(0);
	if (!dmm) {
		dev_err(dev->dev, "could not initialize PAT\n");
		ret = -ENOMEM;
		goto fail;
	}

	omap_dmm->dmm[TILFMT_8BIT]  = dmm;
	omap_dmm->dmm[TILFMT_16BIT] = dmm;
	omap_dmm->dmm[TILFMT_32BIT] = dmm;
	omap_dmm->dmm[TILFMT_PAGE]  = dmm;

	/* clear entire DMM space */
	area = (struct pat_area) {
			.x1 = TILER_WIDTH - 1,
			.y1 = TILER_HEIGHT - 1,
	};
	// TODO error checking
	txn = dmm_txn_init(dmm);
	if (IS_ERR_OR_NULL(txn)) {
		ret = PTR_ERR(txn);
		goto fail;
	}
	ret = dmm_txn_append(txn, &area, NULL) ||
			dmm_txn_commit(txn, false);
	if (ret) {
		dev_err(dev->dev, "could not clear PAT\n");
		goto fail;
	}

	return 0;

fail:
	omap_dmm_deinit(dev);
	return ret;
}
