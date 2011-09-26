/*
 * drivers/staging/omapdrm/omap_dmm.c
 *
 * Copyright (C) 2011 Texas Instruments
 * Author: Rob Clark <rob@ti.com>
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

	/* dummy page for clearing unused DMM/PAT slots */
	struct page *dummy_pg;
	dma_addr_t dummy_pa;

	/* buffer to use for PAT refill.. PAT refill engine DMA's from
	 * this buffer to reprogram DMM/PAT slots
	 */
	u32 *refill_va;
	dma_addr_t refill_pa;

	spinlock_t lock;
} *omap_dmm;

/*
 * DMM programming
 */

static int refill(struct pat_area *area, enum tiler_fmt fmt)
{
	struct pat pat_desc = {
			.ctrl.start = 1,
			.area = *area,
			.data = omap_dmm->refill_pa,
	};

	BUG_ON(!validfmt(fmt));

	return dmm_pat_refill(omap_dmm->dmm[fmt], &pat_desc, MANUAL);
}

static int fill(struct pat_area *area, enum tiler_fmt fmt,
		struct page **pages)
{
	int ret, i = (area->x1 - area->x0) * (area->y1 - area->y0);
	DBG("%d,%d %d,%d", area->x0, area->y0, area->x1, area->y1);
	spin_lock(&omap_dmm->lock);
	while (i--)
		omap_dmm->refill_va[i] = page_to_phys(pages[i]);
	ret = refill(area, fmt);
	spin_unlock(&omap_dmm->lock);
	return ret;
}

static int clear(struct pat_area *area, enum tiler_fmt fmt)
{
	int ret, i = (area->x1 - area->x0) * (area->y1 - area->y0);
	DBG("%d,%d %d,%d", area->x0, area->y0, area->x1, area->y1);
	spin_lock(&omap_dmm->lock);
	while (i--)
		omap_dmm->refill_va[i] = omap_dmm->dummy_pa;
	ret = refill(area, fmt);
	spin_unlock(&omap_dmm->lock);
	return ret;
}

static int slicer(enum tiler_fmt fmt,
		struct tcm_area *area, struct page **pages,
		int (*func)(struct pat_area *, enum tiler_fmt, struct page **))
{
	int ret = 0;
	struct tcm_area slice, area_s;

	tcm_for_each_slice(slice, *area, area_s) {
		struct pat_area p_area = {
				.x0 = slice.p0.x,
				.y0 = slice.p0.y,
				.x1 = slice.p1.x,
				.y1 = slice.p1.y,
		};

		ret = func(&p_area, fmt, pages);
		if (ret)
			break;

		pages += tcm_sizeof(slice);
	}

	return ret;
}

/*
 * Pin/unpin
 */

int omap_dmm_pin(enum tiler_fmt fmt, struct tcm_area *area, struct page **pages)
{
	int ret = slicer(fmt, area, pages, fill);
	if (ret) {
		omap_dmm_unpin(fmt, area);
		return ret;
	}
	return 0;
}

int omap_dmm_unpin(enum tiler_fmt fmt, struct tcm_area *area)
{
	int func(struct pat_area *area, enum tiler_fmt fmt, struct page **pages)
	{
		return clear(area, fmt);
	}
	return slicer(fmt, area, NULL, func);
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
// 23, 248 ->
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

	DBG("%d: %d,%d %d,%d", fmt, area->p0.x, area->p0.y,
			area->p1.x, area->p1.y);

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

	spin_lock_init(&omap_dmm->lock);

	omap_dmm->dummy_pg = alloc_page(GFP_KERNEL | __GFP_DMA32);
	if (!omap_dmm->dummy_pg) {
		dev_err(dev->dev, "could not allocate dummy page\n");
		ret = -ENOMEM;
		goto fail;
	}
	omap_dmm->dummy_pa = page_to_phys(omap_dmm->dummy_pg);

	/*
	 * Array of physical pages for PAT programming, which must be a 16-byte
	 * aligned physical address.
	 */
	omap_dmm->refill_va = dma_alloc_coherent(dev->dev,
			TILER_WIDTH * TILER_WIDTH * sizeof(*omap_dmm->refill_va),
			&omap_dmm->refill_pa, GFP_KERNEL);
	if (!omap_dmm->refill_va) {
		dev_err(dev->dev, "could not allocate refill buffer\n");
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
	ret = clear(&area, TILFMT_8BIT);
	if (ret) {
		dev_err(dev->dev, "could not clear PAT\n");
		goto fail;
	}

	return 0;

fail:
	omap_dmm_deinit(dev);
	return ret;
}
