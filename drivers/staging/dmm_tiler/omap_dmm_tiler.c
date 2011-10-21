/*
 * DMM IOMMU driver support functions for TI OMAP processors.
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
#include <linux/init.h>
#include <linux/module.h>
#include <linux/platform_device.h> /* platform_device() */
#include <linux/errno.h>
#include <linux/sched.h>
#include <linux/wait.h>
#include <linux/interrupt.h>
#include <linux/dma-mapping.h>
#include <linux/slab.h>
#include <linux/vmalloc.h>
#include <linux/delay.h>
#include <linux/mm.h>
#include <linux/time.h>
#include <linux/list.h>
#include <linux/semaphore.h>

#include "tcm.h"
#include "omap_dmm_tiler.h"

/* mappings for associating views to luts */
static struct tcm *containers[TILFMT_NFORMATS];
static struct dmm *omap_dmm;

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
		[TILFMT_8BPP]  = GEOM(0, 0, 1),
		[TILFMT_16BPP] = GEOM(0, 1, 2),
		[TILFMT_32BPP] = GEOM(1, 1, 4),
		[TILFMT_PAGE]  = GEOM(SLOT_WIDTH_BITS, SLOT_HEIGHT_BITS, 1),
};


/* lookup table for registers w/ per-engine instances */
static const uint32_t reg[][4] = {
		[PAT_STATUS] = {DMM_PAT_STATUS__0, DMM_PAT_STATUS__1,
				DMM_PAT_STATUS__2, DMM_PAT_STATUS__3},
		[PAT_DESCR]  = {DMM_PAT_DESCR__0, DMM_PAT_DESCR__1,
				DMM_PAT_DESCR__2, DMM_PAT_DESCR__3},
};

/* simple allocator to grab next 16 byte aligned memory from txn */
static void *alloc_dma(struct dmm_txn *txn, size_t sz, dma_addr_t *pa)
{
	void *ptr;
	struct refill_engine *engine = txn->engine_handle;

	/* dmm programming requires 16 byte aligned addresses */
	txn->current_pa = round_up(txn->current_pa, 16);
	txn->current_va = (void *)round_up((long)txn->current_va, 16);

	ptr = txn->current_va;
	*pa = txn->current_pa;

	txn->current_pa += sz;
	txn->current_va += sz;

	BUG_ON((txn->current_va - engine->refill_va) > REFILL_BUFFER_SIZE);

	return ptr;
}

/* check status and spin until wait_mask comes true */
static int wait_status(struct refill_engine *engine, uint32_t wait_mask)
{
	struct dmm *dmm = engine->dmm;
	uint32_t r = 0, err, i;

	i = DMM_FIXED_RETRY_COUNT;
	while (true) {
		r = readl(dmm->base + reg[PAT_STATUS][engine->id]);
		err = r & DMM_PATSTATUS_ERR;
		if (err)
			return -EFAULT;

		if ((r & wait_mask) == wait_mask)
			break;

		if (--i == 0)
			return -ETIMEDOUT;

		udelay(1);
	}

	return 0;
}

irqreturn_t omap_dmm_irq_handler(int irq, void *arg)
{
	struct dmm *dmm = arg;
	uint32_t status = readl(dmm->base + DMM_PAT_IRQSTATUS);
	int i;

	/* ack IRQ */
	writel(status, dmm->base + DMM_PAT_IRQSTATUS);

	for (i = 0; i < dmm->num_engines; i++) {
		if (status & DMM_IRQSTAT_LST)
			wake_up_interruptible(&dmm->engines[i].wait_for_refill);

		status >>= 8;
	}

	return IRQ_HANDLED;
}

/**
 * Get a handle for a DMM transaction
 */
static struct dmm_txn *dmm_txn_init(struct dmm *dmm, struct tcm *tcm)
{
	struct dmm_txn *txn = NULL;
	struct refill_engine *engine = NULL;

	down(&dmm->engine_sem);

	/* grab an idle engine */
	spin_lock(&dmm->list_lock);
	if (!list_empty(&dmm->idle_head)) {
		engine = list_entry(dmm->idle_head.next, struct refill_engine,
					idle_node);
		list_del(&engine->idle_node);
	}
	spin_unlock(&dmm->list_lock);

	BUG_ON(!engine);

	txn = &engine->txn;
	engine->tcm = tcm;
	txn->engine_handle = engine;
	txn->last_pat = NULL;
	txn->current_va = engine->refill_va;
	txn->current_pa = engine->refill_pa;

	return txn;
}

/**
 * Add region to DMM transaction.  If pages or pages[i] is NULL, then the
 * corresponding slot is cleared (ie. dummy_pa is programmed)
 */
static int dmm_txn_append(struct dmm_txn *txn, struct pat_area *area,
			struct page **pages)
{
	dma_addr_t pat_pa = 0;
	uint32_t *data;
	struct pat *pat;
	struct refill_engine *engine = txn->engine_handle;
	int columns = (1 + area->x1 - area->x0);
	int rows = (1 + area->y1 - area->y0);
	int i = columns*rows;
	u32 *lut = omap_dmm->lut + (engine->tcm->lut_id * omap_dmm->lut_width *
			omap_dmm->lut_height) +
			(area->y0 * omap_dmm->lut_width) + area->x0;

	pat = alloc_dma(txn, sizeof(struct pat), &pat_pa);

	if (txn->last_pat)
		txn->last_pat->next_pa = (uint32_t)pat_pa;

	pat->area = *area;
	pat->ctrl = (struct pat_ctrl){
			.start = 1,
			.lut_id = engine->tcm->lut_id,
		};

	data = alloc_dma(txn, 4*i, &pat->data_pa);

	while (i--) {
		data[i] = (pages && pages[i]) ?
		page_to_phys(pages[i]): engine->dmm->dummy_pa;
	}

	/* fill in lut with new addresses */
	for (i = 0; i < rows; i++, lut += omap_dmm->lut_width)
		memcpy(lut, &data[i*columns], columns * sizeof(u32));

	txn->last_pat = pat;

	return 0;
}

/**
 * Commit the DMM transaction.
 */
static int dmm_txn_commit(struct dmm_txn *txn, bool wait)
{
	int ret = 0;
	struct refill_engine *engine = txn->engine_handle;
	struct dmm *dmm = engine->dmm;

	if (!txn->last_pat) {
		dev_err(engine->dmm->dev, "need at least one txn\n");
		ret = -EINVAL;
		goto cleanup;
	}

	txn->last_pat->next_pa = 0;

	/* write to PAT_DESCR to clear out any pending transaction */
	writel(0x0, dmm->base + reg[PAT_DESCR][engine->id]);

	/* wait for engine ready: */
	ret = wait_status(engine, DMM_PATSTATUS_READY);
	if (ret) {
		ret = -EFAULT;
		goto cleanup;
	}

	/* kick reload */
	writel(engine->refill_pa,
		dmm->base + reg[PAT_DESCR][engine->id]);

	if (wait) {
		if (wait_event_interruptible_timeout(engine->wait_for_refill,
				wait_status(engine, DMM_PATSTATUS_READY) == 0,
				msecs_to_jiffies(1)) <= 0) {
			dev_err(dmm->dev, "timed out waiting for done\n");
			ret = -ETIMEDOUT;
		}
	}

cleanup:
	spin_lock(&dmm->list_lock);
	list_add(&engine->idle_node, &dmm->idle_head);
	spin_unlock(&dmm->list_lock);

	up(&omap_dmm->engine_sem);
	return ret;
}

/*
 * DMM programming
 */
static int fill(struct tcm_area *area, struct page **pages, bool wait)
{
	int ret = 0;
	struct tcm_area slice, area_s;
	struct dmm_txn *txn;

	txn = dmm_txn_init(omap_dmm, area->tcm);
	if (IS_ERR_OR_NULL(txn))
		return PTR_ERR(txn);

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

/* note: slots for which pages[i] == NULL are filled w/ dummy page
 */
int tiler_pin(tiler_handle_t handle, struct page **pages, bool wait)
{
	struct tiler_block *block = handle;
	int ret;

	ret = fill(&block->area, pages, wait);

	if (ret)
		tiler_unpin(handle);

	return ret;
}
EXPORT_SYMBOL(tiler_pin);

int tiler_unpin(tiler_handle_t handle)
{
	struct tiler_block *block = handle;
	return fill(&block->area, NULL, false);
}
EXPORT_SYMBOL(tiler_unpin);

/*
 * Reserve/release
 */
tiler_handle_t tiler_reserve_2d(enum tiler_fmt fmt, uint16_t w, uint16_t h,
			uint16_t align)
{
	struct tiler_block *block = kzalloc(sizeof(*block), GFP_KERNEL);
	u32 min_align = 128;
	int ret;

	BUG_ON(!validfmt(fmt));

	/* convert width/height to slots */
	w = DIV_ROUND_UP(w, geom[fmt].slot_w);
	h = DIV_ROUND_UP(h, geom[fmt].slot_h);

	/* convert alignment to slots */
	min_align = max(min_align, (geom[fmt].slot_w * geom[fmt].cpp));
	align = ALIGN(align, min_align);
	align /= geom[fmt].slot_w * geom[fmt].cpp;

	block->fmt = fmt;

	ret = tcm_reserve_2d(containers[fmt], w, h, align, &block->area);
	if (ret) {
		kfree(block);
		return 0;
	}

	/* add to allocation list */
	spin_lock(&omap_dmm->list_lock);
	list_add(&block->alloc_node, &omap_dmm->alloc_head);
	spin_unlock(&omap_dmm->list_lock);

	return block;
}
EXPORT_SYMBOL(tiler_reserve_2d);

tiler_handle_t tiler_reserve_1d(size_t size)
{
	struct tiler_block *block = kzalloc(sizeof(*block), GFP_KERNEL);
	int num_pages = (size + PAGE_SIZE - 1) >> PAGE_SHIFT;

	if (!block)
		return 0;

	block->fmt = TILFMT_PAGE;

	if (tcm_reserve_1d(containers[TILFMT_PAGE], num_pages,
				&block->area)) {
		kfree(block);
		return 0;
	}

	spin_lock(&omap_dmm->list_lock);
	list_add(&block->alloc_node, &omap_dmm->alloc_head);
	spin_unlock(&omap_dmm->list_lock);

	return block;
}
EXPORT_SYMBOL(tiler_reserve_1d);

/* note: if you have pin'd pages, you should have already unpin'd first! */
int tiler_release(tiler_handle_t handle)
{
	struct tiler_block *block = handle;
	int ret = tcm_free(&block->area);

	if (block->area.tcm)
		dev_err(omap_dmm->dev, "failed to release block\n");

	spin_lock(&omap_dmm->list_lock);
	list_del(&block->alloc_node);
	spin_unlock(&omap_dmm->list_lock);

	kfree(block);
	return ret;
}
EXPORT_SYMBOL(tiler_release);

void tiler_print_allocations(void)
{
	print_allocation_map(NULL, omap_dmm);
}
EXPORT_SYMBOL(tiler_print_allocations);

/*
 * Utils
 */

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

dma_addr_t tiler_ssptr(tiler_handle_t handle)
{
	struct tiler_block *block = handle;
	BUG_ON(!validfmt(handle->fmt));

	return TILVIEW_8BIT + tiler_get_address(0, block->fmt,
			block->area.p0.x * geom[block->fmt].slot_w,
			block->area.p0.y * geom[block->fmt].slot_h);
}
EXPORT_SYMBOL(tiler_ssptr);

void tiler_align(enum tiler_fmt fmt, uint16_t *w, uint16_t *h)
{
	BUG_ON(!validfmt(fmt));
	*w = round_up(*w, geom[fmt].slot_w);
	*h = round_up(*h, geom[fmt].slot_h);
}
EXPORT_SYMBOL(tiler_align);

uint32_t tiler_stride(enum tiler_fmt fmt)
{
	BUG_ON(!validfmt(fmt));

	return 1 << (CONT_WIDTH_BITS + geom[fmt].y_shft);
}
EXPORT_SYMBOL(tiler_stride);

size_t tiler_size(enum tiler_fmt fmt, uint16_t w, uint16_t h)
{
	tiler_align(fmt, &w, &h);
	return geom[fmt].cpp * w * h;
}
EXPORT_SYMBOL(tiler_size);

size_t tiler_vsize(enum tiler_fmt fmt, uint16_t w, uint16_t h)
{
	BUG_ON(!validfmt(fmt));
	return round_up(geom[fmt].cpp * w, PAGE_SIZE) * h;
}
EXPORT_SYMBOL(tiler_vsize);

/*
 *  driver functions
*/
static int omap_dmm_remove(struct platform_device *pdev)
{
	struct dmm *dmm;
	struct tiler_block *block, *_block;
	int i;

	dmm = platform_get_drvdata(pdev);

	if (dmm) {
		/* free all area regions */
		spin_lock(&omap_dmm->list_lock);
		list_for_each_entry_safe(block, _block, &omap_dmm->alloc_head,
					alloc_node) {
			list_del(&block->alloc_node);
			kfree(block);
		}
		spin_unlock(&omap_dmm->list_lock);

		for (i = 0; i < dmm->num_lut; i++)
			if (dmm->tcm && dmm->tcm[i])
				dmm->tcm[i]->deinit(dmm->tcm[i]);
		kfree(dmm->tcm);

		kfree(dmm->engines);
		if (dmm->refill_va)
			dma_free_coherent(dmm->dev,
					REFILL_BUFFER_SIZE * dmm->num_engines,
					dmm->refill_va, dmm->refill_pa);
		if (dmm->dummy_page)
			__free_page(dmm->dummy_page);

		vfree(dmm->lut);

		if (dmm->irq != -1)
			free_irq(dmm->irq, dmm);

		platform_set_drvdata(pdev, NULL);

		kfree(dmm);
	}

	return 0;
}

static int omap_dmm_probe(struct platform_device *pdev)
{
	int ret = -EFAULT, i;
	struct omap_dmm_platform_data *platdata = pdev->dev.platform_data;
	struct tcm_area area = {0};
	u32 hwinfo, pat_geom, lut_table_size;
	enum tiler_fmt fmt;

	if (!platdata)
		return -ENODEV;

	omap_dmm = kzalloc(sizeof(*omap_dmm), GFP_KERNEL);
	if (!omap_dmm) {
		dev_err(&pdev->dev, "failed to allocate driver data section\n");
		goto fail;
	}

	platform_set_drvdata(pdev, omap_dmm);

	/* HACK, fix the dma mask.... cannot rely on omap_device_build() */
	pdev->dev.coherent_dma_mask = DMA_BIT_MASK(32);

	omap_dmm->base = platdata->base;
	omap_dmm->irq = platdata->irq;
	omap_dmm->dev = &pdev->dev;

	hwinfo = readl(omap_dmm->base + DMM_PAT_HWINFO);
	omap_dmm->num_engines = (hwinfo >> 24) & 0x1F;
	omap_dmm->num_lut = (hwinfo >> 16) & 0x1F;
	omap_dmm->container_width = platdata->container_width;
	omap_dmm->container_height = platdata->container_height;

	/* read out actual LUT width and height */
	pat_geom = readl(omap_dmm->base + DMM_PAT_GEOMETRY);
	omap_dmm->lut_width = ((pat_geom >> 16) & 0xF) << 5;
	omap_dmm->lut_height = ((pat_geom >> 24) & 0xF) << 5;

	/* initialize DMM registers */
	writel(0x88888888, omap_dmm->base + DMM_PAT_VIEW__0);
	writel(0x88888888, omap_dmm->base + DMM_PAT_VIEW__1);
	writel(0x80808080, omap_dmm->base + DMM_PAT_VIEW_MAP__0);
	writel(0x80000000, omap_dmm->base + DMM_PAT_VIEW_MAP_BASE);
	writel(0x88888888, omap_dmm->base + DMM_TILER_OR__0);
	writel(0x88888888, omap_dmm->base + DMM_TILER_OR__1);

	ret = request_irq(omap_dmm->irq, omap_dmm_irq_handler, IRQF_SHARED,
				"omap_dmm_irq_handler", omap_dmm);

	if (ret) {
		dev_err(&pdev->dev, "couldn't register IRQ %d, error %d\n",
			omap_dmm->irq, ret);
		omap_dmm->irq = -1;
		goto fail;
	}

	/* enable some interrupts! */
	writel(0xfefefefe, omap_dmm->base + DMM_PAT_IRQENABLE_SET);

	lut_table_size = omap_dmm->lut_width * omap_dmm->lut_height *
			omap_dmm->num_lut;

	omap_dmm->lut = vmalloc(lut_table_size * sizeof(*omap_dmm->lut));
	if (!omap_dmm->lut) {
		dev_err(&pdev->dev, "could not allocate lut table\n");
		ret = -ENOMEM;
		goto fail;
	}

	omap_dmm->dummy_page = alloc_page(GFP_KERNEL | __GFP_DMA32);
	if (!omap_dmm->dummy_page) {
		dev_err(&pdev->dev, "could not allocate dummy page\n");
		ret = -ENOMEM;
		goto fail;
	}
	omap_dmm->dummy_pa = page_to_phys(omap_dmm->dummy_page);

	/* alloc refill memory */
	omap_dmm->refill_va = dma_alloc_coherent(&pdev->dev,
				REFILL_BUFFER_SIZE * omap_dmm->num_engines,
				&omap_dmm->refill_pa, GFP_KERNEL);
	if (!omap_dmm->refill_va) {
		dev_err(&pdev->dev, "could not allocate refill memory\n");
		goto fail;
	}

	/* alloc engines */
	omap_dmm->engines = kzalloc(
			omap_dmm->num_engines * sizeof(struct refill_engine),
			GFP_KERNEL);
	if (!omap_dmm->engines) {
		dev_err(&pdev->dev, "could not allocate engines\n");
		ret = -ENOMEM;
		goto fail;
	}

	sema_init(&omap_dmm->engine_sem, omap_dmm->num_engines);
	INIT_LIST_HEAD(&omap_dmm->idle_head);
	for (i = 0; i < omap_dmm->num_engines; i++) {
		omap_dmm->engines[i].id = i;
		omap_dmm->engines[i].dmm = omap_dmm;
		omap_dmm->engines[i].refill_va = omap_dmm->refill_va +
						(REFILL_BUFFER_SIZE * i);
		omap_dmm->engines[i].refill_pa = omap_dmm->refill_pa +
						(REFILL_BUFFER_SIZE * i);
		init_waitqueue_head(&omap_dmm->engines[i].wait_for_refill);

		list_add(&omap_dmm->engines[i].idle_node, &omap_dmm->idle_head);
	}

	omap_dmm->tcm = kzalloc(omap_dmm->num_lut * sizeof(*omap_dmm->tcm),
				GFP_KERNEL);
	if (!omap_dmm->tcm) {
		dev_err(&pdev->dev, "failed to allocate lut ptrs\n");
		ret = -ENOMEM;
		goto fail;
	}

	/* init containers */
	for (i = 0; i < omap_dmm->num_lut; i++) {
		omap_dmm->tcm[i] = sita_init(omap_dmm->container_width,
						omap_dmm->container_height,
						NULL);

		if (!omap_dmm->tcm[i]) {
			dev_err(&pdev->dev, "failed to allocate container\n");
			ret = -ENOMEM;
			goto fail;
		}

		omap_dmm->tcm[i]->lut_id = i;
	}

	/* assign access mode containers to applicable tcm container */
	for (fmt = TILFMT_8BPP; fmt < TILFMT_NFORMATS; fmt++)
		containers[fmt] = omap_dmm->tcm[platdata->views[fmt].lut_id];

	INIT_LIST_HEAD(&omap_dmm->alloc_head);
	spin_lock_init(&omap_dmm->list_lock);

	area = (struct tcm_area) {
		.is2d = true,
		.tcm = NULL,
		.p1.x = omap_dmm->container_width - 1,
		.p1.y = omap_dmm->container_height - 1,
	};

	for (i = 0; i < lut_table_size; i++)
		omap_dmm->lut[i] = omap_dmm->dummy_pa;

	/* initialize all LUTs to dummy page entries */
	for (i = 0; i < omap_dmm->num_lut; i++) {
		area.tcm = omap_dmm->tcm[i];
		if (fill(&area, NULL, true))
			dev_err(omap_dmm->dev, "refill failed");
	}

	return 0;

fail:
	omap_dmm_remove(pdev);
	return ret;
}

static struct platform_driver omap_dmm_driver = {
	.driver = {
		.name = "dmm",
	},
	.probe		= omap_dmm_probe,
	.remove		= omap_dmm_remove,
};

static int __init omap_dmm_driver_init(void)
{
	return platform_driver_register(&omap_dmm_driver);
}

static void __exit omap_dmm_driver_exit(void)
{
	platform_driver_unregister(&omap_dmm_driver);
}

module_init(omap_dmm_driver_init);
module_exit(omap_dmm_driver_exit);

MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("Rob Clark <rob@ti.com>");
MODULE_AUTHOR("Andy Gross <andy.gross@ti.com>");
