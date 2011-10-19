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
#include <linux/delay.h>
#include <linux/mm.h>
#include <linux/pm.h>
#include <linux/time.h>
#include <linux/list.h>
#include <linux/seq_file.h>

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
static const uint32_t reg[][2] = {
		[PAT_STATUS] = {DMM_PAT_STATUS__0, DMM_PAT_STATUS__1},
		[PAT_DESCR]  = {DMM_PAT_DESCR__0, DMM_PAT_DESCR__1},
};

/* simple allocator to grab next 16 byte aligned memory from txn */
static void * alloc_dma(struct dmm_txn *txn, size_t sz, dma_addr_t *pa)
{
        void *ptr;
	struct refill_engine *engine = txn->engine_handle;

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
		if (err) {
			dev_err(dmm->dev, "error: %02x\n", err >> 10);
			return -EFAULT;
		}

		if ((r & wait_mask) == wait_mask)
			break;

		if (--i == 0) {
			dev_err(dmm->dev, "timedout waiting for status\n");
			break;
		}
		udelay(1);
	}

	return 0;
}

/*
 * IRQ handler
 */
irqreturn_t omap_dmm_irq_handler(int irq, void *arg)
{
	struct dmm *dmm = arg;
	uint32_t status = readl(dmm->base + DMM_PAT_IRQSTATUS);
	bool engine_done[] = {false, false};
	int i;

	if (status & DMM_IRQSTAT_LST) {
		/* refill engine 0 done */
		engine_done[0] = true;
	}

	if ((status >> 8) & DMM_IRQSTAT_LST) {
		/* refill engine 1 done */
		engine_done[1] = true;
	}

	/* ack IRQ */
	writel(status, dmm->base + DMM_PAT_IRQSTATUS);

	/* wake up the people waiting */
	for(i=0; i < dmm->num_engines; i++)
		if (engine_done[i])
			wake_up_interruptible(&dmm->engines[i].wait_for_refill);

	return IRQ_HANDLED;
}

/**
 * Get a handle for a DMM transaction
 */
static struct dmm_txn * dmm_txn_init(struct dmm *dmm)
{
	struct dmm_txn *txn = NULL;

	mutex_lock(&dmm->engines[0].mtx);

	txn = &dmm->engines[0].txn;
	txn->engine_handle = &dmm->engines[0];
	txn->last_pat = NULL;
	txn->current_va = dmm->engines[0].refill_va;
	txn->current_pa = dmm->engines[0].refill_pa;

	return txn;
}

/**
 * Add region to DMM transaction.  If pages or pages[i] is NULL, then the
 * corresponding slot is cleared (ie. dummy_pa is programmed)
 */
static int dmm_txn_append(struct dmm_txn *txn, struct pat_area *area,
                u32 *phys_array)
{
        dma_addr_t pat_pa = 0;
        uint32_t *data;
        struct pat *pat;
	struct refill_engine *engine = txn->engine_handle;
        int i = (1 + area->x1 - area->x0) * (1 + area->y1 - area->y0);

        pat = alloc_dma(txn, sizeof(struct pat), &pat_pa);

        if (txn->last_pat) {
                txn->last_pat->next_pa = (uint32_t)pat_pa;
        }

        pat->area = *area;
        pat->ctrl = (struct pat_ctrl){
                .start = 1,
        };

        data = alloc_dma(txn, 4*i, &pat->data_pa);

        while (i--) {
                data[i] = (phys_array && phys_array[i]) ?
                                phys_array[i] : engine->dmm->dummy_pa;
        }

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
		mutex_unlock(&engine->mtx);
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
	mutex_unlock(&engine->mtx);
	return ret;
}

/*
 * DMM programming
 */

static int fill(struct tcm_area *area, u32 *phys_array, bool wait)
{
	int ret = 0;
	struct tcm_area slice, area_s;
	struct dmm_txn *txn;

	txn = dmm_txn_init(omap_dmm);
	if (IS_ERR_OR_NULL(txn)) {
		return PTR_ERR(txn);
	}

	tcm_for_each_slice(slice, *area, area_s) {
		struct pat_area p_area = {
				.x0 = slice.p0.x,  .y0 = slice.p0.y,
				.x1 = slice.p1.x,  .y1 = slice.p1.y,
		};

		ret = dmm_txn_append(txn, &p_area, phys_array);
		if (ret)
			goto fail;

		if (phys_array)
			phys_array += tcm_sizeof(slice);
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
	int i, ret;

	for (i = 0; i < block->num_pages; i++)
		block->phys_array[i] = (pages && pages[i]) ?
				page_to_phys(pages[i]) : omap_dmm->dummy_pa;

	ret = fill(&block->area, block->phys_array, wait);

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

	block->num_pages = w * h;
	block->phys_array = kzalloc(
				block->num_pages * sizeof(*block->phys_array),
				GFP_KERNEL);
	block->fmt = fmt;

	if (!block->phys_array) {
		dev_err(omap_dmm->dev, "failed to allocate phys_array\n");
		kfree(block);
		return 0;
	}

	ret = tcm_reserve_2d(containers[fmt], w, h, align, &block->area);
	if (ret) {
		kfree(block->phys_array);
		kfree(block);
		return 0;
	}

	if (omap_dmm->alloc_debug)
		dev_info(omap_dmm->dev, "+2d: (%u,%u)-(%u,%u): %dx%d\n",
			block->area.p0.x, block->area.p0.y, block->area.p1.x,
			block->area.p1.y, w, h);

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

	if (!block)
		return 0;

	block->num_pages = (size + PAGE_SIZE -1 ) >> PAGE_SHIFT;
	block->phys_array = kzalloc(
				block->num_pages * sizeof(*block->phys_array),
					GFP_KERNEL);
	block->fmt = TILFMT_PAGE;

	if (!block->phys_array) {
		kfree(block);
		return 0;
	}

	if (tcm_reserve_1d(containers[TILFMT_PAGE], size >> PAGE_SHIFT,
				&block->area)) {
		kfree(block->phys_array);
		kfree(block);
		return 0;
	}

	if (omap_dmm->alloc_debug)
		dev_info(omap_dmm->dev, "+1d: (%u,%u)-(%u,%u): %ld\n",
			block->area.p0.x, block->area.p0.y, block->area.p1.x,
			block->area.p1.y, block->num_pages*PAGE_SIZE);

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

	if (omap_dmm->alloc_debug)
		dev_info(omap_dmm->dev, "-%dd: (%u,%u)-(%u,%u)\n",
			((block->fmt == TILFMT_PAGE ) ? 1 : 2),
			block->area.p0.x, block->area.p0.y, block->area.p1.x,
			block->area.p1.y);

	kfree(block->phys_array);
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

uint32_t tiler_stride(enum tiler_fmt fmt)
{
	BUG_ON(!validfmt(fmt));

	return 1 << (CONT_WIDTH_BITS + geom[fmt].y_shft);
}
EXPORT_SYMBOL(tiler_stride);

size_t tiler_size(enum tiler_fmt fmt, uint16_t w, uint16_t h)
{
	BUG_ON(!validfmt(fmt));
	w = round_up(w, geom[fmt].slot_w);
	h = round_up(h, geom[fmt].slot_h);
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
static int omap_dmm_iommu_remove(struct platform_device *pdev)
{
	struct dmm *dmm;
	struct tiler_block *block;
	int i;

	dmm = platform_get_drvdata(pdev);

	if (dmm) {
		/* free all area regions */
		spin_lock(&omap_dmm->list_lock);
		list_for_each_entry(block, &omap_dmm->alloc_head, alloc_node) {
			kfree(block->phys_array);
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

		if (dmm->irq != -1)
			free_irq(dmm->irq, dmm);

		platform_set_drvdata(pdev, NULL);

#ifdef CONFIG_DEBUG_FS
		dmm_debugfs_remove();
#endif
		kfree(dmm);
	}

	return 0;
}

static int omap_dmm_iommu_probe(struct platform_device *pdev)
{
	int ret = -EFAULT, i;
	struct omap_dmm_platform_data *platdata = pdev->dev.platform_data;
	struct tcm_area area = {0};

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
	omap_dmm->num_engines = platdata->num_engines;
	omap_dmm->num_lut = platdata->num_lut;
	omap_dmm->lut_width = platdata->lut_width;
	omap_dmm->lut_height = platdata->lut_height;

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
	writel(0xfefe, omap_dmm->base + DMM_PAT_IRQENABLE_SET);

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

	for(i=0; i < omap_dmm->num_engines; i++) {
		omap_dmm->engines[i].id = i;
		omap_dmm->engines[i].dmm = omap_dmm;
		omap_dmm->engines[i].refill_va = omap_dmm->refill_va + 
						(REFILL_BUFFER_SIZE * i);
		omap_dmm->engines[i].refill_pa = omap_dmm->refill_pa + 
						(REFILL_BUFFER_SIZE * i);
		mutex_init(&omap_dmm->engines[i].mtx);
		init_waitqueue_head(&omap_dmm->engines[i].wait_for_refill);

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
		omap_dmm->tcm[i] = sita_init(omap_dmm->lut_width,
						omap_dmm->lut_height, NULL);

		if (!omap_dmm->tcm[i]) {
			dev_err(&pdev->dev, "failed to allocate container\n");
			ret = -ENOMEM;
			goto fail;
		}
	}

	/* assign access mode containers to applicable tcm container */
	containers[TILFMT_8BPP] = omap_dmm->tcm[0];
	containers[TILFMT_16BPP] = omap_dmm->tcm[0];
	containers[TILFMT_32BPP] = omap_dmm->tcm[0];
	containers[TILFMT_PAGE] = omap_dmm->tcm[0];

	INIT_LIST_HEAD(&omap_dmm->alloc_head);
	spin_lock_init(&omap_dmm->list_lock);

#ifdef CONFIG_DEBUG_FS
	dev_info(omap_dmm->dev, "init debugs\n");
	dmm_debugfs_create(omap_dmm);
#endif

	area = (struct tcm_area ) {
			.is2d = true,
			.tcm = containers[TILFMT_8BPP],
			.p1.x = omap_dmm->lut_width - 1,
			.p1.y = omap_dmm->lut_height - 1,
		};
	fill(&area, NULL, true);

	return 0;

fail:
	omap_dmm_iommu_remove(pdev);
	return ret;
}

#ifdef CONFIG_PM
static int omap_dmm_resume(struct device *pdev)
{
        struct tiler_block *block;
        struct tcm_area area = {0};

	area = (struct tcm_area ) {
			.is2d = true,
			.tcm = containers[TILFMT_8BPP],
			.p1.x = omap_dmm->lut_width - 1,
			.p1.y = omap_dmm->lut_height - 1,
		};
	fill(&area, NULL, true);

	/* iterate over all the blocks and refresh the PAT entries */
	list_for_each_entry(block, &omap_dmm->alloc_head, alloc_node) {
		if (fill(&block->area, block->phys_array, true)) {
				dev_err(omap_dmm->dev, "failed to restore "
					"PAT entries\n");
		}
	}

        return 0;
}

static const struct dev_pm_ops omap_dmm_pm_ops = {
	.resume = omap_dmm_resume,
};
#endif

static struct platform_driver omap_dmm_iommu_driver = {
	.driver = {
		.name = "dmm",
#ifdef CONFIG_PM
		.pm = &omap_dmm_pm_ops,
#endif
	},
	.probe		= omap_dmm_iommu_probe,
	.remove		= omap_dmm_iommu_remove,
};

static int __init omap_dmm_driver_init(void)
{
	return platform_driver_register(&omap_dmm_iommu_driver);
}

static void __exit omap_dmm_driver_exit(void)
{
	platform_driver_unregister(&omap_dmm_iommu_driver);
}

module_init(omap_dmm_driver_init);
module_exit(omap_dmm_driver_exit);

MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("Rob Clark <rob@ti.com>");
MODULE_AUTHOR("Andy Gross <andy.gross@ti.com>");
