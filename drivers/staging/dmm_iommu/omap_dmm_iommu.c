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
#include <linux/delay.h>
#include <linux/time.h>

#include "omap_dmm_priv.h"
#include "tcm.h"
#include "tcm-sita.h"


struct dmm {
	struct device *dev;
	void __iomem *base;
	int irq;

	struct page *dummy_page;
	dma_addr_t dummy_pa;

	void *refill_va;
	dma_addr_t refill_pa;

	/* refill engines */
	struct  refill_engine *engines;
	int num_engines;

	/* container information */
	int lut_width;
	int lut_height;
	int num_lut;

	/* array of LUT - TCM containers */
	struct tcm **tcm;
};

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
                struct page **pages)
{
        dma_addr_t pat_pa = 0;
        uint32_t *data;
        struct pat *pat;
	struct refill_engine *engine = txn->engine_handle;
        int i = (1 + area->x1 - area->x0) * (1 + area->y1 - area->y0);

	dev_info(engine->dmm->dev, "appending %d pages at (%u,%u)-(%u,%u)\n",
			i, area->x0, area->y0, area->x1, area->y1);
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
                data[i] = (pages && pages[i]) ?
                                page_to_phys(pages[i]) : engine->dmm->dummy_pa;
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

static int fill(enum tiler_mode fmt, struct tcm_area *area,
		struct page **pages, bool wait)
{
	int ret = 0;
	struct tcm_area slice, area_s;
	struct dmm_txn *txn;

	BUG_ON(!validfmt(fmt));

	txn = dmm_txn_init(omap_dmm);
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
int omap_dmm_pin(enum tiler_mode fmt, struct tcm_area *area,
		struct page **pages, bool wait)
{
	int ret = fill(fmt, area, pages, wait);
	if (ret) {
		omap_dmm_unpin(fmt, area);
		return ret;
	}
	return 0;
}

int omap_dmm_unpin(enum tiler_mode fmt, struct tcm_area *area)
{
	return fill(fmt, area, NULL, false);
}

/*
 * Reserve/release
 */

/* note: w/h/align must be slot aligned */
struct tcm_area * omap_dmm_reserve_2d(enum tiler_mode fmt,
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

	ret = tcm_reserve_2d(containers[fmt], w, h, align, area);
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
	int ret = tcm_reserve_1d(containers[TILFMT_8BPP],
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
 *  driver functions
*/
static int omap_dmm_iommu_remove(struct platform_device *pdev)
{
	struct dmm *dmm;
	int i;

	dmm = platform_get_drvdata(pdev);

	if (dmm) {
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

	area = (struct tcm_area ) {
			.is2d = true,
			.tcm = containers[TILFMT_8BPP],
			.p1.x = omap_dmm->lut_width - 1,
			.p1.y = omap_dmm->lut_height - 1,
		};
	omap_dmm_unpin(TILFMT_8BPP, &area);

	return 0;

fail:
	omap_dmm_iommu_remove(pdev);
	return ret;
}

static struct platform_driver omap_dmm_iommu_driver = {
	.driver = {
		.name = "dmm",
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

subsys_initcall(omap_dmm_driver_init);
module_exit(omap_dmm_driver_exit);

MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("Rob Clark <rob@ti.com>");
MODULE_AUTHOR("Andy Gross <andy.gross@ti.com>");
