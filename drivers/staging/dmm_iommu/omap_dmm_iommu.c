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
#include <linux/io.h>              /* ioremap() */
#include <linux/errno.h>
#include <linux/sched.h>
#include <linux/wait.h>
#include <linux/interrupt.h>
#include <linux/dma-mapping.h>
#include <linux/slab.h>
#include <linux/delay.h>
#include <linux/mm.h>
#include <linux/dmapool.h>

#include "omap_dmm_priv.h"

struct dmm {
	struct device *dev;
	void __iomem *base;
	int irq;

	struct page *dummy_page;
	dma_addr_t dummy_pa;

	struct dma_pool *descr_pool;

	/* refill engines */
	struct  refill_engine *engines;
	int num_engines;
};

/* lookup table for registers w/ per-engine instances */
static const uint32_t reg[][4] = {
		[PAT_STATUS] = {DMM_PAT_STATUS__0, DMM_PAT_STATUS__1},
		[PAT_DESCR]  = {DMM_PAT_DESCR__0, DMM_PAT_DESCR__1},
		[PAT_AREA] = {DMM_PAT_AREA__0, DMM_PAT_AREA__1},
		[PAT_CTRL] = {DMM_PAT_CTRL__0, DMM_PAT_CTRL__1},
		[PAT_DATA] = {DMM_PAT_DATA__0, DMM_PAT_DATA__1},
};


/* check status and spin until wait_mask comes true */
static int wait_status(struct refill_engine *engine, uint32_t wait_mask)
{
	struct dmm *dmm = engine->dmm;
	uint32_t r = 0, err, i;

	i = DMM_FIXED_RETRY_COUNT;
	while (true) {
		r = readl(dmm->base + reg[PAT_STATUS][engine->id]);
		err = r & 0xfc00;
		if (err) {
			dev_err(dmm->dev, "error: %02x\n", err >> 10);
			return -EFAULT;
		}

		if ((r & wait_mask) == wait_mask)
			break;

		if (--i == 0) {
			dev_err(dmm->dev, "error: wait_status timed out\n");
			return -EFAULT;
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
		atomic_inc(&dmm->engines[0].irq_count);
	}

	if ((status >> 8) & DMM_IRQSTAT_LST) {
		/* refill engine 1 done */
		engine_done[1] = true;
		atomic_inc(&dmm->engines[1].irq_count);
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
struct dmm_txn * dmm_txn_init(struct dmm *dmm)
{
	struct dmm_txn *txn = NULL;

	/* for now, just use id 0, which we get from the kzalloc */
	txn = kzalloc(sizeof(*txn), GFP_KERNEL);
	txn->engine_handle = &dmm->engines[0];

	return txn;
}

/**
 * Add region to DMM transaction.  If pages or pages[i] is NULL, then the
 * corresponding slot is cleared (ie. dummy_pa is programmed)
 */
int dmm_txn_append(struct dmm_txn *txn, struct pat_area *area,
		struct page **pages)
{
	struct refill_engine *engine = txn->engine_handle;
	struct txn_allocation *txn_alloc = &txn->allocations[txn->num_slices];
	struct txn_allocation *prev_alloc;
	int i = (1 + area->x1 - area->x0) * (1 + area->y1 - area->y0);

	BUG_ON(txn->num_slices >= DMM_MAX_SLICES);

	txn_alloc->descr = dma_pool_alloc(engine->dmm->descr_pool,
					GFP_KERNEL, &txn_alloc->descr_pa);

	if (!txn_alloc->descr) {
		dev_err(engine->dmm->dev, "failed to get descriptor memory\n");
		goto fail;
	}

	txn_alloc->data_size = 4*i;
	txn_alloc->data = dma_alloc_coherent(engine->dmm->dev,
					txn_alloc->data_size,
					&txn_alloc->descr->data_pa, GFP_KERNEL);

	if (!txn_alloc->data) {
		dev_err(engine->dmm->dev, "failed to allocate pat data\n");
		goto fail_pool;
	}

	if (txn->num_slices) {
		prev_alloc = &txn->allocations[txn->num_slices-1];
		prev_alloc->descr->next_pa = (uint32_t)txn_alloc->descr_pa;
	}

	txn_alloc->descr->area = *area;
	txn_alloc->descr->ctrl = (struct pat_ctrl){
		.start = 1,
	};
	txn_alloc->descr->next_pa = (uint32_t)NULL;

	while (i--) {
		txn_alloc->data[i] = (pages && pages[i]) ?
				page_to_phys(pages[i]) : engine->dmm->dummy_pa;
	}

	txn->num_slices++;

	return 0;

fail_pool:
	dma_pool_free(engine->dmm->descr_pool, txn_alloc->descr,
			txn_alloc->descr_pa);
fail:
	return 1;
}

/**
 * Commit the DMM transaction.
 */
int dmm_txn_commit(struct dmm_txn *txn, bool wait)
{
	int ret, i;
	struct refill_engine *engine = txn->engine_handle;
	struct dmm *dmm = engine->dmm;

	if (!txn->num_slices) {
		dev_err(engine->dmm->dev, "need at least one txn\n");
		ret = -EINVAL;
		goto cleanup;
	}

	/* wait on the engine */
	mutex_lock(&engine->mtx);

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
	writel(txn->allocations[0].descr_pa, dmm->base + reg[PAT_DESCR][engine->id]);

	wait_event_interruptible_timeout(engine->wait_for_refill, 0, HZ / 1000);

	mutex_unlock(&engine->mtx);

cleanup:
	/* clean up transaction memory */
	for (i = 0; i < txn->num_slices; i++) {
		dma_free_coherent(engine->dmm->dev,
					txn->allocations[i].data_size,
					txn->allocations[i].data,
					txn->allocations[i].descr->data_pa);
		dma_pool_free(engine->dmm->descr_pool,
				txn->allocations[i].descr,
				txn->allocations[i].descr_pa);
	}

	kfree(txn);
	return ret;
}

static int omap_dmm_iommu_probe(struct platform_device *pdev)
{
	int ret = -EFAULT, i;
	struct dmm *dmm;
	struct omap_dmm_platform_data *platdata = pdev->dev.platform_data;
	struct dmm_txn *re;
	struct page *page_array;
	struct pat_area area = {0};

	printk(KERN_ERR "omap_dmm_iommu_probe\n");

	if (!platdata)
		return -ENODEV;

	dmm = kzalloc(sizeof(*dmm), GFP_KERNEL);
	if (!dmm) {
		pr_err("failed to allocate driver data section\n");
		goto fail;
	}

	/* HACK, fix the dma mask.... cannot rely on omap_device_build() */

	pdev->dev.coherent_dma_mask = DMA_BIT_MASK(32);
	dmm->base = platdata->base;
	dmm->irq = platdata->irq;
	dmm->dev = &pdev->dev;
	dmm->num_engines = platdata->num_engines;

	/* initialize DMM registers */
	writel(0x88888888, dmm->base + DMM_PAT_VIEW__0);
	writel(0x88888888, dmm->base + DMM_PAT_VIEW__1);
	writel(0x80808080, dmm->base + DMM_PAT_VIEW_MAP__0);
	writel(0x80000000, dmm->base + DMM_PAT_VIEW_MAP_BASE);
	writel(0x88888888, dmm->base + DMM_TILER_OR__0);
	writel(0x88888888, dmm->base + DMM_TILER_OR__1);

	ret = request_irq(dmm->irq, omap_dmm_irq_handler, IRQF_SHARED,
				"omap_dmm_irq_handler", dmm);

	if (ret) {
		dev_err(dmm->dev, "error: couldn't register IRQ %d, error %d\n",
			dmm->irq, ret);
		goto fail;
	}

	/* enable some interrupts! */
	writel(0xfefe, dmm->base + DMM_PAT_IRQENABLE_SET);

	dmm->descr_pool = dma_pool_create("dmm_descr", dmm->dev,
				32*sizeof(struct pat_descr), 16, 0);
	if (!dmm->descr_pool) {
		dev_err(dmm->dev, "could not allocate descriptor pool\n");
		ret = -ENOMEM;
		goto fail_irq;
	}

	dmm->dummy_page = alloc_page(GFP_KERNEL | __GFP_DMA32);
	if (!dmm->dummy_page) {
		dev_err(dmm->dev, "could not allocate dummy page\n");
		ret = -ENOMEM;
		goto fail_dmapool;
	}
	dmm->dummy_pa = page_to_phys(dmm->dummy_page);

	/* alloc engines */
	dmm->engines = kzalloc(dmm->num_engines * sizeof(struct refill_engine),
				GFP_KERNEL);
	if (!dmm->engines) {
		dev_err(dmm->dev, "could not allocate engines\n");
		ret = -ENOMEM;
		goto fail_page;
	}

	for(i=0; i < dmm->num_engines; i++) {
		dmm->engines[i].id = i;
		dmm->engines[i].dmm = dmm;

		mutex_init(&dmm->engines[i].mtx);
		init_waitqueue_head(&dmm->engines[i].wait_for_refill);

	}

	platform_set_drvdata(pdev, dmm);

	area.x1 = 255;
	area.y1 = 127;
	re = dmm_txn_init(dmm);
	page_array = kzalloc(256*128*sizeof(struct page*), GFP_KERNEL);
	if (!page_array) {
		dev_err(dmm->dev, "failed to initialize PAT entries with dummy page\n");
		goto fail_engines;
	}

	dmm_txn_append(re, &area, &page_array);
	dmm_txn_commit(re, true);

	kfree(page_array);

	return 0;

fail_engines:
	kfree(dmm->engines);

fail_dmapool:
	dma_pool_destroy(dmm->descr_pool);
fail_page:
	__free_page(dmm->dummy_page);
fail_irq:
	free_irq(dmm->irq, dmm);
fail:
	kfree(dmm);
	return ret;
}

static int omap_dmm_iommu_remove(struct platform_device *pdev)
{
	struct dmm *dmm = NULL;
	int i;

	dmm = platform_get_drvdata(pdev);

	if (dmm) {
		for(i=0; i < dmm->num_engines; i++)
			mutex_destroy(&dmm->engines[i].mtx);

		kfree(dmm->engines);
		__free_page(dmm->dummy_page);
		free_irq(dmm->irq, dmm);
		platform_set_drvdata(pdev, NULL);
		kfree(dmm);
	}

	return 0;
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
