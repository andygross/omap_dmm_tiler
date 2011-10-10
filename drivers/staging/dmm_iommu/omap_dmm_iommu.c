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
#include <linux/delay.h>

#include "omap_dmm_priv.h"

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
static int wait_status(struct refill_engine *engine, uint32_t wait_mask,
			uint32_t timeout)
{
	struct dmm *dmm = engine->dmm;
	uint32_t r = 0, err, i;

	i = timeout;
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
			dev_err(dmm->dev, "timed out waiting for status\n");
			break;
		}
		udelay(1);
	}

	engine->elapsed = timeout - i;
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
struct dmm_txn * dmm_txn_init(struct dmm *dmm)
{
	struct dmm_txn *txn = NULL;

	txn = kzalloc(sizeof(*txn), GFP_KERNEL);
	if (!txn) {
		dev_err(dmm->dev, "failed to allocate txn memory\n");
		goto err;
	}

	mutex_lock(&dmm->engines[0].mtx);
	txn->engine_handle = &dmm->engines[0];
	txn->last_pat = NULL;
	txn->current_va = dmm->engines[0].refill_va;
	txn->current_pa = dmm->engines[0].refill_pa;

err:
	return txn;
}

/**
 * Add region to DMM transaction.  If pages or pages[i] is NULL, then the
 * corresponding slot is cleared (ie. dummy_pa is programmed)
 */
int dmm_txn_append(struct dmm_txn *txn, struct pat_area *area,
                struct page **pages)
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
                data[i] = (pages && pages[i]) ?
                                page_to_phys(pages[i]) : engine->dmm->dummy_pa;
        }

        txn->last_pat = pat;

        return 0;
}

/**
 * Commit the DMM transaction.
 */
int dmm_txn_commit(struct dmm_txn *txn, bool wait)
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
	ret = wait_status(engine, DMM_PATSTATUS_READY, DMM_FIXED_RETRY_COUNT);
	if (ret) {
		ret = -EFAULT;
		mutex_unlock(&engine->mtx);
		goto cleanup;
	}

	/* kick reload */
	writel(engine->refill_pa,
		dmm->base + reg[PAT_DESCR][engine->id]);

	if (wait) {
		wait_event_interruptible_timeout(engine->wait_for_refill, 0,
				msecs_to_jiffies(1));

		ret = wait_status(engine, DMM_PATSTATUS_READY, 1);
	}

cleanup:
	mutex_unlock(&engine->mtx);
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

	dmm->dummy_page = alloc_page(GFP_KERNEL | __GFP_DMA32);
	if (!dmm->dummy_page) {
		dev_err(dmm->dev, "could not allocate dummy page\n");
		ret = -ENOMEM;
		goto fail_irq;
	}
	dmm->dummy_pa = page_to_phys(dmm->dummy_page);

	/* alloc refill memory */
	dmm->refill_va = dma_alloc_coherent(dmm->dev,
				REFILL_BUFFER_SIZE * dmm->num_engines,
				&dmm->refill_pa, GFP_KERNEL);
	if (!dmm->refill_va) {
		dev_err(dmm->dev, "could not allocate refill memory\n");
		goto fail_page;
	}

	/* alloc engines */
	dmm->engines = kzalloc(dmm->num_engines * sizeof(struct refill_engine),
				GFP_KERNEL);
	if (!dmm->engines) {
		dev_err(dmm->dev, "could not allocate engines\n");
		ret = -ENOMEM;
		goto fail_refill;
	}

	for(i=0; i < dmm->num_engines; i++) {
		dmm->engines[i].id = i;
		dmm->engines[i].dmm = dmm;
		dmm->engines[i].refill_va = dmm->refill_va + 
						(REFILL_BUFFER_SIZE * i);
		dmm->engines[i].refill_pa = dmm->refill_pa + 
						(REFILL_BUFFER_SIZE * i);
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
fail_refill:
	dma_free_coherent(dmm->dev, REFILL_BUFFER_SIZE * dmm->num_engines,
				dmm->refill_va, dmm->refill_pa);
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
		for(i=0; i < dmm->num_engines; i++) {
			mutex_destroy(&dmm->engines[i].mtx);
		}
		dma_free_coherent(dmm->dev,
				REFILL_BUFFER_SIZE * dmm->num_engines,
				dmm->refill_va, dmm->refill_pa);
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
