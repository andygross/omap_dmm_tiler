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

#include <mach/dmm.h>

struct pat_ctrl {
	u32 start:4;
	u32 dir:4;
	u32 lut_id:8;
	u32 sync:12;
	u32 ini:4;
};

struct pat_area {
        u32 x0:8;
        u32 y0:8;
        u32 x1:8;
        u32 y1:8;
};

struct pat_descr {
	uint32_t next_pa;
	struct pat_area area;
	struct pat_ctrl ctrl;
	uint32_t data_pa;
};

/* create refill buffer big enough to refill all slots, plus 3 descriptors..
 * 3 descriptors is probably the worst-case for # of 2d-slices in a 1d area,
 * but I guess you don't hit that worst case at the same time as full area
 * refill
 */
#define DESCR_SIZE 128
#define REFILL_BUFFER_SIZE ((4 * 128 * 256) + (3 * DESCR_SIZE))

struct dmm;

/**
 * DMM transaction.
 */
struct refill_engine {

	int id;
	struct dmm *dmm;

	/* buffer to use for PAT refill and PAT descriptor blocks
	 */
	uint8_t *refill_va;
	dma_addr_t refill_pa;

	uint8_t *current_va;
	dma_addr_t current_pa;

	struct pat_descr *last_pat;

	wait_queue_head_t wait_for_refill;
	struct mutex mtx;

	atomic_t irq_count;
};


struct dmm {
	struct device *dev;
	void __iomem *base;
	int irq;

	struct page *dummy_page;
	dma_addr_t dummy_pa;

	/* refill engines */
	struct  refill_engine *engines;
	int num_engines;

	spinlock_t lock;
};

enum {
	PAT_STATUS,
	PAT_DESCR,
	PAT_AREA,
	PAT_CTRL,
	PAT_DATA
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

	i = 1000;
	while (true) {
		r = readl(dmm->base + reg[PAT_STATUS][engine->id]);
		err = r & 0xfc00;
		if (err) {
			dev_err(dmm->dev, "error: %02x", err >> 10);
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

/* simple allocator to grab next 16 byte aligned memory from txn */
static void * alloc_dma(struct refill_engine *engine, size_t sz, dma_addr_t *pa)
{
	void *ptr;

	engine->current_pa = round_up(engine->current_pa, 16);
	engine->current_va = (void *)round_up((long)engine->current_va, 16);

	ptr = engine->current_va;
	*pa = engine->current_pa;

	engine->current_pa += sz;
	engine->current_va += sz;

	BUG_ON((engine->current_va - engine->refill_va) > REFILL_BUFFER_SIZE);

	return ptr;
}


/**
 * Get a handle for a DMM transaction
 */
struct refill_engine * dmm_txn_init(struct dmm *dmm)
{
	struct refill_engine *engine = &dmm->engines[0];
	int ret;

	/* wait on the engine */
	mutex_lock(&engine->mtx);

	/* wait for engine ready: */
	ret = wait_status(engine, 0x1);
	if (ret) {
		mutex_unlock(&engine->mtx);
		return ERR_PTR(ret);
	}

	engine->last_pat   = NULL;
	engine->current_va = engine->refill_va;
	engine->current_pa = engine->refill_pa;

	return engine;
}

/**
 * Add region to DMM transaction.  If pages or pages[i] is NULL, then the
 * corresponding slot is cleared (ie. dummy_pa is programmed)
 */
int dmm_txn_append(struct refill_engine *engine, struct pat_area *area,
		struct page **pages)
{
	dma_addr_t pat_pa = 0;
	uint32_t *data;
	struct pat_descr *descr;
	int i = (1 + area->x1 - area->x0) * (1 + area->y1 - area->y0);

	descr = alloc_dma(engine, sizeof(struct pat_descr), &pat_pa);

	if (engine->last_pat) {
		engine->last_pat->next_pa = (uint32_t)pat_pa;
	}

	descr->area = *area;
	descr->ctrl = (struct pat_ctrl){
		.start = 1,
	};

	data = alloc_dma(engine, 4*i, &descr->data_pa);

	while (i--) {
		data[i] = (pages && pages[i]) ?
				page_to_phys(pages[i]) : engine->dmm->dummy_pa;
	}

	engine->last_pat = descr;

	return 0;
}

/**
 * Commit the DMM transaction.
 *
 * XXX if irq's work, maybe pass a cb fxn instead.. if no cb, then wait?
 */
int dmm_txn_commit(struct refill_engine *engine, bool wait)
{
	struct dmm *dmm = engine->dmm;

	if (!engine->last_pat) {
		dev_err(engine->dmm->dev, "need at least one txn\n");
		mutex_unlock(&engine->mtx);
		return -EINVAL;
	}

	engine->last_pat->next_pa = 0;

	/* clear status */
	writel(0xff << (8 * engine->id), dmm->base + DMM_PAT_IRQSTATUS);

	/* kick reload */
	writel(engine->refill_pa, dmm->base + reg[PAT_DESCR][engine->id]);

	if (wait) {
		/* wait for engine done: */
		int ret = wait_status(engine, 0x3);
		if (ret) {
			mutex_unlock(&engine->mtx);
			return ret;
		}
	}

	mutex_unlock(&engine->mtx);
	return 0;
}

static int omap_dmm_iommu_probe(struct platform_device *pdev)
{
	int ret = 0, i;
	struct dmm *dmm;
	struct omap_dmm_platform_data *platdata = pdev->dev.platform_data;

	if (!platdata)
		return -ENODEV;

	dmm = kzalloc(sizeof(*dmm), GFP_KERNEL);
	if (!dmm) {
		pr_err("failed to allocate driver data section\n");
		goto fail;
	}

	dmm->base = platdata->base;
	dmm->irq = platdata->irq;
	dmm->dev = &pdev->dev;
	dmm->num_engines = platdata->num_engines;

	ret = request_irq(dmm->irq, omap_dmm_irq_handler, IRQF_SHARED,
				"omap_dmm_irq_handler", dmm);

	if (ret) {
		dev_err(dmm->dev, "error: couldn't register IRQ %d, error %d\n",
			dmm->irq, ret);
		goto fail;
	}

	dmm->dummy_page = alloc_page(GFP_KERNEL | __GFP_DMA32);
	if (!dmm->dummy_page) {
		dev_err(dmm->dev, "could not allocate dummy page\n");
		ret = -ENOMEM;
		goto fail_irq;
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
		dmm->engines[i].refill_va = dma_alloc_coherent(dmm->dev,
						REFILL_BUFFER_SIZE,
						&dmm->engines[i].refill_pa,
						GFP_KERNEL);
		if (!dmm->engines[i].refill_va) {
			dev_err(dmm->dev, "could not allocate refill buffer\n");
			goto fail_engines;
		}

		mutex_init(&dmm->engines[i].mtx);
		init_waitqueue_head(&dmm->engines[i].wait_for_refill);
	}

	platform_set_drvdata(pdev, dmm);

	return 0;

fail_engines:
	for(i=0; i < dmm->num_engines; i++) {
		if (dmm->engines && dmm->engines[i].refill_va)
			dma_free_coherent(dmm->dev, REFILL_BUFFER_SIZE,
					dmm->engines[i].refill_va,
					dmm->engines[i].refill_pa);
	}
	kfree(dmm->engines);

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
			if (dmm->engines && dmm->engines[i].refill_va) {
				dma_free_coherent(dmm->dev, REFILL_BUFFER_SIZE,
						dmm->engines[i].refill_va,
						dmm->engines[i].refill_pa);

				mutex_destroy(&dmm->engines[i].mtx);
			}
		}
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
