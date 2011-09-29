/*
 * dmm.c
 *
 * DMM driver support functions for TI OMAP processors.
 *
 * Authors: David Sin <davidsin@ti.com>
 *          Lajos Molnar <molnar@ti.com>
 *          Rob Clark <rob.clark@linaro.org>
 *
 * Copyright (C) 2009-2011 Texas Instruments, Inc.
 *
 * This package is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * THIS PACKAGE IS PROVIDED ``AS IS'' AND WITHOUT ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, WITHOUT LIMITATION, THE IMPLIED
 * WARRANTIES OF MERCHANTIBILITY AND FITNESS FOR A PARTICULAR PURPOSE.
 */

#include <linux/init.h>
#include <linux/module.h>
#include <linux/platform_device.h> /* platform_device() */
#include <linux/io.h>              /* ioremap() */
#include <linux/errno.h>
#include <linux/slab.h>
#include <linux/delay.h>

#include <plat/dmm.h>

#include "dmm_priv.h"
#include "../omap_drv.h"

enum {
	PAT_STATUS,
	PAT_DESCR,
};

/* lookup table for registers w/ per-engine instances */
static const uint32_t reg[][4] = {
		[PAT_STATUS] = {DMM_PAT_STATUS__0, DMM_PAT_STATUS__1},
		[PAT_DESCR]  = {DMM_PAT_DESCR__0, DMM_PAT_DESCR__1},
};

/* simple allocator to grab next 16 byte aligned memory from txn */
static void * alloc_dma(struct dmm_txn *txn, size_t sz, dma_addr_t *pa)
{
	void *ptr;

	txn->current_pa = round_up(txn->current_pa, 16);
	txn->current_va = (void *)round_up((long)txn->current_va, 16);

	ptr = txn->current_va;
	*pa = txn->current_pa;

	txn->current_pa += sz;
	txn->current_va += sz;

	BUG_ON((txn->current_va - txn->refill_va) > REFILL_BUFFER_SIZE);

	return ptr;
}

/* check status and spin until wait_mask comes true */
static int wait_status(struct dmm_txn *txn, uint32_t wait_mask)
{
	struct dmm *dmm = txn->dmm;
	uint32_t r = 0, err;

	while (true) {
		r = readl(dmm->base + reg[PAT_STATUS][txn->id]);
		err = r & 0xfc00;
		if (err) {
			dev_err(dmm->dev, "error: %02x", err >> 10);
			return -EFAULT;
		}
		if ((r & wait_mask) == wait_mask)
			break;
		udelay(1);
	}

	return 0;
}


/**
 * Get a handle for a DMM transaction
 */
struct dmm_txn * dmm_txn_init(struct dmm *dmm)
{
	struct dmm_txn *txn = &dmm->txns[0];
	int ret;

	/* wait for engine ready: */
	ret = wait_status(txn, 0x1);
	if (ret) {
		return ERR_PTR(ret);
	}

	txn->last_pat   = NULL;
	txn->current_va = txn->refill_va;
	txn->current_pa = txn->refill_pa;

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
				page_to_phys(pages[i]) : txn->dmm->dummy_pa;
	}

	txn->last_pat = pat;

	return 0;
}

/**
 * Commit the DMM transaction.
 *
 * XXX if irq's work, maybe pass a cb fxn instead.. if no cb, then wait?
 */
int dmm_txn_commit(struct dmm_txn *txn, bool wait)
{
	struct dmm *dmm = txn->dmm;

	if (!txn->last_pat) {
		dev_err(txn->dmm->dev, "need at least one txn\n");
		return -EINVAL;
	}

	txn->last_pat->next_pa = 0;

	/* clear status */
	writel(0xff << (8 * txn->id), dmm->base + DMM_PAT_IRQSTATUS);

//	dsb();

	/* kick reload */
	writel(txn->refill_pa, dmm->base + reg[PAT_DESCR][txn->id]);

	if (wait) {
		/* wait for engine done: */
		int ret = wait_status(txn, 0x3);
		if (ret) {
			return ret;
		}
	}

	return 0;
}

static irqreturn_t dmm_irq_handler(int irq, void *arg)
{
	struct dmm *dmm = arg;
	uint32_t status = readl(dmm->base + DMM_PAT_IRQSTATUS);
	if (status & 0x0002) {
		/* FILL_LST0 */
		// TODO: callback
	} else if (status & 0x0200) {
		/* FILL_LST1 */
		// TODO: callback
	} else {
		DBG("status: %08x", status);
	}
	writel(status, dmm->base + DMM_PAT_IRQSTATUS);
	return IRQ_HANDLED;
}

static int txn_init(struct dmm *dmm, struct dmm_txn *txn, int id)
{
	txn->id = id;
	txn->dmm = dmm;
	txn->refill_va = dma_alloc_coherent(dmm->dev, REFILL_BUFFER_SIZE,
			&txn->refill_pa, GFP_KERNEL);
	if (!txn->refill_va) {
		dev_err(dmm->dev, "could not allocate refill buffer\n");
		return -ENOMEM;
	}

	return 0;
}

static struct omap_dmm_platform_data *device_data;
static struct device *dmm_dev;

struct dmm * dmm_pat_init(u32 id)
{
	struct dmm *dmm = kmalloc(sizeof(*dmm), GFP_KERNEL);
	int i, ret = 0;

	if (!dmm)
		return NULL;

	dmm->base = device_data->base;
	if (!dmm->base) {
		dev_err(dmm->dev, "no base address\n");
		ret = -ENOENT;
		goto fail;
	}

	dmm->dev = dmm_dev;

	spin_lock_init(&dmm->lock);

	dmm->dummy_pg = alloc_page(GFP_KERNEL | __GFP_DMA32);
	if (!dmm->dummy_pg) {
		dev_err(dmm->dev, "could not allocate dummy page\n");
		ret = -ENOMEM;
		goto fail;
	}
	dmm->dummy_pa = page_to_phys(dmm->dummy_pg);

	/* for now, we just support a single pending transaction.. the slightly
	 * more fancy version would use two, one per refill engine.  An even
	 * more fancy version would use more, queue up txn's when both refill
	 * engines are active, and kick of next queued txn when we get irqs
	 * back from DMM..
	 */

	dmm->ntxns = 1;
	dmm->txns = kzalloc(dmm->ntxns * sizeof(struct dmm_txn), GFP_KERNEL);
	for (i = 0; i < dmm->ntxns; i++) {
		ret = txn_init(dmm, &dmm->txns[i], i);
		if (ret) {
			goto fail;
		}
	}

	writel(0x88888888, dmm->base + DMM_PAT_VIEW__0);
	writel(0x88888888, dmm->base + DMM_PAT_VIEW__1);
	writel(0x80808080, dmm->base + DMM_PAT_VIEW_MAP__0);
	writel(0x80000000, dmm->base + DMM_PAT_VIEW_MAP_BASE);
	writel(0x88888888, dmm->base + DMM_TILER_OR__0);
	writel(0x88888888, dmm->base + DMM_TILER_OR__1);

	ret = request_irq(device_data->irq, dmm_irq_handler,
			IRQF_SHARED, "OMAP DMM", dmm);
	if (ret) {
		printk(KERN_ERR "dmm: could not request irq!!\n");
		/* hmm, cleanup! Or maybe we can fall back to polling?  Or? */
	}

	/* enable some interrupts! */
	writel(0xfefe, dmm->base + DMM_PAT_IRQENABLE_SET);

	return dmm;

fail:
	// XXX cleanup
	return ERR_PTR(ret);
}

/**
 * Clean up the physical address translator.
 * @param dmm    Device data
 * @return an error status.
 */
void dmm_pat_release(struct dmm *dmm)
{
	if (dmm) {
		iounmap(dmm->base);
		kfree(dmm);
	}
}


static int dmm_probe(struct platform_device *pdev)
{
	if (!pdev || !pdev->dev.platform_data) {
		printk(KERN_ERR "dmm: invalid platform data\n");
		return -EINVAL;
	}

	// XXX hack because we can't rely on omap_device_build() to do this:
	pdev->dev.coherent_dma_mask = DMA_BIT_MASK(32);

	dmm_dev = &pdev->dev;
	device_data = pdev->dev.platform_data;

	printk(KERN_INFO "dmm: probe base: %p, irq %d\n",
		device_data->base, device_data->irq);
	writel(0x88888888, device_data->base + DMM_TILER_OR__0);
	writel(0x88888888, device_data->base + DMM_TILER_OR__1);

	return 0;
}

static struct platform_driver dmm_driver_ldm = {
	.probe = dmm_probe,
	.driver = {
		.owner = THIS_MODULE,
		.name = "dmm",
	},
};

static s32 __init dmm_init(void)
{
	return platform_driver_register(&dmm_driver_ldm);
}

static void __exit dmm_exit(void)
{
	platform_driver_unregister(&dmm_driver_ldm);
}

MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("davidsin@ti.com");
MODULE_AUTHOR("molnar@ti.com");
MODULE_AUTHOR("rob.clark@linaro.org");
module_init(dmm_init);
module_exit(dmm_exit);
