/*
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
#ifndef OMAP_DMM_PRIV_H
#define OMAP_DMM_PRIV_H


#include <mach/dmm.h>

#define DMM_REVISION          0x000
#define DMM_HWINFO            0x004
#define DMM_LISA_HWINFO       0x008
#define DMM_DMM_SYSCONFIG     0x010
#define DMM_LISA_LOCK         0x01C
#define DMM_LISA_MAP__0       0x040
#define DMM_LISA_MAP__1       0x044
#define DMM_TILER_HWINFO      0x208
#define DMM_TILER_OR__0       0x220
#define DMM_TILER_OR__1       0x224
#define DMM_PAT_HWINFO        0x408
#define DMM_PAT_GEOMETRY      0x40C
#define DMM_PAT_CONFIG        0x410
#define DMM_PAT_VIEW__0       0x420
#define DMM_PAT_VIEW__1       0x424
#define DMM_PAT_VIEW_MAP__0   0x440
#define DMM_PAT_VIEW_MAP_BASE 0x460
#define DMM_PAT_IRQ_EOI       0x478
#define DMM_PAT_IRQSTATUS_RAW 0x480
#define DMM_PAT_IRQSTATUS     0x490
#define DMM_PAT_IRQENABLE_SET 0x4A0
#define DMM_PAT_IRQENABLE_CLR 0x4B0
#define DMM_PAT_STATUS__0     0x4C0
#define DMM_PAT_STATUS__1     0x4C4
#define DMM_PAT_DESCR__0      0x500
#define DMM_PAT_DESCR__1      0x510
#define DMM_PAT_AREA__0       0x504
#define DMM_PAT_AREA__1	      0x514
#define DMM_PAT_CTRL__0       0x508
#define DMM_PAT_CTRL__1       0x518
#define DMM_PAT_DATA__0       0x50C
#define DMM_PAT_DATA__1       0x51C
#define DMM_PEG_HWINFO        0x608
#define DMM_PEG_PRIO          0x620
#define DMM_PEG_PRIO_PAT      0x640

#define DMM_IRQSTAT_DST			1<<0
#define DMM_IRQSTAT_LST			1<<1
#define DMM_IRQSTAT_ERR_INV_DSC		1<<2
#define DMM_IRQSTAT_ERR_INV_DATA	1<<3
#define DMM_IRQSTAT_ERR_UPD_AREA	1<<4
#define DMM_IRQSTAT_ERR_UPD_CTRL	1<<5
#define DMM_IRQSTAT_ERR_UPD_DATA	1<<6
#define DMM_IRQSTAT_ERR_LUT_MISS	1<<7

#define DMM_IRQSTAT_ERR_MASK	(DMM_IRQ_STAT_ERR_INV_DSC | \
				DMM_IRQ_STAT_ERR_INV_DATA | \
				DMM_IRQ_STAT_ERR_UPD_AREA | \
				DMM_IRQ_STAT_ERR_UPD_CTRL | \
				DMM_IRQ_STAT_ERR_UPD_DATA | \
				DMM_IRQ_STAT_ERR_LUT_MISS )

#define DMM_PATSTATUS_READY		1<<0
#define DMM_PATSTATUS_VALID		1<<1
#define DMM_PATSTATUS_RUN		1<<2
#define DMM_PATSTATUS_DONE		1<<3
#define DMM_PATSTATUS_LINKED		1<<4
#define DMM_PATSTATUS_BYPASSED		1<<7
#define DMM_PATSTATUS_ERR_INV_DESCR	1<<10
#define DMM_PATSTATUS_ERR_INV_DATA	1<<11
#define DMM_PATSTATUS_ERR_UPD_AREA	1<<12
#define DMM_PATSTATUS_ERR_UPD_CTRL	1<<13
#define DMM_PATSTATUS_ERR_UPD_DATA	1<<14
#define DMM_PATSTATUS_ERR_ACCESS	1<<15

#define DMM_PATSTATUS_ERR	(DMM_PATSTATUS_ERR_INV_DESCR | \
				DMM_PATSTATUS_ERR_INV_DATA | \
				DMM_PATSTATUS_ERR_UPD_AREA | \
				DMM_PATSTATUS_ERR_UPD_CTRL | \
				DMM_PATSTATUS_ERR_UPD_DATA | \
				DMM_PATSTATUS_ERR_ACCESS )

enum {
	PAT_STATUS,
	PAT_DESCR
};

struct pat_ctrl {
	u32 start:4;
	u32 dir:4;
	u32 lut_id:8;
	u32 sync:12;
	u32 ini:4;
};

struct pat {
	uint32_t next_pa;
	struct pat_area area;
	struct pat_ctrl ctrl;
	uint32_t data_pa;
};

#define DMM_FIXED_RETRY_COUNT 1000

/* create refill buffer big enough to refill all slots, plus 3 descriptors..
 * 3 descriptors is probably the worst-case for # of 2d-slices in a 1d area,
 * but I guess you don't hit that worst case at the same time as full area
 * refill
 */
#define DESCR_SIZE 128
#define REFILL_BUFFER_SIZE ((4 * 128 * 256) + (3 * DESCR_SIZE))

struct dmm;

struct dmm_txn {
	void *engine_handle;

        uint8_t *current_va;
        dma_addr_t current_pa;

        struct pat *last_pat;
};

struct refill_engine {

	int id;
	struct dmm *dmm;

	uint8_t *refill_va;
	dma_addr_t refill_pa;

	/* only one trans per engine for now */
	struct dmm_txn txn;

	wait_queue_head_t wait_for_refill;
	struct mutex mtx;
};

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

	/* allocation list and lock */
	struct list_head alloc_head;
	spinlock_t list_lock;

	/* debug interfaces */
	bool alloc_debug;
};

struct tiler_block {
	struct list_head alloc_node;	/* node for global iova list */
	u32 *phys_array;		/* list of page physical addresses */
	u32 num_pages;
	struct tcm_area area;		/* area */
	enum tiler_fmt fmt;		/* format */
};

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

/* tiler space addressing bitfields */
#define MASK_XY_FLIP		(1 << 31)
#define MASK_Y_INVERT		(1 << 30)
#define MASK_X_INVERT		(1 << 29)
#define SHIFT_ACC_MODE		27
#define MASK_ACC_MODE		3

#define MASK(bits) ((1 << (bits)) - 1)

#define TILVIEW_8BIT    0x60000000u
#define TILVIEW_16BIT   (TILVIEW_8BIT  + VIEW_SIZE)
#define TILVIEW_32BIT   (TILVIEW_16BIT + VIEW_SIZE)
#define TILVIEW_PAGE    (TILVIEW_32BIT + VIEW_SIZE)
#define TILVIEW_END     (TILVIEW_PAGE  + VIEW_SIZE)

/* create tsptr by adding view orientation and access mode */
#define TIL_ADDR(x, orient, a)\
	((u32) (x) | (orient) | ((a) << SHIFT_ACC_MODE))


/* debugfs functions */
void dmm_debugfs_create(struct dmm *omap_dmm);
void dmm_debugfs_remove(void);
void print_allocation_map(struct seq_file *s, struct dmm *omap_dmm);

#endif
