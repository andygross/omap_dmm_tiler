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

#define DMM_BASE 0x4E000000
#define DMM_SIZE 0x800

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

	long elapsed; 
	wait_queue_head_t wait_for_refill;
	struct mutex mtx;
};

struct dmm_txn * dmm_txn_init(struct dmm *dmm);

int dmm_txn_append(struct dmm_txn *txn, struct pat_area *area,
		struct page **pages);

int dmm_txn_commit(struct dmm_txn *txn, bool wait);

#endif
