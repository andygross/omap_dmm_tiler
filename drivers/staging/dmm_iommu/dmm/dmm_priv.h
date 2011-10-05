/*
 * drivers/staging/omapdrm/dmm/dmm_priv.h
 *
 * Copyright (C) 2011 Texas Instruments
 * Author: Rob Clark <rob.clark@linaro.org>
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

#ifndef __DMM_PRIV_H__
#define __DMM_PRIV_H__

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
#define DMM_PAT_STATUS__2     0x4C8
#define DMM_PAT_STATUS__3     0x4CC
#define DMM_PAT_DESCR__0      0x500
#define DMM_PAT_AREA__0       0x504
#define DMM_PAT_CTRL__0       0x508
#define DMM_PAT_DATA__0       0x50C
#define DMM_PAT_DESCR__1      0x510
#define DMM_PAT_AREA__1       0x514
#define DMM_PAT_CTRL__1       0x518
#define DMM_PAT_DATA__1       0x51C
#define DMM_PEG_HWINFO        0x608
#define DMM_PEG_PRIO          0x620
#define DMM_PEG_PRIO_PAT      0x640

/**
 * PAT refill programming mode.
 */
enum pat_mode {
	MANUAL,
	AUTO
};

/**
 * DMM physical address translator control.
 */
struct pat_ctrl {
	u32 start:4;
	u32 dir:4;
	u32 lut_id:8;
	u32 sync:12;
	u32 ini:4;
};

/**
 * PAT descriptor.
 */
struct pat {
	u32 next_pa;
	struct pat_area area;
	struct pat_ctrl ctrl;
	u32 data_pa;
};

/**
 * DMM device data
 */
struct dmm {
	struct device *dev;
	void __iomem *base;

	/* dummy page for clearing unused DMM/PAT slots */
	struct page *dummy_pg;
	dma_addr_t dummy_pa;

	int ntxns;
	struct dmm_txn *txns;

	spinlock_t lock;
};

/* create refill buffer big enough to refill all slots, plus 3 descriptors..
 * 3 descriptors is probably the worst-case for # of 2d-slices in a 1d area,
 * but I guess you don't hit that worst case at the same time as full area
 * refill
 */
#define DESCR_SIZE 128
#define REFILL_BUFFER_SIZE ((4 * 128 * 256) + (3 * DESCR_SIZE))

/**
 * DMM transaction.
 */
struct dmm_txn {

	int id;
	struct dmm *dmm;

	/* buffer to use for PAT refill and PAT descriptor blocks
	 */
	uint8_t *refill_va;
	dma_addr_t refill_pa;

	uint8_t *current_va;
	dma_addr_t current_pa;

	struct pat *last_pat;
};

#endif /* __DMM_PRIV_H__ */
