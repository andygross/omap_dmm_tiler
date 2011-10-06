#ifndef OMAP_DMM_PRIV_H
#define OMAP_DMM_PRIV_H
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


#include <mach/dmm.h>

enum {
	PAT_STATUS,
	PAT_DESCR,
	PAT_AREA,
	PAT_CTRL,
	PAT_DATA
};



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

#define DMM_MAX_SLICES 3
#define DMM_FIXED_RETRY_COUNT 1000

struct dmm;

struct txn_allocation {
	struct pat_descr *descr;
	dma_addr_t descr_pa;
	uint32_t *data;
	size_t data_size;
};

struct dmm_txn {
	void *engine_handle;

	struct txn_allocation allocations[DMM_MAX_SLICES];
	int num_slices;

	/* callback information */
	void *cb_fxn;
	void *cb_arg;
};

struct refill_engine {

	int id;
	struct dmm *dmm;

	wait_queue_head_t wait_for_refill;
	struct mutex mtx;

	atomic_t irq_count;
};

struct dmm_txn * dmm_txn_init(struct dmm *dmm);

int dmm_txn_append(struct dmm_txn *txn, struct pat_area *area,
		struct page **pages);

int dmm_txn_commit(struct dmm_txn *txn, bool wait);


#endif
