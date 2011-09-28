/*
 * dmm.h
 *
 * DMM driver support functions for TI DMM-TILER hardware block.
 *
 * Authors: David Sin <davidsin@ti.com>
 *          Rob Clark <rob.clark@linaro.org>
 *
 * Copyright (C) 2009-2011 Texas Instruments, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * * Redistributions of source code must retain the above copyright
 *   notice, this list of conditions and the following disclaimer.
 *
 * * Redistributions in binary form must reproduce the above copyright
 *   notice, this list of conditions and the following disclaimer in the
 *   documentation and/or other materials provided with the distribution.
 *
 * * Neither the name of Texas Instruments Incorporated nor the names of
 *   its contributors may be used to endorse or promote products derived
 *   from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef DMM_H
#define DMM_H

#include <linux/types.h>

/*
 * Note: other than omap_dmm_platform_data, maybe the rest of this should
 * move inside the omapdrm driver.. it isn't quite intended to be used
 * simultaneously by multiple drivers..
 */


/**
 * Area definition for DMM physical address translator.
 *
 * XXX this should move.. or better, change _append to use same rect struct
 * as tcm so we don't have to do so much parameter munging..
 */
struct pat_area {
	u32 x0:8;
	u32 y0:8;
	u32 x1:8;
	u32 y1:8;
};


/**
 * DMM device handle
 */
struct dmm;

/**
 * DMM transaction handle
 */
struct dmm_txn;


/**
 * Create and initialize the physical address translator.
 * @param id    PAT id
 * @return pointer to device data
 */
struct dmm * dmm_pat_init(u32 id);

/**
 * Get a handle for a DMM transaction
 */
struct dmm_txn * dmm_txn_init(struct dmm *dmm);

/**
 * Add region to DMM transaction.  If pages or pages[i] is NULL, then the
 * corresponding slot is cleared (ie. dummy_pa is programmed)
 */
int dmm_txn_append(struct dmm_txn *txn, struct pat_area *area,
		struct page **pages);

/**
 * Commit the DMM transaction.
 */
int dmm_txn_commit(struct dmm_txn *txn, bool wait);

/**
 * Clean up the physical address translator.
 * @param dmm    Device data
 * @return an error status.
 */
void dmm_pat_release(struct dmm *dmm);

/**
 * DMM Platform Device Data structure
 */
struct omap_dmm_platform_data {
	const char *oh_name;
	void __iomem *base;
	int irq;
};

/**
 * Only OMAP4 and later has DMM
 */
static inline bool has_dmm(void)
{
	return cpu_is_omap44xx();
}

#endif
