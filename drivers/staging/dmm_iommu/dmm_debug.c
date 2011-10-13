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
#include <linux/list.h>
#include <linux/debugfs.h>

#include "tcm.h"
#include "tcm-sita.h"


/*
 *  Debugfs support
 *  ==========================================================================
 */
struct tiler_debugfs_data {
	char name[17];
	void (*func)(struct seq_file *, u32 arg);
	u32 arg;
};

static void fill_map(char **map, int xdiv, int ydiv, struct tcm_area *a,
							char c, bool ovw)
{
	int x, y;
	for (y = a->p0.y / ydiv; y <= a->p1.y / ydiv; y++)
		for (x = a->p0.x / xdiv; x <= a->p1.x / xdiv; x++)
			if (map[y][x] == ' ' || ovw)
				map[y][x] = c;
}

static void fill_map_pt(char **map, int xdiv, int ydiv, struct tcm_pt *p,
									char c)
{
	map[p->y / ydiv][p->x / xdiv] = c;
}

static char read_map_pt(char **map, int xdiv, int ydiv, struct tcm_pt *p)
{
	return map[p->y / ydiv][p->x / xdiv];
}

static int map_width(int xdiv, int x0, int x1)
{
	return (x1 / xdiv) - (x0 / xdiv) + 1;
}

static void text_map(char **map, int xdiv, char *nice, int yd, int x0, int x1)
{
	char *p = map[yd] + (x0 / xdiv);
	int w = (map_width(xdiv, x0, x1) - strlen(nice)) / 2;
	if (w >= 0) {
		p += w;
		while (*nice)
			*p++ = *nice++;
	}
}

static void map_1d_info(char **map, int xdiv, int ydiv, char *nice,
							struct tcm_area *a)
{
	sprintf(nice, "%dK", tcm_sizeof(*a) * 4);
	if (a->p0.y + 1 < a->p1.y) {
		text_map(map, xdiv, nice, (a->p0.y + a->p1.y) / 2 / ydiv, 0,
							tiler.width - 1);
	} else if (a->p0.y < a->p1.y) {
		if (strlen(nice) < map_width(xdiv, a->p0.x, tiler.width - 1))
			text_map(map, xdiv, nice, a->p0.y / ydiv,
					a->p0.x + xdiv,	tiler.width - 1);
		else if (strlen(nice) < map_width(xdiv, 0, a->p1.x))
			text_map(map, xdiv, nice, a->p1.y / ydiv,
					0, a->p1.y - xdiv);
	} else if (strlen(nice) + 1 < map_width(xdiv, a->p0.x, a->p1.x)) {
		text_map(map, xdiv, nice, a->p0.y / ydiv, a->p0.x, a->p1.x);
	}
}

static void map_2d_info(char **map, int xdiv, int ydiv, char *nice,
							struct tcm_area *a)
{
	sprintf(nice, "(%d*%d)", tcm_awidth(*a), tcm_aheight(*a));
	if (strlen(nice) + 1 < map_width(xdiv, a->p0.x, a->p1.x))
		text_map(map, xdiv, nice, (a->p0.y + a->p1.y) / 2 / ydiv,
							a->p0.x, a->p1.x);
}

static void debug_allocation_map(struct seq_file *s, u32 arg)
{
	int xdiv = (arg >> 8) & 0xFF;
	int ydiv = arg & 0xFF;
	int i;
	char **map, *global_map;
	struct area_info *ai;
	struct mem_info *mi;
	struct tcm_area a, p;
	static char *m2d = "abcdefghijklmnopqrstuvwxyz"
					"ABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789";
	static char *a2d = ".,:;'\"`~!^-+";
	char *m2dp = m2d, *a2dp = a2d;
	char nice[128];

	/* allocate map */
	map = kzalloc(tiler.height / ydiv * sizeof(*map), GFP_KERNEL);
	global_map = kzalloc((tiler.width / xdiv + 1) * tiler.height / ydiv,
								GFP_KERNEL);
	if (!map || !global_map) {
		printk(KERN_ERR "could not allocate map for debug print\n");
		goto error;
	}
	memset(global_map, ' ', (tiler.width / xdiv + 1) * tiler.height / ydiv);
	for (i = 0; i < tiler.height / ydiv; i++) {
		map[i] = global_map + i * (tiler.width / xdiv + 1);
		map[i][tiler.width / xdiv] = 0;
	}

	/* get all allocations */
	mutex_lock(&mtx);

	list_for_each_entry(mi, &blocks, global) {
		if (mi->area.is2d) {
			ai = mi->parent;
			fill_map(map, xdiv, ydiv, &ai->area, *a2dp, false);
			fill_map(map, xdiv, ydiv, &mi->area, *m2dp, true);
			if (!*++a2dp)
				a2dp = a2d;
			if (!*++m2dp)
				m2dp = m2d;
			map_2d_info(map, xdiv, ydiv, nice, &mi->area);
		} else {
			bool start = read_map_pt(map, xdiv, ydiv, &mi->area.p0)
									== ' ';
			bool end = read_map_pt(map, xdiv, ydiv, &mi->area.p1)
									== ' ';
			tcm_for_each_slice(a, mi->area, p)
				fill_map(map, xdiv, ydiv, &a, '=', true);
			fill_map_pt(map, xdiv, ydiv, &mi->area.p0,
							start ? '<' : 'X');
			fill_map_pt(map, xdiv, ydiv, &mi->area.p1,
							end ? '>' : 'X');
			map_1d_info(map, xdiv, ydiv, nice, &mi->area);
		}
	}

	seq_printf(s, "BEGIN TILER MAP\n");
	for (i = 0; i < tiler.height / ydiv; i++)
		seq_printf(s, "%03d:%s\n", i * ydiv, map[i]);
	seq_printf(s, "END TILER MAP\n");

	mutex_unlock(&mtx);

error:
	kfree(map);
	kfree(global_map);
}

const struct tiler_debugfs_data debugfs_maps[] = {
	{ "1x1", debug_allocation_map, 0x0101 },
	{ "2x1", debug_allocation_map, 0x0201 },
	{ "4x1", debug_allocation_map, 0x0401 },
	{ "2x2", debug_allocation_map, 0x0202 },
	{ "4x2", debug_allocation_map, 0x0402 },
	{ "4x4", debug_allocation_map, 0x0404 },
};

static int tiler_debug_show(struct seq_file *s, void *unused)
{
	struct tiler_debugfs_data *fn = s->private;
	fn->func(s, fn->arg);
	return 0;
}

static int tiler_debug_open(struct inode *inode, struct file *file)
{
	return single_open(file, tiler_debug_show, inode->i_private);
}

static const struct file_operations tiler_debug_fops = {
	.open           = tiler_debug_open,
	.read           = seq_read,
	.llseek         = seq_lseek,
	.release        = single_release,
};


