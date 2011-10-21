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
#include <linux/slab.h>
#include <linux/list.h>
#include <linux/debugfs.h>
#include <linux/seq_file.h>

#include "tcm.h"
#include "omap_dmm_tiler.h"

static struct dentry *dfs_root;
static struct dentry *dfs_map;

static const char *alphabet = "abcdefghijklmnopqrstuvwxyz"
				"ABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789";
static const char *special = ".,:;'\"`~!^-+";

/*
 *  Debugfs support
 *  ==========================================================================
 */

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
							256 - 1);
	} else if (a->p0.y < a->p1.y) {
		if (strlen(nice) < map_width(xdiv, a->p0.x, 256 - 1))
			text_map(map, xdiv, nice, a->p0.y / ydiv,
					a->p0.x + xdiv,	256 - 1);
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

static int tiler_debug_show(struct seq_file *s, void *arg)
{
	print_allocation_map(s, s->private);
	return 0;
}

void print_allocation_map(void *sfile, struct dmm *omap_dmm)
{
	struct seq_file *s = (struct seq_file *)sfile;
	int xdiv = 2, ydiv = 1;
	char **map = NULL, *global_map;
	struct tiler_block *block;
	struct tcm_area a, p;
	int i;
	const char *m2d = alphabet;
	const char *a2d = special;
	const char *m2dp = m2d, *a2dp = a2d;
	char nice[128];
	int h_adj = omap_dmm->lut_height / ydiv;
	int w_adj = omap_dmm->lut_width / xdiv;

	map = kzalloc(h_adj * sizeof(*map), GFP_KERNEL);
	global_map = kzalloc((w_adj + 1) * h_adj, GFP_KERNEL);

	if (!map || !global_map)
		goto error;

	memset(global_map, ' ', (w_adj + 1) * h_adj);
	for (i = 0; i < omap_dmm->lut_height; i++) {
		map[i] = global_map + i * (w_adj + 1);
		map[i][w_adj] = 0;
	}
	spin_lock(&omap_dmm->list_lock);

	list_for_each_entry(block, &omap_dmm->alloc_head, alloc_node) {
		if (block->fmt != TILFMT_PAGE) {
			fill_map(map, xdiv, ydiv, &block->area, *m2dp, true);
			if (!*++a2dp)
				a2dp = a2d;
			if (!*++m2dp)
				m2dp = m2d;
			map_2d_info(map, xdiv, ydiv, nice, &block->area);
		} else {
			bool start = read_map_pt(map, xdiv, ydiv,
							&block->area.p0)
									== ' ';
			bool end = read_map_pt(map, xdiv, ydiv, &block->area.p1)
									== ' ';
			tcm_for_each_slice(a, block->area, p)
				fill_map(map, xdiv, ydiv, &a, '=', true);
			fill_map_pt(map, xdiv, ydiv, &block->area.p0,
							start ? '<' : 'X');
			fill_map_pt(map, xdiv, ydiv, &block->area.p1,
							end ? '>' : 'X');
			map_1d_info(map, xdiv, ydiv, nice, &block->area);
		}
	}

	spin_unlock(&omap_dmm->list_lock);

	if (s) {
		seq_printf(s, "BEGIN DMM TILER MAP\n");
		for (i = 0; i < 128; i++)
			seq_printf(s, "%03d:%s\n", i, map[i]);
		seq_printf(s, "END TILER MAP\n");
	} else {
		dev_dbg(omap_dmm->dev, "BEGIN DMM TILER MAP\n");
		for (i = 0; i < 128; i++)
			dev_dbg(omap_dmm->dev, "%03d:%s\n", i, map[i]);
		dev_dbg(omap_dmm->dev, "END TILER MAP\n");
	}

error:
	kfree(map);
	kfree(global_map);
}

static int tiler_debug_open(struct inode *inode, struct file *file)
{
	return single_open(file, tiler_debug_show, inode->i_private);
}

static const struct file_operations tiler_debug_fops = {
	.open		= tiler_debug_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= single_release,
};

void dmm_debugfs_create(struct dmm *omap_dmm)
{

	dfs_root = debugfs_create_dir("dmm_tiler", NULL);
	if (IS_ERR_OR_NULL(dfs_root))
		dev_warn(omap_dmm->dev, "failed to create debug files\n");
	else {
		dfs_map = debugfs_create_file("map", S_IRUGO,
				dfs_root, omap_dmm, &tiler_debug_fops);

		debugfs_create_bool("alloc_debug", S_IRUGO | S_IWUSR, dfs_root,
					(u32 *)&omap_dmm->alloc_debug);
	}
}

void dmm_debugfs_remove(void)
{
	if (dfs_root)
		debugfs_remove_recursive(dfs_root);
}
