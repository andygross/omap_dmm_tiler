/*
 * include/drm/omap_priv.h
 *
 * Copyright (C) 2011 Texas Instruments
 * Author: Rob Clark <rob@ti.com>
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

#ifndef __OMAP_PRIV_H__
#define __OMAP_PRIV_H__

/* Non-userspace facing APIs
 */

#include <linux/types.h>
#include <drm/drmP.h>

/* interface that plug-in drivers (for now just PVR) can implement */
struct omap_drm_plugin {
	const char *name;

	/* drm functions */
	int (*open)(struct drm_device *dev, struct drm_file *file);
	int (*load)(struct drm_device *dev, unsigned long flags);
	int (*unload)(struct drm_device *dev);
	int (*release)(struct drm_device *dev, struct drm_file *file);

	struct drm_ioctl_desc *ioctls;
	int num_ioctls;
	int ioctl_base;

	struct list_head list;  /* note, this means struct can't be const.. */
};

int omap_drm_register_plugin(struct omap_drm_plugin *plugin);
int omap_drm_unregister_plugin(struct omap_drm_plugin *plugin);

int omap_drm_register_mapper(void);
void omap_drm_unregister_mapper(int id);

/* external mappers should get paddr or pages when it needs the pages pinned
 * and put when done..
 */
int omap_gem_get_paddr(struct drm_gem_object *obj,
		dma_addr_t *paddr, bool remap);
int omap_gem_put_paddr(struct drm_gem_object *obj);
int omap_gem_get_pages(struct drm_gem_object *obj, struct page ***pages);
int omap_gem_put_pages(struct drm_gem_object *obj);

uint32_t omap_gem_flags(struct drm_gem_object *obj);
void * omap_gem_priv(struct drm_gem_object *obj, int mapper_id);
void omap_gem_set_priv(struct drm_gem_object *obj, int mapper_id, void *priv);
uint64_t omap_gem_mmap_offset(struct drm_gem_object *obj);

/* for external plugin buffers wrapped as GEM object (via. omap_gem_new_ext())
 * a vm_ops struct can be provided to get callback notification of various
 * events..
 */
struct omap_gem_vm_ops {
	void (*open)(struct vm_area_struct * area);
	void (*close)(struct vm_area_struct * area);
	//maybe: int (*fault)(struct vm_area_struct *vma, struct vm_fault *vmf);

	/* note: mmap is not expected to do anything.. it is just to allow buffer
	 * allocate to update it's own internal state
	 */
	void (*mmap)(struct file *, struct vm_area_struct *);
};

struct drm_gem_object * omap_gem_new_ext(struct drm_device *dev,
		union omap_gem_size gsize, uint32_t flags,
		dma_addr_t paddr, struct page **pages,
		struct omap_gem_vm_ops *ops);

void omap_gem_op_update(void);
int omap_gem_op_start(struct drm_gem_object *obj, enum omap_gem_op op);
int omap_gem_op_finish(struct drm_gem_object *obj, enum omap_gem_op op);
int omap_gem_op_sync(struct drm_gem_object *obj, enum omap_gem_op op);
int omap_gem_op_async(struct drm_gem_object *obj, enum omap_gem_op op,
		void (*fxn)(void *arg), void *arg);
int omap_gem_set_sync_object(struct drm_gem_object *obj, void *syncobj);

/* optional platform data to configure the default configuration of which
 * pipes/overlays/CRTCs are used.. if this is not provided, then instead the
 * first CONFIG_DRM_OMAP_NUM_CRTCS are used, and they are each connected to
 * one manager, with priority given to managers that are connected to
 * detected devices.  This should be a good default behavior for most cases,
 * but yet there still might be times when you wish to do something different.
 */
struct omap_drm_platform_data {
	int ovl_cnt;
	const int *ovl_ids;
	int mgr_cnt;
	const int *mgr_ids;
	int dev_cnt;
	const char **dev_names;
};

#endif /* __OMAP_DRM_H__ */
