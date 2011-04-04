#
# Makefile for the drm device driver.  This driver provides support for the
# Direct Rendering Infrastructure (DRI)
#

ccflags-y := -Iinclude/drm -Werror
omapdrm-y := omap_drv.o omap_crtc.o omap_encoder.o omap_connector.o omap_fb.o omap_fbdev.o omap_gem.o

obj-$(CONFIG_DRM_OMAP)	+= omapdrm.o
