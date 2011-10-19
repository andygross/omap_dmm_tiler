ccflags-y := -Werror

dmm_tiler-y := omap_dmm_tiler.o tcm-sita.o
dmm_tiler-$(CONFIG_DEBUG_FS) += tiler_debugfs.o

obj-$(CONFIG_OMAP_DMM_TILER) += dmm_tiler.o
