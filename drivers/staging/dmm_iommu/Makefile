obj-y := test/
ccflags-y := -Werror

omap_dmm-y := omap_dmm_iommu.o tcm-sita.o
omap_dmm-$(CONFIG_DEBUG_FS) += dmm_debugfs.o

obj-$(CONFIG_OMAP_DMM_IOMMU) += omap_dmm.o
