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

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/io.h>
#include <linux/clk.h>
#include <linux/iommu.h>
#include <linux/interrupt.h>
#include <linux/err.h>
#include <linux/slab.h>

#include <mach/dmm.h>


/*
 * IRQ handler
 */
irqreturn_t omap_dmm_irq_handler(int irq, void *dev_id)
{
	struct omap_dmm_iommu_drvdata *drvdata = dev_id;

	if (!drvdata) {
		pr_err("Invalid data structure\n");
		goto fail;
	}

	pr_err("entered irq\n");

fail:
	return 0;
}

static int omap_dmm_iommu_probe(struct platform_device *pdev)
{
	int ret = 0;
	struct omap_dmm_iommu_drvdata *drvdata;
	struct omap_dmm_platform_data *platdata = pdev->dev.platform_data;

	if (!platdata)
		return -ENODEV;

	drvdata = kzalloc(sizeof(*drvdata), GFP_KERNEL);
	if (!drvdata) {
		pr_err("failed to allocate driver data section\n");
		goto fail;
	}

	drvdata->base = platdata->base;
	drvdata->irq = platdata->irq;

	ret = request_irq(drvdata->irq, omap_dmm_irq_handler, 0,
				"omap_dmm_irq_handler", drvdata);

	if (ret) {
		pr_err("Failed to request DMM IRQ %d, ret=%d\n", drvdata->irq,
			ret);
		goto fail;
	}

	platform_set_drvdata(pdev, drvdata);

	return 0;

fail:
	kfree(drvdata);
	return ret;
}

static int omap_dmm_iommu_remove(struct platform_device *pdev)
{
	struct omap_dmm_iommu_drvdata *drvdata = NULL;

	drvdata = platform_get_drvdata(pdev);

	if (drvdata) {
		free_irq(drvdata->irq, drvdata);
		platform_set_drvdata(pdev, NULL);
		kfree(drvdata);
	}

	return 0;
}

static struct platform_driver omap_dmm_iommu_driver = {
	.driver = {
		.name = "dmm",
	},
	.probe		= omap_dmm_iommu_probe,
	.remove		= omap_dmm_iommu_remove,
};

static int __init omap_dmm_driver_init(void)
{
	return platform_driver_register(&omap_dmm_iommu_driver);
}

static void __exit omap_dmm_driver_exit(void)
{
	platform_driver_unregister(&omap_dmm_iommu_driver);
}

subsys_initcall(omap_dmm_driver_init);
module_exit(omap_dmm_driver_exit);

MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("Andy Gross <andy.gross@ti.com>");
