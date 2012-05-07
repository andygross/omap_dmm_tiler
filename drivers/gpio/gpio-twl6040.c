/*
 * Access to GPOs on TWL6040 chip
 *
 * Copyright (C) 2012 Texas Instruments, Inc.
 *
 * Initial Code:
 *	Sergio Aguirre <saaguirre@ti.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307 USA
 */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/kthread.h>
#include <linux/irq.h>
#include <linux/gpio.h>
#include <linux/platform_device.h>
#include <linux/of.h>

#include <linux/mfd/twl6040.h>

static struct gpio_chip twl6040gpo_chip;

/*----------------------------------------------------------------------*/

static int twl6040gpo_direction_in(struct gpio_chip *chip, unsigned offset)
{
	/* This only drives GPOs, and can't change direction */
	return -EINVAL;
}

static int twl6040gpo_get(struct gpio_chip *chip, unsigned offset)
{
	struct twl6040 *twl6040 = dev_get_drvdata(chip->dev->parent);
	int ret = 0;

	ret = twl6040_reg_read(twl6040, TWL6040_REG_GPOCTL);
	if (ret < 0)
		return ret;

	return (ret >> offset) & 1;
}

static int twl6040gpo_direction_out(struct gpio_chip *chip, unsigned offset, int value)
{
	/* This only drives GPOs, and can't change direction */
	return 0;
}

static void twl6040gpo_set(struct gpio_chip *chip, unsigned offset, int value)
{
	struct twl6040 *twl6040 = dev_get_drvdata(chip->dev->parent);
	int ret;
	u8 gpoctl;

	ret = twl6040_reg_read(twl6040, TWL6040_REG_GPOCTL);
	if (ret < 0)
		return;

	gpoctl = ret & ~(1 << offset);
	gpoctl |= (((value > 0) ? 1 : 0) << offset);

	twl6040_reg_write(twl6040, TWL6040_REG_GPOCTL, gpoctl);
}

static struct gpio_chip twl6040gpo_chip = {
	.label			= "twl6040",
	.owner			= THIS_MODULE,
	.direction_input	= twl6040gpo_direction_in,
	.get			= twl6040gpo_get,
	.direction_output	= twl6040gpo_direction_out,
	.set			= twl6040gpo_set,
	.can_sleep		= 1,
};

/*----------------------------------------------------------------------*/

static int __devinit gpo_twl6040_probe(struct platform_device *pdev)
{
	struct twl6040_gpo_data *pdata = pdev->dev.platform_data;
	struct device *twl6040_core_dev = pdev->dev.parent;
	struct device_node *twl6040_core_node = NULL;
	int ret;

#ifdef CONFIG_OF
	twl6040_core_node = of_find_node_by_name(twl6040_core_dev->of_node,
						 "gpo");
#endif

	if (!pdata && !twl6040_core_node) {
		dev_err(&pdev->dev, "Platform data missing!\n");
		return -EINVAL;
	}

	if (pdata) {
		twl6040gpo_chip.base = pdata->gpio_base;
		twl6040gpo_chip.ngpio = pdata->nr_gpo;
	} else {
		u32 nr_gpio;

		twl6040gpo_chip.base = -1;
		of_property_read_u32(twl6040_core_node, "ti,nr_gpo",
				     &nr_gpio);
		twl6040gpo_chip.ngpio = nr_gpio;
	}

	twl6040gpo_chip.dev = &pdev->dev;
	dev_err(&pdev->dev, "base: %d, nr_gpo: %d\n", twl6040gpo_chip.base,
		twl6040gpo_chip.ngpio);
	ret = gpiochip_add(&twl6040gpo_chip);
	if (ret < 0) {
		dev_err(&pdev->dev, "could not register gpiochip, %d\n", ret);
		twl6040gpo_chip.ngpio = 0;
	}

	return ret;
}

static int __devexit gpo_twl6040_remove(struct platform_device *pdev)
{
	return gpiochip_remove(&twl6040gpo_chip);
}

/* Note:  this hardware lives inside an I2C-based multi-function device. */
MODULE_ALIAS("platform:twl6040-gpo");

static struct platform_driver gpo_twl6040_driver = {
	.driver = {
		.name	= "twl6040-gpo",
		.owner	= THIS_MODULE,
	},
	.probe		= gpo_twl6040_probe,
	.remove		= gpo_twl6040_remove,
};

module_platform_driver(gpo_twl6040_driver);

MODULE_AUTHOR("Texas Instruments, Inc.");
MODULE_DESCRIPTION("GPO interface for TWL6040");
MODULE_LICENSE("GPL");
