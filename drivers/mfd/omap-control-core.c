/*
 * OMAP system control module driver file
 *
 * Copyright (C) 2011-2012 Texas Instruments Incorporated - http://www.ti.com/
 * Contacts:
 * Based on original code written by:
 *    J Keerthy <j-keerthy@ti.com>
 *    Moiz Sonasath <m-sonasath@ti.com>
 * MFD clean up and re-factoring:
 *    Eduardo Valentin <eduardo.valentin@ti.com>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License for more details.
 *
 */

#include <linux/module.h>
#include <linux/export.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/io.h>
#include <linux/err.h>
#include <linux/of_platform.h>
#include <linux/of_address.h>
#include <linux/mfd/core.h>
#include <linux/mfd/omap_control.h>

static struct omap_control *omap_control_module;

/**
 * omap_control_readl: Read a single omap control module register
 * for general core partition.
 * TODO: should be renamed to omap_control_core_readl
 *
 * @dev: device to read from.
 * @reg: register to read.
 * @val: output with register value.
 *
 * returns 0 on success or -EINVAL in case struct device is invalid.
 */
int omap_control_readl(struct device *dev, u32 reg, u32 *val)
{
	struct omap_control *omap_control = dev_get_drvdata(dev);

	if (!omap_control)
		return -EINVAL;

	*val = __raw_readl(omap_control->base + reg);

	return 0;
}
EXPORT_SYMBOL_GPL(omap_control_readl);

/**
 * omap_control_core_pad_readl: Read a single omap control module register
 * for core pad partition.
 *
 * @dev: device to read from.
 * @reg: register to read.
 * @val: output with register value.
 *
 * returns 0 on success or -EINVAL in case struct device is invalid.
 */
int omap_control_core_pad_readl(struct device *dev, u32 reg, u32 *val)
{
	struct omap_control *omap_control = dev_get_drvdata(dev);

	if (!omap_control)
		return -EINVAL;

	*val = __raw_readl(omap_control->base_pad + reg);

	return 0;
}
EXPORT_SYMBOL_GPL(omap_control_core_pad_readl);

/**
 * omap_control_writel: Write a single omap control module register
 * for general core partition.
 * TODO: should be renamed to omap_control_core_writel
 *
 * @dev: device to read from.
 * @val: value to write.
 * @reg: register to write to.
 *
 * returns 0 on success or -EINVAL in case struct device is invalid.
 */
int omap_control_writel(struct device *dev, u32 val, u32 reg)
{
	struct omap_control *omap_control = dev_get_drvdata(dev);

	if (!omap_control)
		return -EINVAL;

	spin_lock(&omap_control->reglock);
	__raw_writel(val, omap_control->base + reg);
	spin_unlock(&omap_control->reglock);

	return 0;
}
EXPORT_SYMBOL_GPL(omap_control_writel);

/**
 * omap_control_core_pad_writel: Write a single omap control module register
 * for core pad partition.
 *
 * @dev: device to read from.
 * @val: value to write.
 * @reg: register to write to.
 *
 * returns 0 on success or -EINVAL in case struct device is invalid.
 */
int omap_control_core_pad_writel(struct device *dev, u32 val, u32 reg)
{
	struct omap_control *omap_control = dev_get_drvdata(dev);

	if (!omap_control)
		return -EINVAL;

	spin_lock(&omap_control->reglock);
	__raw_writel(val, omap_control->base_pad + reg);
	spin_unlock(&omap_control->reglock);

	return 0;
}
EXPORT_SYMBOL_GPL(omap_control_core_pad_writel);

/**
 * omap_control_get: returns the control module device pinter
 *
 * The modules which has to use control module API's to read or write should
 * call this API to get the control module device pointer.
 */
struct device *omap_control_get(void)
{
	if (!omap_control_module)
		return ERR_PTR(-ENODEV);

	spin_lock(&omap_control_module->reglock);
	omap_control_module->use_count++;
	spin_unlock(&omap_control_module->reglock);

	return omap_control_module->dev;
}
EXPORT_SYMBOL_GPL(omap_control_get);

/**
 * omap_control_put: returns the control module device pinter
 *
 * The modules which has to use control module API's to read or write should
 * call this API to get the control module device pointer.
 */
void omap_control_put(struct device *dev)
{
	struct omap_control *omap_control = dev_get_drvdata(dev);

	if (!omap_control)
		return;

	spin_lock(&omap_control->reglock);
	omap_control->use_count--;
	spin_unlock(&omap_control->reglock);
}
EXPORT_SYMBOL_GPL(omap_control_put);


static struct mfd_cell omap4_control_usb_devs[] = {
	{
		.name = "omap-control-usb",
	},
};

static struct resource omap4_bandgap_resources[] = {
	{
		.start = 0,
		.end   = 0,
		.flags = IORESOURCE_IRQ,
	},
};

static struct omap_bdg_data omap4_bg_data = {
	.rev		= 1,
	.cnt		= 1,
	.tshut_gpio	= 86,
	.accurate	= true,
};

static struct omap_bdg_data omap5_bg_data = {
	.rev		= 2,
	.cnt		= 3,
	.tshut_gpio	= 86,
	.accurate	= true,
};

static struct mfd_cell omap_bandgap_devs[] = {
	{
		.name = "omap-bandgap",
		.num_resources = ARRAY_SIZE(omap4_bandgap_resources),
		.resources = omap4_bandgap_resources,
	},
};

static const struct of_device_id of_omap_control_match[] = {
	{ .compatible = "ti,omap3-control", },
	{ .compatible = "ti,omap4-control", },
	{ .compatible = "ti,omap5-control", },
	{ },
};

static int omap_control_add_children(struct platform_device *pdev,
				     struct omap_control *omap_control)
{
	int ret;
	int thermal_irq;
	struct device *dev = &pdev->dev;
	struct omap_control_data *pdata = dev_get_platdata(dev);

	if (!pdata) {
		dev_err(dev, "No Children for the mfd device: Fail probe\n");
		return -ENODEV;
	}

	/* USB-PHY */
	if (pdata->has_usb_phy) {
		ret = mfd_add_devices(dev, -1, omap4_control_usb_devs,
				      ARRAY_SIZE(omap4_control_usb_devs),
				      NULL, 0);
		if (ret) /* If Error,flag it,Continue adding other children */
			dev_err(dev, "failed to populate usb-phy child\n");
	}

	/* Bandgap */
	if (pdata->has_bandgap) {
		void *child_pdata = NULL;
		int size;

		if (pdata->rev == 1) {
			child_pdata = &omap4_bg_data;
			size = sizeof(omap4_bg_data);
		} else if (pdata->rev == 2) {
			child_pdata = &omap5_bg_data;
			size = sizeof(omap5_bg_data);
		}
		omap_bandgap_devs[0].platform_data = child_pdata;
		omap_bandgap_devs[0].pdata_size = size;

		thermal_irq = platform_get_irq_byname(pdev, "thermal_alert");
		if (thermal_irq < 0) {
			dev_err(dev, "Fail: request thermal irq\n");
			goto exit;
		}

		ret = mfd_add_devices(dev, -1, omap_bandgap_devs,
				      ARRAY_SIZE(omap_bandgap_devs),
				      NULL, thermal_irq);
		if (ret) /* If Error, flag it */
			dev_err(dev, "failed to populate bandgap child\n");
	}

exit:
	return 0;

}


static int __devinit omap_control_probe(struct platform_device *pdev)
{
	struct resource *res;
	void __iomem *base;
	struct device *dev = &pdev->dev;
	struct device_node *np = dev->of_node;
	struct omap_control *omap_control;

	omap_control = devm_kzalloc(dev, sizeof(*omap_control), GFP_KERNEL);
	if (!omap_control) {
		dev_err(dev, "not enough memory for omap_control\n");
		return -ENOMEM;
	}

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res) {
		dev_err(dev, "missing core memory base resource\n");
		return -EINVAL;
	}

	base = devm_request_and_ioremap(dev, res);
	if (!base) {
		dev_err(dev, "ioremap failed for resource %s\n", res->name);
		return -EADDRNOTAVAIL;
	}
	omap_control->base = base;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 1);
	if (!res) {
		dev_err(dev, "missing pad memory base resource\n");
		return -EINVAL;
	}

	base = devm_request_and_ioremap(dev, res);
	if (!base) {
		dev_err(dev, "ioremap failed for resource %s\n", res->name);
		return -EADDRNOTAVAIL;
	}
	omap_control->base_pad = base;

	omap_control->dev = dev;
	spin_lock_init(&omap_control->reglock);

	if (pdev->dev.platform_data)
		dev_set_drvdata(dev, omap_control);
	else
		platform_set_drvdata(pdev, omap_control);

	omap_control_module = omap_control;

	if (pdev->dev.platform_data)
		return omap_control_add_children(pdev, omap_control);
	else
		return of_platform_populate(np, of_omap_control_match,
							NULL, dev);
}

static int __devexit omap_control_remove(struct platform_device *pdev)
{
	struct omap_control *omap_control = platform_get_drvdata(pdev);

	spin_lock(&omap_control->reglock);
	if (omap_control->use_count > 0) {
		spin_unlock(&omap_control->reglock);
		dev_err(&pdev->dev, "device removed while still being used\n");
		return -EBUSY;
	}
	spin_unlock(&omap_control->reglock);


	if (pdev->dev.platform_data) {
		mfd_remove_devices(omap_control->dev);
		dev_set_drvdata(omap_control->dev, NULL);
	} else {
		platform_set_drvdata(pdev, NULL);
	}

	return 0;
}

static struct platform_driver omap_control_driver = {
	.probe			= omap_control_probe,
	.remove			= __devexit_p(omap_control_remove),
	.driver = {
		.name		= "omap-control-core",
		.owner		= THIS_MODULE,
		.of_match_table	= of_omap_control_match,
	},
};

static int __init omap_control_init(void)
{
	return platform_driver_register(&omap_control_driver);
}
postcore_initcall_sync(omap_control_init);

static void __exit omap_control_exit(void)
{
	platform_driver_unregister(&omap_control_driver);
}
module_exit(omap_control_exit);
early_platform_init("early_omap_control", &omap_control_driver);

MODULE_DESCRIPTION("OMAP system control module driver");
MODULE_LICENSE("GPL v2");
MODULE_ALIAS("platform:omap-control-core");
MODULE_AUTHOR("Texas Instruments Inc.");
