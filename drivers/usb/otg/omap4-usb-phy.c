/*
 * OMAP4 system control module driver, USB control children
 *
 * Copyright (C) 2012 Texas Instruments Incorporated - http://www.ti.com/
 *
 * Contact:
 *    Kishon Vijay Abraham I <kishon@ti.com>
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
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA
 * 02110-1301 USA
 *
 */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/err.h>
#include <linux/platform_device.h>
#include <linux/mfd/omap_control.h>
#include <linux/usb/omap4_usb_phy.h>

/**
 * omap4_usb_phy_power - power on/off the phy using control module reg
 * @dev: struct device *
 * @on: 0 to off and 1 to on based on powering on or off the PHY
 *
 * omap_usb2 can call this API to power on or off the PHY.
 */
int omap4_usb_phy_power(struct device *dev, bool on)
{
	u32 val;
	int ret;

	if (on) {
		ret = omap_control_readl(dev, CONTROL_DEV_CONF, &val);
		if (!ret && (val & PHY_PD)) {
			ret = omap_control_writel(dev, ~PHY_PD,
						  CONTROL_DEV_CONF);
			mdelay(200);
		}
	} else {
		ret = omap_control_writel(dev, PHY_PD, CONTROL_DEV_CONF);
	}

	return ret;
}
EXPORT_SYMBOL_GPL(omap4_usb_phy_power);

/**
 * omap4_usb_phy_mailbox - write to usb otg mailbox
 * @dev: struct device *
 * @val: the value to be written to the mailbox
 *
 * On detection of a device (ID pin is grounded), the phy should call this API
 * to set AVALID, VBUSVALID and ID pin is grounded.
 *
 * When OMAP is connected to a host (OMAP in device mode), the phy should call
 * this API to set AVALID, VBUSVALID and ID pin in high impedance.
 *
 * The phy should call this API, if OMAP is disconnected from host or device.
 */
int omap4_usb_phy_mailbox(struct device *dev, u32 val)
{
	return omap_control_writel(dev, val, CONTROL_USBOTGHS_CONTROL);
}
EXPORT_SYMBOL_GPL(omap4_usb_phy_mailbox);

static int __devinit omap_usb_phy_probe(struct platform_device *pdev)
{
	struct omap_control *omap_control;

	omap_control = dev_get_drvdata(pdev->dev.parent);
	/* just for the sake */
	if (!omap_control) {
		dev_err(&pdev->dev, "no omap_control in our parent\n");
		return -EINVAL;
	}

	return 0;
}

/**
 * omap5_usb_phy_partial_powerup - power on the serializer using control module
 * @dev: struct device *
 *
 * After the dwc3 module is disabled and enabled again the synchronization
 * between dwc3 and phy goes bad and the device does not get enumerated
 * in superspeed mode. After some trials it was found powering up TX and
 * part of RX PHY helped to solve the issue.
 */
int omap5_usb_phy_partial_powerup(struct device *dev)
{
	int ret;
	u32 val;
	unsigned long rate;
	struct clk *sys_clk;

	sys_clk = clk_get(NULL, "sys_clkin");
	if (IS_ERR(sys_clk)) {
		pr_err("%s: unable to get sys_clkin\n", __func__);
		return -EINVAL;
	}

	rate = clk_get_rate(sys_clk);
	rate = rate/1000000;

	ret = omap_control_readl(dev, CONTROL_PHY_POWER_USB, &val);
	if (ret) {
		pr_err("%s: unable to read register\n", __func__);
		return ret;
	}

	val &= ~(USB_PWRCTL_CLK_CMD_MASK | USB_PWRCTL_CLK_FREQ_MASK);
	val |= (USB3_PHY_PARTIAL_RX_POWERON | USB3_PHY_TX_RX_POWERON)
		<< USB_PWRCTL_CLK_CMD_SHIFT;
	val |= rate << USB_PWRCTL_CLK_FREQ_SHIFT;

	ret = omap_control_writel(dev, val, CONTROL_PHY_POWER_USB);

	return 0;
}
EXPORT_SYMBOL_GPL(omap5_usb_phy_partial_powerup);

/**
 * omap5_usb_phy_power - power on/off the serializer using control module
 * @dev: struct device *
 * @on: 0 to off and 1 to on based on powering on or off the PHY
 *
 * omap_usb3 can call this API to power on or off the PHY.
 */
int omap5_usb_phy_power(struct device *dev, bool on)
{
	int ret;
	u32 val;

	ret = omap_control_readl(dev, CONTROL_PHY_POWER_USB, &val);
	if (ret) {
		pr_err("%s: unable to read register\n", __func__);
		return ret;
	}

	if (on) {
		val &= ~USB_PWRCTL_CLK_CMD_MASK;
		val |= USB3_PHY_TX_RX_POWERON << USB_PWRCTL_CLK_CMD_SHIFT;
	} else {
		val &= ~USB_PWRCTL_CLK_CMD_MASK;
		val |= USB3_PHY_TX_RX_POWEROFF << USB_PWRCTL_CLK_CMD_SHIFT;
	}

	ret = omap_control_writel(dev, val, CONTROL_PHY_POWER_USB);

	return ret;
}
EXPORT_SYMBOL_GPL(omap5_usb_phy_power);

static int __devexit omap_usb_phy_remove(struct platform_device *pdev)
{
	return 0;
}

static const struct of_device_id of_omap_usb_phy_match[] = {
	{ .compatible = "ti,omap4-usb-phy", },
	{ },
};

static struct platform_driver omap_usb_phy_driver = {
	.probe = omap_usb_phy_probe,
	.remove = __devexit_p(omap_usb_phy_remove),
	.driver = {
			.name	= "omap4-usb-phy",
			.owner	= THIS_MODULE,
			.of_match_table	= of_omap_usb_phy_match,
	},
};

static int __init omap_usb_phy_init(void)
{
	return platform_driver_register(&omap_usb_phy_driver);
}
postcore_initcall(omap_usb_phy_init);

static void __exit omap_usb_phy_exit(void)
{
	platform_driver_unregister(&omap_usb_phy_driver);
}
module_exit(omap_usb_phy_exit);

MODULE_DESCRIPTION("OMAP4+ USB-phy driver");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform: omap4-usb-phy");
MODULE_AUTHOR("Texas Instrument Inc.");
