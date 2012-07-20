/*
 * OMAP4 USB-phy
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

#ifndef __OMAP4_USB_PHY_H
#define __OMAP4_USB_PHY_H

#define	PHY_PD				0x1
#define	AVALID				BIT(0)
#define	BVALID				BIT(1)
#define	VBUSVALID			BIT(2)
#define	SESSEND				BIT(3)
#define	IDDIG				BIT(4)

#define	CONTROL_PHY_POWER_USB		0x00000370

#define	USB_PWRCTL_CLK_CMD_MASK		0x003FC000
#define	USB_PWRCTL_CLK_CMD_SHIFT	0xE

#define	USB_PWRCTL_CLK_FREQ_MASK	0xFFC00000
#define	USB_PWRCTL_CLK_FREQ_SHIFT	0x16

#define	USB3_PHY_TX_RX_POWERON		0x3
#define	USB3_PHY_TX_RX_POWEROFF		0x0

#define	USB3_PHY_PARTIAL_RX_POWERON	(0x1 << 6)

/* USB-PHY helpers */
#if (defined(CONFIG_OMAP4_USB_PHY)) || (defined(CONFIG_OMAP4_USB_PHY_MODULE))
extern int omap4_usb_phy_mailbox(struct device *dev, u32 val);
extern int omap4_usb_phy_power(struct device *dev, bool on);
extern int omap5_usb_phy_power(struct device *dev, bool on);
extern int omap5_usb_phy_partial_powerup(struct device *dev);
#else
static int omap4_usb_phy_mailbox(struct device *dev, u32 val)
{
	return 0;
}
static int omap4_usb_phy_power(struct device *dev, bool on)
{
	return 0;
}
static int omap5_usb_phy_power(struct device *dev, bool on)
{
	return 0;
}
static int omap5_usb_phy_partial_powerup(struct device *dev)
{
	return 0;
}
#endif

#endif
