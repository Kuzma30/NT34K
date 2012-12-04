/*
 * otg.c -- USB OTG utility code
 *
 * Copyright (C) 2004 Texas Instruments
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */

#include <linux/kernel.h>
#include <linux/export.h>
#include <linux/device.h>

#include <linux/usb/otg.h>

static LIST_HEAD(phy_list);
static DEFINE_SPINLOCK(phy_lock);

/**
 * usb_get_phy - find the USB PHY
 * @type - the type of the phy the controller requires
 *
 * Returns the phy driver, after getting a refcount to it; or
 * null if there is no such phy.  The caller is responsible for
 * calling usb_put_phy() to release that count.
 *
 * For use by USB host and peripheral drivers.
 */
struct usb_phy *usb_get_phy(enum usb_phy_type type)
{
	struct usb_phy	*phy = NULL;
	unsigned long	flags;

	spin_lock_irqsave(&phy_lock, flags);

	list_for_each_entry(phy, &phy_list, head) {
		if (phy->type == type) {
			get_device(phy->dev);
			spin_unlock_irqrestore(&phy_lock, flags);
			return phy;
		}
	}

	spin_unlock_irqrestore(&phy_lock, flags);

	pr_err("unable to find transceiver of type %d\n", type);

	return phy;
}
EXPORT_SYMBOL(usb_get_phy);

/**
 * usb_put_phy - release the USB PHY
 * @x: the phy returned by usb_get_phy()
 *
 * Releases a refcount the caller received from usb_get_phy().
 *
 * For use by USB host and peripheral drivers.
 */
void usb_put_phy(struct usb_phy *x)
{
	if (x)
		put_device(x->dev);
}
EXPORT_SYMBOL(usb_put_phy);

/**
 * usb_add_phy - declare the USB PHY
 * @x: the USB phy to be used; or NULL
 * @type - the type of this PHY
 *
 * This call is exclusively for use by phy drivers, which
 * coordinate the activities of drivers for host and peripheral
 * controllers, and in some cases for VBUS current regulation.
 */
int usb_add_phy(struct usb_phy *x, enum usb_phy_type type)
{
	int		ret = 0;
	unsigned long	flags;
	struct usb_phy	*phy;

	if (x && x->type != USB_PHY_TYPE_UNDEFINED) {
		dev_err(x->dev, "not accepting initialized PHY %s\n", x->label);
		return -EINVAL;
	}

	spin_lock_irqsave(&phy_lock, flags);

	list_for_each_entry(phy, &phy_list, head) {
		if (phy->type == type) {
			ret = -EBUSY;
			dev_err(x->dev, "transceiver type %s already exists\n",
						usb_phy_type_string(type));
			goto out;
		}
	}

	x->type = type;
	list_add_tail(&x->head, &phy_list);

out:
	spin_unlock_irqrestore(&phy_lock, flags);
	return ret;
}
EXPORT_SYMBOL(usb_add_phy);

/**
 * usb_remove_phy - remove the OTG PHY
 * @x: the USB OTG PHY to be removed;
 *
 * This reverts the effects of usb_add_phy
 */
void usb_remove_phy(struct usb_phy *x)
{
	unsigned long	flags;

	spin_lock_irqsave(&phy_lock, flags);
	if (x)
		list_del(&x->head);
	spin_unlock_irqrestore(&phy_lock, flags);
}
EXPORT_SYMBOL(usb_remove_phy);

const char *otg_state_string(enum usb_otg_state state)
{
	switch (state) {
	case OTG_STATE_A_IDLE:
		return "a_idle";
	case OTG_STATE_A_WAIT_VRISE:
		return "a_wait_vrise";
	case OTG_STATE_A_WAIT_BCON:
		return "a_wait_bcon";
	case OTG_STATE_A_HOST:
		return "a_host";
	case OTG_STATE_A_SUSPEND:
		return "a_suspend";
	case OTG_STATE_A_PERIPHERAL:
		return "a_peripheral";
	case OTG_STATE_A_WAIT_VFALL:
		return "a_wait_vfall";
	case OTG_STATE_A_VBUS_ERR:
		return "a_vbus_err";
	case OTG_STATE_B_IDLE:
		return "b_idle";
	case OTG_STATE_B_SRP_INIT:
		return "b_srp_init";
	case OTG_STATE_B_PERIPHERAL:
		return "b_peripheral";
	case OTG_STATE_B_WAIT_ACON:
		return "b_wait_acon";
	case OTG_STATE_B_HOST:
		return "b_host";
	default:
		return "UNDEFINED";
	}
}

static struct usb_phy *xceiv;
struct usb_phy *otg_get_transceiver(void)
{
	if (xceiv)
		get_device(xceiv->dev);
        return xceiv;
}
EXPORT_SYMBOL(otg_get_transceiver);
void otg_put_transceiver(struct usb_phy *x)
{
	if (x)
		put_device(x->dev);
}
EXPORT_SYMBOL(otg_put_transceiver);
