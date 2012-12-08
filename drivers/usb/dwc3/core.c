/**
 * core.c - DesignWare USB3 DRD Controller Core file
 *
 * Copyright (C) 2010-2011 Texas Instruments Incorporated - http://www.ti.com
 *
 * Authors: Felipe Balbi <balbi@ti.com>,
 *	    Sebastian Andrzej Siewior <bigeasy@linutronix.de>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions, and the following disclaimer,
 *    without modification.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. The names of the above-listed copyright holders may not be used
 *    to endorse or promote products derived from this software without
 *    specific prior written permission.
 *
 * ALTERNATIVELY, this software may be distributed under the terms of the
 * GNU General Public License ("GPL") version 2, as published by the Free
 * Software Foundation.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS
 * IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
 * PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
 * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
 * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/spinlock.h>
#include <linux/platform_device.h>
#include <linux/pm_runtime.h>
#include <linux/interrupt.h>
#include <linux/ioport.h>
#include <linux/io.h>
#include <linux/list.h>
#include <linux/delay.h>
#include <linux/dma-mapping.h>
#include <linux/of.h>

#include <linux/usb/ch9.h>
#include <linux/usb/gadget.h>

#include "core.h"
#include "gadget.h"
#include "io.h"

#include "debug.h"

static char *maximum_speed = "super";
module_param(maximum_speed, charp, 0);
MODULE_PARM_DESC(maximum_speed, "Maximum supported speed.");

/* -------------------------------------------------------------------------- */

#define DWC3_DEVS_POSSIBLE	32

static DECLARE_BITMAP(dwc3_devs, DWC3_DEVS_POSSIBLE);

int dwc3_get_device_id(void)
{
	int		id;

again:
	id = find_first_zero_bit(dwc3_devs, DWC3_DEVS_POSSIBLE);
	if (id < DWC3_DEVS_POSSIBLE) {
		int old;

		old = test_and_set_bit(id, dwc3_devs);
		if (old)
			goto again;
	} else {
		pr_err("dwc3: no space for new device\n");
		id = -ENOMEM;
	}

	return id;
}
EXPORT_SYMBOL_GPL(dwc3_get_device_id);

void dwc3_put_device_id(int id)
{
	int			ret;

	if (id < 0)
		return;

	ret = test_bit(id, dwc3_devs);
	WARN(!ret, "dwc3: ID %d not in use\n", id);
	smp_mb__before_clear_bit();
	clear_bit(id, dwc3_devs);
}
EXPORT_SYMBOL_GPL(dwc3_put_device_id);

void dwc3_set_mode(struct dwc3 *dwc, u32 mode)
{
	u32 reg;

	switch (mode) {
	case DWC3_GCTL_PRTCAP_HOST:
		dwc->mode = DWC3_MODE_HOST;
		break;
	case DWC3_GCTL_PRTCAP_DEVICE:
		dwc->mode = DWC3_MODE_DEVICE;
		break;
	case DWC3_GCTL_PRTCAP_OTG:
		dwc->mode = DWC3_MODE_DRD;
		break;
	default:
		dev_err(dwc->dev, "non existent mode setting\n");
	}

	reg = dwc3_readl(dwc->regs, DWC3_GCTL);
	reg &= ~(DWC3_GCTL_PRTCAPDIR(DWC3_GCTL_PRTCAP_OTG));
	reg |= DWC3_GCTL_PRTCAPDIR(mode);
	dwc3_writel(dwc->regs, DWC3_GCTL, reg);
}

/**
 * dwc3_core_soft_reset - Issues core soft reset and PHY reset
 * @dwc: pointer to our context structure
 */
static void dwc3_core_soft_reset(struct dwc3 *dwc)
{
	u32		reg;

	/* Before Resetting PHY, put Core in Reset */
	reg = dwc3_readl(dwc->regs, DWC3_GCTL);
	reg |= DWC3_GCTL_CORESOFTRESET;
	dwc3_writel(dwc->regs, DWC3_GCTL, reg);

	/* Assert USB3 PHY reset */
	reg = dwc3_readl(dwc->regs, DWC3_GUSB3PIPECTL(0));
	reg |= DWC3_GUSB3PIPECTL_PHYSOFTRST;
	dwc3_writel(dwc->regs, DWC3_GUSB3PIPECTL(0), reg);

	/* Assert USB2 PHY reset */
	reg = dwc3_readl(dwc->regs, DWC3_GUSB2PHYCFG(0));
	reg |= DWC3_GUSB2PHYCFG_PHYSOFTRST;
	dwc3_writel(dwc->regs, DWC3_GUSB2PHYCFG(0), reg);

	usb_phy_init(dwc->usb3_phy);
	mdelay(100);

	/* Clear USB3 PHY reset */
	reg = dwc3_readl(dwc->regs, DWC3_GUSB3PIPECTL(0));
	reg &= ~DWC3_GUSB3PIPECTL_PHYSOFTRST;
	dwc3_writel(dwc->regs, DWC3_GUSB3PIPECTL(0), reg);

	/* Clear USB2 PHY reset */
	reg = dwc3_readl(dwc->regs, DWC3_GUSB2PHYCFG(0));
	reg &= ~DWC3_GUSB2PHYCFG_PHYSOFTRST;
	dwc3_writel(dwc->regs, DWC3_GUSB2PHYCFG(0), reg);

	mdelay(100);

	/* After PHYs are stable we can take Core out of reset state */
	reg = dwc3_readl(dwc->regs, DWC3_GCTL);
	reg &= ~DWC3_GCTL_CORESOFTRESET;
	dwc3_writel(dwc->regs, DWC3_GCTL, reg);
}

/**
 * dwc3_free_one_event_buffer - Frees one event buffer
 * @dwc: Pointer to our controller context structure
 * @evt: Pointer to event buffer to be freed
 */
static void dwc3_free_one_event_buffer(struct dwc3 *dwc,
		struct dwc3_event_buffer *evt)
{
	dma_free_coherent(dwc->dev, evt->length, evt->buf, evt->dma);
	kfree(evt);
}

/**
 * dwc3_alloc_one_event_buffer - Allocates one event buffer structure
 * @dwc: Pointer to our controller context structure
 * @length: size of the event buffer
 *
 * Returns a pointer to the allocated event buffer structure on success
 * otherwise ERR_PTR(errno).
 */
static struct dwc3_event_buffer *__devinit
dwc3_alloc_one_event_buffer(struct dwc3 *dwc, unsigned length)
{
	struct dwc3_event_buffer	*evt;

	evt = kzalloc(sizeof(*evt), GFP_KERNEL);
	if (!evt)
		return ERR_PTR(-ENOMEM);

	evt->dwc	= dwc;
	evt->length	= length;
	evt->buf	= dma_alloc_coherent(dwc->dev, length,
			&evt->dma, GFP_KERNEL);
	if (!evt->buf) {
		kfree(evt);
		return ERR_PTR(-ENOMEM);
	}

	return evt;
}

/**
 * dwc3_free_event_buffers - frees all allocated event buffers
 * @dwc: Pointer to our controller context structure
 */
static void dwc3_free_event_buffers(struct dwc3 *dwc)
{
	struct dwc3_event_buffer	*evt;
	int i;

	for (i = 0; i < dwc->num_event_buffers; i++) {
		evt = dwc->ev_buffs[i];
		if (evt)
			dwc3_free_one_event_buffer(dwc, evt);
	}

	kfree(dwc->ev_buffs);
}

/**
 * dwc3_alloc_event_buffers - Allocates @num event buffers of size @length
 * @dwc: pointer to our controller context structure
 * @length: size of event buffer
 *
 * Returns 0 on success otherwise negative errno. In the error case, dwc
 * may contain some buffers allocated but not all which were requested.
 */
static int __devinit dwc3_alloc_event_buffers(struct dwc3 *dwc, unsigned length)
{
	int			num;
	int			i;

	num = DWC3_NUM_INT(dwc->hwparams.hwparams1);
	dwc->num_event_buffers = num;

	dwc->ev_buffs = kzalloc(sizeof(*dwc->ev_buffs) * num, GFP_KERNEL);
	if (!dwc->ev_buffs) {
		dev_err(dwc->dev, "can't allocate event buffers array\n");
		return -ENOMEM;
	}

	for (i = 0; i < num; i++) {
		struct dwc3_event_buffer	*evt;

		evt = dwc3_alloc_one_event_buffer(dwc, length);
		if (IS_ERR(evt)) {
			dev_err(dwc->dev, "can't allocate event buffer\n");
			return PTR_ERR(evt);
		}
		dwc->ev_buffs[i] = evt;
	}

	return 0;
}

/**
 * dwc3_event_buffers_setup - setup our allocated event buffers
 * @dwc: pointer to our controller context structure
 *
 * Returns 0 on success otherwise negative errno.
 */
static int dwc3_event_buffers_setup(struct dwc3 *dwc)
{
	struct dwc3_event_buffer	*evt;
	int				n;

	for (n = 0; n < dwc->num_event_buffers; n++) {
		evt = dwc->ev_buffs[n];
		dev_dbg(dwc->dev, "Event buf %p dma %08llx length %d\n",
				evt->buf, (unsigned long long) evt->dma,
				evt->length);

		evt->lpos = 0;

		dwc3_writel(dwc->regs, DWC3_GEVNTADRLO(n),
				lower_32_bits(evt->dma));
		dwc3_writel(dwc->regs, DWC3_GEVNTADRHI(n),
				upper_32_bits(evt->dma));
		dwc3_writel(dwc->regs, DWC3_GEVNTSIZ(n),
				evt->length & 0xffff);
		dwc3_writel(dwc->regs, DWC3_GEVNTCOUNT(n), 0);
	}

	return 0;
}

static void dwc3_event_buffers_cleanup(struct dwc3 *dwc)
{
	struct dwc3_event_buffer	*evt;
	int				n;

	for (n = 0; n < dwc->num_event_buffers; n++) {
		evt = dwc->ev_buffs[n];

		evt->lpos = 0;

		dwc3_writel(dwc->regs, DWC3_GEVNTADRLO(n), 0);
		dwc3_writel(dwc->regs, DWC3_GEVNTADRHI(n), 0);
		dwc3_writel(dwc->regs, DWC3_GEVNTSIZ(n), 0);
		dwc3_writel(dwc->regs, DWC3_GEVNTCOUNT(n), 0);
	}
}

static void __devinit dwc3_cache_hwparams(struct dwc3 *dwc)
{
	struct dwc3_hwparams	*parms = &dwc->hwparams;

	parms->hwparams0 = dwc3_readl(dwc->regs, DWC3_GHWPARAMS0);
	parms->hwparams1 = dwc3_readl(dwc->regs, DWC3_GHWPARAMS1);
	parms->hwparams2 = dwc3_readl(dwc->regs, DWC3_GHWPARAMS2);
	parms->hwparams3 = dwc3_readl(dwc->regs, DWC3_GHWPARAMS3);
	parms->hwparams4 = dwc3_readl(dwc->regs, DWC3_GHWPARAMS4);
	parms->hwparams5 = dwc3_readl(dwc->regs, DWC3_GHWPARAMS5);
	parms->hwparams6 = dwc3_readl(dwc->regs, DWC3_GHWPARAMS6);
	parms->hwparams7 = dwc3_readl(dwc->regs, DWC3_GHWPARAMS7);
	parms->hwparams8 = dwc3_readl(dwc->regs, DWC3_GHWPARAMS8);
}

/**
 * dwc3_core_common_init - common initialization of DWC3 Core
 * @dev: device pointer of dwc3 device
 *
 * This function groups the common initialization procedure required
 * as part of the dwc3_core_host_init() and dwc3_core_late_init()
 */
int dwc3_core_common_init(struct device *dev)
{
	struct platform_device	*pdev = to_platform_device(dev);
	struct dwc3		*dwc = platform_get_drvdata(pdev);
	int			ret = 0;
	u32			reg;
	unsigned long		timeout;

	ret = pm_runtime_get_sync(dev);
	if (ret < 0) {
		dev_err(dev, "get_sync failed with err %d\n", ret);
		return ret;
	}

	usb_phy_set_suspend(dwc->usb2_phy, 0);
	usb_phy_set_suspend(dwc->usb3_phy, 0);

	/* issue device SoftReset too */
	timeout = jiffies + msecs_to_jiffies(500);
	dwc3_writel(dwc->regs, DWC3_DCTL, DWC3_DCTL_CSFTRST);
	do {
		reg = dwc3_readl(dwc->regs, DWC3_DCTL);
		if (!(reg & DWC3_DCTL_CSFTRST))
			break;

		if (time_after(jiffies, timeout)) {
			dev_err(dwc->dev, "Reset Timed Out\n");
			ret = -ETIMEDOUT;
			goto exit;
		}

		cpu_relax();
	} while (true);

	dwc3_core_soft_reset(dwc);
	ret = dwc3_event_buffers_setup(dwc);
	if (ret)
		dev_err(dwc->dev, "failed to setup event buffers\n");

exit:
	return ret;
}

/**
 * dwc3_core_host_init - late initialization of DWC3 Core
 * @dev: device pointer of dwc3 device
 *
 * Enables the dwc3 host, performs reset and does a host_init
 */
int dwc3_core_host_init(struct device *dev)
{
	int			ret = 0;
	struct platform_device	*pdev = to_platform_device(dev);
	struct dwc3		*dwc = platform_get_drvdata(pdev);

	ret = dwc3_core_common_init(dev);
	if (ret) {
		dev_err(dev, "dwc3_core_common_init failed with err %d\n", ret);
		goto exit;
	}

	dwc3_set_mode(dwc, DWC3_GCTL_PRTCAP_HOST);
	ret = dwc3_host_init(dwc);
	if (ret)
		dev_err(dev, "failed to initialize host with err %d\n", ret);

exit:
	return ret;
}
EXPORT_SYMBOL_GPL(dwc3_core_host_init);

/**
 * dwc3_core_late_init - late initialization of DWC3 Core
 * @dev: device pointer of dwc3 device
 *
 * Enables the dwc3 module, performs reset and starts the gadget (enables
 * the endpoint). This function should be called on demand i.e whenever dwc3
 * is going to be used.
 *
 * Called from the glue layer on cable connect and when a gadget driver is
 * inserted.
 */
int dwc3_core_late_init(struct device *dev)
{
	int			ret = 0;
	struct platform_device	*pdev = to_platform_device(dev);
	struct dwc3		*dwc = platform_get_drvdata(pdev);

	/* return if a gadget driver is not found on cable connect */
	if (!dwc->gadget_driver) {
		dev_dbg(dev, "%s is not bound to any driver\n",
						dwc->gadget.name);
		return -EINVAL;
	}

	ret = dwc3_core_common_init(dev);
	if (ret < 0) {
		dev_err(dev, "dwc3_core_common_init failed with err %d\n", ret);
		goto exit;
	}

	ret = dwc3_gadget_delayed_start(&dwc->gadget, dwc->gadget_driver);
	if (ret) {
		dev_err(dwc->dev, "failed to start the gadget\n");
		goto exit;
	}

	dwc3_gadget_run_stop(dwc, true);

exit:
	return ret;
}
EXPORT_SYMBOL_GPL(dwc3_core_late_init);

/**
 * dwc3_core_shutdown - disables the DWC3 Core
 * @dev: device pointer of dwc3 device
 *
 * Power off the phy and stop the dwc3 module. This function should be called
 * as soon as the dwc3 module is not going to used.
 *
 * Called from the glue layer on cable dis-connect and when a gadget driver is
 * removed
 */
int dwc3_core_shutdown(struct device *dev)
{
	struct platform_device	*pdev = to_platform_device(dev);
	struct dwc3		*dwc = platform_get_drvdata(pdev);

	/* core should be already shutdown while removing the gadget driver */
	if (!dwc->gadget_driver) {
		dev_dbg(dev, "Core is already shutdown\n");
		return -EINVAL;
	}

	dwc3_gadget_early_stop(&dwc->gadget, dwc->gadget_driver);

	usb_phy_set_suspend(dwc->usb2_phy, 1);
	usb_phy_set_suspend(dwc->usb3_phy, 1);

	dwc3_gadget_run_stop(dwc, false);

	pm_runtime_mark_last_busy(dev);
	pm_runtime_put_autosuspend(dev);

	return 0;
}
EXPORT_SYMBOL_GPL(dwc3_core_shutdown);

/**
 * dwc3_core_init - Low-level initialization of DWC3 Core
 * @dwc: Pointer to our controller context structure
 *
 * Returns 0 on success otherwise negative errno.
 */
static int __devinit dwc3_core_init(struct dwc3 *dwc)
{
	u32			reg;
	int			ret;

	reg = dwc3_readl(dwc->regs, DWC3_GSNPSID);
	/* This should read as U3 followed by revision number */
	if ((reg & DWC3_GSNPSID_MASK) != 0x55330000) {
		dev_err(dwc->dev, "this is not a DesignWare USB3 DRD Core\n");
		ret = -ENODEV;
		goto err0;
	}
	dwc->revision = reg;

	dwc3_cache_hwparams(dwc);

	reg = dwc3_readl(dwc->regs, DWC3_GCTL);
	reg &= ~DWC3_GCTL_SCALEDOWN_MASK;
	reg &= ~DWC3_GCTL_DISSCRAMBLE;

	switch (DWC3_GHWPARAMS1_EN_PWROPT(dwc->hwparams.hwparams1)) {
	case DWC3_GHWPARAMS1_EN_PWROPT_CLK:
		reg &= ~DWC3_GCTL_DSBLCLKGTNG;
		break;
	default:
		dev_dbg(dwc->dev, "No power optimization available\n");
	}

	/*
	 * WORKAROUND: DWC3 revisions <1.90a have a bug
	 * where the device can fail to connect at SuperSpeed
	 * and falls back to high-speed mode which causes
	 * the device to enter a Connect/Disconnect loop
	 */
	if (dwc->revision < DWC3_REVISION_190A)
		reg |= DWC3_GCTL_U2RSTECN;

	dwc3_writel(dwc->regs, DWC3_GCTL, reg);

	ret = dwc3_alloc_event_buffers(dwc, DWC3_EVENT_BUFFERS_SIZE);
	if (ret) {
		dev_err(dwc->dev, "failed to allocate event buffers\n");
		ret = -ENOMEM;
		goto err1;
	}

	return 0;

err1:
	dwc3_free_event_buffers(dwc);

err0:
	return ret;
}

static void dwc3_core_exit(struct dwc3 *dwc)
{
	dwc3_event_buffers_cleanup(dwc);
	dwc3_free_event_buffers(dwc);
}

#define DWC3_ALIGN_MASK		(16 - 1)

static int __devinit dwc3_probe(struct platform_device *pdev)
{
	struct device_node	*node = pdev->dev.of_node;
	struct resource		*res;
	struct dwc3		*dwc;
	struct device		*dev = &pdev->dev;

	int			ret = -ENOMEM;

	void __iomem		*regs;
	void			*mem;

	u8			mode;

	mem = devm_kzalloc(dev, sizeof(*dwc) + DWC3_ALIGN_MASK, GFP_KERNEL);
	if (!mem) {
		dev_err(dev, "not enough memory\n");
		return -ENOMEM;
	}
	dwc = PTR_ALIGN(mem, DWC3_ALIGN_MASK + 1);
	dwc->mem = mem;

	dwc->usb2_phy = usb_get_phy(USB_PHY_TYPE_USB2);
	if (!dwc->usb2_phy) {
		dev_err(dev, "no usb2 phy configured\n");
		return -ENODEV;
	}

	dwc->usb3_phy = usb_get_phy(USB_PHY_TYPE_USB3);
	if (!dwc->usb3_phy) {
		dev_err(dev, "no usb3 phy configured\n");
		ret = -ENODEV;
		goto err0;
	}

	usb_phy_init(dwc->usb2_phy);

	res = platform_get_resource(pdev, IORESOURCE_IRQ, 0);
	if (!res) {
		dev_err(dev, "missing IRQ\n");
		ret = -ENODEV;
		goto err1;
	}
	dwc->xhci_resources[1].start = res->start;
	dwc->xhci_resources[1].end = res->end;
	dwc->xhci_resources[1].flags = res->flags;
	dwc->xhci_resources[1].name = res->name;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res) {
		dev_err(dev, "missing memory resource\n");
		ret = -ENODEV;
		goto err1;
	}
	dwc->xhci_resources[0].start = res->start;
	dwc->xhci_resources[0].end = dwc->xhci_resources[0].start +
					DWC3_XHCI_REGS_END;
	dwc->xhci_resources[0].flags = res->flags;
	dwc->xhci_resources[0].name = res->name;

	 /*
	  * Request memory region but exclude xHCI regs,
	  * since it will be requested by the xhci-plat driver.
	  */
	res = devm_request_mem_region(dev, res->start + DWC3_GLOBALS_REGS_START,
			resource_size(res) - DWC3_GLOBALS_REGS_START,
			dev_name(dev));
	if (!res) {
		dev_err(dev, "can't request mem region\n");
		ret = -ENOMEM;
		goto err1;
	}

	regs = devm_ioremap_nocache(dev, res->start, resource_size(res));
	if (!regs) {
		dev_err(dev, "ioremap failed\n");
		ret = -ENOMEM;
		goto err1;
	}

	spin_lock_init(&dwc->lock);
	platform_set_drvdata(pdev, dwc);

	dwc->regs	= regs;
	dwc->regs_size	= resource_size(res);
	dwc->dev	= dev;
	dwc->is_connected = false;

	if (!strncmp("super", maximum_speed, 5))
		dwc->maximum_speed = DWC3_DCFG_SUPERSPEED;
	else if (!strncmp("high", maximum_speed, 4))
		dwc->maximum_speed = DWC3_DCFG_HIGHSPEED;
	else if (!strncmp("full", maximum_speed, 4))
		dwc->maximum_speed = DWC3_DCFG_FULLSPEED1;
	else if (!strncmp("low", maximum_speed, 3))
		dwc->maximum_speed = DWC3_DCFG_LOWSPEED;
	else
		dwc->maximum_speed = DWC3_DCFG_SUPERSPEED;

	if (of_get_property(node, "tx-fifo-resize", NULL))
		dwc->needs_fifo_resize = true;

	/*
	 * REVISIT: For internal releases, we still don't have a working DT
	 * setup, so we will, temporarily, set this flag to 'true'
	 * unconditionaly until we can use DT properly.
	 */
	dwc->needs_fifo_resize = true;

	pm_runtime_enable(dev);
	pm_runtime_get_sync(dev);

	ret = dwc3_core_init(dwc);
	if (ret) {
		dev_err(dev, "failed to initialize core\n");
		goto err1;
	}

	mode = DWC3_MODE(dwc->hwparams.hwparams0);

	switch (mode) {
	case DWC3_MODE_DEVICE:
	case DWC3_MODE_DRD:
		dwc3_set_mode(dwc, DWC3_GCTL_PRTCAP_DEVICE);
		ret = dwc3_gadget_init(dwc);
		if (ret) {
			dev_err(dev, "failed to initialize gadget\n");
			goto err2;
		}
		break;
	default:
		dev_err(dev, "Unsupported mode of operation %d\n", mode);
		goto err2;
	}

	ret = dwc3_debugfs_init(dwc);
	if (ret) {
		dev_err(dev, "failed to initialize debugfs\n");
		goto err3;
	}

	pm_runtime_put_sync(dev);

	return 0;

err3:
	switch (mode) {
	case DWC3_MODE_DEVICE:
		dwc3_gadget_exit(dwc);
		break;
	case DWC3_MODE_HOST:
		dwc3_host_exit(dwc);
		break;
	case DWC3_MODE_DRD:
		dwc3_host_exit(dwc);
		dwc3_gadget_exit(dwc);
		break;
	default:
		/* do nothing */
		break;
	}

err2:
	dwc3_core_exit(dwc);

err1:
	usb_put_phy(dwc->usb3_phy);

err0:
	usb_put_phy(dwc->usb2_phy);

	return ret;
}

static int __devexit dwc3_remove(struct platform_device *pdev)
{
	struct dwc3	*dwc = platform_get_drvdata(pdev);
	struct resource	*res;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);

	if (!pm_runtime_suspended(&pdev->dev))
		pm_runtime_put(&pdev->dev);

	pm_runtime_disable(&pdev->dev);

	dwc3_debugfs_exit(dwc);

	switch (dwc->mode) {
	case DWC3_MODE_DEVICE:
		dwc3_gadget_exit(dwc);
		break;
	case DWC3_MODE_HOST:
		dwc3_host_exit(dwc);
		break;
	case DWC3_MODE_DRD:
		dwc3_host_exit(dwc);
		dwc3_gadget_exit(dwc);
		break;
	default:
		/* do nothing */
		break;
	}

	dwc3_core_exit(dwc);

	usb_put_phy(dwc->usb3_phy);
	usb_put_phy(dwc->usb2_phy);

	return 0;
}

#ifdef CONFIG_PM

static int dwc3_suspend(struct device *dev)
{
	struct dwc3		*dwc = dev_get_drvdata(dev);
	unsigned long		flags;
	int			ret = 0;

	spin_lock_irqsave(&dwc->lock, flags);
	if (dwc->is_connected) {
		dev_err(dwc->dev, "can't suspend dwc3, device is connected\n");
		ret = -EBUSY;
		goto err0;
	}

err0:
	spin_unlock_irqrestore(&dwc->lock, flags);
	return ret;
}

static int dwc3_runtime_suspend(struct device *dev)
{
	struct dwc3		*dwc = dev_get_drvdata(dev);
	struct dwc3_context_regs *context = &dwc->context;

	context->gctl = dwc3_readl(dwc->regs, DWC3_GCTL);

	return 0;
}

static int dwc3_runtime_resume(struct device *dev)
{
	struct dwc3		*dwc = dev_get_drvdata(dev);
	struct dwc3_context_regs *context = &dwc->context;

	dwc3_writel(dwc->regs, DWC3_GCTL, context->gctl);
	dwc3_enable_irqs(dwc);

	return 0;
}

static const struct dev_pm_ops dwc3_pm_ops = {
	SET_SYSTEM_SLEEP_PM_OPS(dwc3_suspend, NULL)
	SET_RUNTIME_PM_OPS(dwc3_runtime_suspend, dwc3_runtime_resume, NULL)
};

#define DEV_PM_OPS	(&dwc3_pm_ops)
#else
#define DEV_PM_OPS	NULL
#endif

static struct platform_driver dwc3_driver = {
	.probe		= dwc3_probe,
	.remove		= __devexit_p(dwc3_remove),
	.driver		= {
		.name	= "dwc3",
		.pm	= DEV_PM_OPS,
	},
};

static int __init dwc3_init(void)
{
	return platform_driver_register(&dwc3_driver);
}
subsys_initcall(dwc3_init);

static void __exit dwc3_exit(void)
{
	platform_driver_unregister(&dwc3_driver);
}
module_exit(dwc3_exit);

MODULE_ALIAS("platform:dwc3");
MODULE_AUTHOR("Felipe Balbi <balbi@ti.com>");
MODULE_LICENSE("Dual BSD/GPL");
MODULE_DESCRIPTION("DesignWare USB3 DRD Controller Driver");
