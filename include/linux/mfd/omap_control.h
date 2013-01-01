/*
 * OMAP system control module header file
 *
 * Copyright (C) 2011-2012 Texas Instruments Incorporated - http://www.ti.com/
 * Contact:
 *   J Keerthy <j-keerthy@ti.com>
 *   Moiz Sonasath <m-sonasath@ti.com>
 *   Abraham, Kishon Vijay <kishon@ti.com>
 *   Eduardo Valentin <eduardo.valentin@ti.com>
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

#ifndef __DRIVERS_OMAP_CONTROL_H
#define __DRIVERS_OMAP_CONTROL_H

#define	CONTROL_DEV_CONF		0x00000300
#define	CONTROL_USBOTGHS_CONTROL	0x0000033C

/**
 * struct system control module - scm device structure
 * @dev: device pointer
 * @base: Base of the temp I/O for core partition
 * @base_pad: Base of the temp I/O for pad partition
 * @reglock: protect omap_control structure
 * @use_count: track API users
 */
struct omap_control {
	struct device		*dev;
	void __iomem		*base;
	void __iomem		*base_pad;
	/* protect this data structure and register access */
	spinlock_t		reglock;
	int			use_count;
};

/**
 * struct omap_control_data - system control module platform data
 * @rev: IP revision
 * @has_usb_phy: flag to determine if this control module has usb phy
 * @has_bandgap: flag to determine if this control module has BG sensor
 */
struct omap_control_data {
	int			rev;
	bool			has_usb_phy;
	bool			has_bandgap;
};

/**
 * struct omap_bandgap_data - platform data for bandgap
 * @cnt: count of temperature sensor device in scm
 * @rev: Revision of Temperature sensor
 * @tshut_gpio: gpio linked to tshut signal
 * @accurate: Accuracy of the temperature
 */
struct omap_bdg_data {
	u32			cnt;
	int			rev;
	int			tshut_gpio;
	bool			accurate;
};

#ifdef CONFIG_OMAP_BANDGAP
void omap_bandgap_prepare_for_idle(void);
void omap_bandgap_resume_after_idle(void);
#else
static inline void omap_bandgap_prepare_for_idle(void){ }
static inline void omap_bandgap_resume_after_idle(void){ }
#endif

#ifdef CONFIG_MFD_OMAP_CONTROL
extern int omap_control_readl(struct device *dev, u32 reg, u32 *val);
extern int omap_control_writel(struct device *dev, u32 val, u32 reg);
extern int omap_control_core_pad_readl(struct device *dev, u32 reg, u32 *val);
extern int omap_control_core_pad_writel(struct device *dev, u32 val, u32 reg);
extern struct device *omap_control_get(void);
extern void omap_control_put(struct device *dev);
#else
static inline int omap_control_readl(struct device *dev, u32 reg, u32 *val)
{
	return 0;
}

static inline int omap_control_writel(struct device *dev, u32 val, u32 reg)
{
	return 0;
}

static inline int omap_control_core_pad_readl(struct device *dev,
					      u32 reg, u32 *val)
{
	return 0;
}

static inline int omap_control_core_pad_writel(struct device *dev,
					       u32 val, u32 reg)
{
	return 0;
}

static inline struct device *omap_control_get(void)
{
	return ERR_PTR(-EINVAL);
}

static inline void omap_control_put(struct device *dev)
{

}
#endif

#endif /* __DRIVERS_OMAP_CONTROL_H */
