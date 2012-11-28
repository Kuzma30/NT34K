/*
 * omap-mcpdm.h  --  OMAP McPDM
 *
 * Copyright (C) 2012 Texas Instruments
 *
 * Contact: Misael Lopez Cruz <misael.lopez@ti.com>
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

#ifndef _SND_OMAP_MCPDM_H
#define _SND_OMAP_MCPDM_H

struct omap_mcpdm_pdata {
	void (*disable_idle_on_suspend)(struct platform_device *pdev);
};

#endif
