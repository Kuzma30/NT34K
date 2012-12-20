/*
 * arch/arm/mach-omap2/board-54xx-sevm.h
 *
 * Copyright (C) 2012 Texas Instruments
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#ifndef _MACH_OMAP_BOARD_54XX_SEVM_H
#define _MACH_OMAP_BOARD_54XX_SEVM_H

#include "board-omap5evm.h"

int __init sevm_dock_init(void);
int __init sevm_touch_init(void);
int __init sevm_sensor_init(void);
int __init sevm_panel_init(void);

#endif
