
/*
 * Board support file for OMAP4430 SDP.
 *
 * Copyright (C) 2009 Texas Instruments
 *
 * Author: Santosh Shilimkar <santosh.shilimkar@ti.com>
 *
 * Based on mach-omap2/board-3430sdp.c
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/io.h>
#include <linux/gpio.h>
#include <linux/moduleparam.h>
#include <linux/usb/otg.h>
#include <linux/spi/spi.h>
#include <linux/hwspinlock.h>
#include <linux/bootmem.h>
#include <linux/i2c/twl.h>
//#include <linux/i2c/bq2415x.h>
#include <linux/gpio_keys.h>
#include <linux/regulator/machine.h>
#include <linux/regulator/fixed.h>
//#include <linux/regulator/tps6130x.h>
#include <linux/leds.h>
#include <linux/leds_pwm.h>
#include <linux/omapfb.h>
#include <linux/reboot.h>
//#include <linux/twl6040-vib.h>
#include <linux/wl12xx.h>
#include <linux/memblock.h>
//#include <linux/mfd/twl6040-codec.h>

#include <linux/input/ft5x06.h>

#ifdef CONFIG_INPUT_KXTJ9
#include <linux/input/kxtj9.h>
#endif

#ifdef CONFIG_INPUT_KXTF9
#include <linux/input/kxtf9.h>
#endif

#include <linux/power/max17042.h>
#include <linux/power/max8903.h>
#include <linux/platform_data/omap4-keypad.h>

#include <mach/board-nooktablet.h>
#include <mach/hardware.h>
//#include <mach/omap4-common.h>
//#include <mach/emif.h>
//#include <mach/lpddr2-elpida.h>
//#include <mach/lpddr2-samsung.h>
//#include <mach/dmm.h>
#include <asm/hardware/gic.h>
#include <asm/mach-types.h>
#include <asm/mach/arch.h>
#include <asm/mach/map.h>

#include <plat/board.h>
//#include <plat/common.h>
#include <plat/usb.h>
#include <plat/mmc.h>
#include <plat/omap4-keypad.h>
#include <plat/omap_apps_brd_id.h>
#include <plat/omap-serial.h>
#include <plat/drm.h>
#include <plat/remoteproc.h>
#include <video/omapdss.h>
#include <video/omap-panel-nokia-dsi.h>
#include <plat/vram.h>
#include <plat/omap-pm.h>
//#include <plat/android-display.h>
#include <linux/wakelock.h>
//#include "board-blaze.h"
#include <mach/omap4_ion.h>
#include "omap_ram_console.h"
#include "mux.h"
#include "mux44xx.h"
#include "hsmmc.h"
//#include "timer-gp.h"
#include "control.h"
#include "common-board-devices.h"
#include "common.h"
#include "pm.h"
#include "prm-regbits-44xx.h"
#include "prm44xx.h"
/* for TI WiLink devices */
#include <linux/skbuff.h>
#include <linux/ti_wilink_st.h>
#include <plat/omap-serial.h>
#include <mach/omap-secure.h>

#define WILINK_UART_DEV_NAME "/dev/ttyO1"

#ifdef CONFIG_INPUT_KXTF9
#define KXTF9_DEVICE_ID                 "kxtf9"
#define KXTF9_I2C_SLAVE_ADDRESS         0x0F
#define KXTF9_GPIO_FOR_PWR              34
#endif

#ifdef CONFIG_INPUT_KXTJ9
#define KXTJ9_DEVICE_ID                 "kxtj9"
#define KXTJ9_I2C_SLAVE_ADDRESS         0x0F
#define KXTJ9_GPIO_FOR_PWR              34
#endif

#define CONFIG_SERIAL_OMAP_IDLE_TIMEOUT 5

#define GPIO_WIFI_PWEN                  114
#define GPIO_WIFI_PMENA			118
#define GPIO_WIFI_IRQ			115

#define FT5x06_I2C_SLAVEADDRESS  	(0x70 >> 1)
#define OMAP_FT5x06_GPIO         	37	/*99 */
#define OMAP_FT5x06_RESET_GPIO   	39	/*46 */

#define TWL6030_RTC_GPIO 		6
#define BLUETOOTH_UART			UART2
#define CONSOLE_UART			UART1

#define  SAMSUNG_SDRAM 0x1
#define  ELPIDA_SDRAM  0x3
#define  HYNIX_SDRAM   0x6

#define MAX17042_GPIO_FOR_IRQ  65
#define KXTF9_GPIO_FOR_IRQ  66
#define KXTJ9_GPIO_FOR_IRQ  66

#define FIXED_REG_TOUCH_ID	0
#define FIXED_REG_LCD_ID	1
#define FIXED_REG_VWLAN_ID	2

static void acclaim_panel_init(void);

#ifdef CONFIG_BATTERY_MAX17042
static void max17042_dev_init(void)
{
	printk("board-4430sdp.c: max17042_dev_init ...\n");

	if (gpio_request(MAX17042_GPIO_FOR_IRQ, "max17042_irq") < 0) {
		printk(KERN_ERR "Can't get GPIO for max17042 IRQ\n");
		return;
	}

	printk
	    ("board-4430sdp.c: max17042_dev_init > Init max17042 irq pin %d !\n",
	     MAX17042_GPIO_FOR_IRQ);
	gpio_direction_input(MAX17042_GPIO_FOR_IRQ);
	printk("max17042 GPIO pin read %d\n",
	       gpio_get_value(MAX17042_GPIO_FOR_IRQ));
}
#endif

#ifdef CONFIG_INPUT_KXTF9
static void kxtf9_dev_init(void)
{
	printk("board-4430sdp.c: kxtf9_dev_init ...\n");

	if (gpio_request(KXTF9_GPIO_FOR_IRQ, "kxtf9_irq") < 0) {
		printk("Can't get GPIO for kxtf9 IRQ\n");
		return;
	}

	printk("board-4430sdp.c: kxtf9_dev_init > Init kxtf9 irq pin %d !\n",
	       KXTF9_GPIO_FOR_IRQ);
	gpio_direction_input(KXTF9_GPIO_FOR_IRQ);
}

struct kxtf9_platform_data kxtf9_platform_data_here = {
	.min_interval = 1,
	.poll_interval = 1000,

	.g_range = KXTF9_G_8G,
	.shift_adj = SHIFT_ADJ_2G,

	/* Map the axes from the sensor to the device */
	/* SETTINGS FOR acclaim */
	.axis_map_x = 1,
	.axis_map_y = 0,
	.axis_map_z = 2,
	.negate_x = 0,
	.negate_y = 1,
	.negate_z = 0,
	.data_odr_init = ODR12_5F,
//      .res_12bit      = 1,
	.ctrl_reg1_init = KXTF9_G_8G | RES_12BIT | TDTE | WUFE | TPE,
	.int_ctrl_init = KXTF9_IEN | KXTF9_IEA | KXTF9_IEL,
	.int_ctrl_init = KXTF9_IEN,
	.tilt_timer_init = 0x03,
	.engine_odr_init = OTP12_5 | OWUF50 | OTDT400,
	.wuf_timer_init = 0x16,
	.wuf_thresh_init = 0x28,
	.tdt_timer_init = 0x78,
	.tdt_h_thresh_init = 0xFF,
	.tdt_l_thresh_init = 0x14,
	.tdt_tap_timer_init = 0x53,
	.tdt_total_timer_init = 0x24,
	.tdt_latency_timer_init = 0x10,
	.tdt_window_timer_init = 0xA0,

	.gpio = KXTF9_GPIO_FOR_IRQ,
};
#endif

#ifdef CONFIG_INPUT_KXTJ9
static void kxtj9_dev_init(void)
{
	printk("board-4430sdp.c: kxtj9_dev_init ...\n");

	if (gpio_request(KXTJ9_GPIO_FOR_IRQ, "kxtj9_irq") < 0) {
		printk("Can't get GPIO for kxtj9 IRQ\n");
		return;
	}

	printk("board-4430sdp.c: kxtj9_dev_init > Init kxtj9 irq pin %d !\n",
	       KXTJ9_GPIO_FOR_IRQ);
	gpio_direction_input(KXTJ9_GPIO_FOR_IRQ);
}

struct kxtj9_platform_data kxtj9_platform_data_here = {
	.min_interval = 1,
	.g_range = KXTJ9_G_8G,
	/* Map the axes from the sensor to the device */
	/* SETTINGS FOR acclaim */
	.axis_map_x = 0,
	.axis_map_y = 1,
	.axis_map_z = 2,
	.negate_x = 0,
	.negate_y = 0,
	.negate_z = 0,
	.data_odr_init = ODR12_5F,
	.res_12bit = 1,
};
#endif

int __init ft5x06_dev_init(int resource)
{
	if (resource) {
		omap_mux_init_signal("gpmc_ad13.gpio_37",
				     OMAP_PIN_INPUT |
				     OMAP_PIN_OFF_WAKEUPENABLE);
		omap_mux_init_signal("gpmc_ad15.gpio_39", OMAP_PIN_OUTPUT);

		if (gpio_request(OMAP_FT5x06_RESET_GPIO, "ft5x06_reset") < 0) {
			printk(KERN_ERR "can't get ft5x06 xreset GPIO\n");
			return -1;
		}

		if (gpio_request(OMAP_FT5x06_GPIO, "ft5x06_touch") < 0) {
			printk(KERN_ERR "can't get ft5x06 interrupt GPIO\n");
			return -1;
		}

		gpio_direction_input(OMAP_FT5x06_GPIO);
	} else {
		gpio_free(OMAP_FT5x06_GPIO);
		gpio_free(OMAP_FT5x06_RESET_GPIO);
	}

	return 0;
}

static struct ft5x06_platform_data ft5x06_platform_data = {
	.maxx = 600,
	.maxy = 1024,
	.flags = REVERSE_Y_FLAG,
	.reset_gpio = OMAP_FT5x06_RESET_GPIO,
	.use_st = FT_USE_ST,
	.use_mt = FT_USE_MT,
	.use_trk_id = 1,
	.use_sleep = FT_USE_SLEEP,
	.use_gestures = 1,
//      .platform_suspend = ft5x06_platform_suspend,
//      .platform_resume = ft5x06_platform_resume,
};

#ifdef CONFIG_CHARGER_MAX8903

static struct resource max8903_gpio_resources_evt1a[] = {
	{.name = MAX8903_TOKEN_GPIO_CHG_EN,
	 .start = MAX8903_GPIO_CHG_EN,
	 .end = MAX8903_GPIO_CHG_EN,
	 .flags = IORESOURCE_IO,
	 },
	{
	 .name = MAX8903_TOKEN_GPIO_CHG_FLT,
	 .start = MAX8903_GPIO_CHG_FLT,
	 .end = MAX8903_GPIO_CHG_FLT,
	 .flags = IORESOURCE_IO,
	 },
	{
	 .name = MAX8903_TOKEN_GPIO_CHG_IUSB,
	 .start = MAX8903_GPIO_CHG_IUSB,
	 .end = MAX8903_GPIO_CHG_IUSB,
	 .flags = IORESOURCE_IO,
	 },
	{
	 .name = MAX8903_TOKEN_GPIO_CHG_USUS,
	 .start = MAX8903_GPIO_CHG_USUS_EVT1A,
	 .end = MAX8903_GPIO_CHG_USUS_EVT1A,
	 .flags = IORESOURCE_IO,
	 },
	{
	 .name = MAX8903_TOKEN_GPIO_CHG_ILM,
	 .start = MAX8903_GPIO_CHG_ILM_EVT1A,
	 .end = MAX8903_GPIO_CHG_ILM_EVT1A,
	 .flags = IORESOURCE_IO,
	 },
	{
	 .name = MAX8903_TOKEN_GPIO_CHG_UOK,
	 .start = MAX8903_UOK_GPIO_FOR_IRQ,
	 .end = MAX8903_UOK_GPIO_FOR_IRQ,
	 .flags = IORESOURCE_IO,
	 },
	{
	 .name = MAX8903_TOKEN_GPIO_CHG_DOK,
	 .start = MAX8903_DOK_GPIO_FOR_IRQ,
	 .end = MAX8903_DOK_GPIO_FOR_IRQ,
	 .flags = IORESOURCE_IO,
	 }
};

static struct resource max8903_gpio_resources_evt1b[] = {
	{.name = MAX8903_TOKEN_GPIO_CHG_EN,
	 .start = MAX8903_GPIO_CHG_EN,
	 .end = MAX8903_GPIO_CHG_EN,
	 .flags = IORESOURCE_IO,
	 },
	{
	 .name = MAX8903_TOKEN_GPIO_CHG_FLT,
	 .start = MAX8903_GPIO_CHG_FLT,
	 .end = MAX8903_GPIO_CHG_FLT,
	 .flags = IORESOURCE_IO,
	 },
	{
	 .name = MAX8903_TOKEN_GPIO_CHG_IUSB,
	 .start = MAX8903_GPIO_CHG_IUSB,
	 .end = MAX8903_GPIO_CHG_IUSB,
	 .flags = IORESOURCE_IO,
	 },
	{
	 .name = MAX8903_TOKEN_GPIO_CHG_USUS,
	 .start = MAX8903_GPIO_CHG_USUS_EVT1B,
	 .end = MAX8903_GPIO_CHG_USUS_EVT1B,
	 .flags = IORESOURCE_IO,
	 },
	{
	 .name = MAX8903_TOKEN_GPIO_CHG_ILM,
	 .start = MAX8903_GPIO_CHG_ILM_EVT1B,
	 .end = MAX8903_GPIO_CHG_ILM_EVT1B,
	 .flags = IORESOURCE_IO,
	 },
	{
	 .name = MAX8903_TOKEN_GPIO_CHG_UOK,
	 .start = MAX8903_UOK_GPIO_FOR_IRQ,
	 .end = MAX8903_UOK_GPIO_FOR_IRQ,
	 .flags = IORESOURCE_IO,
	 },
	{
	 .name = MAX8903_TOKEN_GPIO_CHG_DOK,
	 .start = MAX8903_DOK_GPIO_FOR_IRQ,
	 .end = MAX8903_DOK_GPIO_FOR_IRQ,
	 .flags = IORESOURCE_IO,
	 }
};

static struct resource max8903_gpio_resources_dvt[] = {
	{.name = MAX8903_TOKEN_GPIO_CHG_EN,
	 .start = MAX8903_GPIO_CHG_EN,
	 .end = MAX8903_GPIO_CHG_EN,
	 .flags = IORESOURCE_IO,
	 },
	{
	 .name = MAX8903_TOKEN_GPIO_CHG_FLT,
	 .start = MAX8903_GPIO_CHG_FLT,
	 .end = MAX8903_GPIO_CHG_FLT,
	 .flags = IORESOURCE_IO,
	 },
	{
	 .name = MAX8903_TOKEN_GPIO_CHG_IUSB,
	 .start = MAX8903_GPIO_CHG_IUSB,
	 .end = MAX8903_GPIO_CHG_IUSB,
	 .flags = IORESOURCE_IO,
	 },
	{
	 .name = MAX8903_TOKEN_GPIO_CHG_USUS,
	 .start = MAX8903_GPIO_CHG_USUS_DVT,
	 .end = MAX8903_GPIO_CHG_USUS_DVT,
	 .flags = IORESOURCE_IO,
	 },
	{
	 .name = MAX8903_TOKEN_GPIO_CHG_ILM,
	 .start = MAX8903_GPIO_CHG_ILM_DVT,
	 .end = MAX8903_GPIO_CHG_ILM_DVT,
	 .flags = IORESOURCE_IO,
	 },
	{
	 .name = MAX8903_TOKEN_GPIO_CHG_UOK,
	 .start = MAX8903_UOK_GPIO_FOR_IRQ,
	 .end = MAX8903_UOK_GPIO_FOR_IRQ,
	 .flags = IORESOURCE_IO,
	 },
	{
	 .name = MAX8903_TOKEN_GPIO_CHG_DOK,
	 .start = MAX8903_DOK_GPIO_FOR_IRQ,
	 .end = MAX8903_DOK_GPIO_FOR_IRQ,
	 .flags = IORESOURCE_IO,
	 }
};

static struct platform_device max8903_charger_device = {
	.name = "max8903_charger",
	.id = -1,
};

static inline void acclaim_init_charger(void)
{
	const int board_type = acclaim_board_type();

	pr_info("Acclaim init charger.\n");
	if (board_type >= DVT) {
		max8903_charger_device.resource = max8903_gpio_resources_dvt;
		max8903_charger_device.num_resources =
		    ARRAY_SIZE(max8903_gpio_resources_dvt);
	} else if (board_type >= EVT1B) {
		max8903_charger_device.resource = max8903_gpio_resources_evt1b;
		max8903_charger_device.num_resources =
		    ARRAY_SIZE(max8903_gpio_resources_evt1b);
	} else if (board_type == EVT1A) {
		max8903_charger_device.resource = max8903_gpio_resources_evt1a;
		max8903_charger_device.num_resources =
		    ARRAY_SIZE(max8903_gpio_resources_evt1a);
	} else {
		pr_err("%s: Acclaim board %d not supported\n", __func__,
		       board_type);
		return;
	}
	platform_device_register(&max8903_charger_device);
}

#endif

#ifdef CONFIG_BATTERY_MAX17042
struct max17042_platform_data max17042_platform_data_here = {

	.gpio = MAX17042_GPIO_FOR_IRQ,

};
#endif
static const uint32_t sdp4430_keymap[] = {
	KEY(0, 0, KEY_VOLUMEUP),
	KEY(1, 0, KEY_VOLUMEDOWN),
};

static struct omap_device_pad keypad_pads[] = {
	{.name = "kpd_col0.kpd_col0",
	 .enable = OMAP_WAKEUP_EN | OMAP_MUX_MODE0,
	 },
	{.name = "kpd_row0.kpd_row0",
	 .enable = OMAP_PULL_ENA | OMAP_PULL_UP | OMAP_WAKEUP_EN |
	 OMAP_MUX_MODE0 | OMAP_INPUT_EN,
	 },
	{.name = "kpd_row1.kpd_row1",
	 .enable = OMAP_PULL_ENA | OMAP_PULL_UP | OMAP_WAKEUP_EN |
	 OMAP_MUX_MODE0 | OMAP_INPUT_EN,
	 },
};

static struct matrix_keymap_data sdp4430_keymap_data = {
	.keymap = sdp4430_keymap,
	.keymap_size = ARRAY_SIZE(sdp4430_keymap),
};

static struct omap4_keypad_platform_data sdp4430_keypad_data = {
	.keymap_data = &sdp4430_keymap_data,
	.rows = 2,
	.cols = 1,
};

static struct omap_board_data keypad_data = {
	.id = 1,
	.pads = keypad_pads,
	.pads_cnt = ARRAY_SIZE(keypad_pads),
};

static struct gpio_keys_button acclaim_gpio_buttons[] = {
	{
	 .code = KEY_POWER,
	 .gpio = 29,
	 .desc = "POWER",
	 .active_low = 0,
	 .wakeup = 1,
	 },			// We use twl6030 PWR Button driver Can we disable it?
	{

	 .code = KEY_HOME,
	 .gpio = 32,
	 .desc = "HOME",
	 .active_low = 1,
	 .wakeup = 1,
	 }
};

static struct gpio_keys_platform_data acclaim_gpio_key_info = {
	.buttons = acclaim_gpio_buttons,
	.nbuttons = ARRAY_SIZE(acclaim_gpio_buttons),
};

static struct platform_device acclaim_keys_gpio = {
	.name = "gpio-keys",
	.id = -1,
	.dev = {
		.platform_data = &acclaim_gpio_key_info,
		},
};

static struct platform_device sdp4430_aic3110 = {
	.name = "tlv320aic3110-codec",
	.id = -1,
};

/*******************************************************/
static struct regulator_consumer_supply acclaim_lcd_supply[] = {
	REGULATOR_SUPPLY("vdds_dsi", "omapdss_dss"),
	REGULATOR_SUPPLY("vdds_dsi", "omapdss_dsi.0"),
	REGULATOR_SUPPLY("vdds_dsi", "omapdss_dsi.1"),
};

static struct regulator_init_data acclaim_lcd_vinit = {
	.constraints = {
			.min_uV = 3300000,
			.max_uV = 3300000,
			.valid_modes_mask =
			REGULATOR_MODE_NORMAL | REGULATOR_MODE_STANDBY,
			.valid_ops_mask =
			REGULATOR_CHANGE_MODE | REGULATOR_CHANGE_STATUS,
			.state_mem = {.enabled = false,.disabled = true,},
			.always_on = true,
			},
	.num_consumer_supplies = ARRAY_SIZE(acclaim_lcd_supply),
	.consumer_supplies = acclaim_lcd_supply,
};

static struct fixed_voltage_config acclaim_lcd_reg = {
	.supply_name = "lcd",
	.microvolts = 3300000,
	.gpio = -EINVAL,
	.enable_high = 1,
	.enabled_at_boot = 1,
	.init_data = &acclaim_lcd_vinit,
};

static struct platform_device acclaim_lcd_regulator = {
	.name = "reg-fixed-voltage",
	.id = 0,
	.dev = {
		.platform_data = &acclaim_lcd_reg,
		},
};

/***************************************************************/
static struct platform_device *sdp4430_devices[] __initdata = {
	//&sdp4430_leds_gpio,
	//&sdp4430_leds_pwm,
	&sdp4430_aic3110,
	&acclaim_keys_gpio,
//      &wl128x_device,
//      &btwilink_device,
	&acclaim_lcd_regulator,
//      &lcd_regulator_device,
//      &touch_regulator_device,
};

static struct omap_board_config_kernel sdp4430_config[] __initdata = {
};

// static void __init omap_4430sdp_init_early(void)
// {
//      omap2_init_common_infrastructure();
//      omap2_init_common_devices(NULL, NULL);
// #ifdef CONFIG_OMAP_32K_TIMER
//      omap2_gp_clockevent_set_gptimer(1);
// #endif
// }

static struct omap_musb_board_data musb_board_data = {
	.interface_type = MUSB_INTERFACE_UTMI,
	.mode = MUSB_OTG,
	.power = 500,
};

static struct twl4030_usb_data omap4_usbphy_data = {
	.phy_init = omap4430_phy_init,
	.phy_exit = omap4430_phy_exit,
	.phy_power = omap4430_phy_power,
	.phy_set_clock = omap4430_phy_set_clk,
	.phy_suspend = omap4430_phy_suspend,
};

static struct omap2_hsmmc_info mmc[] = {
	{
	 .mmc = 2,
	 .caps = MMC_CAP_4_BIT_DATA | MMC_CAP_8_BIT_DATA | MMC_CAP_1_8V_DDR,
	 .gpio_cd = -EINVAL,
	 .gpio_wp = -EINVAL,
	 .ocr_mask = MMC_VDD_165_195,
	 .nonremovable = true,
	 .no_off_init = true,
	 },
	{
	 .mmc = 1,
	 .caps = MMC_CAP_4_BIT_DATA | MMC_CAP_8_BIT_DATA | MMC_CAP_1_8V_DDR,
	 .gpio_cd = -EINVAL,
	 .gpio_wp = -EINVAL,
	 .nonremovable = false,
	 },
	{
	 .mmc = 3,
	 .caps = MMC_CAP_4_BIT_DATA | MMC_CAP_POWER_OFF_CARD,
	 .gpio_cd = -EINVAL,
	 .gpio_wp = -EINVAL,
	 .ocr_mask = MMC_VDD_165_195,
	 .nonremovable = true,
	 },
	{}			/* Terminator */
};

/* External SD-card */
static struct regulator_consumer_supply sdp4430_vmmc_supply[] = {
	{
	 .supply = "vmmc",
	 .dev_name = "omap_hsmmc.0",
	 },
};

/* Internal EMMC memory */
static struct regulator_consumer_supply sdp4430_vemmc_supply[] = {
	{
	 .supply = "vmmc",
	 .dev_name = "omap_hsmmc.1",
	 },
};

static struct regulator_consumer_supply omap4_sdp4430_vwlan_supply = {
	.supply = "vmmc",
	.dev_name = "omap_hsmmc.2",
};

static struct regulator_init_data sdp4430_vwlan = {
	.constraints = {
			.valid_ops_mask = REGULATOR_CHANGE_STATUS,
			},
	.num_consumer_supplies = 1,
	.consumer_supplies = &omap4_sdp4430_vwlan_supply,
};

static struct fixed_voltage_config sdp4430_vwlan_fv = {
	.supply_name = "vwl1271",
	.microvolts = 1800000,	/* 1.8V */
	.gpio = GPIO_WIFI_PMENA,
	.startup_delay = 70000,	/* 70msec */
	.enable_high = 1,
	.enabled_at_boot = 0,
	.init_data = &sdp4430_vwlan,
};

static struct platform_device omap_vwlan_device = {
	.name = "reg-fixed-voltage",
	.id = 1,		//FIXED_REG_VWLAN_ID,
	.dev = {
		.platform_data = &sdp4430_vwlan_fv,
		},
};

static int omap4_twl6030_hsmmc_late_init(struct device *dev)
{
	int irq = 0;
	struct platform_device *pdev = container_of(dev,
						    struct platform_device,
						    dev);
	struct omap_mmc_platform_data *pdata = dev->platform_data;

	/* Setting MMC1 Card detect Irq */
	if (pdev->id == 0) {
		irq = twl6030_mmc_card_detect_config();
		if (irq < 0) {
			pr_err("Failed configuring MMC1 card detect\n");
			return irq;
		}
		pdata->slots[0].card_detect_irq = irq;
		pdata->slots[0].card_detect = twl6030_mmc_card_detect;
	}
//      if (pdev->id == 2) {
//              ret = 0;
//              pdata->slots[0].mmc_data.built_in = 1;
//      }

	return 0;
}

static __init void omap4_twl6030_hsmmc_set_late_init(struct device *dev)
{
	struct omap_mmc_platform_data *pdata;

	/* dev can be null if CONFIG_MMC_OMAP_HS is not set */
	if (!dev) {
		pr_err("Failed %s\n", __func__);
		return;
	}
	pdata = dev->platform_data;
	pdata->init = omap4_twl6030_hsmmc_late_init;
}

static int __init omap4_twl6030_hsmmc_init(struct omap2_hsmmc_info *controllers)
{
	struct omap2_hsmmc_info *c;

	omap_hsmmc_init(controllers);
	for (c = controllers; c->mmc; c++)
		omap4_twl6030_hsmmc_set_late_init(&c->pdev->dev);

	return 0;
}

static struct regulator_consumer_supply audio_supply[] = {
	{.supply = "audio-pwr",},
};

static struct regulator_init_data sdp4430_vaux1 = {
	.constraints = {
			.min_uV = 1000000,
			.max_uV = 3000000,
			.apply_uV = true,
			.valid_modes_mask = REGULATOR_MODE_NORMAL
			| REGULATOR_MODE_STANDBY,
			.valid_ops_mask = REGULATOR_CHANGE_VOLTAGE
			| REGULATOR_CHANGE_MODE | REGULATOR_CHANGE_STATUS,
			.state_mem = {
				      .enabled = false,
				      .disabled = true,
				      },
			.always_on = true,
			},
};

static struct regulator_init_data sdp4430_vaux2 = {
	.constraints = {
			.min_uV = 1200000,
			.max_uV = 2800000,
			.apply_uV = true,
			.valid_modes_mask = REGULATOR_MODE_NORMAL
			| REGULATOR_MODE_STANDBY,
			.valid_ops_mask = REGULATOR_CHANGE_VOLTAGE
			| REGULATOR_CHANGE_MODE | REGULATOR_CHANGE_STATUS,
			.state_mem = {
				      .enabled = false,
				      .disabled = true,
				      },
			},
};

static struct regulator_consumer_supply sdp4430_vwlan_supply[] = {
	{
	 .supply = "vwlan",
	 },
};

static struct regulator_init_data sdp4430_vaux3 = {
	.constraints = {
			.min_uV = 1800000,
			.max_uV = 1800000,
			.apply_uV = true,
			.valid_modes_mask = REGULATOR_MODE_NORMAL
			| REGULATOR_MODE_STANDBY,
			.valid_ops_mask = REGULATOR_CHANGE_VOLTAGE
			| REGULATOR_CHANGE_MODE | REGULATOR_CHANGE_STATUS,
			.state_mem = {
				      .enabled = false,
				      .disabled = true,
				      },
			.always_on = true,
			},
	.num_consumer_supplies = 1,
	.consumer_supplies = sdp4430_vwlan_supply,
};

static struct regulator_init_data sdp4430_vmmc = {
	.constraints = {
			.min_uV = 1200000,
			.max_uV = 3000000,
			.apply_uV = true,
			.valid_modes_mask = REGULATOR_MODE_NORMAL
			| REGULATOR_MODE_STANDBY,
			.valid_ops_mask = REGULATOR_CHANGE_VOLTAGE
			| REGULATOR_CHANGE_MODE | REGULATOR_CHANGE_STATUS,
			.state_mem = {
				      .enabled = false,
				      .disabled = true,
				      },
			},
	.num_consumer_supplies = 1,
	.consumer_supplies = sdp4430_vmmc_supply,
};

static struct regulator_init_data sdp4430_vpp = {
	.constraints = {
			.min_uV = 1800000,
			.max_uV = 2500000,
			.apply_uV = true,
			.valid_modes_mask = REGULATOR_MODE_NORMAL
			| REGULATOR_MODE_STANDBY,
			.valid_ops_mask = REGULATOR_CHANGE_VOLTAGE
			| REGULATOR_CHANGE_MODE | REGULATOR_CHANGE_STATUS,
			.state_mem = {
				      .enabled = false,
				      .disabled = true,
				      },
			},
};

static struct regulator_init_data sdp4430_vusim = {
	.constraints = {
			.min_uV = 1200000,
			.max_uV = 2900000,
			.apply_uV = true,
			.valid_modes_mask = REGULATOR_MODE_NORMAL
			| REGULATOR_MODE_STANDBY,
			.valid_ops_mask = REGULATOR_CHANGE_VOLTAGE
			| REGULATOR_CHANGE_MODE | REGULATOR_CHANGE_STATUS,
			.state_mem = {
				      .enabled = false,
				      .disabled = true,
				      },
			},
	.num_consumer_supplies = 1,
	.consumer_supplies = audio_supply,
};

static struct regulator_init_data sdp4430_vana = {
	.constraints = {
			.min_uV = 2100000,
			.max_uV = 2100000,
			//.apply_uV             = true,
			.valid_modes_mask = REGULATOR_MODE_NORMAL
			| REGULATOR_MODE_STANDBY,
			.valid_ops_mask = REGULATOR_CHANGE_MODE
			| REGULATOR_CHANGE_STATUS,
			.state_mem = {
				      .enabled = false,
				      .disabled = true,
				      },
			},
};

static struct regulator_init_data sdp4430_vcxio = {
	.constraints = {
			.min_uV = 1800000,
			.max_uV = 1800000,
			//.apply_uV             = true,
			.valid_modes_mask = REGULATOR_MODE_NORMAL
			| REGULATOR_MODE_STANDBY,
			.valid_ops_mask = REGULATOR_CHANGE_MODE
			| REGULATOR_CHANGE_STATUS,
			.state_mem = {
				      .enabled = false,
				      .disabled = true,
				      },
			.always_on = true,
			},
};

static struct regulator_init_data sdp4430_vdac = {
	.constraints = {
			.min_uV = 1800000,
			.max_uV = 1800000,
			.apply_uV = true,
			.valid_modes_mask = REGULATOR_MODE_NORMAL
			| REGULATOR_MODE_STANDBY,
			.valid_ops_mask = REGULATOR_CHANGE_MODE
			| REGULATOR_CHANGE_STATUS,
			.state_mem = {
				      .enabled = false,
				      .disabled = true,
				      },
			},
};

static struct regulator_consumer_supply vusb_supply[] = {
	REGULATOR_SUPPLY("vusb", "twl6030_usb"),
};

static struct regulator_init_data sdp4430_vusb = {
	.constraints = {
			.min_uV = 3300000,
			.max_uV = 3300000,
			//.apply_uV             = true,
			.valid_modes_mask = REGULATOR_MODE_NORMAL
			| REGULATOR_MODE_STANDBY,
			.valid_ops_mask = REGULATOR_CHANGE_MODE
			| REGULATOR_CHANGE_STATUS,
			.state_mem = {
				      .disabled = true,
				      },
			.initial_state = PM_SUSPEND_MEM,
			},
	.num_consumer_supplies = ARRAY_SIZE(vusb_supply),
	.consumer_supplies = vusb_supply,
};

static struct regulator_init_data sdp4430_clk32kg = {
	.constraints = {
			.valid_modes_mask = REGULATOR_MODE_NORMAL,
			.valid_ops_mask = REGULATOR_CHANGE_STATUS,
			.always_on = true,
			},
};

static struct twl4030_madc_platform_data sdp4430_gpadc_data = {
	.irq_line = 1,
};

static int sdp4430_batt_table[] = {
	/* adc code for temperature in degree C */
	929, 925,		/* -2 ,-1 */
	920, 917, 912, 908, 904, 899, 895, 890, 885, 880,	/* 00 - 09 */
	875, 869, 864, 858, 853, 847, 841, 835, 829, 823,	/* 10 - 19 */
	816, 810, 804, 797, 790, 783, 776, 769, 762, 755,	/* 20 - 29 */
	748, 740, 732, 725, 718, 710, 703, 695, 687, 679,	/* 30 - 39 */
	671, 663, 655, 647, 639, 631, 623, 615, 607, 599,	/* 40 - 49 */
	591, 583, 575, 567, 559, 551, 543, 535, 527, 519,	/* 50 - 59 */
	511, 504, 496		/* 60 - 62 */
};

/*static struct twl4030_bci_platform_data sdp4430_bci_data = {
	.monitoring_interval		= 10,
	.max_charger_currentmA		= 1500,
	.max_charger_voltagemV		= 4560,
	.max_bat_voltagemV		= 4200,
	.low_bat_voltagemV		= 3300,
	.battery_tmp_tbl		= sdp4430_batt_table,
	.tblsize			= ARRAY_SIZE(sdp4430_batt_table),
};*/

static struct twl4030_platform_data sdp4430_twldata = {
	.irq_base = TWL6030_IRQ_BASE,
	.irq_end = TWL6030_IRQ_END,

	/* Regulators */
	.vmmc = &sdp4430_vmmc,
	.vpp = &sdp4430_vpp,
//      .vusim          = &sdp4430_vusim,
	.vana = &sdp4430_vana,
	.vcxio = &sdp4430_vcxio,
//      .vdac           = &sdp4430_vdac,
//      .vusb           = &sdp4430_vusb,
	.vaux1 = &sdp4430_vaux1,
//      .vaux2          = &sdp4430_vaux2,
	.vaux3 = &sdp4430_vaux3,
	.clk32kg = &sdp4430_clk32kg,
//      .usb            = &omap4_usbphy_data,
//      .bci            = &sdp4430_bci_data,
	/* children */
//      .codec          = &twl6040_codec,
	.madc = &sdp4430_gpadc_data,

};

static struct i2c_board_info __initdata sdp4430_i2c_1_boardinfo[] = {
#ifdef CONFIG_INPUT_KXTF9
	{
	 I2C_BOARD_INFO(KXTF9_DEVICE_ID, KXTF9_I2C_SLAVE_ADDRESS),
	 .platform_data = &kxtf9_platform_data_here,
	 .irq = OMAP_GPIO_IRQ(66),
	 },
#endif
#ifdef CONFIG_INPUT_KXTJ9
	{
	 I2C_BOARD_INFO(KXTJ9_DEVICE_ID, KXTJ9_I2C_SLAVE_ADDRESS),
	 .platform_data = &kxtj9_platform_data_here,
	 .irq = OMAP_GPIO_IRQ(66),
	 },
#endif
	{
	 I2C_BOARD_INFO(MAX17042_DEVICE_ID, MAX17042_I2C_SLAVE_ADDRESS),
	 .platform_data = &max17042_platform_data_here,
	 .irq = OMAP_GPIO_IRQ(65),
	 },
};

static struct i2c_board_info __initdata sdp4430_i2c_2_boardinfo[] = {
	{
	 I2C_BOARD_INFO(FT_DEVICE_5x06_NAME, FT5x06_I2C_SLAVEADDRESS),
	 .platform_data = &ft5x06_platform_data,
	 .irq = OMAP_GPIO_IRQ(OMAP_FT5x06_GPIO),
	 },
//      {
//              I2C_BOARD_INFO("tlv320aic3100", 0x18),
//      },
};

// static struct i2c_board_info __initdata sdp4430_i2c_3_boardinfo[] = {
// 
// };
// 
// static struct i2c_board_info __initdata sdp4430_i2c_4_boardinfo[] = {
// 
// };
static void __init show_acclaim_board_revision(int revision)
{
	switch (revision) {
	case EVT1A:
		printk(KERN_INFO "Board revision %s\n", "EVT1A");
		break;
	case EVT1B:
		printk(KERN_INFO "Board revision %s\n", "EVT1B");
		break;
	case EVT2:
		printk(KERN_INFO "Board revision %s\n", "EVT2");
		break;
	case DVT:
		printk(KERN_INFO "Board revision %s\n", "DVT");
		break;
	case PVT:
		printk(KERN_INFO "Board revision %s\n", "PVT");
		break;
	default:
		printk(KERN_ERR "Board revision UNKNOWN (0x%x)\n", revision);
		break;
	}
}

void __init acclaim_board_init(void)
{
	const int board_type = acclaim_board_type();
	show_acclaim_board_revision(board_type);

	omap_mux_init_signal("sys_pwron_reset_out", OMAP_MUX_MODE3);
	omap_mux_init_signal("fref_clk3_req",
			     OMAP_MUX_MODE0 | OMAP_PIN_INPUT_PULLDOWN);
}

static void __init blaze_pmic_mux_init(void)
{

	omap_mux_init_signal("sys_nirq1", OMAP_PIN_INPUT_PULLUP |
			     OMAP_WAKEUP_EN);
}

static struct omap_i2c_bus_board_data __initdata sdp4430_i2c_1_bus_pdata;
static struct omap_i2c_bus_board_data __initdata sdp4430_i2c_2_bus_pdata;
static struct omap_i2c_bus_board_data __initdata sdp4430_i2c_3_bus_pdata;
static struct omap_i2c_bus_board_data __initdata sdp4430_i2c_4_bus_pdata;

static void __init omap_i2c_hwspinlock_init(int bus_id, int spinlock_id, struct omap_i2c_bus_board_data
					    *pdata)
{
/* spinlock_id should be -1 for a generic lock request */
	if (spinlock_id < 0)
		pdata->handle = hwspin_lock_request(USE_MUTEX_LOCK);
	else
		pdata->handle = hwspin_lock_request_specific(spinlock_id,
							     USE_MUTEX_LOCK);
	if (pdata->handle != NULL) {
		pdata->hwspin_lock_timeout = hwspin_lock_timeout;
		pdata->hwspin_unlock = hwspin_unlock;
	} else {
		pr_err("I2C hwspinlock request failed for bus %d\n", bus_id);
	}
}

static int __init omap4_i2c_init(void)
{
	int err;
	omap_i2c_hwspinlock_init(1, 0, &sdp4430_i2c_1_bus_pdata);
	omap_i2c_hwspinlock_init(2, 1, &sdp4430_i2c_2_bus_pdata);
	omap_i2c_hwspinlock_init(3, 2, &sdp4430_i2c_3_bus_pdata);
	omap_i2c_hwspinlock_init(4, 3, &sdp4430_i2c_4_bus_pdata);

	omap_register_i2c_bus_board_data(1, &sdp4430_i2c_1_bus_pdata);
	omap_register_i2c_bus_board_data(2, &sdp4430_i2c_2_bus_pdata);
	omap_register_i2c_bus_board_data(3, &sdp4430_i2c_3_bus_pdata);
	omap_register_i2c_bus_board_data(4, &sdp4430_i2c_4_bus_pdata);
// 
//      omap4_pmic_init("twl6030", &sdp4430_twldata);
//      
//      err=i2c_register_board_info(1,sdp4430_i2c_1_boardinfo, ARRAY_SIZE(sdp4430_i2c_1_boardinfo));
//      if (err)
//        return err;
// //   i2c_register_board_info(1, sdp4430_i2c_boardinfo,
// //                           ARRAY_SIZE(sdp4430_i2c_boardinfo));
//      omap_register_i2c_bus(2, 400, sdp4430_i2c_2_boardinfo,
//                              ARRAY_SIZE(sdp4430_i2c_2_boardinfo));
//      omap_register_i2c_bus(3, 400, NULL, 0);
//      omap_register_i2c_bus(4, 400, NULL, 0);
//      /*
//       * This will allow unused regulator to be shutdown. This flag
//       * should be set in the board file. Before regulators are registered.
//       */
//      regulator_has_full_constraints();

	omap4_pmic_get_config(&sdp4430_twldata, TWL_COMMON_PDATA_USB,
//                      TWL_COMMON_REGULATOR_VDAC |
//                        TWL_COMMON_REGULATOR_VAUX2 |
//                        TWL_COMMON_REGULATOR_VAUX3 |
//                        TWL_COMMON_REGULATOR_VMMC |
//                        TWL_COMMON_REGULATOR_VPP |
//                        TWL_COMMON_REGULATOR_VANA |
//                        TWL_COMMON_REGULATOR_VCXIO |
			      TWL_COMMON_REGULATOR_VUSB |
			      TWL_COMMON_REGULATOR_CLK32KG);
//                        TWL_COMMON_REGULATOR_CLK32KG );
	omap_pmic_init(1, 400, "twl6030", OMAP44XX_IRQ_SYS_1N,
		       &sdp4430_twldata);
//      ("twl6030", &sdp4430_twldata,NULL, 0);
	err =
	    i2c_register_board_info(1, sdp4430_i2c_1_boardinfo,
				    ARRAY_SIZE(sdp4430_i2c_1_boardinfo));
	if (err)
		return err;
	omap_register_i2c_bus(2, 400, sdp4430_i2c_2_boardinfo,
			      ARRAY_SIZE(sdp4430_i2c_2_boardinfo));
	omap_register_i2c_bus(3, 400, NULL, 0);
	omap_register_i2c_bus(4, 400, NULL, 0);
	return 0;
}

static bool enable_suspend_off = true;
module_param(enable_suspend_off, bool, S_IRUSR | S_IRGRP | S_IROTH);

#ifdef CONFIG_OMAP_MUX
static struct omap_board_mux board_mux[] __initdata = {
	OMAP4_MUX(USBB1_ULPITLL_CLK, OMAP_MUX_MODE3 | OMAP_PIN_OUTPUT),
	{.reg_offset = OMAP_MUX_TERMINATOR},
};

#else
#define board_mux	NULL
#define board_wkup_mux NULL
#endif
static struct omap_device_pad serial1_pads[] __initdata = {
	{
	 .name = "uart1_cts.uart1_cts",
	 .enable = OMAP_PIN_INPUT_PULLUP | OMAP_MUX_MODE0,
	 },
	{
	 .name = "uart1_rts.uart1_rts",
	 .enable = OMAP_PIN_OUTPUT | OMAP_MUX_MODE0,
	 },
	{
	 .name = "uart3_cts_rctx.uart1_tx",
	 .enable = OMAP_PIN_OUTPUT | OMAP_MUX_MODE1,
	 },
	{
	 .name = "mcspi1_cs1.uart1_rx",
	 .flags = OMAP_DEVICE_PAD_REMUX | OMAP_DEVICE_PAD_WAKEUP,
	 .enable =
	 OMAP_PIN_INPUT_PULLUP | OMAP_PIN_OFF_WAKEUPENABLE | OMAP_MUX_MODE1,
	 .idle =
	 OMAP_PIN_INPUT_PULLUP | OMAP_PIN_OFF_WAKEUPENABLE | OMAP_MUX_MODE1,
	 },
};

static struct omap_device_pad serial2_pads[] __initdata = {
	{
	 .name = "uart2_cts.uart2_cts",
	 .enable = OMAP_PIN_INPUT_PULLUP | OMAP_MUX_MODE0,
	 .flags = OMAP_DEVICE_PAD_REMUX,
	 .idle = OMAP_WAKEUP_EN | OMAP_PIN_OFF_INPUT_PULLUP | OMAP_MUX_MODE0,
	 },
	{
	 .name = "uart2_rts.uart2_rts",
	 .flags = OMAP_DEVICE_PAD_REMUX,
	 .enable = OMAP_PIN_OUTPUT | OMAP_MUX_MODE0,
	 .idle = OMAP_PIN_OFF_INPUT_PULLUP | OMAP_MUX_MODE7,
	 },
	{
	 .name = "uart2_tx.uart2_tx",
	 .enable = OMAP_PIN_OUTPUT | OMAP_MUX_MODE0,
	 },
	{
	 .name = "uart2_rx.uart2_rx",
	 .enable = OMAP_PIN_INPUT_PULLUP | OMAP_MUX_MODE0,
	 },
};

static struct omap_device_pad serial3_pads[] __initdata = {
	{
	 .name = "uart3_cts_rctx.uart3_cts_rctx",
	 .enable = OMAP_PIN_INPUT_PULLUP | OMAP_MUX_MODE0,
	 },
	{
	 .name = "uart3_rts_sd.uart3_rts_sd",
	 .enable = OMAP_PIN_OUTPUT | OMAP_MUX_MODE0,
	 },
	{
	 .name = "uart3_tx_irtx.uart3_tx_irtx",
	 .enable = OMAP_PIN_OUTPUT | OMAP_MUX_MODE0,
	 },
	{
	 .name = "uart3_rx_irrx.uart3_rx_irrx",
	 .flags = OMAP_DEVICE_PAD_REMUX,	// | OMAP_DEVICE_PAD_WAKEUP,
	 .enable = OMAP_PIN_INPUT | OMAP_MUX_MODE0,
	 .idle = OMAP_PIN_INPUT | OMAP_MUX_MODE0,
	 },
};

static struct omap_device_pad serial4_pads[] __initdata = {
	{
	 .name = "uart4_tx.uart4_tx",
	 .enable = OMAP_PIN_OUTPUT | OMAP_MUX_MODE0,
	 },
	{
	 .name = "uart4_rx.uart4_rx",
	 .flags = OMAP_DEVICE_PAD_REMUX | OMAP_DEVICE_PAD_WAKEUP,
	 .enable = OMAP_PIN_INPUT | OMAP_MUX_MODE0,
	 .idle = OMAP_PIN_INPUT | OMAP_MUX_MODE0,
	 },
};

static inline void board_serial_init(void)
{
	struct omap_board_data bdata;
	bdata.flags = 0;
	bdata.pads = serial1_pads;
	bdata.pads_cnt = ARRAY_SIZE(serial1_pads);
	bdata.id = 0;

	omap_serial_init_port(&bdata, NULL);
	printk("Serial port 1 init - DONE\n");
}

/************************UART end ***************************************/

static void enable_rtc_gpio(void)
{
	/* To access twl registers we enable gpio6
	 * we need this so the RTC driver can work.
	 */
	gpio_request(6, "msecure");
	gpio_direction_output(6, 1);

	return;
}

static void omap4_tablet_wifi_mux_init(void)
{
	omap_mux_init_gpio(GPIO_WIFI_IRQ, OMAP_PIN_INPUT |
			   OMAP_PIN_OFF_WAKEUPENABLE);
	omap_mux_init_gpio(GPIO_WIFI_PMENA, OMAP_PIN_OUTPUT);
	omap_mux_init_gpio(GPIO_WIFI_PWEN, OMAP_PIN_OUTPUT);
	omap_mux_init_signal("uart2_cts.sdmmc3_clk",
			     OMAP_MUX_MODE1 | OMAP_PIN_INPUT_PULLUP);
	omap_mux_init_signal("uart2_rts.sdmmc3_cmd",
			     OMAP_MUX_MODE1 | OMAP_PIN_INPUT_PULLUP);
	omap_mux_init_signal("uart2_rx.sdmmc3_dat0",
			     OMAP_MUX_MODE1 | OMAP_PIN_INPUT_PULLUP);
	omap_mux_init_signal("uart2_tx.sdmmc3_dat1",
			     OMAP_MUX_MODE1 | OMAP_PIN_INPUT_PULLUP);
	omap_mux_init_signal("abe_mcbsp1_dx.sdmmc3_dat2",
			     OMAP_MUX_MODE1 | OMAP_PIN_INPUT_PULLUP);
	omap_mux_init_signal("abe_mcbsp1_fsx.sdmmc3_dat3",
			     OMAP_MUX_MODE1 | OMAP_PIN_INPUT_PULLUP);
}

static void wlan_set_power(bool enable)
{
	printk("%s(%i)\n", __func__, enable);
	gpio_direction_output(GPIO_WIFI_PWEN, enable);
	mdelay(100);
}

static struct wl12xx_platform_data omap4_tablet_wlan_data __initdata = {
	.irq = OMAP_GPIO_IRQ(GPIO_WIFI_IRQ),
	.board_ref_clock = WL12XX_REFCLOCK_38,
	.board_tcxo_clock = WL12XX_TCXOCLOCK_38_4,
	.set_power = wlan_set_power,
};

static void omap4_sdp4430_wifi_init(void)
{
	int status = 0;

	omap4_tablet_wifi_mux_init();

	status = gpio_request(GPIO_WIFI_PWEN, "wifi_pwen");
	if (unlikely(status < 0)) {
		pr_err("Error requesting WIFI power-en gpio (%d)\n",
		       GPIO_WIFI_PWEN);
		return;
	}
/* Needed for mmc2 initialization */
	gpio_direction_output(GPIO_WIFI_PWEN, 1);

	if (wl12xx_set_platform_data(&omap4_tablet_wlan_data))
		pr_err("Error setting wl12xx data\n");
	platform_device_register(&omap_vwlan_device);
}

    
#if defined(CONFIG_TI_EMIF) || defined(CONFIG_TI_EMIF_MODULE)
static struct __devinitdata emif_custom_configs custom_configs = {
	.mask»  = EMIF_CUSTOM_CONFIG_LPMODE,
	.lpmode»        = EMIF_LP_MODE_SELF_REFRESH,
	.lpmode_timeout_performance = 512,
	.lpmode_timeout_power = 512,
	/* only at OPP100 should we use performance value */
	.lpmode_freq_threshold = 400000000,
	};
#endif

static void __init omap_4430sdp_init(void)
{
	int status;
	int package = OMAP_PACKAGE_CBS;
	ulong sdram_size = get_sdram_size();

	if (omap_rev() == OMAP4430_REV_ES1_0)
		package = OMAP_PACKAGE_CBL;

#if defined(CONFIG_TI_EMIF) || defined(CONFIG_TI_EMIF_MODULE)
	omap_emif_set_device_details(1, &lpddr2_elpida_2G_S4_x2_info,
				     lpddr2_elpida_2G_S4_timings,
				     ARRAY_SIZE(lpddr2_elpida_2G_S4_timings),
				     &lpddr2_elpida_S4_min_tck, &custom_configs);
	omap_emif_set_device_details(2, &lpddr2_elpida_2G_S4_x2_info,
				     lpddr2_elpida_2G_S4_timings,
				     ARRAY_SIZE(lpddr2_elpida_2G_S4_timings),
				     &lpddr2_elpida_S4_min_tck, &custom_configs);
#endif

	omap4_mux_init(board_mux, NULL, package);
	omap4_i2c_init();
	platform_add_devices(sdp4430_devices, ARRAY_SIZE(sdp4430_devices));
	board_serial_init();	//omap_serial_init();
	omap_sdrc_init(NULL, NULL);
	omap4_sdp4430_wifi_init();
	omap4_twl6030_hsmmc_init(mmc);

	acclaim_board_init();

	omap_create_board_props();
//      blaze_pmic_mux_init();
//      blaze_set_osc_timings();

	enable_rtc_gpio();
//      blaze_sensor_init();
//      blaze_touch_init();
#ifdef CONFIG_CHARGER_MAX8903
	acclaim_init_charger();
#endif

//      wake_lock_init(&st_wk_lock, WAKE_LOCK_SUSPEND, "st_wake_lock");

//      /* blaze_modem_init shall be called before omap4_ehci_ohci_init */
//      if (!strcmp(modem_ipc, "hsi"))
//              blaze_modem_init(true);
//      else
//      blaze_modem_init(false);
// #ifdef CONFIG_TIWLAN_SDIO
//      config_wlan_mux();
// #else
//      omap4_4430sdp_wifi_init();
// #endif
#ifdef CONFIG_INPUT_KXTF9
	kxtf9_dev_init();
#endif

#ifdef CONFIG_INPUT_KXTJ9
	kxtj9_dev_init();
#endif

#ifdef CONFIG_BATTERY_MAX17042
	max17042_dev_init();
#endif
//      omap4_ehci_ohci_init();

	usb_musb_init(&musb_board_data);

	status = omap4_keyboard_init(&sdp4430_keypad_data, &keypad_data);
	if (status)
		pr_err("Keypad initialization failed: %d\n", status);
/*	spi_register_board_info(sdp4430_spi_board_info,
			ARRAY_SIZE(sdp4430_spi_board_info));*/

	omap_init_dmm_tiler();
	omap4_register_ion();

	acclaim_panel_init();
	omap_enable_smartreflex_on_init();
	if (enable_suspend_off)
		omap_pm_enable_off_mode();

}

#ifdef CONFIG_ANDROID_RAM_CONSOLE
static inline void ramconsole_reserve_sdram(void)
{
	// make the ram console the size of the printk log buffer
	ulong sdram_size = get_sdram_size();
	if (sdram_size == SZ_512M) {
		reserve_bootmem(NOOKTABLET_RAM_CONSOLE_512MB_START,
				NOOKTABLET_RAM_CONSOLE_SIZE, 0);
	} else {
		reserve_bootmem(NOOKTABLET_RAM_CONSOLE_START,
				NOOKTABLET_RAM_CONSOLE_SIZE, 0);
	}
}
#else
static inline void ramconsole_reserve_sdram(void)
{
}
#endif /* CONFIG_ANDROID_RAM_CONSOLE */

#include <plat/android-display.h>

#define LCD_RST_DELAY		100
#define LCD_INIT_DELAY		200

#define LCD_CABC0_GPIO 44
#define LCD_CABC1_GPIO 45
#define LCD_BL_PWR_EN_GPIO      	38
#define DEFAULT_BACKLIGHT_BRIGHTNESS	105
static void acclaim_init_display_led(void)
{
	if (acclaim_board_type() >= EVT2) {
		printk(KERN_INFO "init_display_led: evt2 hardware\n");
		omap_mux_init_signal("abe_dmic_din2.dmtimer11_pwm_evt",
				     OMAP_MUX_MODE5);
	} else {
		printk(KERN_INFO "init_display_led: evt1 hardware\n");
		printk(KERN_INFO
		       "WARNING: brigthness control disabled on EVT1 hardware\n");
		/* mux the brightness control pin as gpio, because on EVT1 it is connected to
		   timer8 and we cannot use timer8 because of audio conflicts causing crash */
		omap_mux_init_signal("usbb1_ulpitll_dat4.gpio_92",
				     OMAP_MUX_MODE3);
		if (gpio_request(92, "EVT1 BACKLIGHT"))
			printk(KERN_ERR
			       "ERROR: failed to request backlight gpio\n");
		else
			gpio_direction_output(92, 0);
	}
}

static void acclaim4430_disp_backlight_setpower(struct
						omap_pwm_led_platform_data
						*pdata, int on_off)
{
	printk(KERN_INFO "Backlight set power, on_off = %d\n", on_off);
	if (on_off) {
		msleep(350);	// give the boxer resume some time do spi init
		gpio_direction_output(LCD_BL_PWR_EN_GPIO,
				      (acclaim_board_type() >= EVT2) ? 1 : 0);
	} else {
		msleep(100);	// give the pwm led driver some time do dim
		gpio_direction_output(LCD_BL_PWR_EN_GPIO,
				      (acclaim_board_type() >= EVT2) ? 0 : 1);
	}

	gpio_direction_output(LCD_CABC0_GPIO, 0);
	gpio_direction_output(LCD_CABC0_GPIO, 0);

	pr_debug("%s: on_off:%d\n", __func__, on_off);
}

static struct omap_pwm_led_platform_data acclaim4430_disp_backlight_data = {
	.name = "lcd-backlight",
	.default_trigger = "backlight",
	.intensity_timer = 11,
	.bkl_max = 254,
	.bkl_min = 5,
	.bkl_freq = 128,
	.invert = 1,
	.def_brightness = DEFAULT_BACKLIGHT_BRIGHTNESS,
	.set_power = acclaim4430_disp_backlight_setpower,
};

static struct platform_device sdp4430_disp_led = {
	.name = "omap_pwm_led",
	.id = 0,
	.dev = {
		.platform_data = &acclaim4430_disp_backlight_data,
		},
};

static struct platform_device *sdp4430_panel_devices[] __initdata = {
	&sdp4430_disp_led,
};

static void acclaim_panel_get_resource(void)
{
	int ret_val = 0;

	pr_info("acclaim_panel_get_resource\n");
	ret_val = gpio_request(LCD_BL_PWR_EN_GPIO, "BOXER BL PWR EN");

	if (ret_val) {
		printk("%s : Could not request bl pwr en\n", __FUNCTION__);
	}
	ret_val = gpio_request(LCD_CABC0_GPIO, "BOXER CABC0");
	if (ret_val) {
		printk("%s : could not request CABC0\n", __FUNCTION__);
	}
	ret_val = gpio_request(LCD_CABC1_GPIO, "BOXER CABC1");
	if (ret_val) {
		printk("%s: could not request CABC1\n", __FUNCTION__);
	}
}

static struct boxer_panel_data boxer_panel;

static inline struct boxer_panel_data *get_panel_data(struct omap_dss_device
						      *dssdev)
{
	return dssdev->data;
}

static struct omap_dss_device acclaim_boxer_device = {
	.phy = {
		.dpi = {
			.data_lines = 24,
			},
		},
	.clocks = {
		   .dispc = {
			     .channel = {
					 .lck_div = 1,
					 .pck_div = 4,
					 .lcd_clk_src =
					 OMAP_DSS_CLK_SRC_DSI2_PLL_HSDIV_DISPC,
					 },
			     .dispc_fclk_src =
			     OMAP_DSS_CLK_SRC_DSI2_PLL_HSDIV_DISPC,
			     },
		   },
	.panel = {
		  .config = OMAP_DSS_LCD_TFT | OMAP_DSS_LCD_IVS | OMAP_DSS_LCD_IHS,	// | OMAP_DSS_LCD_IPC,
		  .timings = {
			      .x_res = 1024,
			      .y_res = 600,
			      .pixel_clock = 46000,	/* in kHz */
			      .hfp = 160,	/* HFP fix 160 */
			      .hsw = 10,	/* HSW = 1~140 */
			      .hbp = 160,	/* HSW + HBP = 160 */
			      .vfp = 10,	/* VFP fix 12 */
			      .vsw = 2,	/* VSW = 1~20 */
			      .vbp = 23,	/* VSW + VBP = 23 */
			      },
		  .width_in_um = 153000,
		  .height_in_um = 90000,
		  },
	.name = "lcd2",
	.driver_name = "boxer_panel",
	.type = OMAP_DISPLAY_TYPE_DPI,
	.channel = OMAP_DSS_CHANNEL_LCD2,
};

static struct omap_dss_device *sdp4430_dss_devices[] = {
	&acclaim_boxer_device,
};

static struct omap_dss_board_info sdp4430_dss_data = {
	.num_devices = ARRAY_SIZE(sdp4430_dss_devices),
	.devices = sdp4430_dss_devices,
	.default_device = &acclaim_boxer_device,
};

static struct spi_board_info tablet_spi_board_info[] __initdata = {
	[0] = {
	       .modalias = "boxer_disp_spi",
	       .bus_num = 4,	/* McSPI4 */
	       .chip_select = 0,
	       .max_speed_hz = 375000,
	       },
};

#define BLAZE_FB_RAM_SIZE                SZ_16M + SZ_4M	/* 1920×1080*4 * 2 */
static struct dsscomp_platform_data dsscomp_config_boxer = {
    .tiler1d_slotsz = (SZ_16M + SZ_4M),
    };
    
#ifdef CONFIG_FB_OMAP2_NUM_FBS
#define OMAPLFB_NUM_DEV CONFIG_FB_OMAP2_NUM_FBS
#else
#define OMAPLFB_NUM_DEV 1
#endif
    
static struct sgx_omaplfb_config omaplfb_config_boxer[OMAPLFB_NUM_DEV] = {
    {
    .vram_buffers = 2,
    .swap_chain_length = 2,
    }
};

static struct sgx_omaplfb_platform_data omaplfb_plat_data_boxer = {
	.num_configs = OMAPLFB_NUM_DEV,
	    .configs = omaplfb_config_boxer,
	    };

static struct omapfb_platform_data acclaim_fb_data = {
	.mem_desc = {
		     .region_cnt = 1,
		     },
};

void __init omap4_acclaim_display_setup(struct omap_ion_platform_data *ion)
{
	omap_android_display_setup(&sdp4430_dss_data,
				&dsscomp_config_boxer,
				&omaplfb_plat_data_boxer, &acclaim_fb_data, ion);
}

static void __init acclaim_panel_init(void)
{
	int ret;

	acclaim_panel_get_resource();
	acclaim_init_display_led();

	omapfb_set_platform_data(&acclaim_fb_data);
	spi_register_board_info(tablet_spi_board_info,
				ARRAY_SIZE(tablet_spi_board_info));

//    omap_mux_enable_wkup("sys_nirq1");
//    omap_mux_enable_wkup("sys_nirq2");

	platform_add_devices(sdp4430_panel_devices,
			     ARRAY_SIZE(sdp4430_panel_devices));
	omap_display_init(&sdp4430_dss_data);
}

static void __init omap_4430sdp_reserve(void)
{
	omap_init_ram_size();
	omap_ram_console_init(OMAP_RAM_CONSOLE_START_DEFAULT,
			      OMAP_RAM_CONSOLE_SIZE_DEFAULT);
	//ramconsole_reserve_sdram();
	omap_rproc_reserve_cma(RPROC_CMA_OMAP4);

#ifdef CONFIG_ION_OMAP
	omap4_acclaim_display_setup(get_omap_ion_platform_data());
	omap4_ion_init();
#else
	omap4_acclaim_display_setup(NULL);
#endif
	/* do the static reservations first */
	omap_secure_set_secure_workspace_addr(omap_smc_addr(), omap_smc_size());
//	memblock_remove(PHYS_ADDR_SMC_MEM, PHYS_ADDR_SMC_SIZE);
	omap_reserve();
}

MACHINE_START(OMAP4_NOOKTABLET, "acclaim")
    .atag_offset = 0x100,.reserve = omap_4430sdp_reserve,.map_io =
    omap4_map_io,.init_early = omap4430_init_early,.init_irq =
    gic_init_irq,.handle_irq = gic_handle_irq,.init_machine =
    omap_4430sdp_init,.timer = &omap4_timer,.restart =
    omap_prcm_restart, MACHINE_END
