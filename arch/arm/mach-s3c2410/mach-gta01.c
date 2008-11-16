/*
 * linux/arch/arm/mach-s3c2410/mach-gta01.c
 *
 * S3C2410 Machine Support for the FIC Neo1973 GTA01
 *
 * Copyright (C) 2006-2007 by Openmoko, Inc.
 * Author: Harald Welte <laforge@openmoko.org>
 * All rights reserved.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation; either version 2 of
 * the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston,
 * MA 02111-1307 USA
 *
 */

#include <linux/kernel.h>
#include <linux/types.h>
#include <linux/interrupt.h>
#include <linux/list.h>
#include <linux/timer.h>
#include <linux/init.h>
#include <linux/workqueue.h>
#include <linux/platform_device.h>
#include <linux/i2c.h>
#include <linux/serial_core.h>
#include <mach/ts.h>
#include <linux/spi/spi.h>
#include <linux/spi/spi_bitbang.h>
#include <linux/mmc/mmc.h>
#include <linux/mmc/host.h>

#include <linux/mtd/mtd.h>
#include <linux/mtd/nand.h>
#include <linux/mtd/nand_ecc.h>
#include <linux/mtd/partitions.h>

#include <linux/mmc/host.h>

#include <linux/pcf50606.h>

#include <asm/mach/arch.h>
#include <asm/mach/map.h>
#include <asm/mach/irq.h>

#include <mach/hardware.h>
#include <mach/io.h>
#include <asm/irq.h>
#include <asm/mach-types.h>

#include <mach/regs-gpio.h>
#include <mach/fb.h>
#include <mach/mci.h>
#include <mach/spi.h>
#include <mach/spi-gpio.h>
#include <mach/usb-control.h>

#include <mach/gta01.h>

#include <asm/plat-s3c/regs-serial.h>
#include <asm/plat-s3c/nand.h>
#include <asm/plat-s3c24xx/devs.h>
#include <asm/plat-s3c24xx/cpu.h>
#include <asm/plat-s3c24xx/pm.h>
#include <asm/plat-s3c24xx/udc.h>
#include <asm/plat-s3c24xx/neo1973.h>
#include <mach/neo1973-pm-gsm.h>

#include <linux/jbt6k74.h>

static struct map_desc gta01_iodesc[] __initdata = {
	{
		.virtual	= 0xe0000000,
		.pfn		= __phys_to_pfn(S3C2410_CS3+0x01000000),
		.length		= SZ_1M,
		.type		= MT_DEVICE
	},
};

#define UCON S3C2410_UCON_DEFAULT
#define ULCON S3C2410_LCON_CS8 | S3C2410_LCON_PNONE | S3C2410_LCON_STOPB
#define UFCON S3C2410_UFCON_RXTRIG8 | S3C2410_UFCON_FIFOMODE
/* UFCON for the gta01 sets the FIFO trigger level at 4, not 8 */
#define UFCON_GTA01_PORT0 S3C2410_UFCON_FIFOMODE

static struct s3c2410_uartcfg gta01_uartcfgs[] = {
	[0] = {
		.hwport	     = 0,
		.flags	     = 0,
		.ucon	     = UCON,
		.ulcon	     = ULCON,
		.ufcon	     = UFCON_GTA01_PORT0,
	},
	[1] = {
		.hwport	     = 1,
		.flags	     = 0,
		.ucon	     = UCON,
		.ulcon	     = ULCON,
		.ufcon	     = UFCON,
	},
};

/* PMU driver info */

static int pmu_callback(struct device *dev, unsigned int feature,
			enum pmu_event event)
{
	switch (feature) {
	case PCF50606_FEAT_ACD:
		switch (event) {
		case PMU_EVT_INSERT:
			pcf50606_charge_fast(pcf50606_global, 1);
			break;
		case PMU_EVT_REMOVE:
			pcf50606_charge_fast(pcf50606_global, 0);
			break;
		default:
			break;
		}
		break;
	default:
		break;
	}

	return 0;
}

static struct pcf50606_platform_data gta01_pcf_pdata = {
	.used_features	= PCF50606_FEAT_EXTON |
			  PCF50606_FEAT_MBC |
			  PCF50606_FEAT_BBC |
			  PCF50606_FEAT_RTC |
			  PCF50606_FEAT_WDT |
			  PCF50606_FEAT_CHGCUR |
			  PCF50606_FEAT_BATVOLT |
			  PCF50606_FEAT_BATTEMP,
	.onkey_seconds_required = 3,
	.cb		= &pmu_callback,
	.r_fix_batt	= 10000,
	.r_fix_batt_par	= 10000,
	.r_sense_milli	= 220,
	.rails	= {
		[PCF50606_REGULATOR_D1REG] = {
			.name		= "bt_3v15",
			.voltage	= {
				.init	= 3150,
				.max	= 3150,
			},
		},
		[PCF50606_REGULATOR_D2REG] = {
			.name		= "gl_2v5",
			.voltage	= {
				.init	= 2500,
				.max	= 2500,
			},
		},
		[PCF50606_REGULATOR_D3REG] = {
			.name		= "stby_1v8",
			.flags		= PMU_VRAIL_F_SUSPEND_ON,
			.voltage	= {
				.init	= 1800,
				.max	= 2100,
			},
		},
		[PCF50606_REGULATOR_DCD] = {
			.name		= "gl_1v5",
			.voltage	= {
				.init	= 1500,
				.max	= 1500,
			},
		},
		[PCF50606_REGULATOR_DCDE] = {
			.name		= "io_3v3",
			.flags		= PMU_VRAIL_F_SUSPEND_ON,
			.voltage	= {
				.init	= 3300,
				.max	= 3330,
			},
		},
		[PCF50606_REGULATOR_DCUD] = {
			.name		= "core_1v8",
			.flags		= PMU_VRAIL_F_SUSPEND_ON,
			.voltage	= {
				.init	= 2100,
				.max	= 2100,
			},
		},
		[PCF50606_REGULATOR_IOREG] = {
			.name		= "codec_3v3",
			.voltage	= {
				.init	= 3300,
				.max	= 3300,
			},
		},
		[PCF50606_REGULATOR_LPREG] = {
			.name		= "lcm_3v3",
			.voltage	= {
				.init	= 3300,
				.max	= 3300,
			},
		}
	},
};

static void cfg_pmu_vrail(struct pmu_voltage_rail *vrail, char *name,
			  unsigned int flags, unsigned int init,
			  unsigned int max)
{
	vrail->name = name;
	vrail->flags = flags;
	vrail->voltage.init = init;
	vrail->voltage.max = max;
}

static void mangle_pmu_pdata_by_system_rev(void)
{
	switch (system_rev) {
	case GTA01Bv4_SYSTEM_REV:
		gta01_pcf_pdata.used_features |= PCF50606_FEAT_ACD;
		break;
	case GTA01Bv3_SYSTEM_REV:
	case GTA01Bv2_SYSTEM_REV:
		gta01_pcf_pdata.rails[PCF50606_REGULATOR_D3REG]
					.name = "user1";
		gta01_pcf_pdata.rails[PCF50606_REGULATOR_D3REG]
					.flags &= ~PMU_VRAIL_F_SUSPEND_ON;
		gta01_pcf_pdata.rails[PCF50606_REGULATOR_D3REG]
					.flags = PMU_VRAIL_F_UNUSED;
		break;
	case GTA01v4_SYSTEM_REV:
		cfg_pmu_vrail(&gta01_pcf_pdata.rails[PCF50606_REGULATOR_DCUD],
			      "core_1v8", PMU_VRAIL_F_SUSPEND_ON, 1800, 1800);
		cfg_pmu_vrail(&gta01_pcf_pdata.rails[PCF50606_REGULATOR_D1REG],
			      "vrf_3v", 0, 3000, 3000);
		cfg_pmu_vrail(&gta01_pcf_pdata.rails[PCF50606_REGULATOR_D3REG],
			      "vtcxo_2v8", 0, 2800, 2800);
		cfg_pmu_vrail(&gta01_pcf_pdata.rails[PCF50606_REGULATOR_DCD],
			      "gl_3v5", 0, 3500, 3500);
		break;
	case GTA01v3_SYSTEM_REV:
		cfg_pmu_vrail(&gta01_pcf_pdata.rails[PCF50606_REGULATOR_D1REG],
			      "vrf_3v", 0, 3000, 3000);
		cfg_pmu_vrail(&gta01_pcf_pdata.rails[PCF50606_REGULATOR_D2REG],
			      "sd_3v3", 0, 3300, 3300);
		cfg_pmu_vrail(&gta01_pcf_pdata.rails[PCF50606_REGULATOR_D3REG],
			      "codec_3v3", 0, 3300, 3300);
		cfg_pmu_vrail(&gta01_pcf_pdata.rails[PCF50606_REGULATOR_DCD],
			      "gpsio_3v3", 0, 3300, 3300);
		cfg_pmu_vrail(&gta01_pcf_pdata.rails[PCF50606_REGULATOR_DCUD],
			      "core_1v8", PMU_VRAIL_F_SUSPEND_ON, 1800, 1800);
		cfg_pmu_vrail(&gta01_pcf_pdata.rails[PCF50606_REGULATOR_IOREG],
			      "vtcxo_2v8", 0, 2800, 2800);
		break;
	}
}

static struct resource gta01_pmu_resources[] = {
	[0] = {
		.flags	= IORESOURCE_IRQ,
		.start	= GTA01_IRQ_PCF50606,
		.end	= GTA01_IRQ_PCF50606,
	},
};

struct platform_device gta01_pmu_dev = {
	.name 		= "pcf50606",
	.num_resources	= ARRAY_SIZE(gta01_pmu_resources),
	.resource	= gta01_pmu_resources,
	.dev		= {
		.platform_data = &gta01_pcf_pdata,
	},
};

/* LCD driver info */

/* Configuration for 480x640 toppoly TD028TTEC1.
 * Do not mark this as __initdata or it will break! */
static struct s3c2410fb_display gta01_displays[] =  {
	{
		.type		= S3C2410_LCDCON1_TFT,
		.width		= 43,
		.height		= 58,
		.xres		= 480,
		.yres		= 640,
		.bpp		= 16,

		.pixclock	= 40000,	/* HCLK/4 */
		.left_margin	= 104,
		.right_margin	= 8,
		.hsync_len	= 8,
		.upper_margin	= 2,
		.lower_margin	= 16,
		.vsync_len	= 2,
		.lcdcon5	= S3C2410_LCDCON5_FRM565 |
				  S3C2410_LCDCON5_INVVCLK |
				  S3C2410_LCDCON5_INVVLINE |
				  S3C2410_LCDCON5_INVVFRAME |
				  S3C2410_LCDCON5_PWREN |
				  S3C2410_LCDCON5_HWSWP,
	},
	{
		.type		= S3C2410_LCDCON1_TFT,
		.width		= 43,
		.height		= 58,
		.xres		= 480,
		.yres		= 640,
		.bpp		= 32,

		.pixclock	= 40000,	/* HCLK/4 */
		.left_margin	= 104,
		.right_margin	= 8,
		.hsync_len	= 8,
		.upper_margin	= 2,
		.lower_margin	= 16,
		.vsync_len	= 2,
		.lcdcon5	= S3C2410_LCDCON5_FRM565 |
				  S3C2410_LCDCON5_INVVCLK |
				  S3C2410_LCDCON5_INVVLINE |
				  S3C2410_LCDCON5_INVVFRAME |
				  S3C2410_LCDCON5_PWREN |
				  S3C2410_LCDCON5_HWSWP,
	},
	{
		.type		= S3C2410_LCDCON1_TFT,
		.width		= 43,
		.height		= 58,
		.xres		= 240,
		.yres		= 320,
		.bpp		= 16,

		.pixclock	= 40000,	/* HCLK/4 */
		.left_margin	= 104,
		.right_margin	= 8,
		.hsync_len	= 8,
		.upper_margin	= 2,
		.lower_margin	= 16,
		.vsync_len	= 2,
		.lcdcon5	= S3C2410_LCDCON5_FRM565 |
				  S3C2410_LCDCON5_INVVCLK |
				  S3C2410_LCDCON5_INVVLINE |
				  S3C2410_LCDCON5_INVVFRAME |
				  S3C2410_LCDCON5_PWREN |
				  S3C2410_LCDCON5_HWSWP,
	},
};

static struct s3c2410fb_mach_info gta01_lcd_cfg __initdata = {
	.displays	= gta01_displays,
	.num_displays	= ARRAY_SIZE(gta01_displays),
	.default_display = 0,

	.lpcsel		= ((0xCE6) & ~7) | 1<<4,
};

static struct i2c_board_info gta01_i2c_devs[] __initdata = {
	{
		I2C_BOARD_INFO("wm8753", 0x1a)
	}, {
		I2C_BOARD_INFO("neo1973_lm4857", 0x7c) 
	}
};

static struct platform_device *gta01_devices[] __initdata = {
	&s3c_device_usb,
	&s3c_device_lcd,
	&s3c_device_wdt,
	&s3c_device_i2c,
	&s3c_device_iis,
	&s3c_device_sdi,
	&s3c_device_usbgadget,
	&s3c_device_nand,
	&s3c_device_ts,
};

static struct s3c2410_nand_set gta01_nand_sets[] = {
	[0] = {
		.name		= "neo1973-nand",
		.nr_chips	= 1,
		.flags		= S3C2410_NAND_BBT,
	},
};

static struct s3c2410_platform_nand gta01_nand_info = {
	.tacls		= 20,
	.twrph0		= 60,
	.twrph1		= 20,
	.nr_sets	= ARRAY_SIZE(gta01_nand_sets),
	.sets		= gta01_nand_sets,
};

static void gta01_mmc_set_power(unsigned char power_mode, unsigned short vdd)
{
	int bit;
	int mv = 1700; /* 1.7V for MMC_VDD_165_195 */

	printk(KERN_DEBUG "mmc_set_power(power_mode=%u, vdd=%u)\n",
	       power_mode, vdd);

	switch (system_rev) {
	case GTA01v3_SYSTEM_REV:
		switch (power_mode) {
		case MMC_POWER_OFF:
			pcf50606_onoff_set(pcf50606_global,
					   PCF50606_REGULATOR_D2REG, 0);
			break;
		case MMC_POWER_ON:
			/* translate MMC_VDD_* VDD bit to mv */
			for (bit = 8; bit != 24; bit++)
				if (vdd == (1 << bit))
					mv += 100 * (bit - 4);
			pcf50606_voltage_set(pcf50606_global,
					     PCF50606_REGULATOR_D2REG, mv);
			pcf50606_onoff_set(pcf50606_global,
					   PCF50606_REGULATOR_D2REG, 1);
			break;
		}
		break;
	case GTA01v4_SYSTEM_REV:
	case GTA01Bv2_SYSTEM_REV:
	case GTA01Bv3_SYSTEM_REV:
	case GTA01Bv4_SYSTEM_REV:
		switch (power_mode) {
		case MMC_POWER_OFF:
			neo1973_gpb_setpin(GTA01_GPIO_SDMMC_ON, 1);
			break;
		case MMC_POWER_ON:
			neo1973_gpb_setpin(GTA01_GPIO_SDMMC_ON, 0);
			break;
		}
		break;
	}
}

static struct s3c24xx_mci_pdata gta01_mmc_cfg = {
	.gpio_detect	= GTA01_GPIO_nSD_DETECT,
	.set_power	= &gta01_mmc_set_power,
	.ocr_avail	= MMC_VDD_165_195|MMC_VDD_20_21|
			  MMC_VDD_21_22|MMC_VDD_22_23|MMC_VDD_23_24|
			  MMC_VDD_24_25|MMC_VDD_25_26|MMC_VDD_26_27|
			  MMC_VDD_27_28|MMC_VDD_28_29|MMC_VDD_29_30|
			  MMC_VDD_30_31|MMC_VDD_31_32|MMC_VDD_32_33,
};

static void gta01_udc_command(enum s3c2410_udc_cmd_e cmd)
{
	printk(KERN_DEBUG "%s(%d)\n", __func__, cmd);

	switch (cmd) {
	case S3C2410_UDC_P_ENABLE:
		neo1973_gpb_setpin(GTA01_GPIO_USB_PULLUP, 1);
		break;
	case S3C2410_UDC_P_DISABLE:
		neo1973_gpb_setpin(GTA01_GPIO_USB_PULLUP, 0);
		break;
	default:
		break;
	}
}

/* use a work queue, since I2C API inherently schedules
 * and we get called in hardirq context from UDC driver */

struct vbus_draw {
	struct work_struct work;
	int ma;
};
static struct vbus_draw gta01_udc_vbus_drawer;

static void __gta01_udc_vbus_draw(struct work_struct *work)
{
	/* this is a fix to work around boot-time ordering problems if the
	 * s3c2410_udc is initialized before the pcf50606 driver has defined
	 * pcf50606_global */
	if (!pcf50606_global)
		return;

	if (gta01_udc_vbus_drawer.ma >= 500) {
		/* enable fast charge */
		printk(KERN_DEBUG "udc: enabling fast charge\n");
		pcf50606_charge_fast(pcf50606_global, 1);
	} else {
		/* disable fast charge */
		printk(KERN_DEBUG "udc: disabling fast charge\n");
		pcf50606_charge_fast(pcf50606_global, 0);
	}
}

static void gta01_udc_vbus_draw(unsigned int ma)
{
	gta01_udc_vbus_drawer.ma = ma;
	schedule_work(&gta01_udc_vbus_drawer.work);
}

static struct s3c2410_udc_mach_info gta01_udc_cfg = {
	.vbus_draw	= gta01_udc_vbus_draw,
};


/* touchscreen configuration */

static struct ts_filter_median_configuration gta01_ts_median_config = {
	.extent = 31,
	.decimation_below = 24,
	.decimation_threshold = 8 * 3,
	.decimation_above = 12,
};

static struct ts_filter_mean_configuration gta01_ts_mean_config = {
	.bits_filter_length = 5,
	.averaging_threshold = 12
};

static struct s3c2410_ts_mach_info gta01_ts_cfg = {
	.delay = 10000,
	.presc = 0xff, /* slow as we can go */
	.filter_sequence = {
		[0] = &ts_filter_median_api,
		[1] = &ts_filter_mean_api,
	},
	.filter_config = {
		[0] = &gta01_ts_median_config,
		[1] = &gta01_ts_mean_config,
	},
};


/* SPI */

static void gta01_jbt6k74_reset(int devidx, int level)
{
	/* empty place holder; gta01 does not yet use this */
	printk(KERN_DEBUG "gta01_jbt6k74_reset\n");
}

static void gta01_jbt6k74_resuming(int devidx)
{
	gta01bl_deferred_resume();
}

const struct jbt6k74_platform_data gta01_jbt6k74_pdata = {
	.reset		= gta01_jbt6k74_reset,
	.resuming	= gta01_jbt6k74_resuming,
};

static struct spi_board_info gta01_spi_board_info[] = {
	{
		.modalias	= "jbt6k74",
		.platform_data	= &gta01_jbt6k74_pdata,
		/* controller_data */
		/* irq */
		.max_speed_hz	= 10 * 1000 * 1000,
		.bus_num	= 1,
		/* chip_select */
	},
};

static void spi_gpio_cs(struct s3c2410_spigpio_info *spi, int csidx, int cs)
{
	switch (cs) {
	case BITBANG_CS_ACTIVE:
		s3c2410_gpio_setpin(S3C2410_GPG3, 0);
		break;
	case BITBANG_CS_INACTIVE:
		s3c2410_gpio_setpin(S3C2410_GPG3, 1);
		break;
	}
}

static struct s3c2410_spigpio_info spi_gpio_cfg = {
	.pin_clk	= S3C2410_GPG7,
	.pin_mosi	= S3C2410_GPG6,
	.pin_miso	= S3C2410_GPG5,
	.board_size	= ARRAY_SIZE(gta01_spi_board_info),
	.board_info	= gta01_spi_board_info,
	.chip_select	= &spi_gpio_cs,
	.num_chipselect = 2,   /*** Should be 1 or 2 for gta01? ***/
};

static struct resource s3c_spi_lcm_resource[] = {
	[0] = {
		.start = S3C2410_GPG3,
		.end   = S3C2410_GPG3,
	},
	[1] = {
		.start = S3C2410_GPG5,
		.end   = S3C2410_GPG5,
	},
	[2] = {
		.start = S3C2410_GPG6,
		.end   = S3C2410_GPG6,
	},
	[3] = {
		.start = S3C2410_GPG7,
		.end   = S3C2410_GPG7,
	},
};

struct platform_device s3c_device_spi_lcm = {
	.name		  = "spi_s3c24xx_gpio",
	.id		  = 1,
	.num_resources	  = ARRAY_SIZE(s3c_spi_lcm_resource),
	.resource	  = s3c_spi_lcm_resource,
	.dev = {
		.platform_data = &spi_gpio_cfg,
	},
};

static struct gta01bl_machinfo backlight_machinfo = {
	.default_intensity	= 1,
	.max_intensity		= 1,
	.limit_mask		= 1,
	.defer_resume_backlight	= 1,
};

static struct resource gta01_bl_resources[] = {
	[0] = {
		.start	= GTA01_GPIO_BACKLIGHT,
		.end	= GTA01_GPIO_BACKLIGHT,
	},
};

struct platform_device gta01_bl_dev = {
	.name 		= "gta01-bl",
	.num_resources	= ARRAY_SIZE(gta01_bl_resources),
	.resource	= gta01_bl_resources,
	.dev		= {
		.platform_data = &backlight_machinfo,
	},
};

static struct resource gta01_led_resources[] = {
	[0] = {
		.start	= GTA01_GPIO_VIBRATOR_ON,
		.end	= GTA01_GPIO_VIBRATOR_ON,
	},
};

struct platform_device gta01_led_dev = {
	.name		= "neo1973-vibrator",
	.num_resources	= ARRAY_SIZE(gta01_led_resources),
	.resource	= gta01_led_resources,
};

static struct resource gta01_button_resources[] = {
	[0] = {
		.start = GTA01_GPIO_AUX_KEY,
		.end   = GTA01_GPIO_AUX_KEY,
	},
	[1] = {
		.start = GTA01_GPIO_HOLD_KEY,
		.end   = GTA01_GPIO_HOLD_KEY,
	},
	[2] = {
		.start = GTA01_GPIO_JACK_INSERT,
		.end   = GTA01_GPIO_JACK_INSERT,
	},
};

struct platform_device gta01_button_dev = {
	.name		= "neo1973-button",
	.num_resources	= ARRAY_SIZE(gta01_button_resources),
	.resource	= gta01_button_resources,
};

static struct platform_device gta01_pm_gsm_dev = {
	.name		= "neo1973-pm-gsm",
};

/* USB */
static struct s3c2410_hcd_info gta01_usb_info = {
	.port[0]	= {
		.flags	= S3C_HCDFLG_USED,
	},
	.port[1]	= {
		.flags	= 0,
	},
};

static void __init gta01_map_io(void)
{
	s3c24xx_init_io(gta01_iodesc, ARRAY_SIZE(gta01_iodesc));
	s3c24xx_init_clocks(12*1000*1000);
	s3c24xx_init_uarts(gta01_uartcfgs, ARRAY_SIZE(gta01_uartcfgs));
}

static irqreturn_t gta01_modem_irq(int irq, void *param)
{
	printk(KERN_DEBUG "GSM wakeup interrupt (IRQ %d)\n", irq);
	gta_gsm_interrupts++;
	return IRQ_HANDLED;
}

static void __init gta01_machine_init(void)
{
	int rc;

	if (system_rev == GTA01v4_SYSTEM_REV ||
	    system_rev == GTA01Bv2_SYSTEM_REV ||
	    system_rev == GTA01Bv3_SYSTEM_REV ||
	    system_rev == GTA01Bv4_SYSTEM_REV) {
		gta01_udc_cfg.udc_command = gta01_udc_command;
		gta01_mmc_cfg.ocr_avail = MMC_VDD_32_33;
	}

	s3c_device_usb.dev.platform_data = &gta01_usb_info;
	s3c_device_nand.dev.platform_data = &gta01_nand_info;
	s3c_device_sdi.dev.platform_data = &gta01_mmc_cfg;

	s3c24xx_fb_set_platdata(&gta01_lcd_cfg);

	INIT_WORK(&gta01_udc_vbus_drawer.work, __gta01_udc_vbus_draw);
	s3c24xx_udc_set_platdata(&gta01_udc_cfg);
	set_s3c2410ts_info(&gta01_ts_cfg);

	/* Set LCD_RESET / XRES to high */
	s3c2410_gpio_cfgpin(S3C2410_GPC6, S3C2410_GPIO_OUTPUT);
	s3c2410_gpio_setpin(S3C2410_GPC6, 1);

	/* SPI chip select is gpio output */
	s3c2410_gpio_cfgpin(S3C2410_GPG3, S3C2410_GPIO_OUTPUT);
	s3c2410_gpio_setpin(S3C2410_GPG3, 1);
	platform_device_register(&s3c_device_spi_lcm);

	platform_device_register(&gta01_bl_dev);
	platform_device_register(&gta01_button_dev);
	platform_device_register(&gta01_pm_gsm_dev);

	switch (system_rev) {
	case GTA01v3_SYSTEM_REV:
	case GTA01v4_SYSTEM_REV:
		/* just use the default (GTA01_IRQ_PCF50606) */
		break;
	case GTA01Bv2_SYSTEM_REV:
	case GTA01Bv3_SYSTEM_REV:
		/* just use the default (GTA01_IRQ_PCF50606) */
		gta01_led_resources[0].start =
			gta01_led_resources[0].end = GTA01Bv2_GPIO_VIBRATOR_ON;
		break;
	case GTA01Bv4_SYSTEM_REV:
		gta01_pmu_resources[0].start =
			gta01_pmu_resources[0].end = GTA01Bv4_IRQ_PCF50606;
		gta01_led_resources[0].start =
			gta01_led_resources[0].end = GTA01Bv4_GPIO_VIBRATOR_ON;
		break;
	}
	mangle_pmu_pdata_by_system_rev();
	platform_device_register(&gta01_pmu_dev);
	platform_device_register(&gta01_led_dev);

	platform_add_devices(gta01_devices, ARRAY_SIZE(gta01_devices));

	i2c_register_board_info(0, gta01_i2c_devs, 
		ARRAY_SIZE(gta01_i2c_devs));

	s3c2410_pm_init();

	set_irq_type(GTA01_IRQ_MODEM, IRQ_TYPE_EDGE_RISING);
	rc = request_irq(GTA01_IRQ_MODEM, gta01_modem_irq, IRQF_DISABLED,
			 "modem", NULL);
	enable_irq_wake(GTA01_IRQ_MODEM);
	printk(KERN_DEBUG  "Enabled GSM wakeup IRQ %d (rc=%d)\n",
	       GTA01_IRQ_MODEM, rc);
}

MACHINE_START(NEO1973_GTA01, "GTA01")
	.phys_io	= S3C2410_PA_UART,
	.io_pg_offst	= (((u32)S3C24XX_VA_UART) >> 18) & 0xfffc,
	.boot_params	= S3C2410_SDRAM_PA + 0x100,
	.map_io		= gta01_map_io,
	.init_irq	= s3c24xx_init_irq,
	.init_machine	= gta01_machine_init,
	.timer		= &s3c24xx_timer,
MACHINE_END
