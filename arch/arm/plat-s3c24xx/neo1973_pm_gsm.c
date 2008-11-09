/*
 * GSM Management code for the FIC Neo1973 GSM Phone
 *
 * (C) 2007 by OpenMoko Inc.
 * Author: Harald Welte <laforge@openmoko.org>
 * All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation
 *
 */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/platform_device.h>
#include <linux/console.h>
#include <linux/errno.h>

#include <asm/hardware.h>
#include <asm/arch/gta01.h>

struct gta01pm_priv {
	int gpio_ngsm_en;
	struct console *con;
};

static struct gta01pm_priv gta01_gsm;

static struct console *find_s3c24xx_console(void)
{
	struct console *con;

	acquire_console_sem();

	for (con = console_drivers; con; con = con->next) {
		if (!strcmp(con->name, "ttySAC"))
			break;
	}

	release_console_sem();

	return con;
}

static ssize_t gsm_read(struct device *dev, struct device_attribute *attr,
			char *buf)
{
	if (!strcmp(attr->attr.name, "power_on")) {
		if (s3c2410_gpio_getpin(GTA01_GPIO_MODEM_ON))
			goto out_1;
	} else if (!strcmp(attr->attr.name, "reset")) {
		if (s3c2410_gpio_getpin(GTA01_GPIO_MODEM_RST))
			goto out_1;
	} else if (!strcmp(attr->attr.name, "download")) {
		if (s3c2410_gpio_getpin(GTA01_GPIO_MODEM_DNLOAD))
			goto out_1;
	}

	return strlcpy(buf, "0\n", 3);
out_1:
	return strlcpy(buf, "1\n", 3);
}

static ssize_t gsm_write(struct device *dev, struct device_attribute *attr,
			 const char *buf, size_t count)
{
	unsigned long on = simple_strtoul(buf, NULL, 10);

	if (!strcmp(attr->attr.name, "power_on")) {
		if (on) {
			dev_info(dev, "powering up GSM, thus disconnecting "
				 "serial console\n");

			if (gta01_gsm.con)
				console_stop(gta01_gsm.con);

			if (gta01_gsm.gpio_ngsm_en)
				s3c2410_gpio_setpin(gta01_gsm.gpio_ngsm_en, 0);

			s3c2410_gpio_setpin(GTA01_GPIO_MODEM_ON, 1);
		} else {
			s3c2410_gpio_setpin(GTA01_GPIO_MODEM_ON, 0);

			if (gta01_gsm.gpio_ngsm_en)
				s3c2410_gpio_setpin(gta01_gsm.gpio_ngsm_en, 1);

			if (gta01_gsm.con)
				console_start(gta01_gsm.con);

			dev_info(dev, "powered down GSM, thus enabling "
				 "serial console\n");
		}
	} else if (!strcmp(attr->attr.name, "reset")) {
		s3c2410_gpio_setpin(GTA01_GPIO_MODEM_RST, on);
	} else if (!strcmp(attr->attr.name, "download")) {
		s3c2410_gpio_setpin(GTA01_GPIO_MODEM_DNLOAD, on);
	}

	return count;
}

static DEVICE_ATTR(power_on, 0644, gsm_read, gsm_write);
static DEVICE_ATTR(reset, 0644, gsm_read, gsm_write);
static DEVICE_ATTR(download, 0644, gsm_read, gsm_write);

#ifdef CONFIG_PM
static int gta01_gsm_suspend(struct platform_device *pdev, pm_message_t state)
{
	/* GPIO state is saved/restored by S3C2410 core GPIO driver, so we
	 * don't need to do anything here */

	return 0;
}

static int gta01_gsm_resume(struct platform_device *pdev)
{
	/* GPIO state is saved/restored by S3C2410 core GPIO driver, so we
	 * don't need to do anything here */

	/* Make sure that the kernel console on the serial port is still
	 * disabled. FIXME: resume ordering race with serial driver! */
	if (s3c2410_gpio_getpin(GTA01_GPIO_MODEM_ON) && gta01_gsm.con)
		console_stop(gta01_gsm.con);

	return 0;
}
#else
#define gta01_gsm_suspend	NULL
#define gta01_gsm_resume	NULL
#endif

static struct attribute *gta01_gsm_sysfs_entries[] = {
	&dev_attr_power_on.attr,
	&dev_attr_reset.attr,
	NULL,
	NULL
};

static struct attribute_group gta01_gsm_attr_group = {
	.name	= NULL,
	.attrs	= gta01_gsm_sysfs_entries,
};

static int __init gta01_gsm_probe(struct platform_device *pdev)
{
	switch (system_rev) {
	case GTA01v3_SYSTEM_REV:
		gta01_gsm.gpio_ngsm_en = GTA01v3_GPIO_nGSM_EN;
		break;
	case GTA01v4_SYSTEM_REV:
		gta01_gsm.gpio_ngsm_en = 0;
		break;
	case GTA01Bv2_SYSTEM_REV:
	case GTA01Bv3_SYSTEM_REV:
	case GTA01Bv4_SYSTEM_REV:
		gta01_gsm.gpio_ngsm_en = GTA01Bv2_GPIO_nGSM_EN;
		s3c2410_gpio_setpin(GTA01v3_GPIO_nGSM_EN, 0);
		break;
	default:
		dev_warn(&pdev->dev, "Unknown GTA01 Revision 0x%x, "
			 "some PM features not available!!!\n",
			 system_rev);
		break;
	}

	switch (system_rev) {
	case GTA01v4_SYSTEM_REV:
	case GTA01Bv2_SYSTEM_REV:
		gta01_gsm_sysfs_entries[ARRAY_SIZE(gta01_gsm_sysfs_entries)-2] =
							&dev_attr_download.attr;
		break;
	default:
		break;
	}

	gta01_gsm.con = find_s3c24xx_console();
	if (!gta01_gsm.con)
		dev_warn(&pdev->dev, "cannot find S3C24xx console driver\n");

	return sysfs_create_group(&pdev->dev.kobj, &gta01_gsm_attr_group);
}

static int gta01_gsm_remove(struct platform_device *pdev)
{
	sysfs_remove_group(&pdev->dev.kobj, &gta01_gsm_attr_group);

	return 0;
}

static struct platform_driver gta01_gsm_driver = {
	.probe		= gta01_gsm_probe,
	.remove		= gta01_gsm_remove,
	.suspend	= gta01_gsm_suspend,
	.resume		= gta01_gsm_resume,
	.driver		= {
		.name		= "neo1973-pm-gsm",
	},
};

static int __devinit gta01_gsm_init(void)
{
	return platform_driver_register(&gta01_gsm_driver);
}

static void gta01_gsm_exit(void)
{
	platform_driver_unregister(&gta01_gsm_driver);
}

module_init(gta01_gsm_init);
module_exit(gta01_gsm_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Harald Welte <laforge@openmoko.org>");
MODULE_DESCRIPTION("FIC Neo1973 GSM Power Management");
