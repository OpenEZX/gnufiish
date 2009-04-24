/*
 * Support for features of Openmoko 3D7K
 *
 * (C) 2008 by Openmoko Inc.
 * Author: Andy Green <andy@openmoko.com>
 * All rights reserved.
 *
 * Somewhat based on the GTA01 / 02 neo1973_pm_ stuff mainly by Harald Welte
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation
 *
 */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/delay.h>
#include <linux/platform_device.h>

#include <mach/hardware.h>
#include <mach/om-3d7k.h>
#include <asm/mach-types.h>

#include <linux/regulator/consumer.h>
#include <linux/mfd/pcf50633/core.h>
#include <linux/mfd/pcf50633/gpio.h>
#include <linux/mmc/host.h>

#include <plat/sdhci.h>
#include <plat/devs.h>

#include <plat/gpio-cfg.h>

enum feature {
	OM_3D7K_GSM,		/* GSM module */
	OM_3D7K_USBHOST,	/* USB Host power generation */
	OM_3D7K_VIB,		/* Vibrator */

	OM_3D7K_FEATURE_COUNT	/* always last */
};


struct om_3d7k_feature_info {
	const char * name;
	int depower_on_suspend;
	int on;
};

static struct om_3d7k_feature_info feature_info[OM_3D7K_FEATURE_COUNT] = {
	[OM_3D7K_GSM] =	{ "gsm_power",		0, 0 },
	[OM_3D7K_USBHOST] =	{ "usbhost_power",	1, 0 },
	[OM_3D7K_VIB] =	{ "vibrator_power",	1, 0 },
};

static struct regulator *gps_regulator;



static void om_3d7k_features_pwron_set_on(enum feature feature)
{
	switch (feature) {
	case OM_3D7K_GSM:
		/* give power to GSM module */
		s3c_gpio_setpull(OM_3D7K_GPIO_N_MODEM_RESET, S3C_GPIO_PULL_NONE);
		s3c_gpio_setpull(OM_3D7K_GPIO_MODEM_ON, S3C_GPIO_PULL_NONE);
		s3c_gpio_cfgpin(OM_3D7K_GPIO_N_MODEM_RESET, S3C_GPIO_SFN(1));
		s3c_gpio_cfgpin(OM_3D7K_GPIO_MODEM_ON, S3C_GPIO_SFN(1));
		gpio_direction_output(OM_3D7K_GPIO_N_MODEM_RESET, 0);
		gpio_direction_output(OM_3D7K_GPIO_MODEM_ON, 1);
		msleep(10);
		gpio_direction_output(OM_3D7K_GPIO_MODEM_ON, 0);
		msleep(150);
		gpio_direction_output(OM_3D7K_GPIO_N_MODEM_RESET, 1);
		msleep(300);
		gpio_direction_output(OM_3D7K_GPIO_MODEM_ON, 1);
		break;
	case OM_3D7K_USBHOST:
		pcf50633_gpio_set(om_3d7k_pcf, PCF50633_GPO, 1);
		break;
	case OM_3D7K_VIB:
		gpio_direction_output(OM_3D7K_GPIO_VIBRATOR_ON, 1);
		break;
	default:
		break;
	}
}

static void om_3d7k_features_pwron_set_off(enum feature feature)
{
	switch (feature) {
	case OM_3D7K_GSM:
		/* remove power from WLAN / BT module */
		s3c_gpio_cfgpin(OM_3D7K_GPIO_MODEM_ON, S3C_GPIO_SFN(1));
		gpio_direction_output(OM_3D7K_GPIO_MODEM_ON, 0);
		msleep(1100);
		gpio_direction_output(OM_3D7K_GPIO_MODEM_ON, 1);
		break;
	case OM_3D7K_USBHOST:
		pcf50633_gpio_set(om_3d7k_pcf, PCF50633_GPO, 0);
		break;
	case OM_3D7K_VIB:
		gpio_direction_output(OM_3D7K_GPIO_VIBRATOR_ON, 0);
		break;
	default:
		break;
	}
}

static void om_3d7k_features_pwron_set(enum feature feature, int on)
{
	if ((on) && (!feature_info[feature].on))
		om_3d7k_features_pwron_set_on(feature);
	else
		if ((!on) && (feature_info[feature].on))
			om_3d7k_features_pwron_set_off(feature);
}

static ssize_t om_3d7k_feature_read(struct device *dev,
				       struct device_attribute *attr, char *buf)
{
	int on;
	int feature = 0;
	int hit = 0;

	while (!hit && feature < OM_3D7K_FEATURE_COUNT) {
		if (!strcmp(attr->attr.name, feature_info[feature].name))
			hit = 1;
		else
			feature++;
	}

	if (!hit)
		return -EINVAL;

	switch (feature) {
	case OM_3D7K_USBHOST:
		on = pcf50633_gpio_get(om_3d7k_pcf, PCF50633_GPO);
		break;
	default:
		on = feature_info[feature].on;
	}

	*buf++ = '0' + on;
	*buf++='\n';
	*buf = '\0';

	return 3;
}

static ssize_t om_3d7k_feature_write(struct device *dev,
					struct device_attribute *attr,
					const char *buf, size_t count)
{
	int on = !!simple_strtoul(buf, NULL, 10);
	int feature = 0;
	int hit = 0;

	while (!hit && feature < OM_3D7K_FEATURE_COUNT) {
		if (!strcmp(attr->attr.name, feature_info[feature].name))
			hit = 1;
		else
			feature++;
	}

	if (!hit)
		return -EINVAL;

	om_3d7k_features_pwron_set(feature, on);
	feature_info[feature].on = on;

	return count;
}


static DEVICE_ATTR(gsm_power, 0644, om_3d7k_feature_read,
							om_3d7k_feature_write);

static DEVICE_ATTR(usbhost_power, 0644, om_3d7k_feature_read,
							om_3d7k_feature_write);

static DEVICE_ATTR(vibrator_power, 0644, om_3d7k_feature_read,
							om_3d7k_feature_write);


static struct attribute *om_3d7k_features_sysfs_entries[] = {
	&dev_attr_gsm_power.attr,
	&dev_attr_usbhost_power.attr,
	&dev_attr_vibrator_power.attr,
	NULL
};


static struct attribute_group om_3d7k_features_attr_group = {
	.name	= NULL,
	.attrs	= om_3d7k_features_sysfs_entries,
};

static int __init om_3d7k_features_probe(struct platform_device *pdev)
{
	gps_regulator = regulator_get(&pdev->dev, "RF_3V");
	dev_info(&pdev->dev, "starting\n");

	return sysfs_create_group(&pdev->dev.kobj,
						 &om_3d7k_features_attr_group);
}

static int om_3d7k_features_remove(struct platform_device *pdev)
{

	regulator_put(gps_regulator);
	sysfs_remove_group(&pdev->dev.kobj, &om_3d7k_features_attr_group);

	return 0;
}


#ifdef CONFIG_PM
static int om_3d7k_features_suspend(struct platform_device *pdev,
				     pm_message_t state)
{
	int feature;

	for (feature = 0; feature < OM_3D7K_FEATURE_COUNT; feature++)
		if (feature_info[feature].depower_on_suspend)
			om_3d7k_features_pwron_set_off(feature);

	return 0;
}

static int om_3d7k_features_resume(struct platform_device *pdev)
{
	int feature;

	for (feature = 0; feature < OM_3D7K_FEATURE_COUNT; feature++)
		if (feature_info[feature].depower_on_suspend)
			if (feature_info[feature].on)
				om_3d7k_features_pwron_set_on(feature);

	return 0;
}
#else
#define om_3d7k_features_suspend	NULL
#define om_3d7k_features_resume	NULL
#endif

static struct platform_driver om_3d7k_features_driver = {
	.probe		= om_3d7k_features_probe,
	.remove		= om_3d7k_features_remove,
	.suspend	= om_3d7k_features_suspend,
	.resume		= om_3d7k_features_resume,
	.driver		= {
		.name		= "om-3d7k",
	},
};

static int __devinit om_3d7k_features_init(void)
{
	return platform_driver_register(&om_3d7k_features_driver);
}

static void om_3d7k_features_exit(void)
{
	platform_driver_unregister(&om_3d7k_features_driver);
}

module_init(om_3d7k_features_init);
module_exit(om_3d7k_features_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Andy Green <andy@openmoko.com>");
MODULE_DESCRIPTION("Openmoko OM_3D7K Feature Driver");
