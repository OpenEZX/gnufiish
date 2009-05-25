/*
 * FM radio pm code for the glofiish devices
 *
 * (C) 2009 Simon Busch
 * Author: Simon Busch <morphis@gravedo.de>
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
#include <linux/rfkill.h>
#include <linux/err.h>

#include <mach/hardware.h>
#include <asm/mach-types.h>

#ifdef CONFIG_MACH_M800
#include <mach/glofiish.h>
#endif

#ifdef CONFIG_PM
static int glofiish_fm_suspend(struct platform_device *pdev, pm_message_t state)
{
    return 0;
}

static int glofiish_fm_resume(struct platform_device *pdev)
{
    return 0;
}
#else
#define glofiish_fm_suspend NULL
#define glofiish_fn_resume NULL
#endif

static int fm_pwron_get(void)
{
    if (machine_is_m800())
        return !!s3c2410_gpio_getpin(M800_GPIO_FM_nRST);

    return -1;
}

static void fm_pwron_set(int on)
{
    if (machine_is_m800()) 
    {
        if (on) {
            /*
             * Typical power up sequence for the si4700 looks like this
             * - set nRST pin to high will bring the chip out of reset
             * - driving nSEN high on SDA low while rising edge of nRST will select 2-wire mode
            */

            /*
             * currently the following code is only replaying everything
             * the knight program does to intialize the fm radio chip.
             * We currently don't know which one is the nSEN and the nRST pin
             * guess is, that GPB8 is nSEN
            */

            printk(KERN_INFO "glofiish_pm_fm: powering up ...\n");

            /* configure GPB8 as nRST pin */
            s3c2410_gpio_cfgpin(M800_GPIO_FM_nRST, S3C2410_GPIO_OUTPUT);
            s3c2410_gpio_setpin(M800_GPIO_FM_nRST, 0);

            /* GPE15 & GPE14 to output and set them to low */
            s3c2410_gpio_cfgpin(S3C2410_GPE15, S3C2410_GPIO_OUTPUT);
            s3c2410_gpio_cfgpin(S3C2410_GPE14, S3C2410_GPIO_OUTPUT);
            s3c2410_gpio_setpin(S3C2410_GPE14, 0);
            s3c2410_gpio_setpin(S3C2410_GPE15, 0);

            /* Set GPH10 to CLKOUT0 */
            s3c2410_gpio_cfgpin(S3C2410_GPH10, S3C2410_GPH10_CLKOUT1);

            /* Enable the device by setting nRST to high */
            s3c2410_gpio_setpin(M800_GPIO_FM_nRST, 1);

            /* Put SCL and SDA to high */
            s3c2410_gpio_setpin(S3C2410_GPE14, 1);
            s3c2410_gpio_setpin(S3C2410_GPE15, 1);

            /* restore initial configuration */
            s3c2410_gpio_cfgpin(S3C2410_GPE14, S3C2410_GPE14_IICSCL);
            s3c2410_gpio_cfgpin(S3C2410_GPE15, S3C2410_GPE15_IICSDA);

            printk(KERN_INFO "glofiish_pm_fm: powerup done\n");

            return;
        }
        else 
        {
            printk(KERN_INFO "glofiish_pm_fm: powering down ...\n");

            /*
             * Typical power down sequence for the si4700 looks like this
             * - put nRST to low will reset the whole chip and put it in power down mode
             */
            s3c2410_gpio_cfgpin(M800_GPIO_FM_nRST, S3C2410_GPIO_OUTPUT);
            s3c2410_gpio_setpin(M800_GPIO_FM_nRST, 0);

            printk(KERN_INFO "glofiish_pm_fm: powerdown done\n");

        }
    }
}

static ssize_t power_fm_read(struct device *dev,
                             struct device_attribute *attr,
                             char *buf)
{
    int ret = 0;

    if(!strcmp(attr->attr.name, "power_on") ||
       !strcmp(attr->attr.name, "pwron")) 
    {
        ret = fm_pwron_get();
    }

    if(ret)
        return strlcpy(buf, "1\n", 3);
    else
        return strlcpy(buf, "0\n", 3);
}

static ssize_t power_fm_write(struct device *dev,
                              struct device_attribute *attr,
                              const char *buf,
                              size_t count)
{
    unsigned long on = simple_strtoul(buf, NULL, 10);

    if(!strcmp(attr->attr.name, "power_on") ||
       !strcmp(attr->attr.name, "pwron")) 
    {
        fm_pwron_set(on);
    }

    return count;
}

static DEVICE_ATTR(power_on, 0644, power_fm_read, power_fm_write);
static DEVICE_ATTR(pwron, 0644, power_fm_read, power_fm_write);

static struct attribute *glofiish_fm_sysfs_entries[] = {
    &dev_attr_power_on.attr,
    &dev_attr_pwron.attr,
    NULL
};

static struct attribute_group glofiish_fm_attr_group = {
    .name = NULL,
    .attrs = glofiish_fm_sysfs_entries,
};


static int __init glofiish_fm_probe(struct platform_device *pdev)
{
    printk(KERN_INFO "glofiish_fm_probe");
    return sysfs_create_group(&pdev->dev.kobj, &glofiish_fm_attr_group);
}

static int glofiish_fm_remove(struct platform_device *pdev)
{
    printk(KERN_INFO "glofiish_fm_remove");
    sysfs_remove_group(&pdev->dev.kobj, &glofiish_fm_attr_group);

    return 0;
}

static struct platform_driver glofiish_fm_driver = 
{
    .probe = glofiish_fm_probe,
    .remove = glofiish_fm_remove,
    .suspend = glofiish_fm_suspend,
    .resume = glofiish_fm_resume,
    .driver = {
        .name = "glofiish-pm-fm",
    },
};

static __devinit glofiish_fm_init(void)
{
    printk(KERN_INFO "glofiish_fm_init\n");
    return platform_driver_register(&glofiish_fm_driver);
}

static void glofiish_fm_exit(void)
{
    platform_driver_unregister(&glofiish_fm_driver);
}

module_init(glofiish_fm_init);
module_exit(glofiish_fm_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Simon Busch <morphis@gravedo.de>");
MODULE_DESCRIPTION("Glofiish FM radio Power Management");
