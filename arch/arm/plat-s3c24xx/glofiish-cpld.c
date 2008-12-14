/*
 * driver for the E-TEN glofiish CPLD
 *
 * (C) 2008 by Harald Welte <laforge@gnufiish.org>
 * All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 */

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/leds.h>
#include <linux/spinlock.h>
#include <mach/hardware.h>
#include <asm/mach-types.h>
#include <asm/io.h>
#include <mach/glofiish.h>
#include <plat/regs-timer.h>
#include <plat/glofiish-cpld.h>

#define RESSIZE(ressource) (((ressource)->end - (ressource)->start)+1)

/* runtime data structures */
struct gf_cpld_led {
	struct led_classdev cdev;
	struct gf_cpld_led_cfg *cfg;
};

struct gf_cpld {
	struct platform_device *pdev;
	struct gf_cpld_pdata *pdata;
	spinlock_t lock;
	struct resource *mem;
	void *iobase;

	int num_leds;
	struct gf_cpld_led *leds;
};

/* CPLD register access functions */
static inline u_int16_t __reg_read(struct gf_cpld *gc, unsigned int reg)
{
	return readw(gc->iobase + reg);
}

static inline void __reg_write(struct gf_cpld *gc, unsigned int reg,
				u_int16_t val)
{
	writew(val, gc->iobase + reg);
}

static void __reg_set_bit_mask(struct gf_cpld *gc, unsigned int reg,
				u_int16_t mask, u_int16_t val)
{
	u_int16_t tmp;

	tmp = __reg_read(gc, reg);
	tmp &= ~mask;
	tmp |= val;
	__reg_write(gc, reg, tmp);
}

static void reg_set_bit_mask(struct gf_cpld *gc, unsigned int reg,
			     u_int16_t mask, u_int16_t val)
{
	unsigned long flags;

	spin_lock_irqsave(&gc->lock, flags);
	__reg_set_bit_mask(gc, reg, mask, val);
	spin_unlock_irqrestore(&gc->lock, flags);
}

static void __reg_clear_bits(struct gf_cpld *gc, unsigned int reg,
			     u_int16_t val)
{
	u_int16_t tmp;

	tmp = __reg_read(gc, reg);
	tmp &= ~val;
	__reg_write(gc, reg, tmp);
}

static void reg_clear_bits(struct gf_cpld *gc, unsigned int reg,
			   u_int16_t val)
{
	unsigned long flags;

	spin_lock_irqsave(&gc->lock, flags);
	__reg_clear_bits(gc, reg, val);
	spin_unlock_irqrestore(&gc->lock, flags);
}

/* LED specific stuff */

static inline struct gf_cpld_led *to_cpld_led(struct led_classdev *led_cdev)
{
	return container_of(led_cdev, struct gf_cpld_led, cdev);
}

static void gf_cpld_led_set(struct led_classdev *led_cdev,
			    enum led_brightness value)
{
	struct device *parent = led_cdev->dev->parent;
	struct gf_cpld *gc = dev_get_drvdata(parent);
	struct gf_cpld_led *led = to_cpld_led(led_cdev);
	struct gf_cpld_led_cfg *lcfg = led->cfg;
	u_int16_t bit = 1 << lcfg->bit_offset;

	if (value)
		reg_set_bit_mask(gc, lcfg->reg_offset, bit, bit);
	else
		reg_clear_bits(gc, lcfg->reg_offset, bit);
}

static int init_leds(struct gf_cpld *gc, struct gf_cpld_pdata *pdata)
{
	struct device *dev = &gc->pdev->dev;
	int i, rc;

	gc->leds = kzalloc(pdata->led_num * sizeof(struct gf_cpld_led),
			   GFP_KERNEL);
	if (!gc->leds)
		return -ENOMEM;

	gc->num_leds = pdata->led_num;

	for (i = 0; i < gc->num_leds; i++) {
		struct gf_cpld_led *led = &gc->leds[i];
		struct gf_cpld_led_cfg *lcfg = &pdata->led_cfgs[i];

		led->cfg = lcfg;
		led->cdev.name = lcfg->name;
		led->cdev.brightness_set = gf_cpld_led_set;

		rc = led_classdev_register(dev, &led->cdev);
		if (rc < 0) {
			dev_err(dev, "unable to register led #%u: %d\n",
				i, rc);
			while (i--)
				led_classdev_unregister(&gc->leds[i].cdev);
			kfree(gc->leds);
			gc->num_leds = 0;
			return rc;
		}
	}

	return 0;
}

static void finish_leds(struct gf_cpld *gc)
{
	int i;

	for (i = 0; i < gc->num_leds; i++)
		led_classdev_unregister(&gc->leds[i].cdev);
	
	kfree(gc->leds);
	gc->num_leds = 0;
}

/* Device model interaction */

#ifdef CONFIG_PM
static int gf_cpld_suspend(struct platform_device *pdev, pm_message_t state)
{
	struct gf_cpld *cpld = platform_get_drvdata(pdev);
	int i;

	for (i = 0; i < cpld->num_leds; i++)
		led_classdev_suspend(&cpld->leds[i].cdev);

	return 0;
}

static int gf_cpld_resume(struct platform_device *pdev)
{
	struct gf_cpld *cpld = platform_get_drvdata(pdev);
	int i;

	for (i = 0; i < cpld->num_leds; i++)
		led_classdev_resume(&cpld->leds[i].cdev);

	return 0;
}
#endif

static int __init gf_cpld_probe(struct platform_device *pdev)
{
	struct gf_cpld *cpld;
	struct resource *res;
	struct gf_cpld_pdata *pdata = pdev->dev.platform_data;

	if (!machine_is_m800()) {
		dev_err(&pdev->dev, "unknown machine\n");
		return -EIO;
	}

	if (!pdata) {
		dev_err(&pdev->dev, "no platfomrm data\n");
		return -EINVAL;
	}

	cpld = kzalloc(sizeof(struct gf_cpld), GFP_KERNEL);
	if (!cpld)
		return -ENOMEM;
	platform_set_drvdata(pdev, cpld);

	cpld->pdata = pdata;
	cpld->pdev = pdev;
	spin_lock_init(&cpld->lock);

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res)
		goto out_free;

	cpld->mem = request_mem_region(res->start, RESSIZE(res), pdev->name);
	if (!cpld->mem)
		goto out_free;

	cpld->iobase = ioremap(cpld->mem->start, RESSIZE(cpld->mem));
	if (!cpld->iobase)
		goto out_freemem;

	if (init_leds(cpld, pdata) < 0)
		goto out_unmap;

	return 0;

out_unmap:
	iounmap(cpld->iobase);
out_freemem:
	release_mem_region(cpld->mem->start, RESSIZE(cpld->mem));
out_free:
	platform_set_drvdata(pdev, NULL);
	kfree(cpld);

	return -1;
}

static int gf_cpld_remove(struct platform_device *pdev)
{
	struct gf_cpld *cpld = platform_get_drvdata(pdev);

	finish_leds(cpld);
	iounmap(cpld->iobase);
	release_mem_region(cpld->mem->start, RESSIZE(cpld->mem));
	platform_set_drvdata(pdev, NULL);
	kfree(cpld);

	return 0;
}

static struct platform_driver gf_cpld_driver = {
	.probe		= gf_cpld_probe,
	.remove		= gf_cpld_remove,
#ifdef CONFIG_PM
	.suspend	= gf_cpld_suspend,
	.resume		= gf_cpld_resume,
#endif
	.driver		= {
		.name		= "gfish-cpld",
	},
};

static int __init gf_cpld_init(void)
{
	return platform_driver_register(&gf_cpld_driver);
}

static void __exit gf_cpld_exit(void)
{
	platform_driver_unregister(&gf_cpld_driver);
}

module_init(gf_cpld_init);
module_exit(gf_cpld_exit);

MODULE_AUTHOR("Harald Welte <laforge@gnufiish.org>");
MODULE_DESCRIPTION("E-TEN glofiish CPLD driver");
MODULE_LICENSE("GPL");
