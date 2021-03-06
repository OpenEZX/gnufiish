/*
 * Keyboard driver for Armzone QT2410
 *
 * (C) 2006 by Openmoko, Inc.
 * Author: Harald Welte <laforge@openmoko.org>
 * All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 */

#include <linux/delay.h>
#include <linux/platform_device.h>
#include <linux/init.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/jiffies.h>
#include <linux/module.h>
#include <linux/slab.h>

#include <mach/hardware.h>
#include <mach/gta01.h>

struct gta01kbd {
	struct input_dev *input;
	unsigned int suspended;
	unsigned long suspend_jiffies;
};

static irqreturn_t gta01kbd_interrupt(int irq, void *dev_id)
{
	struct gta01kbd *gta01kbd_data = dev_id;

	/* FIXME: use GPIO from platform_dev resources */
	if (s3c2410_gpio_getpin(S3C2410_GPF0))
		input_report_key(gta01kbd_data->input, KEY_PHONE, 1);
	else
		input_report_key(gta01kbd_data->input, KEY_PHONE, 0);

	input_sync(gta01kbd_data->input);

	return IRQ_HANDLED;
}


#ifdef CONFIG_PM
static int gta01kbd_suspend(struct platform_device *dev, pm_message_t state)
{
	struct gta01kbd *gta01kbd = platform_get_drvdata(dev);

	gta01kbd->suspended = 1;

	return 0;
}

static int gta01kbd_resume(struct platform_device *dev)
{
	struct gta01kbd *gta01kbd = platform_get_drvdata(dev);

	gta01kbd->suspended = 0;

	return 0;
}
#else
#define gta01kbd_suspend	NULL
#define gta01kbd_resume		NULL
#endif

static int gta01kbd_probe(struct platform_device *pdev)
{
	struct gta01kbd *gta01kbd;
	struct input_dev *input_dev;
	int irq_911;
	int rc = 0;

	gta01kbd = kzalloc(sizeof(struct gta01kbd), GFP_KERNEL);
	if (!gta01kbd) {
		rc = -ENOMEM;
		goto bail;
	}
	input_dev = input_allocate_device();
	if (!gta01kbd || !input_dev) {
		rc = -ENOMEM;
		goto bail_free;
	}

	if (pdev->resource[0].flags != 0) {\
		rc = -EINVAL;
		goto bail_free_dev;
	}

	irq_911 = s3c2410_gpio_getirq(pdev->resource[0].start);
	if (irq_911 < 0) {
		rc = -EINVAL;
		goto bail_free_dev;
	}

	platform_set_drvdata(pdev, gta01kbd);

	gta01kbd->input = input_dev;

#if 0
	spin_lock_init(&gta01kbd->lock);
	/* Init Keyboard rescan timer */
	init_timer(&corgikbd->timer);
	corgikbd->timer.function = corgikbd_timer_callback;
	corgikbd->timer.data = (unsigned long) corgikbd;

	/* Init Hinge Timer */
	init_timer(&corgikbd->htimer);
	corgikbd->htimer.function = corgikbd_hinge_timer;
	corgikbd->htimer.data = (unsigned long) corgikbd;

	corgikbd->suspend_jiffies=jiffies;

	memcpy(corgikbd->keycode, corgikbd_keycode, sizeof(corgikbd->keycode));
#endif

	input_dev->name = "QT2410 Buttons";
	input_dev->phys = "qt2410kbd/input0";
	input_dev->id.bustype = BUS_HOST;
	input_dev->id.vendor = 0x0001;
	input_dev->id.product = 0x0001;
	input_dev->id.version = 0x0100;

	input_dev->evbit[0] = BIT(EV_KEY);
#if 0
	input_dev->keycode = gta01kbd->keycode;
	input_dev->keycodesize = sizeof(unsigned char);
	input_dev->keycodemax = ARRAY_SIZE(corgikbd_keycode);

	for (i = 0; i < ARRAY_SIZE(corgikbd_keycode); i++)
		set_bit(corgikbd->keycode[i], input_dev->keybit);
	clear_bit(0, input_dev->keybit);
	set_bit(SW_LID, input_dev->swbit);
	set_bit(SW_TABLET_MODE, input_dev->swbit);
	set_bit(SW_HEADPHONE_INSERT, input_dev->swbit);
#endif

	rc = input_register_device(gta01kbd->input);
	if (rc)
		goto bail_free_dev;

	s3c2410_gpio_cfgpin(S3C2410_GPF0, S3C2410_GPF0_EINT0);
	if (request_irq(irq_911, gta01kbd_interrupt,
			IRQF_DISABLED | IRQF_TRIGGER_RISING |
			IRQF_TRIGGER_FALLING, "qt2410kbd_eint0", gta01kbd))
		printk(KERN_WARNING "gta01kbd: Can't get IRQ\n");
	enable_irq_wake(irq_911);

	/* FIXME: headphone insert */

#if 0
	mod_timer(&corgikbd->htimer, jiffies + msecs_to_jiffies(HINGE_SCAN_INTERVAL));

	/* Setup sense interrupts - RisingEdge Detect, sense lines as inputs */
	for (i = 0; i < CORGI_KEY_SENSE_NUM; i++) {
		pxa_gpio_mode(CORGI_GPIO_KEY_SENSE(i) | GPIO_IN);
		if (request_irq(CORGI_IRQ_GPIO_KEY_SENSE(i), corgikbd_interrupt,
				SA_INTERRUPT | SA_TRIGGER_RISING,
				"corgikbd", corgikbd))
			printk(KERN_WARNING "corgikbd: Can't get IRQ: %d!\n", i);
	}

	/* Set Strobe lines as outputs - set high */
	for (i = 0; i < CORGI_KEY_STROBE_NUM; i++)
		pxa_gpio_mode(CORGI_GPIO_KEY_STROBE(i) | GPIO_OUT | GPIO_DFLT_HIGH);

	/* Setup the headphone jack as an input */
	pxa_gpio_mode(CORGI_GPIO_AK_INT | GPIO_IN);
#endif

	return 0;

bail_free_dev:
	input_free_device(input_dev);
bail_free:
	kfree(gta01kbd);
bail:
	return rc;
}

static int gta01kbd_remove(struct platform_device *pdev)
{
	struct gta01kbd *gta01kbd = platform_get_drvdata(pdev);

	free_irq(s3c2410_gpio_getirq(pdev->resource[0].start), gta01kbd);
#if 0
	int i;

	for (i = 0; i < CORGI_KEY_SENSE_NUM; i++)
		free_irq(CORGI_IRQ_GPIO_KEY_SENSE(i), corgikbd);

	del_timer_sync(&corgikbd->htimer);
	del_timer_sync(&corgikbd->timer);
#endif
	input_unregister_device(gta01kbd->input);

	kfree(gta01kbd);

	return 0;
}

static struct platform_driver gta01kbd_driver = {
	.probe		= gta01kbd_probe,
	.remove		= gta01kbd_remove,
	.suspend	= gta01kbd_suspend,
	.resume		= gta01kbd_resume,
	.driver		= {
		.name	= "qt2410-button",
	},
};

static int __devinit gta01kbd_init(void)
{
	return platform_driver_register(&gta01kbd_driver);
}

static void __exit gta01kbd_exit(void)
{
	platform_driver_unregister(&gta01kbd_driver);
}

module_init(gta01kbd_init);
module_exit(gta01kbd_exit);

MODULE_AUTHOR("Harald Welte <laforge@openmoko.org>");
MODULE_DESCRIPTION("Armzone QT2410 Buttons Driver");
MODULE_LICENSE("GPL");
