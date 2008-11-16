/*
 * Keyboard driver for FIC Neo1973 GSM phone
 *
 * (C) 2006-2007 by Openmoko, Inc.
 * Author: Harald Welte <laforge@openmoko.org>
 * All rights reserved.
 *
 * inspired by corkgbd.c by Richard Purdie
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
#include <linux/workqueue.h>

#include <mach/gpio.h>
#include <asm/mach-types.h>

struct neo1973kbd {
	struct input_dev *input;
	struct device *cdev;
	unsigned int suspended;
	struct work_struct work;
	int work_in_progress;
	int hp_irq_count_in_work;
	int hp_irq_count;
	int jack_irq;
};

static struct class *neo1973kbd_switch_class;

#define HEADSET_SWITCH "headset"

static irqreturn_t neo1973kbd_aux_irq(int irq, void *dev_id)
{
	struct neo1973kbd *neo1973kbd_data = dev_id;
	int key_pressed = !gpio_get_value(irq_to_gpio(irq));

	/* GTA02 has inverted sense level compared to GTA01 */
	if (machine_is_neo1973_gta02())
		key_pressed = !key_pressed;
	input_report_key(neo1973kbd_data->input, KEY_PHONE, key_pressed);
	input_sync(neo1973kbd_data->input);

	return IRQ_HANDLED;
}

static irqreturn_t neo1973kbd_hold_irq(int irq, void *dev_id)
{
	struct neo1973kbd *neo1973kbd_data = dev_id;

	int key_pressed = gpio_get_value(irq_to_gpio(irq));
	input_report_key(neo1973kbd_data->input, KEY_PAUSE, key_pressed);
	input_sync(neo1973kbd_data->input);

	return IRQ_HANDLED;
}

static void neo1973kbd_jack_event(struct device *dev, int num)
{
	char name_string[] = "SWITCH_NAME=" HEADSET_SWITCH;
	char event_string[32];
	char state_string[32];
	char *envp[] = { name_string, state_string, event_string, NULL };

	if (num) {
		sprintf(state_string, "SWITCH_STATE=1");
		sprintf(event_string, "EVENT=insert");
	}
	else {
		sprintf(state_string, "SWITCH_STATE=0");
		sprintf(event_string, "EVENT=remove");
	}

	kobject_uevent_env(&dev->kobj, KOBJ_CHANGE, envp);
}


static void neo1973kbd_debounce_jack(struct work_struct *work)
{
	struct neo1973kbd *kbd = container_of(work, struct neo1973kbd, work);
	unsigned long flags;
	int loop = 0;

	do {
		/*
		 * we wait out any multiple interrupt
		 * stuttering in 100ms lumps
		 */
		do {
			kbd->hp_irq_count_in_work = kbd->hp_irq_count;
			msleep(100);
		} while (kbd->hp_irq_count != kbd->hp_irq_count_in_work);
		/*
		 * no new interrupts on jack for 100ms...
		 * ok we will report it
		 */
		input_report_switch(kbd->input, SW_HEADPHONE_INSERT,
				    gpio_get_value(irq_to_gpio(kbd->jack_irq)));
		input_sync(kbd->input);
		neo1973kbd_jack_event(kbd->cdev,
				      gpio_get_value(irq_to_gpio(kbd->jack_irq)));
		/*
		 * we go around the outer loop again if we detect that more
		 * interrupts came while we are servicing here.  But we have
		 * to sequence it carefully with interrupts off
		 */
		local_save_flags(flags);
		/* no interrupts during this work means we can exit the work */
		loop = !!(kbd->hp_irq_count != kbd->hp_irq_count_in_work);
		if (!loop)
			kbd->work_in_progress = 0;
		local_irq_restore(flags);
		/*
		 * interrupt that comes here will either queue a new work action
		 * since work_in_progress is cleared now, or be dealt with
		 * when we loop.
		 */
	} while (loop);
}


static irqreturn_t neo1973kbd_headphone_irq(int irq, void *dev_id)
{
	struct neo1973kbd *neo1973kbd_data = dev_id;

	/*
	 * this interrupt is prone to bouncing and userspace doesn't like
	 * to have to deal with that kind of thing.  So we do not accept
	 * that a jack interrupt is equal to a jack event.  Instead we fire
	 * some work on the first interrupt, and it hangs about in 100ms units
	 * until no more interrupts come.  Then it accepts the state it finds
	 * for jack insert and reports it once
	 */

	neo1973kbd_data->hp_irq_count++;
	/*
	 * the first interrupt we see for a while, we fire the work item
	 * and record the interrupt count when we did that.  If more interrupts
	 * come in the meanwhile, we can tell by the difference in that
	 * stored count and hp_irq_count which increments every interrupt
	 */
	if (!neo1973kbd_data->work_in_progress) {
		neo1973kbd_data->jack_irq = irq;
		neo1973kbd_data->hp_irq_count_in_work =
						neo1973kbd_data->hp_irq_count;
		if (!schedule_work(&neo1973kbd_data->work))
			printk(KERN_ERR
				"Unable to schedule headphone debounce\n");
		else
			neo1973kbd_data->work_in_progress = 1;
	}

	return IRQ_HANDLED;
}

#ifdef CONFIG_PM
static int neo1973kbd_suspend(struct platform_device *dev, pm_message_t state)
{
	struct neo1973kbd *neo1973kbd = platform_get_drvdata(dev);

	neo1973kbd->suspended = 1;

	return 0;
}

static int neo1973kbd_resume(struct platform_device *dev)
{
	struct neo1973kbd *neo1973kbd = platform_get_drvdata(dev);

	neo1973kbd->suspended = 0;

	return 0;
}
#else
#define neo1973kbd_suspend	NULL
#define neo1973kbd_resume	NULL
#endif

static ssize_t neo1973kbd_switch_name_show(struct device *dev,
			struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%s\n", "neo1973 Headset Jack");
}

static ssize_t neo1973kbd_switch_state_show(struct device *dev,
			struct device_attribute *attr, char *buf)
{
	struct neo1973kbd *kbd = dev_get_drvdata(dev);
	return sprintf(buf, "%d\n",
			gpio_get_value(irq_to_gpio(kbd->jack_irq)) ? 1 : 0);
}

static DEVICE_ATTR(name, S_IRUGO , neo1973kbd_switch_name_show, NULL);
static DEVICE_ATTR(state, S_IRUGO , neo1973kbd_switch_state_show, NULL);

static int neo1973kbd_probe(struct platform_device *pdev)
{
	struct neo1973kbd *neo1973kbd;
	struct input_dev *input_dev;
	int rc, irq_aux, irq_hold, irq_jack;

	neo1973kbd = kzalloc(sizeof(struct neo1973kbd), GFP_KERNEL);
	input_dev = input_allocate_device();
	if (!neo1973kbd || !input_dev) {
		kfree(neo1973kbd);
		input_free_device(input_dev);
		return -ENOMEM;
	}

	if (pdev->resource[0].flags != 0)
		return -EINVAL;

	irq_aux = gpio_to_irq(pdev->resource[0].start);
	if (irq_aux < 0)
		return -EINVAL;

	irq_hold = gpio_to_irq(pdev->resource[1].start);
	if (irq_hold < 0)
		return -EINVAL;

	irq_jack = gpio_to_irq(pdev->resource[2].start);
	if (irq_jack < 0)
		return -EINVAL;

	platform_set_drvdata(pdev, neo1973kbd);

	neo1973kbd->input = input_dev;

	INIT_WORK(&neo1973kbd->work, neo1973kbd_debounce_jack);

	input_dev->name = "Neo1973 Buttons";
	input_dev->phys = "neo1973kbd/input0";
	input_dev->id.bustype = BUS_HOST;
	input_dev->id.vendor = 0x0001;
	input_dev->id.product = 0x0001;
	input_dev->id.version = 0x0100;
	input_dev->dev.parent = &pdev->dev;

	input_dev->evbit[0] = BIT(EV_KEY) | BIT(EV_SW);
	set_bit(SW_HEADPHONE_INSERT, input_dev->swbit);
	set_bit(KEY_PHONE, input_dev->keybit);
	set_bit(KEY_PAUSE, input_dev->keybit);

	rc = input_register_device(neo1973kbd->input);
	if (rc)
		goto out_register;

	neo1973kbd->cdev = device_create_drvdata(neo1973kbd_switch_class,
					&pdev->dev, 0, neo1973kbd, HEADSET_SWITCH);
	if(unlikely(IS_ERR(neo1973kbd->cdev))) {
		rc = PTR_ERR(neo1973kbd->cdev);
		goto out_device_create;
	}

	rc = device_create_file(neo1973kbd->cdev, &dev_attr_name);
	if(rc)
		goto out_device_create_file;

	rc = device_create_file(neo1973kbd->cdev, &dev_attr_state);
	if(rc)
		goto out_device_create_file;

	if (request_irq(irq_aux, neo1973kbd_aux_irq, IRQF_DISABLED |
			IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING,
			"Neo1973 AUX button", neo1973kbd)) {
		dev_err(&pdev->dev, "Can't get IRQ %u\n", irq_aux);
		goto out_aux;
	}

	/*
	 * GTA01 revisions before Bv4 can't be resumed by the PMU, so we use
	 * resume by AUX.
	 */
	if (machine_is_neo1973_gta01())
		enable_irq_wake(irq_aux);

	if (request_irq(irq_hold, neo1973kbd_hold_irq, IRQF_DISABLED |
			IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING,
			"Neo1973 HOLD button", neo1973kbd)) {
		dev_err(&pdev->dev, "Can't get IRQ %u\n", irq_hold);
		goto out_hold;
	}

	if (request_irq(irq_jack, neo1973kbd_headphone_irq, IRQF_DISABLED |
			IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING,
			"Neo1973 Headphone Jack", neo1973kbd)) {
		dev_err(&pdev->dev, "Can't get IRQ %u\n", irq_jack);
		goto out_jack;
	}
	enable_irq_wake(irq_jack);

	return 0;

out_jack:
	free_irq(irq_hold, neo1973kbd);
out_hold:
	free_irq(irq_aux, neo1973kbd);
out_aux:
out_device_create_file:
	device_unregister(neo1973kbd->cdev);
out_device_create:
	input_unregister_device(neo1973kbd->input);
out_register:
	input_free_device(neo1973kbd->input);
	platform_set_drvdata(pdev, NULL);
	kfree(neo1973kbd);

	return -ENODEV;
}

static int neo1973kbd_remove(struct platform_device *pdev)
{
	struct neo1973kbd *neo1973kbd = platform_get_drvdata(pdev);

	free_irq(gpio_to_irq(pdev->resource[2].start), neo1973kbd);
	free_irq(gpio_to_irq(pdev->resource[1].start), neo1973kbd);
	free_irq(gpio_to_irq(pdev->resource[0].start), neo1973kbd);

	device_unregister(neo1973kbd->cdev);
	input_unregister_device(neo1973kbd->input);
	input_free_device(neo1973kbd->input);
	platform_set_drvdata(pdev, NULL);
	kfree(neo1973kbd);

	return 0;
}

static struct platform_driver neo1973kbd_driver = {
	.probe		= neo1973kbd_probe,
	.remove		= neo1973kbd_remove,
	.suspend	= neo1973kbd_suspend,
	.resume		= neo1973kbd_resume,
	.driver		= {
		.name	= "neo1973-button",
	},
};

static int __devinit neo1973kbd_init(void)
{
	neo1973kbd_switch_class = class_create(THIS_MODULE, "switch");
	if (IS_ERR(neo1973kbd_switch_class))
		return PTR_ERR(neo1973kbd_switch_class);
	return platform_driver_register(&neo1973kbd_driver);
}

static void __exit neo1973kbd_exit(void)
{
	platform_driver_unregister(&neo1973kbd_driver);
	class_destroy(neo1973kbd_switch_class);
}

module_init(neo1973kbd_init);
module_exit(neo1973kbd_exit);

MODULE_AUTHOR("Harald Welte <laforge@openmoko.org>");
MODULE_DESCRIPTION("FIC Neo1973 buttons input driver");
MODULE_LICENSE("GPL");
