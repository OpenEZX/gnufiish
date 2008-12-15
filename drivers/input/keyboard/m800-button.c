/*
 * Keyboard driver for E-TEN glofiish M800 GSM phone buttons
 *
 * (C) 2008 by Harald Welte <laforge@gnumonks.org>
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
#include <linux/leds.h>

#include <asm/gpio.h>
#include <asm/mach-types.h>

struct m800btn {
	struct input_dev *input;
	unsigned int suspended;
	struct work_struct work;
	int work_in_progress;
	int hp_irq_count_in_work;
	int hp_irq_count;
	int jack_irq;
	struct led_trigger slide_ledtrig;
};

static irqreturn_t m800btn_power_irq(int irq, void *dev_id)
{
	struct m800btn *m800btn_data = dev_id;
	int key_pressed = !gpio_get_value(irq_to_gpio(irq));

	input_report_key(m800btn_data->input, KEY_POWER, key_pressed);
	input_sync(m800btn_data->input);

	return IRQ_HANDLED;
}

static irqreturn_t m800btn_cam_irq(int irq, void *dev_id)
{
	struct m800btn *m800btn_data = dev_id;

	int key_pressed = !gpio_get_value(irq_to_gpio(irq));
	input_report_key(m800btn_data->input, KEY_CAMERA, key_pressed);
	input_sync(m800btn_data->input);

	return IRQ_HANDLED;
}

static irqreturn_t m800btn_rec_irq(int irq, void *dev_id)
{
	struct m800btn *m800btn_data = dev_id;

	int key_pressed = !gpio_get_value(irq_to_gpio(irq));
	input_report_key(m800btn_data->input, KEY_RECORD, key_pressed);
	input_sync(m800btn_data->input);

	return IRQ_HANDLED;
}

static irqreturn_t m800btn_slide_irq(int irq, void *dev_id)
{
	struct m800btn *m800btn_data = dev_id;

	int key_pressed = gpio_get_value(irq_to_gpio(irq));
	input_report_key(m800btn_data->input, SW_LID, key_pressed);
	input_sync(m800btn_data->input);

	led_trigger_event(&m800btn_data->slide_ledtrig, key_pressed);

	return IRQ_HANDLED;
}

#if 0
static void m800btn_debounce_jack(struct work_struct *work)
{
	struct m800btn *kbd = container_of(work, struct m800btn, work);
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


static irqreturn_t m800btn_headphone_irq(int irq, void *dev_id)
{
	struct m800btn *m800btn_data = dev_id;

	/*
	 * this interrupt is prone to bouncing and userspace doesn't like
	 * to have to deal with that kind of thing.  So we do not accept
	 * that a jack interrupt is equal to a jack event.  Instead we fire
	 * some work on the first interrupt, and it hangs about in 100ms units
	 * until no more interrupts come.  Then it accepts the state it finds
	 * for jack insert and reports it once
	 */

	m800btn_data->hp_irq_count++;
	/*
	 * the first interrupt we see for a while, we fire the work item
	 * and record the interrupt count when we did that.  If more interrupts
	 * come in the meanwhile, we can tell by the difference in that
	 * stored count and hp_irq_count which increments every interrupt
	 */
	if (!m800btn_data->work_in_progress) {
		m800btn_data->jack_irq = irq;
		m800btn_data->hp_irq_count_in_work =
						m800btn_data->hp_irq_count;
		if (!schedule_work(&m800btn_data->work))
			printk(KERN_ERR
				"Unable to schedule headphone debounce\n");
		else
			m800btn_data->work_in_progress = 1;
	}

	return IRQ_HANDLED;
}
#endif

#ifdef CONFIG_PM
static int m800btn_suspend(struct platform_device *dev, pm_message_t state)
{
	struct m800btn *m800btn = platform_get_drvdata(dev);

	m800btn->suspended = 1;

	return 0;
}

static int m800btn_resume(struct platform_device *dev)
{
	struct m800btn *m800btn = platform_get_drvdata(dev);

	m800btn->suspended = 0;

	return 0;
}
#else
#define m800btn_suspend	NULL
#define m800btn_resume	NULL
#endif

static int m800btn_probe(struct platform_device *pdev)
{
	struct m800btn *m800btn;
	struct input_dev *input_dev;
	int rc, irq_power, irq_cam, irq_rec, irq_slide;

	m800btn = kzalloc(sizeof(struct m800btn), GFP_KERNEL);
	input_dev = input_allocate_device();
	if (!m800btn || !input_dev) {
		kfree(m800btn);
		input_free_device(input_dev);
		return -ENOMEM;
	}

	if (pdev->resource[0].flags != 0)
		return -EINVAL;

	irq_power = gpio_to_irq(pdev->resource[0].start);
	if (irq_power < 0)
		return -EINVAL;

	irq_cam = gpio_to_irq(pdev->resource[1].start);
	if (irq_cam < 0)
		return -EINVAL;

	irq_rec = gpio_to_irq(pdev->resource[2].start);
	if (irq_rec < 0)
		return -EINVAL;

	irq_slide = gpio_to_irq(pdev->resource[3].start);
	if (irq_slide < 0)
		return -EINVAL;

	platform_set_drvdata(pdev, m800btn);

	m800btn->input = input_dev;

	//INIT_WORK(&m800btn->work, m800btn_debounce_jack);

	input_dev->name = "E-TEN glofiish M800 Buttons";
	input_dev->phys = "m800btn/input0";
	input_dev->id.bustype = BUS_HOST;
	input_dev->id.vendor = 0x0001;
	input_dev->id.product = 0x0001;
	input_dev->id.version = 0x0100;
	input_dev->dev.parent = &pdev->dev;

	input_dev->evbit[0] = BIT(EV_KEY) | BIT(EV_SW);
	set_bit(SW_HEADPHONE_INSERT, input_dev->swbit);
	set_bit(SW_LID, input_dev->swbit);
	set_bit(KEY_POWER, input_dev->keybit);
	set_bit(KEY_CAMERA, input_dev->keybit);
	set_bit(KEY_RECORD, input_dev->keybit);

	m800btn->slide_ledtrig.name = "kbd-slide";
	rc = led_trigger_register(&m800btn->slide_ledtrig);
	if (rc)
		goto out_ledtrig;

	rc = input_register_device(m800btn->input);
	if (rc)
		goto out_register;

	if (request_irq(irq_power, m800btn_power_irq, IRQF_DISABLED |
			IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING,
			"M800 Power button", m800btn)) {
		dev_err(&pdev->dev, "Can't get IRQ %u\n", irq_power);
		goto out_power;
	}

	enable_irq_wake(irq_power);

	if (request_irq(irq_cam, m800btn_cam_irq, IRQF_DISABLED |
			IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING,
			"M800 Camera button", m800btn)) {
		dev_err(&pdev->dev, "Can't get IRQ %u\n", irq_cam);
		goto out_cam;
	}

	if (request_irq(irq_rec, m800btn_rec_irq, IRQF_DISABLED |
			IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING,
			"M800 Record button", m800btn)) {
		dev_err(&pdev->dev, "Can't get IRQ %u\n", irq_rec);
		goto out_record;
	}

	if (request_irq(irq_slide, m800btn_slide_irq, IRQF_DISABLED |
			IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING,
			"M800 Slide", m800btn)) {
		dev_err(&pdev->dev, "Can't get IRQ %u\n", irq_slide);
		goto out_slide;
	}
	enable_irq_wake(irq_slide);

	return 0;

out_slide:
	free_irq(irq_rec, m800btn);
out_record:
	free_irq(irq_cam, m800btn);
out_cam:
	free_irq(irq_power, m800btn);
out_power:
	input_unregister_device(m800btn->input);
out_register:
	input_free_device(m800btn->input);
out_ledtrig:
	led_trigger_unregister(&m800btn->slide_ledtrig);
	platform_set_drvdata(pdev, NULL);
	kfree(m800btn);

	return -ENODEV;
}

static int m800btn_remove(struct platform_device *pdev)
{
	struct m800btn *m800btn = platform_get_drvdata(pdev);

	free_irq(gpio_to_irq(pdev->resource[2].start), m800btn);
	free_irq(gpio_to_irq(pdev->resource[1].start), m800btn);
	free_irq(gpio_to_irq(pdev->resource[0].start), m800btn);

	input_unregister_device(m800btn->input);
	/* input_unregister_device() free's the data */
	led_trigger_unregister(&m800btn->slide_ledtrig);
	platform_set_drvdata(pdev, NULL);
	kfree(m800btn);

	return 0;
}

static struct platform_driver m800btn_driver = {
	.probe		= m800btn_probe,
	.remove		= m800btn_remove,
	.suspend	= m800btn_suspend,
	.resume		= m800btn_resume,
	.driver		= {
		.name	= "m800-button",
	},
};

static int __devinit m800btn_init(void)
{
	return platform_driver_register(&m800btn_driver);
}

static void __exit m800btn_exit(void)
{
	platform_driver_unregister(&m800btn_driver);
}

module_init(m800btn_init);
module_exit(m800btn_exit);

MODULE_AUTHOR("Harald Welte <laforge@gnumonks.org>");
MODULE_DESCRIPTION("E-TEN glofiish M800 GPIO buttons input driver");
MODULE_LICENSE("GPL");
