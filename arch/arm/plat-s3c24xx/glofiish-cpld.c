/*
 * driver for the E-TEN glofiish CPLD
 *
 * (C) 2008 by Harald Welte <laforge@gnufiish.org>
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
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/leds.h>
#include <linux/spinlock.h>
#include <linux/interrupt.h>
#include <linux/input.h>
#include <linux/irq.h>
#include <linux/delay.h>
#include <linux/leds.h>

#include <asm/mach-types.h>
#include <asm/io.h>

#include <mach/glofiish.h>
#include <mach/hardware.h>

#include <plat/regs-timer.h>
#include <plat/glofiish-cpld.h>

#define RESSIZE(ressource) (((ressource)->end - (ressource)->start)+1)

#define NUM_KBD_REGS	8		/* columns */
#define	KBD_REG_MASK	0x3f
#define	KBD_REG_BITS	6		/* rows */

#define SCANCODE(r,c)	(((r)<<3) + (c) + 1)
#define NR_SCANCODES	((KBD_REG_BITS<<3) + 1)


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

	struct {
		int irq;
		unsigned int gpio_scan_en;
		struct input_dev *input_dev;
		u_int16_t keycode[NR_SCANCODES];
		struct led_trigger ledt_bl;
		struct led_trigger ledt_fn;
		struct led_trigger ledt_caps;
	} kbd;
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

/***********************************************************************/
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

/***********************************************************************/
/* Keyboard support */

static u_int16_t gf_kbd_keycode[NR_SCANCODES] = {
	0,
	KEY_Q, KEY_W, KEY_E, KEY_R, KEY_T, KEY_Y, KEY_U, KEY_I,
	KEY_A, KEY_S, KEY_D, KEY_F, KEY_G, KEY_H, KEY_J, KEY_K,
	KEY_LEFTSHIFT, KEY_Z, KEY_X, KEY_C, KEY_V, KEY_B, KEY_N, KEY_M,
	KEY_LEFTALT, KEY_MENU, KEY_OK, KEY_TAB, KEY_SPACE, 0, KEY_DOT, KEY_LEFT,
	KEY_O, KEY_L, KEY_UP, KEY_DOWN, KEY_P, KEY_BACKSPACE, KEY_ENTER, KEY_RIGHT,
	KEY_VOLUMEUP, KEY_VOLUMEDOWN, KEY_PROG1, KEY_PROG2, 0, 0, 0, 0,
};

static void read_kbd_regs(struct gf_cpld *cpld, u_int16_t *regs)
{
	int i;

	for (i = 0; i < NUM_KBD_REGS; i++) {
		u_int16_t tmp = __reg_read(cpld, i*2);
		tmp = ~tmp;
		/* FIXME: this might be M800 specific */
		regs[i] = tmp & KBD_REG_MASK;
	}
}

static void parse_kbd_regs(struct gf_cpld *cpld, u_int16_t *regs)
{
	int reg, bit;
	u_int16_t keycode;
	unsigned int num_pressed = 0;

	printk(KERN_DEBUG "parse_kbd_regs: ");
	for (reg = 0; reg < NUM_KBD_REGS; reg++) {
		printk("0x%02x ", regs[reg]);
		for (bit = 0; bit < KBD_REG_BITS; bit++) {
			int pressed = 0;

			if (regs[reg] & (1 << bit)) {
				/* if this is the first register, or our bit
				 * is set first in this register */
				if (reg == 0 || !(regs[reg-1] & (1 << bit))) {
					pressed = 1;
					num_pressed++;
				}
			}

			keycode = cpld->kbd.keycode[SCANCODE(bit, reg)];

			input_report_key(cpld->kbd.input_dev, keycode, pressed);
		}
	}
	printk("\n");

	input_sync(cpld->kbd.input_dev);
}

static irqreturn_t cpld_kbd_irq(int irq, void *_cpld)
{
	struct gf_cpld *cpld = _cpld;
	u_int16_t regs[NUM_KBD_REGS];

	/* enable keyboard scanning */
	s3c2410_gpio_setpin(cpld->kbd.gpio_scan_en, 1);
	
	udelay(20);

	/* read actual keyboard scan matrix registers */
	read_kbd_regs(cpld, regs);

	/* disable keyboard scanning */
	s3c2410_gpio_setpin(cpld->kbd.gpio_scan_en, 0);

	parse_kbd_regs(cpld, regs);

	/* FIXME: we want to share with the I2C capsense driver */
	return IRQ_HANDLED;
}

/***********************************************************************/
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
	struct gf_cpld_pdata *pdata = pdev->dev.platform_data;
	struct gf_cpld *cpld;
	struct resource *res;

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

	/* Check for and initialize keyboard */
	cpld->kbd.irq = platform_get_irq(pdev, 0);
	if (!cpld->kbd.irq)
		dev_warn(&pdev->dev, "No interrupt -> No keyboard support\n");
	else {
		struct input_dev *indev;
		int rc, i;

		res = platform_get_resource_byname(pdev, 0, "kbd_scan_en");
		if (!res) {
			dev_err(&pdev->dev, "no keyboard scan enable GPIO\n");
			goto out_unmap;
		}
		cpld->kbd.gpio_scan_en = res->start;

		indev = input_allocate_device();
		if (!indev) {
			dev_err(&pdev->dev, "cannot allocate input device\n");
			goto out_unmap;
		}
		cpld->kbd.input_dev = indev;
		indev->name = "glofiish keyboard";
		indev->phys = "CPLD";
		indev->id.bustype = 0;
		indev->evbit[0] = BIT(EV_KEY) | BIT(EV_REP);
		indev->keycode = cpld->kbd.keycode;
		indev->keycodesize = sizeof(u_int16_t);
		indev->keycodemax = ARRAY_SIZE(gf_kbd_keycode);

		memcpy(cpld->kbd.keycode, gf_kbd_keycode, sizeof(cpld->kbd.keycode));
		for (i = 0; i < ARRAY_SIZE(gf_kbd_keycode); i++)
			set_bit(cpld->kbd.keycode[i], indev->keybit);
		clear_bit(0, indev->keybit);

		rc = input_register_device(indev);
		if (rc) {
			dev_err(&pdev->dev, "cannot register input device\n");
			goto out_free_indev;
		}

		cpld->kbd.ledt_bl.name = "kbd-backlight";
		rc = led_trigger_register(&cpld->kbd.ledt_bl);	
		if (rc < 0)
			goto out_unreg_indev;

		cpld->kbd.ledt_fn.name = "kbd-fn";
		rc = led_trigger_register(&cpld->kbd.ledt_fn);	
		if (rc < 0)
			goto out_unreg_bl;

		cpld->kbd.ledt_caps.name = "kbd-caps";
		rc = led_trigger_register(&cpld->kbd.ledt_caps);	
		if (rc < 0)
			goto out_unreg_fn;

		/* IRQF_TRIGGER_FALLING */
		rc = request_irq(cpld->kbd.irq, cpld_kbd_irq,
				 IRQF_SAMPLE_RANDOM | IRQF_SHARED,
				 "glofiish-cpld-kbd", cpld);
		if (rc < 0) {
			dev_err(&pdev->dev, "cannot request kbd irq\n");
			goto out_unreg_caps;
		}
	}

	/* Initialize LED function */
	if (init_leds(cpld, pdata) < 0)
		goto out_free_irq;

	return 0;

out_free_irq:
	if (cpld->kbd.irq)
		free_irq(cpld->kbd.irq, cpld);
out_unreg_caps:
	led_trigger_unregister(&cpld->kbd.ledt_caps);	
out_unreg_fn:
	led_trigger_unregister(&cpld->kbd.ledt_fn);	
out_unreg_bl:
	led_trigger_unregister(&cpld->kbd.ledt_bl);	
out_unreg_indev:
	if (cpld->kbd.input_dev) {
		input_unregister_device(cpld->kbd.input_dev);
		/* input_unregister_device free()'s the input device */
		cpld->kbd.input_dev = NULL;
	}
out_free_indev:
	if (cpld->kbd.input_dev)
		input_free_device(cpld->kbd.input_dev);
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

	if (cpld->kbd.irq) {
		free_irq(cpld->kbd.irq, cpld);

		input_unregister_device(cpld->kbd.input_dev);
		/* input_unregister_device free()'s the input device */

		led_trigger_unregister(&cpld->kbd.ledt_caps);	
		led_trigger_unregister(&cpld->kbd.ledt_fn);	
		led_trigger_unregister(&cpld->kbd.ledt_bl);	
	}

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
