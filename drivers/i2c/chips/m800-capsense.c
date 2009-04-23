/* E-TEN glofiish M800 CapSense driver
 *
 * (C) 2008-2009 by Harald Welte <laforge@gnufiish.org>,
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
 * TODO:
 *	* use a mutex to prevent race conditions
 *	* use proper keymap that can be changed by userspace
 *	* introduce sysfs api for capsense initial valuse
 *	* support for X800, DX900 and other devices
 */

#define DEBUG

#include <linux/module.h>
#include <linux/init.h>
#include <linux/i2c.h>
#include <linux/types.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/workqueue.h>
#include <linux/input.h>
#include <linux/sched.h>
#include <linux/delay.h>
#include <linux/leds.h>

#define KEYSTATE_SIZE	12
struct m800_caps_data {
	struct i2c_client *client;
	int irq;
	int working;
	struct work_struct work;
	struct mutex working_lock;
	struct input_dev *input_dev;
	struct led_classdev led_cdev;
	unsigned char last_keystate[KEYSTATE_SIZE];
	unsigned char keystate_buf[KEYSTATE_SIZE];
};


static unsigned short normal_i2c[] = { 0x30, I2C_CLIENT_END };

I2C_CLIENT_INSMOD_1(m800_caps);

#define GFISH_KS5_AP_END	0x01
#define GFISH_KS5_AP_1		0x02
#define GFISH_KS5_AP_SEND	0x04
#define GFISH_KS5_AP_0		0x08
#define GFISH_KS5_AP_SOFT0	0x10
#define GFISH_KS5_AP_START	0x20
#define GFISH_KS5_AP_SOFT1	0x40
#define GFISH_KS5_AP_OK		0x80

static int send_ee(struct m800_caps_data *gc)
{
	u_int8_t ee = 0xee;

	return i2c_master_send(gc->client, &ee, 1);
}

static void gf_caps_led_set(struct led_classdev *led_cdev,
			    enum led_brightness value)
{
	struct device *parent = led_cdev->dev->parent;
	struct m800_caps_data *gc = dev_get_drvdata(parent);

	if (value)
		gc->keystate_buf[1] = 0x01;
	else
		gc->keystate_buf[1] = 0x00;

	i2c_master_send(gc->client, gc->keystate_buf, 5);
}

static int get_start_thresh(struct m800_caps_data *gc)
{
	int ret;
	u_int8_t aa = 0xaa;
	u_int8_t ks[7];

	ret = i2c_master_send(gc->client, &aa, 1);
	msleep(300);
	ret = i2c_master_recv(gc->client, ks, sizeof(ks));
	/* FIXME: what to do with it? */
}

static u_int8_t get_key_thresh(struct m800_caps_data *gc, u_int8_t key)
{
	int ret;
	u_int8_t ks[7];

	if (key > 7)
		return -EINVAL;

	key |= 0x60;
	ret = i2c_master_send(gc->client, &key, 1);
	msleep(300);
	ret = i2c_master_recv(gc->client, ks, sizeof(ks));

	/* FIXME: only after reading all of them */
	send_ee(gc);

	return ks[0];
}

static int set_key_thresh(struct m800_caps_data *gc, u_int8_t key,
			  u_int8_t val)
{
	int ret;

	if (key > 7)
		return -EINVAL;

	key |= 0x70;
	ret = i2c_master_send(gc->client, &key, 1);

	msleep(200);

	ret = i2c_master_send(gc->client, &val, 1);

	return send_ee(gc);
}

static int set_power(struct m800_caps_data *gc)
{
	int ret;
	struct device *dev = &gc->client->dev;

	gc->keystate_buf[1] = 0x00;
	ret = i2c_master_send(gc->client, gc->keystate_buf, 5);
	if (ret != 1)
		dev_err(dev, "pwr: short write\n");

	msleep(500);

	gc->keystate_buf[1] = 0x03;
	ret = i2c_master_send(gc->client, gc->keystate_buf, 5);
	if (ret != 1)
		dev_err(dev, "pwr: short write 2\n");

	return ret;
}

/* keystate 5 reflects the capsense buttons */
static void process_ks5(struct m800_caps_data *gc, u_int8_t ks5)
{
	int pressed = 1;

	if (ks5 & GFISH_KS5_AP_END)
		input_report_key(gc->input_dev, KEY_END, pressed);

	if (ks5 & GFISH_KS5_AP_1)
		input_report_key(gc->input_dev, KEY_FN_F1, pressed);

	if (ks5 & GFISH_KS5_AP_SEND)
		input_report_key(gc->input_dev, KEY_SEND, pressed);

	if (ks5 & GFISH_KS5_AP_0)
		input_report_key(gc->input_dev, KEY_FN_F2, pressed);

	if (ks5 & GFISH_KS5_AP_SOFT0)
		input_report_key(gc->input_dev, BTN_A, pressed);

	if (ks5 & GFISH_KS5_AP_START)
		input_report_key(gc->input_dev, BTN_START, pressed);

	if (ks5 & GFISH_KS5_AP_SOFT1)
		input_report_key(gc->input_dev, BTN_B, pressed);

	if (ks5 & GFISH_KS5_AP_OK)
		input_report_key(gc->input_dev, KEY_OK, pressed);
}

/* keystate 6 reflects the joystick */
static void process_ks6(struct m800_caps_data *gc, u_int8_t ks6)
{
	/* FIXME: find out which value reflects which key */
}

static void m800_caps_work(struct work_struct *work)
{
	struct m800_caps_data *gc =
			container_of(work, struct m800_caps_data, work);
	struct device *dev = &gc->client->dev;
	u_int8_t ks[KEYSTATE_SIZE];
	int ret; 

	mutex_lock(&gc->working_lock);
	gc->working = 1;

	ks[0] = 0x00;
	ret = i2c_master_send(gc->client, ks, 1);
	if (ret != 1)
		dev_err(dev, "short write2\n");

	ret = i2c_master_recv(gc->client, ks, 7);
	if (ret != 7) {
		dev_err(dev, "wanted %d bytes, got %d\n",
			KEYSTATE_SIZE, ret);
		goto out;
	}

	dev_dbg(dev, "gf-caps: got 0x%02x, 0x%02x, 0x%02x, "
		"0x%02x, 0x%02x, 0x%02x, 0x%02x, 0x%02x, "
		"(0x%02x, 0x%02x, 0x%02x, 0x%02x)\n",
		ks[0], ks[1], ks[2], ks[3], ks[4], ks[5], ks[6],
		ks[7], ks[8], ks[9], ks[10], ks[11]);

	process_ks5(gc, ks[5]);
	process_ks6(gc, ks[6]);

	memcpy(gc->last_keystate, ks, KEYSTATE_SIZE);

	/* FIXME: do we need to starta a timer ? */

	input_sync(gc->input_dev);
out:
	gc->working = 0;
	put_device(&gc->client->dev);
	mutex_unlock(&gc->working_lock);
}

static irqreturn_t m800_caps_irq(int irq, void *_gc)
{
	struct m800_caps_data *gc = _gc;
	struct device *dev = &gc->client->dev;

	dev_dbg(dev,"entering(irq=%u, pcf=%p)\n", irq, _gc);

	get_device(&gc->client->dev);
	if (!schedule_work(&gc->work) && ! gc->working)
		dev_err(dev, "glofiish CPLD work already queued\n");

	return IRQ_HANDLED;
}

static int m800_caps_probe(struct i2c_client *client,
			 const struct i2c_device_id *id)
{
	struct m800_caps_data *gc;
	int err;

	if (!(gc = kzalloc(sizeof(*gc), GFP_KERNEL)))
		return -ENOMEM;

	/* FIXME */
	gc->irq = IRQ_EINT2;

	gc->client = client;
	client->flags |= I2C_M_IGNORE_NAK;
	mutex_init(&gc->working_lock);
	INIT_WORK(&gc->work, m800_caps_work);

	gc->input_dev = input_allocate_device();
	if (!gc->input_dev) {
		err = -ENOMEM;
		goto exit_free;
	}

	gc->input_dev->name = "glofiish capsense";
	gc->input_dev->phys = "I2C";
	gc->input_dev->id.bustype = BUS_I2C;

	gc->input_dev->evbit[0] = BIT(EV_KEY);
	//set_bit(KEY_FIXME, gc->input_dev->keybit);

	err = input_register_device(gc->input_dev);
	if (err) {
		dev_err(&client->dev, "unable to register input device\n");
		goto exit_alloc_inp;
	}

	i2c_set_clientdata(client, gc);


{
	u_int8_t ks[7];
	int i;

	memset(ks, 0, sizeof(ks));
	for (i = 0; i < 10; i++) {
		err = i2c_master_recv(gc->client, ks, 7);
		if (ks[2] != 0xff && ks[3] != 0xff)
			break;
		msleep(500);
	}
	memcpy(gc->keystate_buf, ks, 5);
}
	//set_power(gc);

	err = request_irq(gc->irq, m800_caps_irq, 0, "gf-caps", gc);
	if (err < 0) {
		dev_err(&client->dev, "unable to register irq: %d\n", err);
		goto exit_input;
	}

	gc->led_cdev.name = "glofiish:white:buttons";
	gc->led_cdev.brightness_set = gf_caps_led_set;
	err = led_classdev_register(&client->dev, &gc->led_cdev);
	if (err < 0) {
		dev_err(&client->dev, "unable to register led: %d\n", err);
		goto exit_irq;
	}

	return 0;

exit_irq:
	free_irq(gc->irq, gc);
exit_input:
	i2c_set_clientdata(client, NULL);
	input_unregister_device(gc->input_dev);
exit_alloc_inp:
	input_free_device(gc->input_dev);
exit_free:
	kfree(gc);

	return err;
}


static int __devexit m800_caps_remove(struct i2c_client *client)
{
	struct m800_caps_data *gc = i2c_get_clientdata(client);

	led_classdev_unregister(&gc->led_cdev);
	free_irq(gc->irq, gc);
	i2c_set_clientdata(client, NULL);
	input_unregister_device(gc->input_dev);
	kfree(gc);

	return 0;
}

struct i2c_device_id  m800_caps_idtable[] = {
	{ "m800_capsense", 0 },
	{ }
};

MODULE_DEVICE_TABLE(i2c, m800_caps_idtable);

static struct i2c_driver m800_caps_driver = {
	.driver = {
		.owner	= THIS_MODULE,
		.name 	= "m800-capsense",
	},
	.id_table = m800_caps_idtable,
	.probe = m800_caps_probe,
	.remove = __devexit_p(m800_caps_remove),
};

static int __init m800_caps_init(void)
{
	int rc;

	return i2c_add_driver(&m800_caps_driver);
}

static void m800_caps_exit(void)
{
	i2c_del_driver(&m800_caps_driver);
}

MODULE_DESCRIPTION("I2C driver for E-TEN glofiish M800 capsense buttons");
MODULE_AUTHOR("Harald Welte <laforge@gnufiish.org>");
MODULE_LICENSE("GPL");

module_init(m800_caps_init);
module_exit(m800_caps_exit);
