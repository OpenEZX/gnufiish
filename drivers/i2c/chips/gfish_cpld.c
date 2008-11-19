/* E-TEN glofiish M800 CPLD driver
 *
 * (C) 2008 by Harald Welte <laforge@gnufiish.org>,
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

#define KEYSTATE_SIZE	12
struct gf_cpld_data {
	struct i2c_client *client;
	int irq;
	int working;
	struct work_struct work;
	struct mutex working_lock;
	struct input_dev *input_dev;
	unsigned char last_keystate[KEYSTATE_SIZE];
	unsigned char keystate_buf[KEYSTATE_SIZE];
};


static unsigned short normal_i2c[] = { 0x30, I2C_CLIENT_END };

I2C_CLIENT_INSMOD_1(gfish_cpld);

#define GFISH_KS5_AP_END	0x01
#define GFISH_KS5_AP_1		0x02
#define GFISH_KS5_AP_SEND	0x04
#define GFISH_KS5_AP_0		0x08
#define GFISH_KS5_AP_SOFT0	0x10
#define GFISH_KS5_AP_START	0x20
#define GFISH_KS5_AP_SOFT1	0x40
#define GFISH_KS5_AP_OK		0x80

static int set_led(struct gf_cpld_data *gc, int on)
{
	if (on)
		gc->keystate_buf[1] = 0x01;
	else
		gc->keystate_buf[0] = 0x01;

	return i2c_master_send(gc->client, gc->keystate_buf, 5);
}

static int get_start_thresh(struct gf_cpld_data *gc)
{
	int ret;
	u_int8_t aa = 0xaa;
	u_int8_t ks[7];

	ret = i2c_master_send(gc->client, &aa, 1);
	ret = i2c_master_recv(gc->client, &ks, sizeof(ks));
	/* FIXME: what to do with it? */
}

static int set_power(struct gf_cpld_data *gc)
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

static void process_ks5(struct gf_cpld_data *gc, u_int8_t ks5)
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

static void gf_cpld_work(struct work_struct *work)
{
	struct gf_cpld_data *gc =
			container_of(work, struct gf_cpld_data, work);
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

	dev_dbg(dev, "gf-cpld: got 0x%02x, 0x%02x, 0x%02x, "
		"0x%02x, 0x%02x, 0x%02x, 0x%02x, 0x%02x, "
		"(0x%02x, 0x%02x, 0x%02x, 0x%02x)\n",
		ks[0], ks[1], ks[2], ks[3], ks[4], ks[5], ks[6],
		ks[7], ks[8], ks[9], ks[10], ks[11]);

	process_ks5(gc, ks[5]);

	memcpy(gc->last_keystate, ks, KEYSTATE_SIZE);

	/* FIXME: do we need to starta a timer ? */

	input_sync(gc->input_dev);
out:
	gc->working = 0;
	put_device(&gc->client->dev);
	mutex_unlock(&gc->working_lock);
}

static irqreturn_t gf_cpld_irq(int irq, void *_gc)
{
	struct gf_cpld_data *gc = _gc;
	struct device *dev = &gc->client->dev;

	dev_dbg(dev,"entering(irq=%u, pcf=%p)\n", irq, _gc);

	get_device(&gc->client->dev);
	if (!schedule_work(&gc->work) && ! gc->working)
		dev_err(dev, "glofiish CPLD work already queued\n");

	return IRQ_HANDLED;
}

static int gf_cpld_probe(struct i2c_client *client,
			 const struct i2c_device_id *id)
{
	struct gf_cpld_data *gc;
	int err;

	if (!(gc = kzalloc(sizeof(*gc), GFP_KERNEL)))
		return -ENOMEM;

	/* FIXME */
	gc->irq = IRQ_EINT2;

	gc->client = client;
	client->flags |= I2C_M_IGNORE_NAK;
	mutex_init(&gc->working_lock);
	INIT_WORK(&gc->work, gf_cpld_work);

	gc->input_dev = input_allocate_device();
	if (!gc->input_dev) {
		err = -ENOMEM;
		goto exit_free;
	}

	gc->input_dev->name = "glofiish CLPD";
	gc->input_dev->phys = "FIXME";
	gc->input_dev->id.bustype = BUS_I2C;

	gc->input_dev->evbit[0] = BIT(EV_KEY);
	//set_bit(KEY_FIXME, gc->input_dev->keybit);

	err = input_register_device(gc->input_dev);
	if (err)
		goto exit_alloc_inp;

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

	err = request_irq(gc->irq, gf_cpld_irq, 0, "gf-cpld", gc);
	if (err < 0)
		goto exit_input;

	return 0;

exit_input:
	input_unregister_device(gc->input_dev);
exit_alloc_inp:
	input_free_device(gc->input_dev);
exit_free:
	kfree(gc);

	return err;
}


static int __devexit gf_cpld_remove(struct i2c_client *client)
{
	struct gf_cpld_data *gc = i2c_get_clientdata(client);

	free_irq(gc->irq, gc);
	input_unregister_device(gc->input_dev);
	kfree(gc);

	return 0;
}

struct i2c_device_id  gf_cpld_idtable[] = {
	{ "gf_cpld", 0 },
	{ }
};

MODULE_DEVICE_TABLE(i2c, gf_cpld_idtable);

static struct i2c_driver gf_cpld_driver = {
	.driver = {
		.owner	= THIS_MODULE,
		.name 	= "gf-cpld",
	},
	.id_table = gf_cpld_idtable,
	.probe = gf_cpld_probe,
	.remove = __devexit_p(gf_cpld_remove),
};

static int __init gf_cpld_init(void)
{
	int rc;

	return i2c_add_driver(&gf_cpld_driver);
}

static void gf_cpld_exit(void)
{
	i2c_del_driver(&gf_cpld_driver);
}

MODULE_DESCRIPTION("I2C driver for E-TEN glofiish M800 CPLD");
MODULE_AUTHOR("Harald Welte <laforge@gnufiish.org>");
MODULE_LICENSE("GPL");

module_init(gf_cpld_init);
module_exit(gf_cpld_exit);
