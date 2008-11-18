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

struct gf_cpld_data {
	struct i2c_client *client;
	int irq;
	int working;
	struct work_struct work;
	struct mutex working_lock;
	struct input_dev *input_dev;
};

static unsigned short normal_i2c[] = { 0x30, I2C_CLIENT_END };

I2C_CLIENT_INSMOD_1(gfish_cpld);

static void gf_cpld_work(struct work_struct *work)
{
	struct gf_cpld_data *gc =
			container_of(work, struct gf_cpld_data, work);
	struct device *dev = &gc->client->dev;
	u_int8_t values[7];
	int ret; 

	mutex_lock(&gc->working_lock);
	gc->working = 1;

	/* FIXME */
	ret = i2c_smbus_read_i2c_block_data(gc->client, 0x00,
					    sizeof(values), values);
	if (ret != sizeof(values)) {
		dev_err(dev, "gf-cpld: wanted %d bytes, got %d\n",
			sizeof(values), ret);
	} else
		dev_dbg(dev, "gf-cpld: got 0x%02x, 0x%02x, 0x%02x, "
			"0x%02x, 0x%02x, 0x%02x, 0x%02x\n",
			values[0], values[1], values[2], values[3],
			values[4], values[5], values[6]);

	gc->working = 0;
	input_sync(gc->input_dev);
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
	mutex_init(&gc->working_lock);
	INIT_WORK(&gc->work, gf_cpld_work);

	gc->input_dev = input_allocate_device();
	if (!gc->input_dev)
		goto exit_free;

	gc->input_dev->name = "glofiish CLPD";
	gc->input_dev->phys = "FIXME";
	gc->input_dev->id.bustype = BUS_I2C;

	gc->input_dev->evbit[0] = BIT(EV_KEY);
	//set_bit(KEY_FIXME, gc->input_dev->keybit);

	err = input_register_device(gc->input_dev);
	if (err)
		goto exit_alloc_inp;

	i2c_set_clientdata(client, gc);


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
