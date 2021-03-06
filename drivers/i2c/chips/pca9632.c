/*
 *  Philips/NXP PCA9632 low power LED driver.
 *  Copyright (C) 2008 Matt Hsu <matt_hsu@openmoko.org>
 *
 *  low_level implementation are based on pcf50606 driver
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; version 2 of the License.
 *
 *  TODO:
 *  - attach ledclass??
 *  - add platform data
 *
 */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/i2c.h>
#include <linux/platform_device.h>

#include "pca9632.h"

/* Addresses to scan */
static unsigned short normal_i2c[] = { 0x62, I2C_CLIENT_END };

/* Insmod parameters */
I2C_CLIENT_INSMOD_1(pca9632);

enum pca9632_pwr_state {
	PCA9632_NORMAL,
	PCA9632_SLEEP,
};

enum pca9632_led_output {
	PCA9632_OFF,
	PCA9632_ON,
	PCA9632_CTRL_BY_PWM,
	PCA9632_CTRL_BY_PWM_GRPPWM,
};

static const char *led_output_name[] = {
	[PCA9632_OFF]  			= "off",
	[PCA9632_ON]			= "fully-on",
	[PCA9632_CTRL_BY_PWM] 		= "ctrl-by-pwm",
	[PCA9632_CTRL_BY_PWM_GRPPWM] 	= "ctrl-by-pwm-grppwm",
};

struct pca9632_data {
	struct i2c_client client;
	struct mutex lock;
};

static struct i2c_driver pca9632_driver;
static struct platform_device *pca9632_pdev;

static int pca9632_attach_adapter(struct i2c_adapter *adapter);
static int pca9632_detach_client(struct i2c_client *client);

static int __reg_write(struct pca9632_data *pca, u_int8_t reg, u_int8_t val)
{
	return i2c_smbus_write_byte_data(&pca->client, reg, val);
}

static int reg_write(struct pca9632_data *pca, u_int8_t reg, u_int8_t val)
{
	int ret;

	mutex_lock(&pca->lock);
	ret = __reg_write(pca, reg, val);
	mutex_unlock(&pca->lock);

	return ret;
}

static int32_t __reg_read(struct pca9632_data *pca, u_int8_t reg)
{
	int32_t ret;

	ret = i2c_smbus_read_byte_data(&pca->client, reg);

	return ret;
}

static u_int8_t reg_read(struct pca9632_data *pca, u_int8_t reg)
{
	int32_t ret;

	mutex_lock(&pca->lock);
	ret = __reg_read(pca, reg);
	mutex_unlock(&pca->lock);

	return ret & 0xff;
}

static int reg_set_bit_mask(struct pca9632_data *pca,
			    u_int8_t reg, u_int8_t mask, u_int8_t val)
{
	int ret;
	u_int8_t tmp;

	val &= mask;

	mutex_lock(&pca->lock);

	tmp = __reg_read(pca, reg);
	tmp &= ~mask;
	tmp |= val;
	ret = __reg_write(pca, reg, tmp);

	mutex_unlock(&pca->lock);

	return ret;
}

static inline int calc_dc(uint8_t idc)
{
	return (idc * 100) / 256;
}

/*
 * Software reset
 */
static int software_rst(struct i2c_adapter *adapter)
{
	u8 buf[] = { 0xa5, 0x5a };

	struct i2c_msg msg[] = {
		{
			.addr = 0x3,
			.flags = 0,
			.buf = &buf,
			.len = sizeof(buf)
		}
	};

	return i2c_transfer(adapter, msg, 1);
}

/*
 * Group dmblnk control
 */
static void config_group_dmblnk(struct pca9632_data *pca, int group_dmblnk_mode)
{
	reg_set_bit_mask(pca, PCA9632_REG_MODE2, 0x20,
			group_dmblnk_mode << PCA9632_DMBLNK_SHIFT);
}

static int get_group_dmblnk(struct pca9632_data *pca)
{
	return reg_read(pca, PCA9632_REG_MODE2) >> PCA9632_DMBLNK_SHIFT;
}

static ssize_t show_group_dmblnk(struct device *dev, struct device_attribute
					*attr, char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct pca9632_data *pca = i2c_get_clientdata(client);

	if (get_group_dmblnk(pca))
		return sprintf(buf, "blinking\n");
	else
		return sprintf(buf, "dimming\n");
}

static ssize_t set_group_dmblnk(struct device *dev, struct device_attribute
					*attr, const char *buf, size_t count)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct pca9632_data *pca = i2c_get_clientdata(client);
	unsigned int mode = simple_strtoul(buf, NULL, 10);

	if (mode)
		dev_info(&pca->client.dev, "blinking\n");
	else
		dev_info(&pca->client.dev, "dimming\n");

	config_group_dmblnk(pca, mode);

	return count;
}

static DEVICE_ATTR(group_dmblnk, S_IRUGO | S_IWUSR, show_group_dmblnk,
							set_group_dmblnk);

static int reg_id_by_name(const char *name)
{
	int reg_id = -1;

	if (!strncmp(name, "led0", 4))
		reg_id = PCA9632_REG_PWM0;
	else if (!strncmp(name, "led1", 4))
		reg_id = PCA9632_REG_PWM1;
	else if (!strncmp(name, "led2", 4))
		reg_id = PCA9632_REG_PWM2;
	else if (!strncmp(name, "led3", 4))
		reg_id = PCA9632_REG_PWM3;

	return reg_id;
}

static int get_led_output(struct pca9632_data *pca, int ldrx)
{
	u_int8_t led_state;

	ldrx = ldrx - 2;
	led_state = reg_read(pca, PCA9632_REG_LEDOUT);
	led_state = (led_state >> (2 * ldrx)) & 0x03;

	return led_state;
}

static void config_led_output(struct pca9632_data *pca, int ldrx,
					enum pca9632_led_output led_output)
{
	u_int8_t mask;
	int tmp;

	ldrx = ldrx - 2;
	mask = 0x03 << (2 * ldrx);
	tmp = reg_set_bit_mask(pca, PCA9632_REG_LEDOUT,
					mask, led_output << (2 * ldrx));
}

/*
 * Individual brightness control
 */
static ssize_t show_brightness(struct device *dev, struct device_attribute
					*attr, char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct pca9632_data *pca = i2c_get_clientdata(client);
	int ldrx;

	ldrx = reg_id_by_name(attr->attr.name);

	switch (get_led_output(pca, ldrx)) {

	case PCA9632_OFF:
	case PCA9632_ON:
		return sprintf(buf, "%s",
			led_output_name[get_led_output(pca, ldrx)]);

	case PCA9632_CTRL_BY_PWM:
		return sprintf(buf, "%d%% \n", calc_dc(reg_read(pca, ldrx)));

	case PCA9632_CTRL_BY_PWM_GRPPWM:
		/* check group dmblnk */
		if (get_group_dmblnk(pca))
			return sprintf(buf, "%d%% \n",
				calc_dc(reg_read(pca, ldrx)));
		return sprintf(buf, "%d%% \n",
					 calc_dc((reg_read(pca, ldrx) & 0xfc)));
	default:
		break;
	}

	return sprintf(buf, "invalid argument\n");
}

static ssize_t set_brightness(struct device *dev, struct device_attribute *attr,
			const char *buf, size_t count)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct pca9632_data *pca = i2c_get_clientdata(client);
	unsigned int pwm = simple_strtoul(buf, NULL, 10);
	int ldrx;

	ldrx = reg_id_by_name(attr->attr.name);
	reg_set_bit_mask(pca, ldrx, 0xff, pwm);

	return count;
}

static
DEVICE_ATTR(led0_pwm, S_IRUGO | S_IWUSR, show_brightness, set_brightness);
static
DEVICE_ATTR(led1_pwm, S_IRUGO | S_IWUSR, show_brightness, set_brightness);
static
DEVICE_ATTR(led2_pwm, S_IRUGO | S_IWUSR, show_brightness, set_brightness);
static
DEVICE_ATTR(led3_pwm, S_IRUGO | S_IWUSR, show_brightness, set_brightness);

/*
 * Group frequency control
 */
static ssize_t show_group_freq(struct device *dev, struct device_attribute
				*attr, char *buf)
{
	uint32_t period;
	struct i2c_client *client = to_i2c_client(dev);
	struct pca9632_data *pca = i2c_get_clientdata(client);

	period = ((reg_read(pca, PCA9632_REG_GRPFREQ) + 1) * 1000) / 24;

	return sprintf(buf, "%d ms\n", period);
}

static ssize_t set_group_freq(struct device *dev, struct device_attribute *attr,
			const char *buf, size_t count)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct pca9632_data *pca = i2c_get_clientdata(client);

	unsigned int freq = simple_strtoul(buf, NULL, 10);
	reg_write(pca, PCA9632_REG_GRPFREQ, freq);
	return count;
}

static
DEVICE_ATTR(group_freq, S_IRUGO | S_IWUSR, show_group_freq, set_group_freq);

/*
 * Group duty cycle tonrol*
 */
static ssize_t show_group_dc(struct device *dev, struct device_attribute *attr,
			     char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct pca9632_data *pca = i2c_get_clientdata(client);

	if (get_group_dmblnk(pca)) {

		if (reg_read(pca, PCA9632_REG_GRPFREQ) <= 0x03)
			return sprintf(buf, "%d%% \n",
			     calc_dc(reg_read(pca, PCA9632_REG_GRPPWM) & 0xfc));

		return sprintf(buf, "%d%% \n", calc_dc(reg_read(pca,
							  PCA9632_REG_GRPPWM)));
	}

	return sprintf(buf, "%d%% \n", calc_dc(reg_read(pca,
						   PCA9632_REG_GRPPWM) & 0xf0));
}

static ssize_t set_group_dc(struct device *dev, struct device_attribute *attr,
			const char *buf, size_t count)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct pca9632_data *pca = i2c_get_clientdata(client);

	unsigned int dc = simple_strtoul(buf, NULL, 10);

	reg_set_bit_mask(pca, PCA9632_REG_GRPPWM, 0xff, dc);

	return count;
}

static DEVICE_ATTR(group_dc, S_IRUGO | S_IWUSR, show_group_dc, set_group_dc);

/*
 * LED driver output
 */
static ssize_t show_led_output(struct device *dev, struct device_attribute
					*attr, char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct pca9632_data *pca = i2c_get_clientdata(client);
	int ldrx;

	ldrx = reg_id_by_name(attr->attr.name);

	return sprintf(buf, "%s \n",
				    led_output_name[get_led_output(pca, ldrx)]);

}
static ssize_t set_led_output(struct device *dev, struct device_attribute *attr,
			const char *buf, size_t count)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct pca9632_data *pca = i2c_get_clientdata(client);
	enum pca9632_led_output led_output;
	int ldrx;

	led_output = simple_strtoul(buf, NULL, 10);
	ldrx = reg_id_by_name(attr->attr.name);
	config_led_output(pca, ldrx, led_output);

	return count;
}

static
DEVICE_ATTR(led0_output, S_IRUGO | S_IWUSR, show_led_output, set_led_output);
static
DEVICE_ATTR(led1_output, S_IRUGO | S_IWUSR, show_led_output, set_led_output);
static
DEVICE_ATTR(led2_output, S_IRUGO | S_IWUSR, show_led_output, set_led_output);
static
DEVICE_ATTR(led3_output, S_IRUGO | S_IWUSR, show_led_output, set_led_output);

static struct attribute *pca_sysfs_entries[] = {
	&dev_attr_group_dmblnk.attr,
	&dev_attr_led0_pwm.attr,
	&dev_attr_led1_pwm.attr,
	&dev_attr_led2_pwm.attr,
	&dev_attr_led3_pwm.attr,
	&dev_attr_group_dc.attr,
	&dev_attr_group_freq.attr,
	&dev_attr_led0_output.attr,
	&dev_attr_led1_output.attr,
	&dev_attr_led2_output.attr,
	&dev_attr_led3_output.attr,
	NULL
};

static struct attribute_group pca_attr_group = {
	.name	= NULL,			/* put in device directory */
	.attrs	= pca_sysfs_entries,
};

#ifdef CONFIG_PM
static int pca9632_suspend(struct device *dev, pm_message_t state)
{
	/* FIXME: Not implemented */
	return 0;
}

static int pca9632_resume(struct device *dev)
{
	/* FIXME: Not implemented */
	return 0;
}
#else
#define pca9632_suspend NULL
#define pca9632_resume NULL
#endif

static struct i2c_driver pca9632_driver = {
	.driver = {
		.name	 = "pca9632",
		.suspend = pca9632_suspend,
		.resume	 = pca9632_resume,
	},
	.id		= I2C_DRIVERID_PCA9632,
	.attach_adapter	= pca9632_attach_adapter,
	.detach_client	= pca9632_detach_client,
};

static int pca9632_detect(struct i2c_adapter *adapter, int address, int kind)
{
	struct i2c_client *new_client;
	struct pca9632_data *pca;
	int err;

	pca = kzalloc(sizeof(struct pca9632_data), GFP_KERNEL);
	if (!pca)
		return -ENOMEM;

	mutex_init(&pca->lock);

	new_client = &pca->client;
	i2c_set_clientdata(new_client, pca);
	new_client->addr = address;
	new_client->adapter = adapter;
	new_client->driver = &pca9632_driver;
	new_client->flags = 0;

	strlcpy(new_client->name, "pca9632", I2C_NAME_SIZE);

	/* register with i2c core */
	err = i2c_attach_client(new_client);
	if (err)
		goto exit_kfree;

	err = sysfs_create_group(&new_client->dev.kobj, &pca_attr_group);
	if (err)
		goto exit_detach;

	/* software reset */
	if (!software_rst(adapter))
		dev_info(&pca->client.dev, "pca9632 sw-rst done\n");

	/* enter normal mode */
	reg_set_bit_mask(pca, PCA9632_REG_MODE1, 0x10, PCA9632_NORMAL);

	return 0;

exit_detach:
	i2c_detach_client(new_client);
exit_kfree:
	kfree(pca);

	return err;
}

static int pca9632_attach_adapter(struct i2c_adapter *adapter)
{
	return i2c_probe(adapter, &addr_data, pca9632_detect);
}

static int pca9632_detach_client(struct i2c_client *client)
{
	int err;

	sysfs_remove_group(&client->dev.kobj, &pca_attr_group);
	err = i2c_detach_client(client);

	if (err)
		return err;

	kfree(i2c_get_clientdata(client));

	return 0;
}

static int __init pca9632_plat_probe(struct platform_device *pdev)
{
	/* FIXME: platform data should be attached here */
	pca9632_pdev = pdev;

	return 0;
}

static int pca9632_plat_remove(struct platform_device *pdev)
{
	return 0;
}

static struct platform_driver pca9632_plat_driver = {
	.probe	= pca9632_plat_probe,
	.remove	= pca9632_plat_remove,
	.driver = {
		.owner	= THIS_MODULE,
		.name 	= "pca9632",
	},
};

static int __init pca9632_init(void)
{
	int rc;

	rc = platform_driver_register(&pca9632_plat_driver);
	if (!rc)
		i2c_add_driver(&pca9632_driver);

	return rc;
}

static void __exit pca9632_exit(void)
{
	i2c_del_driver(&pca9632_driver);

	platform_driver_unregister(&pca9632_plat_driver);
}

MODULE_AUTHOR("Matt Hsu <matt_hsu@openmoko.org>");
MODULE_DESCRIPTION("NXP PCA9632 driver");
MODULE_LICENSE("GPL");

module_init(pca9632_init);
module_exit(pca9632_exit);
