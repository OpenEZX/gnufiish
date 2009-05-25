/* kernel includes */
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/input.h>
#include <linux/i2c.h>
#include <linux/version.h>
#include <linux/mutex.h>
#include <linux/delay.h>
#include <asm/unaligned.h>

#include "si470x-lib.h"

static int si470x_i2c_get_register(struct si470x_device *radio, int regnr)
{
    unsigned char buf[RADIO_REGISTER_NUM * 2];
    unsigned char *p = buf;
    int retval;
    int count;
    int n;
    struct i2c_client *client;

    client = (struct i2c_client *) radio->dev;

    if(!client)
        return -ENXIO;

    printk(KERN_INFO "si470x-i2c: i2c-addr = %s\n", client->addr);
 
    /* internal chip read counter starts at addr of STATUSRSSI 
     * so for example we have to read 16 registers to get the 
     * value of the BOOTCONFIG register
     */
    count = ((regnr - STATUSRSSI) & 0xf) + 1; /* num of register we want */
    retval = i2c_master_recv(client,
                             buf, 
                             count * 2); /* per register we read 2 bytes */

    if(retval != count * 2) 
    {
        printk(KERN_WARNING "wanted %d bytes, got %d\n", count * 2, retval);
        return -EINVAL;
    }

    /* write data to our local register copy */
    for(n = 0; n<count; n++)
    {
        /* save all registers we got */
        radio->registers[((n + STATUSRSSI) & 0xf)] = p[0] << 8 | p[1];
        p += 2;
    }

    return 0;
}

static int si470x_i2c_set_register(struct si470x_device *radio, int regnr)
{
    unsigned char buf[RADIO_REGISTER_NUM * 2];
    unsigned char *p = buf;
    uint16_t data;
    int retval;
    int count;
    int n;

    struct i2c_client *client;

    client = (struct i2c_client *) radio->dev;
    if(!client)
        return -ENXIO;

    /* internal chip write counter starts at addr of POWERCFG,
     * as eg. to write the value of the CHANNEL register we have to 
     * write also the value for the POWERCFG register 
     */
    count = ((regnr - POWERCFG) & 0xf) + 1; /* num of registers we want to write */

    /* put everything together */
    for(n=0; n<count; n++)
    {
        data = radio->registers[(n + POWERCFG) & 0xf];
        *p++ = (data >> 8) & 0xff;
        *p++ = data & 0xff;
    }

    /* write data to bus */
    retval = i2c_master_send(client, 
                             buf, 
                             count * 2); /* 2 bytes per register */

    if(retval != count * 2)
    {
        printk(KERN_WARNING "wanted %d bytes to write, wrote %d\n", count * 2, retval);
        return -EINVAL;
    }

    return 0;
}

static int si470x_i2c_get_all_registers(struct si470x_device *radio)
{
    /* 
     * we simpy aquire the CHIPID register and cause the chip starts 
     * internaly with returning the POWERCFG register we got all 
     * registers
     */
    si470x_i2c_get_register(radio, CHIPID);

    return 0;
}

static int si470x_i2c_get_rds_registers(struct si470x_device *radio)
{
    /* 
     * see comment in function si470x_i2c_get_all_registers()
     */
    si470x_i2c_get_register(radio, RDSD);

    return 0;
}


static int si470x_i2c_autopm_put(struct si470x_device *radio)
{
    return 0;
}

static int si470x_i2c_autopm_get(struct si470x_device *radio)
{
    return 0;
}

struct si470x_ops si470x_i2c_ops = {
    .get_register = si470x_i2c_get_register,
    .set_register = si470x_i2c_set_register,
    .get_all_registers = si470x_i2c_get_all_registers,
    .get_rds_registers = si470x_i2c_get_rds_registers,
    .autopm_put = si470x_i2c_autopm_put,
    .autopm_get = si470x_i2c_autopm_get
};

static int si470x_i2c_probe(struct i2c_client *client,
                            const struct i2c_device_id *id)
{
    struct si470x_device *radio;

    printk(KERN_INFO "si470x_i2c driver probe ...\n");

    if(!(radio = kzalloc(sizeof(*radio), GFP_KERNEL)))
        return -ENOMEM;

    radio->dev = (struct device *) client;
    radio->ops = &si470x_i2c_ops;

    si470xlib_probe(radio);

    i2c_set_clientdata(client, radio);
    
    return 0;
}

static int si470x_i2c_remove(struct i2c_client *client)
{
    struct si470x_device *radio = i2c_get_clientdata(client);

    i2c_set_clientdata(client, NULL);
    kfree(radio);
 
    return 0;
}

static int si470x_i2c_suspend(struct i2client *client)
{
    struct si470x_device *radio = i2c_get_clientdata(client);
    return si470xlib_suspend(radio);
}

static int si470x_i2c_resume(struct i2c_client *client)
{
    struct si470x_device *radio = i2c_get_clientdata(client);
    return si470xlib_resume(radio);
}

static struct i2c_device_id si470x_i2c_id_table[] = {
    {"si470x-i2c", 0},
    { }
};

static struct i2c_driver si470x_i2c_driver = {
    .driver = {
        .owner = THIS_MODULE,
        .name = "si470x-i2c",
    },
    .id_table = si470x_i2c_id_table,

    .probe = si470x_i2c_probe,
    .remove = si470x_i2c_remove,
    .suspend = si470x_i2c_suspend,
    .resume = si470x_i2c_resume, 
};

MODULE_DEVICE_TABLE(i2c, si470x_i2c_id_table);


/*
 * si470x_i2c_module_init - module init
 */
static int __init si470x_i2c_module_init(void)
{
    printk(KERN_INFO "si470x_i2c driver loaded!\n");
    return i2c_add_driver(&si470x_i2c_driver);
}


/*
 * si470x_i2c_module_exit - module exit
 */
static void __exit si470x_i2c_module_exit(void)
{
    i2c_del_driver(&si470x_i2c_driver);
}

module_init(si470x_i2c_module_init);
module_exit(si470x_i2c_module_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Simon Busch <morphis@gravedo.de>");
MODULE_DESCRIPTION("I2C driver for the Si470x chipset");


