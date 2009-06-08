#define DEBUG

#include <linux/kernel.h>
#include <linux/types.h>
#include <linux/interrupt.h>
#include <linux/serial_core.h>
#include <linux/serial.h>
#include <linux/clk.h>
#include <linux/platform_device.h>
#include <linux/dma-mapping.h>

#include <asm/io.h>
#include <asm/irq.h>
#include <asm/dma.h>

#include <plat/regs-spi.h>
#include <plat/regs-dma.h>

#include <mach/dma.h>
#include <mach/spi.h>
#include <mach/spi-gpio.h>
#include <mach/regs-gpio.h>
#include <mach/regs-clock.h>
#include <mach/hardware.h>
#include <mach/glofiish.h>

#include <linux/kthread.h>
#include <linux/delay.h>

#ifdef CONFIG_MACH_M800
#include <mach/glofiish.h>
#endif

int pwron = 0;

/*
 * Power Managemnet for the modem
 */
static void glofiish_modem_enable(int enable)
{
    /* 
     * trace from qemu-gnufiish and knight:
     <ACK>OK</ACK>
    GPHCON: GPH01(in)=1 GPH02(in)=1 
    GPHCON: GPH02(TXD[0])=1 GPH03(RXD[0])=0 
    GPHCON: GPH00(nCTS0)=0 GPH01(nRTS0)=1 
    GPHCON: GPH08(UEXTCLK)=0 
    GPHCON: GPH08(in)=0 
    <ACK>OK</ACK>
    cpld_write: addr=0x00000012 value=1
    GPADAT: GPA03(out)=1 
    GPADAT: GPA11(out)=0 
    GPADAT: GPA15(out)=1 
    Modem reset
    GPADAT: GPA15(out)=0 
    GPBDAT: GPB04(out)=1 
    Modem powered up.
    GPBDAT: GPB04(out)=0 
    Modem powered down.

    <PROCESSING>
    KEYON EMP360
    </PROCESSING>
    GPADAT: GPA11(out)=1 
    glofiish_modem_write (len=1):02 
    glofiish_modem_write (len=1):05 
    glofiish_modem_write (len=1):00 
    glofiish_modem_write (len=1):00 
    glofiish_modem_write (len=1):03 
    glofiish_modem_write (len=1):01 
    glofiish_modem_write (len=1):d2 
    glofiish_modem_write (len=1):02 
    <RESULT><ITEM_NAME>AP Version</ITEM_NAME><TEST>FAILED</TEST></RESULT>GPHCON: GPH07(in)=0 
    GPHCON: GPH03(in)=0 
    GPHCON: GPH02(in)=1 
    GPHCON: GPH02(out)=1 
    GPHCON: GPH06(in)=0 
    GPHCON: GPH06(out)=0 
    GPHDAT: GPH02(out)=0 
    */
    if(1)
    {
        if(enable)
        {
            printk(KERN_INFO "glofiish_modem_test: powering up ...\n");

            /* configure UART */
            s3c2410_gpio_cfgpin(S3C2410_GPH2, S3C2410_GPH2_TXD0);
            s3c2410_gpio_cfgpin(S3C2410_GPH3, S3C2410_GPH3_RXD0);
            s3c2410_gpio_cfgpin(S3C2410_GPH0, S3C2410_GPH0_nCTS0);
            s3c2410_gpio_cfgpin(S3C2410_GPH1, S3C2410_GPH1_nRTS0);

            /* knight also configures GPH8,GPA3 and GPA11 and sets GPA3 and GPA11
             * don't know why 
             */
            s3c2410_gpio_cfgpin(S3C2410_GPH8, S3C2410_GPIO_INPUT);

            s3c2410_gpio_cfgpin(S3C2410_GPA3, S3C2410_GPIO_OUTPUT);
            s3c2410_gpio_setpin(S3C2410_GPA3, 1);

            s3c2410_gpio_cfgpin(S3C2410_GPA11, S3C2410_GPIO_OUTPUT);
            s3c2410_gpio_setpin(S3C2410_GPA11, 0);

            /* Take care off that the modem is already down before launching */
            s3c2410_gpio_cfgpin(M800_GPIO_GSM_RST, S3C2410_GPIO_OUTPUT);
            s3c2410_gpio_setpin(M800_GPIO_GSM_RST, 1);
            msleep(3000);

            s3c2410_gpio_setpin(M800_GPIO_GSM_RST, 0);

            /* Modem powerup sequence:
             * - configure GPB4 as output
             * - set it to low for 100 ticks
             * - set it to high for 20000 ticks
             * - set it low again 
             */
            s3c2410_gpio_cfgpin(M800_GPIO_GSM_PWRON, S3C2410_GPIO_OUTPUT);
            s3c2410_gpio_setpin(M800_GPIO_GSM_PWRON, 0);
            msleep(100);

            s3c2410_gpio_setpin(M800_GPIO_GSM_PWRON, 1);
            msleep(20000);

            s3c2410_gpio_setpin(M800_GPIO_GSM_PWRON, 0);

            /* knight waits a long delay after it powered the modem on 
             * and then puts GPA11 to high 
             * waiting for 100000 ticks is only a guess
             */
            msleep(100000);
            s3c2410_gpio_setpin(S3C2410_GPA11, 1);

            pwron = 1;
        }
        else
        {
            printk(KERN_INFO "glofiish_modem_test: powering down ...\n");

            /* reset the modem by
             * - driving RST hight for 3000 ticks
             * - reset RST to low
             */
            s3c2410_gpio_cfgpin(M800_GPIO_GSM_RST, S3C2410_GPIO_OUTPUT);
            s3c2410_gpio_setpin(M800_GPIO_GSM_RST, 1);
            msleep(3000);
            s3c2410_gpio_setpin(M800_GPIO_GSM_RST, 0);

            /* knight also resets the UART gpio configurations flags */
            s3c2410_gpio_cfgpin(S3C2410_GPH3, S3C2410_GPIO_INPUT);
            s3c2410_gpio_cfgpin(S3C2410_GPH2, S3C2410_GPIO_OUTPUT);
            s3c2410_gpio_cfgpin(S3C2410_GPH6, S3C2410_GPIO_OUTPUT);
            s3c2410_gpio_setpin(S3C2410_GPH2, 0);

            pwron = 0;
        }
    }
}

static int glofiish_modem_pwron_get()
{
    return pwron;
}

static ssize_t power_modem_read(struct device *dev,
                             struct device_attribute *attr,
                             char *buf)
{
    int ret = 0;

    if(!strcmp(attr->attr.name, "power_on") ||
       !strcmp(attr->attr.name, "pwron")) 
    {
        ret = glofiish_modem_pwron_get();
    }

    if(ret)
        return strlcpy(buf, "1\n", 3);
    else
        return strlcpy(buf, "0\n", 3);
}

static ssize_t power_modem_write(struct device *dev,
                              struct device_attribute *attr,
                              const char *buf,
                              size_t count)
{
    unsigned long on = simple_strtoul(buf, NULL, 10);

    if(!strcmp(attr->attr.name, "power_on") ||
       !strcmp(attr->attr.name, "pwron")) 
    {
        glofiish_modem_enable(on);
    }

    return count;
}

static DEVICE_ATTR(power_on, 0644, power_modem_read, power_modem_write);
static DEVICE_ATTR(pwron, 0644, power_modem_read, power_modem_write);

static struct attribute *glofiish_pm_modem_sysfs_entries[] = {
    &dev_attr_power_on.attr,
    &dev_attr_pwron.attr,
    NULL
};

static struct attribute_group glofiish_pm_modem_attr_group = {
    .name = NULL,
    .attrs = glofiish_pm_modem_sysfs_entries,
};


static int __init glofiish_pm_modem_probe(struct platform_device *pdev)
{
    printk(KERN_INFO "glofiish_pm_modem: probing ...\n");
    return sysfs_create_group(&pdev->dev.kobj, &glofiish_pm_modem_attr_group);
}

static int glofiish_pm_modem_remove(struct platform_device *pdev)
{
    printk(KERN_INFO "glofiish_pm_modem: removing ...\n");
    sysfs_remove_group(&pdev->dev.kobj, &glofiish_pm_modem_attr_group);
    return 0;
}

static struct platform_driver glofiish_pm_modem_driver = {
	.probe		= glofiish_pm_modem_probe,
	.remove		= glofiish_pm_modem_remove,
	.driver		= {
		.name		= "glofiish-pm-modem",
	},
};

static int __devinit glofiish_pm_modem_init(void)
{
    printk(KERN_INFO "glofiish_pm_modem_init\n");
	return platform_driver_register(&glofiish_pm_modem_driver);
}

static void glofiish_pm_modem_exit(void)
{
    printk(KERN_INFO "glofiish_pm_modem_exit\n");
	platform_driver_unregister(&glofiish_pm_modem_driver);
}

module_init(glofiish_pm_modem_init);
module_exit(glofiish_pm_modem_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Simon Busch <morphis@gravedo.de>");
MODULE_DESCRIPTION("E-TEN glofiish GSM/UMTS Modem Power Management");
