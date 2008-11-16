/* arch/arm/mach-s3c2410/include/mach/gpio.h
 *
 * Copyright (c) 2008 Simtec Electronics
 *	http://armlinux.simtec.co.uk/
 *	Ben Dooks <ben@simtec.co.uk>
 *
 * S3C2410 - GPIO lib support
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
*/

#define gpio_get_value	__gpio_get_value
#define gpio_set_value	__gpio_set_value
#define gpio_cansleep	__gpio_cansleep

/* These two defines should be removed as soon as the
 * generic irq handling makes it upstream */
#include <mach/hardware.h>
#define gpio_to_irq(gpio)               s3c2410_gpio_getirq(gpio)
#define irq_to_gpio(irq)                s3c2410_gpio_irq2pin(irq)
/* -- cut to here when generic irq makes it */

#include <asm-generic/gpio.h>
