/*
 * board-specific data for the libertas_spi driver.
 *
 * Copyright 2008 Analog Devices Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or (at
 * your option) any later version.
 */
#ifndef _LIBERTAS_SPI_H_
#define _LIBERTAS_SPI_H_
struct libertas_spi_platform_data {
	/* There are two ways to read data from the WLAN module's SPI
	 * interface. Setting 0 or 1 here controls which one is used.
	 *
	 * Usually you want to set use_dummy_writes = 1.
	 * However, if that doesn't work or if you are using a slow SPI clock
	 * speed, you may want to use 0 here. */
	u16 use_dummy_writes;

	/* All SPI communications are initiated by the bus master.  In order
	 * for the WLAN module to tell the kernel that it has received data
	 * from the network, it must use an out-of-band IRQ line.  Here we
	 * specify which IRQ line to use.  */
	u16 host_irq_num;

	/* GPIO number to use as chip select */
	u16 gpio_cs;
};
#endif
