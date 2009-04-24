/*
 * E-TEN glofiish GSM/UMTS modem support
 *
 * Copyright (C) 2008 by Harald Welte <laforge@gnumonks.org>
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

 /*
==== SPI interface ====

* uses SPIMISO1/SPIMOSI1/SPICLK1/nSS1 of S3C2442B
* SPI controller registers suggest slave mode
* SPI driver in WM6.1 uses DMA accelerated transfers
* DMA Rx buffer address (according to register 0x4b0000dc): 0x3093f004
<pre>
HaRET(97)# pdump 0x3093f004 0x200
3093f004 | 454d432b 52524520 203a524f b66f0d31 | +CME ERROR: 1.o.
</pre>

===== protocol =====
The protocol is really weird, since AT commands are usually requests by the AP sent to the modem,
so it would make lots of sense to make the AP the SPI master.  As SPI master it could by itself initiate
requests at any point in time.

In any case, for some reason the designers of the M800 did it the other way around:  The modem is the SPI
master, and the AP is the slave.  So how does a typical transfer look like?

* transfer of command from AP to modem
** AP sets GPA16 (GSM_SPI_REQ) to one (tell the modem we want to Tx)
** modem responds by asserting nSS1 and EINT16 (see below)
** SPI transfer proceeds (CPOL=0, CPHA=0)
** AP sets GPA16 (GSM_SPI_REQ) to zero (tell the modem we're done)
** modem responds by de-asserting nSS1 and EINT16 (see below)

NOTE: nSS1 seems to be interconnected to EINT16, in order to assure the AP gets an interrupt whenever the modem wants to transmit something.

*/

#define DEBUG
#define USE_DMA

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
#include <mach/dma.h>
#include <mach/regs-gpio.h>
#include <mach/regs-clock.h>
#include <mach/glofiish.h>

#include <linux/kthread.h>
#include <linux/delay.h>

enum spi_state {
	STATE_NONE,
	STATE_IDLE,		/* idle */
	STATE_WAIT_nSS1,	/* waiting for nSS1 after GPA16 trigger */
	STATE_WAIT_CMD,		/* waiting for an incoming CMD packet */
	STATE_RCV_CMD,		/* receiving bytes of CMD packet */
	STATE_WAIT_RX,		/* waiting for an incoming RX packet */
	STATE_RX,		/* receiving bytes of RX packet */
	STATE_WAIT_TX,		/* waiting for starting TX packet */
	STATE_TX,		/* transmitting bytes of TX packet */
};

static const char *state_names[] = {
	[STATE_NONE]	= "NONE",
	[STATE_IDLE]	= "IDLE",
	[STATE_WAIT_nSS1]= "WAIT_nSS1",
	[STATE_WAIT_CMD]= "WAIT_CMD",
	[STATE_RCV_CMD]	= "RCV_CMD",
	[STATE_WAIT_RX]	= "WAIT_RX",
	[STATE_RX]	= "RX",
	[STATE_WAIT_TX]	= "WAIT_TX",
	[STATE_TX]	= "TX",
};

static inline const char *state_name(enum spi_state state)
{
	if (state > STATE_TX)
		return "UNKNOWN";
	else
		return state_names[state];
}

struct gfish_modem {
	struct platform_device *pdev;
	unsigned int irq;
	unsigned int gpio_req;
	struct {
		/* resources */
		struct resource *ioarea;
		void __iomem *regs;
		unsigned int dmach;
		struct clk *clk;
		unsigned int irq;
		enum spi_state state;
		spinlock_t lock;

		unsigned char rx_buf[1024];
		unsigned int rx_len;
		unsigned int rx_idx;

		unsigned char tx_buf[1024];
		unsigned int tx_len;
		unsigned int tx_idx;
#if 0
		/* buffers */
		struct circ_buf tx_buf;
		struct circ_buf rx_buf;
		/* misc */
		unsigned int count;
#endif
	} spi;
	struct uart_port port;
};

#define port_to_modem(x)	container_of((x), struct gfish_modem, port)
#if 1
/***********************************************************************

 * UART part
 ***********************************************************************/

static void gfish_modem_pm(struct uart_port *port,
			   unsigned int level, unsigned int old)
{
	struct gfish_modem *gm = port_to_modem(port);

	dev_dbg(&gm->pdev->dev, "modem_pm(%u, %u)\n", level, old);

	/* FIXME */
}

static unsigned int gfish_modem_tx_empty(struct uart_port *port)
{
	struct gfish_modem *gm = port_to_modem(port);
	//unsigned int spsta = readb(gm->spi.regs + S3C2410_SPSTA);

	dev_dbg(&gm->pdev->dev, "tx_empty()=1\n");
	//return (spsta & S3C2410_SPSTA_READY);
	return 1;
}

static unsigned int gfish_modem_get_mctrl(struct uart_port *port)
{
	struct gfish_modem *gm = port_to_modem(port);
	dev_dbg(&gm->pdev->dev, "get_mctrl()\n");
	/* FIXME: should we emulate RTS/CTS ? */
	return TIOCM_CAR | TIOCM_DSR | TIOCM_CTS;
}

static void gfish_modem_set_mctrl(struct uart_port *port,
				  unsigned int mctrl)
{
	struct gfish_modem *gm = port_to_modem(port);
	dev_dbg(&gm->pdev->dev, "set_mctrl()\n");
	/* FIXME: should we emulate RTS/CTS ? */
}

/* since we're the SPI slave, we cannot really immediately
 * halt further transmits.  All we can do is signal the master
 * that we don't have any more pending data */
static void gfish_modem_stop_tx(struct uart_port *port)
{
	struct gfish_modem *gm = port_to_modem(port);

	dev_dbg(&gm->pdev->dev, "stop_tx\n");
	//set_gsm_spi_req(gm, 0);
}

static void gfish_modem_start_tx(struct uart_port *port)
{
	struct gfish_modem *gm = port_to_modem(port);

	dev_dbg(&gm->pdev->dev, "start_tx\n");
	/* Assert GPA16 and thus trigger the CMD SPI RX stage
	   and further operations */
	//set_gsm_spi_req(gm, 1);
}

static void gfish_modem_stop_rx(struct uart_port *port)
{
	struct gfish_modem *gm = port_to_modem(port);

	dev_dbg(&gm->pdev->dev, "stop_rx\n");
	/* FIXME: what should we do? */
}

static void gfish_modem_enable_ms(struct uart_port *port)
{
	struct gfish_modem *gm = port_to_modem(port);
	dev_dbg(&gm->pdev->dev, "enable_ms()\n");
	/* we don't support this. just ignore it */
}

static void gfish_modem_break_ctl(struct uart_port *port, int ctl)
{
	struct gfish_modem *gm = port_to_modem(port);
	dev_dbg(&gm->pdev->dev, "break_ctl()\n");
	/* we don't support this. just ignore it */
}

static int gfish_modem_startup(struct uart_port *port)
{
	struct gfish_modem *gm = port_to_modem(port);

	dev_dbg(&gm->pdev->dev, "startup()\n");

	/* enable SPI interrupt generation */
	writeb(S3C2410_SPCON_SMOD_INT, gm->spi.regs + S3C2410_SPCON);

	return 0;
}

static void gfish_modem_shutdown(struct uart_port *port)
{
	struct gfish_modem *gm = port_to_modem(port);

	dev_dbg(&gm->pdev->dev, "shutdown()\n");

	gfish_modem_stop_rx(port);
	gfish_modem_stop_tx(port);
	/* disable SPI interrupt generation */
	writeb(0x00, gm->spi.regs + S3C2410_SPCON);
}

static void gfish_modem_set_termios(struct uart_port *port,
				    struct ktermios *termios,
				    struct ktermios *old)
{
	struct gfish_modem *gm = port_to_modem(port);
	dev_dbg(&gm->pdev->dev, "set_termios()\n");

	/* we don't support mode mcontrol lines */
	termios->c_cflag &= ~(HUPCL | CMSPAR);	
	termios->c_cflag |= CLOCAL;

	//uart_update_timeout(port, termios->c_flag, baud)
}

static const char *gfish_modem_type(struct uart_port *port)
{
	return "glofiish";
}

static int gfish_modem_request_port(struct uart_port *port)
{
	struct gfish_modem *gm = port_to_modem(port);
	dev_dbg(&gm->pdev->dev, "request_port()=-EINVAL\n");
	return -EINVAL;
}

static void gfish_modem_release_port(struct uart_port *port)
{
	struct gfish_modem *gm = port_to_modem(port);
	dev_dbg(&gm->pdev->dev, "release_port()\n");
}

static void gfish_modem_config_port(struct uart_port *port, int x)
{
	struct gfish_modem *gm = port_to_modem(port);
	dev_dbg(&gm->pdev->dev, "config_port()\n");
}

static int gfish_modem_verify_port(struct uart_port *port,
				   struct serial_struct *ser)
{
	struct gfish_modem *gm = port_to_modem(port);
	dev_dbg(&gm->pdev->dev, "verify_port()\n");
	if (ser->type != PORT_UNKNOWN && ser->type != PORT_GFISH)
		return -EINVAL;

	return 0;
}

static struct uart_ops gm_uart_ops = {
	.pm		= gfish_modem_pm,
	.tx_empty	= gfish_modem_tx_empty,
	.get_mctrl	= gfish_modem_get_mctrl,
	.set_mctrl	= gfish_modem_set_mctrl,
	.stop_tx	= gfish_modem_stop_tx,
	.start_tx	= gfish_modem_start_tx,
	.stop_rx	= gfish_modem_stop_rx,
	.enable_ms	= gfish_modem_enable_ms,
	.break_ctl	= gfish_modem_break_ctl,
	.startup	= gfish_modem_startup,
	.shutdown	= gfish_modem_shutdown,
	.set_termios	= gfish_modem_set_termios,
	.type		= gfish_modem_type,
	.release_port	= gfish_modem_release_port,
	.request_port	= gfish_modem_request_port,
	.config_port	= gfish_modem_config_port,
	.verify_port	= gfish_modem_verify_port,
};

static struct uart_driver gm_uart_driver = {
	.owner		= THIS_MODULE,
	.dev_name	= "glofiish_modem",
	.nr		= 1,
	.driver_name	= "ttyGM",
	//.major		= TTY_MAJOR,
};

/***********************************************************************
 * SPI slave driver 
 ***********************************************************************/

static void hexdump(unsigned char *data, int len)
{
	int i;
	for (i = 0; i < len; i++)
		printk("%02x ", data[i]);
	printk("\n");
}

static void rx_complete(struct gfish_modem *gm)
{
	dev_dbg(&gm->pdev->dev, "rx_complete: ");
	hexdump(gm->spi.rx_buf, gm->spi.rx_len);
}

static void tx_complete(struct gfish_modem *gm)
{
	dev_dbg(&gm->pdev->dev, "tx_complete: ");
	hexdump(gm->spi.tx_buf, gm->spi.tx_len);
}

static void set_gsm_spi_req(struct gfish_modem *gm, int on)
{
	s3c2410_gpio_setpin(gm->gpio_req, on);
}

static int prepare_dma(struct gfish_modem *gm, int tx);

/* configure the SPI controller based on the current state */
static int config_spi(struct gfish_modem *gm)
{
	u_int8_t smod = S3C2410_SPCON_SMOD_INT;
	/* initialize length to 4 */
	u_int16_t len = 4;

	dev_dbg(&gm->pdev->dev, "config_spi(state=%s)\n",
		state_name(gm->spi.state));
#ifdef USE_DMA
	smod = S3C2410_SPCON_SMOD_DMA;
#endif

	switch (gm->spi.state) {
	case STATE_WAIT_RX:
		/* determine read length from last command */
		len = (gm->spi.rx_buf[3] << 8 | gm->spi.rx_buf[4]);
		dev_dbg(&gm->pdev->dev, "len=%d+4\n", len);
		/* add four bytes for the header */
		len += 4;
		if (len > sizeof(gm->spi.rx_buf)) {
			dev_err(&gm->pdev->dev,
				"cannot read %d bytes, limiting to %d\n",
				len, sizeof(gm->spi.rx_buf));
			len = sizeof(gm->spi.rx_buf);
		}
		/* fallthrough */
	case STATE_NONE:
	case STATE_WAIT_CMD:
	case STATE_WAIT_nSS1:
		/* make sure we read bytes until we have reached this length */
		gm->spi.rx_len = len;
		/* read two bytes garbage */
		writeb(S3C2410_SPCON_TAGD, gm->spi.regs + S3C2410_SPCON);
		readb(gm->spi.regs + S3C2410_SPRDAT);
		readb(gm->spi.regs + S3C2410_SPRDAT);
		/* enable interrupt mode */
		writeb(S3C2410_SPCON_TAGD|smod, gm->spi.regs + S3C2410_SPCON);
#ifdef USE_DMA
		prepare_dma(gm, 0);
#endif
		/* Assert GPA16 */
		set_gsm_spi_req(gm, 1);
		/* ... we will generate SPI interrupts or DMA completion events
		 * and continue there */
		break;
	case STATE_WAIT_TX:
		/* the caller has initialized spi.tx_len and tx_buf */
		/* enable interrupt mode */
		writeb(smod, gm->spi.regs + S3C2410_SPCON);
#ifdef USE_DMA
		prepare_dma(gm, 1);
#endif
		/* Assert GPA16 */
		set_gsm_spi_req(gm, 1);
		/* ... we will generate SPI interrupts and continue there */
		break;
	default:
		dev_dbg(&gm->pdev->dev, "can't configure state %s\n",
			state_name(gm->spi.state));
	}
	
	return 0;
}

#ifdef USE_DMA
static void dma_done_callback(struct s3c2410_dma_chan *dma_ch,
			      void *buf_id, int size,
			      enum s3c2410_dma_buffresult result)
{
	struct gfish_modem *gm = buf_id;
	struct device *dev = &gm->pdev->dev;
	
	dev_dbg(dev, "SPI DMA done old_state=%s size=%d\n", 
		state_name(gm->spi.state), size);

	if (result != S3C2410_RES_OK) {
		dev_dbg(dev, "DMA FAILED: result=0x%x\n", result);
		return;
	}

	switch (gm->spi.state) {
	case STATE_WAIT_TX:
		//dma_unmap_single(dev, FIXME, gm->spi.tx_len, DMA_TO_DEVICE);
		gm->spi.tx_idx = 0;
		/* once finished, de-assert GPA16 */
		set_gsm_spi_req(gm, 0);
		/* FIXME: notify caller about transmitted data */
		tx_complete(gm);
		/* re-start with reading four bytes */
		gm->spi.state = STATE_WAIT_RX;
		break;
	case STATE_WAIT_RX:
	case STATE_WAIT_CMD:
		//dma_unmap_single(dev, FIXME, gm->spi.rx_len, DMA_FROM_DEVICE);
		/* reset command index for next transfer */
		gm->spi.rx_idx = 0;
		/* de-assert GPA16 */
		set_gsm_spi_req(gm, 0);

		//if (gm->spi.rx_len > 4) {
			/* FIXME: process received data */
			rx_complete(gm);
		//}
		if (gm->spi.rx_buf[0] != 0x00)
			dev_dbg(dev, "first byte of buffer not 0x00?\n");

		switch (gm->spi.rx_buf[1]) {
		case 0x00:	/* read from modem */
			gm->spi.state = STATE_WAIT_RX;
			break;
		case 0x01:	/* write to modem */
			gm->spi.state = STATE_WAIT_TX;
			break;
		case 0x02:	/* finish */
			gm->spi.state = STATE_NONE;
			break;
		}
		break;
	default:
		dev_dbg(dev, "state %s not valid for DMA\n",
			state_name(gm->spi.state));
		return;
		break;
	}
	/* configure SPI controller based on new gm->spi.state */
	config_spi(gm);
}

static void dma_setup(struct gfish_modem *gm, enum s3c2410_dmasrc source)
{
	struct device *dev = &gm->pdev->dev;
	static enum s3c2410_dmasrc last_source = -1;
	static int setup_ok;
	unsigned long devaddr;

	dev_dbg(dev, "dma_setup(source=%u)\n", source);

	if (last_source == source) {
		dev_dbg(dev, "dma_setup(): skipping, same as last source\n");
		return;
	}

	last_source = source;

	if (source == S3C2410_DMASRC_HW) 
		devaddr = gm->spi.ioarea->start + S3C2410_SPRDAT;
	else
		devaddr = gm->spi.ioarea->start + S3C2410_SPTDAT;

	dev_dbg(dev, "dma_setup(): devaddr = 0x%08lx\n", devaddr);

	/* '3' means: fixed address, APB */
	s3c2410_dma_devconfig(gm->spi.dmach, source, 3, devaddr);

	if (!setup_ok) {
		dev_dbg(dev, "dma_setup(): first-time setup\n");
		/* '1' means byte-wise transfers */
		s3c2410_dma_config(gm->spi.dmach, 1,
			(S3C2410_DCON_HANDSHAKE | S3C2410_DCON_HWTRIG |
			 S3C2410_DCON_CH3_SPI));
		s3c2410_dma_set_buffdone_fn(gm->spi.dmach,
					    dma_done_callback);
		s3c2410_dma_setflags(gm->spi.dmach, S3C2410_DMAF_AUTOSTART);
		setup_ok = 1;
	}
}

static int prepare_dma(struct gfish_modem *gm, int tx)
{
	struct device *dev = &gm->pdev->dev;
	dma_addr_t data;
	int rc, size;

	dma_setup(gm, tx ? S3C2410_DMASRC_MEM : S3C2410_DMASRC_HW);
	s3c2410_dma_ctrl(gm->spi.dmach, S3C2410_DMAOP_FLUSH);

	if (tx) {
		data = dma_map_single(dev, gm->spi.tx_buf, gm->spi.tx_len,
				      DMA_TO_DEVICE);
		size = gm->spi.tx_len;
	} else {
		data = dma_map_single(dev, gm->spi.rx_buf, gm->spi.rx_len,
				      DMA_FROM_DEVICE);
		size = gm->spi.rx_len;
	}
	dev_dbg(dev, "prepare_dma(tx=%d): data=0x%08x size=%d\n",
		tx, data, size);

	rc = s3c2410_dma_enqueue(gm->spi.dmach, gm, data, size);
	if (rc < 0)
		dev_err(dev, "dma_enqueue failed with %d\n", rc);

	rc = s3c2410_dma_ctrl(gm->spi.dmach, S3C2410_DMAOP_START);
	if (rc < 0)
		dev_err(dev, "dma_ctrl(START) failed with %d\n", rc);

	return rc;
}

static struct s3c2410_dma_client gm_dma_client = {
	.name	= "gfish-modem-dma",
};
#endif /* USE_DMA */

static int s3c24xx_spi_slave_init(struct gfish_modem *gm)
{
	dev_dbg(&gm->pdev->dev, "slave_init()\n");

	/* set upt the GPIO config */
	s3c2410_gpio_cfgpin(S3C2410_GPD8, S3C2440_GPD8_nSPIMISO1);
	s3c2410_gpio_cfgpin(S3C2410_GPD9, S3C2440_GPD9_nSPIMOSI1);
	//s3c2410_gpio_cfgpin(S3C2410_GPD10, S3C2440_GPD10_nSPICLK1);
	s3c2410_gpio_cfgpin(S3C2410_GPD10, S3C2410_GPIO_INPUT);
	s3c2410_gpio_cfgpin(S3C2410_GPG3, S3C2440_GPG3_nSS1);

	/* Set SPI slave mode S3C2410_SPCON_MSTR */
	/* FIXME writeb(S3C2410_SPCON_MSTR, gm->spi.regs + S3C2410_SPCON); */
	//writeb(S3C2410_SPPIN_KEEP, gm->spi.regs + S3C2410_SPPIN);

	return 0;
}

/* test command: transmit ATZ and wait for response */
const char atz[] = { 'A', 'T', 'Z', '\r', 0xff, 0xff };
static void tx_atz(struct gfish_modem *gm)
{
	/* put ATZ command in transmit buffer */
	memcpy(gm->spi.tx_buf, atz, sizeof(atz));
	gm->spi.tx_len = sizeof(atz);
	/* initialize state and SPI controller + ask modem via GPA16 */
	gm->spi.state = STATE_WAIT_nSS1;
	config_spi(gm);
}
#endif
/* This interrupt is called by the SPI controller */
static irqreturn_t s3c24xx_spi_irq(int irq, void *priv)
{
printk("Got SPI IRQ\n");

#if 1
	struct gfish_modem *gm = priv;
	struct device *dev = &gm->pdev->dev;
	unsigned int spsta = readb(gm->spi.regs + S3C2410_SPSTA);
#ifndef USE_DMA
	unsigned char ch, tx_ch;
	struct uart_port *port = &gm->port;
	struct circ_buf *xmit = NULL;
	u_int8_t byte;
#endif

	dev_dbg(dev, "SPI IRQ old_state=%s spsta=0x%x\n",
		state_name(gm->spi.state), spsta);

	if (spsta & S3C2410_SPSTA_DCOL) {
		dev_dbg(dev, "data-collision\n");
		goto irq_done;
	}

#ifndef USE_DMA
	switch (gm->spi.state) {
	case STATE_WAIT_RX:
		/* received 1st character of RX transfer */
		byte = readb(gm->spi.regs + S3C2410_SPRDAT);
		gm->spi.rx_buf[gm->spi.rx_idx++] = byte;
		dev_dbg(dev, "received first RX byte %02x\n", byte);
		gm->spi.state = STATE_RX;
		break;
	case STATE_WAIT_CMD:
		/* received 1st character of CMD transfer */
		byte = readb(gm->spi.regs + S3C2410_SPRDAT);
		gm->spi.rx_buf[gm->spi.rx_idx++] = byte;
		dev_dbg(dev, "received first CMD byte %02x\n", byte);
		gm->spi.state = STATE_RCV_CMD;
		break;
	case STATE_RCV_CMD:
	case STATE_RX:
		/* received 2nd...4th character of CMD or RX transfer */
		byte = readb(gm->spi.regs + S3C2410_SPRDAT);
		gm->spi.rx_buf[gm->spi.rx_idx++] = byte;
		dev_dbg(dev, "received 2nd... CMD/RX byte %02x\n", byte);
		if (gm->spi.rx_idx == gm->spi.rx_len) {
			/* reset command index for next transfer */
			gm->spi.rx_idx = 0;
			/* de-assert GPA16 */
			set_gsm_spi_req(gm, 0);

			if (gm->spi.rx_len > 4) {
				/* FIXME: process received data */
				rx_complete(gm);
			}
			if (gm->spi.rx_buf[0] != 0x00)
				dev_dbg(dev, "first byte of buffer not 0x00?\n");

			switch (gm->spi.rx_buf[1]) {
			case 0x00:	/* read from modem */
				gm->spi.state = STATE_WAIT_RX;
				break;
			case 0x01:	/* write to modem */
				gm->spi.state = STATE_WAIT_TX;
				break;
			case 0x02:	/* finish */
				gm->spi.state = STATE_NONE;
				break;
			}
			/* configure SPI controller based on new gm->spi.state */
			config_spi(gm);
		}
		break;
	case STATE_WAIT_TX:
		if (!(spsta & S3C2410_SPSTA_READY)) {
			dev_dbg(dev, "spi not ready for tx?\n");
			goto irq_done;
		}
		/* send 1st byte of TX transfer */
		byte = gm->spi.tx_buf[gm->spi.tx_idx++];
		writeb(byte, gm->spi.regs + S3C2410_SPTDAT);
		dev_dbg(dev, "wrote first TX byte %02x\n", byte);
		gm->spi.state = STATE_TX;
		break;
	case STATE_TX:
		if (!(spsta & S3C2410_SPSTA_READY)) {
			dev_dbg(dev, "spi not ready for tx?\n");
			goto irq_done;
		}
		/* send 2nd... byte of TX transfer */
		byte = gm->spi.tx_buf[gm->spi.tx_idx++];
		writeb(byte, gm->spi.regs + S3C2410_SPTDAT);
		dev_dbg(dev, "wrote 2nd... TX byte %02x\n", byte);
		if (gm->spi.tx_idx == gm->spi.tx_len) {
			gm->spi.tx_idx = 0;
			/* once finished, de-assert GPA16 */
			set_gsm_spi_req(gm, 0);
			/* FIXME: notify caller about transmitted data */
			tx_complete(gm);
			/* re-start with reading four bytes */
			gm->spi.state = STATE_WAIT_RX;
		}
		break;
	default:
		dev_dbg(dev, "not sure what to do from state %s\n",
			state_name(gm->spi.state));
	}
#endif /* ! USE_DMA */
#endif
#if 0
	if (port && port->info) {
 		xmit = &port->info->xmit;
	} else
		dev_dbg(dev, "no port->info\n");

if (xmit) {

	/* send more data */
	if (uart_circ_empty(xmit) || uart_tx_stopped(port)) {
		/* transmit a dummy character if we don't have more data */
		tx_ch = 0x00;
	} else {
		tx_ch = xmit->buf[xmit->tail];
		xmit->tail = (xmit->tail + 1) & (UART_XMIT_SIZE -1);
	}
	
	writeb(tx_ch, gm->spi.regs + S3C2410_SPTDAT);
	port->icount.tx++;

	if (uart_circ_chars_pending(xmit) < WAKEUP_CHARS)
		uart_write_wakeup(port);

	if (uart_circ_empty(xmit))
		gfish_modem_stop_tx(port);
	
}

	/* read data */
	port->icount.rx++;
	ch = readb(gm->spi.regs + S3C2410_SPRDAT);
	uart_insert_char(port, 0, 0, ch, TTY_NORMAL);

	dev_dbg(dev, "Rx: 0x%x Tx: 0x%x\n", ch, tx_ch);
#endif

irq_done:
	return IRQ_HANDLED;
}
#if 1
#if 0
/* high level function to transceive some data */
int gfish_modem_transceive(struct gfish_modem *gm, const u_int8_t *tx,
			   u_int8_t *rx, int len)
{
	/* tell the modem we want to talk */
	s3c24xx_gpio_setpin(gm->gpio_req, 1);
	
	/* FIXME: initialize the DMA descriptors */

	/* now the modem will assert nSS1 and thus assert nSS1 */
	wait_for_completion(&gm->nSS1_compl);

	/* do the actual SPI transfer */
	/* FIXME */
	
	/* once we're finished, de-assert GPIO */
	s3c24xx_gpio_setpin(gm->gpio_req, 0);

	return len;
}
#endif

/* commands as sent by the modem.  They basically tell the
 * application processor what to do.  So 'READ' means read
 * by the AP from the GSM modem */
#define GFISH_SPI_CMD_DO_READ	0x0000
#define GFISH_SPI_CMD_DO_WRITE	0x0001
#define GFISH_SPI_CMD_DONE	0x0002

struct gfish_spi_cmd {
	u_int16_t	cmd;
	u_int16_t	dummy;
};
#endif
/* This interrupt is called every every time the modem asserts nSS1
 * and thereby activates the SPI transfer with the application processor */
static irqreturn_t gfish_modem_irq(int irq, void *priv)
{
#if 1
	struct gfish_modem *gm = priv;
	struct device *dev = &gm->pdev->dev;
	int level = s3c2410_gpio_getpin(S3C2410_GPG3);
#endif
	printk("Got GSM IRQ\n");
#if 1
	dev_dbg(dev, "Modem nSS1 IRQ(%u) state(%s)\n",
		level ? 1 : 0, state_name(gm->spi.state));

	if (level) {
		dev_dbg(dev, "ignoring nSS1 release\n");
		goto out;
	}

	switch (gm->spi.state) {
	case STATE_WAIT_nSS1:
		/* we've been waiting for nSS1 after a AP-initiated
		 * communication start */
		gm->spi.state = STATE_WAIT_CMD;
		/* do not config_spi, as we've already been config'ed */
		break;
	default:
		config_spi(gm);	
		break;
	}
out:
#endif
	return IRQ_HANDLED;
}
#if 1
static ssize_t set_txatz(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	struct gfish_modem *gm = dev_get_drvdata(dev);
	tx_atz(gm);
	return count;
}

static DEVICE_ATTR(tx_atz, S_IWUSR | S_IRUGO,
		   NULL, set_txatz);
#endif
static int spi_clk_poller(void *d)
{
	struct device *dev = d;
	dev_dbg(dev, "Poller thread started");
	int level = -1, last_level = -2;

	while(!kthread_should_stop()) {
		level = !!s3c2410_gpio_getpin(S3C2410_GPD10);
		if (level != last_level) {
			printk("***** Got an edge on SPICLK1: %i\n", level);
			last_level = level;
		}
		//msleep(1);
	}

	return 0;
}

static struct task_struct *poller_thread;

static int __init gm_probe(struct platform_device *pdev)
{
	int rc;
	struct gfish_modem *gm;
	struct resource *res;

	gm = kzalloc(sizeof(*gm), GFP_KERNEL);
	if (!gm)
		return -ENOMEM;

	gm->pdev = pdev;

	/* FIXME: move this to platform data */
	//gm->irq = M800_IRQ_GSM;
	gm->irq = platform_get_irq(pdev, 0);
	gm->spi.irq = platform_get_irq(pdev, 1);
	gm->gpio_req = M800_GPIO_GSM_REQ;
	gm->spi.dmach = DMACH_SPI1;
	platform_set_drvdata(pdev, gm);

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res) {
		dev_err(&pdev->dev, "Cannot get IORESOURCE_MEM\n");
		rc = -ENOENT;
		goto out_free;
	}

	/* Configure the GSM_REQ pin as output */
	s3c2410_gpio_cfgpin(M800_GPIO_GSM_REQ, S3C2410_GPIO_OUTPUT);

	/* de-assert the GSM SPI request */
	set_gsm_spi_req(gm, 0);

	gm->spi.ioarea = request_mem_region(res->start,
					    (res->end - res->start)+1,
					    pdev->name);
	if (!gm->spi.ioarea) {
		dev_err(&pdev->dev, "Cannot reserve region\n");
		rc = -ENXIO;
		goto out_free;
	}

	gm->spi.regs = ioremap(res->start, (res->end - res->start)+1);
	if (!gm->spi.regs) {
		dev_err(&pdev->dev, "Cannot map IO\n");
		rc = -ENXIO;
		goto out_reqmem;
	}

	gm->spi.clk = clk_get(&pdev->dev, "spi");
	if (IS_ERR(gm->spi.clk)) {
		dev_err(&pdev->dev, "No clock for spi\n");
		rc = PTR_ERR(gm->spi.clk);
		goto out_remap;
	}
#if 1
	rc = uart_register_driver(&gm_uart_driver);
	if (rc < 0) {
		dev_err(&pdev->dev, "Cannot register UART driver\n");
		goto out_clk;
	}

	gm->port.dev = &pdev->dev;
	gm->port.irq = gm->spi.irq;
	gm->port.type = PORT_GFISH;
	gm->port.membase = gm->spi.ioarea->start;
	gm->port.fifosize = 1;
	gm->port.ops = &gm_uart_ops;
	rc = uart_add_one_port(&gm_uart_driver, &gm->port);
	if (rc < 0) {
		dev_err(&pdev->dev, "Cannot add UART port\n");
		goto out_driver;
	}
#endif
	rc = request_irq(gm->irq, gfish_modem_irq,
			 IRQF_TRIGGER_RISING|IRQF_TRIGGER_FALLING,
			 pdev->name, gm);
	if (rc) {
		dev_err(&pdev->dev, "Cannot claim IRQ\n");
		goto out_port;
	}
	enable_irq_wake(gm->irq);

	rc = request_irq(gm->spi.irq, s3c24xx_spi_irq, 0,
			 "gfish_modem SPI slave", gm);
	if (rc) {
		dev_err(&pdev->dev, "Cannot claim IRQ\n");
		goto out_irq;
	}
#if 1
	rc = device_create_file(&pdev->dev, &dev_attr_tx_atz);
	if (rc < 0) {
		dev_err(&pdev->dev, "failed to add sysfs file\n");
		goto out_irq2;
	}

#ifdef USE_DMA
	rc = s3c2410_dma_request(gm->spi.dmach, &gm_dma_client, NULL);
	if (rc < 0) {
		dev_err(&pdev->dev, "failed to request DMA channel\n");
		goto out_file;
	}
#endif
#endif
	poller_thread = kthread_run(spi_clk_poller, gm, "spi-slave-clk-poller");
		//dev_err(&pdev->dev, "failed to setup spi slave poller thread");

	s3c24xx_spi_slave_init(gm);

	return 0;

out_dma:
#ifdef USE_DMA
	s3c2410_dma_free(gm->spi.dmach, &gm_dma_client);
#endif
out_file:
	device_remove_file(&pdev->dev, &dev_attr_tx_atz);
out_irq2:
	free_irq(gm->spi.irq, gm);
out_irq:
	disable_irq_wake(gm->irq);
	free_irq(gm->irq, gm);
out_port:
	uart_remove_one_port(&gm_uart_driver, &gm->port);
out_driver:
	uart_unregister_driver(&gm_uart_driver);
out_clk: 
	clk_put(gm->spi.clk);
out_remap:
	iounmap(gm->spi.regs);
out_reqmem:
	release_resource(gm->spi.ioarea);
	kfree(gm->spi.ioarea);
out_free:
	platform_set_drvdata(pdev, NULL);
	kfree(gm);

	return rc;
}

static int gm_remove(struct platform_device *pdev)
{
	struct gfish_modem *gm = platform_get_drvdata(pdev);

	kthread_stop(poller_thread);

	/* de-assert the GSM SPI request */
	set_gsm_spi_req(gm, 0);
#if 1
#ifdef USE_DMA
	s3c2410_dma_ctrl(gm->spi.dmach, S3C2410_DMAOP_FLUSH);
	s3c2410_dma_free(gm->spi.dmach, &gm_dma_client);
#endif
	device_remove_file(&pdev->dev, &dev_attr_tx_atz);
#endif
	free_irq(gm->spi.irq, gm);
	//disable_irq_wake(gm->irq);
	free_irq(gm->irq, gm);
	uart_remove_one_port(&gm_uart_driver, &gm->port);
	uart_unregister_driver(&gm_uart_driver);
	clk_put(gm->spi.clk);
	iounmap(gm->spi.regs);
	release_resource(gm->spi.ioarea);
	kfree(gm->spi.ioarea);
	platform_set_drvdata(pdev, NULL);
	kfree(gm);

	return 0;
}

static struct platform_driver gfish_modem_driver = {
	.probe		= gm_probe,
	.remove		= gm_remove,
	.driver		= {
		.name		= "gfish-modem",
	},
};

static int __devinit gm_init(void)
{
	return platform_driver_register(&gfish_modem_driver);
}

static void gm_exit(void)
{
	platform_driver_unregister(&gfish_modem_driver);
}

module_init(gm_init);
module_exit(gm_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Harald Welte <laforge@gnumonks.org>");
MODULE_DESCRIPTION("E-TEN glofiish GSM/UMTS Modem driver");
