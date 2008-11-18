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

#include <linux/kernel.h>
#include <linux/types.h>
#include <linux/interrupt.h>
#include <linux/tty.h>
#include <linux/tty_flip.h>
#include <linux/serial_core.h>
#include <linux/serial.h>
#include <linux/clk.h>
#include <linux/platform_device.h>

#include <asm/io.h>
#include <asm/irq.h>
#include <asm/dma.h>

#include <asm/plat-s3c24xx/regs-spi.h>
#include <asm/arch/dma.h>
#include <asm/arch/regs-gpio.h>
#include <asm/arch/regs-clock.h>
#include <asm/arch/glofiish.h>

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
	unsigned int spsta = readb(gm->spi.regs + S3C2410_SPSTA);

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
	s3c2410_gpio_setpin(gm->gpio_req, 0);
}

static void gfish_modem_start_tx(struct uart_port *port)
{
	struct gfish_modem *gm = port_to_modem(port);

	dev_dbg(&gm->pdev->dev, "start_tx\n");
	s3c2410_gpio_setpin(gm->gpio_req, 1);
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

static int s3c24xx_spi_slave_init(struct gfish_modem *gf)
{
	dev_dbg(&gf->pdev->dev, "slave_init()\n");

	/* set upt the GPIO config */
	s3c2410_gpio_cfgpin(S3C2410_GPD8, S3C2440_GPD8_nSPIMISO1);
	s3c2410_gpio_cfgpin(S3C2410_GPD9, S3C2440_GPD9_nSPIMOSI1);
	s3c2410_gpio_cfgpin(S3C2410_GPD10, S3C2440_GPD10_nSPICLK1);
	s3c2410_gpio_cfgpin(S3C2410_GPG3, S3C2440_GPG3_nSS1);

	writeb(0x00, gf->spi.regs + S3C2410_SPCON);
	//writeb(S3C2410_SPPIN_KEEP, gf->spi.regs + S3C2410_SPPIN);

	return 0;
}

/* This interrupt is called by the SPI controller */
static irqreturn_t s3c24xx_spi_irq(int irq, void *dev)
{
	struct gfish_modem *gm = dev;
	unsigned int spsta = readb(gm->spi.regs + S3C2410_SPSTA);
	unsigned char ch, tx_ch;
	struct uart_port *port = &gm->port;
	struct circ_buf *xmit = NULL;
	struct tty_struct *tty = NULL;

	dev_dbg(&gm->pdev->dev, "SPI IRQ\n");

	if (port && port->info) {
 		xmit = &port->info->xmit;
 		if (port->info->tty)
			tty = port->info->tty;
		else
			dev_dbg(&gm->pdev->dev, "no port->info->tty\n");
	} else
		dev_dbg(&gm->pdev->dev, "no port->info\n");

	if (spsta & S3C2410_SPSTA_DCOL) {
		dev_dbg(&gm->pdev->dev, "data-collision\n");
		goto irq_done;
	}

	if (!(spsta & S3C2410_SPSTA_READY)) {
		dev_dbg(&gm->pdev->dev, "spi not ready for tx?\n");
		goto irq_done;
	}

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

	if (tty)
		tty_flip_buffer_push(tty);

	dev_dbg(&gm->pdev->dev, "Rx: 0x%x Tx: 0x%x\n", ch, tx_ch);

irq_done:
	return IRQ_HANDLED;
}

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

/* This interrupt is called every every time the modem asserts nSS1
 * and thereby activates the SPI transfer with the application processor */
static irqreturn_t gfish_modem_irq(int irq, void *dev)
{
	struct gfish_modem *gm = dev;
	int level = s3c2410_gpio_getpin(S3C2410_GPG8);

	dev_dbg(&gm->pdev->dev, "Modem nSS1 IRQ: %d\n", level ? 1 : 0);

	/* Once this interrupt is called, we have to keep transmitting
	 * and receiving octets until the SPI master de-asserts nSS1 */

	return IRQ_HANDLED;
}

static struct s3c2410_dma_client gm_dma_client = {
	.name	= "gfish-modem",
};

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

	/* de-assert the GSM SPI request */
	s3c2410_gpio_setpin(gm->gpio_req, 0);

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

#ifdef USE_DMA
	/* FIXME */
	s3c2410_dma_xxx(gm->spi.dmach, &gm_dma_client);
	{
		goto out_irq2;
	}
#endif

	s3c24xx_spi_slave_init(gm);

	return rc;

out_dma:
#ifdef USE_DMA
	s3c2410_dma_free(gm->spi.dmach, &gm_dma_client);
#endif
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

	/* de-assert the GSM SPI request */
	s3c2410_gpio_setpin(gm->gpio_req, 0);

#ifdef USE_DMA
	s3c2410_dma_free(gm->spi.dmach, &gm_dma_client);
#endif
	free_irq(gm->spi.irq, gm);
	disable_irq_wake(gm->irq);
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
