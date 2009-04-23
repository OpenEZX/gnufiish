/*
 *  drivers/media/radio/si470x/si470x-usb.c
 *
 *  Driver for USB radios for the Silicon Labs Si470x FM Radio Receivers:
 *   - Silicon Labs USB FM Radio Reference Design
 *   - ADS/Tech FM Radio Receiver (formerly Instant FM Music) (RDX-155-EF)
 *
 *  Copyright (c) 2008 Tobias Lorenz <tobias.lorenz@gmx.net>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA
 */


/*
 * User Notes:
 * - USB Audio is provided by the alsa snd_usb_audio module.
 *   For listing you have to redirect the sound, for example using:
 *   arecord -D hw:1,0 -r96000 -c2 -f S16_LE | artsdsp aplay -B -
 * - regarding module parameters in /sys/module/radio_si470x/parameters:
 *   the contents of read-only files (0444) are not updated, even if
 *   space, band and de are changed using private video controls
 * - increase tune_timeout, if you often get -EIO errors
 * - hw_freq_seek returns -EAGAIN, when timed out or band limit is reached
 */


/*
 * History:
 * 2008-01-12	Tobias Lorenz <tobias.lorenz@gmx.net>
 *		Version 1.0.0
 *		- First working version
 * 2008-01-13   Tobias Lorenz <tobias.lorenz@gmx.net>
 *		Version 1.0.1
 *		- Improved error handling, every function now returns errno
 *		- Improved multi user access (start/mute/stop)
 *		- Channel doesn't get lost anymore after start/mute/stop
 *		- RDS support added (polling mode via interrupt EP 1)
 *		- marked default module parameters with *value*
 *		- switched from bit structs to bit masks
 *		- header file cleaned and integrated
 * 2008-01-14	Tobias Lorenz <tobias.lorenz@gmx.net>
 * 		Version 1.0.2
 * 		- hex values are now lower case
 * 		- commented USB ID for ADS/Tech moved on todo list
 * 		- blacklisted si470x in hid-quirks.c
 * 		- rds buffer handling functions integrated into *_work, *_read
 * 		- rds_command in si470x_poll exchanged against simple retval
 * 		- check for firmware version 15
 * 		- code order and prototypes still remain the same
 * 		- spacing and bottom of band codes remain the same
 * 2008-01-16	Tobias Lorenz <tobias.lorenz@gmx.net>
 *		Version 1.0.3
 * 		- code reordered to avoid function prototypes
 *		- switch/case defaults are now more user-friendly
 *		- unified comment style
 *		- applied all checkpatch.pl v1.12 suggestions
 *		  except the warning about the too long lines with bit comments
 *		- renamed FMRADIO to RADIO to cut line length (checkpatch.pl)
 * 2008-01-22	Tobias Lorenz <tobias.lorenz@gmx.net>
 *		Version 1.0.4
 *		- avoid poss. locking when doing copy_to_user which may sleep
 *		- RDS is automatically activated on read now
 *		- code cleaned of unnecessary rds_commands
 *		- USB Vendor/Product ID for ADS/Tech FM Radio Receiver verified
 *		  (thanks to Guillaume RAMOUSSE)
 * 2008-01-27	Tobias Lorenz <tobias.lorenz@gmx.net>
 *		Version 1.0.5
 *		- number of seek_retries changed to tune_timeout
 *		- fixed problem with incomplete tune operations by own buffers
 *		- optimization of variables and printf types
 *		- improved error logging
 * 2008-01-31	Tobias Lorenz <tobias.lorenz@gmx.net>
 *		Oliver Neukum <oliver@neukum.org>
 *		Version 1.0.6
 *		- fixed coverity checker warnings in *_usb_driver_disconnect
 *		- probe()/open() race by correct ordering in probe()
 *		- DMA coherency rules by separate allocation of all buffers
 *		- use of endianness macros
 *		- abuse of spinlock, replaced by mutex
 *		- racy handling of timer in disconnect,
 *		  replaced by delayed_work
 *		- racy interruptible_sleep_on(),
 *		  replaced with wait_event_interruptible()
 *		- handle signals in read()
 * 2008-02-08	Tobias Lorenz <tobias.lorenz@gmx.net>
 *		Oliver Neukum <oliver@neukum.org>
 *		Version 1.0.7
 *		- usb autosuspend support
 *		- unplugging fixed
 * 2008-05-07	Tobias Lorenz <tobias.lorenz@gmx.net>
 *		Version 1.0.8
 *		- hardware frequency seek support
 *		- afc indication
 *		- more safety checks, let si470x_get_freq return errno
 *		- vidioc behavior corrected according to v4l2 spec
 *
 * ToDo:
 * - add firmware download/update support
 * - RDS support: interrupt mode, instead of polling
 * - add LED status output (check if that's not already done in firmware)
 */



/* driver definitions */
#define DRIVER_AUTHOR "Tobias Lorenz <tobias.lorenz@gmx.net>"
#define DRIVER_NAME "radio-si470x"
#define DRIVER_KERNEL_VERSION KERNEL_VERSION(1, 0, 8)
#define DRIVER_CARD "Silicon Labs Si470x FM Radio Receiver"
#define DRIVER_DESC "USB radio driver for Si470x FM Radio Receivers"
#define DRIVER_VERSION "1.0.8"


/* kernel includes */
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/input.h>
#include <linux/usb.h>
#include <linux/hid.h>
#include <linux/version.h>
#include <linux/videodev2.h>
#include <linux/mutex.h>
#include <media/v4l2-common.h>
#include <media/v4l2-ioctl.h>
#include <media/rds.h>
#include <asm/unaligned.h>

#include "si470x-lib.h"

#define si470x_to_usb_device(__dev) ((struct usb_device*)(__dev))


/**************************************************************************
 * USB HID Reports
 **************************************************************************/

/* Reports 1-16 give direct read/write access to the 16 Si470x registers */
/* with the (REPORT_ID - 1) corresponding to the register address across USB */
/* endpoint 0 using GET_REPORT and SET_REPORT */
#define REGISTER_REPORT_SIZE	(RADIO_REGISTER_SIZE + 1)
#define REGISTER_REPORT(reg)	((reg) + 1)

/* Report 17 gives direct read/write access to the entire Si470x register */
/* map across endpoint 0 using GET_REPORT and SET_REPORT */
#define ENTIRE_REPORT_SIZE	(RADIO_REGISTER_NUM * RADIO_REGISTER_SIZE + 1)
#define ENTIRE_REPORT		17

/* Report 18 is used to send the lowest 6 Si470x registers up the HID */
/* interrupt endpoint 1 to Windows every 20 milliseconds for status */
#define RDS_REPORT_SIZE		(RDS_REGISTER_NUM * RADIO_REGISTER_SIZE + 1)
#define RDS_REPORT		18

/* Report 19: LED state */
#define LED_REPORT_SIZE		3
#define LED_REPORT		19

/* Report 19: stream */
#define STREAM_REPORT_SIZE	3
#define	STREAM_REPORT		19

/* Report 20: scratch */
#define SCRATCH_PAGE_SIZE	63
#define SCRATCH_REPORT_SIZE	(SCRATCH_PAGE_SIZE + 1)
#define SCRATCH_REPORT		20

/* Reports 19-22: flash upgrade of the C8051F321 */
#define WRITE_REPORT		19
#define FLASH_REPORT		20
#define CRC_REPORT		21
#define RESPONSE_REPORT		22

/* Report 23: currently unused, but can accept 60 byte reports on the HID */
/* interrupt out endpoint 2 every 1 millisecond */
#define UNUSED_REPORT		23



/* USB timeout */
static unsigned int usb_timeout = 500;
module_param(usb_timeout, uint, 0644);
MODULE_PARM_DESC(usb_timeout, "USB timeout (ms): *500*");

/* USB Device ID List */
static struct usb_device_id si470x_usb_driver_id_table[] = {
	/* Silicon Labs USB FM Radio Reference Design */
	{ USB_DEVICE_AND_INTERFACE_INFO(0x10c4, 0x818a, USB_CLASS_HID, 0, 0) },
	/* ADS/Tech FM Radio Receiver (formerly Instant FM Music) */
	{ USB_DEVICE_AND_INTERFACE_INFO(0x06e1, 0xa155, USB_CLASS_HID, 0, 0) },
	/* Terminating entry */
	{ }
};
MODULE_DEVICE_TABLE(usb, si470x_usb_driver_id_table);

/**************************************************************************
 * USB Interface
 **************************************************************************/

/*
 * si470x_get_report - receive a HID report
 */
static int si470x_usb_get_report(struct si470x_device *radio, void *buf, int size)
{
	unsigned char *report = (unsigned char *) buf;
	int retval;

	retval = usb_control_msg(si470x_to_usb_device(radio->dev),
		usb_rcvctrlpipe(si470x_to_usb_device(radio->dev), 0),
		HID_REQ_GET_REPORT,
		USB_TYPE_CLASS | USB_RECIP_INTERFACE | USB_DIR_IN,
		report[0], 2,
		buf, size, usb_timeout);

	if (retval < 0)
		printk(KERN_WARNING DRIVER_NAME
			": si470x_get_report: usb_control_msg returned %d\n",
			retval);
	return retval;
}


/*
 * si470x_set_report - send a HID report
 */
static int si470x_usb_set_report(struct si470x_device *radio, void *buf, int size)
{
	unsigned char *report = (unsigned char *) buf;
	int retval;

	retval = usb_control_msg(si470x_to_usb_device(radio->dev),
		usb_sndctrlpipe(si470x_to_usb_device(radio->dev), 0),
		HID_REQ_SET_REPORT,
		USB_TYPE_CLASS | USB_RECIP_INTERFACE | USB_DIR_OUT,
		report[0], 2,
		buf, size, usb_timeout);

	if (retval < 0)
		printk(KERN_WARNING DRIVER_NAME
			": si470x_usb_set_report: usb_control_msg returned %d\n",
			retval);
	return retval;
}


/*
 * si470x_get_register - read register
 */
static int si470x_usb_get_register(struct si470x_device *radio, int regnr)
{
	unsigned char buf[REGISTER_REPORT_SIZE];
	int retval;

	buf[0] = REGISTER_REPORT(regnr);

	retval = si470x_usb_get_report(radio, (void *) &buf, sizeof(buf));

	if (retval >= 0)
		radio->registers[regnr] = get_unaligned_be16(&buf[1]);

	return (retval < 0) ? -EINVAL : 0;
}

/*
 * si470x_set_register - write register
 */
static int si470x_usb_set_register(struct si470x_device *radio, int regnr)
{
	unsigned char buf[REGISTER_REPORT_SIZE];
	int retval;

	buf[0] = REGISTER_REPORT(regnr);
	put_unaligned_be16(radio->registers[regnr], &buf[1]);

	retval = si470x_usb_set_report(radio, (void *) &buf, sizeof(buf));

	return (retval < 0) ? -EINVAL : 0;
}

/*
 * si470x_get_all_registers - read entire registers
 */
static int si470x_usb_get_all_registers(struct si470x_device *radio)
{
	unsigned char buf[ENTIRE_REPORT_SIZE];
	int retval;
	unsigned char regnr;

	buf[0] = ENTIRE_REPORT;

	retval = si470x_usb_get_report(radio, (void *) &buf, sizeof(buf));

	if (retval >= 0)
		for (regnr = 0; regnr < RADIO_REGISTER_NUM; regnr++)
			radio->registers[regnr] = get_unaligned_be16(
				&buf[regnr * RADIO_REGISTER_SIZE + 1]);

	return (retval < 0) ? -EINVAL : 0;
}


/*
 * si470x_get_rds_registers - read rds registers
 */
static int si470x_usb_get_rds_registers(struct si470x_device *radio)
{
	unsigned char buf[RDS_REPORT_SIZE];
	int retval;
	int size;
	unsigned char regnr;

	buf[0] = RDS_REPORT;

	retval = usb_interrupt_msg(si470x_to_usb_device(radio->dev),
		usb_rcvintpipe(si470x_to_usb_device(radio->dev), 1),
		(void *) &buf, sizeof(buf), &size, usb_timeout);
	if (size != sizeof(buf))
		printk(KERN_WARNING DRIVER_NAME ": si470x_get_rds_registers: "
			"return size differs: %d != %zu\n", size, sizeof(buf));
	if (retval < 0)
		printk(KERN_WARNING DRIVER_NAME ": si470x_get_rds_registers: "
			"usb_interrupt_msg returned %d\n", retval);

	if (retval >= 0)
		for (regnr = 0; regnr < RDS_REGISTER_NUM; regnr++)
			radio->registers[STATUSRSSI + regnr] =
				get_unaligned_be16(
				&buf[regnr * RADIO_REGISTER_SIZE + 1]);

	return (retval < 0) ? -EINVAL : 0;
}


static int si470x_usb_autopm_put(struct si470x_device *radio)
{
    usb_autopm_put_interface(to_usb_interface(si470x_to_usb_device(radio->dev)));
    return 0;
}

static int si470x_usb_autopm_get(struct si470x_device *radio)
{
    usb_autopm_get_interface(to_usb_interface(si470x_to_usb_device(radio->dev)));
    return 0;
}

struct si470x_ops si470x_usb_ops = {
    .get_register = si470x_usb_get_register,
    .set_register = si470x_usb_set_register,
    .get_all_registers = si470x_usb_get_all_registers,
    .get_rds_registers = si470x_usb_get_rds_registers,
    .autopm_put = si470x_usb_autopm_put,
    .autopm_get = si470x_usb_autopm_get
};

/*
 * si470x_usb_driver_probe - probe for the device
 */
static int si470x_usb_driver_probe(struct usb_interface *intf,
		const struct usb_device_id *id)
{
    int retval;
	struct si470x_device *radio;
    struct usb_device *dev;

    dev = interface_to_usbdev(intf);
    
    /* private data allocation and initialization */
	radio = kzalloc(sizeof(struct si470x_device), GFP_KERNEL);
	if (!radio) {
		retval = -ENOMEM;
		goto err_initial;
	}

    radio->ops = &si470x_usb_ops;
    radio->dev = dev;

    /* Connect to the si470x chip */
    retval = si470xlib_probe(radio);
    
    usb_set_intfdata(intf, radio);

    return 0;

err_initial:
	return retval;
}


/*
 * si470x_usb_driver_suspend - suspend the device
 */
static int si470x_usb_driver_suspend(struct usb_interface *intf,
		pm_message_t message)
{
	struct si470x_device *radio = usb_get_intfdata(intf);
    
    si470xlib_suspend(radio);

	return 0;
}


/*
 * si470x_usb_driver_resume - resume the device
 */
static int si470x_usb_driver_resume(struct usb_interface *intf)
{
	struct si470x_device *radio = usb_get_intfdata(intf);
    
    si470xlib_resume(radio);

	return 0;
}


/*
 * si470x_usb_driver_disconnect - disconnect the device
 */
static void si470x_usb_driver_disconnect(struct usb_interface *intf)
{
	struct si470x_device *radio = usb_get_intfdata(intf);
    
    si470xlib_remove(radio);
	usb_set_intfdata(intf, NULL);
}

/*
 * si470x_usb_driver - usb driver interface
 */
static struct usb_driver si470x_usb_driver = {
	.name			= DRIVER_NAME,
	.probe			= si470x_usb_driver_probe,
	.disconnect		= si470x_usb_driver_disconnect,
	.suspend		= si470x_usb_driver_suspend,
	.resume			= si470x_usb_driver_resume,
	.id_table		= si470x_usb_driver_id_table,
	.supports_autosuspend	= 1,
};



/**************************************************************************
 * Module Interface
 **************************************************************************/

/*
 * si470x_module_init - module init
 */
static int __init si470x_usb_module_init(void)
{
	printk(KERN_INFO DRIVER_DESC ", Version " DRIVER_VERSION "\n");
	return usb_register(&si470x_usb_driver);
}


/*
 * si470x_module_exit - module exit
 */
static void __exit si470x_usb_module_exit(void)
{
	usb_deregister(&si470x_usb_driver);
}

module_init(si470x_usb_module_init);
module_exit(si470x_usb_module_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR(DRIVER_AUTHOR);
MODULE_DESCRIPTION(DRIVER_DESC);
MODULE_VERSION(DRIVER_VERSION);
