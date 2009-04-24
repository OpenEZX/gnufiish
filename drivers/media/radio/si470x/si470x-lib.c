/*
 *  drivers/media/radio/si470x/si470x-lib.c
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

#define DRIVER_AUTHOR "Tobias Lorenz <tobias.lorenz@gmx.net>"
#define DRIVER_NAME "si470x-lib"
#define DRIVER_KERNEL_VERSION KERNEL_VERSION(1, 0, 8)
#define DRIVER_CARD "Silicon Labs Si470x FM Radio Receiver"
#define DRIVER_DESC "USB radio driver for Si470x FM Radio Receivers"
#define DRIVER_VERSION "1.0.8"



/**************************************************************************
 * Module Parameters
 **************************************************************************/

/* Radio Nr */
static int radio_nr = -1;
module_param(radio_nr, int, 0444);
MODULE_PARM_DESC(radio_nr, "Radio Nr");

/* Spacing (kHz) */
/* 0: 200 kHz (USA, Australia) */
/* 1: 100 kHz (Europe, Japan) */
/* 2:  50 kHz */
static unsigned short space = 2;
module_param(space, ushort, 0444);
MODULE_PARM_DESC(space, "Spacing: 0=200kHz 1=100kHz *2=50kHz*");

/* Bottom of Band (MHz) */
/* 0: 87.5 - 108 MHz (USA, Europe)*/
/* 1: 76   - 108 MHz (Japan wide band) */
/* 2: 76   -  90 MHz (Japan) */
static unsigned short band = 1;
module_param(band, ushort, 0444);
MODULE_PARM_DESC(band, "Band: 0=87.5..108MHz *1=76..108MHz* 2=76..90MHz");

/* De-emphasis */
/* 0: 75 us (USA) */
/* 1: 50 us (Europe, Australia, Japan) */
static unsigned short de = 1;
module_param(de, ushort, 0444);
MODULE_PARM_DESC(de, "De-emphasis: 0=75us *1=50us*");

/* USB timeout */
static unsigned int usb_timeout = 500;
module_param(usb_timeout, uint, 0644);
MODULE_PARM_DESC(usb_timeout, "USB timeout (ms): *500*");

/* Tune timeout */
static unsigned int tune_timeout = 3000;
module_param(tune_timeout, uint, 0644);
MODULE_PARM_DESC(tune_timeout, "Tune timeout: *3000*");

/* Seek timeout */
static unsigned int seek_timeout = 5000;
module_param(seek_timeout, uint, 0644);
MODULE_PARM_DESC(seek_timeout, "Seek timeout: *5000*");

/* RDS buffer blocks */
static unsigned int rds_buf = 100;
module_param(rds_buf, uint, 0444);
MODULE_PARM_DESC(rds_buf, "RDS buffer entries: *100*");

/* RDS maximum block errors */
static unsigned short max_rds_errors = 1;
/* 0 means   0  errors requiring correction */
/* 1 means 1-2  errors requiring correction (used by original USBRadio.exe) */
/* 2 means 3-5  errors requiring correction */
/* 3 means   6+ errors or errors in checkword, correction not possible */
module_param(max_rds_errors, ushort, 0644);
MODULE_PARM_DESC(max_rds_errors, "RDS maximum block errors: *1*");

/* RDS poll frequency */
static unsigned int rds_poll_time = 40;
/* 40 is used by the original USBRadio.exe */
/* 50 is used by radio-cadet */
/* 75 should be okay */
/* 80 is the usual RDS receive interval */
module_param(rds_poll_time, uint, 0644);
MODULE_PARM_DESC(rds_poll_time, "RDS poll time (ms): *40*");



/*
 * si470x_set_chan - set the channel
 */
static int si470x_set_chan(struct si470x_device *radio, unsigned short chan)
{
	int retval;
	unsigned long timeout;
	bool timed_out = 0;

	/* start tuning */
	radio->registers[CHANNEL] &= ~CHANNEL_CHAN;
	radio->registers[CHANNEL] |= CHANNEL_TUNE | chan;
	retval = radio->ops->set_register(radio, CHANNEL);
	if (retval < 0)
		goto done;

	/* wait till tune operation has completed */
	timeout = jiffies + msecs_to_jiffies(tune_timeout);
	do {
		retval = radio->ops->get_register(radio, STATUSRSSI); 
		if (retval < 0)
			goto stop;
		timed_out = time_after(jiffies, timeout);
	} while (((radio->registers[STATUSRSSI] & STATUSRSSI_STC) == 0) &&
		(!timed_out));
	if ((radio->registers[STATUSRSSI] & STATUSRSSI_STC) == 0)
		printk(KERN_WARNING DRIVER_NAME ": tune does not complete\n");
	if (timed_out)
		printk(KERN_WARNING DRIVER_NAME
			": tune timed out after %u ms\n", tune_timeout);

stop:
	/* stop tuning */
	radio->registers[CHANNEL] &= ~CHANNEL_TUNE;
	retval = radio->ops->set_register(radio, CHANNEL);

done:
	return retval;
}


/*
 * si470x_get_freq - get the frequency
 */
static int si470x_get_freq(struct si470x_device *radio, unsigned int *freq)
{
	unsigned int spacing, band_bottom;
	unsigned short chan;
	int retval;

	/* Spacing (kHz) */
	switch ((radio->registers[SYSCONFIG2] & SYSCONFIG2_SPACE) >> 4) {
	/* 0: 200 kHz (USA, Australia) */
	case 0:
		spacing = 0.200 * FREQ_MUL; break;
	/* 1: 100 kHz (Europe, Japan) */
	case 1:
		spacing = 0.100 * FREQ_MUL; break;
	/* 2:  50 kHz */
	default:
		spacing = 0.050 * FREQ_MUL; break;
	};

	/* Bottom of Band (MHz) */
	switch ((radio->registers[SYSCONFIG2] & SYSCONFIG2_BAND) >> 6) {
	/* 0: 87.5 - 108 MHz (USA, Europe) */
	case 0:
		band_bottom = 87.5 * FREQ_MUL; break;
	/* 1: 76   - 108 MHz (Japan wide band) */
	default:
		band_bottom = 76   * FREQ_MUL; break;
	/* 2: 76   -  90 MHz (Japan) */
	case 2:
		band_bottom = 76   * FREQ_MUL; break;
	};

	/* read channel */
	retval = radio->ops->get_register(radio, READCHAN);
	chan = radio->registers[READCHAN] & READCHAN_READCHAN;

	/* Frequency (MHz) = Spacing (kHz) x Channel + Bottom of Band (MHz) */
	*freq = chan * spacing + band_bottom;

	return retval;
}


/*
 * si470x_set_freq - set the frequency
 */
static int si470x_set_freq(struct si470x_device *radio, unsigned int freq)
{
	unsigned int spacing, band_bottom;
	unsigned short chan;

	/* Spacing (kHz) */
	switch ((radio->registers[SYSCONFIG2] & SYSCONFIG2_SPACE) >> 4) {
	/* 0: 200 kHz (USA, Australia) */
	case 0:
		spacing = 0.200 * FREQ_MUL; break;
	/* 1: 100 kHz (Europe, Japan) */
	case 1:
		spacing = 0.100 * FREQ_MUL; break;
	/* 2:  50 kHz */
	default:
		spacing = 0.050 * FREQ_MUL; break;
	};

	/* Bottom of Band (MHz) */
	switch ((radio->registers[SYSCONFIG2] & SYSCONFIG2_BAND) >> 6) {
	/* 0: 87.5 - 108 MHz (USA, Europe) */
	case 0:
		band_bottom = 87.5 * FREQ_MUL; break;
	/* 1: 76   - 108 MHz (Japan wide band) */
	default:
		band_bottom = 76   * FREQ_MUL; break;
	/* 2: 76   -  90 MHz (Japan) */
	case 2:
		band_bottom = 76   * FREQ_MUL; break;
	};

	/* Chan = [ Freq (Mhz) - Bottom of Band (MHz) ] / Spacing (kHz) */
	chan = (freq - band_bottom) / spacing;

	return si470x_set_chan(radio, chan);
}


/*
 * si470x_set_seek - set seek
 */
static int si470x_set_seek(struct si470x_device *radio,
		unsigned int wrap_around, unsigned int seek_upward)
{
	int retval = 0;
	unsigned long timeout;
	bool timed_out = 0;

	/* start seeking */
	radio->registers[POWERCFG] |= POWERCFG_SEEK;
	if (wrap_around == 1)
		radio->registers[POWERCFG] &= ~POWERCFG_SKMODE;
	else
		radio->registers[POWERCFG] |= POWERCFG_SKMODE;
	if (seek_upward == 1)
		radio->registers[POWERCFG] |= POWERCFG_SEEKUP;
	else
		radio->registers[POWERCFG] &= ~POWERCFG_SEEKUP;
	retval = radio->ops->set_register(radio, POWERCFG);
	if (retval < 0)
		goto done;

	/* wait till seek operation has completed */
	timeout = jiffies + msecs_to_jiffies(seek_timeout);
	do {
		retval = radio->ops->get_register(radio, STATUSRSSI);
		if (retval < 0)
			goto stop;
		timed_out = time_after(jiffies, timeout);
	} while (((radio->registers[STATUSRSSI] & STATUSRSSI_STC) == 0) &&
		(!timed_out));
	if ((radio->registers[STATUSRSSI] & STATUSRSSI_STC) == 0)
		printk(KERN_WARNING DRIVER_NAME ": seek does not complete\n");
	if (radio->registers[STATUSRSSI] & STATUSRSSI_SF)
		printk(KERN_WARNING DRIVER_NAME
			": seek failed / band limit reached\n");
	if (timed_out)
		printk(KERN_WARNING DRIVER_NAME
			": seek timed out after %u ms\n", seek_timeout);

stop:
	/* stop seeking */
	radio->registers[POWERCFG] &= ~POWERCFG_SEEK;
	retval = radio->ops->set_register(radio, POWERCFG);

done:
	/* try again, if timed out */
	if ((retval == 0) && timed_out)
		retval = -EAGAIN;

	return retval;
}


/*
 * si470x_start - switch on radio
 */
static int si470x_start(struct si470x_device *radio)
{
	int retval;

	/* powercfg */
	radio->registers[POWERCFG] =
		POWERCFG_DMUTE | POWERCFG_ENABLE | POWERCFG_RDSM;
	retval = radio->ops->set_register(radio, POWERCFG);
	if (retval < 0)
		goto done;

	/* sysconfig 1 */
	radio->registers[SYSCONFIG1] = SYSCONFIG1_DE;
	retval = radio->ops->set_register(radio, SYSCONFIG1);
	if (retval < 0)
		goto done;

	/* sysconfig 2 */
	radio->registers[SYSCONFIG2] =
		(0x3f  << 8) |				/* SEEKTH */
		((band  << 6) & SYSCONFIG2_BAND)  |	/* BAND */
		((space << 4) & SYSCONFIG2_SPACE) |	/* SPACE */
		15;					/* VOLUME (max) */
	retval = radio->ops->set_register(radio, SYSCONFIG2);
	if (retval < 0)
		goto done;

	/* reset last channel */
	retval = si470x_set_chan(radio,
		radio->registers[CHANNEL] & CHANNEL_CHAN);

done:
	return retval;
}


/*
 * si470x_stop - switch off radio
 */
static int si470x_stop(struct si470x_device *radio)
{
	int retval;

	/* sysconfig 1 */
	radio->registers[SYSCONFIG1] &= ~SYSCONFIG1_RDS;
	retval = radio->ops->set_register(radio, SYSCONFIG1);
	if (retval < 0)
		goto done;

	/* powercfg */
	radio->registers[POWERCFG] &= ~POWERCFG_DMUTE;
	/* POWERCFG_ENABLE has to automatically go low */
	radio->registers[POWERCFG] |= POWERCFG_ENABLE |	POWERCFG_DISABLE;
	retval = radio->ops->set_register(radio, POWERCFG);

done:
	return retval;
}


/*
 * si470x_rds_on - switch on rds reception
 */
static int si470x_rds_on(struct si470x_device *radio)
{
	int retval;

	/* sysconfig 1 */
	mutex_lock(&radio->lock);
	radio->registers[SYSCONFIG1] |= SYSCONFIG1_RDS;
	retval = radio->ops->set_register(radio, SYSCONFIG1);
	if (retval < 0)
		radio->registers[SYSCONFIG1] &= ~SYSCONFIG1_RDS;
	mutex_unlock(&radio->lock);

	return retval;
}

/**************************************************************************
 * RDS Driver Functions
 **************************************************************************/

/*
 * si470x_rds - rds processing function
 */
static void si470x_rds(struct si470x_device *radio)
{
	unsigned char blocknum;
	unsigned short bler; /* rds block errors */
	unsigned short rds;
	unsigned char tmpbuf[3];

	/* get rds blocks */
	if (radio->ops->get_rds_registers(radio) < 0)
		return;
	if ((radio->registers[STATUSRSSI] & STATUSRSSI_RDSR) == 0) {
		/* No RDS group ready */
		return;
	}
	if ((radio->registers[STATUSRSSI] & STATUSRSSI_RDSS) == 0) {
		/* RDS decoder not synchronized */
		return;
	}

	/* copy all four RDS blocks to internal buffer */
	mutex_lock(&radio->lock);
	for (blocknum = 0; blocknum < 4; blocknum++) {
		switch (blocknum) {
		default:
			bler = (radio->registers[STATUSRSSI] &
					STATUSRSSI_BLERA) >> 9;
			rds = radio->registers[RDSA];
			break;
		case 1:
			bler = (radio->registers[READCHAN] &
					READCHAN_BLERB) >> 14;
			rds = radio->registers[RDSB];
			break;
		case 2:
			bler = (radio->registers[READCHAN] &
					READCHAN_BLERC) >> 12;
			rds = radio->registers[RDSC];
			break;
		case 3:
			bler = (radio->registers[READCHAN] &
					READCHAN_BLERD) >> 10;
			rds = radio->registers[RDSD];
			break;
		};

		/* Fill the V4L2 RDS buffer */
		put_unaligned_le16(rds, &tmpbuf);
		tmpbuf[2] = blocknum;		/* offset name */
		tmpbuf[2] |= blocknum << 3;	/* received offset */
		if (bler > max_rds_errors)
			tmpbuf[2] |= 0x80; /* uncorrectable errors */
		else if (bler > 0)
			tmpbuf[2] |= 0x40; /* corrected error(s) */

		/* copy RDS block to internal buffer */
		memcpy(&radio->buffer[radio->wr_index], &tmpbuf, 3);
		radio->wr_index += 3;

		/* wrap write pointer */
		if (radio->wr_index >= radio->buf_size)
			radio->wr_index = 0;

		/* check for overflow */
		if (radio->wr_index == radio->rd_index) {
			/* increment and wrap read pointer */
			radio->rd_index += 3;
			if (radio->rd_index >= radio->buf_size)
				radio->rd_index = 0;
		}
	}
	mutex_unlock(&radio->lock);

	/* wake up read queue */
	if (radio->wr_index != radio->rd_index)
		wake_up_interruptible(&radio->read_queue);
}


/*
 * si470x_work - rds work function
 */
static void si470x_work(struct work_struct *work)
{
	struct si470x_device *radio = container_of(work, struct si470x_device,
		work.work);

	/* safety checks */
	if (radio->disconnected)
		return;
	if ((radio->registers[SYSCONFIG1] & SYSCONFIG1_RDS) == 0)
		return;

	si470x_rds(radio);
	schedule_delayed_work(&radio->work, msecs_to_jiffies(rds_poll_time));
}



/**************************************************************************
 * File Operations Interface
 **************************************************************************/

/*
 * si470x_fops_read - read RDS data
 */
static ssize_t si470x_fops_read(struct file *file, char __user *buf,
		size_t count, loff_t *ppos)
{
	struct si470x_device *radio = video_drvdata(file);
	int retval = 0;
	unsigned int block_count = 0;

	/* switch on rds reception */
	if ((radio->registers[SYSCONFIG1] & SYSCONFIG1_RDS) == 0) {
		si470x_rds_on(radio);
		schedule_delayed_work(&radio->work,
			msecs_to_jiffies(rds_poll_time));
	}

	/* block if no new data available */
	while (radio->wr_index == radio->rd_index) {
		if (file->f_flags & O_NONBLOCK) {
			retval = -EWOULDBLOCK;
			goto done;
		}
		if (wait_event_interruptible(radio->read_queue,
			radio->wr_index != radio->rd_index) < 0) {
			retval = -EINTR;
			goto done;
		}
	}

	/* calculate block count from byte count */
	count /= 3;

	/* copy RDS block out of internal buffer and to user buffer */
	mutex_lock(&radio->lock);
	while (block_count < count) {
		if (radio->rd_index == radio->wr_index)
			break;

		/* always transfer rds complete blocks */
		if (copy_to_user(buf, &radio->buffer[radio->rd_index], 3))
			/* retval = -EFAULT; */
			break;

		/* increment and wrap read pointer */
		radio->rd_index += 3;
		if (radio->rd_index >= radio->buf_size)
			radio->rd_index = 0;

		/* increment counters */
		block_count++;
		buf += 3;
		retval += 3;
	}
	mutex_unlock(&radio->lock);

done:
	return retval;
}


/*
 * si470x_fops_poll - poll RDS data
 */
static unsigned int si470x_fops_poll(struct file *file,
		struct poll_table_struct *pts)
{
	struct si470x_device *radio = video_drvdata(file);
	int retval = 0;

	/* switch on rds reception */
	if ((radio->registers[SYSCONFIG1] & SYSCONFIG1_RDS) == 0) {
		si470x_rds_on(radio);
		schedule_delayed_work(&radio->work,
			msecs_to_jiffies(rds_poll_time));
	}

	poll_wait(file, &radio->read_queue, pts);

	if (radio->rd_index != radio->wr_index)
		retval = POLLIN | POLLRDNORM;

	return retval;
}


/*
 * si470x_fops_open - file open
 */
static int si470x_fops_open(struct inode *inode, struct file *file)
{
	struct si470x_device *radio = video_drvdata(file);
	int retval;

	lock_kernel();
	radio->users++;

    if(radio->ops->autopm_get) 
        retval = radio->ops->autopm_get(radio);

	if (retval < 0) {
		radio->users--;
		retval = -EIO;
		goto done;
	}

	if (radio->users == 1) {
		retval = si470x_start(radio);
		if (retval < 0 && radio->ops->autopm_put)
            radio->ops->autopm_put(radio);
	}

done:
	unlock_kernel();
	return retval;
}


/*
 * si470x_fops_release - file release
 */
static int si470x_fops_release(struct inode *inode, struct file *file)
{
	struct si470x_device *radio = video_drvdata(file);
	int retval = 0;

	/* safety check */
	if (!radio) {
		retval = -ENODEV;
		goto done;
	}

	mutex_lock(&radio->disconnect_lock);
	radio->users--;
	if (radio->users == 0) {
		if (radio->disconnected) {
			video_unregister_device(radio->videodev);
			kfree(radio->buffer);
			kfree(radio);
			goto unlock;
		}

		/* stop rds reception */
		cancel_delayed_work_sync(&radio->work);

		/* cancel read processes */
		wake_up_interruptible(&radio->read_queue);

		retval = si470x_stop(radio);

        if(radio->ops->autopm_put) 
            radio->ops->autopm_put(radio);
	}

unlock:
	mutex_unlock(&radio->disconnect_lock);

done:
	return retval;
}


/*
 * si470x_fops - file operations interface
 */
static const struct file_operations si470x_fops = {
	.owner		= THIS_MODULE,
	.llseek		= no_llseek,
	.read		= si470x_fops_read,
	.poll		= si470x_fops_poll,
	.ioctl		= video_ioctl2,
#ifdef CONFIG_COMPAT
	.compat_ioctl	= v4l_compat_ioctl32,
#endif
	.open		= si470x_fops_open,
	.release	= si470x_fops_release,
};

/**************************************************************************
 * Video4Linux Interface
 **************************************************************************/

/*
 * si470x_v4l2_queryctrl - query control
 */
static struct v4l2_queryctrl si470x_v4l2_queryctrl[] = {
	{
		.id		= V4L2_CID_AUDIO_VOLUME,
		.type		= V4L2_CTRL_TYPE_INTEGER,
		.name		= "Volume",
		.minimum	= 0,
		.maximum	= 15,
		.step		= 1,
		.default_value	= 15,
	},
	{
		.id		= V4L2_CID_AUDIO_MUTE,
		.type		= V4L2_CTRL_TYPE_BOOLEAN,
		.name		= "Mute",
		.minimum	= 0,
		.maximum	= 1,
		.step		= 1,
		.default_value	= 1,
	},
};


/*
 * si470x_vidioc_querycap - query device capabilities
 */
static int si470x_vidioc_querycap(struct file *file, void *priv,
		struct v4l2_capability *capability)
{
	strlcpy(capability->driver, DRIVER_NAME, sizeof(capability->driver));
	strlcpy(capability->card, DRIVER_CARD, sizeof(capability->card));
	sprintf(capability->bus_info, "USB");
	capability->version = DRIVER_KERNEL_VERSION;
	capability->capabilities = V4L2_CAP_HW_FREQ_SEEK |
		V4L2_CAP_TUNER | V4L2_CAP_RADIO;

	return 0;
}


/*
 * si470x_vidioc_queryctrl - enumerate control items
 */
static int si470x_vidioc_queryctrl(struct file *file, void *priv,
		struct v4l2_queryctrl *qc)
{
	unsigned char i = 0;
	int retval = -EINVAL;

	/* abort if qc->id is below V4L2_CID_BASE */
	if (qc->id < V4L2_CID_BASE)
		goto done;

	/* search video control */
	for (i = 0; i < ARRAY_SIZE(si470x_v4l2_queryctrl); i++) {
		if (qc->id == si470x_v4l2_queryctrl[i].id) {
			memcpy(qc, &(si470x_v4l2_queryctrl[i]), sizeof(*qc));
			retval = 0; /* found */
			break;
		}
	}

	/* disable unsupported base controls */
	/* to satisfy kradio and such apps */
	if ((retval == -EINVAL) && (qc->id < V4L2_CID_LASTP1)) {
		qc->flags = V4L2_CTRL_FLAG_DISABLED;
		retval = 0;
	}

done:
	if (retval < 0)
		printk(KERN_WARNING DRIVER_NAME
			": query controls failed with %d\n", retval);
	return retval;
}


/*
 * si470x_vidioc_g_ctrl - get the value of a control
 */
static int si470x_vidioc_g_ctrl(struct file *file, void *priv,
		struct v4l2_control *ctrl)
{
	struct si470x_device *radio = video_drvdata(file);
	int retval = 0;

	/* safety checks */
	if (radio->disconnected) {
		retval = -EIO;
		goto done;
	}

	switch (ctrl->id) {
	case V4L2_CID_AUDIO_VOLUME:
		ctrl->value = radio->registers[SYSCONFIG2] &
				SYSCONFIG2_VOLUME;
		break;
	case V4L2_CID_AUDIO_MUTE:
		ctrl->value = ((radio->registers[POWERCFG] &
				POWERCFG_DMUTE) == 0) ? 1 : 0;
		break;
	default:
		retval = -EINVAL;
	}

done:
	if (retval < 0)
		printk(KERN_WARNING DRIVER_NAME
			": get control failed with %d\n", retval);
	return retval;
}


/*
 * si470x_vidioc_s_ctrl - set the value of a control
 */
static int si470x_vidioc_s_ctrl(struct file *file, void *priv,
		struct v4l2_control *ctrl)
{
	struct si470x_device *radio = video_drvdata(file);
	int retval = 0;

	/* safety checks */
	if (radio->disconnected) {
		retval = -EIO;
		goto done;
	}

	switch (ctrl->id) {
	case V4L2_CID_AUDIO_VOLUME:
		radio->registers[SYSCONFIG2] &= ~SYSCONFIG2_VOLUME;
		radio->registers[SYSCONFIG2] |= ctrl->value;
		retval = radio->ops->set_register(radio, SYSCONFIG2);
		break;
	case V4L2_CID_AUDIO_MUTE:
		if (ctrl->value == 1)
			radio->registers[POWERCFG] &= ~POWERCFG_DMUTE;
		else
			radio->registers[POWERCFG] |= POWERCFG_DMUTE;
		retval = radio->ops->set_register(radio, POWERCFG);
		break;
	default:
		retval = -EINVAL;
	}

done:
	if (retval < 0)
		printk(KERN_WARNING DRIVER_NAME
			": set control failed with %d\n", retval);
	return retval;
}


/*
 * si470x_vidioc_g_audio - get audio attributes
 */
static int si470x_vidioc_g_audio(struct file *file, void *priv,
		struct v4l2_audio *audio)
{
	/* driver constants */
	audio->index = 0;
	strcpy(audio->name, "Radio");
	audio->capability = V4L2_AUDCAP_STEREO;
	audio->mode = 0;

	return 0;
}


/*
 * si470x_vidioc_g_tuner - get tuner attributes
 */
static int si470x_vidioc_g_tuner(struct file *file, void *priv,
		struct v4l2_tuner *tuner)
{
	struct si470x_device *radio = video_drvdata(file);
	int retval = 0;

	/* safety checks */
	if (radio->disconnected) {
		retval = -EIO;
		goto done;
	}
	if (tuner->index != 0) {
		retval = -EINVAL;
		goto done;
	}

	retval = radio->ops->get_register(radio, STATUSRSSI);
	if (retval < 0)
		goto done;

	/* driver constants */
	strcpy(tuner->name, "FM");
	tuner->type = V4L2_TUNER_RADIO;
	tuner->capability = V4L2_TUNER_CAP_LOW | V4L2_TUNER_CAP_STEREO;

	/* range limits */
	switch ((radio->registers[SYSCONFIG2] & SYSCONFIG2_BAND) >> 6) {
	/* 0: 87.5 - 108 MHz (USA, Europe, default) */
	default:
		tuner->rangelow  =  87.5 * FREQ_MUL;
		tuner->rangehigh = 108   * FREQ_MUL;
		break;
	/* 1: 76   - 108 MHz (Japan wide band) */
	case 1 :
		tuner->rangelow  =  76   * FREQ_MUL;
		tuner->rangehigh = 108   * FREQ_MUL;
		break;
	/* 2: 76   -  90 MHz (Japan) */
	case 2 :
		tuner->rangelow  =  76   * FREQ_MUL;
		tuner->rangehigh =  90   * FREQ_MUL;
		break;
	};

	/* stereo indicator == stereo (instead of mono) */
	if ((radio->registers[STATUSRSSI] & STATUSRSSI_ST) == 1)
		tuner->rxsubchans = V4L2_TUNER_SUB_MONO | V4L2_TUNER_SUB_STEREO;
	else
		tuner->rxsubchans = V4L2_TUNER_SUB_MONO;

	/* mono/stereo selector */
	if ((radio->registers[POWERCFG] & POWERCFG_MONO) == 1)
		tuner->audmode = V4L2_TUNER_MODE_MONO;
	else
		tuner->audmode = V4L2_TUNER_MODE_STEREO;

	/* min is worst, max is best; signal:0..0xffff; rssi: 0..0xff */
	tuner->signal = (radio->registers[STATUSRSSI] & STATUSRSSI_RSSI)
				* 0x0101;

	/* automatic frequency control: -1: freq to low, 1 freq to high */
	/* AFCRL does only indicate that freq. differs, not if too low/high */
	tuner->afc = (radio->registers[STATUSRSSI] & STATUSRSSI_AFCRL) ? 1 : 0;

done:
	if (retval < 0)
		printk(KERN_WARNING DRIVER_NAME
			": get tuner failed with %d\n", retval);
	return retval;
}


/*
 * si470x_vidioc_s_tuner - set tuner attributes
 */
static int si470x_vidioc_s_tuner(struct file *file, void *priv,
		struct v4l2_tuner *tuner)
{
	struct si470x_device *radio = video_drvdata(file);
	int retval = -EINVAL;

	/* safety checks */
	if (radio->disconnected) {
		retval = -EIO;
		goto done;
	}
	if (tuner->index != 0)
		goto done;

	/* mono/stereo selector */
	switch (tuner->audmode) {
	case V4L2_TUNER_MODE_MONO:
		radio->registers[POWERCFG] |= POWERCFG_MONO;  /* force mono */
		break;
	case V4L2_TUNER_MODE_STEREO:
		radio->registers[POWERCFG] &= ~POWERCFG_MONO; /* try stereo */
		break;
	default:
		goto done;
	}

	retval = radio->ops->set_register(radio, POWERCFG);

done:
	if (retval < 0)
		printk(KERN_WARNING DRIVER_NAME
			": set tuner failed with %d\n", retval);
	return retval;
}


/*
 * si470x_vidioc_g_frequency - get tuner or modulator radio frequency
 */
static int si470x_vidioc_g_frequency(struct file *file, void *priv,
		struct v4l2_frequency *freq)
{
	struct si470x_device *radio = video_drvdata(file);
	int retval = 0;

	/* safety checks */
	if (radio->disconnected) {
		retval = -EIO;
		goto done;
	}
	if (freq->tuner != 0) {
		retval = -EINVAL;
		goto done;
	}

	freq->type = V4L2_TUNER_RADIO;
	retval = si470x_get_freq(radio, &freq->frequency);

done:
	if (retval < 0)
		printk(KERN_WARNING DRIVER_NAME
			": get frequency failed with %d\n", retval);
	return retval;
}


/*
 * si470x_vidioc_s_frequency - set tuner or modulator radio frequency
 */
static int si470x_vidioc_s_frequency(struct file *file, void *priv,
		struct v4l2_frequency *freq)
{
	struct si470x_device *radio = video_drvdata(file);
	int retval = 0;

	/* safety checks */
	if (radio->disconnected) {
		retval = -EIO;
		goto done;
	}
	if (freq->tuner != 0) {
		retval = -EINVAL;
		goto done;
	}

	retval = si470x_set_freq(radio, freq->frequency);

done:
	if (retval < 0)
		printk(KERN_WARNING DRIVER_NAME
			": set frequency failed with %d\n", retval);
	return retval;
}


/*
 * si470x_vidioc_s_hw_freq_seek - set hardware frequency seek
 */
static int si470x_vidioc_s_hw_freq_seek(struct file *file, void *priv,
		struct v4l2_hw_freq_seek *seek)
{
	struct si470x_device *radio = video_drvdata(file);
	int retval = 0;

	/* safety checks */
	if (radio->disconnected) {
		retval = -EIO;
		goto done;
	}
	if (seek->tuner != 0) {
		retval = -EINVAL;
		goto done;
	}

	retval = si470x_set_seek(radio, seek->wrap_around, seek->seek_upward);

done:
	if (retval < 0)
		printk(KERN_WARNING DRIVER_NAME
			": set hardware frequency seek failed with %d\n",
			retval);
	return retval;
}


/*
 * si470x_ioctl_ops - video device ioctl operations
 */
static const struct v4l2_ioctl_ops si470x_ioctl_ops = {
	.vidioc_querycap	= si470x_vidioc_querycap,
	.vidioc_queryctrl	= si470x_vidioc_queryctrl,
	.vidioc_g_ctrl		= si470x_vidioc_g_ctrl,
	.vidioc_s_ctrl		= si470x_vidioc_s_ctrl,
	.vidioc_g_audio		= si470x_vidioc_g_audio,
	.vidioc_g_tuner		= si470x_vidioc_g_tuner,
	.vidioc_s_tuner		= si470x_vidioc_s_tuner,
	.vidioc_g_frequency	= si470x_vidioc_g_frequency,
	.vidioc_s_frequency	= si470x_vidioc_s_frequency,
	.vidioc_s_hw_freq_seek	= si470x_vidioc_s_hw_freq_seek,
};

/*
 * si470x_viddev_template - video device interface
 */
static struct video_device si470x_viddev_template = {
	.fops			= &si470x_fops,
	.name			= DRIVER_NAME,
	.release		= video_device_release,
	.ioctl_ops		= &si470x_ioctl_ops,
};

/*
 * Public functions
 */
int si470xlib_probe(struct si470x_device *radio) 
{
	int retval = 0;

	radio->users = 0;
	radio->disconnected = 0;
	mutex_init(&radio->disconnect_lock);
	mutex_init(&radio->lock);

	/* video device allocation and initialization */
	radio->videodev = video_device_alloc();
	if (!radio->videodev) {
		retval = -ENOMEM;
		goto err_radio;
	}
	memcpy(radio->videodev, &si470x_viddev_template,
			sizeof(si470x_viddev_template));
	video_set_drvdata(radio->videodev, radio);

	/* show some infos about the specific device */
	if (radio->ops->get_all_registers(radio) < 0) {
		retval = -EIO;
		goto err_all;
	}
	printk(KERN_INFO DRIVER_NAME ": DeviceID=0x%4.4hx ChipID=0x%4.4hx\n",
			radio->registers[DEVICEID], radio->registers[CHIPID]);

	/* check if firmware is current */
	if ((radio->registers[CHIPID] & CHIPID_FIRMWARE)
			< RADIO_SW_VERSION_CURRENT) {
		printk(KERN_WARNING DRIVER_NAME
			": This driver is known to work with "
			"firmware version %hu,\n", RADIO_SW_VERSION_CURRENT);
		printk(KERN_WARNING DRIVER_NAME
			": but the device has firmware version %hu.\n",
			radio->registers[CHIPID] & CHIPID_FIRMWARE);
		printk(KERN_WARNING DRIVER_NAME
			": If you have some trouble using this driver,\n");
		printk(KERN_WARNING DRIVER_NAME
			": please report to V4L ML at "
			"video4linux-list@redhat.com\n");
	}

	/* set initial frequency */
	si470x_set_freq(radio, 87.5 * FREQ_MUL); /* available in all regions */

	/* rds buffer allocation */
	radio->buf_size = rds_buf * 3;
	radio->buffer = kmalloc(radio->buf_size, GFP_KERNEL);
	if (!radio->buffer) {
		retval = -EIO;
		goto err_all;
	}

	/* rds buffer configuration */
	radio->wr_index = 0;
	radio->rd_index = 0;
	init_waitqueue_head(&radio->read_queue);

	/* prepare rds work function */
	INIT_DELAYED_WORK(&radio->work, si470x_work);

	/* register video device */
	retval = video_register_device(radio->videodev, VFL_TYPE_RADIO, radio_nr);
	if (retval) {
		printk(KERN_WARNING DRIVER_NAME
				": Could not register video device\n");
		goto err_all;
	}

	return 0;
err_all:
	video_device_release(radio->videodev);
	kfree(radio->buffer);
err_radio:
	kfree(radio);
err_initial:
	return retval;
}
EXPORT_SYMBOL_GPL(si470xlib_probe);

int si470xlib_suspend(struct si470x_device *radio)
{
    cancel_delayed_work_sync(&radio->work);
    return 0;
}
EXPORT_SYMBOL_GPL(si470xlib_suspend);

int si470xlib_resume(struct si470x_device *radio)
{
    mutex_lock(&radio->lock);
	if (radio->users && radio->registers[SYSCONFIG1] & SYSCONFIG1_RDS)
		schedule_delayed_work(&radio->work,
			msecs_to_jiffies(rds_poll_time));
	mutex_unlock(&radio->lock);

    return 0;
}
EXPORT_SYMBOL_GPL(si470xlib_resume);

void si470xlib_remove(struct si470x_device *radio)
{
    mutex_lock(&radio->disconnect_lock);
	radio->disconnected = 1;
	cancel_delayed_work_sync(&radio->work);
	if (radio->users == 0) {
		video_unregister_device(radio->videodev);
		kfree(radio->buffer);
		kfree(radio);
	}
	mutex_unlock(&radio->disconnect_lock);
}
EXPORT_SYMBOL_GPL(si470xlib_remove);

MODULE_LICENSE("GPL");
MODULE_AUTHOR(DRIVER_AUTHOR);
MODULE_DESCRIPTION(DRIVER_DESC);
MODULE_VERSION(DRIVER_VERSION);
