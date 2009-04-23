#ifndef SI470X_LIB_H_
#define SI470X_LIB_H_

/**************************************************************************
 * Register Definitions
 **************************************************************************/
#define RADIO_REGISTER_SIZE	2	/* 16 register bit width */
#define RADIO_REGISTER_NUM	16	/* DEVICEID   ... RDSD */
#define RDS_REGISTER_NUM	6	/* STATUSRSSI ... RDSD */

#define DEVICEID		0	/* Device ID */
#define DEVICEID_PN		0xf000	/* bits 15..12: Part Number */
#define DEVICEID_MFGID		0x0fff	/* bits 11..00: Manufacturer ID */

#define CHIPID			1	/* Chip ID */
#define CHIPID_REV		0xfc00	/* bits 15..10: Chip Version */
#define CHIPID_DEV		0x0200	/* bits 09..09: Device */
#define CHIPID_FIRMWARE		0x01ff	/* bits 08..00: Firmware Version */

#define POWERCFG		2	/* Power Configuration */
#define POWERCFG_DSMUTE		0x8000	/* bits 15..15: Softmute Disable */
#define POWERCFG_DMUTE		0x4000	/* bits 14..14: Mute Disable */
#define POWERCFG_MONO		0x2000	/* bits 13..13: Mono Select */
#define POWERCFG_RDSM		0x0800	/* bits 11..11: RDS Mode (Si4701 only) */
#define POWERCFG_SKMODE		0x0400	/* bits 10..10: Seek Mode */
#define POWERCFG_SEEKUP		0x0200	/* bits 09..09: Seek Direction */
#define POWERCFG_SEEK		0x0100	/* bits 08..08: Seek */
#define POWERCFG_DISABLE	0x0040	/* bits 06..06: Powerup Disable */
#define POWERCFG_ENABLE		0x0001	/* bits 00..00: Powerup Enable */

#define CHANNEL			3	/* Channel */
#define CHANNEL_TUNE		0x8000	/* bits 15..15: Tune */
#define CHANNEL_CHAN		0x03ff	/* bits 09..00: Channel Select */

#define SYSCONFIG1		4	/* System Configuration 1 */
#define SYSCONFIG1_RDSIEN	0x8000	/* bits 15..15: RDS Interrupt Enable (Si4701 only) */
#define SYSCONFIG1_STCIEN	0x4000	/* bits 14..14: Seek/Tune Complete Interrupt Enable */
#define SYSCONFIG1_RDS		0x1000	/* bits 12..12: RDS Enable (Si4701 only) */
#define SYSCONFIG1_DE		0x0800	/* bits 11..11: De-emphasis (0=75us 1=50us) */
#define SYSCONFIG1_AGCD		0x0400	/* bits 10..10: AGC Disable */
#define SYSCONFIG1_BLNDADJ	0x00c0	/* bits 07..06: Stereo/Mono Blend Level Adjustment */
#define SYSCONFIG1_GPIO3	0x0030	/* bits 05..04: General Purpose I/O 3 */
#define SYSCONFIG1_GPIO2	0x000c	/* bits 03..02: General Purpose I/O 2 */
#define SYSCONFIG1_GPIO1	0x0003	/* bits 01..00: General Purpose I/O 1 */

#define SYSCONFIG2		5	/* System Configuration 2 */
#define SYSCONFIG2_SEEKTH	0xff00	/* bits 15..08: RSSI Seek Threshold */
#define SYSCONFIG2_BAND		0x0080	/* bits 07..06: Band Select */
#define SYSCONFIG2_SPACE	0x0030	/* bits 05..04: Channel Spacing */
#define SYSCONFIG2_VOLUME	0x000f	/* bits 03..00: Volume */

#define SYSCONFIG3		6	/* System Configuration 3 */
#define SYSCONFIG3_SMUTER	0xc000	/* bits 15..14: Softmute Attack/Recover Rate */
#define SYSCONFIG3_SMUTEA	0x3000	/* bits 13..12: Softmute Attenuation */
#define SYSCONFIG3_SKSNR	0x00f0	/* bits 07..04: Seek SNR Threshold */
#define SYSCONFIG3_SKCNT	0x000f	/* bits 03..00: Seek FM Impulse Detection Threshold */

#define TEST1			7	/* Test 1 */
#define TEST1_AHIZEN		0x4000	/* bits 14..14: Audio High-Z Enable */

#define TEST2			8	/* Test 2 */
/* TEST2 only contains reserved bits */

#define BOOTCONFIG		9	/* Boot Configuration */
/* BOOTCONFIG only contains reserved bits */

#define STATUSRSSI		10	/* Status RSSI */
#define STATUSRSSI_RDSR		0x8000	/* bits 15..15: RDS Ready (Si4701 only) */
#define STATUSRSSI_STC		0x4000	/* bits 14..14: Seek/Tune Complete */
#define STATUSRSSI_SF		0x2000	/* bits 13..13: Seek Fail/Band Limit */
#define STATUSRSSI_AFCRL	0x1000	/* bits 12..12: AFC Rail */
#define STATUSRSSI_RDSS		0x0800	/* bits 11..11: RDS Synchronized (Si4701 only) */
#define STATUSRSSI_BLERA	0x0600	/* bits 10..09: RDS Block A Errors (Si4701 only) */
#define STATUSRSSI_ST		0x0100	/* bits 08..08: Stereo Indicator */
#define STATUSRSSI_RSSI		0x00ff	/* bits 07..00: RSSI (Received Signal Strength Indicator) */

#define READCHAN		11	/* Read Channel */
#define READCHAN_BLERB		0xc000	/* bits 15..14: RDS Block D Errors (Si4701 only) */
#define READCHAN_BLERC		0x3000	/* bits 13..12: RDS Block C Errors (Si4701 only) */
#define READCHAN_BLERD		0x0c00	/* bits 11..10: RDS Block B Errors (Si4701 only) */
#define READCHAN_READCHAN	0x03ff	/* bits 09..00: Read Channel */

#define RDSA			12	/* RDSA */
#define RDSA_RDSA		0xffff	/* bits 15..00: RDS Block A Data (Si4701 only) */

#define RDSB			13	/* RDSB */
#define RDSB_RDSB		0xffff	/* bits 15..00: RDS Block B Data (Si4701 only) */

#define RDSC			14	/* RDSC */
#define RDSC_RDSC		0xffff	/* bits 15..00: RDS Block C Data (Si4701 only) */

#define RDSD			15	/* RDSD */
#define RDSD_RDSD		0xffff	/* bits 15..00: RDS Block D Data (Si4701 only) */


/**************************************************************************
 * Software/Hardware Versions
 **************************************************************************/
#define RADIO_SW_VERSION_NOT_BOOTLOADABLE	6
#define RADIO_SW_VERSION			7
#define RADIO_SW_VERSION_CURRENT		15
#define RADIO_HW_VERSION			1

#define SCRATCH_PAGE_SW_VERSION	1
#define SCRATCH_PAGE_HW_VERSION	2

/**************************************************************************
 * LED State Definitions
 **************************************************************************/
#define LED_COMMAND		0x35

#define NO_CHANGE_LED		0x00
#define ALL_COLOR_LED		0x01	/* streaming state */
#define BLINK_GREEN_LED		0x02	/* connect state */
#define BLINK_RED_LED		0x04
#define BLINK_ORANGE_LED	0x10	/* disconnect state */
#define SOLID_GREEN_LED		0x20	/* tuning/seeking state */
#define SOLID_RED_LED		0x40	/* bootload state */
#define SOLID_ORANGE_LED	0x80

/**************************************************************************
 * Stream State Definitions
 **************************************************************************/
#define STREAM_COMMAND	0x36
#define STREAM_VIDPID	0x00
#define STREAM_AUDIO	0xff

/**************************************************************************
 * Bootloader / Flash Commands
 **************************************************************************/

/* unique id sent to bootloader and required to put into a bootload state */
#define UNIQUE_BL_ID		0x34

/* mask for the flash data */
#define FLASH_DATA_MASK		0x55

/* bootloader commands */
#define GET_SW_VERSION_COMMAND	0x00
#define	SET_PAGE_COMMAND	0x01
#define ERASE_PAGE_COMMAND	0x02
#define WRITE_PAGE_COMMAND	0x03
#define CRC_ON_PAGE_COMMAND	0x04
#define READ_FLASH_BYTE_COMMAND	0x05
#define RESET_DEVICE_COMMAND	0x06
#define GET_HW_VERSION_COMMAND	0x07
#define BLANK			0xff

/* bootloader command responses */
#define COMMAND_OK		0x01
#define COMMAND_FAILED		0x02
#define COMMAND_PENDING		0x03

/* buffer sizes */
#define COMMAND_BUFFER_SIZE	4
#define RESPONSE_BUFFER_SIZE	2
#define FLASH_BUFFER_SIZE	64
#define CRC_BUFFER_SIZE		3

struct si470x_device;

struct si470x_ops {
    int (*get_register)(struct si470x_device *radio, int regnr);
    int (*set_register)(struct si470x_device *radio, int regnr); 
    int (*get_all_registers)(struct si470x_device *radio);
    int (*get_rds_registers)(struct si470x_device *radio);
    int (*autopm_put)(struct si470x_device *radio);
    int (*autopm_get)(struct si470x_device *radio);
};

struct si470x_device {
    struct devce *dev;
    struct si470x_ops *ops;

    /* video device */
    struct video_device *videodev;

    /* driver management */
    unsigned int users;
    unsigned char disconnected;
    struct mutex disconnect_lock;

    /* Silabs internal registers (0..15) */
    unsigned short registers[RADIO_REGISTER_NUM];

    /* RDS receive buffer */
    struct delayed_work work;
    wait_queue_head_t read_queue;
    struct mutex lock; /* buffer locking */
    unsigned char *buffer; /* size is always multiple of three */
    unsigned int buf_size;
    unsigned int rd_index;
    unsigned int wr_index;
};

/*
 * The frequency is set in units of 62.5 Hz when using V4L2_TUNER_CAP_LOW,
 * 62.5 kHz otherwise.
 * The tuner is able to have a channel spacing of 50, 100 or 200 kHz.
 * tuner->capability is therefore set to V4L2_TUNER_CAP_LOW
 * The FREQ_MUL is then: 1 MHz / 62.5 Hz = 16000
 */
#define FREQ_MUL (1000000 / 62.5)

int si470xlib_probe(struct si470x_device *radio);
int si470xlib_suspend(struct si470x_device *radio);
int si470xlib_resume(struct si470x_device *radio);
void si470xlib_remove(struct si470x_device *radio);

#endif
