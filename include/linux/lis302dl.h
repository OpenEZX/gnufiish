#ifndef _LINUX_LIS302DL_H
#define _LINUX_LIS302DL_H

#include <linux/types.h>
#include <linux/spi/spi.h>
#include <linux/input.h>


struct lis302dl_info;

struct lis302dl_platform_data {
	char *name;
	unsigned long pin_chip_select;
	unsigned long pin_clk;
	unsigned long pin_mosi;
	unsigned long pin_miso;
	int open_drain;
	void (*lis302dl_bitbang_read)(struct lis302dl_info *);
};

struct lis302dl_info {
	struct lis302dl_platform_data *pdata;
	struct spi_device *spi_dev;
	struct input_dev *input_dev;
	struct mutex lock;
	unsigned int flags;
	u_int8_t regs[0x40];
};

enum lis302dl_reg {
	LIS302DL_REG_WHO_AM_I		= 0x0f,
	LIS302DL_REG_CTRL1		= 0x20,
	LIS302DL_REG_CTRL2		= 0x21,
	LIS302DL_REG_CTRL3		= 0x22,
	LIS302DL_REG_HP_FILTER_RESET	= 0x23,
	LIS302DL_REG_STATUS		= 0x27,
	LIS302DL_REG_OUT_X		= 0x29,
	LIS302DL_REG_OUT_Y		= 0x2b,
	LIS302DL_REG_OUT_Z		= 0x2d,
	LIS302DL_REG_FF_WU_CFG_1	= 0x30,
	LIS302DL_REG_FF_WU_SRC_1	= 0x31,
	LIS302DL_REG_FF_WU_THS_1	= 0x32,
	LIS302DL_REG_FF_WU_DURATION_1	= 0x33,
	LIS302DL_REG_FF_WU_CFG_2	= 0x34,
	LIS302DL_REG_FF_WU_SRC_2	= 0x35,
	LIS302DL_REG_FF_WU_THS_2	= 0x36,
	LIS302DL_REG_FF_WU_DURATION_2	= 0x37,
	LIS302DL_REG_CLICK_CFG		= 0x38,
	LIS302DL_REG_CLICK_SRC		= 0x39,
	LIS302DL_REG_CLICK_THSY_X	= 0x3b,
	LIS302DL_REG_CLICK_THSZ		= 0x3c,
	LIS302DL_REG_CLICK_TIME_LIMIT	= 0x3d,
	LIS302DL_REG_CLICK_LATENCY	= 0x3e,
	LIS302DL_REG_CLICK_WINDOW	= 0x3f,
};

enum lis302dl_reg_ctrl1 {
	LIS302DL_CTRL1_Xen		= 0x01,
	LIS302DL_CTRL1_Yen		= 0x02,
	LIS302DL_CTRL1_Zen		= 0x04,
	LIS302DL_CTRL1_STM		= 0x08,
	LIS302DL_CTRL1_STP		= 0x10,
	LIS302DL_CTRL1_FS		= 0x20,
	LIS302DL_CTRL1_PD		= 0x40,
	LIS302DL_CTRL1_DR		= 0x80,
};

enum lis302dl_reg_ctrl3 {
	LIS302DL_CTRL3_PP_OD		= 0x40,
	LIS302DL_CTRL3_IHL		= 0x80,
};

enum lis302dl_reg_status {
	LIS302DL_STATUS_XDA		= 0x01,
	LIS302DL_STATUS_YDA		= 0x02,
	LIS302DL_STATUS_ZDA		= 0x04,
	LIS302DL_STATUS_XYZDA		= 0x08,
	LIS302DL_STATUS_XOR		= 0x10,
	LIS302DL_STATUS_YOR		= 0x20,
	LIS302DL_STATUS_ZOR		= 0x40,
	LIS302DL_STATUS_XYZOR		= 0x80,
};

enum lis302dl_reg_ffwusrc1 {
	LIS302DL_FFWUSRC1_XL		= 0x01,
	LIS302DL_FFWUSRC1_XH		= 0x02,
	LIS302DL_FFWUSRC1_YL		= 0x04,
	LIS302DL_FFWUSRC1_YH		= 0x08,
	LIS302DL_FFWUSRC1_ZL		= 0x10,
	LIS302DL_FFWUSRC1_ZH		= 0x20,
	LIS302DL_FFWUSRC1_IA		= 0x40,
};

enum lis302dl_reg_cloik_src {
	LIS302DL_CLICKSRC_SINGLE_X	= 0x01,
	LIS302DL_CLICKSRC_DOUBLE_X	= 0x02,
	LIS302DL_CLICKSRC_SINGLE_Y	= 0x04,
	LIS302DL_CLICKSRC_DOUBLE_Y	= 0x08,
	LIS302DL_CLICKSRC_SINGLE_Z	= 0x10,
	LIS302DL_CLICKSRC_DOUBLE_Z	= 0x20,
	LIS302DL_CLICKSRC_IA		= 0x40,
};

#define LIS302DL_WHO_AM_I_MAGIC		0x3b

#define LIS302DL_F_WUP_FF		0x0001	/* wake up from free fall */
#define LIS302DL_F_WUP_CLICK		0x0002
#define LIS302DL_F_POWER		0x0010
#define LIS302DL_F_FS			0x0020 	/* ADC full scale */


#endif /* _LINUX_LIS302DL_H */

