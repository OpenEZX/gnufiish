#ifndef _LINUX_GLAMOFB_H
#define _LINUX_GLAMOFB_H

#include <linux/spi/glamo.h>

struct glamofb_val {
	unsigned int defval;
	unsigned int min;
	unsigned int max;
};

struct glamo_core;

struct glamofb_platform_data {
	int width, height;
	int pixclock;
	int left_margin, right_margin;
	int upper_margin, lower_margin;
	int hsync_len, vsync_len;
	int fb_mem_size;

	struct glamofb_val xres;
	struct glamofb_val yres;
	struct glamofb_val bpp;

	struct glamo_spi_info *spi_info;
	struct glamo_spigpio_info *spigpio_info;
	struct glamo_core *glamo;

	/* glamo mmc platform specific info */
	void		(*glamo_set_mci_power)(unsigned char power_mode,
				     unsigned short vdd);
	int		(*glamo_irq_is_wired)(void);
};

void glamofb_cmd_mode(struct glamofb_handle *gfb, int on);
int glamofb_cmd_write(struct glamofb_handle *gfb, u_int16_t val);
void glamo_lcm_reset(int level);

#endif
