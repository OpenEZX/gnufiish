/* Smedia Glamo 336x/337x driver
 *
 * (C) 2007 by OpenMoko, Inc.
 * Author: Harald Welte <laforge@openmoko.org>
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
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/string.h>
#include <linux/mm.h>
#include <linux/slab.h>
#include <linux/delay.h>
#include <linux/fb.h>
#include <linux/init.h>
#include <linux/vmalloc.h>
#include <linux/dma-mapping.h>
#include <linux/interrupt.h>
#include <linux/workqueue.h>
#include <linux/wait.h>
#include <linux/platform_device.h>
#include <linux/clk.h>

#include <asm/io.h>
#include <asm/uaccess.h>
#include <asm/div64.h>

#ifdef CONFIG_PM
#include <linux/pm.h>
#endif

#include <linux/glamofb.h>

#include "glamo-regs.h"
#include "glamo-core.h"

#ifdef DEBUG
#define GLAMO_LOG(...)
#else
#define GLAMO_LOG(...) \
do { \
	printk(KERN_DEBUG "in %s:%s:%d", __FILE__, __func__, __LINE__); \
	printk(KERN_DEBUG __VA_ARGS__); \
} while (0);
#endif


#define RESSIZE(ressource) (((ressource)->end - (ressource)->start)+1)

struct glamofb_handle {
	struct fb_info *fb;
	struct device *dev;
	struct resource *reg;
	struct resource *fb_res;
	char __iomem *base;
	struct glamofb_platform_data *mach_info;
	char __iomem *cursor_addr;
	u_int32_t pseudo_pal[16];
};

/* 'sibling' spi device for lcm init */
static struct platform_device glamo_spi_dev = {
	.name		= "glamo-lcm-spi",
};


static int reg_read(struct glamofb_handle *glamo,
			   u_int16_t reg)
{
	glamo_reg_access_delay();
	return readw(glamo->base + reg);
}

static void reg_write(struct glamofb_handle *glamo,
			     u_int16_t reg, u_int16_t val)
{
	glamo_reg_access_delay();
	writew(val, glamo->base + reg);
}

static struct glamo_script glamo_regs[] = {
	{ GLAMO_REG_LCD_MODE1, 0x0020 },
	/* no display rotation, no hardware cursor, no dither, no gamma,
	 * no retrace flip, vsync low-active, hsync low active,
	 * no TVCLK, no partial display, hw dest color from fb,
	 * no partial display mode, LCD1, software flip,  */
	{ GLAMO_REG_LCD_MODE2, 0x1020 },
	  /* no video flip, no ptr, no ptr, dhclk,
	   * normal mode,  no cpuif,
	   * res, serial msb first, single fb, no fr ctrl,
	   * cpu if bits all zero, no crc
	   * 0000 0000 0010  0000 */
	{ GLAMO_REG_LCD_MODE3, 0x0b40 },
	  /* src data rgb565, res, 18bit rgb666
	   * 000 01 011 0100 0000 */
	{ GLAMO_REG_LCD_POLARITY, 0x440c },
	  /* DE high active, no cpu/lcd if, cs0 force low, a0 low active,
	   * np cpu if, 9bit serial data, sclk rising edge latch data
	   * 01 00 0 100 0 000 01 0 0 */
	{ GLAMO_REG_LCD_A_BASE1, 0x0000 }, /* display A base address 15:0 */
	{ GLAMO_REG_LCD_A_BASE2, 0x0000 }, /* display A base address 22:16 */
};

static int glamofb_run_script(struct glamofb_handle *glamo,
				struct glamo_script *script, int len)
{
	int i;

	for (i = 0; i < len; i++) {
		struct glamo_script *line = &script[i];

		if (line->reg == 0xffff)
			return 0;
		else if (line->reg == 0xfffe)
			msleep(line->val);
		else
			reg_write(glamo, script[i].reg, script[i].val);
	}

	return 0;
}

static int glamofb_check_var(struct fb_var_screeninfo *var,
			     struct fb_info *info)
{
	struct glamofb_handle *glamo = info->par;

	if (var->yres > glamo->mach_info->yres.max)
		var->yres = glamo->mach_info->yres.max;
	else if (var->yres < glamo->mach_info->yres.min)
		var->yres = glamo->mach_info->yres.min;

	if (var->xres > glamo->mach_info->xres.max)
		var->xres = glamo->mach_info->xres.max;
	else if (var->xres < glamo->mach_info->xres.min)
		var->xres = glamo->mach_info->xres.min;

	if (var->bits_per_pixel > glamo->mach_info->bpp.max)
		var->bits_per_pixel = glamo->mach_info->bpp.max;
	else if (var->bits_per_pixel < glamo->mach_info->bpp.min)
		var->bits_per_pixel = glamo->mach_info->bpp.min;

	/* FIXME: set rgb positions */
	switch (var->bits_per_pixel) {
	case 16:
		switch (reg_read(glamo, GLAMO_REG_LCD_MODE3) & 0xc000) {
		case GLAMO_LCD_SRC_RGB565:
			var->red.offset		= 11;
			var->green.offset	= 5;
			var->blue.offset	= 0;
			var->red.length		= 5;
			var->green.length	= 6;
			var->blue.length	= 5;
			var->transp.length	= 0;
			break;
		case GLAMO_LCD_SRC_ARGB1555:
			var->transp.offset	= 15;
			var->red.offset		= 10;
			var->green.offset	= 5;
			var->blue.offset	= 0;
			var->transp.length	= 1;
			var->red.length		= 5;
			var->green.length	= 5;
			var->blue.length	= 5;
			break;
		case GLAMO_LCD_SRC_ARGB4444:
			var->transp.offset	= 12;
			var->red.offset		= 8;
			var->green.offset	= 4;
			var->blue.offset	= 0;
			var->transp.length	= 4;
			var->red.length		= 4;
			var->green.length	= 4;
			var->blue.length	= 4;
			break;
		}
		break;
	case 24:
	case 32:
	default:
		/* The Smedia Glamo doesn't support anything but 16bit color */
		printk(KERN_ERR
		       "Smedia driver does not [yet?] support 24/32bpp\n");
		return -EINVAL;
		break;
	}

	return 0;
}

static void reg_set_bit_mask(struct glamofb_handle *glamo,
			     u_int16_t reg, u_int16_t mask,
			     u_int16_t val)
{
	u_int16_t tmp;

	val &= mask;

	tmp = reg_read(glamo, reg);
	tmp &= ~mask;
	tmp |= val;
	reg_write(glamo, reg, tmp);
}

#define GLAMO_LCD_WIDTH_MASK 0x03FF
#define GLAMO_LCD_HEIGHT_MASK 0x03FF
#define GLAMO_LCD_PITCH_MASK 0x07FE
#define GLAMO_LCD_HV_TOTAL_MASK 0x03FF
#define GLAMO_LCD_HV_RETR_START_MASK 0x03FF
#define GLAMO_LCD_HV_RETR_END_MASK 0x03FF
#define GLAMO_LCD_HV_RETR_DISP_START_MASK 0x03FF
#define GLAMO_LCD_HV_RETR_DISP_END_MASK 0x03FF

enum orientation {
    ORIENTATION_PORTRAIT,
    ORIENTATION_LANDSCAPE
};


static void rotate_lcd(struct glamofb_handle *glamo,
                       __u32 rotation)
{
	int glamo_rot;

	switch (rotation) {
		case FB_ROTATE_UR:
			glamo_rot = GLAMO_LCD_ROT_MODE_0;
			break;
		case FB_ROTATE_CW:
			glamo_rot = GLAMO_LCD_ROT_MODE_90;
			break;
		case FB_ROTATE_UD:
			glamo_rot = GLAMO_LCD_ROT_MODE_180;
			break;
		case FB_ROTATE_CCW:
			glamo_rot = GLAMO_LCD_ROT_MODE_270;
			break;
		default:
			glamo_rot = GLAMO_LCD_ROT_MODE_0;
			break;
	}
	glamofb_cmd_mode(glamo, 1);
	reg_set_bit_mask(glamo,
			 GLAMO_REG_LCD_WIDTH,
			 GLAMO_LCD_ROT_MODE_MASK,
			 glamo_rot);
	reg_set_bit_mask(glamo,
			 GLAMO_REG_LCD_MODE1,
			 GLAMO_LCD_MODE1_ROTATE_EN,
			 (glamo_rot != GLAMO_LCD_ROT_MODE_0)?
				 GLAMO_LCD_MODE1_ROTATE_EN : 0);
	glamofb_cmd_mode(glamo, 0);
}

static enum orientation get_orientation(struct fb_var_screeninfo *var)
{
	GLAMO_LOG("mark\n")
	if (var->xres <= var->yres) {
		GLAMO_LOG("portrait\n")
		return ORIENTATION_PORTRAIT;
	}
	GLAMO_LOG("landscape\n")
	return ORIENTATION_LANDSCAPE;
}

static int will_orientation_change(struct fb_var_screeninfo *var)
{
	enum  orientation orient = get_orientation(var);
	switch (orient) {
		case ORIENTATION_LANDSCAPE:
			if (var->rotate == FB_ROTATE_UR || var->rotate == FB_ROTATE_UD)
				return 1;
			break;
		case ORIENTATION_PORTRAIT:
			if (var->rotate == FB_ROTATE_CW || var->rotate == FB_ROTATE_CCW)
				return 1;
			break;
	}
	return 0;
}

static void glamofb_update_lcd_controller(struct glamofb_handle *glamo,
					  struct fb_var_screeninfo *var)
{
	int sync, bp, disp, fp, total, xres, yres, pitch, orientation_changing;

	GLAMO_LOG("enter: glamo:%#x, var:%#x\n", (unsigned)glamo, (unsigned)var);
	if (!glamo || !var)
		return;

	glamofb_cmd_mode(glamo, 1);

	if (var->pixclock)
		glamo_engine_reclock(glamo->mach_info->glamo,
				     GLAMO_ENGINE_LCD,
				     var->pixclock);

	xres = var->xres;
	yres = var->yres;
	GLAMO_LOG("xres:%d, yres:%d, rotate:%d\n", xres, yres, var->rotate);

	/*
	 * figure out if orientation is going to change
	 */
	orientation_changing = will_orientation_change(var);
	GLAMO_LOG("orientation_changing:%d\n", orientation_changing);

        /*
         * adjust the pitch according to new orientation to come
         */
        if (orientation_changing) {
		pitch = var->yres * var->bits_per_pixel / 8;
        } else {
		pitch = var->xres * var->bits_per_pixel / 8;
        }
	GLAMO_LOG("pitch:%d\n", pitch);

	/*
	 * set the awaiten LCD geometry
	 */
	reg_set_bit_mask(glamo,
			 GLAMO_REG_LCD_WIDTH,
			 GLAMO_LCD_WIDTH_MASK,
			 xres);
	reg_set_bit_mask(glamo,
			 GLAMO_REG_LCD_HEIGHT,
			 GLAMO_LCD_HEIGHT_MASK,
			 yres);
	reg_set_bit_mask(glamo,
			 GLAMO_REG_LCD_PITCH,
			 GLAMO_LCD_PITCH_MASK,
			 pitch);

	GLAMO_LOG("mark:\n");
	/*
	 * honour the rotation request
	 */
	rotate_lcd(glamo, var->rotate);

	/*
	 * update the reported geometry
	 * of the framebuffer.
	 */
	if (orientation_changing) {
		var->xres_virtual = var->xres = yres;
		var->yres_virtual = var->yres = xres;
	} else {
		var->xres_virtual = var->xres = xres;
		var->yres_virtual = var->yres = yres;
	}

	GLAMO_LOG("reported res:(%d,%d)\n", var->xres, var->yres);
	/*
	 * update scannout timings
	 */
	sync = 0;
	bp = sync + var->hsync_len;
	disp = bp + var->left_margin;
	fp = disp + xres;
	total = fp + var->right_margin;

	reg_set_bit_mask(glamo, GLAMO_REG_LCD_HORIZ_TOTAL,
			 GLAMO_LCD_HV_TOTAL_MASK, total);
	reg_set_bit_mask(glamo, GLAMO_REG_LCD_HORIZ_RETR_START,
			 GLAMO_LCD_HV_RETR_START_MASK, sync);
	reg_set_bit_mask(glamo, GLAMO_REG_LCD_HORIZ_RETR_END,
			 GLAMO_LCD_HV_RETR_END_MASK, bp);
	reg_set_bit_mask(glamo, GLAMO_REG_LCD_HORIZ_DISP_START,
			  GLAMO_LCD_HV_RETR_DISP_START_MASK, disp);
	reg_set_bit_mask(glamo, GLAMO_REG_LCD_HORIZ_DISP_END,
			 GLAMO_LCD_HV_RETR_DISP_END_MASK, fp);

	GLAMO_LOG("mark:\n");

	sync = 0;
	bp = sync + var->vsync_len;
	disp = bp + var->upper_margin;
	fp = disp + yres;
	total = fp + var->lower_margin;

	reg_set_bit_mask(glamo, GLAMO_REG_LCD_VERT_TOTAL,
			 GLAMO_LCD_HV_TOTAL_MASK, total);
	reg_set_bit_mask(glamo, GLAMO_REG_LCD_VERT_RETR_START,
			  GLAMO_LCD_HV_RETR_START_MASK, sync);
	reg_set_bit_mask(glamo, GLAMO_REG_LCD_VERT_RETR_END,
			 GLAMO_LCD_HV_RETR_END_MASK, bp);
	reg_set_bit_mask(glamo, GLAMO_REG_LCD_VERT_DISP_START,
			 GLAMO_LCD_HV_RETR_DISP_START_MASK, disp);
	reg_set_bit_mask(glamo, GLAMO_REG_LCD_VERT_DISP_END,
			 GLAMO_LCD_HV_RETR_DISP_END_MASK, fp);

	GLAMO_LOG("mark:\n");
	glamofb_cmd_mode(glamo, 0);

	GLAMO_LOG("leave:\n");
}

static int glamofb_set_par(struct fb_info *info)
{
	struct glamofb_handle *glamo = info->par;
	struct fb_var_screeninfo *var = &info->var;

	switch (var->bits_per_pixel) {
	case 16:
		info->fix.visual = FB_VISUAL_TRUECOLOR;
		break;
	default:
		printk("Smedia driver doesn't support != 16bpp\n");
		return -EINVAL;
	}

	info->fix.line_length = (var->xres * var->bits_per_pixel) / 8;

	glamofb_update_lcd_controller(glamo, var);

	return 0;
}

static int glamofb_blank(int blank_mode, struct fb_info *info)
{
	/* FIXME */
	return 0;
}

static inline unsigned int chan_to_field(unsigned int chan, struct fb_bitfield *bf)
{
	chan &= 0xffff;
	chan >>= 16 - bf->length;
	return chan << bf->offset;
}

static int glamofb_setcolreg(unsigned regno,
			     unsigned red, unsigned green, unsigned blue,
			     unsigned transp, struct fb_info *info)
{
	struct glamofb_handle *glamo = info->par;
	unsigned int val;

	switch (glamo->fb->fix.visual) {
	case FB_VISUAL_TRUECOLOR:
	case FB_VISUAL_DIRECTCOLOR:
		/* true-colour, use pseuo-palette */

		if (regno < 16) {
			u32 *pal = glamo->fb->pseudo_palette;

			val  = chan_to_field(red, &glamo->fb->var.red);
			val |= chan_to_field(green, &glamo->fb->var.green);
			val |= chan_to_field(blue, &glamo->fb->var.blue);

			pal[regno] = val;
		};
		break;
	default:
		return 1; /* unknown type */
	}

	return 0;
}

static int glamofb_cursor(struct fb_info *info, struct fb_cursor *cursor)
{
	struct glamofb_handle *glamo = info->par;
	u_int16_t reg;

	if (cursor->image.depth > 2)
		return -EINVAL;

	reg = reg_read(glamo, GLAMO_REG_LCD_MODE1);

	if (cursor->enable)
		reg_write(glamo, GLAMO_REG_LCD_MODE1,
			  reg | GLAMO_LCD_MODE1_CURSOR_EN);
	else
		reg_write(glamo, GLAMO_REG_LCD_MODE1,
			  reg & ~GLAMO_LCD_MODE1_CURSOR_EN);

	if (cursor->set & FB_CUR_SETPOS) {
		reg_write(glamo, GLAMO_REG_LCD_CURSOR_X_POS,
			  cursor->image.dx);
		reg_write(glamo, GLAMO_REG_LCD_CURSOR_Y_POS,
			  cursor->image.dy);
	}

	if (cursor->set & FB_CUR_SETCMAP) {
		/* FIXME */
	}

	if (cursor->set & FB_CUR_SETSIZE ||
	    cursor->set & (FB_CUR_SETIMAGE | FB_CUR_SETSHAPE)) {
		int x, y, op;
		const unsigned char *pcol = cursor->image.data;
		const unsigned char *pmsk = cursor->mask;
		void __iomem *dst = glamo->cursor_addr;
		unsigned char dcol = 0;
		unsigned char dmsk = 0;

		reg_write(glamo, GLAMO_REG_LCD_CURSOR_X_SIZE,
			  cursor->image.width);
		reg_write(glamo, GLAMO_REG_LCD_CURSOR_PITCH,
			  cursor->image.width * 2);
		reg_write(glamo, GLAMO_REG_LCD_CURSOR_Y_SIZE,
			  cursor->image.height);

		for (op = 0; op < (cursor->image.width *
				   cursor->image.height * 2)/8; op += 4)
			writel(0x0, dst + op);

		for (y = 0; y < cursor->image.height; y++) {
			for (x = 0; x < cursor->image.width; x++) {
				if ((x % 8) == 0) {
					dcol = *pcol++;
					dmsk = *pmsk++;
				} else {
					dcol >>= 1;
					dmsk >>= 1;
				}

				if (dmsk & 1) {
					unsigned int op;

					op = (dcol & 1) ? 1 : 3;
					op <<= ((x % 4) * 2);

					op |= readb(dst + (x / 4));
					writeb(op, dst + (x / 4));
				}
			}
		}
	}
}

static inline int glamofb_cmdq_empty(struct glamofb_handle *gfb)
{
	/* DGCMdQempty -- 1 == command queue is empty */
	return reg_read(gfb, GLAMO_REG_LCD_STATUS1) & (1 << 15);
}

void glamofb_cmd_mode(struct glamofb_handle *gfb, int on)
{
	int timeout = 20000;

	dev_dbg(gfb->dev, "glamofb_cmd_mode(gfb=%p, on=%d)\n", gfb, on);
	if (on) {
		dev_dbg(gfb->dev, "%s: waiting for cmdq empty: ",
			__FUNCTION__);
		while ((!glamofb_cmdq_empty(gfb)) && (timeout--))
			yield();
		if (timeout < 0) {
			printk(KERN_ERR"*************"
				       "glamofb cmd_queue never got empty"
				       "*************\n");
			return;
		}
		dev_dbg(gfb->dev, "empty!\n");

		/* display the entire frame then switch to command */
		reg_write(gfb, GLAMO_REG_LCD_COMMAND1,
			  GLAMO_LCD_CMD_TYPE_DISP |
			  GLAMO_LCD_CMD_DATA_FIRE_VSYNC);

		/* wait until LCD is idle */
		dev_dbg(gfb->dev, "waiting for LCD idle: ");
		timeout = 2000;
		while ((!reg_read(gfb, GLAMO_REG_LCD_STATUS2) & (1 << 12)) &&
		      (timeout--))
			yield();
		if (timeout < 0) {
			printk(KERN_ERR"*************"
				       "glamofb lcd never idle"
				       "*************\n");
			return;
		}
		dev_dbg(gfb->dev, "idle!\n");

		msleep(90);
	} else {
		/* RGB interface needs vsync/hsync */
		if (reg_read(gfb, GLAMO_REG_LCD_MODE3) & GLAMO_LCD_MODE3_RGB)
			reg_write(gfb, GLAMO_REG_LCD_COMMAND1,
				  GLAMO_LCD_CMD_TYPE_DISP |
				  GLAMO_LCD_CMD_DATA_DISP_SYNC);

		reg_write(gfb, GLAMO_REG_LCD_COMMAND1,
			  GLAMO_LCD_CMD_TYPE_DISP |
			  GLAMO_LCD_CMD_DATA_DISP_FIRE);
	}
}
EXPORT_SYMBOL_GPL(glamofb_cmd_mode);

int glamofb_cmd_write(struct glamofb_handle *gfb, u_int16_t val)
{
	int timeout = 20000;

	dev_dbg(gfb->dev, "%s: waiting for cmdq empty\n",
		__FUNCTION__);
	while ((!glamofb_cmdq_empty(gfb)) && (timeout--))
		yield();
	if (timeout < 0) {
		printk(KERN_ERR"*************"
				"glamofb cmd_queue never got empty"
				"*************\n");
		return 1;
	}
	dev_dbg(gfb->dev, "idle, writing 0x%04x\n", val);

	reg_write(gfb, GLAMO_REG_LCD_COMMAND1, val);

	return 0;
}
EXPORT_SYMBOL_GPL(glamofb_cmd_write);

static struct fb_ops glamofb_ops = {
	.owner		= THIS_MODULE,
	.fb_check_var	= glamofb_check_var,
	.fb_set_par	= glamofb_set_par,
	.fb_blank	= glamofb_blank,
	.fb_setcolreg	= glamofb_setcolreg,
	//.fb_cursor	= glamofb_cursor,
	.fb_fillrect	= cfb_fillrect,
	.fb_copyarea	= cfb_copyarea,
	.fb_imageblit	= cfb_imageblit,
};

static int glamofb_init_regs(struct glamofb_handle *glamo)
{
	struct fb_info *info = glamo->fb;

	glamofb_check_var(&info->var, info);
	glamofb_run_script(glamo, glamo_regs, ARRAY_SIZE(glamo_regs));
	glamofb_set_par(info);

	return 0;
}

static int __init glamofb_probe(struct platform_device *pdev)
{
	int rc = -EIO;
	struct fb_info *fbinfo;
	struct glamofb_handle *glamofb;
	struct glamofb_platform_data *mach_info = pdev->dev.platform_data;

	printk(KERN_INFO "SMEDIA Glamo frame buffer driver (C) 2007 "
		"OpenMoko, Inc.\n");

	fbinfo = framebuffer_alloc(sizeof(struct glamofb_handle), &pdev->dev);
	if (!fbinfo)
		return -ENOMEM;

	glamofb = fbinfo->par;
	glamofb->fb = fbinfo;
	glamofb->dev = &pdev->dev;

	strcpy(fbinfo->fix.id, "SMedia Glamo");

	glamofb->reg = platform_get_resource_byname(pdev, IORESOURCE_MEM,
						    "glamo-fb-regs");
	if (!glamofb->reg) {
		dev_err(&pdev->dev, "platform device with no registers?\n");
		rc = -ENOENT;
		goto out_free;
	}

	glamofb->fb_res = platform_get_resource_byname(pdev, IORESOURCE_MEM,
							"glamo-fb-mem");
	if (!glamofb->fb_res) {
		dev_err(&pdev->dev, "platform device with no memory ?\n");
		rc = -ENOENT;
		goto out_free;
	}

	glamofb->reg = request_mem_region(glamofb->reg->start,
					  RESSIZE(glamofb->reg), pdev->name);
	if (!glamofb->reg) {
		dev_err(&pdev->dev, "failed to request mmio region\n");
		goto out_free;
	}

	glamofb->fb_res = request_mem_region(glamofb->fb_res->start,
					     mach_info->fb_mem_size,
					     pdev->name);
	if (!glamofb->fb_res) {
		dev_err(&pdev->dev, "failed to request vram region\n");
		goto out_release_reg;
	}

	/* we want to remap only the registers required for this core
	 * driver. */
	glamofb->base = ioremap(glamofb->reg->start, RESSIZE(glamofb->reg));
	if (!glamofb->base) {
		dev_err(&pdev->dev, "failed to ioremap() mmio memory\n");
		goto out_release_fb;
	}
	fbinfo->fix.smem_start = (unsigned long) glamofb->fb_res->start;
	fbinfo->fix.smem_len = mach_info->fb_mem_size;

	fbinfo->screen_base = ioremap(glamofb->fb_res->start,
				       RESSIZE(glamofb->fb_res));
	if (!fbinfo->screen_base) {
		dev_err(&pdev->dev, "failed to ioremap() vram memory\n");
		goto out_release_fb;
	}

	platform_set_drvdata(pdev, fbinfo);

	glamofb->mach_info = pdev->dev.platform_data;

	fbinfo->fix.visual = FB_VISUAL_TRUECOLOR;
	fbinfo->fix.type = FB_TYPE_PACKED_PIXELS;
	fbinfo->fix.type_aux = 0;
	fbinfo->fix.xpanstep = 0;
	fbinfo->fix.ypanstep = 0;
	fbinfo->fix.ywrapstep = 0;
	fbinfo->fix.accel = FB_ACCEL_NONE; /* FIXME */

	fbinfo->var.nonstd = 0;
	fbinfo->var.activate = FB_ACTIVATE_NOW;
	fbinfo->var.height = mach_info->height;
	fbinfo->var.width = mach_info->width;
	fbinfo->var.accel_flags = 0;
	fbinfo->var.vmode = FB_VMODE_NONINTERLACED;

	fbinfo->fbops = &glamofb_ops;
	fbinfo->flags = FBINFO_FLAG_DEFAULT;
	fbinfo->pseudo_palette = &glamofb->pseudo_pal;

	fbinfo->var.xres = mach_info->xres.defval;
	fbinfo->var.xres_virtual = mach_info->xres.defval;
	fbinfo->var.yres = mach_info->yres.defval;
	fbinfo->var.yres_virtual = mach_info->yres.defval;
	fbinfo->var.bits_per_pixel = mach_info->bpp.defval;

	fbinfo->var.pixclock = mach_info->pixclock;
	fbinfo->var.left_margin = mach_info->left_margin;
	fbinfo->var.right_margin = mach_info->right_margin;
	fbinfo->var.upper_margin = mach_info->upper_margin;
	fbinfo->var.lower_margin = mach_info->lower_margin;
	fbinfo->var.hsync_len = mach_info->hsync_len;
	fbinfo->var.vsync_len = mach_info->vsync_len;

	memset(fbinfo->screen_base, 0, fbinfo->fix.smem_len);

	glamo_engine_enable(mach_info->glamo, GLAMO_ENGINE_LCD);
	glamo_engine_reset(mach_info->glamo, GLAMO_ENGINE_LCD);
	glamofb_init_regs(glamofb);

	rc = register_framebuffer(fbinfo);
	if (rc < 0) {
		dev_err(&pdev->dev, "failed to register framebuffer\n");
		goto out_unmap_fb;
	}

	if (mach_info->spi_info) {
		/* register the sibling spi device */
		mach_info->spi_info->glamofb_handle = glamofb;
		glamo_spi_dev.dev.parent = &pdev->dev;
		glamo_spi_dev.dev.platform_data = mach_info->spi_info;
		platform_device_register(&glamo_spi_dev);
	}

	printk(KERN_INFO "fb%d: %s frame buffer device\n",
		fbinfo->node, fbinfo->fix.id);

	return 0;

out_unmap_fb:
	iounmap(fbinfo->screen_base);
	iounmap(glamofb->base);
out_release_fb:
	release_mem_region(glamofb->fb_res->start, RESSIZE(glamofb->fb_res));
out_release_reg:
	release_mem_region(glamofb->reg->start, RESSIZE(glamofb->reg));
out_free:
	framebuffer_release(fbinfo);
	return rc;
}

static int glamofb_remove(struct platform_device *pdev)
{
	struct glamofb_handle *glamofb = platform_get_drvdata(pdev);

	platform_set_drvdata(pdev, NULL);
	iounmap(glamofb->base);
	release_mem_region(glamofb->reg->start, RESSIZE(glamofb->reg));
	kfree(glamofb);

	return 0;
}

#ifdef CONFIG_PM
static int glamofb_suspend(struct platform_device *pdev, pm_message_t state)
{
	return 0;
}

static int glamofb_resume(struct platform_device *pdev)
{
	return 0;
}
#else
#define glamofb_suspend NULL
#define glamofb_resume  NULL
#endif

static struct platform_driver glamofb_driver = {
	.probe		= glamofb_probe,
	.remove		= glamofb_remove,
	.suspend	= glamofb_suspend,
	.resume		= glamofb_resume,
	.driver		= {
		.name	= "glamo-fb",
		.owner	= THIS_MODULE,
	},
};

static int __devinit glamofb_init(void)
{
	return platform_driver_register(&glamofb_driver);
}

static void __exit glamofb_cleanup(void)
{
	platform_driver_unregister(&glamofb_driver);
}

module_init(glamofb_init);
module_exit(glamofb_cleanup);

MODULE_AUTHOR("Harald Welte <laforge@openmoko.org>");
MODULE_DESCRIPTION("Smedia Glamo 336x/337x framebuffer driver");
MODULE_LICENSE("GPL");
