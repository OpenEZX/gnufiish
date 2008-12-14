#ifndef _GLOFIISH_CPLD_H
#define _GLOFIISH_CPLD_H

/* platform data structures for calling driver */
struct gf_cpld_led_cfg {
	unsigned int reg_offset;
	unsigned int bit_offset;
	const char *name;
};

struct gf_cpld_pdata {
	unsigned int led_num;
	struct gf_cpld_led_cfg *led_cfgs;
};

#endif
