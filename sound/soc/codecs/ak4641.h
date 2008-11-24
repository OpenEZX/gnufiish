/*
 * ak4641.h  --  AK4641 SoC Audio driver
 *
 * Copyright 2008 Harald Welte <laforge@gnufiish.org>
 *
 * Based on ak4641.h
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef _AK4641_H
#define _AK4641_H

/* AK4641 register space */

#define AK4641_PM1		0x00
#define AK4641_PM2		0x01
#define AK4641_SIG1		0x02
#define AK4641_SIG2		0x03
#define AK4641_MODE1		0x04
#define AK4641_MODE2		0x05
#define AK4641_DAC		0x06
#define AK4641_MIC		0x07
#define AK4641_TIMER		0x08
#define AK4641_ALC1		0x09
#define AK4641_ALC2		0x0a
#define AK4641_PGA		0x0b
#define AK4641_LATT		0x0c
#define AK4641_RATT		0x0d
#define AK4641_VOL		0x0e
#define AK4641_STATUS		0x0f
#define AK4641_EQLO		0x10
#define AK4641_EQMID		0x11
#define AK4641_EQHI		0x12
#define AK4641_BTIF		0x13

//#define AK4641_CACHEREGNUM 	0x13

struct ak4641_setup_data {
	int            i2c_bus;
	unsigned short i2c_address;
};

extern struct snd_soc_dai ak4641_dai;
extern struct snd_soc_codec_device soc_codec_dev_ak4641;

#if 0
#define REC_MASK        (SOUND_MASK_LINE | SOUND_MASK_MIC)
#define DEV_MASK        (REC_MASK | SOUND_MASK_PCM | SOUND_MASK_BASS)

#define REG_PWR1_PMADC  0x01
#define REG_PWR1_PMMIC  0x02
#define REG_PWR1_PMAUX  0x04
#define REG_PWR1_PMMO   0x08
#define REG_PWR1_PMLO   0x10
/* unused               0x20 */
/* unused               0x40 */
#define REG_PWR1_PMVCM  0x80

#define REG_PWR2_PMDAC  0x01
/* unused               0x02 */
/* unused               0x04 */
#define REG_PWR2_PMMO2  0x08
#define REG_PWR2_MCKAC  0x10
/* unused               0x20 */
/* unused               0x40 */
#define REG_PWR2_MCKPD  0x80

#define REG_SEL1_PSMO2  0x01
/* unused               0x02 */
/* unused               0x04 */
/* unused               0x08 */
#define REG_SEL1_MICM   0x10
#define REG_SEL1_DACM   0x20
#define REG_SEL1_PSMO   0x40
#define REG_SEL1_MOGN   0x80

#define REG_SEL2_PSLOR  0x01
#define REG_SEL2_PSLOL  0x02
#define REG_SEL2_AUXSI  0x04
/* unused               0x08 */
#define REG_SEL2_MICL   0x10
#define REG_SEL2_AUXL   0x20
/* unused               0x40 */
#define REG_SEL2_DACL   0x80

#define REG_MODE1_DIF0  0x01
#define REG_MODE1_DIF1  0x02
/* unused               0x04 */
/* unused               0x08 */
/* unused               0x10 */
/* unused               0x20 */
/* unused               0x40 */
/* unused               0x80 */

/* unused               0x01 */
#define REG_MODE2_LOOP  0x02
#define REG_MODE2_HPM   0x04
/* unused               0x08 */
/* unused               0x10 */
#define REG_MODE2_MCK0  0x20
#define REG_MODE2_MCK1  0x40
/* unused               0x80 */

#define REG_DAC_DEM0    0x01
#define REG_DAC_DEM1    0x02
#define REG_DAC_EQ      0x04
/* unused               0x08 */
#define REG_DAC_DATTC   0x10
#define REG_DAC_SMUTE   0x20
#define REG_DAC_TM      0x40
/* unused               0x80 */

#define REG_MIC_MGAIN   0x01
#define REG_MIC_MSEL    0x02
#define REG_MIC_MICAD   0x04
#define REG_MIC_MPWRI   0x08
#define REG_MIC_MPWRE   0x10
#define REG_MIC_AUXAD   0x20
/* unused               0x40 */
/* unused               0x80 */

#define REG_TIMER_LTM0  0x01
#define REG_TIMER_LTM1  0x02
#define REG_TIMER_WTM0  0x04
#define REG_TIMER_WTM1  0x08
#define REG_TIMER_ZTM0  0x10
#define REG_TIMER_ZTM1  0x20
/* unused               0x40 */
/* unused               0x80 */

#define REG_ALC1_LMTH   0x01
#define REG_ALC1_RATT   0x02
#define REG_ALC1_LMAT0  0x04
#define REG_ALC1_LMAT1  0x08
#define REG_ALC1_ZELM   0x10
#define REG_ALC1_ALC1   0x20
/* unused               0x40 */
/* unused               0x80 */

/* REG_EQ controls use 4 bits for each of 5 EQ levels */

/* Bluetooth not implemented */
#define REG_BTIF_PMAD2  0x01
#define REG_BTIF_PMDA2  0x02
#define REG_BTIF_PMBIF  0x04
#define REG_BTIF_ADC2   0x08
#define REG_BTIF_DAC2   0x10
#define REG_BTIF_BTFMT0 0x20
#define REG_BTIF_BTFMT1 0x40
/* unused               0x80 */

static struct ak4641_reg_info {
    unsigned char num;
    unsigned char default_value;
} ak4641_reg_info[] = {
    { REG_PWR1, REG_PWR1_PMVCM | REG_PWR1_PMADC | REG_PWR1_PMLO },
    { REG_PWR2, REG_PWR2_PMMO2 | REG_PWR2_PMDAC },
    { REG_SEL1, REG_SEL1_DACM | REG_SEL1_PSMO2 },
    { REG_SEL2, REG_SEL2_PSLOL | REG_SEL2_PSLOR | REG_SEL2_DACL },
    { REG_MODE1, REG_MODE1_DIF1 },
    { REG_MODE2, 0x0 },
    { REG_DAC, REG_DAC_DEM0 | REG_DAC_DATTC },
    { REG_MIC, REG_MIC_MICAD | REG_MIC_MGAIN },
    { REG_TIMER, 0 },
    { REG_ALC1, 0 },
    { REG_ALC2, 0x36 },
    { REG_PGA, 0x10 },
    { REG_RATT, 0x0 },
    { REG_LATT, 0x0 },
    { REG_VOL, 0x53 },
    { REG_STATUS, 0 },
    { REG_EQLO, 0x88 },
    { REG_EQMID, 0x88 },
    { REG_EQHI, 0x08 },
    { REG_BTIF, REG_BTIF_ADC2 }
};
#endif /*0*/

#endif
