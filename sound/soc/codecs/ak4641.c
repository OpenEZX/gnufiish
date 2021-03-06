/*
 * ak4641.c  --  AK4641 ALSA Soc Audio driver
 *
 * Copyright (C) 2008 Harald Welte <laforge@gnufiish.org>
 *
 * Based on ak4353.c by Richard Purdie
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/init.h>
#include <linux/delay.h>
#include <linux/pm.h>
#include <linux/i2c.h>
#include <linux/platform_device.h>
#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>
#include <sound/soc-dapm.h>
#include <sound/initval.h>

#include "ak4641.h"

#define AK4641_VERSION "0.1"

struct snd_soc_codec_device soc_codec_dev_ak4641;

/* codec private data */
struct ak4641_priv {
	unsigned int sysclk;
};

/*
 * ak4535 register cache
 */
static const u16 ak4641_reg[AK4641_CACHEREGNUM] = {
    0x0098, 0x0009, 0x0021, 0x0083,
    0x0002, 0x0000, 0x0011, 0x0005,
    0x0000, 0x0000, 0x0036, 0x0010,
    0x0000, 0x0000, 0x0053, 0x0000,
    0x0088, 0x0088, 0x0008, 0x0008
};

/*
 * read ak4641 register cache
 */
static inline unsigned int ak4641_read_reg_cache(struct snd_soc_codec *codec,
	unsigned int reg)
{
	u16 *cache = codec->reg_cache;
	if (reg >= AK4641_CACHEREGNUM)
		return -1;
	return cache[reg];
}

static inline unsigned int ak4641_read(struct snd_soc_codec *codec,
	unsigned int reg)
{
	u8 data;
	data = reg;

	if (codec->hw_write(codec->control_data, &data, 1) != 1)
		return -EIO;

	if (codec->hw_read(codec->control_data, &data, 1) != 1)
		return -EIO;

	return data;
};

/*
 * write ak4641 register cache
 */
static inline void ak4641_write_reg_cache(struct snd_soc_codec *codec,
	u16 reg, unsigned int value)
{
	u16 *cache = codec->reg_cache;
	if (reg >= AK4641_CACHEREGNUM)
		return;
	cache[reg] = value;
}

/*
 * write to the AK4641 register space
 */
static int ak4641_write(struct snd_soc_codec *codec, unsigned int reg,
	unsigned int value)
{
	u8 data[2];

	/* data is
	 *   D15..D8 AK4641 register offset
	 *   D7...D0 register data
	 */
	data[0] = reg & 0xff;
	data[1] = value & 0xff;

	ak4641_write_reg_cache(codec, reg, value);
	if (codec->hw_write(codec->control_data, data, 2) == 2)
		return 0;
	else
		return -EIO;
}

static int ak4641_sync(struct snd_soc_codec *codec)
{
	u16 *cache = codec->reg_cache;
	int i, r = 0;

	for (i = 0; i < AK4641_CACHEREGNUM; i++)
		r |= ak4641_write(codec, i, cache[i]);

	return r;
};

/* Equalizer Controls */

#define AK4641_EQUAL(xname, xindex) \
{ .iface = SNDRV_CTL_ELEM_IFACE_MIXER, .name = xname, .index = xindex, \
  .info = snd_info_equalizer, .get = snd_get_equalizer, \
  .put = snd_put_equalizer, .private_value = xindex }

static int snd_info_equalizer(struct snd_kcontrol *kcontrol,
			      struct snd_ctl_elem_info *uinfo)
{
	uinfo->type = SNDRV_CTL_ELEM_TYPE_INTEGER;
	uinfo->count = 1;
	/* Stepsize is 1.5dB, ALSA doesn't do non-integral steps */
	uinfo->value.integer.min = -7;	/* -10.5dB */
	uinfo->value.integer.max = 8;	/* +12dB */

	return 0;
}

static int snd_get_equalizer(struct snd_kcontrol *kcontrol,
			     struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_kcontrol_chip(kcontrol);
	struct ak4641_priv *ak4641 = codec->private_data;
	u_int16_t reg = AK4641_EQLO;
	u_int8_t val;
	int idx = kcontrol->private_value;

	if (idx > 4)
		return -EINVAL;

	reg += idx/2;
	val = ak4641_read_reg_cache(codec, reg);

	if (idx % 2)
		val >> 4;
	else
		val &= 0x0f;

	ucontrol->value.integer.value[0] = 8 - val;
	return 0;
}

static int snd_put_equalizer(struct snd_kcontrol *kcontrol,
			     struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_kcontrol_chip(kcontrol);
	struct ak4641_priv *ak4641 = codec->private_data;
	u_int16_t reg = AK4641_EQLO;
	u_int8_t val, bits;
	int idx = kcontrol->private_value;

	printk(KERN_DEBUG "put_equalizer (idx = %d)\n", idx);

	if (idx > 4)
		return -EINVAL;

	reg += idx/2;
	val = ak4641_read_reg_cache(codec, reg);
	bits = ucontrol->value.integer.value[0] - 8;

	if (idx % 2) {
		val &= 0x0f;	
		val |= (bits << 4);
	} else {
		val &= 0xf0;
		val |= bits;
	}

	ak4641_write_reg_cache(codec, reg, val);
	return 0;
}

static const char *ak4641_mono_gain[] = {"+6dB", "-17dB"};
static const char *ak4641_mono_out[] = {"(L + R)/2", "Hi-Z"};
static const char *ak4641_hp_out[] = {"Stereo", "Mono"};
static const char *ak4641_deemp[] = {"44.1kHz", "Off", "48kHz", "32kHz"};
static const char *ak4641_mic_select[] = {"Internal", "External"};
static const char *ak4641_mic_or_dac[] = {"Microphone", "Voice DAC"};

static const struct soc_enum ak4641_enum[] = {
	SOC_ENUM_SINGLE(AK4641_SIG1, 7, 2, ak4641_mono_gain),
	SOC_ENUM_SINGLE(AK4641_SIG1, 6, 2, ak4641_mono_out),
	SOC_ENUM_SINGLE(AK4641_MODE2, 2, 2, ak4641_hp_out),
	SOC_ENUM_SINGLE(AK4641_DAC, 0, 4, ak4641_deemp),
	SOC_ENUM_SINGLE(AK4641_MIC, 1, 2, ak4641_mic_select),
	SOC_ENUM_SINGLE(AK4641_BTIF, 4, 2, ak4641_mic_or_dac),
};

static const struct snd_kcontrol_new ak4641_snd_controls[] = {
	//SOC_SINGLE("ALC2 Switch", AK4535_SIG1, 1, 1, 0),
	SOC_ENUM("Mono 1 Output", ak4641_enum[1]),
	SOC_ENUM("Mono 1 Gain", ak4641_enum[0]),
	SOC_ENUM("Headphone Output", ak4641_enum[2]),
	SOC_ENUM("Playback Deemphasis", ak4641_enum[3]),

	SOC_SINGLE("Mic Boost (+20dB) Switch", AK4641_MIC, 0, 1, 0),
	SOC_ENUM("Mic Select", ak4641_enum[4]),
	SOC_ENUM("Input Select", ak4641_enum[5]),

	SOC_SINGLE("ALC Operation Time", AK4641_TIMER, 0, 3, 0),
	SOC_SINGLE("ALC Recovery Time", AK4641_TIMER, 2, 3, 0),
	SOC_SINGLE("ALC ZC Time", AK4641_TIMER, 4, 3, 0),

	SOC_SINGLE("ALC 1 Switch", AK4641_ALC1, 5, 1, 0),

	SOC_SINGLE("ALC Volume", AK4641_ALC2, 0, 127, 0),

	SOC_SINGLE("Capture Volume", AK4641_PGA, 0, 127, 0),

	SOC_DOUBLE_R("Master Playback Volume", AK4641_LATT, AK4641_RATT, 0, 255, 1),

	SOC_SINGLE("AUX In Volume", AK4641_VOL, 0, 15, 0),
	SOC_SINGLE("Mic In Volume", AK4641_VOL, 4, 7, 0),
	SOC_SINGLE("Mic In -4dB", AK4641_VOL, 7, 1, 0),

	AK4641_EQUAL("Equalizer <= 100 Hz", 0),
	AK4641_EQUAL("Equalizer 250 Hz", 1),
	AK4641_EQUAL("Equalizer 1 kHz", 2),
	AK4641_EQUAL("Equalizer 3.5 kHz", 3),
	AK4641_EQUAL("Equalizer >= 10 kHz", 4),
};

/* add non dapm controls */
static int ak4641_add_controls(struct snd_soc_codec *codec)
{
	int err, i;

	for (i = 0; i < ARRAY_SIZE(ak4641_snd_controls); i++) {
		err = snd_ctl_add(codec->card,
			snd_soc_cnew(&ak4641_snd_controls[i], codec, NULL));
		if (err < 0)
			return err;
	}

	return 0;
}

/* Mono 1 Mixer */
static const struct snd_kcontrol_new ak4641_mono1_mixer_controls[] = {
	SOC_DAPM_SINGLE("Mic Mono Sidetone Switch", AK4641_SIG1, 4, 1, 0),
	SOC_DAPM_SINGLE("Mono Playback Switch", AK4641_SIG1, 5, 1, 0),
};

/* Stereo Mixer */
static const struct snd_kcontrol_new ak4641_stereo_mixer_controls[] = {
	SOC_DAPM_SINGLE("Mic Sidetone Switch", AK4641_SIG2, 4, 1, 0),
	SOC_DAPM_SINGLE("Playback Switch", AK4641_SIG2, 7, 1, 0),
	SOC_DAPM_SINGLE("Aux Bypass Switch", AK4641_SIG2, 5, 1, 0),
};

/* Input Mixer */
static const struct snd_kcontrol_new ak4641_input_mixer_controls[] = {
	SOC_DAPM_SINGLE("Mic Capture Switch", AK4641_MIC, 2, 1, 0),
	SOC_DAPM_SINGLE("Aux Capture Switch", AK4641_MIC, 5, 1, 0),
};

/* Mic mux */
static const struct snd_kcontrol_new ak4641_mic_mux_control =
	SOC_DAPM_ENUM("Mic Select", ak4641_enum[4]);

/* Input mux */
static const struct snd_kcontrol_new ak4641_input_mux_control =
	SOC_DAPM_ENUM("Input Select", ak4641_enum[5]);

/* HP L switch */
static const struct snd_kcontrol_new ak4641_hpl_control =
	SOC_DAPM_SINGLE("Switch", AK4641_SIG2, 1, 1, 1);

/* HP R switch */
static const struct snd_kcontrol_new ak4641_hpr_control =
	SOC_DAPM_SINGLE("Switch", AK4641_SIG2, 0, 1, 1);

/* mono 2 switch */
static const struct snd_kcontrol_new ak4641_mono2_control =
	SOC_DAPM_SINGLE("Switch", AK4641_SIG1, 0, 1, 0);

/* ak4535 dapm widgets */
static const struct snd_soc_dapm_widget ak4641_dapm_widgets[] = {
	SND_SOC_DAPM_MIXER("Stereo Mixer", SND_SOC_NOPM, 0, 0,
		&ak4641_stereo_mixer_controls[0],
		ARRAY_SIZE(ak4641_stereo_mixer_controls)),
	SND_SOC_DAPM_MIXER("Mono1 Mixer", SND_SOC_NOPM, 0, 0,
		&ak4641_mono1_mixer_controls[0],
		ARRAY_SIZE(ak4641_mono1_mixer_controls)),
	SND_SOC_DAPM_MIXER("Input Mixer", SND_SOC_NOPM, 0, 0,
		&ak4641_input_mixer_controls[0],
		ARRAY_SIZE(ak4641_input_mixer_controls)),
	SND_SOC_DAPM_MUX("Mic Mux", SND_SOC_NOPM, 0, 0,
		&ak4641_mic_mux_control),
	SND_SOC_DAPM_MUX("Input Mux", SND_SOC_NOPM, 0, 0,
		&ak4641_input_mux_control),
	SND_SOC_DAPM_SWITCH("Mono 2 Enable", SND_SOC_NOPM, 0, 0,
		&ak4641_mono2_control),
	/* speaker powersave bit */
	SND_SOC_DAPM_SWITCH("Left Out Enable", SND_SOC_NOPM, 0, 0,
		&ak4641_hpl_control),
	SND_SOC_DAPM_SWITCH("Right Out Enable", SND_SOC_NOPM, 0, 0,
		&ak4641_hpr_control),

	SND_SOC_DAPM_OUTPUT("LOUT"),
	SND_SOC_DAPM_OUTPUT("ROUT"),
	SND_SOC_DAPM_OUTPUT("MOUT1"),
	SND_SOC_DAPM_OUTPUT("MOUT2"),
	SND_SOC_DAPM_OUTPUT("MICOUT"),

	SND_SOC_DAPM_ADC("ADC", "Capture", AK4641_PM1, 0, 0),
	SND_SOC_DAPM_PGA("Mic", AK4641_PM1, 1, 0, NULL, 0),
	SND_SOC_DAPM_PGA("AUX In", AK4641_PM1, 2, 0, NULL, 0),
	SND_SOC_DAPM_PGA("Mono Out", AK4641_PM1, 3, 0, NULL, 0),
	SND_SOC_DAPM_PGA("Line Out", AK4641_PM1, 4, 0, NULL, 0),

	SND_SOC_DAPM_DAC("DAC", "Playback", AK4641_PM2, 0, 0),
	SND_SOC_DAPM_PGA("Mono Out 2", AK4641_PM2, 3, 0, NULL, 0),

	SND_SOC_DAPM_ADC("Voice ADC", "Voice Capture", AK4641_BTIF, 0, 0),
	SND_SOC_DAPM_ADC("Voice DAC", "Voice Playback", AK4641_BTIF, 1, 0),

	SND_SOC_DAPM_MICBIAS("Mic Int Bias", AK4641_MIC, 3, 0),
	SND_SOC_DAPM_MICBIAS("Mic Ext Bias", AK4641_MIC, 4, 0),

	SND_SOC_DAPM_INPUT("MICIN"),
	SND_SOC_DAPM_INPUT("MICEXT"),
	SND_SOC_DAPM_INPUT("AUX"),
	SND_SOC_DAPM_INPUT("AIN"),
};

static const struct snd_soc_dapm_route audio_map[] = {
	/* Stereo Mixer */
	{"Stereo Mixer", "Playback Switch", "DAC"},
	{"Stereo Mixer", "Mic Sidetone Switch", "Input"},
	{"Stereo Mixer", "Aux Bypass Switch", "AUX In"},

	/* Mono 1 Mixer */
	{"Mono1 Mixer", "Mic Mono Sidetone Switch", "Input"},
	{"Mono1 Mixer", "Mono Playback Switch", "DAC"},

	/* Mic */
	{"Mic", NULL, "AIN"},
	{"Mic Mux", "Internal", "Mic Int Bias"},
	{"Mic Mux", "External", "Mic Ext Bias"},
	{"Mic Int Bias", NULL, "MICIN"},
	{"Mic Ext Bias", NULL, "MICEXT"},
	{"MICOUT", NULL, "Mic Mux"},

	/* Input Mux */
	{"Input", "Input Mux", "Mic"},
	{"Input", "Input Mux", "Voice DAC"},

	/* Line Out */
	{"LOUT", NULL, "Line Out Enable"},
	{"ROUT", NULL, "Line Out Enable"},
	{"Line Out Enable", "Switch", "Line Out"},
	{"Line Out", NULL, "Stereo Mixer"},

	/* Mono 1 Out */
	{"MOUT1", NULL, "Mono Out"},
	{"Mono Out", NULL, "Mono1 Mixer"},
	
	/* Mono 2 Out */
	{"MOUT2", NULL, "Mono 2 Enable"},
	{"Mono 2 Enable", "Switch", "Stereo Mixer"},
	{"Voice ADC", NULL, "Mono 2 Enable"},

	/* Aux In */
	{"AUX In", NULL, "AUX"},

	/* ADC */
	{"ADC", NULL, "Input Mixer"},
	{"Input Mixer", "Mic Capture Switch", "Mic"},
	{"Input Mixer", "Aux Capture Switch", "AUX In"},
};

static int ak4641_add_widgets(struct snd_soc_codec *codec)
{
	snd_soc_dapm_new_controls(codec, ak4641_dapm_widgets,
				  ARRAY_SIZE(ak4641_dapm_widgets));

	snd_soc_dapm_add_routes(codec, audio_map, ARRAY_SIZE(audio_map));

	snd_soc_dapm_new_widgets(codec);
	return 0;
}

static int ak4641_set_dai_sysclk(struct snd_soc_dai *codec_dai,
	int clk_id, unsigned int freq, int dir)
{
	struct snd_soc_codec *codec = codec_dai->codec;
	struct ak4641_priv *ak4641 = codec->private_data;

	ak4641->sysclk = freq;
	return 0;
}

static int ak4641_pcm_hw_params(struct snd_pcm_substream *substream,
	struct snd_pcm_hw_params *params)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_device *socdev = rtd->socdev;
	struct snd_soc_codec *codec = socdev->codec;
	struct ak4641_priv *ak4641 = codec->private_data;

	/* FIXME */
}

static int ak4641_i2s_hw_params(struct snd_pcm_substream *substream,
	struct snd_pcm_hw_params *params)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_device *socdev = rtd->socdev;
	struct snd_soc_codec *codec = socdev->codec;
	struct ak4641_priv *ak4641 = codec->private_data;
	u8 mode2 = ak4641_read_reg_cache(codec, AK4641_MODE2) & ~(0x3 << 5);
	int rate = params_rate(params), fs = 256;

	if (rate)
		fs = ak4641->sysclk / rate;

	/* set fs */
	switch (fs) {
	case 1024:
		mode2 |= (0x2 << 5);
		break;
	case 512:
		mode2 |= (0x1 << 5);
		break;
	case 256:
		break;
	}

	/* set rate */
	ak4641_write(codec, AK4641_MODE2, mode2);
	return 0;
}

static int ak4641_pcm_set_dai_fmt(struct snd_soc_dai *codec_dai,
				  unsigned int fmt)
{
	struct snd_soc_codec *codec = codec_dai->codec;
	u8 btif = ak4641_read_reg_cache(codec, AK4641_BTIF) & ~(0x3 << 5);

	/* interface format */
	switch (fmt & SND_SOC_DAIFMT_FORMAT_MASK) {
	case SND_SOC_DAIFMT_I2S:
		btif |= (0x3 << 5);
		break;
	case SND_SOC_DAIFMT_LEFT_J:
		btif |= (0x2 << 5);
		break;
	case SND_SOC_DAIFMT_DSP_A:	/* MSB after FRM */
		btif |= (0x0 << 5);
		break;
	case SND_SOC_DAIFMT_DSP_B:	/* MSB during FRM */
		btif |= (0x1 << 5);
		break;
	default:
		return -EINVAL;
	}

	ak4641_write(codec, AK4641_BTIF, btif);
	return 0;
}

static int ak4641_i2s_set_dai_fmt(struct snd_soc_dai *codec_dai,
		unsigned int fmt)
{
	struct snd_soc_codec *codec = codec_dai->codec;
	u8 mode1 = 0;

	/* interface format */
	switch (fmt & SND_SOC_DAIFMT_FORMAT_MASK) {
	case SND_SOC_DAIFMT_I2S:
		mode1 = 0x0002;
		break;
	case SND_SOC_DAIFMT_LEFT_J:
		mode1 = 0x0001;
		break;
	default:
		return -EINVAL;
	}

	/* use 32 fs for BCLK to save power */
	mode1 |= 0x4;

	ak4641_write(codec, AK4641_MODE1, mode1);
	return 0;
}

static int ak4641_mute(struct snd_soc_dai *dai, int mute)
{
	struct snd_soc_codec *codec = dai->codec;
	u16 mute_reg = ak4641_read_reg_cache(codec, AK4641_DAC) & 0xffdf;
	if (!mute)
		ak4641_write(codec, AK4641_DAC, mute_reg);
	else
		ak4641_write(codec, AK4641_DAC, mute_reg | 0x20);
	return 0;
}

static int ak4641_set_bias_level(struct snd_soc_codec *codec,
	enum snd_soc_bias_level level)
{
	u16 i;

	switch (level) {
	case SND_SOC_BIAS_ON:
		ak4641_mute(codec->dai, 0);
		break;
	case SND_SOC_BIAS_PREPARE:
		ak4641_mute(codec->dai, 1);
		break;
	case SND_SOC_BIAS_STANDBY:
		i = ak4641_read_reg_cache(codec, AK4641_PM1);
		ak4641_write(codec, AK4641_PM1, i | 0x80);
		i = ak4641_read_reg_cache(codec, AK4641_PM2);
		ak4641_write(codec, AK4641_PM2, i & (~0x80));
		break;
	case SND_SOC_BIAS_OFF:
		i = ak4641_read_reg_cache(codec, AK4641_PM1);
		ak4641_write(codec, AK4641_PM1, i & (~0x80));
		break;
	}
	codec->bias_level = level;
	return 0;
}

#define AK4641_RATES (SNDRV_PCM_RATE_8000 | SNDRV_PCM_RATE_11025 |\
		SNDRV_PCM_RATE_16000 | SNDRV_PCM_RATE_22050 |\
		SNDRV_PCM_RATE_44100 | SNDRV_PCM_RATE_48000)
#define AK4641_RATES_BT (SNDRV_PCM_RATE_8000 | SNDRV_PCM_RATE_11025 |\
			 SNDRV_PCM_RATE_16000)
#define AK4641_FORMATS	(SNDRV_PCM_FMTBIT_S16_LE)

struct snd_soc_dai ak4641_dai[] = {
{
	.name = "AK4641 HiFi",
	.id = 1,
	.playback = {
		.stream_name = "HiFi Playback",
		.channels_min = 1,
		.channels_max = 2,
		.rates = AK4641_RATES,
		.formats = AK4641_FORMATS,
	},
	.capture = {
		.stream_name = "Capture",
		.channels_min = 1,
		.channels_max = 2,
		.rates = AK4641_RATES,
		.formats = AK4641_FORMATS,
	},
	.ops = {
		.hw_params = ak4641_i2s_hw_params,
		.set_fmt = ak4641_i2s_set_dai_fmt,
		.digital_mute = ak4641_mute,
		.set_sysclk = ak4641_set_dai_sysclk,
	},
},
{
	.name = "AK4641 Voice",
	.id = 1,
	.playback = {
		.stream_name = "Voice Playback",
		.channels_min = 1,
		.channels_max = 1,	
		.rates = AK4641_RATES_BT,
		.formats = AK4641_FORMATS,
	},
	.capture = {
		.stream_name = "Capture",
		.channels_min = 1,
		.channels_max = 1,
		.rates = AK4641_RATES_BT,
		.formats = AK4641_FORMATS,
	},
	.ops = {
		.hw_params = ak4641_pcm_hw_params,
		.set_fmt = ak4641_pcm_set_dai_fmt,
		.digital_mute = ak4641_mute,
		.set_sysclk = ak4641_set_dai_sysclk,
	},
},
};

EXPORT_SYMBOL_GPL(ak4641_dai);

static int ak4641_suspend(struct platform_device *pdev, pm_message_t state)
{
	struct snd_soc_device *socdev = platform_get_drvdata(pdev);
	struct snd_soc_codec *codec = socdev->codec;

	ak4641_set_bias_level(codec, SND_SOC_BIAS_OFF);
	return 0;
}

static int ak4641_resume(struct platform_device *pdev)
{
	struct snd_soc_device *socdev = platform_get_drvdata(pdev);
	struct snd_soc_codec *codec = socdev->codec;
	ak4641_sync(codec);
	ak4641_set_bias_level(codec, SND_SOC_BIAS_STANDBY);
	ak4641_set_bias_level(codec, codec->suspend_bias_level);
	return 0;
}

/*
 * initialise the AK4641 driver
 * register the mixer and dsp interfaces with the kernel
 */
static int ak4641_init(struct snd_soc_device *socdev)
{
	struct snd_soc_codec *codec = socdev->codec;
	int ret = 0;

	codec->name = "AK4641";
	codec->owner = THIS_MODULE;
	codec->read = ak4641_read_reg_cache;
	codec->write = ak4641_write;
	codec->set_bias_level = ak4641_set_bias_level;
	codec->dai = ak4641_dai;
	codec->num_dai = 2;
	codec->reg_cache_size = ARRAY_SIZE(ak4641_reg);
	codec->reg_cache = kmemdup(ak4641_reg, sizeof(ak4641_reg), GFP_KERNEL);

	if (codec->reg_cache == NULL)
		return -ENOMEM;

	/* register pcms */
	ret = snd_soc_new_pcms(socdev, SNDRV_DEFAULT_IDX1, SNDRV_DEFAULT_STR1);
	if (ret < 0) {
		printk(KERN_ERR "ak4641: failed to create pcms\n");
		goto pcm_err;
	}

	/* power on device */
	ak4641_set_bias_level(codec, SND_SOC_BIAS_STANDBY);

	ak4641_add_controls(codec);
	ak4641_add_widgets(codec);
	ret = snd_soc_init_card(socdev);
	if (ret < 0) {
		printk(KERN_ERR "ak4641: failed to register card\n");
		goto card_err;
	}

	return ret;

card_err:
	snd_soc_free_pcms(socdev);
	snd_soc_dapm_free(socdev);
pcm_err:
	kfree(codec->reg_cache);

	return ret;
}

static struct snd_soc_device *ak4641_socdev;

#if defined(CONFIG_I2C) || defined(CONFIG_I2C_MODULE)

static int ak4641_i2c_probe(struct i2c_client *i2c,
			    const struct i2c_device_id *id)
{
	struct snd_soc_device *socdev = ak4641_socdev;
	struct snd_soc_codec *codec = socdev->codec;
	int ret;

	i2c_set_clientdata(i2c, codec);
	codec->control_data = i2c;

	ret = ak4641_init(socdev);
	if (ret < 0)
		printk(KERN_ERR "failed to initialise AK4641\n");

	return ret;
}

static int __devexit ak4641_i2c_remove(struct i2c_client *client)
{
	struct snd_soc_codec *codec = i2c_get_clientdata(client);
	kfree(codec->reg_cache);
	return 0;
}

static const struct i2c_device_id ak4641_i2c_id[] = {
	{ "ak4641", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, ak4641_i2c_id);

static struct i2c_driver ak4641_i2c_driver = {
	.driver = {
		.name = "AK4641 I2C Codec",
		.owner = THIS_MODULE,
	},
	.probe =    ak4641_i2c_probe,
	.remove =   ak4641_i2c_remove,
	.id_table = ak4641_i2c_id,
};

static int ak4641_add_i2c_device(struct platform_device *pdev,
				 const struct ak4641_setup_data *setup)
{
	struct i2c_board_info info;
	struct i2c_adapter *adapter;
	struct i2c_client *client;
	int ret;

	ret = i2c_add_driver(&ak4641_i2c_driver);
	if (ret != 0) {
		dev_err(&pdev->dev, "can't add i2c driver\n");
		return ret;
	}

	memset(&info, 0, sizeof(struct i2c_board_info));
	info.addr = setup->i2c_address;
	strlcpy(info.type, "ak4641", I2C_NAME_SIZE);

	adapter = i2c_get_adapter(setup->i2c_bus);
	if (!adapter) {
		dev_err(&pdev->dev, "can't get i2c adapter %d\n",
			setup->i2c_bus);
		goto err_driver;
	}

	client = i2c_new_device(adapter, &info);
	i2c_put_adapter(adapter);
	if (!client) {
		dev_err(&pdev->dev, "can't add i2c device at 0x%x\n",
			(unsigned int)info.addr);
		goto err_driver;
	}

	return 0;

err_driver:
	i2c_del_driver(&ak4641_i2c_driver);
	return -ENODEV;
}
#endif

static int ak4641_probe(struct platform_device *pdev)
{
	struct snd_soc_device *socdev = platform_get_drvdata(pdev);
	struct ak4641_setup_data *setup;
	struct snd_soc_codec *codec;
	struct ak4641_priv *ak4641;
	int ret;

	printk(KERN_INFO "AK4641 Audio Codec %s", AK4641_VERSION);

	setup = socdev->codec_data;
	codec = kzalloc(sizeof(struct snd_soc_codec), GFP_KERNEL);
	if (codec == NULL)
		return -ENOMEM;

	ak4641 = kzalloc(sizeof(struct ak4641_priv), GFP_KERNEL);
	if (ak4641 == NULL) {
		kfree(codec);
		return -ENOMEM;
	}

	codec->private_data = ak4641;
	socdev->codec = codec;
	mutex_init(&codec->mutex);
	INIT_LIST_HEAD(&codec->dapm_widgets);
	INIT_LIST_HEAD(&codec->dapm_paths);

	ak4641_socdev = socdev;
	ret = -ENODEV;

#if defined(CONFIG_I2C) || defined(CONFIG_I2C_MODULE)
	if (setup->i2c_address) {
		codec->hw_write = (hw_write_t)i2c_master_send;
		codec->hw_read = (hw_read_t)i2c_master_recv;
		ret = ak4641_add_i2c_device(pdev, setup);
	}
#endif

	if (ret != 0) {
		kfree(codec->private_data);
		kfree(codec);
	}
	return ret;
}

/* power down chip */
static int ak4641_remove(struct platform_device *pdev)
{
	struct snd_soc_device *socdev = platform_get_drvdata(pdev);
	struct snd_soc_codec *codec = socdev->codec;

	if (codec->control_data)
		ak4641_set_bias_level(codec, SND_SOC_BIAS_OFF);

	snd_soc_free_pcms(socdev);
	snd_soc_dapm_free(socdev);
#if defined(CONFIG_I2C) || defined(CONFIG_I2C_MODULE)
	i2c_unregister_device(codec->control_data);
	i2c_del_driver(&ak4641_i2c_driver);
#endif
	kfree(codec->private_data);
	kfree(codec);

	return 0;
}

struct snd_soc_codec_device soc_codec_dev_ak4641 = {
	.probe = 	ak4641_probe,
	.remove = 	ak4641_remove,
	.suspend = 	ak4641_suspend,
	.resume =	ak4641_resume,
};
EXPORT_SYMBOL_GPL(soc_codec_dev_ak4641);

MODULE_DESCRIPTION("SoC AK4641 driver");
MODULE_AUTHOR("Harald Welte <laforge@gnufiish.org>");
MODULE_LICENSE("GPL");
