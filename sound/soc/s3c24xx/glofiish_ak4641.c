/*
 * glofiish_ak4641.c  --  SoC audio for E-TEN glofiish M800
 *
 * Copyright 2008 Harald Welte <laforge@gnufiish.org>
 *
 *  This program is free software; you can redistribute  it and/or modify it
 *  under  the terms of  the GNU General  Public License as published by the
 *  Free Software Foundation;  either version 2 of the  License, or (at your
 *  option) any later version.
 *
 */

#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/timer.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <linux/i2c.h>
#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/soc.h>
#include <sound/soc-dapm.h>
#include <sound/tlv.h>

#include <asm/mach-types.h>
#include <asm/hardware/scoop.h>
#include <mach/regs-clock.h>
#include <mach/regs-gpio.h>
#include <mach/hardware.h>
#include <plat/audio.h>
#include <mach/glofiish.h>
#include <linux/io.h>
#include <mach/spi-gpio.h>
#include <asm/mach-types.h>
#include <plat/regs-iis.h>

#include "../codecs/ak4641.h"
#include "s3c24xx-pcm.h"
#include "s3c24xx-i2s.h"

/* Debugging stuff */
#define S3C24XX_SOC_GFISH_AK4641_DEBUG 1
#if S3C24XX_SOC_GFISH_AK4641_DEBUG
#define DBG(x...) printk(KERN_DEBUG "s3c24xx-soc-gfish-ak4641: " x)
#else
#define DBG(x...)
#endif

/* define the scenarios */
#define NEO_AUDIO_OFF			0
#define NEO_GSM_CALL_AUDIO_HANDSET	1
#define NEO_GSM_CALL_AUDIO_HEADSET	2
#define NEO_GSM_CALL_AUDIO_BLUETOOTH	3
#define NEO_STEREO_TO_SPEAKERS		4
#define NEO_STEREO_TO_HEADPHONES	5
#define NEO_CAPTURE_HANDSET		6
#define NEO_CAPTURE_HEADSET		7
#define NEO_CAPTURE_BLUETOOTH		8

static struct snd_soc_machine gfish;

static int gfish_hifi_hw_params(struct snd_pcm_substream *substream,
	struct snd_pcm_hw_params *params)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai *codec_dai = rtd->dai->codec_dai;
	struct snd_soc_dai *cpu_dai = rtd->dai->cpu_dai;
	unsigned int pll_out = 0, bclk = 0;
	int ret = 0;
	unsigned long iis_clkrate;

	DBG("Entered %s\n", __func__);

	iis_clkrate = s3c24xx_i2s_get_clockrate();

#if 0
	switch (params_rate(params)) {
	case 8000:
	case 16000:
		pll_out = 12288000;
		break;
	case 48000:
		bclk = WM8753_BCLK_DIV_4;
		pll_out = 12288000;
		break;
	case 96000:
		bclk = WM8753_BCLK_DIV_2;
		pll_out = 12288000;
		break;
	case 11025:
		bclk = WM8753_BCLK_DIV_16;
		pll_out = 11289600;
		break;
	case 22050:
		bclk = WM8753_BCLK_DIV_8;
		pll_out = 11289600;
		break;
	case 44100:
		bclk = WM8753_BCLK_DIV_4;
		pll_out = 11289600;
		break;
	case 88200:
		bclk = WM8753_BCLK_DIV_2;
		pll_out = 11289600;
		break;
	}

	/* set codec DAI configuration */
	ret = snd_soc_dai_set_fmt(codec_dai,
		SND_SOC_DAIFMT_I2S | SND_SOC_DAIFMT_NB_NF |
		SND_SOC_DAIFMT_CBM_CFM);
	if (ret < 0)
		return ret;

	/* set cpu DAI configuration */
	ret = snd_soc_dai_set_fmt(cpu_dai,
		SND_SOC_DAIFMT_I2S | SND_SOC_DAIFMT_NB_NF |
		SND_SOC_DAIFMT_CBM_CFM);
	if (ret < 0)
		return ret;

	/* set the codec system clock for DAC and ADC */
	ret = snd_soc_dai_set_sysclk(codec_dai, WM8753_MCLK, pll_out,
		SND_SOC_CLOCK_IN);
	if (ret < 0)
		return ret;

	/* set MCLK division for sample rate */
	ret = snd_soc_dai_set_clkdiv(cpu_dai, S3C24XX_DIV_MCLK,
		S3C2410_IISMOD_32FS);
	if (ret < 0)
		return ret;

	/* set codec BCLK division for sample rate */
	ret = snd_soc_dai_set_clkdiv(codec_dai, WM8753_BCLKDIV, bclk);
	if (ret < 0)
		return ret;

	/* set prescaler division for sample rate */
	ret = snd_soc_dai_set_clkdiv(cpu_dai, S3C24XX_DIV_PRESCALER,
		S3C24XX_PRESCALE(4, 4));
	if (ret < 0)
		return ret;

	/* codec PLL input is PCLK/4 */
	ret = snd_soc_dai_set_pll(codec_dai, WM8753_PLL1,
		iis_clkrate / 4, pll_out);
	if (ret < 0)
		return ret;
#endif

	return 0;
}

static int gfish_hifi_hw_free(struct snd_pcm_substream *substream)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai *codec_dai = rtd->dai->codec_dai;

	DBG("Entered %s\n", __func__);
	
	/* disable the PLL */
	//return snd_soc_dai_set_pll(codec_dai, WM8753_PLL1, 0, 0);
	return 0;
}

/*
 * glofiish AK4641 HiFi DAI opserations.
 */
static struct snd_soc_ops gfish_hifi_ops = {
	.hw_params = gfish_hifi_hw_params,
	.hw_free = gfish_hifi_hw_free,
};

static int gfish_voice_hw_params(struct snd_pcm_substream *substream,
	struct snd_pcm_hw_params *params)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai *codec_dai = rtd->dai->codec_dai;
	unsigned int pcmdiv = 0;
	int ret = 0;
	unsigned long iis_clkrate;

	DBG("Entered %s\n", __func__);

	iis_clkrate = s3c24xx_i2s_get_clockrate();

	if (params_rate(params) != 8000)
		return -EINVAL;
	if (params_channels(params) != 1)
		return -EINVAL;

#if 0
	pcmdiv = WM8753_PCM_DIV_6; /* 2.048 MHz */

	/* todo: gg check mode (DSP_B) against CSR datasheet */
	/* set codec DAI configuration */
	ret = snd_soc_dai_set_fmt(codec_dai, SND_SOC_DAIFMT_DSP_B |
		SND_SOC_DAIFMT_NB_NF | SND_SOC_DAIFMT_CBS_CFS);
	if (ret < 0)
		return ret;

	/* set the codec system clock for DAC and ADC */
	ret = snd_soc_dai_set_sysclk(codec_dai, WM8753_PCMCLK, 12288000,
		SND_SOC_CLOCK_IN);
	if (ret < 0)
		return ret;

	/* set codec PCM division for sample rate */
	ret = snd_soc_dai_set_clkdiv(codec_dai, WM8753_PCMDIV, pcmdiv);
	if (ret < 0)
		return ret;

	/* configue and enable PLL for 12.288MHz output */
	ret = snd_soc_dai_set_pll(codec_dai, WM8753_PLL2,
		iis_clkrate / 4, 12288000);
	if (ret < 0)
		return ret;
#endif
	return 0;
}

static int gfish_voice_hw_free(struct snd_pcm_substream *substream)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai *codec_dai = rtd->dai->codec_dai;

	DBG("Entered %s\n", __func__);

	/* disable the PLL */
	//return snd_soc_dai_set_pll(codec_dai, WM8753_PLL2, 0, 0);
	return 0;
}

static struct snd_soc_ops gfish_voice_ops = {
	.hw_params = gfish_voice_hw_params,
	.hw_free = gfish_voice_hw_free,
};

static int neo1973_scenario;

static int neo1973_get_scenario(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	ucontrol->value.integer.value[0] = neo1973_scenario;
	return 0;
}

static int set_scenario_endpoints(struct snd_soc_codec *codec, int scenario)
{
	DBG("Entered %s\n", __func__);

	switch (neo1973_scenario) {
	case NEO_AUDIO_OFF:
		snd_soc_dapm_disable_pin(codec, "Audio Out");
		snd_soc_dapm_disable_pin(codec, "GSM Line Out");
		snd_soc_dapm_disable_pin(codec, "GSM Line In");
		snd_soc_dapm_disable_pin(codec, "Headset Mic");
		snd_soc_dapm_disable_pin(codec, "Call Mic");
		break;
	case NEO_GSM_CALL_AUDIO_HANDSET:
		snd_soc_dapm_enable_pin(codec, "Audio Out");
		snd_soc_dapm_enable_pin(codec, "GSM Line Out");
		snd_soc_dapm_enable_pin(codec, "GSM Line In");
		snd_soc_dapm_disable_pin(codec, "Headset Mic");
		snd_soc_dapm_enable_pin(codec, "Call Mic");
		break;
	case NEO_GSM_CALL_AUDIO_HEADSET:
		snd_soc_dapm_enable_pin(codec, "Audio Out");
		snd_soc_dapm_enable_pin(codec, "GSM Line Out");
		snd_soc_dapm_enable_pin(codec, "GSM Line In");
		snd_soc_dapm_enable_pin(codec, "Headset Mic");
		snd_soc_dapm_disable_pin(codec, "Call Mic");
		break;
	case NEO_GSM_CALL_AUDIO_BLUETOOTH:
		snd_soc_dapm_disable_pin(codec, "Audio Out");
		snd_soc_dapm_enable_pin(codec, "GSM Line Out");
		snd_soc_dapm_enable_pin(codec, "GSM Line In");
		snd_soc_dapm_disable_pin(codec, "Headset Mic");
		snd_soc_dapm_disable_pin(codec, "Call Mic");
		break;
	case NEO_STEREO_TO_SPEAKERS:
		snd_soc_dapm_enable_pin(codec, "Audio Out");
		snd_soc_dapm_disable_pin(codec, "GSM Line Out");
		snd_soc_dapm_disable_pin(codec, "GSM Line In");
		snd_soc_dapm_disable_pin(codec, "Headset Mic");
		snd_soc_dapm_disable_pin(codec, "Call Mic");
		break;
	case NEO_STEREO_TO_HEADPHONES:
		snd_soc_dapm_enable_pin(codec, "Audio Out");
		snd_soc_dapm_disable_pin(codec, "GSM Line Out");
		snd_soc_dapm_disable_pin(codec, "GSM Line In");
		snd_soc_dapm_disable_pin(codec, "Headset Mic");
		snd_soc_dapm_disable_pin(codec, "Call Mic");
		break;
	case NEO_CAPTURE_HANDSET:
		snd_soc_dapm_disable_pin(codec, "Audio Out");
		snd_soc_dapm_disable_pin(codec, "GSM Line Out");
		snd_soc_dapm_disable_pin(codec, "GSM Line In");
		snd_soc_dapm_disable_pin(codec, "Headset Mic");
		snd_soc_dapm_enable_pin(codec, "Call Mic");
		break;
	case NEO_CAPTURE_HEADSET:
		snd_soc_dapm_disable_pin(codec, "Audio Out");
		snd_soc_dapm_disable_pin(codec, "GSM Line Out");
		snd_soc_dapm_disable_pin(codec, "GSM Line In");
		snd_soc_dapm_enable_pin(codec, "Headset Mic");
		snd_soc_dapm_disable_pin(codec, "Call Mic");
		break;
	case NEO_CAPTURE_BLUETOOTH:
		snd_soc_dapm_disable_pin(codec, "Audio Out");
		snd_soc_dapm_disable_pin(codec, "GSM Line Out");
		snd_soc_dapm_disable_pin(codec, "GSM Line In");
		snd_soc_dapm_disable_pin(codec, "Headset Mic");
		snd_soc_dapm_disable_pin(codec, "Call Mic");
		break;
	default:
		snd_soc_dapm_disable_pin(codec, "Audio Out");
		snd_soc_dapm_disable_pin(codec, "GSM Line Out");
		snd_soc_dapm_disable_pin(codec, "GSM Line In");
		snd_soc_dapm_disable_pin(codec, "Headset Mic");
		snd_soc_dapm_disable_pin(codec, "Call Mic");
	}

	snd_soc_dapm_sync(codec);

	return 0;
}

static int neo1973_set_scenario(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_kcontrol_chip(kcontrol);

	DBG("Entered %s\n", __func__);

	if (neo1973_scenario == ucontrol->value.integer.value[0])
		return 0;

	neo1973_scenario = ucontrol->value.integer.value[0];
	set_scenario_endpoints(codec, neo1973_scenario);
	return 1;
}

static const struct snd_soc_dapm_widget ak4641_dapm_widgets[] = {
	SND_SOC_DAPM_LINE("Audio Out", NULL),
	SND_SOC_DAPM_LINE("GSM Line Out", NULL),
	SND_SOC_DAPM_LINE("GSM Line In", NULL),
	SND_SOC_DAPM_MIC("Headset Mic", NULL),
	SND_SOC_DAPM_MIC("Call Mic", NULL),
};


static const struct snd_soc_dapm_route dapm_routes[] = {
	/* Connections to headset and amp */
	{"Audio Out", NULL, "LOUT"},
	{"Audio Out", NULL, "ROUT"},

	/* Connections to the GSM Module */
	{"GSM Line Out", NULL, "MOUT1"},
	{"AUX", NULL, "GSM Line In"},

	/* Connections to Headset */
	{"MICEXT", NULL, "Headset Mic"},

	/* Call Mic */
	{"MICIN", NULL, "Call Mic"},

	/* Connect the ALC pins */
	{"AIN", NULL, "MICOUT"},
};

static const char *neo_scenarios[] = {
	"Off",
	"GSM Handset",
	"GSM Headset",
	"GSM Bluetooth",
	"Speakers",
	"Headphones",
	"Capture Handset",
	"Capture Headset",
	"Capture Bluetooth"
};

static const struct soc_enum neo_scenario_enum[] = {
	SOC_ENUM_SINGLE_EXT(ARRAY_SIZE(neo_scenarios), neo_scenarios),
};

static int m800_amp_get(struct snd_kcontrol *kcontrol,
			struct snd_ctl_elem_value *ucontrol)
{
	if (s3c2410_gpio_getpin(M800_GPIO_AUDIO_AMP))
		ucontrol->value.integer.value[0] = 1;
	else
		ucontrol->value.integer.value[0] = 0;

	return 0;
}

static int m800_amp_set(struct snd_kcontrol *kcontrol,
			struct snd_ctl_elem_value *ucontrol)
{
	if (ucontrol->value.integer.value[0])
		s3c2410_gpio_setpin(M800_GPIO_AUDIO_AMP, 1);
	else
		s3c2410_gpio_setpin(M800_GPIO_AUDIO_AMP, 0);

	return 1;
}

static const struct snd_kcontrol_new m800_controls[] = {
	SOC_SINGLE_BOOL_EXT("Amp Power", 0, m800_amp_get, m800_amp_set),
	SOC_ENUM_EXT("Neo Mode", neo_scenario_enum[0],
		neo1973_get_scenario, neo1973_set_scenario),
};

/*
 * This is an example machine initialisation for a ak4641 connected to a
 * glofiish M800. It is missing logic to detect hp/mic insertions and logic
 * to re-route the audio in such an event.
 */
static int gfish_ak4641_init(struct snd_soc_codec *codec)
{
	int i, err;

	DBG("Entered %s\n", __func__);

#if 0
	/* set up NC codec pins */
	snd_soc_dapm_nc_pin(codec, "LOUT2");
	snd_soc_dapm_nc_pin(codec, "ROUT2");
	snd_soc_dapm_nc_pin(codec, "OUT3");
	snd_soc_dapm_nc_pin(codec, "OUT4");
	snd_soc_dapm_nc_pin(codec, "LINE1");
	snd_soc_dapm_nc_pin(codec, "LINE2");
#endif

	/* Add neo1973 specific widgets */
	snd_soc_dapm_new_controls(codec, ak4641_dapm_widgets,
				  ARRAY_SIZE(ak4641_dapm_widgets));

	/* set endpoints to default mode */
	set_scenario_endpoints(codec, NEO_AUDIO_OFF);

	/* add M800 specific controls */
	for (i = 0; i < ARRAY_SIZE(m800_controls); i++) {
		err = snd_ctl_add(codec->card,
				snd_soc_cnew(&m800_controls[i],
				codec, NULL));
		if (err < 0)
			return err;
	}

	/* set up neo1973 specific audio routes */
	err = snd_soc_dapm_add_routes(codec, dapm_routes,
				      ARRAY_SIZE(dapm_routes));

	snd_soc_dapm_sync(codec);
	return 0;
}

/*
 * BT Codec DAI
 */
static struct snd_soc_dai bt_dai = {
	.name = "Bluetooth",
	.id = 0,
	.playback = {
		.channels_min = 1,
		.channels_max = 1,
		.rates = SNDRV_PCM_RATE_8000,
		.formats = SNDRV_PCM_FMTBIT_S16_LE,},
	.capture = {
		.channels_min = 1,
		.channels_max = 1,
		.rates = SNDRV_PCM_RATE_8000,
		.formats = SNDRV_PCM_FMTBIT_S16_LE,},
};

static struct snd_soc_dai_link gfish_dai[] = {
{ /* Hifi Playback - for similatious use with voice below */
	.name = "AK4641",
	.stream_name = "AK4641 HiFi",
	.cpu_dai = &s3c24xx_i2s_dai,
	.codec_dai = &ak4641_dai[0],
	.init = gfish_ak4641_init,
	.ops = &gfish_hifi_ops,
},
{ /* Voice via BT */
	.name = "Bluetooth",
	.stream_name = "Voice",
	.cpu_dai = &bt_dai,
	.codec_dai = &ak4641_dai[AK4641_DAI_VOICE],
	.ops = &gfish_voice_ops,
},
};

static struct snd_soc_card gfish_m800 = {
	.name = "glofiish M800",
    .platform = &s3c24xx_soc_platform,
	.dai_link = gfish_dai,
	.num_links = ARRAY_SIZE(gfish_dai),
};

static struct ak4641_setup_data soc_codec_data_ak4641_gfish = {
	.i2c_bus = 0,
	.i2c_address = 0x12,
};

static struct snd_soc_device gfish_snd_devdata = {
	.card = &gfish_m800,
	.codec_dev = &soc_codec_dev_ak4641,
	.codec_data = &soc_codec_data_ak4641_gfish,
};

static struct platform_device *gfish_snd_device;

static int __init gfish_init(void)
{
	int ret;

	printk(KERN_DEBUG "Entered %s\n", __func__);

	if (!machine_is_m800()) {
		printk(KERN_INFO
			"Only M800 hardware supported by ASoC driver\n");
		return -ENODEV;
	}

	gfish_snd_device = platform_device_alloc("soc-audio", -1);
	if (!gfish_snd_device)
		return -ENOMEM;

	platform_set_drvdata(gfish_snd_device, &gfish_snd_devdata);
	gfish_snd_devdata.dev = &gfish_snd_device->dev;
	ret = platform_device_add(gfish_snd_device);

	if (ret) {
		platform_device_put(gfish_snd_device);
		return ret;
	}

	return ret;
}

static void __exit gfish_exit(void)
{
	DBG("Entered %s\n", __func__);

	platform_device_unregister(gfish_snd_device);
}

module_init(gfish_init);
module_exit(gfish_exit);

/* Module information */
MODULE_AUTHOR("Harald Welte <laforge@gnufiish.org>");
MODULE_DESCRIPTION("ALSA SoC glofiish M800");
MODULE_LICENSE("GPL");
