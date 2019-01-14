// SPDX-License-Identifier: GPL-2.0
// Copyright(c) 2018 Intel Corporation.

/*
 * Intel Cometlake I2S Machine Driver with RT5682 Codecs
 *
 * Modified from:
 *   Intel Apollolake I2S Machine driver
 */

#include <linux/input.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <sound/core.h>
#include <sound/jack.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>
#include "../../codecs/rt5682.h"
#include "../../codecs/hdac_hdmi.h"

#define CML_PLAT_CLK_FREQ 24000000
#define RT5682_PLL_FREQ (48000 * 512)
#define DUAL_CHANNEL 2
#define QUAD_CHANNEL 4
#define NAME_SIZE 32

static struct snd_soc_jack cometlake_hdmi[3];

struct cml_hdmi_pcm {
	struct list_head head;
	struct snd_soc_dai *codec_dai;
	int device;
};

struct cml_card_private {
	struct snd_soc_jack cometlake_headset;
	struct list_head hdmi_pcm_list;
};

enum {
	CML_DPCM_AUDIO_PB = 0,
	CML_DPCM_AUDIO_CP,
	CML_DPCM_AUDIO_HS_PB,
	CML_DPCM_AUDIO_ECHO_REF_CP,
	CML_DPCM_AUDIO_REF_CP,
	CML_DPCM_AUDIO_DMIC_CP,
	CML_DPCM_AUDIO_HDMI1_PB,
	CML_DPCM_AUDIO_HDMI2_PB,
	CML_DPCM_AUDIO_HDMI3_PB,
};

static int cometlake_ssp_fixup(struct snd_soc_pcm_runtime *rtd,
			struct snd_pcm_hw_params *params)
{
	struct snd_interval *rate = hw_param_interval(params,
			SNDRV_PCM_HW_PARAM_RATE);
	struct snd_interval *channels = hw_param_interval(params,
			SNDRV_PCM_HW_PARAM_CHANNELS);
	struct snd_mask *fmt = hw_param_mask(params, SNDRV_PCM_HW_PARAM_FORMAT);

	/* The ADSP will convert the FE rate to 48k, stereo */
	rate->min = rate->max = 48000;
	channels->min = channels->max = DUAL_CHANNEL;

	/* set SSP to 24 bit */
	snd_mask_none(fmt);
	snd_mask_set(fmt, SNDRV_PCM_FORMAT_S24_LE);

	return 0;
}

static int cometlake_rt5682_hw_params(struct snd_pcm_substream *substream,
	struct snd_pcm_hw_params *params)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai *codec_dai = rtd->codec_dai;
	int ret;

	/* Set valid bitmask & configuration for I2S in 24 bit */
	ret = snd_soc_dai_set_tdm_slot(codec_dai, 0x0, 0x0, 2, 24);
	if (ret < 0) {
		dev_err(rtd->dev, "set TDM slot err:%d\n", ret);
		return ret;
	}

	return ret;
}

static struct snd_soc_ops cometlake_rt5682_ops = {
	.hw_params = cometlake_rt5682_hw_params,
};

static int cometlake_hdmi_init(struct snd_soc_pcm_runtime *rtd)
{
	struct cml_card_private *ctx = snd_soc_card_get_drvdata(rtd->card);
	struct snd_soc_dai *dai = rtd->codec_dai;
	struct cml_hdmi_pcm *pcm;

	pcm = devm_kzalloc(rtd->card->dev, sizeof(*pcm), GFP_KERNEL);
	if (!pcm)
		return -ENOMEM;

	pcm->device = CML_DPCM_AUDIO_HDMI1_PB + dai->id;
	pcm->codec_dai = dai;

	list_add_tail(&pcm->head, &ctx->hdmi_pcm_list);

	return 0;
}

static int cometlake_rt5682_codec_init(struct snd_soc_pcm_runtime *rtd)
{
	struct cml_card_private *ctx = snd_soc_card_get_drvdata(rtd->card);
	struct snd_soc_component *component = rtd->codec_dai->component;
	struct snd_soc_dai *codec_dai = rtd->codec_dai;
	struct snd_soc_jack *jack;
	int ret;

	ret = snd_soc_dai_set_pll(codec_dai, 0, RT5682_PLL1_S_MCLK,
					CML_PLAT_CLK_FREQ, RT5682_PLL_FREQ);
	if (ret < 0) {
		dev_err(rtd->dev, "can't set codec pll: %d\n", ret);
		return ret;
	}

	/* Configure sysclk for codec */
	ret = snd_soc_dai_set_sysclk(codec_dai, RT5682_SCLK_S_PLL1,
					RT5682_PLL_FREQ, SND_SOC_CLOCK_IN);
	if (ret < 0)
		dev_err(rtd->dev, "snd_soc_dai_set_sysclk err = %d\n", ret);

	/*
	 * Headset buttons map to the google Reference headset.
	 * These can be configured by userspace.
	 */
	ret = snd_soc_card_jack_new(rtd->card, "Headset Jack",
			SND_JACK_HEADSET | SND_JACK_BTN_0 | SND_JACK_BTN_1 |
			SND_JACK_BTN_2 | SND_JACK_BTN_3 | SND_JACK_LINEOUT,
			&ctx->cometlake_headset, NULL, 0);
	if (ret) {
		dev_err(rtd->dev, "Headset Jack creation failed: %d\n", ret);
		return ret;
	}

	jack = &ctx->cometlake_headset;

	snd_jack_set_key(jack->jack, SND_JACK_BTN_0, KEY_PLAYPAUSE);
	snd_jack_set_key(jack->jack, SND_JACK_BTN_1, KEY_VOLUMEUP);
	snd_jack_set_key(jack->jack, SND_JACK_BTN_2, KEY_VOLUMEDOWN);
	snd_jack_set_key(jack->jack, SND_JACK_BTN_3, KEY_VOICECOMMAND);
	ret = snd_soc_component_set_jack(component, jack, NULL);

	if (ret) {
		dev_err(rtd->dev, "Headset Jack call-back failed: %d\n", ret);
		return ret;
	}

	return ret;
};

static int cometlake_dmic_fixup(struct snd_soc_pcm_runtime *rtd,
		struct snd_pcm_hw_params *params)
{
	struct snd_interval *channels = hw_param_interval(params,
				SNDRV_PCM_HW_PARAM_CHANNELS);

	/*
	 * set BE channel constraint as user FE channels
	 */
	channels->min = channels->max = 4;

	return 0;
}

/* cometlake digital audio interface glue - connects codec <--> CPU */
static struct snd_soc_dai_link cometlake_dais[] = {
	/* Back End DAI links */
	{
		/* SSP1 - Codec */
		.name = "SSP1-Codec",
		.id = 0,
		.cpu_dai_name = "SSP1 Pin",
		.platform_name = "sof-audio",
		.no_pcm = 1,
		.codec_name = "i2c-10EC5682:00",
		.codec_dai_name = "rt5682-aif1",
		.init = cometlake_rt5682_codec_init,
		.dai_fmt = SND_SOC_DAIFMT_I2S | SND_SOC_DAIFMT_NB_NF |
			SND_SOC_DAIFMT_CBS_CFS,
		.ignore_pmdown_time = 1,
		.be_hw_params_fixup = cometlake_ssp_fixup,
		.ops = &cometlake_rt5682_ops,
		.dpcm_playback = 1,
		.dpcm_capture = 1,
	},
	{
		.name = "dmic01",
		.id = 1,
		.cpu_dai_name = "DMIC01 Pin",
		.codec_name = "dmic-codec",
		.codec_dai_name = "dmic-hifi",
		.platform_name = "0000:00:0e.0",
		.ignore_suspend = 1,
		.be_hw_params_fixup = cometlake_dmic_fixup,
		.dpcm_capture = 1,
		.no_pcm = 1,
	},
	{
		.name = "iDisp1",
		.id = 2,
		.cpu_dai_name = "iDisp1 Pin",
		.codec_name = "ehdaudio0D2",
		.codec_dai_name = "intel-hdmi-hifi1",
		.platform_name = "0000:00:05.0",
		.init = cometlake_hdmi_init,
		.dpcm_playback = 1,
		.no_pcm = 1,
	},
	{
		.name = "iDisp2",
		.id = 3,
		.cpu_dai_name = "iDisp2 Pin",
		.codec_name = "ehdaudio0D2",
		.codec_dai_name = "intel-hdmi-hifi2",
		.platform_name = "0000:00:05.0",
		.init = cometlake_hdmi_init,
		.dpcm_playback = 1,
		.no_pcm = 1,
	},
	{
		.name = "iDisp3",
		.id = 4,
		.cpu_dai_name = "iDisp3 Pin",
		.codec_name = "ehdaudio0D2",
		.codec_dai_name = "intel-hdmi-hifi3",
		.platform_name = "0000:00:05.0",
		.init = cometlake_hdmi_init,
		.dpcm_playback = 1,
		.no_pcm = 1,
	},
};

static int cml_card_late_probe(struct snd_soc_card *card)
{
	struct cml_card_private *ctx = snd_soc_card_get_drvdata(card);
	struct snd_soc_component *component = NULL;
	char jack_name[NAME_SIZE];
	struct cml_hdmi_pcm *pcm;
	int err = 0;
	int i = 0;

	list_for_each_entry(pcm, &ctx->hdmi_pcm_list, head) {
		component = pcm->codec_dai->component;
		snprintf(jack_name, sizeof(jack_name),
			"HDMI/DP, pcm=%d Jack", pcm->device);
		err = snd_soc_card_jack_new(card, jack_name,
					SND_JACK_AVOUT, &cometlake_hdmi[i],
					NULL, 0);

		if (err)
			return err;

		err = hdac_hdmi_jack_init(pcm->codec_dai, pcm->device,
						&cometlake_hdmi[i]);
		if (err < 0)
			return err;

		i++;
	}

	if (!component)
		return -EINVAL;

	return hdac_hdmi_jack_port_init(component, &card->dapm);
}

/* cometlake audio machine driver for RT5682 */
static struct snd_soc_card cml_audio_card_rt5682 = {
	.name = "cml_rt5682",
	.owner = THIS_MODULE,
	.dai_link = cometlake_dais,
	.num_links = ARRAY_SIZE(cometlake_dais),
	.fully_routed = false,
	.late_probe = cml_card_late_probe,
};

static int cometlake_audio_probe(struct platform_device *pdev)
{
	struct cml_card_private *ctx;

	ctx = devm_kzalloc(&pdev->dev, sizeof(*ctx), GFP_KERNEL);
	if (!ctx)
		return -ENOMEM;

	INIT_LIST_HEAD(&ctx->hdmi_pcm_list);

	cml_audio_card_rt5682.dev = &pdev->dev;
	snd_soc_card_set_drvdata(&cml_audio_card_rt5682, ctx);
	return devm_snd_soc_register_card(&pdev->dev,
					&cml_audio_card_rt5682);
}

static const struct platform_device_id cml_board_ids[] = {
	{
		.name = "cml_rt5682",
		.driver_data =
			(kernel_ulong_t)&cml_audio_card_rt5682,
	},
	{ }
};

static struct platform_driver cometlake_audio = {
	.probe = cometlake_audio_probe,
	.driver = {
		.name = "cml_rt5682",
		.pm = &snd_soc_pm_ops,
	},
	.id_table = cml_board_ids,
};
module_platform_driver(cometlake_audio)

/* Module information */
MODULE_DESCRIPTION("CometLake Audio Machine driver-RT5682 in I2S mode");
MODULE_AUTHOR("Zhu Yingjiang<yingjiang.zhu@intel.com>");
MODULE_LICENSE("GPL v2");
MODULE_ALIAS("platform:cml_rt5682");
