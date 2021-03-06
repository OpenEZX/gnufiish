/* linux/arch/arm/plat-s3c64xx/cpufreq.c
 *
 * Copyright 2009 Wolfson Microelectronics plc
 *
 * S3C64XX CPUfreq Support
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/kernel.h>
#include <linux/types.h>
#include <linux/init.h>
#include <linux/cpufreq.h>
#include <linux/clk.h>
#include <linux/err.h>
#include <linux/regulator/consumer.h>

#include <mach/cpu.h>

static struct clk *armclk;
static struct regulator *vddarm;

struct s3c64xx_dvfs {
	unsigned int vddarm_min;
	unsigned int vddarm_max;
};

static struct s3c64xx_dvfs s3c6410_dvfs_table[] = {
	[0] = { 1000000, 1000000 },
	[1] = { 1000000, 1050000 },
	[2] = { 1050000, 1100000 },
	[3] = { 1050000, 1150000 },
	[4] = { 1250000, 1350000 },
};

static struct cpufreq_frequency_table s3c6410_freq_table[] = {
	{ 0,  66000 },
	{ 0, 133000 },
	{ 1, 222000 },
	{ 1, 266000 },
	{ 2, 333000 },
	{ 2, 400000 },
	{ 3, 532000 },
	{ 3, 533000 },
	{ 4, 667000 },
	{ 0, CPUFREQ_TABLE_END },
};

/* Data tables for current CPU and maximum index into it */
static struct cpufreq_frequency_table *s3c64xx_freq_table;
static struct s3c64xx_dvfs *s3c64xx_dvfs_table;

static int s3c64xx_cpufreq_verify_speed(struct cpufreq_policy *policy)
{
	if (policy->cpu != 0)
		return -EINVAL;

	return cpufreq_frequency_table_verify(policy, s3c64xx_freq_table);
}

static unsigned int s3c64xx_cpufreq_get_speed(unsigned int cpu)
{
	if (cpu != 0)
		return 0;

	return clk_get_rate(armclk) / 1000;
}

static int s3c64xx_cpufreq_set_target(struct cpufreq_policy *policy,
				      unsigned int target_freq,
				      unsigned int relation)
{
	int ret = 0;
	unsigned int i;
	struct cpufreq_freqs freqs;
	struct s3c64xx_dvfs *dvfs;

	ret = cpufreq_frequency_table_target(policy, s3c64xx_freq_table,
					     target_freq, relation, &i);
	if (ret != 0)
		return ret;

	freqs.cpu = 0;
	freqs.old = clk_get_rate(armclk) / 1000;
	freqs.new = s3c64xx_freq_table[i].frequency;
	freqs.flags = 0;
	dvfs = &s3c64xx_dvfs_table[s3c64xx_freq_table[i].index];

	if (freqs.old == freqs.new)
		return 0;

	pr_debug("cpufreq: Transition %d-%dkHz\n", freqs.old, freqs.new);

	cpufreq_notify_transition(&freqs, CPUFREQ_PRECHANGE);

#ifdef CONFIG_REGULATOR
	if (vddarm && freqs.new > freqs.old) {
		ret = regulator_set_voltage(vddarm,
					    dvfs->vddarm_min,
					    dvfs->vddarm_max);
		if (ret != 0) {
			pr_err("cpufreq: Failed to set VDDARM for %dkHz: %d\n",
			       freqs.new, ret);
			goto err;
		}
	}
#endif

	ret = clk_set_rate(armclk, freqs.new * 1000);
	if (ret < 0) {
		pr_err("cpufreq: Failed to set rate %dkHz: %d\n",
		       freqs.new, ret);
		goto err;
	}

#ifdef CONFIG_REGULATOR
	if (vddarm && freqs.new < freqs.old) {
		ret = regulator_set_voltage(vddarm,
					    dvfs->vddarm_min,
					    dvfs->vddarm_max);
		if (ret != 0) {
			pr_err("cpufreq: Failed to set VDDARM for %dkHz: %d\n",
			       freqs.new, ret);
			goto err_clk;
		}
	}
#endif

	cpufreq_notify_transition(&freqs, CPUFREQ_POSTCHANGE);

	pr_debug("cpufreq: Set actual frequency %lukHz\n",
		 clk_get_rate(armclk) / 1000);

	return 0;

err_clk:
	if (clk_set_rate(armclk, freqs.old * 1000) < 0)
		pr_err("Failed to restore original clock rate\n");
err:
	cpufreq_notify_transition(&freqs, CPUFREQ_POSTCHANGE);

	return ret;
}


static int __init s3c64xx_cpufreq_driver_init(struct cpufreq_policy *policy)
{
	int ret;
	struct cpufreq_frequency_table *freq;;

	if (policy->cpu != 0)
		return -EINVAL;

	if (cpu_is_s3c6410()) {
		s3c64xx_freq_table = s3c6410_freq_table;
		s3c64xx_dvfs_table = s3c6410_dvfs_table;
	}

	if (s3c64xx_freq_table == NULL) {
		pr_err("cpufreq: No frequency information for this CPU\n");
		return -ENODEV;
	}

	armclk = clk_get(NULL, "armclk");
	if (IS_ERR(armclk)) {
		pr_err("cpufreq: Unable to obtain ARMCLK: %ld\n",
		       PTR_ERR(armclk));
		return PTR_ERR(armclk);
	}

#ifdef CONFIG_REGULATOR
	vddarm = regulator_get(NULL, "vddarm");
	if (IS_ERR(vddarm)) {
		ret = PTR_ERR(vddarm);
		pr_err("cpufreq: Failed to obtain VDDARM: %d\n", ret);
		pr_err("cpufreq: Only frequency scaling available\n");
		vddarm = NULL;
	}
#endif

	/* Check for frequencies we can generate */
	freq = s3c64xx_freq_table;
	while (freq->frequency != CPUFREQ_TABLE_END) {
		unsigned long r;

		r = clk_round_rate(armclk, freq->frequency * 1000);
		r /= 1000;

		if (r != freq->frequency)
			freq->frequency = CPUFREQ_ENTRY_INVALID;

		freq++;
	}

	policy->cur = clk_get_rate(armclk) / 1000;
	policy->cpuinfo.transition_latency = CPUFREQ_ETERNAL;

	ret = cpufreq_frequency_table_cpuinfo(policy, s3c64xx_freq_table);
	if (ret == 0)
		return ret;

	pr_err("cpufreq: Failed to configure frequency table: %d\n", ret);

	regulator_put(vddarm);
	clk_put(armclk);
	return ret;
}

static struct cpufreq_driver s3c64xx_cpufreq_driver = {
	.owner		= THIS_MODULE,
	.flags          = 0,
	.verify		= s3c64xx_cpufreq_verify_speed,
	.target		= s3c64xx_cpufreq_set_target,
	.get		= s3c64xx_cpufreq_get_speed,
	.init		= s3c64xx_cpufreq_driver_init,
	.name		= "s3c64xx",
};

static int __init s3c64xx_cpufreq_init(void)
{
	return cpufreq_register_driver(&s3c64xx_cpufreq_driver);
}
module_init(s3c64xx_cpufreq_init);
