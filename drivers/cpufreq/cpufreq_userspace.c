/*
 * Copyright (C) 2001, Russell King.
 * Copyright (C) 2002-2004, Dominik Brodowski <linux@brodo.de>
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#define pr_fmt(fmt)	KBUILD_MODNAME ": " fmt

#include <linux/cpufreq.h>
#include <linux/module.h>
#include <linux/err.h>

struct us_cpu_info_s {
	bool cpu_is_managed;
	u32 cpu_max_freq;
	u32 cpu_min_freq;
	u32 cpu_cur_freq;
	u32 cpu_set_freq;
};
static DEFINE_PER_CPU(struct us_cpu_info_s, us_cpu_info);

/* Number of CPUs that currently use this governor */
static u32 gov_enable_cnt;

/* userspace_mutex protects governor frequency transitions */
static DEFINE_MUTEX(userspace_mutex);

/**
 * set_speed() - set the CPU frequency to a passed value.
 * @policy: pointer to cpufreq policy, where frequency is being set.
 * @freq: target frequency in kHz.
 *
 * Sets the CPU frequency to a passed value aligning to frequency limits.
 */
static int set_speed(struct cpufreq_policy *policy, u32 freq)
{
	struct us_cpu_info_s *info = &per_cpu(us_cpu_info, policy->cpu);
	int ret = -EINVAL;

	mutex_lock(&userspace_mutex);
	if (!info->cpu_is_managed)
		goto err;

	pr_debug("%s for cpu%u, freq -> %ukHz\n", __func__, policy->cpu, freq);

	freq = min(max(freq, info->cpu_min_freq), info->cpu_max_freq);
	info->cpu_set_freq = freq;

	/*
	 * We're safe from concurrent calls to ->target() here as we hold the
	 * userspace_mutex lock.  If we were calling cpufreq_driver_target, a
	 * deadlock situation might occur:
	 * A: set_speed (lock userspace_mutex) ->
	 *      cpufreq_driver_target(lock policy->lock).
	 * B: set_speed_policy(lock policy->lock) ->
	 *      __cpufreq_governor ->
	 *          cpufreq_governor_algorithm (lock userspace_mutex).
	 */
	ret = __cpufreq_driver_target(policy, freq, CPUFREQ_RELATION_L);

err:
	mutex_unlock(&userspace_mutex);

	return ret;
}

/**
 * show_speed() - return a saved current frequency of a CPU.
 * @policy: pointer to cpufreq policy, where frequency was set.
 * @buf: buffer for returnable data.
 *
 * Returns userspace current frequency of a CPU.
 */
static ssize_t show_speed(struct cpufreq_policy *policy, char *buf)
{
	struct us_cpu_info_s *info = &per_cpu(us_cpu_info, policy->cpu);

	return scnprintf(buf, sizeof(u32) * 8 + 1, "%u\n", info->cpu_cur_freq);
}

static int cpufreq_notifier(struct notifier_block *nb,
			    unsigned long val, void *data)
{
	struct cpufreq_freqs *freq = data;
	struct us_cpu_info_s *info = &per_cpu(us_cpu_info, freq->cpu);

	if (!info->cpu_is_managed || val != CPUFREQ_POSTCHANGE)
		return NOTIFY_OK;

	pr_debug("saving cpu_cur_freq of cpu%u to be%u kHz\n",
		  freq->cpu, freq->new);
	info->cpu_cur_freq = freq->new;

	return NOTIFY_DONE;
}

static struct notifier_block cpufreq_notifier_block = {
	.notifier_call = cpufreq_notifier,
};

static int cpufreq_governor_algorithm(struct cpufreq_policy *policy,
				      unsigned int event)
{
	int ret;
	u32 cpu = policy->cpu;
	struct us_cpu_info_s *info = &per_cpu(us_cpu_info, cpu);

	switch (event) {
	case CPUFREQ_GOV_START:
		if (!cpu_online(cpu) || !policy->cur)
			return -EINVAL;

		mutex_lock(&userspace_mutex);
		if (++gov_enable_cnt != 1)
			goto skip_init;

		ret = cpufreq_register_notifier(&cpufreq_notifier_block,
						CPUFREQ_TRANSITION_NOTIFIER);
		if (IS_ERR_VALUE(ret)) {
			pr_err("unable to register cpufreq notifier\n");
			mutex_unlock(&userspace_mutex);
			return ret;
		}

skip_init:
		info->cpu_min_freq = policy->min;
		info->cpu_max_freq = policy->max;
		info->cpu_cur_freq = info->cpu_set_freq = policy->cur;
		info->cpu_is_managed = true;

		pr_debug("cpu%u started to use userspace policy "
			 "(%u - %ukHz, currently %ukHz)\n", cpu,
			 info->cpu_min_freq,
			 info->cpu_max_freq,
			 info->cpu_cur_freq);
		mutex_unlock(&userspace_mutex);
		break;
	case CPUFREQ_GOV_STOP:
		mutex_lock(&userspace_mutex);
		info->cpu_is_managed = false;

		if (--gov_enable_cnt != 0)
			goto skip_exit;

		cpufreq_unregister_notifier(&cpufreq_notifier_block,
					    CPUFREQ_TRANSITION_NOTIFIER);

skip_exit:
		info->cpu_min_freq = info->cpu_max_freq = 0;
		info->cpu_cur_freq = info->cpu_set_freq = 0;

		pr_debug("cpu%u stopped to use userspace policy\n", cpu);
		mutex_unlock(&userspace_mutex);
		break;
	case CPUFREQ_GOV_LIMITS:
		mutex_lock(&userspace_mutex);
		pr_debug("limit event for cpu%u: %u - %ukHz, "
			 "currently %ukHz, last set to %ukHz\n",
			 cpu, policy->min, policy->max,
			 info->cpu_cur_freq,
			 info->cpu_set_freq);

		if (info->cpu_set_freq > policy->max)
			__cpufreq_driver_target(policy, policy->max,
						CPUFREQ_RELATION_H);
		else if (info->cpu_set_freq < policy->min)
			__cpufreq_driver_target(policy, policy->min,
						CPUFREQ_RELATION_L);
		else
			__cpufreq_driver_target(policy, info->cpu_set_freq,
						CPUFREQ_RELATION_L);

		info->cpu_min_freq = policy->min;
		info->cpu_max_freq = policy->max;
		info->cpu_cur_freq = policy->cur;
		mutex_unlock(&userspace_mutex);
		break;
	}

	return 0;
}

#ifndef CONFIG_CPU_FREQ_DEFAULT_GOV_USERSPACE
static
#endif
struct cpufreq_governor cpufreq_gov_userspace = {
	.name		= "userspace",
	.governor	= cpufreq_governor_algorithm,
	.store_setspeed	= set_speed,
	.show_setspeed	= show_speed,
	.owner		= THIS_MODULE,
};

static int __init cpufreq_governor_init(void)
{
	return cpufreq_register_governor(&cpufreq_gov_userspace);
}

static void __exit cpufreq_governor_exit(void)
{
	cpufreq_unregister_governor(&cpufreq_gov_userspace);
}

#ifdef CONFIG_CPU_FREQ_DEFAULT_GOV_USERSPACE
fs_initcall(cpufreq_governor_init);
#else
module_init(cpufreq_governor_init);
#endif
module_exit(cpufreq_governor_exit);

MODULE_AUTHOR("Dominik Brodowski <linux@brodo.de>, "
	      "Russell King <rmk@arm.linux.org.uk>");
MODULE_DESCRIPTION("CPUfreq policy governor 'userspace'");
MODULE_LICENSE("GPLv2");
