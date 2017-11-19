/*
 * Copyright (C) 2001, Russell King.
 * Copyright (C) 2003, Venkatesh Pallipadi <venkatesh.pallipadi@intel.com>
 * Copyright (C) 2003, Jun Nakajima <jun.nakajima@intel.com>
 * Copyright (C) 2009, Alexander Clouter <alex@digriz.org.uk>
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

#include "cpufreq_governor.h"

#define DEF_FREQUENCY_UP_THRESHOLD		(80)
#define DEF_FREQUENCY_DOWN_THRESHOLD		(20)
#define DEF_FREQUENCY_UP_STEP			(5)
#define DEF_FREQUENCY_DOWN_STEP			(10)

static DEFINE_PER_CPU(struct cs_cpu_dbs_info_s, cs_cpu_dbs_info);
define_get_cpu_dbs_routines(cs_cpu_dbs_info);

static struct cs_dbs_tuners cs_tuners = {
	.up_threshold = DEF_FREQUENCY_UP_THRESHOLD,
	.down_threshold = DEF_FREQUENCY_DOWN_THRESHOLD,
	.freq_up_step = DEF_FREQUENCY_UP_STEP,
	.freq_down_step = DEF_FREQUENCY_DOWN_STEP,
};

static inline u32 get_policy_max_load(struct cpufreq_policy *policy)
{
	u32 max_load = 0, cur_load, load_at_max_freq, j;

	for_each_cpu(j, policy->cpus) {
		struct cpu_dbs_common_info *j_cdbs = get_cpu_cdbs(j);
		u64 cur_idle_time, cur_wall_time;
		u32 idle_time, wall_time;

		cur_idle_time = get_cpu_idle_time(j, &cur_wall_time,
						  cs_tuners.io_is_busy);

		idle_time = (u32)(cur_idle_time - j_cdbs->prev_cpu_idle);
		j_cdbs->prev_cpu_idle = cur_idle_time;

		wall_time = (u32)(cur_wall_time - j_cdbs->prev_cpu_wall);
		j_cdbs->prev_cpu_wall = cur_wall_time;

		if (unlikely(!wall_time || wall_time < idle_time))
			continue;

		cur_load = 100 * (wall_time - idle_time) / wall_time;
		max_load = max(max_load, cur_load);
	}

	load_at_max_freq = max_load * policy->cur / policy->max;
	cpufreq_notify_utilization(policy, load_at_max_freq);

	return max_load;
}

static inline void scale_freq(struct cs_cpu_dbs_info_s *dbs_info,
			      struct cpufreq_policy *policy,
			      u32 freq_diff, bool decrease)
{
	if (policy->cur == (!decrease ? policy->max : policy->min))
		return;

	dbs_info->requested_freq += !decrease ? freq_diff : -freq_diff;
	dbs_info->requested_freq  = min(max((s32)dbs_info->requested_freq,
			(s32)policy->min), (s32)policy->max);

	__cpufreq_driver_target(policy, dbs_info->requested_freq,
				CPUFREQ_RELATION_C);
}

static void cs_check_cpu(struct cs_cpu_dbs_info_s *dbs_info)
{
	struct cpufreq_policy *policy = dbs_info->cdbs.cur_policy;
	u32 max_load = get_policy_max_load(policy), freq_diff;

	if (max_load >= cs_tuners.up_threshold) {
		freq_diff = (policy->max * cs_tuners.freq_up_step) / 100;
		scale_freq(dbs_info, policy, freq_diff, false);
	} else if (max_load <= cs_tuners.down_threshold) {
		freq_diff = (policy->max * cs_tuners.freq_down_step) / 100;
		scale_freq(dbs_info, policy, freq_diff, true);
	}
}

static int cs_cpufreq_notifier(struct notifier_block *nb,
			       unsigned long val, void *data)
{
	struct cpufreq_policy *policy;
	struct cpufreq_freqs *freq = data;
	struct cs_cpu_dbs_info_s *dbs_info = get_cpu_dbs_info_s(freq->cpu);

	if (!dbs_info->enabled)
		return NOTIFY_OK;

	policy = dbs_info->cdbs.cur_policy;

	/*
	 * We only care if our internally tracked freq moves outside the 'valid'
	 * ranges of freqency available to us. Otherwise we do not change it.
	 */
	if (dbs_info->requested_freq < policy->min ||
	    dbs_info->requested_freq > policy->max)
		dbs_info->requested_freq = freq->new;

	return NOTIFY_DONE;
}

static struct notifier_block cs_cpufreq_notifier_block = {
	.notifier_call = cs_cpufreq_notifier,
};

define_sampling_rate_min_node(cs);
define_one_dbs_node(cs, sampling_rate, min_sampling_rate, UINT_MAX);
define_one_dbs_node(cs, up_threshold, (cs_tuners.down_threshold + 1), 100);
define_one_dbs_node(cs, down_threshold, 1, (cs_tuners.up_threshold - 1));
define_one_dbs_node(cs, freq_up_step, 1, 100);
define_one_dbs_node(cs, freq_down_step, 1, 100);
define_one_dbs_node(cs, io_is_busy, 0, 1);

static struct attribute *dbs_attributes[] = {
	&sampling_rate_min.attr,
	&sampling_rate.attr,
	&up_threshold.attr,
	&down_threshold.attr,
	&freq_up_step.attr,
	&freq_down_step.attr,
	&io_is_busy.attr,
	NULL,
};

static struct attribute_group dbs_attr_group = {
	.name = "conservative",
	.attrs = dbs_attributes,
};

/* Conservative dbs_timer is initialized here */
define_gov_dbs_timer(cs, 1);

static inline void dbs_timer_init(struct cs_cpu_dbs_info_s *dbs_info)
{
	int delay = delay_for_sampling_rate(cs_tuners.sampling_rate, 1);

	dbs_info->enabled = true;

	INIT_DELAYED_WORK_DEFERRABLE(&dbs_info->cdbs.work, cs_dbs_timer);
	queue_delayed_work_on(dbs_info->cdbs.cpu, dbs_wq,
			     &dbs_info->cdbs.work, delay);
}

static inline void dbs_timer_exit(struct cs_cpu_dbs_info_s *dbs_info)
{
	dbs_info->enabled = false;
	cancel_delayed_work_sync(&dbs_info->cdbs.work);
}

static int cpufreq_governor_dbs(struct cpufreq_policy *policy,
				unsigned int event)
{
	int ret;
	u32 cpu = policy->cpu, latency, j;
	struct cs_cpu_dbs_info_s *dbs_info = get_cpu_dbs_info_s(cpu);
	struct cpu_dbs_common_info *cdbs = get_cpu_cdbs(cpu);

	switch (event) {
	case CPUFREQ_GOV_START:
		if (!cpu_online(cpu) || !policy->cur)
			return -EINVAL;

		mutex_lock(&dbs_mutex);
		for_each_cpu(j, policy->cpus) {
			struct cpu_dbs_common_info *j_cdbs = get_cpu_cdbs(j);

			j_cdbs->cur_policy = policy;
			j_cdbs->prev_cpu_idle = get_cpu_idle_time(j,
				&j_cdbs->prev_cpu_wall, should_io_be_busy());
		}

		cdbs->cpu = cpu;
		dbs_info->requested_freq = policy->cur;

		if (++gov_enable_cnt != 1)
			goto skip_init;

		/* Policy latency is in nS. Convert it to uS first. */
		latency = policy->cpuinfo.transition_latency / 1000;
		if (!latency)
			latency = 1;

		/* Bring kernel and HW constraints together */
		min_sampling_rate = max(min_sampling_rate,
				latency * MIN_LATENCY_MULTIPLIER);
		cs_tuners.sampling_rate = max(min_sampling_rate,
				latency * LATENCY_MULTIPLIER);

		if (likely(!cs_tuners.io_is_busy))
			cs_tuners.io_is_busy = should_io_be_busy();

		ret = cpufreq_register_notifier(&cs_cpufreq_notifier_block,
						CPUFREQ_TRANSITION_NOTIFIER);
		if (IS_ERR_VALUE(ret)) {
			pr_err("%s: unable to register cpufreq notifier\n",
						__func__);
			goto fail_notifier;
		}

		ret = sysfs_create_group(cpufreq_global_kobject,
					&dbs_attr_group);
		if (IS_ERR_VALUE(ret)) {
			pr_err("%s: unable to create sysfs group\n", __func__);
			goto fail_sysfs;
		}

skip_init:
		mutex_unlock(&dbs_mutex);

		dbs_timer_init(dbs_info);
		break;
fail_sysfs:
		cpufreq_unregister_notifier(&cs_cpufreq_notifier_block,
					    CPUFREQ_TRANSITION_NOTIFIER);
fail_notifier:
		mutex_unlock(&dbs_mutex);
		return ret;
	case CPUFREQ_GOV_STOP:
		dbs_timer_exit(dbs_info);

		mutex_lock(&dbs_mutex);
		if (--gov_enable_cnt != 0)
			goto skip_exit;

		sysfs_remove_group(cpufreq_global_kobject, &dbs_attr_group);
		cpufreq_unregister_notifier(&cs_cpufreq_notifier_block,
					    CPUFREQ_TRANSITION_NOTIFIER);

skip_exit:
		mutex_unlock(&dbs_mutex);
		break;
	case CPUFREQ_GOV_LIMITS:
		mutex_lock(&cdbs->timer_mutex);
		if (cdbs->cur_policy->cur > policy->max)
			__cpufreq_driver_target(cdbs->cur_policy,
					policy->max, CPUFREQ_RELATION_H);
		else if (cdbs->cur_policy->cur < policy->min)
			__cpufreq_driver_target(cdbs->cur_policy,
					policy->min, CPUFREQ_RELATION_L);
		cs_check_cpu(dbs_info);
		mutex_unlock(&cdbs->timer_mutex);
		break;
	}

	return 0;
}

#ifndef CONFIG_CPU_FREQ_DEFAULT_GOV_CONSERVATIVE
static
#endif
struct cpufreq_governor cpufreq_gov_conservative = {
	.name			= "conservative",
	.governor		= cpufreq_governor_dbs,
	.max_transition_latency	= TRANSITION_LATENCY_LIMIT,
	.owner			= THIS_MODULE,
};

static int __init cpufreq_gov_dbs_init(void)
{
	int cpu;

	dbs_wq = alloc_workqueue("conservative_dbs_wq", WQ_HIGHPRI, 0);
	if (IS_ERR_OR_NULL(dbs_wq)) {
		pr_err("%s: unable to allocate workqueue\n", __func__);
		return -EFAULT;
	}

	for_each_possible_cpu(cpu) {
		struct cpu_dbs_common_info *cdbs = get_cpu_cdbs(cpu);
		mutex_init(&cdbs->timer_mutex);
	}

	/*
	 * In NOHZ/micro accounting case we set the minimum frequency
	 * not depending on HZ, but fixed (very low).  The deferred
	 * timer might skip some samples if idle/sleeping as needed.
	 */
	if (nohz_sampling_rate_is_used())
		min_sampling_rate = MICRO_FREQUENCY_MIN_SAMPLE_RATE;
	else
		min_sampling_rate = sampling_rate_by_jiffies();

	return cpufreq_register_governor(&cpufreq_gov_conservative);
}

static void __exit cpufreq_gov_dbs_exit(void)
{
	int cpu;

	cpufreq_unregister_governor(&cpufreq_gov_conservative);

	for_each_possible_cpu(cpu) {
		struct cpu_dbs_common_info *cdbs = get_cpu_cdbs(cpu);
		mutex_destroy(&cdbs->timer_mutex);
	}

	destroy_workqueue(dbs_wq);
}

#ifdef CONFIG_CPU_FREQ_DEFAULT_GOV_CONSERVATIVE
fs_initcall(cpufreq_gov_dbs_init);
#else
module_init(cpufreq_gov_dbs_init);
#endif
module_exit(cpufreq_gov_dbs_exit);

MODULE_AUTHOR("Alexander Clouter <alex@digriz.org.uk>");
MODULE_DESCRIPTION("'cpufreq_conservative' - A dynamic cpufreq governor for "
		"Low Latency Frequency Transition capable processors "
		"optimised for use in a battery environment");
MODULE_LICENSE("GPLv2");
