/*
 * Header file for CPUFreq governors common code.
 *
 * Copyright (C) 2001, Russell King.
 * Copyright (C) 2003, Venkatesh Pallipadi <venkatesh.pallipadi@intel.com>
 * Copyright (C) 2003, Jun Nakajima <jun.nakajima@intel.com>
 * Copyright (C) 2009, Alexander Clouter <alex@digriz.org.uk>
 * Copyright (c) 2012, Viresh Kumar <viresh.kumar@linaro.org>
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

#ifndef __CPUFREQ_GOVERNOR_COMMON__
#define __CPUFREQ_GOVERNOR_COMMON__

#include <linux/cpu.h>
#include <linux/cpufreq.h>
#include <linux/kernel_stat.h>
#include <linux/module.h>
#include <linux/tick.h>
#include <linux/kthread.h>
#include <linux/input.h>
#include <linux/slab.h>

/* Macroses to implement governor sysfs nodes */
#define show_one_dbs(_gov, _name)					 \
static ssize_t show_##_name(struct kobject *kobj,			 \
			    struct attribute *attr,			 \
			    char *buf)					 \
{									 \
	return scnprintf(buf, sizeof(u32) * 8 + 1, "%u\n",		 \
			 _gov##_tuners._name);				 \
}

#define store_one_dbs(_gov, _name, down_limit, up_limit)		 \
static ssize_t store_##_name(struct kobject *kobj,			 \
			     struct attribute *attr,			 \
			     const char *buf, size_t count)		 \
{									 \
	int ret;							 \
	u32 val;							 \
									 \
	ret = kstrtouint(buf, 10, &val);				 \
	if (ret || (s32)val < down_limit || val > up_limit)		 \
		return -EINVAL;						 \
									 \
	_gov##_tuners._name = val;					 \
									 \
	return count;							 \
}

#define define_one_dbs_node(_gov, _name, down_limit, up_limit)		 \
show_one_dbs(_gov, _name);						 \
store_one_dbs(_gov, _name, down_limit, up_limit);			 \
define_one_global_rw(_name);

#define define_sampling_rate_min_node(_gov)				 \
static ssize_t show_sampling_rate_min(struct kobject *kobj,		 \
				      struct attribute *attr,		 \
				      char *buf)			 \
{									 \
	return scnprintf(buf, sizeof(u32) * 8 + 1, "%u\n",		 \
			 min_sampling_rate);				 \
}									 \
define_one_global_ro(sampling_rate_min);

#define define_one_sdf_node(_gov, mult_factor)				 \
static ssize_t store_sampling_down_factor(struct kobject *kobj,		 \
					  struct attribute *attr,	 \
					  const char *buf, size_t count) \
{									 \
	struct _gov##_cpu_dbs_info_s *dbs_info;				 \
	u32 val, cpu;							 \
	int ret;							 \
									 \
	ret = kstrtouint(buf, 10, &val);				 \
	if (ret || (s32)val < 1 ||					 \
	    val == _gov##_tuners.sampling_down_factor)			 \
		return -EINVAL;						 \
									 \
	_gov##_tuners.sampling_down_factor = val;			 \
									 \
	get_online_cpus();						 \
	for_each_online_cpu(cpu) {					 \
		dbs_info = get_cpu_dbs_info_s(cpu);			 \
		dbs_info->mult_factor = 1;				 \
	}								 \
	put_online_cpus();						 \
									 \
	return count;							 \
}									 \
show_one_dbs(_gov, sampling_down_factor);				 \
define_one_global_rw(sampling_down_factor);

#define define_gov_dbs_timer(_gov, mult_factor)				 \
static void _gov##_dbs_timer(struct work_struct *work)			 \
{									 \
	struct _gov##_cpu_dbs_info_s *dbs_info = container_of(work,	 \
			struct _gov##_cpu_dbs_info_s, cdbs.work.work);	 \
	int delay = delay_for_sampling_rate(				 \
			_gov##_tuners.sampling_rate, mult_factor);	 \
	u32 cpu = dbs_info->cdbs.cpu;					 \
									 \
	mutex_lock(&dbs_info->cdbs.timer_mutex);			 \
	_gov##_check_cpu(dbs_info);					 \
									 \
	queue_delayed_work_on(cpu, dbs_wq, &dbs_info->cdbs.work, delay); \
	mutex_unlock(&dbs_info->cdbs.timer_mutex);			 \
}

#define define_get_cpu_dbs_routines(dbs_info)				 \
static struct cpu_dbs_common_info *get_cpu_cdbs(u32 cpu)		 \
{									 \
	return &per_cpu(dbs_info, cpu).cdbs;				 \
}									 \
									 \
static void *get_cpu_dbs_info_s(u32 cpu)				 \
{									 \
	return &per_cpu(dbs_info, cpu);					 \
}

/*
 * The polling frequency of this governor depends on the capability of the
 * processor. Default polling frequency is 1000 times the transition latency
 * of the processor.
 *
 * This governor will work on any processor with transition latency <= 10mS,
 * using appropriate sampling rate. For CPUs with transition latency > 10mS
 * (mostly drivers with CPUFREQ_ETERNAL) this governor will not work.
 *
 * All times here are in uS.
 */
#define MICRO_FREQUENCY_MIN_SAMPLE_RATE		(10000)

static u32 min_sampling_rate;

#define SAMPLING_RATE_MIN_RATIO			(2)
#define MIN_LATENCY_MULTIPLIER			(100)
#define LATENCY_MULTIPLIER			(1000)
#define TRANSITION_LATENCY_LIMIT		(10 * 1000 * 1000)

/* Number of CPUs that currently use this governor */
static u32 gov_enable_cnt;

/* dbs_mutex protects gov_enable_cnt and dbs_info during start/stop */
static DEFINE_MUTEX(dbs_mutex);

/* Workqueue used to run governor on */
static struct workqueue_struct *dbs_wq;

/*
 * Not all CPUs want IO time to be accounted as busy; this dependson how
 * efficient idling at a higher frequency/voltage is. Pavel Machek says this
 * is not so for various generations of AMD and old Intel systems. Mike Chan
 * (androidlcom) calis this is also not true for ARM.
 *
 * Because of this, whitelist specific known (series) of CPUs by default, and
 * leave all others up to the user.
 */
static inline u32 should_io_be_busy(void)
{
#ifdef CONFIG_X86
	/* For Intel, Core 2 (model 15) andl later have an efficient idle */
	if (boot_cpu_data.x86_vendor == X86_VENDOR_INTEL &&
	    boot_cpu_data.x86 == 6 &&
	    boot_cpu_data.x86_model >= 15)
		return 1;
#elif CONFIG_ARM
	/* All ARM targets want iowait time to be subtracted from idle time */
	return 1;
#endif

	return 0;
}

/**
 * delay_for_sampling_rate() - align sampling_rate with online cpus and
 *			       sampling_down_factor multiplicator.
 * @sampling_rate: delay to be aligned.
 * @rate_mult: multiplicator used on sampling_rate.
 *
 * Returns aligned delay in jiffies.
 */
static inline int delay_for_sampling_rate(u32 sampling_rate, u32 rate_mult)
{
	int delay;

	/* We want all CPUs to do sampling nearly on same jiffy */
	delay = usecs_to_jiffies(sampling_rate * rate_mult);
	if (num_online_cpus() > 1)
		delay -= jiffies % delay;

	return delay;
}

/**
 * sampling_rate_by_jiffies() - get a sampling_rate multiplied by jiffies.
 */
static inline u32 sampling_rate_by_jiffies(void)
{
	return (u32)(jiffies_to_usecs(10) * SAMPLING_RATE_MIN_RATIO);
}

/**
 * nohz_sampling_rate_is_used() - callback to ensure that Micro/NO_HZ idle
 *				  accounting should be used.
 */
static inline bool nohz_sampling_rate_is_used(void)
{
	int cpu;
	u64 idle_time;

	cpu = get_cpu();
	idle_time = get_cpu_idle_time_us(cpu, NULL);
	put_cpu();

	if (idle_time != -1ULL)
		return true;

	return false;
}

struct cpu_dbs_common_info {
	struct cpufreq_policy *cur_policy;
	struct delayed_work work;
	u64 prev_cpu_idle;
	u64 prev_cpu_wall;
	u32 cpu;
	/*
	 * percpu mutex that serializes governor limit change with dbs_timer
	 * invocation. We do not want dbs_timer to run when user is changing
	 * the governor or limits.
	 */
	struct mutex timer_mutex;
};

struct od_cpu_dbs_info_s {
	struct cpu_dbs_common_info cdbs;
	u32 rate_mult;
	u32 prev_load;
	u32 max_load;

	struct task_struct *sync_thread;
	wait_queue_head_t sync_wq;
	atomic_t src_sync_cpu;
	atomic_t sync_enabled;
};

struct od_dbs_tuners {
	u32 sampling_rate;
	u32 sampling_down_factor;
	u32 up_threshold;
	u32 up_threshold_multi_core;
	u32 up_threshold_any_cpu_load;
	u32 down_differential;
	u32 down_differential_multi_core;
	u32 io_is_busy:1;
	u32 sync_on_migrate:1;
	u32 sync_freq;
	u32 optimal_freq;
	u32 input_boost_freq;
};

struct cs_cpu_dbs_info_s {
	struct cpu_dbs_common_info cdbs;
	u32 requested_freq;
	bool enabled;
};

struct cs_dbs_tuners {
	u32 sampling_rate;
	u32 up_threshold;
	u32 down_threshold;
	u32 freq_up_step;
	u32 freq_down_step;
	u32 io_is_busy:1;
};

#endif /* __CPUFREQ_GOVERNOR_COMMON__ */
