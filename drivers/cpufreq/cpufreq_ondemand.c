/*
 * Copyright (C) 2001, Russell King.
 * Copyright (C) 2003, Venkatesh Pallipadi <venkatesh.pallipadi@intel.com>
 * Copyright (C) 2003, Jun Nakajima <jun.nakajima@intel.com>
 * Copyright (C) 2013, The Linux Foundation. All rights reserved.
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
#define DEF_FREQUENCY_DOWN_DIFFERENTIAL		(10)
#define DEF_FREQUENCY_SYNCHRONIZATION		(1)
#define DEF_SAMPLING_DOWN_FACTOR		(1)
#define MICRO_FREQUENCY_UP_THRESHOLD		(95)
#define MICRO_FREQUENCY_DOWN_DIFFERENTIAL	(3)

struct dbs_work_struct {
	struct work_struct work;
	u32 cpu;
};

static DEFINE_PER_CPU(struct dbs_work_struct, dbs_refresh_work);
static DEFINE_PER_CPU(struct od_cpu_dbs_info_s, od_cpu_dbs_info);
define_get_cpu_dbs_routines(od_cpu_dbs_info);

static struct od_dbs_tuners od_tuners = {
	.sampling_down_factor = DEF_SAMPLING_DOWN_FACTOR,
	.up_threshold = DEF_FREQUENCY_UP_THRESHOLD,
	.up_threshold_multi_core = DEF_FREQUENCY_UP_THRESHOLD,
	.up_threshold_any_cpu_load = DEF_FREQUENCY_UP_THRESHOLD,
	.down_differential = DEF_FREQUENCY_DOWN_DIFFERENTIAL,
	.down_differential_multi_core = MICRO_FREQUENCY_DOWN_DIFFERENTIAL,
	.sync_on_migrate = DEF_FREQUENCY_SYNCHRONIZATION,
	.sync_freq = 0,
	.optimal_freq = 0,
	.input_boost_freq = 0,
};

static inline u32 get_policy_max_load_freq(struct cpufreq_policy *policy)
{
	u32 max_load_freq = 0, cur_load, freq_avg, load_freq, j;
	u32 load_at_max_freq;

	for_each_cpu(j, policy->cpus) {
		struct cpu_dbs_common_info *j_cdbs = get_cpu_cdbs(j);
		struct od_cpu_dbs_info_s *j_dbs_info = get_cpu_dbs_info_s(j);
		u64 cur_idle_time, cur_wall_time;
		u32 idle_time, wall_time;

		cur_idle_time = get_cpu_idle_time(j, &cur_wall_time,
						  od_tuners.io_is_busy);

		idle_time = (u32)(cur_idle_time - j_cdbs->prev_cpu_idle);
		j_cdbs->prev_cpu_idle = cur_idle_time;

		wall_time = (u32)(cur_wall_time - j_cdbs->prev_cpu_wall);
		j_cdbs->prev_cpu_wall = cur_wall_time;

		if (unlikely(!wall_time || wall_time < idle_time))
			continue;

		cur_load = 100 * (wall_time - idle_time) / wall_time;

		freq_avg = __cpufreq_driver_getavg(policy, j);
		if (freq_avg <= 0)
			freq_avg = policy->cur;

		load_freq = cur_load * freq_avg;
		max_load_freq = max(max_load_freq, load_freq);

		j_dbs_info->max_load  = max(cur_load, j_dbs_info->prev_load);
		j_dbs_info->prev_load = cur_load;
	}

	load_at_max_freq = max_load_freq / policy->max;
	cpufreq_notify_utilization(policy, load_at_max_freq);

	return max_load_freq;
}

static inline u32 get_other_cpu_max_load(struct cpufreq_policy *policy)
{
	u32 max_load_other_cpu = 0, j;

	for_each_online_cpu(j) {
		struct cpu_dbs_common_info *j_cdbs = get_cpu_cdbs(j);
		struct od_cpu_dbs_info_s *j_dbs_info = get_cpu_dbs_info_s(j);

		if (j == policy->cpu)
			continue;

		max_load_other_cpu =
			max(max_load_other_cpu, j_dbs_info->max_load);

		/*
		 * The other cpu could be running at higher frequency but may
		 * not have completed it's sampling_down_factor. For that case
		 * consider other cpu is loaded so that frequency imbalance
		 * does not occur.
		 */
		if (j_cdbs->cur_policy &&
		    j_cdbs->cur_policy->cur == j_cdbs->cur_policy->max &&
		    policy->cur >= od_tuners.optimal_freq)
			max_load_other_cpu =
				od_tuners.up_threshold_any_cpu_load;
	}

	return max_load_other_cpu;
}

static inline void dbs_freq_increase(struct cpufreq_policy *policy, u32 freq)
{
	if (policy->cur == policy->max)
		return;

	__cpufreq_driver_target(policy, freq, CPUFREQ_RELATION_H);
}

static void od_check_cpu(struct od_cpu_dbs_info_s *dbs_info)
{
	struct cpufreq_policy *policy = dbs_info->cdbs.cur_policy;
	u32 max_load_freq = get_policy_max_load_freq(policy);
	u32 max_load_other_cpu = get_other_cpu_max_load(policy);

	if (max_load_freq > od_tuners.up_threshold * policy->cur) {
		if (policy->cur < policy->max)
			dbs_info->rate_mult = od_tuners.sampling_down_factor;

		dbs_freq_increase(policy, policy->max);
		return;
	}

	if (num_online_cpus() > 1) {
		if (max_load_other_cpu > od_tuners.up_threshold_any_cpu_load) {
			if (policy->cur < od_tuners.sync_freq)
				dbs_freq_increase(policy, od_tuners.sync_freq);
			return;
		}

		if (max_load_freq > od_tuners.up_threshold_multi_core *
		    policy->cur) {
			if (policy->cur < od_tuners.optimal_freq)
				dbs_freq_increase(policy,
					od_tuners.optimal_freq);
			return;
		}
	}

	/*
	 * Check for frequency decrease.
	 * If we cannot reduce the frequency anymore, break out early.
	 */
	if (policy->cur == policy->min)
		return;

	/*
	 * The optimal frequency is the frequency that is the lowest that
	 * can support the current CPU usage without triggering the up
	 * policy.
	 */
	if (max_load_freq <
	   (od_tuners.up_threshold - od_tuners.down_differential) *
	    policy->cur) {
		u32 freq_next;

		freq_next = max_load_freq / (od_tuners.up_threshold -
					     od_tuners.down_differential);
		freq_next = max(freq_next, policy->min);

		/* No longer fully busy, reset rate_mult */
		dbs_info->rate_mult = 1;

		if (num_online_cpus() > 1) {
			if (max_load_other_cpu >
			    od_tuners.up_threshold_multi_core -
			    od_tuners.down_differential &&
			    freq_next < od_tuners.sync_freq)
				freq_next = od_tuners.sync_freq;

			if (max_load_freq >
			   (od_tuners.up_threshold_multi_core -
			    od_tuners.down_differential_multi_core) *
			    policy->cur && freq_next < od_tuners.optimal_freq)
				freq_next = od_tuners.optimal_freq;
		}

		__cpufreq_driver_target(policy, freq_next, CPUFREQ_RELATION_C);
	}
}

static int dbs_migration_notify(struct notifier_block *nb,
				unsigned long target_cpu,
				void *arg)
{
	struct od_cpu_dbs_info_s *dbs_info = get_cpu_dbs_info_s(target_cpu);

	if (!od_tuners.sync_on_migrate)
		return NOTIFY_OK;

	atomic_set(&dbs_info->src_sync_cpu, (int)arg);
	wake_up(&dbs_info->sync_wq);

	return NOTIFY_DONE;
}

static struct notifier_block dbs_migration_nb = {
	.notifier_call = dbs_migration_notify,
};

static int sync_pending(struct od_cpu_dbs_info_s *dbs_info)
{
	return atomic_read(&dbs_info->src_sync_cpu) >= 0;
}

static int dbs_sync_thread(void *data)
{
	int src_cpu, cpu = (int)data, delay;
	u32 src_freq, src_max_load;
	struct od_cpu_dbs_info_s *cur_dbs_info = get_cpu_dbs_info_s(cpu);
	struct od_cpu_dbs_info_s *src_dbs_info;
	struct cpufreq_policy *policy;

	while (1) {
		wait_event_interruptible(cur_dbs_info->sync_wq,
			sync_pending(cur_dbs_info) || kthread_should_stop());

		if (kthread_should_stop())
			break;

		get_online_cpus();
		if (!atomic_read(&cur_dbs_info->sync_enabled)) {
			atomic_set(&cur_dbs_info->src_sync_cpu, -1);
			put_online_cpus();
			continue;
		}

		src_cpu = atomic_read(&cur_dbs_info->src_sync_cpu);
		src_dbs_info = get_cpu_dbs_info_s(src_cpu);

		if (src_dbs_info && src_dbs_info->cdbs.cur_policy) {
			src_freq = src_dbs_info->cdbs.cur_policy->cur;
			src_max_load = src_dbs_info->max_load;
		} else {
			src_freq = od_tuners.sync_freq;
			src_max_load = 0;
		}

		if (IS_ERR_VALUE(lock_policy_rwsem_write(cpu)))
			goto bail_acq_sema_failed;

		/* Check if cpu really uses ondemand governor */
		policy = cur_dbs_info->cdbs.cur_policy;
		if (IS_ERR_OR_NULL(policy))
			goto bail_incorrect_governor;

		delay = usecs_to_jiffies(od_tuners.sampling_rate);

		if (policy->cur < src_freq) {
			cancel_delayed_work_sync(&cur_dbs_info->cdbs.work);

			/*
			 * Arch specific cpufreq driver may fail.
			 * Don't update governor frequency upon failure.
			 */
			if (__cpufreq_driver_target(policy,
			    src_freq, CPUFREQ_RELATION_L) >= 0) {
				policy->cur = src_freq;

				if (src_max_load > cur_dbs_info->max_load) {
					cur_dbs_info->max_load = src_max_load;
					cur_dbs_info->prev_load = src_max_load;
				}
			}

			mutex_lock(&cur_dbs_info->cdbs.timer_mutex);
			schedule_delayed_work_on(cpu,
				&cur_dbs_info->cdbs.work, delay);
			mutex_unlock(&cur_dbs_info->cdbs.timer_mutex);
		}

bail_incorrect_governor:
		unlock_policy_rwsem_write(cpu);
bail_acq_sema_failed:
		put_online_cpus();
		atomic_set(&cur_dbs_info->src_sync_cpu, -1);
	}

	return 0;
}

static void dbs_refresh_callback(struct work_struct *work)
{
	struct cpufreq_policy *policy;
	struct cpu_dbs_common_info *cdbs;
	struct dbs_work_struct *dbs_work =
		container_of(work, struct dbs_work_struct, work);
	u32 cpu = dbs_work->cpu, target_freq;

	get_online_cpus();
	if (IS_ERR_VALUE(lock_policy_rwsem_write(cpu)))
		goto bail_acq_sema_failed;

	cdbs = get_cpu_cdbs(cpu);

	/* Check if cpu really uses ondemand governor */
	policy = cdbs->cur_policy;
	if (IS_ERR_OR_NULL(policy))
		goto bail_incorrect_governor;

	if (od_tuners.input_boost_freq)
		target_freq = od_tuners.input_boost_freq;
	else
		target_freq = policy->max;

	if (policy->cur < target_freq) {
		/*
		 * Arch specific cpufreq driver may fail.
		 * Don't update governor frequency upon failure.
		 */
		if (__cpufreq_driver_target(policy,
		    target_freq, CPUFREQ_RELATION_L) >= 0)
			policy->cur = target_freq;

		cdbs->prev_cpu_idle = get_cpu_idle_time(cpu,
				&cdbs->prev_cpu_wall, od_tuners.io_is_busy);
	}

bail_incorrect_governor:
	unlock_policy_rwsem_write(cpu);
bail_acq_sema_failed:
	put_online_cpus();

	return;
}

static void dbs_input_event(struct input_handle *handle,
			    unsigned int type, unsigned int code,
			    int value)
{
	int cpu;

	for_each_online_cpu(cpu)
		queue_work_on(cpu, dbs_wq,
			&per_cpu(dbs_refresh_work, cpu).work);
}

static int dbs_input_connect(struct input_handler *handler,
			     struct input_dev *dev,
			     const struct input_device_id *id)
{
	struct input_handle *handle;
	int error;

	handle = kzalloc(sizeof(*handle), GFP_KERNEL);
	if (IS_ERR_OR_NULL(handle))
		return -ENOMEM;

	handle->dev = dev;
	handle->handler = handler;
	handle->name = handler->name;

	error = input_register_handle(handle);
	if (IS_ERR_VALUE(error))
		goto err2;

	error = input_open_device(handle);
	if (IS_ERR_VALUE(error))
		goto err1;

	return 0;

err1:
	input_unregister_handle(handle);
err2:
	kfree(handle);

	return error;
}

static void dbs_input_disconnect(struct input_handle *handle)
{
	input_close_device(handle);
	input_unregister_handle(handle);
	kfree(handle);
}

static const struct input_device_id dbs_ids[] = {
	{ .driver_info = 1 },
	{ },
};

static struct input_handler dbs_input_handler = {
	.event		= dbs_input_event,
	.connect	= dbs_input_connect,
	.disconnect	= dbs_input_disconnect,
	.name		= "cpufreq_od",
	.id_table	= dbs_ids,
};

define_sampling_rate_min_node(od);
define_one_sdf_node(od, rate_mult);
define_one_dbs_node(od, sampling_rate, min_sampling_rate, UINT_MAX);
define_one_dbs_node(od, up_threshold, 1, 100);
define_one_dbs_node(od, up_threshold_multi_core, 1, 100);
define_one_dbs_node(od, up_threshold_any_cpu_load, 1, 100);
define_one_dbs_node(od, down_differential, 1, 100);
define_one_dbs_node(od, down_differential_multi_core, 1, 100);
define_one_dbs_node(od, io_is_busy, 0, 1);
define_one_dbs_node(od, sync_on_migrate, 0, 1);
define_one_dbs_node(od, sync_freq, 0, UINT_MAX);
define_one_dbs_node(od, optimal_freq, 0, UINT_MAX);
define_one_dbs_node(od, input_boost_freq, 0, UINT_MAX);

static struct attribute *dbs_attributes[] = {
	&sampling_rate_min.attr,
	&sampling_rate.attr,
	&sampling_down_factor.attr,
	&up_threshold.attr,
	&up_threshold_multi_core.attr,
	&up_threshold_any_cpu_load.attr,
	&down_differential.attr,
	&down_differential_multi_core.attr,
	&io_is_busy.attr,
	&sync_on_migrate.attr,
	&sync_freq.attr,
	&optimal_freq.attr,
	&input_boost_freq.attr,
	NULL,
};

static struct attribute_group dbs_attr_group = {
	.name = "ondemand",
	.attrs = dbs_attributes,
};

/* Ondemand dbs_timer with sampling_down_factor support is initialized here */
define_gov_dbs_timer(od, dbs_info->rate_mult);

static inline void dbs_timer_init(struct od_cpu_dbs_info_s *dbs_info)
{
	int delay = delay_for_sampling_rate(od_tuners.sampling_rate, 1);

	INIT_DELAYED_WORK_DEFERRABLE(&dbs_info->cdbs.work, od_dbs_timer);
	queue_delayed_work_on(dbs_info->cdbs.cpu, dbs_wq,
			     &dbs_info->cdbs.work, delay);
}

static inline void dbs_timer_exit(struct od_cpu_dbs_info_s *dbs_info)
{
	cancel_delayed_work_sync(&dbs_info->cdbs.work);
}

static int cpufreq_governor_dbs(struct cpufreq_policy *policy,
				unsigned int event)
{
	int ret;
	u32 cpu = policy->cpu, latency, j;
	struct od_cpu_dbs_info_s *dbs_info = get_cpu_dbs_info_s(cpu);
	struct cpu_dbs_common_info *cdbs = get_cpu_cdbs(cpu);

	switch (event) {
	case CPUFREQ_GOV_START:
		if (!cpu_online(cpu) || !policy->cur)
			return -EINVAL;

		mutex_lock(&dbs_mutex);
		for_each_cpu(j, policy->cpus) {
			struct cpu_dbs_common_info *j_cdbs = get_cpu_cdbs(j);
			struct od_cpu_dbs_info_s *j_dbs_info =
					get_cpu_dbs_info_s(j);

			j_cdbs->cur_policy = policy;
			j_cdbs->prev_cpu_idle = get_cpu_idle_time(j,
				&j_cdbs->prev_cpu_wall, should_io_be_busy());

			set_cpus_allowed(j_dbs_info->sync_thread,
					 *cpumask_of(j));
			atomic_set(&j_dbs_info->sync_enabled, 1);
		}

		cdbs->cpu = cpu;
		dbs_info->rate_mult = 1;

		/*
		 * Start the timerschedule work, when this governor
		 * is used for the first time.
		 */
		if (++gov_enable_cnt != 1)
			goto skip_init;

		/* Policy latency is in nS. Convert it to uS first. */
		latency = policy->cpuinfo.transition_latency / 1000;
		if (!latency)
			latency = 1;

		/* Bring kernel and HW constraints together */
		min_sampling_rate = max(min_sampling_rate,
				latency * MIN_LATENCY_MULTIPLIER);
		od_tuners.sampling_rate = max(min_sampling_rate,
				latency * LATENCY_MULTIPLIER);

		if (likely(!od_tuners.io_is_busy))
			od_tuners.io_is_busy = should_io_be_busy();
		if (likely(!od_tuners.optimal_freq))
			od_tuners.optimal_freq = policy->min;
		if (likely(!od_tuners.sync_freq))
			od_tuners.sync_freq = policy->min;

		ret = atomic_notifier_chain_register(&migration_notifier_head,
						     &dbs_migration_nb);
		if (IS_ERR_VALUE(ret)) {
			pr_err("%s: unable to register atomic notifier\n",
						__func__);
			goto fail_notifier;
		}

		ret = input_register_handler(&dbs_input_handler);
		if (IS_ERR_VALUE(ret)) {
			pr_err("%s: unable to register input handler\n",
						__func__);
			goto fail_input;
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
		input_unregister_handler(&dbs_input_handler);
fail_input:
		atomic_notifier_chain_unregister(&migration_notifier_head,
						 &dbs_migration_nb);
fail_notifier:
		for_each_cpu(j, policy->cpus) {
			struct od_cpu_dbs_info_s *j_dbs_info =
					get_cpu_dbs_info_s(j);
			atomic_set(&j_dbs_info->sync_enabled, 0);
		}
		mutex_unlock(&dbs_mutex);

		return ret;
	case CPUFREQ_GOV_STOP:
		dbs_timer_exit(dbs_info);

		mutex_lock(&dbs_mutex);
		for_each_cpu(j, policy->cpus) {
			struct od_cpu_dbs_info_s *j_dbs_info =
					get_cpu_dbs_info_s(j);
			atomic_set(&j_dbs_info->sync_enabled, 0);
		}

		/* If device is being removed, policy is no longer valid */
		cdbs->cur_policy = NULL;

		if (--gov_enable_cnt != 0)
			goto skip_exit;

		sysfs_remove_group(cpufreq_global_kobject, &dbs_attr_group);
		input_unregister_handler(&dbs_input_handler);
		atomic_notifier_chain_unregister(&migration_notifier_head,
						 &dbs_migration_nb);

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
		od_check_cpu(dbs_info);
		mutex_unlock(&cdbs->timer_mutex);
		break;
	}

	return 0;
}

#ifndef CONFIG_CPU_FREQ_DEFAULT_GOV_ONDEMAND
static
#endif
struct cpufreq_governor cpufreq_gov_ondemand = {
	.name			= "ondemand",
	.governor		= cpufreq_governor_dbs,
	.max_transition_latency	= TRANSITION_LATENCY_LIMIT,
	.owner			= THIS_MODULE,
};

static int __init cpufreq_gov_dbs_init(void)
{
	int cpu;

	dbs_wq = alloc_workqueue("ondemand_dbs_wq", WQ_HIGHPRI, 0);
	if (IS_ERR_OR_NULL(dbs_wq)) {
		pr_err("%s: unable to allocate workqueue\n", __func__);
		return -EFAULT;
	}

	for_each_possible_cpu(cpu) {
		struct cpu_dbs_common_info *cdbs = get_cpu_cdbs(cpu);
		struct od_cpu_dbs_info_s *dbs_info = get_cpu_dbs_info_s(cpu);
		struct dbs_work_struct *dbs_work =
				&per_cpu(dbs_refresh_work, cpu);

		mutex_init(&cdbs->timer_mutex);

		dbs_work->cpu = cpu;
		INIT_WORK(&dbs_work->work, dbs_refresh_callback);

		init_waitqueue_head(&dbs_info->sync_wq);
		atomic_set(&dbs_info->src_sync_cpu, -1);

		dbs_info->sync_thread = kthread_run(dbs_sync_thread,
					(void *)cpu, "dbs_sync/%d", cpu);
	}

	/*
	 * In NOHZ/micro accounting case we set the minimum frequency
	 * not depending on HZ, but fixed (very low).  The deferred
	 * timer might skip some samples if idle/sleeping as needed.
	 */
	if (nohz_sampling_rate_is_used()) {
		od_tuners.up_threshold = MICRO_FREQUENCY_UP_THRESHOLD;
		od_tuners.down_differential = MICRO_FREQUENCY_DOWN_DIFFERENTIAL;
		min_sampling_rate = MICRO_FREQUENCY_MIN_SAMPLE_RATE;
	} else {
		min_sampling_rate = sampling_rate_by_jiffies();
	}

	return cpufreq_register_governor(&cpufreq_gov_ondemand);
}

static void __exit cpufreq_gov_dbs_exit(void)
{
	int cpu;

	cpufreq_unregister_governor(&cpufreq_gov_ondemand);

	for_each_possible_cpu(cpu) {
		struct cpu_dbs_common_info *cdbs = get_cpu_cdbs(cpu);
		struct od_cpu_dbs_info_s *dbs_info = get_cpu_dbs_info_s(cpu);

		kthread_stop(dbs_info->sync_thread);
		atomic_set(&dbs_info->src_sync_cpu, -1);

		mutex_destroy(&cdbs->timer_mutex);
	}

	destroy_workqueue(dbs_wq);
}

#ifdef CONFIG_CPU_FREQ_DEFAULT_GOV_ONDEMAND
fs_initcall(cpufreq_gov_dbs_init);
#else
module_init(cpufreq_gov_dbs_init);
#endif
module_exit(cpufreq_gov_dbs_exit);

MODULE_AUTHOR("Venkatesh Pallipadi <venkatesh.pallipadi@intel.com>");
MODULE_AUTHOR("Alexey Starikovskiy <alexey.y.starikovskiy@intel.com>");
MODULE_DESCRIPTION("'cpufreq_ondemand' - A dynamic cpufreq governor for "
		"Low Latency Frequency Transition capable processors");
MODULE_LICENSE("GPLv2");
