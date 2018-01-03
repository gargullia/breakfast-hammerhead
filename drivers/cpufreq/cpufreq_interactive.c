/*
 * Copyright (C) 2010, Mike Chan <mike@android.com>
 * Copyright (C) 2010, Google Inc. All rights reserved.
 * Copyright (C) 2017-2018, Alex Saiko <solcmdr@gmail.com>
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

#define pr_fmt(fmt)	KBUILD_MODNAME ": %s: " fmt, __func__

#include <linux/cpufreq.h>
#include <linux/module.h>
#include <linux/sched.h>
#include <linux/kthread.h>
#include <linux/slab.h>

#define CREATE_TRACE_POINTS
#include <trace/events/cpufreq_interactive.h>

static int gov_enable_cnt;

struct cpufreq_interactive_cpuinfo {
	struct timer_list cpu_timer;
	struct timer_list cpu_slack_timer;
	struct cpufreq_policy *policy;
	struct cpufreq_frequency_table *freq_table;
	struct rw_semaphore enable_sem;
	spinlock_t load_lock; /* Protects the next 4 fields */
	u64 time_in_idle;
	u64 time_in_idle_timestamp;
	u64 cputime_speedadj;
	u64 cputime_speedadj_timestamp;
	u64 last_evaluated_jiffy;
	spinlock_t target_freq_lock; /* Protects target freq */
	u32 target_freq;
	u32 floor_freq;
	u32 min_freq;
	u64 floor_validate_time;
	u64 hispeed_validate_time; /* Cluster hispeed_validate_time */
	u64 local_hvtime; /* Per-cpu hispeed_validate_time */
	u64 max_freq_hyst_start_time;
	int governor_enabled:1;
};

static DEFINE_PER_CPU(struct cpufreq_interactive_cpuinfo, cpuinfo);

#define TRANSITION_LATENCY_LIMIT	(10 * 1000 * 1000)

/* Real-time thread handles frequency scaling */
static struct task_struct *speedchange_task;
static cpumask_t speedchange_cpumask;
static spinlock_t speedchange_cpumask_lock;
static struct mutex gov_lock;

/* Hi speed to bump to from lo speed when load burst (default max) */
static unsigned int hispeed_freq;

/* Go to hi speed when CPU load at or above this value */
static unsigned int go_hispeed_load = 99;

/*
 * Frequency calculation threshold. Use aggressive frequency scaling below
 * this frequency and allow for dynamic changes above (default min_freq).
 */
static unsigned int freq_calc_thresh;

/* Target load. Lower values result in higher CPU speeds. */
static unsigned int __read_mostly default_target_loads[] = { 90 };
static spinlock_t target_loads_lock;
static unsigned int *target_loads = default_target_loads;
static int __read_mostly ntarget_loads = ARRAY_SIZE(default_target_loads);

/* The minimum amount of time to spend at a frequency before we can ramp down */
#define DEFAULT_MIN_SAMPLE_TIME (80 * USEC_PER_MSEC)
static unsigned long min_sample_time = DEFAULT_MIN_SAMPLE_TIME;

/* The sample rate of the timer used to increase frequency */
#define DEFAULT_TIMER_RATE (20 * USEC_PER_MSEC)
static unsigned long timer_rate = DEFAULT_TIMER_RATE;

/*
 * Wait this long before raising speed above hispeed, by default a single
 * timer interval.
 */
static unsigned int __read_mostly default_above_hispeed_delay[] = {
	DEFAULT_TIMER_RATE,
};
static spinlock_t above_hispeed_delay_lock;
static unsigned int *above_hispeed_delay = default_above_hispeed_delay;
static int __read_mostly nabove_hispeed_delay =
		ARRAY_SIZE(default_above_hispeed_delay);

/* Non-zero means indefinite speed boost active */
static int boost_val;
/* Duration of a boost pulse in usecs */
static unsigned long boostpulse_duration_val = DEFAULT_MIN_SAMPLE_TIME;
/* End time of boost pulse in ktime converted to usecs */
static u64 boostpulse_endtime;

/*
 * Max additional time to wait in idle, beyond timer_rate, at speeds above
 * minimum before wakeup to reduce speed, or -1 if unnecessary.
 */
#define DEFAULT_TIMER_SLACK (4 * DEFAULT_TIMER_RATE)
static int timer_slack_val = DEFAULT_TIMER_SLACK;

/* Whether to align timer windows across all CPUs */
static unsigned int __read_mostly align_windows = 1;

/*
 * Stay at max freq for at least max_freq_hysteresis milliseconds before
 * dropping frequency.
 */
static unsigned int max_freq_hysteresis;

/* Subtract iowait time from idle one during load computations */
static unsigned int __read_mostly io_is_busy = 1;

/* Round to starting jiffy of next evaluation window */
static inline u64 round_to_nw_start(u64 jif)
{
	unsigned long step = usecs_to_jiffies(timer_rate);

	if (likely(align_windows)) {
		do_div(jif, step);
		return (u64)(++jif * step);
	}

	return (u64)(jiffies + step);
}

static inline void
cpufreq_interactive_timer_resched(unsigned long cpu, bool slack_only)
{
	struct cpufreq_interactive_cpuinfo *pcpu = &per_cpu(cpuinfo, cpu);
	u64 now = ktime_to_us(ktime_get()), expires;
	unsigned long flags;

	spin_lock_irqsave(&pcpu->load_lock, flags);
	/* Count next sample time */
	expires = round_to_nw_start(pcpu->last_evaluated_jiffy);

	if (likely(!slack_only)) {
		/* Reset time slices first */
		pcpu->time_in_idle = get_cpu_idle_time(smp_processor_id(),
				&pcpu->time_in_idle_timestamp, io_is_busy);
		pcpu->cputime_speedadj_timestamp = pcpu->time_in_idle_timestamp;
		pcpu->cputime_speedadj = 0;

		/* Restart interactive timer with a new sample time */
		del_timer(&pcpu->cpu_timer);
		pcpu->cpu_timer.expires = expires;
		add_timer_on(&pcpu->cpu_timer, cpu);
	}

	/* Rearm slack timer only if device is not in idle state */
	if (timer_slack_val >= 0 &&
	   (pcpu->target_freq > pcpu->policy->min ||
	   (pcpu->target_freq == pcpu->policy->min &&
	    now < boostpulse_endtime))) {
		/* Slack sample is an additional time to delay samples */
		expires += usecs_to_jiffies(timer_slack_val);

		/* Restart slack timer with a new sample time */
		del_timer(&pcpu->cpu_slack_timer);
		pcpu->cpu_slack_timer.expires = expires;
		add_timer_on(&pcpu->cpu_slack_timer, cpu);
	}
	spin_unlock_irqrestore(&pcpu->load_lock, flags);
}

/*
 * The caller shall take enable_sem write semaphore to avoid any timer race.
 * The cpu_timer and cpu_slack_timer must be deactivated when calling this
 * function.
 */
static inline void cpufreq_interactive_timer_start(int cpu)
{
	struct cpufreq_interactive_cpuinfo *pcpu = &per_cpu(cpuinfo, cpu);
	u64 expires = round_to_nw_start(pcpu->last_evaluated_jiffy);
	u64 now = ktime_to_us(ktime_get());
	unsigned long flags;

	spin_lock_irqsave(&pcpu->load_lock, flags);
	/* Prepare initial interactive timer and start it onto a passed cpu */
	pcpu->cpu_timer.expires = expires;
	add_timer_on(&pcpu->cpu_timer, cpu);

	/* Rearm slack timer only if device is not in idle state */
	if (timer_slack_val >= 0 &&
	   (pcpu->target_freq > pcpu->policy->min ||
	   (pcpu->target_freq == pcpu->policy->min &&
	    now < boostpulse_endtime))) {
		/* Slack sample is an additional time to delay samples */
		expires += usecs_to_jiffies(timer_slack_val);

		/* Prepare initial slack timer and start it onto a passed cpu */
		pcpu->cpu_slack_timer.expires = expires;
		add_timer_on(&pcpu->cpu_slack_timer, cpu);
	}

	/* Reset time slices to refresh frequency calculation */
	pcpu->time_in_idle = get_cpu_idle_time(cpu,
			&pcpu->time_in_idle_timestamp, io_is_busy);
	pcpu->cputime_speedadj_timestamp = pcpu->time_in_idle_timestamp;
	pcpu->cputime_speedadj = 0;
	spin_unlock_irqrestore(&pcpu->load_lock, flags);
}

#define freq_to_val(name)						\
static inline u32 freq_to_##name(u32 freq)				\
{									\
	unsigned long flags;						\
	u32 target;							\
	int i;								\
									\
	spin_lock_irqsave(&name##_lock, flags);				\
	for (i = 0; i < (n##name - 1) && freq >= name[i + 1]; i += 2)	\
		;							\
									\
	target = name[i];						\
	spin_unlock_irqrestore(&name##_lock, flags);			\
									\
	return target;							\
}

freq_to_val(above_hispeed_delay);
freq_to_val(target_loads);

/*
 * If increasing frequencies never map to a lower target load then
 * choose_freq() will find the minimum frequency that does not exceed its
 * target load given the current load.
 */
static unsigned int choose_freq(struct cpufreq_interactive_cpuinfo *pcpu,
				u32 loadadjfreq, int cpu_load)
{
	u32 freq = pcpu->policy->cur, prevfreq, tl, freqmin = 0, freqmax = ~0;
	u32 min_f = pcpu->policy->cpuinfo.min_freq;
	u32 max_f = pcpu->policy->cpuinfo.max_freq;
	int index;

	/*
	 * Scale the frequency using a current load as a multiplier if current
	 * frequency is below frequency calculation threshold.
	 */
	if (likely(freq <= freq_calc_thresh))
		return (min_f + cpu_load * (max_f - min_f) / 100);

	do {
		prevfreq = freq;
		/* Get target load for a current frequency */
		tl = freq_to_target_loads(freq);

		/*
		 * Find the lowest frequency where the computed] load is less
		 * than or equal to the target load.
		 */
		if (cpufreq_frequency_table_target(pcpu->policy,
		    pcpu->freq_table, loadadjfreq / tl,
		    CPUFREQ_RELATION_L, &index))
			break;

		freq = pcpu->freq_table[index].frequency;
		if (freq > prevfreq) {
			/* The previous frequency is too low */
			freqmin = prevfreq;
			if (freq >= freqmax) {
				/*
				 * Find the highest frequency that is less than
				 * freqmax.
				 */
				if (cpufreq_frequency_table_target(pcpu->policy,
				    pcpu->freq_table, freqmax - 1,
				    CPUFREQ_RELATION_H, &index))
					break;

				freq = pcpu->freq_table[index].frequency;
				if (freq == freqmin) {
					/*
					 * The first frequency below freqmax has
					 * already been found to be too low.
					 * freqmax is the lowest speed we found
					 * that is fast enough.
					 */
					freq = freqmax;
					break;
				}
			}
		} else if (freq < prevfreq) {
			/* The previous frequency is high enough */
			freqmax = prevfreq;
			if (freq <= freqmin) {
				/*
				 * Find the lowest frequency that is higher
				 * than freqmin.
				 */
				if (cpufreq_frequency_table_target(pcpu->policy,
				    pcpu->freq_table, freqmin + 1,
				    CPUFREQ_RELATION_L, &index))
					break;

				/*
				 * If freqmax is the first frequency above
				 * freqmin then we have already found that
				 * this speed is fast enough.
				 */
				freq = pcpu->freq_table[index].frequency;
				if (freq == freqmax)
					break;
			}
		}
	/* If same frequency chosen as previous then done */
	} while (freq != prevfreq);

	return freq;
}

static inline u64 update_load(int cpu)
{
	struct cpufreq_interactive_cpuinfo *pcpu = &per_cpu(cpuinfo, cpu);
	u64 now, now_idle, active_time;
	u32 delta_idle, delta_time;

	/* Some targets want iowait time to be subtracted from idle */
	now_idle = get_cpu_idle_time(cpu, &now, io_is_busy);

	/* Update idle time slice */
	delta_idle = (u32)(now_idle - pcpu->time_in_idle);
	pcpu->time_in_idle = now_idle;

	/*
	 * Update wall time slice after idle one to avoid theoretical
	 * time shift.
	 */
	delta_time = (u32)(now - pcpu->time_in_idle_timestamp);
	pcpu->time_in_idle_timestamp = now;

	/* Safely count active time out of the wall */
	active_time = (delta_time > delta_idle) ? (delta_time - delta_idle) : 0;
	/*
	 * Speed adjustment is a sum of multiplies of a current frequency and
	 * current busy time. This sum gets divided during interactive timer
	 * sample by the differential between timer samples to provide smooth
	 * and averaged load values.
	 */
	pcpu->cputime_speedadj += active_time * pcpu->policy->cur;

	return now;
}

static void cpufreq_interactive_timer(unsigned long data)
{
	struct cpufreq_interactive_cpuinfo *pcpu = &per_cpu(cpuinfo, data);
	u32 delta_time, new_freq, loadadjfreq, index, boosted_freq;
	u64 now, cputime_speedadj;
	unsigned long flags;
	bool boosted;
	int cpu_load;

	if (unlikely(!down_read_trylock(&pcpu->enable_sem)))
		return;
	if (unlikely(!pcpu->governor_enabled))
		goto exit;

	spin_lock_irqsave(&pcpu->load_lock, flags);
	/* Get both speed adjustment and overall wall system time */
	now = update_load(data);

	/*
	 * Count the difference between the last timer reschedule and the
	 * current time. delta_time indicates the interval between this and
	 * previous timer fire.
	 */
	delta_time = (u32)(now - pcpu->cputime_speedadj_timestamp);

	cputime_speedadj = pcpu->cputime_speedadj;
	pcpu->last_evaluated_jiffy = get_jiffies_64();
	spin_unlock_irqrestore(&pcpu->load_lock, flags);

	/* Stop the timer if 2 samples were taken at once */
	if (unlikely(!delta_time))
		goto rearm;

	spin_lock_irqsave(&pcpu->target_freq_lock, flags);
	/*
	 * Count current speed adjustment by dividing it by the delta between
	 * timer samples. It mitigates current load and makes frequency scaling
	 * smoother.
	 */
	do_div(cputime_speedadj, delta_time);

	/*
	 * Now we can count averaged 'load' value just like the ondemand
	 * governor does. Speed adjustment is softened by the above division,
	 * so loadadjfreq practically indicates the average load multiplied
	 * by the average frequency that tries to handle that load right now.
	 */
	loadadjfreq = (u32)cputime_speedadj * 100;

	/*
	 * Detect boost scenario. Also ensure that hispeed_freq is not less
	 * than the minimum frequency of a policy at the moment.
	 */
	boosted = boost_val || now < boostpulse_endtime;
	boosted_freq = max(hispeed_freq, pcpu->policy->min);

	/*
	 * Count average cpu 'load' value by dividing load adjusted frequency
	 * by the current frequency. As:
	 *
	 * loadadjfreq = pcpu->policy->cur *
	 *		(wall_time - idle_time) / aligned(wall_time) * 100;
	 *
	 * We can get current cpu 'load' value by simply dividing it by the
	 * current cpu frequency.
	 */
	cpu_load = loadadjfreq / pcpu->policy->cur;
	if (cpu_load >= go_hispeed_load || boosted) {
		if (pcpu->policy->cur < boosted_freq) {
			new_freq = boosted_freq;
		} else {
			new_freq = choose_freq(pcpu, loadadjfreq, cpu_load);
			new_freq = max(new_freq, boosted_freq);
		}
	} else {
		new_freq = choose_freq(pcpu, loadadjfreq, cpu_load);
		if (new_freq > boosted_freq && pcpu->target_freq < boosted_freq)
			new_freq = boosted_freq;
	}

	/*
	 * Do not switch to a new frequency if it is higher than hispeed_freq
	 * and above hispeed_freq delay of the current frequency is not
	 * completed yet.
	 */
	if (pcpu->policy->cur >= boosted_freq &&
	    new_freq > pcpu->policy->cur &&
	    freq_to_above_hispeed_delay(pcpu->policy->cur) >
	   (now - pcpu->hispeed_validate_time)) {
		trace_cpufreq_interactive_notyet(data, cpu_load,
						 pcpu->target_freq,
						 pcpu->policy->cur, new_freq);
		spin_unlock_irqrestore(&pcpu->target_freq_lock, flags);
		goto rearm;
	}

	/*
	 * Above hispeed_freq delay is passed. Update hispeed validation
	 * timestamp to be ready for a next sample.
	 */
	pcpu->local_hvtime = now;

	/* Try to safely get the closest to new_freq real cpu frequency */
	if (cpufreq_frequency_table_target(pcpu->policy, pcpu->freq_table,
	    new_freq, CPUFREQ_RELATION_C, &index)) {
		spin_unlock_irqrestore(&pcpu->target_freq_lock, flags);
		goto rearm;
	}

	/* Assign closest real frequency to a target frequency */
	new_freq = pcpu->freq_table[index].frequency;

	/*
	 * Do not scale down the frequency if maximum frequency hysteresis is
	 * not completed yet. Up scales are untouched.
	 */
	if (new_freq < pcpu->target_freq &&
	    max_freq_hysteresis > (now - pcpu->max_freq_hyst_start_time)) {
		trace_cpufreq_interactive_notyet(data, cpu_load,
						 pcpu->target_freq,
						 pcpu->policy->cur, new_freq);
		spin_unlock_irqrestore(&pcpu->target_freq_lock, flags);
		goto rearm;
	}

	/*
	 * Do not scale below floor_freq unless we have been at or above the
	 * floor frequency for the minimum sample time since last validated.
	 */
	if (new_freq < pcpu->floor_freq &&
	    min_sample_time > (now - pcpu->floor_validate_time)) {
		trace_cpufreq_interactive_notyet(data, cpu_load,
						 pcpu->target_freq,
						 pcpu->policy->cur, new_freq);
		spin_unlock_irqrestore(&pcpu->target_freq_lock, flags);
		goto rearm;
	}

	/*
	 * Update the timestamp for checking whether speed has been held at or
	 * above the selected frequency for a minimum of min_sample_time, if not
	 * boosted to boosted_freq. If boosted to boosted_freq then we allow the
	 * speed to drop as soon as the boostpulse duration expires (or the
	 * indefinite boost is turned off).
	 */
	if (!boosted || new_freq > boosted_freq) {
		pcpu->floor_freq = new_freq;
		pcpu->floor_validate_time = now;
	}

	if (new_freq == pcpu->policy->max)
		pcpu->max_freq_hyst_start_time = now;

	/* Return early if target frequency is equal to a current frequency */
	if (pcpu->target_freq == new_freq &&
	    pcpu->target_freq <= pcpu->policy->cur) {
		trace_cpufreq_interactive_already(data, cpu_load,
						  pcpu->target_freq,
						  pcpu->policy->cur, new_freq);
		spin_unlock_irqrestore(&pcpu->target_freq_lock, flags);
		goto rearm;
	}

	trace_cpufreq_interactive_target(data, cpu_load, pcpu->target_freq,
					 pcpu->policy->cur, new_freq);

	pcpu->target_freq = new_freq;
	spin_unlock_irqrestore(&pcpu->target_freq_lock, flags);

	spin_lock_irqsave(&speedchange_cpumask_lock, flags);
	/* Set the cpu which is running this timer to a speedchange mask */
	cpumask_set_cpu(data, &speedchange_cpumask);
	spin_unlock_irqrestore(&speedchange_cpumask_lock, flags);

	/* Wake-up frequency scaling thread */
	wake_up_process(speedchange_task);
rearm:
	if (!timer_pending(&pcpu->cpu_timer))
		cpufreq_interactive_timer_resched(data, false);
exit:
	up_read(&pcpu->enable_sem);
}

static inline void cpufreq_interactive_idle_end(void)
{
	struct cpufreq_interactive_cpuinfo *pcpu =
			&per_cpu(cpuinfo, smp_processor_id());

	if (unlikely(!down_read_trylock(&pcpu->enable_sem)))
		return;
	if (unlikely(!pcpu->governor_enabled))
		goto exit;

	/* Arm the timer for 1-2 ticks later if not already */
	if (!timer_pending(&pcpu->cpu_timer)) {
		cpufreq_interactive_timer_resched(smp_processor_id(), false);
	} else if (time_after_eq(jiffies, pcpu->cpu_timer.expires)) {
		del_timer(&pcpu->cpu_timer);
		del_timer(&pcpu->cpu_slack_timer);
		cpufreq_interactive_timer(smp_processor_id());
	}
exit:
	up_read(&pcpu->enable_sem);
}

static int cpufreq_interactive_speedchange_task(void *data)
{
	unsigned long flags;
	cpumask_t tmp_mask;
	u32 cpu;

	while (1) {
		set_current_state(TASK_INTERRUPTIBLE);

		spin_lock_irqsave(&speedchange_cpumask_lock, flags);
		if (cpumask_empty(&speedchange_cpumask)) {
			spin_unlock_irqrestore(&speedchange_cpumask_lock,
					       flags);

			schedule();
			if (kthread_should_stop())
				break;

			spin_lock_irqsave(&speedchange_cpumask_lock, flags);
		}

		set_current_state(TASK_RUNNING);

		tmp_mask = speedchange_cpumask;
		cpumask_clear(&speedchange_cpumask);
		spin_unlock_irqrestore(&speedchange_cpumask_lock, flags);

		/* Iterate over all cpus in a mask */
		for_each_cpu(cpu, &tmp_mask) {
			struct cpufreq_interactive_cpuinfo *pcpu, *pjcpu;
			u32 max_freq = 0, j;
			u64 hvt = 0;

			pcpu = &per_cpu(cpuinfo, cpu);
			if (unlikely(!down_read_trylock(&pcpu->enable_sem)))
				continue;
			if (unlikely(!pcpu->governor_enabled)) {
				up_read(&pcpu->enable_sem);
				continue;
			}

			/*
			 * Get the maximum target frequency across all cpus in
			 * a policy along with the earliest hispeed validation
			 * timestamp.
			 */
			for_each_cpu(j, pcpu->policy->cpus) {
				pjcpu = &per_cpu(cpuinfo, j);
				if (pjcpu->target_freq > max_freq) {
					max_freq = pjcpu->target_freq;
					hvt = pjcpu->local_hvtime;
				} else if (pjcpu->target_freq == max_freq) {
					hvt = min(hvt, pjcpu->local_hvtime);
				}
			}

			/*
			 * Try to gracefully switch to a maximum target
			 * frequency and refresh hispeed validation time
			 * again as it is the moment the frequency is really
			 * scaled.
			 */
			if (max_freq != pcpu->policy->cur) {
				__cpufreq_driver_target(pcpu->policy, max_freq,
							CPUFREQ_RELATION_C);
				for_each_cpu(j, pcpu->policy->cpus) {
					pjcpu = &per_cpu(cpuinfo, j);
					pjcpu->hispeed_validate_time = hvt;
				}
			}

			trace_cpufreq_interactive_setspeed(cpu,
							   pcpu->target_freq,
							   pcpu->policy->cur);
			up_read(&pcpu->enable_sem);
		}
	}

	return 0;
}

static inline void cpufreq_interactive_boost(void)
{
	unsigned long flags[2];
	int anyboost = 0, i;

	spin_lock_irqsave(&speedchange_cpumask_lock, flags[0]);
	for_each_online_cpu(i) {
		struct cpufreq_interactive_cpuinfo *pcpu = &per_cpu(cpuinfo, i);

		spin_lock_irqsave(&pcpu->target_freq_lock, flags[1]);
		/*
		 * Wake-up frequency scaling thread only if target frequency is
		 * less than hispeed_freq.
		 */
		if (pcpu->target_freq < hispeed_freq) {
			pcpu->target_freq = hispeed_freq;
			cpumask_set_cpu(i, &speedchange_cpumask);
			pcpu->hispeed_validate_time = ktime_to_us(ktime_get());
			anyboost = 1;
		}

		/* Set floor freq and (re)start timer for when last validated */
		pcpu->floor_freq = hispeed_freq;
		pcpu->floor_validate_time = ktime_to_us(ktime_get());
		spin_unlock_irqrestore(&pcpu->target_freq_lock, flags[1]);
	}
	spin_unlock_irqrestore(&speedchange_cpumask_lock, flags[0]);

	if (anyboost)
		wake_up_process(speedchange_task);
}

static int cpufreq_interactive_notifier(struct notifier_block *nb,
					unsigned long val, void *data)
{
	struct cpufreq_interactive_cpuinfo *pcpu, *pjcpu;
	struct cpufreq_freqs *freq = data;
	unsigned long flags;
	int cpu;

	if (val != CPUFREQ_PRECHANGE)
		return NOTIFY_OK;

	pcpu = &per_cpu(cpuinfo, freq->cpu);
	if (unlikely(!down_read_trylock(&pcpu->enable_sem)))
		return NOTIFY_OK;
	if (unlikely(!pcpu->governor_enabled))
		goto exit;

	for_each_cpu(cpu, pcpu->policy->cpus) {
		pjcpu = &per_cpu(cpuinfo, cpu);

		/* A main cpu is helding the lock already */
		if (cpu != freq->cpu) {
			if (unlikely(!down_read_trylock(&pjcpu->enable_sem)))
				continue;
			if (unlikely(!pjcpu->governor_enabled)) {
				up_read(&pjcpu->enable_sem);
				continue;
			}
		}

		spin_lock_irqsave(&pjcpu->load_lock, flags);
		/* Update time slices before frequency change */
		update_load(cpu);
		spin_unlock_irqrestore(&pjcpu->load_lock, flags);

		if (cpu != freq->cpu)
			up_read(&pjcpu->enable_sem);
	}
exit:
	up_read(&pcpu->enable_sem);

	return NOTIFY_DONE;
}

static struct notifier_block cpufreq_notifier_block = {
	.notifier_call = cpufreq_interactive_notifier,
};

#define show_one_u32(name)					\
static ssize_t show_##name(struct kobject *kobj,		\
			   struct attribute *attr,		\
			   char *buf)				\
{								\
	return scnprintf(buf, 12, "%u\n", name);		\
}

#define store_one_u32(name, min, max)				\
static ssize_t store_##name(struct kobject *kobj,		\
			    struct attribute *attr,		\
			    const char *buf, size_t count)	\
{								\
	int ret;						\
	unsigned int val;					\
								\
	ret = kstrtouint(buf, 10, &val);			\
	if (ret || val < min || val > max)			\
		return -EINVAL;					\
								\
	name = val;						\
								\
	return count;						\
}

#define create_one_global_rw(name)				\
static struct global_attr name##_attr =				\
	__ATTR(name, 0644, show_##name, store_##name);

#define define_one_u32_rw(name, min, max)			\
show_one_u32(name);						\
store_one_u32(name, min, max);					\
create_one_global_rw(name);

static inline u32 *get_tokenized_data(const char *buf, int *num_tokens)
{
	int ntokens = 1, err = -EINVAL, ret, i = 0;
	const char *cp = buf;
	u32 *tokenized_data;

	while ((cp = strpbrk(cp + 1, " :")))
		ntokens++;

	if (!(ntokens & 0x1))
		goto err;

	tokenized_data = kzalloc(sizeof(*tokenized_data) * ntokens, GFP_KERNEL);
	if (IS_ERR_OR_NULL(tokenized_data)) {
		err = -ENOMEM;
		goto err;
	}

	cp = buf;
	while (i < ntokens) {
		ret = sscanf(cp, "%u", &tokenized_data[i++]);
		if (ret != 1)
			goto err_kfree;

		cp = strpbrk(cp, " :");
		if (IS_ERR_OR_NULL(cp))
			break;
		cp++;
	}

	if (unlikely(i != ntokens))
		goto err_kfree;

	*num_tokens = ntokens;

	return tokenized_data;

err_kfree:
	kfree(tokenized_data);
err:
	return ERR_PTR(err);
}

static ssize_t show_target_loads(struct kobject *kobj,
				 struct attribute *attr,
				 char *buf)
{
	unsigned long flags;
	ssize_t len = 0;
	int i;

	spin_lock_irqsave(&target_loads_lock, flags);
	for (i = 0; i < ntarget_loads; i++)
		len += scnprintf(buf + len, 12, "%u%s", target_loads[i],
				 i & 0x1 ? ":" : " ");

	scnprintf(buf + len - 1, 2, "\n");
	spin_unlock_irqrestore(&target_loads_lock, flags);

	return len;
}

static ssize_t store_target_loads(struct kobject *kobj,
				  struct attribute *attr,
				  const char *buf, size_t count)
{
	unsigned int *new_target_loads;
	unsigned long flags;
	int ntokens = 0;

	new_target_loads = get_tokenized_data(buf, &ntokens);
	if (IS_ERR_OR_NULL(new_target_loads))
		return -EINVAL;

	spin_lock_irqsave(&target_loads_lock, flags);
	if (likely(target_loads != default_target_loads))
		kzfree(target_loads);

	target_loads = new_target_loads;
	ntarget_loads = ntokens;
	spin_unlock_irqrestore(&target_loads_lock, flags);

	return count;
}

create_one_global_rw(target_loads);

static ssize_t show_above_hispeed_delay(struct kobject *kobj,
					struct attribute *attr,
					char *buf)
{
	unsigned long flags;
	ssize_t len = 0;
	int i;

	spin_lock_irqsave(&above_hispeed_delay_lock, flags);
	for (i = 0; i < nabove_hispeed_delay; i++)
		len += scnprintf(buf + len, 12, "%u%s", above_hispeed_delay[i],
				 i & 0x1 ? ":" : " ");

	scnprintf(buf + len - 1, 2, "\n");
	spin_unlock_irqrestore(&above_hispeed_delay_lock, flags);

	return len;
}

static ssize_t store_above_hispeed_delay(struct kobject *kobj,
					 struct attribute *attr,
					 const char *buf, size_t count)
{
	unsigned int *new_above_hispeed_delay;
	unsigned long flags;
	int ntokens = 0, i;

	new_above_hispeed_delay = get_tokenized_data(buf, &ntokens);
	if (IS_ERR_OR_NULL(new_above_hispeed_delay))
		return -EINVAL;

	/* Make sure frequencies are in ascending order */
	for (i = 3; i < ntokens; i += 2) {
		if (unlikely(new_above_hispeed_delay[i] <=
		    new_above_hispeed_delay[i - 2])) {
			kfree(new_above_hispeed_delay);
			return -EINVAL;
		}
	}

	spin_lock_irqsave(&above_hispeed_delay_lock, flags);
	if (likely(above_hispeed_delay != default_above_hispeed_delay))
		kzfree(above_hispeed_delay);

	above_hispeed_delay = new_above_hispeed_delay;
	nabove_hispeed_delay = ntokens;
	spin_unlock_irqrestore(&above_hispeed_delay_lock, flags);

	return count;
}

create_one_global_rw(above_hispeed_delay);

static ssize_t show_min_sample_time(struct kobject *kobj,
				    struct attribute *attr,
				    char *buf)
{
	return scnprintf(buf, 22, "%lu\n", min_sample_time);
}

static ssize_t store_min_sample_time(struct kobject *kobj,
				     struct attribute *attr,
				     const char *buf, size_t count)
{
	int ret;
	unsigned long val;

	ret = kstrtoul(buf, 10, &val);
	if (IS_ERR_VALUE(ret))
		return -EINVAL;

	min_sample_time = val;

	return count;
}

create_one_global_rw(min_sample_time);

static ssize_t show_timer_rate(struct kobject *kobj,
			       struct attribute *attr,
			       char *buf)
{
	return scnprintf(buf, 22, "%lu\n", timer_rate);
}

static ssize_t store_timer_rate(struct kobject *kobj,
				struct attribute *attr,
				const char *buf, size_t count)
{
	int ret;
	unsigned long val, val_round;

	ret = kstrtoul(buf, 10, &val);
	if (IS_ERR_VALUE(ret))
		return -EINVAL;

	val_round = jiffies_to_usecs(usecs_to_jiffies(val));
	if (unlikely(val != val_round))
		pr_debug("timer_rate was rounded up to %lu\n", val_round);

	timer_rate = val_round;

	return count;
}

create_one_global_rw(timer_rate);

static ssize_t show_timer_slack(struct kobject *kobj,
				struct attribute *attr,
				char *buf)
{
	return scnprintf(buf, 12, "%d\n", timer_slack_val);
}

static ssize_t store_timer_slack(struct kobject *kobj,
				 struct attribute *attr,
				 const char *buf, size_t count)
{
	int ret;
	long val;

	ret = kstrtol(buf, 10, &val);
	if (IS_ERR_VALUE(ret))
		return -EINVAL;

	timer_slack_val = val;

	return count;
}

define_one_global_rw(timer_slack);

static ssize_t show_boost(struct kobject *kobj,
			  struct attribute *attr,
			  char *buf)
{
	return scnprintf(buf, 3, "%d\n", boost_val);
}

static ssize_t store_boost(struct kobject *kobj,
			   struct attribute *attr,
			   const char *buf, size_t count)
{
	int ret;
	unsigned long val;

	ret = kstrtoul(buf, 2, &val);
	if (IS_ERR_VALUE(ret))
		return -EINVAL;

	boost_val = val;
	if (boost_val) {
		trace_cpufreq_interactive_boost("on");
		cpufreq_interactive_boost();
	} else {
		boostpulse_endtime = ktime_to_us(ktime_get());
		trace_cpufreq_interactive_unboost("off");
	}

	return count;
}

define_one_global_rw(boost);

static ssize_t store_boostpulse(struct kobject *kobj,
				struct attribute *attr,
				const char *buf, size_t count)
{
	int ret;
	unsigned long val;

	ret = kstrtoul(buf, 10, &val);
	if (IS_ERR_VALUE(ret))
		return -EINVAL;

	boostpulse_endtime = ktime_to_us(ktime_get()) + boostpulse_duration_val;

	trace_cpufreq_interactive_boost("pulse");
	cpufreq_interactive_boost();

	return count;
}

static struct global_attr boostpulse =
	__ATTR(boostpulse, 0200, NULL, store_boostpulse);

static ssize_t show_boostpulse_duration(struct kobject *kobj,
					struct attribute *attr,
					char *buf)
{
	return scnprintf(buf, 22, "%lu\n", boostpulse_duration_val);
}

static ssize_t store_boostpulse_duration(struct kobject *kobj,
					 struct attribute *attr,
					 const char *buf, size_t count)
{
	int ret;
	unsigned long val;

	ret = kstrtoul(buf, 10, &val);
	if (IS_ERR_VALUE(ret))
		return -EINVAL;

	boostpulse_duration_val = val;

	return count;
}

define_one_global_rw(boostpulse_duration);

define_one_u32_rw(hispeed_freq, 0, UINT_MAX);
define_one_u32_rw(max_freq_hysteresis, 0, UINT_MAX);
define_one_u32_rw(align_windows, 0, 1);
define_one_u32_rw(go_hispeed_load, 0, 100);
define_one_u32_rw(freq_calc_thresh, 0, UINT_MAX);
define_one_u32_rw(io_is_busy, 0, 1);

static struct attribute *interactive_attributes[] = {
	&target_loads_attr.attr,
	&above_hispeed_delay_attr.attr,
	&hispeed_freq_attr.attr,
	&go_hispeed_load_attr.attr,
	&freq_calc_thresh_attr.attr,
	&min_sample_time_attr.attr,
	&timer_rate_attr.attr,
	&timer_slack.attr,
	&boost.attr,
	&boostpulse.attr,
	&boostpulse_duration.attr,
	&io_is_busy_attr.attr,
	&max_freq_hysteresis_attr.attr,
	&align_windows_attr.attr,
	NULL,
};

static struct attribute_group interactive_attr_group = {
	.name = "interactive",
	.attrs = interactive_attributes,
};

static int cpufreq_interactive_idle_notifier(struct notifier_block *nb,
					     unsigned long val,
					     void *data)
{
	if (val != IDLE_END)
		return NOTIFY_OK;

	/* Reschedule interactive timer when cpu exits idle */
	cpufreq_interactive_idle_end();

	return NOTIFY_DONE;
}

static struct notifier_block cpufreq_interactive_idle_nb = {
	.notifier_call = cpufreq_interactive_idle_notifier,
};

static inline int it_init(void)
{
	int ret;

	/* Constants below should be initialized only once */
	if (++gov_enable_cnt != 1)
		return 0;

	ret = cpufreq_register_notifier(&cpufreq_notifier_block,
					CPUFREQ_TRANSITION_NOTIFIER);
	if (IS_ERR_VALUE(ret)) {
		pr_err("Unable to register cpufreq notifier\n");
		goto fail_cpufreq;
	}

	idle_notifier_register(&cpufreq_interactive_idle_nb);

	ret = sysfs_create_group(cpufreq_global_kobject,
				&interactive_attr_group);
	if (IS_ERR_VALUE(ret)) {
		pr_err("Unable to create sysfs group\n");
		goto fail_sysfs;
	}

	return 0;

fail_sysfs:
	idle_notifier_unregister(&cpufreq_interactive_idle_nb);
	cpufreq_unregister_notifier(&cpufreq_notifier_block,
				    CPUFREQ_TRANSITION_NOTIFIER);
fail_cpufreq:
	gov_enable_cnt--;

	return ret;
}

static inline void it_exit(void)
{
	/* Constants below should be voided only once */
	if (--gov_enable_cnt != 0)
		return;

	sysfs_remove_group(cpufreq_global_kobject, &interactive_attr_group);
	idle_notifier_unregister(&cpufreq_interactive_idle_nb);
	cpufreq_unregister_notifier(&cpufreq_notifier_block,
				    CPUFREQ_TRANSITION_NOTIFIER);
}

static int
cpufreq_governor_interactive(struct cpufreq_policy *policy, u32 event)
{
	struct cpufreq_frequency_table *freq_table;
	struct cpufreq_interactive_cpuinfo *pcpu;
	int cpu = policy->cpu, ret;
	unsigned long flags;
	u32 anyboost, j;

	switch (event) {
	case CPUFREQ_GOV_START:
		/* This cannot happen, but be safe anyway */
		if (unlikely(!cpu_online(cpu) || !policy->cur))
			return -EINVAL;

		mutex_lock(&gov_lock);
		/* Check current values of some variables */
		if (!hispeed_freq)
			hispeed_freq = policy->max;

		freq_calc_thresh =
			clamp(freq_calc_thresh, policy->min, policy->max);

		freq_table = cpufreq_frequency_get_table(cpu);
		/* Update percpu data in a whole policy */
		for_each_cpu(j, policy->cpus) {
			pcpu = &per_cpu(cpuinfo, j);

			pcpu->policy = policy;
			pcpu->target_freq = policy->cur;
			pcpu->freq_table = freq_table;
			pcpu->floor_freq = pcpu->target_freq;
			pcpu->floor_validate_time = ktime_to_us(ktime_get());
			pcpu->hispeed_validate_time = pcpu->floor_validate_time;
			pcpu->local_hvtime = pcpu->floor_validate_time;
			pcpu->min_freq = policy->min;

			down_write(&pcpu->enable_sem);
			del_timer_sync(&pcpu->cpu_timer);
			del_timer_sync(&pcpu->cpu_slack_timer);

			pcpu->last_evaluated_jiffy = get_jiffies_64();
			/* Start the governor */
			cpufreq_interactive_timer_start(j);

			pcpu->governor_enabled = 1;
			up_write(&pcpu->enable_sem);
		}

		/* Some resources have to be initialized only once */
		ret = it_init();
		if (IS_ERR_VALUE(ret)) {
			mutex_unlock(&gov_lock);
			return ret;
		}
		mutex_unlock(&gov_lock);
		break;
	case CPUFREQ_GOV_STOP:
		mutex_lock(&gov_lock);
		for_each_cpu(j, policy->cpus) {
			pcpu = &per_cpu(cpuinfo, j);

			down_write(&pcpu->enable_sem);
			pcpu->governor_enabled = pcpu->target_freq = 0;

			del_timer_sync(&pcpu->cpu_timer);
			del_timer_sync(&pcpu->cpu_slack_timer);
			up_write(&pcpu->enable_sem);
		}

		it_exit();
		mutex_unlock(&gov_lock);
		break;
	case CPUFREQ_GOV_LIMITS:
		/* Do not miss a sample here */
		__cpufreq_driver_target(policy, policy->cur,
					CPUFREQ_RELATION_L);

		for_each_cpu(j, policy->cpus) {
			pcpu = &per_cpu(cpuinfo, j);

			down_read(&pcpu->enable_sem);
			if (unlikely(!pcpu->governor_enabled)) {
				up_read(&pcpu->enable_sem);
				continue;
			}

			spin_lock_irqsave(&pcpu->target_freq_lock, flags);
			if (pcpu->target_freq > policy->max) {
				pcpu->target_freq = policy->max;
			} else if (pcpu->target_freq <= policy->min) {
				pcpu->target_freq = policy->min;
				anyboost = 1;
			}
			spin_unlock_irqrestore(&pcpu->target_freq_lock, flags);

			if (policy->min < pcpu->min_freq)
				cpufreq_interactive_timer_resched(j, true);

			pcpu->min_freq = policy->min;
			up_read(&pcpu->enable_sem);

			if (anyboost) {
				u64 now = ktime_to_us(ktime_get());

				cpumask_set_cpu(j, &speedchange_cpumask);

				pcpu->hispeed_validate_time = now;
				pcpu->floor_freq = policy->min;
				pcpu->floor_validate_time = now;
			}
		}

		if (anyboost)
			wake_up_process(speedchange_task);
		break;
	}

	return 0;
}

#ifndef CONFIG_CPU_FREQ_DEFAULT_GOV_INTERACTIVE
static
#endif
struct cpufreq_governor cpufreq_gov_interactive = {
	.name			= "interactive",
	.governor		= cpufreq_governor_interactive,
	.max_transition_latency = TRANSITION_LATENCY_LIMIT,
	.owner			= THIS_MODULE,
};

static void cpufreq_interactive_nop_timer(unsigned long data) { }

static int __init cpufreq_interactive_init(void)
{
	struct cpufreq_interactive_cpuinfo *pcpu;
	struct sched_param param = { .sched_priority = MAX_RT_PRIO - 1 };
	int cpu;

	/* Initalize per-cpu timers */
	for_each_possible_cpu(cpu) {
		pcpu = &per_cpu(cpuinfo, cpu);

		init_timer_deferrable(&pcpu->cpu_timer);
		pcpu->cpu_timer.function = cpufreq_interactive_timer;
		pcpu->cpu_timer.data = cpu;

		init_timer(&pcpu->cpu_slack_timer);
		pcpu->cpu_slack_timer.function = cpufreq_interactive_nop_timer;

		spin_lock_init(&pcpu->load_lock);
		spin_lock_init(&pcpu->target_freq_lock);
		init_rwsem(&pcpu->enable_sem);
	}

	spin_lock_init(&target_loads_lock);
	spin_lock_init(&speedchange_cpumask_lock);
	spin_lock_init(&above_hispeed_delay_lock);

	mutex_init(&gov_lock);

	speedchange_task = kthread_create(cpufreq_interactive_speedchange_task,
					  NULL, "cfinteractive");
	if (IS_ERR_OR_NULL(speedchange_task))
		return PTR_ERR(speedchange_task);

	sched_setscheduler_nocheck(speedchange_task, SCHED_FIFO, &param);
	get_task_struct(speedchange_task);

	/* NB: Wake up so the thread does not look hung to the freezer */
	wake_up_process(speedchange_task);

	return cpufreq_register_governor(&cpufreq_gov_interactive);
}

static void __exit cpufreq_interactive_exit(void)
{
	cpufreq_unregister_governor(&cpufreq_gov_interactive);

	kthread_stop(speedchange_task);
	put_task_struct(speedchange_task);

	if (target_loads != default_target_loads)
		kzfree(target_loads);
	if (above_hispeed_delay != default_above_hispeed_delay)
		kzfree(above_hispeed_delay);
}

#ifdef CONFIG_CPU_FREQ_DEFAULT_GOV_INTERACTIVE
fs_initcall(cpufreq_interactive_init);
#else
module_init(cpufreq_interactive_init);
#endif
module_exit(cpufreq_interactive_exit);

MODULE_AUTHOR("Mike Chan <mike@android.com>");
MODULE_DESCRIPTION("'cpufreq_interactive' - A cpufreq governor for "
		   "Latency sensitive workloads");
MODULE_LICENSE("GPL v2");
