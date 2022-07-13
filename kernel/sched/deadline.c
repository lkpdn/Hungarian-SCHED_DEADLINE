// SPDX-License-Identifier: GPL-2.0
/*
 * Deadline Scheduling Class (SCHED_DEADLINE)
 *
 * Earliest Deadline First (EDF) + Constant Bandwidth Server (CBS).
 *
 * Tasks that periodically executes their instances for less than their
 * runtime won't miss any of their deadlines.
 * Tasks that are not periodic or sporadic or that tries to execute more
 * than their reserved bandwidth will be slowed down (and may potentially
 * miss some of their deadlines), and won't affect any other task.
 *
 * Copyright (C) 2012 Dario Faggioli <raistlin@linux.it>,
 *                    Juri Lelli <juri.lelli@gmail.com>,
 *                    Michael Trimarchi <michael@amarulasolutions.com>,
 *                    Fabio Checconi <fchecconi@gmail.com>
 */

/*
 * Default limits for DL period; on the top end we guard against small util
 * tasks still getting ridiculously long effective runtimes, on the bottom end we
 * guard against timer DoS.
 */
static unsigned int sysctl_sched_dl_period_max = 1 << 22; /* ~4 seconds */
static unsigned int sysctl_sched_dl_period_min = 100;     /* 100 us */

/*
 *  SCHED_DEADLINE is now Unr-EDF-like.
 *
 *  0 - (default) use the original phi.
 *
 *      \Phi_i(t) \triangleq T_max + D_i(t) - d_i(t) if \tau_i in \tau^P(t) else 0
 *
 *  1 - use pseudo-release time instead of pseudo-deadline as an approx. current time,
 *      so as to make EDF work amongst just-released jobs from different tasks.
 *
 *      \Phi_i(t) \triangleq T_max + R_i(t) - d_i(t) if \tau_i in \tau^P(t) else 0
 *
 *  2 - on top of 1, make it laxity-aware (LLGF(Least Laxity-Group First)-like)
 *
 *      \Phi_i(t) \triangleq T_max + R_i(t) - (d_i(t) - c_i(t)) if \tau_i in \tau^P(t) else 0
 */
static unsigned int sysctl_sched_dl_phi_mode = 0;

#ifdef CONFIG_SYSCTL
static struct ctl_table sched_dl_sysctls[] = {
	{
		.procname       = "sched_deadline_period_max_us",
		.data           = &sysctl_sched_dl_period_max,
		.maxlen         = sizeof(unsigned int),
		.mode           = 0644,
		.proc_handler   = proc_douintvec_minmax,
		.extra1         = (void *)&sysctl_sched_dl_period_min,
	},
	{
		.procname       = "sched_deadline_period_min_us",
		.data           = &sysctl_sched_dl_period_min,
		.maxlen         = sizeof(unsigned int),
		.mode           = 0644,
		.proc_handler   = proc_douintvec_minmax,
		.extra2         = (void *)&sysctl_sched_dl_period_max,
	},
	{
		.procname       = "sched_deadline_phi_mode",
		.data           = &sysctl_sched_dl_phi_mode,
		.maxlen         = sizeof(unsigned int),
		.mode           = 0644,
		.proc_handler   = proc_douintvec,
	},
	{}
};

static int __init sched_dl_sysctl_init(void)
{
	register_sysctl_init("kernel", sched_dl_sysctls);
	return 0;
}
late_initcall(sched_dl_sysctl_init);
#endif

static inline struct task_struct *dl_task_of(struct sched_dl_entity *dl_se)
{
	return container_of(dl_se, struct task_struct, dl);
}

static inline struct rq *rq_of_dl_rq(struct dl_rq *dl_rq)
{
	return container_of(dl_rq, struct rq, dl);
}

static inline struct dl_rq *dl_rq_of_se(struct sched_dl_entity *dl_se)
{
	struct task_struct *p = dl_task_of(dl_se);
	struct rq *rq = task_rq(p);

	return &rq->dl;
}

static inline int on_dl_rq(struct sched_dl_entity *dl_se)
{
	return dl_se->xi != -1;
}

static inline int assigned_cpu_of_task(struct task_struct *p)
{
	struct sched_dl_entity *dl_se = &p->dl;

	if (!on_dl_rq(dl_se))
		return -1;

	return hm_mate_of_xi(task_rq(p)->rd->dl_hg, dl_se->xi);
}

static inline int assigned_runnable_cpu_of_task(struct task_struct *p)
{
	int cpu;

	cpu = assigned_cpu_of_task(p);
	/*
	 * Even if a matched vertex for the given task is present in the graph,
	 * its mate cpu is regarded as not being assigned to it unless included
	 * in the allowed cpu mask.
	 */
	if (cpu >= 0 && cpumask_test_cpu(cpu, &p->cpus_mask))
		return cpu;
	return -1;
}

#ifdef CONFIG_RT_MUTEXES
static inline struct sched_dl_entity *pi_of(struct sched_dl_entity *dl_se)
{
	return dl_se->pi_se;
}

static inline bool is_dl_boosted(struct sched_dl_entity *dl_se)
{
	return pi_of(dl_se) != dl_se;
}
#else
static inline struct sched_dl_entity *pi_of(struct sched_dl_entity *dl_se)
{
	return dl_se;
}

static inline bool is_dl_boosted(struct sched_dl_entity *dl_se)
{
	return false;
}
#endif

#ifdef CONFIG_SMP
static inline struct dl_bw *dl_bw_of(int i)
{
	RCU_LOCKDEP_WARN(!rcu_read_lock_sched_held(),
			 "sched RCU must be held");
	return &cpu_rq(i)->rd->dl_bw;
}

static inline int dl_bw_cpus(int i)
{
	struct root_domain *rd = cpu_rq(i)->rd;
	int cpus;

	RCU_LOCKDEP_WARN(!rcu_read_lock_sched_held(),
			 "sched RCU must be held");

	if (cpumask_subset(rd->span, cpu_active_mask))
		return cpumask_weight(rd->span);

	cpus = 0;

	for_each_cpu_and(i, rd->span, cpu_active_mask)
		cpus++;

	return cpus;
}

static inline unsigned long __dl_bw_capacity(int i)
{
	struct root_domain *rd = cpu_rq(i)->rd;
	unsigned long cap = 0;

	RCU_LOCKDEP_WARN(!rcu_read_lock_sched_held(),
			 "sched RCU must be held");

	for_each_cpu_and(i, rd->span, cpu_active_mask)
		cap += capacity_orig_of(i);

	return cap;
}

/*
 * XXX Fix: If 'rq->rd == def_root_domain' perform AC against capacity
 * of the CPU the task is running on rather rd's \Sum CPU capacity.
 */
static inline unsigned long dl_bw_capacity(int i)
{
	if (!static_branch_unlikely(&sched_asym_cpucapacity) &&
	    capacity_orig_of(i) == SCHED_CAPACITY_SCALE) {
		return dl_bw_cpus(i) << SCHED_CAPACITY_SHIFT;
	} else {
		return __dl_bw_capacity(i);
	}
}

static inline bool dl_bw_visited(int cpu, u64 gen)
{
	struct root_domain *rd = cpu_rq(cpu)->rd;

	if (rd->visit_gen == gen)
		return true;

	rd->visit_gen = gen;
	return false;
}

static inline
void __dl_update(struct dl_bw *dl_b, s64 bw)
{
	struct root_domain *rd = container_of(dl_b, struct root_domain, dl_bw);
	int i;

	RCU_LOCKDEP_WARN(!rcu_read_lock_sched_held(),
			 "sched RCU must be held");
	for_each_cpu_and(i, rd->span, cpu_active_mask) {
		struct rq *rq = cpu_rq(i);

		rq->dl.extra_bw += bw;
	}
}
#else
static inline struct dl_bw *dl_bw_of(int i)
{
	return &cpu_rq(i)->dl.dl_bw;
}

static inline int dl_bw_cpus(int i)
{
	return 1;
}

static inline unsigned long dl_bw_capacity(int i)
{
	return SCHED_CAPACITY_SCALE;
}

static inline bool dl_bw_visited(int cpu, u64 gen)
{
	return false;
}

static inline
void __dl_update(struct dl_bw *dl_b, s64 bw)
{
	struct dl_rq *dl = container_of(dl_b, struct dl_rq, dl_bw);

	dl->extra_bw += bw;
}
#endif

static inline
void __dl_sub(struct dl_bw *dl_b, u64 tsk_bw, int cpus)
{
	dl_b->total_bw -= tsk_bw;
	__dl_update(dl_b, (s32)tsk_bw / cpus);
}

static inline
void __dl_add(struct dl_bw *dl_b, u64 tsk_bw, int cpus)
{
	dl_b->total_bw += tsk_bw;
	__dl_update(dl_b, -((s32)tsk_bw / cpus));
}

static inline bool
__dl_overflow(struct dl_bw *dl_b, unsigned long cap, u64 old_bw, u64 new_bw)
{
	return dl_b->bw != -1 &&
	       cap_scale(dl_b->bw, cap) < dl_b->total_bw - old_bw + new_bw;
}

static inline
void __add_running_bw(u64 dl_bw, struct dl_rq *dl_rq)
{
	u64 old = dl_rq->running_bw;

	lockdep_assert_rq_held(rq_of_dl_rq(dl_rq));
	dl_rq->running_bw += dl_bw;
	SCHED_WARN_ON(dl_rq->running_bw < old); /* overflow */
	SCHED_WARN_ON(dl_rq->running_bw > dl_rq->this_bw);
	/* kick cpufreq (see the comment in kernel/sched/sched.h). */
	cpufreq_update_util(rq_of_dl_rq(dl_rq), 0);
}

static inline
void __sub_running_bw(u64 dl_bw, struct dl_rq *dl_rq)
{
	u64 old = dl_rq->running_bw;

	lockdep_assert_rq_held(rq_of_dl_rq(dl_rq));
	dl_rq->running_bw -= dl_bw;
	if (SCHED_WARN_ON(dl_rq->running_bw > old)) /* underflow */
		dl_rq->running_bw = 0;
	/* kick cpufreq (see the comment in kernel/sched/sched.h). */
	cpufreq_update_util(rq_of_dl_rq(dl_rq), 0);
}

static inline
void __add_rq_bw(u64 dl_bw, struct dl_rq *dl_rq)
{
	u64 old = dl_rq->this_bw;

	lockdep_assert_rq_held(rq_of_dl_rq(dl_rq));
	dl_rq->this_bw += dl_bw;
	SCHED_WARN_ON(dl_rq->this_bw < old); /* overflow */
}

static inline
void __sub_rq_bw(u64 dl_bw, struct dl_rq *dl_rq)
{
	u64 old = dl_rq->this_bw;

	lockdep_assert_rq_held(rq_of_dl_rq(dl_rq));
	dl_rq->this_bw -= dl_bw;
	SCHED_WARN_ON(dl_rq->this_bw > old); /* underflow */
	if (dl_rq->this_bw > old)
		dl_rq->this_bw = 0;
	SCHED_WARN_ON(dl_rq->running_bw > dl_rq->this_bw);
}

static inline
void add_rq_bw(struct sched_dl_entity *dl_se, struct dl_rq *dl_rq)
{
	if (!dl_entity_is_special(dl_se))
		__add_rq_bw(dl_se->dl_bw, dl_rq);
}

static inline
void sub_rq_bw(struct sched_dl_entity *dl_se, struct dl_rq *dl_rq)
{
	if (!dl_entity_is_special(dl_se))
		__sub_rq_bw(dl_se->dl_bw, dl_rq);
}

static inline
void add_running_bw(struct sched_dl_entity *dl_se, struct dl_rq *dl_rq)
{
	if (!dl_entity_is_special(dl_se))
		__add_running_bw(dl_se->dl_bw, dl_rq);
}

static inline
void sub_running_bw(struct sched_dl_entity *dl_se, struct dl_rq *dl_rq)
{
	if (!dl_entity_is_special(dl_se))
		__sub_running_bw(dl_se->dl_bw, dl_rq);
}

static inline
void inc_dl_runnable(struct task_struct *p)
{
	int cpu;

	for_each_cpu(cpu, &p->cpus_mask)
		atomic_inc(&cpu_rq(cpu)->dl.dl_nr_runnable);
}

static inline
void dec_dl_runnable(struct task_struct *p)
{
	int cpu;

	for_each_cpu(cpu, &p->cpus_mask)
		atomic_dec(&cpu_rq(cpu)->dl.dl_nr_runnable);
}

static inline
void update_dl_runnable(struct task_struct *p, const struct cpumask *new_mask)
{
	cpumask_t cpus_xor;
	int cpu;

	cpumask_xor(&cpus_xor, &p->cpus_mask, new_mask);
	for_each_cpu(cpu, &cpus_xor) {
		if (cpumask_test_cpu(cpu, &p->cpus_mask))
			atomic_dec(&cpu_rq(cpu)->dl.dl_nr_runnable);
		else
			atomic_inc(&cpu_rq(cpu)->dl.dl_nr_runnable);
	}
}

static void dl_change_utilization(struct task_struct *p, u64 new_bw)
{
	struct rq *rq;

	BUG_ON(p->dl.flags & SCHED_FLAG_SUGOV);

	if (task_on_rq_queued(p))
		return;

	rq = task_rq(p);
	if (p->dl.dl_non_contending) {
		sub_running_bw(&p->dl, &rq->dl);
		p->dl.dl_non_contending = 0;
		/*
		 * If the timer handler is currently running and the
		 * timer cannot be canceled, inactive_task_timer()
		 * will see that dl_not_contending is not set, and
		 * will not touch the rq's active utilization,
		 * so we are still safe.
		 */
		if (hrtimer_try_to_cancel(&p->dl.inactive_timer) == 1)
			put_task_struct(p);
	}
	__sub_rq_bw(p->dl.dl_bw, &rq->dl);
	__add_rq_bw(new_bw, &rq->dl);
}

/*
 * The utilization of a task cannot be immediately removed from
 * the rq active utilization (running_bw) when the task blocks.
 * Instead, we have to wait for the so called "0-lag time".
 *
 * If a task blocks before the "0-lag time", a timer (the inactive
 * timer) is armed, and running_bw is decreased when the timer
 * fires.
 *
 * If the task wakes up again before the inactive timer fires,
 * the timer is canceled, whereas if the task wakes up after the
 * inactive timer fired (and running_bw has been decreased) the
 * task's utilization has to be added to running_bw again.
 * A flag in the deadline scheduling entity (dl_non_contending)
 * is used to avoid race conditions between the inactive timer handler
 * and task wakeups.
 *
 * The following diagram shows how running_bw is updated. A task is
 * "ACTIVE" when its utilization contributes to running_bw; an
 * "ACTIVE contending" task is in the TASK_RUNNING state, while an
 * "ACTIVE non contending" task is a blocked task for which the "0-lag time"
 * has not passed yet. An "INACTIVE" task is a task for which the "0-lag"
 * time already passed, which does not contribute to running_bw anymore.
 *                              +------------------+
 *             wakeup           |    ACTIVE        |
 *          +------------------>+   contending     |
 *          | add_running_bw    |                  |
 *          |                   +----+------+------+
 *          |                        |      ^
 *          |                dequeue |      |
 * +--------+-------+                |      |
 * |                |   t >= 0-lag   |      | wakeup
 * |    INACTIVE    |<---------------+      |
 * |                | sub_running_bw |      |
 * +--------+-------+                |      |
 *          ^                        |      |
 *          |              t < 0-lag |      |
 *          |                        |      |
 *          |                        V      |
 *          |                   +----+------+------+
 *          | sub_running_bw    |    ACTIVE        |
 *          +-------------------+                  |
 *            inactive timer    |  non contending  |
 *            fired             +------------------+
 *
 * The task_non_contending() function is invoked when a task
 * blocks, and checks if the 0-lag time already passed or
 * not (in the first case, it directly updates running_bw;
 * in the second case, it arms the inactive timer).
 *
 * The task_contending() function is invoked when a task wakes
 * up, and checks if the task is still in the "ACTIVE non contending"
 * state or not (in the second case, it updates running_bw).
 */
static void task_non_contending(struct task_struct *p)
{
	struct sched_dl_entity *dl_se = &p->dl;
	struct hrtimer *timer = &dl_se->inactive_timer;
	struct dl_rq *dl_rq = dl_rq_of_se(dl_se);
	struct rq *rq = rq_of_dl_rq(dl_rq);
	s64 zerolag_time;

	/*
	 * If this is a non-deadline task that has been boosted,
	 * do nothing
	 */
	if (dl_se->dl_runtime == 0)
		return;

	if (dl_entity_is_special(dl_se))
		return;

	WARN_ON(dl_se->dl_non_contending);

	zerolag_time = dl_se->deadline -
		 div64_long((dl_se->runtime * dl_se->dl_period),
			dl_se->dl_runtime);

	/*
	 * Using relative times instead of the absolute "0-lag time"
	 * allows to simplify the code
	 */
	zerolag_time -= rq_clock(rq);

	/*
	 * If the "0-lag time" already passed, decrease the active
	 * utilization now, instead of starting a timer
	 */
	if ((zerolag_time < 0) || hrtimer_active(&dl_se->inactive_timer)) {
		if (dl_task(p))
			sub_running_bw(dl_se, dl_rq);

		if (!dl_task(p) || READ_ONCE(p->__state) == TASK_DEAD) {
			struct dl_bw *dl_b = dl_bw_of(task_cpu(p));

			if (READ_ONCE(p->__state) == TASK_DEAD)
				sub_rq_bw(&p->dl, &rq->dl);
			raw_spin_lock(&dl_b->lock);
			__dl_sub(dl_b, p->dl.dl_bw, dl_bw_cpus(task_cpu(p)));
			__dl_clear_params(p);
			raw_spin_unlock(&dl_b->lock);
			dec_dl_runnable(p);
		}

		return;
	}

	dl_se->dl_non_contending = 1;
	get_task_struct(p);
	hrtimer_start(timer, ns_to_ktime(zerolag_time), HRTIMER_MODE_REL_HARD);
}

static void task_contending(struct sched_dl_entity *dl_se, int flags)
{
	struct dl_rq *dl_rq = dl_rq_of_se(dl_se);

	/*
	 * If this is a non-deadline task that has been boosted,
	 * do nothing
	 */
	if (dl_se->dl_runtime == 0)
		return;

	if (flags & ENQUEUE_MIGRATED)
		add_rq_bw(dl_se, dl_rq);

	if (dl_se->dl_non_contending) {
		dl_se->dl_non_contending = 0;
		/*
		 * If the timer handler is currently running and the
		 * timer cannot be canceled, inactive_task_timer()
		 * will see that dl_not_contending is not set, and
		 * will not touch the rq's active utilization,
		 * so we are still safe.
		 */
		if (hrtimer_try_to_cancel(&dl_se->inactive_timer) == 1)
			put_task_struct(dl_task_of(dl_se));
	} else {
		/*
		 * Since "dl_non_contending" is not set, the
		 * task's utilization has already been removed from
		 * active utilization (either when the task blocked,
		 * when the "inactive timer" fired).
		 * So, add it back.
		 */
		add_running_bw(dl_se, dl_rq);
	}
}

static void init_dl_rq_bw_ratio(struct dl_rq *dl_rq);

void init_dl_bandwidth(struct dl_bandwidth *dl_b, u64 period, u64 runtime)
{
	raw_spin_lock_init(&dl_b->dl_runtime_lock);
	dl_b->dl_period = period;
	dl_b->dl_runtime = runtime;
}

void init_dl_bw(struct dl_bw *dl_b)
{
	raw_spin_lock_init(&dl_b->lock);
	if (global_rt_runtime() == RUNTIME_INF)
		dl_b->bw = -1;
	else
		dl_b->bw = to_ratio(global_rt_period(), global_rt_runtime());
	dl_b->total_bw = 0;
}

void init_dl_rq(struct dl_rq *dl_rq)
{
#ifndef CONFIG_SMP
	init_dl_bw(&dl_rq->dl_bw);
#endif
	atomic_set(&dl_rq->dl_nr_runnable, 0);
	dl_rq->yi = -1;
	INIT_LIST_HEAD(&dl_rq->push_dl);
	INIT_LIST_HEAD(&dl_rq->latch_dl);

	dl_rq->running_bw = 0;
	dl_rq->this_bw = 0;
	init_dl_rq_bw_ratio(dl_rq);
}

static inline struct sched_statistics * __schedstats_from_dl_se(struct sched_dl_entity *dl_se);

static void push_dl_entry_add(struct rq *rq, struct task_struct *p)
{
	struct sched_dl_entity *dl_se = &p->dl;
	struct dl_rq *dl_rq = &rq->dl;

	lockdep_assert_rq_held(rq);

	if (!list_empty(&dl_se->push_dl_entry))
		/*
		 * No matter whether 'rq' is different from the source rq of the
		 * ongoing pushing operation, just let it proceed.
		 */
		return;

	dl_se->push_dl_rq = rq;
	list_add_tail(&dl_se->push_dl_entry, &dl_rq->push_dl);
}

static void push_dl_entry_del(struct rq *rq, struct task_struct *p)
{
	struct sched_dl_entity *dl_se = &p->dl;

	lockdep_assert_rq_held(rq);

	if (list_empty(&dl_se->push_dl_entry))
		return;

	if (SCHED_WARN_ON(dl_se->push_dl_rq != rq))
		return;

	list_del_init(&dl_se->push_dl_entry);
	dl_se->push_dl_rq = NULL;
}

static void latch_dl_entry_add(struct rq *rq, struct task_struct *p)
{
	struct sched_dl_entity *dl_se = &p->dl;
	struct dl_rq *dl_rq = &rq->dl;

	lockdep_assert_rq_held(rq);

	if (!list_empty(&dl_se->latch_dl_entry))
		/*
		 * It's already latched so we don't need to perturb it.
		 */
		return;

	dl_se->latch_dl_rq = rq;
	list_add_tail(&dl_se->latch_dl_entry, &dl_rq->latch_dl);
}

static void latch_dl_entry_del(struct rq *rq, struct task_struct *p)
{
	struct sched_dl_entity *dl_se = &p->dl;

	lockdep_assert_rq_held(rq);

	if (SCHED_WARN_ON(dl_se->latch_dl_rq && dl_se->latch_dl_rq != rq))
		return;

	list_del_init(&dl_se->latch_dl_entry);
	dl_se->latch_dl_rq = NULL;
}

#ifdef CONFIG_SMP

static inline int dl_overloaded(struct rq *rq)
{
	return atomic_read(&rq->rd->dlo_count);
}

static inline void dl_set_overload(struct rq *rq)
{
	if (!rq->online)
		return;

	cpumask_set_cpu(rq->cpu, rq->rd->dlo_mask);
	/*
	 * Must be visible before the overload count is
	 * set (as in sched_rt.c).
	 *
	 * Matched by the barrier in pull_dl_task().
	 */
	smp_wmb();
	atomic_inc(&rq->rd->dlo_count);
}

static inline void dl_clear_overload(struct rq *rq)
{
	if (!rq->online)
		return;

	atomic_dec(&rq->rd->dlo_count);
	cpumask_clear_cpu(rq->cpu, rq->rd->dlo_mask);
}

static inline bool need_pull_dl_task(struct rq *rq, struct task_struct *prev)
{
	return rq->online && dl_task(prev);
}

static DEFINE_PER_CPU(struct callback_head, dl_push_head);
static DEFINE_PER_CPU(struct callback_head, dl_pull_head);

static void push_dl_task(struct rq *rq, struct task_struct *p);
static void push_dl_tasks(struct rq *);
static void pull_dl_task(struct rq *);

static inline void deadline_queue_push_tasks(struct rq *rq)
{
	queue_balance_callback(rq, &per_cpu(dl_push_head, rq->cpu), push_dl_tasks);
}

static inline void deadline_queue_pull_task(struct rq *rq)
{
	queue_balance_callback(rq, &per_cpu(dl_pull_head, rq->cpu), pull_dl_task);
}

static struct rq *find_lock_rq(struct task_struct *task, struct rq *rq);

static struct rq *dl_task_offline_migration(struct rq *rq, struct task_struct *p)
{
	struct rq *later_rq = NULL;
	struct dl_bw *dl_b;

	later_rq = find_lock_rq(p, rq);
	if (!later_rq) {
		int cpu;

		/*
		 * If we cannot preempt any rq, fall back to pick any
		 * online CPU:
		 */
		cpu = cpumask_any_and(cpu_active_mask, p->cpus_ptr);
		if (cpu >= nr_cpu_ids) {
			/*
			 * Try a little harder to let the task run.
			 */
			cpu = cpumask_any(cpu_active_mask);
		}
		later_rq = cpu_rq(cpu);
		double_lock_balance(rq, later_rq);
	}

	if (p->dl.dl_non_contending || p->dl.dl_throttled) {
		/*
		 * Inactive timer is armed (or callback is running, but
		 * waiting for us to release rq locks). In any case, when it
		 * will fire (or continue), it will see running_bw of this
		 * task migrated to later_rq (and correctly handle it).
		 */
		sub_running_bw(&p->dl, &rq->dl);
		sub_rq_bw(&p->dl, &rq->dl);

		add_rq_bw(&p->dl, &later_rq->dl);
		add_running_bw(&p->dl, &later_rq->dl);
	} else {
		sub_rq_bw(&p->dl, &rq->dl);
		add_rq_bw(&p->dl, &later_rq->dl);
	}

	/*
	 * And we finally need to fixup root_domain(s) bandwidth accounting,
	 * since p is still hanging out in the old (now moved to default) root
	 * domain.
	 */
	dl_b = &rq->rd->dl_bw;
	raw_spin_lock(&dl_b->lock);
	__dl_sub(dl_b, p->dl.dl_bw, cpumask_weight(rq->rd->span));
	raw_spin_unlock(&dl_b->lock);

	dl_b = &later_rq->rd->dl_bw;
	raw_spin_lock(&dl_b->lock);
	__dl_add(dl_b, p->dl.dl_bw, cpumask_weight(later_rq->rd->span));
	raw_spin_unlock(&dl_b->lock);

	set_task_cpu(p, later_rq->cpu);
	double_unlock_balance(later_rq, rq);

	return later_rq;
}

#else

static inline void deadline_queue_push_tasks(struct rq *rq)
{
}

static inline void deadline_queue_pull_task(struct rq *rq)
{
}
#endif /* CONFIG_SMP */

static void enqueue_task_dl(struct rq *rq, struct task_struct *p, int flags);
static void __dequeue_task_dl(struct rq *rq, struct task_struct *p, int flags);
static void check_preempt_curr_dl(struct rq *rq, struct task_struct *p, int flags);
static int restart_resched_timer(struct task_struct *p);

/*
 * We are being explicitly informed that a new instance is starting,
 * and this means that:
 *  - the absolute deadline of the entity has to be placed at
 *    current time + relative deadline;
 *  - the runtime of the entity has to be set to the maximum value.
 *
 * The capability of specifying such event is useful whenever a -deadline
 * entity wants to (try to!) synchronize its behaviour with the scheduler's
 * one, and to (try to!) reconcile itself with its own scheduling
 * parameters.
 */
static inline void setup_new_dl_entity(struct sched_dl_entity *dl_se)
{
	struct dl_rq *dl_rq = dl_rq_of_se(dl_se);
	struct rq *rq = rq_of_dl_rq(dl_rq);

	WARN_ON(is_dl_boosted(dl_se));
	WARN_ON(dl_time_before(rq_clock(rq), dl_se->deadline));

	/*
	 * We are racing with the deadline timer. So, do nothing because
	 * the deadline timer handler will take care of properly recharging
	 * the runtime and postponing the deadline
	 */
	if (dl_se->dl_throttled)
		return;

	/*
	 * We use the regular wall clock time to set deadlines in the
	 * future; in fact, we must consider execution overheads (time
	 * spent on hardirq context, etc.).
	 */
	dl_se->deadline = rq_clock(rq) + dl_se->dl_deadline;
	dl_se->runtime = dl_se->dl_runtime;
}

static inline u64 dl_next_period(struct sched_dl_entity *dl_se)
{
	return dl_se->deadline - dl_se->dl_deadline + dl_se->dl_period;
}

/*
 * Pure Earliest Deadline First (EDF) scheduling does not deal with the
 * possibility of a entity lasting more than what it declared, and thus
 * exhausting its runtime.
 *
 * Here we are interested in making runtime overrun possible, but we do
 * not want a entity which is misbehaving to affect the scheduling of all
 * other entities.
 * Therefore, a budgeting strategy called Constant Bandwidth Server (CBS)
 * is used, in order to confine each entity within its own bandwidth.
 *
 * This function deals exactly with that, and ensures that when the runtime
 * of a entity is replenished, its deadline is also postponed. That ensures
 * the overrunning entity can't interfere with other entity in the system and
 * can't make them miss their deadlines. Reasons why this kind of overruns
 * could happen are, typically, a entity voluntarily trying to overcome its
 * runtime, or it just underestimated it during sched_setattr().
 */
static void replenish_dl_entity(struct sched_dl_entity *dl_se)
{
	struct dl_rq *dl_rq = dl_rq_of_se(dl_se);
	struct rq *rq = rq_of_dl_rq(dl_rq);

	BUG_ON(pi_of(dl_se)->dl_runtime <= 0);

	/*
	 * This could be the case for a !-dl task that is boosted.
	 * Just go with full inherited parameters.
	 */
	if (dl_se->dl_deadline == 0) {
		dl_se->deadline = rq_clock(rq) + pi_of(dl_se)->dl_deadline;
		dl_se->runtime = pi_of(dl_se)->dl_runtime;
	}

	if (dl_se->dl_yielded && dl_se->runtime > 0)
		dl_se->runtime = 0;

	/*
	 * We keep moving the deadline away until we get some
	 * available runtime for the entity. This ensures correct
	 * handling of situations where the runtime overrun is
	 * arbitrary large.
	 */
	while (dl_se->runtime <= 0) {
		dl_se->deadline += pi_of(dl_se)->dl_period;
		dl_se->runtime += pi_of(dl_se)->dl_runtime;
	}

	/*
	 * At this point, the deadline really should be "in
	 * the future" with respect to rq->clock. If it's
	 * not, we are, for some reason, lagging too much!
	 * Anyway, after having warn userspace abut that,
	 * we still try to keep the things running by
	 * resetting the deadline and the budget of the
	 * entity.
	 */
	if (dl_time_before(dl_se->deadline, rq_clock(rq))) {
		printk_deferred_once("sched: DL replenish lagged too much\n");

		dl_se->deadline = rq_clock(rq) + pi_of(dl_se)->dl_deadline;
		dl_se->runtime = pi_of(dl_se)->dl_runtime;
	}

	if (dl_se->dl_yielded)
		dl_se->dl_yielded = 0;
	if (dl_se->dl_throttled)
		dl_se->dl_throttled = 0;
}

/*
 * Here we check if --at time t-- an entity (which is probably being
 * [re]activated or, in general, enqueued) can use its remaining runtime
 * and its current deadline _without_ exceeding the bandwidth it is
 * assigned (function returns true if it can't). We are in fact applying
 * one of the CBS rules: when a task wakes up, if the residual runtime
 * over residual deadline fits within the allocated bandwidth, then we
 * can keep the current (absolute) deadline and residual budget without
 * disrupting the schedulability of the system. Otherwise, we should
 * refill the runtime and set the deadline a period in the future,
 * because keeping the current (absolute) deadline of the task would
 * result in breaking guarantees promised to other tasks (refer to
 * Documentation/scheduler/sched-deadline.rst for more information).
 *
 * This function returns true if:
 *
 *   runtime / (deadline - t) > dl_runtime / dl_deadline ,
 *
 * IOW we can't recycle current parameters.
 *
 * Notice that the bandwidth check is done against the deadline. For
 * task with deadline equal to period this is the same of using
 * dl_period instead of dl_deadline in the equation above.
 */
static bool dl_entity_overflow(struct sched_dl_entity *dl_se, u64 t)
{
	u64 left, right;

	/*
	 * left and right are the two sides of the equation above,
	 * after a bit of shuffling to use multiplications instead
	 * of divisions.
	 *
	 * Note that none of the time values involved in the two
	 * multiplications are absolute: dl_deadline and dl_runtime
	 * are the relative deadline and the maximum runtime of each
	 * instance, runtime is the runtime left for the last instance
	 * and (deadline - t), since t is rq->clock, is the time left
	 * to the (absolute) deadline. Even if overflowing the u64 type
	 * is very unlikely to occur in both cases, here we scale down
	 * as we want to avoid that risk at all. Scaling down by 10
	 * means that we reduce granularity to 1us. We are fine with it,
	 * since this is only a true/false check and, anyway, thinking
	 * of anything below microseconds resolution is actually fiction
	 * (but still we want to give the user that illusion >;).
	 */
	left = (pi_of(dl_se)->dl_deadline >> DL_SCALE) * (dl_se->runtime >> DL_SCALE);
	right = ((dl_se->deadline - t) >> DL_SCALE) *
		(pi_of(dl_se)->dl_runtime >> DL_SCALE);

	return dl_time_before(right, left);
}

/*
 * Revised wakeup rule [1]: For self-suspending tasks, rather then
 * re-initializing task's runtime and deadline, the revised wakeup
 * rule adjusts the task's runtime to avoid the task to overrun its
 * density.
 *
 * Reasoning: a task may overrun the density if:
 *    runtime / (deadline - t) > dl_runtime / dl_deadline
 *
 * Therefore, runtime can be adjusted to:
 *     runtime = (dl_runtime / dl_deadline) * (deadline - t)
 *
 * In such way that runtime will be equal to the maximum density
 * the task can use without breaking any rule.
 *
 * [1] Luca Abeni, Giuseppe Lipari, and Juri Lelli. 2015. Constant
 * bandwidth server revisited. SIGBED Rev. 11, 4 (January 2015), 19-24.
 */
static void
update_dl_revised_wakeup(struct sched_dl_entity *dl_se, struct rq *rq)
{
	u64 laxity = dl_se->deadline - rq_clock(rq);

	/*
	 * If the task has deadline < period, and the deadline is in the past,
	 * it should already be throttled before this check.
	 *
	 * See update_dl_entity() comments for further details.
	 */
	WARN_ON(dl_time_before(dl_se->deadline, rq_clock(rq)));

	dl_se->runtime = (dl_se->dl_density * laxity) >> BW_SHIFT;
}

/*
 * Regarding the deadline, a task with implicit deadline has a relative
 * deadline == relative period. A task with constrained deadline has a
 * relative deadline <= relative period.
 *
 * We support constrained deadline tasks. However, there are some restrictions
 * applied only for tasks which do not have an implicit deadline. See
 * update_dl_entity() to know more about such restrictions.
 *
 * The dl_is_implicit() returns true if the task has an implicit deadline.
 */
static inline bool dl_is_implicit(struct sched_dl_entity *dl_se)
{
	return dl_se->dl_deadline == dl_se->dl_period;
}

/*
 * When a deadline entity is placed in the runqueue, its runtime and deadline
 * might need to be updated. This is done by a CBS wake up rule. There are two
 * different rules: 1) the original CBS; and 2) the Revisited CBS.
 *
 * When the task is starting a new period, the Original CBS is used. In this
 * case, the runtime is replenished and a new absolute deadline is set.
 *
 * When a task is queued before the begin of the next period, using the
 * remaining runtime and deadline could make the entity to overflow, see
 * dl_entity_overflow() to find more about runtime overflow. When such case
 * is detected, the runtime and deadline need to be updated.
 *
 * If the task has an implicit deadline, i.e., deadline == period, the Original
 * CBS is applied. the runtime is replenished and a new absolute deadline is
 * set, as in the previous cases.
 *
 * However, the Original CBS does not work properly for tasks with
 * deadline < period, which are said to have a constrained deadline. By
 * applying the Original CBS, a constrained deadline task would be able to run
 * runtime/deadline in a period. With deadline < period, the task would
 * overrun the runtime/period allowed bandwidth, breaking the admission test.
 *
 * In order to prevent this misbehave, the Revisited CBS is used for
 * constrained deadline tasks when a runtime overflow is detected. In the
 * Revisited CBS, rather than replenishing & setting a new absolute deadline,
 * the remaining runtime of the task is reduced to avoid runtime overflow.
 * Please refer to the comments update_dl_revised_wakeup() function to find
 * more about the Revised CBS rule.
 */
static void update_dl_entity(struct sched_dl_entity *dl_se)
{
	struct dl_rq *dl_rq = dl_rq_of_se(dl_se);
	struct rq *rq = rq_of_dl_rq(dl_rq);

	if (dl_time_before(dl_se->deadline, rq_clock(rq)) ||
	    dl_entity_overflow(dl_se, rq_clock(rq))) {

		if (unlikely(!dl_is_implicit(dl_se) &&
			     !dl_time_before(dl_se->deadline, rq_clock(rq)) &&
			     !is_dl_boosted(dl_se))) {
			update_dl_revised_wakeup(dl_se, rq);
			return;
		}

		dl_se->deadline = rq_clock(rq) + pi_of(dl_se)->dl_deadline;
		dl_se->runtime = pi_of(dl_se)->dl_runtime;
	}
}

/*
 * If the entity depleted all its runtime, and if we want it to sleep
 * while waiting for some new execution time to become available, we
 * set the bandwidth replenishment timer to the replenishment instant
 * and try to activate it.
 *
 * Notice that it is important for the caller to know if the timer
 * actually started or not (i.e., the replenishment instant is in
 * the future or in the past).
 */
static int start_dl_timer(struct task_struct *p)
{
	struct sched_dl_entity *dl_se = &p->dl;
	struct hrtimer *timer = &dl_se->dl_timer;
	struct rq *rq = task_rq(p);
	ktime_t now, act;
	s64 delta;

	lockdep_assert_rq_held(rq);

	/*
	 * We want the timer to fire at the deadline, but considering
	 * that it is actually coming from rq->clock and not from
	 * hrtimer's time base reading.
	 */
	act = ns_to_ktime(dl_next_period(dl_se));
	now = hrtimer_cb_get_time(timer);
	delta = ktime_to_ns(now) - rq_clock(rq);
	act = ktime_add_ns(act, delta);

	/*
	 * If the expiry time already passed, e.g., because the value
	 * chosen as the deadline is too small, don't even try to
	 * start the timer in the past!
	 */
	if (ktime_us_delta(act, now) < 0)
		return 0;

	/*
	 * Resched timer is unnecessary as the task will be enqueued
	 * at the next pseudo-release time.
	 */
	if (hrtimer_try_to_cancel(&p->dl.resched_timer) == 1)
		put_task_struct(p);

	/*
	 * !enqueued will guarantee another callback; even if one is already in
	 * progress. This ensures a balanced {get,put}_task_struct().
	 *
	 * The race against __run_timer() clearing the enqueued state is
	 * harmless because we're holding task_rq()->lock, therefore the timer
	 * expiring after we've done the check will wait on its task_rq_lock()
	 * and observe our state.
	 */
	if (!hrtimer_is_queued(timer)) {
		get_task_struct(p);
		hrtimer_start(timer, act, HRTIMER_MODE_ABS_HARD);
	}

	return 1;
}

/*
 * This is the bandwidth enforcement timer callback. If here, we know
 * a task is not on its dl_rq, since the fact that the timer was running
 * means the task is throttled and needs a runtime replenishment.
 *
 * However, what we actually do depends on the fact the task is active,
 * (it is on its rq) or has been removed from there by a call to
 * dequeue_task_dl(). In the former case we must issue the runtime
 * replenishment and add the task back to the dl_rq; in the latter, we just
 * do nothing but clearing dl_throttled, so that runtime and deadline
 * updating (and the queueing back to dl_rq) will be done by the
 * next call to enqueue_task_dl().
 */
static enum hrtimer_restart dl_task_timer(struct hrtimer *timer)
{
	struct sched_dl_entity *dl_se = container_of(timer,
						     struct sched_dl_entity,
						     dl_timer);
	struct task_struct *p = dl_task_of(dl_se);
	struct rq_flags rf;
	struct rq *rq;

	rq = task_rq_lock(p, &rf);

	/*
	 * The task might have changed its scheduling policy to something
	 * different than SCHED_DEADLINE (through switched_from_dl()).
	 */
	if (!dl_task(p))
		goto unlock;

	/*
	 * The task might have been boosted by someone else and might be in the
	 * boosting/deboosting path, its not throttled.
	 */
	if (is_dl_boosted(dl_se))
		goto unlock;

	/*
	 * Spurious timer due to start_dl_timer() race; or we already received
	 * a replenishment from rt_mutex_setprio().
	 */
	if (!dl_se->dl_throttled)
		goto unlock;

	sched_clock_tick();
	update_rq_clock(rq);

	/*
	 * If the throttle happened during sched-out; like:
	 *
	 *   schedule()
	 *     deactivate_task()
	 *       dequeue_task_dl()
	 *         update_curr_dl()
	 *           start_dl_timer()
	 *         __dequeue_task_dl()
	 *     prev->on_rq = 0;
	 *
	 * We can be both throttled and !queued. Replenish the counter
	 * but do not enqueue -- wait for our wakeup to do that.
	 */
	if (!task_on_rq_queued(p)) {
		replenish_dl_entity(dl_se);
		goto unlock;
	}

#ifdef CONFIG_SMP
	if (unlikely(!rq->online)) {
		/*
		 * If the runqueue is no longer available, migrate the
		 * task elsewhere. This necessarily changes rq.
		 */
		lockdep_unpin_lock(__rq_lockp(rq), rf.cookie);
		rq = dl_task_offline_migration(rq, p);
		rf.cookie = lockdep_pin_lock(__rq_lockp(rq));
		update_rq_clock(rq);

		/*
		 * Now that the task has been migrated to the new RQ and we
		 * have that locked, proceed as normal and enqueue the task
		 * there.
		 */
	}
#endif

	enqueue_task_dl(rq, p, ENQUEUE_REPLENISH);
	/*
	 * The task is latched and needs to be picked or pushed.
	 * TODO: optimize and directly push it away right now if any
	 *       as we're holding the rq lock.
	 */
	resched_curr(rq);

unlock:
	task_rq_unlock(rq, p, &rf);

	/*
	 * This can free the task_struct, including this hrtimer, do not touch
	 * anything related to that after this.
	 */
	put_task_struct(p);

	return HRTIMER_NORESTART;
}

void init_dl_task_timer(struct sched_dl_entity *dl_se)
{
	struct hrtimer *timer = &dl_se->dl_timer;

	hrtimer_init(timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL_HARD);
	timer->function = dl_task_timer;
}

/*
 * During the activation, CBS checks if it can reuse the current task's
 * runtime and period. If the deadline of the task is in the past, CBS
 * cannot use the runtime, and so it replenishes the task. This rule
 * works fine for implicit deadline tasks (deadline == period), and the
 * CBS was designed for implicit deadline tasks. However, a task with
 * constrained deadline (deadline < period) might be awakened after the
 * deadline, but before the next period. In this case, replenishing the
 * task would allow it to run for runtime / deadline. As in this case
 * deadline < period, CBS enables a task to run for more than the
 * runtime / period. In a very loaded system, this can cause a domino
 * effect, making other tasks miss their deadlines.
 *
 * To avoid this problem, in the activation of a constrained deadline
 * task after the deadline but before the next period, throttle the
 * task and set the replenishing timer to the begin of the next period,
 * unless it is boosted.
 */
static inline void dl_check_constrained_dl(struct sched_dl_entity *dl_se)
{
	struct task_struct *p = dl_task_of(dl_se);
	struct rq *rq = rq_of_dl_rq(dl_rq_of_se(dl_se));

	if (dl_time_before(dl_se->deadline, rq_clock(rq)) &&
	    dl_time_before(rq_clock(rq), dl_next_period(dl_se))) {
		if (unlikely(is_dl_boosted(dl_se) || !start_dl_timer(p)))
			return;
		dl_se->dl_throttled = 1;
		if (dl_se->runtime > 0)
			dl_se->runtime = 0;
	}
}

static
int dl_runtime_exceeded(struct sched_dl_entity *dl_se)
{
	return (dl_se->runtime <= 0);
}

/*
 * This function implements the GRUB accounting rule:
 * according to the GRUB reclaiming algorithm, the runtime is
 * not decreased as "dq = -dt", but as
 * "dq = -max{u / Umax, (1 - Uinact - Uextra)} dt",
 * where u is the utilization of the task, Umax is the maximum reclaimable
 * utilization, Uinact is the (per-runqueue) inactive utilization, computed
 * as the difference between the "total runqueue utilization" and the
 * runqueue active utilization, and Uextra is the (per runqueue) extra
 * reclaimable utilization.
 * Since rq->dl.running_bw and rq->dl.this_bw contain utilizations
 * multiplied by 2^BW_SHIFT, the result has to be shifted right by
 * BW_SHIFT.
 * Since rq->dl.bw_ratio contains 1 / Umax multiplied by 2^RATIO_SHIFT,
 * dl_bw is multiped by rq->dl.bw_ratio and shifted right by RATIO_SHIFT.
 * Since delta is a 64 bit variable, to have an overflow its value
 * should be larger than 2^(64 - 20 - 8), which is more than 64 seconds.
 * So, overflow is not an issue here.
 */
static u64 grub_reclaim(u64 delta, struct rq *rq, struct sched_dl_entity *dl_se)
{
	u64 u_inact = rq->dl.this_bw - rq->dl.running_bw; /* Utot - Uact */
	u64 u_act;
	u64 u_act_min = (dl_se->dl_bw * rq->dl.bw_ratio) >> RATIO_SHIFT;

	/*
	 * Instead of computing max{u * bw_ratio, (1 - u_inact - u_extra)},
	 * we compare u_inact + rq->dl.extra_bw with
	 * 1 - (u * rq->dl.bw_ratio >> RATIO_SHIFT), because
	 * u_inact + rq->dl.extra_bw can be larger than
	 * 1 * (so, 1 - u_inact - rq->dl.extra_bw would be negative
	 * leading to wrong results)
	 */
	if (u_inact + rq->dl.extra_bw > BW_UNIT - u_act_min)
		u_act = u_act_min;
	else
		u_act = BW_UNIT - u_inact - rq->dl.extra_bw;

	return (delta * u_act) >> BW_SHIFT;
}

/*
 * Update the current task's runtime statistics (provided it is still
 * a -deadline task and has not been removed from the dl_rq).
 */
static void update_curr_dl(struct rq *rq)
{
	struct task_struct *curr = rq->curr;
	struct sched_dl_entity *dl_se = &curr->dl;
	u64 delta_exec, scaled_delta_exec;
	int cpu = cpu_of(rq);
	u64 now;

	if (!dl_task(curr) || !on_dl_rq(dl_se))
		return;

	/*
	 * Consumed budget is computed considering the time as
	 * observed by schedulable tasks (excluding time spent
	 * in hardirq context, etc.). Deadlines are instead
	 * computed using hard walltime. This seems to be the more
	 * natural solution, but the full ramifications of this
	 * approach need further study.
	 */
	now = rq_clock_task(rq);
	delta_exec = now - curr->se.exec_start;
	if (unlikely((s64)delta_exec <= 0)) {
		if (unlikely(dl_se->dl_yielded))
			goto throttle;
		return;
	}

	schedstat_set(curr->stats.exec_max,
		      max(curr->stats.exec_max, delta_exec));

	curr->se.sum_exec_runtime += delta_exec;
	account_group_exec_runtime(curr, delta_exec);

	curr->se.exec_start = now;
	cgroup_account_cputime(curr, delta_exec);

	if (dl_entity_is_special(dl_se)) {
		trace_sched_stat_runtime(curr, delta_exec, 0);
		return;
	}

	/*
	 * For tasks that participate in GRUB, we implement GRUB-PA: the
	 * spare reclaimed bandwidth is used to clock down frequency.
	 *
	 * For the others, we still need to scale reservation parameters
	 * according to current frequency and CPU maximum capacity.
	 */
	if (unlikely(dl_se->flags & SCHED_FLAG_RECLAIM)) {
		scaled_delta_exec = grub_reclaim(delta_exec,
						 rq,
						 &curr->dl);
	} else {
		unsigned long scale_freq = arch_scale_freq_capacity(cpu);
		unsigned long scale_cpu = arch_scale_cpu_capacity(cpu);

		scaled_delta_exec = cap_scale(delta_exec, scale_freq);
		scaled_delta_exec = cap_scale(scaled_delta_exec, scale_cpu);
	}

	dl_se->runtime -= scaled_delta_exec;

	/*
	 * Overload CFS vruntime field as we want to check scaled delta_exec.
	 * TODO: refactor
	 */
	trace_sched_stat_runtime(curr, delta_exec, scaled_delta_exec);

throttle:
	if (dl_runtime_exceeded(dl_se) || dl_se->dl_yielded) {
		dl_se->dl_throttled = 1;

		/* If requested, inform the user about runtime overruns. */
		if (dl_runtime_exceeded(dl_se) &&
		    (dl_se->flags & SCHED_FLAG_DL_OVERRUN))
			dl_se->dl_overrun = 1;

		__dequeue_task_dl(rq, curr, 0);
		if (unlikely(is_dl_boosted(dl_se) || !start_dl_timer(curr)))
			enqueue_task_dl(rq, curr, ENQUEUE_REPLENISH);

		resched_curr(rq);
	}

	/*
	 * Because -- for now -- we share the rt bandwidth, we need to
	 * account our runtime there too, otherwise actual rt tasks
	 * would be able to exceed the shared quota.
	 *
	 * Account to the root rt group for now.
	 *
	 * The solution we're working towards is having the RT groups scheduled
	 * using deadline servers -- however there's a few nasties to figure
	 * out before that can happen.
	 */
	if (rt_bandwidth_enabled()) {
		struct rt_rq *rt_rq = &rq->rt;

		raw_spin_lock(&rt_rq->rt_runtime_lock);
		/*
		 * We'll let actual RT tasks worry about the overflow here, we
		 * have our own CBS to keep us inline; only account when RT
		 * bandwidth is relevant.
		 */
		if (sched_rt_bandwidth_account(rt_rq))
			rt_rq->rt_time += delta_exec;
		raw_spin_unlock(&rt_rq->rt_runtime_lock);
	}
}

static enum hrtimer_restart inactive_task_timer(struct hrtimer *timer)
{
	struct sched_dl_entity *dl_se = container_of(timer,
						     struct sched_dl_entity,
						     inactive_timer);
	struct task_struct *p = dl_task_of(dl_se);
	struct rq_flags rf;
	struct rq *rq;

	rq = task_rq_lock(p, &rf);

	sched_clock_tick();
	update_rq_clock(rq);

	if (!dl_task(p) || READ_ONCE(p->__state) == TASK_DEAD) {
		struct dl_bw *dl_b = dl_bw_of(task_cpu(p));

		if (READ_ONCE(p->__state) == TASK_DEAD && dl_se->dl_non_contending) {
			sub_running_bw(&p->dl, dl_rq_of_se(&p->dl));
			sub_rq_bw(&p->dl, dl_rq_of_se(&p->dl));
			dl_se->dl_non_contending = 0;
		}

		raw_spin_lock(&dl_b->lock);
		__dl_sub(dl_b, p->dl.dl_bw, dl_bw_cpus(task_cpu(p)));
		raw_spin_unlock(&dl_b->lock);
		__dl_clear_params(p);
		dec_dl_runnable(p);

		goto unlock;
	}
	if (dl_se->dl_non_contending == 0)
		goto unlock;

	sub_running_bw(dl_se, &rq->dl);
	dl_se->dl_non_contending = 0;
unlock:
	task_rq_unlock(rq, p, &rf);
	put_task_struct(p);

	return HRTIMER_NORESTART;
}

void init_dl_inactive_task_timer(struct sched_dl_entity *dl_se)
{
	struct hrtimer *timer = &dl_se->inactive_timer;

	hrtimer_init(timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL_HARD);
	timer->function = inactive_task_timer;
}

#ifdef CONFIG_SMP

static void inc_dl_deadline(struct dl_rq *dl_rq, u64 deadline)
{
	struct rq *rq = rq_of_dl_rq(dl_rq);

	cpupri_set(&rq->rd->cpupri, rq->cpu, CPUPRI_HIGHER);
}

static void dec_dl_deadline(struct dl_rq *dl_rq, u64 deadline)
{
	struct rq *rq = rq_of_dl_rq(dl_rq);

	cpupri_set(&rq->rd->cpupri, rq->cpu, rq->rt.highest_prio.curr);
}

#else

static inline void inc_dl_deadline(struct dl_rq *dl_rq, u64 deadline) {}
static inline void dec_dl_deadline(struct dl_rq *dl_rq, u64 deadline) {}

#endif /* CONFIG_SMP */

static inline
void inc_dl_tasks(struct sched_dl_entity *dl_se, struct dl_rq *dl_rq)
{
	int prio = dl_task_of(dl_se)->prio;
	u64 deadline = dl_se->deadline;

	WARN_ON(!dl_prio(prio));
	add_nr_running(rq_of_dl_rq(dl_rq), 1);

	inc_dl_deadline(dl_rq, deadline);
}

static inline
void dec_dl_tasks(struct sched_dl_entity *dl_se, struct dl_rq *dl_rq)
{
	int prio = dl_task_of(dl_se)->prio;

	WARN_ON(!dl_prio(prio));
	sub_nr_running(rq_of_dl_rq(dl_rq), 1);

	dec_dl_deadline(dl_rq, dl_se->deadline);
}

static inline struct sched_statistics *
__schedstats_from_dl_se(struct sched_dl_entity *dl_se)
{
	return &dl_task_of(dl_se)->stats;
}

static inline void
update_stats_wait_start_dl(struct dl_rq *dl_rq, struct sched_dl_entity *dl_se)
{
	struct sched_statistics *stats;

	if (!schedstat_enabled())
		return;

	stats = __schedstats_from_dl_se(dl_se);
	__update_stats_wait_start(rq_of_dl_rq(dl_rq), dl_task_of(dl_se), stats);
}

static inline void
update_stats_wait_end_dl(struct dl_rq *dl_rq, struct sched_dl_entity *dl_se)
{
	struct sched_statistics *stats;

	if (!schedstat_enabled())
		return;

	stats = __schedstats_from_dl_se(dl_se);
	__update_stats_wait_end(rq_of_dl_rq(dl_rq), dl_task_of(dl_se), stats);
}

static inline void
update_stats_enqueue_sleeper_dl(struct dl_rq *dl_rq, struct sched_dl_entity *dl_se)
{
	struct sched_statistics *stats;

	if (!schedstat_enabled())
		return;

	stats = __schedstats_from_dl_se(dl_se);
	__update_stats_enqueue_sleeper(rq_of_dl_rq(dl_rq), dl_task_of(dl_se), stats);
}

static inline void
update_stats_enqueue_dl(struct dl_rq *dl_rq, struct sched_dl_entity *dl_se,
			int flags)
{
	if (!schedstat_enabled())
		return;

	if (flags & ENQUEUE_WAKEUP)
		update_stats_enqueue_sleeper_dl(dl_rq, dl_se);
}

static inline void
update_stats_dequeue_dl(struct dl_rq *dl_rq, struct sched_dl_entity *dl_se,
			int flags)
{
	struct task_struct *p = dl_task_of(dl_se);

	if (!schedstat_enabled())
		return;

	if ((flags & DEQUEUE_SLEEP)) {
		unsigned int state;

		state = READ_ONCE(p->__state);
		if (state & TASK_INTERRUPTIBLE)
			__schedstat_set(p->stats.sleep_start,
					rq_clock(rq_of_dl_rq(dl_rq)));

		if (state & TASK_UNINTERRUPTIBLE)
			__schedstat_set(p->stats.block_start,
					rq_clock(rq_of_dl_rq(dl_rq)));
	}
}

static inline u64 latest_pseudo_release(struct task_struct *p)
{
	u64 now, ret;
	struct sched_dl_entity *dl_se;

	dl_se = &p->dl;
	now = rq_clock(task_rq(p));
	if (dl_se->deadline > pi_of(dl_se)->dl_deadline) {
		ret = dl_se->deadline - pi_of(dl_se)->dl_deadline;
		if (unlikely(now - ret > pi_of(dl_se)->dl_period))
			ret += rounddown(now - ret,
					 pi_of(dl_se)->dl_period);
	} else
		ret = round_down(now, pi_of(dl_se)->dl_period);

	return ret;
}

static u64 latest_pseudo_deadline(struct task_struct *p)
{
	struct sched_dl_entity *dl_se;
	dl_se = &p->dl;

	return latest_pseudo_release(p) + pi_of(dl_se)->dl_period;
}

static inline u64 unr_edf_phi(struct task_struct *p)
{
	unsigned long phi_mode;
	u64 t_max, ret;

	if (p->dl.dl_throttled)
		return 0;

	SCHED_WARN_ON(p->dl.dl_yielded);
	SCHED_WARN_ON(p->dl.dl_non_contending);

	t_max = (u64)READ_ONCE(sysctl_sched_dl_period_max) * NSEC_PER_USEC;
	phi_mode = READ_ONCE(sysctl_sched_dl_phi_mode);

	/* See sched/sysctl.h for what each mode means */
	if (unlikely(phi_mode == 1))
		ret = t_max + latest_pseudo_release(p) - p->dl.deadline;
	else if (unlikely(phi_mode == 2))
		ret = t_max + latest_pseudo_release(p) - p->dl.deadline + p->dl.runtime;
	else
		ret = t_max + latest_pseudo_deadline(p) - p->dl.deadline;

	return ret;
}

static void hu_update_weight(struct task_struct *p)
{
	struct rq *rq;
	int cpu;
	u64 phi;

	for_each_cpu(cpu, &p->cpus_mask) {
		rq = cpu_rq(cpu);
		phi = unr_edf_phi(p);
		phi = cap_scale(phi, arch_scale_cpu_capacity(cpu));
		phi = cap_scale(phi, arch_scale_freq_capacity(cpu));

		/*
		 * Add one to break a tie so that non-competing cpus
		 * are prioritized.
		 */
		if (atomic_read(&rq->dl.dl_nr_runnable) <= 1)
			phi++;

		p->dl.hm_weight[cpu] = phi;
	}
}

static void hu_enqueue_task(struct task_struct *p)
{
	struct rq *rq;
	int cpu, xi;

	if (SCHED_WARN_ON(p->dl.xi != -1))
		return;

	cpu = cpumask_first(&p->cpus_mask);
	if (SCHED_WARN_ON(cpu >= nr_cpu_ids))
		return;

	rq = cpu_rq(cpu);

	/* calculate weight */
	hu_update_weight(p);

	/* add a vertex */
	xi = hm_add_x(rq->rd->dl_hg, p->dl.hm_weight, NR_CPUS, p);
	if (xi >= 0) {
		p->dl.rd = rq->rd;
		p->dl.xi = xi;
	}
}

static void hu_dequeue_task(struct task_struct *p)
{
	struct sched_dl_entity *dl_se = &p->dl;

	if (SCHED_WARN_ON(!on_dl_rq(dl_se)))
		return;

	/*
	 * Note that dl_se->rd might be different from task_rq(p)->rd,
	 * e.g., migration_cpu_stop case. Just dequeue from the graph
	 * this task is being enqueued on.
	 */
	hm_del_x(dl_se->rd->dl_hg, dl_se->xi);
	dl_se->rd = NULL;
	dl_se->xi = -1;
}

static void hu_refresh_task(struct task_struct *p)
{
	struct sched_dl_entity *dl_se = &p->dl;

	if (!on_dl_rq(dl_se))
		return;

	/*
	 * Even though we try to cancel the resched_timer on .set_cpus_allowed(),
	 * there's still a chance that the callback has already started and
	 * before reaching here, migration has proceeded. In that case,
	 * we should not only update weights but also move onto the new graph.
	 * SCHED_WARN_ON(dl_se->rd != task_rq(p)->rd) does not suffice.
	 * TODO: implement
	 */
	hu_update_weight(p);
	hm_update_x(dl_se->rd->dl_hg, dl_se->hm_weight, NR_CPUS, dl_se->xi);
}

static inline struct task_struct *assigned_task_of_cpu(int cpu)
{
	struct hm_graph *hg;
	int xi;

	hg = cpu_rq(cpu)->rd->dl_hg;
	xi = hm_mate_of_yi(hg, cpu);
	if (xi >= 0)
		return hm_opaque_x(hg, xi);
	return NULL;
}

static inline struct task_struct *assigned_runnable_task_of_cpu(int cpu)
{
	struct task_struct *p;

	p = assigned_task_of_cpu(cpu);
	/*
	 * Even if a mate for the given cpu is present in the graph,
	 * its mate task is regarded as not being assigned to it unless
	 * it includes the cpu in the allowed cpu mask.
	 */
	if (p && cpumask_test_cpu(cpu, &p->cpus_mask))
		return p;
	return NULL;
}

static void __enqueue_dl_entity(struct sched_dl_entity *dl_se)
{
	struct dl_rq *dl_rq = dl_rq_of_se(dl_se);
	struct task_struct *p = dl_task_of(dl_se);
	struct rq *rq = rq_of_dl_rq(dl_rq);

	SCHED_WARN_ON(!list_empty(&dl_se->latch_dl_entry));
	latch_dl_entry_add(rq, p);

	inc_dl_tasks(dl_se, dl_rq);
}

static void __dequeue_dl_entity(struct sched_dl_entity *dl_se)
{
	struct dl_rq *dl_rq = dl_rq_of_se(dl_se);
	struct task_struct *p = dl_task_of(dl_se);
	struct rq *rq = rq_of_dl_rq(dl_rq);

	if (!list_empty(&dl_se->latch_dl_entry))
		latch_dl_entry_del(rq, p);

	dec_dl_tasks(dl_se, dl_rq);
}

static void enqueue_dl_entity(struct sched_dl_entity *dl_se, int flags)
{
	struct task_struct *p = dl_task_of(dl_se);

	if (on_dl_rq(dl_se)) {
		if (flags & ENQUEUE_REPLENISH)
			/* if replenishing, need to run a hungarian stage */
			hu_dequeue_task(dl_task_of(dl_se));
		else if(!(flags & ENQUEUE_KEEP_CPU))
			BUG_ON(1);
	}

	update_stats_enqueue_dl(dl_rq_of_se(dl_se), dl_se, flags);

	/*
	 * If this is a wakeup or a new instance, the scheduling
	 * parameters of the task might need updating. Otherwise,
	 * we want a replenishment of its runtime.
	 */
	if (flags & ENQUEUE_WAKEUP) {
		task_contending(dl_se, flags);
		update_dl_entity(dl_se);
		restart_resched_timer(p);
	} else if (flags & ENQUEUE_REPLENISH) {
		replenish_dl_entity(dl_se);
		restart_resched_timer(p);
	} else if ((flags & ENQUEUE_RESTORE) &&
		  dl_time_before(dl_se->deadline,
				 rq_clock(rq_of_dl_rq(dl_rq_of_se(dl_se))))) {
		setup_new_dl_entity(dl_se);
		restart_resched_timer(p);
	}

	if (!(flags & ENQUEUE_KEEP_CPU))
		hu_enqueue_task(dl_task_of(dl_se));

	__enqueue_dl_entity(dl_se);
}

static void dequeue_dl_entity(struct sched_dl_entity *dl_se, int flags)
{
	if (!on_dl_rq(dl_se))
		return;

	if (!(flags & DEQUEUE_KEEP_CPU))
		hu_dequeue_task(dl_task_of(dl_se));

	__dequeue_dl_entity(dl_se);
}

static void enqueue_task_dl(struct rq *rq, struct task_struct *p, int flags)
{
	if (is_dl_boosted(&p->dl)) {
		/*
		 * Because of delays in the detection of the overrun of a
		 * thread's runtime, it might be the case that a thread
		 * goes to sleep in a rt mutex with negative runtime. As
		 * a consequence, the thread will be throttled.
		 *
		 * While waiting for the mutex, this thread can also be
		 * boosted via PI, resulting in a thread that is throttled
		 * and boosted at the same time.
		 *
		 * In this case, the boost overrides the throttle.
		 */
		if (p->dl.dl_throttled) {
			/*
			 * The replenish timer needs to be canceled. No
			 * problem if it fires concurrently: boosted threads
			 * are ignored in dl_task_timer().
			 */
			if (hrtimer_try_to_cancel(&p->dl.dl_timer) == 1)
				put_task_struct(p);
			p->dl.dl_throttled = 0;
		}
	} else if (!dl_prio(p->normal_prio)) {
		/*
		 * Special case in which we have a !SCHED_DEADLINE task that is going
		 * to be deboosted, but exceeds its runtime while doing so. No point in
		 * replenishing it, as it's going to return back to its original
		 * scheduling class after this. If it has been throttled, we need to
		 * clear the flag, otherwise the task may wake up as throttled after
		 * being boosted again with no means to replenish the runtime and clear
		 * the throttle.
		 */
		p->dl.dl_throttled = 0;
		BUG_ON(!is_dl_boosted(&p->dl) || flags != ENQUEUE_REPLENISH);
		return;
	}

	/*
	 * Check if a constrained deadline task was activated
	 * after the deadline but before the next period.
	 * If that is the case, the task will be throttled and
	 * the replenishment timer will be set to the next period.
	 */
	if (!p->dl.dl_throttled && !dl_is_implicit(&p->dl))
		dl_check_constrained_dl(&p->dl);

	if (p->on_rq == TASK_ON_RQ_MIGRATING || flags & ENQUEUE_RESTORE) {
		add_rq_bw(&p->dl, &rq->dl);
		add_running_bw(&p->dl, &rq->dl);
	}

	/*
	 * If p is throttled, we do not enqueue it. In fact, if it exhausted
	 * its budget it needs a replenishment and, since it now is on
	 * its rq, the bandwidth timer callback (which clearly has not
	 * run yet) will take care of this.
	 * However, the active utilization does not depend on the fact
	 * that the task is on the runqueue or not (but depends on the
	 * task's state - in GRUB parlance, "inactive" vs "active contending").
	 * In other words, even if a task is throttled its utilization must
	 * be counted in the active utilization; hence, we need to call
	 * add_running_bw().
	 */
	if (p->dl.dl_throttled && !(flags & ENQUEUE_REPLENISH)) {
		if (flags & ENQUEUE_WAKEUP)
			task_contending(&p->dl, flags);

		return;
	}

	enqueue_dl_entity(&p->dl, flags);

	check_schedstat_required();
	update_stats_wait_start_dl(dl_rq_of_se(&p->dl), &p->dl);
}

static void __dequeue_task_dl(struct rq *rq, struct task_struct *p, int flags)
{
	update_stats_dequeue_dl(&rq->dl, &p->dl, flags);
	dequeue_dl_entity(&p->dl, flags);
}

static void dequeue_task_dl(struct rq *rq, struct task_struct *p, int flags)
{
	update_curr_dl(rq);
	__dequeue_task_dl(rq, p, flags);

	if (p->on_rq == TASK_ON_RQ_MIGRATING || flags & DEQUEUE_SAVE) {
		sub_running_bw(&p->dl, &rq->dl);
		sub_rq_bw(&p->dl, &rq->dl);
	}

	/*
	 * This check allows to start the inactive timer (or to immediately
	 * decrease the active utilization, if needed) in two cases:
	 * when the task blocks and when it is terminating
	 * (p->state == TASK_DEAD). We can handle the two cases in the same
	 * way, because from GRUB's point of view the same thing is happening
	 * (the task moves from "active contending" to "active non contending"
	 * or "inactive")
	 */
	if (flags & DEQUEUE_SLEEP)
		task_non_contending(p);
}

/*
 * Yield task semantic for -deadline tasks is:
 *
 *   get off from the CPU until our next instance, with
 *   a new runtime. This is of little use now, since we
 *   don't have a bandwidth reclaiming mechanism. Anyway,
 *   bandwidth reclaiming is planned for the future, and
 *   yield_task_dl will indicate that some spare budget
 *   is available for other task instances to use it.
 */
static void yield_task_dl(struct rq *rq)
{
	/*
	 * We make the task go to sleep until its current deadline by
	 * forcing its runtime to zero. This way, update_curr_dl() stops
	 * it and the bandwidth timer will wake it up and will give it
	 * new scheduling parameters (thanks to dl_yielded=1).
	 */
	rq->curr->dl.dl_yielded = 1;

	update_rq_clock(rq);
	update_curr_dl(rq);
	/*
	 * Tell update_rq_clock() that we've just updated,
	 * so we don't do microscopic update in schedule()
	 * and double the fastpath cost.
	 */
	rq_clock_skip_update(rq);
}

#ifdef CONFIG_SMP

static int
select_task_rq_dl(struct task_struct *p, int cpu, int flags)
{
	int assigned_cpu;

	assigned_cpu = assigned_cpu_of_task(p);
	if (assigned_cpu >= 0)
		return assigned_cpu;

	/*
	 * If an active cpu is not assigned to the task right now,
	 * just return the current cpu since migration won't help.
	 * The caller might try to wake it up and it will possibly
	 * be latched. Its fate will be determined then.
	 */
	return task_cpu(p);
}

static void migrate_task_rq_dl(struct task_struct *p, int new_cpu __maybe_unused)
{
	struct rq_flags rf;
	struct rq *rq;

	if (READ_ONCE(p->__state) != TASK_WAKING)
		return;

	rq = task_rq(p);

	/*
	 * Since p->state == TASK_WAKING, set_task_cpu() has been called
	 * from try_to_wake_up(). Hence, p->pi_lock is locked, but
	 * rq->lock is not... So, lock it
	 */
	rq_lock(rq, &rf);
	if (p->dl.dl_non_contending) {
		update_rq_clock(rq);
		sub_running_bw(&p->dl, &rq->dl);
		p->dl.dl_non_contending = 0;
		/*
		 * If the timer handler is currently running and the
		 * timer cannot be canceled, inactive_task_timer()
		 * will see that dl_not_contending is not set, and
		 * will not touch the rq's active utilization,
		 * so we are still safe.
		 */
		if (hrtimer_try_to_cancel(&p->dl.inactive_timer) == 1)
			put_task_struct(p);
	}
	sub_rq_bw(&p->dl, &rq->dl);
	rq_unlock(rq, &rf);
}

static int balance_dl(struct rq *rq, struct task_struct *p, struct rq_flags *rf)
{
	struct task_struct *new_tsk;

	new_tsk = assigned_runnable_task_of_cpu(rq->cpu);
	if (!new_tsk)
		return sched_stop_runnable(rq);

	if (need_pull_dl_task(rq, p) && task_rq(new_tsk) != rq) {
		rq_unpin_lock(rq, rf);
		pull_dl_task(rq);
		rq_repin_lock(rq, rf);
		return 1;
	}

	return sched_stop_runnable(rq);
}
#endif /* CONFIG_SMP */

/*
 * Only called when both the current and waking task are -deadline
 * tasks.
 */
static void check_preempt_curr_dl(struct rq *rq, struct task_struct *p,
				  int flags)
{
	struct task_struct *tsk;

	tsk = assigned_runnable_task_of_cpu(cpu_of(rq));
	if (tsk && tsk != rq->curr)
		resched_curr(rq);
}

#ifdef CONFIG_SCHED_HRTICK
static void start_hrtick_dl(struct rq *rq, struct task_struct *p)
{
	hrtick_start(rq, p->dl.runtime);
}
#else /* !CONFIG_SCHED_HRTICK */
static void start_hrtick_dl(struct rq *rq, struct task_struct *p)
{
}
#endif

static void set_next_task_dl(struct rq *rq, struct task_struct *p, bool first)
{
	struct sched_dl_entity *dl_se = &p->dl;
	struct dl_rq *dl_rq = &rq->dl;

	__set_task_cpu(p, rq->cpu);

	p->se.exec_start = rq_clock_task(rq);
	if (on_dl_rq(dl_se))
		update_stats_wait_end_dl(dl_rq, dl_se);

	if (!first)
		return;

	if (hrtick_enabled_dl(rq))
		start_hrtick_dl(rq, p);

	if (rq->curr->sched_class != &dl_sched_class)
		update_dl_rq_load_avg(rq_clock_pelt(rq), rq, 0);

	deadline_queue_push_tasks(rq);
}

static struct task_struct *pick_task_dl(struct rq *rq)
{
	struct task_struct *p = NULL, *latched, *tmp;
	struct sched_dl_entity *dl_se;
	bool curr_pushed = false;

	p = assigned_runnable_task_of_cpu(rq->cpu);

	list_for_each_entry_safe(latched, tmp, &rq->dl.latch_dl, dl.latch_dl_entry) {
		if (p != latched) {
			if (latched == rq->curr)
				curr_pushed = true;

			/* if push_dl_task is already running for this task, let him do the job. */
			if (list_empty(&latched->dl.push_dl_entry)) {
				push_dl_entry_add(rq, latched);
				deadline_queue_push_tasks(rq);
			}
		}
		latch_dl_entry_del(rq, latched);
	}

	if (p) {
		if (task_rq(p) != rq) {
			/* needs to pull at first. */
			deadline_queue_pull_task(rq);
			p = NULL;
		} else
			latch_dl_entry_del(rq, p);
	}

	if (dl_task(rq->curr) && p != rq->curr && !curr_pushed) {
		dl_se = &rq->curr->dl;
		if (list_empty(&dl_se->push_dl_entry))
			/*
			 * Otherwise, there must be an ongoing push operation.
			 */
			push_dl_entry_add(rq, rq->curr);

		deadline_queue_push_tasks(rq);
	}

	return p;
}

static struct task_struct *pick_next_task_dl(struct rq *rq)
{
	struct task_struct *p;

	p = pick_task_dl(rq);
	if (p)
		set_next_task_dl(rq, p, true);

	return p;
}

static void put_prev_task_dl(struct rq *rq, struct task_struct *p)
{
	struct sched_dl_entity *dl_se = &p->dl;
	struct dl_rq *dl_rq = &rq->dl;

	if (on_dl_rq(&p->dl))
		update_stats_wait_start_dl(dl_rq, dl_se);

	update_curr_dl(rq);

	update_dl_rq_load_avg(rq_clock_pelt(rq), rq, 1);
}

/*
 * scheduler tick hitting a task of our scheduling class.
 *
 * NOTE: This function can be called remotely by the tick offload that
 * goes along full dynticks. Therefore no local assumption can be made
 * and everything must be accessed through the @rq and @curr passed in
 * parameters.
 */
static void task_tick_dl(struct rq *rq, struct task_struct *p, int queued)
{
	update_curr_dl(rq);

	update_dl_rq_load_avg(rq_clock_pelt(rq), rq, 1);

	if (hrtick_enabled_dl(rq) && queued && p->dl.runtime > 0)
		start_hrtick_dl(rq, p);
}

static void task_fork_dl(struct task_struct *p)
{
	/*
	 * SCHED_DEADLINE tasks cannot fork and this is achieved through
	 * sched_fork()
	 */
}

#ifdef CONFIG_SMP

/* Only try algorithms three times */
#define DL_MAX_TRIES 3

/* Locks the rq it finds */
static struct rq *find_lock_rq(struct task_struct *p, struct rq *rq)
{
	int cpu;

	/*
	 * Even if the assigned cpu is not the one the task is runnable on,
	 * we need to push away so as to make it available to the latest
	 * eligible task. So, not assigned_runnable_cpu_of_task() here.
	 */
	cpu = assigned_cpu_of_task(p);
	if (cpu == -1 || cpu == rq->cpu)
		return NULL;

	double_lock_balance(rq, cpu_rq(cpu));

	return cpu_rq(cpu);
}

static void resched_idle_core(void *arg, int cpu)
{
	struct rq *rq = arg;
	int tries;

	if (rq->cpu == cpu)
		return;

	if (!cpu_online(cpu))
		return;

	/*
	 * If the updated vertex is outside of the root domain span,
	 * it's just a dummy zero-speed cpu. No task can run on it so
	 * no need to resched.
	 */
	if (!cpumask_test_cpu(cpu, rq->rd->span))
		return;

	double_lock_balance(rq, cpu_rq(cpu));
	resched_curr(cpu_rq(cpu));
	double_unlock_balance(rq, cpu_rq(cpu));
}

static int restart_resched_timer(struct task_struct *p)
{
	struct sched_dl_entity *dl_se = &p->dl;
	struct hrtimer *timer = &dl_se->resched_timer;
	struct rq *rq = task_rq(p);
	unsigned long flags;
	ktime_t now, act;
	s64 delta;

	lockdep_assert_rq_held(rq);

	/*
	 * We want the timer to fire at the next pseudo-release, but
	 * considering that it is actually coming from rq->clock and not from
	 * hrtimer's time base reading.
	 */
	act = ns_to_ktime(dl_next_period(dl_se));
	now = hrtimer_cb_get_time(timer);
	delta = ktime_to_ns(now) - rq_clock(rq);
	act = ktime_add_ns(act, delta);

	/*
	 * If the expiry time already passed, e.g., because the value
	 * chosen as the deadline is too small, don't even try to
	 * start the timer in the past!
	 */
	if (ktime_us_delta(act, now) < 0)
		return 0;

	/*
	 * !enqueued will guarantee another callback; even if one is already in
	 * progress. This ensures a balanced {get,put}_task_struct().
	 *
	 * The race against __run_timer() clearing the enqueued state is
	 * harmless because we're holding task_rq()->lock, therefore the timer
	 * expiring after we've done the check will wait on its task_rq_lock()
	 * and observe our state.
	 */
	raw_spin_lock_irqsave(&dl_se->resched_timer_lock, flags);
	if (!hrtimer_is_queued(timer)) {
		get_task_struct(p);
		hrtimer_start(timer, act, HRTIMER_MODE_ABS_HARD);
	}
	raw_spin_unlock_irqrestore(&dl_se->resched_timer_lock, flags);

	return 1;
}

static enum hrtimer_restart resched_task_timer(struct hrtimer *timer)
{
	struct sched_dl_entity *dl_se = container_of(timer,
						     struct sched_dl_entity,
						     resched_timer);
	enum hrtimer_restart ret = HRTIMER_NORESTART;
	struct task_struct *p = dl_task_of(dl_se);
	struct rq_flags rf;
	struct rq *rq;

	rq = task_rq_lock(p, &rf);

	if (!dl_task(p) || READ_ONCE(p->__state) == TASK_DEAD) {
		put_task_struct(p);
		goto unlock;
	}

	sched_clock_tick();
	update_rq_clock(rq);

	hu_refresh_task(p);

	raw_spin_lock(&dl_se->resched_timer_lock);
	if (!hrtimer_is_queued(timer)) {
		ret = HRTIMER_RESTART;
		hrtimer_forward_now(timer, p->dl.dl_period);
	}
	raw_spin_unlock(&dl_se->resched_timer_lock);

unlock:
	task_rq_unlock(rq, p, &rf);
	return ret;
}

void init_dl_resched_task_timer(struct sched_dl_entity *dl_se)
{
	struct hrtimer *timer = &dl_se->resched_timer;

	raw_spin_lock_init(&dl_se->resched_timer_lock);
	hrtimer_init(timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL_HARD);
	timer->function = resched_task_timer;
}

static void push_dl_task(struct rq *src_rq, struct task_struct *p)
{
	struct rq *dst_rq;
	int cpu, tries;
	int queue_flag;

	lockdep_assert_rq_held(src_rq);

	for (tries = 0; tries < DL_MAX_TRIES; tries++) {
		if (is_migration_disabled(p)) {
			push_dl_entry_del(src_rq, p);
			return;
		}

		cpu = assigned_cpu_of_task(p);
		if (cpu < 0) {
			push_dl_entry_del(src_rq, p);
			/* DO NOT deactivate this task */
			return;
		}

		/*
		 * Some hungarian stage runs slipped in between and, as a result, the assigned
		 * cpu is now identical with src_rq. Just reset push_dl_entry and resched.
		 */
		if (cpu == src_rq->cpu) {
			push_dl_entry_del(src_rq, p);
			resched_curr(src_rq);
			return;
		}

		if (!cpu_online(cpu)) {
			push_dl_entry_del(src_rq, p);
			return;
		}

		dst_rq = cpu_rq(cpu);
		get_task_struct(p);

		if (double_lock_balance(src_rq, dst_rq)) {
			if (unlikely(task_rq(p) != src_rq)) {
				/*
				 * Someone slipped in while we double lock and picked this up
				 * on the latest assignet cpu.
				 */
				push_dl_entry_del(src_rq, p);
				double_unlock_balance(src_rq, dst_rq);
				put_task_struct(p);
				return;
			}
			if (assigned_cpu_of_task(p) != cpu) {
				/* The assignment has been renewed so reselect target rq. */
				double_unlock_balance(src_rq, dst_rq);
				put_task_struct(p);
				continue;
			}
			if (list_empty(&p->dl.push_dl_entry)) {
				/* in the small unlocked window above, someone picked this task up and
				 * finished pushing */
				double_unlock_balance(src_rq, dst_rq);
				put_task_struct(p);
				return;
			}
			if (SCHED_WARN_ON(!cpu_online(cpu))) {
				push_dl_entry_del(src_rq, p);
				double_unlock_balance(src_rq, dst_rq);
				put_task_struct(p);
				return;
			}
			push_dl_entry_del(src_rq, p);

			/* At least p is not INACTIVE here so running_bw is subtracted as usual. */
			queue_flag = 0;
			if (src_rq->rd == dst_rq->rd)
				queue_flag |= DEQUEUE_KEEP_CPU;

			deactivate_task(src_rq, p, queue_flag);
			set_task_cpu(p, cpu);
			update_rq_clock(dst_rq);
			activate_task(dst_rq, p, queue_flag | ENQUEUE_NOCLOCK);
			resched_curr(dst_rq);
			double_unlock_balance(src_rq, dst_rq);

			put_task_struct(p);
			return;
		}
		put_task_struct(p);
	}
	push_dl_entry_del(src_rq, p);
}

static void push_dl_tasks(struct rq *rq)
{
	struct task_struct *p;

	while ((p = list_first_entry_or_null(&rq->dl.push_dl,
					     struct task_struct, dl.push_dl_entry)))
		push_dl_task(rq, p);

	hm_for_each_updated_y(rq->rd->dl_hg, resched_idle_core, rq);
}

static void pull_dl_task(struct rq *dst_rq)
{
	int this_cpu = dst_rq->cpu;
	struct task_struct *p;
	struct rq *src_rq;
	int queue_flag;
	int tries;

	lockdep_assert_rq_held(dst_rq);

	for (tries = 0; tries < DL_MAX_TRIES; tries++) {
		p = assigned_runnable_task_of_cpu(dst_rq->cpu);
		if (!p || p == dst_rq->curr)
			return;

		/* NOTE: src_rq might be a stale rq for p. */
		src_rq = task_rq(p);
		if (src_rq == dst_rq)
			return;

		get_task_struct(p);
		if (double_lock_balance(dst_rq, src_rq)) {
			if (task_rq(p) != src_rq) {
				double_unlock_balance(dst_rq, src_rq);
				put_task_struct(p);
				continue;
			}
			if (task_current(src_rq, p)) {
				resched_curr(src_rq);
				double_unlock_balance(dst_rq, src_rq);
				put_task_struct(p);
				return;
			}
			if (task_on_rq_migrating(p) || is_migration_disabled(p)) {
				double_unlock_balance(dst_rq, src_rq);
				put_task_struct(p);
				return;
			}
			if (p->dl.dl_non_contending) {
				sub_running_bw(&p->dl, &src_rq->dl);
				sub_rq_bw(&p->dl, &src_rq->dl);
				add_rq_bw(&p->dl, &dst_rq->dl);
				add_running_bw(&p->dl, &dst_rq->dl);
				set_task_cpu(p, this_cpu);

				double_unlock_balance(dst_rq, src_rq);
				put_task_struct(p);
				return;
			}

			update_rq_clock(dst_rq);
			queue_flag = 0;

			if (task_on_rq_queued(p)) {
				/* should be in ACTIVE state */
				if (dst_rq->rd == src_rq->rd)
					queue_flag |= DEQUEUE_KEEP_CPU;
				deactivate_task(src_rq, p, queue_flag);
				/* ensure latch_dl_entry is reset */
				latch_dl_entry_del(src_rq, p);
			}

			set_task_cpu(p, this_cpu);
			activate_task(dst_rq, p, queue_flag | ENQUEUE_NOCLOCK);
			resched_curr(dst_rq);
			double_unlock_balance(dst_rq, src_rq);
			put_task_struct(p);
			return;
		}
		put_task_struct(p);
	}
}

static void set_cpus_allowed_dl(struct task_struct *p,
				const struct cpumask *new_mask,
				u32 flags)
{
	struct root_domain *src_rd;
	struct rq *rq;

	BUG_ON(!dl_task(p));

	rq = task_rq(p);
	src_rd = rq->rd;
	/*
	 * Migrating a SCHED_DEADLINE task between exclusive
	 * cpusets (different root_domains) entails a bandwidth
	 * update. We already made space for us in the destination
	 * domain (see cpuset_can_attach()).
	 */
	if (!cpumask_intersects(src_rd->span, new_mask)) {
		struct dl_bw *src_dl_b;

		src_dl_b = dl_bw_of(cpu_of(rq));
		/*
		 * We now free resources of the root_domain we are migrating
		 * off. In the worst case, sched_setattr() may temporary fail
		 * until we complete the update.
		 */
		raw_spin_lock(&src_dl_b->lock);
		__dl_sub(src_dl_b, p->dl.dl_bw, dl_bw_cpus(task_cpu(p)));
		raw_spin_unlock(&src_dl_b->lock);

		/* Cancel resched_task_timer */
		if (hrtimer_try_to_cancel(&p->dl.resched_timer) == 1)
			put_task_struct(p);
	}

	/* reset all edge weights to zero */
	memset(&p->dl.hm_weight, 0, sizeof(p->dl.hm_weight));

	update_dl_runnable(p, new_mask);

	set_cpus_allowed_common(p, new_mask, flags);
}

/* Assumes rq->lock is held */
static void rq_online_dl(struct rq *rq)
{
	int yi;

	/* add Y vertex */
	yi = hm_add_y(rq->rd->dl_hg, rq);
	if (yi < 0)
		return;

	rq->dl.yi = yi;
}

/* Assumes rq->lock is held */
static void rq_offline_dl(struct rq *rq)
{
	/* remove Y vertex */
	hm_del_y(rq->rd->dl_hg, rq->dl.yi);
}

void __init init_sched_dl_class(void)
{
}

void dl_add_task_root_domain(struct task_struct *p)
{
	struct rq_flags rf;
	struct rq *rq;
	struct dl_bw *dl_b;

	raw_spin_lock_irqsave(&p->pi_lock, rf.flags);
	if (!dl_task(p)) {
		raw_spin_unlock_irqrestore(&p->pi_lock, rf.flags);
		return;
	}

	rq = __task_rq_lock(p, &rf);

	dl_b = &rq->rd->dl_bw;
	raw_spin_lock(&dl_b->lock);

	__dl_add(dl_b, p->dl.dl_bw, cpumask_weight(rq->rd->span));

	raw_spin_unlock(&dl_b->lock);

	task_rq_unlock(rq, p, &rf);
}

void dl_clear_root_domain(struct root_domain *rd)
{
	unsigned long flags;

	raw_spin_lock_irqsave(&rd->dl_bw.lock, flags);
	rd->dl_bw.total_bw = 0;
	raw_spin_unlock_irqrestore(&rd->dl_bw.lock, flags);
}

#endif /* CONFIG_SMP */

static void switched_from_dl(struct rq *rq, struct task_struct *p)
{
	/*
	 * task_non_contending() can start the "inactive timer" (if the 0-lag
	 * time is in the future). If the task switches back to dl before
	 * the "inactive timer" fires, it can continue to consume its current
	 * runtime using its current deadline. If it stays outside of
	 * SCHED_DEADLINE until the 0-lag time passes, inactive_task_timer()
	 * will reset the task parameters.
	 */
	if (task_on_rq_queued(p) && p->dl.dl_runtime)
		task_non_contending(p);

	if (!task_on_rq_queued(p)) {
		/*
		 * Inactive timer is armed. However, p is leaving DEADLINE and
		 * might migrate away from this rq while continuing to run on
		 * some other class. We need to remove its contribution from
		 * this rq running_bw now, or sub_rq_bw (below) will complain.
		 */
		if (p->dl.dl_non_contending)
			sub_running_bw(&p->dl, &rq->dl);
		sub_rq_bw(&p->dl, &rq->dl);
	}

	/*
	 * We cannot use inactive_task_timer() to invoke sub_running_bw()
	 * at the 0-lag time, because the task could have been migrated
	 * while SCHED_OTHER in the meanwhile.
	 */
	if (p->dl.dl_non_contending)
		p->dl.dl_non_contending = 0;
}

/*
 * When switching to -deadline, we may overload the rq, then
 * we try to push someone off, if possible.
 */
static void switched_to_dl(struct rq *rq, struct task_struct *p)
{
	if (hrtimer_try_to_cancel(&p->dl.inactive_timer) == 1)
		put_task_struct(p);

	/* If p is not queued we will update its parameters at next wakeup. */
	if (!task_on_rq_queued(p)) {
		add_rq_bw(&p->dl, &rq->dl);

		return;
	}

	if (rq->curr != p) {
#ifdef CONFIG_SMP
		if (assigned_cpu_of_task(p) != rq->cpu)
			deadline_queue_push_tasks(rq);
#endif
		if (dl_task(rq->curr))
			check_preempt_curr_dl(rq, p, 0);
		else
			resched_curr(rq);
	} else {
		update_dl_rq_load_avg(rq_clock_pelt(rq), rq, 0);
	}
}

/*
 * If the scheduling parameters of a -deadline task changed,
 * a push or pull operation might be needed.
 */
static void prio_changed_dl(struct rq *rq, struct task_struct *p,
			    int oldprio)
{
	if (task_on_rq_queued(p) || task_current(rq, p)) {
		hu_refresh_task(p);
		hm_for_each_updated_y(rq->rd->dl_hg, resched_idle_core, rq);
	}
}

DEFINE_SCHED_CLASS(dl) = {

	.enqueue_task		= enqueue_task_dl,
	.dequeue_task		= dequeue_task_dl,
	.yield_task		= yield_task_dl,

	.check_preempt_curr	= check_preempt_curr_dl,

	.pick_next_task		= pick_next_task_dl,
	.put_prev_task		= put_prev_task_dl,
	.set_next_task		= set_next_task_dl,

#ifdef CONFIG_SMP
	.balance		= balance_dl,
	.pick_task		= pick_task_dl,
	.select_task_rq		= select_task_rq_dl,
	.migrate_task_rq	= migrate_task_rq_dl,
	.set_cpus_allowed       = set_cpus_allowed_dl,
	.rq_online              = rq_online_dl,
	.rq_offline             = rq_offline_dl,
	.find_lock_rq		= find_lock_rq,
#endif

	.task_tick		= task_tick_dl,
	.task_fork              = task_fork_dl,

	.prio_changed           = prio_changed_dl,
	.switched_from		= switched_from_dl,
	.switched_to		= switched_to_dl,

	.update_curr		= update_curr_dl,
};

/* Used for dl_bw check and update, used under sched_rt_handler()::mutex */
static u64 dl_generation;

int sched_dl_global_validate(void)
{
	u64 runtime = global_rt_runtime();
	u64 period = global_rt_period();
	u64 new_bw = to_ratio(period, runtime);
	u64 gen = ++dl_generation;
	struct dl_bw *dl_b;
	int cpu, cpus, ret = 0;
	unsigned long flags;

	/*
	 * Here we want to check the bandwidth not being set to some
	 * value smaller than the currently allocated bandwidth in
	 * any of the root_domains.
	 */
	for_each_possible_cpu(cpu) {
		rcu_read_lock_sched();

		if (dl_bw_visited(cpu, gen))
			goto next;

		dl_b = dl_bw_of(cpu);
		cpus = dl_bw_cpus(cpu);

		raw_spin_lock_irqsave(&dl_b->lock, flags);
		if (new_bw * cpus < dl_b->total_bw)
			ret = -EBUSY;
		raw_spin_unlock_irqrestore(&dl_b->lock, flags);

next:
		rcu_read_unlock_sched();

		if (ret)
			break;
	}

	return ret;
}

static void init_dl_rq_bw_ratio(struct dl_rq *dl_rq)
{
	if (global_rt_runtime() == RUNTIME_INF) {
		dl_rq->bw_ratio = 1 << RATIO_SHIFT;
		dl_rq->extra_bw = 1 << BW_SHIFT;
	} else {
		dl_rq->bw_ratio = to_ratio(global_rt_runtime(),
			  global_rt_period()) >> (BW_SHIFT - RATIO_SHIFT);
		dl_rq->extra_bw = to_ratio(global_rt_period(),
						    global_rt_runtime());
	}
}

void sched_dl_do_global(void)
{
	u64 new_bw = -1;
	u64 gen = ++dl_generation;
	struct dl_bw *dl_b;
	int cpu;
	unsigned long flags;

	if (global_rt_runtime() != RUNTIME_INF)
		new_bw = to_ratio(global_rt_period(), global_rt_runtime());

	for_each_possible_cpu(cpu) {
		rcu_read_lock_sched();

		if (dl_bw_visited(cpu, gen)) {
			rcu_read_unlock_sched();
			continue;
		}

		dl_b = dl_bw_of(cpu);

		raw_spin_lock_irqsave(&dl_b->lock, flags);
		dl_b->bw = new_bw;
		raw_spin_unlock_irqrestore(&dl_b->lock, flags);

		rcu_read_unlock_sched();
		init_dl_rq_bw_ratio(&cpu_rq(cpu)->dl);
	}
}

/*
 * We must be sure that accepting a new task (or allowing changing the
 * parameters of an existing one) is consistent with the bandwidth
 * constraints. If yes, this function also accordingly updates the currently
 * allocated bandwidth to reflect the new situation.
 *
 * This function is called while holding p's rq->lock.
 */
int sched_dl_overflow(struct task_struct *p, int policy,
		      const struct sched_attr *attr)
{
	u64 period = attr->sched_period ?: attr->sched_deadline;
	u64 runtime = attr->sched_runtime;
	u64 new_bw = dl_policy(policy) ? to_ratio(period, runtime) : 0;
	int cpus, err = -1, cpu = task_cpu(p);
	struct dl_bw *dl_b = dl_bw_of(cpu);
	unsigned long cap;

	if (attr->sched_flags & SCHED_FLAG_SUGOV)
		return 0;

	/* !deadline task may carry old deadline bandwidth */
	if (new_bw == p->dl.dl_bw && task_has_dl_policy(p))
		return 0;

	/*
	 * Either if a task, enters, leave, or stays -deadline but changes
	 * its parameters, we may need to update accordingly the total
	 * allocated bandwidth of the container.
	 */
	raw_spin_lock(&dl_b->lock);
	cpus = dl_bw_cpus(cpu);
	cap = dl_bw_capacity(cpu);

	if (dl_policy(policy) && !task_has_dl_policy(p) &&
	    !__dl_overflow(dl_b, cap, 0, new_bw)) {
		if (hrtimer_active(&p->dl.inactive_timer))
			__dl_sub(dl_b, p->dl.dl_bw, cpus);
		__dl_add(dl_b, new_bw, cpus);
		err = 0;
	} else if (dl_policy(policy) && task_has_dl_policy(p) &&
		   !__dl_overflow(dl_b, cap, p->dl.dl_bw, new_bw)) {
		/*
		 * XXX this is slightly incorrect: when the task
		 * utilization decreases, we should delay the total
		 * utilization change until the task's 0-lag point.
		 * But this would require to set the task's "inactive
		 * timer" when the task is not inactive.
		 */
		__dl_sub(dl_b, p->dl.dl_bw, cpus);
		__dl_add(dl_b, new_bw, cpus);
		dl_change_utilization(p, new_bw);
		err = 0;
	} else if (!dl_policy(policy) && task_has_dl_policy(p)) {
		/*
		 * Do not decrease the total deadline utilization here,
		 * switched_from_dl() will take care to do it at the correct
		 * (0-lag) time.
		 */
		err = 0;
	}
	raw_spin_unlock(&dl_b->lock);

	return err;
}

/*
 * This function initializes the sched_dl_entity of a newly becoming
 * SCHED_DEADLINE task.
 *
 * Only the static values are considered here, the actual runtime and the
 * absolute deadline will be properly calculated when the task is enqueued
 * for the first time with its new policy.
 */
void __setparam_dl(struct task_struct *p, const struct sched_attr *attr)
{
	struct sched_dl_entity *dl_se = &p->dl;

	dl_se->dl_runtime = attr->sched_runtime;
	dl_se->dl_deadline = attr->sched_deadline;
	dl_se->dl_period = attr->sched_period ?: dl_se->dl_deadline;
	dl_se->flags = attr->sched_flags & SCHED_DL_FLAGS;
	dl_se->dl_bw = to_ratio(dl_se->dl_period, dl_se->dl_runtime);
	dl_se->dl_density = to_ratio(dl_se->dl_deadline, dl_se->dl_runtime);

	inc_dl_runnable(p);
}

void __getparam_dl(struct task_struct *p, struct sched_attr *attr)
{
	struct sched_dl_entity *dl_se = &p->dl;

	attr->sched_priority = p->rt_priority;
	attr->sched_runtime = dl_se->dl_runtime;
	attr->sched_deadline = dl_se->dl_deadline;
	attr->sched_period = dl_se->dl_period;
	attr->sched_flags &= ~SCHED_DL_FLAGS;
	attr->sched_flags |= dl_se->flags;
}

/*
 * This function validates the new parameters of a -deadline task.
 * We ask for the deadline not being zero, and greater or equal
 * than the runtime, as well as the period of being zero or
 * greater than deadline. Furthermore, we have to be sure that
 * user parameters are above the internal resolution of 1us (we
 * check sched_runtime only since it is always the smaller one) and
 * below 2^63 ns (we have to check both sched_deadline and
 * sched_period, as the latter can be zero).
 */
bool __checkparam_dl(const struct sched_attr *attr)
{
	u64 period, max, min;

	/* special dl tasks don't actually use any parameter */
	if (attr->sched_flags & SCHED_FLAG_SUGOV)
		return true;

	/* deadline != 0 */
	if (attr->sched_deadline == 0)
		return false;

	/*
	 * Since we truncate DL_SCALE bits, make sure we're at least
	 * that big.
	 */
	if (attr->sched_runtime < (1ULL << DL_SCALE))
		return false;

	/*
	 * Since we use the MSB for wrap-around and sign issues, make
	 * sure it's not set (mind that period can be equal to zero).
	 */
	if (attr->sched_deadline & (1ULL << 63) ||
	    attr->sched_period & (1ULL << 63))
		return false;

	period = attr->sched_period;
	if (!period)
		period = attr->sched_deadline;

	/* runtime <= deadline <= period (if period != 0) */
	if (period < attr->sched_deadline ||
	    attr->sched_deadline < attr->sched_runtime)
		return false;

	max = (u64)READ_ONCE(sysctl_sched_dl_period_max) * NSEC_PER_USEC;
	min = (u64)READ_ONCE(sysctl_sched_dl_period_min) * NSEC_PER_USEC;

	if (period < min || period > max)
		return false;

	return true;
}

/*
 * This function clears the sched_dl_entity static params.
 */
void __dl_clear_params(struct task_struct *p)
{
	struct sched_dl_entity *dl_se = &p->dl;

	dl_se->dl_runtime		= 0;
	dl_se->dl_deadline		= 0;
	dl_se->dl_period		= 0;
	dl_se->flags			= 0;
	dl_se->dl_bw			= 0;
	dl_se->dl_density		= 0;

	dl_se->dl_throttled		= 0;
	dl_se->dl_yielded		= 0;
	dl_se->dl_non_contending	= 0;
	dl_se->dl_overrun		= 0;

#ifdef CONFIG_RT_MUTEXES
	dl_se->pi_se			= dl_se;
#endif
}

bool dl_param_changed(struct task_struct *p, const struct sched_attr *attr)
{
	struct sched_dl_entity *dl_se = &p->dl;

	if (dl_se->dl_runtime != attr->sched_runtime ||
	    dl_se->dl_deadline != attr->sched_deadline ||
	    dl_se->dl_period != attr->sched_period ||
	    dl_se->flags != (attr->sched_flags & SCHED_DL_FLAGS))
		return true;

	return false;
}

#ifdef CONFIG_SMP
int dl_cpuset_cpumask_can_shrink(const struct cpumask *cur,
				 const struct cpumask *trial)
{
	int ret = 1, trial_cpus;
	struct dl_bw *cur_dl_b;
	unsigned long flags;

	rcu_read_lock_sched();
	cur_dl_b = dl_bw_of(cpumask_any(cur));
	trial_cpus = cpumask_weight(trial);

	raw_spin_lock_irqsave(&cur_dl_b->lock, flags);
	if (cur_dl_b->bw != -1 &&
	    cur_dl_b->bw * trial_cpus < cur_dl_b->total_bw)
		ret = 0;
	raw_spin_unlock_irqrestore(&cur_dl_b->lock, flags);
	rcu_read_unlock_sched();

	return ret;
}

int dl_cpu_busy(int cpu, struct task_struct *p)
{
	unsigned long flags, cap;
	struct dl_bw *dl_b;
	bool overflow;

	rcu_read_lock_sched();
	dl_b = dl_bw_of(cpu);
	raw_spin_lock_irqsave(&dl_b->lock, flags);
	cap = dl_bw_capacity(cpu);
	overflow = __dl_overflow(dl_b, cap, 0, p ? p->dl.dl_bw : 0);

	if (!overflow && p) {
		/*
		 * We reserve space for this task in the destination
		 * root_domain, as we can't fail after this point.
		 * We will free resources in the source root_domain
		 * later on (see set_cpus_allowed_dl()).
		 */
		__dl_add(dl_b, p->dl.dl_bw, dl_bw_cpus(cpu));
	}

	raw_spin_unlock_irqrestore(&dl_b->lock, flags);
	rcu_read_unlock_sched();

	return overflow ? -EBUSY : 0;
}
#endif

#ifdef CONFIG_SCHED_DEBUG
void print_dl_stats(struct seq_file *m, int cpu)
{
	print_dl_rq(m, cpu, &cpu_rq(cpu)->dl);
}
#endif /* CONFIG_SCHED_DEBUG */
