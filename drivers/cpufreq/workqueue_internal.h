/*
 * kernel/workqueue_internal.h
 *
 * Workqueue internal header file.  Only to be included by workqueue and
 * core kernel subsystems.
 */
#ifndef _KERNEL_WORKQUEUE_INTERNAL_H
#define _KERNEL_WORKQUEUE_INTERNAL_H

#include <linux/workqueue.h>
#include <linux/kthread.h>

#define for_each_busy_worker(worker, i, pos, gcwq)			\
	for (i = 0; i < BUSY_WORKER_HASH_SIZE; i++)			\
		hlist_for_each_entry(worker, pos, &gcwq->busy_hash[i], hentry)

enum {
	NR_WORKER_POOLS		= 2,		/* # worker pools per gcwq */
	BUSY_WORKER_HASH_ORDER	= 6,		/* 64 pointers */
	BUSY_WORKER_HASH_SIZE	= 1 << BUSY_WORKER_HASH_ORDER,
};

static inline int __next_gcwq_cpu(int cpu, const struct cpumask *mask,
				  unsigned int sw)
{
	if (cpu < nr_cpu_ids) {
		if (sw & 1) {
			cpu = cpumask_next(cpu, mask);
			if (cpu < nr_cpu_ids)
				return cpu;
		}
		if (sw & 2)
			return WORK_CPU_UNBOUND;
	}
	return WORK_CPU_NONE;
}

/*
 * CPU iterators
 *
 * An extra gcwq is defined for an invalid cpu number
 * (WORK_CPU_UNBOUND) to host workqueues which are not bound to any
 * specific CPU.  The following iterators are similar to
 * for_each_*_cpu() iterators but also considers the unbound gcwq.
 *
 * for_each_gcwq_cpu()		: possible CPUs + WORK_CPU_UNBOUND
 * for_each_online_gcwq_cpu()	: online CPUs + WORK_CPU_UNBOUND
 * for_each_cwq_cpu()		: possible CPUs for bound workqueues,
 *				  WORK_CPU_UNBOUND for unbound workqueues
 */
#define for_each_gcwq_cpu(cpu)						\
	for ((cpu) = __next_gcwq_cpu(-1, cpu_possible_mask, 3);		\
	     (cpu) < WORK_CPU_NONE;					\
	     (cpu) = __next_gcwq_cpu((cpu), cpu_possible_mask, 3))

struct global_cwq;
struct worker_pool;

struct worker_pool;

/*
 * The poor guys doing the actual heavy lifting.  All on-duty workers are
 * either serving the manager role, on idle list or on busy hash.  For
 * details on the locking annotation (L, I, X...), refer to workqueue.c.
 *
 * Only to be used in workqueue and async.
 */
struct worker {
	/* on idle list while idle, on busy hash table while busy */
	union {
		struct list_head	entry;	/* L: while idle */
		struct hlist_node	hentry;	/* L: while busy */
	};

	struct work_struct	*current_work;	/* L: work being processed */
	work_func_t		current_func;	/* L: current_work's fn */
	struct pool_workqueue	*current_pwq; /* L: current_work's pwq */
	bool			desc_valid;	/* ->desc is valid */
	struct list_head	scheduled;	/* L: scheduled works */

	/* 64 bytes boundary on 64bit, 32 on 32bit */

	struct task_struct	*task;		/* I: worker task */
	struct worker_pool	*pool;		/* I: the associated pool */
						/* L: for rescuers */

	unsigned long		last_active;	/* L: last active timestamp */
	unsigned int		flags;		/* X: flags */
	int			id;		/* I: worker id */

	/*
	 * Opaque string set with work_set_desc().  Printed out with task
	 * dump for debugging - WARN, BUG, panic or sysrq.
	 */
	char			desc[WORKER_DESC_LEN];

	/* used only by rescuers to point to the target workqueue */
	struct workqueue_struct	*rescue_wq;	/* I: the workqueue to rescue */
};

/**
 * current_wq_worker - return struct worker if %current is a workqueue worker
 */
static inline struct worker *current_wq_worker(void)
{
	if (current->flags & PF_WQ_WORKER)
		return kthread_data(current);
	return NULL;
}

/*
 * Scheduler hooks for concurrency managed workqueue.  Only to be used from
 * sched.c and workqueue.c.
 */
void wq_worker_waking_up(struct task_struct *task, int cpu);
struct task_struct *wq_worker_sleeping(struct task_struct *task, int cpu);

#endif /* _KERNEL_WORKQUEUE_INTERNAL_H */

static struct cpu_workqueue_struct *get_work_cwq(struct work_struct *work)
{
	unsigned long data = atomic_long_read(&work->data);

	if (data & WORK_STRUCT_CWQ)
		return (void *)(data & WORK_STRUCT_WQ_DATA_MASK);
	else
		return NULL;
}


/*
 * Global per-cpu workqueue.  There's one and only one for each cpu
 * and all works are queued and processed here regardless of their
 * target workqueues.
 */
struct global_cwq {
	spinlock_t		lock;		/* the gcwq lock */
	unsigned int		cpu;		/* I: the associated cpu */
	unsigned int		flags;		/* L: GCWQ_* flags */

	/* workers are chained either in busy_hash or pool idle_list */
	struct hlist_head	busy_hash[BUSY_WORKER_HASH_SIZE];
						/* L: hash of busy workers */

	struct worker_pool	pools[2];	/* normal and highpri pools */

	struct task_struct	*trustee;	/* L: for gcwq shutdown */
	unsigned int		trustee_state;	/* L: trustee state */
	wait_queue_head_t	trustee_wait;	/* trustee wait */
} ____cacheline_aligned_in_smp;

/*
 * Global cpu workqueue and nr_running counter for unbound gcwq.  The
 * gcwq is always online, has GCWQ_DISASSOCIATED set, and all its
 * workers have WORKER_UNBOUND set.
 */
static struct global_cwq unbound_global_cwq;
static atomic_t unbound_pool_nr_running[NR_WORKER_POOLS] = {
	[0 ... NR_WORKER_POOLS - 1]	= ATOMIC_INIT(0),	/* always 0 */
};

/*
 * The almighty global cpu workqueues.  nr_running is the only field
 * which is expected to be used frequently by other cpus via
 * try_to_wake_up().  Put it in a separate cacheline.
 */
static DEFINE_PER_CPU(struct global_cwq, global_cwq);

static struct global_cwq *get_gcwq(unsigned int cpu)
{
	if (cpu != WORK_CPU_UNBOUND)
		return &per_cpu(global_cwq, cpu);
	else
		return &unbound_global_cwq;
}

/*
 * Test whether @work is being queued from another work executing on the
 * same workqueue.  This is rather expensive and should only be used from
 * cold paths.
 */
static bool is_chained_work(struct workqueue_struct *wq)
{
	unsigned long flags;
	unsigned int cpu;

	for_each_gcwq_cpu(cpu) {
		struct global_cwq *gcwq = get_gcwq(cpu);
		struct worker *worker;
		struct hlist_node *pos;
		int i;

		spin_lock_irqsave(&gcwq->lock, flags);
		for_each_busy_worker(worker, i, pos, gcwq) {
			if (worker->task != current)
				continue;
			spin_unlock_irqrestore(&gcwq->lock, flags);
			/*
			 * I'm @worker, no locking necessary.  See if @work
			 * is headed to the same workqueue.
			 */
			return worker->current_cwq->wq == wq;
		}
		spin_unlock_irqrestore(&gcwq->lock, flags);
	}
	return false;
}

static struct global_cwq *get_work_gcwq(struct work_struct *work)
{
	unsigned long data = atomic_long_read(&work->data);
	unsigned int cpu;

	if (data & WORK_STRUCT_CWQ)
		return ((struct cpu_workqueue_struct *)
			(data & WORK_STRUCT_WQ_DATA_MASK))->pool->gcwq;

	cpu = data >> WORK_STRUCT_FLAG_BITS;
	if (cpu == WORK_CPU_NONE)
		return NULL;

	BUG_ON(cpu >= nr_cpu_ids && cpu != WORK_CPU_UNBOUND);
	return get_gcwq(cpu);
}

static void *work_debug_hint(void *addr)
{
	return ((struct work_struct *) addr)->func;
}

/*
 * fixup_activate is called when:
 * - an active object is activated
 * - an unknown object is activated (might be a statically initialized object)
 */
static int work_fixup_activate(void *addr, enum debug_obj_state state)
{
	struct work_struct *work = addr;

	switch (state) {

	case ODEBUG_STATE_NOTAVAILABLE:
		/*
		 * This is not really a fixup. The work struct was
		 * statically initialized. We just make sure that it
		 * is tracked in the object tracker.
		 */
		if (test_bit(WORK_STRUCT_STATIC_BIT, work_data_bits(work))) {
			debug_object_init(work, &work_debug_descr);
			debug_object_activate(work, &work_debug_descr);
			return 0;
		}
		WARN_ON_ONCE(1);
		return 0;

	case ODEBUG_STATE_ACTIVE:
		WARN_ON(1);

	default:
		return 0;
	}
}

/*
 * fixup_init is called when:
 * - an active object is initialized
 */
static int work_fixup_init(void *addr, enum debug_obj_state state)
{
	struct work_struct *work = addr;

	switch (state) {
	case ODEBUG_STATE_ACTIVE:
		cancel_work_sync(work);
		debug_object_init(work, &work_debug_descr);
		return 1;
	default:
		return 0;
	}
}

/*
 * fixup_free is called when:
 * - an active object is freed
 */
static int work_fixup_free(void *addr, enum debug_obj_state state)
{
	struct work_struct *work = addr;

	switch (state) {
	case ODEBUG_STATE_ACTIVE:
		cancel_work_sync(work);
		debug_object_free(work, &work_debug_descr);
		return 1;
	default:
		return 0;
	}
}

static struct debug_obj_descr work_debug_descr;

static struct debug_obj_descr work_debug_descr = {
	.name		= "work_struct",
	.debug_hint	= work_debug_hint,
	.fixup_init	= work_fixup_init,
	.fixup_activate	= work_fixup_activate,
	.fixup_free	= work_fixup_free,
};

static inline void debug_work_activate(struct work_struct *work)
{
	debug_object_activate(work, &work_debug_descr);
}

static void __queue_work(unsigned int cpu, struct workqueue_struct *wq,
			 struct work_struct *work)
{
	struct global_cwq *gcwq;
	struct cpu_workqueue_struct *cwq;
	struct list_head *worklist;
	unsigned int work_flags;
	unsigned long flags;

	debug_work_activate(work);

	/* if dying, only works from the same workqueue are allowed */
	if (unlikely(wq->flags & WQ_DRAINING) &&
	    WARN_ON_ONCE(!is_chained_work(wq)))
		return;

	/* determine gcwq to use */
	if (!(wq->flags & WQ_UNBOUND)) {
		struct global_cwq *last_gcwq;

		if (unlikely(cpu == WORK_CPU_UNBOUND))
			cpu = raw_smp_processor_id();

		/*
		 * It's multi cpu.  If @wq is non-reentrant and @work
		 * was previously on a different cpu, it might still
		 * be running there, in which case the work needs to
		 * be queued on that cpu to guarantee non-reentrance.
		 */
		gcwq = get_gcwq(cpu);
		if (wq->flags & WQ_NON_REENTRANT &&
		    (last_gcwq = get_work_gcwq(work)) && last_gcwq != gcwq) {
			struct worker *worker;

			spin_lock_irqsave(&last_gcwq->lock, flags);

			worker = find_worker_executing_work(last_gcwq, work);

			if (worker && worker->current_cwq->wq == wq)
				gcwq = last_gcwq;
			else {
				/* meh... not running there, queue here */
				spin_unlock_irqrestore(&last_gcwq->lock, flags);
				spin_lock_irqsave(&gcwq->lock, flags);
			}
		} else
			spin_lock_irqsave(&gcwq->lock, flags);
	} else {
		gcwq = get_gcwq(WORK_CPU_UNBOUND);
		spin_lock_irqsave(&gcwq->lock, flags);
	}

	/* gcwq determined, get cwq and queue */
	cwq = get_cwq(gcwq->cpu, wq);
	trace_workqueue_queue_work(cpu, cwq, work);

	BUG_ON(!list_empty(&work->entry));

	cwq->nr_in_flight[cwq->work_color]++;
	work_flags = work_color_to_flags(cwq->work_color);

	if (likely(cwq->nr_active < cwq->max_active)) {
		trace_workqueue_activate_work(work);
		cwq->nr_active++;
		worklist = &cwq->pool->worklist;
	} else {
		work_flags |= WORK_STRUCT_DELAYED;
		worklist = &cwq->delayed_works;
	}

	insert_work(cwq, work, worklist, work_flags);

	spin_unlock_irqrestore(&gcwq->lock, flags);
}

static inline void debug_work_activate(struct work_struct *work)
{
	debug_object_activate(work, &work_debug_descr);
}

static struct global_cwq *get_gcwq(unsigned int cpu)
{
	if (cpu != WORK_CPU_UNBOUND)
		return &per_cpu(global_cwq, cpu);
	else
		return &unbound_global_cwq;
}

static inline void debug_work_activate(struct work_struct *work)
{
	debug_object_activate(work, &work_debug_descr);
}


