/*
 * Detect Hung Task
 *
 * kernel/hung_task.c - kernel thread for detecting tasks stuck in D state
 *
 */

#include <linux/mm.h>
#include <linux/cpu.h>
#include <linux/nmi.h>
#include <linux/init.h>
#include <linux/delay.h>
#include <linux/freezer.h>
#include <linux/kthread.h>
#include <linux/lockdep.h>
#include <linux/export.h>
#include <linux/sysctl.h>
#include <linux/utsname.h>
#include <trace/events/sched.h>
#include <linux/sched/sysctl.h>

#ifdef VENDOR_EDIT
#define HUNG_TASK_OPPO_KILL_LEN	128
char __read_mostly sysctl_hung_task_oppo_kill[HUNG_TASK_OPPO_KILL_LEN];
char last_stopper_comm[64];

#define TWICE_DEATH_PERIOD	300000000000ULL	//300s
#define MAX_DEATH_COUNT	3
#define MAX_IO_WAIT_HUNG 3
#endif

/*
 * The number of tasks checked:
 */
int __read_mostly sysctl_hung_task_check_count = PID_MAX_LIMIT;

/*
 * Selective monitoring of hung tasks.
 *
 * if set to 1, khungtaskd skips monitoring tasks, which has
 * task_struct->hang_detection_enabled value not set, else monitors all tasks.
 */
int sysctl_hung_task_selective_monitoring = 1;

/*
 * Limit number of tasks checked in a batch.
 *
 * This value controls the preemptibility of khungtaskd since preemption
 * is disabled during the critical section. It also controls the size of
 * the RCU grace period. So it needs to be upper-bound.
 */
#define HUNG_TASK_BATCHING 1024

/*
 * Zero means infinite timeout - no checking done:
 */
unsigned long __read_mostly sysctl_hung_task_timeout_secs = CONFIG_DEFAULT_HUNG_TASK_TIMEOUT;

int __read_mostly sysctl_hung_task_warnings = 10;

static int __read_mostly did_panic;

static struct task_struct *watchdog_task;

/*
 * Should we panic (and reboot, if panic_timeout= is set) when a
 * hung task is detected:
 */
unsigned int __read_mostly sysctl_hung_task_panic =
				CONFIG_BOOTPARAM_HUNG_TASK_PANIC_VALUE;

static int __init hung_task_panic_setup(char *str)
{
	int rc = kstrtouint(str, 0, &sysctl_hung_task_panic);

	if (rc)
		return rc;
	return 1;
}
__setup("hung_task_panic=", hung_task_panic_setup);

static int
hung_task_panic(struct notifier_block *this, unsigned long event, void *ptr)
{
	did_panic = 1;

	return NOTIFY_DONE;
}

static struct notifier_block panic_block = {
	.notifier_call = hung_task_panic,
};

#if defined(VENDOR_EDIT)
static bool is_zygote_process(struct task_struct *t)
{
	const struct cred *tcred = __task_cred(t);
	if(!strcmp(t->comm, "main") && (tcred->uid.val == 0) && (t->parent != 0 && !strcmp(t->parent->comm,"init"))  )
		return true;
	else
		return false;
	return false;
}
#endif

#if defined(VENDOR_EDIT)
static bool is_fsck_process(struct task_struct *t)
{
	if (!strcmp(t->comm, "fsck.fat") || !strcmp(t->comm, "fsck_msdos") || !strcmp(t->comm, "criticallog_syn") || !strncmp(t->comm, "kworker/u16", 11))
		return true;
	else
		return false;
	return false;
}
#endif

#if defined(VENDOR_EDIT) && defined(CONFIG_DEATH_HEALER)
static void check_hung_task(struct task_struct *t, unsigned long timeout, unsigned int *iowait_count)
#else
static void check_hung_task(struct task_struct *t, unsigned long timeout)
#endif
{
	unsigned long switch_count = t->nvcsw + t->nivcsw;

	#if defined(VENDOR_EDIT) && defined(CONFIG_DEATH_HEALER)
	static unsigned long long last_death_time = 0;
	unsigned long long cur_death_time = 0;
	static int death_count = 0;
	#endif /* VENDOR_EDIT */

	#ifdef VENDOR_EDIT
	#define DISP_TASK_COMM_LEN_MASK 10 //SDM845 change the new display thread with multi output, use len for masking
	if(!strncmp(t->comm,"mdss_dsi_event", TASK_COMM_LEN)||
		!strncmp(t->comm,"msm-core:sampli", TASK_COMM_LEN)||
		!strncmp(t->comm,"kworker/u16:1", TASK_COMM_LEN) ||
		!strncmp(t->comm,"mdss_fb0", TASK_COMM_LEN)||
		!strncmp(t->comm,"mdss_fb_ffl0", TASK_COMM_LEN)||
		!strncmp(t->comm,"crtc_commit", DISP_TASK_COMM_LEN_MASK)||
		!strncmp(t->comm,"crtc_event", DISP_TASK_COMM_LEN_MASK)){
		return;
	}
	#endif

	/*
	 * Ensure the task is not frozen.
	 * Also, skip vfork and any other user process that freezer should skip.
	 */
	if (unlikely(t->flags & (PF_FROZEN | PF_FREEZER_SKIP)))
	#if defined(VENDOR_EDIT) && defined(CONFIG_DEATH_HEALER)
	{
		if (is_zygote_process(t) || is_fsck_process(t) || !strncmp(t->comm,"system_server", TASK_COMM_LEN)
			|| !strncmp(t->comm,"surfaceflinger", TASK_COMM_LEN)) {
			if (t->flags & PF_FROZEN)
				return;
		}
		else
			return;
	}
	#else
		return;
	#endif

	/*
	 * When a freshly created task is scheduled once, changes its state to
	 * TASK_UNINTERRUPTIBLE without having ever been switched out once, it
	 * musn't be checked.
	 */
	if (unlikely(!switch_count))
		return;

	if (switch_count != t->last_switch_count) {
		t->last_switch_count = switch_count;
		return;
	}

	trace_sched_process_hang(t);

	#if defined(VENDOR_EDIT) && defined(CONFIG_DEATH_HEALER)
	//if this task blocked at iowait. so maybe we should reboot system first
	if(t->in_iowait && !is_fsck_process(t)) {
		printk(KERN_ERR "DeathHealer io wait too long time\n");
		*iowait_count = *iowait_count + 1;
	}
	if (is_zygote_process(t) || is_fsck_process(t) || !strncmp(t->comm,"system_server", TASK_COMM_LEN)
		|| !strncmp(t->comm,"surfaceflinger", TASK_COMM_LEN) ) {
		if (t->state == TASK_UNINTERRUPTIBLE)
			snprintf(sysctl_hung_task_oppo_kill, HUNG_TASK_OPPO_KILL_LEN, "%s,uninterruptible for %ld seconds", t->comm, timeout);
		else if (t->state == TASK_STOPPED)
			snprintf(sysctl_hung_task_oppo_kill, HUNG_TASK_OPPO_KILL_LEN, "%s,stopped for %ld seconds by %s", t->comm, timeout, last_stopper_comm);
		else if (t->state == TASK_TRACED)
			snprintf(sysctl_hung_task_oppo_kill, HUNG_TASK_OPPO_KILL_LEN, "%s,traced for %ld seconds", t->comm, timeout);
		else
			snprintf(sysctl_hung_task_oppo_kill, HUNG_TASK_OPPO_KILL_LEN, "%s,unknown hung for %ld seconds", t->comm, timeout);

		printk(KERN_ERR "DeathHealer: task %s:%d blocked for more than %ld seconds in state 0x%lx. Count:%d\n",
			t->comm, t->pid, timeout, t->state, death_count+1);

		death_count++;
		cur_death_time = local_clock();
		if (death_count >= MAX_DEATH_COUNT) {
			if (cur_death_time - last_death_time < TWICE_DEATH_PERIOD) {
				if (!is_fsck_process(t)) {
					printk(KERN_ERR "DeathHealer task %s:%d has been triggered %d times, \
						last time at: %llu\n", t->comm, t->pid, death_count, last_death_time);
					BUG();
				} else {
					printk(KERN_ERR "DeathHealer task %s:%d has been triggered %d times, \
						last time at: %llu  (is_fsck_process ignore)\n", t->comm, t->pid, death_count, last_death_time);
				}
			}
		}
		last_death_time = cur_death_time;

		#ifdef CONFIG_OPPO_SPECIAL_BUILD
		BUG();
		#else
		sched_show_task(t);
		debug_show_held_locks(t);
		trigger_all_cpu_backtrace();

		t->flags |= PF_OPPO_KILLING;
		do_send_sig_info(SIGKILL, SEND_SIG_FORCED, t, true);
		wake_up_process(t);
		#endif
	}
	#endif

	if (!sysctl_hung_task_warnings && !sysctl_hung_task_panic)
		return;

	/*
	 * Ok, the task did not get scheduled for more than 2 minutes,
	 * complain:
	 */
	if (sysctl_hung_task_warnings && !is_fsck_process(t)) {
		sysctl_hung_task_warnings--;
		pr_err("INFO: task %s:%d blocked for more than %ld seconds.\n",
			t->comm, t->pid, timeout);
		pr_err("      %s %s %.*s\n",
			print_tainted(), init_utsname()->release,
			(int)strcspn(init_utsname()->version, " "),
			init_utsname()->version);
		pr_err("\"echo 0 > /proc/sys/kernel/hung_task_timeout_secs\""
			" disables this message.\n");
		sched_show_task(t);
		debug_show_all_locks();
	}

	touch_nmi_watchdog();

	if (sysctl_hung_task_panic) {
	#ifdef VENDOR_EDIT
		if (is_zygote_process(t) || !strncmp(t->comm,"system_server", TASK_COMM_LEN)
			|| !strncmp(t->comm,"surfaceflinger", TASK_COMM_LEN)) {
			if (!is_fsck_process(t)) {
				trigger_all_cpu_backtrace();
				panic("hung_task: blocked tasks");
			} else {
				pr_info("hung_task: blocked tasks: task %s:%d  (is_fsck_process ignore).\n", t->comm, t->pid);
			}
		}
	#endif
	}
}

/*
 * To avoid extending the RCU grace period for an unbounded amount of time,
 * periodically exit the critical section and enter a new one.
 *
 * For preemptible RCU it is sufficient to call rcu_read_unlock in order
 * to exit the grace period. For classic RCU, a reschedule is required.
 */
static bool rcu_lock_break(struct task_struct *g, struct task_struct *t)
{
	bool can_cont;

	get_task_struct(g);
	get_task_struct(t);
	rcu_read_unlock();
	cond_resched();
	rcu_read_lock();
	can_cont = pid_alive(g) && pid_alive(t);
	put_task_struct(t);
	put_task_struct(g);

	return can_cont;
}

/*
 * Check whether a TASK_UNINTERRUPTIBLE does not get woken up for
 * a really long time (120 seconds). If that happens, print out
 * a warning.
 */
static void check_hung_uninterruptible_tasks(unsigned long timeout)
{
	int max_count = sysctl_hung_task_check_count;
	int batch_count = HUNG_TASK_BATCHING;
	struct task_struct *g, *t;
	#if defined(VENDOR_EDIT) && defined(CONFIG_DEATH_HEALER)
	unsigned int iowait_count = 0;
	#endif

	/*
	 * If the system crashed already then all bets are off,
	 * do not report extra hung tasks:
	 */
	if (test_taint(TAINT_DIE) || did_panic)
		return;

	rcu_read_lock();
	for_each_process_thread(g, t) {
		if (!max_count--)
			goto unlock;
		if (!--batch_count) {
			batch_count = HUNG_TASK_BATCHING;
			if (!rcu_lock_break(g, t))
				goto unlock;
		}
		/* use "==" to skip the TASK_KILLABLE tasks waiting on NFS */
		#if defined(VENDOR_EDIT) && defined(CONFIG_DEATH_HEALER)
		if (t->state == TASK_UNINTERRUPTIBLE || t->state == TASK_STOPPED || t->state == TASK_TRACED)
			check_hung_task(t, timeout,&iowait_count);
		#else
		if (t->state == TASK_UNINTERRUPTIBLE)
			/* Check for selective monitoring */
			if (!sysctl_hung_task_selective_monitoring ||
			    t->hang_detection_enabled)
				check_hung_task(t, timeout);
		#endif
	}
 unlock:
	#if defined(VENDOR_EDIT) && defined(CONFIG_DEATH_HEALER)
	if(iowait_count >= MAX_IO_WAIT_HUNG){
		if(!is_fsck_process(t)) {
			panic("hung_task:%s=[%u]IO blocked too long time",t->comm, iowait_count);
		} else {
			pr_info("hung_task:%s=[%u]IO blocked too long time (is_fsck_process ignore).\n", t->comm,iowait_count);
		}
	}
	#endif
	rcu_read_unlock();
}

static long hung_timeout_jiffies(unsigned long last_checked,
				 unsigned long timeout)
{
	/* timeout of 0 will disable the watchdog */
	return timeout ? last_checked - jiffies + timeout * HZ :
		MAX_SCHEDULE_TIMEOUT;
}

/*
 * Process updating of timeout sysctl
 */
int proc_dohung_task_timeout_secs(struct ctl_table *table, int write,
				  void __user *buffer,
				  size_t *lenp, loff_t *ppos)
{
	int ret;

	ret = proc_doulongvec_minmax(table, write, buffer, lenp, ppos);

	if (ret || !write)
		goto out;

	wake_up_process(watchdog_task);

 out:
	return ret;
}

static atomic_t reset_hung_task = ATOMIC_INIT(0);

void reset_hung_task_detector(void)
{
	atomic_set(&reset_hung_task, 1);
}
EXPORT_SYMBOL_GPL(reset_hung_task_detector);

/*
 * kthread which checks for tasks stuck in D state
 */
static int watchdog(void *dummy)
{
	unsigned long hung_last_checked = jiffies;

	set_user_nice(current, 0);

	for ( ; ; ) {
		unsigned long timeout = sysctl_hung_task_timeout_secs;
		long t = hung_timeout_jiffies(hung_last_checked, timeout);

		if (t <= 0) {
			if (!atomic_xchg(&reset_hung_task, 0))
				check_hung_uninterruptible_tasks(timeout);
			hung_last_checked = jiffies;
			continue;
		}
		schedule_timeout_interruptible(t);
	}

	return 0;
}

static int __init hung_task_init(void)
{
	atomic_notifier_chain_register(&panic_notifier_list, &panic_block);
	watchdog_task = kthread_run(watchdog, NULL, "khungtaskd");

	return 0;
}
subsys_initcall(hung_task_init);
