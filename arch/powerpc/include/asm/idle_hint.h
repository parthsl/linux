#ifndef _ASM_POWERPC_IDLEHINT_H
#define _ASM_POWERPC_IDLEHINT_H

#include <linux/kvm_host.h>

extern void kvmppc_idle_hint_set(struct kvm_vcpu *vcpu, int idle_hint);

extern int idle_hint_is_active;

static inline int prev_cpu_of_kvm(struct kvm_vcpu *vcpu)
{
	struct pid *pid;
	struct task_struct *task = NULL;

	rcu_read_lock();
	pid = rcu_dereference(vcpu->pid);
	if (pid)
		task = get_pid_task(pid, PIDTYPE_PID);
	rcu_read_unlock();

	if (!task)
		return -1;

	return task_cpu(task);
}

DECLARE_PER_CPU(struct list_head, idle_hint_subscribers);
DECLARE_PER_CPU(spinlock_t, idle_hint_subscribers_lock);

#endif
