#ifndef _ASM_POWERPC_IDLEHINT_H
#define _ASM_POWERPC_IDLEHINT_H

extern void kvmppc_idle_hint_set(struct kvm_vcpu *vcpu, int idle_hint);

DECLARE_PER_CPU(struct list_head, idle_hint_subscribers);
DECLARE_PER_CPU(spinlock_t, idle_hint_subscribers_lock);

#endif
