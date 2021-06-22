#include "cpufreq_governor.h"

/* Common implementation */
#define CPUS_PER_QUAD 16
#define CPUS_PER_POLICY 4
#define POLICY_PER_QUAD (CPUS_PER_QUAD/CPUS_PER_POLICY)

/*
 * A structure to keep track of CPU topology
 * @smt_mode = Number of CPUs per core.
 * @nr_cpus = Total number of CPUs in the system.
 * @nr_policies = Total number of policies in the system. Traditionally, there
 * is one policy per core, but TokenSmart requires to have one policy per
 * frequency-domain (which can have more than one core also).
 */
struct tg_topology {
	unsigned int smt_mode;
	unsigned int nr_cpus;
	unsigned int nr_policies;
};

/*
 * policy_id can be different from cpu-id.
 * Hence this array can provide direct mapping from cpu-id to policy-id
 */
static int* cpu_to_policy_map;
static struct tg_topology topology;

#define for_each_policy(pos) \
	for(pos=1; pos<POLICY_PER_QUAD; pos++)

#define get_policy_id(policy) \
	cpu_to_policy_map[policy->cpu]

#define tgdbs_policy(tg_data, policy) \
	tg_data[get_policy_id(policy)]

int exceptional_policy(struct cpufreq_policy* policy) { return 0;}

/* Usually, policy in cpufreq indicates frequency domain */
static int get_first_thread(struct cpufreq_policy* policy)
{
	int cpu = policy->cpu;
	return (cpu-CPUS_PER_POLICY)*CPUS_PER_POLICY;
}

static int next_arch_policy_id(struct cpufreq_policy* policy)
{
	int cpu = policy->cpu;
	return cpu_to_policy_map[(cpu+CPUS_PER_POLICY)%topology.nr_policies];
}

/* For Power9 arch */
#ifndef CPUFREQ_TOKENSMART_P9
#define CPUFREQ_TOKENSMART_P9

#define exceptional_policy exceptional_p9_policy
int exceptional_p9_policy(struct cpufreq_policy* policy)
{
	if (policy->cpu >= 88) return 1;

	return 0;
}

#define get_first_thread get_first_thread_in_quad
static int get_first_thread_in_quad(struct cpufreq_policy* policy)
{
	int cpu = policy->cpu;
	if (cpu > 71) {
		return ((cpu - 72)/CPUS_PER_QUAD)*CPUS_PER_QUAD + 72;
	}

	return (cpu/16)*16;
}

#define next_arch_policy_id next_p9_policy_id
static int next_p9_policy_id(struct cpufreq_policy* policy)
{
	int next;
	int cpu = policy->cpu;
	if (cpu >= 72)
		next = cpu_to_policy_map[0];
	else if (cpu == 64)
		next = cpu_to_policy_map[72];
	else
		next = cpu_to_policy_map[cpu+16];

	return next;
}

#define build_arch_topology build_P9_topology
static void build_P9_topology(struct cpufreq_policy *policy){
	unsigned int iter;
	struct cpufreq_policy *iterator;
	topology.smt_mode = 0;

	//setup smt_mode
	for_each_cpu(iter, policy->cpus)
		topology.smt_mode++;

	//setup nr_cpus
	topology.nr_cpus = topology.nr_policies * topology.smt_mode;

	cpu_to_policy_map = kzalloc(sizeof(int)*topology.nr_cpus,GFP_KERNEL);

	iter = 0;
	list_for_each_entry(iterator, &policy->policy_list, policy_list){
		cpu_to_policy_map[iterator->cpu] = iter;
		pr_info("policy-cpu=%d id=%d\n",iterator->cpu,iter);
		iter++;
	}
}

#define destroy_arch_topology destroy_P9_topology
static void destroy_P9_topology(void)
{
	kfree(cpu_to_policy_map);
}
#endif
