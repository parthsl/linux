#include "cpufreq_governor.h"

/*
 * A structure to keep track of CPU topology
 * @nr_cpus = Total number of CPUs in the system.
 * @nr_policies = Total number of policies in the system. Traditionally, there
 * @cpus_per_policy = Number of CPUs per policy.
 * is one policy per core, but TokenSmart requires to have one policy per
 * frequency-domain (which can have more than one core also).
 */
struct tg_topology {
	unsigned int nr_cpus;
	unsigned int nr_policies;
	unsigned int cpus_per_policy;
};

/*
 * policy_id can be different from cpu-id.
 * Hence this array can provide direct mapping from cpu-id to policy-id
 */
static int* cpu_to_policy_map;
static struct tg_topology topology;
static unsigned int policies_per_fd = 1;

#define for_each_policy(pos) \
	for(pos = 1; pos < policies_per_fd; pos++)

#define get_policy_id(policy) \
	cpu_to_policy_map[policy->cpu]

#define tgdbs_policy(tg_data, policy) \
	tg_data[get_policy_id(policy)]

#include <asm/cpufreq_tokensmart.h>

#ifndef CPUS_PER_FD
#define CPUS_PER_FD 1
#endif

#ifndef exceptional_policy
#define exceptional_policy exceptional_policy
int exceptional_policy(struct cpufreq_policy* policy)
{
	return 0;
}
#endif

#ifndef get_first_thread
#define get_first_thread get_first_thread_in_quad
static int get_first_thread_in_quad(struct cpufreq_policy* policy)
{
	return (policy->cpu / CPUS_PER_FD) * CPUS_PER_FD;
}
#endif

#ifndef next_policy_id
#define next_policy_id next_policy_id
static int next_policy_id(struct cpufreq_policy* policy)
{
	int cpu = policy->cpu;
	return cpu_to_policy_map[cpu + CPUS_PER_FD];
}
#endif

#ifndef build_arch_topology
#define build_arch_topology build_arch_topology
static void build_arch_topology(struct cpufreq_policy *policy)
{
	unsigned int iter;
	struct cpufreq_policy *iterator;

	topology.cpus_per_policy = 0;
	topology.nr_policies = 0;

	// Find total policies in the systems
	list_for_each_entry(iterator, &policy->policy_list, policy_list){
		topology.nr_policies++;
	}

	//setup cpus_per_policy
	for_each_cpu(iter, policy->cpus)
		topology.cpus_per_policy++;

	// Define policies_per_fd
	policies_per_fd = CPUS_PER_FD / topology.cpus_per_policy;

	//setup nr_cpus
	topology.nr_cpus = topology.nr_policies * topology.cpus_per_policy;

	cpu_to_policy_map = kzalloc(sizeof(int)*topology.nr_cpus,GFP_KERNEL);

	iter = 0;
	list_for_each_entry(iterator, &policy->policy_list, policy_list){
		cpu_to_policy_map[iterator->cpu] = iter;
		pr_info("policy-cpu=%d id=%d\n",iterator->cpu,iter);
		iter++;
	}
}
#endif

#ifndef destroy_arch_topology
#define destroy_arch_topology destroy_P9_topology
static void destroy_P9_topology(void)
{
	kfree(cpu_to_policy_map);
}
#endif
