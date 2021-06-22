#define CPUS_PER_FD 16

/* Power9 specific */
#define exceptional_policy exceptional_policy
int exceptional_policy(struct cpufreq_policy* policy)
{
	if (policy->cpu >= 88) return 1;

	return 0;
}

#define build_arch_topology build_P9_topology
static void build_P9_topology(struct cpufreq_policy *policy){
	unsigned int iter;
	struct cpufreq_policy *iterator;
	topology.cpus_per_policy = 0;

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

#define destroy_arch_topology destroy_P9_topology
static void destroy_P9_topology(void)
{
	kfree(cpu_to_policy_map);
}

#define get_first_thread get_first_thread
/* Usually, policy in cpufreq indicates frequency domain */
static int get_first_thread(struct cpufreq_policy* policy)
{
	int cpu = policy->cpu;
	if (cpu > 71) {
		return ((cpu - 72)/CPUS_PER_FD)*CPUS_PER_FD + 72;
	}

	return (cpu - CPUS_PER_FD) * CPUS_PER_FD;
}

#define next_policy_id next_policy_id
static int next_policy_id(struct cpufreq_policy* policy)
{
	int cpu = policy->cpu;
	return cpu_to_policy_map[(cpu + CPUS_PER_FD) % (topology.nr_policies/topology.cpus_per_policy)];
}
