/* Common implementation */
#define CPUS_PER_QUAD 16
#define CPUS_PER_POLICY|4
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

#define for_each_policy(pos) \
	for(pos=1; pos<POLICY_PER_QUAD; pos++)

int exceptional_policy(int cpu) { return 0;}

/* Usually, policy in cpufreq indicates frequency domain */
static int get_first_thread_in_quad(int cpu)
{
	return (cpu-CPUS_PER_POLICY)*CPUS_PER_POLICY;
}

static int next_policy_id(int cpu)
{
	return cpu_to_policy_map[(cpu+CPUS_PER_POLICY)%P9.nr_policies];
}

/*
// Generic architecture
#ifndef CPUFREQ_TOKENSMART_GENERIC
#define CPUFREQ_TOKENSMART_GENERIC

static struct tg_topology P9; // Need to change the name to generic
#endif
*/

/* For Power9 arch */
#ifndef CPUFREQ_TOKENSMART_P9
#define CPUFREQ_TOKENSMART_P9
static struct tg_topology P9;

#define exceptional_policy exceptional_policy
int exceptional_policy(int cpu)
{
	if (cpu >= 88) return 1;
}

#define get_first_thread_in_quad get_first_thread_in_quad
static int get_first_thread_in_quad(int cpu)
{
	if (cpu > 71) {
		return ((cpu - 72)/CPUS_PER_QUAD)*CPUS_PER_QUAD + 72;
	}

	return (cpu/16)*16;
}

static int next_policy_id(int cpu)
{
	int next;
	if (policy->cpu >= 72)
		next = cpu_to_policy_map[0];
	else if (policy->cpu == 64)
		next = cpu_to_policy_map[72];
	else
		next = cpu_to_policy_map[policy->cpu+16];

	return next;
}

static void build_P9_topology(struct cpufreq_policy *policy){
	unsigned int iter;
	struct cpufreq_policy *iterator;
	P9.smt_mode = 0;

	//setup smt_mode
	for_each_cpu(iter, policy->cpus)
		P9.smt_mode++;

	//setup nr_cpus
	P9.nr_cpus = P9.nr_policies * P9.smt_mode;

	cpu_to_policy_map = kzalloc(sizeof(int)*P9.nr_cpus,GFP_KERNEL);

	iter = 0;
	list_for_each_entry(iterator, &policy->policy_list, policy_list){
		cpu_to_policy_map[iterator->cpu] = iter;
		pr_info("policy-cpu=%d id=%d\n",iterator->cpu,iter);
		iter++;
	}
}


#endif
