/*
 *  drivers/cpufreq/cpufreq_tokensmart.c
 *
 *  Copyright (C)  2019 Parth Shah <parth@linux.ibm.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#define pr_fmt(fmt) KBUILD_MODNAME ": " fmt

#include <linux/cpu.h>
#include <linux/slab.h>
#include <linux/tick.h>
#include <linux/sched/cpufreq.h>

#define CPUS_PER_QUAD 16
#define CPUS_PER_POLICY	4
#define POLICY_PER_QUAD (CPUS_PER_QUAD/CPUS_PER_POLICY)

/*
 * Initial value of tokenpool decides the power budget of the system.
 * pool_turn is used by each frequency-domain to find where the tokenPool has
 * reached. Each frequency-domain has unique id defined by policy_id to make
 * such decisions.
 */
static unsigned int tokenPool;
static unsigned int pool_turn;
/* Tunable to set iterations after which token starvation is detected */
const unsigned int starvation_threshold = 32;
/*
 * Global variable used by every policy to determine if the tokenpool is in
 * GREEDY or FAIR mode. In the default GREEDY mode, each policy can take any
 * number tokens based on the requirements, but in FAIR mode there is upper
 * bound decided by the ratio of "total_tokens" to the "total policies".
 */
enum pool_mode {GREEDY, FAIR} pool_mode;
static unsigned int fair_tokens;

static DEFINE_MUTEX(gov_dbs_tokenpool_mutex);
static unsigned int barrier;

/*
 * TokenSmart uses conservative approach to collect tokens. It starts from 1,
 * and then on each iteration it picks double the tokens than last time with a
 * capping defined by the ramp_up_limit tunable.
 */
const unsigned int ramp_up_limit = 32;

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

static struct tg_topology P9;

/*
 * A structure to keep persistent data across iterations.
 * tg_update() triggers periodically and uses this structure to store
 * information to be used in the next update cycle.
 */
struct tgdbs {
	/* Tokens acquired by the policy */
	unsigned int my_tokens;

	/* Keep counts of times we did not received any token */
	unsigned int starvation;
	int set_fair_mode;

	/* Ramp up freq giving factor */
	unsigned int last_ramp_up;
};

/* Keep array of tg_dbs per policy */
struct tgdbs * tg_data;

/*
 * policy_id can be different from cpu-id.
 * Hence this array can provide direct mapping from cpu-id to policy-id
 */
static int* cpu_to_policy_map;

struct tg_policy_dbs_info {
	struct policy_dbs_info policy_dbs;
	unsigned int local;
};

static inline struct tg_policy_dbs_info *to_dbs_info(struct policy_dbs_info *policy_dbs)
{
	        return container_of(policy_dbs, struct tg_policy_dbs_info, policy_dbs);
}

struct avg_load_per_quad {
	unsigned int load[POLICY_PER_QUAD];
};

struct avg_load_per_quad* avg_load_per_quad;

/* Function to find maximum load among all CPUs */
unsigned int max_of(struct avg_load_per_quad avgload)
{
	int i = 1;
	unsigned int max_load = avgload.load[0];
	for(; i<POLICY_PER_QUAD; i++)
		max_load = max_load < avgload.load[i] ? avgload.load[i]: max_load;

	return max_load;
}

/*
 * This function gets triggered periodically. It consists of four parts:
 * 1. Computation phase: Collect metrics like load or MIPS from all CPUs and
 * decide the token requirement
 * 2. Communication Phase: pass the token to the
 * next node
 * 3. Interaction phase: Request frequency change to the firmware
 */
static void tg_update(struct cpufreq_policy *policy)
{
	unsigned int load = dbs_update(policy);
	unsigned int required_tokens = 0;
	unsigned int freq_next, min_f, max_f;
	unsigned int policy_id = cpu_to_policy_map[policy->cpu];
	unsigned int need_tokens;
	struct tgdbs *tgg  = &tg_data[policy_id];
	int first_thread_in_quad = (policy->cpu/16)*16;

	min_f = policy->cpuinfo.min_freq;
	max_f = policy->cpuinfo.max_freq;

	/* No need to run tokensmart on other socket */
	if (policy->cpu >= 88) {
		__cpufreq_driver_target(policy, max_f, CPUFREQ_RELATION_C);
		return;
	}
	//if (policy->cpu >= 48) return;
	if (policy->cpu > 71)
		first_thread_in_quad = ((policy->cpu - 72)/CPUS_PER_QUAD)*CPUS_PER_QUAD + 72;

	avg_load_per_quad[first_thread_in_quad].load[(policy->cpu-first_thread_in_quad)/POLICY_PER_QUAD] = load;

	// Token passing is for only first thread in quad
	if(policy->cpu != first_thread_in_quad) {
		__cpufreq_driver_target(policy, min_f, CPUFREQ_RELATION_C);
		return;
	}

	/* 1. Computation Phase */
	load = max_of(avg_load_per_quad[first_thread_in_quad]);


	/* Calculate the next frequency proportional to load */
	required_tokens = load;

	if (pool_turn != policy_id) return ;

	/* Donate extra tokens */
	if(required_tokens <= tgg->my_tokens){
		tokenPool += (tgg->my_tokens - required_tokens);
		tgg->my_tokens -= (tgg->my_tokens - required_tokens);
		tgg->last_ramp_up = 0;
	}
	/* Accept tokens from the tokenPool */
	else{
			/* Be conservative in nature. Slowly gather tokens */
			need_tokens = tgg->last_ramp_up ? tgg->last_ramp_up*2 : 1;
			need_tokens = need_tokens > ramp_up_limit ? ramp_up_limit : need_tokens;

			if((required_tokens - tgg->my_tokens) < need_tokens)
				need_tokens = (required_tokens - tgg->my_tokens);

			tgg->last_ramp_up = need_tokens;

			if (tokenPool == 0) {
				tgg->starvation++;
				if (tgg->starvation >= starvation_threshold) {
					pool_mode = FAIR;
					tgg->set_fair_mode = 1;
				}

				goto abide_fairness;
			}

			if(tokenPool > need_tokens){//tokenPool has sufficient tokens
				tgg->my_tokens += need_tokens;
				tokenPool -= need_tokens;
			}
			else{//tokenPool has fewer token than required. so get all that available
				tgg->my_tokens += tokenPool;
				tgg->last_ramp_up += tokenPool;
				tokenPool = 0;
			}

			/*
			 * We have acquired some tokens, so reset pool mode to
			 * GREEDY and also reset the tgg->starvation counter.
			 */
			if (tgg->set_fair_mode == 1 &&
			   (tgg->my_tokens >= fair_tokens ||
			    tgg->my_tokens >= required_tokens))
			{
				pool_mode = GREEDY;
				tgg->set_fair_mode = 0;
			}
			tgg->starvation = 0;
	}

abide_fairness:
	/* relinquish any extra tokens when FAIR mode is on */
	if (pool_mode == FAIR && tgg->my_tokens > fair_tokens) {
		tokenPool += (tgg->my_tokens - fair_tokens);
		tgg->my_tokens -= (tgg->my_tokens - fair_tokens);
	}

	/* 2. Communication Phase: Set pool_turn to next FD in the ring */
	if (policy->cpu >= 72)
		pool_turn = cpu_to_policy_map[0];
	else if (policy->cpu == 64)
		pool_turn = cpu_to_policy_map[72];
	else
		pool_turn = cpu_to_policy_map[policy->cpu+16];

	/* 3. Interaction Phase: Set new frequency based on avaialble tokens */
	freq_next = min_f + (tgg->my_tokens) * (max_f - min_f) / 100;
	__cpufreq_driver_target(policy, freq_next, CPUFREQ_RELATION_C);
}

static unsigned int tg_dbs_update(struct cpufreq_policy *policy)
{
	tg_update(policy);
	return 8000;
}

/************************** sysfs interface ************************/
static struct dbs_governor tg_dbs_gov;

static ssize_t store_central_pool(struct gov_attr_set *attr_set, const char *buf,
				size_t count)
{
	int input;
	int ret;

	ret = sscanf(buf, "%d", &input);
	if (ret != 1)
		return -EINVAL;

	mutex_lock(&gov_dbs_tokenpool_mutex);
	tokenPool += input;
	mutex_unlock(&gov_dbs_tokenpool_mutex);

	return count;
}

static ssize_t show_central_pool(struct gov_attr_set *attr_set, char *buf)
{
	int i;
	return sprintf(buf, "tokenPool=%u, turn for policy %u total %u policies\n", tokenPool, pool_turn, P9.nr_policies);
}


gov_attr_rw(central_pool);

static struct attribute *tg_attributes[] = {
	&central_pool.attr,
	NULL
};

/************************** sysfs end ************************/

static struct policy_dbs_info *tg_alloc(void)
{
	struct tg_policy_dbs_info *dbs_info;

	dbs_info = kzalloc(sizeof(*dbs_info), GFP_KERNEL);
	return dbs_info ? &dbs_info->policy_dbs : NULL;
}

static void tg_free(struct policy_dbs_info *policy_dbs)
{
	kfree(to_dbs_info(policy_dbs));
}

static int tg_init(struct dbs_data *dbs_data)
{
	dbs_data->tuners = &tokenPool;
	tokenPool = 500;
	barrier = 0;
	return 0;
}

static void tg_exit(struct dbs_data *dbs_data)
{
	kfree(cpu_to_policy_map);
	kfree(avg_load_per_quad);
	kfree(tg_data);
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

static void tg_start(struct cpufreq_policy *policy)
{
	struct cpufreq_policy* iterator;
	P9.nr_policies = 0;

	if(policy->cpu==0)
	{
		list_for_each_entry(iterator, &policy->policy_list, policy_list){
			P9.nr_policies++;
		}

		tg_data = kzalloc(sizeof(struct tgdbs)*P9.nr_policies, GFP_KERNEL);

		build_P9_topology(policy);

		avg_load_per_quad = kzalloc(sizeof(struct avg_load_per_quad)*P9.nr_cpus, GFP_KERNEL);
		pool_turn = 0;
		fair_tokens = tokenPool/(P9.nr_policies/4);
		pool_mode = GREEDY;

		barrier=1;
	}
	while(barrier==0);
	tg_data[cpu_to_policy_map[policy->cpu]].my_tokens = 0;
	tg_data[cpu_to_policy_map[policy->cpu]].last_ramp_up = 0;
	pr_info("I'm cpu=%d policies=%d\n",policy->cpu, cpu_to_policy_map[policy->cpu]);
}

static struct dbs_governor tg_dbs_gov = {
	.gov = CPUFREQ_DBS_GOVERNOR_INITIALIZER("tokensmart"),
	.kobj_type = { .default_attrs = tg_attributes },
	.gov_dbs_update = tg_dbs_update,
	.alloc = tg_alloc,
	.free = tg_free,
	.init = tg_init,
	.exit = tg_exit,
	.start = tg_start,
};

#define CPU_FREQ_GOV_TOKENGOV	(&tg_dbs_gov.gov)

static int __init cpufreq_gov_dbs_init(void)
{
	return cpufreq_register_governor(CPU_FREQ_GOV_TOKENGOV);
}

static void __exit cpufreq_gov_dbs_exit(void)
{
	cpufreq_unregister_governor(CPU_FREQ_GOV_TOKENGOV);
}

MODULE_AUTHOR("Parth Shah <pshah015@in.ibm.com>");
MODULE_DESCRIPTION("'cpufreq_tokensmart' - A dynamic cpufreq governor for "
	"scaling frequency using token passing algorithm");
MODULE_LICENSE("GPL");

#ifdef CONFIG_CPU_FREQ_DEFAULT_GOV_TOKENGOV
struct cpufreq_governor *cpufreq_default_governor(void)
{
	return CPU_FREQ_GOV_TOKENGOV;
}

fs_initcall(cpufreq_gov_dbs_init);
#else
module_init(cpufreq_gov_dbs_init);
#endif
module_exit(cpufreq_gov_dbs_exit);
