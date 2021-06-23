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
#include <linux/percpu-defs.h>
#include "cpufreq_governor.h"
#include "cpufreq_tokensmart_arch.h"

/*
 * Initial value of tokenpool decides the power budget of the system.
 * pool_turn is used by each frequency-domain to find where the tokenPool has
 * reached. Each frequency-domain has unique id defined by policy_id to make
 * such decisions.
 */
static unsigned int tokenPool;
static unsigned int pool_turn;

static DEFINE_MUTEX(gov_dbs_tokenpool_mutex);
static unsigned int barrier;

/*
 * TokenSmart uses conservative approach to collect tokens. It starts from 1,
 * and then on each iteration it picks double the tokens than last time with a
 * capping defined by the ramp_up_limit tunable.
 */
const unsigned int ramp_up_limit = 32;

/*
 * A structure to keep persistent data across iterations.
 * tg_update() triggers periodically and uses this structure to store
 * information to be used in the next update cycle.
 */
struct tgdbs {
	/* Tokens acquired by the policy */
	unsigned int my_tokens;

	/* Ramp up freq giving factor */
	unsigned int last_ramp_up;
};

/* Keep array of tg_dbs per policy */
struct tgdbs * tg_data;

struct tg_policy_dbs_info {
	struct policy_dbs_info policy_dbs;
	unsigned int local;
};

static inline struct tg_policy_dbs_info *to_dbs_info(struct policy_dbs_info *policy_dbs)
{
	        return container_of(policy_dbs, struct tg_policy_dbs_info, policy_dbs);
}

struct avg_load_per_quad {
	unsigned int* load;
};

struct avg_load_per_quad* avg_load_per_quad;

/* Function to find maximum load among all CPUs */
unsigned int max_of(struct avg_load_per_quad avgload)
{
	int i = 1;
	unsigned int max_load = avgload.load[0];
	for_each_policy(i)
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
	unsigned int policy_id = get_policy_id(policy);
	unsigned int need_tokens;
	struct tgdbs *tgg  = &tg_data[policy_id];
	int first_thread_in_quad = get_first_thread(policy);

	min_f = policy->cpuinfo.min_freq;
	max_f = policy->cpuinfo.max_freq;

	/* No need to run tokensmart on other socket */
	if(exceptional_policy(policy)) {
		__cpufreq_driver_target(policy, max_f, CPUFREQ_RELATION_C);
		return;
	}

	avg_load_per_quad[first_thread_in_quad].load[(policy->cpu-first_thread_in_quad)/policies_per_fd] = load;

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

			if(tokenPool > need_tokens){//tokenPool has sufficient tokens
				tgg->my_tokens += need_tokens;
				tokenPool -= need_tokens;
			}
			else{//tokenPool has fewer token than required. so get all that available
				tgg->my_tokens += tokenPool;
				tgg->last_ramp_up += tokenPool;
				tokenPool = 0;
			}
	}

	/* 2. Communication Phase: Set pool_turn to next FD in the ring */
	pool_turn = next_policy_id(policy);

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
	return sprintf(buf, "tokenPool=%u, turn for policy %u total %u policies\n", tokenPool, pool_turn, topology.nr_policies);
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
	destroy_arch_topology();
	kfree(avg_load_per_quad);
	kfree(tg_data);
}

static void tg_start(struct cpufreq_policy *policy)
{
	int cpu;

	topology.nr_policies = 0;

	if(policy->cpu==0)
	{
		build_arch_topology(policy);

		tg_data = kzalloc(sizeof(struct tgdbs)*topology.nr_policies, GFP_KERNEL);

		avg_load_per_quad = kzalloc(sizeof(struct avg_load_per_quad)*topology.nr_cpus, GFP_KERNEL);
		for(cpu = 0; cpu < topology.nr_cpus; cpu++)
			avg_load_per_quad[cpu].load = (unsigned int*) kmalloc(sizeof(unsigned int) * policies_per_fd, GFP_KERNEL);

		pool_turn = 0;

		barrier=1;
	}
	while(barrier==0);

	tgdbs_policy(tg_data, policy).my_tokens = 0;
	tgdbs_policy(tg_data, policy).last_ramp_up = 0;
	pr_info("I'm cpu=%d policies=%d\n",policy->cpu, get_policy_id(policy));
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
