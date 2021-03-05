/*
 *  drivers/cpufreq/cpufreq_ondemand.c
 *
 *  Copyright (C)  2001 Russell King
 *            (C)  2003 Venkatesh Pallipadi <venkatesh.pallipadi@intel.com>.
 *                      Jun Nakajima <jun.nakajima@intel.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#define pr_fmt(fmt) KBUILD_MODNAME ": " fmt

#include <linux/cpu.h>
#include <linux/percpu-defs.h>
#include <linux/slab.h>
#include <linux/tick.h>
#include <linux/sched/cpufreq.h>
#include "cpufreq_governor.h"

/* Scenario-4 specific tunables */
static unsigned int pool;
static unsigned int pool_turn;
enum pool_mode {GREEDY, FAIR} pool_mode;

const unsigned int starvation_threshold = 32;
static unsigned int tokens_in_system;
static unsigned int fair_tokens;

static unsigned int npolicies;

static int debug;

static DEFINE_MUTEX(gov_dbs_tokenpool_mutex);

struct tg_topology {
	unsigned int smt_mode;
	unsigned int nr_cpus;
	unsigned int nr_policies;
};

struct tgdbs {
	unsigned int my_tokens;
	unsigned int starvation;
	int set_fair_mode;
};

struct tgdbs * tg_data;

static struct tg_topology P9;

static unsigned int barrier;

static int* cpu_to_policy_map;

struct tg_policy_dbs_info {
	struct policy_dbs_info policy_dbs;
	unsigned int local;
};

static inline struct tg_policy_dbs_info *to_dbs_info(struct policy_dbs_info *policy_dbs)
{
	        return container_of(policy_dbs, struct tg_policy_dbs_info, policy_dbs);
}

/*
 * Every sampling_rate, we check, if current idle time is less than 20%
 * (default), then we try to increase frequency. Else, we adjust the frequency
 * proportional to load.
 */
static void tg_update(struct cpufreq_policy *policy)
{
	unsigned int load = dbs_update(policy);
	unsigned int required_tokens = 0;
	unsigned int freq_next, min_f, max_f;
	unsigned int policy_id = cpu_to_policy_map[policy->cpu];
	unsigned int need_tokens;
	struct tgdbs *tgg = &tg_data[policy_id];

	/* Calculate the next frequency proportional to load */

	min_f = policy->cpuinfo.min_freq;
	max_f = policy->cpuinfo.max_freq;

	if(debug)
		printk("Cpu=%d token=%d\n",policy->cpu,tgg->my_tokens);

	required_tokens = load;

	//if token_pool reached to me, then only  i will doante/accept tokens
	if(pool_turn == policy_id){
		if(required_tokens < tgg->my_tokens){//donate
			pool += (tgg->my_tokens - required_tokens);
			tgg->my_tokens -= (tgg->my_tokens - required_tokens);
			if(tgg->my_tokens > 100)printk("%u:::::::%u\n",tgg->my_tokens , required_tokens);
		}
		else{
			if(pool==0){//if not getting tokens for 32 loops then starve
				tgg->starvation++;
				if(tgg->starvation >= starvation_threshold){
					pool_mode = FAIR;
					tgg->set_fair_mode = 1;
				}
			}
			else {
				need_tokens = (required_tokens - tgg->my_tokens);
				if(pool > need_tokens){//pool has sufficient tokens
					tgg->my_tokens += need_tokens;
					if(tgg->my_tokens > 100)printk("%u:::::::%u\n",tgg->my_tokens , need_tokens);
					pool -= need_tokens;
				}
				else{
					tgg->my_tokens += pool;
					if(tgg->my_tokens > 100)printk("%u:::::::%u\n",tgg->my_tokens , pool);
					pool = 0;
				}
				if(tgg->set_fair_mode==1){
					pool_mode = GREEDY;
					tgg->set_fair_mode = 0;
				}
				tgg->starvation = 0;
			}
		}
		if(pool_mode == FAIR && tgg->my_tokens > fair_tokens){
			pool += (tgg->my_tokens-fair_tokens);
			tgg->my_tokens -= (tgg->my_tokens - fair_tokens);
			if(tgg->my_tokens > 100)printk("%u:::::::%u\n",tgg->my_tokens , fair_tokens);
		}
		pool_turn = (pool_turn + 1)%npolicies;
	}
	
	/* Set new frequency based on avaialble tokens */
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
	unsigned int input;
	int ret;

	ret = sscanf(buf, "%u", &input);
	if (ret != 1)
		return -EINVAL;

	if(input==0)debug = ~debug;

	mutex_lock(&gov_dbs_tokenpool_mutex);
	pool += input;
	tokens_in_system += input;
	mutex_unlock(&gov_dbs_tokenpool_mutex);

	return count;
}

static ssize_t show_central_pool(struct gov_attr_set *attr_set, char *buf)
{
	int i;
	for(i=0;i<npolicies;i++)
	{
		printk("policy=%d:%d\n",i,tg_data[i].my_tokens );
	}
	return sprintf(buf, "pool=%u, turn for policy %u total %u policies\n", pool, pool_turn, npolicies);
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
	dbs_data->tuners = &pool;
	pool = 100; //Two can be at max freq
	tokens_in_system = pool;
	barrier = 0;
	return 0;
}

static void tg_exit(struct dbs_data *dbs_data)
{
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
	npolicies = P9.nr_policies;

	cpu_to_policy_map = kzalloc(sizeof(int),GFP_KERNEL);

	iter = 0;
	list_for_each_entry(iterator, &policy->policy_list, policy_list){
		cpu_to_policy_map[iterator->cpu] = iter;
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

		tg_data = kzalloc(sizeof(struct per_policy_ds*)*P9.nr_policies, GFP_KERNEL);

		build_P9_topology(policy);
		pool_turn = 0;
		pool_mode = GREEDY;
		fair_tokens = tokens_in_system/P9.nr_policies;
		printk("Fair part=%u\n",fair_tokens);
		barrier=1;
	}
	while(barrier==0);
	tg_data[cpu_to_policy_map[policy->cpu]].set_fair_mode = 0;
	tg_data[cpu_to_policy_map[policy->cpu]].my_tokens = 0;
	pr_info("I'm cpu=%d policies=%u\n",policy->cpu, npolicies);
}

static struct dbs_governor tg_dbs_gov = {
	.gov = CPUFREQ_DBS_GOVERNOR_INITIALIZER("tokengov"),
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
MODULE_DESCRIPTION("'cpufreq_tokengov' - A dynamic cpufreq governor for "
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
