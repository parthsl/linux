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


static unsigned int pool;
static int debug;

static DEFINE_MUTEX(gov_dbs_tokenpool_mutex);

struct thread_communication {
	int recv;
	int requested;
	struct mutex rwlock;
};

struct tg_policy_dbs_info {
	struct policy_dbs_info policy_dbs;
	unsigned int local;
};

struct per_policy_ds {
	unsigned int my_tokens;
	unsigned int spare_tokens;
	unsigned int forward_request_flag;
	unsigned int policy_id;
	struct thread_communication my_tc;
	struct thread_communication *left_neighbour;
	struct thread_communication *right_neighbour;
};

struct tg_topology {
	unsigned int smt_mode;
	unsigned int nr_cpus;
	unsigned int nr_policies;
};

static struct per_policy_ds *tg_data;
static struct tg_topology P9;

static int npolicies;
static unsigned int barrier;

static int* cpu_to_policy_map;

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
	unsigned int cpuid = policy->cpu;
	unsigned int policy_id = cpu_to_policy_map[cpuid];
	struct per_policy_ds *tgg = tg_data;

	struct per_policy_ds *tgdbs = &tgg[policy_id];
	int need_tokens;

	/* Calculate the next frequency proportional to load */

	min_f = policy->cpuinfo.min_freq;
	max_f = policy->cpuinfo.max_freq;

	if(policy->cpu==0){
		mutex_lock(&gov_dbs_tokenpool_mutex);
		tgdbs->my_tokens += pool;
		pool = 0;
		mutex_unlock(&gov_dbs_tokenpool_mutex);
	}
	if(debug)
		printk("Cpu=%d token=%d\n",policy->cpu, tgdbs->my_tokens);

	required_tokens = load;

	//take available tokens from thread_communication struct
	
	mutex_lock(&tgdbs->my_tc.rwlock);
	if(tgdbs->my_tc.recv > 0){
		tgdbs->spare_tokens+= tgdbs->my_tc.recv;
		tgdbs->my_tc.recv = 0;
		tgdbs->forward_request_flag = 0;
	}
	mutex_unlock(&tgdbs->my_tc.rwlock);

	if(required_tokens < tgdbs->my_tokens)//I have un-needed tokens. adding to spare
	{
		tgdbs->spare_tokens += (tgdbs->my_tokens - required_tokens);
		tgdbs->my_tokens -= (tgdbs->my_tokens-required_tokens);
		tgdbs->forward_request_flag = 0;
	}
	else if(required_tokens > tgdbs->my_tokens)//i want tokens
	{
		need_tokens = required_tokens - tgdbs->my_tokens;
		if(tgdbs->spare_tokens > 0){//and have spares
			if(tgdbs->spare_tokens>=need_tokens){
				tgdbs->spare_tokens -= need_tokens;
				tgdbs->my_tokens += need_tokens;
				need_tokens = 0;
			}
			else{//not having sufficient spares
				tgdbs->my_tokens += tgdbs->spare_tokens;
				need_tokens -= tgdbs->spare_tokens;
				tgdbs->spare_tokens = 0;
			}
		}

		if(need_tokens>0){//still needs some tokens. so requesting neighbour
			mutex_lock(&tgdbs->right_neighbour->rwlock);
			tgdbs->right_neighbour->requested += need_tokens;
			mutex_unlock(&tgdbs->right_neighbour->rwlock);
			tgdbs->forward_request_flag = 1;
		}
	}


	/* Set new frequency based on avaialble tokens */
	freq_next = min_f + (tgdbs->my_tokens) * (max_f - min_f) / 100;
	__cpufreq_driver_target(policy, freq_next, CPUFREQ_RELATION_C);

	/*
	 * Serving request of others
	 */

	mutex_lock(&tgdbs->my_tc.rwlock);
	need_tokens = tgdbs->my_tc.requested;
	mutex_unlock(&tgdbs->my_tc.rwlock);

	if(need_tokens > 0){
		if(tgdbs->spare_tokens>0){//have spare ones
			if(tgdbs->spare_tokens >= need_tokens){//and is sufficient

				mutex_lock(&tgdbs->left_neighbour->rwlock);
				tgdbs->left_neighbour->recv += need_tokens;
				mutex_unlock(&tgdbs->left_neighbour->rwlock);

				mutex_lock(&tgdbs->my_tc.rwlock);
				tgdbs->my_tc.requested -= need_tokens;
				mutex_unlock(&tgdbs->my_tc.rwlock);

				tgdbs->spare_tokens -= need_tokens;
				need_tokens = 0;
			}
			else{//in-sufficient spares

				mutex_lock(&tgdbs->left_neighbour->rwlock);
				tgdbs->left_neighbour->recv += tgdbs->spare_tokens;
				mutex_unlock(&tgdbs->left_neighbour->rwlock);

				mutex_lock(&tgdbs->my_tc.rwlock);
				tgdbs->my_tc.requested -= tgdbs->spare_tokens;
				mutex_unlock(&tgdbs->my_tc.rwlock);

				need_tokens -= tgdbs->spare_tokens;
				tgdbs->spare_tokens = 0;
			}
		}
		if(need_tokens>0){//still needs some tokens.
			if(tgdbs->forward_request_flag){//unable to server as I too want the tokens

				mutex_lock(&tgdbs->my_tc.rwlock);
				tgdbs->my_tc.requested = 0;
				mutex_unlock(&tgdbs->my_tc.rwlock);
			}
			else{//forwarding the request

				mutex_lock(&tgdbs->right_neighbour->rwlock);
				tgdbs->right_neighbour->requested += need_tokens;
				mutex_unlock(&tgdbs->right_neighbour->rwlock);
				tgdbs->forward_request_flag = true;
			}
		}
	}
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
	mutex_unlock(&gov_dbs_tokenpool_mutex);

	return count;
}

static ssize_t show_central_pool(struct gov_attr_set *attr_set, char *buf)
{
	int i=0, offset=0;

	for(i=0;i<P9.nr_policies;i++)
	{
		offset = sprintf(buf+offset, "%d", tg_data[i].my_tokens);
		pr_info("policy=%d:%d spare=%d\n",tg_data[i].policy_id, tg_data[i].my_tokens, tg_data[i].spare_tokens);
	}
	return sprintf(buf+offset, "%u\n", pool);
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
	dbs_data->tuners = tg_data;
	pool = 200; //Two can be at max freq
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
	P9.nr_cpus = npolicies * P9.smt_mode;
	P9.nr_policies = npolicies;

	cpu_to_policy_map = kzalloc(sizeof(int),GFP_KERNEL);

	iter = 0;
	list_for_each_entry(iterator, &policy->policy_list, policy_list){
		unsigned int policy_id = iterator->cpu/P9.smt_mode;
		cpu_to_policy_map[iterator->cpu] = iter;
		tg_data[iter].policy_id = iter;
		tg_data[policy_id].my_tc.recv = 0;
		tg_data[policy_id].my_tc.requested = 0;
		tg_data[policy_id].right_neighbour = &tg_data[(policy_id+1)%npolicies].my_tc;
		tg_data[policy_id].left_neighbour = &tg_data[(policy_id+npolicies-1)%npolicies].my_tc;
		tg_data[policy_id].my_tokens = 0;
		tg_data[policy_id].spare_tokens = 0;
		tg_data[policy_id].forward_request_flag = 0;
		iter++;
	}
}

static void tg_start(struct cpufreq_policy *policy)
{
	struct cpufreq_policy* iterator;
	int i;
	npolicies = 0;

	if(policy->cpu==0)
	{
		list_for_each_entry(iterator, &policy->policy_list, policy_list){
			npolicies++;
		}

		tg_data = kzalloc(sizeof(struct per_policy_ds*)*npolicies, GFP_KERNEL);

		build_P9_topology(policy);
		for(i=0;i<npolicies;i++)
		{
			mutex_init(&tg_data[i].my_tc.rwlock); 
		}

		barrier=1;
	}
	while(barrier==0);

	pr_info("I'm cpu =%d with policy_id=%d\n",policy->cpu, tg_data[cpu_to_policy_map[policy->cpu]].policy_id);
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
