/*
 *  drivers/cpufreq/cpufreq_tokengov.c
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
#include "cpufreq_tokengov.h"

#define BUCKET_SIZE 10
#define PAST_MIPS_WEIGHT 8
#define CURRENT_MIPS_WEIGHT (10-PAST_MIPS_WEIGHT)
#define CPUS_PER_QUAD 16
#define TO_MS	1000000
/* MIPS_PERIOD in ms */
#define MIPS_PERIOD	100
#define MIPS_DROP_MARGIN	110
#define DROP_THRESHOLD	1

/* Boston system version, 9 or 16 */
const int bostonv = 9;

/* Scenario-4 specific tunables */
static unsigned int pool;
static unsigned int pool_turn;
enum pool_mode {GREEDY, FAIR} pool_mode;

const unsigned int starvation_threshold = 320000;
static unsigned int tokens_in_system;
static unsigned int fair_tokens;

static unsigned int npolicies;

static int scaledown = 9;
static int scaleup = 300;

static int debug;

static DEFINE_MUTEX(gov_dbs_tokenpool_mutex);

const unsigned int ramp_up_limit = 32;

struct tg_topology {
	unsigned int smt_mode;
	unsigned int nr_cpus;
	unsigned int nr_policies;
};

struct tgdbs {
	unsigned int my_tokens;
	unsigned int starvation;
	int set_fair_mode;
	u64	policy_mips;
	u64	policy_last_mips;
	u64	last_policy_mips;
	u64	last_instructions[CPUS_PER_QUAD];
	u64	instructions[CPUS_PER_QUAD];
	u64	timestamp[CPUS_PER_QUAD];
	u64	last_timestamp[CPUS_PER_QUAD];
	u64	mips[CPUS_PER_QUAD][BUCKET_SIZE];
	u64	cpu_mips[CPUS_PER_QUAD];
	int bucket_pointer[CPUS_PER_QUAD];
	u64	last_mips[CPUS_PER_QUAD];
	u64	mips_when_boosted;
	int	mips_updated;
	int taking_token;
	int	drop_threshold;
	int	is_dropped;
	unsigned long start,end;

	/* Ramp up freq giving factor */
	unsigned int last_ramp_up;
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

struct avg_load_per_quad {
	unsigned int load[4];
};

struct avg_load_per_quad* avg_load_per_quad;

unsigned int max_of(struct avg_load_per_quad avgload, int flag)
{
	int i = 1;
	unsigned int max_load = avgload.load[0];
	if(flag)
		trace_printk("llll%u:%u:%u:%u\n",avgload.load[0],avgload.load[1],avgload.load[2],avgload.load[3]);
	for(; i<4; i++)
		max_load = max_load < avgload.load[i] ? avgload.load[i]: max_load;

	return max_load;
}

void calc_mips(struct tgdbs *tgg, int cpu, int first_quad_cpu, int cpusperquad)
{
	u64 ips;
	u64 time_passed;
	int thread_index = cpu - first_quad_cpu;
	int iter = 0;
	u64 perf_instr = 0;

	if(thread_index>=cpusperquad)
		pr_info("bcz %d %d\n",cpu,first_quad_cpu);

	tgg->timestamp[thread_index] = mftb();
	time_passed = tgg->timestamp[thread_index] - tgg->last_timestamp[thread_index];
	time_passed /= TO_MS;
	if(time_passed<=0) time_passed = 1;
	if (time_passed < MIPS_PERIOD) return ;

	/* Read Perf counters */
	perf_instr = read_perf_event(cpu);
	tgg->instructions[thread_index] = perf_instr;

	ips = tgg->instructions[thread_index] - tgg->last_instructions[thread_index];
	

	tgg->last_mips[thread_index] = tgg->mips[thread_index][tgg->bucket_pointer[thread_index]];
	
	if(avg_load_per_quad[first_quad_cpu].load[thread_index/4]<10)
		tgg->mips[thread_index][tgg->bucket_pointer[thread_index]] = 0;
	else
		tgg->mips[thread_index][tgg->bucket_pointer[thread_index]] = (tgg->mips[thread_index][(tgg->bucket_pointer[thread_index]+BUCKET_SIZE-1)%BUCKET_SIZE]*PAST_MIPS_WEIGHT + ips*CURRENT_MIPS_WEIGHT)/10;
		//tgg->mips[thread_index][tgg->bucket_pointer[thread_index]] = ips;
	tgg->bucket_pointer[thread_index] = (tgg->bucket_pointer[thread_index]+1)%BUCKET_SIZE;

	if (debug && cpu == 0)	
	trace_printk("mlips=%llu time=%llu\n", ips, time_passed);
	
	ips = ips/time_passed;
	tgg->cpu_mips[thread_index] = (tgg->cpu_mips[thread_index]*PAST_MIPS_WEIGHT + ips*CURRENT_MIPS_WEIGHT)/10;
	if (debug )	
	trace_printk("cpu=%d mips = %llu ips=%llu time=%llu\n", cpu, tgg->cpu_mips[thread_index], ips, time_passed);
	/*
	for(iter = 0; iter<BUCKET_SIZE;  iter++)
		tgg->cpu_mips[thread_index] += tgg->mips[thread_index][iter]/BUCKET_SIZE;
	tgg->cpu_mips[thread_index] /= time_passed;
	*/
	tgg->last_instructions[thread_index] = tgg->instructions[thread_index];
	tgg->last_timestamp[thread_index] = tgg->timestamp[thread_index];
	tgg->mips_updated = 1;
}
void calc_policy_mips(struct tgdbs *tgg, int first_quad_cpu, int cpusperquad)
{
	int cpu;

	for (cpu=first_quad_cpu; cpu<(first_quad_cpu+cpusperquad); cpu++)
		calc_mips(tgg, cpu, first_quad_cpu, cpusperquad);

	tgg->policy_mips = tgg->cpu_mips[0];
	for (cpu=first_quad_cpu; cpu<(first_quad_cpu+cpusperquad); cpu++)
		tgg->policy_mips = tgg->policy_mips < tgg->cpu_mips[cpu-first_quad_cpu] ? tgg->cpu_mips[cpu-first_quad_cpu]:tgg->policy_mips;
}

static void tg_update(struct cpufreq_policy *policy)
{
	unsigned int load = dbs_update(policy);
	unsigned int required_tokens = 0;
	unsigned int freq_next, min_f, max_f;
	unsigned int policy_id = cpu_to_policy_map[policy->cpu];
	unsigned int need_tokens;
	struct tgdbs *tgg  = &tg_data[policy_id];
	int first_thread_in_quad = (policy->cpu/16)*16;
	int mips_increased = 0;
	unsigned long start,start2,end;
	u64 expected_mips, instruction_diff;

	//min_f = policy->cpuinfo.min_freq;
	//max_f = policy->cpuinfo.max_freq;
	min_f = 2166000;	
	max_f = 3800000;

	start = mftb();
	//trace_printk("cpu=%d start time=%lu\n",policy->cpu, start);
	//trace_printk("cpu=%d laststartdiff=%lu\n",policy->cpu, start-tgg->start);
	tgg->start = start;

	if (bostonv == 9)
	{
		/* No need to run tokengov on other socket */
		if (policy->cpu >= 88) {
			__cpufreq_driver_target(policy, max_f, CPUFREQ_RELATION_C);
			return;
		}
		//if (policy->cpu >= 48) return;
		if (policy->cpu > 71)
			first_thread_in_quad = ((policy->cpu - 72)/16)*16 + 72;
	}
	//trace_printk("cpu=%d first_thread in quad=%d\n",policy->cpu, first_thread_in_quad);
	avg_load_per_quad[first_thread_in_quad].load[(policy->cpu-first_thread_in_quad)/4] = load;

	// Token passing is for only first thread in quad
	if(policy->cpu != first_thread_in_quad) {
		__cpufreq_driver_target(policy, min_f, CPUFREQ_RELATION_C);
		return;
	}

	start2 = mftb()-start;
	//trace_printk("cpu=%d start for pool time=%lu\n",policy->cpu, start2);
	//trace_printk("cpu=%d lastenddiff=%lu\n",policy->cpu, start-tgg->end);
	
	// should be called by first quad cpu only
	// which goes and calculates for each cpu in that quad


	if (bostonv == 9 && policy->cpu==64)
		calc_policy_mips(tgg, first_thread_in_quad, 4);
	else
		calc_policy_mips(tgg, first_thread_in_quad, 4);
	
	//if ( policy->cpu == 0 )//&& tgg->last_policy_mips > 10*tgg->policy_mips )
	//	trace_printk("last MIPS=%llu current MIPS=%llu\n",tgg->last_policy_mips, tgg->policy_mips);

	load = max_of(avg_load_per_quad[first_thread_in_quad],0);

	/* Calculate the next frequency proportional to load */


	required_tokens = load;

	/* Don't perform any action for MIPS_PERIOD if we want tokens as we dont have any MIPS info */
	if (!tgg->mips_updated && required_tokens >= tgg->my_tokens) return ;
	tgg->mips_updated = 0;
	if (pool_turn != policy_id) return ;

	/* In case of increase in load, check if MIPS also increased
	 * If MIPS is not increasing then possibly the wirkload is 
	 * frequency insensitive and hence dont accept/donate tokens
	 */
	instruction_diff = (17000*tgg->last_ramp_up)*2/4; //instruction_diff is really MIPS diff
	expected_mips = tgg->mips_when_boosted + instruction_diff;
	expected_mips -= (instruction_diff*5/100); //keep 5% error margin

	if(tgg->taking_token) {
		if ( tgg->policy_mips > expected_mips) {
			mips_increased = 1;
			if ( policy->cpu == 0)
			trace_printk("dff: %llu %llu %u %llu\n",tgg->policy_mips, tgg->mips_when_boosted, tgg->last_ramp_up, expected_mips);
		}
		else {
			if ( policy->cpu == 0)
			trace_printk("regret: %llu %llu %u %llu\n",tgg->policy_mips, tgg->mips_when_boosted, tgg->last_ramp_up, expected_mips);
		}
	}
	if(tgg->taking_token && mips_increased)
		tgg->taking_token = 0;
	else if(tgg->taking_token && !mips_increased && required_tokens > tgg->my_tokens)
		required_tokens = tgg->my_tokens-1;
	else if (tgg->taking_token){
		tgg->taking_token = 0;
	}

	if ( tgg->policy_mips*MIPS_DROP_MARGIN < 100*tgg->policy_last_mips ) {
		if(!--tgg->drop_threshold) {
			tgg->is_dropped = 1;
			if ( policy->cpu == 0)
				trace_printk("dropped: %llu %llu\n",tgg->policy_mips, tgg->policy_last_mips);
		}
		
	} else {
		tgg->drop_threshold = DROP_THRESHOLD;
		if ( debug &&  policy->cpu == 0)
			trace_printk("not dropped: %llu %llu\n",tgg->policy_mips, tgg->policy_last_mips);
	}
	tgg->policy_last_mips = tgg->policy_mips;

	/*
	if(debug && policy->cpu==0)
		trace_printk("last_ramp=%u required_tokens=%u\n",tgg->last_ramp_up, required_tokens);
	*/

	if(debug && pool_turn==policy_id)
		trace_printk("my turn quad %d mips=%llu %u %u\n",policy->cpu, tgg->policy_mips, load, tgg->my_tokens);
	//else if(debug)
	//	trace_printk("quad %d mips=%llu %u %u\n",policy->cpu, tgg->policy_mips, load, tgg->my_tokens);

	/* Interaction Phase */
transaction:
	if (tgg->is_dropped) {
		required_tokens = tgg->my_tokens*0;
		tgg->is_dropped = 0;
	}

	if(required_tokens <= tgg->my_tokens){//donate
		pool += (tgg->my_tokens - required_tokens);
		tgg->my_tokens -= (tgg->my_tokens - required_tokens);
		tgg->taking_token = 0;
		if(tgg->my_tokens > 100)trace_printk("%u:::::::%u\n",tgg->my_tokens , required_tokens);
		tgg->last_ramp_up = 0;
	}
	else{
		if(pool==0) {//if not getting tokens for 32 loops then starve
			tgg->starvation++;
			if(tgg->starvation >= starvation_threshold) {
				pool_mode = FAIR;
				tgg->set_fair_mode = 1;
			}
		}
		else {
			//need_tokens = (required_tokens - tgg->my_tokens);
			need_tokens = tgg->last_ramp_up ? tgg->last_ramp_up*2 : 1;
			need_tokens = need_tokens > ramp_up_limit ? ramp_up_limit : need_tokens;

			if((required_tokens - tgg->my_tokens) < need_tokens)
				need_tokens = (required_tokens - tgg->my_tokens);

			tgg->last_ramp_up = need_tokens;

			if(pool > need_tokens){//pool has sufficient tokens
				tgg->my_tokens += need_tokens;
				if(tgg->my_tokens > 100)trace_printk("%u:::::::%u\n",tgg->my_tokens , need_tokens);
				pool -= need_tokens;
				tgg->mips_when_boosted = tgg->policy_mips;
			}
			else{//pool has fewer token than required. so get all that available
				tgg->my_tokens += pool;
				tgg->last_ramp_up += pool;
				if(tgg->my_tokens > 100)trace_printk("%u:::::::%u\n",tgg->my_tokens , pool);
				pool = 0;
				tgg->mips_when_boosted = tgg->policy_mips;
			}
			tgg->taking_token = 1;

			if(tgg->set_fair_mode==1){//reset to greedy mode once we go some tokens and reset starvation counter
				pool_mode = GREEDY;
				tgg->set_fair_mode = 0;
			}
			tgg->starvation = 0;
		}
	}
	if(pool_mode == FAIR && tgg->my_tokens > fair_tokens){
		pool += (tgg->my_tokens-fair_tokens);
		tgg->my_tokens -= (tgg->my_tokens - fair_tokens);
		if(tgg->my_tokens > 100)trace_printk("%u:::::::%u\n",tgg->my_tokens , fair_tokens);
	}

	//trace_printk("cpu %d pool_turn %d\n",policy->cpu, pool_turn);
	
	/* Communication Phase: Pass token pool to the next frequency-domain in the ring */
communication:
	if (bostonv == 16)
		pool_turn = (pool_turn+4)%npolicies;//+4 bcz jumping by 3 policies to next quad; policy is per core(or 4 SMTs)
	else {
		if (policy->cpu >= 72)
			pool_turn = cpu_to_policy_map[0];
		else if (policy->cpu == 64)
			pool_turn = cpu_to_policy_map[72];
		else
			pool_turn = cpu_to_policy_map[policy->cpu+16];
	}

set_frequency:
	/* Set new frequency based on avaialble tokens */
	freq_next = min_f + (tgg->my_tokens) * (max_f - min_f) / 100;
	__cpufreq_driver_target(policy, freq_next, CPUFREQ_RELATION_C);
	end = mftb() - start;
	//trace_printk("cpu=%d end time=%lu\n",policy->cpu, end);
	tgg->end = mftb();
	tgg->last_policy_mips = tgg->policy_mips;
}

static unsigned int tg_dbs_update(struct cpufreq_policy *policy)
{
	//trace_printk("cpu=%d tgdbs start time=%lu\n",policy->cpu,mftb());

	tg_update(policy);
	//trace_printk("cpu=%d tgdbs end=%lu\n",policy->cpu, mftb());

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

	if(input==0)debug = ~debug;
	if (input<0) {
		scaledown = -1*input;
		trace_printk("scaledown=%d\n",scaledown);
		return count;
	}

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
		trace_printk("policy id=%d:%d %llu\n",i,tg_data[i].my_tokens, tg_data[i].policy_mips);
	}


/*	
	for(i=0;i<P9.nr_cpus; i+=16)
	{
		trace_printk("quad policy=%d-%u:%u:%u:%u::::::%u %llu",i,avg_load_per_quad[i].load[0],avg_load_per_quad[i].load[1],avg_load_per_quad[i].load[2],avg_load_per_quad[i].load[3],max_of(avg_load_per_quad[i],0),tg_data[cpu_to_policy_map[i]].policy_mips);
	}	
*/	

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
	if(policy_dbs->policy)
		free_perf_event(policy_dbs->policy);
	kfree(to_dbs_info(policy_dbs));
}

static int tg_init(struct dbs_data *dbs_data)
{
	dbs_data->tuners = &pool;
	pool = 310; //Two can be at max freq
	tokens_in_system = pool;
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
	npolicies = P9.nr_policies;

	cpu_to_policy_map = kzalloc(sizeof(int)*P9.nr_cpus,GFP_KERNEL);

	iter = 0;
	list_for_each_entry(iterator, &policy->policy_list, policy_list){
		cpu_to_policy_map[iterator->cpu] = iter;
		printk("policy-cpu=%d id=%d\n",iterator->cpu,iter);
		iter++;
	}
}

static void tg_start(struct cpufreq_policy *policy)
{
	struct cpufreq_policy* iterator;
	int tmp;
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
		pool_mode = GREEDY;
		fair_tokens = tokens_in_system/P9.nr_policies;
		printk("Fair part=%u\n",fair_tokens);

		barrier=1;
	}
	while(barrier==0);
	tg_data[cpu_to_policy_map[policy->cpu]].set_fair_mode = 0;
	tg_data[cpu_to_policy_map[policy->cpu]].my_tokens = 0;
	tg_data[cpu_to_policy_map[policy->cpu]].last_ramp_up = 0;
	tg_data[cpu_to_policy_map[policy->cpu]].mips_updated = 0;
	tg_data[cpu_to_policy_map[policy->cpu]].drop_threshold = DROP_THRESHOLD;
	tg_data[cpu_to_policy_map[policy->cpu]].is_dropped = 0;
	pr_info("I'm cpu=%d policies=%d\n",policy->cpu, cpu_to_policy_map[policy->cpu]);
	/* Read perf based inctructions counter from hardware */
	
	for_each_cpu(tmp, policy->cpus)
	{
		init_perf_event(tmp);
		enable_perf_event(tmp);
	}
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
