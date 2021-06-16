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
#include "cpufreq_tokensmart_arch.h"
#include "cpufreq_tokensmart.h"

#define PAST_MIPS_WEIGHT 8
#define CURRENT_MIPS_WEIGHT (10 - PAST_MIPS_WEIGHT)
/* Threshold to determine the drop in MIPS. 110 -> 10% reduction in MIPS */
#define MIPS_DROP_MARGIN 110
#define DROP_THRESHOLD 5
/* Frequency at which MIPS is calculated */
#define MIPS_PERIOD 100
/* Factor to convert time from nano seconds to milli-seconds */
#define NS_TO_MS 1000000

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
 * MIPS Threshold above which a CPU is considered to be benefiting from higher
 * frequency.
 * In POWER9 systems, P-step size is 17MHz. Hence default=8500 indicates that a
 * workload having IPC of 0.5 or above is getting benefits of 1 P-state higher
 * frequency.
 */
static unsigned int IPC_threshold = 17000/2;
/*
 * Global variable used by every policy to determine if the tokenpool is in
 * GREEDY or FAIR mode. In the default GREEDY mode, each policy can take any
 * number tokens based on the requirements, but in FAIR mode there is upper
 * bound decided by the ratio of "total_tokens" to the "total policies".
 */
enum pool_mode {GREEDY, FAIR} pool_mode;
static unsigned int fair_tokens;

static DEFINE_MUTEX(gov_dbs_tokenpool_mutex);
static DEFINE_MUTEX(policy_mips_lock);
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

	/* Keep counts of times we did not received any token */
	unsigned int starvation;
	int set_fair_mode;

	/* Ramp up freq giving factor */
	unsigned int last_ramp_up;
	/*
	 * Few variables for calculating and stroing MIPS value.
	 * @policy.*mips: Keeps track of max MIPS among all CPUs in a policy.
	 * @.*instructions: instructions completed for each CPU.
	 * @.*timestamp: time passed across two consequtive iterations
	 */
	u64 policy_mips;
	u64 last_policy_mips;
	u64 last_instructions[CPUS_PER_FD];
	u64 instructions[CPUS_PER_FD];
	u64 timestamp[CPUS_PER_FD];
	u64 last_timestamp[CPUS_PER_FD];
	u64 cpu_mips[CPUS_PER_FD];

	/* Track if MIPS updated in last iteration */
	int mips_updated;
	u64 mips_when_boosted;
	int drop_threshold;
	int taking_token;
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

void calc_mips(struct tgdbs *tgg, int cpu, int tid)
{
	u64 ips;
	u64 time_passed;
	u64 perf_instr = 0;

	/* Store current timestamp and calculate time passed from last time. */
	tgg->timestamp[tid] = mftb();
	time_passed = tgg->timestamp[tid] - tgg->last_timestamp[tid];
	time_passed /= NS_TO_MS;

	/* Calculate MIPS only after defined period */
	if (time_passed < MIPS_PERIOD)
		return;

	/* Read perf instruction counters */
	perf_instr = read_perf_event(cpu);
	tgg->instructions[tid] = perf_instr;

	/* Calculate total instruction completed from last time */
	ips = perf_instr - tgg->last_instructions[tid];
	ips = ips/time_passed;

	/* Add current IPS value by decaying last known value */
	tgg->cpu_mips[tid] = (tgg->cpu_mips[tid] * PAST_MIPS_WEIGHT +
			      ips * CURRENT_MIPS_WEIGHT) / 10;

	tgg->last_instructions[tid] = tgg->instructions[tid];
	tgg->last_timestamp[tid] = tgg->timestamp[tid];
	tgg->mips_updated = 1;
}

void calc_policy_mips(struct tgdbs *tgg, int first_quad_cpu)
{
	int cpu;
	int tid = cpu - first_quad_cpu;

	mutex_lock(&policy_mips_lock);
	for (cpu = first_quad_cpu; cpu < (first_quad_cpu + CPUS_PER_FD); cpu++)
		calc_mips(tgg, cpu, tid);

	tgg->policy_mips = tgg->cpu_mips[0];
	for(cpu = first_quad_cpu; cpu < (first_quad_cpu + CPUS_PER_FD); cpu++)
		if (tgg->policy_mips < tgg->cpu_mips[tid])
			tgg->policy_mips = tgg->cpu_mips[tid];
	mutex_unlock(&policy_mips_lock);
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
	u64 mips_delta, expected_mips;

	min_f = policy->cpuinfo.min_freq;
	max_f = policy->cpuinfo.max_freq;

	/* No need to run tokensmart on other socket */
	if(exceptional_policy(policy)) {
		__cpufreq_driver_target(policy, max_f, CPUFREQ_RELATION_C);
		return;
	}

	avg_load_per_quad[first_thread_in_quad].load[(policy->cpu-first_thread_in_quad)/policies_per_fd] = load;

	/* Calculate MIPS value for this policy */
	calc_policy_mips(tgg, first_thread_in_quad);

	// Token passing is for only first thread in quad
	if(policy->cpu != first_thread_in_quad) {
		__cpufreq_driver_target(policy, min_f, CPUFREQ_RELATION_C);
		return;
	}

	/* 1. Computation Phase */
	load = max_of(avg_load_per_quad[first_thread_in_quad]);


	/* Calculate the next frequency proportional to load */
	required_tokens = load;

	/* Don't take any more tokens if time passed is less than MIPS_PERIOD */
	if (!tgg->mips_updated && required_tokens >= tgg->my_tokens)
		return;
	tgg->mips_updated = 0;

	if (pool_turn != policy_id) return ;

	/*
	 * Compute the expected MIPS value. if mips value if below this, then it
	 * is considered as no increase in MIPS and hence token should not be
	 * granted in such cases.
	 *
	 * Since last_ramp_up stores the amount of token accepted from the
	 * tokenPool, we can use it here to predict the increase in MIPS.
	 */
	mips_delta = (IPC_threshold * tgg->last_ramp_up);
	expected_mips = tgg->mips_when_boosted + mips_delta;
	expected_mips -= (mips_delta * 5) / 100; //keep 5% error margin

	/*
	 * Unless we grant a token, we cannot know if the workload is getting
	 * higher MIPS with frequency increase or not. Hence, we first grant a
	 * token and wait for increase in MIPS till next iteration.
	 *
	 * If we took token in last iteration, then find if MIPS is increased
	 * proportionally to the frequency increase or not. If not, then the
	 * granted token should be relinquished here.
	 */
	if (tgg->taking_token && tgg->policy_mips <= expected_mips)
		required_tokens = tgg->my_tokens - 1;
	tgg->taking_token = 0;

	/*
	 * In case of higher CPU load but decreasing MIPS value, the token
	 * should be relinquished. This is achieved by maintaining a
	 * drop_threshold, i.e., if a policy sees drop in MIPS for multiple
	 * consequtive iterations then it should drop tokens.
	 */
	if (tgg->policy_mips * MIPS_DROP_MARGIN < 100 * tgg->last_policy_mips) {
		if (!--tgg->drop_threshold)
			required_tokens = 0;
	} else {
		tgg->drop_threshold = DROP_THRESHOLD;
	}

	tgg->last_policy_mips = tgg->policy_mips;

	/* Interaction with tokenPool */
	/* Donate extra tokens */
	if(required_tokens <= tgg->my_tokens){
		tokenPool += (tgg->my_tokens - required_tokens);
		tgg->my_tokens -= (tgg->my_tokens - required_tokens);
		tgg->last_ramp_up = 0;
		tgg->taking_token = 0;
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

			tgg->taking_token = 1;
			tgg->mips_when_boosted = tgg->policy_mips;
	}

abide_fairness:
	/* relinquish any extra tokens when FAIR mode is on */
	if (pool_mode == FAIR && tgg->my_tokens > fair_tokens) {
		tokenPool += (tgg->my_tokens - fair_tokens);
		tgg->my_tokens -= (tgg->my_tokens - fair_tokens);
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

static ssize_t store_mips_threshold(struct gov_attr_set *attr_set, const char *buf,
				    size_t count)
{
	int ret;

	ret = sscanf(buf, "%d", &IPC_threshold);
	if (ret != 1)
		return -EINVAL;

	return count;
}

static ssize_t show_mips_threshold(struct gov_attr_set *attr_set, char *buf)
{
	return sprintf(buf, "MIPS Threshold = %d\n", IPC_threshold);
}

gov_attr_rw(central_pool);
gov_attr_rw(mips_threshold);

static struct attribute *tg_attributes[] = {
	&central_pool.attr,
	&mips_threshold.attr,
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
	if (policy_dbs->policy)
		free_perf_event(policy_dbs->policy);
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
		fair_tokens = tokenPool/(topology.nr_policies/4);
		pool_mode = GREEDY;

		/* Setup perf infrastructure to read Instructions completed */
		for_each_possible_cpu(cpu)
		{
			init_perf_event(cpu);
			enable_perf_event(cpu);
		}

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
