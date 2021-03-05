#include <linux/perf_event.h>
#include <linux/percpu-defs.h>
#include "cpufreq_governor.h"

static struct perf_event *pe[176];
static struct perf_event_attr pea[176];

static int init_perf_event(int cpu)
{
	struct perf_event_attr *pea_obj = &pea[cpu];

	pea_obj->type = PERF_TYPE_HARDWARE;
	pea_obj->size = sizeof(struct perf_event_attr);
	pea_obj->config = PERF_COUNT_HW_INSTRUCTIONS;
	pea_obj->disabled = 1;
	pea_obj->inherit = 1;
	pea_obj->exclude_guest = 1;

	pe[cpu] = perf_event_create_kernel_counter(pea_obj, cpu, NULL, NULL, NULL);

	if (!pe[cpu]){
		pr_info("Failed to create perf event for cpu %d\n", cpu);
		return 0;
	}
	return 1;
}

static inline void enable_perf_event(int cpu)
{
	perf_event_enable(pe[cpu]);
}

static inline long long int read_perf_event(int cpu)
{
	u64 enabled=0, running=0;

	return perf_event_read_value(pe[cpu], &enabled, &running);
}

static inline void free_perf_event(struct cpufreq_policy* policy)
{
	int cpu;
	//perf_event_disable(per_cpu(pe,0));
	//perf_event_release_kernel(per_cpu(pe,0));
	
	for_each_cpu(cpu, policy->cpus){
		perf_event_disable(pe[cpu]);
		perf_event_release_kernel(pe[cpu]);
	}
	
	return;
}
