#include <linux/perf_event.h>
#include <linux/percpu-defs.h>
#include "cpufreq_governor.h"

static DEFINE_PER_CPU(struct perf_event_attr, pea_obj);
static DEFINE_PER_CPU(struct perf_event*, pe);

struct perf_event_attr pea;
struct perf_event* pee;

static int init_perf_event(int cpu, enum perf_hw_id event)
{
	struct perf_event_attr pea_obj = per_cpu(pea_obj, cpu);
	struct perf_event *pe = per_cpu(pe, cpu);
	pea_obj.type = PERF_TYPE_HARDWARE;
	pea_obj.size = sizeof(struct perf_event_attr);
	pea_obj.config = event;
	pea_obj.disabled = 1;
	pea_obj.inherit = 1;
	pea_obj.exclude_guest = 1;

	pe = perf_event_create_kernel_counter(&pea_obj, cpu, NULL, NULL, NULL);
	/*

	if (!pe){
		pr_info("Failed to create perf event\n");
		return 0;
	}
	*/
	return 1;
}

static inline void enable_perf_event(int cpu)
{
	perf_event_enable(per_cpu(pe, cpu));
}

static inline long long int read_perf_event(int cpu)
{
	u64 enabled=0, running=0;

	return perf_event_read_value(per_cpu(pe, cpu), &enabled, &running);
}

static inline void free_perf_event(struct cpufreq_policy* policy)
{
	int cpu;
	for_each_cpu(cpu, policy->cpus)
		perf_event_release_kernel(per_cpu(pe,cpu));
	return;
}
