/*
 * POWERNV cpufreq driver for the IBM POWER processors
 *
 * (C) Copyright IBM 2014
 *
 * Author: Vaidyanathan Srinivasan <svaidy at linux.vnet.ibm.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2, or (at your option)
 * any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#define pr_fmt(fmt)	"powernv-cpufreq: " fmt

#include <linux/module.h>
#include <linux/cpufreq.h>
#include <linux/of.h>
#include <asm/cputhreads.h>

/* Per-Core locking for frequency transitions */
static DEFINE_PER_CPU(struct mutex, freq_switch_lock);

#define lock_core_freq(cpu)				\
			mutex_lock(&per_cpu(freq_switch_lock,\
				cpu_first_thread_sibling(cpu)));
#define unlock_core_freq(cpu)				\
			mutex_unlock(&per_cpu(freq_switch_lock,\
				cpu_first_thread_sibling(cpu)));

#define POWERNV_MAX_PSTATES	256

static struct cpufreq_frequency_table powernv_freqs[POWERNV_MAX_PSTATES+1];
static int powernv_pstate_ids[POWERNV_MAX_PSTATES+1];

struct powernv_pstate_info {
	int pstate_min_id;
	int pstate_max_id;
	int pstate_nominal_id;
	int nr_pstates;
};
static struct powernv_pstate_info powernv_pstate_info;

/*
 * Initialize the freq table based on data obtained
 * from the firmware passed via device-tree
 */

static int init_powernv_pstates(void)
{
	struct device_node *power_mgt;
	int nr_pstates = 0;
	int pstate_min, pstate_max, pstate_nominal;
	const __be32 *pstate_ids, *pstate_freqs;
	int i;
	u32 len_ids, len_freqs;

	power_mgt = of_find_node_by_path("/ibm,opal/power-mgt");
	if (!power_mgt) {
		pr_warn("power-mgt node not found\n");
		return -ENODEV;
	}

	if (of_property_read_u32(power_mgt, "ibm,pstate-min", &pstate_min)) {
		pr_warn("ibm,pstate-min node not found\n");
		return -ENODEV;
	}

	if (of_property_read_u32(power_mgt, "ibm,pstate-max", &pstate_max)) {
		pr_warn("ibm,pstate-max node not found\n");
		return -ENODEV;
	}

	if (of_property_read_u32(power_mgt, "ibm,pstate-nominal",
				 &pstate_nominal)) {
		pr_warn("ibm,pstate-nominal not found\n");
		return -ENODEV;
	}
	pr_info("cpufreq pstate min %d nominal %d max %d\n", pstate_min,
		pstate_nominal, pstate_max);

	pstate_ids = of_get_property(power_mgt, "ibm,pstate-ids", &len_ids);
	if (!pstate_ids) {
		pr_warn("ibm,pstate-ids not found\n");
		return -ENODEV;
	}

	pstate_freqs = of_get_property(power_mgt, "ibm,pstate-frequencies-mhz",
				      &len_freqs);
	if (!pstate_freqs) {
		pr_warn("ibm,pstate-frequencies-mhz not found\n");
		return -ENODEV;
	}

	WARN_ON(len_ids != len_freqs);
	nr_pstates = min(len_ids, len_freqs) / sizeof(u32);
	WARN_ON(!nr_pstates);

	pr_debug("NR PStates %d\n", nr_pstates);
	for (i = 0; i < nr_pstates; i++) {
		u32 id = be32_to_cpu(pstate_ids[i]);
		u32 freq = be32_to_cpu(pstate_freqs[i]);

		pr_debug("PState id %d freq %d MHz\n", id, freq);
		powernv_freqs[i].driver_data = i;
		powernv_freqs[i].frequency = freq * 1000; /* kHz */
		powernv_pstate_ids[i] = id;
	}
	/* End of list marker entry */
	powernv_freqs[i].driver_data = 0;
	powernv_freqs[i].frequency = CPUFREQ_TABLE_END;

	/* Print frequency table */
	for (i = 0; powernv_freqs[i].frequency != CPUFREQ_TABLE_END; i++)
		pr_debug("%d: %d\n", i, powernv_freqs[i].frequency);

	powernv_pstate_info.pstate_min_id = pstate_min;
	powernv_pstate_info.pstate_max_id = pstate_max;
	powernv_pstate_info.pstate_nominal_id = pstate_nominal;
	powernv_pstate_info.nr_pstates = nr_pstates;

	return 0;
}

/**
 * Returns the cpu frequency corresponding to the pstate_id.
 */
static unsigned int pstate_id_to_freq(int pstate_id)
{
	int i;

	i = powernv_pstate_info.pstate_max_id - pstate_id;

	BUG_ON(i >= powernv_pstate_info.nr_pstates || i < 0);
	WARN_ON(powernv_pstate_ids[i] != pstate_id);
	return powernv_freqs[i].frequency;
}

/**
 * show_cpuinfo_nominal_freq - Show the nominal CPU frequency as indicated by
 * the firmware
 */
static ssize_t show_cpuinfo_nominal_freq(struct cpufreq_policy *policy,
					char *buf)
{
	int nominal_freq;
	nominal_freq = pstate_id_to_freq(powernv_pstate_info.pstate_nominal_id);
	return sprintf(buf, "%u\n", nominal_freq);
}


struct freq_attr cpufreq_freq_attr_cpuinfo_nominal_freq = {
	.attr = { .name = "cpuinfo_nominal_freq",
		  .mode = 0444,
		},
	.show = show_cpuinfo_nominal_freq,
};


static struct freq_attr *powernv_cpu_freq_attr[] = {
	&cpufreq_freq_attr_scaling_available_freqs,
	&cpufreq_freq_attr_cpuinfo_nominal_freq,
	NULL,
};

/* Helper routines */

/**
 * Sets the bits corresponding to the thread-siblings of cpu in its core
 * in 'cpus'.
 */
static void powernv_cpu_to_core_mask(unsigned int cpu, cpumask_var_t cpus)
{
	int base, i;

	base = cpu_first_thread_sibling(cpu);

	for (i = 0; i < threads_per_core; i++) {
		cpumask_set_cpu(base + i, cpus);
	}

	return;
}

/* Access helpers to power mgt SPR */

static inline unsigned long get_pmspr(unsigned long sprn)
{
	switch (sprn) {
	case SPRN_PMCR:
		return mfspr(SPRN_PMCR);

	case SPRN_PMICR:
		return mfspr(SPRN_PMICR);

	case SPRN_PMSR:
		return mfspr(SPRN_PMSR);
	}
	BUG();
}

static inline void set_pmspr(unsigned long sprn, unsigned long val)
{
	switch (sprn) {
	case SPRN_PMCR:
		mtspr(SPRN_PMCR, val);
		return;

	case SPRN_PMICR:
		mtspr(SPRN_PMICR, val);
		return;

	case SPRN_PMSR:
		mtspr(SPRN_PMSR, val);
		return;
	}
	BUG();
}

/*
 * Computes the current frequency on this cpu
 * and stores the result in *ret_freq.
 */
static void powernv_read_cpu_freq(void *ret_freq)
{
	unsigned long pmspr_val;
	s8 local_pstate_id;
	int *cur_freq, freq, pstate_id;

	cur_freq = (int *)ret_freq;
	pmspr_val = get_pmspr(SPRN_PMSR);

	/* The local pstate id corresponds bits 48..55 in the PMSR.
         * Note: Watch out for the sign! */
	local_pstate_id = (pmspr_val >> 48) & 0xFF;
	pstate_id = local_pstate_id;

	freq = pstate_id_to_freq(pstate_id);
	pr_debug("cpu %d pmsr %lx pstate_id %d frequency %d \n",
		smp_processor_id(), pmspr_val, pstate_id, freq);
	*cur_freq = freq;
}

/*
 * Returns the cpu frequency as reported by the firmware for 'cpu'.
 * This value is reported through the sysfs file cpuinfo_cur_freq.
 */
unsigned int powernv_cpufreq_get(unsigned int cpu)
{
	int ret_freq;
	cpumask_var_t sibling_mask;

	if (unlikely(!zalloc_cpumask_var(&sibling_mask, GFP_KERNEL))) {
		smp_call_function_single(cpu, powernv_read_cpu_freq,
					&ret_freq, 1);
		return ret_freq;
	}

	powernv_cpu_to_core_mask(cpu, sibling_mask);
	smp_call_function_any(sibling_mask, powernv_read_cpu_freq,
			&ret_freq, 1);

	free_cpumask_var(sibling_mask);
	return ret_freq;
}

static void set_pstate(void *pstate)
{
	unsigned long val;
	unsigned long pstate_ul = *(unsigned long *) pstate;

	val = get_pmspr(SPRN_PMCR);
	val = val & 0x0000ffffffffffffULL;
	/* Set both global(bits 56..63) and local(bits 48..55) PStates */
	val = val | (pstate_ul << 56) | (pstate_ul << 48);
	pr_debug("Setting cpu %d pmcr to %016lX\n", smp_processor_id(), val);
	set_pmspr(SPRN_PMCR, val);
}

static int powernv_set_freq(cpumask_var_t cpus, unsigned int new_index)
{
	unsigned long val = (unsigned long) powernv_pstate_ids[new_index];

	/*
	 * Use smp_call_function to send IPI and execute the
	 * mtspr on target cpu.  We could do that without IPI
	 * if current CPU is within policy->cpus (core)
	 */

	val = val & 0xFF;
	smp_call_function_any(cpus, set_pstate, &val, 1);
	return 0;
}

static int powernv_cpufreq_cpu_init(struct cpufreq_policy *policy)
{
#ifdef CONFIG_SMP
	powernv_cpu_to_core_mask(policy->cpu, policy->cpus);
#endif
	policy->cpuinfo.transition_latency = 25000;

	policy->cur = powernv_freqs[0].frequency;
	cpufreq_frequency_table_get_attr(powernv_freqs, policy->cpu);
	return cpufreq_frequency_table_cpuinfo(policy, powernv_freqs);
}

static int powernv_cpufreq_cpu_exit(struct cpufreq_policy *policy)
{
	cpufreq_frequency_table_put_attr(policy->cpu);
	return 0;
}

static int powernv_cpufreq_verify(struct cpufreq_policy *policy)
{
	return cpufreq_frequency_table_verify(policy, powernv_freqs);
}

static int powernv_cpufreq_target(struct cpufreq_policy *policy,
			      unsigned int target_freq,
			      unsigned int relation)
{
	int rc;
	struct cpufreq_freqs freqs;
	unsigned int new_index;

	cpufreq_frequency_table_target(policy, powernv_freqs, target_freq,
				       relation, &new_index);

	freqs.old = policy->cur;
	freqs.new = powernv_freqs[new_index].frequency;
	freqs.cpu = policy->cpu;

	lock_core_freq(policy->cpu);
	cpufreq_notify_transition(policy, &freqs, CPUFREQ_PRECHANGE);

	pr_debug("setting frequency for cpu %d to %d kHz index %d pstate %d",
		 policy->cpu,
		 powernv_freqs[new_index].frequency,
		 new_index,
		 powernv_pstate_ids[new_index]);

	rc = powernv_set_freq(policy->cpus, new_index);

	cpufreq_notify_transition(policy, &freqs, CPUFREQ_POSTCHANGE);
	unlock_core_freq(policy->cpu);

	return rc;
}

static struct cpufreq_driver powernv_cpufreq_driver = {
	.verify		= powernv_cpufreq_verify,
	.target		= powernv_cpufreq_target,
	.get		= powernv_cpufreq_get,
	.init		= powernv_cpufreq_cpu_init,
	.exit		= powernv_cpufreq_cpu_exit,
	.name		= "powernv-cpufreq",
	.flags		= CPUFREQ_CONST_LOOPS,
	.attr		= powernv_cpu_freq_attr,
};

static int __init powernv_cpufreq_init(void)
{
	int cpu, rc = 0;

	/* Discover pstates from device tree and init */

	rc = init_powernv_pstates();

	if (rc) {
		pr_info("powernv-cpufreq disabled\n");
		return rc;
	}
	/* Init per-core mutex */
	for_each_possible_cpu(cpu) {
		mutex_init(&per_cpu(freq_switch_lock, cpu));
	}

	rc = cpufreq_register_driver(&powernv_cpufreq_driver);
	return rc;
}

static void __exit powernv_cpufreq_exit(void)
{
	cpufreq_unregister_driver(&powernv_cpufreq_driver);
}

module_init(powernv_cpufreq_init);
module_exit(powernv_cpufreq_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Vaidyanathan Srinivasan <svaidy at linux.vnet.ibm.com>");
