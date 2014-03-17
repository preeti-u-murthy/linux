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

	return 0;
}

static struct freq_attr *powernv_cpu_freq_attr[] = {
	&cpufreq_freq_attr_scaling_available_freqs,
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
