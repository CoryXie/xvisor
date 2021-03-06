/**
 * Copyright (c) 2014 Cory Xie.
 * All rights reserved.
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
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 *
 * @file brd_smp.c
 * @author Cory Xie (cory.xie@gmail.com)
 * @brief board specific smp functions
 */

#include <vmm_error.h>
#include <vmm_types.h>
#include <vmm_smp.h>
#include <vmm_stdio.h>
#include <vmm_host_io.h>
#include <vmm_host_irq.h>
#include <vmm_host_aspace.h>
#include <vmm_compiler.h>
#include <libs/libfdt.h>
#include <smp_scu.h>
#include <gic.h>

static virtual_addr_t scu_base;
static virtual_addr_t src_base;

/* SRC registers definition */

#define SRC_SCR                                 0x000
#define SRC_GPR1                                0x020

/* register bit definition */

#define SRC_SCR_WARM_RESET_ENABLE               (0x1 << 0)
#define SRC_SCR_CORE1_ENABLE_OFFSET             (22)
#define SRC_SCR_CORE0_RESET_OFFSET              (13)
#define SRC_SCR_MASK_WDOG_RST_DEFAULT           (0xa << 7)
#define SRC_SCR_MASK_WDOG_RST_MASK              (0xf << 7)

int __init arch_smp_init_cpus(void)
{
	u32 ncores;
	int i, rc = VMM_OK;
	struct vmm_devtree_node *node;

	/* Get the System Reset Controller (SRC) node in the dev tree */
	node = vmm_devtree_getnode(VMM_DEVTREE_PATH_SEPARATOR_STRING "src");
	if (!node) {
		return VMM_EFAIL;
	}

	/* Map the SRC physical address to virtual address */
	rc = vmm_devtree_regmap(node, &src_base, 0);
	if (rc) {
		return rc;
	}

	/* Get the SCU node in the dev tree */
	node = vmm_devtree_getnode(VMM_DEVTREE_PATH_SEPARATOR_STRING "scu");
	if (!node) {
		return VMM_EFAIL;
	}

	/* Map the SCU physical address to virtual address */
	rc = vmm_devtree_regmap(node, &scu_base, 0);
	if (rc) {
		return rc;
	}

	/* How many ARM core do we have */
	ncores = scu_get_core_count((void *)scu_base);
    
	/* Update the cpu_possible bitmap based on SCU */
	for (i = 0; i < CONFIG_CPU_COUNT; i++) {
		if (i < ncores) {
		    
			vmm_set_cpu_possible(i, TRUE);
		}
	}

	return VMM_OK;
}

extern u8 _start_secondary;

int __init arch_smp_prepare_cpus(unsigned int max_cpus)
{
	int i, rc;
	physical_addr_t _start_secondary_pa;

	/* Get physical address secondary startup code */
	rc = vmm_host_va2pa((virtual_addr_t)&_start_secondary, 
			    &_start_secondary_pa);
	if (rc) {
		return rc;
	}

	/* Update the cpu_present bitmap */
	for (i = 0; i < max_cpus; i++) {
		vmm_set_cpu_present(i, TRUE);

    	if (src_base) {
    		/* Write the entry address for the secondary cpus */
    		vmm_writel((u32)_start_secondary_pa, 
    		    (void *)src_base + SRC_GPR1 + (i * 8));
    	}
	}

	if (scu_base) {
		/* Enable snooping through SCU */
		scu_enable((void *)scu_base);
	}

	return VMM_OK;
}

int __init arch_smp_start_cpu(u32 cpu)
{
	const struct vmm_cpumask *mask = get_cpu_mask(cpu);
    u32 val;
    
    val = vmm_readl ((volatile void *)(src_base + SRC_SCR));
    val |= (1 << (SRC_SCR_CORE1_ENABLE_OFFSET + (cpu - 1)));
    vmm_writel (val, (volatile void *)(src_base + SRC_SCR));

	/* Wakeup target cpu from wfe/wfi by sending an IPI */
	gic_raise_softirq(mask, 0);

	return VMM_OK;
}

static vmm_irq_return_t smp_ipi_handler(int irq_no, void *dev)
{
	/* Call core code to handle IPI1 */
	vmm_smp_ipi_exec();	

	return VMM_IRQ_HANDLED;
}

void arch_smp_ipi_trigger(const struct vmm_cpumask *dest)
{
	/* Send IPI1 to other cores */
	gic_raise_softirq(dest, 1);
}

int __cpuinit arch_smp_ipi_init(void)
{
	int rc;
	u32 cpu = vmm_smp_processor_id();

	if (!cpu) {
		/* Register IPI1 interrupt handler */
		rc = vmm_host_irq_register(1, "IPI1", &smp_ipi_handler, NULL);
		if (rc) {
			return rc;
		}

		/* Mark IPI1 interrupt as per-cpu */
		rc = vmm_host_irq_mark_per_cpu(1);
		if (rc) {
			return rc;
		}
	}

	/* Explicitly enable IPI1 interrupt */
	gic_enable_ppi(1);

	return VMM_OK;
}
