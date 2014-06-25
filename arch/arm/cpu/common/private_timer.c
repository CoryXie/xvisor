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
 * @file mpcore_private_timer.c
 * @author Cory Xie (cory.xie@gmail.com)
 * @brief source file for ARM Cortex A9 per-cpu private timers.
 */

#include <vmm_types.h>
#include <vmm_host_io.h>
#include <vmm_host_irq.h>
#include <vmm_compiler.h>
#include <vmm_stdio.h>
#include <vmm_smp.h>
#include <vmm_clocksource.h>
#include <vmm_clockchip.h>
#include <vmm_wallclock.h>
#include <vmm_heap.h>
#include <vmm_error.h>
#include <vmm_delay.h>
#include <private_timer.h>

#include <linux/clk.h>

#define MIN_REG_COMPARE			(0x800)
#define MAX_REG_COMPARE			(0xfffffffe)

struct private_timer_clocksource {
	u32 cnt_high;
	u32 cnt_low;
	virtual_addr_t base;
	struct vmm_clocksource clksrc;
};

static u64 private_timer_clksrc_read(struct vmm_clocksource *cs)
{
	u32 temp;
	struct private_timer_clocksource *ecs = cs->priv;

	/*
	 * Get the current count. As the timer is decrementing we 
	 * invert the result.
	 */
	temp = ~ vmm_readl((void *)(ecs->base + PRIVATE_TIMER_REG_COUNTER));

	/*
	 * if the timer wrapped around we increase the high 32 bits part
	 * Note: the clock source is read fairly often and therefore it
	 *       should not be possible for the 32 bits counter to wrap
	 *       arround several time between 2 reads.
	 */
	if (temp < ecs->cnt_low) {
		ecs->cnt_high++;
	}

	ecs->cnt_low = temp;

	/*
	 * We can combine the two 32 bits couters to make a 64 bits
	 * counter.
	 */
	return (((u64) ecs->cnt_high) << 32) | ecs->cnt_low;
}

int __init private_timer_clocksource_init(void)
{
	int rc = VMM_ENODEV;
	u32 clock;
	struct vmm_devtree_node *node;
	struct private_timer_clocksource *ecs;

	/* find a private timer compatible node */
	node = vmm_devtree_find_compatible(NULL, NULL, "arm,a9mpcore-private-timer");
	if (!node) {
		goto fail;
	}

	/* Read clock frequency from node */
	rc = vmm_devtree_clock_frequency(node, &clock);
	if (rc) {
		goto fail;
	}

	/* allocate our struct */
	ecs = vmm_zalloc(sizeof(struct private_timer_clocksource));
	if (!ecs) {
		rc = VMM_ENOMEM;
		goto fail;
	}

	/* Map timer registers */
	rc = vmm_devtree_regmap(node, &ecs->base, 0);
	if (rc) {
		goto regmap_fail;
	}

	/* Setup clocksource */
	ecs->clksrc.name = node->name;
	ecs->clksrc.rating = 300;
	ecs->clksrc.read = private_timer_clksrc_read;
	ecs->clksrc.mask = VMM_CLOCKSOURCE_MASK(32);
	vmm_clocks_calc_mult_shift(&ecs->clksrc.mult,
				   &ecs->clksrc.shift,
				   clock, VMM_NSEC_PER_SEC, 10);
	ecs->clksrc.priv = ecs;

	/* Register clocksource */
	rc = vmm_clocksource_register(&ecs->clksrc);
	if (rc) {
		goto register_fail;
	}

	return VMM_OK;

 register_fail:
	vmm_devtree_regunmap(node, ecs->base, 0);
 regmap_fail:
	vmm_free(ecs);
 fail:
	return rc;
}

struct private_timer_clockchip {
	enum vmm_clockchip_mode clockevent_mode;
	virtual_addr_t base;
	struct vmm_clockchip clkchip;
};

struct private_timer_clockchip timer_clockchip[CONFIG_CPU_COUNT];

static inline void private_timer_irq_disable(struct private_timer_clockchip *ecc)
{
	u32 val;

	val = vmm_readl((void *)(ecc->base + PRIVATE_TIMER_REG_CONTROL));
	val &= ~PRI_TIMER_CTRL_ENABLE;
	vmm_writel(val, (void *)(ecc->base + PRIVATE_TIMER_REG_CONTROL));
}

static inline void private_timer_irq_enable(struct private_timer_clockchip *ecc)
{
	u32 val;

	val = vmm_readl((void *)(ecc->base + PRIVATE_TIMER_REG_CONTROL));
	val |= PRI_TIMER_CTRL_ENABLE;
	vmm_writel(val, (void *)(ecc->base + PRIVATE_TIMER_REG_CONTROL));
}

static void private_timer_irq_acknowledge(struct private_timer_clockchip *ecc)
{
	vmm_writel(PRI_TIMER_INT_STATUS_EVENT_FLAG, 
        (void *)(ecc->base + PRIVATE_TIMER_REG_INT_STATUS));
}

static int private_timer_set_next_event(unsigned long cycles, struct vmm_clockchip *evt)
{
	struct private_timer_clockchip *ecc = evt->priv;
    u32 ctrl;
    
	vmm_writel(cycles, (void *)(ecc->base + PRIVATE_TIMER_REG_LOAD));
    
    ctrl = vmm_readl((void *)(ecc->base + PRIVATE_TIMER_REG_CONTROL));
    
    vmm_writel(ctrl | PRI_TIMER_CTRL_ENABLE,
           (void *)(ecc->base + PRIVATE_TIMER_REG_CONTROL));

	return VMM_OK;
}

static void private_timer_set_mode(enum vmm_clockchip_mode mode,
			  struct vmm_clockchip *evt)
{
	struct private_timer_clockchip *ecc = evt->priv;
	unsigned long flags;

	/*
	 * The timer interrupt generation is disabled at least
	 * for enough time to call private_timer_set_next_event()
	 */
	arch_cpu_irq_save(flags);

	/* Disable interrupt */
	private_timer_irq_disable(ecc);

	if (mode != ecc->clockevent_mode) {
		/*
		 * Set event time into far-far future.
		 * The further we can go is to let the timer wrap arround
		 * once.
		 */
	
		vmm_writel(0xffffffff, (void *)(ecc->base + PRIVATE_TIMER_REG_LOAD));

		/* Clear pending interrupt */
		private_timer_irq_acknowledge(ecc);
	}

	/* Remember timer mode */
	ecc->clockevent_mode = mode;
	arch_cpu_irq_restore(flags);

	switch (mode) {
	case VMM_CLOCKCHIP_MODE_PERIODIC:
        vmm_printf("private_timer_set_mode - VMM_CLOCKCHIP_MODE_PERIODIC\n");
        vmm_writel(PRI_TIMER_CTRL_ENABLE |
                   PRI_TIMER_CTRL_AUTO_RELOAD |
                   PRI_TIMER_CTRL_IRQ_ENABLE,
               (void *)(ecc->base + PRIVATE_TIMER_REG_CONTROL));
		break;
	case VMM_CLOCKCHIP_MODE_ONESHOT:
        vmm_printf("private_timer_set_mode - VMM_CLOCKCHIP_MODE_ONESHOT\n");
		/*
		 * Do not put overhead of interrupt enable/disable into
		 * private_timer_set_next_event(), the core has about 4 minutes
		 * to call private_timer_set_next_event() or shutdown clock after
		 * mode switching
		 */
		arch_cpu_irq_save(flags);
        vmm_writel(PRI_TIMER_CTRL_ENABLE | 
                   PRI_TIMER_CTRL_IRQ_ENABLE,
               (void *)(ecc->base + PRIVATE_TIMER_REG_CONTROL));
		arch_cpu_irq_restore(flags);
		break;
	case VMM_CLOCKCHIP_MODE_SHUTDOWN:
	case VMM_CLOCKCHIP_MODE_UNUSED:
	case VMM_CLOCKCHIP_MODE_RESUME:
		/* Left event sources disabled, no more interrupts appear */
		break;
	}
}

/*
 * IRQ handler for the timer
 */
static vmm_irq_return_t private_timer_timer_interrupt(int irq, void *dev)
{
	struct private_timer_clockchip *ecc = dev;
    
	private_timer_irq_acknowledge(ecc);

	if (ecc->clkchip.event_handler)
        ecc->clkchip.event_handler(&ecc->clkchip);

	return VMM_IRQ_HANDLED;
}

int __cpuinit private_timer_clockchip_init(void)
{
	int rc = VMM_ENODEV;
	u32 clock, hirq, ctrl;
    u32 cpu = vmm_smp_processor_id();
	struct vmm_devtree_node *node;
	struct private_timer_clockchip *ecc;
    
	/* find the first private timer compatible node */
	node = vmm_devtree_find_compatible(NULL, NULL, "arm,a9mpcore-private-timer");
	if (!node) {
		goto fail;
	}
    
	/* Read clock frequency */
	rc = vmm_devtree_clock_frequency(node, &clock);
	if (rc) {
		goto fail;
	}

	/* Read irq attribute */
	rc = vmm_devtree_irq_get(node, &hirq, 0);
	if (rc) {
		goto fail;
	}    

	/* allocate our struct */
	ecc = &timer_clockchip[cpu];
    
    memset(ecc, 0, sizeof(struct private_timer_clockchip));
    
	/* Map timer registers */
	rc = vmm_devtree_regmap(node, &ecc->base, 0);
	if (rc) {
		goto regmap_fail;
	}

	/* Setup clockchip */
	ecc->clkchip.name = node->name;
	ecc->clkchip.hirq = hirq;
	ecc->clkchip.rating = 400;
	ecc->clkchip.cpumask = vmm_cpumask_of(vmm_smp_processor_id());
	ecc->clkchip.features = VMM_CLOCKCHIP_FEAT_ONESHOT/* | 
                            VMM_CLOCKCHIP_FEAT_PERIODIC*/;
	vmm_clocks_calc_mult_shift(&ecc->clkchip.mult,
				   &ecc->clkchip.shift,
				   VMM_NSEC_PER_SEC, clock, 10);
	ecc->clkchip.min_delta_ns = vmm_clockchip_delta2ns(MIN_REG_COMPARE,
							   &ecc->clkchip);
	ecc->clkchip.max_delta_ns = vmm_clockchip_delta2ns(MAX_REG_COMPARE,
							   &ecc->clkchip);
	ecc->clkchip.set_mode = private_timer_set_mode;
	ecc->clkchip.set_next_event = private_timer_set_next_event;
	ecc->clkchip.priv = ecc;

	/*
	 * Initialise to a known state (all timers off, and timing reset)
	 */
	vmm_writel(0x0, (void *)(ecc->base + PRIVATE_TIMER_REG_CONTROL));
	/*
	 * Initialize the load register to the max value to decrement.
	 */
	vmm_writel(0xffff, (void *)(ecc->base + PRIVATE_TIMER_REG_LOAD));
	vmm_writel(0xffff, (void *)(ecc->base + PRIVATE_TIMER_REG_COUNTER));
    
	/* Register interrupt handler */
	rc = vmm_host_irq_register(hirq, ecc->clkchip.name,
				   &private_timer_timer_interrupt, ecc);
	if (rc) {
		goto irq_fail;
	}

	/* Register clockchip */
	rc = vmm_clockchip_register(&ecc->clkchip);
	if (rc) {
		goto register_fail;
	}

	/*
	 * enable the timer, set it to the high reference clock
	 */
    ctrl = vmm_readl((void *)(ecc->base + PRIVATE_TIMER_REG_CONTROL));
    #if 1
    ctrl &= ~(PRI_TIMER_CTRL_AUTO_RELOAD);
    #else
    ctrl |= (PRI_TIMER_CTRL_AUTO_RELOAD);
    #endif
    ctrl |= PRI_TIMER_CTRL_IRQ_ENABLE;
	vmm_writel(ctrl, (void *)(ecc->base + PRIVATE_TIMER_REG_CONTROL));
    vmm_printf("private_timer_clockchip_init - OK\n");
	return VMM_OK;

 register_fail:
	vmm_host_irq_unregister(hirq, ecc);
 irq_fail:
	vmm_devtree_regunmap(node, ecc->base, 0);
 regmap_fail:
	vmm_free(ecc);
 fail:
	return rc;
}
