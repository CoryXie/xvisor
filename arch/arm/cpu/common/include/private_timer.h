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
 * @file mpcore_private_timer.h
 * @author Cory Xie (cory.xie@gmail.com)
 * @brief API for ARM Cortex A9 per-cpu private timers
 */

#ifndef __PRIVATE_TIMER_H__
#define __PRIVATE_TIMER_H__

#include <vmm_types.h>

enum {
	PRIVATE_TIMER_REG_LOAD = 0x0,
	PRIVATE_TIMER_REG_COUNTER = 0x4,
	PRIVATE_TIMER_REG_CONTROL = 0x8,
	PRIVATE_TIMER_REG_INT_STATUS = 0xC,
    WATCHDOG_TIMER_REG_LOAD = 0x20,
    WATCHDOG_TIMER_REG_COUNTER = 0x24,
    WATCHDOG_TIMER_REG_CONTROL = 0x28,
    WATCHDOG_TIMER_REG_INT_STATUS = 0x2C,
    WATCHDOG_TIMER_REG_RESET_STATUS = 0x30,
    WATCHDOG_TIMER_REG_DISABLE = 0x34,
};

/* Private Timer Control Register bits */

#define PRI_TIMER_CTRL_ENABLE (1 << 0)
#define PRI_TIMER_CTRL_AUTO_RELOAD (1 << 1)
#define PRI_TIMER_CTRL_IRQ_ENABLE (1 << 2)
#define PRI_TIMER_CTRL_PRESCALE_MSK (0xFF << 8)
#define PRI_TIMER_CTRL_PRESCALE_OFF (8)

/* Private Timer Interrupt Status Register bits */

#define PRI_TIMER_INT_STATUS_EVENT_FLAG (1 << 0)

int private_timer_clocksource_init(void); 

int private_timer_clockchip_init(void);

#endif /* __PRIVATE_TIMER_H__ */

