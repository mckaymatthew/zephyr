/*
 * Copyright (c) 2018 - 2019 Antmicro <www.antmicro.com>
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT litex_timer0

#include <kernel.h>
#include <arch/cpu.h>
#include <device.h>
#include <irq.h>
#include <spinlock.h>
#include <drivers/timer/system_timer.h>


#define TIMER_BASE	        DT_INST_REG_ADDR(0)
#define TIMER_LOAD_ADDR		((TIMER_BASE) + 0x00) //timer0_load
#define TIMER_RELOAD_ADDR	((TIMER_BASE) + 0x04) //timer0_reload
#define TIMER_EN_ADDR		((TIMER_BASE) + 0x08) //timer0_en
#define TIMER_EV_PENDING_ADDR	((TIMER_BASE) + 0x18) //timer0_ev_pending
#define TIMER_EV_ENABLE_ADDR	((TIMER_BASE) + 0x1c) //timer0_ev_enable
#define TIMER_TOTAL_UPDATE	((TIMER_BASE) + 0x0c) //timer0_update_value
#define TIMER_TOTAL		((TIMER_BASE) + 0x10) //timer0_value

#define TIMER_EV	0x1
#define TIMER_IRQ	DT_INST_IRQN(0)
#define TIMER_DISABLE	0x0
#define TIMER_ENABLE	0x1
#define UPDATE_TOTAL	0x1

static void litex_timer_irq_handler(const void *device)
{
	int key = irq_lock();

	sys_write8(TIMER_EV, TIMER_EV_PENDING_ADDR);
	sys_clock_announce(1);

	irq_unlock(key);
}

uint32_t sys_clock_cycle_get_32(void)
{
	static struct k_spinlock lock;
	uint32_t timer_total;
	k_spinlock_key_t key = k_spin_lock(&lock);

	litex_write8(UPDATE_TOTAL, TIMER_TOTAL_UPDATE);
	timer_total = (uint32_t)litex_read64(TIMER_TOTAL);

	k_spin_unlock(&lock, key);

	return timer_total;
}

/* tickless kernel is not supported */
uint32_t sys_clock_elapsed(void)
{
	return 0;
}

int sys_clock_driver_init(const struct device *dev)
{
	ARG_UNUSED(dev);
	IRQ_CONNECT(TIMER_IRQ, DT_INST_IRQ(0, priority),
			litex_timer_irq_handler, NULL, 0);
	irq_enable(TIMER_IRQ);

	//Disable the timer
	*((uint32_t*)(TIMER_EN_ADDR)) = TIMER_DISABLE;
// 	sys_write8(TIMER_DISABLE, TIMER_EN_ADDR);
	//Set the load and reload counts
	*((uint32_t*)(TIMER_RELOAD_ADDR)) = k_ticks_to_cyc_floor32(1);
	*((uint32_t*)(TIMER_LOAD_ADDR)) = k_ticks_to_cyc_floor32(1);
// 	sys_write32(k_ticks_to_cyc_floor32(1), TIMER_RELOAD_ADDR);
// 	sys_write32(k_ticks_to_cyc_floor32(1), TIMER_LOAD_ADDR);

// 	sys_write8(TIMER_ENABLE, TIMER_EN_ADDR);
	//Enable the timer
	*((uint32_t*)(TIMER_EN_ADDR)) = TIMER_ENABLE;

//
// 	//Clear all currently asserted events
	*((volatile uint32_t*)(TIMER_EV_PENDING_ADDR)) = *((volatile uint32_t*)(TIMER_EV_PENDING_ADDR));

// 	sys_write8(sys_read8(TIMER_EV_PENDING_ADDR), TIMER_EV_PENDING_ADDR);
// 	sys_write8(TIMER_EV, TIMER_EV_ENABLE_ADDR);
	//Enable the interrupt
	*((uint32_t*)(TIMER_EV_ENABLE_ADDR)) = TIMER_EV;

	return 0;
}
