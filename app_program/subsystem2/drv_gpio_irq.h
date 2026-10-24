/*
 *	drv_gpio_irq.h
 *	Subsystem 2 -- the ONE shared GPIO edge interrupt for the whole robot
 *	(IO_BANK0, IRQ 13). Owned by subsystem 2; other subsystems only
 *	register a branch for their own pin (TEAM.md interface IF-S2-GPIO).
 *
 *	Pins: encoders GP2/3/16/17 (S2), barcode edges GP7 (S3),
 *	      ultrasonic echo GP14 (S5).
 *
 *	Rules for every branch (Week 6 design / Tribunal 1):
 *	  - finish in under 2 us
 *	  - never wait; only count, timestamp, and tk_set_flg / tk_sig_sem
 *	  - write shared data with single whole-word stores
 */

#ifndef DRV_GPIO_IRQ_H
#define DRV_GPIO_IRQ_H

#include <stdint.h>
#include <stdbool.h>

#define GPIO_EVT_FALL	(0x4u)	/* edge low  */
#define GPIO_EVT_RISE	(0x8u)	/* edge high */

/* gpio: pin that fired; events: GPIO_EVT_* bits; levels: all input pins
   read once at IRQ entry; t_us: microsecond timer read once at IRQ entry */
typedef void (*gpio_branch_fn)(uint32_t gpio, uint32_t events,
			       uint32_t levels, uint32_t t_us);

/* Register a branch for one pin and enable both-edge interrupts on it.
   Call from your subsystemN_init() or task. */
void gpio_irq_register(uint32_t gpio, gpio_branch_fn fn);

/* Install the handler and enable IRQ 13 on THIS core. Called once by the
   motion task, which is pinned to core 1 (TP_PRC2), so the IRQ always runs
   on core 1. Safe to call more than once. */
int32_t gpio_irq_enable(void);

/* Microsecond timer (TIMERAWL), for timestamps outside the IRQ */
uint32_t gpio_irq_time_us(void);

/* TRUE once gpio_irq_enable() has succeeded (used by test S2-U01). */
bool gpio_irq_is_enabled(void);

#endif /* DRV_GPIO_IRQ_H */
