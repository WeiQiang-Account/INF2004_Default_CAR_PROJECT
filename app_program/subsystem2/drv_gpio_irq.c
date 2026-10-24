/*
 *	drv_gpio_irq.c
 *	Subsystem 2 -- shared GPIO edge interrupt (IO_BANK0 -> NVIC IRQ 13).
 *
 *	APIs used: tk_def_int, DI / EI, in_w / out_w (NVIC set by word access)
 *
 *	The handler reads the time and all pin levels once, then for every
 *	pin with a pending edge: clears the edge, and calls the branch
 *	registered for that pin.
 */

#include "../app.h"
#include "drv_gpio_irq.h"

/* RP2040 IO_BANK0 interrupt registers (datasheet 2.19.6.1) */
#define IOB_INTR0		(IO_BANK0_BASE + 0x0F0)	/* raw, write 1 to clear edge */
/* The IRQ runs on core 1 (the core of the motion task, which calls
   gpio_irq_enable).  Core 0 is busy with the USB console and the WiFi
   polling task, which delayed the motion loop by up to a whole period. */
#define GPIO_IRQ_PRC		(2)			/* tk_get_prc(): 2 = core 1 */
#define IOB_INTE0		(IO_BANK0_BASE + 0x130)	/* PROC1_INTE0: enable for core 1 */
#define IOB_INTS0		(IO_BANK0_BASE + 0x150)	/* PROC1_INTS0: status for core 1 */
#define REG_ATOMIC_SET		(0x2000)		/* RP2040 set-alias offset */
#define REG_ATOMIC_CLR		(0x3000)		/* RP2040 clear-alias offset */

#define INTNO_IO_BANK0		(13)
#define GPIO_IRQ_INTPRI		(2)
#define GPIO_IRQ_PINS		(30)

LOCAL gpio_branch_fn	s_branch[GPIO_IRQ_PINS];
LOCAL volatile INT	s_enabled = 0;

uint32_t gpio_irq_time_us(void)
{
	return (uint32_t)in_w(TIMER_TIMERAWL);
}

LOCAL void gpio_irq_hdr(UINT intno)
{
	UW	t_us   = in_w(TIMER_TIMERAWL);
	UW	levels = in_w(GPIO_IN);
	UINT	reg;

	(void)intno;

	for (reg = 0; reg < 4; reg++) {
		UW ints = in_w(IOB_INTS0 + (reg * 4));

		while (ints != 0) {
			UINT bit   = (UINT)__builtin_ctz(ints);
			UINT slot  = bit / 4;			/* pin within register */
			UINT gpio  = (reg * 8) + slot;
			UW   ev    = (ints >> (slot * 4)) & 0xFu;

			/* clear this pin's edge bits (level bits are not used) */
			out_w(IOB_INTR0 + (reg * 4), (ev & 0xCu) << (slot * 4));
			ints &= ~(0xFu << (slot * 4));

			if ((gpio < GPIO_IRQ_PINS) && (s_branch[gpio] != NULL)) {
				s_branch[gpio](gpio, ev, levels, t_us);
			}
		}
	}
}

void gpio_irq_register(uint32_t gpio, gpio_branch_fn fn)
{
	UINT reg, slot;

	if (gpio >= GPIO_IRQ_PINS) {
		return;
	}
	reg  = gpio / 8;
	slot = gpio % 8;
	s_branch[gpio] = fn;

	/* clear any stale edge, then enable both edges for core 1.
	   The set-alias write is atomic, so two subsystems registering pins
	   at the same time cannot overwrite each other's enable bits. */
	out_w(IOB_INTR0 + (reg * 4), 0xCu << (slot * 4));
	out_w((IOB_INTE0 + (reg * 4)) | REG_ATOMIC_SET, 0xCu << (slot * 4));
}

bool gpio_irq_is_enabled(void)
{
	return s_enabled != 0;
}

int32_t gpio_irq_enable(void)
{
	T_DINT	dint;
	ER	er;

	if (s_enabled != 0) {
		return E_OK;
	}
	if (tk_get_prc() != GPIO_IRQ_PRC) {
		return E_CTX;	/* must be called on core 1 (the NVIC is per core) */
	}
	dint.intatr = TA_HLNG;
	dint.inthdr = (FP)gpio_irq_hdr;
	er = tk_def_int(INTNO_IO_BANK0, &dint);
	if (er == E_OK) {
		UINT	imask;
		UW	reg   = NVIC_IPR_BASE + ((INTNO_IO_BANK0 / 4) * 4);
		UINT	shift = (INTNO_IO_BANK0 % 4) * 8;
		UW	v;

		/* Do NOT use EnableInt() here: it writes the priority with a
		   BYTE store, and the Cortex-M0+ NVIC only supports WORD
		   access.  A byte store also changes the priority of IRQ 15
		   (the kernel's inter-core interrupt) in the same register,
		   which breaks dispatching and stops the whole system (USB
		   never appears).  Read-modify-write the whole word instead,
		   the same way the kernel does in smp_rp2040.c.             */
		DI(imask);
		v  = in_w(reg);
		v &= ~(0xFFu << shift);
		v |= (((UW)GPIO_IRQ_INTPRI & 0x3u) << 6) << shift;
		out_w(reg, v);
		out_w(NVIC_ICPR(INTNO_IO_BANK0), 1u << (INTNO_IO_BANK0 % 32));
		out_w(NVIC_ISER(INTNO_IO_BANK0), 1u << (INTNO_IO_BANK0 % 32));
		EI(imask);
		s_enabled = 1;
	}
	return (int32_t)er;
}
