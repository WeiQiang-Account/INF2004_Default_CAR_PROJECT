/*
 *	drv_encoder.c
 *	Subsystem 2 -- x4 quadrature decoding for both wheel encoders.
 *
 *	Shared-data rules (Week 6 design + minimum-bit change requirement):
 *	  - one aligned 32-bit counter per wheel, written ONLY by the IRQ with
 *	    a single store -> any task, on either core, reads it atomically
 *	  - the counters are NOT packed with other fields: packing would turn
 *	    every count into a read-modify-write of shared data
 *	  - the error counter only ever increases; the task remembers what it
 *	    has already seen, so no task ever writes IRQ data (dual-core safe)
 */

#include "drv_encoder.h"
#include "motion_config.h"

#ifndef HOST_TEST
#include "../app.h"
#include "drv_gpio_irq.h"
#endif

/* Index = (previous AB << 2) | current AB, with AB = (A << 1) | B.
   Forward (A leads B): 00 -> 10 -> 11 -> 01 -> 00 gives +1 per edge.
   0 at 3, 6, 9, 12 = both pins changed = a skipped step. */
static const int8_t k_qdec[16] = {
	 0, -1, +1,  0,
	+1,  0,  0, -1,
	-1,  0,  0, +1,
	 0, +1, -1,  0
};

static const uint8_t k_pin_a[ENC_COUNT] = { ENC_L_PIN_A, ENC_R_PIN_A };
static const uint8_t k_pin_b[ENC_COUNT] = { ENC_L_PIN_B, ENC_R_PIN_B };
static const int8_t  k_sign[ENC_COUNT]  = { ENC_L_SIGN,  ENC_R_SIGN  };

static volatile int32_t  s_count[ENC_COUNT];	/* written by IRQ only */
static volatile uint32_t s_errors[ENC_COUNT];	/* written by IRQ only */
static volatile uint8_t  s_prev_ab[ENC_COUNT];	/* IRQ only            */
static uint32_t          s_errors_seen[ENC_COUNT];	/* motion task only */

static inline uint8_t enc_ab(enc_wheel_t w, uint32_t levels)
{
	uint8_t a = (uint8_t)((levels >> k_pin_a[w]) & 1u);
	uint8_t b = (uint8_t)((levels >> k_pin_b[w]) & 1u);
	return (uint8_t)((a << 1) | b);
}

void encoder_on_edge(enc_wheel_t wheel, uint32_t levels)
{
	uint8_t cur  = enc_ab(wheel, levels);
	uint8_t idx  = (uint8_t)((s_prev_ab[wheel] << 2) | cur);
	int8_t  step = k_qdec[idx];

	if (step != 0) {
		s_count[wheel] = s_count[wheel] + ((int32_t)step * (int32_t)k_sign[wheel]);
	} else if (cur != s_prev_ab[wheel]) {
		s_errors[wheel] = s_errors[wheel] + 1u;	/* missed an edge */
	} else {
		/* same state again (contact bounce): ignore */
	}
	s_prev_ab[wheel] = cur;
}

void encoder_read(int32_t *p_left, int32_t *p_right)
{
	*p_left  = s_count[ENC_LEFT];
	*p_right = s_count[ENC_RIGHT];
}

uint32_t encoder_new_errors(enc_wheel_t wheel)
{
	uint32_t now = s_errors[wheel];
	uint32_t n   = now - s_errors_seen[wheel];
	s_errors_seen[wheel] = now;
	return n;
}

#ifndef HOST_TEST
static void enc_branch_left(uint32_t gpio, uint32_t ev, uint32_t levels, uint32_t t_us)
{
	(void)gpio; (void)ev; (void)t_us;
	encoder_on_edge(ENC_LEFT, levels);
}

static void enc_branch_right(uint32_t gpio, uint32_t ev, uint32_t levels, uint32_t t_us)
{
	(void)gpio; (void)ev; (void)t_us;
	encoder_on_edge(ENC_RIGHT, levels);
}

static void enc_pin_input(uint32_t pin)
{
	(void)gpio_set_pin(pin, GPIO_MODE_IN);	/* libbsp: SIO input       */
	set_w(GPIO(pin), GPIO_PUE | GPIO_SHEMITT);	/* pull-up + Schmitt      */
}

void encoder_init(void)
{
	uint32_t w;

	for (w = 0; w < (uint32_t)ENC_COUNT; w++) {
		enc_pin_input(k_pin_a[w]);
		enc_pin_input(k_pin_b[w]);
		s_count[w] = 0;
		s_errors[w] = 0;
		s_errors_seen[w] = 0;
		s_prev_ab[w] = enc_ab((enc_wheel_t)w, (uint32_t)in_w(GPIO_IN));
	}
	gpio_irq_register(ENC_L_PIN_A, enc_branch_left);
	gpio_irq_register(ENC_L_PIN_B, enc_branch_left);
	gpio_irq_register(ENC_R_PIN_A, enc_branch_right);
	gpio_irq_register(ENC_R_PIN_B, enc_branch_right);
}
#else
void encoder_init(void) { encoder_test_reset(); }

void encoder_test_inject(enc_wheel_t wheel, int32_t counts)
{
	s_count[wheel] = s_count[wheel] + counts;
}

void encoder_test_reset(void)
{
	uint32_t w;
	for (w = 0; w < (uint32_t)ENC_COUNT; w++) {
		s_count[w] = 0; s_errors[w] = 0; s_errors_seen[w] = 0; s_prev_ab[w] = 0;
	}
}
#endif
