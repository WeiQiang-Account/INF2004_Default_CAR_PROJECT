/*
 *	drv_ultrasonic.c
 *	HC-SR04 style sensor (TRIG GP13, ECHO GP14) and scan servo (GP12, PWM slice 6).
 *	Called only from the Ultrasonic Scan task (single-owner rule, TEAM.md section 3).
 *
 *	The echo edges arrive through Subsystem 2's shared GPIO interrupt
 *	(drv_gpio_irq.c): we register a branch for GP14. The branch only stores
 *	the timestamp it is given and signals sem_echo (< 2 us rule).
 *
 *	APIs used: gpio_irq_register (S2), tk_sig_sem, tk_wai_sem, tk_dly_tsk,
 *	           libbsp gpio_* / pwm_* (through s5_hw.h)
 */
#include "../subsystem2/drv_gpio_irq.h"
#include "drv_ultrasonic.h"
#include "s5_hw.h"

LOCAL volatile UW	t_rise_us, width_us;
LOCAL volatile UB	rise_seen, fall_done;

/* Runs inside the shared GPIO IRQ (core 1): timestamp + signal, nothing else. */
LOCAL void echo_branch(uint32_t gpio, uint32_t events, uint32_t levels, uint32_t t_us)
{
	(void)gpio; (void)levels;
	if ((events & GPIO_EVT_RISE) != 0u) {
		t_rise_us = t_us;
		rise_seen = 1;
	} else if (((events & GPIO_EVT_FALL) != 0u) && (rise_seen != 0) && (fall_done == 0)) {
		width_us  = t_us - t_rise_us;
		fall_done = 1;
		(void)tk_sig_sem(sem_echo, 1);
	}
}

void us_hw_init(void)
{
	s5_pwm_unreset();
	s5_gpio_out_init(S5_PIN_TRIG);
	s5_gpio_in_init(S5_PIN_ECHO, 0, 1);			/* pull-down: reads low if unplugged */

	/* servo: 1 MHz tick, 20 ms period -> 50 Hz. GP12 = slice 6 (S2 keeps slices 4/5) */
	s5_pwm_start(S5_PIN_SERVO, S5_SYS_CLK_MHZ, 19999u);
	servo_set_deg(S5_SERVO_CENTRE_DEG);

	gpio_irq_register(S5_PIN_ECHO, echo_branch);		/* IF-11; enabled by motion_task on core 1 */
}

void servo_set_deg(int deg)
{
	UW us;

	if (deg < 0)   deg = 0;
	if (deg > 180) deg = 180;
#if S5_SERVO_REVERSED
	deg = 180 - deg;
#endif
	us = S5_SERVO_US_0DEG + (UW)(((INT)(S5_SERVO_US_180DEG - S5_SERVO_US_0DEG) * deg) / 180);
	s5_pwm_level(S5_PIN_SERVO, us);
}

us_status_t us_ping(uint16_t *mm)
{
	UW t0, d;
	UB seen;
	INT i;

	rise_seen = 0;
	fall_done = 0;
	while (tk_wai_sem(sem_echo, 1, TMO_POL) == E_OK) { }	/* drop stale signals */

	s5_gpio_put(S5_PIN_TRIG, 1);
	t0 = s5_time_us();
	while ((UW)(s5_time_us() - t0) < S5_TRIG_US) { }
	s5_gpio_put(S5_PIN_TRIG, 0);

	if (tk_wai_sem(sem_echo, 1, S5_PING_TMO_MS) == E_OK) {
		d = (width_us * 343u) / 2000u;			/* mm; speed of sound 343 m/s */
		if (d > S5_MAX_RANGE_MM) { *mm = S5_DIST_NO_ECHO; return US_NO_ECHO; }
		*mm = (uint16_t)d;
		return US_OK;
	}
	seen = rise_seen;
	for (i = 0; (i < 15) && (s5_gpio_get(S5_PIN_ECHO) != 0); i++) {
		(void)tk_dly_tsk(1);				/* let a long echo finish */
	}
	*mm = S5_DIST_NO_ECHO;
	return (seen != 0) ? US_NO_ECHO : US_NO_SENSOR;
}

us_status_t us_ping2(uint16_t *mm)
{
	uint16_t a, b, lo, hi;
	us_status_t sa = us_ping(&a);
	us_status_t sb;

	(void)tk_dly_tsk(S5_PING_GAP_MS);
	sb = us_ping(&b);

	if ((sa == US_NO_SENSOR) && (sb == US_NO_SENSOR)) { *mm = S5_DIST_NO_ECHO; return US_NO_SENSOR; }
	if ((sa != US_OK) && (sb != US_OK))               { *mm = S5_DIST_NO_ECHO; return US_NO_ECHO; }
	if (sa != US_OK) { *mm = b; return US_OK; }		/* one echo only: trust it (safer) */
	if (sb != US_OK) { *mm = a; return US_OK; }
	lo = (a < b) ? a : b;
	hi = (a < b) ? b : a;
	*mm = ((hi - lo) > 20) ? lo : (uint16_t)((a + b) / 2);	/* disagree -> take the nearer */
	return US_OK;
}
