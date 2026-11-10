/*
 *	drv_ultrasonic.c
 *	HC-SR04 style sensor (TRIG GP0, ECHO GP1) and scan servo (GP12, PWM slice 6).
 *	Called only from the Ultrasonic Scan task (single-owner rule, TEAM.md section 3).
 *
 *	The echo is timed by polling ECHO with dispatching disabled (see below).
 *
 *	APIs used: tk_dis_dsp, tk_ena_dsp, tk_dly_tsk, libbsp gpio_* / pwm_* (s5_hw.h)
 */
#include "drv_ultrasonic.h"
#include "s5_hw.h"

/* 2026-11-06: the echo is now timed by POLLING the ECHO pin with dispatching
   disabled on this core (at most S5_ECHO_WAIT_US), like the stand-alone test
   (Temp_Test/Ultrasonic_Test) that measured 12-90 cm correctly. The old way,
   a branch on S2's shared GPIO interrupt, only ever saw a short false pulse
   right after the trigger on this robot. sem_echo is no longer used. */

/* Diagnostics for telemetry: ECHO rise / fall of the last ping in us after the
   trigger (0xFFFFFFFF = not seen), edges seen since boot, last ping result. */
LOCAL volatile UW	diag_rise_us = 0xFFFFFFFFu, diag_fall_us = 0xFFFFFFFFu, diag_edges;
LOCAL volatile UW	ping_w_us, ping_st = 2u, ping_mm = 0xFFFFu;
LOCAL UW		last_trig_ms;
LOCAL BOOL		pinged;

LOCAL UW now_ms(void) { SYSTIM t; (void)tk_get_tim(&t); return (UW)t.lo; }

void s5_ping_diag(uint32_t *width_us, uint32_t *status, uint32_t *mm)
{
	*width_us = ping_w_us;
	*status   = ping_st;
	*mm       = ping_mm;
}

void s5_echo_diag(uint32_t *rise_us, uint32_t *fall_us, uint32_t *edges, uint32_t *level)
{
	*rise_us = diag_rise_us;
	*fall_us = diag_fall_us;
	*edges   = diag_edges;
	*level   = (uint32_t)s5_gpio_get(S5_PIN_ECHO);
}

void us_hw_init(void)
{
	s5_pwm_unreset();
	s5_gpio_out_init(S5_PIN_TRIG);
	s5_gpio_in_init(S5_PIN_ECHO, 0, 0);			/* no pull: same as the stand-alone test that works */
	s5_gpio_put(S5_PIN_TRIG, 0);

	/* servo: 1 MHz tick, 20 ms period -> 50 Hz. GP12 = slice 6 (S2 keeps slices 4/5) */
	s5_pwm_start(S5_PIN_SERVO, S5_SYS_CLK_MHZ, 19999u);
	servo_set_deg(S5_SERVO_CENTRE_DEG);

}

void servo_set_deg(int deg)
{
	UW us;

	if (deg < S5_SERVO_LIMIT_MIN) deg = S5_SERVO_LIMIT_MIN;	/* hard limits: the head pops off beyond */
	if (deg > S5_SERVO_LIMIT_MAX) deg = S5_SERVO_LIMIT_MAX;
	deg += S5_SERVO_TRIM_DEG;		/* mounting correction: the arm sits a few degrees off */
	if (deg < 0)   deg = 0;
	if (deg > 180) deg = 180;
#if S5_SERVO_REVERSED
	deg = 180 - deg;
#endif
	us = S5_SERVO_US_0DEG + (UW)(((INT)(S5_SERVO_US_180DEG - S5_SERVO_US_0DEG) * deg) / 180);
	s5_pwm_level(S5_PIN_SERVO, us);
}

LOCAL us_status_t us_ping_raw(uint16_t *mm);

us_status_t us_ping(uint16_t *mm)
{
	us_status_t st = us_ping_raw(mm);
	ping_st = (UW)st;
	ping_mm = *mm;
	return st;
}

LOCAL us_status_t us_ping_raw(uint16_t *mm)
{
	UW	t0, t_rise = 0, t_fall = 0, d, w;
	INT	i;
	BOOL	rose = FALSE, fell = FALSE;

	/* The sensor needs time between measurements: triggered again too soon
	   it answers with a ~20-90 us false pulse (seen 2026-11-06 at 20-60 ms;
	   the stand-alone test at 240 ms was clean). Every caller gets the gap. */
	if (pinged) {
		UW since = now_ms() - last_trig_ms;
		if (since < S5_PING_MIN_GAP_MS) { (void)tk_dly_tsk((TMO)(S5_PING_MIN_GAP_MS - since)); }
	}

	/* a "nothing found" echo from the last ping can last ~38 ms: let it end
	   first, or the sensor ignores this trigger */
	for (i = 0; (i < 40) && (s5_gpio_get(S5_PIN_ECHO) != 0); i++) {
		(void)tk_dly_tsk(1);
	}
	if (s5_gpio_get(S5_PIN_ECHO) != 0) {		/* stuck high: cannot measure now */
		*mm = S5_DIST_NO_ECHO;
		return US_NO_ECHO;
	}

	last_trig_ms = now_ms();
	pinged = TRUE;
	(void)tk_dis_dsp();				/* no task switch on this core while timing (<= 4 ms) */
	s5_gpio_put(S5_PIN_TRIG, 1);
	t0 = s5_time_us();
	while ((UW)(s5_time_us() - t0) < S5_TRIG_US) { }
	s5_gpio_put(S5_PIN_TRIG, 0);

	while ((UW)(s5_time_us() - t0) < S5_ECHO_RISE_TMO_US) {	/* wait for the echo to start (~215 us) */
		if (s5_gpio_get(S5_PIN_ECHO) != 0) { t_rise = s5_time_us(); rose = TRUE; break; }
	}
	if (rose) {
		while ((UW)(s5_time_us() - t_rise) < S5_ECHO_WAIT_US) {	/* then for it to end */
			if (s5_gpio_get(S5_PIN_ECHO) == 0) { t_fall = s5_time_us(); fell = TRUE; break; }
		}
	}
	(void)tk_ena_dsp();

	diag_rise_us = rose ? (t_rise - t0) : 0xFFFFFFFFu;
	diag_fall_us = fell ? (t_fall - t0) : 0xFFFFFFFFu;
	diag_edges  += (rose ? 1u : 0u) + (fell ? 1u : 0u);
#if S5_US_DEBUG_PRINT
	{
		static UW n;
		if ((++n % 4u) == 0u) {			/* ~3 lines per second on the console */
			tm_printf((UB*)"[S5 US] rise %d fall %d -> %s\n",
				  rose ? (INT)(t_rise - t0) : -1, fell ? (INT)(t_fall - t0) : -1,
				  !rose ? "no sensor" : !fell ? "clear" :
				  ((t_fall - t_rise) < S5_MIN_ECHO_US) ? "GLITCH" : "OK");
		}
	}
#endif

	*mm = S5_DIST_NO_ECHO;
	if (!rose) {
		return US_NO_SENSOR;			/* ECHO never rose: unplugged / wiring */
	}
	if (!fell) {
		return US_NO_ECHO;			/* still high: nothing within range -> clear */
	}
	w = t_fall - t_rise;
	ping_w_us = w;
	d = (w * 343u) / 2000u;				/* mm; speed of sound 343 m/s */
	if ((w < S5_MIN_ECHO_US) || (d < S5_MIN_RANGE_MM) || (d > S5_MAX_RANGE_MM)) {
		return US_NO_ECHO;
	}
	*mm = (uint16_t)d;
	return US_OK;
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
