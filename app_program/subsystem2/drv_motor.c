/*
 *	drv_motor.c
 *	Subsystem 2 -- Robo Pico motor driver control with libbsp PWM.
 *
 *	APIs used: pwm_set_pin, pwm_set_wrap, pwm_set_cc, pwm_set_enabled,
 *	           gpio_set_pin, gpio_set_val (libbsp)
 *
 *	Each motor has two driver inputs (A/B):
 *	    forward : A = PWM, B = 0
 *	    reverse : A = 0,   B = PWM
 *	    stop    : A = B = 0  (or both high, see MOTOR_STOP_BOTH_HIGH)
 *	GP8/GP9 = PWM slice 4 (A/B), GP10/GP11 = slice 5. The servo (GP12)
 *	is slice 6, so motors and servo never share a slice.
 */

#include "../app.h"
#include "drv_motor.h"
#include "motion_config.h"

#define MOTOR_WRAP	((SYSCLK * 1000000u / MOTOR_PWM_FREQ_HZ) - 1u)	/* 6249 */
#define PWM_DIV_1	(1u << 4)	/* integer divider 1, no fraction */

LOCAL const UINT k_pin_a[2]  = { MOTOR_L_PIN_A, MOTOR_R_PIN_A };
LOCAL const UINT k_pin_b[2]  = { MOTOR_L_PIN_B, MOTOR_R_PIN_B };
LOCAL const INT  k_invert[2] = { MOTOR_L_INVERT, MOTOR_R_INVERT };

LOCAL void motor_pin_setup(UINT pin)
{
	UINT slice = (pin >> 1) & 0x7u;

	(void)pwm_set_pin(pin);
	out_w(PWM_BASE + (slice * 0x14) + PWM_CHx_DIV, PWM_DIV_1);
	(void)pwm_set_wrap(pin, MOTOR_WRAP);
	(void)pwm_set_cc(pin, 0);
	(void)pwm_set_enabled(pin, TRUE);
}

void motor_init(void)
{
	/* The kernel only takes the PWM block out of reset when USE_PTMR is 1
	   (config.h has 0), so do it here.  Only CLEAR the reset bit: setting
	   it would wipe PWM slices other subsystems (e.g. the servo) already
	   configured.  While PWM is in reset every write is ignored and the
	   motors never turn.                                               */
	if ((in_w(RESETS_RESET_DONE) & RESETS_RESET_PWM) == 0u) {
		clr_w(RESETS_RESET, RESETS_RESET_PWM);
		while ((in_w(RESETS_RESET_DONE) & RESETS_RESET_PWM) == 0u) {
			/* takes a few clock cycles */
		}
	}

	motor_pin_setup(MOTOR_L_PIN_A);
	motor_pin_setup(MOTOR_L_PIN_B);
	motor_pin_setup(MOTOR_R_PIN_A);
	motor_pin_setup(MOTOR_R_PIN_B);

#if (MOTION_DEBUG_PIN >= 0)
	(void)gpio_set_pin(MOTION_DEBUG_PIN, GPIO_MODE_OUT);
	(void)gpio_set_val(MOTION_DEBUG_PIN, 0);
#endif
}

LOCAL UW motor_level(INT duty_abs)
{
	if (duty_abs > DUTY_MAX) {
		duty_abs = DUTY_MAX;
	}
	return ((UW)duty_abs * (MOTOR_WRAP + 1u)) / (UW)DUTY_MAX;
}

void motor_set(motor_side_t side, int32_t duty)
{
	UINT a = k_pin_a[side];
	UINT b = k_pin_b[side];

	if (k_invert[side] != 0) {
		duty = -duty;
	}
	if (duty > 0) {
		(void)pwm_set_cc(a, motor_level((INT)duty));
		(void)pwm_set_cc(b, 0);
	} else if (duty < 0) {
		(void)pwm_set_cc(a, 0);
		(void)pwm_set_cc(b, motor_level((INT)-duty));
	} else {
#if (MOTOR_STOP_BOTH_HIGH != 0)
		(void)pwm_set_cc(a, MOTOR_WRAP + 1u);
		(void)pwm_set_cc(b, MOTOR_WRAP + 1u);
#else
		(void)pwm_set_cc(a, 0);
		(void)pwm_set_cc(b, 0);
#endif
	}
}

void motor_stop_all(void)
{
	motor_set(MOTOR_LEFT, 0);
	motor_set(MOTOR_RIGHT, 0);
}

void motion_debug_pin(bool high)
{
#if (MOTION_DEBUG_PIN >= 0)
	(void)gpio_set_val(MOTION_DEBUG_PIN, high ? 1 : 0);
#else
	(void)high;
#endif
}
