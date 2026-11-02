/* s5_hw.h - thin wrappers over the template's libbsp (gpio_*, pwm_*) and sysdef.h
 * register macros, so S5 uses exactly the same low-level API as Subsystem 2.     */
#ifndef S5_HW_H
#define S5_HW_H
#include "../app.h"

static inline void s5_pwm_unreset(void)       /* same guard as S2's motor_init(): only CLEAR the bit */
{
	if ((in_w(RESETS_RESET_DONE) & RESETS_RESET_PWM) == 0u) {
		clr_w(RESETS_RESET, RESETS_RESET_PWM);
		while ((in_w(RESETS_RESET_DONE) & RESETS_RESET_PWM) == 0u) { }
	}
}
static inline void s5_gpio_out_init(UINT pin) { (void)gpio_set_pin(pin, GPIO_MODE_OUT); (void)gpio_set_val(pin, 0); }
static inline void s5_gpio_in_init(UINT pin, int pull_up, int pull_down)
{
	(void)gpio_set_pin(pin, GPIO_MODE_IN);
	clr_w(GPIO(pin), GPIO_PUE | GPIO_PDE);
	if (pull_up)   set_w(GPIO(pin), GPIO_PUE);
	if (pull_down) set_w(GPIO(pin), GPIO_PDE);
}
static inline void s5_gpio_put(UINT pin, int v) { (void)gpio_set_val(pin, (UINT)v); }
static inline int  s5_gpio_get(UINT pin)        { return (int)gpio_get_val(pin); }
static inline UW   s5_time_us(void)             { return in_w(TIMER_TIMERAWL); }

/* PWM channel: integer divider, wrap (TOP), compare 0 -> output low, enabled */
static inline void s5_pwm_start(UINT pin, UINT div_int, UW wrap)
{
	UINT slice = (pin >> 1) & 0x7u;
	(void)pwm_set_pin(pin);
	out_w(PWM_BASE + (slice * 0x14) + PWM_CHx_DIV, div_int << 4);
	(void)pwm_set_wrap(pin, wrap);
	(void)pwm_set_cc(pin, 0);
	(void)pwm_set_enabled(pin, TRUE);
}
static inline void s5_pwm_level(UINT pin, UW level) { (void)pwm_set_cc(pin, level); }
#endif
