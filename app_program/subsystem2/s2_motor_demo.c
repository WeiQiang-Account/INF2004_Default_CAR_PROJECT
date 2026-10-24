/*
 *	s2_motor_demo.c
 *	Subsystem 2 -- simple motor bring-up demo (no PID, no state machine).
 *	Built only when S2_MOTOR_DEMO is 1 (motion_config.h). It then REPLACES
 *	the motion task: subsystem2_start() starts s2_motor_demo_task instead.
 *
 *	Sequence (repeats forever), after a 5 s start-up pause:
 *	  1. left wheel FORWARD   2 s
 *	  2. stop                 1 s
 *	  3. left wheel BACKWARD  2 s
 *	  4. stop                 1 s
 *	  5. ROTATE (left forward, right backward if fitted)  3 s
 *	  6. stop                 3 s
 *	Encoder counts are printed after every step (USB serial, optional).
 *
 *	APIs used: tk_dly_tsk, tm_printf
 */

#include "subsystem2.h"
#include "motion_config.h"
#include "drv_motor.h"

#if (S2_MOTOR_DEMO != 0)

#define DEMO_DUTY	(600)	/* 60 % of full power (permille) */

LOCAL void demo_step(const char *name, int32_t duty_l, int32_t duty_r, INT ms)
{
	int32_t l0, r0, l1, r1;

	encoder_read(&l0, &r0);
	motor_set(MOTOR_LEFT, duty_l);
#if (MOTION_RIGHT_MOTOR_OK != 0)
	motor_set(MOTOR_RIGHT, duty_r);
#else
	(void)duty_r;
	motor_set(MOTOR_RIGHT, 0);
#endif
	tk_dly_tsk((RELTIM)ms);
	encoder_read(&l1, &r1);
	tm_printf((UB*)"[DEMO] %s %d ms: left counts %d, right counts %d\n",
		  name, ms, (INT)(l1 - l0), (INT)(r1 - r0));
}

EXPORT void s2_motor_demo_task(INT stacd, void *exinf)
{
	INT i;

	(void)stacd;
	(void)exinf;
	(void)gpio_irq_enable();	/* encoder counting (this task is on core 1) */
	motor_stop_all();

	for (i = 5; i > 0; i--) {
		tm_printf((UB*)"[DEMO] motor demo starts in %d s - lift the wheels!\n", i);
		tk_dly_tsk(1000);
	}

	for (;;) {
		demo_step("FORWARD",   DEMO_DUTY,  DEMO_DUTY, 2000);
		demo_step("stop",      0,          0,         1000);
		demo_step("BACKWARD", -DEMO_DUTY, -DEMO_DUTY, 2000);
		demo_step("stop",      0,          0,         1000);
		demo_step("ROTATE",    DEMO_DUTY, -DEMO_DUTY, 3000);
		demo_step("stop",      0,          0,         3000);
	}
}

#endif /* S2_MOTOR_DEMO */
