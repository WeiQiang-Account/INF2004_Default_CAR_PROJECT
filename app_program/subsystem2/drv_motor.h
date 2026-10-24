/*
 *	drv_motor.h
 *	Subsystem 2 -- Robo Pico motor driver on GP8-GP11, 20 kHz PWM
 *	(PWM slices 4 and 5). Only the motion task calls these.
 */

#ifndef DRV_MOTOR_H
#define DRV_MOTOR_H

#include <stdint.h>
#include <stdbool.h>

typedef enum {
	MOTOR_LEFT  = 0,
	MOTOR_RIGHT = 1
} motor_side_t;

void motor_init(void);
void motor_set(motor_side_t side, int32_t duty);	/* -1000..+1000 permille */
void motor_stop_all(void);				/* both to 0 at once     */
void motion_debug_pin(bool high);			/* GP15 timing pin       */

#endif /* DRV_MOTOR_H */
