/*
 *	subsystem2.h
 *	Subsystem 2 -- Motion Control (motors, encoders, PID).
 *	Student ID: 2501989 (Buddy 2).
 *
 *	PUBLIC interface: what other subsystems may use. Include it as
 *	"../subsystem2/subsystem2.h".
 *
 *	  Vehicle Controller : motion_drive / move / turn_left / turn_right /
 *	                       uturn / stop / reset, then wait on flg_move_done
 *	  Line Follow (S3)   : steer_publish(correction) every 5 ms
 *	  Barcode (S3)       : gpio_irq_register(7, ...), encoder_read()
 *	  IMU (S4)           : tk_set_flg(flg_estop, ESTOP_BIT); encoder_read()
 *	  Ultrasonic (S5)    : gpio_irq_register(14, ...)
 *	  Telemetry (S1)     : override motion_status_hook(), motion_status_pack()
 *	  Everyone           : report faults with tk_set_flg(flg_fault, FAULT_BIT_*)
 */

#ifndef SUBSYSTEM2_H
#define SUBSYSTEM2_H

#include "../app.h"
#include "motion.h"
#include "motion_types.h"
#include "drv_encoder.h"
#include "drv_gpio_irq.h"

/* Priority band 14-15 (TEAM.md). NOTE: our design puts IMU Safety ABOVE
   Motion and Line Follow BELOW it; the team must agree bands that keep
   that order (see TEAM.md section 4). */
#define S2_PRI_MOTION		14
#define S2_MOTION_CORE		TP_PRC2		/* core 1: also runs the GPIO IRQ.
						   Core 0 runs USB + WiFi polling. */

/* Kernel objects owned by subsystem 2 (created in subsystem2_init) */
IMPORT ID	mbf_motion_cmd;	/* commands -> motion task (via the API)   */
IMPORT ID	flg_move_done;	/* MOVE_DONE_BIT when a move/turn/stop ends */
IMPORT ID	flg_estop;	/* ESTOP_BIT: IMU sets, motion stops        */
IMPORT ID	flg_fault;	/* shared fault bitmask, FAULT_BIT_*         */
IMPORT ID	tid_motion;
IMPORT void	s2_motor_demo_task(INT stacd, void *exinf);	/* S2_MOTOR_DEMO only */
IMPORT void	s2_telemetry_drive_task(INT stacd, void *exinf);	/* MQTT test only */

/* -------------------------------------------------------------------
 * Template example (producer for subsystems 3 and 4). Kept ONLY so the
 * example in subsystems 3/4 still builds. Set to 0 once they have
 * replaced their example code, then delete example_producer.c.
 * ------------------------------------------------------------------- */
#ifndef S2_KEEP_TEMPLATE_EXAMPLE
#define S2_KEEP_TEMPLATE_EXAMPLE	1
#endif

#if S2_KEEP_TEMPLATE_EXAMPLE
#define S2_PRI_PRODUCER		15
#define S2_PERIOD_MS		300
typedef struct {
	T_MSG	hdr;
	UW	seq;
	UW	time_ms;
	INT	value;
	INT	prc;
} SAMPLE_MSG;
#define S2_POOL_COUNT	4
IMPORT ID	sem_slots;
IMPORT ID	mpf_samples;
IMPORT void	producer_task(INT stacd, void *exinf);
IMPORT INT	s2_make_value(UW seq);
IMPORT ER	s2_example_init(void);
IMPORT ER	s2_example_start(void);
#endif

#endif /* SUBSYSTEM2_H */
