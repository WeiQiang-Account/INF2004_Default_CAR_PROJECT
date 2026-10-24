/*
 *	motion.h
 *	Subsystem 2 -- Motion Control core API (no kernel headers, so the PC
 *	tests can use it too). Other subsystems include subsystem2.h instead.
 *
 *	Task: every 20 ms, woken at once by flg_estop.
 *	Inputs : mbf_motion_cmd, g_steer_latest, encoder counts, flg_estop
 *	Outputs: motor PWM GP8-GP11, flg_move_done, fault bits, status
 */

#ifndef MOTION_H
#define MOTION_H

#include <stdbool.h>
#include <stdint.h>
#include "motion_types.h"
#include "s2_os_port.h"

/* Kernel objects the motion core uses (created in subsystem2_init) */
typedef struct {
	os_id_t flg_estop;	/* ESTOP_BIT, set by IMU Safety Monitor (S4)  */
	os_id_t flg_fault;	/* shared fault bitmask (FAULT_BIT_*)         */
	os_id_t mbf_cmd;	/* mbf_motion_cmd: commands from the VC       */
	os_id_t flg_move_done;	/* MOVE_DONE_BIT, waited on by the VC         */
} motion_links_t;

void     motion_core_init(const motion_links_t *p_links);
void     motion_task(int32_t stacd, void *p_exinf);	/* task entry */

/* Command API (Vehicle Controller). Each returns true if queued. */
bool     motion_drive(int32_t speed_mmps, bool reverse);
bool     motion_move(int32_t distance_mm, int32_t speed_mmps, bool reverse);
bool     motion_turn_left(int32_t angle_deg);
bool     motion_turn_right(int32_t angle_deg);
bool     motion_uturn(void);
bool     motion_stop(void);
bool     motion_reset(void);

/* Latest status, consistent copy, safe from any task on either core */
void     motion_get_status(motion_status_t *p_out);
uint32_t motion_cycle_count(void);

/* Hooks other subsystems may override (weak defaults do nothing) */
void     motion_status_hook(const motion_status_t *p_status);	/* telemetry */
void     motion_watchdog_checkin(void);				/* NFR20     */

/* Used by the tests */
void     motion_step(uint32_t now_ms, bool estop);
void     motion_reset_for_test(void);

#endif /* MOTION_H */
