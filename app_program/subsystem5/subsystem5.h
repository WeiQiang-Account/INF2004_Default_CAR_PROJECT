/*
 *	subsystem5.h
 *	Subsystem 5 -- Obstacle handling (Ultrasonic Scan) + system coordination
 *	(Vehicle Controller).  Student ID: 2401151 (Buddy 5).
 *
 *	PUBLIC interface: what other subsystems may use.
 *	  Everyone     : vc_post(VC_EV_MAKE(type, arg))  -- send an event to the VC
 *	                 (formats in vc_events.h; non-blocking, never stalls you)
 *	  Buddy 3 (S3) : provide  int s5_hook_line_detected(void)   (see s5_port.c)
 *	                 post VC_EV_BARCODE / JUNCTION / LINE_LOST / LINE_FOUND / FINISH
 *	  Buddy 4 (S4) : post VC_EV_HUMP_START / HUMP_DONE / IMPACT / IMU_LOST
 *	                 optionally provide  int s5_hook_imu_ready(void)
 *	  Buddy 1 (S1) : read vc_status_byte(), s5_status_byte(), s5_last_distance_mm()
 *	                 optionally provide  int s5_hook_remote_poll(void)
 */
#ifndef SUBSYSTEM5_H
#define SUBSYSTEM5_H

#include "../app.h"
#include "s5_config.h"
#include "obstacle_profile.h"
#include "vc_events.h"

#define S5_PRI_MAIN	S5_PRI_VC	/* template name; priority band 20-21 (TEAM.md) */

/* Kernel objects owned by S5 (created in subsystem5_init) */
IMPORT ID flg_scan_req;		/* VC -> Ultrasonic Scan */
IMPORT ID mbf_vc_events;	/* everyone -> VC        */
IMPORT ID sem_echo;		/* echo branch -> Ultrasonic Scan */

/* flg_scan_req bits */
#define S5_REQ_SCAN		(1u << 0)	/* robot stopped (v = 0): full scan            */
#define S5_REQ_RESCAN		(1u << 1)	/* same, counts against S5_MAX_RESCAN          */
#define S5_REQ_GUARD_R		(1u << 2)	/* bypass: obstacle on the right, watch it     */
#define S5_REQ_GUARD_L		(1u << 3)
#define S5_REQ_GUARD_OFF	(1u << 4)
#define S5_REQ_BYPASS_DONE	(1u << 5)	/* line re-acquired: reset attempts            */
#define S5_REQ_SERVO		(1u << 6)	/* manual servo angle changed (dashboard)      */
#define S5_REQ_ALL		0x7Fu

/* Internal, shared between the S5 files */
IMPORT void s5_scan_task(INT stacd, void *exinf);
IMPORT void s5_vc_task(INT stacd, void *exinf);
IMPORT void s5_scan_request(UINT bits);

/* Snapshot of the last completed scan (written by the scan task, read by tests/telemetry) */
typedef struct {
	volatile UW	count;		/* increments when a scan finishes */
	volatile UW	coarse_ms;	/* NFR16: must stay <= 1200        */
	volatile UW	total_ms;
	volatile UW	action;		/* s5_action_t                     */
	volatile UW	closest_mm;
	volatile UW	width_mm;
	volatile UW	clr_l_mm;	/* free space left / right of the obstacle (0 = none) */
	volatile UW	clr_r_mm;
} s5_scan_stats_t;
IMPORT s5_scan_stats_t g_s5_scan;

#define S5_PRINT(fmt, ...)	tm_printf((UB*)(fmt), ##__VA_ARGS__)

#endif /* SUBSYSTEM5_H */
