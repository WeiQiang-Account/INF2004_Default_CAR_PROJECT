/*
 *	subsystem4.h
 *	Subsystem 4 -- Buddy 4: IMU-based motion and terrain monitoring.
 *	Student ID: 2502347.
 *
 *	Public interface of subsystem 4. Other subsystems include it as
 *	"../subsystem4/subsystem4.h".
 *
 *	What Buddy 4 provides (TEAM.md section 5):
 *	  IF-08  S4 -> S2   E-stop          tk_set_flg(flg_estop, ESTOP_BIT) on impact / tilt
 *	                                     (S4 clears it after 1 s level and calm)
 *	  IF-10  S4 -> all  fault bits      FAULT_BIT_IMU (IMU lost), FAULT_BIT_IMPACT in flg_fault
 *	  S4 -> VC          events          vc_post(): VC_EV_HUMP_START, VC_EV_HUMP_DONE (peak mm),
 *	                                     VC_EV_IMPACT, VC_EV_IMU_LOST  (vc_events.h, owned by S5)
 *	  S4 -> VC          hook            int s5_hook_imu_ready(void)  (overrides S5's weak default)
 *	  S4 -> S1          read-only       s4_*() getters below (telemetry)
 *	  NFR20             hardware watchdog, fed only while Motion and IMU Safety both run
 *
 *	Subsystem 4 owns: I2C0 (GP4/GP5, IMU Safety task only) and the RP2040 watchdog.
 *	Tasks: imu_safety_task  (S4_PRI_SAFETY, 20 ms, core 1)
 *	       imu_terrain_task (S4_PRI_TERRAIN, woken by every new sample)
 */
#ifndef SUBSYSTEM4_H
#define SUBSYSTEM4_H

#include "../app.h"
#include "s4_config.h"
#include "s4_logic.h"

/* Event flag: the Safety task sets it for every new sample, Terrain waits on it */
IMPORT ID	flg_imu_new;
#define FLG_S4_SAMPLE	0x00000001u

/* IMU state for telemetry (2 bits) */
#define S4_IMU_CALIBRATING	0u
#define S4_IMU_MONITORING	1u
#define S4_IMU_FAULT		2u	/* impact / tilt: E-stop set */
#define S4_IMU_LOST		3u	/* degraded: no hump detection */

#define S4_PRINT(fmt, ...)	tm_printf((UB*)(fmt), ##__VA_ARGS__)

/* ---- Read-only values for telemetry (Buddy 1), the VC and tests. Safe from any task. ---- */
UINT	s4_imu_state(void);		/* S4_IMU_*                                   */
BOOL	s4_imu_ready(void);		/* calibrated, or declared LOST (degraded)    */
INT	s4_pitch_x10(void);		/* filtered pitch in 0.1 deg, nose up = +     */
INT	s4_turn_rate_dps(void);		/* yaw rate, left = +  (FR19)                 */
UINT	s4_motion_class(void);		/* s4_mclass_t (FR18)                         */
UINT	s4_terrain_state(void);		/* s4_terrain_t                               */
UINT	s4_hump_mm(void);		/* height now, during a hump                  */
UINT	s4_hump_last_mm(void);		/* peak of the last hump                      */
UINT	s4_hump_max_mm(void);		/* highest peak of the run (mission item 5)   */
UINT	s4_hump_count(void);
UINT	s4_peak_accel_cg(void);		/* largest |a| seen, in 0.01 g (A8 evidence)  */
INT	s4_heading_deg(void);		/* compass heading, deg, left = +; 999 = none */
UINT	s4_estop_why(void);		/* last E-stop: 1 impact, 2 tilt, 3 both, 0 none */
UINT	s4_estop_cg(void);		/* |a| on that sample, 0.01 g                 */
UINT	s4_estop_tilt(void);		/* tilt on that sample, deg                   */
BOOL	s4_watchdog_reset_seen(void);	/* the last reset was the watchdog (NFR20)    */
UW	s4_sample_count(void);

/* ---- Private to subsystem 4 ---- */
typedef struct {			/* one calibrated sample, shared Safety -> Terrain */
	s4_sample_t	s;
	UW		seq;
	BOOL		valid;
} s4_shared_t;
IMPORT void	s4_shared_read(s4_shared_t *out);	/* consistent copy (sequence lock) */
IMPORT void	s4_shared_write(const s4_sample_t *s);
IMPORT void	s4_set_imu_state(UINT st);
IMPORT void	s4_terrain_publish(INT pitch_x10, UINT mclass, INT yaw, const s4_hump_t *h);
IMPORT void	s4_note_accel(float mag);
IMPORT void	s4_note_estop(UINT why, float mag, float tilt);
IMPORT void	s4_heading_publish(float deg, BOOL ok);
IMPORT void	s4_watchdog_start(void);
IMPORT void	s4_watchdog_feed(void);
IMPORT void	s4_tick_check(void);		/* keep the TIMER 1 us tick at 12 x 12 MHz */

IMPORT void	imu_safety_task(INT stacd, void *exinf);
IMPORT void	imu_terrain_task(INT stacd, void *exinf);

#endif /* SUBSYSTEM4_H */
