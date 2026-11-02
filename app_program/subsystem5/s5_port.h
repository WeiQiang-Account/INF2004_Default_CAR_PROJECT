/* s5_port.h - the only place where Buddy 5's code reaches into other subsystems.
 * Motion (S2) is real. Things that belong to Buddies 3/4/1, who have not pushed
 * code yet, are WEAK HOOKS: they work with a safe default today and the owner
 * overrides them simply by defining a function with the same name (same pattern
 * S2 uses for motion_status_hook).                                              */
#ifndef S5_PORT_H
#define S5_PORT_H
#include "subsystem5.h"

/* Motion (Buddy 2). deg: + = left, - = right.  mm: + forward, - reverse. */
void s5_motion_stop(void);
void s5_motion_drive(int mm_per_s);
void s5_motion_move(int mm);
void s5_motion_turn(int deg);
void s5_motion_reset(void);
void s5_motion_clear_done(void);	/* call BEFORE the command you will wait on */
ER   s5_motion_wait_done(TMO tmo);	/* E_OK once flg_move_done fired            */
int  s5_motion_faulted(void);		/* 1 if Motion Control is in FAULT          */
int  s5_motion_ready(void);		/* 1 once Motion left INIT and is not FAULT */

void s5_fault_set(int on);		/* flg_fault: FAULT_BIT_NO_ECHO             */

/* ---- weak hooks (override in your own subsystem) ----------------------------- */
int  s5_hook_line_detected(void);	/* Buddy 3: any IR sensor over the line (default 0) */
int  s5_hook_imu_ready(void);		/* Buddy 4: IMU calibrated (default 1)              */
int  s5_hook_remote_poll(void);		/* Buddy 1: 0 none, 1 start, 2 stop (default 0)     */
#endif
