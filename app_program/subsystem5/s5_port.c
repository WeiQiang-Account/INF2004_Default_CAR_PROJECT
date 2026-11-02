/*
 *	s5_port.c
 *	Adapters from Buddy 5's code to Subsystem 2's public API (subsystem2.h),
 *	plus weak hooks for Buddies 3, 4 and 1.
 *	APIs used: motion_* (S2), flg_move_done / flg_fault (S2), tk_clr_flg,
 *	           tk_wai_flg, tk_set_flg
 */
#include "../subsystem2/subsystem2.h"
#include "s5_port.h"

void s5_motion_stop(void)       { (void)motion_stop(); }
void s5_motion_drive(int v)     { (void)motion_drive(v, FALSE); }
void s5_motion_reset(void)      { (void)motion_reset(); }
void s5_motion_move(int mm)
{
	(void)motion_move((mm < 0) ? -mm : mm, S5_BYPASS_SPEED_MM_S, (mm < 0));
}
void s5_motion_turn(int deg)
{
	if (deg >= 0) (void)motion_turn_left(deg);
	else          (void)motion_turn_right(-deg);
}

void s5_motion_clear_done(void)
{
	(void)tk_clr_flg(flg_move_done, ~(UINT)MOVE_DONE_BIT);	/* tk_clr_flg KEEPS the bits you pass */
}
ER s5_motion_wait_done(TMO tmo)
{
	UINT p = 0;
	return tk_wai_flg(flg_move_done, MOVE_DONE_BIT, TWF_ORW | TWF_BITCLR, &p, tmo);
}

int s5_motion_faulted(void)
{
	motion_status_t st;
	motion_get_status(&st);
	return st.state == MOTION_STATE_FAULT;
}
int s5_motion_ready(void)
{
	motion_status_t st;
	motion_get_status(&st);
	return (st.state != MOTION_STATE_INIT) && (st.state != MOTION_STATE_FAULT);
}

void s5_fault_set(int on)
{
	if (on) (void)tk_set_flg(flg_fault, FAULT_BIT_NO_ECHO);
	else    (void)tk_clr_flg(flg_fault, ~(UINT)FAULT_BIT_NO_ECHO);
}

/* ------------------------------------------------------------------ weak hooks */
__attribute__((weak)) int s5_hook_line_detected(void) { return 0; }	/* Buddy 3 overrides */
__attribute__((weak)) int s5_hook_imu_ready(void)     { return 1; }	/* Buddy 4 overrides */
__attribute__((weak)) int s5_hook_remote_poll(void)   { return 0; }	/* Buddy 1 overrides */
