/*
 *	s2_telemetry_drive.c
 *	Subsystem 2 -- drive pattern for the Wi-Fi telemetry test (Buddy 2).
 *
 *	Built only with make NET=lwip MQTT=1 and S2_TELEMETRY_TEST = 1.
 *	Uses the real motion task (PID) through the public motion API, so the
 *	telemetry shows the speed controller at work. LIFT THE WHEELS.
 *
 *	After 20 s (time for Wi-Fi + MQTT to connect), repeats forever:
 *	  forward 200 mm/s 3 s, stop 2 s, backward 200 mm/s 3 s, stop 2 s,
 *	  turn left 90 deg, pause 3 s.
 *
 *	APIs used: tk_dly_tsk, tk_wai_flg, tm_printf
 */

#include "subsystem2.h"
#include "motion_config.h"

#if defined(TM_MQTT) && (TM_MQTT == 1) && (S2_TELEMETRY_TEST != 0)

#define DRIVE_SPEED_MMPS	(200)

LOCAL void wait_move_done(TMO tmo_ms)
{
	UINT ptn;
	(void)tk_wai_flg(flg_move_done, MOVE_DONE_BIT, TWF_ORW, &ptn, tmo_ms);
}

EXPORT void s2_telemetry_drive_task(INT stacd, void *exinf)
{
	(void)stacd;
	(void)exinf;

	tm_printf((UB*)"s2-drive: drive pattern starts in 20 s - lift the wheels!\n");
	tk_dly_tsk(20000);

	for (;;) {
		(void)motion_reset();		/* leave FAULT (e.g. stall) if needed */
		tk_dly_tsk(100);

		tm_printf((UB*)"s2-drive: forward\n");
		(void)motion_drive(DRIVE_SPEED_MMPS, FALSE);
		tk_dly_tsk(3000);
		(void)motion_stop();
		wait_move_done(1000);
		tk_dly_tsk(2000);

		tm_printf((UB*)"s2-drive: backward\n");
		(void)motion_drive(DRIVE_SPEED_MMPS, TRUE);
		tk_dly_tsk(3000);
		(void)motion_stop();
		wait_move_done(1000);
		tk_dly_tsk(2000);

		tm_printf((UB*)"s2-drive: turn left 90\n");
		(void)motion_turn_left(90);
		wait_move_done(6000);
		tk_dly_tsk(3000);
	}
}

#endif /* TM_MQTT && S2_TELEMETRY_TEST */
