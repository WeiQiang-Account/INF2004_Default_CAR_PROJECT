/*
 *	s2_telemetry_drive.c
 *	Subsystem 2 -- movement test pattern for the Wi-Fi telemetry test (Buddy 2).
 *
 *	Built only with make NET=lwip MQTT=1, S2_TELEMETRY_TEST = 1 and
 *	S2_TELEMETRY_DRIVE = 1 (motion_config.h). Uses the real motion task
 *	(PID + encoders) through the public motion API. The current step name
 *	is sent in the telemetry ("step") so you can see which move is running.
 *
 *	FIRST RUN: LIFT THE WHEELS. If a wheel direction or encoder sign is
 *	wrong, that wheel may spin at full speed.
 *
 *	After 15 s (time for Wi-Fi + MQTT to connect), repeats forever,
 *	with a 2 s pause after every move:
 *	  1 FORWARD      drive forward 150 mm/s for 2 s
 *	  2 BACKWARD     drive backward 150 mm/s for 2 s
 *	  3 MOVE_FWD     move forward exactly 300 mm
 *	  4 MOVE_BWD     move backward (reverse) exactly 300 mm
 *	  5 TURN_RIGHT   spin right 90 deg on the spot
 *	  6 TURN_LEFT    spin left 90 deg on the spot
 *	  7 UTURN        spin 180 deg on the spot
 *	  8 UTURN_BACK   spin 180 deg again (back to the start heading)
 *
 *	APIs used: tk_dly_tsk, tk_wai_flg, tm_printf
 */

#include "subsystem2.h"
#include "motion_config.h"

/* Name of the move now running; read by s2_telemetry_mqtt.c ("step"). */
const char * volatile g_s2_drive_step = "WAIT";

#if defined(TM_MQTT) && (TM_MQTT == 1) && (S2_TELEMETRY_TEST != 0)

#define DRIVE_SPEED_MMPS	(100)
#define MOVE_DIST_MM		(300)
#define PAUSE_MS		(2000)

LOCAL void wait_move_done(TMO tmo_ms)
{
	UINT ptn;
	(void)tk_wai_flg(flg_move_done, MOVE_DONE_BIT, TWF_ORW, &ptn, tmo_ms);
}

LOCAL void step_begin(const char *name)
{
	g_s2_drive_step = name;
	tm_printf((UB*)"s2-drive: %s\n", name);
	(void)tk_clr_flg(flg_move_done, ~MOVE_DONE_BIT);
}

LOCAL void step_pause(void)
{
	g_s2_drive_step = "PAUSE";
	tk_dly_tsk(PAUSE_MS);
}

EXPORT void s2_telemetry_drive_task(INT stacd, void *exinf)
{
	(void)stacd;
	(void)exinf;

	tm_printf((UB*)"s2-drive: movement test starts in 15 s - LIFT THE WHEELS for the first run!\n");
	tk_dly_tsk(15000);

	for (;;) {
		(void)motion_reset();		/* leave FAULT (e.g. stall) if needed */
		tk_dly_tsk(100);

		step_begin("FORWARD");
		(void)motion_drive(DRIVE_SPEED_MMPS, FALSE);
		tk_dly_tsk(2000);
		(void)motion_stop();
		wait_move_done(500);
		step_pause();

		step_begin("BACKWARD");
		(void)motion_drive(DRIVE_SPEED_MMPS, TRUE);
		tk_dly_tsk(2000);
		(void)motion_stop();
		wait_move_done(1000);
		step_pause();

		step_begin("MOVE_FWD_300MM");
		(void)motion_move(MOVE_DIST_MM, DRIVE_SPEED_MMPS, FALSE);
		wait_move_done(5000);
		step_pause();

		step_begin("MOVE_BWD_300MM");
		(void)motion_move(MOVE_DIST_MM, DRIVE_SPEED_MMPS, TRUE);
		wait_move_done(5000);
		step_pause();

		step_begin("TURN_RIGHT_90");
		(void)motion_turn_right(90);
		wait_move_done(6000);
		step_pause();

		step_begin("TURN_LEFT_90");
		(void)motion_turn_left(90);
		wait_move_done(6000);
		step_pause();

		step_begin("UTURN_180");
		(void)motion_uturn();
		wait_move_done(8000);
		step_pause();

		step_begin("UTURN_BACK_180");
		(void)motion_uturn();
		wait_move_done(8000);
		step_pause();

		g_s2_drive_step = "REPEAT";
		tk_dly_tsk(3000);
	}
}

#endif /* TM_MQTT && S2_TELEMETRY_TEST */
