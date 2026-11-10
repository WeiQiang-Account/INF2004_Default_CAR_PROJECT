/*
 *	subsystem2_test.c
 *	On-target tests for subsystem 2 (Motion).  Built only with: make TEST=1
 *	Record results in testing/subsystem2/TEST_LOG.md.  IDs: S2-U01, ...
 *
 *	Manual tests (wheels, tape measure, scope) are listed in TEST_LOG.md.
 *	Lift the wheels before running: the start-up self-test spins them.
 */

#include "subsystem2.h"
#include "motion_config.h"
#include "../test_report.h"

#if APP_TEST

#define PWM_REG(slice, off)	(PWM_BASE + ((slice) * 0x14) + (off))

#if (S2_MANUAL_TESTS != 0)
LOCAL void s2_print_status(const char *tag)
{
	motion_status_t st;
	INT l, r;
	motion_get_status(&st);
	encoder_read((int32_t*)&l, (int32_t*)&r);
	tm_printf((UB*)"[MANUAL] %s state=%d cause=%d v=(%d,%d) mm/s counts=(%d,%d) dist=%d mm\n",
		  tag, (INT)st.state, (INT)st.cause, (INT)st.speed_l_mmps,
		  (INT)st.speed_r_mmps, l, r, (INT)st.distance_mm);
}

LOCAL void s2_wait_done(UW timeout_ms)
{
	UINT ptn;
	(void)tk_wai_flg(flg_move_done, MOVE_DONE_BIT, TWF_ORW, &ptn, (TMO)timeout_ms);
}

LOCAL void s2_countdown(const char *what, INT s)
{
	tm_printf((UB*)"[MANUAL] %s in %d s ...\n", what, s);
	tk_dly_tsk((RELTIM)(s * 1000));
	/* 2026-11-07: putting the robot down can trip the IMU E-stop, which leaves
	   Motion in FAULT; a FAULT ignores the move, so the robot never drove.
	   Clear it first (refused only while the IMU still holds the E-stop). */
	(void)motion_reset();
	tk_dly_tsk(100);
	s2_print_status("before the move");
}

#if (S2_MANUAL_SPEED != 0)
/* S2-M05: hold 300 mm/s with the wheels in the air. 1 s to settle, then
   30 samples over 3 s. Pass: every wheel average 270..330 mm/s (+-10 %).
   Results are printed AND visible on the dashboard (wheel speeds).      */
LOCAL void s2_speed_test(void)
{
	motion_status_t st;
	INT i, n = 0;
	INT sum_l = 0, sum_r = 0, min_l = 9999, max_l = -9999, min_r = 9999, max_r = -9999;
	const INT lo = (S2_MANUAL_SPEED_MMPS * 9) / 10, hi = (S2_MANUAL_SPEED_MMPS * 11) / 10;

	tm_printf((UB*)"\n[MANUAL] S2-M05 speed hold: PROP THE ROBOT UP so both wheels spin freely\n");
	s2_countdown("S2-M05 wheels at 300 mm/s", 15);
	motion_ignore_steering(true);			/* no line under the propped-up robot */
	(void)motion_drive(S2_MANUAL_SPEED_MMPS, FALSE);
	tk_dly_tsk(1000);					/* speed up and settle */
	for (i = 0; i < 30; i++) {
		tk_dly_tsk(100);
		motion_get_status(&st);
		sum_l += st.speed_l_mmps; sum_r += st.speed_r_mmps; n++;
		if (st.speed_l_mmps < min_l) { min_l = st.speed_l_mmps; }
		if (st.speed_l_mmps > max_l) { max_l = st.speed_l_mmps; }
		if (st.speed_r_mmps < min_r) { min_r = st.speed_r_mmps; }
		if (st.speed_r_mmps > max_r) { max_r = st.speed_r_mmps; }
	}
	(void)motion_stop();
	motion_ignore_steering(false);
	tm_printf((UB*)"[MANUAL] S2-M05 target %d mm/s: left avg %d (min %d, max %d), right avg %d (min %d, max %d)\n",
		  S2_MANUAL_SPEED_MMPS, sum_l / n, min_l, max_l, sum_r / n, min_r, max_r);
	tm_printf((UB*)"[MANUAL] S2-M05 %s (both averages must be %d..%d mm/s)\n",
		  ((sum_l / n >= lo) && (sum_l / n <= hi) && (sum_r / n >= lo) && (sum_r / n <= hi)) ? "PASS" : "FAIL",
		  lo, hi);
}
#endif

/* Guided manual tests: record the measured values in TEST_LOG.md */
LOCAL void s2_manual_tests(void)
{
	INT i;

#if (MOTION_RIGHT_MOTOR_OK == 0)
	tm_printf((UB*)"\n[MANUAL] NOTE: right motor disabled (MOTION_RIGHT_MOTOR_OK 0)."
		  " Moves use the LEFT wheel only, so the robot pivots on the right wheel.\n");
#endif
#if (S2_MANUAL_SPEED != 0)
	(void)i;
	s2_speed_test();
	tm_printf((UB*)"[MANUAL] subsystem2 manual tests finished (speed test only; S2_MANUAL_SPEED 0 for moves/turns)\n");
	return;
#endif
#if (S2_MANUAL_CPR != 0)
	tm_printf((UB*)"\n[MANUAL] S2-M01 CPR: lift the robot, turn the LEFT wheel"
		  " forward exactly 10 turns, then the RIGHT wheel (20 s)\n");
	for (i = 0; i < 40; i++) {
		tk_dly_tsk(500);
		s2_print_status("S2-M01");
	}
	tm_printf((UB*)"[MANUAL] S2-M01 CPR = counts / 10 (expected about 2535, measured 2026-10-24)\n");
#else
	(void)i;
#endif

	tm_printf((UB*)"[MANUAL] straight move: %d mm, so after the right turn the right wheel is at %d mm\n",
		  (INT)S2_MANUAL_MOVE_MM, (INT)S2_MANUAL_MARK_MM);
	s2_countdown("S2-M02 put the robot on the floor; straight forward", 10);
	(void)motion_move(S2_MANUAL_MOVE_MM, 150, FALSE);	/* 150 mm/s: gentle start, no IMU E-stop (2026-11-07) */
	s2_wait_done(12000);
	s2_print_status("S2-M02 done (measure the distance with a tape)");
	{
		INT e10 = (INT)(motion_heading_error() * 10.0f);
		tm_printf((UB*)"[MANUAL] compass heading hold: %s, heading error at the end %d.%d deg\n",
			  motion_heading_used() ? "ON" : "off (no compass)",
			  e10 / 10, ((e10 < 0) ? -e10 : e10) % 10);
	}

#if (S2_MANUAL_TURNS != 0)
#if (S2_MANUAL_TURN_RIGHT != 0)
	s2_countdown("S2-M03 90 deg RIGHT turn (measure the heading NOW)", S2_MANUAL_PAUSE_S);
	(void)motion_turn_right(90);
	s2_wait_done(5000);
	s2_print_status("S2-M03 done (measure: 85..95 deg)");
#endif
#if (S2_MANUAL_AFTER_TURN_MM > 0)
	s2_countdown("straight on after the right turn", 5);
	(void)motion_move(S2_MANUAL_AFTER_TURN_MM, 150, FALSE);
	s2_wait_done(12000);
	s2_print_status("straight after the turn done (measure distance and drift)");
#endif

#if (S2_MANUAL_TURN_LEFT != 0)
	s2_countdown("S2-M04 90 deg LEFT turn (measure the heading NOW)", S2_MANUAL_PAUSE_S);
	(void)motion_turn_left(90);
	s2_wait_done(5000);
	s2_print_status("S2-M04 done (measure: 85..95 deg)");
#endif
#endif
	tm_printf((UB*)"[MANUAL] subsystem2 manual tests finished\n");
}
#endif

/* Time to open the serial monitor after plugging in: output sent before
   the PC connects is lost.  Set to 0 to start at once.                   */
#ifndef S2_TEST_START_DELAY_MS
#define S2_TEST_START_DELAY_MS	(5000)
#endif

EXPORT void subsystem2_test(void)
{
#if (S2_TEST_START_DELAY_MS > 0)
	tm_printf((UB*)"[TEST] subsystem2 tests start in %d s - open the serial monitor now\n",
		  (INT)(S2_TEST_START_DELAY_MS / 1000));
	tk_dly_tsk(S2_TEST_START_DELAY_MS);
#endif
	TEST_INIT();

	/* S2-U01: every kernel object exists */
	TEST_CHECK("S2-U01", mbf_motion_cmd > 0 && flg_move_done > 0 &&
		   flg_estop > 0 && flg_fault > 0 && tid_motion > 0 &&
		   gpio_irq_is_enabled(),
		   "motion kernel objects created, GPIO IRQ enabled on core 1");

	/* S2-U02: packed command round trip and clamping (minimum-bit) */
	{
		motion_cmd_t in = { MOTION_CMD_MOVE_FWD, 300, 1000, 0, 0 }, out;
		motion_cmd_packed_t w = motion_cmd_pack(&in);
		BOOL ok;
		motion_cmd_unpack(w, &out);
		ok = (out.type == MOTION_CMD_MOVE_FWD) && (out.speed_mmps == 300) &&
		     (out.distance_mm == 1000) && ((w >> 24) == 0);
		in.distance_mm = 99999; in.speed_mmps = 9999;
		motion_cmd_unpack(motion_cmd_pack(&in), &out);
		ok = ok && (out.distance_mm == 2047) && (out.speed_mmps == 511);
		TEST_CHECK("S2-U02", ok && sizeof(motion_cmd_packed_t) == 4,
			   "4-byte command packs, unpacks and clamps");
	}

	/* S2-U03: packed telemetry status round trip (8 bytes) */
	{
		motion_status_t s = { MOTION_STATE_TURNING, MOTION_CAUSE_STALL, TRUE,
				      -512, 1023, -46, 127, 54321 }, s2;
		uint8_t b[MOTION_STATUS_PACKED_BYTES];
		motion_status_pack(&s, b);
		motion_status_unpack(b, &s2);
		TEST_CHECK("S2-U03", s2.state == s.state && s2.cause == s.cause &&
			   s2.speed_l_mmps == -512 && s2.speed_r_mmps == 1023 &&
			   s2.delta_l == -46 && s2.distance_mm == 54321,
			   "8-byte status packs and unpacks");
	}

	/* S2-U04: PWM slices 4 and 5 run at 20 kHz (TOP 6249, divider 1) */
	tm_printf((UB*)"[TEST] S2-U04 info: TOP4=%d TOP5=%d DIV4=0x%x CSR4=0x%x CSR5=0x%x\n",
		  (INT)in_w(PWM_REG(4, PWM_CHx_TOP)), (INT)in_w(PWM_REG(5, PWM_CHx_TOP)),
		  (INT)in_w(PWM_REG(4, PWM_CHx_DIV)), (INT)in_w(PWM_REG(4, PWM_CHx_CSR)),
		  (INT)in_w(PWM_REG(5, PWM_CHx_CSR)));
	TEST_CHECK("S2-U04",
		   in_w(PWM_REG(4, PWM_CHx_TOP)) == 6249 &&
		   in_w(PWM_REG(5, PWM_CHx_TOP)) == 6249 &&
		   in_w(PWM_REG(4, PWM_CHx_DIV)) == (1u << 4) &&
		   (in_w(PWM_REG(4, PWM_CHx_CSR)) & PWM_CH_CSR_EN) &&
		   (in_w(PWM_REG(5, PWM_CHx_CSR)) & PWM_CH_CSR_EN),
		   "motor PWM configured for 20 kHz");

	/* S2-U05: the motion loop runs every 20 ms (about 50 cycles in 1 s) */
	{
		UW c0 = motion_cycle_count(), n;
		tk_dly_tsk(1000);
		n = motion_cycle_count() - c0;
		tm_printf((UB*)"[TEST] S2-U05 info: %d cycles in 1000 ms\n", (INT)n);
		TEST_CHECK("S2-U05", n >= 48 && n <= 52, "20 ms motion period (NFR5)");
	}

	/* S2-M06 (software, no logic analyser): the motion task times its own
	   wake-ups with the 1 us timer for 3 s (about 150 cycles) while this
	   task just sleeps. Pass: every period 20 ms +-2 ms (18..22 ms).
	   (2026-11-10: a busy-wait here on core 0 froze the core-1 timing.)  */
	{
		uint32_t mn, mx, av, n;
		motion_period_reset();
		tk_dly_tsk(3000);
		motion_period_stats(&mn, &mx, &av, &n);
		tm_printf((UB*)"[TEST] S2-M06 info: %d periods, avg %d us, min %d us, max %d us, jitter %d us\n",
			  (INT)n, (INT)av, (INT)mn, (INT)mx, (n > 0u) ? (INT)(mx - mn) : 0);
		TEST_CHECK("S2-M06", (n >= 140u) && (mn >= 18000u) && (mx <= 22000u),
			   "motion loop 20 ms +-2 ms, timed by the task itself (NFR5)");
	}

	/* S2-U06: the motion task sleeps between cycles (no busy loop).
	   Main check: the task's own record of how long each cycle keeps it busy
	   (taken in the same 3 s as S2-M06). A cycle must leave the CPU free for
	   most of its 20 ms. The task-state samples are printed as extra info:
	   on the dual-core kernel a sample can catch the task mid-cycle.      */
	{
		uint32_t bmax, bavg;
		motion_busy_stats(&bmax, &bavg);
		tm_printf((UB*)"[TEST] S2-U06 info: work per 20 ms cycle avg %d us, max %d us\n", (INT)bavg, (INT)bmax);
		TEST_CHECK("S2-U06", (bmax > 0u) && (bmax < 5000u) && (bavg < 1000u),
			   "motion task works < 5 ms of each 20 ms cycle, sleeps the rest");
	}
	{
		T_RTSK rtsk;
		INT i, waiting = 0, run = 0, rdy = 0, other = 0;
		for (i = 0; i < 10; i++) {
			tk_dly_tsk(7);
			if (tk_ref_tsk(tid_motion, &rtsk) != E_OK) {
				other++;
			} else if ((rtsk.tskstat & TTS_WAI) != 0) {
				waiting++;
			} else if (rtsk.tskstat == TTS_RUN) {
				run++;
			} else if (rtsk.tskstat == TTS_RDY) {
				rdy++;
			} else {
				other++;
			}
		}
		tm_printf((UB*)"[TEST] S2-U06 info: of 10 samples wait=%d run=%d ready=%d other=%d (state now 0x%x)\n",
			  waiting, run, rdy, other, (INT)rtsk.tskstat);
		/* info only (see above) */
	}

#if (S2_MANUAL_TESTS != 0)
	/* 2026-11-09: U07/U08 spin the wheels for 0.1 s, which jolted the robot out
	   of line just before the guided straight-line test. They run whenever
	   S2_MANUAL_TESTS is 0.                                               */
	tm_printf((UB*)"[TEST] S2-U07 SKIP  (manual mode: it spins the wheels)\n");
	tm_printf((UB*)"[TEST] S2-U08 SKIP  (manual mode)\n");
#else
	/* S2-U07: E-stop stops both motors within one cycle (NFR9) */
	{
		motion_status_t st;
		/* Start from IDLE: without wheels/encoders (e.g. Pico off the
		   board) the start-up self-test leaves the task in FAULT.    */
		(void)motion_reset();
		tk_dly_tsk(60);
		(void)motion_drive(200, FALSE);
		tk_dly_tsk(100);
		tk_set_flg(flg_estop, ESTOP_BIT);
		tk_dly_tsk(MOTION_PERIOD_MS + 5);
		motion_get_status(&st);
		TEST_CHECK("S2-U07", st.state == MOTION_STATE_FAULT &&
			   st.cause == MOTION_CAUSE_ESTOP &&
			   (in_w(PWM_REG(4, PWM_CHx_CC)) == 0) &&
			   (in_w(PWM_REG(5, PWM_CHx_CC)) == 0),
			   "E-stop -> FAULT and PWM at zero within 25 ms");
	}

	/* S2-U08: reset is refused while E-stop is set, accepted after */
	{
		motion_status_t st1, st2;
		(void)motion_reset();
		tk_dly_tsk(60);
		motion_get_status(&st1);
		tk_clr_flg(flg_estop, ~ESTOP_BIT);
		(void)motion_reset();
		tk_dly_tsk(60);
		motion_get_status(&st2);
		TEST_CHECK("S2-U08", st1.state == MOTION_STATE_FAULT &&
			   st2.state == MOTION_STATE_IDLE,
			   "reset only after the E-stop is cleared");
	}

#endif /* S2_MANUAL_TESTS */

	TEST_SUMMARY("subsystem2");

#if (S2_MANUAL_TESTS != 0)
	s2_manual_tests();
#endif
}

#endif /* APP_TEST */
