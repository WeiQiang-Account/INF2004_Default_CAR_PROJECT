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
}

/* Guided manual tests: record the measured values in TEST_LOG.md */
LOCAL void s2_manual_tests(void)
{
	INT i;

#if (MOTION_RIGHT_MOTOR_OK == 0)
	tm_printf((UB*)"\n[MANUAL] NOTE: right motor disabled (MOTION_RIGHT_MOTOR_OK 0)."
		  " Moves use the LEFT wheel only, so the robot pivots on the right wheel.\n");
#endif
	tm_printf((UB*)"\n[MANUAL] S2-M01 CPR: lift the robot, turn the LEFT wheel"
		  " forward exactly 10 turns, then the RIGHT wheel (20 s)\n");
	for (i = 0; i < 40; i++) {
		tk_dly_tsk(500);
		s2_print_status("S2-M01");
	}
	tm_printf((UB*)"[MANUAL] S2-M01 CPR = counts / 10 (expected about 937)\n");

	s2_countdown("S2-M02 put the robot on the floor; 1 m forward", 10);
	(void)motion_move(1000, 300, FALSE);
	s2_wait_done(8000);
	s2_print_status("S2-M02 done (measure with a tape: 950..1050 mm)");

	s2_countdown("S2-M03 90 deg RIGHT turn", 5);
	(void)motion_turn_right(90);
	s2_wait_done(5000);
	s2_print_status("S2-M03 done (measure: 85..95 deg)");

	s2_countdown("S2-M04 90 deg LEFT turn", 5);
	(void)motion_turn_left(90);
	s2_wait_done(5000);
	s2_print_status("S2-M04 done (measure: 85..95 deg)");
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

	/* S2-U06: the motion task sleeps between cycles (no busy loop) */
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
		TEST_CHECK("S2-U06", waiting >= 9, "motion task waits between cycles");
	}

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

	TEST_SUMMARY("subsystem2");

#if (S2_MANUAL_TESTS != 0)
	s2_manual_tests();
#endif
}

#endif /* APP_TEST */
