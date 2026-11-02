/*
 *	subsystem5_test.c
 *	On-target tests for subsystem 5.  Built only with: make TEST=1
 *	Record results in testing/subsystem5/TEST_LOG.md.  IDs: S5-U01 .. S5-U08.
 *
 *	These run in the initial task while the S5 tasks are live, so they never
 *	touch the sensor or servo directly: they ask the Ultrasonic Scan task
 *	for a scan (flg_scan_req) and read its results (g_s5_scan).
 *	S5-U06/U07 therefore also prove the wiring: TRIG, ECHO, servo, and
 *	Subsystem 2's GPIO interrupt.
 *	Set S5_MANUAL_TESTS to 1 for the guided distance test (S5-M01).
 */
#include "subsystem5.h"
#include "../test_report.h"

#ifndef S5_MANUAL_TESTS
#define S5_MANUAL_TESTS 0
#endif

#if APP_TEST

LOCAL s5_profile_t mk_profile(UINT closest, UINT clr_l, UINT clr_r, INT xl, UINT width)
{
	s5_profile_t p = {0};
	p.found = 1; p.closest_mm = closest; p.clr_l_mm = clr_l; p.clr_r_mm = clr_r;
	p.xl_mm = xl; p.width_mm = width; p.xr_mm = xl - (INT)width;
	return p;
}

EXPORT void subsystem5_test(void)
{
	TEST_INIT();
	s5_profile_t p, q;
	UW w[2];
	UW n0, dropped0;
	INT i;

	/* ---- pure logic (the same code the PC test checks) ---- */
	p = mk_profile(187, 211, 340, -37, 95);
	p.action = S5_ACT_TURN_RIGHT;
	s5_profile_pack(&p, w, VC_EV_PROFILE);
	s5_profile_unpack(w, &q);
	TEST_CHECK("S5-U01", (VC_EV_TYPE(w[0]) == VC_EV_PROFILE) && (q.action == p.action) &&
		   (q.closest_mm == 187) && (q.width_mm == 95) && (q.clr_l_mm == 211) &&
		   (q.clr_r_mm == 340) && (q.xl_mm == -37), "profile pack/unpack round trip (8 bytes)");

	p = mk_profile(180, 500, 500, 40, 80);
	TEST_CHECK("S5-U02", (s5_plan_action(&p, 0) == S5_ACT_TURN_LEFT) || (s5_plan_action(&p, 0) == S5_ACT_TURN_RIGHT),
		   "open space both sides: plan chooses a side");
	p = mk_profile(180, 40, 300, 40, 80);
	TEST_CHECK("S5-U03", s5_plan_action(&p, 0) == S5_ACT_TURN_RIGHT, "left side too narrow: plan goes right");
	p = mk_profile(80, 500, 500, 40, 80);
	TEST_CHECK("S5-U04", (s5_plan_action(&p, 0) == S5_ACT_REVERSE_RETRY) && (s5_plan_action(&p, S5_MAX_RESCAN) == S5_ACT_STOP),
		   "closer than 10 cm: reverse and retry, then stop after the rescan limit");
	TEST_CHECK("S5-U05", (S5_STATUS_PACK(7, 3, 7) == 0xFF) && (s5_lateral_mm(150, 200) > 150) && (s5_lateral_mm(30, 200) < -150),
		   "status byte packing (3+2+3 bits) and left/right geometry sign");

	/* ---- event queue to the Vehicle Controller ---- */
	dropped0 = vc_dropped_events();
	for (i = 0; i < 4; i++) {
		(void)vc_post(VC_EV_MAKE(VC_EV_JUNCTION, i));
		(void)tk_dly_tsk(20);
	}
	TEST_CHECK("S5-U06", vc_dropped_events() == dropped0, "4 events posted to mbf_vc_events, none dropped");

	/* ---- live scan: proves servo + TRIG + ECHO + S2's GPIO IRQ + scan task ---- */
	n0 = g_s5_scan.count;
	s5_scan_request(S5_REQ_SCAN);
	for (i = 0; (i < 400) && (g_s5_scan.count == n0); i++) (void)tk_dly_tsk(10);
	TEST_CHECK("S5-U07", g_s5_scan.count != n0, "scan task completes a scan (else: check GP12/13/14 wiring, GPIO IRQ)");
	tm_printf((UB*)"[S5] coarse scan %u ms, total %u ms, closest %u mm, action %u\n",
		  (UW)g_s5_scan.coarse_ms, (UW)g_s5_scan.total_ms, (UW)g_s5_scan.closest_mm, (UW)g_s5_scan.action);
	TEST_CHECK("S5-U08", (g_s5_scan.count != n0) && (g_s5_scan.coarse_ms <= 1200), "coarse scan of 5 angles <= 1200 ms (NFR16)");

#if S5_MANUAL_TESTS
	/* S5-M01: hold a flat board at 10, 20, 40 cm. Read the console, compare with a ruler (+/-1 cm, NFR17). */
	tm_printf((UB*)"[S5-M01] hold a flat board straight ahead; 10 readings follow\n");
	(void)tk_dly_tsk(3000);
	for (i = 0; i < 10; i++) {
		tm_printf((UB*)"[S5-M01] ahead = %u mm\n", (UW)s5_last_distance_mm());
		(void)tk_dly_tsk(500);
	}
#endif
	TEST_SUMMARY("subsystem5");
}

#endif /* APP_TEST */
