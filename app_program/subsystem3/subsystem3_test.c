/*
 *	subsystem3_test.c
 *	On-target tests for subsystem 3.  Built only with: make TEST=1
 *	Record unit results in testing/subsystem3/TEST_LOG.md and the
 *	INT- results in testing/integration/TEST_LOG.md.
 *
 *	S3-U01..U09 run the SAME checks as the PC host test (s3_selftest.c), so a
 *	result on the PC and on the robot means the same thing.
 *	S3-U10..U12 look at the live tasks and need the sensors swept over black
 *	and white during the first S3_CAL_MS: set S3_LIVE_TESTS to 1 for them.
 */

#include "subsystem3.h"
#include "s3_selftest.h"
#include "drv_ir.h"
#include "../test_report.h"

#ifndef S3_LIVE_TESTS
#define S3_LIVE_TESTS 0
#endif

#if APP_TEST

EXPORT void subsystem3_test(void)
{
	TEST_INIT();

	/* ---- pure logic (the same code the PC test checks) ---- */
	TEST_CHECK("S3-U01", s3_st_decode_fwd(),    "barcodes A-Z (all 43 Code 39 chars) decode right (15 % noise)");
	TEST_CHECK("S3-U02", s3_st_decode_rev(),    "barcodes read backwards decode the same character");
	TEST_CHECK("S3-U03", s3_st_decode_reject(), "28 elements / flat widths / damaged code are rejected");
	TEST_CHECK("S3-U04", s3_st_line_steer(),    "line position, steering sign, calibration range check");
	TEST_CHECK("S3-U05", s3_st_line_events(),   "line lost in 8 samples (40 ms), found, junction once, no junction during a barcode");
	TEST_CHECK("S3-U06", s3_st_bc_speeds(),     "edges -> command at 3 speeds, both ways, timer wrap, within 100 ms (FR15)");
	TEST_CHECK("S3-U07", s3_st_bc_glitch(),     "spikes shorter than 1 ms inside the code are filtered");
	TEST_CHECK("S3-U08", s3_st_bc_errors(),     "incomplete or damaged code -> error, lone black mark -> ignored");
	TEST_CHECK("S3-U09", s3_st_bc_cooldown(),   "same barcode not read twice inside 300 ms");

#if S3_LIVE_TESTS
	{
		UH	w1, w2;
		UW	e0;
		INT	i;

		/* S3-U10: the ADC works. Wait for calibration (sweep the sensors over black and white now!) */
		for (i = 0; (i < (INT)(S3_CAL_MS / 100) + 20) && !s3_line_ready(); i++) {
			(void)tk_dly_tsk(100);
		}
		TEST_CHECK("S3-U10", s3_line_ready() && (ir_adc_errors() == 0),
			   "calibration finished on all 3 sensors, no ADC timeouts");

		/* S3-U11: Line Follow runs every 5 ms: in 100 ms the sequence number moves about 20 steps */
		w1 = g_steer_latest;
		(void)tk_dly_tsk(100);
		w2 = g_steer_latest;
		{
			INT8 a = 0; UB s1 = 0, s2 = 0;
			steer_unpack(w1, (int8_t *)&a, &s1);
			steer_unpack(w2, (int8_t *)&a, &s2);
			TEST_CHECK("S3-U11", (UB)(s2 - s1) >= 15 && (UB)(s2 - s1) <= 25,
				   "steer_latest sequence advances every ~5 ms (about 20 in 100 ms)");
		}

		/* S3-U12: GP7 is alive: edges are counted while you wave the right sensor over a barcode / tape */
		e0 = ir_edge_count();
		S3_PRINT("[S3-U12] wave the RIGHT sensor over black and white for 5 s...\n");
		(void)tk_dly_tsk(5000);
		TEST_CHECK("S3-U12", (ir_edge_count() - e0) >= 4, "GP7 edge interrupt counts edges (>= 4 in 5 s)");
	}
#endif

	TEST_SUMMARY("subsystem3");
}

#endif /* APP_TEST */
