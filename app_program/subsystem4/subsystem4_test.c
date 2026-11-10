/*
 *	subsystem4_test.c
 *	On-target tests for subsystem 4.  Built only with: make TEST=1
 *	Record results in testing/subsystem4/TEST_LOG.md.
 *
 *	S4-U01..U07 run the SAME checks as the PC host test (s4_selftest.c).
 *	S4-U08..U10 look at the live tasks. With no IMU wired, U09 reports the
 *	IMU as LOST (degraded) - that is the designed behaviour, not a crash.
 */
#include "subsystem4.h"
#include "s4_selftest.h"
#include "drv_mpu6050.h"
#include "../test_report.h"

#if APP_TEST

EXPORT void subsystem4_test(void)
{
	UW n0;

	TEST_INIT();

	/* ---- pure logic (the same code the PC test checks) ---- */
	TEST_CHECK("S4-U01", s4_st_math(),        "sqrt / sin / atan2 without libm");
	TEST_CHECK("S4-U02", s4_st_calibration(), "static offsets (FR16); moving robot refused");
	TEST_CHECK("S4-U03", s4_st_safety(),      "impact on 1 sample (NFR14), tilt 60 ms, release after 1 s");
	TEST_CHECK("S4-U04", s4_st_pitch(),       "pitch sign and complementary filter");
	TEST_CHECK("S4-U05", s4_st_hump(),        "humps of 20 / 10 / 23 mm (any shape) to +-1 mm (FR17, NFR15)");
	TEST_CHECK("S4-U06", s4_st_hump_reject(), "40 ms spike ignored, long climb = ramp");
	TEST_CHECK("S4-U07", s4_st_class(),       "motion classes held 60 ms (FR18)");

	/* ---- live ---- */
	TEST_CHECK("S4-U08", flg_imu_new > 0, "sample event flag created");

	/* the VC needs 'IMU ready' within 5 s: calibration (2 s) or LOST, never stuck */
	{
		INT i;
		for (i = 0; (i < 60) && !s4_imu_ready(); i++) { tk_dly_tsk(100); }
		tm_printf((UB*)"[TEST] S4-U09 info: state %d (0 cal, 1 ok, 2 fault, 3 lost), WHO_AM_I 0x%x, I2C errors %d\n",
			  (INT)s4_imu_state(), (UINT)mpu_who_am_i(), (INT)mpu_errors());
		TEST_CHECK("S4-U09", s4_imu_ready(), "IMU ready or degraded within 6 s");
	}

	/* 50 Hz sampling (NFR13), only meaningful with the IMU connected */
	n0 = s4_sample_count();
	tk_dly_tsk(1000);
	{
		UW n = s4_sample_count() - n0;
		tm_printf((UB*)"[TEST] S4-U10 info: %d samples in 1000 ms\n", (INT)n);
		TEST_CHECK("S4-U10", (s4_imu_state() == S4_IMU_LOST) || ((n >= 48u) && (n <= 52u)),
			   "IMU sampled every 20 ms (NFR13), or LOST without an IMU");
	}

	TEST_SUMMARY("subsystem4");
}

#endif /* APP_TEST */
