/*
 *	task_imu_safety.c
 *	Subsystem 4 -- IMU Safety Monitor (S4_PRI_SAFETY, every 20 ms, core 1).
 *
 *	Kept MINIMAL (Week 6 task table): read the MPU-6050, check impact and tilt,
 *	share the sample with IMU Terrain, feed the watchdog. Nothing else.
 *	  CALIBRATING : 100 still samples -> static offsets (FR16)
 *	  MONITORING  : impact > 2.5 g (one sample, NFR14) or tilt > 30 deg (60 ms)
 *	                -> flg_estop + FAULT_BIT_IMPACT + VC_EV_IMPACT
 *	  FAULT       : E-stop held until 1 s level and calm, then released
 *	  LOST        : 3 failed reads -> VC_EV_IMU_LOST + FAULT_BIT_IMU, NO E-stop
 *	                (degraded mode); retry every 500 ms
 *	It is the ONLY task that touches I2C0 (TEAM.md section 3).
 *
 *	APIs used: tk_dly_tsk, tk_get_tim, tk_set_flg, tk_clr_flg, vc_post (S5),
 *	           motion_cycle_count (S2)
 */
#include "subsystem4.h"
#include "drv_mpu6050.h"
#include "../subsystem2/subsystem2.h"		/* flg_estop, flg_fault, motion_cycle_count() */
#include "../subsystem5/vc_events.h"		/* vc_post(), VC_EV_* */

LOCAL s4_cal_t		cal;
LOCAL s4_safety_t	safety;
LOCAL UW		motion_last, motion_stale;

LOCAL uint32_t now_ms(void) { SYSTIM t; (void)tk_get_tim(&t); return (uint32_t)t.lo; }

/* Watchdog (NFR20): feed only if Motion Control has also completed a cycle recently */
LOCAL void watchdog_tick(void)
{
	UW mc = motion_cycle_count();

	s4_tick_check();				/* every 20 ms, before anything else */

	if (mc != motion_last) {
		motion_last = mc;
		motion_stale = 0;
	} else if (motion_stale < 255u) {
		motion_stale++;
	}
	if (motion_stale <= S4_MOTION_STALE_CYC) {
		s4_watchdog_feed();
	}
}

LOCAL void go_lost(void)
{
	s4_set_imu_state(S4_IMU_LOST);
	(void)tk_set_flg(flg_fault, FAULT_BIT_IMU);
	(void)vc_post(VC_EV_MAKE(VC_EV_IMU_LOST, 0));
	S4_PRINT("[S4] IMU LOST (I2C0 GP4/GP5 not answering): degraded, no hump detection\n");
}

/* 2 s of still samples. Returns FALSE if the sensor stops answering. */
LOCAL BOOL calibrate(void)
{
	s4_sample_t	raw;
	UINT		fails = 0, tries;
	BOOL		ok = FALSE;

	s4_set_imu_state(S4_IMU_CALIBRATING);
	for (tries = 1; (tries <= S4_CAL_TRIES) && !ok; tries++) {
		S4_PRINT("[S4] calibrating IMU (%d s) - keep the robot STILL and level\n",
			 (INT)((S4_CAL_SAMPLES * S4_PERIOD_MS) / 1000));
		s4_cal_begin(&cal);
		while (!s4_cal_done(&cal)) {
			if (mpu_read(&raw)) {
				fails = 0;
				s4_cal_feed(&cal, &raw);
			} else if (++fails >= S4_LOST_READS) {
				return FALSE;
			}
			watchdog_tick();
			(void)tk_dly_tsk(S4_PERIOD_MS);
		}
		/* last try: accept even if it moved, so the VC is not kept in INIT (5 s limit) */
		ok = s4_cal_end(&cal, tries == S4_CAL_TRIES);
		if (!ok) {
			S4_PRINT("[S4] robot moved during calibration, again\n");
		} else if (cal.moved) {
			S4_PRINT("[S4] WARNING: calibrated while moving, offsets may be off\n");
		}
	}
	S4_PRINT("[S4] IMU offsets: acc %d %d %d mg, gyro %d %d %d (0.01 dps)\n",
		 (INT)(cal.acc_off[0] * 1000.0f), (INT)(cal.acc_off[1] * 1000.0f), (INT)(cal.acc_off[2] * 1000.0f),
		 (INT)(cal.gyr_off[0] * 100.0f), (INT)(cal.gyr_off[1] * 100.0f), (INT)(cal.gyr_off[2] * 100.0f));
	return TRUE;
}

/* Compass heading, left turn = +. The x/y field is smoothed (not the angle,
   so 359 -> 0 deg does not upset the filter). This task owns I2C0, so the
   compass is read here, right after the accelerometer.                    */
LOCAL void compass_step(void)
{
	static float	fx, fy;
	static BOOL	init;
	float		mx, my, mz;

	if (!mpu_has_mag() || !mpu_mag_read(&mx, &my, &mz)) {
		s4_heading_publish(0.0f, FALSE);
		return;
	}
	(void)mz;
	if (!init) { fx = mx; fy = my; init = TRUE; }
	fx = (S4_HDG_ALPHA * fx) + ((1.0f - S4_HDG_ALPHA) * mx);
	fy = (S4_HDG_ALPHA * fy) + ((1.0f - S4_HDG_ALPHA) * my);
	if ((fx == 0.0f) && (fy == 0.0f)) {
		s4_heading_publish(0.0f, FALSE);
		return;
	}
	/* robot turns left -> the field turns right in the robot's frame */
	s4_heading_publish(-(float)S4_MAG_SIGN * s4_atan2f(fy, fx) * S4_RAD2DEG, TRUE);
}

LOCAL void check_safety(const s4_sample_t *s)
{
	BOOL		released = FALSE;
	uint32_t	start;
	bool		rel = false;

	s4_note_accel(s4_accel_mag(s));
	start = s4_safety_step(&safety, s, &rel);
	released = rel ? TRUE : FALSE;

	if (start != S4_SAFE_OK) {
		(void)tk_set_flg(flg_estop, ESTOP_BIT);		/* IF-08: Motion stops at once, bypassing the VC */
		(void)tk_set_flg(flg_fault, FAULT_BIT_IMPACT);
		(void)vc_post(VC_EV_MAKE(VC_EV_IMPACT, start));
		s4_set_imu_state(S4_IMU_FAULT);
		s4_note_estop((UINT)start, s4_accel_mag(s), s4_tilt_deg(s));
		S4_PRINT("[S4] E-STOP: %s%s |a| %d cg, tilt %d deg\n",
			 ((start & S4_SAFE_IMPACT) != 0u) ? "IMPACT " : "",
			 ((start & S4_SAFE_TILT) != 0u) ? "TILT" : "",
			 (INT)(s4_accel_mag(s) * 100.0f), (INT)s4_tilt_deg(s));
	}
	if (released) {
		(void)tk_clr_flg(flg_estop, ~(UINT)ESTOP_BIT);	/* tk_clr_flg KEEPS the bits you pass */
		(void)tk_clr_flg(flg_fault, ~(UINT)FAULT_BIT_IMPACT);
		s4_set_imu_state(S4_IMU_MONITORING);
		S4_PRINT("[S4] level and calm for 1 s: E-stop released (press START to reset)\n");
	}
}

EXPORT void imu_safety_task(INT stacd, void *exinf)
{
	s4_sample_t	s;
	UINT		fails = 0;
	uint32_t	next, retry_at = 0;
	INT		d;
	BOOL		lost = FALSE;

	(void)stacd; (void)exinf;
	s4_safety_init(&safety);
	s4_cal_begin(&cal);

	if (!mpu_init(true) || !calibrate()) {
		lost = TRUE;
		go_lost();
		retry_at = now_ms() + S4_RETRY_MS;
	} else {
		S4_PRINT("[S4] IMU ready: %s at 0x%x (WHO_AM_I 0x%x)%s\n", (UB*)mpu_chip_name(),
			 (UINT)mpu_addr(), (UINT)mpu_who_am_i(),
			 (UB*)(mpu_has_gyro() ? "" : " - pitch from accelerometer, turn rate from wheels"));
		s4_set_imu_state(S4_IMU_MONITORING);
	}
	safety.tilt_alpha = mpu_has_gyro() ? 0.0f : S4_TILT_ALPHA_NOGYRO;
	s4_watchdog_start();
	next = now_ms();

	for (;;) {
		if (lost) {
			if ((int32_t)(now_ms() - retry_at) >= 0) {
				retry_at = now_ms() + S4_RETRY_MS;
				if (mpu_init(false) && (cal.valid || calibrate())) {
					lost = FALSE;
					fails = 0;
					safety.tilt_alpha = mpu_has_gyro() ? 0.0f : S4_TILT_ALPHA_NOGYRO;
					(void)tk_clr_flg(flg_fault, ~(UINT)FAULT_BIT_IMU);
					s4_set_imu_state(S4_IMU_MONITORING);
					S4_PRINT("[S4] IMU back (offsets kept); the VC stays degraded until reset\n");
				}
			}
		} else if (mpu_read(&s)) {
			fails = 0;
			s4_cal_apply(&cal, &s);
			check_safety(&s);
			s4_shared_write(&s);
			(void)tk_set_flg(flg_imu_new, FLG_S4_SAMPLE);	/* wake IMU Terrain */
			compass_step();
		} else if (++fails >= S4_LOST_READS) {
			lost = TRUE;
			go_lost();
			retry_at = now_ms() + S4_RETRY_MS;
		}

		watchdog_tick();

		next += S4_PERIOD_MS;				/* drift-free 20 ms (NFR13) */
		d = (INT)(int32_t)(next - now_ms());
		if (d > 0) {
			(void)tk_dly_tsk((RELTIM)d);
		} else {
			next = now_ms();
			(void)tk_dly_tsk(1);
		}
	}
}
