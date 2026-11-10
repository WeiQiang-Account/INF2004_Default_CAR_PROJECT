/*
 *	task_imu_terrain.c
 *	Subsystem 4 -- IMU Terrain (S4_PRI_TERRAIN), woken by every new sample (20 ms).
 *
 *	Region A (hump): pitch from the complementary filter; h += sin(pitch) x ds,
 *	  ds = mean left/right encoder distance (IF-12), so the error does not grow
 *	  with time (NFR15). HUMP_START / HUMP_DONE (peak = max h) go to the VC.
 *	Region B (motion class): STATIONARY / MOVING / TURNING / ACCELERATING and
 *	  the yaw rate (FR18, FR19), read by the VC and telemetry through getters.
 *	While the IMU is LOST, hump detection is off (degraded mode).
 *
 *	APIs used: tk_wai_flg, encoder_read (S2), vc_post (S5)
 */
#include "subsystem4.h"
#include "../subsystem2/drv_encoder.h"		/* encoder_read() */
#include "../subsystem2/motion_config.h"	/* MM_PER_COUNT (measured CPR 2535, wheel 63 mm) */
#include "../subsystem5/vc_events.h"		/* vc_post(), VC_EV_* */
#include "drv_mpu6050.h"			/* mpu_has_gyro() */

LOCAL s4_pitch_t	pitch;
LOCAL s4_hump_t		hump;
LOCAL s4_class_t	mclass;

EXPORT void imu_terrain_task(INT stacd, void *exinf)
{
	UINT		ptn;
	ER		er;
	s4_shared_t	sh;
	int32_t		l, r, l0, r0;
	UW		last_seq = 0;
	const float	dt = (float)S4_PERIOD_MS / 1000.0f;
	float		spd_prev = 0.0f, acc_g = 0.0f;

	(void)stacd; (void)exinf;
	s4_pitch_init(&pitch);
	s4_hump_init(&hump);
	s4_class_init(&mclass);
	encoder_read(&l0, &r0);

	for (;;) {
		er = tk_wai_flg(flg_imu_new, FLG_S4_SAMPLE, TWF_ORW | TWF_BITCLR, &ptn, 5 * S4_PERIOD_MS);

		encoder_read(&l, &r);
		{
			float ds = ((float)((l - l0) + (r - r0)) * 0.5f) * MM_PER_COUNT;   /* mean of both wheels */
			float spd = ds / dt;
			/* turn rate from the wheels (left turn = +), used when the IMU has no gyro (GY-511) */
			float wyaw = (((float)((r - r0) - (l - l0)) * MM_PER_COUNT) / TRACK_WIDTH_MM) / dt * 57.29578f;
			float yaw;
			float dv = spd - spd_prev;		/* speed change over this 20 ms */
			float p, lin;
			uint8_t ev;
			s4_mclass_t c;
			l0 = l;
			r0 = r;
			spd_prev = spd;

			if ((er != E_OK) || (s4_imu_state() == S4_IMU_LOST)) {
				/* no IMU: no humps; motion class from the wheels only */
				s4_hump_init(&hump);
				c = s4_class_step(&mclass, wyaw, spd, 0.0f);
				s4_terrain_publish(0, (UINT)c, (INT)wyaw, &hump);
				continue;
			}
			s4_shared_read(&sh);
			if (!sh.valid || (sh.seq == last_seq)) {
				continue;
			}
			last_seq = sh.seq;

			pitch.alpha = mpu_has_gyro() ? S4_ALPHA : S4_ALPHA_NOGYRO;
			yaw = mpu_has_gyro() ? s4_yaw_rate_dps(&sh.s) : wyaw;
			if (!mpu_has_gyro()) {
				/* robot's own speed change from the wheels, lightly smoothed */
				acc_g = (0.5f * acc_g) + (0.5f * ((dv / dt) / 9810.0f));
				s4_remove_fwd_accel(&sh.s, acc_g);
			}
			p = s4_pitch_step(&pitch, &sh.s, dt);
			lin = mpu_has_gyro() ? (s4_fwd_accel_g(&sh.s) - s4_sinf(p * S4_DEG2RAD))   /* remove gravity along the slope */
					     : acc_g;
			c = s4_class_step(&mclass, yaw, spd, lin);
			ev = s4_hump_step(&hump, p, ds, S4_PERIOD_MS);

			if (ev == S4_HUMP_EV_START) {
				(void)vc_post(VC_EV_MAKE(VC_EV_HUMP_START, 0));
#if S4_VERBOSE
				S4_PRINT("[S4] hump START (pitch %d.%d deg)\n", (INT)p, (INT)(((p < 0.0f) ? -p : p) * 10.0f) % 10);
#endif
			} else if (ev == S4_HUMP_EV_DONE) {
				(void)vc_post(VC_EV_MAKE(VC_EV_HUMP_DONE, hump.last_peak_mm));   /* FR17: within 200 ms of leaving */
#if S4_VERBOSE
				S4_PRINT("[S4] hump DONE: peak %d mm (highest so far %d mm)\n",
					 (INT)hump.last_peak_mm, (INT)hump.max_peak_mm);
#endif
			} else if (ev == S4_HUMP_EV_RAMP) {
#if S4_VERBOSE
				S4_PRINT("[S4] climb > 3 s without a peak: ramp, ignored\n");
#endif
			} else {
				/* nothing new */
			}
			s4_terrain_publish((INT)(p * 10.0f), (UINT)c, (INT)yaw, &hump);
		}
	}
}
