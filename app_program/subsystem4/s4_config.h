/* s4_config.h - Buddy 4 tunables (IMU safety + terrain).
 * Every value is #ifndef-guarded so it can be overridden from the makefile.
 * Values marked MEASURE must be confirmed on the real robot (Week 6 A7/A8). */
#ifndef S4_CONFIG_H
#define S4_CONFIG_H

/* ---- Pins / bus (Week 6 pin table: MPU-6050 on I2C0, Grove 3) ------------ */
#define S4_PIN_SDA            4    /* GP4 = I2C0 SDA                           */
#define S4_PIN_SCL            5    /* GP5 = I2C0 SCL                           */
#define S4_I2C_HZ        400000u   /* fast mode                                */
#define S4_MPU_ADDR        0x68u   /* AD0 low (GY-521 default)                 */
/* 2026-11-07: the robot has a GY-511 (LSM303: accelerometer + compass, NO gyro)
 * on GP4/GP5. The driver finds whichever chip answers: MPU-6050 at 0x68/0x69,
 * else LSM303 accelerometer at 0x19/0x18. Without a gyro: pitch comes from the
 * accelerometer only (S4_ALPHA_NOGYRO) and the turn rate from the wheel encoders. */
#define S4_LSM_ADDR        0x19u   /* LSM303DLHC / LSM303AGR accelerometer     */
#define S4_I2C_TMO_US      2000u   /* one transfer must finish within this     */

/* ---- Task priorities (1 = highest; TEAM.md section 4) -------------------- */
#ifndef S4_PRI_SAFETY
#define S4_PRI_SAFETY        10    /* ABOVE Motion (14): safety first (Week 6) */
#endif
#ifndef S4_PRI_TERRAIN
#define S4_PRI_TERRAIN       19    /* below Line Follow, VC and Barcode        */
#endif
#define S4_PERIOD_MS         20    /* NFR13: 50 Hz                             */

/* ---- Mounting: which IMU axis points FORWARD (MEASURE) --------------------
 * 1 = Y forward (X to the right, Z up)  -> pitch rate = +gyro X
 * 0 = X forward (Y to the left,  Z up)  -> pitch rate = -gyro Y
 * Flip S4_PITCH_SIGN if "nose up" shows a NEGATIVE pitch in telemetry.       */
#ifndef S4_FWD_AXIS_Y
#define S4_FWD_AXIS_Y         1
#endif
#ifndef S4_PITCH_SIGN
#define S4_PITCH_SIGN        (-1)   /* 2026-11-07: GY-511 showed nose up (IR end lifted) as NEGATIVE, so flipped */
#endif

/* ---- Calibration (FR16) --------------------------------------------------- */
#define S4_CAL_SAMPLES      100    /* 100 x 20 ms = 2 s                        */
#define S4_CAL_STILL_DPS    3.0f   /* gyro must stay below this (robot still)  */
#define S4_CAL_STILL_G      0.05f  /* |a| must stay within 1 g +/- this        */
#define S4_CAL_TRIES          2    /* VC needs "IMU ready" within 5 s          */

/* ---- Safety (statechart 6.4, A8: MEASURE the thresholds) ----------------- */
#define S4_IMPACT_G          2.5f  /* |a| above this = impact (one sample)     */
#define S4_TILT_DEG         30.0f  /* tilt from vertical above this ...        */
#define S4_TILT_SAMPLES       3    /* ... for 3 x 20 ms = tilt fault           */
/* Compass (GY-511) heading, used by Motion to hold a straight line (2026-11-10).
 * Shown on the dashboard as "Compass heading", LEFT turn = heading goes UP.
 * Check: turn the robot LEFT by hand -> the number must go UP. If it goes
 * DOWN, set S4_MAG_SIGN to -1.                                              */
#ifndef S4_MAG_SIGN
#define S4_MAG_SIGN           1
#endif
#ifndef S4_HDG_ALPHA
#define S4_HDG_ALPHA        0.6f   /* smoothing of the compass (0 = none, 0.9 = heavy) */
#endif
#ifndef S4_TILT_ALPHA_NOGYRO
#define S4_TILT_ALPHA_NOGYRO 0.85f /* no gyro: tilt from smoothed accel (~0.1 s), so shaking is not a tip-over */
#endif
#define S4_CLEAR_SAMPLES     50    /* level and calm for 1 s -> release E-stop */
#define S4_LOST_READS         3    /* failed reads in a row -> IMU LOST        */
#define S4_RETRY_MS         500    /* retry period while LOST                  */

/* ---- Pitch filter and hump detection (FR17, NFR15) ----------------------- */
#define S4_ALPHA            0.98f  /* complementary filter: gyro weight        */
#ifndef S4_ALPHA_NOGYRO
#define S4_ALPHA_NOGYRO     0.80f  /* no gyro (GY-511): smooth the accel pitch, ~80 ms lag. Higher = smoother but slower */
#endif
/* 2026-11-07: start/end are tested on the measured HEIGHT, not the pitch. The
 * whole robot only tilts asin(20/230) = 5 deg on a 2 cm hump, so the old
 * "pitch > 5 deg" start missed real humps.                                    */
#ifndef S4_HUMP_START_MM
#define S4_HUMP_START_MM     8.0f  /* probe height above this (~2 deg) ...      */
#endif
#define S4_CLIMB_SAMPLES      3    /* ... for 3 samples while moving = HUMP_START */
#define S4_HUMP_DROP_MM      3.0f  /* this far below the peak = coming down     */
#define S4_HUMP_END_MM       4.0f  /* probe height below this ...               */
#define S4_LEVEL_SAMPLES      5    /* ... for 100 ms = HUMP_DONE               */
#define S4_RAMP_MM          300    /* still rising after this distance = ramp, dropped (distance, so speed does not matter) */
#define S4_TOP_MM            50    /* no new peak for this distance = on the flat top */
#define S4_HUMP_MAX_MM      511    /* 9-bit event argument                     */
/* Hump height uses the robot as a probe (2026-11-07): the leading ground contact
 * is S4_WHEELBASE_MM ahead of the trailing one, and the trailing one later rolls
 * over the same spot. So  height(lead) = height(trail) + WHEELBASE x sin(pitch),
 * where height(trail) is the lead height stored one wheelbase of travel ago.
 * This gives the real height of any hump shape (a peak, or a top shorter than
 * the robot), which "h += sin(pitch) ds" could not: it gave the AVERAGE height
 * under the robot (~12 mm for the 23 mm course hump).                         */
#ifndef S4_WHEELBASE_MM
#define S4_WHEELBASE_MM     80     /* MEASURED 2026-11-07: wheel axle to ball-caster centre, mm */
#endif
#define S4_PROBE_BIN_MM      5     /* distance step of the stored heights       */
#define S4_PROBE_BINS      ((S4_WHEELBASE_MM + (S4_PROBE_BIN_MM / 2)) / S4_PROBE_BIN_MM)
#if (S4_PROBE_BINS < 4) || (S4_PROBE_BINS > 120)
#error "S4_WHEELBASE_MM must be between 20 and 600 mm"
#endif

/* ---- Motion class (FR18, FR19) ------------------------------------------- */
#define S4_TURN_DPS         30.0f  /* |yaw rate| above this = TURNING          */
#define S4_MOVING_MMPS      20.0f  /* wheel speed above this = MOVING          */
#define S4_ACCEL_G          0.15f  /* forward accel above this = ACCELERATING  */
#define S4_CLASS_SAMPLES      3    /* new class must hold 60 ms (stable)       */

/* ---- Hardware watchdog (NFR20) --------------------------------------------
 * Fed by the IMU Safety task only while BOTH it and Motion Control complete
 * their 20 ms cycles. Off in test builds (tests may pause tasks) and in
 * the S2 motor demo.           */
#include "../subsystem2/motion_config.h"	/* S2_MOTOR_DEMO: no motion task, nothing to watch */
/* 2026-11-06: OFF. With the hardware watchdog running, every microsecond
 * measurement went ~13x too small (ultrasonic, barcode) and, once the us tick
 * was corrected, the board reset before USB came up. Bench tests #6/#7 (watchdog
 * off) were clean with and without the tick fix. Re-enable only after it has
 * been re-tested on the robot (TEST_LOG S4-M04).                               */
#ifndef S4_WATCHDOG_ENABLE
#define S4_WATCHDOG_ENABLE    0
#endif
#define S4_WATCHDOG_MS      100    /* 5 x 20 ms                                */
#define S4_MOTION_STALE_CYC   4    /* motion may miss this many of our cycles  */

#ifndef S4_VERBOSE
#define S4_VERBOSE            1    /* one console line per hump / fault event  */
#endif

#endif /* S4_CONFIG_H */
