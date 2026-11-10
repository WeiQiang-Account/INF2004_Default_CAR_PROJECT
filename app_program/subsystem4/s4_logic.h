/* s4_logic.h - Buddy 4's PURE logic: no hardware, no kernel calls, no libm.
 * The same file is compiled on a PC for the host tests
 * (testing/subsystem4/host_tests/), so every rule can be checked without the
 * robot. Each state machine is an INSTANCE (a struct).
 *   calibration  : static offsets while the robot is still (FR16)
 *   safety       : impact / tilt decision (statechart 6.4 fault region)
 *   pitch filter : complementary filter, gyro + accelerometer
 *   hump         : FLAT -> CLIMBING -> ON_TOP -> DESCENDING, h = sum sin(pitch) ds (FR17)
 *   motion class : STATIONARY / MOVING / TURNING / ACCELERATING (FR18, FR19)   */
#ifndef S4_LOGIC_H
#define S4_LOGIC_H

#include <stdint.h>
#include <stdbool.h>
#include "s4_config.h"

/* ---------------------------------------------------------------- maths (no libm) */
float s4_sqrtf(float x);
float s4_sinf(float x_rad);
float s4_atan2f(float y, float x);          /* radians, -pi..pi */
#define S4_PI          3.14159265f
#define S4_RAD2DEG     (180.0f / S4_PI)
#define S4_DEG2RAD     (S4_PI / 180.0f)

/* ---------------------------------------------------------------- one sample */
typedef struct {
	float ax, ay, az;       /* g   (offsets removed after calibration) */
	float gx, gy, gz;       /* deg/s                                   */
} s4_sample_t;

/* ---------------------------------------------------------------- calibration (FR16) */
typedef struct {
	float    sum[6];
	uint16_t n;
	bool     moved;
	float    acc_off[3];    /* subtract from ax, ay, az (z keeps +1 g) */
	float    gyr_off[3];
	bool     valid;
} s4_cal_t;

void s4_cal_begin(s4_cal_t *c);
void s4_cal_feed(s4_cal_t *c, const s4_sample_t *raw);  /* raw = before offsets */
bool s4_cal_done(const s4_cal_t *c);                    /* S4_CAL_SAMPLES fed    */
bool s4_cal_end(s4_cal_t *c, bool accept_moving);       /* false = robot moved   */
void s4_cal_apply(const s4_cal_t *c, s4_sample_t *s);   /* in place              */

/* ---------------------------------------------------------------- safety */
#define S4_SAFE_OK      0u
#define S4_SAFE_IMPACT  (1u << 0)
#define S4_SAFE_TILT    (1u << 1)
typedef struct {
	uint8_t tilt_count;
	uint8_t calm_count;
	bool    latched;        /* fault seen, waiting for 1 s of calm */
	float   tilt_alpha;     /* 0 = tilt from each raw sample; >0 = smoothed (no gyro) */
	float   f[3];           /* smoothed accel used for the tilt test */
	bool    f_init;
} s4_safety_t;

void     s4_safety_init(s4_safety_t *s);
float    s4_accel_mag(const s4_sample_t *s);
float    s4_tilt_deg(const s4_sample_t *s);         /* angle from vertical */
/* Returns S4_SAFE_* bits that START a fault on this sample (0 if none).
 * *released becomes true on the sample where a latched fault may be cleared. */
uint32_t s4_safety_step(s4_safety_t *s, const s4_sample_t *smp, bool *released);

/* ---------------------------------------------------------------- pitch */
float s4_accel_pitch_deg(const s4_sample_t *s);     /* nose up = +       */
float s4_pitch_rate_dps(const s4_sample_t *s);      /* nose up = +       */
float s4_yaw_rate_dps(const s4_sample_t *s);        /* left turn = +     */
float s4_fwd_accel_g(const s4_sample_t *s);
/* Take the robot's own forward acceleration (a_g, from the wheels, + = speeding
 * up) out of the forward axis, so starting/braking is not seen as a tilt.
 * Needed without a gyro (GY-511): 0.3 g of start-up push = 17 deg fake pitch. */
void  s4_remove_fwd_accel(s4_sample_t *s, float a_g);
typedef struct { float pitch; bool init; float alpha; } s4_pitch_t;   /* alpha = gyro weight (S4_ALPHA) */
void  s4_pitch_init(s4_pitch_t *p);
float s4_pitch_step(s4_pitch_t *p, const s4_sample_t *s, float dt_s);

/* ---------------------------------------------------------------- hump (FR17) */
typedef enum { S4_FLAT = 0, S4_CLIMBING, S4_ON_TOP, S4_DESCENDING } s4_terrain_t;
#define S4_HUMP_EV_NONE   0
#define S4_HUMP_EV_START  1
#define S4_HUMP_EV_DONE   2
#define S4_HUMP_EV_RAMP   3     /* climb dropped (no peak within 3 s) */
typedef struct {
	s4_terrain_t state;
	uint8_t  count;         /* samples meeting the current exit test */
	float    h_mm;          /* height of the leading contact (probe) */
	float    ring[S4_PROBE_BINS];  /* probe heights over the last wheelbase of travel */
	float    bin_mm;        /* distance into the current bin         */
	uint8_t  idx;           /* oldest entry = height under the trailing contact */
	float    peak_mm;       /* max(h) of this hump                   */
	float    climb_mm;      /* distance travelled while CLIMBING     */
	float    stall_mm;      /* distance since the peak last rose     */
	bool     armed;         /* false after a ramp until back on flat ground */
	uint16_t last_peak_mm;  /* result of the last finished hump      */
	uint16_t max_peak_mm;   /* highest hump of the run (mission 5)   */
	uint16_t humps;         /* finished humps                        */
} s4_hump_t;

void    s4_hump_init(s4_hump_t *h);
/* pitch in deg, ds = distance moved this step in mm (mean of both wheels). */
uint8_t s4_hump_step(s4_hump_t *h, float pitch_deg, float ds_mm, uint32_t dt_ms);

/* ---------------------------------------------------------------- motion class (FR18) */
typedef enum { S4_STATIONARY = 0, S4_MOVING, S4_TURNING, S4_ACCELERATING } s4_mclass_t;
typedef struct { s4_mclass_t cls, cand; uint8_t count; } s4_class_t;
void        s4_class_init(s4_class_t *c);
s4_mclass_t s4_class_step(s4_class_t *c, float yaw_dps, float speed_mmps, float fwd_accel_g);

#endif /* S4_LOGIC_H */
