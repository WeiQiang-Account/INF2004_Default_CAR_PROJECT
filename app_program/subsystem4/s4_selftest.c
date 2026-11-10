/*
 *	s4_selftest.c
 *	Subsystem 4 -- checks of the PURE logic (s4_logic.c). No hardware, no kernel.
 *	Run on the robot by subsystem4_test.c (make TEST=1) AND on a PC by
 *	testing/subsystem4/host_tests, so both mean the same thing.
 */
#include "s4_selftest.h"
#include "s4_logic.h"

static float fabs_(float v) { return (v < 0.0f) ? -v : v; }
static bool near(float a, float b, float tol) { return fabs_(a - b) <= tol; }

static s4_sample_t level(void)
{
	s4_sample_t s = { 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f };
	return s;
}

/* S4-U01: sqrt / sin / atan2 replacements (the template links no libm) */
bool s4_st_math(void)
{
	return near(s4_sqrtf(2.0f), 1.41421356f, 1e-5f) && near(s4_sqrtf(0.0f), 0.0f, 1e-9f) &&
	       near(s4_sinf(30.0f * S4_DEG2RAD), 0.5f, 1e-5f) && near(s4_sinf(-100.0f * S4_DEG2RAD), -0.98480775f, 1e-4f) &&
	       near(s4_atan2f(1.0f, 1.0f) * S4_RAD2DEG, 45.0f, 0.01f) &&
	       near(s4_atan2f(1.0f, -1.0f) * S4_RAD2DEG, 135.0f, 0.01f) &&
	       near(s4_atan2f(-1.0f, -1.0f) * S4_RAD2DEG, -135.0f, 0.01f) &&
	       near(s4_atan2f(0.1736482f, 0.9848078f) * S4_RAD2DEG, 10.0f, 0.01f);
}

/* S4-U02: static offsets (FR16); a moving robot is refused */
bool s4_st_calibration(void)
{
	s4_cal_t c;
	s4_sample_t r = { 0.02f, -0.03f, 1.04f, 1.5f, -0.7f, 0.3f }, t;
	int i;
	bool ok;

	s4_cal_begin(&c);
	for (i = 0; i < S4_CAL_SAMPLES; i++) { s4_cal_feed(&c, &r); }
	ok = s4_cal_done(&c) && s4_cal_end(&c, false);
	t = r;
	s4_cal_apply(&c, &t);
	ok = ok && near(t.ax, 0.0f, 1e-4f) && near(t.ay, 0.0f, 1e-4f) && near(t.az, 1.0f, 1e-4f) &&
	     near(t.gx, 0.0f, 1e-4f) && near(t.gz, 0.0f, 1e-4f);

	s4_cal_begin(&c);
	for (i = 0; i < S4_CAL_SAMPLES; i++) {
		s4_sample_t m = r;
		if (i == 50) { m.gz = 20.0f; }                 /* someone bumped it */
		s4_cal_feed(&c, &m);
	}
	ok = ok && !s4_cal_end(&c, false) && s4_cal_end(&c, true);   /* last try accepts */
	return ok;
}

/* S4-U03: impact on ONE sample (NFR14), tilt after 60 ms, release after 1 s calm */
bool s4_st_safety(void)
{
	s4_safety_t s;
	s4_sample_t lv = level(), hit = { 1.8f, 0.0f, 1.9f, 0.0f, 0.0f, 0.0f }, tilt = { 0.0f, 0.5736f, 0.8192f, 0.0f, 0.0f, 0.0f };
	bool rel = false, ok = true;
	int i;

	s4_safety_init(&s);
	ok = ok && (s4_safety_step(&s, &lv, &rel) == S4_SAFE_OK);
	ok = ok && (s4_safety_step(&s, &hit, &rel) == S4_SAFE_IMPACT);          /* |a| = 2.6 g */
	for (i = 0; i < (S4_CLEAR_SAMPLES - 1); i++) { ok = ok && (s4_safety_step(&s, &lv, &rel) == S4_SAFE_OK) && !rel; }
	ok = ok && (s4_safety_step(&s, &lv, &rel) == S4_SAFE_OK) && rel;          /* released at 1 s */

	s4_safety_init(&s);                                                        /* 35 deg tilt */
	ok = ok && (s4_safety_step(&s, &tilt, &rel) == S4_SAFE_OK);
	ok = ok && (s4_safety_step(&s, &tilt, &rel) == S4_SAFE_OK);
	ok = ok && (s4_safety_step(&s, &tilt, &rel) == S4_SAFE_TILT);
	ok = ok && (s4_safety_step(&s, &tilt, &rel) == S4_SAFE_OK);              /* latched: reported once */
	{   /* a 15 deg hump slope is NOT a tilt fault */
		s4_sample_t slope = { 0.0f, 0.2588f, 0.9659f, 0.0f, 0.0f, 0.0f };
		s4_safety_init(&s);
		for (i = 0; i < 10; i++) { ok = ok && (s4_safety_step(&s, &slope, &rel) == S4_SAFE_OK); }
	}
	{   /* no gyro (smoothed tilt): a 60 ms shake that reads 50 deg is NOT a tip-over,
	       a real 50 deg tip-over is still caught within 0.3 s */
		s4_sample_t shake = { 0.0f, 0.99f, 0.83f, 0.0f, 0.0f, 0.0f };     /* 1.29 g, 50 deg */
		s4_sample_t over  = { 0.0f, 0.766f, 0.643f, 0.0f, 0.0f, 0.0f };   /* 50 deg, 1 g   */
		uint32_t got = S4_SAFE_OK;
		s4_safety_init(&s);
		s.tilt_alpha = S4_TILT_ALPHA_NOGYRO;
		for (i = 0; i < 5; i++)  { got |= s4_safety_step(&s, &lv, &rel); }
		for (i = 0; i < 3; i++)  { got |= s4_safety_step(&s, &shake, &rel); }
		for (i = 0; i < 20; i++) { got |= s4_safety_step(&s, &lv, &rel); }
		ok = ok && (got == S4_SAFE_OK);
		for (i = 0; (i < 15) && (got == S4_SAFE_OK); i++) { got |= s4_safety_step(&s, &over, &rel); }
		ok = ok && (got == S4_SAFE_TILT);
	}
	return ok;
}

/* S4-U04: pitch sign and filter: accelerometer and gyro agree on "nose up" */
bool s4_st_pitch(void)
{
	s4_pitch_t p;
	s4_sample_t s = { 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f };
	float v = 0.0f, rate;
	int i;
	bool ok;

	/* tilt the nose up by 10 deg (sign per S4_FWD_AXIS_Y / S4_PITCH_SIGN) */
#if S4_FWD_AXIS_Y
	s.ay = (float)S4_PITCH_SIGN * 0.1736482f;
#else
	s.ax = (float)S4_PITCH_SIGN * 0.1736482f;
#endif
	s.az = 0.9848078f;
	ok = near(s4_accel_pitch_deg(&s), 10.0f, 0.05f);
	s4_pitch_init(&p);
	for (i = 0; i < 200; i++) { v = s4_pitch_step(&p, &s, 0.02f); }
	ok = ok && near(v, 10.0f, 0.1f);

	/* nose-up rotation on the gyro gives a positive pitch rate */
	s = level();
#if S4_FWD_AXIS_Y
	s.gx = (float)S4_PITCH_SIGN * 20.0f;
#else
	s.gy = -(float)S4_PITCH_SIGN * 20.0f;
#endif
	rate = s4_pitch_rate_dps(&s);
	ok = ok && near(rate, 20.0f, 1e-4f);

	/* level robot speeding up at 0.3 g looks 17 deg nose-up until the wheel
	   acceleration is taken out (GY-511, no gyro) */
	s = level();
#if S4_FWD_AXIS_Y
	s.ay = (float)S4_PITCH_SIGN * 0.3f;
#else
	s.ax = (float)S4_PITCH_SIGN * 0.3f;
#endif
	ok = ok && (s4_accel_pitch_deg(&s) > 15.0f);
	s4_remove_fwd_accel(&s, 0.3f);
	return ok && near(s4_accel_pitch_deg(&s), 0.0f, 0.05f);
}

/* drive over a trapezoid hump: up at +a deg for 'run' mm, top 'top' mm, down -a deg */
static uint8_t drive(s4_hump_t *h, float deg, float run_mm, float step_mm, uint8_t *seen)
{
	float d;
	uint8_t ev, last = S4_HUMP_EV_NONE;
	for (d = 0.0f; d < run_mm; d += step_mm) {
		ev = s4_hump_step(h, deg, step_mm, 20u);
		if (ev != S4_HUMP_EV_NONE) { *seen |= (uint8_t)(1u << ev); last = ev; }
	}
	return last;
}

/* Ground height at x for a hump starting at x = 0: ramp up 'up' mm to 'hgt',
   flat top 'top' mm, ramp down 'dn' mm. */
typedef struct { float up, top, dn, hgt; } hump_shape_t;
static float ground(const hump_shape_t *g, float x)
{
	if (x <= 0.0f) { return 0.0f; }
	if (x < g->up) { return g->hgt * x / g->up; }
	x -= g->up;
	if (x < g->top) { return g->hgt; }
	x -= g->top;
	if (x < g->dn) { return g->hgt * (1.0f - (x / g->dn)); }
	return 0.0f;
}

/* Push the real robot shape over it: trailing contact at x, leading contact at
   x + wheelbase, body pitch from the two ground heights, 2 mm per 20 ms. */
static void roll_over(s4_hump_t *h, const hump_shape_t *g, uint8_t *seen)
{
	const float L = (float)S4_WHEELBASE_MM;
	float x;
	for (x = -L - 100.0f; x < (g->up + g->top + g->dn + 150.0f); x += 2.0f) {
		float d = ground(g, x + L) - ground(g, x);
		float deg = s4_atan2f(d, s4_sqrtf((L * L) - (d * d))) * S4_RAD2DEG;
		uint8_t ev = s4_hump_step(h, deg, 2.0f, 20u);
		if (ev != S4_HUMP_EV_NONE) { *seen |= (uint8_t)(1u << ev); }
	}
}

/* S4-U05: humps of known height and different shapes -> START, DONE, peak
   within +-1 mm (NFR15 budget +-5 mm), including a peak with no flat top
   (like the course hump) and a top shorter than the robot. */
bool s4_st_hump(void)
{
	s4_hump_t h;
	uint8_t seen = 0;
	bool ok;
	const hump_shape_t trap  = { 115.0f, 60.0f, 115.0f, 20.0f };   /* top shorter than the robot */
	const hump_shape_t small = {  72.0f,  0.0f,  72.0f, 10.0f };
	const hump_shape_t peak  = { 200.0f,  0.0f, 120.0f, 23.0f };   /* like the orange course hump */

	s4_hump_init(&h);
	roll_over(&h, &trap, &seen);
	ok = ((seen & (1u << S4_HUMP_EV_START)) != 0u) && ((seen & (1u << S4_HUMP_EV_DONE)) != 0u);
	ok = ok && (h.humps == 1u) && (h.last_peak_mm >= 19u) && (h.last_peak_mm <= 21u);

	/* a second, smaller hump: the run maximum keeps the bigger one */
	roll_over(&h, &small, &seen);
	ok = ok && (h.humps == 2u) && (h.last_peak_mm >= 9u) && (h.last_peak_mm <= 11u);
	ok = ok && (h.max_peak_mm >= 19u) && (h.max_peak_mm <= 21u);

	roll_over(&h, &peak, &seen);
	ok = ok && (h.humps == 3u) && (h.last_peak_mm >= 22u) && (h.last_peak_mm <= 24u);
	ok = ok && (h.max_peak_mm >= 22u) && (h.max_peak_mm <= 24u);
	return ok;
}

/* S4-U06: a 40 ms spike is not a hump; a long climb is a ramp (dropped) */
bool s4_st_hump_reject(void)
{
	s4_hump_t h;
	uint8_t seen = 0;
	const hump_shape_t ramp = { 800.0f, 400.0f, 800.0f, 84.0f };   /* 6 deg for 800 mm */
	const float L = (float)S4_WHEELBASE_MM;
	float x;

	s4_hump_init(&h);
	(void)drive(&h, 12.0f, 4.0f, 2.0f, &seen);           /* 2 samples = 40 ms */
	(void)drive(&h, 0.0f, 50.0f, 2.0f, &seen);
	if (seen != 0u) { return false; }
	for (x = -L - 100.0f; x < 500.0f; x += 2.0f) {       /* both contacts well up the ramp */
		float d = ground(&ramp, x + L) - ground(&ramp, x);
		uint8_t ev = s4_hump_step(&h, s4_atan2f(d, s4_sqrtf((L * L) - (d * d))) * S4_RAD2DEG, 2.0f, 20u);
		if (ev != S4_HUMP_EV_NONE) { seen |= (uint8_t)(1u << ev); }
	}
	return ((seen & (1u << S4_HUMP_EV_RAMP)) != 0u) && ((seen & (1u << S4_HUMP_EV_DONE)) == 0u) && (h.humps == 0u);
}

/* S4-U07: motion classes, held 60 ms before they change (stable) */
bool s4_st_class(void)
{
	s4_class_t c;
	bool ok = true;
	int i;

	s4_class_init(&c);
	ok = ok && (s4_class_step(&c, 0.0f, 0.0f, 0.0f) == S4_STATIONARY);
	ok = ok && (s4_class_step(&c, 0.0f, 150.0f, 0.0f) == S4_STATIONARY);   /* 1 sample: not yet */
	ok = ok && (s4_class_step(&c, 0.0f, 150.0f, 0.0f) == S4_STATIONARY);
	ok = ok && (s4_class_step(&c, 0.0f, 150.0f, 0.0f) == S4_MOVING);       /* 3 samples */
	for (i = 0; i < 3; i++) { (void)s4_class_step(&c, 90.0f, 0.0f, 0.0f); }
	ok = ok && (c.cls == S4_TURNING);
	for (i = 0; i < 3; i++) { (void)s4_class_step(&c, 0.0f, 50.0f, 0.3f); }
	ok = ok && (c.cls == S4_ACCELERATING);
	return ok;
}
