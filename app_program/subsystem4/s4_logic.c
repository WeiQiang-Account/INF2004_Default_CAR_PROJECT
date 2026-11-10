/*
 *	s4_logic.c
 *	Subsystem 4 -- IMU logic. PURE (no hardware, no kernel calls, no libm:
 *	the template does not link the maths library, so sqrt/sin/atan2 are here).
 */
#include "s4_logic.h"

/* ================================================================ maths */
float s4_sqrtf(float x)
{
	union { float f; uint32_t u; } v;
	float y;
	int   i;

	if (x <= 0.0f) {
		return 0.0f;
	}
	v.f = x;
	v.u = 0x1FBD1DF5u + (v.u >> 1);          /* first guess */
	y = v.f;
	for (i = 0; i < 3; i++) {
		y = 0.5f * (y + (x / y));            /* Newton: error < 1e-6 relative */
	}
	return y;
}

float s4_sinf(float x)
{
	float x2;

	while (x > S4_PI)  { x -= 2.0f * S4_PI; }
	while (x < -S4_PI) { x += 2.0f * S4_PI; }
	if (x > (0.5f * S4_PI))  { x = S4_PI - x; }
	if (x < (-0.5f * S4_PI)) { x = -S4_PI - x; }
	x2 = x * x;                              /* Taylor to x^9: error < 4e-6 on [-pi/2, pi/2] */
	return x * (1.0f - x2 / 6.0f * (1.0f - x2 / 20.0f * (1.0f - x2 / 42.0f * (1.0f - x2 / 72.0f))));
}

static float atan_unit(float z)              /* |z| <= 1, error ~1e-5 rad */
{
	float z2 = z * z;
	return z * (0.9998660f + z2 * (-0.3302995f + z2 * (0.1801410f + z2 * (-0.0851330f + z2 * 0.0208351f))));
}

float s4_atan2f(float y, float x)
{
	float ax = (x < 0.0f) ? -x : x;
	float ay = (y < 0.0f) ? -y : y;
	float a;

	if ((ax == 0.0f) && (ay == 0.0f)) {
		return 0.0f;
	}
	if (ax >= ay) {
		a = atan_unit(ay / ax);
	} else {
		a = (0.5f * S4_PI) - atan_unit(ax / ay);
	}
	if (x < 0.0f) {
		a = S4_PI - a;
	}
	return (y < 0.0f) ? -a : a;
}

static float absf(float v) { return (v < 0.0f) ? -v : v; }

/* ================================================================ calibration */
void s4_cal_begin(s4_cal_t *c)
{
	int i;
	for (i = 0; i < 6; i++) { c->sum[i] = 0.0f; }
	for (i = 0; i < 3; i++) { c->acc_off[i] = 0.0f; c->gyr_off[i] = 0.0f; }
	c->n = 0;
	c->moved = false;
	c->valid = false;
}

void s4_cal_feed(s4_cal_t *c, const s4_sample_t *r)
{
	float g = s4_sqrtf((r->gx * r->gx) + (r->gy * r->gy) + (r->gz * r->gz));
	float a = s4_sqrtf((r->ax * r->ax) + (r->ay * r->ay) + (r->az * r->az));

	if ((g > S4_CAL_STILL_DPS) || (absf(a - 1.0f) > S4_CAL_STILL_G)) {
		c->moved = true;                     /* robot moved: these offsets are not static */
	}
	c->sum[0] += r->ax; c->sum[1] += r->ay; c->sum[2] += r->az;
	c->sum[3] += r->gx; c->sum[4] += r->gy; c->sum[5] += r->gz;
	if (c->n < 0xFFFFu) { c->n++; }
}

bool s4_cal_done(const s4_cal_t *c) { return c->n >= S4_CAL_SAMPLES; }

bool s4_cal_end(s4_cal_t *c, bool accept_moving)
{
	float n;

	if ((c->n == 0u) || (c->moved && !accept_moving)) {
		c->valid = false;
		return false;
	}
	n = (float)c->n;
	c->acc_off[0] = c->sum[0] / n;           /* level robot: x = y = 0 g ...      */
	c->acc_off[1] = c->sum[1] / n;
	c->acc_off[2] = (c->sum[2] / n) - 1.0f;  /* ... and z = +1 g                  */
	c->gyr_off[0] = c->sum[3] / n;
	c->gyr_off[1] = c->sum[4] / n;
	c->gyr_off[2] = c->sum[5] / n;
	c->valid = true;
	return true;
}

void s4_cal_apply(const s4_cal_t *c, s4_sample_t *s)
{
	s->ax -= c->acc_off[0]; s->ay -= c->acc_off[1]; s->az -= c->acc_off[2];
	s->gx -= c->gyr_off[0]; s->gy -= c->gyr_off[1]; s->gz -= c->gyr_off[2];
}

/* ================================================================ safety */
void s4_safety_init(s4_safety_t *s)
{
	s->tilt_count = 0;
	s->calm_count = 0;
	s->latched = false;
	s->tilt_alpha = 0.0f;
	s->f_init = false;
}

float s4_accel_mag(const s4_sample_t *s)
{
	return s4_sqrtf((s->ax * s->ax) + (s->ay * s->ay) + (s->az * s->az));
}

float s4_tilt_deg(const s4_sample_t *s)
{
	float horiz = s4_sqrtf((s->ax * s->ax) + (s->ay * s->ay));
	return s4_atan2f(horiz, s->az) * S4_RAD2DEG;   /* 0 = level, 90 = on its side */
}

uint32_t s4_safety_step(s4_safety_t *s, const s4_sample_t *smp, bool *released)
{
	float    mag  = s4_accel_mag(smp);       /* impact: raw sample (NFR14, no delay) */
	float    tilt;
	uint32_t start = S4_SAFE_OK;

	/* Tilt from smoothed accel when there is no gyro (GY-511): shaking and the
	   robot's own speeding up / braking push the raw reading 50 deg off for a
	   moment (2026-11-07: "tilt 50 deg at 1.29 g" while driving on a floor).
	   A real tip-over lasts, so it still shows after ~0.1 s.               */
	if (s->tilt_alpha > 0.0f) {
		s4_sample_t f = *smp;
		if (!s->f_init) {
			s->f[0] = smp->ax; s->f[1] = smp->ay; s->f[2] = smp->az;
			s->f_init = true;
		} else {
			s->f[0] = (s->tilt_alpha * s->f[0]) + ((1.0f - s->tilt_alpha) * smp->ax);
			s->f[1] = (s->tilt_alpha * s->f[1]) + ((1.0f - s->tilt_alpha) * smp->ay);
			s->f[2] = (s->tilt_alpha * s->f[2]) + ((1.0f - s->tilt_alpha) * smp->az);
		}
		f.ax = s->f[0]; f.ay = s->f[1]; f.az = s->f[2];
		tilt = s4_tilt_deg(&f);
	} else {
		tilt = s4_tilt_deg(smp);
	}

	*released = false;

	if (tilt > S4_TILT_DEG) {
		if (s->tilt_count < 255u) { s->tilt_count++; }
	} else {
		s->tilt_count = 0;
	}

	if (!s->latched) {
		if (mag > S4_IMPACT_G) {
			start |= S4_SAFE_IMPACT;         /* NFR14: decided on this very sample */
		}
		if (s->tilt_count >= S4_TILT_SAMPLES) {
			start |= S4_SAFE_TILT;
		}
		if (start != S4_SAFE_OK) {
			s->latched = true;
			s->calm_count = 0;
		}
	} else {
		/* release only after 1 s level and calm, so the robot can be reset */
		if ((absf(mag - 1.0f) < 0.2f) && (tilt < (S4_TILT_DEG - 10.0f))) {
			if (s->calm_count < 255u) { s->calm_count++; }
			if (s->calm_count >= S4_CLEAR_SAMPLES) {
				s->latched = false;
				s->calm_count = 0;
				*released = true;
			}
		} else {
			s->calm_count = 0;
		}
	}
	return start;
}

/* ================================================================ pitch */
static float fwd_axis(const s4_sample_t *s)
{
#if S4_FWD_AXIS_Y
	return (float)S4_PITCH_SIGN * s->ay;
#else
	return (float)S4_PITCH_SIGN * s->ax;
#endif
}

float s4_accel_pitch_deg(const s4_sample_t *s)
{
	return s4_atan2f(fwd_axis(s), s->az) * S4_RAD2DEG;
}

float s4_pitch_rate_dps(const s4_sample_t *s)
{
	/* right-hand rule with Z up: Y forward -> X points right, nose up = +X rotation;
	   X forward -> Y points left, nose up = -Y rotation */
#if S4_FWD_AXIS_Y
	return (float)S4_PITCH_SIGN * s->gx;
#else
	return -(float)S4_PITCH_SIGN * s->gy;
#endif
}

float s4_yaw_rate_dps(const s4_sample_t *s) { return s->gz; }   /* Z up: left turn = + */
float s4_fwd_accel_g(const s4_sample_t *s)  { return fwd_axis(s); }

void s4_remove_fwd_accel(s4_sample_t *s, float a_g)
{
	/* fwd_axis() = PITCH_SIGN x raw, so this lowers fwd_axis() by exactly a_g */
#if S4_FWD_AXIS_Y
	s->ay -= (float)S4_PITCH_SIGN * a_g;
#else
	s->ax -= (float)S4_PITCH_SIGN * a_g;
#endif
}

void s4_pitch_init(s4_pitch_t *p)
{
	p->pitch = 0.0f;
	p->init = false;
	p->alpha = S4_ALPHA;
}

float s4_pitch_step(s4_pitch_t *p, const s4_sample_t *s, float dt_s)
{
	float acc = s4_accel_pitch_deg(s);

	if (!p->init) {
		p->pitch = acc;                      /* start from the accelerometer */
		p->init = true;
	} else {
		/* gyro and accelerometer measure the SAME angle (both nose-up pitch) */
		p->pitch = (p->alpha * (p->pitch + (s4_pitch_rate_dps(s) * dt_s))) + ((1.0f - p->alpha) * acc);
	}
	return p->pitch;
}

/* ================================================================ hump */
void s4_hump_init(s4_hump_t *h)
{
	h->state = S4_FLAT;
	h->count = 0;
	h->h_mm = 0.0f;
	h->peak_mm = 0.0f;
	h->climb_mm = 0.0f;
	h->stall_mm = 0.0f;
	h->armed = true;
	h->last_peak_mm = 0;
	h->max_peak_mm = 0;
	h->humps = 0;
	h->bin_mm = 0.0f;
	h->idx = 0;
	for (unsigned i = 0; i < (unsigned)S4_PROBE_BINS; i++) { h->ring[i] = 0.0f; }
}

/* Probe: one ring entry per S4_PROBE_BIN_MM of forward travel. ring[idx] is
   the leading-contact height stored one wheelbase ago = the ground under the
   trailing contact now. On FLAT ground the trailing contact is on the floor
   (height 0), so pitch alone gives the lead height: no drift can build up. */
static float lead_height(const s4_hump_t *h, float pitch)
{
	float base = (h->state == S4_FLAT) ? 0.0f : h->ring[h->idx];
	return base + ((float)S4_WHEELBASE_MM * s4_sinf(pitch * S4_DEG2RAD));
}

static void probe_step(s4_hump_t *h, float pitch, float ds)
{
	h->h_mm = lead_height(h, pitch);
	if (ds <= 0.0f) { return; }                          /* stopped or reversing: keep */
	h->bin_mm += ds;
	while (h->bin_mm >= (float)S4_PROBE_BIN_MM) {
		h->ring[h->idx] = lead_height(h, pitch);
		h->idx = (uint8_t)((h->idx + 1u) % (unsigned)S4_PROBE_BINS);
		h->bin_mm -= (float)S4_PROBE_BIN_MM;
	}
}

static void peak_update(s4_hump_t *h)
{
	if (h->h_mm > h->peak_mm) { h->peak_mm = h->h_mm; }
}

uint8_t s4_hump_step(s4_hump_t *h, float pitch, float ds, uint32_t dt_ms)
{
	uint8_t ev = S4_HUMP_EV_NONE;
	(void)dt_ms;

	probe_step(h, pitch, ds);                            /* h->h_mm = height of the leading contact */

	switch (h->state) {
	case S4_FLAT:
		if (h->h_mm < S4_HUMP_END_MM) { h->armed = true; }
		if (h->armed && (ds > 0.0f) && (h->h_mm > S4_HUMP_START_MM)) {   /* only while moving: lifting it by hand is no hump */
			if (++h->count >= S4_CLIMB_SAMPLES) {
				h->state = S4_CLIMBING;
				h->count = 0;
				h->peak_mm = h->h_mm;
				h->climb_mm = 0.0f;
				h->stall_mm = 0.0f;
				ev = S4_HUMP_EV_START;
			}
		} else if (h->h_mm <= S4_HUMP_START_MM) {
			h->count = 0;
		}
		break;

	case S4_CLIMBING:
		if (ds > 0.0f) { h->climb_mm += ds; h->stall_mm += ds; }
		if (h->h_mm > (h->peak_mm + 0.5f)) { h->peak_mm = h->h_mm; h->stall_mm = 0.0f; }
		if (h->h_mm < (h->peak_mm - S4_HUMP_DROP_MM)) {
			h->state = S4_DESCENDING;            /* past the top */
			h->count = 0;
		} else if (h->stall_mm > (float)S4_TOP_MM) {
			h->state = S4_ON_TOP;                /* stopped rising: a flat top */
		} else if (h->climb_mm > (float)S4_RAMP_MM) {   /* still rising: a ramp, not a hump */
			h->state = S4_FLAT;
			h->count = 0;
			h->armed = false;                    /* no new hump until level again */
			ev = S4_HUMP_EV_RAMP;
		}
		break;

	case S4_ON_TOP:
		peak_update(h);
		if (h->h_mm < (h->peak_mm - S4_HUMP_DROP_MM)) {
			h->state = S4_DESCENDING;
			h->count = 0;
		}
		break;

	case S4_DESCENDING:
	default:
		peak_update(h);                              /* a second rise of the same hump */
		if (h->h_mm < S4_HUMP_END_MM) {
			if (++h->count >= S4_LEVEL_SAMPLES) {    /* front back down for 100 ms: done */
				float pk = h->peak_mm + 0.5f;
				uint16_t mm = (pk <= 0.0f) ? 0u : ((pk >= (float)S4_HUMP_MAX_MM) ? (uint16_t)S4_HUMP_MAX_MM : (uint16_t)pk);
				h->last_peak_mm = mm;
				if (mm > h->max_peak_mm) { h->max_peak_mm = mm; }
				if (h->humps < 0xFFFFu) { h->humps++; }
				h->state = S4_FLAT;
				h->count = 0;
				ev = S4_HUMP_EV_DONE;
			}
		} else {
			h->count = 0;
		}
		break;
	}
	return ev;
}

/* ================================================================ motion class */
void s4_class_init(s4_class_t *c)
{
	c->cls = S4_STATIONARY;
	c->cand = S4_STATIONARY;
	c->count = 0;
}

s4_mclass_t s4_class_step(s4_class_t *c, float yaw_dps, float speed_mmps, float lin_accel_g)
{
	s4_mclass_t now;

	if (absf(yaw_dps) > S4_TURN_DPS) {
		now = S4_TURNING;
	} else if (absf(lin_accel_g) > S4_ACCEL_G) {
		now = S4_ACCELERATING;
	} else if (absf(speed_mmps) > S4_MOVING_MMPS) {
		now = S4_MOVING;
	} else {
		now = S4_STATIONARY;
	}

	if (now == c->cls) {
		c->count = 0;                            /* still the same class */
	} else if (now == c->cand) {
		if (++c->count >= S4_CLASS_SAMPLES) {    /* held for 60 ms: accept (stable) */
			c->cls = now;
			c->count = 0;
		}
	} else {
		c->cand = now;
		c->count = 1;
	}
	return c->cls;
}
