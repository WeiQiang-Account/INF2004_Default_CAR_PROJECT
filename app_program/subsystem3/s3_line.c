/*
 *	s3_line.c
 *	Subsystem 3 -- line logic. PURE (no hardware, no kernel calls).
 *	Takes the three raw ADC readings of one 5 ms step and works out: where the
 *	line is, the steering correction, junction, and line lost / found.
 */
#include "s3_logic.h"

void s3_line_init(s3_line_t *l)
{
    for (int i = 0; i < 3; i++) { l->raw_min[i] = 4095; l->raw_max[i] = 0; }
    l->calibrated     = false;
    l->last_steer     = 0;
    l->last_pos       = 0;
    l->last_side      = 0;
    l->lost_count     = 0;
    l->lost           = false;
    l->black_count    = 0;
    l->junction_armed = true;
    l->straight_count = 0;
}

void s3_line_cal_begin(s3_line_t *l)
{
    s3_line_init(l);               /* min starts high and max starts low, so the first reading replaces them */
}

void s3_line_cal_feed(s3_line_t *l, const uint16_t raw[3])
{
    for (int i = 0; i < 3; i++) {
        if (raw[i] < l->raw_min[i]) l->raw_min[i] = raw[i];
        if (raw[i] > l->raw_max[i]) l->raw_max[i] = raw[i];
    }
}

bool s3_line_cal_end(s3_line_t *l)
{
    for (int i = 0; i < 3; i++) {
        if ((int)l->raw_max[i] - (int)l->raw_min[i] < S3_MIN_CAL_RANGE) {
            l->calibrated = false;
            return false;
        }
    }
    l->calibrated = true;
    return true;
}

/* scale a raw reading to 0..1000 where 1000 always means BLACK */
static int16_t normalise(const s3_line_t *l, int i, uint16_t raw)
{
    int32_t span = (int32_t)l->raw_max[i] - (int32_t)l->raw_min[i];
    int32_t v;

    if (span <= 0) return 0;                       /* avoid divide by zero */
    v = ((int32_t)raw - (int32_t)l->raw_min[i]) * 1000 / span;
    if (v < 0)    v = 0;
    if (v > 1000) v = 1000;
#if !S3_BLACK_IS_HIGH
    v = 1000 - v;                                  /* sensor reads LOW over black */
#endif
    return (int16_t)v;
}

int16_t s3_line_norm(const s3_line_t *l, int i, uint16_t raw) { return normalise(l, i, raw); }

void s3_line_step(s3_line_t *l, const uint16_t raw[3], bool suppress_junction, s3_line_out_t *out)
{
    int32_t sum, s;

    out->events = 0;
    for (int i = 0; i < 3; i++) out->norm[i] = normalise(l, i, raw[i]);
    /* 2026-11-11: the barcodes lie BESIDE the line, under the barcode sensor. While one is
       being read ('suppress_junction' = barcode busy) and the centre is on the line, the
       bars are not the line: ignore that sensor so the robot does not swerve onto them. */
    /* 2026-11-18: the junction test below uses the REAL readings (jn). Zeroing the barcode
       sensor also hid every cross-junction while the reader was busy (it starts on the
       cross line itself, or on the track line brushing the left sensor): the robot read the
       barcode but never saw the junction, so it never did the move. A barcode alone cannot
       make all three black (its bars are beside the line, the right sensor stays white). */
    int16_t jn[3] = { out->norm[0], out->norm[1], out->norm[2] };
    /* 2026-11-18 (photo of the robot): the barcode sensor sits ~6 cm left of and ~3 cm behind
       the centre/right pair. With the line under BOTH of the pair, black under the left
       sensor is a barcode bar or the cross line of a junction, never "the line is to the
       left": do not let it yank the steering left in the middle of a crossing. */
    if ((suppress_junction || (out->norm[2] > S3_BLACK_THRESHOLD)) && (out->norm[1] > S3_BLACK_THRESHOLD))
        out->norm[S3_BC_SENSOR_IDX] = 0;
    sum = (int32_t)out->norm[0] + out->norm[1] + out->norm[2];
    {
        int16_t mx = out->norm[0];
        if (out->norm[1] > mx) mx = out->norm[1];
        if (out->norm[2] > mx) mx = out->norm[2];
        /* 2026-11-06: small readings on white floor (noise) gave positions like
           +79 and spun the robot the wrong way. A line must make at least ONE
           sensor clearly dark, not just add up to a small sum.               */
        if (mx < S3_LINE_PRESENT_MIN) sum = 0;
    }

    /* 1. is there a line under us at all? */
    if (sum < S3_LINE_LOST_SUM) {
        if (l->lost_count < 255) l->lost_count++;
        if ((l->lost_count >= S3_LINE_LOST_SAMPLES) && !l->lost) {
            l->lost = true;
            out->events |= S3_LINE_EVT_LOST;       /* reported ONCE */
            /* steer firmly back to the side a side sensor last saw the line on
               (the line can vanish in the gap between two sensors, so the last
               average may still say "centred") */
            {
                /* centre was last: the line slipped into the WIDE gap (a side
                   sensor on the narrow side would have caught it on the way) */
                int32_t side = (l->last_side != 0) ? l->last_side : S3_WIDE_GAP_SIDE;
                s = side * ((S3_STEER_KP_X10 >= 0) ? 100 : -100);   /* full steer, sign of the gain */
                if (s >  100) s =  100;
                if (s < -100) s = -100;
                l->last_steer = (int8_t)s;
                l->last_pos   = (int16_t)(side * 100);
            }
        }
        /* otherwise keep last_steer: there is no new information */
    } else {
        l->lost_count = 0;
        if (l->lost) {
            l->lost = false;
            out->events |= S3_LINE_EVT_FOUND;
        }
        /* 2. weighted average: left = -100, centre = 0, right = +100 */
        l->last_pos = (int16_t)((100 * ((int32_t)out->norm[2] - out->norm[0])) / sum);
        if ((out->norm[2] >= S3_SIDE_SEEN) && (out->norm[2] >= out->norm[0])) l->last_side = 1;
        else if (out->norm[0] >= S3_SIDE_SEEN)                               l->last_side = -1;
        else if (out->norm[1] >= S3_SIDE_SEEN)                               l->last_side = 0;   /* centre only */
        /* 3. correction = position x gain, limited to -100..+100 */
        s = (int32_t)l->last_pos * S3_STEER_KP_X10 / 10;
        if (s >  100) s =  100;
        if (s < -100) s = -100;
        l->last_steer = (int8_t)s;
    }
    /* 4. junction = all three black for a few samples (ONE event per junction) */
    if ((jn[0] > S3_BLACK_THRESHOLD) && (jn[1] > S3_BLACK_THRESHOLD) &&
        (jn[2] > S3_BLACK_THRESHOLD)) {
        if (l->black_count < 255) l->black_count++;
        if ((l->black_count >= S3_JUNCTION_SAMPLES) && l->junction_armed) {
            l->junction_armed = false;
            /* only when it came in straight: on a bend the line crosses all three diagonally */
            if (l->straight_count >= S3_JUNC_STRAIGHT_SAMPLES) out->events |= S3_LINE_EVT_JUNCTION;
        }
    } else {
        l->black_count = 0;
        l->junction_armed = true;
        if (!l->lost && (l->last_pos <= S3_JUNC_STRAIGHT_POS) && (l->last_pos >= -S3_JUNC_STRAIGHT_POS)) {
            if (l->straight_count < 255) l->straight_count++;
        } else {
            l->straight_count = 0;
        }
    }

    out->pos          = l->last_pos;           /* held while the line is lost */
    out->steer        = l->last_steer;
    out->lost         = l->lost;
    out->centre_black = (out->norm[1] > S3_BLACK_THRESHOLD);
    out->any_black    = (out->norm[0] > S3_BLACK_THRESHOLD) || out->centre_black ||
                        (out->norm[2] > S3_BLACK_THRESHOLD);
}

/* ------------------------------------------------------------------------------------
 * Junction from the LEFT (barcode) sensor, 2026-11-18.
 * On our robot the left sensor is ~6 cm left of and ~3 cm behind the centre/right pair,
 * so "all three black at once" never happens at a cross (the tape is only ~2 cm wide):
 * the pair has left the cross line before the left sensor reaches it. Instead:
 *   a cross line = ONE dark mark under the left sensor, 8..45 mm long (by the wheels),
 *   with white before it (>= 25 mm) and after it (>= 35 mm), while the pair is on the
 *   line and the robot was going straight. A barcode has many bars with narrow gaps,
 *   so it never passes the white-before/after test.
 * The event says how far the robot has already gone past the middle of the cross line,
 * so the junction move can still put the wheel axle on the crossing.
 * ------------------------------------------------------------------------------------ */
void s3_jx_init(s3_jx_t *j)
{
    j->st = 0; j->have_white_end = false; j->white_end = 0; j->m0 = 0; j->m1 = 0; j->ok = false;
}

bool s3_jx_step(s3_jx_t *j, int16_t left, bool pair_on_line, bool straight, int32_t enc, int16_t *mm_past)
{
    const int32_t C = S3_JX_COUNTS_PER_10MM;                       /* counts per 10 mm */
    int32_t d;
    switch (j->st) {
    case 0:                                                        /* white under the left sensor */
        if (left > S3_JX_BLACK) {
            /* white before it: long enough since the last dark mark (or none seen yet) */
            d = enc - j->white_end; if (d < 0) d = -d;
            j->ok = pair_on_line && straight && (!j->have_white_end || (d * 10 >= S3_JX_QUIET_BEFORE_MM * C));
            j->m0 = enc; j->st = 1;
        }
        break;
    case 1:                                                        /* over a dark mark */
        if (!pair_on_line) j->ok = false;
        d = enc - j->m0; if (d < 0) d = -d;
        if (d * 10 > S3_JX_MAX_MM * C) j->ok = false;              /* too long: the line itself, not a cross line */
        if (left < S3_JX_WHITE) {
            j->m1 = enc; j->white_end = enc; j->have_white_end = true;
            if (j->ok && (d * 10 >= S3_JX_MIN_MM * C)) j->st = 2; else j->st = 0;
        }
        break;
    default:                                                       /* after a good mark: wait for white */
        if (left > S3_JX_BLACK) {                                  /* another bar: a barcode, not a junction */
            j->ok = false; j->m0 = enc; j->st = 1;                 /* follow this mark (it cannot count either) */
            break;
        }
        j->white_end = enc;
        d = enc - j->m1; if (d < 0) d = -d;
        if (d * 10 >= S3_JX_QUIET_AFTER_MM * C) {
            int32_t mid = j->m0 + (j->m1 - j->m0) / 2;
            int32_t past = enc - mid; if (past < 0) past = -past;
            *mm_past = (int16_t)((past * 10) / C);
            j->st = 0;
            return true;
        }
        break;
    }
    return false;
}
