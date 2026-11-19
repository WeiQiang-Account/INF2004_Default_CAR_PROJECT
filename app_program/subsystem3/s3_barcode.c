/*
 *	s3_barcode.c
 *	Subsystem 3 -- barcode state machine (statechart 6.3). PURE logic:
 *	it is fed edges (time, encoder, level) and the current time, and returns
 *	events. No hardware and no kernel calls, so it is tested on a PC.
 *
 *	WAITING -> (first black edge) -> READING -> (quiet zone) -> decode
 *	        -> COOLDOWN (300 ms, ignore edges) -> WAITING
 *	any failure -> error event, back to WAITING (the robot keeps following the line)
 */
#include "s3_logic.h"

static void ev_clear(s3_bc_ev_t *ev) { ev->type = S3_BC_EV_NONE; ev->value = 0; }

void s3_bc_init(s3_bc_t *b)
{
    b->state          = S3_BC_WAITING;
    b->have_pend      = false;
    b->n_elem         = 0;
    b->max_us         = 0;
    b->max_t_us       = 0;
    b->use_enc        = false;
    b->fail_enc       = false;
    b->cooldown_until = 0;
    b->errors         = 0;
    b->last_cmd       = -1;
    b->fail_n         = 0;
    b->now_enc        = 0;
    b->have_now_enc   = false;
    b->last_partial   = false;
    b->partial_count  = 0;
    b->fit_count      = 0;
    b->fail_reason    = 0;
    b->prev.t_us = 0; b->prev.enc = 0; b->prev.level = 0;
    b->pend = b->prev;
}

bool s3_bc_busy(const s3_bc_t *b) { return b->state != S3_BC_WAITING; }

void s3_bc_abort(s3_bc_t *b)
{
    b->have_pend = false;
    if (b->state == S3_BC_READING) { b->state = S3_BC_WAITING; b->n_elem = 0; }
}

/* a read failed: remember what we measured (for debugging), report, go back to waiting */
static void fail(s3_bc_t *b, int reason, s3_bc_ev_t *ev)
{
    b->state = S3_BC_WAITING;
    if (b->n_elem < S3_BC_MIN_ERR_ELEMENTS) return;   /* too short to be a barcode attempt: noise, a stain or a crossing line */
    b->errors++;
    b->fail_reason = reason;
    b->fail_n = b->n_elem;
    b->fail_enc = b->use_enc;
    for (int i = 0; i < b->n_elem && i < S3_BC_MAX_ELEM; i++) b->fail_widths[i] = b->use_enc ? b->widths[i] : b->widths_t[i];
    ev->type  = S3_BC_EV_ERROR;
    ev->value = reason;
}

/* Try to decode. At the 29th element (final = false) only the first 29 are tried and a
 * failure keeps reading. At the end of the read (quiet zone, timeout, too many: final = true)
 * every run of 29 that starts on a bar is tried, so a stray mark before or after the
 * barcode does not spoil it (2026-11-11). */
static void try_decode(s3_bc_t *b, uint32_t t_us, s3_bc_ev_t *ev, bool final)
{
    char     ch = 0;
    s3_cmd_t cmd = S3_CMD_STRAIGHT;
    const uint32_t *w = b->widths_t;
    s3_dec_t r = S3_DEC_ERR_COUNT, r0 = S3_DEC_ERR_COUNT;
    bool partial = false;
    if (b->use_enc) {                               /* distance widths, if the wheels really moved */
        uint32_t span = 0;
        for (int i = 0; i < b->n_elem; i++) span += b->widths[i];
        if (span >= S3_BC_MIN_ENC_SPAN) w = b->widths;
    }
    if (!final) {
        r = s3_decode(w, S3_BC_ELEMENTS, &ch, &cmd);
        if (r != S3_DEC_OK) return;                   /* maybe a stray mark: keep reading */
    } else {
        for (int s = 0; (s + S3_BC_ELEMENTS) <= b->n_elem; s += 2) {    /* windows start on a bar */
            r = s3_decode(&w[s], S3_BC_ELEMENTS, &ch, &cmd);
            if (s == 0) r0 = r;
            if (r == S3_DEC_OK) break;
        }
        if (r != S3_DEC_OK) r = (b->n_elem < S3_BC_ELEMENTS) ? S3_DEC_ERR_COUNT : r0;
#if S3_BC_FIT
        if (r != S3_DEC_OK) {                         /* blurred widths: best fit, all 29 or '*' + letter */
            for (int s = 0; (s + S3_BC_PARTIAL_ELEMENTS) <= b->n_elem; s += 2) {
                if (s3_decode_fit(&w[s], b->n_elem - s, &ch) == S3_DEC_OK) {
                    r = S3_DEC_OK; partial = ((b->n_elem - s) < S3_BC_ELEMENTS); b->fit_count++; break;
                }
            }
        }
#endif
#if S3_BC_PARTIAL && !S3_BC_FIT                   /* (the best fit already covers '*' + letter, with more checks) */
        if (r != S3_DEC_OK) {                         /* lost the last bars? '*' + letter is enough */
            for (int s = 0; (s + S3_BC_PARTIAL_ELEMENTS) <= b->n_elem; s += 2) {
                if (s3_decode_partial(&w[s], b->n_elem - s, &ch) == S3_DEC_OK) { r = S3_DEC_OK; partial = true; break; }
            }
        }
#endif
    }

    if (r == S3_DEC_OK) {
        (void)cmd;                                    /* the character is what counts now */
        b->last_cmd       = (int)(unsigned char)ch;
        b->last_partial   = partial;
        if (partial) b->partial_count++;
        ev->type          = S3_BC_EV_CMD;
        ev->value         = (int)(unsigned char)ch;
        b->state          = S3_BC_COOLDOWN;           /* never read the same barcode twice */
        b->cooldown_until = t_us + S3_COOLDOWN_US;
    } else {
        fail(b, (int)r, ev);
    }
}

/* a committed (glitch-free) edge */
static void handle_edge(s3_bc_t *b, const s3_edge_t *e, s3_bc_ev_t *ev)
{
    /* COOLDOWN: ignore edges until the time is up ((int32_t) keeps working when the 32-bit us timer wraps) */
    if (b->state == S3_BC_COOLDOWN) {
        if ((int32_t)(e->t_us - b->cooldown_until) < 0) return;
        b->state = S3_BC_WAITING;
    }

    if (b->state == S3_BC_WAITING) {
        if (e->level == S3_BAR_LEVEL_BLACK) {       /* first black edge = start of the code */
            b->n_elem = 0;
            b->max_us = 0;
            b->max_t_us = 0;
            b->dir    = 0;
            b->prev   = *e;
            b->state  = S3_BC_READING;
        }
        return;
    }

    /* READING: this edge ends the element that started at 'prev' */
    if (e->level == b->prev.level) { fail(b, S3_BC_ERR_MISSED_EDGE, ev); return; }

    /* 2026-11-18: the robot changed direction in the middle of a read (barcode test scan
       stopped short, then drove back). The bars before and after the turn do not belong to
       one pass: finish this read (decode if it has enough, quietly drop it if not), then
       start a fresh read for the way back. */
    if (b->use_enc) {
        int32_t dd = e->enc - b->prev.enc;
        int8_t  sd = (dd > 0) ? 1 : ((dd < 0) ? -1 : 0);
        if ((sd != 0) && (b->dir != 0) && (sd != b->dir)) {
            if (b->n_elem >= S3_BC_ELEMENTS) { try_decode(b, e->t_us, ev, true); if (ev->type != S3_BC_EV_NONE) return; }
            b->state = S3_BC_WAITING;                 /* too short to be a barcode: no error */
            if (e->level == S3_BAR_LEVEL_BLACK) {     /* this edge may start the next read */
                b->n_elem = 0; b->max_us = 0; b->max_t_us = 0; b->dir = 0; b->prev = *e; b->state = S3_BC_READING;
            }
            return;
        }
        if ((b->dir == 0) && (sd != 0)) b->dir = sd;
    }
    if (b->n_elem >= S3_BC_MAX_ELEM) {               /* far too many: last try, then give up */
        try_decode(b, e->t_us, ev, true);              /* success -> command, else it reports the error */
        return;
    }

    {
        uint32_t t = e->t_us - b->prev.t_us;
        int32_t  d = e->enc - b->prev.enc;
        uint32_t w = b->use_enc ? (uint32_t)((d < 0) ? -d : d) : t;   /* distance (counts) or time (us) */
        b->widths_t[b->n_elem] = t;
        b->widths[b->n_elem++] = w;
        if (w > b->max_us) b->max_us = w;
        if (t > b->max_t_us) b->max_t_us = t;
    }
    b->prev = *e;

#if S3_BC_DECODE_ON_29
    if (b->n_elem == S3_BC_ELEMENTS) try_decode(b, e->t_us, ev, false);   /* the stop bar just ended */
#endif
}

void s3_bc_edge(s3_bc_t *b, uint32_t t_us, int32_t enc, uint8_t level, s3_bc_ev_t *ev)
{
    s3_edge_t e;

    ev_clear(ev);
    e.t_us = t_us; e.enc = enc; e.level = level;

    if (b->have_pend) {
        if ((uint32_t)(e.t_us - b->pend.t_us) < S3_GLITCH_US) {
            b->have_pend = false;                 /* a pulse shorter than the glitch limit: drop BOTH edges */
            return;
        }
        handle_edge(b, &b->pend, ev);             /* the pending edge survived: commit it */
    }
    b->pend = e;
    b->have_pend = true;
}

void s3_bc_poll(s3_bc_t *b, uint32_t now_us, s3_bc_ev_t *ev)
{
    ev_clear(ev);

    /* 1. a pending edge that has waited out the glitch window is real */
    if (b->have_pend && ((uint32_t)(now_us - b->pend.t_us) >= S3_GLITCH_US)) {
        b->have_pend = false;
        handle_edge(b, &b->pend, ev);
        if (ev->type != S3_BC_EV_NONE) return;
    }

    /* 2. timeouts (no edge arrives in a quiet zone, so we need the clock) */
    if (b->state == S3_BC_READING) {
        uint32_t gap = now_us - b->prev.t_us;     /* time since the last committed edge */

        /* 2026-11-18: distance quiet zone. With wheel distance we know when the sensor has
           left the barcode: white for more than 3x the widest element (and >= ~20 mm).
           Decode right there (all 29, or '*' + letter if the last bars were lost) instead of
           waiting seconds for the time-out while the robot drives on. */
        if (b->use_enc && b->have_now_enc && (b->n_elem > 0)) {
            int32_t  dg  = b->now_enc - b->prev.enc;
            uint32_t ge  = (uint32_t)((dg < 0) ? -dg : dg);
            uint32_t qz  = 3u * b->max_us;
            if (qz < (uint32_t)S3_BC_QUIET_ENC_MIN) qz = S3_BC_QUIET_ENC_MIN;
            if ((b->n_elem >= S3_BC_PARTIAL_ELEMENTS) && (ge > qz)) { try_decode(b, now_us, ev, true); return; }
            if ((b->n_elem <  S3_BC_PARTIAL_ELEMENTS) && (ge > (uint32_t)S3_BC_LOST_ENC) && (ge > 4u * b->max_us)) {
                fail(b, S3_BC_ERR_TIMEOUT, ev); return;
            }
        }

        if (b->n_elem >= S3_BC_ELEMENTS) {
            /* got all 29: wait for the quiet zone (a white gap > 3x the widest element) */
            if (gap > 3u * b->max_t_us) {
                try_decode(b, now_us, ev, true);
            } else if (gap > 10u * (uint32_t)S3_ABORT_US) {
                fail(b, S3_BC_ERR_TIMEOUT, ev);
            }
        } else {
            /* still collecting: give up only after a gap that is long compared with the
             * widest element so far (or S3_ABORT_US, whichever is bigger) */
            uint32_t limit = 4u * b->max_t_us;
            uint32_t floor_us = b->use_enc ? (uint32_t)S3_ABORT_ENC_US : (uint32_t)S3_ABORT_US;
            if (limit < floor_us) limit = floor_us;
            if (gap > limit) {
                if (b->n_elem >= S3_BC_PARTIAL_ELEMENTS) try_decode(b, now_us, ev, true);   /* maybe '*' + letter */
                else fail(b, S3_BC_ERR_TIMEOUT, ev);   /* a short mark is dropped silently inside fail() */
            }
        }
    }
}

/* ---------------- analog barcode sensor -> edges (see s3_logic.h) ---------------- */
void s3_bcadc_init(s3_bcadc_t *a)
{
    a->n = 0; a->lvl = 0; a->ext = 1000; a->ext_i = 0; a->pv = 0; a->pv_i = 0;
    a->low_out = true; a->run_hi = 0; a->run_lo = 1000; a->have_hi = false;
    a->idle = 1000000; a->swing = 0;
    a->r1 = -1; a->r2 = -1; a->noise16 = 16 * 8;
}

/* the edge between extreme i0 (value v0) and extreme i1 (value v1): first sample where the
   signal crosses the middle, interpolated between that sample and the one before */
static void place(const s3_bcadc_t *a, uint32_t i0, int16_t v0, uint32_t i1, int16_t v1, uint8_t lvl, s3_edge_t *e)
{
    int32_t mid = ((int32_t)v0 + v1) / 2;
    bool rising = (v1 > v0);
    if ((a->n - i0) > S3_BCADC_HIST) i0 = a->n - S3_BCADC_HIST;    /* older than the history: start at the oldest */
    uint32_t k = i0 + 1u;
    for (; k <= i1; k++) {
        int16_t v = a->h[k & (S3_BCADC_HIST - 1u)].v;
        if (rising ? (v >= mid) : (v <= mid)) break;
    }
    if (k > i1) k = i1;
    {
        const s3_bcadc_smp_t *p = &a->h[(k - 1u) & (S3_BCADC_HIST - 1u)];
        const s3_bcadc_smp_t *q = &a->h[k & (S3_BCADC_HIST - 1u)];
        int32_t dv = (int32_t)q->v - p->v, f = 0;                 /* fraction 0..256 from p to q */
        if (dv != 0) { f = ((mid - p->v) * 256) / dv; if (f < 0) f = 0; if (f > 256) f = 256; }
        e->t_us  = p->t_us + (uint32_t)(((uint64_t)(q->t_us - p->t_us) * (uint32_t)f) >> 8);
        e->enc   = p->enc + (int32_t)(((int64_t)(q->enc - p->enc) * f) / 256);
        e->level = lvl;
    }
}

bool s3_bcadc_step(s3_bcadc_t *a, const s3_bcadc_cfg_t *c, int16_t v, uint32_t t_us, int32_t enc, s3_edge_t *e)
{
    uint32_t i = a->n;
    bool in_run = (a->idle < (int32_t)c->active);
    int16_t sw = c->swing_max, raw = v;

    /* 3-sample average: noise down, edges hardly moved (3 x 5 ms) */
    if (a->r1 < 0) { a->r1 = raw; a->r2 = raw; }
    v = (int16_t)(((int32_t)raw + a->r1 + a->r2) / 3);
    {   /* noise estimate: |raw step|, follows quiet stretches (falls fast, rises slowly), edges hardly count */
        int32_t d = (int32_t)raw - a->r1; if (d < 0) d = -d;
        int32_t d16 = d * 16;
        if (d16 > a->noise16) a->noise16 += (d16 - a->noise16) / 64; else a->noise16 -= (a->noise16 - d16) / 16;
    }
    a->r2 = a->r1; a->r1 = raw;

    a->h[i & (S3_BCADC_HIST - 1u)].v = v;
    a->h[i & (S3_BCADC_HIST - 1u)].t_us = t_us;
    a->h[i & (S3_BCADC_HIST - 1u)].enc = enc;
    a->n = i + 1u;

    if (!in_run) { a->have_hi = false; a->run_hi = 0; a->run_lo = 1000; }
    if (a->idle < 0x3FFFFFFF) a->idle++;
    if (a->have_hi && (a->run_hi > a->run_lo)) {
        int32_t k = ((int32_t)(a->run_hi - a->run_lo) * c->pct) / 100;
        if (k < c->swing_min) k = c->swing_min;
        if (k > c->swing_max) k = c->swing_max;
        sw = (int16_t)k;
    }
    {   /* never closer to the noise than noise_k/4 x noise (the 3-average leaves ~0.6 of it) */
        int32_t nf = (a->noise16 * c->noise_k) / 64;
        if (nf > c->swing_max * 2) nf = c->swing_max * 2;
        if (sw < nf) sw = (int16_t)nf;
    }
    a->swing = sw;

    if (a->lvl == 0u) {                                               /* over white: follow the low */
        if (v < a->ext) { a->ext = v; a->ext_i = i; }
        if ((v > a->ext + sw) && (in_run || (v >= c->start_black))) {
            bool out = false;
            /* the low is confirmed: place the falling edge peak -> low, unless done already */
            if (!a->low_out && a->have_hi) { place(a, a->pv_i, a->pv, a->ext_i, a->ext, (uint8_t)!S3_BAR_LEVEL_BLACK, e); out = true; }
            if (a->ext < a->run_lo) a->run_lo = a->ext;
            a->pv = a->ext; a->pv_i = a->ext_i;                       /* this low starts the next bar */
            a->lvl = 1u; a->ext = v; a->ext_i = i; a->idle = 0;
            a->low_out = false;
            return out;
        }
        /* quiet zone: the low has not moved for 'settle' samples -> place the last falling edge now */
        if (!a->low_out && a->have_hi && ((i - a->ext_i) >= (uint32_t)c->settle)) {
            place(a, a->pv_i, a->pv, a->ext_i, a->ext, (uint8_t)!S3_BAR_LEVEL_BLACK, e);
            a->low_out = true;
            return true;
        }
    } else {                                                          /* over a bar: follow the peak */
        if (v > a->ext) { a->ext = v; a->ext_i = i; }
        if (a->ext > a->run_hi) a->run_hi = a->ext;
        if (v < a->ext - sw) {
            /* the peak is confirmed: place the rising edge low -> peak */
            place(a, a->pv_i, a->pv, a->ext_i, a->ext, (uint8_t)S3_BAR_LEVEL_BLACK, e);
            a->have_hi = true;
            a->pv = a->ext; a->pv_i = a->ext_i;                       /* this peak starts the next gap */
            a->lvl = 0u; a->ext = v; a->ext_i = i; a->idle = 0;
            a->low_out = false;
            return true;
        }
    }
    return false;
}
