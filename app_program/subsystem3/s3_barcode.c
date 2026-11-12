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
    b->fail_reason    = 0;
    b->prev.t_us = 0; b->prev.enc = 0; b->prev.level = 0;
    b->pend = b->prev;
}

bool s3_bc_busy(const s3_bc_t *b) { return b->state != S3_BC_WAITING; }

/* a read failed: remember what we measured (for debugging), report, go back to waiting */
static void fail(s3_bc_t *b, int reason, s3_bc_ev_t *ev)
{
    b->state = S3_BC_WAITING;
    if (b->n_elem < S3_BC_MIN_ERR_ELEMENTS) return;   /* too short to be a barcode attempt: noise, a stain or a crossing line */
    b->errors++;
    b->fail_reason = reason;
    b->fail_n = b->n_elem;
    b->fail_enc = b->use_enc;
    for (int i = 0; i < b->n_elem && i < S3_BC_ELEMENTS; i++) b->fail_widths[i] = b->use_enc ? b->widths[i] : b->widths_t[i];
    ev->type  = S3_BC_EV_ERROR;
    ev->value = reason;
}

/* all 29 elements collected: decode them */
static void try_decode(s3_bc_t *b, uint32_t t_us, s3_bc_ev_t *ev)
{
    char     ch = 0;
    s3_cmd_t cmd = S3_CMD_STRAIGHT;
    const uint32_t *w = b->widths_t;
    if (b->use_enc) {                               /* distance widths, if the wheels really moved */
        uint32_t span = 0;
        for (int i = 0; i < b->n_elem; i++) span += b->widths[i];
        if (span >= S3_BC_MIN_ENC_SPAN) w = b->widths;
    }
    s3_dec_t r = s3_decode(w, b->n_elem, &ch, &cmd);

    if (r == S3_DEC_OK) {
        (void)cmd;                                    /* the character is what counts now */
        b->last_cmd       = (int)(unsigned char)ch;
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
            b->prev   = *e;
            b->state  = S3_BC_READING;
        }
        return;
    }

    /* READING: this edge ends the element that started at 'prev' */
    if (e->level == b->prev.level) { fail(b, S3_BC_ERR_MISSED_EDGE, ev); return; }
    if (b->n_elem >= S3_BC_ELEMENTS) { fail(b, S3_BC_ERR_TOO_MANY, ev); return; }

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
    if (b->n_elem == S3_BC_ELEMENTS) try_decode(b, e->t_us, ev);   /* the stop bar just ended */
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

        if (b->n_elem >= S3_BC_ELEMENTS) {
            /* got all 29: wait for the quiet zone (a white gap > 3x the widest element) */
            if (gap > 3u * b->max_t_us) {
                try_decode(b, now_us, ev);
            } else if (gap > 10u * (uint32_t)S3_ABORT_US) {
                fail(b, S3_BC_ERR_TIMEOUT, ev);
            }
        } else {
            /* still collecting: give up only after a gap that is long compared with the
             * widest element so far (or S3_ABORT_US, whichever is bigger) */
            uint32_t limit = 4u * b->max_t_us;
            uint32_t floor_us = b->use_enc ? (uint32_t)S3_ABORT_ENC_US : (uint32_t)S3_ABORT_US;
            if (limit < floor_us) limit = floor_us;
            if (gap > limit) fail(b, S3_BC_ERR_TIMEOUT, ev);   /* a short mark is dropped silently inside fail() */
        }
    }
}
