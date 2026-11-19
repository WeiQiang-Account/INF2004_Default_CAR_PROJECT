/* s3_logic.h - Buddy 3's PURE logic: no hardware, no kernel calls, only
 * <stdint.h>. The same files are compiled on a PC for the host tests
 * (testing/subsystem3/host_tests/), so every rule below can be checked
 * without the robot.
 *   s3_decode.c   Code 39 decoder
 *   s3_line.c     line position, steering, junction, line lost
 *   s3_barcode.c  barcode state machine (edges -> command)
 * Each state machine is an INSTANCE (a struct), so the live task and the
 * tests never share state.                                                    */
#ifndef S3_LOGIC_H
#define S3_LOGIC_H

#include <stdint.h>
#include <stdbool.h>
#include "s3_config.h"

/* ============================ Code 39 decoder ============================== */
typedef enum {                  /* barcode command, same numbers as VC_BARCODE_CMD() */
    S3_CMD_LEFT     = 0,        /* A */
    S3_CMD_RIGHT    = 1,        /* B */
    S3_CMD_STRAIGHT = 2,        /* C */
    S3_CMD_UTURN    = 3         /* D */
} s3_cmd_t;

typedef enum {
    S3_DEC_OK = 0,
    S3_DEC_ERR_COUNT,           /* not exactly 29 elements                   */
    S3_DEC_ERR_RATIO,           /* wide is not clearly wider than narrow     */
    S3_DEC_ERR_NO_MATCH,        /* 9-element pattern not in the table        */
    S3_DEC_ERR_FORMAT           /* not * X *, or gaps not narrow             */
} s3_dec_t;

/* w[] = 29 element widths, first one is a BAR, then a space, a bar, ...
 * On S3_DEC_OK: *out_char = the character (A-Z, 0-9, - . space $ / + %),
 * *out_cmd = legacy A-D meaning (others STRAIGHT; the real mapping comes over MQTT).
 * Also tries the widths reversed (robot read the code backwards).            */
s3_dec_t s3_decode(const uint32_t *w, int n, char *out_char, s3_cmd_t *out_cmd);
/* Start '*' + gap + letter + gap only (first 20 of n >= 20 elements), either direction.
 * For reads that lost the last bars. Same S3_DEC_* results. */
s3_dec_t s3_decode_partial(const uint32_t *w, int n, char *out_char);
/* Best fit (blurred sensor): fits narrow/wide/blur-bias on the '*' and gaps, then picks the
 * letter that fits clearly best. n >= 20 (first 20 or all 29 used), either direction. */
s3_dec_t s3_decode_fit(const uint32_t *w, int n, char *out_char);

/* ============================ Line logic =================================== */
#define S3_LINE_EVT_JUNCTION  (1u << 0)
#define S3_LINE_EVT_LOST      (1u << 1)
#define S3_LINE_EVT_FOUND     (1u << 2)

typedef struct {
    uint16_t raw_min[3];        /* calibration: smallest reading per sensor  */
    uint16_t raw_max[3];        /* calibration: biggest reading per sensor   */
    bool     calibrated;
    int8_t   last_steer;        /* held while the line is lost               */
    int16_t  last_pos;          /* last known line position                  */
    int8_t   last_side;         /* -1 left, 0 centre, +1 right: sensor that last clearly saw the line */
    uint8_t  lost_count;
    bool     lost;
    uint8_t  black_count;
    bool     junction_armed;    /* one junction = ONE event                  */
    uint8_t  straight_count;    /* samples in a row going straight (junction needs it) */
} s3_line_t;

typedef struct {
    int16_t norm[3];            /* 0..1000, 1000 = black. 0=Left 1=Centre 2=Right */
    int16_t pos;                /* -100 (line far left) .. +100 (far right)  */
    int8_t  steer;              /* -100..+100, + = steer right (motion_types.h) */
    uint8_t events;             /* S3_LINE_EVT_* bits raised by THIS sample  */
    bool    any_black;          /* any sensor over the line                  */
    bool    centre_black;       /* centre sensor over the line               */
    bool    lost;               /* line lost (>= S3_LINE_LOST_SAMPLES in a row) */
} s3_line_out_t;

void s3_line_init(s3_line_t *l);
void s3_line_cal_begin(s3_line_t *l);
void s3_line_cal_feed(s3_line_t *l, const uint16_t raw[3]);
bool s3_line_cal_end(s3_line_t *l);          /* false if any sensor range < S3_MIN_CAL_RANGE */
/* One 5 ms step. 'suppress_junction' = true while a barcode is being read
 * (a wide bar looks black under all three sensors).                           */
/* one sensor scaled to 0..1000 (1000 = black) with the line calibration */
int16_t s3_line_norm(const s3_line_t *l, int i, uint16_t raw);
void s3_line_step(s3_line_t *l, const uint16_t raw[3], bool suppress_junction, s3_line_out_t *out);

/* junction from the left (barcode) sensor: one lone dark mark 8..45 mm (see s3_line.c) */
typedef struct {
    uint8_t st;                 /* 0 white, 1 over a mark, 2 after a good mark */
    bool    have_white_end, ok;
    int32_t white_end, m0, m1;  /* encoder: end of the last mark, start/end of this mark */
} s3_jx_t;
void s3_jx_init(s3_jx_t *j);
/* one 5 ms sample: left sensor 0..1000, pair (centre/right) on the line, robot was going
   straight, encoder count (average of both wheels). true = junction; *mm_past = how far
   past the middle of the cross line the left sensor already is. */
bool s3_jx_step(s3_jx_t *j, int16_t left, bool pair_on_line, bool straight, int32_t enc, int16_t *mm_past);

/* ============================ Barcode state machine ======================== */
typedef enum { S3_BC_WAITING = 0, S3_BC_READING, S3_BC_COOLDOWN } s3_bc_state_t;

typedef enum { S3_BC_EV_NONE = 0, S3_BC_EV_CMD, S3_BC_EV_ERROR } s3_bc_evtype_t;
typedef struct {
    s3_bc_evtype_t type;
    int            value;       /* CMD: the CHARACTER read ('A'..'Z' ...).  ERROR: reason, see below */
} s3_bc_ev_t;

/* ERROR reasons: 1..4 = s3_dec_t (1 count, 2 ratio, 3 no match, 4 format)
 *   10 missed edge (same level twice)   11 more than 29 elements
 *   12 read started but never finished (timeout)                              */
enum { S3_BC_ERR_MISSED_EDGE = 10, S3_BC_ERR_TOO_MANY = 11, S3_BC_ERR_TIMEOUT = 12 };

typedef struct { uint32_t t_us; int32_t enc; uint8_t level; } s3_edge_t;

typedef struct {
    s3_bc_state_t state;
    s3_edge_t     prev;                       /* previous committed edge       */
    s3_edge_t     pend;                       /* edge waiting out the glitch window */
    bool          have_pend;
    uint32_t      widths[S3_BC_MAX_ELEM];
    int           n_elem;
    uint32_t      max_us;                     /* widest element so far (same unit as widths) */
    uint32_t      widths_t[S3_BC_MAX_ELEM];   /* element durations, us (always)        */
    uint32_t      max_t_us;                   /* longest element in TIME (timeouts)    */
    bool          use_enc;                    /* widths from wheel distance, not time  */
    bool          fail_enc;                   /* last failed read used distance widths */
    int32_t       now_enc;                    /* encoder now (set before s3_bc_poll), for the distance quiet zone */
    bool          have_now_enc;               /* now_enc is valid                      */
    bool          last_partial;               /* last good read used the start '*' + letter only */
    uint32_t      partial_count;              /* how many reads were partial            */
    uint32_t      fit_count;                  /* how many reads needed the best-fit decode */
    int8_t        dir;                        /* distance mode: +1 / -1 travel direction of this read, 0 unknown */
    uint32_t      cooldown_until;
    uint32_t      errors;
    int           last_cmd;                   /* last character read, -1 = nothing yet */
    uint32_t      fail_widths[S3_BC_MAX_ELEM];/* widths of the last failed read */
    int           fail_n;
    int           fail_reason;
} s3_bc_t;

void s3_bc_init(s3_bc_t *b);
/* Feed one edge (t_us timestamp, encoder count, pin level AFTER the edge).
 * Pulses shorter than S3_GLITCH_US are removed here (both edges dropped). */
void s3_bc_edge(s3_bc_t *b, uint32_t t_us, int32_t enc, uint8_t level, s3_bc_ev_t *ev);
/* Call regularly (every <= 5 ms) with the same clock as the edges: handles the
 * glitch window, the quiet zone (decode) and the timeouts.                    */
void s3_bc_poll(s3_bc_t *b, uint32_t now_us, s3_bc_ev_t *ev);
bool s3_bc_busy(const s3_bc_t *b);            /* reading or cooling down        */
/* drop a read in progress quietly (no error): the track line, not a barcode, is under the sensor */
#define S3_BC_LEVEL_ABORT 2u
void s3_bc_abort(s3_bc_t *b);

/* ===================== Barcode sensor: analog -> edges ====================== */
/* 2026-11-18. The sensor spot is wider than a narrow bar, so the signal is a blurred
 * wave: thin bars only rise a little, thin gaps only dip a little.
 *  1. FIND the bars and gaps: a peak/valley follower. The step that counts as a new
 *     bar/gap (the "swing") follows the contrast of THIS barcode: pct % of (darkest
 *     peak - whitest low) in the current run, kept between swing_min and swing_max.
 *     A run starts with a bar that reaches start_black; inside a run (an edge in the
 *     last 'active' samples) a bar only needs the swing.
 *  2. PLACE each edge halfway: where the signal crosses the middle between the peak
 *     and the low on each side of it (interpolated). Placing it "one swing after the
 *     peak" made thin bars look wide and the gaps next to them look narrow.
 *  An edge is known only once the NEXT peak/low is found, so edges come out a little
 *  late but carry their own time and encoder count. The last falling edge comes out
 *  when the low has been steady for 'settle' samples (the quiet zone). */
typedef struct {
    int16_t swing_max;          /* step before any contrast is known (and the most ever used) */
    int16_t swing_min;          /* never below this (sensor/floor noise)                      */
    int16_t pct;                /* swing = pct % of the run's contrast                        */
    int16_t start_black;        /* first bar of a run must reach this (0..1000)               */
    int16_t active;             /* samples after the last edge that still count as the same run */
    int16_t settle;             /* samples the low must hold before the last falling edge is placed */
    int16_t noise_k;            /* swing is at least noise_k/4 x the measured sample-to-sample noise */
} s3_bcadc_cfg_t;

#define S3_BCADC_HIST 256       /* samples kept (power of 2): 256 x 5 ms = 1.28 s */
typedef struct { int16_t v; uint32_t t_us; int32_t enc; } s3_bcadc_smp_t;

typedef struct {
    s3_bcadc_smp_t h[S3_BCADC_HIST];
    uint32_t n;                 /* samples seen                          */
    uint8_t  lvl;               /* 1 = over a bar (detector, not delayed) */
    int16_t  ext;  uint32_t ext_i;   /* peak/low followed now, and its sample number */
    int16_t  pv;   uint32_t pv_i;    /* the previous CONFIRMED extreme (a low while lvl=1, a peak while lvl=0) */
    bool     low_out;           /* the falling edge into the current low was already placed */
    int16_t  run_hi, run_lo;
    bool     have_hi;
    int32_t  idle;              /* samples since the last confirmed bar/gap */
    int16_t  swing;             /* swing in use (for the dashboard)      */
    int16_t  r1, r2;            /* the two previous raw samples (3-sample average) */
    int32_t  noise16;           /* sample-to-sample noise estimate x16   */
} s3_bcadc_t;

void s3_bcadc_init(s3_bcadc_t *a);
/* one sample v (0..1000, 1000 = black) with its time and encoder count.
 * Returns true and fills *e when an edge is placed (e->level = level AFTER the edge). */
bool s3_bcadc_step(s3_bcadc_t *a, const s3_bcadc_cfg_t *c, int16_t v, uint32_t t_us, int32_t enc, s3_edge_t *e);

#endif /* S3_LOGIC_H */
