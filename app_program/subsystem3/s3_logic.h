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
    S3_DEC_ERR_FORMAT           /* not * X * with X in A-D, or gaps not narrow */
} s3_dec_t;

/* w[] = 29 element widths, first one is a BAR, then a space, a bar, ...
 * On S3_DEC_OK: *out_char = 'A'..'D', *out_cmd = matching command.
 * Also tries the widths reversed (robot read the code backwards).            */
s3_dec_t s3_decode(const uint32_t *w, int n, char *out_char, s3_cmd_t *out_cmd);

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
void s3_line_step(s3_line_t *l, const uint16_t raw[3], bool suppress_junction, s3_line_out_t *out);

/* ============================ Barcode state machine ======================== */
typedef enum { S3_BC_WAITING = 0, S3_BC_READING, S3_BC_COOLDOWN } s3_bc_state_t;

typedef enum { S3_BC_EV_NONE = 0, S3_BC_EV_CMD, S3_BC_EV_ERROR } s3_bc_evtype_t;
typedef struct {
    s3_bc_evtype_t type;
    int            value;       /* CMD: s3_cmd_t.  ERROR: reason, see below */
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
    uint32_t      widths[S3_BC_ELEMENTS];
    int           n_elem;
    uint32_t      max_us;                     /* widest element so far         */
    uint32_t      cooldown_until;
    uint32_t      errors;
    int           last_cmd;                   /* -1 = nothing decoded yet      */
    uint32_t      fail_widths[S3_BC_ELEMENTS];/* widths of the last failed read */
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

#endif /* S3_LOGIC_H */
