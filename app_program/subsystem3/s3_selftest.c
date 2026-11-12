/*
 *	s3_selftest.c
 *	Self-checks of Buddy 3's pure logic. See s3_selftest.h.
 *	Compiled only for the test image (APP_TEST) or a PC build (S3_HOST_TEST).
 */
#if defined(S3_HOST_TEST) || (defined(APP_TEST) && APP_TEST)

#include "s3_logic.h"
#include "s3_selftest.h"

/* ---------------------------------------------------------------- helpers */
static const char *const PAT[5] = {      /* * A B C D, 1 = wide (independent of the decoder's own table) */
    "010010100", "100001001", "001001001", "101001000", "000011001"
};

/* every Code 39 character (standard table, typed in again here so the test does
   not trust the decoder's own copy): character, 9 elements, 1 = wide          */
static const struct { char ch; const char *p; } ALL39[] = {
    {'0',"000110100"},{'1',"100100001"},{'2',"001100001"},{'3',"101100000"},{'4',"000110001"},
    {'5',"100110000"},{'6',"001110000"},{'7',"000100101"},{'8',"100100100"},{'9',"001100100"},
    {'A',"100001001"},{'B',"001001001"},{'C',"101001000"},{'D',"000011001"},{'E',"100011000"},
    {'F',"001011000"},{'G',"000001101"},{'H',"100001100"},{'I',"001001100"},{'J',"000011100"},
    {'K',"100000011"},{'L',"001000011"},{'M',"101000010"},{'N',"000010011"},{'O',"100010010"},
    {'P',"001010010"},{'Q',"000000111"},{'R',"100000110"},{'S',"001000110"},{'T',"000010110"},
    {'U',"110000001"},{'V',"011000001"},{'W',"111000000"},{'X',"010010001"},{'Y',"110010000"},
    {'Z',"011010000"},{'-',"010000101"},{'.',"110000100"},{' ',"011000100"},{'$',"010101000"},
    {'/',"010100010"},{'+',"010001010"},{'%',"000101010"}
};
#define N_ALL39 ((int)(sizeof(ALL39) / sizeof(ALL39[0])))

static uint32_t rng_state = 12345u;
static int rnd(int lo, int hi)           /* small LCG, same on every machine */
{
    rng_state = rng_state * 1664525u + 1013904223u;
    return lo + (int)((rng_state >> 16) % (uint32_t)(hi - lo + 1));
}

/* the 29 element widths of  * X *  with X's pattern 'mid' */
static void make_widths_p(const char *mid, uint32_t narrow, int wide_x10, int jitter_pct, uint32_t *w)
{
    int idx = 0;
    for (int c = 0; c < 3; c++) {
        const char *p = (c == 1) ? mid : PAT[0];
        for (int k = 0; k < 9; k++) {
            uint32_t base = (p[k] == '1') ? (narrow * (uint32_t)wide_x10 / 10u) : narrow;
            if ((idx % 2) == 1) base = base * 9u / 10u;                  /* spaces print a little thinner than bars */
            w[idx++] = base * (uint32_t)(100 + (jitter_pct ? rnd(-jitter_pct, jitter_pct) : 0)) / 100u;
        }
        if (c < 2) w[idx++] = narrow * 9u / 10u;                          /* narrow gap between characters */
    }
}

/* x = 1..4 means A..D */
static void make_widths(int x, uint32_t narrow, int wide_x10, int jitter_pct, uint32_t *w)
{
    make_widths_p(PAT[x], narrow, wide_x10, jitter_pct, w);
}

static void reverse29(const uint32_t *a, uint32_t *b)
{
    for (int i = 0; i < S3_BC_ELEMENTS; i++) b[i] = a[S3_BC_ELEMENTS - 1 - i];
}

/* ---------------------------------------------------------------- decoder */
int s3_st_decode_fwd(void)
{
    uint32_t w[S3_BC_ELEMENTS];
    char ch; s3_cmd_t cmd;
    int ok = 1;

    for (int x = 1; x <= 4; x++) {
        for (int t = 0; t < 50; t++) {
            make_widths(x, 100, 30, 15, w);
            if (s3_decode(w, S3_BC_ELEMENTS, &ch, &cmd) != S3_DEC_OK) ok = 0;
            else if ((ch != 'A' + x - 1) || ((int)cmd != x - 1))      ok = 0;
        }
    }
    for (int i = 0; i < N_ALL39; i++) {                  /* 2026-11-11: every character, A-Z and the rest */
        for (int t = 0; t < 10; t++) {
            make_widths_p(ALL39[i].p, 100, 30, 15, w);
            if ((s3_decode(w, S3_BC_ELEMENTS, &ch, &cmd) != S3_DEC_OK) || (ch != ALL39[i].ch)) ok = 0;
        }
    }
    return ok;
}

int s3_st_decode_rev(void)
{
    uint32_t w[S3_BC_ELEMENTS], r[S3_BC_ELEMENTS];
    char ch; s3_cmd_t cmd;
    int ok = 1;

    for (int x = 1; x <= 4; x++) {
        for (int t = 0; t < 50; t++) {
            make_widths(x, 100, 30, 15, w);
            reverse29(w, r);
            if (s3_decode(r, S3_BC_ELEMENTS, &ch, &cmd) != S3_DEC_OK) ok = 0;
            else if ((ch != 'A' + x - 1) || ((int)cmd != x - 1))      ok = 0;
        }
    }
    for (int i = 0; i < N_ALL39; i++) {                  /* every character read backwards decodes the same */
        for (int t = 0; t < 10; t++) {
            make_widths_p(ALL39[i].p, 100, 30, 15, w);
            reverse29(w, r);
            if ((s3_decode(r, S3_BC_ELEMENTS, &ch, &cmd) != S3_DEC_OK) || (ch != ALL39[i].ch)) ok = 0;
        }
    }
    return ok;
}

int s3_st_decode_reject(void)
{
    uint32_t w[S3_BC_ELEMENTS];
    char ch; s3_cmd_t cmd;
    int ok = 1;

    make_widths(1, 100, 30, 0, w);
    if (s3_decode(w, S3_BC_ELEMENTS - 1, &ch, &cmd) != S3_DEC_ERR_COUNT) ok = 0;   /* 28 elements */

    for (int i = 0; i < S3_BC_ELEMENTS; i++) w[i] = 100;                            /* nothing is wider */
    if (s3_decode(w, S3_BC_ELEMENTS, &ch, &cmd) != S3_DEC_ERR_RATIO) ok = 0;

    make_widths(1, 100, 30, 0, w);                                                   /* damage a character */
    w[3] = 300; w[4] = 300;
    if (s3_decode(w, S3_BC_ELEMENTS, &ch, &cmd) == S3_DEC_OK) ok = 0;

    make_widths(1, 100, 30, 0, w);                                                   /* wrong gap: not narrow */
    w[9] = 300;
    if (s3_decode(w, S3_BC_ELEMENTS, &ch, &cmd) == S3_DEC_OK) ok = 0;
    return ok;
}

/* ---------------------------------------------------------------- line logic */
#define RAW_LO  300
#define RAW_HI  3500

/* raw reading of a sensor that is 'b' (0..1000) black, for the configured polarity */
static uint16_t raw_for(int b)
{
#if S3_BLACK_IS_HIGH
    return (uint16_t)(RAW_LO + (RAW_HI - RAW_LO) * b / 1000);
#else
    return (uint16_t)(RAW_HI - (RAW_HI - RAW_LO) * b / 1000);
#endif
}

static void set3(uint16_t r[3], int bl, int bc, int br) { r[0] = raw_for(bl); r[1] = raw_for(bc); r[2] = raw_for(br); }

static void calibrate_ok(s3_line_t *l)
{
    uint16_t r[3];
    s3_line_cal_begin(l);
    set3(r, 0, 0, 0);       s3_line_cal_feed(l, r);     /* white */
    set3(r, 1000, 1000, 1000); s3_line_cal_feed(l, r);  /* black */
    (void)s3_line_cal_end(l);
}

int s3_st_line_steer(void)
{
    static s3_line_t l;
    s3_line_out_t o;
    uint16_t r[3];
    int ok = 1;

    /* calibration refuses a sensor that never changed (unplugged / too far from the surface) */
    s3_line_cal_begin(&l);
    for (int i = 0; i < 20; i++) { r[0] = 200; r[1] = 3000; r[2] = 3000;
                                   s3_line_cal_feed(&l, r);
                                   r[0] = 230; s3_line_cal_feed(&l, r); r[0] = 200; r[1] = 300; r[2] = 300; s3_line_cal_feed(&l, r); }
    if (s3_line_cal_end(&l)) ok = 0;                    /* sensor 0 only moved 30 counts: must fail */

    calibrate_ok(&l);
    if (!l.calibrated) return 0;

    set3(r, 0, 1000, 0);   s3_line_step(&l, r, false, &o);          /* line centred */
    if (o.pos != 0 || o.steer != 0 || !o.centre_black || !o.any_black) ok = 0;

    set3(r, 0, 700, 500);  s3_line_step(&l, r, false, &o);          /* line towards the RIGHT sensor */
    if (!(o.pos > 30 && o.pos < 50)) ok = 0;
#if S3_STEER_KP_X10 > 0
    if (!(o.steer > 0)) ok = 0;                                      /* + = steer right (motion_types.h) */
#else
    if (!(o.steer < 0)) ok = 0;
#endif

    set3(r, 500, 700, 0);  s3_line_step(&l, r, false, &o);          /* towards the LEFT sensor */
    if (!(o.pos < -30 && o.pos > -50)) ok = 0;
    if ((o.steer > 0) == (S3_STEER_KP_X10 > 0)) ok = 0;              /* opposite sign to the right case */

    set3(r, 0, 0, 1000);   s3_line_step(&l, r, false, &o);          /* only the right sensor: full scale */
    if (o.pos != 100) ok = 0;
    if (o.steer > 100 || o.steer < -100) ok = 0;                     /* limited */
    return ok;
}

int s3_st_line_events(void)
{
    static s3_line_t l;
    s3_line_out_t o;
    uint16_t r[3];
    int ok = 1, lost = 0, found = 0, junc = 0, first_lost_at = -1;
    int8_t steer_before_loss;

    calibrate_ok(&l);

    for (int i = 0; i < 10; i++) { set3(r, 0, 1000, 0); s3_line_step(&l, r, false, &o); if (o.events) ok = 0; }   /* on the line: no events */

    set3(r, 0, 700, 500); s3_line_step(&l, r, false, &o);          /* a steering value to be held */
    steer_before_loss = o.steer;

    for (int i = 1; i <= 30; i++) {                                  /* the line disappears */
        set3(r, 0, 0, 0); s3_line_step(&l, r, false, &o);
        if (o.events & S3_LINE_EVT_LOST) { lost++; if (first_lost_at < 0) first_lost_at = i; }
        if (i < S3_LINE_LOST_SAMPLES) {                              /* not lost yet: steering held */
            if (o.steer != steer_before_loss) ok = 0;
        } else {                                                     /* lost: full steer to the side last seen (right) */
            if (o.steer != ((S3_STEER_KP_X10 > 0) ? 100 : -100)) ok = 0;
        }
    }
    if (lost != 1) ok = 0;
    if (first_lost_at != S3_LINE_LOST_SAMPLES) ok = 0;               /* 8 samples x 5 ms = 40 ms < 50 ms (FR14) */
    if (S3_LINE_LOST_SAMPLES * S3_LINE_PERIOD_MS > 50) ok = 0;

    /* centre was the last to see it: lost -> steer to the wide-gap side */
    for (int i = 0; i < 5; i++) { set3(r, 0, 1000, 0); s3_line_step(&l, r, false, &o); }
    for (int i = 0; i < S3_LINE_LOST_SAMPLES; i++) { set3(r, 0, 0, 0); s3_line_step(&l, r, false, &o); }
    if (!o.lost) ok = 0;
    if ((o.steer > 0) != ((S3_WIDE_GAP_SIDE * S3_STEER_KP_X10) > 0) || (o.steer == 0)) ok = 0;

    /* noise on white floor (small readings) is NOT a line: no position, no "found" */
    for (int i = 0; i < 5; i++) {
        set3(r, 0, 80, 120); s3_line_step(&l, r, false, &o);
        if (o.events & S3_LINE_EVT_FOUND) ok = 0;
        if (!o.lost) ok = 0;
    }

    for (int i = 0; i < 5; i++) {                                    /* the line returns */
        set3(r, 0, 1000, 0); s3_line_step(&l, r, false, &o);
        if (o.events & S3_LINE_EVT_FOUND) found++;
    }
    if (found != 1) ok = 0;

    /* a bend: the line crosses all three sensors diagonally after strong steering = NOT a junction */
    for (int i = 0; i < 30; i++) { set3(r, 0, 300, 1000); s3_line_step(&l, r, false, &o); }    /* far right */
    for (int i = 0; i < 6; i++) { set3(r, 1000, 1000, 1000); s3_line_step(&l, r, false, &o); if (o.events & S3_LINE_EVT_JUNCTION) ok = 0; }

    /* junction: all three black after going straight. 2 samples = nothing, 3rd = ONE event, no repeat */
    for (int i = 0; i < S3_JUNC_STRAIGHT_SAMPLES + 2; i++) { set3(r, 0, 1000, 0); s3_line_step(&l, r, false, &o); }
    for (int i = 1; i <= 30; i++) {
        set3(r, 1000, 1000, 1000); s3_line_step(&l, r, false, &o);
        if (o.events & S3_LINE_EVT_JUNCTION) { junc++; if (i != S3_JUNCTION_SAMPLES) ok = 0; }
    }
    if (junc != 1) ok = 0;

    set3(r, 0, 1000, 0); s3_line_step(&l, r, false, &o);             /* leave and come back: a second junction */
    junc = 0;
    for (int i = 0; i < 10; i++) { set3(r, 1000, 1000, 1000); s3_line_step(&l, r, false, &o); if (o.events & S3_LINE_EVT_JUNCTION) junc++; }
    if (junc != 1) ok = 0;

    set3(r, 0, 1000, 0); s3_line_step(&l, r, false, &o);             /* a barcode is being read: no junction event */
    junc = 0;
    for (int i = 0; i < 10; i++) { set3(r, 1000, 1000, 1000); s3_line_step(&l, r, true, &o); if (o.events & S3_LINE_EVT_JUNCTION) junc++; }
    if (junc != 0) ok = 0;
    return ok;
}

/* ---------------------------------------------------------------- barcode state machine */
#define BLACK ((uint8_t)S3_BAR_LEVEL_BLACK)
#define WHITE ((uint8_t)(1 - S3_BAR_LEVEL_BLACK))
#define MAX_EDGES 80

typedef struct { uint32_t t; uint8_t lv; } st_edge_t;

/* the edges of 29 widths: the start of element i has level BLACK for bars (even i) and WHITE for spaces,
 * plus the trailing edge of the last bar. Returns the number of edges (30). */
static int edges_from_widths(const uint32_t *w, uint32_t t0, st_edge_t *e)
{
    uint32_t t = t0;
    int n = 0;
    for (int i = 0; i < S3_BC_ELEMENTS; i++) {
        e[n].t = t; e[n].lv = ((i % 2) == 0) ? BLACK : WHITE; n++;
        t += w[i];
    }
    e[n].t = t; e[n].lv = WHITE; n++;
    return n;
}

typedef struct { int cmds; int errors; int first_value; s3_bc_evtype_t first_type; uint32_t first_at_us; } st_result_t;

static void note(st_result_t *res, const s3_bc_ev_t *ev, uint32_t now)
{
    if (ev->type == S3_BC_EV_NONE) return;
    if (ev->type == S3_BC_EV_CMD)   res->cmds++;
    if (ev->type == S3_BC_EV_ERROR) res->errors++;
    if (res->first_type == S3_BC_EV_NONE) { res->first_type = ev->type; res->first_value = ev->value; res->first_at_us = now; }
}

/* run the edges through the state machine, polling every 1 ms in between and for 'tail_us' after the last edge */
static st_result_t run(s3_bc_t *b, const st_edge_t *e, int n, uint32_t tail_us)
{
    st_result_t res = { 0, 0, 0, S3_BC_EV_NONE, 0 };
    s3_bc_ev_t ev;
    uint32_t now = e[0].t - 2000u;

    for (int i = 0; i < n; i++) {
        while ((int32_t)(e[i].t - now) > 0) {
            now += 1000u; if ((int32_t)(now - e[i].t) > 0) now = e[i].t;
            s3_bc_poll(b, now, &ev); note(&res, &ev, now);
        }
        s3_bc_edge(b, e[i].t, 0, e[i].lv, &ev); note(&res, &ev, e[i].t);
    }
    {
        uint32_t end = e[n - 1].t + tail_us;
        while ((int32_t)(end - now) > 0) { now += 1000u; s3_bc_poll(b, now, &ev); note(&res, &ev, now); }
    }
    return res;
}

static s3_bc_t bc;
static st_edge_t edges[MAX_EDGES];
static uint32_t widths[S3_BC_ELEMENTS], rev[S3_BC_ELEMENTS];

int s3_st_bc_speeds(void)
{
    static const uint32_t narrow[3] = { 3000u, 25000u, 100000u };   /* fast, robot speed, slow hand sweep (us) */
    static const uint32_t starts[2] = { 1000000u, 0xFFFF0000u };    /* normal, and the 32-bit us timer wraps mid-code */
    int ok = 1;

    for (int x = 1; x <= 4; x++)
        for (int s = 0; s < 3; s++)
            for (int dir = 0; dir < 2; dir++)
                for (int st = 0; st < 2; st++) {
                    make_widths(x, narrow[s], 25 + 5 * (x % 2), 12, widths);      /* wide = 2.5x or 3.0x, 12 % noise */
                    if (dir) { reverse29(widths, rev); edges_from_widths(rev, starts[st], edges); }
                    else                                edges_from_widths(widths, starts[st], edges);
                    s3_bc_init(&bc);
                    st_result_t r = run(&bc, edges, 30, 4000000u);
                    if (r.cmds != 1 || r.errors != 0 || r.first_type != S3_BC_EV_CMD || r.first_value != 'A' + x - 1) ok = 0;
#if S3_BC_DECODE_ON_29
                    /* FR15: the command is out within 100 ms of the stop bar (glitch window + poll = a few ms) */
                    if (r.first_at_us - edges[29].t > 100000u) ok = 0;
                    if ((int32_t)(r.first_at_us - edges[29].t) > 5000) ok = 0;
#endif
                }
    return ok;
}

int s3_st_bc_glitch(void)
{
    int ok = 1;

    for (int x = 1; x <= 4; x++) {
        static st_edge_t g[MAX_EDGES];   /* static: 1 KB initial-task stack in the TEST build */
        int n = 0;

        make_widths(x, 25000u, 30, 8, widths);
        edges_from_widths(widths, 1000000u, edges);
        /* after every 3rd edge insert a 200 us spike: two edges that cancel each other */
        for (int i = 0; i < 30; i++) {
            g[n++] = edges[i];
            if (i % 3 == 1 && i < 29) {
                g[n].t = edges[i].t + 8000u; g[n].lv = (uint8_t)(1 - edges[i].lv); n++;
                g[n].t = edges[i].t + 8200u; g[n].lv = edges[i].lv; n++;
            }
        }
        s3_bc_init(&bc);
        st_result_t r = run(&bc, g, n, 3000000u);
        if (r.cmds != 1 || r.errors != 0 || r.first_value != 'A' + x - 1) ok = 0;
    }
    return ok;
}

int s3_st_bc_errors(void)
{
    int ok = 1;
    st_result_t r;

    /* (a) the last bar never ends: only 28 edges. Must end in ONE error, never a command */
    make_widths(2, 25000u, 30, 0, widths);
    edges_from_widths(widths, 1000000u, edges);
    s3_bc_init(&bc);
    r = run(&bc, edges, 29, 3000000u);
    if (r.cmds != 0 || r.errors != 1 || r.first_value != S3_BC_ERR_TIMEOUT) ok = 0;

    /* (b) a damaged character: error, never a command */
    make_widths(2, 25000u, 30, 0, widths);
    widths[3] = widths[4] = 75000u;
    edges_from_widths(widths, 1000000u, edges);
    s3_bc_init(&bc);
    r = run(&bc, edges, 30, 3000000u);
    if (r.cmds != 0 || r.errors != 1) ok = 0;

    /* (c) one lost edge (the pin level repeats): error 'missed edge' */
    make_widths(2, 25000u, 30, 0, widths);
    edges_from_widths(widths, 1000000u, edges);
    for (int i = 10; i < 29; i++) edges[i] = edges[i + 1];           /* delete edge 10 */
    s3_bc_init(&bc);
    r = run(&bc, edges, 29, 3000000u);
    if (r.cmds != 0 || r.errors < 1) ok = 0;

    /* (d) a lone black mark (a junction line, a stain): silently ignored, no event at all */
    edges[0].t = 1000000u; edges[0].lv = BLACK;
    edges[1].t = 1060000u; edges[1].lv = WHITE;
    s3_bc_init(&bc);
    r = run(&bc, edges, 2, 3000000u);
    if (r.cmds != 0 || r.errors != 0) ok = 0;
    if (s3_bc_busy(&bc)) ok = 0;                                      /* back to waiting */

    /* (e) a first edge that is WHITE does not start a read */
    edges[0].t = 1000000u; edges[0].lv = WHITE;
    s3_bc_init(&bc);
    r = run(&bc, edges, 1, 500000u);
    if (r.cmds != 0 || r.errors != 0 || s3_bc_busy(&bc)) ok = 0;
    return ok;
}

int s3_st_bc_cooldown(void)
{
    static const uint32_t starts[2] = { 1000000u, 0xFFFD0000u };    /* normal, and the 32-bit us timer wraps inside the cooldown */
    int ok = 1;
    static st_edge_t two[2 * MAX_EDGES];
    st_result_t r;

    for (int st = 0; st < 2; st++) {
        /* the same barcode twice, 150 ms apart (inside the 300 ms cooldown): only the first is read */
        make_widths(3, 8000u, 30, 0, widths);
        edges_from_widths(widths, starts[st], edges);
        for (int i = 0; i < 30; i++) two[i] = edges[i];
        edges_from_widths(widths, edges[29].t + 150000u, edges);
        for (int i = 0; i < 30; i++) two[30 + i] = edges[i];
        s3_bc_init(&bc);
        r = run(&bc, two, 60, 2000000u);
        if (r.cmds != 1) ok = 0;

        /* 600 ms apart (cooldown over): both are read */
        edges_from_widths(widths, starts[st], edges);
        for (int i = 0; i < 30; i++) two[i] = edges[i];
        edges_from_widths(widths, edges[29].t + 600000u, edges);
        for (int i = 0; i < 30; i++) two[30 + i] = edges[i];
        s3_bc_init(&bc);
        r = run(&bc, two, 60, 2000000u);
        if (r.cmds != 2) ok = 0;
    }

    /* the 32-bit us timer wraps INSIDE the cooldown: the first code ends 100 ms before the wrap, the
       cooldown ends 200 ms after it, and a second code (60 ms later, all of it inside the cooldown) must be ignored */
    make_widths(3, 3000u, 30, 0, widths);
    edges_from_widths(widths, 0u, edges);
    {
        uint32_t span = edges[29].t;
        edges_from_widths(widths, 0xFFFE795Fu - span, edges);
    }
    for (int i = 0; i < 30; i++) two[i] = edges[i];
    edges_from_widths(widths, edges[29].t + 60000u, edges);
    for (int i = 0; i < 30; i++) two[30 + i] = edges[i];
    s3_bc_init(&bc);
    r = run(&bc, two, 60, 2000000u);
    if (r.cmds != 1) ok = 0;
    return ok;
}

#endif /* S3_HOST_TEST || APP_TEST */
