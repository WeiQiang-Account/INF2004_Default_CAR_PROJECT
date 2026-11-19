/*
 *	s3_decode.c
 *	Subsystem 3 -- Code 39 decoder. PURE logic (no hardware, no kernel calls).
 *
 *	Code 39 basics:
 *	  - each character = 9 elements: bar, space, bar, space, ... bar
 *	  - exactly 3 of the 9 are WIDE, 6 are NARROW
 *	  - one narrow space (the gap) between characters
 *	  - our barcode is  * X *  (X = any Code 39 character except '*')  ->  9 + 1 + 9 + 1 + 9 = 29 elements
 */
#include "s3_logic.h"

/* 9-bit patterns, first element on the left, 1 = wide */
typedef struct { uint16_t pattern; char ch; } code39_t;

static const code39_t table[] = {      /* full Code 39 set (2026-11-11: demo needs A-Z) */
    { 0x034, '0' },     /* 000110100 */
    { 0x121, '1' },     /* 100100001 */
    { 0x061, '2' },     /* 001100001 */
    { 0x160, '3' },     /* 101100000 */
    { 0x031, '4' },     /* 000110001 */
    { 0x130, '5' },     /* 100110000 */
    { 0x070, '6' },     /* 001110000 */
    { 0x025, '7' },     /* 000100101 */
    { 0x124, '8' },     /* 100100100 */
    { 0x064, '9' },     /* 001100100 */
    { 0x109, 'A' },     /* 100001001 */
    { 0x049, 'B' },     /* 001001001 */
    { 0x148, 'C' },     /* 101001000 */
    { 0x019, 'D' },     /* 000011001 */
    { 0x118, 'E' },     /* 100011000 */
    { 0x058, 'F' },     /* 001011000 */
    { 0x00D, 'G' },     /* 000001101 */
    { 0x10C, 'H' },     /* 100001100 */
    { 0x04C, 'I' },     /* 001001100 */
    { 0x01C, 'J' },     /* 000011100 */
    { 0x103, 'K' },     /* 100000011 */
    { 0x043, 'L' },     /* 001000011 */
    { 0x142, 'M' },     /* 101000010 */
    { 0x013, 'N' },     /* 000010011 */
    { 0x112, 'O' },     /* 100010010 */
    { 0x052, 'P' },     /* 001010010 */
    { 0x007, 'Q' },     /* 000000111 */
    { 0x106, 'R' },     /* 100000110 */
    { 0x046, 'S' },     /* 001000110 */
    { 0x016, 'T' },     /* 000010110 */
    { 0x181, 'U' },     /* 110000001 */
    { 0x0C1, 'V' },     /* 011000001 */
    { 0x1C0, 'W' },     /* 111000000 */
    { 0x091, 'X' },     /* 010010001 */
    { 0x190, 'Y' },     /* 110010000 */
    { 0x0D0, 'Z' },     /* 011010000 */
    { 0x085, '-' },     /* 010000101 */
    { 0x184, '.' },     /* 110000100 */
    { 0x0C4, ' ' },     /* 011000100 */
    { 0x094, '*' },     /* 010010100 */
    { 0x0A8, '$' },     /* 010101000 */
    { 0x0A2, '/' },     /* 010100010 */
    { 0x08A, '+' },     /* 010001010 */
    { 0x02A, '%' },     /* 000101010 */
};
#define TABLE_SIZE  ((int)(sizeof(table) / sizeof(table[0])))

static char lookup(uint16_t pattern)            /* 0 = not found */
{
    for (int i = 0; i < TABLE_SIZE; i++) {
        if (table[i].pattern == pattern) return table[i].ch;
    }
    return 0;
}

/* Try to decode the 29 widths in the order given. */
static s3_dec_t decode_once(const uint32_t *w, char *out_char)
{
    uint32_t bar_min = 0xFFFFFFFFu, bar_max = 0;
    uint32_t sp_min  = 0xFFFFFFFFu, sp_max  = 0;
    char     chars[3];

    /* 1. smallest/biggest width, bars and spaces SEPARATELY (even index = bar) */
    for (int i = 0; i < S3_BC_ELEMENTS; i++) {
        if ((i % 2) == 0) {
            if (w[i] < bar_min) bar_min = w[i];
            if (w[i] > bar_max) bar_max = w[i];
        } else {
            if (w[i] < sp_min) sp_min = w[i];
            if (w[i] > sp_max) sp_max = w[i];
        }
    }

    /* 2. wide must be clearly wider than narrow (>= 1.7x). 64-bit: no overflow on slow reads */
    if (((uint64_t)bar_max * 10u < (uint64_t)bar_min * 17u) ||
        ((uint64_t)sp_max  * 10u < (uint64_t)sp_min  * 17u)) {
        return S3_DEC_ERR_RATIO;
    }

    /* 3. threshold = halfway between smallest and biggest */
    uint32_t thr_bar = bar_min + (bar_max - bar_min) / 2u;
    uint32_t thr_sp  = sp_min  + (sp_max  - sp_min)  / 2u;

    /* 4. each group of 9 elements -> a 9-bit pattern -> a character */
    for (int c = 0; c < 3; c++) {
        uint16_t pattern = 0;
        for (int k = 0; k < 9; k++) {
            int idx  = c * 10 + k;                       /* +1 gap after each character */
            int wide = ((idx % 2) == 0) ? (w[idx] > thr_bar) : (w[idx] > thr_sp);
            pattern = (uint16_t)((pattern << 1) | (uint16_t)wide);
        }
        chars[c] = lookup(pattern);
        if (chars[c] == 0) return S3_DEC_ERR_NO_MATCH;
    }

    /* 5. the two gaps (index 9 and 19) must be narrow spaces */
    if ((w[9] > thr_sp) || (w[19] > thr_sp)) return S3_DEC_ERR_FORMAT;

    /* 6. must read  * X *  with X any character except the start/stop '*' */
    if ((chars[0] != '*') || (chars[2] != '*')) return S3_DEC_ERR_FORMAT;
    if (chars[1] == '*')                        return S3_DEC_ERR_FORMAT;

    *out_char = chars[1];
    return S3_DEC_OK;
}

s3_dec_t s3_decode(const uint32_t *w, int n, char *out_char, s3_cmd_t *out_cmd)
{
    char     ch = 0;
    s3_dec_t r;
    uint32_t rev[S3_BC_ELEMENTS];

    if (n != S3_BC_ELEMENTS) return S3_DEC_ERR_COUNT;

    r = decode_once(w, &ch);

    /* failed? maybe the robot read it backwards: reverse the widths and retry */
    if (r != S3_DEC_OK) {
        for (int i = 0; i < S3_BC_ELEMENTS; i++) rev[i] = w[S3_BC_ELEMENTS - 1 - i];
        if (decode_once(rev, &ch) != S3_DEC_OK) return r;     /* report the first error */
    }

    *out_char = ch;
    /* Legacy fixed meaning (old A-D course only). The robot now maps the CHARACTER to a
       movement with the table sent over MQTT (vehicle_ctrl.c), not with this. */
    switch (ch) {
    case 'A': *out_cmd = S3_CMD_LEFT;     break;
    case 'B': *out_cmd = S3_CMD_RIGHT;    break;
    case 'D': *out_cmd = S3_CMD_UTURN;    break;
    default:  *out_cmd = S3_CMD_STRAIGHT; break;
    }
    return S3_DEC_OK;
}

/* 2026-11-18: the start '*' + gap + the letter + the next gap is enough.
 * Used when a read loses some of the LAST bars (the stop '*'), so one pass still gives the
 * letter. Read backwards, the first character is the stop '*' reversed (0x052) and the
 * letter's bits come out reversed. The letter must pass the normal 3-wide-of-9 check. */
static uint16_t rev9(uint16_t p)
{
    uint16_t r = 0;
    for (int k = 0; k < 9; k++) { r = (uint16_t)((r << 1) | (p & 1u)); p >>= 1; }
    return r;
}

s3_dec_t s3_decode_partial(const uint32_t *w, int n, char *out_char)
{
    uint32_t bar_min = 0xFFFFFFFFu, bar_max = 0, sp_min = 0xFFFFFFFFu, sp_max = 0;
    uint16_t p[2] = { 0, 0 };
    char     ch;

    if (n < S3_BC_PARTIAL_ELEMENTS) return S3_DEC_ERR_COUNT;
    for (int i = 0; i < S3_BC_PARTIAL_ELEMENTS; i++) {
        if ((i % 2) == 0) { if (w[i] < bar_min) bar_min = w[i]; if (w[i] > bar_max) bar_max = w[i]; }
        else              { if (w[i] < sp_min)  sp_min  = w[i]; if (w[i] > sp_max)  sp_max  = w[i]; }
    }
    if (((uint64_t)bar_max * 10u < (uint64_t)bar_min * 17u) ||
        ((uint64_t)sp_max  * 10u < (uint64_t)sp_min  * 17u)) return S3_DEC_ERR_RATIO;
    uint32_t thr_bar = bar_min + (bar_max - bar_min) / 2u;
    uint32_t thr_sp  = sp_min  + (sp_max  - sp_min)  / 2u;
    for (int c = 0; c < 2; c++) {
        for (int k = 0; k < 9; k++) {
            int idx = c * 10 + k;
            int wide = ((idx % 2) == 0) ? (w[idx] > thr_bar) : (w[idx] > thr_sp);
            p[c] = (uint16_t)((p[c] << 1) | (uint16_t)wide);
        }
    }
    if ((w[9] > thr_sp) || (w[19] > thr_sp)) return S3_DEC_ERR_FORMAT;   /* both gaps narrow */
    if (p[0] == 0x094u)      ch = lookup(p[1]);          /* forwards: starts with '*'      */
    else if (p[0] == 0x052u) ch = lookup(rev9(p[1]));    /* backwards: '*' read in reverse */
    else return S3_DEC_ERR_FORMAT;
    if (ch == 0)   return S3_DEC_ERR_NO_MATCH;
    if (ch == '*') return S3_DEC_ERR_FORMAT;
    *out_char = ch;
    return S3_DEC_OK;
}

/* ---------------------------------------------------------------------------------
 * 2026-11-18: best-fit decode, for a blurred sensor.
 * The sensor spot is about as wide as a narrow bar, so measured widths are bent: bars
 * come out wider and gaps narrower (or the other way), and a narrow element next to a
 * wide one is pulled towards it. The halfway threshold then gets single elements wrong.
 * Here every element is modelled as   width = N + D x wide + B x (bar ? +1 : -1)
 * (N narrow width, D extra for wide, B the blur bias between bars and gaps). N, D, B are
 * fitted (least squares) on the parts we KNOW: the start/stop '*' and the two gaps.
 * Then every letter in the table is tried for the middle 9 and the one that fits best
 * wins, if it fits clearly better than the next best. Forwards and backwards are both
 * tried. Needs the whole '*' + gap + letter + gap (20 elements); the stop '*' helps the
 * fit when it is there.
 * --------------------------------------------------------------------------------- */
static float fab(float x) { return (x < 0.0f) ? -x : x; }
static int is_wide(uint16_t p, int k) { return (p >> (8 - k)) & 1; }   /* k = 0..8, first element = top bit */

/* x[29] in forwards order, m[29] = measured. Returns the best letter (0 = none), *err = its
   squared error / D^2, *err2 = the second best's */
static char fit_one(const float *x, const uint8_t *m, float *err, float *err2)
{
    /* model of the known elements: 0-8 '*', 9 gap, 19 gap, 20-28 '*' */
    float a[3][4] = { {0} };                         /* normal equations [A^T A | A^T x] */
    int used = 0;
    for (int i = 0; i < 29; i++) {
        if (!m[i] || (i >= 10 && i <= 18)) continue;
        int wide = 0;
        if (i <= 8)       wide = is_wide(0x094u, i);
        else if (i >= 20) wide = is_wide(0x094u, i - 20);
        float r[3] = { 1.0f, (float)wide, (i % 2 == 0) ? 1.0f : -1.0f };
        for (int p = 0; p < 3; p++) { for (int q = 0; q < 3; q++) a[p][q] += r[p] * r[q]; a[p][3] += r[p] * x[i]; }
        used++;
    }
    if (used < 11) return 0;
    for (int c = 0; c < 3; c++) {                    /* Gauss-Jordan, 3x3 */
        int piv = c;
        for (int r = c + 1; r < 3; r++) if (fab(a[r][c]) > fab(a[piv][c])) piv = r;
        if (fab(a[piv][c]) < 1e-6f) return 0;
        if (piv != c) for (int k = 0; k < 4; k++) { float t = a[c][k]; a[c][k] = a[piv][k]; a[piv][k] = t; }
        for (int r = 0; r < 3; r++) if (r != c) {
            float f = a[r][c] / a[c][c];
            for (int k = c; k < 4; k++) a[r][k] -= f * a[c][k];
        }
    }
    float N = a[0][3] / a[0][0], D = a[1][3] / a[1][1], B = a[2][3] / a[2][2];
    if (!(N > 0.0f) || !(D > 0.4f * N)) return 0;   /* wide must really be wider (>= 1.4x) */

    float e_known = 0.0f;
    for (int i = 0; i < 29; i++) {
        if (!m[i] || (i >= 10 && i <= 18)) continue;
        int wide = (i <= 8) ? is_wide(0x094u, i) : ((i >= 20) ? is_wide(0x094u, i - 20) : 0);
        float d = x[i] - (N + D * (float)wide + B * ((i % 2 == 0) ? 1.0f : -1.0f));
        e_known += d * d;
    }
    e_known /= (D * D);
    if (e_known > S3_FIT_KNOWN_ERR * (float)used) return 0;      /* the start/stop do not look like a star: not a barcode */

    float best = 1e30f, second = 1e30f; char bc = 0;
    for (int t = 0; t < TABLE_SIZE; t++) {
        if (table[t].ch == '*') continue;
        float e = 0.0f;
        for (int k = 0; k < 9; k++) {
            int i = 10 + k;
            float d = x[i] - (N + D * (float)is_wide(table[t].pattern, k) + B * ((i % 2 == 0) ? 1.0f : -1.0f));
            e += d * d;
        }
        e /= (D * D);
        if (e < best) { second = best; best = e; bc = table[t].ch; }
        else if (e < second) second = e;
    }
    *err = best; *err2 = second;
    return bc;
}

s3_dec_t s3_decode_fit(const uint32_t *w, int n, char *out_char)
{
    float   x[29];
    uint8_t m[29];
    char    best_c = 0;
    float   best_e = 1e30f, best_e2 = 1e30f;

    if (n < S3_BC_PARTIAL_ELEMENTS) return S3_DEC_ERR_COUNT;
    if (n > S3_BC_ELEMENTS) n = S3_BC_ELEMENTS;
    for (int dir = 0; dir < 2; dir++) {
        for (int i = 0; i < 29; i++) { x[i] = 0.0f; m[i] = 0; }
        for (int i = 0; i < n; i++) {
            int j = (dir == 0) ? i : (28 - i);           /* backwards: the read starts at the stop '*' */
            x[j] = (float)w[i]; m[j] = 1;
        }
        float e, e2;
        char c = fit_one(x, m, &e, &e2);
        if (c && (e < best_e)) { best_c = c; best_e = e; best_e2 = e2; }
    }
    if (best_c == 0) return S3_DEC_ERR_FORMAT;
    /* clear winner: small error, and the next letter fits much worse */
    {   /* without the whole stop '*' the fit has less to check: ask for a clearer winner */
        float margin = (n < S3_BC_ELEMENTS) ? S3_FIT_MARGIN_PART : S3_FIT_MARGIN;
        float maxe   = (n < S3_BC_ELEMENTS) ? (S3_FIT_MAX_ERR * 0.6f) : S3_FIT_MAX_ERR;
        if ((best_e > maxe) || (best_e2 < margin * best_e) || (best_e2 < S3_FIT_MIN_GAP)) return S3_DEC_ERR_NO_MATCH;
    }
    *out_char = best_c;
    return S3_DEC_OK;
}
