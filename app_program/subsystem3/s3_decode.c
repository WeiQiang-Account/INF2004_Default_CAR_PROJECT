/*
 *	s3_decode.c
 *	Subsystem 3 -- Code 39 decoder. PURE logic (no hardware, no kernel calls).
 *
 *	Code 39 basics:
 *	  - each character = 9 elements: bar, space, bar, space, ... bar
 *	  - exactly 3 of the 9 are WIDE, 6 are NARROW
 *	  - one narrow space (the gap) between characters
 *	  - our barcode is  * X *  (X = A..D)  ->  9 + 1 + 9 + 1 + 9 = 29 elements
 */
#include "s3_logic.h"

/* 9-bit patterns, first element on the left, 1 = wide */
typedef struct { uint16_t pattern; char ch; } code39_t;

static const code39_t table[] = {
    { 0x094, '*' },     /* 010010100 */
    { 0x109, 'A' },     /* 100001001 */
    { 0x049, 'B' },     /* 001001001 */
    { 0x148, 'C' },     /* 101001000 */
    { 0x019, 'D' }      /* 000011001 */
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

    /* 6. must read  * X *  with X in A..D */
    if ((chars[0] != '*') || (chars[2] != '*')) return S3_DEC_ERR_FORMAT;
    if ((chars[1] < 'A') || (chars[1] > 'D'))   return S3_DEC_ERR_FORMAT;

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
    switch (ch) {
    case 'A': *out_cmd = S3_CMD_LEFT;     break;
    case 'B': *out_cmd = S3_CMD_RIGHT;    break;
    case 'C': *out_cmd = S3_CMD_STRAIGHT; break;
    default:  *out_cmd = S3_CMD_UTURN;    break;      /* 'D' */
    }
    return S3_DEC_OK;
}
