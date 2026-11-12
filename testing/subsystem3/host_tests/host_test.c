/* PC test for Buddy 3's pure logic (decoder, line, barcode state machine): make */
#include <stdio.h>
#include "s3_selftest.h"

static int fails;
static void check(const char *id, int ok, const char *what)
{
	printf("[HOST] %s %s  %s\n", id, ok ? "PASS" : "FAIL", what);
	if (!ok) { fails++; }
}

int main(void)
{
	check("S3-U01", s3_st_decode_fwd(),    "A-D + all 43 Code 39 chars forwards, 15 % noise");
	check("S3-U02", s3_st_decode_rev(),    "A-D + all 43 chars backwards -> same character");
	check("S3-U03", s3_st_decode_reject(), "wrong count / flat widths / damaged code rejected");
	check("S3-U04", s3_st_line_steer(),    "line position, steering sign, calibration check");
	check("S3-U05", s3_st_line_events(),   "lost in 40 ms, found, junction once, suppressed");
	check("S3-U06", s3_st_bc_speeds(),     "barcode at 3 speeds, both directions, wrap-safe");
	check("S3-U07", s3_st_bc_glitch(),     "short spikes filtered");
	check("S3-U08", s3_st_bc_errors(),     "incomplete code -> error, lone mark -> nothing");
	check("S3-U09", s3_st_bc_cooldown(),   "same barcode not read twice");
	printf("%d failed\n", fails);
	return fails;
}
