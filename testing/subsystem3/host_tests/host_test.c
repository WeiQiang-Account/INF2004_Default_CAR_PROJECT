/* Host-side check of Buddy 3's pure logic (decoder, line logic, barcode state machine).
 * Build + run on a PC, from this folder:
 *   gcc -Wall -Wextra -DS3_HOST_TEST -I../../../app_program/subsystem3 host_test.c \
 *       ../../../app_program/subsystem3/s3_decode.c ../../../app_program/subsystem3/s3_line.c \
 *       ../../../app_program/subsystem3/s3_barcode.c ../../../app_program/subsystem3/s3_selftest.c \
 *       -o host_test && ./host_test
 * Re-run with other settings to check the options, e.g. add  -DS3_BLACK_IS_HIGH=0  -DS3_BAR_LEVEL_BLACK=0
 * -DS3_BC_DECODE_ON_29=0  -DS3_STEER_KP_X10=-10                                                  */
#include <stdio.h>
#include "s3_selftest.h"

static int fails;
#define RUN(f, text) do { int r_ = f(); printf("%-5s %-16s %s\n", r_ ? "PASS" : "FAIL", #f, text); if (!r_) fails++; } while (0)

int main(void)
{
    RUN(s3_st_decode_fwd,    "S3-U01 barcodes A-D, 15 % noise");
    RUN(s3_st_decode_rev,    "S3-U02 read backwards");
    RUN(s3_st_decode_reject, "S3-U03 bad codes rejected");
    RUN(s3_st_line_steer,    "S3-U04 position / steering sign / calibration");
    RUN(s3_st_line_events,   "S3-U05 line lost 40 ms, found, junction once");
    RUN(s3_st_bc_speeds,     "S3-U06 edges -> command, 3 speeds, wrap, <100 ms");
    RUN(s3_st_bc_glitch,     "S3-U07 spikes filtered");
    RUN(s3_st_bc_errors,     "S3-U08 errors / lone mark ignored");
    RUN(s3_st_bc_cooldown,   "S3-U09 cooldown");
    printf("%s (%d failed)\n", fails ? "SOME TESTS FAILED" : "ALL PASSED", fails);
    return fails ? 1 : 0;
}
