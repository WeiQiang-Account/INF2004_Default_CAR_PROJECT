/* s3_selftest.h - self-checks of Buddy 3's pure logic (decoder, line, barcode state machine).
 * ONE source, TWO places it runs:
 *   - on a PC:       testing/subsystem3/host_tests/host_test.c   (gcc -DS3_HOST_TEST)
 *   - on the robot:  subsystem3_test.c, inside "make TEST=1"
 * Each function returns 1 = pass, 0 = fail.                                         */
#ifndef S3_SELFTEST_H
#define S3_SELFTEST_H

int s3_st_decode_fwd(void);      /* A-D read forwards  -> right command, with 15 % noise          */
int s3_st_decode_rev(void);      /* A-D read backwards -> same command                            */
int s3_st_decode_reject(void);   /* wrong count / flat widths / damaged code are rejected         */
int s3_st_line_steer(void);      /* position and steering sign; calibration check                 */
int s3_st_line_events(void);     /* lost in 8 samples (40 ms), found, junction once, suppress     */
int s3_st_bc_speeds(void);       /* edges -> command at 3 speeds, both directions, 1 event, wrap-safe, fast */
int s3_st_bc_glitch(void);       /* short spikes inside the code are filtered out                 */
int s3_st_bc_errors(void);       /* incomplete code -> error, lone black mark -> nothing          */
int s3_st_bc_cooldown(void);     /* the same barcode is not read twice                            */

#endif
