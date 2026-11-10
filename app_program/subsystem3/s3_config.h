/* s3_config.h - Buddy 3 tunables (line sensing + barcode).
 * Every value is #ifndef-guarded so it can be overridden from the makefile
 * (-DS3_BLACK_IS_HIGH=0 ...). Values marked MEASURE must be confirmed on the
 * real robot - they are starting points, not facts.                          */
#ifndef S3_CONFIG_H
#define S3_CONFIG_H

/* ---- Pins (Week 6 pin table; claim in TEAM.md section 2) ----------------- */
#define S3_PIN_IR_LEFT       26   /* GP26 = ADC0, Grove 6                        */
#define S3_PIN_IR_CENTER     27   /* GP27 = ADC1, Grove 6                        */
#define S3_PIN_IR_RIGHT      28   /* GP28 = ADC2, Grove 7 (VBAT-sense jumper off) */
#define S3_PIN_IR_RIGHT_DO    7   /* right IR digital output: barcode edges      */
#ifndef S3_SWAP_LR
#define S3_SWAP_LR            1   /* 2026-11-06: the line PHYSICALLY on the robot's left showed on the
                                     "R" bar, so it steered away from it. 1 = swap the two outer
                                     readings. Check: from behind, tape under your LEFT -> L bar dark. */
#endif

/* ---- Task priorities: TEAM.md gives S3 the band 16-17 (1 = highest) ------- */
#ifndef S3_PRI_LINE
#define S3_PRI_LINE          16   /* below Motion (14), per the Week 6 design    */
#endif
#ifndef S3_PRI_BARCODE
#define S3_PRI_BARCODE       18   /* below the VC (17), Week 6 priority order */
#endif
/* Stack size is fixed by app_start_task(): TASK_STACK_SIZE = 2048 bytes.     */

/* ---- Sensor polarity (MEASURE on the robot) ------------------------------- */
#ifndef S3_BLACK_IS_HIGH
#define S3_BLACK_IS_HIGH      1   /* 1 = ADC reading goes UP over black          */
#endif
#ifndef S3_BAR_LEVEL_BLACK
#define S3_BAR_LEVEL_BLACK    1   /* GP7 level over a black bar (set the pot)    */
#endif

/* ---- Line following ------------------------------------------------------- */
#define S3_LINE_PERIOD_MS     5   /* Line Follow period (NFR10)                  */
#ifndef S3_CAL_MS
#define S3_CAL_MS         10000   /* calibration window: sweep black AND white (retries until OK) */
#endif
#ifndef S3_MIN_CAL_RANGE
#define S3_MIN_CAL_RANGE    500   /* black/white must differ by this many counts */
#endif
#ifndef S3_BLACK_THRESHOLD
#define S3_BLACK_THRESHOLD  600   /* normalised 0..1000: above = black           */
#endif
#ifndef S3_LINE_PRESENT_MIN
#define S3_LINE_PRESENT_MIN 300   /* one sensor must be at least this dark (0..1000), else = no line */
#endif
#ifndef S3_EDGE_POS
#define S3_EDGE_POS          95   /* |line position| this far out = only a side sensor sees it:
                                     counts as "lost" for the 3 s line-lost timeout (VC)      */
#endif
#ifndef S3_SIDE_SEEN
#define S3_SIDE_SEEN        250   /* a side sensor this dark = "the line went to that side" */
#endif
#ifndef S3_WIDE_GAP_SIDE
#define S3_WIDE_GAP_SIDE   (-1)   /* MEASURE: -1 = the BIG gap is between the centre and the LEFT (L bar)
                                     sensor, +1 = between centre and RIGHT. 2026-11-06 photo: left.  */
#endif
#ifndef S3_LINE_LOST_SUM
#define S3_LINE_LOST_SUM    150   /* sum of 3 sensors below this = no line       */
#endif
#define S3_LINE_LOST_SAMPLES  8   /* 8 x 5 ms = 40 ms (FR14: within 50 ms)       */
#define S3_JUNCTION_SAMPLES   3   /* all 3 black for 3 x 5 ms = junction         */
#ifndef S3_STEER_KP_X10
#define S3_STEER_KP_X10       7   /* steering gain x10 (10 = 1.0). NEGATIVE inverts it:
                                     use that if the robot steers AWAY from the line  */
#endif

/* ---- Barcode (Code 39 "*X*" = 3 characters) ------------------------------- */
#define S3_BC_ELEMENTS       29   /* 3 chars x 9 elements + 2 gaps               */
#ifndef S3_GLITCH_US
#define S3_GLITCH_US       1000   /* pulses shorter than this are noise (bench: DO chatters ~200 us) */
#endif
#ifndef S3_ABORT_US
#define S3_ABORT_US     1000000   /* no edge for this long (and long vs the widest element) = give up */
#endif
#ifndef S3_COOLDOWN_US
#define S3_COOLDOWN_US   300000   /* ignore edges after a good read              */
#endif
#define S3_EDGE_BUF_SIZE    128   /* ISR -> task ring buffer, power of 2         */
#define S3_BC_MIN_ERR_ELEMENTS 5  /* a failed read with fewer elements than this is noise (stain, crossing line): dropped silently */
#define S3_BC_POLL_MS         2   /* Barcode task wakes at least this often (glitch window, timeouts) */
#ifndef S3_BC_DECODE_ON_29
#define S3_BC_DECODE_ON_29    1   /* 1 = decode the moment the 29th element ends (meets FR15: <= 100 ms).
                                     0 = also wait for the quiet zone (white gap > 3x widest element), as
                                     in the Week 6 statechart text; that wait alone can exceed 100 ms. */
#endif

/* ---- Console output (tm_printf) ------------------------------------------- */
#ifndef S3_VERBOSE
#define S3_VERBOSE            1   /* print one line per barcode / junction / line-lost event */
#endif
#ifndef S3_STATUS_MS
#define S3_STATUS_MS          0   /* >0: print raw/normalised/steer every N ms (bring-up). 0 = off */
#endif

/* ---- Optional timing pin for the 5 ms jitter test (A2). -1 = off ---------- */
#ifndef S3_TIMING_PIN
#define S3_TIMING_PIN        -1   /* claim a free pin in TEAM.md first (e.g. GP6) */
#endif

/* ---- Template example (consumer) --------------------------------------------
 * Kept ONLY so the template example in S2 (example_producer.c) and S4
 * (task_monitor.c) still builds. They include subsystem3.h and use mbx_samples,
 * mtx_stats and app_stats. Set to 0 once S2 and S4 have replaced their
 * examples, then delete task_consumer.c.   (S2 does the same, see
 * S2_KEEP_TEMPLATE_EXAMPLE.)                                                  */
#ifndef S3_KEEP_TEMPLATE_EXAMPLE
#define S3_KEEP_TEMPLATE_EXAMPLE 0
#endif

#endif /* S3_CONFIG_H */
