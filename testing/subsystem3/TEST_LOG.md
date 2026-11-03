# Subsystem 3 — test log

**Subsystem:** Sensors, line tracking, Code 39 barcode  ·  **Student ID:** 2500142 (Buddy 3)  ·  **Code:** `app_program/subsystem3/`

See `testing/README.md` for how to run tests and record results.

## Test cases

Define a test here before running it. Test IDs never change.

| ID | What is tested | Requirement / interface | Method | Steps / how | Expected result |
|---|---|---|---|---|---|
| S3-U01 | decode `*A*..*D*` forward, 15 % jitter | FR15 | auto | `subsystem3_test.c` → `s3_st_decode_fwd` | `[TEST] S3-U01 PASS` |
| S3-U02 | decode reversed (scanned right-to-left) | FR15 | auto | `s3_st_decode_rev` | PASS |
| S3-U03 | reject bad count / ratio / damaged char | FR15, NFR11 | auto | `s3_st_decode_reject` | PASS |
| S3-U04 | steering sign and clamp (+ = steer right) | IF-09 | auto | `s3_st_line_steer` | PASS |
| S3-U05 | junction / line-lost / line-found events | FR14 | auto | `s3_st_line_events` | PASS |
| S3-U06 | barcode state machine at slow/fast speeds | FR15 | auto | `s3_st_bc_speeds` | PASS |
| S3-U07 | glitch pulses (< 1 ms) ignored | NFR10 | auto | `s3_st_bc_glitch` | PASS |
| S3-U08 | errors: missed edge, too many, timeout; lone mark dropped silently | FR15 | auto | `s3_st_bc_errors` | PASS |
| S3-U09 | cooldown, incl. 32-bit µs timer wrap | FR15 | auto | `s3_st_bc_cooldown` | PASS |
| S3-U10 | calibration ready, no ADC timeouts (robot on track, swept in first 5 s) | FR13 | auto (live) | build with `-DS3_LIVE_TESTS=1` | PASS |
| S3-U11 | `steer_latest` sequence advances ≈ 20 in 100 ms | IF-09, NFR10 | auto (live) | `-DS3_LIVE_TESTS=1` | PASS |
| S3-U12 | GP7 gives ≥ 4 edges in 5 s while a barcode is moved past | IF-11 | auto (live) | `-DS3_LIVE_TESTS=1`, wave barcode | PASS |
| S3-M01 | ADC bring-up: L, C, R each swing > 500 counts black↔white | FR13 | manual | `S3_STATUS_MS=200`, read raw values on console | all three swing |
| S3-M02 | steering sign: line under left sensor → steer negative | IF-09 | manual | `S3_STATUS_MS=200`, move line under L / R | sign matches; else set `S3_STEER_KP_X10` negative |
| S3-M03 | GP7 raw edges clean (no chatter) | NFR10 | manual | scope/console on GP7 while hand-sweeping | no pulses < 1 ms; else tune pot |
| S3-M04 | barcode scan A,B,C,D, ≥ 19 of 20 passes | NFR11 | manual | 5 passes each, record count | ≥ 95 % |
| S3-M05 | loop period 5 ms, edge latency | NFR10 | manual | `S3_TIMING_PIN` on a scope | 5 ms ± jitter |
| S3-M06 | line lost reported within 50 ms | FR14 | manual | lift robot, note time to event | ≤ 50 ms |
| INT-S3-01 | S3 → S2 steer reaches motion task | IF-09 | manual | run whole build, move line | robot steers toward line |
| INT-S3-02 | S3 → VC events (junction, lost, found, barcode) | IF-VC | manual | VC log shows events | events arrive, valid=1 |
| INT-S3-03 | `s5_hook_line_detected()` override | S5 | manual | place robot on / off line | 1 on line, 0 off |
| INT-S3-04 | GP7 branch under real IRQ load, < 2 µs | IF-11 | manual | timing pin + encoders running | no dropped edges |

*Method:* **auto** = `TEST_CHECK` in code, result printed on the console;
**manual** = a person observes the result (LED, scope, meter, phone app…).

**PC-only check (no board):**
`cd testing/subsystem3/host_tests && gcc -Wall -Wextra -DS3_HOST_TEST -I../../../app_program/subsystem3 host_test.c ../../../app_program/subsystem3/{s3_decode,s3_line,s3_barcode,s3_selftest}.c -o host_test && ./host_test`

## Results

Add a new row for every run. Never edit or delete old rows.

| Date | Test ID | Commit | Tester | Result | Log / evidence | Notes |
|---|---|---|---|---|---|---|
| YYYY-MM-DD | S3-U01 | `abc1234` | | PASS / FAIL | `logs/YYYY-MM-DD_….txt` | |
