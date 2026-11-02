# Subsystem 5 — test log

**Subsystem:** Obstacle handling + Vehicle Controller (Buddy 5)  ·  **Student ID:** 2401151  ·  **Code:** `app_program/subsystem5/`

See `testing/README.md` for how to run tests and record results.

## Test cases

Define a test here before running it. Test IDs never change.

| ID | What is tested | Requirement / interface | Method | Steps / how | Expected result |
|---|---|---|---|---|---|
| S5-U01 | Profile pack/unpack round trip (8-byte PROFILE event) | IF-16, Tribunal min-bit rule | auto | `subsystem5_test.c` | `[TEST] S5-U01 PASS` |
| S5-U02 | Open space on both sides: plan picks a side | FR23 | auto | `subsystem5_test.c` | `[TEST] S5-U02 PASS` |
| S5-U03 | Left side too narrow: plan goes right | FR22, FR23 | auto | `subsystem5_test.c` | `[TEST] S5-U03 PASS` |
| S5-U04 | Closer than 10 cm: reverse and retry; stop after 3 rescans | FR23, NFR18 | auto | `subsystem5_test.c` | `[TEST] S5-U04 PASS` |
| S5-U05 | Status byte packing (3+2+3 bits) and left/right geometry sign | Tribunal min-bit rule | auto | `subsystem5_test.c` | `[TEST] S5-U05 PASS` |
| S5-U06 | 4 events posted to `mbf_vc_events`, none dropped | IF-14, R-interface | auto | `subsystem5_test.c` | `[TEST] S5-U06 PASS` |
| S5-U07 | Scan task completes a live scan (proves TRIG/ECHO/servo wiring and S2's GPIO IRQ) | FR20, A9 | auto | `subsystem5_test.c`, sensor wired, nothing within 30 cm | `[TEST] S5-U07 PASS` |
| S5-U08 | Coarse scan of 5 angles <= 1200 ms | NFR16, A10 | auto | `subsystem5_test.c` (prints `[S5] coarse scan N ms`) | `[TEST] S5-U08 PASS` |
| S5-M01 | Distance accuracy at 10, 20, 40 cm (flat board, ruler) | NFR17, A9 | manual | set `S5_MANUAL_TESTS 1`; hold board; compare `[S5-M01] ahead = N mm` with ruler | within +/-10 mm |
| S5-M02 | ECHO pin voltage before connecting the sensor | A9 | manual | multimeter on GP14 with divider fitted | <= 3.3 V |
| S5-M03 | Stop distance: robot stops >= 100 mm before a box, 10 runs | NFR18 | manual | line + box ahead, video with ruler in view | 10/10 stop >= 100 mm |
| S5-M04 | Obstacle on the line: stop, scan, bypass, line found again | FR20-FR24, NFR19 | manual | box on the line; serial log shows `[S5]` and `[VC]` lines | rejoins line within 2 s of `[VC] ... recovery` start |
| S5-M05 | Scripted state sequence of the Vehicle Controller | system statechart | manual | press START (GP20), observe `[VC]` log, press STOP (GP21), START again | READY -> RUNNING -> PAUSED -> RUNNING |

*Method:* **auto** = `TEST_CHECK` in code, result printed on the console;
**manual** = a person observes the result (LED, scope, meter, phone app…).

## Results

Add a new row for every run. Never edit or delete old rows.

| Date | Test ID | Commit | Tester | Result | Log / evidence | Notes |
|---|---|---|---|---|---|---|
| YYYY-MM-DD | S5-U01 | `abc1234` | | PASS / FAIL | `logs/YYYY-MM-DD_….txt` | |
