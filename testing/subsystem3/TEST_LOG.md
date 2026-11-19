# Subsystem 3 — test log

**Subsystem:** Line following + barcode  ·  **Student ID:** 2500142  ·  **Code:** `app_program/subsystem3/`

See `testing/README.md` for how to run tests and record results.

## Test cases

Define a test here before running it. Test IDs never change.

| ID | What is tested | Requirement / interface | Method | Steps / how | Expected result |
|---|---|---|---|---|---|
| S3-U01 | barcodes A–D decode to the right command (15 % noise) | FR15 | auto | `subsystem3_test.c` | `[TEST] S3-U01 PASS` |
| S3-U02 | barcodes read backwards decode the same | FR15 | auto | as above | `[TEST] S3-U02 PASS` |
| S3-U03 | 28 elements / flat widths / damaged code rejected | FR15 | auto | as above | `[TEST] S3-U03 PASS` |
| S3-U04 | line position, steering sign, calibration range check | FR13 | auto | as above | `[TEST] S3-U04 PASS` |
| S3-U05 | line lost in 8 samples (40 ms), found, junction once; a cross-junction is still seen while the barcode reader is busy, bars beside the line are not a junction (2026-11-18) | FR14 | auto | as above | `[TEST] S3-U05 PASS` |
| S3-U06 | edges → command at 3 speeds, both ways, timer wrap, within 100 ms | FR15 | auto | as above | `[TEST] S3-U06 PASS` |
| S3-U07 | spikes shorter than 1 ms inside the code filtered | FR15 | auto | as above | `[TEST] S3-U07 PASS` |
| S3-U08 | too short or damaged code → error; lost last bars (stop `*`) → letter still read from start `*` + letter; lone black mark ignored | FR15 | auto | as above | `[TEST] S3-U08 PASS` |
| S3-U09 | same barcode not read twice inside 300 ms | FR15 | auto | as above | `[TEST] S3-U09 PASS` |
| S3-U13 | thin bars with a blurred sensor: analog signal with a 9 mm spot, 6 mm narrow bars, only 50 % dark, noise → A–Z both directions read, ≥ 50/52 right, no wrong letter (adaptive bar finder + halfway edges + best-fit decode, 2026-11-18) | FR15 | auto | as above | `[TEST] S3-U13 PASS` |
| S3-U14 | junction from the left (barcode) sensor, which sits ~3 cm behind the centre/right pair: a 20 mm cross line → one junction, reported ~45 mm past its middle; a barcode, the track line (60 mm), the pair off the line, or a bend → none (2026-11-18) | FR13 | auto | as above | `[TEST] S3-U14 PASS` |
| S3-U10 | IR sensors calibrated, no ADC errors | IF: IR ADC | auto | as above (sweep the sensors over black and white) | `[TEST] S3-U10 PASS` |
| S3-U11 | line task runs every 5 ms (15–25 samples per 100 ms) | NFR | auto | as above | `[TEST] S3-U11 PASS` |
| S3-U12 | GP7 edge interrupt counts edges (≥ 4 in 5 s) | IF: GPIO IRQ | auto | as above (pass a black mark under the barcode sensor) | `[TEST] S3-U12 PASS` |
| S3-M01 | follow the line for a full lap of the track | FR13 | manual | normal build; calibrate IR on the track; START on a straight; video robot + dashboard | robot stays on the line, no FAULT |
| S3-M02 | barcodes A / B / C / D read while driving | FR15 | manual | each barcode on the track 3 times; dashboard "Last barcode" | right letter every time; A left, B right, C straight, D U-turn |

*Method:* **auto** = `TEST_CHECK` in code, result printed on the console;
**manual** = a person observes the result (LED, scope, meter, phone app…).

## Results

Add a new row for every run. Never edit or delete old rows.

| Date | Test ID | Commit | Tester | Result | Log / evidence | Notes |
|---|---|---|---|---|---|---|
| 2026-11-10 | S3-U01 … S3-U09 | n/a (Temp_Test copy, no git) | 2500142 | PASS (9/9) | serial output read live (not saved); full re-run with saved log planned 2026-11-11 | `make NET=lwip MQTT=1 TEST=1`, USB only. U10–U12 need the IR sweep / barcode mark: record them in the 2026-11-11 re-run |
