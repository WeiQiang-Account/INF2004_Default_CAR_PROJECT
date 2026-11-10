# Subsystem 4 — test log

**Subsystem:** IMU: safety, hump height, motion class, watchdog  ·  **Student ID:** 2502347  ·  **Code:** `app_program/subsystem4/`

See `testing/README.md` for how to run tests and record results.

## Test cases

Define a test here before running it. Test IDs never change.

| ID | What is tested | Requirement / interface | Method | Steps / how | Expected result |
|---|---|---|---|---|---|
| S4-U01 | sqrt / sin / atan2 without libm | – | auto + host | `subsystem4_test.c`; PC: `make -C testing/subsystem4/host_tests` | `S4-U01 PASS` |
| S4-U02 | static offsets; moving robot refused | FR16 | auto + host | as above | `S4-U02 PASS` |
| S4-U03 | impact on 1 sample, tilt after 60 ms, release after 1 s calm | NFR14 | auto + host | as above | `S4-U03 PASS` |
| S4-U04 | pitch sign and complementary filter | FR17 | auto + host | as above | `S4-U04 PASS` |
| S4-U05 | humps of 20 / 10 / 23 mm (any shape) measured to ±1 mm | FR17, NFR15 | auto + host | as above | `S4-U05 PASS` |
| S4-U06 | 40 ms spike ignored, long climb = ramp | FR17 | auto + host | as above | `S4-U06 PASS` |
| S4-U07 | motion classes held 60 ms | FR18 | auto + host | as above | `S4-U07 PASS` |
| S4-U08 | sample event flag created | – | auto | `subsystem4_test.c` | `S4-U08 PASS` |
| S4-U09 | IMU ready (or LOST) within 6 s | IF-16 | auto | `subsystem4_test.c` (prints WHO_AM_I, I2C errors) | `S4-U09 PASS`; GY-511 (LSM303DLHC) found at 0x19 (an MPU-6050 would be 0x68) |
| S4-U10 | 50 Hz sampling | NFR13 | auto | `subsystem4_test.c` | 48–52 samples/s |
| S4-M01 | tap the robot → E-stop | NFR14, IF-08 | manual | robot RUNNING on blocks; tap the chassis | wheels stop; console `[S4] IMPACT`; VC goes FAULT; telemetry fault bit 5 |
| S4-M02 | hump height | FR17, NFR15 | manual | drive over a measured hump (ruler) 3 times | `hump DONE: peak N mm` within ±5 mm of the ruler |
| S4-M03 | IMU unplugged | degraded mode | manual | unplug the IMU while running | `IMU LOST`, robot keeps line-following, telemetry `degraded:true` |
| S4-M04 | watchdog (OFF since 2026-11-06: broke the us timer, see s4_config.h) | NFR20 | manual | normal image (not TEST); temporarily stop motion_task (debug build) | board resets within 100 ms; `WATCHDOG_RESET` event after reconnect |

*Method:* **auto** = `TEST_CHECK` in code, result printed on the console;
**manual** = a person observes the result (LED, scope, meter, phone app…).

## Results

Add a new row for every run. Never edit or delete old rows.

| Date | Test ID | Commit | Tester | Result | Log / evidence | Notes |
|---|---|---|---|---|---|---|
| 2026-11-10 | S4-U01 … S4-U10 | n/a (Temp_Test copy, no git) | 2502347 | PASS (10/10) | serial output read live (not saved); full re-run with saved log planned 2026-11-11 | `make NET=lwip MQTT=1 TEST=1`, USB only. IMU is a GY-511 (LSM303DLHC, no gyro): driver added, accelerometer-only pitch; hump measured with the wheel-probe method |
| 2026-11-10 | S4-M01 | n/a (Temp_Test copy, no git) | 2502347 | PASS | `logs/2026-11-10_s4_m01_impact_estop_dashboard.jpg`, `logs/2026-11-10_s4_tilt_estop_dashboard.jpg` | real impact while RUNNING (4.31 g): wheels stopped, FAULT "impact or tilt detected by the IMU", fault bit 5; RESET → READY. Planned tap test on blocks still to video |
