# Subsystem 1 — test log

**Subsystem:** Wi-Fi telemetry + remote commands (MQTT)  ·  **Student ID:** 2502287  ·  **Code:** `app_program/subsystem1/`

See `testing/README.md` for how to run tests and record results.

## Test cases

Define a test here before running it. Test IDs never change.

| ID | What is tested | Requirement / interface | Method | Steps / how | Expected result |
|---|---|---|---|---|---|
| S1-U01 | telemetry JSON flat, design fields, longest case < 700 B (publish buffer 768 B) | 6.5 telemetry | auto + host | `subsystem1_test.c`; PC: `make -C testing/subsystem1/host_tests` | `S1-U01 PASS` |
| S1-U02 | command parser: good / unknown / missing seq / > 64 B | 6.5 cmd validation | auto + host | as above | `S1-U02 PASS` |
| S1-U03 | state rules: start READY, resume/cancel PAUSED, reset FAULT + still, stop any | 6.5, VC statechart | auto + host | as above | `S1-U03 PASS` |
| S1-U04 | each change → one event; WATCHDOG_RESET first | 6.5 events, NFR20 | auto + host | as above | `S1-U04 PASS` |
| S1-U05 | too-small buffer → 0 bytes, never a cut JSON | robustness | auto + host | as above | `S1-U05 PASS` |
| S1-U06 | remote command flag created | IF-15 | auto | `subsystem1_test.c` | `S1-U06 PASS` |
| S1-U07 | replayed seq and unknown cmd rejected, never forwarded | IF-15 | auto | `subsystem1_test.c` | `S1-U07 PASS` |
| S1-U08 | live snapshot → telemetry JSON | IF-18 | auto | `subsystem1_test.c` (prints the JSON) | `S1-U08 PASS` |
| S1-I01 | telemetry every 200 ms, heartbeat every 1 s | 6.5 | manual | `make NET=lwip MQTT=1`; power bank only; MQTT Explorer on the laptop's broker IP | `se14/robot/telemetry` ~5 msg/s, `heartbeat` 1 msg/s with rssi |
| S1-I02 | events on change | 6.5 | manual | press START / STOP on the robot | `se14/robot/event` gets `state` READY→RUNNING→PAUSED |
| S1-I03 | remote start / stop | IF-15 | manual | publish `{"cmd":"start","seq":1}` then `{"cmd":"stop","seq":2}` to `se14/robot/cmd` (wheels lifted) | `cmd_accepted` events; robot starts, then pauses |
| S1-I04 | replay / wrong state rejected | IF-15 | manual | publish `{"cmd":"stop","seq":2}` again; `{"cmd":"resume","seq":3}` in READY | `cmd_rejected` with `old_seq` / `not_allowed_now`; robot does not move |
| S1-I05 | link loss | FAULT_BIT_WIFI | manual | turn the hotspot off for 10 s, then on | robot keeps running; after reconnect `fault` bit 4 event, then cleared; heartbeat `drops` +1 |

*Method:* **auto** = `TEST_CHECK` in code, result printed on the console;
**manual** = a person observes the result (LED, scope, meter, phone app…).

## Results

Add a new row for every run. Never edit or delete old rows.

| Date | Test ID | Commit | Tester | Result | Log / evidence | Notes |
|---|---|---|---|---|---|---|
| 2026-11-10 | S1-U01 … S1-U08 | n/a (Temp_Test copy, no git) | 2502287 | PASS (8/8) | serial output read live (not saved); full re-run with saved log planned 2026-11-11 | `make NET=lwip MQTT=1 TEST=1`, USB only. S1-U01 limit raised 480 → 700 B: telemetry gained m_cause, ee_l/ee_r, es, es_cg, es_tilt, a_pk, hdg (MQTT buffer 768 B, lwIP ring buffer 1536 B) |
| 2026-11-10 | S1-I01, S1-I02, S1-I03 | n/a (Temp_Test copy, no git) | 2502287 | PASS | `logs/2026-11-10_s1_i01_telemetry_live_dashboard.jpg` (dashboard v12, from screen recording 2026-11-10 22:07) | telemetry ~5 msg/s shown on the dashboard; state events READY→RUNNING→PAUSED/FAULT appear in the event log; remote START / STOP / RESET accepted (`start accepted`) |
| 2026-11-10 | S1-I04 | n/a (Temp_Test copy, no git) | 2502287 | PASS | `logs/2026-11-10_s1_i04_resume_rejected.jpg`, `logs/2026-11-10_s1_i04_start_rejected.jpg` | START in the wrong state also rejected (`start rejected: not allowed in this state`); RESUME sent in FAULT → `cmd_rejected` `not_allowed_now`, robot did not move |
