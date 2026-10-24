# Subsystem 2 — test log

**Subsystem:** Motion Control (motors, encoders, PID) · **Student ID:** 2501989 · **Code:** `app\_program/subsystem2/`

See `testing/README.md` for how to run tests and record results.
PC tests (no robot): `make -C testing/subsystem2/host\_tests`.

## Test cases

Define a test here before running it. Test IDs never change.

|ID|What is tested|Requirement / interface|Method|Steps / how|Expected result|
|-|-|-|-|-|-|
|S2-U01|motion kernel objects exist|IF: mbf\_motion\_cmd, flg\_move\_done, flg\_estop, flg\_fault|auto|`subsystem2\_test.c`|`\[TEST] S2-U01 PASS`|
|S2-U02|4-byte command packs, unpacks and clamps|minimum-bit change|auto|`subsystem2\_test.c`|`\[TEST] S2-U02 PASS`|
|S2-U03|8-byte telemetry status packs and unpacks|minimum-bit change, IF to telemetry|auto|`subsystem2\_test.c`|`\[TEST] S2-U03 PASS`|
|S2-U04|motor PWM slices 4/5 at 20 kHz (TOP 6249, div 1)|FR6|auto|`subsystem2\_test.c`|`\[TEST] S2-U04 PASS`|
|S2-U05|motion loop runs every 20 ms (48–52 cycles/s)|NFR5|auto|`subsystem2\_test.c`|`\[TEST] S2-U05 PASS`|
|S2-U06|motion task sleeps between cycles|NFR4 (no CPU hogging)|auto|`subsystem2\_test.c`|`\[TEST] S2-U06 PASS`|
|S2-U07|E-stop → FAULT and PWM 0 within 25 ms|NFR9, FR12, flg\_estop|auto|`subsystem2\_test.c`|`\[TEST] S2-U07 PASS`|
|S2-U08|reset refused while E-stop set, accepted after|statechart 6.1|auto|`subsystem2\_test.c`|`\[TEST] S2-U08 PASS`|
|S2-M01|encoder counts per wheel turn (CPR) and direction|FR7, A3|manual|set `S2\_MANUAL\_TESTS 1`, `make TEST=1`; turn each wheel 10 times by hand|both counts go up; counts/10 ≈ 937 (record real value)|
|S2-M02|1 m distance move|FR9, NFR7|manual|S2-M02 runs automatically after S2-M01; tape measure|950–1050 mm|
|S2-M03|90° right turn|FR9, NFR8|manual|runs after S2-M02; protractor / floor tape|85–95°|
|S2-M04|90° left turn|FR9, NFR8|manual|runs after S2-M03|85–95°|
|S2-M05|speed hold at 300 mm/s|FR8, NFR6|manual|Vehicle Controller or test code calls `motion\_drive(300, FALSE)`; read status|both wheels 270–330 mm/s|
|S2-M06|20 ms timing on a logic analyser|NFR5, A2|manual|probe GP15 (high during each cycle) with telemetry on and off|period 20 ms, jitter < 2 ms|
|S2-M07|stall detection|statechart 6.1|manual|hold a wheel while driving|FAULT (cause 2) after \~300 ms, motors off|

*Method:* **auto** = `TEST\_CHECK` in code, result printed on the console;
**manual** = a person observes the result (LED, scope, meter, phone app…).

## Results

Add a new row for every run. Never edit or delete old rows.

|Date|Test ID|Commit|Tester|Result|Log / evidence|Notes|
|-|-|-|-|-|-|-|
|2026-10-23|S2-U01, U02, U03, U05, U07, U08|n/a (no git yet)|Donghan|PASS|`logs/2026-10-23\_unit\_fix1.png`|first on-target run (`make TEST=1`), Pico off the board on USB; right motor disabled (MOTION\_RIGHT\_MOTOR\_OK 0)|
|2026-10-23|S2-U04|n/a (no git yet)|Donghan|FAIL|`logs/2026-10-23\_unit\_fix1.png`|PWM registers read back wrong: the kernel never takes the PWM block out of reset (USE\_PTMR 0 in config.h), so motors could not turn. Fixed in `drv\_motor.c` (motor\_init releases PWM reset)|
|2026-10-23|S2-U06|n/a (no git yet)|Donghan|FAIL|`logs/2026-10-23\_unit\_fix1.png`|motion task not waiting between cycles|
|2026-10-23|S2-U04|n/a (no git yet)|Donghan|PASS|`logs/2026-10-23\_unit\_fix2.png`|after the PWM reset fix: TOP4=TOP5=6249, DIV4=0x10, CSR4=CSR5=0x1|
|2026-10-23|S2-U06|n/a (no git yet)|Donghan|FAIL|`logs/2026-10-23\_unit\_fix2.png`|info line: wait=0 run=0 ready=10 - task always READY but not running: core 0 is busy with USB + WiFi polling (pri 3/4). Fixed by moving motion\_task and the GPIO IRQ to core 1 (TP\_PRC2)|
|2026-10-24|S2-U01 … S2-U08|n/a (no git yet)|Donghan|PASS (8/8)|`logs/2026-10-24\_unit.png`|after both fixes; U05 51 cycles/s, U06 wait=10 run=0 ready=0; Pico off the board on USB; right motor disabled|
|2026-10-24|S2-M01|n/a (no git yet)|Donghan|PASS|`logs/2026-10-24\_cpr\_run1\_start.png`, `logs/2026-10-24\_cpr\_run1\_end.png` (enc\_l 2505 → 27853)|CPR = 2535 (left wheel, 10 turns by hand); config updated from 937|
|2026-10-24|S2-M01|n/a (no git yet)|Donghan|PASS|run 1: `logs/2026-10-24\_cpr\_run1\_*.png` (2505→27853); run 2: values read in MQTT Explorer (2757→28112), no screenshot|CPR = 2535 counts/wheel turn (avg of 2 runs, 10 turns each, left wheel); config updated from assumed 937 (A3)|
|2026-10-24|S2-M05|n/a (no git yet)|Donghan|PASS|`logs/2026-10-24\_mqtt\_speed\_stall.png`, `logs/2026-10-24\_mqtt\_telemetry\_driving.png`, link: `logs/2026-10-24\_wifi\_mqtt\_connected.png`|tested at ±200 mm/s (telemetry drive pattern) instead of 300; left wheel only, wheels lifted, power bank, data over WiFi/MQTT. v\_l = 202, 199, -199 mm/s (within ±1 %, NFR6 allows ±10 %). Right wheel not tested (motor faulty)|
|2026-10-24|S2-M07|n/a (no git yet)|Donghan|PASS|`logs/2026-10-24\_mqtt\_speed\_stall.png`|left wheel held by hand while driving: telemetry state FAULT, cause 2 (stall), v\_l 0 (motor off)|
