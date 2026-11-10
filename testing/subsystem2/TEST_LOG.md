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
|S2-M01|encoder counts per wheel turn (CPR) and direction|FR7, A3|manual|set `S2\_MANUAL\_TESTS 1`, `make TEST=1`; turn each wheel 10 times by hand|both counts go up; counts/10 ≈ 2535 (measured 2026-10-24, was assumed 937)|
|S2-M02|1 m distance move|FR9, NFR7|manual|`S2\_MANUAL\_TESTS 1`, `S2\_MANUAL\_TURNS 0`, `make NET=lwip MQTT=1 TEST=1`; robot on the floor, back of the wheels on a start mark; tape measure|950–1050 mm|
|S2-M03|90° right turn|FR9, NFR8|manual|runs after S2-M02; protractor / floor tape|85–95°|
|S2-M04|90° left turn|FR9, NFR8|manual|runs after S2-M03|85–95°|
|S2-M05|speed hold at 300 mm/s|FR8, NFR6|manual|`S2\_MANUAL\_TESTS 1`, `S2\_MANUAL\_SPEED 1`, test build; wheels lifted; console prints average / min / max per wheel|both wheels 270–330 mm/s|
|S2-M06|20 ms timing (motion task times itself with the 1 µs timer; a logic analyser on GP15 is optional)|NFR5, A2|auto|`subsystem2\_test.c`, test build with Wi-Fi + MQTT running|`\[TEST] S2-M06 PASS`: ≥ 140 periods, every period 18–22 ms|
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
|2026-11-10|S2-U01 … S2-U08, S2-M06|n/a (Temp\_Test copy, no git)|2501989|PASS (9/9)|serial output read live (not saved); full re-run with saved log planned 2026-11-11|both motors working (right motor enabled); `make NET=lwip MQTT=1 TEST=1`, USB only. S2-M06 now self-timed by the motion task (a busy-wait test froze core 1); S2-U06 uses the measured busy time|
|2026-11-08|S2-M01|n/a (Temp\_Test copy, no git)|2501989|PASS|dashboard screenshots sent 2026-11-08 (start / end, each wheel)|right wheel then left wheel, 10 turns by hand each: both counts go up, CPR 2535 confirmed|
|2026-11-09|S2-M02|n/a (Temp\_Test copy, no git)|2501989|PASS|`logs/2026-11-10\_s2\_m02\_1m\_distance\_dashboard.jpg` (dashboard: distance 1.02 m) + tape-measure photos on the phone|1 m move ended within ±10 mm of the 1 m mark (wheel diameter set to 66.8 mm effective). Drift to the side 0–20 mm over 1 m varies run to run (floor / wheel slip)|
|2026-11-10|S2-M03|n/a (Temp\_Test copy, no git)|2501989|PASS|`logs/2026-11-10\_s2\_m03\_turn\_dashboard.jpg` + gap A / gap B photos on the phone|90° right turn after 600 mm; first runs ~87° → `TURN\_SCALE` tuned to 1.075; then within 85–95°|
|2026-11-10|S2-M04|n/a (Temp\_Test copy, no git)|2501989|PASS|phone photos 2026-11-10|90° left turn: 93.8°, 90°, 90° (3 runs)|
|2026-11-10|S2-M05|n/a (Temp\_Test copy, no git)|2501989|PASS|phone video of the console 2026-11-10|300 mm/s target: left avg 298, right avg 298 mm/s (NFR6 ±10 %: 270–330)|
|2026-11-10|S2-M07|n/a (Temp\_Test copy, no git)|2501989|PASS|`logs/2026-11-10\_s2\_m07\_stall\_fault\_dashboard.jpg` (FAULT "a wheel stalled", Wheel stalled lit)|each wheel held by hand while driving: FAULT cause stall, motors off|
|2026-11-10|unplanned: encoder health while driving|n/a (Temp\_Test copy, no git)|2501989|FAIL|`logs/2026-11-10\_s2\_encoder\_fault\_false\_trip.jpg`|while line-following, FAULT "wheel encoder error" after 0.19 m: left encoder noise bursts (errors 56/7), wires picked up motor noise. Fix: fault only after 5 noisy 20 ms cycles in a row (`ENC\_BAD\_CYCLES`); re-test 2026-11-11|
|2026-11-10|S2-M01|n/a (Temp\_Test copy, no git)|2501989|PASS|`logs/2026-11-10\_s2\_m01\_cpr\_right\_start.png`, `logs/2026-11-10\_s2\_m01\_cpr\_right\_end.png`, `logs/2026-11-10\_s2\_m01\_cpr\_left\_start.png`, `logs/2026-11-10\_s2\_m01\_cpr\_left\_end.png`|normal build, dashboard over MQTT, wheels lifted, power bank only, 10 turns forward by hand. Right: 622 → 25972 = 2535.0 counts/turn; left: 605 → 25932 = 2532.7 counts/turn (−0.1 % vs CPR 2535). Both counts went up going forward. Encoder errors L/R 7/3 (recovered skips)|
|2026-11-11|S2-M02|n/a (Temp\_Test copy, no git)|2501989|PASS (3/3)|`logs/2026-11-11\_s2\_m02\_dashboard\_run{1,2,3}\_{start,end}.png`, `logs/2026-11-11\_s2\_m02\_tape\_run1\_end.jpg`, `logs/2026-11-11\_s2\_m02\_tape\_run{2,3}\_{start,end}.jpg`|test build (`S2\_MANUAL\_TESTS 1`, speed + turns off), power bank only, 1000 mm at 150 mm/s on floor tiles, back of tyre from a tile joint. Tape: run 1 ≈ 975 mm, run 2 ≈ 988 mm, run 3 ≈ 975 mm (all within 950–1050). Encoders: 997 / 997 / 996 mm. Tape read from photos (±5 mm, parallax). Left encoder errors 144 / 66 / 65 per run (skips recovered) - check left encoder wiring|
|2026-11-11|S2-M07|n/a (Temp\_Test copy, no git)|2501989|PASS (left + right)|`logs/2026-11-11\_s2\_m07\_stall\_left.jpg`, `logs/2026-11-11\_s2\_m07\_stall\_right.jpg` (stills from phone videos 20261111\_004207.mp4 = left, 20261111\_004254.mp4 = right; YouTube link: TODO)|test build speed test (300 mm/s), robot propped up level, power bank only. Each wheel gripped by hand while driving: Motion control "FAULT: stall", "Wheel stalled" lit red, both wheels 0 mm/s. Held wheel shows the lower count (left run: L 11253 / R 14809; right run: L 12445 / R 9030)|
|2026-11-11|S2-M03|n/a (Temp\_Test copy, no git)|2501989|PASS (2 runs)|`logs/2026-11-11\_s2\_m03\_run{1,2}\_{before,after}\_{A\_front,B\_back}.jpg`|test build (`S2\_MANUAL\_TURNS 1`, right turn only), power bank, floor tiles. Gaps from the right side plate (front corner A, back corner B) to the tile joint before and after the turn; turn = 90° − atan(Δ(A−B)/L), L ≈ 150–200 mm. Run 1: A−B −13 → −4 mm ⇒ ≈ 87°; run 2: −7 → +2.5 mm ⇒ ≈ 87°. Both within 85–95°; turns about 3° short (`TURN\_SCALE` 1.075 shared with left turns)|
|2026-11-11|S2-M04|n/a (Temp\_Test copy, no git)|2501989|PASS (2 runs)|`logs/2026-11-11\_s2\_m04\_run{1,2}\_{before,after}\_{A\_front,B\_back}.jpg`|test build (`S2\_MANUAL\_TURNS 1`, left turn only), power bank, floor tiles; same gap method as S2-M03, left turn = 90° + atan(Δ(A−B)/L), L ≈ 150–200 mm. Run 1: A−B −8 → 0 mm ⇒ ≈ 92.5°; run 2: −3 → −4.5 mm ⇒ ≈ 89.5°. Both within 85–95° (gaps read from photos, ±2 mm)|
