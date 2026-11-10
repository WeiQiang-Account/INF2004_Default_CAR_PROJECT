# System — test log

Tests of the complete product against its requirements, run on the final hardware set-up. Run the full set before every demo and before hand-in.

See `testing/README.md` for how to run tests and record results.

## Test cases

Define a test here before running it. Test IDs never change.

| ID | What is tested | Requirement / interface | Method | Steps / how | Expected result |
|---|---|---|---|---|---|
| SYS-01 | full mission on the track: line, barcodes, hump, obstacle, finish | all FR | manual | 1. Normal build, power bank only. 2. Calibrate IR on the track. 3. START. 4. Video robot + dashboard to the end. | MISSION COMPLETE, no FAULT; time and hump peak recorded |
| SYS-02 | live telemetry during the run | 6.5, IF-18 | manual | dashboard open during SYS-01 | ~5 telemetry msg/s, state and events match the robot |
| SYS-03 | safety stop and recovery while driving | NFR9, NFR14 | manual | 1. Robot RUNNING. 2. Firm tap on the chassis. 3. RESET, START. | wheels stop at once (FAULT), RESET → READY, START drives again |

*Method:* **auto** = `TEST_CHECK` in code, result printed on the console;
**manual** = a person observes the result (LED, scope, meter, phone app…).

## Results

Add a new row for every run. Never edit or delete old rows.

| Date | Test ID | Commit | Tester | Result | Log / evidence | Notes |
|---|---|---|---|---|---|---|
