# Subsystem 4 — test log

**Subsystem:** IMU Safety & Terrain Subsystem (Buddy 4)  ·  **Student ID:** <student ID>  ·  **Code:** `app_program/subsystem4/`

See `testing/README.md` for how to run tests and record results.

## Test cases

Define a test here before running it. Test IDs never change.

| ID | What is tested | Requirement / interface | Method | Steps / how | Expected result |
|---|---|---|---|---|---|
| S4-U01 | LSM303 Register Communication & Initialization | I2C Read/Write (`0x19`) | auto | `test_subsystem4.c` (`test_lsm303_init`) | `PASS` (`g_subsystem_states.safety_state == 1`) |
| S4-U02 | Over-Tilt & High-Impact E-Stop Flag Activation | Safety Watchdog | auto | `test_subsystem4.c` (`test_estop_trigger`) | `PASS` (`g_flg_estop == true`, bitmask updated) |
| S4-U03 | Consecutive I2C Read Failure & Fault Bit Masking | Hardware Watchdog | auto | `test_subsystem4.c` (`test_i2c_loss_fault`) | `PASS` (`FAULT_BIT_IMU_LOST` set on 3 failures) |
| S4-U04 | Hump Detection & Low-Pass Filter Pitch Estimation | Terrain Classifier | auto | `test_subsystem4.c` (`test_hump_traversal`) | `PASS` (`EVENT_HUMP_DONE` event generated with valid peak height) |
| S4-U05 | 16-Bit Bit-Packed Inter-Task Queue Payload Alignment | Inter-Subsystem Queue | auto | `test_subsystem4.c` (`test_bitpacked_payload`) | `PASS` (`sizeof(imu_packed_event_t) == 2`) |

*Method:* **auto** = `TEST_CHECK` in code, result printed on the console;
**manual** = a person observes the result (LED, scope, meter, phone app…).

## Results

Add a new row for every run. Never edit or delete old rows.

| Date | Test ID | Commit | Tester | Result | Log / evidence | Notes |
|---|---|---|---|---|---|---|
| 2026-11-05 | S4-U01 | `main_lsm303` | Buddy 4 | PASS | USB Serial Output | Validated hardware initialization on GP4/GP5 |
| 2026-11-05 | S4-U02 | `main_lsm303` | Buddy 4 | PASS | Host/Target Unit Test | Emergency Stop flag set correctly when >2.5g |
| 2026-11-05 | S4-U03 | `main_lsm303` | Buddy 4 | PASS | Host/Target Unit Test | Bitwise fault bitmask sets `FAULT_BIT_IMU_LOST` |
| 2026-11-05 | S4-U04 | `main_lsm303` | Buddy 4 | PASS | Host/Target Unit Test | Traversal state machine transitions FLAT->CLIMB->DESCENT->FLAT |
| 2026-11-05 | S4-U05 | `main_lsm303` | Buddy 4 | PASS | Host/Target Unit Test | Struct packing confirmed at exactly 16 bits (2 bytes) |