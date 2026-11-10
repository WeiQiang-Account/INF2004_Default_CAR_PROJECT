# Integration changes (for team review before pushing)

All five subsystems now build together and talk through the interfaces in
`TEAM.md` section 5. Build: `make -j8` (USB), `make NET=lwip MQTT=1 -j8`
(Wi-Fi telemetry), and `TEST=1` for the self-tests. Every variant
compiled with no warnings. All PC host tests pass (S1 5/5, S2 49/49,
S3 9/9, S4 7/7, S5 profile).

## Subsystem 1: Wi-Fi telemetry + commands (new)
- The template's LED heartbeat is removed (`task_led.c` deleted).
- `s1_json.c/.h`: pure JSON builders (telemetry, heartbeat, events, command
  result), the command parser (≤ 64 bytes, `{"cmd":..,"seq":N}`) and the
  state rules (start in READY, resume/cancel in PAUSED, reset in FAULT with
  the robot still, stop at any time).
- `subsystem1.c`: builds a snapshot from the other subsystems' getters,
  checks commands (seq must increase), and overrides the VC hook
  `s5_hook_remote_poll()`. Uses event flag `flg_remote_cmd`. A command is
  dropped if the VC state changed between the check and the poll.
- `s1_mqtt.c` (MQTT build only; runs on the Wi-Fi task): telemetry every
  200 ms, heartbeat every 1 s (with RSSI), events at QoS 1, subscribes
  `se14/robot/cmd`.
- Tests: `subsystem1_test.c` (S1-U01..U08) and `testing/subsystem1/host_tests`.

## Subsystem 2: Motion
- `S2_TELEMETRY_TEST`, `S2_TELEMETRY_DRIVE` and `S2_KEEP_TEMPLATE_EXAMPLE`
  are set to 0.
- The S2 quick test now hooks in through `s2_telemetry_test_on_connected()`
  and topic `se14/robot/s2test`, so it never clashes with S1.
- `FAULT_BIT_IMPACT` (bit 5) added.
- Host test updated to the measured CPR 2535 / 63 mm wheel.

## Subsystem 3: Line + barcode
- Template consumer turned off.
- Barcode priority 17 → 18 (below the VC).
- New getter: `s3_barcode_count()`.
- New PC host test runner: `testing/subsystem3/host_tests`.

## Subsystem 4: IMU (rewritten for micro T-Kernel)
- MPU-6050 driver on I2C0 (GP4/GP5), polled with a 2 ms timeout.
- Calibration (FR16).
- Impact > 2.5 g and tilt > 30° → E-stop (NFR14).
- Hump height from pitch × encoder distance (FR17, NFR15).
- Motion class and yaw rate (FR18, FR19).
- Hardware watchdog of 100 ms, fed only while Motion runs (NFR20). It is off
  in TEST builds and in the S2 motor demo.
- IMU loss puts the robot in degraded mode; it is not an E-stop.
- Tasks:
  - `imu_safety`: priority 10, core 1
  - `imu_terrain`: priority 19
- The template monitor is removed (`task_monitor.c` deleted).

## Subsystem 5: VC + ultrasonic
- VC priority 20 → 17 (Week 6 order).
- Scan stats now carry the left/right clearances.

## Shared files
- `lwipopts.h`:
  - MQTT output buffer 1024 (the default 256 is too small for the telemetry JSON)
  - 8 requests in flight
  - 2 more sys_timeout slots
- `wifi_secrets.example.h`: client id `se14-robot`, debug topic
  `se14/robot/debug`. Update your own `wifi_secrets.h` the same way.
- `TEAM.md`: members, pins, peripherals, priorities, interfaces IF-13..19
  and the object budget.
- `app_main.c` header comments.

## Not yet tested on the robot
- MPU-6050 driver, the axis/sign settings in `s4_config.h`, and the watchdog.
- MQTT commands, events and the RSSI read.
- Nobody posts `VC_EV_FINISH` yet (A14, the finish marker, is still open).
- Never build with `CRASH_REPORT=1`: GP0/GP1 are the ultrasonic pins.
