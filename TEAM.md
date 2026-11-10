# Team and subsystems

The template's example (LED / producer / consumer / monitor) has been
replaced by the robot code; the rows below describe the robot.

Fill this in at the start of the project and keep it up to date. It is the
single place that says **who owns what**, so that five people can work in
one firmware image without clashing over pins, priorities, or kernel objects.

A **subsystem** is one student's part of the product, with its own folder.
It may contain several tasks and own several peripherals (see section 3).

Rule: **only the owner edits a subsystem's folders.** To change another
subsystem, ask its owner or open a pull request for them to review.

## 1. Members and subsystems

| # | Subsystem (what it does) | Student ID | GitHub username | Code | Tests |
|---|---|---|---|---|---|
| 1 | Wi-Fi telemetry + remote commands over MQTT (Buddy 1) | 2502287 | | `app_program/subsystem1/` | `testing/subsystem1/` |
| 2 | Motion Control: motors, encoders, PID, E-stop (Buddy 2) | 2501989 | | `app_program/subsystem2/` | `testing/subsystem2/` |
| 3 | Line following + barcode (Buddy 3) | 2500142 | | `app_program/subsystem3/` | `testing/subsystem3/` |
| 4 | IMU: safety (impact / tilt / E-stop), hump height, motion class, watchdog (Buddy 4) | 2502347 | | `app_program/subsystem4/` | `testing/subsystem4/` |
| 5 | Vehicle Controller + ultrasonic obstacle scan (Buddy 5) | 2401151 | | `app_program/subsystem5/` | `testing/subsystem5/` |

Shared files (`app_program/app_main.c`, `app_program/app.h`, this file)
are changed only by agreement of the team. The exception is claiming an empty
row in this file (a pin, a peripheral, an interface ID), which anyone may do
(section 2).

## 2. Pins

One owner per pin. Claim a pin here **before** you wire or code it.

**Why:** two subsystems using the same pin is one of the most common and
hardest-to-find mistakes in a team project. The code builds without any
error, but the hardware misbehaves. Each subsystem can pass its own tests
and then fail once they are combined, and two outputs driving the same pin
against each other can damage it. Writing the claim down first means a clash
is spotted when someone edits this table, not hours later on the bench.

**No instructor check needed.** The team keeps this table itself. You do not
need the instructor, or the rest of the team, to check or approve a claim:
if the row is empty, fill it in and commit. If two people claim the same
pin, they agree between themselves who moves. Only the *reserved* rows are
fixed.

| Pin | Owner | Used for |
|---|---|---|
| GP23, GP24, GP25, GP29 | *reserved* | Pico W radio |
| GP0 | S5 | ultrasonic TRIG (so **never build with `CRASH_REPORT=1`**) |
| GP1 | S5 | ultrasonic ECHO via 1k/2k divider (edge IRQ branch) |
| GP2 | S2 | right encoder A (Grove 2) |
| GP3 | S2 | right encoder B (Grove 2) |
| GP4 | S4 | I2C0 SDA (IMU: GY-511 LSM303 fitted; MPU-6050 also supported) |
| GP5 | S4 | I2C0 SCL (IMU) |
| GP6 | | |
| GP7 | S3 | right IR digital out: barcode edges (edge IRQ branch) |
| GP8 | S2 | M1A left motor (PWM slice 4A) |
| GP9 | S2 | M1B left motor (PWM slice 4B) |
| GP10 | S2 | M2A right motor (PWM slice 5A) |
| GP11 | S2 | M2B right motor (PWM slice 5B) |
| GP12 | S5 | ultrasonic servo (PWM slice 6A, 50 Hz) |
| GP13 | | |
| GP14 | | |
| GP15 | S2 | motion timing pin, OFF by default (`MOTION_DEBUG_PIN`); servo header, keep the servo off it |
| GP16 | S2 | left encoder A (Grove 4) |
| GP17 | S2 | left encoder B (Grove 4) |
| GP18 | | |
| GP19 | | (was the template LED, now free) |
| GP20 | S5 | START button (VC) |
| GP21 | S5 | STOP button (VC) |
| GP22 | S5 | buzzer (PWM slice 3A) |
| GP26 | S3 | IR left (ADC0) |
| GP27 | S3 | IR centre (ADC1) |
| GP28 | S3 | IR right (ADC2, VBAT-sense jumper off) |

## 3. Peripherals

**Subsystems, tasks and peripherals are three different things:**

| Term | What it is | Example |
|---|---|---|
| **Subsystem** | one student's part of the product, one folder | "environment sensing", "motor control", "user interface" |
| **Task** | one thread of execution. A subsystem may have **as many tasks as it needs** | a sampling task and a filtering task, both in "environment sensing" |
| **Peripheral** | one hardware block on the RP2040 | I2C0, UART1, the ADC |

A driver is **not** a subsystem. It is code *inside* the subsystem that
needs that hardware: the "environment sensing" subsystem contains the I2C
sensor code, and owns I2C0. Write it as `drv_<name>.c` / `drv_<name>.h` in
that subsystem's folder, never under `device/`, and call it only from the
peripheral's owning task. The kernel's own drivers in `device/` (serial,
I2C, ADC) may be used instead; switching one on in `config/config_device.h`
changes shared configuration, so record it in the table below first.

**The rule: each peripheral is driven by exactly one task, its *owning
task*.** That task can be in any subsystem, and one subsystem may own
several peripherals (one owning task each, or one task owning all of them).
Every other task, in the same subsystem or another one, asks the owning task
through a kernel object (section 5) and never touches that peripheral itself.

*Why:* this port runs tasks on both cores at the same time. If two tasks
drive the same I2C or UART block at once, their transfers get mixed up, and
the drivers have no locking to stop it.

| Peripheral | Owner subsystem | Owning task | Pins |
|---|---|---|---|
| UART1 | | | |
| I2C0 | S4 | `imu_safety` (only task that touches it) | GP4, GP5 |
| I2C1 | | | |
| SPI0 | | | |
| SPI1 | | | |
| ADC | S3 | `line_follow_task` | GP26–GP28 |
| PWM slices (list) | S2 | `motion_task` | slices 4, 5 (GP8–GP11) |
| PWM slices 3, 6 | S5 | `vc_task` (buzzer), `scan_task` (servo) | GP22, GP12 |
| Watchdog (WATCHDOG, PSM WDSEL) | S4 | `imu_safety` feeds it every 20 ms while Motion runs (NFR20) | – |
| GPIO edge IRQ (IO_BANK0, IRQ 13) | S2 (shared dispatcher `drv_gpio_irq.c`) | IRQ on core 1; branches: S2 encoders, S3 GP7 (S5 ECHO GP1 is polled since 2026-11-06) | GP2, 3, 7, 16, 17 |
| WiFi radio, TCP/IP, MQTT | *kernel* | `cyw43_utk_task` (built in) | GP23–25, GP29 |

### WiFi and MQTT are special

The radio and the network stack belong to the kernel's built-in WiFi service
task, `cyw43_utk_task` (`lib/libwifi/sysdepend/pico_rp2040/`). **No student
task may call `cyw43_...`, lwIP or MQTT functions, not even from a
"WiFi subsystem".** The calls are not safe from any other task, on either core.

What students *can* do:

- **Read the connection state** from any task, which is always safe:
  ```c
  #include "cyw43_utk.h"
  T_CYW43_UTK_STATUS st;
  cyw43_utk_get_status(&st);   /* st.ready, st.link_status, st.ip_addr, ... */
  ```
- **Send or receive data over the network** by adding code that *runs
  inside the service task*. The MQTT client is in
  `lib/liblwip/sysdepend/pico_rp2040/mqtt_utk.c`. It publishes a heartbeat
  today. A "cloud / reporting" subsystem would add a kernel object
  (e.g. a message buffer) that application tasks fill. The MQTT code then
  reads it with `TMO_POL` (never blocking) and publishes it.
  Because this changes shared library code, **agree it with the team and
  the instructor first**, and record it as an interface in section 5.
  It needs the `NET=lwip MQTT=1` build (README section 3).

## 4. Task priorities

1 is the highest. 1–4 are used by the kernel (USB console 3, WiFi 4). Each subsystem gets its own band by default;
change it only by team agreement. Write down every task you create.

| Subsystem | Priority band | Tasks (name – priority – core) |
|---|---|---|
| 1 | (Wi-Fi task, 4) | no task of its own: publishing runs inside `cyw43_utk_task` (kernel, 4, core 0) via `mqtt_utk_on_connected()`; the remote hook runs on `vc_task` |
| 2 | 14–15 | motion_task – 14 – core 1 (TP_PRC2; core 0 is busy with USB + WiFi polling); *test only:* s2_drive – 15 – core 1 (MQTT build, `S2_TELEMETRY_DRIVE`) |
| 3 | 16, 18 | line_follow_task – 16 – any; barcode_task – 18 – any |
| 4 | 10, 19 | imu_safety – **10** – core 1 (above Motion: impact E-stop, NFR14); imu_terrain – 19 – any |
| 5 | 17, 21 | vc_task – 17 – any; scan_task – 21 – any |

Week 6 order, highest first: IMU Safety (10) > Motion (14) > Line Follow (16)
> Vehicle Controller (17) > Barcode (18) > IMU Terrain (19) > Ultrasonic
Scan (21) > telemetry (inside the Wi-Fi task, which only sleeps/polls).

Core: `TP_PRC1` = core 0, `TP_PRC2` = core 1, or "any".

## 5. Interfaces between subsystems

Every arrow between two subsystems is listed here. The **provider** creates
the kernel object and publishes its ID in its own `subsystemN.h`; the **user** only
sends to it or waits on it. Agree on the data format before coding.

| ID | From → To | Mechanism | Object ID name | Data / meaning | Agreed by |
|---|---|---|---|---|---|
| IF-01 | S2 → S3 | mailbox (owned by S3) | `mbx_samples` | `SAMPLE_MSG` in `subsystem2.h` | removed (template example off) |
| IF-02 | S3 → S2 | semaphore + fixed memory pool (owned by S2) | `sem_slots`, `mpf_samples` | receiver returns the block, then 1 slot | removed (template example off) |
| IF-03 | S3 → S4 | message buffer + event flag (owned by S4) | `mbf_summary`, `flg_events` | text line ≤ `S4_SUMMARY_MAX` bytes, then `FLG_BATCH` | removed (template example off) |
| IF-04 | S4 → S3 | shared data under a mutex (owned by S3) | `app_stats`, `mtx_stats` | `APP_STATS`, read only while locked | removed (template example off) |
| IF-05 | S4 → S1 | task control | `tid_led` | `tk_sus_tsk` / `tk_rsm_tsk` / `tk_chg_pri` | removed (template example off) |
| IF-06 | VC → S2 | message buffer (owned by S2) | `mbf_motion_cmd` | 4-byte packed command via `motion_drive/move/turn_*/stop/reset()` | proposed |
| IF-07 | S2 → VC | event flag (owned by S2) | `flg_move_done` | `MOVE_DONE_BIT` when a move/turn/stop ends | proposed |
| IF-08 | S4 → S2 | event flag (owned by S2) | `flg_estop` | IMU sets `ESTOP_BIT`; S4 clears it | proposed |
| IF-09 | S3 → S2 | shared 16-bit word (written by S3 only) | `g_steer_latest` | `steer_publish(-100..100)` every 5 ms; stale after 60 ms | proposed |
| IF-10 | all → all | event flag (created by S2) | `flg_fault` | one bit per fault (`FAULT_BIT_*` in `motion_types.h`) | proposed |
| IF-11 | S3, S5 → S2 | GPIO IRQ branch | `gpio_irq_register()` | GP7 (S3) and GP14 (S5) branches, < 2 µs each | proposed |
| IF-12 | S2 → S1, S3, S4 | read-only counters | `encoder_read()` | wheel counts, + = forward | proposed |
| IF-13 | template MQTT → S1 | function hook (weak) | `mqtt_utk_on_connected(client)` in `lib/liblwip/sysdepend/pico_rp2040/mqtt_utk.c` | defined by S1 (`s1_mqtt.c`); runs on the WiFi task each time the broker accepts the connection. S1 then calls S2's weak test hook `s2_telemetry_test_on_connected()` (topic `se14/robot/s2test`, only with `S2_TELEMETRY_TEST=1`) | proposed |
| IF-14 | all → VC | message buffer (owned by S5) | `mbf_vc_events` | 32-bit `VC_EV_MAKE(type, arg)` via `vc_post()` (`vc_events.h`): S3 barcode / line lost / found / junction, S4 HUMP_START, HUMP_DONE(peak mm), IMPACT, IMU_LOST, S5 obstacle / profile | proposed |
| IF-15 | S1 → VC | event flag (owned by S1) + hook | `flg_remote_cmd`, `s5_hook_remote_poll()` | validated MQTT command → 1 START / 2 STOP, tagged with the VC state it was checked in (dropped if the state changed) | proposed |
| IF-16 | S4 → VC | hook | `s5_hook_imu_ready()` | 1 once calibrated, or once the IMU is declared LOST (degraded mode) | proposed |
| IF-17 | S3 → VC | hook | `s5_hook_line_detected()` | 1 if any IR sensor sees the line | proposed |
| IF-18 | S2, S3, S4, S5 → S1 | read-only getters | `motion_get_status()`, `s3_*()`, `s4_*()`, `vc_status_byte()`, `g_s5_scan` | telemetry snapshot every 200 ms (`s1_snapshot_take()`) | proposed |
| IF-19 | S4 → S2 | read-only counter | `motion_cycle_count()` | the watchdog is fed only while the motion loop keeps running (NFR20) | proposed |

Each interface is tested by at least one integration test
(`testing/integration/TEST_LOG.md`).

## 6. Kernel object budget

Limits are set in `config/config.h`. If the team needs more, raise the
limit there. The numbers below are the robot's usage (template examples off).

| Object | Limit | S1 | S2 | S3 | S4 | S5 |
|---|---|---|---|---|---|---|
| Task | 32 | 0 | 1 (+1 MQTT test) | 2 | 2 | 2 |
| Semaphore | 16 | | | | | 1 |
| Event flag | 16 | 1 | 3 | 1 | 1 | 1 |
| Mailbox | 12 | | | | | |
| Mutex | 12 | | | | | |
| Message buffer | 12 | | 1 | | | 1 |
| Fixed memory pool | 12 | | | | | |
| Variable memory pool | 12 | | | | | |
| Cyclic handler | 12 | | | | | |
| Alarm handler | 12 | | | | | |

(The kernel's USB and WiFi service tasks also use a few of these.)
