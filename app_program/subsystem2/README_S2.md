# Subsystem 2 – Motion Control (Buddy 2)

Built for the INF2004 micro T-Kernel template (dual-core, `make` in `build_make`).
Not compiled itself: the template only compiles `.c` files.

## Files

| File | What it does |
|---|---|
| `subsystem2.h` | public interface for the other subsystems |
| `subsystem2.c` | `subsystem2_init()` creates `mbf_motion_cmd`, `flg_move_done`, `flg_estop`, `flg_fault`; `subsystem2_start()` starts `motion_task` on core 1 |
| `task_motion.c` | the 20 ms motion task: PID, moves, turns, stop, E-stop, stall (statechart 6.1) |
| `motion_config.h` | pins, wheel/encoder numbers, gains – **measure and edit** |
| `motion_types.h` | packed command (4 B), steering word (16 bit), fault bits, packed status (8 B) |
| `drv_motor.c` | 20 kHz PWM on GP8–GP11 with libbsp `pwm_*` |
| `drv_encoder.c` | x4 quadrature, one 32-bit counter per wheel |
| `drv_gpio_irq.c` | the one shared GPIO edge interrupt (IRQ 13) for encoders, barcode (GP7), echo (GP14) |
| `pid.c` | PID with anti-windup |
| `s2_os_port.c` | the few `tk_*` calls the motion core uses |
| `subsystem2_test.c` | on-target tests S2-U01..U08 (+ guided manual tests S2-M01..M04) |
| `example_producer.c` | the template's example, kept so subsystems 3/4 still build – delete later |

PC tests: `make -C testing/subsystem2/host_tests` (same sources, simulated robot).

## Temporary: right motor faulty

`MOTION_RIGHT_MOTOR_OK` is `0` in `motion_config.h` until the replacement
motor arrives. The motion task then drives and measures with the LEFT wheel
only (no right stall/encoder/self-test checks, right duty 0). Set it back to
`1` when the new motor is fitted. Encoder pins match our wiring: left on
GP16/17, right on GP2/3.

## Build and test

```
cd build_make
make -j8            # normal image
make TEST=1 -j8     # runs S2-U01..U08 at power-up, prints [TEST] lines
```
Set `S2_MANUAL_TESTS` to 1 in `motion_config.h` for the guided CPR / 1 m / 90° tests.
Lift the wheels for the first power-up: the self-test spins them for 0.1 s.

## Things the team must agree (TEAM.md)

- Priority bands: our design needs IMU Safety > Motion > Line Follow.
  Template defaults put subsystem 4 (IMU) below subsystem 2.
- Interfaces IF-06..IF-12 are marked *proposed* in TEAM.md section 5.
- When subsystems 3/4 replace their examples: set `S2_KEEP_TEMPLATE_EXAMPLE` 0
  and delete `example_producer.c`.
