/**
 * @file    motion_config.h
 * @brief   Buddy 2 (Motion Control) configuration: pins, mechanics, timing,
 *          PWM and controller gains.
 *
 * Values marked CONFIRM must be measured on the robot (Week 6 A3/A4) before
 * trusting distance, turn or speed results.
 */
#ifndef MOTION_CONFIG_H
#define MOTION_CONFIG_H

/* ------------------------------------------------------------------------- */
/* Pins (Robo Pico)                                                          */
/* ------------------------------------------------------------------------- */
#define MOTOR_L_PIN_A           (8u)    /* M1A - left motor driver input A   */
#define MOTOR_L_PIN_B           (9u)    /* M1B - left motor driver input B   */
#define MOTOR_R_PIN_A           (10u)   /* M2A - right motor driver input A  */
#define MOTOR_R_PIN_B           (11u)   /* M2B - right motor driver input B  */

/* Wired on our robot: LEFT encoder on Grove GP16/17, RIGHT on Grove GP2/3 */
#define ENC_L_PIN_A             (16u)   /* Grove 4: plug pin 02 (red)        */
#define ENC_L_PIN_B             (17u)   /*          plug pin 03 (green)      */
#define ENC_R_PIN_A             (2u)    /* Grove 2: plug pin 02 (red)        */
#define ENC_R_PIN_B             (3u)    /*          plug pin 03 (green)      */

/* Right motor replaced 2026-10-29: 1 = normal two-wheel operation. With 0 the
 * motion task drives and measures with the LEFT wheel only:
 *  - no stall / encoder-fault / self-test check on the right wheel
 *  - distance and turn progress come from the left encoder
 *  - the right motor gets 0 duty
 * (0 is only for running with one broken motor.)                          */
#ifndef MOTION_RIGHT_MOTOR_OK
#define MOTION_RIGHT_MOTOR_OK   (1)
#endif

/* Direction fixes: set so that driving FORWARD gives positive counts and a
 * positive duty moves the wheel forward. The right side is usually mirrored.
 * CONFIRM by pushing the robot forward and reading the counts.             */
#define ENC_L_SIGN              (1)
#define ENC_R_SIGN              (-1)
#define MOTOR_L_INVERT          (0)
#define MOTOR_R_INVERT          (1)   /* 2026-10-29: new right motor turned backward on forward */

/* Optional timing pin for logic-analyser tests: high during each 20 ms Motion
 * cycle. OFF (-1) by default: GP15 is also a servo header on the Robo Pico, and
 * a servo plugged there reads that pulse as "go to the end stop".
 * Set to 15 only for the timing test, with nothing else on GP15.           */
#define MOTION_DEBUG_PIN        (-1)

/* ------------------------------------------------------------------------- */
/* Mechanics - CONFIRM (A3)                                                  */
/* ------------------------------------------------------------------------- */
/* MEASURED 2026-10-24 (S2-M01): 10 wheel turns by hand = 25348 counts,
 * so 2535 counts per wheel turn (x4 decoding). Matches a 13-pulse Hall
 * encoder with a ~1:48 gearbox (13 x 4 x 48.75 = 2535). Re-measure once more
 * and with the replacement motor.                                          */
#define ENC_PPR                 (13.0f)   /* pulses per motor rev, 1 channel */
#define ENC_GEAR_RATIO          (48.75f)  /* motor revs per wheel rev (from measured CPR) */
#define ENC_DECODE_FACTOR       (4.0f)    /* x4 quadrature                   */
#define ENC_CPR                 (ENC_PPR * ENC_DECODE_FACTOR * ENC_GEAR_RATIO)

#define WHEEL_DIAMETER_MM       (66.8f)   /* EFFECTIVE, 2026-11-07: 1.02 m driven on the floor read 0.962 m with 63 mm (ruler, 2026-10-24); the tyre rolls like a 66.8 mm wheel. Re-check with S2-M02 */
#define TRACK_WIDTH_MM          (110.0f)  /* MEASURED 2026-10-24: wheel centre to centre */
/* Right wheel distance per count / left wheel distance per count. Tyres are
 * never exactly the same size, so equal counts can still curve the robot.
 * From a straight 1 m run (S2-M02) with sideways drift d mm at L mm:
 *     drifts RIGHT:  WHEEL_R_SCALE = 1 - 2*d*TRACK / L^2   (e.g. 25 mm -> 0.9945)
 *     drifts LEFT:   WHEEL_R_SCALE = 1 + 2*d*TRACK / L^2
 * 1.0 = both wheels the same.                                               */
/* On-the-spot turns: the tyres scrub sideways, so the robot turns a little
 * less than the wheel arc says. Arc is multiplied by this.
 * 2026-11-09: 90 deg asked -> ~87 deg turned, so 90 / 87 = 1.0345.
 * Turned too little -> raise it; too far -> lower it (new = old x 90 / measured). */
#ifndef TURN_SCALE
#define TURN_SCALE              (1.075f)  /* 2026-11-10: 1.070 -> ~1.7 deg short, 1.085 -> ~1.7 deg too far (crossed the joint ~3 cm over 1 m) -> in between */
#endif
#ifndef WHEEL_R_SCALE
#define WHEEL_R_SCALE           (1.0012f) /* FINAL 2026-11-10: 1.0007 -> 5 L x3; 1.0018 -> 20 R, 5 L, 0 (avg 5 R). Centre between them. Runs vary about +-10 mm (placement, start, caster) */
#endif
#define MOTION_PI               (3.14159265f)
#define MM_PER_COUNT            ((MOTION_PI * WHEEL_DIAMETER_MM) / ENC_CPR)

/* ------------------------------------------------------------------------- */
/* Timing                                                                    */
/* ------------------------------------------------------------------------- */
#define MOTION_PERIOD_MS        (20u)     /* NFR5                            */
#define STEER_STALE_MS          (60u)     /* steer_latest considered stale   */
#define STALL_TIMEOUT_MS        (300u)    /* PWM on, no pulses -> FAULT      */
#define STOP_TIMEOUT_MS         (300u)    /* max time spent in STOPPING      */
#define STOP_SETTLE_CYCLES      (2u)      /* cycles below STOPPED speed      */
#define CMD_SEND_TIMEOUT_MS     (5)       /* API -> mbf_motion_cmd           */

/* ------------------------------------------------------------------------- */
/* PWM                                                                       */
/* ------------------------------------------------------------------------- */
#define MOTOR_PWM_FREQ_HZ       (20000u)  /* above hearing range             */
#define DUTY_MAX                (1000)    /* duty is in permille (0..1000)   */
#define STALL_MIN_DUTY          (250)     /* only check stall above this     */

/* Stop behaviour of the Robo Pico driver. Our statechart uses A = B = LOW.
 * If the datasheet shows HIGH/HIGH is "brake" and LOW/LOW is "coast", set
 * this to 1 to brake harder.                                                */
#define MOTOR_STOP_BOTH_HIGH    (0)

/* ------------------------------------------------------------------------- */
/* Speeds and manoeuvres                                                     */
/* ------------------------------------------------------------------------- */
#define MAX_SPEED_MMPS          (500)
#define DEFAULT_MOVE_SPEED_MMPS (250)
/* Line following speeds up gently instead of jumping to full speed in one
 * 20 ms step. 2026-11-07: the jump jolted/rocked the robot on a hard floor and
 * the IMU (GY-511, no gyro) took it for a tilt -> E-stop. 800 mm/s^2 reaches
 * 290 mm/s in ~0.36 s. 0 = no ramp (old behaviour).                         */
#ifndef DRIVE_ACCEL_MMPS2
#define DRIVE_ACCEL_MMPS2       (800)
#endif
#define TURN_SPEED_MMPS         (120)
#define SLOW_SPEED_MMPS         (60)      /* final approach speed            */
#define MOVE_SLOW_FRACTION      (0.85f)   /* slow down after 85 % of a move  */
/* Distance moves start at this speed and then speed up (DRIVE_ACCEL_MMPS2).
 * 2026-11-09: a jump straight to 150 mm/s kicked the robot to the right at the
 * start (one wheel bites first); a ramp from 0 was worse (both wheels crawl
 * through the motor dead band). Starting at the slow approach speed, which
 * both motors turn at, is the middle way. 0 = jump to full speed at once.  */
/* Before a distance move from standstill, both motors get the SAME small duty
 * for a moment. This takes up the slack in both gearboxes, so both wheels
 * bite together when the real start comes. 2026-11-09: without it, one wheel
 * bit first and the robot jolted right just as it set off. 0 = off.        */
/* Compass heading hold for straight distance moves (EXPERIMENT, 2026-11-10).
 * The GY-511 has no gyro, but it has a compass. At the start of a move the
 * heading is stored; while driving, any twist (start jolt, slip, caster) is
 * steered back. Without a working compass the move falls back to counts only.
 * 0 = off (counts only, as before).                                         */
#ifndef HEADING_HOLD
#define HEADING_HOLD            (0)   /* 2026-11-10: OFF to test: after the turn the 1 m leg bent ~6 deg with it on */
#endif
#ifndef HEADING_KP
#define HEADING_KP              (3.0f)    /* mm/s wheel difference per degree of heading error */
#endif
#ifndef HEADING_MAX_MMPS
#define HEADING_MAX_MMPS        (40.0f)   /* never steer harder than this                      */
#endif
#ifndef HEADING_COUNTS_KP
#define HEADING_COUNTS_KP       (0.0f)    /* count-equality gain while the compass steers (STRAIGHT_KP otherwise).
                                             0: any count term fights the compass and leaves part of a twist (host test) */
#endif
#ifndef MOVE_PRELOAD_DUTY
#define MOVE_PRELOAD_DUTY       (0)       /* 2026-11-09: OFF - with it the robot drifted ~10 mm right whatever WHEEL_R_SCALE was. 120 = on */
#endif
#ifndef MOVE_PRELOAD_MS
#define MOVE_PRELOAD_MS         (200u)
#endif
#ifndef MOVE_START_MMPS
#define MOVE_START_MMPS         (0)   /* 2026-11-09: off again: firm start gave the best straight runs */
#endif
#define TURN_SLOW_FRACTION      (0.80f)   /* slow down after 80 % of a turn  */
#define STOPPED_SPEED_MMPS      (20.0f)
/* The robot keeps rolling briefly after PWM goes to 0. Moves and turns end
 * this early: coast = measured speed x STOP_COAST_S. TUNE on the robot by
 * measuring overshoot at SLOW_SPEED_MMPS (0 disables the compensation).   */
#define STOP_COAST_S            (0.08f)

#define STEER_GAIN_MMPS         (1.5f)    /* mm/s per steering unit (+-100)  */
#define STEER_MAX_FRACTION      (0.7f)    /* steer at most 70 % of the speed: inner wheel keeps >= 30 % */
#define STRAIGHT_KP             (2.0f)    /* mm/s per count of L/R mismatch  */

/* ------------------------------------------------------------------------- */
/* Wheel-speed PID - TUNE (A4). Output is duty in permille.                  */
/* ------------------------------------------------------------------------- */
#define SPEED_FF                (1.6f)    /* duty per mm/s (feed-forward)    */
#define SPEED_KP                (0.8f)
#define SPEED_KI                (4.0f)    /* per second                      */
#define SPEED_KD                (0.0f)
#define SPEED_I_LIMIT           (400.0f)  /* anti-windup clamp on I term     */

/* ------------------------------------------------------------------------- */
/* Encoder health                                                            */
/* ------------------------------------------------------------------------- */
#define ENC_MAX_BAD_PER_CYCLE   (10u)     /* skipped quadrature states in ONE 20 ms cycle. 2026-11-06:
                                             4 tripped "encoder error" during a hard turn on the robot */
/* ... and only a FAULT if that happens in this many cycles IN A ROW. 2026-11-10:
 * the left encoder's wires pick up motor noise in bursts (56 skips in ~2 s); one
 * noisy cycle tripped "wheel encoder error" a few cm after START. Skips are now
 * recovered (drv_encoder.c), so only a lasting fault (broken wire) stops it. */
#ifndef ENC_BAD_CYCLES
#define ENC_BAD_CYCLES          (5u)      /* 5 x 20 ms = 100 ms of continuous errors */
#endif
#define MOTION_SELFTEST_ENABLE  (1)       /* short pulse at start-up         */
#define SELFTEST_DUTY           (300)
#define SELFTEST_MS             (100u)
#define SELFTEST_MIN_COUNTS     (5)

/* ------------------------------------------------------------------------- */
/* Week 13 change requirement: build the "before" baseline with 0 and the    */
/* minimum-bit version with 1, then compare RAM and message sizes.           */
/* ------------------------------------------------------------------------- */
#ifndef MOTION_USE_PACKED
#define MOTION_USE_PACKED       (1)
#endif

/* Size of mbf_motion_cmd (holds 32 packed or 6 unpacked commands).
 * Task priority and core are in subsystem2.h (S2_PRI_MOTION).              */
#define MOTION_CMD_QUEUE_BYTES  (128u)

/* Test build (make TEST=1) only: 1 = after the automatic tests, run the
 * guided manual tests S2-M01..M04 (CPR count, 1 m, 90 deg turns). Read the
 * console instructions; the robot drives, so give it floor space.         */
#define S2_MANUAL_TESTS         (0)   /* 0 = automatic tests only (U01-U08 + M06 timing). 1 = guided moves/turns/speed after them */
/* Parts of the guided test (only with S2_MANUAL_TESTS 1):                  */
#ifndef S2_MANUAL_SPEED
#define S2_MANUAL_SPEED         (0)   /* 1 = S2-M05 speed hold at 300 mm/s INSTEAD of the moves/turns (robot propped up!) */
#endif
#ifndef S2_MANUAL_SPEED_MMPS
#define S2_MANUAL_SPEED_MMPS    (300)
#endif
#ifndef S2_MANUAL_CPR
#define S2_MANUAL_CPR           (0)   /* 1 = 20 s hand-turn count first (done 2026-11-07) */
#endif
#ifndef S2_MANUAL_TURNS
#define S2_MANUAL_TURNS         (1)   /* 1 = 90 deg turns (on the spot) after the straight move */
#endif
#ifndef S2_MANUAL_AFTER_TURN_MM
#define S2_MANUAL_AFTER_TURN_MM (0)   /* straight on after the right turn, mm (0 = stop, measure the turn with a long straight edge) */
#endif
#ifndef S2_MANUAL_PAUSE_S
#define S2_MANUAL_PAUSE_S       (20)  /* seconds stopped before the turn: time to measure the heading before turning */
#endif
#ifndef S2_MANUAL_TURN_RIGHT
#define S2_MANUAL_TURN_RIGHT    (0)   /* 1 = 90 deg RIGHT turn (S2-M03). 2026-11-10: off, testing the left turn */
#endif
#ifndef S2_MANUAL_TURN_LEFT
#define S2_MANUAL_TURN_LEFT     (1)   /* 1 = 90 deg LEFT turn (S2-M04); after the right turn if both are on */
#endif
/* Guided test geometry (2026-11-10, floor tiles of 600 mm).
 * START: the BACK of both tyres touches the tile joint (the start line), so
 * the axle is one wheel radius (31.5 mm, 63 mm tyre) in front of it.
 * TURN: the 90 deg RIGHT spin is about the axle centre; afterwards the right
 * wheel sits half a track (55 mm) BEHIND the centre. To put the middle of
 * the right tyre exactly on the next joint (600 mm) the robot drives
 *     600 + 55 - 31.5 = 623.5 -> 624 mm.                                   */
#ifndef S2_MANUAL_MARK_MM
#define S2_MANUAL_MARK_MM       (625) /* where the right tyre must be after the turn, from the start line (600 + 25: 2026-11-10 runs kept stopping short) */
#endif
#ifndef S2_MANUAL_START_AXLE_MM
#define S2_MANUAL_START_AXLE_MM (31.5f) /* axle in front of the start line: back of tyre on the line */
#endif
#ifndef S2_MANUAL_MOVE_MM
#if (S2_MANUAL_TURNS != 0)
#define S2_MANUAL_MOVE_MM       ((INT)((float)S2_MANUAL_MARK_MM + (TRACK_WIDTH_MM / 2.0f) - S2_MANUAL_START_AXLE_MM + 0.5f))
#else
#define S2_MANUAL_MOVE_MM       (1000) /* straight-only check: 1 m, as in the best straight runs */
#endif
#endif

/* 1 = motor bring-up demo (s2_motor_demo.c) INSTEAD of the motion task:
 * forward / backward / rotate with fixed power, no PID. Use 0 for the
 * real program and for the S2 tests.                                      */
/* Wi-Fi telemetry QUICK TEST (s2_telemetry_mqtt.c / s2_telemetry_drive.c).
 * Only active in an MQTT build: make NET=lwip MQTT=1. Publishes motion data
 * on se14/robot/s2test (Buddy 1 owns se14/robot/telemetry) and runs a
 * drive pattern. Subsystem 1 calls it through s2_telemetry_test_on_connected().                 */
#ifndef S2_TELEMETRY_TEST
#define S2_TELEMETRY_TEST       (0)   /* 0: Buddy 1 (subsystem 1) publishes telemetry now */
#endif

/* Only with S2_TELEMETRY_TEST: 1 = also run the drive pattern
 * (forward / backward / turn), 0 = motors stay still so you can turn the
 * wheels by hand (CPR test) and watch enc_l in MQTT Explorer.            */
#ifndef S2_TELEMETRY_DRIVE
#define S2_TELEMETRY_DRIVE      (0)
#endif

#ifndef S2_MOTOR_DEMO
#define S2_MOTOR_DEMO           (0)
#endif

#endif /* MOTION_CONFIG_H */
