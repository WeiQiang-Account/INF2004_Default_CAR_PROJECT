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

/* TEMPORARY: right motor is faulty (waiting for a replacement). With 0 the
 * motion task drives and measures with the LEFT wheel only:
 *  - no stall / encoder-fault / self-test check on the right wheel
 *  - distance and turn progress come from the left encoder
 *  - the right motor gets 0 duty
 * Set back to 1 when the new motor is fitted.                              */
#ifndef MOTION_RIGHT_MOTOR_OK
#define MOTION_RIGHT_MOTOR_OK   (0)
#endif

/* Direction fixes: set so that driving FORWARD gives positive counts and a
 * positive duty moves the wheel forward. The right side is usually mirrored.
 * CONFIRM by pushing the robot forward and reading the counts.             */
#define ENC_L_SIGN              (1)
#define ENC_R_SIGN              (-1)
#define MOTOR_L_INVERT          (0)
#define MOTOR_R_INVERT          (0)

/* Optional timing pin for logic-analyser tests (GP15 is spare; claim it in
 * TEAM.md). Set to -1 to disable. High during each 20 ms Motion cycle.     */
#define MOTION_DEBUG_PIN        (15)

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

#define WHEEL_DIAMETER_MM       (63.0f)   /* MEASURED 2026-10-24 with a ruler */
#define TRACK_WIDTH_MM          (110.0f)  /* MEASURED 2026-10-24: wheel centre to centre */
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
#define TURN_SPEED_MMPS         (150)
#define SLOW_SPEED_MMPS         (80)      /* final approach speed            */
#define MOVE_SLOW_FRACTION      (0.85f)   /* slow down after 85 % of a move  */
#define TURN_SLOW_FRACTION      (0.80f)   /* slow down after 80 % of a turn  */
#define STOPPED_SPEED_MMPS      (20.0f)
/* The robot keeps rolling briefly after PWM goes to 0. Moves and turns end
 * this early: coast = measured speed x STOP_COAST_S. TUNE on the robot by
 * measuring overshoot at SLOW_SPEED_MMPS (0 disables the compensation).   */
#define STOP_COAST_S            (0.08f)

#define STEER_GAIN_MMPS         (1.5f)    /* mm/s per steering unit (+-100)  */
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
#define ENC_MAX_BAD_PER_CYCLE   (4u)      /* skipped quadrature states       */
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
#define S2_MANUAL_TESTS         (0)

/* 1 = motor bring-up demo (s2_motor_demo.c) INSTEAD of the motion task:
 * forward / backward / rotate with fixed power, no PID. Use 0 for the
 * real program and for the S2 tests.                                      */
/* Wi-Fi telemetry QUICK TEST (s2_telemetry_mqtt.c / s2_telemetry_drive.c).
 * Only active in an MQTT build: make NET=lwip MQTT=1. Publishes motion data
 * on se14/robot/telemetry and runs a drive pattern. Set to 0 once Buddy 1's
 * telemetry subsystem publishes the motion data instead.                 */
#ifndef S2_TELEMETRY_TEST
#define S2_TELEMETRY_TEST       (0)
#endif

/* Only with S2_TELEMETRY_TEST: 1 = also run the drive pattern
 * (forward / backward / turn), 0 = motors stay still so you can turn the
 * wheels by hand (CPR test) and watch enc_l in MQTT Explorer.            */
#ifndef S2_TELEMETRY_DRIVE
#define S2_TELEMETRY_DRIVE      (1)
#endif

#ifndef S2_MOTOR_DEMO
#define S2_MOTOR_DEMO           (0)
#endif

#endif /* MOTION_CONFIG_H */
