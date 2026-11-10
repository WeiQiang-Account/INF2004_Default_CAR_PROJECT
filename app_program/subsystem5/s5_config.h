/* s5_config.h - Buddy 5 tunables (obstacle handling + system coordination).
 * Every value is #ifndef-guarded so it can be overridden from the makefile
 * (-DS5_ROBOT_WIDTH_MM=170 ...). Values marked MEASURE must be confirmed on
 * the real robot - they are starting points, not facts.                      */
#ifndef S5_CONFIG_H
#define S5_CONFIG_H

/* ---- Pins (Tribunal 1 pin table; claim in TEAM.md section 2) ------------ */
#ifndef S5_PIN_SERVO
#define S5_PIN_SERVO        12   /* PWM slice 6 ch A, 50 Hz                    */
#endif
#ifndef S5_PIN_TRIG
#define S5_PIN_TRIG         0   /* GPIO out (servo header)                    */
#endif
#ifndef S5_PIN_ECHO
#define S5_PIN_ECHO         1   /* GPIO in via 1k/2k divider (A9)             */
#endif
#define S5_PIN_BTN_START    20   /* Vehicle Controller owns buttons + buzzer   */
#define S5_PIN_BTN_STOP     21
#define S5_PIN_BUZZER       22   /* PWM slice 3 ch A                           */

/* ---- Task priorities: TEAM.md gives S5 the band 20-21 (1 = highest) ------ */
#ifndef S5_PRI_VC
#define S5_PRI_VC           17   /* Week 6 order: Line (16) > VC > Barcode (18) */
#endif
#ifndef S5_PRI_SCAN
#define S5_PRI_SCAN         21
#endif
/* Stack size is fixed by app_start_task(): TASK_STACK_SIZE = 2048 bytes. Keep locals small. */

/* ---- Servo ---------------------------------------------------------------- */
#define S5_SYS_CLK_MHZ      SYSCLK /* sysdef.h: 125 -> PWM divider 125 = 1 MHz tick */
#ifndef S5_SERVO_US_0DEG
#define S5_SERVO_US_0DEG    500   /* MEASURE: pulse for 0 deg                  */
#endif
#ifndef S5_SERVO_US_180DEG
#define S5_SERVO_US_180DEG  2400  /* MEASURE: pulse for 180 deg                */
#endif
#define S5_SERVO_REVERSED   0     /* 1 if increasing angle turns to the RIGHT  */
/* HARD LIMITS: no servo command anywhere (scan, bypass guard, dashboard) can go
 * outside these. 2026-11-06: the head popped off at 30/150 (+/-60 deg), so the
 * range is now 45/135 (+/-45 deg). The scan angles follow these automatically.
 * 90 = straight ahead, >90 = left. Widen only if the head is fixed better.    */
#ifndef S5_SERVO_LIMIT_MIN
#define S5_SERVO_LIMIT_MIN  45    /* furthest RIGHT */
#endif
#ifndef S5_SERVO_LIMIT_MAX
#define S5_SERVO_LIMIT_MAX  135   /* furthest LEFT  */
#endif
#define S5_SERVO_MIN_DEG    S5_SERVO_LIMIT_MIN   /* fine-scan edge search stops here */
#define S5_SERVO_MAX_DEG    S5_SERVO_LIMIT_MAX
#define S5_SERVO_CENTRE_DEG 90    /* 90 = straight ahead, >90 = left           */
#ifndef S5_SERVO_DEG_PER_S
#define S5_SERVO_DEG_PER_S  200   /* servo speed in deg/s: lower = slower, gentler on the head. Check the serial line "coarse scan N ms (limit 1200)" for NFR16; raise towards 300 if it is over */
#endif
#define S5_SERVO_STEP_DEG   3     /* size of one small step                                          */
#define S5_SERVO_MIN_SETTLE_MS 30 /* always wait this long after arriving before a ping              */
#ifndef S5_SERVO_TRIM_DEG
#define S5_SERVO_TRIM_DEG   (-10) /* 2026-11-06: centre pointed ~5-15 deg LEFT, so turn 10 deg right.
                                     Still left -> more negative; now right -> towards 0       */
#endif
#ifndef S5_SERVO_BOOT_WIGGLE
#define S5_SERVO_BOOT_WIGGLE 1    /* at start-up: look left, right, centre (shows the servo follows the code) */
#endif
#define S5_GUARD_LEFT_DEG   S5_SERVO_LIMIT_MAX   /* watch the obstacle while passing it */
#define S5_GUARD_RIGHT_DEG  S5_SERVO_LIMIT_MIN

/* ---- Timing (ms) ---------------------------------------------------------- */
#define S5_SETTLE_FIRST_MS  150   /* first move of a coarse scan (A10)         */
#define S5_SETTLE_COARSE_MS 100   /* 30 deg step                               */
#define S5_SETTLE_FINE_MS   40    /* 5 deg step                                */
#define S5_PING_TMO_MS      30    /* wait on sem_echo                          */
#define S5_PING_GAP_MS      60    /* between the two pings of a median read    */
#ifndef S5_US_DEBUG_PRINT
#define S5_US_DEBUG_PRINT   0     /* print every 4th ping on the console (debug the ultrasonic) */
#endif
#define S5_PING_MIN_GAP_MS  60    /* never trigger the sensor sooner than this after the last ping */
#define S5_MONITOR_MS       80    /* forward-monitor period (>= S5_PING_MIN_GAP_MS) */
#define S5_FAULT_RETRY_MS   500
#define S5_TRIG_US          12    /* same as the stand-alone test that measured correctly */
#define S5_NOSENSOR_LIMIT   3     /* consecutive "no echo edge at all" -> fault*/

/* ---- Distances (mm) ------------------------------------------------------- */
#define S5_MIN_RANGE_MM     30    /* closer than this = glitch (HC-SR04 minimum ~2-3 cm), ignored */
#define S5_MIN_ECHO_US      ((S5_MIN_RANGE_MM * 2000) / 343)   /* 174 us: shorter ECHO pulses are glitches */
#define S5_ECHO_RISE_TMO_US 2000  /* ECHO must rise within this after the trigger (measured ~215 us) */
#define S5_ECHO_WAIT_US     ((S5_MAX_RANGE_MM * 2000) / 343 + 500)   /* ~3.4 ms: longer = beyond range = clear */
#define S5_MAX_RANGE_MM     500   /* 9-bit packing limit is 511                */
#define S5_OBSTACLE_TRIGGER_MM 200 /* stop-and-scan trigger. Braking distance
                                      must keep the robot >= 100 mm away (NFR18) */
#define S5_UNLATCH_MARGIN_MM 100  /* path counts as clear again beyond trigger + this ...      */
#define S5_UNLATCH_PINGS    3     /* ... for this many pings in a row                          */
#define S5_COARSE_CLEAR_MM  300   /* nothing nearer than this -> CONTINUE      */
#define S5_TOO_CLOSE_MM     100   /* bypass guard / reverse threshold          */
#define S5_EDGE_JUMP_MM     60    /* reading farther than closest+this = off the obstacle */
#define S5_CLEAR_MAX_MM     500
#define S5_MIN_WIDTH_MM     30
#define S5_BEAM_INSET_DEG   5     /* beam ~15 deg wide: pull each edge inward  */
#define S5_MAX_RESCAN       3

/* ---- Robot geometry (MEASURE) -------------------------------------------- */
#define S5_ROBOT_WIDTH_MM   150
#define S5_ROBOT_LENGTH_MM  180
#define S5_SAFETY_MARGIN_MM 30
#define S5_OBST_MIN_DEPTH_MM 100  /* ultrasound only sees the front face; depth = max(width, this) */
#define S5_REVERSE_MM       100

/* ---- Vehicle Controller --------------------------------------------------- */
#define S5_VC_TICK_MS       10
#ifndef S5_INIT_TMO_MS
#define S5_INIT_TMO_MS      20000 /* INIT -> FAULT if Motion or IMU not ready (IR calibration: waits, no FAULT) */
#endif
#define S5_CRUISE_MM_S      100   /* line-following speed. 2026-11-07: halved from 200 (less jolting on start/stop, fewer false IMU stops); raise later if needed */
#define S5_CRUISE_DEGRADED_MM_S 100 /* after IMU_LOST (Tribunal: degrade, not E-stop) */
#define S5_CREEP_MM_S       100
#define S5_LINE_TOL_MM      40    /* line is accepted only within +/- this of the expected crossing */
#define S5_LINE_LOST_TMO_MS 3000  /* system statechart                          */
#ifndef S5_LINE_FOUND_MS
#define S5_LINE_FOUND_MS   1000u  /* line must be seen this long to cancel the line-lost timer.
                                     2026-11-10: off the end of the line the robot spins, brushes
                                     the line end for a moment, and that reset the 3 s timer every
                                     time, so it never stopped */
#endif
#define S5_RECOVERY_TARGET_MS 2000 /* NFR19 (logged, not fatal)                 */
#define S5_PROFILE_TMO_MS   8000  /* VC waits this long for a scan result. 2026-11-06: 4000 was too short once the servo was slowed to 200 deg/s (a full coarse + fine scan took ~5 s -> step-timeout FAULT) */
#define S5_STEP_TMO_MS      6000
#define S5_BTN_DEBOUNCE     3     /* ticks                                      */
#define S5_SENSOR_FAULT_IS_FATAL 1

/* flg_fault bit: motion_types.h already reserves FAULT_BIT_NO_ECHO (1<<3) for us. */
#define S5_BYPASS_SPEED_MM_S 150  /* speed for the distance moves of the bypass */

#endif /* S5_CONFIG_H */
