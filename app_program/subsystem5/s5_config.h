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
#define S5_PIN_TRIG         13   /* GPIO out (servo header)                    */
#endif
#ifndef S5_PIN_ECHO
#define S5_PIN_ECHO         14   /* GPIO in via 1k/2k divider (A9)             */
#endif
#define S5_PIN_BTN_START    20   /* Vehicle Controller owns buttons + buzzer   */
#define S5_PIN_BTN_STOP     21
#define S5_PIN_BUZZER       22   /* PWM slice 3 ch A                           */

/* ---- Task priorities: TEAM.md gives S5 the band 20-21 (1 = highest) ------ */
#ifndef S5_PRI_VC
#define S5_PRI_VC           20
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
#define S5_SERVO_MIN_DEG    15
#define S5_SERVO_MAX_DEG    165
#define S5_SERVO_CENTRE_DEG 90    /* 90 = straight ahead, >90 = left           */
#define S5_GUARD_LEFT_DEG   165
#define S5_GUARD_RIGHT_DEG  15

/* ---- Timing (ms) ---------------------------------------------------------- */
#define S5_SETTLE_FIRST_MS  150   /* first move of a coarse scan (A10)         */
#define S5_SETTLE_COARSE_MS 100   /* 30 deg step                               */
#define S5_SETTLE_FINE_MS   40    /* 5 deg step                                */
#define S5_PING_TMO_MS      30    /* wait on sem_echo                          */
#define S5_PING_GAP_MS      20    /* between the two pings of a median read    */
#define S5_MONITOR_MS       60    /* forward-monitor period                    */
#define S5_FAULT_RETRY_MS   500
#define S5_TRIG_US          12
#define S5_NOSENSOR_LIMIT   3     /* consecutive "no echo edge at all" -> fault*/

/* ---- Distances (mm) ------------------------------------------------------- */
#define S5_MIN_RANGE_MM     20
#define S5_MAX_RANGE_MM     500   /* 9-bit packing limit is 511                */
#define S5_OBSTACLE_TRIGGER_MM 200 /* stop-and-scan trigger. Braking distance
                                      must keep the robot >= 100 mm away (NFR18) */
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
#define S5_CRUISE_MM_S      200   /* matches whatever S2 calls "normal speed"  */
#define S5_CRUISE_DEGRADED_MM_S 100 /* after IMU_LOST (Tribunal: degrade, not E-stop) */
#define S5_CREEP_MM_S       100
#define S5_LINE_TOL_MM      40    /* line is accepted only within +/- this of the expected crossing */
#define S5_LINE_LOST_TMO_MS 3000  /* system statechart                          */
#define S5_RECOVERY_TARGET_MS 2000 /* NFR19 (logged, not fatal)                 */
#define S5_PROFILE_TMO_MS   4000
#define S5_STEP_TMO_MS      6000
#define S5_BTN_DEBOUNCE     3     /* ticks                                      */
#define S5_SENSOR_FAULT_IS_FATAL 1

/* flg_fault bit: motion_types.h already reserves FAULT_BIT_NO_ECHO (1<<3) for us. */
#define S5_BYPASS_SPEED_MM_S 150  /* speed for the distance moves of the bypass */

#endif /* S5_CONFIG_H */
