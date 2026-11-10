/**
 * @file    task_motion.c
 * @brief   Subsystem 2 -- Motion Control task (Buddy 2): wheel-speed PID,
 *          distance moves, encoder turns, stopping, E-stop and stall handling.
 *
 * The same file is compiled for the robot and for the PC tests
 * (testing/subsystem2/host_tests, built with -DHOST_TEST).
 *
 * Implements statechart 6.1:
 *   INIT -> IDLE -> {DRIVING, DIST_MOVE, TURNING} -> STOPPING -> IDLE
 *   any state --flg_estop / stall / encoder fault--> FAULT --reset--> IDLE
 * A new command while active switches sub-state directly (latest wins).
 */
#include <stddef.h>
#include "motion.h"
#include "motion_config.h"
#include "drv_encoder.h"
#include "drv_motor.h"
#include "pid.h"

#ifndef HOST_TEST
#include "drv_gpio_irq.h"
#endif

#define MOTION_WEAK __attribute__((weak))

/* Compass heading (deg, left turn = +) from subsystem 4. This weak default
   says "no compass"; subsystem4.c has the real one.                        */
MOTION_WEAK bool s2_hook_heading(float *deg) { (void)deg; return false; }

/* Steering mailbox: written only by Line Follow, read only here.          */
volatile uint16_t g_steer_latest = 0u;

/* ------------------------------------------------------------------------- */
/* Command message (packed for the change requirement, or the baseline)     */
/* ------------------------------------------------------------------------- */
#if (MOTION_USE_PACKED != 0)
typedef motion_cmd_packed_t motion_msg_t;     /* 4 bytes  */
#else
typedef motion_cmd_t        motion_msg_t;     /* 20 bytes */
#endif

/* ------------------------------------------------------------------------- */
/* Task context                                                             */
/* ------------------------------------------------------------------------- */
typedef struct
{
    motion_state_t state;
    motion_cause_t cause;

    int32_t  last_l;            /* encoder snapshot from the previous cycle */
    int32_t  last_r;
    int32_t  start_l;           /* snapshot at the start of a move / turn   */
    int32_t  start_r;
    int32_t  target_counts;     /* move / turn length in counts             */
    int32_t  speed_mmps;        /* commanded speed magnitude                */
    int32_t  dir;               /* +1 / -1 (turn: +1 = right)               */

    float    meas_l;            /* measured wheel speeds, mm/s              */
    float    meas_r;
    float    base_now;          /* ramped line-following speed, mm/s        */
    uint32_t preload_cycles;    /* gearbox take-up cycles left (move start) */
    uint32_t bad_cycles;        /* cycles in a row with too many encoder skips */
    bool     hdg_on;            /* this move holds a compass heading        */
    float    hdg_ref;           /* heading at the start of the move, deg    */
    float    hdg_err;           /* last heading error, deg (for the log)    */
    bool     ramp_on;           /* base_now is being used                   */
    float    tgt_l;             /* target wheel speeds, mm/s                */
    float    tgt_r;
    int32_t  duty_l;            /* last duty applied, permille              */
    int32_t  duty_r;
    int32_t  dl;                /* counts in the last cycle                 */
    int32_t  dr;

    uint32_t stall_ms_l;
    uint32_t stall_ms_r;
    uint32_t stop_start_ms;
    uint32_t stop_settle;
    uint32_t selftest_start_ms;
    bool     selftest_running;

    uint8_t  steer_seq;
    bool     steer_seen;
    bool     steer_stale;
    uint32_t steer_change_ms;

    float    distance_mm;
} motion_ctx_t;

static motion_ctx_t    s_ctx;
static motion_status_t s_status;
static pid_ctrl_t      s_pid_l;
static pid_ctrl_t      s_pid_r;
static motion_links_t  s_links;
static volatile uint32_t s_status_seq;     /* odd while s_status is being written */
static volatile uint32_t s_cycles;         /* completed 20 ms cycles             */
/* S2-M06: the task times its own wake-ups (1 us timer), so no other task has
   to busy-wait to measure it. Written by the motion task only.           */
static volatile uint32_t s_per_min = 0xFFFFFFFFu, s_per_max, s_per_sum, s_per_n;
static volatile uint32_t s_busy_max, s_busy_sum;   /* time spent in one cycle's work, us */
static uint32_t          s_per_last;
static volatile bool     s_per_reset = true;

/* ------------------------------------------------------------------------- */
/* Weak hooks (other buddies override)                                      */
/* ------------------------------------------------------------------------- */
MOTION_WEAK void motion_status_hook(const motion_status_t *p_status)
{
    (void)p_status;
}

MOTION_WEAK void motion_watchdog_checkin(void)
{
}

/* ------------------------------------------------------------------------- */
/* Helpers                                                                  */
/* ------------------------------------------------------------------------- */
static float motion_absf(float v)
{
    return (v < 0.0f) ? -v : v;
}

static int32_t motion_abs(int32_t v)
{
    return (v < 0) ? -v : v;
}

static int32_t motion_clamp(int32_t v, int32_t lo, int32_t hi)
{
    if (v < lo) { return lo; }
    if (v > hi) { return hi; }
    return v;
}

static int8_t motion_to_i8(int32_t v)
{
    return (int8_t)motion_clamp(v, -128, 127);
}

static int32_t motion_mm_to_counts(float mm)
{
    return (int32_t)((mm / MM_PER_COUNT) + 0.5f);
}

/* Counts the robot will still roll after braking at its current speed */
static int32_t motion_coast_counts(void)
{
    float v = (motion_absf(s_ctx.meas_l) + motion_absf(s_ctx.meas_r)) * 0.5f;

    return (int32_t)((v * STOP_COAST_S) / MM_PER_COUNT);
}

static void motion_reset_controllers(void)
{
    pid_reset(&s_pid_l);
    pid_reset(&s_pid_r);
    s_ctx.ramp_on = false;
    s_ctx.stall_ms_l = 0u;
    s_ctx.stall_ms_r = 0u;
}

static void motion_mark_start(void)
{
    s_ctx.start_l = s_ctx.last_l;
    s_ctx.start_r = s_ctx.last_r;
}

static void motion_enter_fault(motion_cause_t cause)
{
    motor_stop_all();
    s_ctx.duty_l = 0;
    s_ctx.duty_r = 0;
    s_ctx.tgt_l = 0.0f;
    s_ctx.tgt_r = 0.0f;
    s_ctx.state = MOTION_STATE_FAULT;
    s_ctx.cause = cause;
    motion_reset_controllers();

    if (cause == MOTION_CAUSE_STALL)
    {
        os_flag_set(s_links.flg_fault, FAULT_BIT_MOTOR_STALL);
    }
    else if ((cause == MOTION_CAUSE_ENCODER) || (cause == MOTION_CAUSE_SELFTEST))
    {
        os_flag_set(s_links.flg_fault, FAULT_BIT_ENCODER);
    }
    else
    {
        /* E-stop bit is owned and cleared by IMU Safety Monitor */
    }
}

static void motion_enter_stopping(uint32_t now_ms)
{
    s_ctx.ramp_on = false;
    s_ctx.state = MOTION_STATE_STOPPING;
    s_ctx.stop_start_ms = now_ms;
    s_ctx.stop_settle = 0u;
    s_ctx.tgt_l = 0.0f;
    s_ctx.tgt_r = 0.0f;
}

static void motion_finish_move(void)
{
    s_ctx.state = MOTION_STATE_IDLE;
    motion_reset_controllers();
    os_flag_set(s_links.flg_move_done, MOVE_DONE_BIT);
}

static bool motion_is_active(motion_state_t st)
{
    return (st == MOTION_STATE_DRIVING) || (st == MOTION_STATE_DIST_MOVE) ||
           (st == MOTION_STATE_TURNING);
}

/* ------------------------------------------------------------------------- */
/* Command handling                                                         */
/* ------------------------------------------------------------------------- */
static void motion_handle_cmd(const motion_cmd_t *p_cmd, uint32_t now_ms)
{
    int32_t speed = motion_clamp(p_cmd->speed_mmps, 0, MAX_SPEED_MMPS);
    bool    was_active = motion_is_active(s_ctx.state);

    if (s_ctx.state == MOTION_STATE_FAULT)
    {
        if ((p_cmd->type == (int32_t)MOTION_CMD_RESET) &&
            (os_flag_poll(s_links.flg_estop, ESTOP_BIT) == false))
        {
            os_flag_clear(s_links.flg_fault, FAULT_BIT_MOTOR_STALL | FAULT_BIT_ENCODER);
            s_ctx.cause = MOTION_CAUSE_NONE;
            s_ctx.state = MOTION_STATE_IDLE;
            motion_reset_controllers();
        }
        return;   /* everything else is ignored in FAULT */
    }

    if (s_ctx.state == MOTION_STATE_INIT)
    {
        return;
    }

    switch ((motion_cmd_type_t)p_cmd->type)
    {
        case MOTION_CMD_DRIVE:
            os_flag_clear(s_links.flg_move_done, MOVE_DONE_BIT);
            s_ctx.dir = (p_cmd->reverse != 0) ? -1 : 1;
            s_ctx.speed_mmps = speed;
            if (!was_active) { motion_reset_controllers(); }
            motion_mark_start();
            s_ctx.state = MOTION_STATE_DRIVING;
            break;

        case MOTION_CMD_MOVE_FWD:
        case MOTION_CMD_MOVE_BWD:
            os_flag_clear(s_links.flg_move_done, MOVE_DONE_BIT);
            s_ctx.dir = (p_cmd->type == (int32_t)MOTION_CMD_MOVE_BWD) ? -1 : 1;
            s_ctx.speed_mmps = (speed > 0) ? speed : DEFAULT_MOVE_SPEED_MMPS;
            s_ctx.target_counts = motion_mm_to_counts((float)motion_clamp(p_cmd->distance_mm, 0, 2047));
            if (!was_active) { motion_reset_controllers(); }
            s_ctx.ramp_on = false;          /* new move: soft start again */
            s_ctx.preload_cycles = (!was_active && (MOVE_PRELOAD_DUTY > 0)) ?
                                   (MOVE_PRELOAD_MS / MOTION_PERIOD_MS) : 0u;
            /* heading taken now, motors still off (their field moves the compass) */
            s_ctx.hdg_on = (HEADING_HOLD != 0) && s2_hook_heading(&s_ctx.hdg_ref);
            s_ctx.hdg_err = 0.0f;
            motion_mark_start();
            if (s_ctx.target_counts <= 0)
            {
                motion_enter_stopping(now_ms);
            }
            else
            {
                s_ctx.state = MOTION_STATE_DIST_MOVE;
            }
            break;

        case MOTION_CMD_TURN_LEFT:
        case MOTION_CMD_TURN_RIGHT:
        {
            float arc_mm = (MOTION_PI * TRACK_WIDTH_MM * TURN_SCALE *
                            (float)motion_clamp(p_cmd->angle_deg, 0, 360)) / 360.0f;

            os_flag_clear(s_links.flg_move_done, MOVE_DONE_BIT);
            s_ctx.dir = (p_cmd->type == (int32_t)MOTION_CMD_TURN_RIGHT) ? 1 : -1;
            s_ctx.speed_mmps = TURN_SPEED_MMPS;
            s_ctx.target_counts = motion_mm_to_counts(arc_mm);
            motion_reset_controllers();     /* wheels change direction */
            motion_mark_start();
            if (s_ctx.target_counts <= 0)
            {
                motion_enter_stopping(now_ms);
            }
            else
            {
                s_ctx.state = MOTION_STATE_TURNING;
            }
            break;
        }

        case MOTION_CMD_STOP:
            if (was_active)
            {
                motion_enter_stopping(now_ms);
            }
            else if (s_ctx.state == MOTION_STATE_IDLE)
            {
                os_flag_set(s_links.flg_move_done, MOVE_DONE_BIT);   /* acknowledge */
            }
            else
            {
                /* already STOPPING */
            }
            break;

        case MOTION_CMD_NONE:
        case MOTION_CMD_RESET:
        default:
            break;
    }
}

static void motion_drain_commands(uint32_t now_ms)
{
    motion_msg_t msg;
    motion_cmd_t cmd;

    while (os_mbf_recv_poll(s_links.mbf_cmd, &msg) == (int32_t)sizeof(msg))
    {
#if (MOTION_USE_PACKED != 0)
        motion_cmd_unpack(msg, &cmd);
#else
        cmd = msg;
#endif
        motion_handle_cmd(&cmd, now_ms);
    }
}

/* ------------------------------------------------------------------------- */
/* Per-state target speeds                                                  */
/* ------------------------------------------------------------------------- */
/* Test only (S2-M05 speed hold): ignore Line Follow's steering, so both
   wheels get the same target while the robot is propped up off the line. */
static volatile bool s_ignore_steer;
void motion_ignore_steering(bool on) { s_ignore_steer = on; }

static void motion_update_steering(uint32_t now_ms, int8_t *p_corr)
{
    uint16_t w = g_steer_latest;          /* single 16-bit read: never torn */
    int8_t   corr;
    uint8_t  seq;

    steer_unpack(w, &corr, &seq);
    if ((!s_ctx.steer_seen && (w != 0u)) || (s_ctx.steer_seen && (seq != s_ctx.steer_seq)))
    {
        s_ctx.steer_seq = seq;
        s_ctx.steer_seen = true;
        s_ctx.steer_change_ms = now_ms;
    }

    s_ctx.steer_stale = (!s_ctx.steer_seen) ||
                        ((uint32_t)(now_ms - s_ctx.steer_change_ms) > STEER_STALE_MS);
    *p_corr = (s_ctx.steer_stale || s_ignore_steer) ? (int8_t)0 : corr;
}

/* Move the ramped speed one step towards 'target' (mm/s). 'start' is used on
   the first call after a stop / new move: what the wheels do now.          */
static float motion_ramp(float target, float start)
{
#if (DRIVE_ACCEL_MMPS2 > 0)
    float step = (float)DRIVE_ACCEL_MMPS2 * ((float)MOTION_PERIOD_MS / 1000.0f);
    if (!s_ctx.ramp_on) {
        s_ctx.base_now = start;
        s_ctx.ramp_on = true;
    }
    if (s_ctx.base_now < (target - step))      { s_ctx.base_now += step; }
    else if (s_ctx.base_now > (target + step)) { s_ctx.base_now -= step; }
    else                                       { s_ctx.base_now = target; }
    return s_ctx.base_now;
#else
    (void)start;
    return target;
#endif
}

static void motion_targets_driving(uint32_t now_ms)
{
    int8_t corr;
    float  base = (float)(s_ctx.dir * s_ctx.speed_mmps);
    float  diff;

    /* ramp towards the commanded speed (start from what the wheels do now) */
    base = motion_ramp(base, (s_ctx.meas_l + s_ctx.meas_r) * 0.5f);
    motion_update_steering(now_ms, &corr);
    diff = (float)corr * STEER_GAIN_MMPS;     /* + = steer right: left faster */
    {
        /* Never make the inner wheel stop or reverse while following the line: a slow or
           reversing wheel cannot be held by the PID (motor dead band) and trips the stall
           check. 2026-11-06: at 100 mm/s, full steer asked the right wheel for -27 mm/s. */
        float lim = STEER_MAX_FRACTION * ((base < 0.0f) ? -base : base);
        if (diff >  lim) { diff =  lim; }
        if (diff < -lim) { diff = -lim; }
    }
    s_ctx.tgt_l = base + diff;
    s_ctx.tgt_r = base - diff;
}

static void motion_targets_move(uint32_t now_ms, int32_t l, int32_t r)
{
    int32_t pl = (l - s_ctx.start_l) * s_ctx.dir;
    int32_t pr = (int32_t)((float)((r - s_ctx.start_r) * s_ctx.dir) * WHEEL_R_SCALE);   /* in left-wheel counts */
#if (MOTION_RIGHT_MOTOR_OK != 0)
    int32_t progress = (pl + pr) / 2;
    float   mismatch = (float)(pl - pr);
#else
    int32_t progress = pl;          /* left wheel only (temporary) */
    float   mismatch = 0.0f;
    (void)pr;
#endif
    float   v = (float)s_ctx.speed_mmps;

    if ((progress + motion_coast_counts()) >= s_ctx.target_counts)
    {
        motion_enter_stopping(now_ms);
        return;
    }
    if (((float)progress > ((float)s_ctx.target_counts * MOVE_SLOW_FRACTION)) &&
        (v > (float)SLOW_SPEED_MMPS))
    {
        v = (float)SLOW_SPEED_MMPS;
    }
#if (MOVE_START_MMPS > 0)
    {
        /* soft start: begin at MOVE_START_MMPS (above the dead band), then ramp */
        float now_v = (float)s_ctx.dir * (s_ctx.meas_l + s_ctx.meas_r) * 0.5f;
        if (now_v < (float)MOVE_START_MMPS) { now_v = (float)MOVE_START_MMPS; }
        if (v > (float)MOVE_START_MMPS) { v = motion_ramp(v, now_v); }
    }
#endif
    /* Straight-line correction: slow the wheel that is ahead */
    {
        float kp = STRAIGHT_KP;
        float steer = 0.0f;                        /* + = turn right (left wheel faster) */
        float h;
        if (s_ctx.hdg_on && s2_hook_heading(&h))
        {
            float e = h - s_ctx.hdg_ref;           /* + = robot has turned LEFT */
            if (e > 180.0f)  { e -= 360.0f; }
            if (e < -180.0f) { e += 360.0f; }
            s_ctx.hdg_err = e;
            steer = HEADING_KP * e;
            if (steer >  HEADING_MAX_MMPS) { steer =  HEADING_MAX_MMPS; }
            if (steer < -HEADING_MAX_MMPS) { steer = -HEADING_MAX_MMPS; }
            /* rotation = (v_r - v_l) / track, the same whichever way it drives,
               so a left twist is undone by a faster LEFT wheel even when reversing */
            kp = HEADING_COUNTS_KP;                /* the compass leads, counts only damp */
        }
        s_ctx.tgt_l = (float)s_ctx.dir * (v - (kp * mismatch)) + steer;
        s_ctx.tgt_r = (float)s_ctx.dir * (v + (kp * mismatch)) - steer;
    }
}

static void motion_targets_turn(uint32_t now_ms, int32_t l, int32_t r)
{
    int32_t pl = (l - s_ctx.start_l) * s_ctx.dir;      /* left forward on right turn */
    int32_t pr = (int32_t)((float)(-(r - s_ctx.start_r) * s_ctx.dir) * WHEEL_R_SCALE);   /* right backward on right turn; left-wheel counts */
#if (MOTION_RIGHT_MOTOR_OK != 0)
    int32_t progress = (pl + pr) / 2;
    float   mismatch = (float)(pl - pr);
#else
    /* Left wheel only: the robot pivots on the right wheel, so the left
     * wheel must travel twice the normal arc (temporary).               */
    int32_t progress = pl / 2;
    float   mismatch = 0.0f;
    (void)pr;
#endif
    float   v = (float)s_ctx.speed_mmps;

    if ((progress + motion_coast_counts()) >= s_ctx.target_counts)
    {
        motion_enter_stopping(now_ms);
        return;
    }
    if ((float)progress > ((float)s_ctx.target_counts * TURN_SLOW_FRACTION))
    {
        v = (float)SLOW_SPEED_MMPS;
    }
    s_ctx.tgt_l =  (float)s_ctx.dir * (v - (STRAIGHT_KP * mismatch));
    s_ctx.tgt_r = -(float)s_ctx.dir * (v + (STRAIGHT_KP * mismatch));
}

/* ------------------------------------------------------------------------- */
/* Self-test (statechart: "Encoder self-test failed" -> FAULT)              */
/* ------------------------------------------------------------------------- */
static void motion_step_init(uint32_t now_ms, int32_t l, int32_t r)
{
#if (MOTION_SELFTEST_ENABLE != 0)
    if (!s_ctx.selftest_running)
    {
        s_ctx.selftest_running = true;
        s_ctx.selftest_start_ms = now_ms;
        motion_mark_start();
        s_ctx.duty_l = SELFTEST_DUTY;
        s_ctx.duty_r = (MOTION_RIGHT_MOTOR_OK != 0) ? SELFTEST_DUTY : 0;
        return;
    }
    if ((uint32_t)(now_ms - s_ctx.selftest_start_ms) < SELFTEST_MS)
    {
        return;
    }
    s_ctx.selftest_running = false;
    s_ctx.duty_l = 0;
    s_ctx.duty_r = 0;
    if ((motion_abs(l - s_ctx.start_l) < SELFTEST_MIN_COUNTS) ||
        ((MOTION_RIGHT_MOTOR_OK != 0) &&
         (motion_abs(r - s_ctx.start_r) < SELFTEST_MIN_COUNTS)))
    {
        motion_enter_fault(MOTION_CAUSE_SELFTEST);
        return;
    }
#else
    (void)now_ms; (void)l; (void)r;
#endif
    s_ctx.state = MOTION_STATE_IDLE;
}

/* ------------------------------------------------------------------------- */
/* Control, stall check, status                                             */
/* ------------------------------------------------------------------------- */
static void motion_check_stall(void)
{
    if ((motion_abs(s_ctx.duty_l) >= STALL_MIN_DUTY) && (s_ctx.dl == 0))
    {
        s_ctx.stall_ms_l += MOTION_PERIOD_MS;
    }
    else
    {
        s_ctx.stall_ms_l = 0u;
    }

    if ((MOTION_RIGHT_MOTOR_OK != 0) &&
        (motion_abs(s_ctx.duty_r) >= STALL_MIN_DUTY) && (s_ctx.dr == 0))
    {
        s_ctx.stall_ms_r += MOTION_PERIOD_MS;
    }
    else
    {
        s_ctx.stall_ms_r = 0u;
    }

    if ((s_ctx.stall_ms_l >= STALL_TIMEOUT_MS) || (s_ctx.stall_ms_r >= STALL_TIMEOUT_MS))
    {
        motion_enter_fault(MOTION_CAUSE_STALL);
    }
}

static void motion_publish_status(void)
{
    motion_status_t st;

    st.state        = s_ctx.state;
    st.cause        = s_ctx.cause;
    st.steer_stale  = s_ctx.steer_stale;
    st.speed_l_mmps = (int16_t)motion_clamp((int32_t)s_ctx.meas_l, -1024, 1023);
    st.speed_r_mmps = (int16_t)motion_clamp((int32_t)s_ctx.meas_r, -1024, 1023);
    st.delta_l      = motion_to_i8(s_ctx.dl);
    st.delta_r      = motion_to_i8(s_ctx.dr);
    st.distance_mm  = (uint16_t)((uint32_t)s_ctx.distance_mm & 0xFFFFu);

    /* Sequence lock: the motion task is the only writer; readers on either
     * core retry if they saw an odd or changed sequence number.            */
    s_status_seq++;
    __sync_synchronize();
    s_status = st;
    __sync_synchronize();
    s_status_seq++;

    motion_status_hook(&st);
}

void motion_step(uint32_t now_ms, bool estop)
{
    int32_t l;
    int32_t r;
    float   dt_s = (float)MOTION_PERIOD_MS / 1000.0f;
    uint32_t bad;

    motion_debug_pin(true);

    /* 1. Measure: change in counts over the last 20 ms -> wheel speed */
    encoder_read(&l, &r);
    s_ctx.dl = l - s_ctx.last_l;
    s_ctx.dr = r - s_ctx.last_r;
    s_ctx.last_l = l;
    s_ctx.last_r = r;
    s_ctx.meas_l = ((float)s_ctx.dl * MM_PER_COUNT) / dt_s;
    s_ctx.meas_r = ((float)s_ctx.dr * MM_PER_COUNT * WHEEL_R_SCALE) / dt_s;
#if (MOTION_RIGHT_MOTOR_OK != 0)
    s_ctx.distance_mm += (((float)motion_abs(s_ctx.dl) + ((float)motion_abs(s_ctx.dr) * WHEEL_R_SCALE)) * 0.5f) * MM_PER_COUNT;
#else
    s_ctx.distance_mm += (float)motion_abs(s_ctx.dl) * MM_PER_COUNT;
#endif

    /* 2. Safety first: E-stop and encoder health override everything */
    bad = encoder_new_errors(ENC_LEFT);
#if (MOTION_RIGHT_MOTOR_OK != 0)
    bad += encoder_new_errors(ENC_RIGHT);
#else
    (void)encoder_new_errors(ENC_RIGHT);   /* read and ignore (temporary) */
#endif
    if (estop && (s_ctx.state != MOTION_STATE_FAULT))
    {
        motion_enter_fault(MOTION_CAUSE_ESTOP);
    }
    else if ((bad > ENC_MAX_BAD_PER_CYCLE) && (s_ctx.state != MOTION_STATE_FAULT) &&
             (++s_ctx.bad_cycles >= ENC_BAD_CYCLES))
    {
        motion_enter_fault(MOTION_CAUSE_ENCODER);
    }
    else
    {
        /* no override */
    }
    if (bad <= ENC_MAX_BAD_PER_CYCLE) { s_ctx.bad_cycles = 0u; }   /* the run of noisy cycles ended */

    /* 3. Commands (latest wins) */
    motion_drain_commands(now_ms);

    /* 4. State logic -> target wheel speeds */
    switch (s_ctx.state)
    {
        case MOTION_STATE_INIT:      motion_step_init(now_ms, l, r);        break;
        case MOTION_STATE_DRIVING:   motion_targets_driving(now_ms);        break;
        case MOTION_STATE_DIST_MOVE: motion_targets_move(now_ms, l, r);     break;
        case MOTION_STATE_TURNING:   motion_targets_turn(now_ms, l, r);     break;
        case MOTION_STATE_STOPPING:
            if ((motion_absf(s_ctx.meas_l) < STOPPED_SPEED_MMPS) &&
                (motion_absf(s_ctx.meas_r) < STOPPED_SPEED_MMPS))
            {
                s_ctx.stop_settle++;
            }
            else
            {
                s_ctx.stop_settle = 0u;
            }
            if ((s_ctx.stop_settle >= STOP_SETTLE_CYCLES) ||
                ((uint32_t)(now_ms - s_ctx.stop_start_ms) >= STOP_TIMEOUT_MS))
            {
                motion_finish_move();
            }
            break;
        case MOTION_STATE_IDLE:
        case MOTION_STATE_FAULT:
        default:
            break;
    }

    /* 5. Output */
    if ((s_ctx.state == MOTION_STATE_DIST_MOVE) && (s_ctx.preload_cycles > 0u))
    {
        /* gearbox take-up: same small duty on both wheels, no PID yet */
        s_ctx.duty_l = s_ctx.dir * MOVE_PRELOAD_DUTY;
        s_ctx.duty_r = s_ctx.dir * MOVE_PRELOAD_DUTY;
        s_ctx.preload_cycles--;
    }
    else if (motion_is_active(s_ctx.state))
    {
        s_ctx.duty_l = (int32_t)pid_update(&s_pid_l, s_ctx.tgt_l - s_ctx.meas_l, dt_s, SPEED_FF * s_ctx.tgt_l);
#if (MOTION_RIGHT_MOTOR_OK != 0)
        s_ctx.duty_r = (int32_t)pid_update(&s_pid_r, s_ctx.tgt_r - s_ctx.meas_r, dt_s, SPEED_FF * s_ctx.tgt_r);
#else
        s_ctx.duty_r = 0;                   /* right motor not fitted (temporary) */
#endif
        motion_check_stall();
    }
    else if (s_ctx.state != MOTION_STATE_INIT)
    {
        s_ctx.duty_l = 0;     /* IDLE, STOPPING (brake) and FAULT: motors off */
        s_ctx.duty_r = 0;
    }
    else
    {
        /* INIT: self-test duty already set */
    }

    if (s_ctx.state == MOTION_STATE_FAULT)
    {
        motor_stop_all();
    }
    else
    {
        motor_set(MOTOR_LEFT, s_ctx.duty_l);
        motor_set(MOTOR_RIGHT, s_ctx.duty_r);
    }

    /* 6. Report */
    motion_publish_status();

    s_cycles++;
    motion_debug_pin(false);
}

/* ------------------------------------------------------------------------- */
/* Task                                                                     */
/* ------------------------------------------------------------------------- */
void motion_task(int32_t stacd, void *p_exinf)
{
    uint32_t next;

    (void)stacd;
    (void)p_exinf;

#ifndef HOST_TEST
    /* This task is pinned to core 1, so the shared GPIO interrupt is
     * enabled on core 1 and always runs there.                           */
    (void)gpio_irq_enable();
#endif
    next = os_now_ms();

    for (;;)
    {
        uint32_t now;
        int32_t  wait;
        bool     estop;

        next += MOTION_PERIOD_MS;
        now = os_now_ms();
        wait = (int32_t)(next - now);
        if (wait < 0)
        {
            wait = 0;
            next = now;      /* overran: resynchronise instead of bursting */
        }

        if (s_ctx.state == MOTION_STATE_FAULT)
        {
            /* E-stop flag may stay set: plain delay so we never busy-loop */
            os_sleep_ms((uint32_t)wait);
            estop = os_flag_poll(s_links.flg_estop, ESTOP_BIT);
        }
        else
        {
            /* Sleep until the next 20 ms tick, but wake at once on E-stop */
            estop = os_flag_wait_any(s_links.flg_estop, ESTOP_BIT, (uint32_t)wait);
            if (estop)
            {
                motor_stop_all();          /* fastest possible path to PWM 0 */
                next = os_now_ms();
            }
        }

        {
            uint32_t us = os_now_us();
            if (s_per_reset)
            {
                s_per_min = 0xFFFFFFFFu; s_per_max = 0u; s_per_sum = 0u; s_per_n = 0u;
                s_busy_max = 0u; s_busy_sum = 0u;
                s_per_reset = false;
            }
            else if (!estop)                 /* an E-stop wake is early on purpose */
            {
                uint32_t p = us - s_per_last;
                if (p < s_per_min) { s_per_min = p; }
                if (p > s_per_max) { s_per_max = p; }
                s_per_sum += p;
                s_per_n++;
            }
            s_per_last = us;
        }
        motion_step(os_now_ms(), estop);
        motion_watchdog_checkin();
        {
            uint32_t busy = os_now_us() - s_per_last;    /* work done this cycle */
            if (busy > s_busy_max) { s_busy_max = busy; }
            s_busy_sum += busy;
        }
    }
}

/* ------------------------------------------------------------------------- */
/* Public API                                                               */
/* ------------------------------------------------------------------------- */
static bool motion_send(const motion_cmd_t *p_cmd)
{
#if (MOTION_USE_PACKED != 0)
    motion_msg_t msg = motion_cmd_pack(p_cmd);
#else
    motion_msg_t msg = *p_cmd;
#endif
    return os_mbf_send(s_links.mbf_cmd, &msg, (uint32_t)sizeof(msg), CMD_SEND_TIMEOUT_MS);
}

static bool motion_send_simple(motion_cmd_type_t type, int32_t speed, int32_t dist, int32_t angle, bool rev)
{
    motion_cmd_t cmd;

    cmd.type = (int32_t)type;
    cmd.speed_mmps = speed;
    cmd.distance_mm = dist;
    cmd.angle_deg = angle;
    cmd.reverse = rev ? 1 : 0;
    return motion_send(&cmd);
}

bool motion_drive(int32_t speed_mmps, bool reverse)
{
    return motion_send_simple(MOTION_CMD_DRIVE, speed_mmps, 0, 0, reverse);
}

bool motion_move(int32_t distance_mm, int32_t speed_mmps, bool reverse)
{
    return motion_send_simple(reverse ? MOTION_CMD_MOVE_BWD : MOTION_CMD_MOVE_FWD,
                              speed_mmps, distance_mm, 0, false);
}

bool motion_turn_left(int32_t angle_deg)
{
    return motion_send_simple(MOTION_CMD_TURN_LEFT, 0, 0, angle_deg, false);
}

bool motion_turn_right(int32_t angle_deg)
{
    return motion_send_simple(MOTION_CMD_TURN_RIGHT, 0, 0, angle_deg, false);
}

bool motion_uturn(void)
{
    return motion_turn_left(180);
}

bool motion_stop(void)
{
    return motion_send_simple(MOTION_CMD_STOP, 0, 0, 0, false);
}

float motion_heading_error(void) { return s_ctx.hdg_on ? s_ctx.hdg_err : 0.0f; }
bool  motion_heading_used(void)  { return s_ctx.hdg_on; }

bool motion_reset(void)
{
    return motion_send_simple(MOTION_CMD_RESET, 0, 0, 0, false);
}

void motion_get_status(motion_status_t *p_out)
{
    uint32_t seq;

    do {
        seq = s_status_seq;
        __sync_synchronize();
        *p_out = s_status;
        __sync_synchronize();
    } while (((seq & 1u) != 0u) || (seq != s_status_seq));
}

/* S2-M06: loop period statistics since the last reset (us) */
void motion_period_reset(void) { s_per_reset = true; }
void motion_busy_stats(uint32_t *p_max, uint32_t *p_avg)
{
    uint32_t n = s_per_n;
    *p_max = s_busy_max;
    *p_avg = (n > 0u) ? (s_busy_sum / n) : 0u;
}

void motion_period_stats(uint32_t *p_min, uint32_t *p_max, uint32_t *p_avg, uint32_t *p_n)
{
    uint32_t n = s_per_n;
    *p_min = s_per_min;
    *p_max = s_per_max;
    *p_avg = (n > 0u) ? (s_per_sum / n) : 0u;
    *p_n   = n;
}

uint32_t motion_cycle_count(void)
{
    return s_cycles;
}

void motion_reset_for_test(void)
{
    s_ctx = (motion_ctx_t){ 0 };
    s_ctx.state = MOTION_STATE_INIT;
    pid_init(&s_pid_l, SPEED_KP, SPEED_KI, SPEED_KD, SPEED_I_LIMIT, (float)DUTY_MAX);
    pid_init(&s_pid_r, SPEED_KP, SPEED_KI, SPEED_KD, SPEED_I_LIMIT, (float)DUTY_MAX);
}

void motion_core_init(const motion_links_t *p_links)
{
    s_links = *p_links;
    motion_reset_for_test();
    motor_init();
    encoder_init();
    encoder_read(&s_ctx.last_l, &s_ctx.last_r);
}
