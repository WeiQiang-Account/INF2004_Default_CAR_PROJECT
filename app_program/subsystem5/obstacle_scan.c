/* obstacle_scan.c - "Ultrasonic Scan" task (Buddy 5), statechart 6.2.
 * States: INITIALISING -> FORWARD_MONITOR <-> SCANNING(coarse->fine->profile->plan)
 *         -> BYPASS_GUARD (while VC drives round the obstacle), SENSOR_FAULT.
 * It only SUGGESTS an action; the Vehicle Controller commands Motion Control
 * (single motor owner). Full scans only start when VC reports v = 0 (flg_scan_req). */
#include "../subsystem2/drv_gpio_irq.h"
#include "subsystem5.h"
#include "drv_ultrasonic.h"
#include "s5_port.h"

enum { ST_INIT = 0, ST_MONITOR, ST_SCANNING, ST_GUARD, ST_FAULT };      /* 3 bits */
enum { STG_COARSE = 0, STG_FINE, STG_PROFILE, STG_PLAN };               /* 2 bits */

/* FR20: 5 angles spread over the safe range (s5_config.h limits), e.g. 45..135 */
static const int16_t COARSE_ANG[S5_COARSE_N] = {
    S5_SERVO_LIMIT_MIN,
    (S5_SERVO_LIMIT_MIN + S5_SERVO_CENTRE_DEG) / 2,
    S5_SERVO_CENTRE_DEG,
    (S5_SERVO_CENTRE_DEG + S5_SERVO_LIMIT_MAX) / 2,
    S5_SERVO_LIMIT_MAX };

s5_scan_stats_t g_s5_scan;
static volatile uint8_t  g_status;
static volatile uint16_t g_last_dist = S5_DIST_NO_ECHO;
static int  g_state = ST_INIT, g_attempt, g_latched, g_hit, g_nosensor, g_guard_deg, g_clear;
static uint8_t g_stage, g_action;

uint8_t  s5_status_byte(void)       { return g_status; }
uint16_t s5_last_distance_mm(void)  { return g_last_dist; }

static void set_status(int state, int stage, int action)
{
    g_state = state; g_stage = (uint8_t)stage; g_action = (uint8_t)action;
    g_status = S5_STATUS_PACK(state, stage, action);
}
static uint32_t now_ms(void) { SYSTIM t; (void)tk_get_tim(&t); return (uint32_t)t.lo; }

static volatile int g_servo_now = S5_SERVO_CENTRE_DEG;
static volatile int g_manual_deg = -1;                            /* -1 = automatic */

/* Move the servo in small steps at S5_SERVO_DEG_PER_S (a jump made the sensor
   swing and shake, 2026-11-06), then wait for the rest of the settle time. */
static void go(int deg, int settle_ms)
{
    int cur = g_servo_now, travel = 0;
    if (deg < S5_SERVO_LIMIT_MIN) deg = S5_SERVO_LIMIT_MIN;       /* hard limits (s5_config.h) */
    if (deg > S5_SERVO_LIMIT_MAX) deg = S5_SERVO_LIMIT_MAX;
    int step_ms = (S5_SERVO_STEP_DEG * 1000) / S5_SERVO_DEG_PER_S;
    if (step_ms < 1) step_ms = 1;
    while (cur != deg) {
        int d = deg - cur;
        if (d >  S5_SERVO_STEP_DEG) d =  S5_SERVO_STEP_DEG;
        if (d < -S5_SERVO_STEP_DEG) d = -S5_SERVO_STEP_DEG;
        cur += d;
        servo_set_deg(cur);
        g_servo_now = cur;
        tk_dly_tsk(step_ms);
        travel += step_ms;
    }
    servo_set_deg(deg);
    g_servo_now = deg;
    settle_ms -= travel;
    if (settle_ms < S5_SERVO_MIN_SETTLE_MS) settle_ms = S5_SERVO_MIN_SETTLE_MS;
    tk_dly_tsk(settle_ms);
}

/* dashboard servo test (via S1). Any task may call it: only the scan task moves the servo. */
void s5_servo_manual(int deg)
{
    if (deg >= 0) {
        if (deg < S5_SERVO_LIMIT_MIN) deg = S5_SERVO_LIMIT_MIN;
        if (deg > S5_SERVO_LIMIT_MAX) deg = S5_SERVO_LIMIT_MAX;
    }
    g_manual_deg = (deg < 0) ? -1 : deg;
    s5_scan_request(S5_REQ_SERVO);
}
int s5_servo_angle(void)     { return g_servo_now; }
int s5_servo_is_manual(void) { return g_manual_deg >= 0; }

/* returns 1 if the sensor is now considered faulty */
static int note_status(us_status_t st)
{
    if (st == US_NO_SENSOR) g_nosensor++; else g_nosensor = 0;
    return g_nosensor >= S5_NOSENSOR_LIMIT;
}

static void enter_fault(void)
{
    go(S5_SERVO_CENTRE_DEG, 100);
    set_status(ST_FAULT, 0, 0);
    s5_fault_set(1);
    vc_post(VC_EV_MAKE(VC_EV_SENSOR_FAULT, 1));
    S5_PRINT("[S5] SENSOR FAULT: no echo edge on GP%d (check divider/wiring)\n", S5_PIN_ECHO);
}

static void enter_monitor(void)
{
    go(S5_SERVO_CENTRE_DEG, S5_SETTLE_COARSE_MS);
    g_hit = 0;
    set_status(ST_MONITOR, 0, 0);
}

/* ---- FORWARD_MONITOR: one ping straight ahead every 60 ms --------------- */
static void monitor_tick(void)
{
    uint16_t d;
    us_status_t st = us_ping(&d);
    if (note_status(st)) { enter_fault(); return; }
    g_last_dist = d;
    if (d != S5_DIST_NO_ECHO && d < S5_OBSTACLE_TRIGGER_MM) {
        if (++g_hit >= 2 && !g_latched) {              /* 2 in a row rejects a single bad echo */
            g_latched = 1;
            vc_post(VC_EV_MAKE(VC_EV_OBSTACLE, d));
            S5_PRINT("[S5] obstacle at %u mm\n", d);
        }
    } else {
        g_hit = 0;
        /* 2026-11-06: the latch stayed set forever after an obstacle was seen while
           the robot was NOT running (the VC ignores it then), so later obstacles
           were never reported and the robot drove into them. Re-arm once the
           path ahead has been clear for a few pings.                          */
        if ((d == S5_DIST_NO_ECHO) || (d >= S5_OBSTACLE_TRIGGER_MM + S5_UNLATCH_MARGIN_MM)) {
            if (g_latched && (++g_clear >= S5_UNLATCH_PINGS)) { g_latched = 0; g_clear = 0; }
        } else {
            g_clear = 0;
        }
    }
}

/* ---- BYPASS_GUARD: watch the obstacle while VC drives past it ------------- */
static void guard_tick(void)
{
    uint16_t d;
    us_status_t st = us_ping(&d);
    if (note_status(st)) { enter_fault(); return; }
    if (d != S5_DIST_NO_ECHO && d < S5_GUARD_TOO_CLOSE_MM && !g_latched) {
        g_latched = 1;
        vc_post(VC_EV_MAKE(VC_EV_TOO_CLOSE, d));
    }
}

static uint16_t read2(int *fault)
{
    uint16_t d; us_status_t st = us_ping2(&d);
    if (note_status(st)) *fault = 1;
    return d;
}
static uint16_t read1(int *fault)
{
    uint16_t d; us_status_t st = us_ping(&d);
    if (note_status(st)) *fault = 1;
    return d;
}

/* ---- FINE SCAN: walk outwards from the nearest hit in 5 deg steps until the
 *      readings leave the obstacle (forward-depth test) or the servo limit. ---- */
static void fine_scan(s5_scan_data_t *sd, int *fault)
{
    int a0 = sd->closest.angle;
    sd->have_fine = 1;
    sd->left_edge = sd->right_edge = sd->closest;

    for (int a = a0 + 5; !*fault; a += 5) {                       /* left  */
        if (a > S5_SERVO_MAX_DEG) { sd->left_clipped = 1; break; }
        go(a, S5_SETTLE_FINE_MS);
        uint16_t d = read1(fault);
        s5_scan_note(sd, a, d);
        s5_reading_t r = { (int16_t)a, d };
        if (s5_is_obstacle(sd, a, d)) sd->left_edge = r; else { sd->left_next = r; break; }
    }
    for (int a = a0 - 5; !*fault; a -= 5) {                       /* right */
        if (a < S5_SERVO_MIN_DEG) { sd->right_clipped = 1; break; }
        go(a, a == a0 - 5 ? S5_SETTLE_FIRST_MS : S5_SETTLE_FINE_MS);   /* long hop back across the obstacle */
        uint16_t d = read1(fault);
        s5_scan_note(sd, a, d);
        s5_reading_t r = { (int16_t)a, d };
        if (s5_is_obstacle(sd, a, d)) sd->right_edge = r; else { sd->right_next = r; break; }
    }
}

static void post_profile(const s5_profile_t *p)
{
    uint32_t w[2];
    s5_profile_pack(p, w, VC_EV_PROFILE);
    vc_post2(w[0], w[1]);
}

/* ---- SCANNING --------------------------------------------------------------- */
static void do_scan(int is_rescan)
{
    if (is_rescan) {
        if (g_attempt >= S5_MAX_RESCAN) {                         /* 3 rescans used: stop looping */
            vc_post(VC_EV_MAKE(VC_EV_NO_PATH, 0));
            g_attempt = 0;
            enter_monitor();
            return;
        }
        g_attempt++;
    } else g_attempt = 0;

    int fault = 0;
    static s5_scan_data_t sd;                                     /* static: task stack is only 2 KB */
    static s5_profile_t prof;
    s5_scan_init(&sd);
    uint32_t t0 = now_ms();

    set_status(ST_SCANNING, STG_COARSE, 0);
    for (int i = 0; i < S5_COARSE_N && !fault; i++) {             /* FR20 */
        go(COARSE_ANG[i], i == 0 ? S5_SETTLE_FIRST_MS : S5_SETTLE_COARSE_MS);
        uint16_t d = read2(&fault);
        sd.coarse[i].angle = COARSE_ANG[i]; sd.coarse[i].dist = d;
        s5_scan_note(&sd, COARSE_ANG[i], d);
    }
    uint32_t coarse_ms = now_ms() - t0;
    if (fault) { enter_fault(); return; }
    S5_PRINT("[S5] coarse scan %u ms (limit 1200): %u %u %u %u %u mm\n", (unsigned)coarse_ms,
              sd.coarse[0].dist, sd.coarse[1].dist, sd.coarse[2].dist, sd.coarse[3].dist, sd.coarse[4].dist);

    if (sd.closest.dist == S5_DIST_NO_ECHO) {                     /* nothing within 30 cm */
        g_latched = 0;                                            /* allow the next obstacle to be reported */
        s5_make_profile(&sd, &prof);                              /* found = 0, CONTINUE  */
        prof.action = S5_ACT_CONTINUE;
        set_status(ST_SCANNING, STG_PLAN, prof.action);
        g_s5_scan.coarse_ms = coarse_ms; g_s5_scan.total_ms = coarse_ms; g_s5_scan.action = prof.action;
        g_s5_scan.closest_mm = 0; g_s5_scan.width_mm = 0;
        g_s5_scan.clr_l_mm = 0; g_s5_scan.clr_r_mm = 0; g_s5_scan.count++;
        post_profile(&prof);
        enter_monitor();
        return;
    }

    set_status(ST_SCANNING, STG_FINE, 0);                         /* FR21 */
    fine_scan(&sd, &fault);
    if (fault) { enter_fault(); return; }

    set_status(ST_SCANNING, STG_PROFILE, 0);                      /* FR22 */
    s5_make_profile(&sd, &prof);

    set_status(ST_SCANNING, STG_PLAN, 0);                         /* FR23 */
    prof.action = s5_plan_action(&prof, g_attempt);
    set_status(ST_SCANNING, STG_PLAN, prof.action);
    S5_PRINT("[S5] profile: closest %u mm, width %u, xl %d xr %d, clr L %u R %u -> action %d (total %u ms)\n",
              prof.closest_mm, prof.width_mm, prof.xl_mm, prof.xr_mm, prof.clr_l_mm, prof.clr_r_mm,
              (int)prof.action, (unsigned)(now_ms() - t0));
    g_s5_scan.coarse_ms = coarse_ms; g_s5_scan.total_ms = now_ms() - t0;
    g_s5_scan.action = prof.action; g_s5_scan.closest_mm = prof.closest_mm; g_s5_scan.width_mm = prof.width_mm;
    g_s5_scan.clr_l_mm = prof.clr_l_mm; g_s5_scan.clr_r_mm = prof.clr_r_mm;
    g_s5_scan.count++;
    post_profile(&prof);
    go(S5_SERVO_CENTRE_DEG, S5_SETTLE_COARSE_MS);
    set_status(ST_MONITOR, 0, prof.action);                       /* latched: no repeat OBSTACLE event */
}

EXPORT void s5_scan_task(INT stacd, void *exinf)
{
    (void)stacd; (void)exinf;
    set_status(ST_INIT, 0, 0);
    uint16_t d;
    for (int w = 0; (w < 100) && !gpio_irq_is_enabled(); w++) tk_dly_tsk(10);   /* motion_task (core 1) enables IRQ 13 */
#if S5_SERVO_BOOT_WIGGLE
    /* visible check that the servo follows the code: look left, right, then straight ahead */
    S5_PRINT("[S5] servo check: left, right, centre (%d deg)\n", S5_SERVO_CENTRE_DEG);
    go(S5_SERVO_CENTRE_DEG + 30, 400);
    go(S5_SERVO_CENTRE_DEG - 30, 400);
    go(S5_SERVO_CENTRE_DEG, 300);
#endif
    for (int i = 0; i < 3; i++) {                                 /* test ping (INITIALISING) */
        go(S5_SERVO_CENTRE_DEG, S5_SETTLE_FIRST_MS);
        us_status_t st = us_ping(&d);
        if (st != US_NO_SENSOR) { g_nosensor = 0; break; }
        g_nosensor++;
    }
    if (g_nosensor >= S5_NOSENSOR_LIMIT) enter_fault(); else enter_monitor();

    for (;;) {
        UINT ptn = 0;
        TMO  tmo = (g_state == ST_FAULT) ? S5_FAULT_RETRY_MS : S5_MONITOR_MS;
        ER er = tk_wai_flg(flg_scan_req, S5_REQ_ALL, TWF_ORW | TWF_BITCLR, &ptn, tmo);

        if (er == E_TMOUT) {
            if      (g_state == ST_MONITOR) monitor_tick();
            else if (g_state == ST_GUARD)   guard_tick();
            else if (g_state == ST_FAULT) {                       /* retry every 500 ms */
                us_status_t st = us_ping(&d);
                if (st != US_NO_SENSOR) {
                    g_nosensor = 0;
                    s5_fault_set(0);
                    vc_post(VC_EV_MAKE(VC_EV_SENSOR_FAULT, 0));
                    enter_monitor();
                }
            }
            continue;
        }
        if (er != E_OK) continue;
        if (g_state == ST_FAULT) continue;                        /* ignore requests while faulty */

        if (ptn & S5_REQ_SERVO) {                                 /* dashboard servo test */
            if (g_manual_deg >= 0) {
                go(g_manual_deg, 50);                             /* stays there; monitor pings at this angle */
                S5_PRINT("[S5] servo manual: %d deg (+ trim %d)\n", g_manual_deg, (int)S5_SERVO_TRIM_DEG);
            } else {
                enter_monitor();                                  /* automatic: straight ahead */
                S5_PRINT("[S5] servo automatic\n");
            }
            if ((ptn & ~S5_REQ_SERVO) == 0u) continue;
        }
        if (ptn & (S5_REQ_SCAN | S5_REQ_RESCAN | S5_REQ_GUARD_R | S5_REQ_GUARD_L)) {
            g_manual_deg = -1;                                    /* the mission needs the servo back */
        }
        if (ptn & S5_REQ_BYPASS_DONE) {                           /* line re-acquired */
            g_attempt = 0; g_latched = 0;
            enter_monitor();
            continue;
        }
        if (ptn & S5_REQ_GUARD_OFF) { g_latched = 0; enter_monitor(); }
        if (ptn & (S5_REQ_GUARD_R | S5_REQ_GUARD_L)) {
            g_guard_deg = (ptn & S5_REQ_GUARD_R) ? S5_GUARD_RIGHT_DEG : S5_GUARD_LEFT_DEG;
            go(g_guard_deg, S5_SETTLE_COARSE_MS);
            g_latched = 0;
            set_status(ST_GUARD, 0, 0);
        }
        if      (ptn & S5_REQ_RESCAN) do_scan(1);
        else if (ptn & S5_REQ_SCAN)   do_scan(0);
    }
}
