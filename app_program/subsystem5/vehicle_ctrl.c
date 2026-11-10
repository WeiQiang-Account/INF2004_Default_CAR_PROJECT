/* vehicle_ctrl.c - Vehicle Controller (Team task, implemented by Buddy 5).
 * The ONLY task that makes mission decisions (system statechart, section 5):
 *   top level (3 bits): INIT, READY, RUNNING, PAUSED, FAULT, COMPLETE
 *   RUNNING / Region A (2 bits): LINE_FOLLOW, BARCODE_TURN, OBSTACLE_AVOID, LINE_RECOVERY
 *   RUNNING / Region B: terrain flag (hump), driven by IMU events
 * It handles one event at a time from mbf_vc_events and drives Motion Control
 * only through the motion API (single motor owner).
 *
 * Bypass (box-shaped), side = +1 left / -1 right, obstacle on the opposite side:
 *   B1 turn side*90 -> B2 move lat -> B3 turn -side*90 -> B4 move fwd (guard on)
 *   -> B5 turn -side*90 (face the line) -> B6A move lat-TOL -> B6B creep until the
 *   line is seen inside the +/-TOL window -> B7 turn side*90 -> LINE_FOLLOW.        */
#include "subsystem5.h"
#include "s5_port.h"
#include "s5_hw.h"
#include "vc_events.h"

enum { VC_INIT = 0, VC_READY, VC_RUNNING, VC_PAUSED, VC_FAULT, VC_COMPLETE };
enum { NAV_LINE = 0, NAV_BARCODE, NAV_OBSTACLE, NAV_RECOVERY };
enum { OA_IDLE = 0, OA_WAIT_STOP, OA_WAIT_PROFILE, OA_REVERSE,
       B1_TURN, B2_LAT, B3_TURN, B4_FWD, B5_TURN, B6A_MOVE, B6B_SEEK, B6C_STOP, B7_ALIGN, BC_TURN };
enum { CAUSE_NONE = 0, CAUSE_INIT, CAUSE_IMPACT, CAUSE_LINE_LOST, CAUSE_TIMEOUT, CAUSE_SENSOR, CAUSE_RECOVERY, CAUSE_MOTION };

EXPORT ID mbf_vc_events;
static volatile uint32_t g_dropped;
static volatile uint8_t  g_status;

static struct {
    uint8_t  state, nav, hist, step;
    int8_t   side;                 /* +1 left, -1 right */
    uint8_t  on_hump, degraded, cause, use_rescan;
    uint16_t hump_peak_mm;
    uint32_t t_state, t_step, t_seek, lost_since, found_since, rescan_at, buzz_off_at;
    int      lat_mm, fwd_mm;
    uint8_t  seek_hits, btn_start_cnt, btn_stop_cnt;
} vc;

static uint32_t now_ms(void) { SYSTIM t; (void)tk_get_tim(&t); return (uint32_t)t.lo; }
static void update_status(void)
{
    g_status = (uint8_t)((vc.state & 7u) | ((vc.nav & 3u) << 3) | ((vc.on_hump & 1u) << 5) | ((vc.degraded & 1u) << 6));
}
uint8_t  vc_status_byte(void)      { return g_status; }
uint32_t vc_dropped_events(void)   { return g_dropped; }
uint8_t  vc_fault_cause(void)      { return vc.cause; }   /* why the last FAULT happened (CAUSE_*) */

/* ---------------------------------------------------------------- posting API */
int vc_post2(uint32_t w0, uint32_t w1)
{
    uint32_t m[2] = { w0, w1 };
    UINT n = (VC_EV_TYPE(w0) == VC_EV_PROFILE) ? 8 : 4;
    if (tk_snd_mbf(mbf_vc_events, m, n, TMO_POL) != E_OK) { g_dropped++; return -1; }
    return 0;
}
int vc_post(uint32_t w0) { return vc_post2(w0, 0); }

EXPORT void s5_scan_request(UINT bits) { (void)tk_set_flg(flg_scan_req, bits); }

/* ---------------------------------------------------------------- buzzer (GP22, PWM slice 3) */
static void buzz(unsigned ms_on)
{
    s5_pwm_level(S5_PIN_BUZZER, 250u);               /* 2 kHz, 50 % */
    vc.buzz_off_at = now_ms() + ms_on;
}
static void buzz_tick(void)
{
    if (vc.buzz_off_at && (int32_t)(now_ms() - vc.buzz_off_at) >= 0) { s5_pwm_level(S5_PIN_BUZZER, 0u); vc.buzz_off_at = 0; }
}

/* ---------------------------------------------------------------- helpers */
static void cruise(void) { s5_motion_drive(vc.degraded ? S5_CRUISE_DEGRADED_MM_S : S5_CRUISE_MM_S); }

static void to_state(int s) { vc.state = (uint8_t)s; vc.t_state = now_ms(); update_status(); }

static void enter_fault(int cause)
{
    s5_motion_stop();
    s5_scan_request(S5_REQ_GUARD_OFF);
    vc.cause = (uint8_t)cause; vc.step = OA_IDLE; vc.lost_since = 0; vc.found_since = 0; vc.rescan_at = 0;
    to_state(VC_FAULT);
    buzz(1000);
    S5_PRINT("[VC] FAULT cause=%d\n", cause);
}

static void pause_mission(const char *why)
{
    vc.hist = vc.nav;                                /* shallow history (H) */
    s5_motion_stop();
    s5_scan_request(S5_REQ_GUARD_OFF);
    vc.lost_since = 0; vc.found_since = 0; vc.rescan_at = 0;
    to_state(VC_PAUSED);
    buzz(300);
    S5_PRINT("[VC] PAUSED (%s), history nav=%d\n", why, vc.hist);
}

static void nav_to(int n) { if (n != vc.nav) { vc.lost_since = 0; vc.found_since = 0; } vc.nav = (uint8_t)n; update_status(); }   /* fresh line-lost timer per mode */

static void step_to(int s) { vc.step = (uint8_t)s; vc.t_step = now_ms(); }

static void finish_bypass(void)
{
    s5_scan_request(S5_REQ_BYPASS_DONE);             /* S5: reset attempts, unlatch, back to monitor */
    nav_to(NAV_LINE); vc.step = OA_IDLE;
    cruise();
    S5_PRINT("[VC] line re-acquired, %u ms since recovery start\n", (unsigned)(now_ms() - vc.t_seek));
}

/* ---------------------------------------------------------------- obstacle sequence */
static void oa_begin(void)                           /* FR23 handoff: stop first, scan only at v = 0 */
{
    nav_to(NAV_OBSTACLE);
    vc.use_rescan = 0;
    s5_motion_clear_done();
    s5_motion_stop();
    step_to(OA_WAIT_STOP);
}

static void issue_step(int s)
{
    s5_motion_clear_done();
    step_to(s);
    switch (s) {
    case B1_TURN:  s5_motion_turn( vc.side * 90);              break;
    case B2_LAT:   s5_motion_move(vc.lat_mm);                  break;
    case B3_TURN:  s5_motion_turn(-vc.side * 90);              break;
    case B4_FWD:   s5_scan_request(vc.side > 0 ? S5_REQ_GUARD_R : S5_REQ_GUARD_L);   /* obstacle is on the other side */
                   s5_motion_move(vc.fwd_mm);                  break;
    case B5_TURN:  s5_scan_request(S5_REQ_GUARD_OFF);
                   s5_motion_turn(-vc.side * 90);              break;
    case B6A_MOVE: s5_motion_move(vc.lat_mm - S5_LINE_TOL_MM); break;
    case B7_ALIGN: s5_motion_turn( vc.side * 90);              break;
    case OA_REVERSE: s5_motion_move(-S5_REVERSE_MM);           break;
    default: break;
    }
}

static void start_seek(void)
{
    nav_to(NAV_RECOVERY);
    step_to(B6B_SEEK);
    vc.t_seek = now_ms();
    vc.seek_hits = 0;
    s5_motion_drive(S5_CREEP_MM_S);
}

static void on_move_done(void)
{
    switch (vc.step) {
    case OA_WAIT_STOP:                               /* v = 0 confirmed: now the sweep is trustworthy */
        s5_scan_request(vc.use_rescan ? S5_REQ_RESCAN : S5_REQ_SCAN);
        step_to(OA_WAIT_PROFILE); break;
    case OA_REVERSE:
        s5_scan_request(S5_REQ_RESCAN);
        step_to(OA_WAIT_PROFILE); break;
    case B1_TURN: issue_step(B2_LAT);  break;
    case B2_LAT:  issue_step(B3_TURN); break;
    case B3_TURN: issue_step(B4_FWD);  break;
    case B4_FWD:  issue_step(B5_TURN); break;
    case B5_TURN:
        nav_to(NAV_RECOVERY); vc.t_seek = now_ms();
        if (vc.lat_mm - S5_LINE_TOL_MM > 20) issue_step(B6A_MOVE); else start_seek();
        break;
    case B6A_MOVE: start_seek(); break;
    case B6C_STOP: issue_step(B7_ALIGN); break;     /* turn only after the stop is acknowledged */
    case B7_ALIGN: finish_bypass(); break;
    case BC_TURN:  nav_to(NAV_LINE); vc.step = OA_IDLE; cruise(); break;
    default: break;
    }
}

static void on_profile(const uint32_t w[2])
{
    if (vc.step != OA_WAIT_PROFILE) return;          /* stale */
    s5_profile_t p; s5_profile_unpack(w, &p);
    S5_PRINT("[VC] profile: action=%d closest=%u width=%u clrL=%u clrR=%u\n",
              (int)p.action, p.closest_mm, p.width_mm, p.clr_l_mm, p.clr_r_mm);
    switch (p.action) {
    case S5_ACT_CONTINUE:
        finish_bypass(); break;                      /* nothing in the way after all */
    case S5_ACT_TURN_LEFT:
    case S5_ACT_TURN_RIGHT: {
        int left = (p.action == S5_ACT_TURN_LEFT);
        vc.side   = left ? 1 : -1;
        vc.lat_mm = s5_bypass_lateral_mm(&p, left);
        vc.fwd_mm = s5_bypass_forward_mm(&p);
        S5_PRINT("[VC] bypass %s: lateral %d mm, forward %d mm\n", left ? "LEFT" : "RIGHT", vc.lat_mm, vc.fwd_mm);
        issue_step(B1_TURN); break; }
    case S5_ACT_REVERSE_RETRY:
        issue_step(OA_REVERSE); break;
    case S5_ACT_STOP:
    default:
        vc.use_rescan = 1; vc.rescan_at = now_ms() + 500;
        vc.step = OA_IDLE; break;                    /* blocked: look again in 500 ms (bounded by S5's rescan limit) */
    }
}

/* ---------------------------------------------------------------- mission control */
static void start_run(void)
{
    if (s5_servo_is_manual()) s5_servo_manual(-1);           /* dashboard servo test: back to automatic */
    s5_scan_request(S5_REQ_GUARD_OFF);                       /* fresh start: re-arm obstacle detection */
    s5_motion_reset();   /* 2026-11-07: an E-stop while READY (robot picked up / knocked) left S2 in
                            FAULT, which ignores DRIVE, so START did nothing. No-op if S2 is fine. */
    to_state(VC_RUNNING); nav_to(NAV_LINE); vc.step = OA_IDLE; vc.lost_since = 0; vc.found_since = 0;
    cruise();
    S5_PRINT("[VC] RUNNING\n");
}

static void resume(void)
{
    if (s5_servo_is_manual()) s5_servo_manual(-1);
    s5_motion_reset();                               /* same as start_run(): clear an S2 E-stop seen while PAUSED */
    to_state(VC_RUNNING);
    switch (vc.hist) {
    case NAV_OBSTACLE:                               /* restart the scan from standstill */
        nav_to(NAV_OBSTACLE); vc.use_rescan = 1; s5_scan_request(S5_REQ_RESCAN); step_to(OA_WAIT_PROFILE); break;
    case NAV_RECOVERY:
        start_seek(); break;
    default:                                         /* LINE_FOLLOW, or a barcode turn that was interrupted */
        nav_to(NAV_LINE); vc.step = OA_IDLE; cruise(); break;
    }
    S5_PRINT("[VC] RESUMED nav=%d\n", vc.nav);
}

static void on_start(void)
{
    switch (vc.state) {
    case VC_READY:    start_run(); break;
    case VC_PAUSED:   resume();    break;
    case VC_FAULT:    s5_motion_reset(); to_state(VC_INIT); break;     /* S2 accepts reset only once the cause is cleared */
    case VC_COMPLETE: to_state(VC_READY); break;
    default: break;
    }
}
static void on_stop(void)
{
    if (vc.state == VC_RUNNING) pause_mission("stop pressed");
    else if (vc.state == VC_PAUSED) { to_state(VC_READY); nav_to(NAV_LINE); }   /* cancel */
}

static void on_event(const uint32_t w[2])
{
    unsigned type = VC_EV_TYPE(w[0]);
    uint32_t arg  = VC_EV_ARG(w[0]);

    if (type == VC_EV_IMPACT) { if (vc.state == VC_RUNNING || vc.state == VC_PAUSED) enter_fault(CAUSE_IMPACT); return; }
    if (type == VC_EV_IMU_LOST) { vc.degraded = 1; update_status(); if (vc.state == VC_RUNNING && vc.nav == NAV_LINE) cruise(); return; }
    if (type == VC_EV_HUMP_START) { vc.on_hump = 1; update_status(); return; }
    if (type == VC_EV_HUMP_DONE)  { vc.on_hump = 0; vc.hump_peak_mm = (uint16_t)(arg & 0x1FF); update_status();
                                    S5_PRINT("[VC] hump peak %u mm\n", vc.hump_peak_mm); return; }
    if (type == VC_EV_SENSOR_FAULT) {
        if (arg && S5_SENSOR_FAULT_IS_FATAL && vc.state == VC_RUNNING) enter_fault(CAUSE_SENSOR);
        return;
    }
    if (vc.state != VC_RUNNING) return;              /* everything below only matters while running */

    switch (type) {
    case VC_EV_BARCODE:
        if (vc.nav != NAV_LINE || !VC_BARCODE_VALID(arg)) break;     /* ignored during bypass */
        switch (VC_BARCODE_CMD(arg)) {
        case 0: nav_to(NAV_BARCODE); issue_step(BC_TURN); s5_motion_turn(90);  break;   /* A left   */
        case 1: nav_to(NAV_BARCODE); issue_step(BC_TURN); s5_motion_turn(-90); break;   /* B right  */
        case 3: nav_to(NAV_BARCODE); issue_step(BC_TURN); s5_motion_turn(180); break;   /* D U-turn */
        default: break;                                                                 /* C straight */
        }
        break;
    case VC_EV_LINE_LOST:  if (vc.nav == NAV_LINE && !vc.lost_since) vc.lost_since = now_ms(); break;
    case VC_EV_LINE_FOUND: if (s5_hook_line_lost() < 0) vc.lost_since = 0; break;   /* polled path decides (needs a steady line) */
    case VC_EV_OBSTACLE:   if (vc.nav == NAV_LINE) oa_begin(); break;
    case VC_EV_PROFILE:    on_profile(w); break;
    case VC_EV_TOO_CLOSE:  if (vc.nav == NAV_OBSTACLE && vc.step == B4_FWD) pause_mission("too close during bypass"); break;
    case VC_EV_NO_PATH:    if (vc.nav == NAV_OBSTACLE) { vc.step = OA_IDLE; pause_mission("no clear path after rescans"); } break;
    case VC_EV_FINISH:     s5_motion_stop(); to_state(VC_COMPLETE); buzz(600); S5_PRINT("[VC] MISSION COMPLETE, hump peak %u mm\n", vc.hump_peak_mm); break;
    default: break;
    }
}

/* ---------------------------------------------------------------- 10 ms housekeeping */
static int btn(unsigned pin, uint8_t *cnt)
{
    if (!s5_gpio_get(pin)) { if (*cnt < 255) (*cnt)++; } else *cnt = 0;      /* active low, pull-up */
    return *cnt == S5_BTN_DEBOUNCE;                                           /* fires once per press */
}

static void tick(void)
{
    uint32_t t = now_ms();
    buzz_tick();

    int go_start = btn(S5_PIN_BTN_START, &vc.btn_start_cnt);
    int go_stop  = btn(S5_PIN_BTN_STOP,  &vc.btn_stop_cnt);
    int r = s5_hook_remote_poll();
    if (r == 1) go_start = 1;
    if (r == 2) go_stop = 1;
    if (go_stop)  on_stop();                          /* stop wins over start */
    else if (go_start) on_start();

    if ((vc.state == VC_RUNNING || vc.state == VC_PAUSED) && s5_motion_faulted()) enter_fault(CAUSE_MOTION);

    switch (vc.state) {
    case VC_INIT:
        /* READY only once Motion passed its self-test, the IMU is calibrated (or LOST) and the
           IR sensors are calibrated: otherwise START would drive with no steering */
        if (s5_motion_ready() && s5_hook_imu_ready() && s5_hook_line_ready()) { to_state(VC_READY); S5_PRINT("[VC] READY - press START (GP20)\n"); }
        else if (s5_motion_ready() && s5_hook_imu_ready()) {
            /* only the IR calibration is missing: wait as long as it takes (S3 keeps retrying), no FAULT */
            if (t - vc.t_state > S5_INIT_TMO_MS) { vc.t_state = t; S5_PRINT("[VC] waiting for IR calibration - sweep the sensors over black and white\n"); }
        }
        else if (t - vc.t_state > S5_INIT_TMO_MS) enter_fault(CAUSE_INIT);
        break;

    case VC_RUNNING:
        if (vc.rescan_at && (int32_t)(t - vc.rescan_at) >= 0) {
            vc.rescan_at = 0; vc.use_rescan = 1; s5_scan_request(S5_REQ_RESCAN); step_to(OA_WAIT_PROFILE);
        }
        if (vc.step == B6B_SEEK) {                    /* creep until the original line shows up in the window */
            uint32_t el = t - vc.t_seek;
            int moved = (int)(el * S5_CREEP_MM_S / 1000u);                        /* time-based distance estimate */
            if (s5_hook_line_detected()) { if (++vc.seek_hits >= 2) {                  /* 2 ticks = 20 ms debounce */
                    if (el > S5_RECOVERY_TARGET_MS) S5_PRINT("[VC] NFR19 exceeded: %u ms\n", (unsigned)el);
                    s5_motion_clear_done(); s5_motion_stop(); step_to(B6C_STOP);
            } } else vc.seek_hits = 0;
            if (vc.step == B6B_SEEK && (moved > 2 * S5_LINE_TOL_MM + 40 || el > S5_LINE_LOST_TMO_MS)) enter_fault(CAUSE_RECOVERY);
        } else if (vc.step != OA_IDLE && vc.step != OA_WAIT_PROFILE) {
            if (s5_motion_wait_done(TMO_POL) == E_OK) on_move_done();
            else if (t - vc.t_step > S5_STEP_TMO_MS) enter_fault(CAUSE_TIMEOUT);
        } else if (vc.step == OA_WAIT_PROFILE && t - vc.t_step > S5_PROFILE_TMO_MS) {
            enter_fault(CAUSE_TIMEOUT);
        }
        if (vc.nav == NAV_LINE) {
            int lost = s5_hook_line_lost();
            if (lost >= 0) {
                /* Line Follow reports its state directly: the timer runs for as long
                   as the line is really lost and only resets when it is really found */
                if (!lost) {                            /* only a STEADY line cancels it */
                    if (vc.lost_since) {
                        if (!vc.found_since) vc.found_since = t ? t : 1u;
                        else if (t - vc.found_since >= S5_LINE_FOUND_MS) { vc.lost_since = 0; vc.found_since = 0; }
                    }
                }
                else {
                    vc.found_since = 0;
                    if (!vc.lost_since) vc.lost_since = t ? t : 1u;
                    else if (t - vc.lost_since > S5_LINE_LOST_TMO_MS) enter_fault(CAUSE_LINE_LOST);
                }
            } else if (vc.lost_since) {                 /* no Line Follow status: event-based */
                if (s5_hook_line_detected()) vc.lost_since = 0;
                else if (t - vc.lost_since > S5_LINE_LOST_TMO_MS) enter_fault(CAUSE_LINE_LOST);
            }
        }
        break;
    default: break;
    }
}

EXPORT void s5_vc_task(INT stacd, void *exinf)
{
    UB *b;
    UINT i;
    uint32_t m[2];
    INT n;

    (void)stacd; (void)exinf;
    s5_pwm_unreset();
    s5_gpio_in_init(S5_PIN_BTN_START, 1, 0);          /* Robo Pico buttons: active low, pull-up */
    s5_gpio_in_init(S5_PIN_BTN_STOP,  1, 0);
    s5_pwm_start(S5_PIN_BUZZER, S5_SYS_CLK_MHZ, 499u);   /* 2 kHz, off until buzz() */

    b = (UB*)&vc;
    for (i = 0; i < sizeof vc; i++) b[i] = 0;
    vc.side = 1;
    to_state(VC_INIT);
    S5_PRINT("[VC] INITIALISING\n");

    for (;;) {
        n = tk_rcv_mbf(mbf_vc_events, m, S5_VC_TICK_MS);
        while (n > 0) {                               /* drain everything queued, in order */
            if (n < 8) m[1] = 0;
            on_event(m);
            n = tk_rcv_mbf(mbf_vc_events, m, TMO_POL);
        }
        tick();
    }
}
