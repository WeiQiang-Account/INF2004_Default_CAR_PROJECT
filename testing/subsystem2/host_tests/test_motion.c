/*
 * Host unit tests + simple robot simulation for the Buddy 2 Motion module.
 * Build: make -C tests   (see Makefile; runs both packed and baseline builds)
 */
#include <stdio.h>
#include <math.h>
#include <string.h>
#include "motion.h"
#include "motion_config.h"
#include "drv_encoder.h"
#include "pid.h"

/* object creation exists only in the mocks (the robot creates them in subsystem2_init) */
os_id_t os_flag_create(uint32_t init);
os_id_t os_mbf_create(uint32_t buf_bytes, uint32_t max_msg);

extern uint32_t g_mock_now_ms;
extern uint32_t g_mock_flags[];
extern int32_t  g_mock_duty[2];
extern int      g_mock_stop_all_calls;

static int s_fail = 0, s_pass = 0;
#define CHECK(cond, ...) do { if (cond) { s_pass++; } else { s_fail++; printf("  FAIL %s:%d: ", __FILE__, __LINE__); printf(__VA_ARGS__); printf("\n"); } } while (0)

/* ------------------------------------------------------------------ */
/* Plant: first-order motor + wheel, generates encoder counts          */
/* ------------------------------------------------------------------ */
typedef struct
{
    float v[2];          /* wheel speed mm/s           */
    float s[2];          /* wheel travel mm            */
    float frac[2];       /* fractional counts          */
    float gain[2];       /* mm/s per permille duty     */
    int   locked[2];     /* wheel held (stall test)    */
} plant_t;
static plant_t P;

static void plant_reset(void)
{
    memset(&P, 0, sizeof(P));
    P.gain[0] = 0.625f;          /* 1000 permille -> 625 mm/s */
    P.gain[1] = 0.625f * 0.90f;  /* right motor 10 % weaker   */
}

static void plant_step_1ms(void)
{
    const float tau = 0.08f, dt = 0.001f;
    int w;
    for (w = 0; w < 2; w++)
    {
        float duty = (float)g_mock_duty[w];
        float vt = (fabsf(duty) < 60.0f) ? 0.0f : duty * P.gain[w];   /* static friction */
        if (P.locked[w]) { P.v[w] = 0.0f; continue; }
        P.v[w] += (vt - P.v[w]) * (dt / tau);
        P.s[w] += P.v[w] * dt;
        P.frac[w] += (P.v[w] * dt) / MM_PER_COUNT;
        {
            int32_t c = (int32_t)P.frac[w];
            /* encoder sign: left counts +, right mounted mirrored (sign -1 in config).
             * encoder_test_inject adds raw counts AFTER sign, so inject the forward count. */
            if (c != 0) { encoder_test_inject((enc_wheel_t)w, c); P.frac[w] -= (float)c; }
        }
    }
}

static int s_estop_id, s_fault_id, s_move_done_id, s_mbf_id;

/* Run the whole system for ms milliseconds: plant every 1 ms, Motion every 20 ms */
static void run_ms(uint32_t ms)
{
    uint32_t i;
    for (i = 0; i < ms; i++)
    {
        plant_step_1ms();
        g_mock_now_ms++;
        if ((g_mock_now_ms % MOTION_PERIOD_MS) == 0u)
        {
            motion_step(g_mock_now_ms, (g_mock_flags[s_estop_id] & ESTOP_BIT) != 0u);
        }
    }
}

static float heading_deg(void) { return (P.s[0] - P.s[1]) / TRACK_WIDTH_MM * 57.29578f; }

static void system_reset(void)
{
    motion_links_t links;
    static int created = 0;
    plant_reset();
    encoder_test_reset();
    g_mock_now_ms = 0u;
    if (!created)
    {
        s_estop_id = os_flag_create(0u);
        s_fault_id = os_flag_create(0u);
        s_move_done_id = os_flag_create(0u);
        s_mbf_id = os_mbf_create(128u, 20u);
        created = 1;
    }
    links.flg_estop = s_estop_id;
    links.flg_fault = s_fault_id;
    links.flg_move_done = s_move_done_id;
    links.mbf_cmd = s_mbf_id;
    motion_core_init(&links);
    g_mock_flags[s_estop_id] = 0u;
    g_mock_flags[s_fault_id] = 0u;
    g_mock_flags[s_move_done_id] = 0u;
    run_ms(200);     /* self-test then IDLE */
    memset(P.s, 0, sizeof(P.s));
}

static motion_state_t state_now(void) { motion_status_t st; motion_get_status(&st); return st.state; }
static int move_done(void) { return (g_mock_flags[s_move_done_id] & MOVE_DONE_BIT) != 0u; }

/* ------------------------------------------------------------------ */
static void test_quadrature(void)
{
    /* AB states for forward: 00 -> 10 -> 11 -> 01 -> 00 ; A=GP2 B=GP3 */
    const uint32_t seq[4] = { (1u << ENC_L_PIN_A), (1u << ENC_L_PIN_A) | (1u << ENC_L_PIN_B), (1u << ENC_L_PIN_B), 0u };
    int32_t l, r; int i;
    printf("[quadrature]\n");
    encoder_test_reset();
    for (i = 0; i < 16; i++) { encoder_on_edge(ENC_LEFT, seq[i % 4]); }
    encoder_read(&l, &r);
    CHECK(l == 16, "forward 4 cycles should give +16, got %d", (int)l);
    for (i = 15; i >= 0; i--) { encoder_on_edge(ENC_LEFT, seq[(i + 3) % 4]); }
    encoder_read(&l, &r);
    CHECK(l == 0, "reverse 4 cycles should return to 0, got %d", (int)l);
    encoder_on_edge(ENC_LEFT, (1u << ENC_L_PIN_A) | (1u << ENC_L_PIN_B));   /* 00 -> 11 : skipped */
    CHECK(encoder_new_errors(ENC_LEFT) == 1u, "skipped step should count as an error");
    /* right wheel is mirrored (sign -1) */
    encoder_test_reset();
    {
        const uint32_t rs[4] = { (1u << ENC_R_PIN_A), (1u << ENC_R_PIN_A) | (1u << ENC_R_PIN_B), (1u << ENC_R_PIN_B), 0u };
        for (i = 0; i < 8; i++) { encoder_on_edge(ENC_RIGHT, rs[i % 4]); }
    }
    encoder_read(&l, &r);
    CHECK(r == -8, "right wheel A-leads-B should count -8 (mirrored), got %d", (int)r);
}

static void test_packing(void)
{
    motion_cmd_t in = { MOTION_CMD_MOVE_FWD, 300, 1000, 0, 0 }, out;
    motion_cmd_packed_t w;
    motion_status_t s = { MOTION_STATE_TURNING, MOTION_CAUSE_STALL, true, -512, 1023, -46, 127, 54321u }, s2;
    uint8_t buf[MOTION_STATUS_PACKED_BYTES];
    int8_t c; uint8_t q;

    printf("[packing]\n");
    CHECK(sizeof(motion_cmd_packed_t) == 4u, "packed command must be 4 bytes");
    CHECK(sizeof(motion_cmd_t) == 20u, "unpacked command is 20 bytes");
    w = motion_cmd_pack(&in); motion_cmd_unpack(w, &out);
    CHECK(out.type == in.type && out.speed_mmps == 300 && out.distance_mm == 1000, "move round trip");
    in.type = MOTION_CMD_TURN_RIGHT; in.angle_deg = 90; in.distance_mm = 0;
    w = motion_cmd_pack(&in); motion_cmd_unpack(w, &out);
    CHECK(out.type == MOTION_CMD_TURN_RIGHT && out.angle_deg == 90, "turn round trip");
    in.type = MOTION_CMD_MOVE_FWD; in.distance_mm = 99999; in.speed_mmps = 9999;
    w = motion_cmd_pack(&in); motion_cmd_unpack(w, &out);
    CHECK(out.distance_mm == 2047 && out.speed_mmps == 511, "out-of-range values clamp to field max");
    CHECK((w >> 24) == 0u, "reserved bits stay zero");

    motion_status_pack(&s, buf); motion_status_unpack(buf, &s2);
    CHECK(s2.state == s.state && s2.cause == s.cause && s2.steer_stale == s.steer_stale, "status flags round trip");
    CHECK(s2.speed_l_mmps == -512 && s2.speed_r_mmps == 1023, "11-bit signed speeds round trip (%d,%d)", s2.speed_l_mmps, s2.speed_r_mmps);
    CHECK(s2.delta_l == -46 && s2.delta_r == 127 && s2.distance_mm == 54321u, "deltas and distance round trip");

    steer_unpack(steer_pack(-100, 200), &c, &q);
    CHECK(c == -100 && q == 200, "steer mailbox round trip");
}

static void test_pid(void)
{
    pid_ctrl_t p; int i; float out = 0.0f;
    printf("[pid]\n");
    pid_init(&p, 1.0f, 10.0f, 0.0f, 400.0f, 1000.0f);
    for (i = 0; i < 500; i++) { out = pid_update(&p, 5000.0f, 0.02f, 0.0f); }
    CHECK(out == 1000.0f, "output clamps at limit");
    CHECK(p.integ <= 400.0f, "integral does not wind up past its limit (%f)", p.integ);
    out = pid_update(&p, -10.0f, 0.02f, 0.0f);
    CHECK(out < 1000.0f, "recovers immediately when error changes sign");
}

static void test_selftest_and_conversions(void)
{
    printf("[conversions + self-test]\n");
    CHECK(fabsf(ENC_CPR - 937.2f) < 0.5f, "CPR about 937 (got %.1f)", ENC_CPR);
    CHECK(fabsf(MM_PER_COUNT - 0.2179f) < 0.001f, "about 0.22 mm per count (got %.4f)", MM_PER_COUNT);
    system_reset();
    CHECK(state_now() == MOTION_STATE_IDLE, "self-test passes and enters IDLE (state %d)", state_now());

    /* self-test fails if a wheel cannot move */
    plant_reset(); encoder_test_reset(); motion_reset_for_test(); P.locked[1] = 1;
    run_ms(200);
    {
        motion_status_t st; motion_get_status(&st);
        CHECK(st.state == MOTION_STATE_FAULT && st.cause == MOTION_CAUSE_SELFTEST, "locked wheel fails self-test");
        CHECK((g_mock_flags[s_fault_id] & FAULT_BIT_ENCODER) != 0u, "encoder fault bit set");
    }
}

static void test_distance(void)
{
    float travelled;
    printf("[1 m distance move, right motor 10%% weaker]\n");
    system_reset();
    CHECK(motion_move(1000, 300, false), "command queued");
    run_ms(6000);
    travelled = (P.s[0] + P.s[1]) / 2.0f;
    CHECK(move_done(), "flg_move_done set");
    CHECK(state_now() == MOTION_STATE_IDLE, "back to IDLE");
    CHECK(fabsf(travelled - 1000.0f) <= 50.0f, "distance within +-5%% (NFR7): %.1f mm", travelled);
    CHECK(fabsf(heading_deg()) <= 3.0f, "stays straight: heading drift %.2f deg", heading_deg());
    printf("  travelled %.1f mm, heading drift %.2f deg\n", travelled, heading_deg());
}

static void test_turn(void)
{
    printf("[90 deg turns]\n");
    system_reset();
    CHECK(motion_turn_right(90), "queued");
    run_ms(4000);
    CHECK(move_done(), "turn done flag");
    CHECK(fabsf(heading_deg() - 90.0f) <= 5.0f, "right turn within +-5 deg (NFR8): %.2f", heading_deg());
    printf("  right turn: %.2f deg\n", heading_deg());
    system_reset();
    motion_turn_left(90);
    run_ms(4000);
    CHECK(fabsf(heading_deg() + 90.0f) <= 5.0f, "left turn within +-5 deg: %.2f", heading_deg());
    printf("  left turn: %.2f deg\n", heading_deg());
}

static void test_speed_hold(void)
{
    float vl, vr;
    printf("[speed hold at 300 mm/s]\n");
    system_reset();
    motion_drive(300, false);
    run_ms(2000);
    vl = P.v[0]; vr = P.v[1];
    CHECK(fabsf(vl - 300.0f) <= 30.0f && fabsf(vr - 300.0f) <= 30.0f, "both wheels within +-10%% (NFR6): %.1f / %.1f", vl, vr);
    printf("  left %.1f mm/s, right %.1f mm/s\n", vl, vr);
}

static void test_steering(void)
{
    int i;
    motion_status_t st;
    printf("[steering mailbox]\n");
    system_reset();
    motion_drive(250, false);
    for (i = 0; i < 50; i++) { steer_publish(40); run_ms(5); }
    CHECK(P.v[0] > P.v[1] + 60.0f, "positive correction makes the left wheel faster (%.0f vs %.0f)", P.v[0], P.v[1]);
    motion_get_status(&st);
    CHECK(!st.steer_stale, "fresh steering is not stale");
    run_ms(100);   /* Line Follow stops publishing */
    motion_get_status(&st);
    CHECK(st.steer_stale, "steering marked stale after 60 ms");
    run_ms(600);
    CHECK(fabsf(P.v[0] - P.v[1]) < 25.0f, "stale steering -> correction zero (%.0f vs %.0f)", P.v[0], P.v[1]);
}

static void test_estop(void)
{
    motion_status_t st;
    int calls;
    printf("[E-stop]\n");
    system_reset();
    motion_drive(400, false);
    run_ms(1000);
    calls = g_mock_stop_all_calls;
    os_flag_set(s_estop_id, ESTOP_BIT);
    run_ms(MOTION_PERIOD_MS);      /* next Motion cycle */
    motion_get_status(&st);
    CHECK(g_mock_duty[0] == 0 && g_mock_duty[1] == 0, "PWM at zero within one 20 ms cycle (NFR9)");
    CHECK(g_mock_stop_all_calls > calls, "motor_stop_all called");
    CHECK(st.state == MOTION_STATE_FAULT && st.cause == MOTION_CAUSE_ESTOP, "FAULT with cause ESTOP");
    motion_drive(300, false); run_ms(100);
    CHECK(g_mock_duty[0] == 0, "commands ignored in FAULT");
    motion_reset(); run_ms(40);
    CHECK(state_now() == MOTION_STATE_FAULT, "reset refused while E-stop still set");
    os_flag_clear(s_estop_id, ESTOP_BIT);
    motion_reset(); run_ms(40);
    CHECK(state_now() == MOTION_STATE_IDLE, "reset accepted after E-stop cleared");
}

static void test_stall(void)
{
    motion_status_t st;
    uint32_t t0, t_fault = 0u;
    printf("[stall detection]\n");
    system_reset();
    motion_drive(300, false);
    run_ms(500);
    P.locked[0] = 1;
    t0 = g_mock_now_ms;
    while ((g_mock_now_ms - t0) < 1000u)
    {
        run_ms(1);
        motion_get_status(&st);
        if (st.state == MOTION_STATE_FAULT) { t_fault = g_mock_now_ms - t0; break; }
    }
    CHECK(st.state == MOTION_STATE_FAULT && st.cause == MOTION_CAUSE_STALL, "stall -> FAULT");
    CHECK(t_fault >= 300u && t_fault <= 360u, "detected about 300 ms after the wheel locked (%u ms)", (unsigned)t_fault);
    CHECK((g_mock_flags[s_fault_id] & FAULT_BIT_MOTOR_STALL) != 0u, "stall bit set in shared fault mask");
    CHECK(g_mock_duty[0] == 0 && g_mock_duty[1] == 0, "motors off");
}

static void test_latest_wins(void)
{
    printf("[latest command wins]\n");
    system_reset();
    motion_drive(300, false);
    run_ms(400);
    motion_turn_left(90);       /* interrupts DRIVING directly */
    run_ms(20);
    CHECK(state_now() == MOTION_STATE_TURNING, "DRIVING -> TURNING without IDLE");
    motion_stop();
    run_ms(20);
    CHECK(state_now() == MOTION_STATE_STOPPING, "stop() -> STOPPING");
    run_ms(500);
    CHECK(state_now() == MOTION_STATE_IDLE && move_done(), "STOPPING -> IDLE with flg_move_done");
}

int main(void)
{
    printf("Motion tests (MOTION_USE_PACKED=%d, command message = %u bytes)\n",
           MOTION_USE_PACKED, (unsigned)(MOTION_USE_PACKED ? sizeof(motion_cmd_packed_t) : sizeof(motion_cmd_t)));
    test_quadrature();
    test_packing();
    test_pid();
    test_selftest_and_conversions();
    test_distance();
    test_turn();
    test_speed_hold();
    test_steering();
    test_estop();
    test_stall();
    test_latest_wins();
    printf("\n%d passed, %d failed\n", s_pass, s_fail);
    return (s_fail == 0) ? 0 : 1;
}
