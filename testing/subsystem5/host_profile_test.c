/* Host-side check of obstacle_profile.c.  Build + run on a PC:
 *   gcc -Wall -Wextra -I../../app_program/subsystem5 host_profile_test.c \
 *       ../../app_program/subsystem5/obstacle_profile.c -lm -o hp && ./hp   */
#include <stdio.h>
#include <stdlib.h>
#include "obstacle_profile.h"

static int fails;
#define CHECK(c) do { if (!(c)) { printf("FAIL line %d: %s\n", __LINE__, #c); fails++; } } while (0)

/* Simulate an obstacle: flat face at forward distance `fwd`, lateral extent [xr,xl]. */
static uint16_t sim_read(int angle, int fwd, int xr, int xl)
{
    int phi = angle - 90;
    /* ray hits the face if its lateral position at depth fwd lies inside the box,
       widened by a ~21 deg half-beam: the real HC-SR04 (2026-11-11 track: a ~10 cm box read
       24-36 cm wide with a 5 deg inset) */
    for (int d = -21; d <= 21; d += 7) {
        int a = phi + d;
        if (a <= -85 || a >= 85) continue;
        /* x = fwd * tan(a) */
        double x = fwd * __builtin_tan(a * 3.14159265 / 180.0);
        if (x >= xr && x <= xl) {
            double dist = fwd / __builtin_cos(a * 3.14159265 / 180.0);
            return (uint16_t)dist;
        }
    }
    return S5_DIST_NO_ECHO;
}

static void run(const char *name, int fwd, int xr, int xl, s5_profile_t *out)
{
    s5_scan_data_t sd; s5_scan_init(&sd);
    static const int ang[5] = { S5_SERVO_LIMIT_MIN, (S5_SERVO_LIMIT_MIN + 90) / 2, 90,
                                (90 + S5_SERVO_LIMIT_MAX) / 2, S5_SERVO_LIMIT_MAX };   /* same as obstacle_scan.c */
    int hit = -1;
    for (int i = 0; i < 5; i++) {
        uint16_t d = sim_read(ang[i], fwd, xr, xl);
        sd.coarse[i].angle = ang[i]; sd.coarse[i].dist = d;
        s5_scan_note(&sd, ang[i], d);
    }
    for (int i = 0; i < 5; i++) if (sd.coarse[i].dist == sd.closest.dist) hit = i;
    if (hit >= 0) {
        sd.have_fine = 1;
        int a0 = sd.closest.angle;
        sd.left_edge = sd.right_edge = sd.closest;
        for (int a = a0 + 5;; a += 5) {                       /* left  */
            if (a > S5_SERVO_MAX_DEG) { sd.left_clipped = 1; break; }
            uint16_t d = sim_read(a, fwd, xr, xl); s5_scan_note(&sd, a, d);
            s5_reading_t r = {(int16_t)a, d};
            if (s5_is_obstacle(&sd, a, d)) sd.left_edge = r; else { sd.left_next = r; break; }
        }
        for (int a = a0 - 5;; a -= 5) {                       /* right */
            if (a < S5_SERVO_MIN_DEG) { sd.right_clipped = 1; break; }
            uint16_t d = sim_read(a, fwd, xr, xl); s5_scan_note(&sd, a, d);
            s5_reading_t r = {(int16_t)a, d};
            if (s5_is_obstacle(&sd, a, d)) sd.right_edge = r; else { sd.right_next = r; break; }
        }
    }
    s5_make_profile(&sd, out);
    out->action = s5_plan_action(out, 0);
    printf("%-22s found=%d closest=%3u width=%3u xl=%4d xr=%4d clrL=%3u clrR=%3u action=%d\n",
           name, out->found, out->closest_mm, out->width_mm, out->xl_mm, out->xr_mm,
           out->clr_l_mm, out->clr_r_mm, (int)out->action);
}

int main(void)
{
    s5_profile_t p;

    run("nothing ahead", 450, 0, 0, &p);                CHECK(!p.found && p.action == S5_ACT_CONTINUE);
    run("box 80mm wide centred", 180, -40, 40, &p);     CHECK(p.found && abs((int)p.width_mm - 80) <= 40);
                                                        CHECK(p.action == S5_ACT_TURN_LEFT || p.action == S5_ACT_TURN_RIGHT);
    run("box shifted left", 200, 10, 110, &p);          CHECK(p.action == S5_ACT_TURN_RIGHT);
    run("box shifted right", 200, -110, -10, &p);       CHECK(p.action == S5_ACT_TURN_LEFT);
    run("too close (80mm)", 80, -40, 40, &p);           CHECK(p.action == S5_ACT_REVERSE_RETRY);
    run("wide wall", 150, -400, 400, &p);               CHECK(p.action != S5_ACT_CONTINUE);   /* 2026-11-11: reverse, or go round the more open side - never drive into it */
    run("wall at 250 (ok range)", 250, -600, 600, &p);   CHECK(p.action != S5_ACT_CONTINUE);

    p.action = S5_ACT_TURN_RIGHT; p.closest_mm = 187; p.width_mm = 95; p.clr_l_mm = 211; p.clr_r_mm = 340; p.xl_mm = -37; p.found = 1;
    uint32_t w[2]; s5_profile_pack(&p, w, 10);
    s5_profile_t q; s5_profile_unpack(w, &q);
    CHECK((w[0] >> 28) == 10);
    CHECK(q.action == p.action && q.closest_mm == p.closest_mm && q.width_mm == p.width_mm);
    CHECK(q.clr_l_mm == p.clr_l_mm && q.clr_r_mm == p.clr_r_mm && q.xl_mm == p.xl_mm);

    CHECK(s5_lateral_mm(150, 200) > 150 && s5_lateral_mm(30, 200) < -150);
    CHECK(s5_forward_mm(90, 200) == 200);
    CHECK(S5_STATUS_PACK(7, 3, 7) == 0xFF && S5_STATUS_PACK(1, 0, 2) == 0x22);

    printf(fails ? "\n%d FAILED\n" : "\nALL PASSED\n", fails);
    return fails != 0;
}
