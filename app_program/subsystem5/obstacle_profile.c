#include "obstacle_profile.h"

/* sin(0..90 deg, 5 deg steps) in Q12 */
static const int16_t sin_q12[19] = {
    0, 357, 711, 1060, 1401, 1731, 2048, 2349, 2633, 2896,
    3138, 3355, 3547, 3712, 3849, 3957, 4034, 4080, 4096 };

static int sin12(int deg)
{
    int neg = deg < 0;
    if (neg) deg = -deg;
    if (deg > 90) deg = 90;
    int i = (deg + 2) / 5;                 /* nearest 5 deg step */
    int v = sin_q12[i > 18 ? 18 : i];
    return neg ? -v : v;
}
static int cos12(int deg) { int a = deg < 0 ? -deg : deg; return sin12(90 - (a > 90 ? 90 : a)); }

int s5_lateral_mm(int angle_deg, uint16_t dist)
{ return (int)(((int32_t)dist * sin12(angle_deg - S5_SERVO_CENTRE_DEG)) / 4096); }
int s5_forward_mm(int angle_deg, uint16_t dist)
{ return (int)(((int32_t)dist * cos12(angle_deg - S5_SERVO_CENTRE_DEG)) / 4096); }

void s5_scan_init(s5_scan_data_t *sd)
{
    uint8_t *b = (uint8_t *)sd;
    for (unsigned i = 0; i < sizeof *sd; i++) b[i] = 0;
    sd->closest.dist = S5_DIST_NO_ECHO;
    sd->left_edge = sd->right_edge = sd->left_next = sd->right_next = sd->closest;
}

void s5_scan_note(s5_scan_data_t *sd, int angle, uint16_t dist)
{
    if (dist != S5_DIST_NO_ECHO && dist < S5_COARSE_CLEAR_MM && dist < sd->closest.dist) {
        sd->closest.angle = (int16_t)angle;
        sd->closest.dist  = dist;
    }
}

/* A reading belongs to the obstacle if its FORWARD distance is close to the
 * closest point's forward distance. (Raw range grows as 1/cos for a flat
 * face seen at an angle, so comparing raw range would end the obstacle early.) */
int s5_is_obstacle(const s5_scan_data_t *sd, int angle, uint16_t dist)
{
    if (dist == S5_DIST_NO_ECHO || dist > S5_MAX_RANGE_MM || sd->closest.dist == S5_DIST_NO_ECHO)
        return 0;
    int fc = s5_forward_mm(sd->closest.angle, sd->closest.dist);
    int fr = s5_forward_mm(angle, dist);
    int d  = fr > fc ? fr - fc : fc - fr;
    return d <= S5_EDGE_JUMP_MM;
}

static int clampi(int v, int lo, int hi) { return v < lo ? lo : (v > hi ? hi : v); }

void s5_make_profile(const s5_scan_data_t *sd, s5_profile_t *p)
{
    uint8_t *b = (uint8_t *)p;
    for (unsigned i = 0; i < sizeof *p; i++) b[i] = 0;
    if (sd->closest.dist == S5_DIST_NO_ECHO) { p->action = S5_ACT_CONTINUE; return; }

    s5_reading_t le = sd->left_edge, re = sd->right_edge, ln = sd->left_next, rn = sd->right_next;
    uint8_t lclip = sd->left_clipped, rclip = sd->right_clipped;

    if (!sd->have_fine) {                  /* fall back to the coarse readings only */
        int lo = -1, hi = -1;
        for (int i = 0; i < S5_COARSE_N; i++)
            if (sd->coarse[i].dist != S5_DIST_NO_ECHO && s5_is_obstacle(sd, sd->coarse[i].angle, sd->coarse[i].dist)) {
                if (lo < 0) lo = i;
                hi = i;
            }
        re = sd->coarse[lo]; le = sd->coarse[hi];
        rn = (lo > 0) ? sd->coarse[lo - 1] : (s5_reading_t){0, S5_DIST_NO_ECHO};
        ln = (hi < S5_COARSE_N - 1) ? sd->coarse[hi + 1] : (s5_reading_t){0, S5_DIST_NO_ECHO};
        rclip = (lo == 0); lclip = (hi == S5_COARSE_N - 1);
    }

    int xl = s5_lateral_mm(le.angle - S5_BEAM_INSET_DEG, le.dist);   /* beam inset */
    int xr = s5_lateral_mm(re.angle + S5_BEAM_INSET_DEG, re.dist);
    if (xl < xr) { int m = (xl + xr) / 2; xl = m; xr = m; }
    int w = xl - xr;
    if (w < S5_MIN_WIDTH_MM) { int m = (xl + xr) / 2; xl = m + S5_MIN_WIDTH_MM / 2; xr = m - S5_MIN_WIDTH_MM / 2; w = S5_MIN_WIDTH_MM; }

    int cl, cr;
    if (lclip) cl = 0;
    else if (ln.dist == S5_DIST_NO_ECHO) cl = S5_CLEAR_MAX_MM;
    else cl = s5_lateral_mm(ln.angle, ln.dist) - xl;
    if (rclip) cr = 0;
    else if (rn.dist == S5_DIST_NO_ECHO) cr = S5_CLEAR_MAX_MM;
    else cr = xr - s5_lateral_mm(rn.angle, rn.dist);

    p->found         = 1;
    p->closest_mm    = sd->closest.dist;
    p->closest_angle = sd->closest.angle;
    p->width_mm      = (uint16_t)w;
    p->xl_mm         = (int16_t)xl;
    p->xr_mm         = (int16_t)xr;
    p->clr_l_mm      = (uint16_t)clampi(cl, 0, S5_CLEAR_MAX_MM);
    p->clr_r_mm      = (uint16_t)clampi(cr, 0, S5_CLEAR_MAX_MM);
}

s5_action_t s5_plan_action(const s5_profile_t *p, int rescans_used)
{
    if (!p->found) return S5_ACT_CONTINUE;
    if (p->closest_mm < S5_TOO_CLOSE_MM)
        return rescans_used < S5_MAX_RESCAN ? S5_ACT_REVERSE_RETRY : S5_ACT_STOP;

    const int need = S5_ROBOT_WIDTH_MM + 2 * S5_SAFETY_MARGIN_MM;
    int lok = p->clr_l_mm >= need, rok = p->clr_r_mm >= need;
    if (!lok && !rok) {
        /* both look narrow: the ultrasonic beam makes them look narrower than they are. Only
           reverse if BOTH are really blocked; otherwise go round on the more open side. */
        if ((p->clr_l_mm < S5_SIDE_BLOCKED_MM) && (p->clr_r_mm < S5_SIDE_BLOCKED_MM))
            return rescans_used < S5_MAX_RESCAN ? S5_ACT_REVERSE_RETRY : S5_ACT_STOP;
        return p->clr_l_mm >= p->clr_r_mm ? S5_ACT_TURN_LEFT : S5_ACT_TURN_RIGHT;
    }
    if (lok && rok) {
        if (p->clr_l_mm >= S5_CLEAR_MAX_MM && p->clr_r_mm >= S5_CLEAR_MAX_MM)   /* both open: shorter detour */
            return s5_bypass_lateral_mm(p, 1) <= s5_bypass_lateral_mm(p, 0) ? S5_ACT_TURN_LEFT : S5_ACT_TURN_RIGHT;
        return p->clr_l_mm >= p->clr_r_mm ? S5_ACT_TURN_LEFT : S5_ACT_TURN_RIGHT;  /* else: more clearance */
    }
    return lok ? S5_ACT_TURN_LEFT : S5_ACT_TURN_RIGHT;
}

int s5_bypass_lateral_mm(const s5_profile_t *p, int side_left)
{
    int edge = side_left ? p->xl_mm : -p->xr_mm;          /* distance of the far edge on that side */
    int lat  = edge + S5_ROBOT_WIDTH_MM / 2 + S5_SAFETY_MARGIN_MM + S5_BYPASS_EXTRA_MM;
    int min  = S5_ROBOT_WIDTH_MM / 2 + S5_SAFETY_MARGIN_MM;
    if (lat > S5_BYPASS_LAT_MAX_MM) lat = S5_BYPASS_LAT_MAX_MM;
    return lat < min ? min : lat;
}

int s5_bypass_forward_mm(const s5_profile_t *p)
{
    int depth = p->width_mm > S5_OBST_MIN_DEPTH_MM ? p->width_mm : S5_OBST_MIN_DEPTH_MM; /* ASSUMPTION: roughly square */
    if (depth > S5_OBST_MAX_DEPTH_MM) depth = S5_OBST_MAX_DEPTH_MM;
    return (int)p->closest_mm + depth + S5_ROBOT_LENGTH_MM + S5_SAFETY_MARGIN_MM;
}

void s5_profile_pack(const s5_profile_t *p, uint32_t w[2], uint32_t type)
{
    uint32_t c  = p->closest_mm > 511 ? 511 : p->closest_mm;
    uint32_t wd = p->width_mm   > 511 ? 511 : p->width_mm;
    uint32_t cl = p->clr_l_mm   > 511 ? 511 : p->clr_l_mm;
    uint32_t cr = p->clr_r_mm   > 511 ? 511 : p->clr_r_mm;
    uint32_t xl = (uint32_t)clampi(p->xl_mm + 512, 0, 1023);
    w[0] = (type << 28) | (((uint32_t)p->action & 7u) << 25) | (c << 16) | (wd << 7);
    w[1] = (cl << 23) | (cr << 14) | (xl << 4);
}

void s5_profile_unpack(const uint32_t w[2], s5_profile_t *p)
{
    p->action     = (s5_action_t)((w[0] >> 25) & 7u);
    p->closest_mm = (w[0] >> 16) & 0x1FFu;
    p->width_mm   = (w[0] >> 7) & 0x1FFu;
    p->clr_l_mm   = (w[1] >> 23) & 0x1FFu;
    p->clr_r_mm   = (w[1] >> 14) & 0x1FFu;
    p->xl_mm      = (int16_t)(((w[1] >> 4) & 0x3FFu) - 512);
    p->xr_mm      = (int16_t)(p->xl_mm - (int)p->width_mm);
    p->found      = 1;
    p->closest_angle = S5_SERVO_CENTRE_DEG;
}
