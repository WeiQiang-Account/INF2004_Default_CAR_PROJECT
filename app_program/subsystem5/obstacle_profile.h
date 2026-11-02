/* obstacle_profile.h - pure-C obstacle maths (no RTOS, no hardware).
 * Builds the profile (location, width, closest point, left/right clearance)
 * from scan readings and chooses the avoidance action (FR21-FR23).
 * Compiles on a PC: see testing/subsystem5/host_profile_test.c.            */
#ifndef OBSTACLE_PROFILE_H
#define OBSTACLE_PROFILE_H
#include <stdint.h>
#include "s5_config.h"

#define S5_DIST_NO_ECHO  0xFFFFu
#define S5_COARSE_N      5

/* 3-bit action (Tribunal min-bit rule) */
typedef enum {
    S5_ACT_CONTINUE = 0, S5_ACT_STOP, S5_ACT_TURN_LEFT,
    S5_ACT_TURN_RIGHT, S5_ACT_REVERSE_RETRY
} s5_action_t;

typedef struct { int16_t angle; uint16_t dist; } s5_reading_t;

typedef struct {
    s5_reading_t coarse[S5_COARSE_N];
    s5_reading_t closest;                 /* nearest obstacle reading            */
    s5_reading_t left_edge, right_edge;   /* last readings still on the obstacle */
    s5_reading_t left_next, right_next;   /* first reading beyond each edge      */
    uint8_t left_clipped, right_clipped;  /* servo limit hit while still on it   */
    uint8_t have_fine;
} s5_scan_data_t;

typedef struct {
    uint8_t  found;
    uint16_t closest_mm;
    int16_t  closest_angle;
    uint16_t width_mm;
    int16_t  xl_mm, xr_mm;                /* edge lateral offsets, + = LEFT of robot centreline */
    uint16_t clr_l_mm, clr_r_mm;          /* free space beyond each edge                         */
    s5_action_t action;
} s5_profile_t;

void s5_scan_init(s5_scan_data_t *sd);
void s5_scan_note(s5_scan_data_t *sd, int angle, uint16_t dist); /* tracks closest */
int  s5_is_obstacle(const s5_scan_data_t *sd, int angle, uint16_t dist); /* same forward depth as closest */

int  s5_lateral_mm(int angle_deg, uint16_t dist);   /* + = left  (angle 90 = ahead) */
int  s5_forward_mm(int angle_deg, uint16_t dist);

void        s5_make_profile(const s5_scan_data_t *sd, s5_profile_t *p);
s5_action_t s5_plan_action(const s5_profile_t *p, int rescans_used);

/* Box-shaped bypass geometry. side_left: 1 = go round the left, 0 = right. */
int s5_bypass_lateral_mm(const s5_profile_t *p, int side_left);
int s5_bypass_forward_mm(const s5_profile_t *p);

/* Wire format to the Vehicle Controller: two explicit-mask words (no C bit-field
 * layout dependence). w0: type[31:28] action[27:25] closest[24:16] width[15:7]
 * w1: clrL[31:23] clrR[22:14] (xl+512)[13:4]                                   */
void s5_profile_pack(const s5_profile_t *p, uint32_t w[2], uint32_t type);
void s5_profile_unpack(const uint32_t w[2], s5_profile_t *p);

/* One status byte for telemetry: state(3) | stage(2) | action(3) */
#define S5_STATUS_PACK(state, stage, action) \
    ((uint8_t)((((state) & 7u) << 5) | (((stage) & 3u) << 3) | ((action) & 7u)))
#endif
