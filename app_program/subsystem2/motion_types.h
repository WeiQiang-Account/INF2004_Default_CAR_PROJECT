/**
 * @file    motion_types.h
 * @brief   Shared Motion Control data formats: states, commands (packed and
 *          unpacked), steering mailbox, fault bits and telemetry status.
 *
 * Minimum-bit rule: every packed field below uses only the bits its valid
 * range needs. Packing uses explicit masks and shifts (not C bit-fields) so
 * the layout does not depend on the compiler.
 */
#ifndef MOTION_TYPES_H
#define MOTION_TYPES_H

#include <stdint.h>
#include <stdbool.h>

/* ------------------------------------------------------------------------- */
/* Motion states (statechart 6.1): 7 states -> 3 bits                        */
/* ------------------------------------------------------------------------- */
typedef enum
{
    MOTION_STATE_INIT      = 0,
    MOTION_STATE_IDLE      = 1,
    MOTION_STATE_DRIVING   = 2,
    MOTION_STATE_DIST_MOVE = 3,
    MOTION_STATE_TURNING   = 4,
    MOTION_STATE_STOPPING  = 5,
    MOTION_STATE_FAULT     = 6
} motion_state_t;

/* Fault cause reported with the FAULT state: 5 values -> 3 bits            */
typedef enum
{
    MOTION_CAUSE_NONE     = 0,
    MOTION_CAUSE_ESTOP    = 1,
    MOTION_CAUSE_STALL    = 2,
    MOTION_CAUSE_ENCODER  = 3,
    MOTION_CAUSE_SELFTEST = 4
} motion_cause_t;

/* ------------------------------------------------------------------------- */
/* Commands: 8 types -> 3 bits                                               */
/* ------------------------------------------------------------------------- */
typedef enum
{
    MOTION_CMD_NONE       = 0,
    MOTION_CMD_DRIVE      = 1,  /* hold speed, apply line steering          */
    MOTION_CMD_MOVE_FWD   = 2,  /* drive a distance forward                 */
    MOTION_CMD_MOVE_BWD   = 3,  /* drive a distance backward                */
    MOTION_CMD_TURN_LEFT  = 4,  /* spin on the spot by an angle             */
    MOTION_CMD_TURN_RIGHT = 5,
    MOTION_CMD_STOP       = 6,
    MOTION_CMD_RESET      = 7   /* leave FAULT once the cause is cleared    */
} motion_cmd_type_t;

/* Unpacked command (used inside the task, and as the Week 13 baseline)     */
typedef struct
{
    int32_t type;        /* motion_cmd_type_t                              */
    int32_t speed_mmps;  /* 0..511                                         */
    int32_t distance_mm; /* MOVE: 0..2047                                  */
    int32_t angle_deg;   /* TURN: 0..360                                   */
    int32_t reverse;     /* DRIVE: 1 = backwards                           */
} motion_cmd_t;          /* 20 bytes                                       */

/*
 * Packed command: one 32-bit word (4 bytes)
 *   bits  0..2   type        3 bits  (0..7)
 *   bits  3..11  speed       9 bits  (0..511 mm/s)
 *   bits 12..22  argument   11 bits  (distance 0..2047 mm, or angle 0..360)
 *   bit  23      reverse     1 bit
 *   bits 24..31  reserved    8 bits  (0)
 */
typedef uint32_t motion_cmd_packed_t;

#define CMD_TYPE_SHIFT   (0u)
#define CMD_TYPE_MASK    (0x7u)
#define CMD_SPEED_SHIFT  (3u)
#define CMD_SPEED_MASK   (0x1FFu)
#define CMD_ARG_SHIFT    (12u)
#define CMD_ARG_MASK     (0x7FFu)
#define CMD_REV_SHIFT    (23u)
#define CMD_REV_MASK     (0x1u)

static inline uint32_t motion_clamp_u(int32_t v, uint32_t max)
{
    uint32_t out = 0u;

    if (v > 0)
    {
        out = ((uint32_t)v > max) ? max : (uint32_t)v;
    }
    return out;
}

static inline motion_cmd_packed_t motion_cmd_pack(const motion_cmd_t *p_cmd)
{
    uint32_t arg;

    if ((p_cmd->type == (int32_t)MOTION_CMD_TURN_LEFT) ||
        (p_cmd->type == (int32_t)MOTION_CMD_TURN_RIGHT))
    {
        arg = motion_clamp_u(p_cmd->angle_deg, 360u);
    }
    else
    {
        arg = motion_clamp_u(p_cmd->distance_mm, CMD_ARG_MASK);
    }

    return ((motion_clamp_u(p_cmd->type, CMD_TYPE_MASK) & CMD_TYPE_MASK) << CMD_TYPE_SHIFT)
         | ((motion_clamp_u(p_cmd->speed_mmps, CMD_SPEED_MASK)) << CMD_SPEED_SHIFT)
         | ((arg & CMD_ARG_MASK) << CMD_ARG_SHIFT)
         | (((p_cmd->reverse != 0) ? 1u : 0u) << CMD_REV_SHIFT);
}

static inline void motion_cmd_unpack(motion_cmd_packed_t w, motion_cmd_t *p_cmd)
{
    uint32_t arg = (w >> CMD_ARG_SHIFT) & CMD_ARG_MASK;

    p_cmd->type       = (int32_t)((w >> CMD_TYPE_SHIFT) & CMD_TYPE_MASK);
    p_cmd->speed_mmps = (int32_t)((w >> CMD_SPEED_SHIFT) & CMD_SPEED_MASK);
    p_cmd->reverse    = (int32_t)((w >> CMD_REV_SHIFT) & CMD_REV_MASK);
    p_cmd->distance_mm = 0;
    p_cmd->angle_deg   = 0;

    if ((p_cmd->type == (int32_t)MOTION_CMD_TURN_LEFT) ||
        (p_cmd->type == (int32_t)MOTION_CMD_TURN_RIGHT))
    {
        p_cmd->angle_deg = (int32_t)arg;
    }
    else
    {
        p_cmd->distance_mm = (int32_t)arg;
    }
}

/* ------------------------------------------------------------------------- */
/* Steering mailbox (Line Follow -> Motion Control)                          */
/*   One 16-bit word, written in a single store, so it can never be torn:   */
/*     bits 0..7   correction  int8  (-100 .. +100, + = steer right)        */
/*     bits 8..15  sequence    uint8 (incremented on every write)           */
/* ------------------------------------------------------------------------- */
extern volatile uint16_t g_steer_latest;

static inline uint16_t steer_pack(int8_t correction, uint8_t seq)
{
    return (uint16_t)(((uint16_t)seq << 8) | (uint16_t)(uint8_t)correction);
}

static inline void steer_unpack(uint16_t w, int8_t *p_corr, uint8_t *p_seq)
{
    *p_corr = (int8_t)(uint8_t)(w & 0xFFu);
    *p_seq  = (uint8_t)(w >> 8);
}

/** Called by Line Follow (Buddy 3) every 5 ms. Only Line Follow writes.    */
static inline void steer_publish(int8_t correction)
{
    static uint8_t s_seq = 0u;

    if (correction > 100)  { correction = 100; }
    if (correction < -100) { correction = -100; }
    s_seq++;
    g_steer_latest = steer_pack(correction, s_seq);   /* single 16-bit store */
}

/* ------------------------------------------------------------------------- */
/* Shared fault bitmask (team): one bit per fault, held in a kernel event   */
/* flag so set/clear is atomic. Motion owns the first two bits.             */
/* ------------------------------------------------------------------------- */
#define FAULT_BIT_MOTOR_STALL   (1u << 0)
#define FAULT_BIT_ENCODER       (1u << 1)
#define FAULT_BIT_IMU           (1u << 2)
#define FAULT_BIT_NO_ECHO       (1u << 3)
#define FAULT_BIT_WIFI          (1u << 4)

/* Event-flag bit patterns used by Motion                                   */
#define ESTOP_BIT               (1u << 0)   /* in flg_estop (set by IMU)     */
#define MOVE_DONE_BIT           (1u << 0)   /* in flg_move_done (to VC)      */

/* ------------------------------------------------------------------------- */
/* Motion status for telemetry                                              */
/* ------------------------------------------------------------------------- */
typedef struct
{
    motion_state_t state;
    motion_cause_t cause;
    bool           steer_stale;
    int16_t        speed_l_mmps;   /* -1024..1023 */
    int16_t        speed_r_mmps;
    int8_t         delta_l;        /* counts in the last 20 ms */
    int8_t         delta_r;
    uint16_t       distance_mm;    /* since start, wraps at 65.5 m */
} motion_status_t;

#define MOTION_STATUS_PACKED_BYTES (8u)

/**
 * Packs the status into 8 bytes, little-endian, masks/shifts only:
 *   byte 0      : state(3) | cause(3)<<3 | steer_stale(1)<<6
 *   bytes 1..3  : speed_l(11, two's complement) | speed_r(11)<<11  (22 bits)
 *   byte 4      : delta_l (int8)
 *   byte 5      : delta_r (int8)
 *   bytes 6..7  : distance_mm (uint16)
 */
static inline void motion_status_pack(const motion_status_t *p_s, uint8_t out[MOTION_STATUS_PACKED_BYTES])
{
    uint32_t sl = (uint32_t)(int32_t)p_s->speed_l_mmps & 0x7FFu;
    uint32_t sr = (uint32_t)(int32_t)p_s->speed_r_mmps & 0x7FFu;
    uint32_t sp = sl | (sr << 11);

    out[0] = (uint8_t)(((uint32_t)p_s->state & 0x7u)
                     | (((uint32_t)p_s->cause & 0x7u) << 3)
                     | ((p_s->steer_stale ? 1u : 0u) << 6));
    out[1] = (uint8_t)(sp & 0xFFu);
    out[2] = (uint8_t)((sp >> 8) & 0xFFu);
    out[3] = (uint8_t)((sp >> 16) & 0x3Fu);
    out[4] = (uint8_t)p_s->delta_l;
    out[5] = (uint8_t)p_s->delta_r;
    out[6] = (uint8_t)(p_s->distance_mm & 0xFFu);
    out[7] = (uint8_t)(p_s->distance_mm >> 8);
}

static inline int16_t motion_sext11(uint32_t v)
{
    v &= 0x7FFu;
    return (int16_t)(((v & 0x400u) != 0u) ? ((int32_t)v - 2048) : (int32_t)v);
}

static inline void motion_status_unpack(const uint8_t in[MOTION_STATUS_PACKED_BYTES], motion_status_t *p_s)
{
    uint32_t sp = (uint32_t)in[1] | ((uint32_t)in[2] << 8) | ((uint32_t)(in[3] & 0x3Fu) << 16);

    p_s->state        = (motion_state_t)(in[0] & 0x7u);
    p_s->cause        = (motion_cause_t)((in[0] >> 3) & 0x7u);
    p_s->steer_stale  = (((in[0] >> 6) & 0x1u) != 0u);
    p_s->speed_l_mmps = motion_sext11(sp);
    p_s->speed_r_mmps = motion_sext11(sp >> 11);
    p_s->delta_l      = (int8_t)in[4];
    p_s->delta_r      = (int8_t)in[5];
    p_s->distance_mm  = (uint16_t)((uint32_t)in[6] | ((uint32_t)in[7] << 8));
}

#endif /* MOTION_TYPES_H */
