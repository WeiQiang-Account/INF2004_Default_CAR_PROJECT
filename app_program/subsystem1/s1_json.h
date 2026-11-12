/* s1_json.h - Buddy 1's PURE logic: no hardware, no kernel, no lwIP.
 * Builds the MQTT payloads and validates incoming commands (Week 6 section 6.5).
 * Compiled on a PC for the host tests (testing/subsystem1/host_tests).        */
#ifndef S1_JSON_H
#define S1_JSON_H

#include <stdint.h>
#include <stdbool.h>

/* ---------------------------------------------------------------- snapshot
 * One consistent picture of the robot, taken every 200 ms from the other
 * subsystems' read-only getters (s1_snapshot_take, subsystem1.c).          */
typedef struct {
	uint32_t t_ms;
	uint8_t  vc_state, vc_nav, on_hump, degraded;    /* Vehicle Controller (S5)  */
	uint8_t  m_state, m_cause;                       /* Motion Control (S2)      */
	int16_t  spd_l, spd_r;                           /* mm/s                     */
	int32_t  enc_l, enc_r;
	uint32_t enc_err_l, enc_err_r;                   /* skipped encoder steps    */
	uint32_t dist_mm;
	int16_t  line;                                   /* -100..100 (S3)           */
	uint8_t  line_lost;
	uint16_t ir[3];
	char     bar;                                    /* last character read, 0 none (S3)   */
	char     bar_pend;                               /* stored, waiting for a junction (VC) */
	char     map[4];                                 /* MQTT map F U L R, '-' = unset      */
	uint8_t  bar_act;                                /* what the map gives 'bar': S5_JA_*  */
	uint8_t  bar_why;                                /* last failed read reason            */
	uint8_t  bar_n;                                  /* read in progress: elements so far (of 29) */
	uint8_t  bar_fail_n;                             /* last failed read: how many bars/gaps ...  */
	uint8_t  bar_fail_w[29];                         /* ... and their widths (bar, gap, bar ...) */
	uint8_t  bar_fail_mm10;                          /* 1 = widths in 0.1 mm, 0 = in ms           */
	uint32_t bar_edges;                              /* GP7 edges so far                   */
	uint16_t junc_seen, junc;                        /* junctions seen by S3 / handled by VC */
	uint8_t  j_act;                                  /* movement at the last junction      */
	char     j_ch;                                   /* character used there, 0 none       */
	uint16_t lost_ms;                                /* VC line-lost timer (FAULT at 3000) */
	uint32_t bar_count, bar_err;
	uint16_t hump_mm, hump_last_mm, hump_max_mm, humps;   /* S4                */
	int16_t  pitch_x10, yaw_dps;
	uint8_t  es_why;                                 /* last IMU E-stop: 1 impact 2 tilt 3 both */
	uint16_t es_cg, es_tilt, a_pk_cg;                /* |a| 0.01 g, tilt deg, peak |a| 0.01 g   */
	int16_t  hdg;                                    /* compass heading, deg, left +; 999 none  */
	uint8_t  mclass, imu_state;
	uint16_t obs_mm, obs_w_mm, clr_l_mm, clr_r_mm;   /* S5 last scan / ping      */
	uint8_t  act;
	uint32_t scans;
	uint32_t faults;                                 /* flg_fault pattern        */
	uint32_t vc_drops;
	uint8_t  wdt_reset;
	uint8_t  vc_cause;                               /* VC FAULT cause (vc_fault_cause)  */
	uint8_t  ir_cal;                                 /* 1 = IR sensors calibrated (S3)   */
	int32_t  us_rise, us_fall;                       /* ultrasonic echo edges, us after trigger, -1 none */
	uint32_t us_edges;
	uint8_t  us_level;
	uint16_t us_w, us_mm;                           /* last ping: width used (us), result mm */
	uint8_t  us_st, s5_state;
	int16_t  servo_deg;                              /* scan servo angle (before trim)   */
	uint8_t  servo_manual;                           /* 1 = held by the dashboard        */                        /* ping status, scan task state (0 init 1 monitor 2 scanning 3 guard 4 fault) */
} s1_snapshot_t;

/* ---------------------------------------------------------------- payloads
 * Each returns the length written (no '\0' needed by MQTT), 0 if it did not fit. */
uint16_t s1_json_telemetry(char *buf, uint16_t size, const s1_snapshot_t *s);
uint16_t s1_json_heartbeat(char *buf, uint16_t size, uint32_t t_ms, uint32_t up_s,
			   int32_t rssi, uint32_t drops, uint32_t tx_fail);
/* Events published on change (se14/robot/event). prev may equal NULL on the first call.
 * Writes at most one event per call and returns its length (0 = nothing new).          */
uint16_t s1_json_next_event(char *buf, uint16_t size, const s1_snapshot_t *now, s1_snapshot_t *seen);
/* Plain-English one-liner for se14/robot/status, e.g.
 *   "FAULT: a wheel is stalled ... Press START (GP20) to reset."            */
uint16_t s1_status_text(char *buf, uint16_t size, const s1_snapshot_t *s);
const char *s1_fault_why(const s1_snapshot_t *s);
uint16_t s1_json_cmd_result(char *buf, uint16_t size, uint32_t t_ms, const char *cmd,
			    uint32_t seq, bool ok, const char *reason);

/* ---------------------------------------------------------------- commands */
typedef enum { S1_CMD_NONE = 0, S1_CMD_START, S1_CMD_STOP, S1_CMD_RESUME, S1_CMD_CANCEL, S1_CMD_RESET,
               S1_CMD_SERVO /* {"cmd":"servo","deg":0..180 or -1,"seq":N}: dashboard test */,
               S1_CMD_MAP   /* {"cmd":"map","m":"FULR","seq":N}: barcode characters for Forward,
                               U-turn, Left, Right (2026-11-11 demo: sent from the PC, not hard-coded) */ } s1_cmd_t;
typedef enum {
	S1_OK = 0,
	S1_REJ_TOO_LONG,        /* payload > 64 bytes            */
	S1_REJ_BAD_JSON,        /* no "cmd" string or "seq" number */
	S1_REJ_UNKNOWN,         /* cmd not on the list           */
	S1_REJ_SEQ,             /* seq not higher than the last accepted (duplicate / replay) */
	S1_REJ_STATE            /* not allowed in the current state */
} s1_rej_t;
#define S1_CMD_MAX_LEN   64u

/* VC state numbers (vc_status_byte() bits 2..0, vehicle_ctrl.c) */
#define S1_VC_INIT      0u
#define S1_VC_READY     1u
#define S1_VC_RUNNING   2u
#define S1_VC_PAUSED    3u
#define S1_VC_FAULT     4u
#define S1_VC_COMPLETE  5u

/* Remote codes for the VC hook s5_hook_remote_poll(): 0 nothing, 1 START, 2 STOP */
#define S1_REMOTE_NONE   0u
#define S1_REMOTE_START  1u
#define S1_REMOTE_STOP   2u

s1_rej_t    s1_cmd_parse(const char *p, uint16_t len, s1_cmd_t *cmd, uint32_t *seq);
/* optional integer field, e.g. "deg": true and *v if present and valid */
bool        s1_cmd_int(const char *p, uint16_t len, const char *key, int32_t *v);
/* the barcode map "m": exactly 4 different characters A-Z / 0-9 (or '-' = unused). Upper-cases. */
bool        s1_cmd_map(const char *p, uint16_t len, char m[4]);
const char *s1_act_name(uint8_t a);   /* S5_JA_*: "none" "forward" "uturn" "left" "right" */
/* State rules: start in READY; resume and cancel in PAUSED; reset only in FAULT
 * with the robot still; stop at any time (forwarded only while RUNNING).
 * *remote = what to give the VC (S1_REMOTE_*).                               */
s1_rej_t    s1_cmd_check(s1_cmd_t cmd, uint8_t vc_state, bool robot_still, uint8_t *remote);
const char *s1_cmd_name(s1_cmd_t c);
const char *s1_rej_name(s1_rej_t r);
const char *s1_vc_state_name(uint8_t st);

#endif /* S1_JSON_H */
