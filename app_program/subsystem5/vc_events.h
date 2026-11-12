/* vc_events.h - event word format for mbf_vc_events (IF: all tasks -> Vehicle Controller).
 * PROPOSED shared format - agree it with the team before others post to it.
 * Every event is one 32-bit word: type in bits [31:28], 28-bit argument.
 * PROFILE events are two words (8 bytes, see s5_profile_pack).
 * Explicit shifts/masks only (Tribunal: wire formats must not depend on C bit-field layout). */
#ifndef VC_EVENTS_H
#define VC_EVENTS_H
#include <stdint.h>

enum {
    VC_EV_NONE = 0,
    VC_EV_BARCODE,       /* S3: arg = valid<<7 | character (ASCII, 7 bits). 2026-11-11: the VC maps the
                            character to a movement with the table sent over MQTT, at the next junction */
    VC_EV_JUNCTION,      /* S3 */
    VC_EV_LINE_LOST,     /* S3 */
    VC_EV_LINE_FOUND,    /* S3 */
    VC_EV_HUMP_START,    /* S4 */
    VC_EV_HUMP_DONE,     /* S4: arg = peak height mm (9 bits) */
    VC_EV_IMPACT,        /* S4 */
    VC_EV_IMU_LOST,      /* S4 */
    VC_EV_OBSTACLE,      /* S5: arg = distance mm */
    VC_EV_PROFILE,       /* S5: 2 words */
    VC_EV_TOO_CLOSE,     /* S5 bypass guard */
    VC_EV_NO_PATH,       /* S5: rescans exhausted */
    VC_EV_SENSOR_FAULT,  /* S5: arg 1 = fault, 0 = recovered */
    VC_EV_FINISH,        /* S3: finish barcode (A14 still to be confirmed) */
    VC_EV_RESERVED
};

#define VC_EV_MAKE(type, arg) (((uint32_t)(type) << 28) | ((uint32_t)(arg) & 0x0FFFFFFFu))
#define VC_EV_TYPE(w)         ((unsigned)((w) >> 28))
#define VC_EV_ARG(w)          ((uint32_t)((w) & 0x0FFFFFFFu))
#define VC_BARCODE_VALID(arg) (((arg) >> 7) & 1u)
#define VC_BARCODE_CHAR(arg)  ((char)((arg) & 0x7Fu))
#define VC_EV_MAX_BYTES       8

/* Safe from any task: non-blocking (TMO_POL). Returns 0 = queued, -1 = dropped
 * (the drop counter vc_dropped_events() is the Tribunal "stays at 0" evidence). */
int      vc_post(uint32_t w0);
int      vc_post2(uint32_t w0, uint32_t w1);
uint32_t vc_dropped_events(void);
uint8_t  vc_fault_cause(void);   /* 0 none, 1 init, 2 impact, 3 line lost, 4 timeout, 5 sensor, 6 recovery, 7 motion */
/* Ultrasonic diagnostics: first rising / falling ECHO edge of the last ping, in us after
   the trigger (0xFFFFFFFF = none), total edges since boot, ECHO pin level now. */
void     s5_echo_diag(uint32_t *rise_us, uint32_t *fall_us, uint32_t *edges, uint32_t *level);
/* Result of the last us_ping(): pulse width it used (us), status 0 ok / 1 no echo / 2 no sensor, mm. */
void     s5_ping_diag(uint32_t *width_us, uint32_t *status, uint32_t *mm);
/* Manual servo (dashboard test): deg 0..180 holds that angle, -1 = automatic
   (back to straight ahead). The VC puts it back to automatic on START.       */
void     s5_servo_manual(int deg);
int      s5_servo_angle(void);      /* angle last commanded, before the trim  */
int      s5_servo_is_manual(void);

/* Status bytes for telemetry (single-byte reads/writes are atomic on RP2040). */
uint8_t  vc_status_byte(void);   /* state[2:0] | nav[4:3] | on_hump[5] | degraded[6] */
uint8_t  s5_status_byte(void);   /* state[7:5] | stage[4:3] | action[2:0]            */
uint16_t s5_last_distance_mm(void);

/* Barcode -> movement at the next cross-junction (2026-11-11 demo rules).
   The mapping is NOT hard-coded: S1 sets it from MQTT {"cmd":"map","m":"FULR",...}:
   4 characters for Forward, U-turn, Left, Right. Unmapped / nothing stored = forward. */
enum { S5_JA_NONE = 0, S5_JA_FORWARD, S5_JA_UTURN, S5_JA_LEFT, S5_JA_RIGHT };
void     s5_set_barcode_map(const char m[4]);   /* any task; 0 / '-' = unset           */
void     s5_get_barcode_map(char m[4]);         /* '-' where unset                     */
char     s5_barcode_pending(void);              /* stored, waiting for a junction; 0 = none */
uint8_t  s5_barcode_action(char c);             /* S5_JA_* the map gives this character */
uint16_t s5_junction_count(void);               /* junctions handled by the VC         */
uint8_t  s5_junction_last_act(void);            /* S5_JA_* done at the last junction   */
char     s5_junction_last_char(void);           /* character used there, 0 = none      */
uint16_t s5_line_lost_ms(void);                 /* VC line-lost timer now, 0 = line seen */
#endif
