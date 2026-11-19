/* s1_bridge.h - what the lwIP half of subsystem 1 (s1_mqtt.c) may call in the
 * kernel half (subsystem1.c). Plain C types only: s1_mqtt.c includes lwIP and
 * must NOT include tk/tkernel.h (same rule as the template's mqtt_utk.c).     */
#ifndef S1_BRIDGE_H
#define S1_BRIDGE_H

#include "s1_json.h"

/* One consistent picture of the robot from the other subsystems' getters. */
void     s1_snapshot_take(s1_snapshot_t *s);

/* Validate one command payload (size, JSON, seq, state) and, if accepted, hand
 * it to the VC. Writes the cmd_accepted / cmd_rejected event into reply and
 * returns its length. len may be larger than the payload kept (too long).     */
uint16_t s1_cmd_submit(const char *p, uint16_t len, char *reply, uint16_t size);

uint32_t s1_now_ms(void);
void     s1_wifi_fault(bool lost);	/* FAULT_BIT_WIFI in flg_fault (telemetry only) */
void     s1_log(const char *msg);	/* console line, prefixed "[S1] " */
/* barcode test page: stream on? and the barcode-sensor samples since *from (5 ms each) */
bool     s1_bcraw_on(void);
bool     s1_bcraw_all3(void);         /* recorded run: left + centre + right */
uint16_t s1_bcraw_get3(uint32_t *from, uint8_t *l, uint8_t *c, uint8_t *r, uint16_t max);
uint16_t s1_bcraw_get(uint32_t *from, uint8_t *out, uint16_t max);

#endif /* S1_BRIDGE_H */
