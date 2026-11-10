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

#endif /* S1_BRIDGE_H */
