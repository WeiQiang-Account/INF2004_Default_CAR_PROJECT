/*
 *	s2_os_port.h
 *	Subsystem 2 -- the few kernel calls the motion core uses, behind a thin
 *	wrapper so the same motion code also runs in the PC tests
 *	(testing/subsystem2/host_tests/), where these are mocked.
 *
 *	On the robot they map 1:1 onto micro T-Kernel (s2_os_port.c).
 */

#ifndef S2_OS_PORT_H
#define S2_OS_PORT_H

#include <stdint.h>
#include <stdbool.h>

typedef int32_t os_id_t;

/* Event flags */
void     os_flag_set(os_id_t flg, uint32_t bits);
void     os_flag_clear(os_id_t flg, uint32_t bits);
bool     os_flag_poll(os_id_t flg, uint32_t bits);
/* Wait until any of bits is set or timeout_ms passes. Does not clear.
   Returns true if a bit was set, false on timeout. */
bool     os_flag_wait_any(os_id_t flg, uint32_t bits, uint32_t timeout_ms);

/* Message buffers */
bool     os_mbf_send(os_id_t mbf, const void *p_msg, uint32_t size, int32_t timeout_ms);
int32_t  os_mbf_recv_poll(os_id_t mbf, void *p_msg);   /* size, or -1 if empty */

/* Time */
uint32_t os_now_ms(void);
void     os_sleep_ms(uint32_t ms);

#endif /* S2_OS_PORT_H */
