/*
 *	subsystem1.h
 *	Subsystem 1 -- Buddy 1: Wi-Fi telemetry and remote commands.  Student ID: 2502287.
 *
 *	Public interface of subsystem 1 (TEAM.md section 5):
 *	  IF-13  template -> S1   mqtt_utk_on_connected()  (s1_mqtt.c, Wi-Fi task)
 *	  IF-15  S1 -> VC         int s5_hook_remote_poll(void)  0 none / 1 START / 2 STOP
 *	  S1 reads only the other subsystems' read-only getters; it never drives the
 *	  motors and never posts to the VC queue.
 *
 *	Split in two halves because lwIP and tk/tkernel.h must not meet in one file:
 *	  subsystem1.c  kernel side: event flag, snapshot, command checks (this header)
 *	  s1_mqtt.c     lwIP side, runs ONLY on the Wi-Fi task (s1_bridge.h, plain types)
 *	  s1_json.c     pure logic, also compiled on a PC (testing/subsystem1/host_tests)
 *
 *	MQTT topics (Week 6, 6.5):
 *	  se14/robot/telemetry  every 200 ms, QoS 0
 *	  se14/robot/heartbeat  every 1 s,    QoS 0
 *	  se14/robot/event      on change,    QoS 1 (state, fault, barcode, hump, obstacle, imu, cmd result)
 *	  se14/robot/cmd        subscribed,   QoS 1  {"cmd":"start|stop|resume|cancel|reset","seq":N}
 */
#ifndef SUBSYSTEM1_H
#define SUBSYSTEM1_H

#include "../app.h"
#include "s1_json.h"

/* Remote command flag (S1 -> VC). Bit = 1 << (VC state the command was checked
 * in), +8 for STOP; the VC hook drops a command whose state has changed since. */
IMPORT ID	flg_remote_cmd;
#define S1_FLG_START(st)	(1u << ((st) & 7u))
#define S1_FLG_STOP(st)		(1u << (8u + ((st) & 7u)))
#define S1_FLG_ALL		0x00003F3Fu

#define S1_STILL_MMPS		20	/* "robot still" for reset: both wheels slower than this */

#endif /* SUBSYSTEM1_H */
