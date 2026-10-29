/*
 *	s2_telemetry_mqtt.c
 *	Subsystem 2 -- QUICK Wi-Fi telemetry test (Buddy 2).
 *
 *	NOT the team's telemetry subsystem: that is Buddy 1's (subsystem 1).
 *	This only lets Buddy 2 watch the motion data over Wi-Fi while the robot
 *	runs on the power bank (no USB). Built only with
 *	    make NET=lwip MQTT=1      (TM_MQTT=1)
 *	and S2_TELEMETRY_TEST = 1 (motion_config.h). Set it to 0 when Buddy 1's
 *	telemetry takes over.
 *
 *	Every 200 ms it publishes one JSON line on "se14/robot/telemetry", e.g.
 *	  {"seq":12,"state":"DRIVING","cause":0,"v_l":201,"v_r":0,
 *	   "dist_mm":350,"enc_l":2210,"enc_r":0,"d_l":46,"d_r":0}
 *
 *	All lwIP calls run on the template's Wi-Fi task (core 0), as lwIP
 *	requires: the template's mqtt_utk.c calls mqtt_utk_on_connected() from
 *	that task, and sys_timeout() keeps the 200 ms tick on that same task.
 *	Reading the motion status from here is safe: motion_get_status() uses
 *	a sequence lock and encoder_read() reads 32-bit words.
 *
 *	Like mqtt_utk.c, this file includes lwIP but NOT tk/tkernel.h.
 */

#include "motion_config.h"

#if defined(TM_MQTT) && (TM_MQTT == 1) && (S2_TELEMETRY_TEST != 0)

#include "lwip/apps/mqtt.h"
#include "lwip/timeouts.h"
#include <tm/tmonitor.h>
#include "motion.h"
#include "drv_encoder.h"

#define S2_TELEM_TOPIC		"se14/robot/telemetry"
#define S2_TELEM_PERIOD_MS	(200)

/* Current move of the drive pattern (s2_telemetry_drive.c) */
extern const char * volatile g_s2_drive_step;

static mqtt_client_t	*s_client;
static unsigned int	s_seq;
static unsigned int	s_fail;

static void telem_tick(void *arg);

static const char *state_name(int s)
{
	static const char *const k_names[] = {
		"INIT", "IDLE", "DRIVING", "DIST_MOVE", "TURNING", "STOPPING", "FAULT"
	};
	return ((s >= 0) && (s <= 6)) ? k_names[s] : "?";
}

static void publish_done(void *arg, err_t result)
{
	(void)arg;
	if (result != ERR_OK) {
		s_fail++;
	}
}

/* Called by mqtt_utk.c (template) each time the broker accepts us. */
void mqtt_utk_on_connected(mqtt_client_t *client)
{
	s_client = client;
	sys_untimeout(telem_tick, NULL);	/* only one chain, even after a reconnect */
	sys_timeout(S2_TELEM_PERIOD_MS, telem_tick, NULL);
	tm_printf((UB*)"s2-telem: publishing on " S2_TELEM_TOPIC " every %d ms\n",
		  S2_TELEM_PERIOD_MS);
}

static void telem_tick(void *arg)
{
	motion_status_t	st;
	int32_t		enc_l, enc_r;
	UB		buf[240];
	INT		len;

	(void)arg;

	if ((s_client != NULL) && mqtt_client_is_connected(s_client)) {
		motion_get_status(&st);
		encoder_read(&enc_l, &enc_r);
		len = tm_sprintf(buf, (const UB*)
			"{\"seq\":%u,\"step\":\"%s\",\"state\":\"%s\",\"cause\":%d,\"v_l\":%d,\"v_r\":%d,"
			"\"dist_mm\":%u,\"enc_l\":%d,\"enc_r\":%d,\"d_l\":%d,\"d_r\":%d}",
			s_seq++, g_s2_drive_step, state_name((int)st.state), (int)st.cause,
			(int)st.speed_l_mmps, (int)st.speed_r_mmps,
			(unsigned int)st.distance_mm, (int)enc_l, (int)enc_r,
			(int)st.delta_l, (int)st.delta_r);
		if (len > 0) {
			/* QoS 0: if the send buffer is full this one is skipped */
			(void)mqtt_publish(s_client, S2_TELEM_TOPIC, buf, (u16_t)len,
					   0, 0, publish_done, NULL);
		}
		if ((s_seq % 50u) == 0u) {	/* every 10 s on the USB console */
			tm_printf((UB*)"s2-telem: %u sent, %u failed, last: %s\n",
				  s_seq, s_fail, buf);
		}
	}
	sys_timeout(S2_TELEM_PERIOD_MS, telem_tick, NULL);
}

#endif /* TM_MQTT && S2_TELEMETRY_TEST */
