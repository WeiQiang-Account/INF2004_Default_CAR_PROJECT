/*
 *	s1_mqtt.c
 *	Subsystem 1 -- lwIP side: publishes telemetry / heartbeat / events and
 *	receives commands. Built only with  make NET=lwip MQTT=1  (TM_MQTT=1).
 *
 *	Everything here runs on the template's Wi-Fi task, as lwIP requires:
 *	mqtt_utk.c calls mqtt_utk_on_connected() (IF-13) from that task, and the
 *	200 ms tick is a sys_timeout() on the same task. Like mqtt_utk.c this file
 *	includes lwIP but NOT tk/tkernel.h; it reaches the kernel only through
 *	s1_bridge.h.
 *
 *	  tick every 200 ms: telemetry (QoS 0), up to 2 events (QoS 1),
 *	                     heartbeat + plain-English status every 5th tick = 1 s (QoS 0)
 *	  se14/robot/cmd   : each message -> s1_cmd_submit() -> result event
 */
#include "s1_bridge.h"

#if defined(TM_MQTT) && (TM_MQTT == 1)

#include "lwip/apps/mqtt.h"
#include "lwip/timeouts.h"

#define S1_T_TELEM	"se14/robot/telemetry"
#define S1_T_HB		"se14/robot/heartbeat"
#define S1_T_EVENT	"se14/robot/event"
#define S1_T_CMD	"se14/robot/cmd"
#define S1_T_STATUS	"se14/robot/status"
#define S1_TICK_MS	200u
#define S1_HB_TICKS	5u		/* heartbeat every 1 s */
#define S1_EVENTS_PER_TICK 2u

/* 1 = heartbeat reads the RSSI from the radio (cyw43 driver, same task). */
#ifndef S1_READ_RSSI
#define S1_READ_RSSI	1
#endif

#if S1_READ_RSSI
/* cyw43.h is not included (it clashes with this file's lwIP/T-Monitor headers,
 * see mqtt_utk.c); these two match its declarations. */
struct _cyw43_t;
extern struct _cyw43_t cyw43_state;
int cyw43_wifi_get_rssi(struct _cyw43_t *self, int32_t *rssi);
#endif

/* S2's optional Wi-Fi motion test (s2_telemetry_mqtt.c) hooks in here */
void s2_telemetry_test_on_connected(mqtt_client_t *c);
__attribute__((weak)) void s2_telemetry_test_on_connected(mqtt_client_t *c) { (void)c; }

static mqtt_client_t	*s_cl;
static uint32_t		s_t0, s_ticks, s_connects, s_tx_fail;
static s1_snapshot_t	s_seen;
static bool		s_seen_ok;
static bool		s_was_up;

static char		s_rx[S1_CMD_MAX_LEN + 1u];
static u16_t		s_rx_len;
static u32_t		s_rx_tot;
static bool		s_rx_cmd;

static void tick(void *arg);

static void pub_done(void *arg, err_t r)
{
	(void)arg;
	if (r != ERR_OK) { s_tx_fail++; }
}

static err_t pub(const char *topic, const char *buf, u16_t len, u8_t qos)
{
	err_t e;
	if ((len == 0u) || (s_cl == NULL)) { return ERR_ARG; }
	e = mqtt_publish(s_cl, topic, buf, len, qos, 0, pub_done, NULL);
	if (e != ERR_OK) { s_tx_fail++; }
	return e;
}

/* ---------------------------------------------------------------- commands */
static void in_topic(void *arg, const char *topic, u32_t tot_len)
{
	const char *c = S1_T_CMD;
	u16_t i = 0;
	(void)arg;
	while ((c[i] != '\0') && (topic[i] == c[i])) { i++; }
	s_rx_cmd = (c[i] == '\0') && (topic[i] == '\0');
	s_rx_len = 0;
	s_rx_tot = tot_len;
}

static void in_data(void *arg, const u8_t *data, u16_t len, u8_t flags)
{
	char	reply[112];
	u16_t	i, n;
	(void)arg;

	if (!s_rx_cmd) { return; }			/* debug topic etc.: ignored */
	for (i = 0; i < len; i++) {
		if (s_rx_len < S1_CMD_MAX_LEN) { s_rx[s_rx_len++] = (char)data[i]; }
	}
	if ((flags & MQTT_DATA_FLAG_LAST) != 0u) {
		/* a payload longer than 64 bytes is passed with its real length: rejected */
		u16_t full = (s_rx_tot > S1_CMD_MAX_LEN) ? (u16_t)(S1_CMD_MAX_LEN + 1u) : s_rx_len;
		n = s1_cmd_submit(s_rx, full, reply, (u16_t)sizeof(reply));
		(void)pub(S1_T_EVENT, reply, n, 1);
		s_rx_cmd = false;
	}
}

static void sub_done(void *arg, err_t r)
{
	(void)arg;
	s1_log((r == ERR_OK) ? "subscribed to " S1_T_CMD : "subscribe to " S1_T_CMD " FAILED");
}

/* IF-13: called by mqtt_utk.c on the Wi-Fi task each time the broker accepts us */
void mqtt_utk_on_connected(mqtt_client_t *client)
{
	s_cl = client;
	s_connects++;
	if (s_t0 == 0u) { s_t0 = s1_now_ms(); }

	/* replaces the template's echo callbacks (they only printed the debug topic) */
	mqtt_set_inpub_callback(client, in_topic, in_data, NULL);
	if (mqtt_subscribe(client, S1_T_CMD, 1, sub_done, NULL) != ERR_OK) {
		s1_log("mqtt_subscribe() failed");
	}
	sys_untimeout(tick, NULL);			/* one chain, even after a reconnect */
	sys_timeout(S1_TICK_MS, tick, NULL);
	s1_wifi_fault(false);
	s_was_up = true;
	s1_log("MQTT connected: telemetry 200 ms, heartbeat 1 s, events on change");

	s2_telemetry_test_on_connected(client);
}

/* ---------------------------------------------------------------- 200 ms tick */
static void tick(void *arg)
{
	static char	buf[1024];		/* static: the Wi-Fi task stack is small. 2026-11-11: 768 -> 1024 (barcode/junction fields) */
	static s1_snapshot_t	now, next;	/* static: big (barcode widths), Wi-Fi task stack is small */
	u16_t		n;
	uint32_t	k;
	(void)arg;

	if ((s_cl != NULL) && mqtt_client_is_connected(s_cl)) {
		s1_snapshot_take(&now);
		if (!s_seen_ok) {			/* first time: report the current state once */
			s_seen = now;
			s_seen.vc_state = 0xFFu;
			s_seen.wdt_reset = 0;
			s_seen_ok = true;
		}

		n = s1_json_telemetry(buf, (u16_t)sizeof(buf), &now);
		(void)pub(S1_T_TELEM, buf, n, 0);

		for (k = 0; k < S1_EVENTS_PER_TICK; k++) {
			next = s_seen;
			n = s1_json_next_event(buf, (u16_t)sizeof(buf), &now, &next);
			if (n == 0u) { break; }
			if (pub(S1_T_EVENT, buf, n, 1) != ERR_OK) { break; }   /* retried next tick */
			s_seen = next;
		}

		if ((++s_ticks % S1_HB_TICKS) == 0u) {
			int32_t rssi = 0;
#if S1_READ_RSSI
			if (cyw43_wifi_get_rssi(&cyw43_state, &rssi) != 0) { rssi = 0; }
#endif
			n = s1_json_heartbeat(buf, (u16_t)sizeof(buf), now.t_ms, (now.t_ms - s_t0) / 1000u,
					      rssi, (s_connects > 0u) ? (s_connects - 1u) : 0u, s_tx_fail);
			(void)pub(S1_T_HB, buf, n, 0);
			n = s1_status_text(buf, (u16_t)sizeof(buf), &now);
			(void)pub(S1_T_STATUS, buf, n, 0);	/* plain English, for people */
		}
	} else if (s_was_up) {
		s_was_up = false;
		s1_wifi_fault(true);			/* mqtt.c reconnects on its own */
		s1_log("MQTT link lost");
	}
	sys_timeout(S1_TICK_MS, tick, NULL);
}

#endif /* TM_MQTT */
