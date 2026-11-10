/*
 *	s1_selftest.c
 *	Subsystem 1 -- checks of the PURE logic (s1_json.c). No hardware, no kernel.
 */
#include "s1_selftest.h"
#include "s1_json.h"

static bool has(const char *b, uint16_t n, const char *w)
{
	uint16_t i, k;
	for (i = 0; i < n; i++) {
		for (k = 0; (w[k] != '\0') && ((uint16_t)(i + k) < n) && (b[i + k] == w[k]); k++) { }
		if (w[k] == '\0') { return true; }
	}
	return false;
}

static uint16_t LEN(const char *s) { uint16_t i = 0; while (s[i] != '\0') { i++; } return i; }

static s1_snapshot_t busy(void)		/* every field at a large value: longest JSON */
{
	s1_snapshot_t s = {0};
	s.t_ms = 4000000000u; s.vc_state = S1_VC_COMPLETE; s.vc_nav = 3; s.m_state = 6;
	s.spd_l = -1024; s.spd_r = -1024; s.enc_l = -2000000000; s.enc_r = -2000000000;
	s.m_cause = 4; s.enc_err_l = 99999u; s.enc_err_r = 99999u;
	s.es_why = 3; s.es_cg = 65535; s.es_tilt = 65535; s.a_pk_cg = 65535; s.hdg = -180;
	s.dist_mm = 65535; s.line = -100; s.ir[0] = s.ir[1] = s.ir[2] = 4095; s.bar = 3;
	s.hump_mm = 511; s.hump_max_mm = 511; s.pitch_x10 = -900; s.mclass = 3;
	s.obs_mm = 4000; s.obs_w_mm = 4000; s.clr_l_mm = 4000; s.clr_r_mm = 4000; s.act = 4;
	s.faults = 0xFFFFFFFFu;
	s.us_rise = 2000000000; s.us_fall = 2000000000; s.us_edges = 4000000000u; s.us_level = 1;
	s.us_w = 65535; s.us_st = 2; s.us_mm = 65535; s.s5_state = 4;
	s.servo_deg = -180; s.servo_manual = 1;
	return s;
}

/* S1-U01: telemetry is one flat JSON object with the design's fields; the
   longest case stays under 700 B (the publish buffer in s1_mqtt.c is 768 B) */
bool s1_st_telemetry(void)
{
	static char b[768];	/* static: the test runs on the 1 KB initial-task stack (2026-11-07: on the stack it crashed the TEST build) */
	s1_snapshot_t s = busy();
	uint16_t n = s1_json_telemetry(b, sizeof b, &s);
	return (n > 0u) && (n < 700u) && (b[0] == '{') && (b[n - 1u] == '}') &&
	       has(b, n, "\"state\":\"COMPLETE\"") && has(b, n, "\"enc_l\":-2000000000") &&
	       has(b, n, "\"ir\":[4095,4095,4095]") && has(b, n, "\"bar\":\"uturn\"") &&
	       has(b, n, "\"pitch\":-900") && has(b, n, "\"fault\":4294967295");
}

/* S1-U02: command parser - good, bad, unknown, too long */
bool s1_st_parse(void)
{
	s1_cmd_t c; uint32_t q; bool ok = true;
	const char *a = "{\"cmd\":\"start\",\"seq\":7}";
	const char *b = "{ \"seq\" : 12 , \"cmd\" : \"reset\" }";
	const char *u = "{\"cmd\":\"dance\",\"seq\":3}";
	const char *x = "{\"cmd\":\"stop\"}";
	const char *y = "{\"cmd\":\"stopp\",\"seq\":1}";
	const char *l = "{\"cmd\":\"stop\",\"seq\":1,\"pad\":\"xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx\"}";
	ok = ok && (s1_cmd_parse(a, LEN(a), &c, &q) == S1_OK) && (c == S1_CMD_START) && (q == 7u);
	ok = ok && (s1_cmd_parse(b, LEN(b), &c, &q) == S1_OK) && (c == S1_CMD_RESET) && (q == 12u);
	ok = ok && (s1_cmd_parse(u, LEN(u), &c, &q) == S1_REJ_UNKNOWN);
	ok = ok && (s1_cmd_parse(x, LEN(x), &c, &q) == S1_REJ_BAD_JSON);
	ok = ok && (s1_cmd_parse(y, LEN(y), &c, &q) == S1_REJ_UNKNOWN);
	ok = ok && (s1_cmd_parse(l, LEN(l), &c, &q) == S1_REJ_TOO_LONG);
	ok = ok && (s1_cmd_parse("hello", 5, &c, &q) == S1_REJ_BAD_JSON);
	{	/* dashboard servo test: {"cmd":"servo","deg":165,"seq":4} and deg -1 */
		const char *sv = "{\"cmd\":\"servo\",\"deg\":165,\"seq\":4}";
		const char *sa = "{\"cmd\":\"servo\",\"deg\":-1,\"seq\":5}";
		int32_t d = 0;
		ok = ok && (s1_cmd_parse(sv, LEN(sv), &c, &q) == S1_OK) && (c == S1_CMD_SERVO) && (q == 4u);
		ok = ok && s1_cmd_int(sv, LEN(sv), "deg", &d) && (d == 165);
		ok = ok && s1_cmd_int(sa, LEN(sa), "deg", &d) && (d == -1);
		ok = ok && !s1_cmd_int(a, LEN(a), "deg", &d);
	}
	return ok;
}

/* S1-U03: state rules (start READY; resume/cancel PAUSED; reset FAULT + still; stop any) */
bool s1_st_rules(void)
{
	uint8_t r; bool ok = true;
	ok = ok && (s1_cmd_check(S1_CMD_START, S1_VC_READY, true, &r) == S1_OK) && (r == S1_REMOTE_START);
	ok = ok && (s1_cmd_check(S1_CMD_START, S1_VC_RUNNING, true, &r) == S1_REJ_STATE) && (r == S1_REMOTE_NONE);
	ok = ok && (s1_cmd_check(S1_CMD_RESUME, S1_VC_PAUSED, true, &r) == S1_OK) && (r == S1_REMOTE_START);
	ok = ok && (s1_cmd_check(S1_CMD_RESUME, S1_VC_READY, true, &r) == S1_REJ_STATE);
	ok = ok && (s1_cmd_check(S1_CMD_CANCEL, S1_VC_PAUSED, true, &r) == S1_OK) && (r == S1_REMOTE_STOP);
	ok = ok && (s1_cmd_check(S1_CMD_RESET, S1_VC_FAULT, true, &r) == S1_OK) && (r == S1_REMOTE_START);
	ok = ok && (s1_cmd_check(S1_CMD_RESET, S1_VC_FAULT, false, &r) == S1_REJ_STATE);   /* still moving */
	ok = ok && (s1_cmd_check(S1_CMD_STOP, S1_VC_RUNNING, false, &r) == S1_OK) && (r == S1_REMOTE_STOP);
	ok = ok && (s1_cmd_check(S1_CMD_STOP, S1_VC_PAUSED, true, &r) == S1_OK) && (r == S1_REMOTE_NONE);
	ok = ok && (s1_cmd_check(S1_CMD_STOP, S1_VC_INIT, true, &r) == S1_OK) && (r == S1_REMOTE_NONE);
	ok = ok && (s1_cmd_check(S1_CMD_SERVO, S1_VC_READY, true, &r) == S1_OK) && (r == S1_REMOTE_NONE);
	ok = ok && (s1_cmd_check(S1_CMD_SERVO, S1_VC_RUNNING, false, &r) == S1_REJ_STATE);
	return ok;
}

/* S1-U04: events - each change reported once, watchdog reset first */
bool s1_st_events(void)
{
	char b[160];
	static s1_snapshot_t seen, now;	/* static: 1 KB initial-task stack in the TEST build */
	static const s1_snapshot_t zero = {0};
	uint16_t n;
	bool ok = true;
	seen = zero; now = zero;

	ok = ok && (s1_json_next_event(b, sizeof b, &now, &seen) == 0u);          /* nothing new */
	now.wdt_reset = 1; now.vc_state = S1_VC_READY; now.humps = 1; now.hump_last_mm = 20;
	n = s1_json_next_event(b, sizeof b, &now, &seen);
	ok = ok && has(b, n, "\"ev\":\"WATCHDOG_RESET\"");
	n = s1_json_next_event(b, sizeof b, &now, &seen);
	ok = ok && has(b, n, "\"from\":\"INIT\"") && has(b, n, "\"to\":\"READY\"");
	n = s1_json_next_event(b, sizeof b, &now, &seen);
	ok = ok && has(b, n, "\"ev\":\"hump\"") && has(b, n, "\"peak_mm\":20");
	ok = ok && (s1_json_next_event(b, sizeof b, &now, &seen) == 0u);          /* all sent */
	n = s1_json_cmd_result(b, sizeof b, 5, "start", 9, false, "old_seq");
	ok = ok && has(b, n, "\"ev\":\"cmd_rejected\"") && has(b, n, "\"reason\":\"old_seq\"");

	/* plain-English status: a motion stall says so, and how to recover */
	now.vc_state = S1_VC_FAULT; now.vc_cause = 7; now.m_cause = 2;
	n = s1_status_text(b, sizeof b, &now);
	ok = ok && has(b, n, "FAULT: a wheel stalled") && has(b, n, "Press START");
	n = s1_json_next_event(b, sizeof b, &now, &seen);
	ok = ok && has(b, n, "\"to\":\"FAULT\"") && has(b, n, "\"why\":\"a wheel stalled");
	now.vc_state = S1_VC_INIT; now.m_state = 1; now.imu_state = 1; now.ir_cal = 0;
	n = s1_status_text(b, sizeof b, &now);
	ok = ok && has(b, n, "INIT: waiting for [IR calibration");
	return ok;
}

/* S1-U05: a buffer that is too small gives 0 (never a cut-off JSON) */
bool s1_st_small_buf(void)
{
	char b[64];
	s1_snapshot_t s = busy();
	return (s1_json_telemetry(b, sizeof b, &s) == 0u) &&
	       (s1_json_heartbeat(b, 20, 1, 2, -50, 0, 0) == 0u) &&
	       (s1_json_heartbeat(b, sizeof b, 1, 2, -50, 0, 0) > 0u);
}
