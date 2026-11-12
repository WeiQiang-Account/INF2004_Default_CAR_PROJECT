/*
 *	s1_json.c
 *	Subsystem 1 -- MQTT payloads and command validation. PURE (no kernel, no lwIP).
 *	Flat JSON, field names from the Week 6 design (6.5).
 */
#include "s1_json.h"

/* ================================================================ tiny writer */
typedef struct { char *b; uint16_t n, size; bool ok; } w_t;

static void put_c(w_t *w, char c)
{
	if ((uint16_t)(w->n + 1u) < w->size) { w->b[w->n++] = c; } else { w->ok = false; }
}
static void put_s(w_t *w, const char *s)
{
	while (*s != '\0') { put_c(w, *s++); }
}
static void put_i(w_t *w, int32_t v)
{
	char t[12];
	int  i = 0;
	uint32_t u = (v < 0) ? (uint32_t)(-(v + 1)) + 1u : (uint32_t)v;

	if (v < 0) { put_c(w, '-'); }
	do { t[i++] = (char)('0' + (u % 10u)); u /= 10u; } while ((u != 0u) && (i < 11));
	while (i > 0) { put_c(w, t[--i]); }
}
static void put_u(w_t *w, uint32_t u)
{
	char t[11];
	int  i = 0;
	do { t[i++] = (char)('0' + (u % 10u)); u /= 10u; } while ((u != 0u) && (i < 10));
	while (i > 0) { put_c(w, t[--i]); }
}
static void key(w_t *w, const char *k)        /* ,"k": */
{
	if (w->n > 1u) { put_c(w, ','); }
	put_c(w, '"'); put_s(w, k); put_s(w, "\":");
}
static void kv_i(w_t *w, const char *k, int32_t v)  { key(w, k); put_i(w, v); }
static void kv_u(w_t *w, const char *k, uint32_t v) { key(w, k); put_u(w, v); }
static void kv_s(w_t *w, const char *k, const char *v) { key(w, k); put_c(w, '"'); put_s(w, v); put_c(w, '"'); }
static void kv_b(w_t *w, const char *k, bool v)     { key(w, k); put_s(w, v ? "true" : "false"); }
static void begin(w_t *w, char *b, uint16_t size) { w->b = b; w->n = 0; w->size = size; w->ok = (size > 2u); put_c(w, '{'); }
static uint16_t end(w_t *w) { put_c(w, '}'); return w->ok ? w->n : 0u; }

/* ================================================================ names */
const char *s1_vc_state_name(uint8_t st)
{
	static const char *const n[] = { "INIT", "READY", "RUNNING", "PAUSED", "FAULT", "COMPLETE" };
	return (st < 6u) ? n[st] : "?";
}
static const char *nav_name(uint8_t v)
{
	static const char *const n[] = { "LINE", "BARCODE_TURN", "OBSTACLE", "RECOVERY" };
	return (v < 4u) ? n[v] : "?";
}
static void put_ch(w_t *w, char c)            /* one character as a JSON string body */
{
	if ((c == '"') || (c == '\\')) { put_c(w, '\\'); }
	put_c(w, c);
}
static void kv_ch(w_t *w, const char *k, char c)   /* "k":"X" or "k":"" */
{
	key(w, k); put_c(w, '"'); if ((c >= ' ') && (c <= '~')) { put_ch(w, c); } put_c(w, '"');
}
const char *s1_act_name(uint8_t a)
{
	static const char *const n[] = { "none", "forward", "uturn", "left", "right" };
	return (a < 5u) ? n[a] : "?";
}
static const char *act_name(uint8_t a)
{
	static const char *const n[] = { "continue", "stop", "turn_left", "turn_right", "reverse" };
	return (a < 5u) ? n[a] : "?";
}
static const char *mclass_name(uint8_t c)
{
	static const char *const n[] = { "stationary", "moving", "turning", "accelerating" };
	return (c < 4u) ? n[c] : "?";
}
static const char *motion_name(uint8_t m)
{
	static const char *const n[] = { "INIT", "IDLE", "DRIVING", "DIST_MOVE", "TURNING", "STOPPING", "FAULT" };
	return (m < 7u) ? n[m] : "?";
}
const char *s1_cmd_name(s1_cmd_t c)
{
	static const char *const n[] = { "none", "start", "stop", "resume", "cancel", "reset", "servo", "map" };
	return ((unsigned)c < 8u) ? n[c] : "?";
}
const char *s1_rej_name(s1_rej_t r)
{
	static const char *const n[] = { "ok", "too_long", "bad_json", "unknown_cmd", "old_seq", "not_allowed_now" };
	return ((unsigned)r < 6u) ? n[r] : "?";
}

/* ================================================================ payloads */
uint16_t s1_json_telemetry(char *buf, uint16_t size, const s1_snapshot_t *s)
{
	w_t w;
	begin(&w, buf, size);
	kv_u(&w, "t", s->t_ms);
	kv_s(&w, "state", s1_vc_state_name(s->vc_state));
	kv_b(&w, "ir_cal", s->ir_cal != 0u);
	kv_s(&w, "nav", nav_name(s->vc_nav));
	kv_s(&w, "motion", motion_name(s->m_state));
	kv_u(&w, "m_cause", s->m_cause);           /* 1 E-stop, 2 stall, 3 encoder, 4 self-test */
	kv_i(&w, "spd_l", s->spd_l);
	kv_i(&w, "spd_r", s->spd_r);
	kv_i(&w, "enc_l", s->enc_l);
	kv_i(&w, "enc_r", s->enc_r);
	kv_u(&w, "ee_l", s->enc_err_l);            /* skipped encoder steps (capped 99999) */
	kv_u(&w, "ee_r", s->enc_err_r);
	kv_u(&w, "dist", s->dist_mm);
	kv_i(&w, "line", s->line);
	key(&w, "ir");
	put_c(&w, '['); put_u(&w, s->ir[0]); put_c(&w, ','); put_u(&w, s->ir[1]); put_c(&w, ','); put_u(&w, s->ir[2]); put_c(&w, ']');
	kv_ch(&w, "bar", s->bar);                   /* last barcode character, "" none     */
	kv_ch(&w, "bar_p", s->bar_pend);            /* stored for the next junction        */
	kv_s(&w, "bar_act", s1_act_name(s->bar_act));
	key(&w, "map"); put_c(&w, '"'); { int i; for (i = 0; i < 4; i++) { put_ch(&w, s->map[i] ? s->map[i] : '-'); } } put_c(&w, '"');
	kv_u(&w, "bc_edges", s->bar_edges);
	kv_u(&w, "bc_ok", s->bar_count);
	kv_u(&w, "bc_err", s->bar_err);
	kv_u(&w, "bc_why", s->bar_why);
	kv_u(&w, "bc_n", s->bar_n);                 /* reading now: bars/gaps so far of 29 */
	kv_u(&w, "junc_s", s->junc_seen);
	kv_u(&w, "junc", s->junc);
	kv_s(&w, "j_act", s1_act_name(s->j_act));
	kv_ch(&w, "j_ch", s->j_ch);
	kv_b(&w, "lost", s->line_lost != 0u);
	kv_u(&w, "lost_ms", s->lost_ms);
	kv_u(&w, "hump_mm", s->hump_mm);
	kv_u(&w, "hump_max_mm", s->hump_max_mm);
	kv_i(&w, "pitch", s->pitch_x10);               /* 0.1 deg */
	kv_u(&w, "es", s->es_why);                     /* last E-stop: 1 impact, 2 tilt, 3 both */
	kv_u(&w, "es_cg", s->es_cg);
	kv_u(&w, "es_tilt", s->es_tilt);
	kv_u(&w, "a_pk", s->a_pk_cg);
	kv_i(&w, "hdg", s->hdg);                      /* compass, deg, left +; 999 = none */
	kv_s(&w, "mclass", mclass_name(s->mclass));
	kv_i(&w, "yaw", s->yaw_dps);
	kv_b(&w, "on_hump", s->on_hump != 0u);
	kv_b(&w, "degraded", s->degraded != 0u);    /* IMU lost: no hump detection */
	kv_u(&w, "obs_cm", (s->obs_mm == 0xFFFFu) ? 0u : (uint32_t)(s->obs_mm / 10u));
	kv_u(&w, "obs_w_cm", (uint32_t)(s->obs_w_mm / 10u));
	kv_u(&w, "clr_l_cm", (uint32_t)(s->clr_l_mm / 10u));
	kv_u(&w, "clr_r_cm", (uint32_t)(s->clr_r_mm / 10u));
	kv_s(&w, "act", act_name(s->act));
	kv_i(&w, "us_rise", s->us_rise);           /* echo diagnostics, us after trigger */
	kv_i(&w, "us_fall", s->us_fall);
	kv_u(&w, "us_edges", s->us_edges);
	kv_u(&w, "us_pin", s->us_level);
	kv_u(&w, "us_w", s->us_w);
	kv_u(&w, "us_st", s->us_st);
	kv_u(&w, "us_mm", s->us_mm);
	kv_u(&w, "s5", s->s5_state);
	kv_i(&w, "servo", s->servo_deg);
	kv_b(&w, "servo_man", s->servo_manual != 0u);
	kv_u(&w, "fault", s->faults);
	return end(&w);
}

uint16_t s1_json_heartbeat(char *buf, uint16_t size, uint32_t t_ms, uint32_t up_s,
			   int32_t rssi, uint32_t drops, uint32_t tx_fail)
{
	w_t w;
	begin(&w, buf, size);
	kv_u(&w, "t", t_ms);
	kv_u(&w, "up_s", up_s);
	kv_i(&w, "rssi", rssi);
	kv_u(&w, "drops", drops);
	kv_u(&w, "tx_fail", tx_fail);
	return end(&w);
}

uint16_t s1_json_cmd_result(char *buf, uint16_t size, uint32_t t_ms, const char *cmd,
			    uint32_t seq, bool ok, const char *reason)
{
	w_t w;
	begin(&w, buf, size);
	kv_u(&w, "t", t_ms);
	kv_s(&w, "ev", ok ? "cmd_accepted" : "cmd_rejected");
	kv_s(&w, "cmd", cmd);
	kv_u(&w, "seq", seq);
	if (!ok) { kv_s(&w, "reason", reason); }
	return end(&w);
}

/* One event per call, in a fixed order; 'seen' remembers what was already sent. */
uint16_t s1_json_next_event(char *buf, uint16_t size, const s1_snapshot_t *s, s1_snapshot_t *seen)
{
	w_t w;
	begin(&w, buf, size);
	kv_u(&w, "t", s->t_ms);

	if (s->wdt_reset && !seen->wdt_reset) {
		seen->wdt_reset = 1;
		kv_s(&w, "ev", "WATCHDOG_RESET");               /* NFR20 */
	} else if (s->vc_state != seen->vc_state) {
		kv_s(&w, "ev", "state");
		kv_s(&w, "from", s1_vc_state_name(seen->vc_state));
		kv_s(&w, "to", s1_vc_state_name(s->vc_state));
		if (s->vc_state == S1_VC_FAULT) { kv_s(&w, "why", s1_fault_why(s)); }
		seen->vc_state = s->vc_state;
	} else if (s->faults != seen->faults) {
		kv_s(&w, "ev", "fault");
		kv_u(&w, "bits", s->faults);
		seen->faults = s->faults;
	} else if (s->bar_count != seen->bar_count) {
		kv_s(&w, "ev", "barcode");
		kv_ch(&w, "ch", s->bar);
		kv_s(&w, "act", s1_act_name(s->bar_act));     /* what it will do at the next junction */
		seen->bar_count = s->bar_count;
	} else if (s->bar_err != seen->bar_err) {
		kv_s(&w, "ev", "barcode_error");
		kv_u(&w, "count", s->bar_err);
		kv_u(&w, "why", s->bar_why);
		kv_u(&w, "n", s->bar_fail_n);
		kv_s(&w, "u", s->bar_fail_mm10 ? "mm10" : "ms");
		key(&w, "w"); put_c(&w, '[');                  /* widths in ms: bar, gap, bar ... */
		{ unsigned i; for (i = 0; (i < s->bar_fail_n) && (i < 29u); i++) { if (i > 0u) { put_c(&w, ','); } put_u(&w, s->bar_fail_w[i]); } }
		put_c(&w, ']');
		seen->bar_err = s->bar_err;
	} else if (s->junc != seen->junc) {
		kv_s(&w, "ev", "junction");
		kv_ch(&w, "ch", s->j_ch);
		kv_s(&w, "act", s1_act_name(s->j_act));
		seen->junc = s->junc;
	} else if ((s->lost_ms >= 1000u) != (seen->lost_ms >= 1000u)) {
		kv_s(&w, "ev", (s->lost_ms >= 1000u) ? "line_lost" : "line_found");
		seen->lost_ms = s->lost_ms;
	} else if (s->humps != seen->humps) {
		kv_s(&w, "ev", "hump");
		kv_u(&w, "peak_mm", s->hump_last_mm);
		kv_u(&w, "max_mm", s->hump_max_mm);
		seen->humps = s->humps;
	} else if (s->scans != seen->scans) {
		kv_s(&w, "ev", "obstacle");
		kv_s(&w, "act", act_name(s->act));
		kv_u(&w, "closest_cm", (uint32_t)(s->obs_mm / 10u));
		kv_u(&w, "w_cm", (uint32_t)(s->obs_w_mm / 10u));
		seen->scans = s->scans;
	} else if (s->imu_state != seen->imu_state) {
		static const char *const n[] = { "calibrating", "ok", "fault", "lost" };
		kv_s(&w, "ev", "imu");
		kv_s(&w, "state", n[s->imu_state & 3u]);
		seen->imu_state = s->imu_state;
	} else {
		return 0u;
	}
	return end(&w);
}

/* ================================================================ plain-English status */
const char *s1_fault_why(const s1_snapshot_t *s)
{
	switch (s->vc_cause) {
	case 1: return "not ready after 20 s (motor self-test or IMU did not finish)";
	case 2: return "impact or tilt detected by the IMU (E-stop)";
	case 3: return "line lost for more than 3 s";
	case 4: return "a move or scan took too long (step timeout)";
	case 5: return "ultrasonic sensor not answering";
	case 6: return "could not find the line again after the obstacle";
	case 7:
		switch (s->m_cause) {
		case 1: return "motion stopped by the IMU E-stop (impact or tilt)";
		case 2: return "a wheel stalled: motor powered but encoder not moving (power bank on? wheel blocked?)";
		case 3: return "wheel encoder error (check encoder wires)";
		case 4: return "motor self-test failed: wheels did not turn at start-up (power bank on? motor wires?)";
		default: return "motion control fault";
		}
	default: return "unknown";
	}
}

uint16_t s1_status_text(char *buf, uint16_t size, const s1_snapshot_t *s)
{
	static const char *const navs[] = { "following the line", "junction move",
					    "going around an obstacle", "searching for the line" };
	w_t w;
	w.b = buf; w.n = 0; w.size = size; w.ok = (size > 1u);

	switch (s->vc_state) {
	case S1_VC_INIT:
		put_s(&w, "INIT: waiting for");
		if (s->m_state == 0u)     { put_s(&w, " [motor self-test]"); }
		if (s->imu_state == 0u)   { put_s(&w, " [IMU calibration - keep the robot still]"); }
		if (s->ir_cal == 0u)      { put_s(&w, " [IR calibration - slide the robot left/right over black AND white]"); }
		break;
	case S1_VC_READY:
		put_s(&w, "READY: centre the robot on the line, then press START (GP20) or send {\"cmd\":\"start\",\"seq\":N}");
		break;
	case S1_VC_RUNNING:
		put_s(&w, "RUNNING: "); put_s(&w, navs[s->vc_nav & 3u]);
		put_s(&w, ", wheels L "); put_i(&w, s->spd_l); put_s(&w, " R "); put_i(&w, s->spd_r);
		put_s(&w, " mm/s, line "); put_i(&w, s->line);
		if (s->line_lost) { put_s(&w, " (LOST)"); }
		if (s->lost_ms >= 500u) { put_s(&w, " | line lost "); put_u(&w, s->lost_ms / 100u / 10u); put_c(&w, '.'); put_u(&w, (s->lost_ms / 100u) % 10u); put_s(&w, " s (FAULT at 3 s)"); }
		if (s->bar_pend != 0) { put_s(&w, " | barcode "); put_ch(&w, s->bar_pend); put_s(&w, " stored -> "); put_s(&w, (s->bar_act != 0u) ? s1_act_name(s->bar_act) : "forward (not in the map)"); put_s(&w, " at the next junction"); }
		break;
	case S1_VC_PAUSED:
		put_s(&w, "PAUSED: press START to resume, STOP to cancel");
		break;
	case S1_VC_FAULT:
		put_s(&w, "FAULT: "); put_s(&w, s1_fault_why(s));
		put_s(&w, ". Press START (GP20) once to reset, then START again to run");
		break;
	case S1_VC_COMPLETE:
		put_s(&w, "COMPLETE: run finished. Press START for a new run");
		break;
	default:
		put_s(&w, "?");
		break;
	}
	if (s->degraded) { put_s(&w, " | IMU lost: slow mode, no hump detection"); }
	if ((s->faults & 0x08u) != 0u) { put_s(&w, " | ultrasonic not answering"); }
	if (!w.ok) { return 0u; }
	return w.n;
}

/* ================================================================ commands */
static bool is_space(char c) { return (c == ' ') || (c == '\t') || (c == '\r') || (c == '\n'); }

/* find "name" followed by optional spaces and ':'; returns index after ':' or -1 */
static int find_key(const char *p, uint16_t len, const char *name)
{
	uint16_t i, k;
	for (i = 0; i < len; i++) {
		if (p[i] != '"') { continue; }
		for (k = 0; (name[k] != '\0') && ((uint16_t)(i + 1u + k) < len) && (p[i + 1u + k] == name[k]); k++) { }
		if ((name[k] == '\0') && ((uint16_t)(i + 1u + k) < len) && (p[i + 1u + k] == '"')) {
			uint16_t j = (uint16_t)(i + 2u + k);
			while ((j < len) && is_space(p[j])) { j++; }
			if ((j < len) && (p[j] == ':')) {
				j++;
				while ((j < len) && is_space(p[j])) { j++; }
				return (int)j;
			}
		}
	}
	return -1;
}

static bool word_is(const char *p, uint16_t n, const char *w)
{
	uint16_t i;
	for (i = 0; i < n; i++) { if ((w[i] == '\0') || (p[i] != w[i])) { return false; } }
	return w[n] == '\0';
}

s1_rej_t s1_cmd_parse(const char *p, uint16_t len, s1_cmd_t *cmd, uint32_t *seq)
{
	int i, j;
	uint16_t n;
	uint32_t v = 0;
	bool digit = false;
	static const char *const names[] = { "start", "stop", "resume", "cancel", "reset", "servo", "map" };
	unsigned c;

	*cmd = S1_CMD_NONE;
	*seq = 0;
	if (len > S1_CMD_MAX_LEN) { return S1_REJ_TOO_LONG; }
	if ((len < 2u) || (p[0] != '{')) { return S1_REJ_BAD_JSON; }

	i = find_key(p, len, "cmd");
	if ((i < 0) || ((uint16_t)i >= len) || (p[i] != '"')) { return S1_REJ_BAD_JSON; }
	for (j = i + 1; (j < (int)len) && (p[j] != '"'); j++) { }
	if (j >= (int)len) { return S1_REJ_BAD_JSON; }
	n = (uint16_t)(j - (i + 1));

	i = find_key(p, len, "seq");
	if (i < 0) { return S1_REJ_BAD_JSON; }
	while ((i < (int)len) && (p[i] >= '0') && (p[i] <= '9')) {
		if (v > 400000000u) { return S1_REJ_BAD_JSON; }       /* keep it far from overflow */
		v = (v * 10u) + (uint32_t)(p[i] - '0');
		digit = true;
		i++;
	}
	if (!digit) { return S1_REJ_BAD_JSON; }

	{
		int k = find_key(p, len, "cmd") + 1;                  /* first character of the word */
		for (c = 0; c < 7u; c++) {
			if (word_is(&p[k], n, names[c])) {
				*cmd = (s1_cmd_t)(c + 1u);
				*seq = v;
				return S1_OK;
			}
		}
	}
	*seq = v;
	return S1_REJ_UNKNOWN;
}

bool s1_cmd_int(const char *p, uint16_t len, const char *key, int32_t *v)
{
	int i = find_key(p, len, key);
	bool neg = false, digit = false;
	int32_t x = 0;

	if (i < 0) { return false; }
	if ((i < (int)len) && (p[i] == '-')) { neg = true; i++; }
	while ((i < (int)len) && (p[i] >= '0') && (p[i] <= '9')) {
		if (x > 100000) { return false; }
		x = (x * 10) + (p[i] - '0');
		digit = true;
		i++;
	}
	if (!digit) { return false; }
	*v = neg ? -x : x;
	return true;
}

bool s1_cmd_map(const char *p, uint16_t len, char m[4])
{
	int i = find_key(p, len, "m"), k, j;
	char t[4];
	if ((i < 0) || ((i + 5) >= (int)len) || (p[i] != '"') || (p[i + 5] != '"')) { return false; }
	for (k = 0; k < 4; k++) {
		char c = p[i + 1 + k];
		if ((c >= 'a') && (c <= 'z')) { c = (char)(c - 'a' + 'A'); }
		if (!(((c >= 'A') && (c <= 'Z')) || ((c >= '0') && (c <= '9')) || (c == '-'))) { return false; }
		t[k] = c;
	}
	for (k = 0; k < 4; k++) {                      /* one character = one movement */
		for (j = k + 1; j < 4; j++) { if ((t[k] != '-') && (t[k] == t[j])) { return false; } }
	}
	for (k = 0; k < 4; k++) { m[k] = t[k]; }
	return true;
}

s1_rej_t s1_cmd_check(s1_cmd_t cmd, uint8_t st, bool still, uint8_t *remote)
{
	*remote = S1_REMOTE_NONE;
	switch (cmd) {
	case S1_CMD_START:
		if (st != S1_VC_READY) { return S1_REJ_STATE; }
		*remote = S1_REMOTE_START;
		break;
	case S1_CMD_RESUME:
		if (st != S1_VC_PAUSED) { return S1_REJ_STATE; }
		*remote = S1_REMOTE_START;                     /* VC: START in PAUSED = resume (history) */
		break;
	case S1_CMD_CANCEL:
		if (st != S1_VC_PAUSED) { return S1_REJ_STATE; }
		*remote = S1_REMOTE_STOP;                      /* VC: STOP in PAUSED = cancel -> READY */
		break;
	case S1_CMD_RESET:
		if ((st != S1_VC_FAULT) || !still) { return S1_REJ_STATE; }
		*remote = S1_REMOTE_START;                     /* VC: START in FAULT = reset */
		break;
	case S1_CMD_STOP:
		/* allowed at any time; only RUNNING has something to stop (in PAUSED a STOP
		   would mean "cancel", so it is accepted and NOT forwarded) */
		*remote = (st == S1_VC_RUNNING) ? S1_REMOTE_STOP : S1_REMOTE_NONE;
		break;
	case S1_CMD_SERVO:
		/* test tool only: the scan needs the servo while the robot drives */
		if (st == S1_VC_RUNNING) { return S1_REJ_STATE; }
		break;
	case S1_CMD_MAP:
		/* allowed in any state except while driving through a junction would be odd but harmless:
		   the new map is used from the next junction on */
		break;
	case S1_CMD_NONE:
	default:
		return S1_REJ_UNKNOWN;
	}
	return S1_OK;
}
