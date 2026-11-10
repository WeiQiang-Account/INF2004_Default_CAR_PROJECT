/*
 *	subsystem1.c
 *	Subsystem 1 -- kernel side: remote-command flag, robot snapshot, command
 *	checks and the VC remote hook. No task of its own: publishing runs on the
 *	template's Wi-Fi task (s1_mqtt.c), the hook runs on the VC task.
 *
 *	APIs used: tk_cre_flg, tk_set_flg, tk_clr_flg, tk_wai_flg (TMO_POL), tk_ref_flg, tk_get_tim
 */
#include "subsystem1.h"
#include "s1_bridge.h"
#include "../subsystem2/subsystem2.h"		/* flg_fault */
#include "../subsystem2/motion.h"		/* motion_get_status() */
#include "../subsystem2/motion_types.h"		/* FAULT_BIT_WIFI */
#include "../subsystem2/drv_encoder.h"		/* encoder_read() */
#include "../subsystem3/subsystem3.h"		/* s3_* getters */
#include "../subsystem4/subsystem4.h"		/* s4_* getters */
#include "../subsystem5/subsystem5.h"		/* g_s5_scan */
#include "../subsystem5/vc_events.h"		/* vc_status_byte(), s5_last_distance_mm() */

EXPORT ID	flg_remote_cmd;

LOCAL UW	last_seq;		/* highest accepted seq (replay / duplicate guard) */
LOCAL BOOL	have_seq;
LOCAL UW	n_accepted, n_rejected;

EXPORT ER subsystem1_init(void)
{
	T_CFLG	cflg = {0};

	cflg.flgatr  = TA_TFIFO | TA_WMUL;
	cflg.iflgptn = 0;
	flg_remote_cmd = tk_cre_flg(&cflg);
	return (flg_remote_cmd < E_OK) ? flg_remote_cmd : E_OK;
}

EXPORT ER subsystem1_start(void)
{
#if defined(TM_MQTT) && (TM_MQTT == 1)
	tm_printf((UB*)"[S1] telemetry on se14/robot/telemetry, commands on se14/robot/cmd\n");
#else
	tm_printf((UB*)"[S1] no MQTT in this build (make NET=lwip MQTT=1): remote commands off\n");
#endif
	return E_OK;
}

/* ------------------------------------------------------------ bridge (plain types) */
uint32_t s1_now_ms(void)
{
	SYSTIM t;
	(void)tk_get_tim(&t);
	return (uint32_t)t.lo;
}

void s1_log(const char *msg)
{
	tm_printf((UB*)"[S1] %s\n", msg);
}

void s1_wifi_fault(bool lost)
{
	if (flg_fault <= 0) { return; }
	if (lost) { (void)tk_set_flg(flg_fault, FAULT_BIT_WIFI); }
	else      { (void)tk_clr_flg(flg_fault, ~(UINT)FAULT_BIT_WIFI); }
}

LOCAL BOOL robot_still(void)
{
	motion_status_t m;
	motion_get_status(&m);
	return (m.speed_l_mmps > -S1_STILL_MMPS) && (m.speed_l_mmps < S1_STILL_MMPS) &&
	       (m.speed_r_mmps > -S1_STILL_MMPS) && (m.speed_r_mmps < S1_STILL_MMPS);
}

void s1_snapshot_take(s1_snapshot_t *s)
{
	motion_status_t	m;
	T_RFLG		rf;
	uint8_t		vc = vc_status_byte();
	INT		bar;

	s->t_ms     = s1_now_ms();
	s->vc_state = (uint8_t)(vc & 7u);
	s->vc_nav   = (uint8_t)((vc >> 3) & 3u);
	s->on_hump  = (uint8_t)((vc >> 5) & 1u);
	s->degraded = (uint8_t)((vc >> 6) & 1u);

	motion_get_status(&m);
	s->m_state = (uint8_t)m.state;
	s->m_cause = (uint8_t)m.cause;
	s->spd_l   = m.speed_l_mmps;
	s->spd_r   = m.speed_r_mmps;
	s->dist_mm = m.distance_mm;
	encoder_read(&s->enc_l, &s->enc_r);
	s->enc_err_l = encoder_error_total(ENC_LEFT);
	s->enc_err_r = encoder_error_total(ENC_RIGHT);
	if (s->enc_err_l > 99999u) { s->enc_err_l = 99999u; }
	if (s->enc_err_r > 99999u) { s->enc_err_r = 99999u; }

	s->line      = (int16_t)s3_line_position();
	s->line_lost = s3_line_lost() ? 1u : 0u;
	s->ir[0] = s3_ir_raw(0); s->ir[1] = s3_ir_raw(1); s->ir[2] = s3_ir_raw(2);
	bar          = s3_last_barcode_cmd();
	s->bar       = (int8_t)(((bar >= 0) && (bar <= 3)) ? bar : -1);
	s->bar_count = s3_barcode_count();
	s->bar_err   = s3_barcode_errors();

	s->hump_mm      = (uint16_t)s4_hump_mm();
	s->hump_last_mm = (uint16_t)s4_hump_last_mm();
	s->hump_max_mm  = (uint16_t)s4_hump_max_mm();
	s->humps        = (uint16_t)s4_hump_count();
	s->pitch_x10    = (int16_t)s4_pitch_x10();
	s->es_why       = (uint8_t)s4_estop_why();
	s->es_cg        = (uint16_t)s4_estop_cg();
	s->es_tilt      = (uint16_t)s4_estop_tilt();
	s->a_pk_cg      = (uint16_t)s4_peak_accel_cg();
	s->hdg          = (int16_t)s4_heading_deg();
	s->yaw_dps      = (int16_t)s4_turn_rate_dps();
	s->mclass       = (uint8_t)s4_motion_class();
	s->imu_state    = (uint8_t)s4_imu_state();
	s->wdt_reset    = s4_watchdog_reset_seen() ? 1u : 0u;

	s->obs_mm   = s5_last_distance_mm();		/* 0xFFFF = no echo */
	s->obs_w_mm = (uint16_t)g_s5_scan.width_mm;
	s->clr_l_mm = (uint16_t)g_s5_scan.clr_l_mm;
	s->clr_r_mm = (uint16_t)g_s5_scan.clr_r_mm;
	s->act      = (uint8_t)g_s5_scan.action;
	s->scans    = g_s5_scan.count;

	s->faults   = ((flg_fault > 0) && (tk_ref_flg(flg_fault, &rf) == E_OK)) ? (uint32_t)rf.flgptn : 0u;
	s->vc_drops = vc_dropped_events();
	s->vc_cause = vc_fault_cause();
	s->ir_cal   = s3_line_ready() ? 1u : 0u;
	{
		uint32_t r, f, e, lv;
		s5_echo_diag(&r, &f, &e, &lv);
		s->us_rise  = (r == 0xFFFFFFFFu) ? -1 : (int32_t)r;
		s->us_fall  = (f == 0xFFFFFFFFu) ? -1 : (int32_t)f;
		s->us_edges = e;
		s->us_level = (uint8_t)lv;
		s5_ping_diag(&r, &f, &e);
		s->us_w     = (uint16_t)((r > 65535u) ? 65535u : r);
		s->us_st    = (uint8_t)f;
		s->us_mm    = (uint16_t)e;
		s->s5_state = (uint8_t)(s5_status_byte() >> 5);
		s->servo_deg    = (int16_t)s5_servo_angle();
		s->servo_manual = s5_servo_is_manual() ? 1u : 0u;
	}
}

/* Runs on the Wi-Fi task. Only the event flag crosses to the VC. */
uint16_t s1_cmd_submit(const char *p, uint16_t len, char *reply, uint16_t size)
{
	s1_cmd_t	cmd;
	uint32_t	seq;
	uint8_t		remote = S1_REMOTE_NONE;
	uint8_t		st = (uint8_t)(vc_status_byte() & 7u);
	s1_rej_t	r;

	r = s1_cmd_parse(p, len, &cmd, &seq);
	if ((r == S1_OK) && have_seq && (seq <= last_seq)) {
		r = S1_REJ_SEQ;
	}
	if (r == S1_OK) {
		r = s1_cmd_check(cmd, st, robot_still(), &remote);
	}
	if ((r == S1_OK) && (cmd == S1_CMD_SERVO)) {
		int32_t deg;
		if (!s1_cmd_int(p, len, "deg", &deg) || (deg < -1) || (deg > 180)) {
			r = S1_REJ_BAD_JSON;
		} else {
			s5_servo_manual((int)deg);		/* -1 = automatic */
		}
	}
	if (r == S1_OK) {
		last_seq = seq;
		have_seq = TRUE;
		n_accepted++;
		if (remote == S1_REMOTE_START) { (void)tk_set_flg(flg_remote_cmd, S1_FLG_START(st)); }
		if (remote == S1_REMOTE_STOP)  { (void)tk_set_flg(flg_remote_cmd, S1_FLG_STOP(st)); }
	} else {
		n_rejected++;
	}
	tm_printf((UB*)"[S1] cmd %s seq %u in %s: %s\n", (UB*)s1_cmd_name(cmd), (UINT)seq,
		  (UB*)s1_vc_state_name(st), (UB*)((r == S1_OK) ? "accepted" : s1_rej_name(r)));
	return s1_json_cmd_result(reply, size, s1_now_ms(), s1_cmd_name(cmd), seq, r == S1_OK, s1_rej_name(r));
}

/* ------------------------------------------------------------ VC hook (IF-15)
 * Overrides S5's weak default. Called by the VC every loop. A command is used
 * only if the VC is still in the state it was checked in (else it is dropped):
 * e.g. a STOP checked in RUNNING must not become a "cancel" after an auto-pause. */
int s5_hook_remote_poll(void)
{
	UINT	ptn = 0;
	UINT	st;

	if (tk_wai_flg(flg_remote_cmd, S1_FLG_ALL, TWF_ORW | TWF_BITCLR, &ptn, TMO_POL) != E_OK) {
		return 0;
	}
	st = (UINT)(vc_status_byte() & 7u);
	if ((ptn & S1_FLG_STOP(st)) != 0u)  { return 2; }	/* stop wins */
	if ((ptn & S1_FLG_START(st)) != 0u) { return 1; }
	tm_printf((UB*)"[S1] remote command dropped: VC state changed (now %s)\n", (UB*)s1_vc_state_name((uint8_t)st));
	return 0;
}

/* for subsystem1_test.c */
EXPORT UW s1_accepted_count(void) { return n_accepted; }
EXPORT UW s1_rejected_count(void) { return n_rejected; }
EXPORT void s1_reset_seq_for_test(void) { have_seq = FALSE; last_seq = 0; }
