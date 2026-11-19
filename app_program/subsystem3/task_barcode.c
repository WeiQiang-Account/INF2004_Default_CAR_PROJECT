/*
 *	task_barcode.c
 *	Subsystem 3 -- the Barcode task (priority S3_PRI_BARCODE, event driven).
 *
 *	Sleeps on flg_bar_edge. The GP7 edge branch (drv_ir.c) saves every edge
 *	(time, encoder, level) in a ring buffer; this task takes the edges out and
 *	runs the barcode state machine (s3_barcode.c). A decoded command goes to the
 *	Vehicle Controller; a failed read is reported and the robot keeps following the line.
 *
 *	APIs used: tk_wai_flg, tm_printf, vc_post (S5)
 */

#include "subsystem3.h"
#include "drv_ir.h"
#include "../subsystem2/drv_encoder.h"		/* encoder_read(): distance quiet zone */
#include "../subsystem5/vc_events.h"		/* vc_post(), VC_EV_* */

LOCAL s3_bc_t		bc;
LOCAL volatile UB	busy_flag;
LOCAL volatile H	last_cmd = -1;
LOCAL volatile UW	err_count;
LOCAL volatile UW	ok_count;
LOCAL volatile UB	last_why;	/* reason of the last failed read (s3_logic.h) */
LOCAL volatile UB	prog_n;		/* bars/gaps collected so far in the read in progress (0 = not reading) */

BOOL s3_barcode_busy(void)       { return busy_flag != 0; }
INT  s3_last_barcode_cmd(void)   { return last_cmd; }	/* the CHARACTER read, -1 none */
INT  s3_barcode_fail_reason(void) { return last_why; }
INT  s3_barcode_progress(void)    { return prog_n; }
/* widths of the last FAILED read, for the dashboard: in 0.1 mm (distance mode, *mm10 = 1)
   or ms (time mode, *mm10 = 0), max 255. Returns how many (0..29). */
INT  s3_barcode_fail_widths(UB *out, INT max, UB *mm10)
{
	INT i, n = bc.fail_n;
	if (n > max) n = max;
	*mm10 = bc.fail_enc ? 1u : 0u;
	for (i = 0; i < n; i++) {
		UW v = bc.fail_enc ? (bc.fail_widths[i] * 10u * 1000u / 12077u)	/* counts -> 0.1 mm (12.08 counts/mm) */
				   : ((bc.fail_widths[i] + 500u) / 1000u);		/* us -> ms */
		out[i] = (UB)((v > 255u) ? 255u : v);
	}
	return n;
}
UW   s3_barcode_errors(void)     { return err_count; }
UW   s3_barcode_count(void)      { return ok_count; }
UW   s3_barcode_partial(void)    { return bc.partial_count; }	/* reads that used '*' + letter only */


LOCAL void report(const s3_bc_ev_t *ev)
{
	if (ev->type == S3_BC_EV_CMD) {
		last_cmd = (H)ev->value;
		ok_count++;		/* telemetry: a new barcode event (S1) */
		/* arg = valid<<7 | character (vc_events.h). Valid is 1: the checks passed */
		(void)vc_post(VC_EV_MAKE(VC_EV_BARCODE, (1u << 7) | ((uint32_t)ev->value & 0x7Fu)));
#if S3_VERBOSE
		S3_PRINT("[S3] barcode '%c' read\n", (INT)ev->value);
#endif
	} else if (ev->type == S3_BC_EV_ERROR) {
		err_count++;
		last_why = (UB)ev->value;		/* counted for telemetry (s3_barcode_errors). NOT posted to the Vehicle
					   Controller: it ignores a bad read, and noise must not fill its queue. */
#if S3_VERBOSE
		S3_PRINT("[S3] barcode read failed, reason %d\n", (INT)ev->value);
#endif
	}
}

EXPORT void barcode_task(INT stacd, void *exinf)
{
	UINT		ptn;
	s3_edge_t	e;
	s3_bc_ev_t	ev;

	(void)stacd; (void)exinf;
	s3_bc_init(&bc);
	bc.use_enc = (S3_BC_WIDTH_FROM_ENC != 0);

	while (1) {
		/* Sleep until the IRQ reports an edge, OR a few ms pass: the glitch window and the
		   timeouts need the clock because no edge arrives in a quiet zone.
		   TWF_BITCLR clears the bit when we wake. E_TMOUT is normal here. */
		(void)tk_wai_flg(flg_bar_edge, FLG_S3_EDGE, TWF_ORW | TWF_BITCLR, &ptn, S3_BC_POLL_MS);

		/* 1. every edge the IRQ saved */
#if S3_BC_FROM_ADC
		while (ir_edge_pop(&e)) { }		/* GP7 edges: drained and ignored (still counted by the IRQ) */
		while (s3_adc_edge_pop(&e)) {
#else
		while (ir_edge_pop(&e)) {
#endif
#if S3_BC_FROM_ADC
			if (e.level == S3_BC_LEVEL_ABORT) { s3_bc_abort(&bc); continue; }	/* track line under the sensor */
#endif
			s3_bc_edge(&bc, e.t_us, e.enc, e.level, &ev);
			report(&ev);
		}
		/* 2. glitch window, quiet zone, timeouts */
		{ int32_t l, r; encoder_read(&l, &r); bc.now_enc = (l + r) / 2; bc.have_now_enc = true; }
		s3_bc_poll(&bc, ir_time_us(), &ev);
		report(&ev);

		/* 2026-11-18: "busy" (line task: ignore the barcode sensor, no junction) only once this
		   really looks like a barcode (bar, gap, bar...). The left sensor also brushes the
		   track line on bends and crosses it at every junction: one or two marks must not hide
		   a junction for the next 60 mm. */
		busy_flag = ((bc.state == S3_BC_READING) && (bc.n_elem >= S3_BC_BUSY_ELEM)) ? 1 : 0;
		prog_n    = (bc.state == S3_BC_READING) ? (UB)bc.n_elem : 0u;
	}
}
