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
#include "../subsystem5/vc_events.h"		/* vc_post(), VC_EV_* */

LOCAL s3_bc_t		bc;
LOCAL volatile UB	busy_flag;
LOCAL volatile H	last_cmd = -1;
LOCAL volatile UW	err_count;
LOCAL volatile UW	ok_count;

BOOL s3_barcode_busy(void)       { return busy_flag != 0; }
INT  s3_last_barcode_cmd(void)   { return last_cmd; }
UW   s3_barcode_errors(void)     { return err_count; }
UW   s3_barcode_count(void)      { return ok_count; }

LOCAL const char *const name[4] = { "A", "B", "C", "D" };

LOCAL void report(const s3_bc_ev_t *ev)
{
	if (ev->type == S3_BC_EV_CMD) {
		last_cmd = (H)ev->value;
		ok_count++;		/* telemetry: a new barcode event (S1) */
		/* arg = valid<<2 | cmd (vc_events.h). Valid is 1: the checks passed */
		(void)vc_post(VC_EV_MAKE(VC_EV_BARCODE, (1u << 2) | ((uint32_t)ev->value & 3u)));
#if S3_VERBOSE
		S3_PRINT("[S3] barcode %s -> command %d (0 left, 1 right, 2 straight, 3 U-turn)\n",
			 name[ev->value & 3], (INT)ev->value);
#endif
	} else if (ev->type == S3_BC_EV_ERROR) {
		err_count++;		/* counted for telemetry (s3_barcode_errors). NOT posted to the Vehicle
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

	while (1) {
		/* Sleep until the IRQ reports an edge, OR a few ms pass: the glitch window and the
		   timeouts need the clock because no edge arrives in a quiet zone.
		   TWF_BITCLR clears the bit when we wake. E_TMOUT is normal here. */
		(void)tk_wai_flg(flg_bar_edge, FLG_S3_EDGE, TWF_ORW | TWF_BITCLR, &ptn, S3_BC_POLL_MS);

		/* 1. every edge the IRQ saved */
		while (ir_edge_pop(&e)) {
			s3_bc_edge(&bc, e.t_us, e.enc, e.level, &ev);
			report(&ev);
		}
		/* 2. glitch window, quiet zone, timeouts */
		s3_bc_poll(&bc, ir_time_us(), &ev);
		report(&ev);

		busy_flag = s3_bc_busy(&bc) ? 1 : 0;
	}
}
