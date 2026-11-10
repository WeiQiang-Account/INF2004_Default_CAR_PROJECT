/*
 *	task_line_follow.c
 *	Subsystem 3 -- the Line Follow task (priority S3_PRI_LINE, every 5 ms).
 *
 *	1. Calibrate: sweep the 3 IR sensors over black AND white (S3_CAL_MS).
 *	2. Every 5 ms: read the ADC, find the line (s3_line.c), publish the steering
 *	   to Motion Control (steer_publish, IF-09) and post junction / line-lost /
 *	   line-found events to the Vehicle Controller (vc_post).
 *	It is the ONLY task that touches the ADC (TEAM.md section 3).
 *
 *	APIs used: tk_dly_tsk, tk_get_tim, tm_printf, steer_publish (S2), vc_post (S5)
 */

#include "subsystem3.h"
#include "drv_ir.h"
#include "../subsystem2/motion_types.h"		/* steer_publish() */
#include "../subsystem5/vc_events.h"		/* vc_post(), VC_EV_* */

LOCAL s3_line_t		line;
LOCAL volatile UH	raw_now[3];
LOCAL volatile H	pos_now;
LOCAL volatile UB	ready_flag, any_black_flag, lost_flag;

UH   s3_ir_raw(INT s)      { return raw_now[(s < 0 || s > 2) ? 0 : s]; }
INT  s3_line_position(void) { return pos_now; }
BOOL s3_line_ready(void)    { return ready_flag != 0; }
BOOL s3_line_lost(void)     { return lost_flag != 0; }

/* Buddy 5's Vehicle Controller asks this while it searches for the original line after a bypass.
   Strong definition: overrides the weak default (returns 0) in subsystem5/s5_port.c. */
int s5_hook_line_detected(void)
{
	return (ready_flag != 0) && (any_black_flag != 0);
}

/* VC: is the line lost right now? Polled every VC tick for the 3 s line-lost
   timeout (2026-11-10: the one-off LINE_LOST event plus a cancel on any dark
   blip let the timeout switch itself off for good on a floor with joints). */
int s5_hook_line_lost(void)
{
	H p = pos_now;
	if (ready_flag == 0) return -1;
	if (lost_flag != 0) return 1;
	/* 2026-11-10: off the end of the line the robot circled for ever. Only the
	   weak far-left sensor half-saw the floor (position pinned at -100), which
	   counted as "found" and kept cancelling the 3 s timer. A line seen ONLY at
	   the very edge is not a line we are following: count it as lost too.
	   (A real sharp corner sits at the edge for well under 3 s.) */
	if (p <= -S3_EDGE_POS || p >= S3_EDGE_POS) return 1;
	return 0;
}

/* The VC waits for this before READY (no steering until the IR sensors are calibrated). */
int s5_hook_line_ready(void)
{
	return ready_flag != 0;
}

LOCAL uint32_t now_ms(void) { SYSTIM t; (void)tk_get_tim(&t); return (uint32_t)t.lo; }

/* Calibration: min and max of every sensor while the robot is moved over black and white (A5) */
LOCAL void calibrate(void)
{
	uint16_t	raw[3];
	UW		n, loops = S3_CAL_MS / S3_LINE_PERIOD_MS;
	INT		i;

	while (1) {
		S3_PRINT("[S3] calibrating %d s: sweep ALL THREE IR sensors over BLACK and WHITE now\n",
			 (INT)(S3_CAL_MS / 1000));
		s3_line_cal_begin(&line);
		for (n = 0; n < loops; n++) {
			if (ir_adc_read3(raw)) {
				s3_line_cal_feed(&line, raw);
				for (i = 0; i < 3; i++) raw_now[i] = raw[i];
			}
			(void)tk_dly_tsk(S3_LINE_PERIOD_MS);
		}
		if (s3_line_cal_end(&line)) {
			S3_PRINT("[S3] calibration OK  L %u..%u  C %u..%u  R %u..%u\n",
				 (UW)line.raw_min[0], (UW)line.raw_max[0], (UW)line.raw_min[1],
				 (UW)line.raw_max[1], (UW)line.raw_min[2], (UW)line.raw_max[2]);
			return;
		}
		S3_PRINT("[S3] calibration FAILED (needs a swing of %d on every sensor): "
			 "L %u..%u  C %u..%u  R %u..%u. Check wiring/height, retrying\n", (INT)S3_MIN_CAL_RANGE,
			 (UW)line.raw_min[0], (UW)line.raw_max[0], (UW)line.raw_min[1],
			 (UW)line.raw_max[1], (UW)line.raw_min[2], (UW)line.raw_max[2]);
	}
}

EXPORT void line_follow_task(INT stacd, void *exinf)
{
	uint16_t	raw[3];
	s3_line_out_t	out;
	uint32_t	next;
	INT		d;
#if S3_STATUS_MS > 0
	UW		tick = 0;
#endif
#if S3_TIMING_PIN >= 0
	UINT		lvl = 0;
#endif

	(void)stacd; (void)exinf;

#if S3_TIMING_PIN >= 0
	(void)gpio_set_pin(S3_TIMING_PIN, GPIO_MODE_OUT);
#endif
	calibrate();
	ready_flag = 1;
	next = now_ms();

	while (1) {
#if S3_TIMING_PIN >= 0
		lvl ^= 1u;
		(void)gpio_set_val(S3_TIMING_PIN, lvl);		/* scope/logic analyser: period should be 2 x 5 ms */
#endif
		if (ir_adc_read3(raw)) {
			INT i;
			for (i = 0; i < 3; i++) raw_now[i] = raw[i];
			s3_line_step(&line, raw, s3_barcode_busy(), &out);
			pos_now        = out.pos;
			lost_flag      = out.lost ? 1 : 0;
			any_black_flag = out.any_black ? 1 : 0;

			steer_publish((int8_t)out.steer);		/* IF-09: one 16-bit store, seq + correction */

			if ((out.events & S3_LINE_EVT_JUNCTION) != 0u) {
				(void)vc_post(VC_EV_MAKE(VC_EV_JUNCTION, (uint32_t)(out.pos + 100)));
#if S3_VERBOSE
				S3_PRINT("[S3] junction, line pos %d\n", (INT)out.pos);
#endif
			}
			if ((out.events & S3_LINE_EVT_LOST) != 0u) {
				(void)vc_post(VC_EV_MAKE(VC_EV_LINE_LOST, 0));
#if S3_VERBOSE
				S3_PRINT("[S3] line LOST\n");
#endif
			}
			if ((out.events & S3_LINE_EVT_FOUND) != 0u) {
				(void)vc_post(VC_EV_MAKE(VC_EV_LINE_FOUND, 0));
#if S3_VERBOSE
				S3_PRINT("[S3] line found\n");
#endif
			}
#if S3_STATUS_MS > 0
			if ((++tick * S3_LINE_PERIOD_MS) >= S3_STATUS_MS) {
				tick = 0;
				S3_PRINT("[S3] raw %u %u %u  norm %d %d %d  pos %d steer %d lost %d\n",
					 (UW)raw[0], (UW)raw[1], (UW)raw[2], (INT)out.norm[0], (INT)out.norm[1],
					 (INT)out.norm[2], (INT)out.pos, (INT)out.steer, (INT)out.lost);
			}
#endif
		}
		/* ADC timed out: publish nothing. Motion Control sees the stale sequence number after
		   60 ms and drives straight (safe), instead of using a wrong value. */

		/* drift-free 5 ms period: wait until the NEXT slot, not "5 ms from now" */
		next += S3_LINE_PERIOD_MS;
		d = (INT)(int32_t)(next - now_ms());
		if (d > 0) {
			(void)tk_dly_tsk((RELTIM)d);
		} else {
			next = now_ms();			/* we are late: resynchronise instead of racing to catch up */
			(void)tk_dly_tsk(1);			/* always let lower-priority tasks run */
		}
	}
}
