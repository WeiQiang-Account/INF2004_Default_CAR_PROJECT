/*
 *	subsystem3.h
 *	Subsystem 3 -- Buddy 3: line sensing + barcode.  Student ID: 2500142.
 *
 *	Public interface of subsystem 3.  Other subsystems include it as
 *	"../subsystem3/subsystem3.h".
 *
 *	What Buddy 3 provides (TEAM.md section 5):
 *	  IF-09  S3 -> S2  steering        steer_publish(corr) every 5 ms (motion_types.h, owned by S2)
 *	  IF-11  S3, S5 -> S2  GP7 edge branch via gpio_irq_register()  (drv_gpio_irq.h)
 *	  S3 -> VC         events          vc_post(VC_EV_MAKE(type, arg))  (vc_events.h, queue owned by S5)
 *	                     VC_EV_BARCODE  arg = valid<<2 | cmd  (0 A=left 1 B=right 2 C=straight 3 D=U-turn)
 *	                     VC_EV_JUNCTION arg = line position + 100 (0..200)
 *	                     VC_EV_LINE_LOST / VC_EV_LINE_FOUND
 *	  S3 -> VC         hook            int s5_hook_line_detected(void)  (overrides S5's weak default)
 *	  S3 -> S1         read-only       s3_ir_raw(), s3_last_barcode_cmd(), ... (telemetry)
 *
 *	Subsystem 3 owns: the ADC (GP26-GP28, Line Follow task only) and GP7.
 *	It NEVER drives the motors.
 */

#ifndef SUBSYSTEM3_H
#define SUBSYSTEM3_H

#include "../app.h"
#include "s3_config.h"
#include "s3_logic.h"

/* Event flag: the GP7 edge branch (IRQ) sets it, the Barcode task waits on it */
IMPORT ID	flg_bar_edge;
#define FLG_S3_EDGE	0x00000001u

#define S3_PRINT(fmt, ...)	tm_printf((UB*)(fmt), ##__VA_ARGS__)

/* Tasks (private to subsystem 3) */
IMPORT void	line_follow_task(INT stacd, void *exinf);
IMPORT void	barcode_task(INT stacd, void *exinf);

/* ---- Read-only values for telemetry (Buddy 1) and tests. Safe from any task. ---- */
UH	s3_ir_raw(INT sensor);		/* latest raw ADC, 0..4095. 0 = left, 1 = centre, 2 = right */
INT	s3_line_position(void);		/* -100 (line far left) .. +100 (far right)  */
BOOL	s3_line_ready(void);		/* calibration finished and OK               */
BOOL	s3_line_lost(void);
INT	s3_last_barcode_cmd(void);	/* -1 = none yet, else 0..3 (s3_cmd_t)       */
UW	s3_barcode_errors(void);	/* failed barcode reads                      */
UW	s3_barcode_count(void);		/* good barcode reads (increments per read)  */
BOOL	s3_barcode_busy(void);		/* a barcode is being read                   */

/* ---- Template example (consumer). See S3_KEEP_TEMPLATE_EXAMPLE in s3_config.h ---- */
#if S3_KEEP_TEMPLATE_EXAMPLE
#define S3_PRI_CONSUMER		17	/* priority band 16-17 (TEAM.md) */
#define S3_BATCH_SIZE		5	/* report to subsystem 4 every N samples */

/* IF-01: send SAMPLE_MSG (subsystem2.h) here with tk_snd_mbx */
IMPORT ID	mbx_samples;

/* IF-04: statistics.  Read them ONLY while holding mtx_stats:
 *	tk_loc_mtx(mtx_stats, TMO_FEVR);  copy = app_stats;  tk_unl_mtx(mtx_stats);
 */
typedef struct {
	UW	received;	/* samples received so far */
	INT	min;
	INT	max;
	W	sum;
} APP_STATS;

IMPORT ID		mtx_stats;
IMPORT APP_STATS	app_stats;

IMPORT void	consumer_task(INT stacd, void *exinf);
#endif

#endif /* SUBSYSTEM3_H */
