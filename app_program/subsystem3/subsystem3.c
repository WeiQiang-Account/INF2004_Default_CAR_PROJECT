/*
 *	subsystem3.c
 *	Subsystem 3 -- Buddy 3: creates its objects and starts its tasks.
 *
 *	subsystem3_init():  event flag, IR hardware (ADC pins, GP7 + its IRQ branch)
 *	subsystem3_start(): Line Follow task and Barcode task
 *	Tasks : line_follow_task  priority 16  every 5 ms
 *	        barcode_task      priority 17  event driven (flg_bar_edge)
 *	APIs used: tk_cre_flg, tk_cre_mbx, tk_cre_mtx (template example), app_start_task
 */

#include "subsystem3.h"
#include "drv_ir.h"

EXPORT ID		flg_bar_edge;		/* IRQ -> Barcode task */

#if S3_KEEP_TEMPLATE_EXAMPLE
EXPORT ID		mbx_samples;		/* IF-01 */
EXPORT ID		mtx_stats;		/* IF-04 */
EXPORT APP_STATS	app_stats;
#endif

EXPORT ER subsystem3_init(void)
{
	T_CFLG	cflg = {0};

	/* Event flag: the GP7 edge branch sets FLG_S3_EDGE, the Barcode task waits on it. */
	cflg.flgatr  = TA_TFIFO | TA_WMUL;
	cflg.iflgptn = 0;
	flg_bar_edge = tk_cre_flg(&cflg);
	if (flg_bar_edge < E_OK) {
		return flg_bar_edge;
	}

	/* ADC out of reset, GP26-GP28 analog, GP7 input, edge branch registered with S2's
	   shared GPIO IRQ (the IRQ itself is enabled by motion_task on core 1). */
	ir_hw_init();

#if S3_KEEP_TEMPLATE_EXAMPLE
	{
		T_CMBX	cmbx = {0};
		T_CMTX	cmtx = {0};

		/* template example, unchanged: mailbox (zero copy) + mutex (priority inheritance) */
		cmbx.mbxatr = TA_TFIFO | TA_MFIFO;
		mbx_samples = tk_cre_mbx(&cmbx);
		if (mbx_samples < E_OK) {
			return mbx_samples;
		}
		cmtx.mtxatr = TA_INHERIT;
		mtx_stats = tk_cre_mtx(&cmtx);
		if (mtx_stats < E_OK) {
			return mtx_stats;
		}
	}
#endif
	return E_OK;
}

EXPORT ER subsystem3_start(void)
{
	ID	tid;

	/* prc = 0: any core (same as S5). Core 1 runs motion_task + the GPIO IRQ, core 0 runs
	   USB + WiFi polling. Measure the 5 ms jitter (A2, S3_TIMING_PIN) before fixing a core. */
	tid = app_start_task((FP)line_follow_task, S3_PRI_LINE, 0, "line_follow_task");
	if (tid < E_OK) {
		return tid;
	}
	tid = app_start_task((FP)barcode_task, S3_PRI_BARCODE, 0, "barcode_task");
	if (tid < E_OK) {
		return tid;
	}
#if S3_KEEP_TEMPLATE_EXAMPLE
	/* Pinned to core 0 (TP_PRC1): opposite core to the producer */
	tid = app_start_task(consumer_task, S3_PRI_CONSUMER, TP_PRC1, "consumer_task");
	if (tid < E_OK) {
		return tid;
	}
#endif
	return E_OK;
}
