/*
 *	subsystem5.c
 *	Subsystem 5 -- Obstacle handling + system coordination (Buddy 5).
 *
 *	subsystem5_init():  create kernel objects, set up the ultrasonic hardware
 *	subsystem5_start(): start the Vehicle Controller and the Ultrasonic Scan task
 *
 *	Objects: flg_scan_req, sem_echo (here), mbf_vc_events (vehicle_ctrl.c)
 *	Tasks  : vc_task   priority 20  (system statechart, all mission decisions)
 *	         scan_task priority 21  (statechart 6.2, owns servo + TRIG/ECHO)
 *	APIs used: tk_cre_flg, tk_cre_sem, tk_cre_mbf, app_start_task
 */
#include "subsystem5.h"
#include "drv_ultrasonic.h"

EXPORT ID flg_scan_req;
EXPORT ID sem_echo;
LOCAL  ID tid_vc, tid_scan;

EXPORT ER subsystem5_init(void)
{
	T_CFLG cflg = {0};
	T_CSEM csem = {0};
	T_CMBF cmbf = {0};

	cflg.flgatr  = TA_TFIFO | TA_WMUL;
	cflg.iflgptn = 0;
	flg_scan_req = tk_cre_flg(&cflg);
	if (flg_scan_req < E_OK) return flg_scan_req;

	csem.sematr  = TA_TFIFO | TA_FIRST;
	csem.isemcnt = 0;
	csem.maxsem  = 1;
	sem_echo = tk_cre_sem(&csem);
	if (sem_echo < E_OK) return sem_echo;

	cmbf.mbfatr = TA_TFIFO;
	cmbf.bufsz  = 24 * (VC_EV_MAX_BYTES + 4);		/* ~24 events of headroom (each message has a 4-byte header) */
	cmbf.maxmsz = VC_EV_MAX_BYTES;
	mbf_vc_events = tk_cre_mbf(&cmbf);
	if (mbf_vc_events < E_OK) return mbf_vc_events;

	us_hw_init();		/* pins, servo PWM, echo branch on S2's GPIO IRQ */
	return E_OK;
}

EXPORT ER subsystem5_start(void)
{
	/* VC first, so events posted by the scan task are never dropped. prc = 0: any core
	   (core 1 runs motion_task + the GPIO IRQ, core 0 runs USB and WiFi polling). */
	tid_vc = app_start_task((FP)s5_vc_task, S5_PRI_VC, 0, "vc_task");
	if (tid_vc < E_OK) return tid_vc;
	tid_scan = app_start_task((FP)s5_scan_task, S5_PRI_SCAN, 0, "scan_task");
	return (tid_scan < E_OK) ? tid_scan : E_OK;
}
