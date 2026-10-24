/*
 *	subsystem2.c
 *	Subsystem 2 -- Motion Control: creates its kernel objects and starts
 *	the motion task.
 *
 *	APIs used: tk_cre_flg, tk_cre_mbf, app_start_task
 */

#include "subsystem2.h"
#include "motion_config.h"

EXPORT ID	mbf_motion_cmd;
EXPORT ID	flg_move_done;
EXPORT ID	flg_estop;
EXPORT ID	flg_fault;
EXPORT ID	tid_motion;

LOCAL ID s2_cre_flg(void)
{
	T_CFLG cflg = {0};

	cflg.flgatr  = TA_TFIFO | TA_WMUL;	/* several tasks may wait */
	cflg.iflgptn = 0;
	return tk_cre_flg(&cflg);
}

EXPORT ER subsystem2_init(void)
{
	T_CMBF		cmbf = {0};
	motion_links_t	links;

	/* Message buffer: commands from the Vehicle Controller.  With the
	   minimum-bit change each command is 4 bytes (motion_cmd_packed_t). */
	cmbf.mbfatr = TA_TFIFO;
	cmbf.bufsz  = MOTION_CMD_QUEUE_BYTES;
	cmbf.maxmsz = (MOTION_USE_PACKED != 0) ? (INT)sizeof(motion_cmd_packed_t)
					       : (INT)sizeof(motion_cmd_t);
	mbf_motion_cmd = tk_cre_mbf(&cmbf);
	if (mbf_motion_cmd < E_OK) return mbf_motion_cmd;

	/* Event flags.  flg_estop is created here because the motion task is
	   the one that RECEIVES the stop; subsystem 4 only sets it.  flg_fault
	   is the team's shared fault bitmask (one bit per fault): the kernel
	   sets/clears bits atomically, so tasks on both cores are safe. */
	flg_move_done = s2_cre_flg();
	if (flg_move_done < E_OK) return flg_move_done;
	flg_estop = s2_cre_flg();
	if (flg_estop < E_OK) return flg_estop;
	flg_fault = s2_cre_flg();
	if (flg_fault < E_OK) return flg_fault;

	links.flg_estop     = flg_estop;
	links.flg_fault     = flg_fault;
	links.mbf_cmd       = mbf_motion_cmd;
	links.flg_move_done = flg_move_done;
	motion_core_init(&links);	/* motors off, encoder pins + IRQ branches */

#if S2_KEEP_TEMPLATE_EXAMPLE
	return s2_example_init();
#else
	return E_OK;
#endif
}

EXPORT ER subsystem2_start(void)
{
#if (S2_MOTOR_DEMO != 0)
	tid_motion = app_start_task((FP)s2_motor_demo_task, S2_PRI_MOTION, S2_MOTION_CORE,
				    "motor_demo");
#else
	tid_motion = app_start_task((FP)motion_task, S2_PRI_MOTION, S2_MOTION_CORE,
				    "motion_task");
#endif
	if (tid_motion < E_OK) return tid_motion;

#if defined(TM_MQTT) && (TM_MQTT == 1) && (S2_TELEMETRY_TEST != 0) && \
    (S2_TELEMETRY_DRIVE != 0) && (S2_MOTOR_DEMO == 0)
	/* Wi-Fi telemetry quick test: drive pattern (s2_telemetry_drive.c) */
	if (app_start_task((FP)s2_telemetry_drive_task, S2_PRI_MOTION + 1, S2_MOTION_CORE,
			   "s2_drive") < E_OK) {
		return E_SYS;
	}
#endif

#if S2_KEEP_TEMPLATE_EXAMPLE
	return s2_example_start();
#else
	return E_OK;
#endif
}
