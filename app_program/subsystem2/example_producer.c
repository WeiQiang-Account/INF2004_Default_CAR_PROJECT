/*
 *	example_producer.c
 *	TEMPLATE EXAMPLE kept only so subsystems 3/4's example code still
 *	builds. Unchanged from the template except the init/start names.
 *	Delete this file and set S2_KEEP_TEMPLATE_EXAMPLE to 0 (subsystem2.h)
 *	once subsystems 3 and 4 have replaced their examples.
 */

#include "subsystem2.h"

#if S2_KEEP_TEMPLATE_EXAMPLE
#include "../subsystem3/subsystem3.h"	/* IF-01: mbx_samples */

/* A fake measurement: a triangle wave between 0 and 100 */
EXPORT INT s2_make_value(UW seq)
{
	INT	phase = (INT)(seq % 20);
	return (phase < 10 ? phase : 20 - phase) * 10;
}

EXPORT void producer_task(INT stacd, void *exinf)
{
	UW		seq = 0;
	SAMPLE_MSG	*msg;
	SYSTIM		now;
	ER		er;

	(void)stacd; (void)exinf;

	while (1) {
		/* 1. Take a free slot.  If the consumer is behind and all
		      slots are in use, this task blocks here until one is
		      returned (flow control). */
		er = tk_wai_sem(sem_slots, 1, TMO_FEVR);
		CHECK(er, "tk_wai_sem");

		/* 2. Get a memory block for the message.  The semaphore
		      guarantees one is free, so poll (TMO_POL = don't wait). */
		er = tk_get_mpf(mpf_samples, (void **)&msg, TMO_POL);
		if (er < E_OK) {
			CHECK(er, "tk_get_mpf");
			tk_sig_sem(sem_slots, 1);	/* give the slot back */
			tk_dly_tsk(S2_PERIOD_MS);
			continue;
		}

		/* 3. Fill it in.  tk_get_tim gives milliseconds since boot
		      as a 64-bit hi:lo pair; lo is enough for ~49 days. */
		tk_get_tim(&now);
		msg->seq     = seq++;
		msg->time_ms = now.lo;
		msg->value   = s2_make_value(msg->seq);
#if TK_SUPPORT_SMP
		msg->prc     = tk_get_prc();	/* which core am I on? */
#else
		msg->prc     = 1;
#endif

		/* 4. Send the POINTER to subsystem 3.  From here on the block
		      belongs to the receiver: do not touch msg again. */
		er = tk_snd_mbx(mbx_samples, (T_MSG *)msg);
		CHECK(er, "tk_snd_mbx");

		/* 5. Wait for the next period.  Other tasks run meanwhile. */
		tk_dly_tsk(S2_PERIOD_MS);
	}
}

EXPORT ID	sem_slots;	/* IF-02, see subsystem2.h */
EXPORT ID	mpf_samples;

EXPORT ER s2_example_init(void)
{
	T_CSEM	csem = {0};
	T_CMPF	cmpf = {0};

	/* Semaphore: a counter of free "slots".  The producer takes one
	   (tk_wai_sem) before each sample; the consumer gives one back
	   (tk_sig_sem).  It never exceeds the number of memory blocks, so
	   the producer can never run out of memory. */
	csem.sematr  = TA_TFIFO;	/* waiting tasks queue in FIFO order */
	csem.isemcnt = S2_POOL_COUNT;	/* initial count */
	csem.maxsem  = S2_POOL_COUNT;	/* maximum count */
	sem_slots = tk_cre_sem(&csem);
	if (sem_slots < E_OK) {
		return sem_slots;
	}

	/* Fixed-size memory pool: S2_POOL_COUNT blocks, each exactly one
	   SAMPLE_MSG.  Faster and more predictable than malloc(). */
	cmpf.mpfatr = TA_TFIFO | TA_RNG3;
	cmpf.mpfcnt = S2_POOL_COUNT;		/* number of blocks */
	cmpf.blfsz  = sizeof(SAMPLE_MSG);	/* size of one block */
	mpf_samples = tk_cre_mpf(&cmpf);
	return (mpf_samples < E_OK) ? mpf_samples : E_OK;
}

EXPORT ER s2_example_start(void)
{
	/* Pinned to core 1 (TP_PRC2): samples cross to subsystem 3 on core 0 */
	ID tid = app_start_task(producer_task, S2_PRI_PRODUCER, TP_PRC2,
				"producer_task");
	return (tid < E_OK) ? tid : E_OK;
}

#endif /* S2_KEEP_TEMPLATE_EXAMPLE */
