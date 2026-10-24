/*
 *	s2_os_port.c
 *	Subsystem 2 -- s2_os_port.h implemented with micro T-Kernel calls.
 *
 *	APIs used: tk_set_flg, tk_clr_flg, tk_wai_flg, tk_snd_mbf, tk_rcv_mbf,
 *	           tk_get_tim, tk_dly_tsk
 */

#include "../app.h"
#include "s2_os_port.h"

void os_flag_set(os_id_t flg, uint32_t bits)
{
	(void)tk_set_flg((ID)flg, (UINT)bits);
}

void os_flag_clear(os_id_t flg, uint32_t bits)
{
	/* tk_clr_flg KEEPS the bits you pass (pattern &= clrptn),
	   so pass the inverse of the bits to clear. */
	(void)tk_clr_flg((ID)flg, (UINT)~bits);
}

bool os_flag_poll(os_id_t flg, uint32_t bits)
{
	UINT ptn = 0;
	return tk_wai_flg((ID)flg, (UINT)bits, TWF_ORW, &ptn, TMO_POL) == E_OK;
}

bool os_flag_wait_any(os_id_t flg, uint32_t bits, uint32_t timeout_ms)
{
	UINT ptn = 0;
	return tk_wai_flg((ID)flg, (UINT)bits, TWF_ORW, &ptn, (TMO)timeout_ms) == E_OK;
}

bool os_mbf_send(os_id_t mbf, const void *p_msg, uint32_t size, int32_t timeout_ms)
{
	return tk_snd_mbf((ID)mbf, p_msg, (INT)size, (TMO)timeout_ms) == E_OK;
}

int32_t os_mbf_recv_poll(os_id_t mbf, void *p_msg)
{
	INT n = tk_rcv_mbf((ID)mbf, p_msg, TMO_POL);
	return (n >= 0) ? (int32_t)n : -1;
}

uint32_t os_now_ms(void)
{
	SYSTIM t;
	(void)tk_get_tim(&t);
	return (uint32_t)t.lo;
}

void os_sleep_ms(uint32_t ms)
{
	(void)tk_dly_tsk((RELTIM)ms);
}
