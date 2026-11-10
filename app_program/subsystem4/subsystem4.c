/*
 *	subsystem4.c
 *	Subsystem 4 -- Buddy 4: creates its objects and starts its tasks.
 *	Also: the shared sample (Safety -> Terrain), telemetry getters, the VC
 *	"IMU ready" hook and the hardware watchdog (NFR20).
 *
 *	Tasks : imu_safety_task   S4_PRI_SAFETY  (10) every 20 ms, core 1 (next to Motion)
 *	        imu_terrain_task  S4_PRI_TERRAIN (19) woken by flg_imu_new
 *	APIs used: tk_cre_flg, app_start_task, in_w / out_w (watchdog registers)
 */
#include "subsystem4.h"

EXPORT ID	flg_imu_new;

/* ------------------------------------------------------------------ shared sample */
LOCAL volatile UW	sh_seq;			/* odd while being written */
LOCAL s4_sample_t	sh_sample;
LOCAL volatile BOOL	sh_valid;
LOCAL volatile UW	sh_count;

void s4_shared_write(const s4_sample_t *s)
{
	sh_seq++;
	__sync_synchronize();
	sh_sample = *s;
	sh_valid = TRUE;
	sh_count++;
	__sync_synchronize();
	sh_seq++;
}

void s4_shared_read(s4_shared_t *out)
{
	UW a, b;
	do {				/* the writer is on the other core: retry on a torn copy */
		a = sh_seq;
		__sync_synchronize();
		out->s = sh_sample;
		out->valid = sh_valid;
		__sync_synchronize();
		b = sh_seq;
	} while ((a != b) || ((a & 1u) != 0u));
	out->seq = a;
}

/* ------------------------------------------------------------------ telemetry values */
LOCAL volatile UB	g_state = S4_IMU_CALIBRATING;
LOCAL volatile BOOL	g_ready;
LOCAL volatile H	g_pitch_x10;
LOCAL volatile H	g_yaw;
LOCAL volatile UB	g_mclass, g_terrain;
LOCAL volatile UH	g_hump_now, g_hump_last, g_hump_max, g_humps, g_peak_cg;
LOCAL volatile BOOL	g_wdt_reset;
LOCAL volatile float	g_hdg;			/* compass heading, deg, left = + */
LOCAL volatile BOOL	g_hdg_ok;
LOCAL volatile UB	g_es_why;		/* last E-stop: S4_SAFE_IMPACT / S4_SAFE_TILT bits */
LOCAL volatile UH	g_es_cg, g_es_tilt;	/* |a| (0.01 g) and tilt (deg) on that sample */

void s4_set_imu_state(UINT st)
{
	g_state = (UB)st;
	if ((st == S4_IMU_MONITORING) || (st == S4_IMU_LOST)) {
		g_ready = TRUE;		/* VC may leave INIT: degraded is allowed, not a blocker */
	}
}

void s4_terrain_publish(INT pitch_x10, UINT mclass, INT yaw, const s4_hump_t *h)
{
	g_pitch_x10 = (H)pitch_x10;
	g_mclass = (UB)mclass;
	g_yaw = (H)yaw;
	g_terrain = (UB)h->state;
	g_hump_now = (h->state == S4_FLAT) ? 0u : (UH)((h->h_mm > 0.0f) ? (h->h_mm + 0.5f) : 0.0f);
	g_hump_last = h->last_peak_mm;
	g_hump_max = h->max_peak_mm;
	g_humps = h->humps;
}

void s4_note_accel(float mag)
{
	UH cg = (UH)((mag * 100.0f) + 0.5f);
	if (cg > g_peak_cg) {
		g_peak_cg = cg;
	}
}

void s4_note_estop(UINT why, float mag, float tilt)
{
	g_es_cg   = (UH)((mag * 100.0f) + 0.5f);
	g_es_tilt = (UH)((tilt > 0.0f) ? (tilt + 0.5f) : 0.0f);
	g_es_why  = (UB)why;
}
UINT s4_estop_why(void)          { return g_es_why; }
UINT s4_estop_cg(void)           { return g_es_cg; }
UINT s4_estop_tilt(void)         { return g_es_tilt; }

void s4_heading_publish(float deg, BOOL ok)
{
	g_hdg = deg;
	g_hdg_ok = ok;
}
/* dashboard: whole degrees -180..180, or 999 when there is no compass */
INT  s4_heading_deg(void)        { return g_hdg_ok ? (INT)((g_hdg >= 0.0f) ? (g_hdg + 0.5f) : (g_hdg - 0.5f)) : 999; }

/* Motion (S2) asks this at the start of a straight move and every 20 ms.
   Strong definition: overrides the weak default in subsystem2/task_motion.c. */
bool s2_hook_heading(float *deg)
{
	if (!g_hdg_ok) { return false; }
	*deg = g_hdg;
	return true;
}

UINT s4_imu_state(void)          { return g_state; }
BOOL s4_imu_ready(void)          { return g_ready; }
INT  s4_pitch_x10(void)          { return g_pitch_x10; }
INT  s4_turn_rate_dps(void)      { return g_yaw; }
UINT s4_motion_class(void)       { return g_mclass; }
UINT s4_terrain_state(void)      { return g_terrain; }
UINT s4_hump_mm(void)            { return g_hump_now; }
UINT s4_hump_last_mm(void)       { return g_hump_last; }
UINT s4_hump_max_mm(void)        { return g_hump_max; }
UINT s4_hump_count(void)         { return g_humps; }
UINT s4_peak_accel_cg(void)      { return g_peak_cg; }
BOOL s4_watchdog_reset_seen(void){ return g_wdt_reset; }
UW   s4_sample_count(void)       { return sh_count; }

/* Buddy 5's Vehicle Controller asks this in INIT. Strong definition: overrides the
   weak default in subsystem5/s5_port.c. Ready also when the IMU is LOST: the robot
   runs degraded (50 % speed, no humps) instead of refusing to start (Week 6 6.4). */
int s5_hook_imu_ready(void)
{
	return (g_ready != FALSE) ? 1 : 0;
}

/* ------------------------------------------------------------------ watchdog (NFR20) */
#define S4_WDT_BASE		0x40058000u
#define S4_WDT_CTRL		(S4_WDT_BASE + 0x00u)
#define S4_WDT_LOAD		(S4_WDT_BASE + 0x04u)
#define S4_WDT_REASON		(S4_WDT_BASE + 0x08u)
#define S4_WDT_CTRL_ENABLE	(1u << 30)
#define S4_WDT_CTRL_PAUSE	((1u << 24) | (1u << 25) | (1u << 26))	/* stop while a debugger halts */
#define S4_WDT_REASON_TIMER	(1u << 0)
#define S4_WDT_TICK		(S4_WDT_BASE + 0x2Cu)
#define S4_WDT_TICK_ENABLE	(1u << 9)
#define S4_WDT_TICK_CYCLES	12u		/* clk_ref = 12 MHz crystal (cpu_clock.c) -> 1 us */
#define S4_PSM_WDSEL		(0x40010000u + 0x08u)
#define S4_PSM_WDSEL_ALL	0x0001FFFCu	/* reset everything except the oscillators */
/* RP2040-E1: the counter decrements twice per 1 us tick, so load 2x the time. */
#define S4_WDT_TICKS		((UW)S4_WATCHDOG_MS * 1000u * 2u)

LOCAL BOOL wdt_running;

void s4_watchdog_start(void)
{
#if S4_WATCHDOG_ENABLE
	out_w(S4_PSM_WDSEL, S4_PSM_WDSEL_ALL);
	out_w(S4_WDT_LOAD, S4_WDT_TICKS);
	set_w(S4_WDT_CTRL, S4_WDT_CTRL_PAUSE | S4_WDT_CTRL_ENABLE);
	s4_tick_check();
	wdt_running = TRUE;
	S4_PRINT("[S4] hardware watchdog on: %d ms (NFR20)\n", (INT)S4_WATCHDOG_MS);
#endif
}

/* Keep the 1 us tick right: if anything changed it, put it back (and say so once). */
void s4_tick_check(void)
{
	static BOOL told = FALSE;
	UW t = in_w(S4_WDT_TICK) & 0x3FFu;		/* ENABLE + CYCLES */
	if (t != (S4_WDT_TICK_ENABLE | S4_WDT_TICK_CYCLES)) {
		out_w(S4_WDT_TICK, S4_WDT_TICK_ENABLE | S4_WDT_TICK_CYCLES);
		if (!told) {
			told = TRUE;
			S4_PRINT("[S4] us tick was changed to 0x%x - restored to 1 us\n", (UINT)t);
		}
	}
}

void s4_watchdog_feed(void)
{
	if (wdt_running) {
		out_w(S4_WDT_LOAD, S4_WDT_TICKS);
	}
}

/* ------------------------------------------------------------------ init / start */
EXPORT ER subsystem4_init(void)
{
	T_CFLG	cflg = {0};

	cflg.flgatr  = TA_TFIFO | TA_WMUL;
	cflg.iflgptn = 0;
	flg_imu_new = tk_cre_flg(&cflg);
	if (flg_imu_new < E_OK) {
		return flg_imu_new;
	}

	/* The watchdog block also makes the 1 us tick that drives TIMER (TIMERAWL):
	   every us measurement in the robot (ultrasonic echo, barcode widths, I2C
	   timeouts) depends on it, and nothing in the kernel sets it up. 2026-11-06:
	   with S4 running, echo times read ~13x too small (215 us -> 16 us). Set it
	   to 12 cycles of the 12 MHz clk_ref = exactly 1 us, before anyone uses it. */
	{
		UW before = in_w(S4_WDT_TICK);
		out_w(S4_WDT_TICK, S4_WDT_TICK_ENABLE | S4_WDT_TICK_CYCLES);
		S4_PRINT("[S4] us tick: was 0x%x, set to %d cycles of 12 MHz (1 us)\n", (UINT)before, (INT)S4_WDT_TICK_CYCLES);
	}

	/* Was the last reset caused by the watchdog? Reported once over telemetry (NFR20). */
	g_wdt_reset = ((in_w(S4_WDT_REASON) & S4_WDT_REASON_TIMER) != 0u) ? TRUE : FALSE;
	if (g_wdt_reset) {
		S4_PRINT("[S4] WATCHDOG_RESET: the last run stopped completing its 20 ms cycles\n");
	}
	return E_OK;
}

EXPORT ER subsystem4_start(void)
{
	ID tid;

	/* Safety on core 1 next to Motion Control: core 0 runs USB (pri 3) and WiFi
	   polling (pri 4), which would delay a 20 ms safety loop placed there. */
	tid = app_start_task((FP)imu_safety_task, S4_PRI_SAFETY, TP_PRC2, "imu_safety");
	if (tid < E_OK) {
		return tid;
	}
	tid = app_start_task((FP)imu_terrain_task, S4_PRI_TERRAIN, 0, "imu_terrain");
	return (tid < E_OK) ? tid : E_OK;
}
