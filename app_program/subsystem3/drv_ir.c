/*
 *	drv_ir.c
 *	Subsystem 3 -- IR hardware.
 *
 *	ADC: read with the RP2040 registers directly (same style as S5's s5_hw.h),
 *	because the kernel's ADC device driver would need tk_opn_dev and an
 *	interrupt per conversion. ONE task (Line Follow) reads it: TEAM.md section 3.
 *
 *	GP7 edges: the shared GPIO interrupt (S2, drv_gpio_irq.c) calls our branch
 *	with a timestamp it took at IRQ entry. The branch ONLY stores
 *	(time, encoder count, level) in a ring buffer and sets flg_bar_edge
 *	(< 2 us rule, IF-11).
 *
 *	APIs used: gpio_irq_register (S2), encoder_read (S2, IF-12), tk_set_flg,
 *	           libbsp gpio_set_pin, sysdef.h register macros
 */
#include "subsystem3.h"
#include "drv_ir.h"
#include "../subsystem2/drv_gpio_irq.h"	/* gpio_irq_register(), GPIO_EVT_* */
#include "../subsystem2/drv_encoder.h"		/* encoder_read() */

/* RP2040 ADC registers (datasheet 4.9). Own names: the kernel's ADC driver uses ADC_CS etc. */
#define S3_ADC_BASE		0x4004C000u
#define S3_ADC_CS		(S3_ADC_BASE + 0x00)
#define S3_ADC_RESULT		(S3_ADC_BASE + 0x04)
#define S3_ADC_DIV		(S3_ADC_BASE + 0x10)
#define S3_ADC_CS_EN		(1u << 0)
#define S3_ADC_CS_START_ONCE	(1u << 2)
#define S3_ADC_CS_READY		(1u << 8)
#define S3_ADC_CS_AINSEL_POS	12
#define S3_GPIO_FUNC_NULL	31u			/* pin function "none" (analog) */

LOCAL const UB	adc_pin[3] = { S3_PIN_IR_LEFT, S3_PIN_IR_CENTER, S3_PIN_IR_RIGHT };	/* ADC0..ADC2 = GP26..GP28 */
LOCAL volatile UW	adc_errors;

/* ------------------------------------------------------------------ edge ring buffer */
LOCAL volatile s3_edge_t	edge_buf[S3_EDGE_BUF_SIZE];
LOCAL volatile UW	edge_head;		/* written only by the IRQ  */
LOCAL volatile UW	edge_tail;		/* written only by the task */
LOCAL volatile UW	edge_total;
LOCAL volatile UW	edge_lost;

#define MEM_BARRIER()	__asm volatile("dmb" ::: "memory")	/* the IRQ runs on core 1, the task may run on core 0 */

/* Runs inside the shared GPIO IRQ (core 1): store the edge, wake the task. Nothing else. */
LOCAL void edge_branch(uint32_t gpio, uint32_t events, uint32_t levels, uint32_t t_us)
{
	UW	h = edge_head;
	UW	lv;
	W	l, r;

	(void)gpio;
	if (((events & GPIO_EVT_RISE) != 0u) && ((events & GPIO_EVT_FALL) != 0u)) {
		return;				/* rise AND fall latched together: a spike shorter than the IRQ latency */
	}
	lv = ((events & GPIO_EVT_RISE) != 0u) ? 1u : 0u;	/* direction of THIS edge (levels may already have moved on) */
	(void)levels;

	edge_total++;
	if ((UW)(h - edge_tail) >= S3_EDGE_BUF_SIZE) {		/* buffer full: count it, never block */
		edge_lost++;
		return;
	}
	encoder_read((int32_t *)&l, (int32_t *)&r);		/* IF-12: two aligned 32-bit reads */
	edge_buf[h & (S3_EDGE_BUF_SIZE - 1)].t_us  = t_us;
	edge_buf[h & (S3_EDGE_BUF_SIZE - 1)].enc   = (l + r) / 2;
	edge_buf[h & (S3_EDGE_BUF_SIZE - 1)].level = (UB)lv;
	MEM_BARRIER();
	edge_head = h + 1;				/* move head LAST: the task never sees a half-written entry */

	(void)tk_set_flg(flg_bar_edge, FLG_S3_EDGE);
}

BOOL ir_edge_pop(s3_edge_t *e)
{
	UW	t = edge_tail;

	if (t == edge_head) {
		return FALSE;
	}
	MEM_BARRIER();
	e->t_us  = edge_buf[t & (S3_EDGE_BUF_SIZE - 1)].t_us;
	e->enc   = edge_buf[t & (S3_EDGE_BUF_SIZE - 1)].enc;
	e->level = edge_buf[t & (S3_EDGE_BUF_SIZE - 1)].level;
	MEM_BARRIER();
	edge_tail = t + 1;
	return TRUE;
}

UW ir_edge_count(void)   { return edge_total; }
UW ir_edge_dropped(void) { return edge_lost; }
UW ir_adc_errors(void)   { return adc_errors; }
UW ir_time_us(void)      { return in_w(TIMER_TIMERAWL); }

/* ------------------------------------------------------------------ hardware setup */
void ir_hw_init(void)
{
	INT	i;

	/* ADC out of reset (same guard as S2's motor_init: only CLEAR the bit) */
	if ((in_w(RESETS_RESET_DONE) & RESETS_RESET_ADC) == 0u) {
		clr_w(RESETS_RESET, RESETS_RESET_ADC);
		while ((in_w(RESETS_RESET_DONE) & RESETS_RESET_ADC) == 0u) { }
	}
	out_w(S3_ADC_CS, S3_ADC_CS_EN);

	/* GP26..GP28: analog. No digital input buffer, no pull resistors (the default pull-down would load the sensor) */
	for (i = 0; i < 3; i++) {
		clr_w(GPIO(adc_pin[i]), GPIO_IE | GPIO_PUE | GPIO_PDE);
		out_w(GPIO_CTRL(adc_pin[i]), S3_GPIO_FUNC_NULL);
	}

	/* GP7: digital input, no pull; edges arrive through S2's shared GPIO IRQ */
	(void)gpio_set_pin(S3_PIN_IR_RIGHT_DO, GPIO_MODE_IN);
	clr_w(GPIO(S3_PIN_IR_RIGHT_DO), GPIO_PUE | GPIO_PDE);
	gpio_irq_register(S3_PIN_IR_RIGHT_DO, edge_branch);	/* IF-11; enabled by motion_task on core 1 */
}

/* One conversion on ADC input 'ch' (0..2). 12-bit result. */
LOCAL BOOL adc_one(UINT ch, uint16_t *out)
{
	UW	n;

	out_w(S3_ADC_CS, S3_ADC_CS_EN | (ch << S3_ADC_CS_AINSEL_POS) | S3_ADC_CS_START_ONCE);
	for (n = 0; n < 2000u; n++) {			/* a conversion takes ~2 us; 2000 loops is far more than enough */
		if ((in_w(S3_ADC_CS) & S3_ADC_CS_READY) != 0u) {
			*out = (uint16_t)(in_w(S3_ADC_RESULT) & 0x0FFFu);
			return TRUE;
		}
	}
	adc_errors++;
	return FALSE;
}

BOOL ir_adc_read3(uint16_t raw[3])
{
	BOOL	ok = TRUE;
	UINT	ch;

	for (ch = 0; ch < 3; ch++) {
		if (!adc_one(ch, &raw[ch])) {
			raw[ch] = 0;
			ok = FALSE;
		}
	}
	return ok;
}
