/*
 *	drv_ir.h
 *	Subsystem 3 -- IR hardware: 3 analog inputs (ADC, GP26-GP28) read by the
 *	Line Follow task, and the right IR digital output (GP7) whose edges come
 *	through Subsystem 2's shared GPIO interrupt.
 */
#ifndef DRV_IR_H
#define DRV_IR_H

#include "../app.h"
#include "s3_logic.h"

void	ir_hw_init(void);			/* ADC, GP26-28 analog, GP7 input + IRQ branch */
BOOL	ir_adc_read3(uint16_t raw[3]);		/* TRUE = ok. Line Follow task ONLY (single-owner rule) */

/* ISR -> Barcode task ring buffer (single producer = the IRQ, single consumer = the task) */
BOOL	ir_edge_pop(s3_edge_t *e);		/* TRUE = one edge returned */
UW	ir_edge_count(void);			/* edges recorded so far                     */
UW	ir_edge_dropped(void);			/* edges lost because the buffer was full    */
UW	ir_adc_errors(void);			/* ADC conversions that timed out            */
UW	ir_time_us(void);			/* same microsecond clock as the edge stamps */

#endif /* DRV_IR_H */
