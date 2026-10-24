/*
 *	drv_encoder.h
 *	Subsystem 2 -- x4 quadrature wheel encoders (left GP2/GP3, right GP16/GP17).
 *	Counting runs in the shared GPIO interrupt; tasks only read.
 *
 *	Other subsystems may call encoder_read() (read-only), e.g. IMU Terrain
 *	for the distance step and Barcode for bar widths.
 */

#ifndef DRV_ENCODER_H
#define DRV_ENCODER_H

#include <stdint.h>

typedef enum {
	ENC_LEFT  = 0,
	ENC_RIGHT = 1,
	ENC_COUNT = 2
} enc_wheel_t;

void     encoder_init(void);			/* pins + register IRQ branches */
void     encoder_on_edge(enc_wheel_t wheel, uint32_t levels);	/* IRQ branch */

/* Running counts (+ = forward). Each is one aligned 32-bit word written
   only by the IRQ, so every read is atomic. */
void     encoder_read(int32_t *p_left, int32_t *p_right);

/* Skipped quadrature steps since the last call (motion task only) */
uint32_t encoder_new_errors(enc_wheel_t wheel);

#ifdef HOST_TEST
void     encoder_test_inject(enc_wheel_t wheel, int32_t counts);
void     encoder_test_reset(void);
#endif

#endif /* DRV_ENCODER_H */
