/*
 *	drv_mpu6050.h
 *	Subsystem 4 -- IMU on I2C0, GP4 (SDA) / GP5 (SCL): MPU-6050 (GY-521) or
 *	LSM303 (GY-511, no gyro), found automatically by mpu_init().
 *	Called ONLY by the IMU Safety task: it owns I2C0 (TEAM.md section 3).
 */
#ifndef DRV_MPU6050_H
#define DRV_MPU6050_H

#include <stdint.h>
#include <stdbool.h>
#include "s4_logic.h"

/* Resets I2C0, sets the pins and 400 kHz, wakes the sensor, checks WHO_AM_I
 * and sets the ranges (accel +-4 g so a 2.5 g impact is not clipped,
 * gyro +-250 dps, 44 Hz low-pass). Returns false if the sensor does not answer.
 * full_reset = true also resets the chip and waits 100 ms: use it only at start-up,
 * before the watchdog runs (a retry while running uses false: a few ms).        */
bool     mpu_init(bool full_reset);
/* One 14-byte burst read, converted to g and deg/s. false = bus error/timeout. */
bool     mpu_read(s4_sample_t *out);
uint8_t  mpu_who_am_i(void);      /* value read by mpu_init (0x68 for an MPU-6050) */
uint32_t mpu_errors(void);        /* failed transfers so far                       */
bool     mpu_has_gyro(void);      /* false for the LSM303 (GY-511): gx/gy/gz are 0  */
bool     mpu_has_mag(void);       /* GY-511 compass found                           */
bool     mpu_mag_read(float *mx, float *my, float *mz);   /* raw compass, false = bus error */
const char *mpu_chip_name(void);  /* which chip mpu_init() found                    */
uint8_t  mpu_addr(void);          /* its I2C address                                */

#endif /* DRV_MPU6050_H */
