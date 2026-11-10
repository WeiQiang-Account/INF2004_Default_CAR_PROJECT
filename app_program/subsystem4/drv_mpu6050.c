/*
 *	drv_mpu6050.c
 *	Subsystem 4 -- IMU driver, I2C0 by direct register access (polled).
 *	Finds the chip at start-up: MPU-6050 (GY-521, accel + gyro) or LSM303
 *	(GY-511, accel + compass, NO gyro; 2026-11-07 this is what the robot has).
 *	Both give the same s4_sample_t; with the LSM303 the gyro fields are 0 and
 *	mpu_has_gyro() is false (S4 then uses accel-only pitch, wheel turn rate).
 *
 *	Why not the Pico SDK (hardware/i2c.h, sleep_ms, printf)? The project runs
 *	on micro T-Kernel and does not link the SDK runtime. Why not the kernel's
 *	"iica" driver? It is off by default (DEVCNF_USE_IIC 0), its default pins
 *	are GP8/GP9 (our left motor) and GP6/GP7 (GP7 = barcode), and its timeout
 *	is 1 s. A polled 14-byte read at 400 kHz takes about 0.4 ms, with a 2 ms
 *	timeout, which suits the 20 ms safety loop. Same style as S3's ADC driver.
 *
 *	APIs used: in_w / out_w / set_w / clr_w (sysdef.h), libbsp GPIO macros
 */
#include "../app.h"
#include "drv_mpu6050.h"

/* RP2040 I2C0 (DW_apb_i2c, datasheet 4.3.17). Own names: the kernel driver uses I2C_*. */
#define S4_I2C_BASE		0x40044000u
#define S4_IC_CON		(S4_I2C_BASE + 0x00u)
#define S4_IC_TAR		(S4_I2C_BASE + 0x04u)
#define S4_IC_DATA_CMD		(S4_I2C_BASE + 0x10u)
#define S4_IC_FS_SCL_HCNT	(S4_I2C_BASE + 0x1Cu)
#define S4_IC_FS_SCL_LCNT	(S4_I2C_BASE + 0x20u)
#define S4_IC_INTR_MASK		(S4_I2C_BASE + 0x30u)
#define S4_IC_RAW_INTR_STAT	(S4_I2C_BASE + 0x34u)
#define S4_IC_RX_TL		(S4_I2C_BASE + 0x38u)
#define S4_IC_TX_TL		(S4_I2C_BASE + 0x3Cu)
#define S4_IC_CLR_TX_ABRT	(S4_I2C_BASE + 0x54u)
#define S4_IC_CLR_STOP_DET	(S4_I2C_BASE + 0x60u)
#define S4_IC_ENABLE		(S4_I2C_BASE + 0x6Cu)
#define S4_IC_TXFLR		(S4_I2C_BASE + 0x74u)
#define S4_IC_RXFLR		(S4_I2C_BASE + 0x78u)
#define S4_IC_SDA_HOLD		(S4_I2C_BASE + 0x7Cu)
#define S4_IC_ENABLE_STATUS	(S4_I2C_BASE + 0x9Cu)
#define S4_IC_FS_SPKLEN		(S4_I2C_BASE + 0xA0u)

#define S4_CON_MASTER		(1u << 0)
#define S4_CON_SPEED_FAST	(2u << 1)
#define S4_CON_RESTART_EN	(1u << 5)
#define S4_CON_SLAVE_DISABLE	(1u << 6)
#define S4_CON_TX_EMPTY_CTRL	(1u << 8)
#define S4_CMD_READ		(1u << 8)
#define S4_CMD_STOP		(1u << 9)
#define S4_CMD_RESTART		(1u << 10)
#define S4_INTR_TX_ABRT		(1u << 6)
#define S4_INTR_STOP_DET	(1u << 9)
#define S4_FIFO_DEPTH		16u
#define S4_GPIO_FUNC_I2C	3u

/* MPU-6050 registers */
#define MPU_SMPLRT_DIV		0x19u
#define MPU_CONFIG		0x1Au
#define MPU_GYRO_CONFIG		0x1Bu
#define MPU_ACCEL_CONFIG	0x1Cu
#define MPU_ACCEL_XOUT_H	0x3Bu
#define MPU_PWR_MGMT_1		0x6Bu
#define MPU_WHO_AM_I		0x75u

#define MPU_ACC_LSB_PER_G	8192.0f		/* +-4 g   */

/* LSM303DLHC / LSM303AGR accelerometer registers */
#define LSM_WHO_AM_I_A		0x0Fu		/* AGR: 0x33 (the DLHC may not have it) */
#define LSM_CTRL_REG1_A		0x20u
#define LSM_CTRL_REG4_A		0x23u
#define LSM_CTRL_REG5_A		0x24u
#define LSM_OUT_X_L_A		0x28u
#define LSM_AUTO_INC		0x80u		/* sub-address MSB: read several registers */
#define LSM_ACC_LSB_PER_G	8000.0f		/* +-4 g, high-res: 2 mg per 12-bit step, data left-aligned (x16) */

/* LSM303 magnetometer (compass) at 0x1E: DLHC or AGR register sets */
#define MAG_ADDR		0x1Eu
#define DLHC_CRA_REG_M		0x00u
#define DLHC_CRB_REG_M		0x01u
#define DLHC_MR_REG_M		0x02u
#define DLHC_OUT_X_H_M		0x03u		/* X_H X_L Z_H Z_L Y_H Y_L (big-endian, X Z Y) */
#define DLHC_IRA_REG_M		0x0Au		/* reads 'H' (0x48) */
#define AGR_WHO_AM_I_M		0x4Fu		/* reads 0x40 */
#define AGR_CFG_REG_A_M		0x60u
#define AGR_CFG_REG_C_M		0x62u
#define AGR_OUTX_L_REG_M	0x68u		/* X_L X_H Y_L Y_H Z_L Z_H (little-endian) */

enum { CHIP_NONE = 0, CHIP_MPU6050, CHIP_LSM303 };
enum { MAG_NONE = 0, MAG_DLHC, MAG_AGR };
LOCAL uint8_t	mag = MAG_NONE;
LOCAL uint8_t	chip = CHIP_NONE;
LOCAL uint8_t	chip_addr;
#define MPU_GYR_LSB_PER_DPS	131.0f		/* +-250 dps */

LOCAL uint8_t	who;
LOCAL volatile uint32_t	errors;

LOCAL uint32_t now_us(void) { return (uint32_t)in_w(TIMER_TIMERAWL); }

LOCAL void bus_enable(bool on)
{
	uint32_t t0 = now_us();
	out_w(S4_IC_ENABLE, on ? 1u : 0u);
	while (((in_w(S4_IC_ENABLE_STATUS) & 1u) != (on ? 1u : 0u)) && ((now_us() - t0) < 1000u)) {
		/* takes a few I2C clock periods */
	}
}

LOCAL bool aborted(void)
{
	if ((in_w(S4_IC_RAW_INTR_STAT) & S4_INTR_TX_ABRT) != 0u) {
		(void)in_w(S4_IC_CLR_TX_ABRT);	/* reading clears it */
		return true;			/* NAK: sensor missing or address wrong */
	}
	return false;
}

/* Write nw bytes, then (if nr > 0) a repeated START and read nr bytes. */
LOCAL bool xfer(const uint8_t *w, uint32_t nw, uint8_t *r, uint32_t nr)
{
	uint32_t t0 = now_us();
	uint32_t got = 0, sent = 0, total = nw + nr;

	(void)in_w(S4_IC_CLR_TX_ABRT);
	(void)in_w(S4_IC_CLR_STOP_DET);

	while ((sent < total) || (got < nr)) {
		if (aborted() || ((now_us() - t0) > S4_I2C_TMO_US)) {
			bus_enable(false);		/* flush the FIFOs, then start clean */
			bus_enable(true);
			errors++;
			return false;
		}
		/* never ask for more reads than the 16-byte RX FIFO can hold */
		if ((sent < total) && (in_w(S4_IC_TXFLR) < S4_FIFO_DEPTH) &&
		    ((sent < nw) || (((sent - nw) - got) < S4_FIFO_DEPTH))) {
			uint32_t cmd;
			bool last = (sent == (total - 1u));
			if (sent < nw) {
				cmd = w[sent];
			} else {
				cmd = S4_CMD_READ | ((sent == nw) && (nw > 0u) ? S4_CMD_RESTART : 0u);
			}
			if (last) { cmd |= S4_CMD_STOP; }
			out_w(S4_IC_DATA_CMD, cmd);
			sent++;
		}
		while ((got < nr) && (in_w(S4_IC_RXFLR) > 0u)) {
			r[got++] = (uint8_t)(in_w(S4_IC_DATA_CMD) & 0xFFu);
		}
	}
	if (nr == 0u) {				/* write only: wait for the STOP */
		while ((in_w(S4_IC_RAW_INTR_STAT) & S4_INTR_STOP_DET) == 0u) {
			if (aborted() || ((now_us() - t0) > S4_I2C_TMO_US)) {
				errors++;
				return false;
			}
		}
	}
	return true;
}

/* IC_TAR may only change while the block is disabled */
LOCAL void set_target(uint8_t addr)
{
	bus_enable(false);
	out_w(S4_IC_TAR, addr);
	bus_enable(true);
}

LOCAL bool reg_write(uint8_t reg, uint8_t val)
{
	uint8_t b[2] = { reg, val };
	return xfer(b, 2u, (uint8_t *)0, 0u);
}

LOCAL bool reg_read(uint8_t reg, uint8_t *buf, uint32_t n)
{
	return xfer(&reg, 1u, buf, n);
}

bool mpu_init(bool full_reset)
{
	uint8_t v = 0;
	INT	p;
	const UB pins[2] = { S4_PIN_SDA, S4_PIN_SCL };

	/* I2C0 out of reset (only CLEAR the bit, same guard as S2/S3/S5) */
	if ((in_w(RESETS_RESET_DONE) & RESETS_RESET_I2C0) == 0u) {
		clr_w(RESETS_RESET, RESETS_RESET_I2C0);
		while ((in_w(RESETS_RESET_DONE) & RESETS_RESET_I2C0) == 0u) { }
	}
	/* GP4/GP5: I2C function, input on, pull-ups on (GY-521 also has 4.7k) */
	for (p = 0; p < 2; p++) {
		set_w(GPIO(pins[p]), GPIO_IE | GPIO_PUE);
		clr_w(GPIO(pins[p]), GPIO_PDE | GPIO_OD);
		out_w(GPIO_CTRL(pins[p]), S4_GPIO_FUNC_I2C);
	}

	bus_enable(false);
	out_w(S4_IC_CON, S4_CON_MASTER | S4_CON_SPEED_FAST | S4_CON_RESTART_EN |
			 S4_CON_SLAVE_DISABLE | S4_CON_TX_EMPTY_CTRL);
	out_w(S4_IC_TX_TL, 0u);
	out_w(S4_IC_RX_TL, 0u);
	out_w(S4_IC_INTR_MASK, 0u);		/* polled: no interrupt */
	/* 400 kHz from 125 MHz: 312 cycles per bit, 40 % high (same counts as the Pico SDK) */
	out_w(S4_IC_FS_SCL_HCNT, 125u);
	out_w(S4_IC_FS_SCL_LCNT, 187u);
	out_w(S4_IC_FS_SPKLEN, 11u);
	out_w(S4_IC_SDA_HOLD, 38u);		/* 300 ns hold */
	out_w(S4_IC_TAR, S4_MPU_ADDR);
	bus_enable(true);

	if (chip == CHIP_NONE) {			/* first time: find the chip */
		static const uint8_t mpu_a[2] = { S4_MPU_ADDR, (uint8_t)(S4_MPU_ADDR + 1u) };
		static const uint8_t lsm_a[2] = { S4_LSM_ADDR, (uint8_t)(S4_LSM_ADDR - 1u) };
		uint32_t e0 = errors;			/* a NAK while probing is not an error */
		for (p = 0; (p < 2) && (chip == CHIP_NONE); p++) {
			set_target(mpu_a[p]);
			if (reg_read(MPU_WHO_AM_I, &v, 1u)) { chip = CHIP_MPU6050; chip_addr = mpu_a[p]; }
		}
		for (p = 0; (p < 2) && (chip == CHIP_NONE); p++) {
			set_target(lsm_a[p]);
			if (reg_read(LSM_CTRL_REG1_A, &v, 1u)) { chip = CHIP_LSM303; chip_addr = lsm_a[p]; }
		}
		errors = e0;
		if (chip == CHIP_NONE) { return false; }	/* nothing on GP4/GP5 */
	}
	set_target(chip_addr);

	if (chip == CHIP_LSM303) {
		if (full_reset) {
			(void)reg_write(LSM_CTRL_REG5_A, 0x80u);	/* reboot memory content */
			(void)tk_dly_tsk(20);
		}
		if (!reg_read(LSM_WHO_AM_I_A, &v, 1u)) { v = 0u; }
		who = v;				/* 0x33 = LSM303 (AGR, most DLHC) */
		if (!reg_write(LSM_CTRL_REG1_A, 0x47u) ||	/* 50 Hz (= our 20 ms loop, less vibration), X Y Z on */
		    !reg_write(LSM_CTRL_REG4_A, 0x98u)) {	/* block update, +-4 g, high resolution */
			return false;
		}
		/* compass (optional: heading hold for straight moves). A missing or
		   silent compass never fails the IMU: mpu_has_mag() just stays false. */
		{
			uint32_t e0 = errors;
			mag = MAG_NONE;
			set_target(MAG_ADDR);
			if (reg_read(AGR_WHO_AM_I_M, &v, 1u) && (v == 0x40u)) {
				if (reg_write(AGR_CFG_REG_A_M, 0x8Cu) &&	/* temp. comp., 100 Hz, continuous */
				    reg_write(AGR_CFG_REG_C_M, 0x10u)) {	/* block data update */
					mag = MAG_AGR;
				}
			} else if (reg_read(DLHC_IRA_REG_M, &v, 1u) && (v == 0x48u)) {
				if (reg_write(DLHC_CRA_REG_M, 0x18u) &&		/* 75 Hz */
				    reg_write(DLHC_CRB_REG_M, 0x20u) &&		/* +-1.3 gauss */
				    reg_write(DLHC_MR_REG_M, 0x00u)) {		/* continuous */
					mag = MAG_DLHC;
				}
			} else {
				/* no compass answered */
			}
			errors = e0;
			set_target(chip_addr);
		}
		return true;
	}

	if (full_reset) {
		if (!reg_write(MPU_PWR_MGMT_1, 0x80u)) {	/* device reset */
			return false;
		}
		(void)tk_dly_tsk(100);			/* 100 ms after a reset (start-up only) */
	}
	if (!reg_write(MPU_PWR_MGMT_1, 0x01u)) {	/* wake, clock = gyro X PLL */
		return false;
	}
	if (!reg_read(MPU_WHO_AM_I, &v, 1u)) {
		return false;
	}
	who = v;					/* 0x68 = MPU-6050; clones may read 0x70/0x72/0x98 */
	if (!reg_write(MPU_SMPLRT_DIV, 0x00u) ||
	    !reg_write(MPU_CONFIG, 0x03u) ||		/* 44 Hz low-pass */
	    !reg_write(MPU_GYRO_CONFIG, 0x00u) ||	/* +-250 dps */
	    !reg_write(MPU_ACCEL_CONFIG, 0x08u)) {	/* +-4 g (2 g would clip a 2.5 g impact) */
		return false;
	}
	return true;
}

bool mpu_read(s4_sample_t *o)
{
	uint8_t b[14];
	int16_t v[7];
	int	i;

	if (chip == CHIP_LSM303) {
		if (!reg_read((uint8_t)(LSM_OUT_X_L_A | LSM_AUTO_INC), b, 6u)) {
			return false;
		}
		for (i = 0; i < 3; i++) {		/* little-endian: low byte first */
			v[i] = (int16_t)(((uint16_t)b[(2 * i) + 1] << 8) | b[2 * i]);
		}
		o->ax = (float)v[0] / LSM_ACC_LSB_PER_G;
		o->ay = (float)v[1] / LSM_ACC_LSB_PER_G;
		o->az = (float)v[2] / LSM_ACC_LSB_PER_G;
		o->gx = 0.0f; o->gy = 0.0f; o->gz = 0.0f;	/* no gyro on this board */
		return true;
	}
	if (!reg_read(MPU_ACCEL_XOUT_H, b, 14u)) {
		return false;
	}
	for (i = 0; i < 7; i++) {
		v[i] = (int16_t)(((uint16_t)b[2 * i] << 8) | b[(2 * i) + 1]);
	}
	o->ax = (float)v[0] / MPU_ACC_LSB_PER_G;
	o->ay = (float)v[1] / MPU_ACC_LSB_PER_G;
	o->az = (float)v[2] / MPU_ACC_LSB_PER_G;
	/* v[3] = temperature, not used */
	o->gx = (float)v[4] / MPU_GYR_LSB_PER_DPS;
	o->gy = (float)v[5] / MPU_GYR_LSB_PER_DPS;
	o->gz = (float)v[6] / MPU_GYR_LSB_PER_DPS;
	return true;
}

uint8_t  mpu_who_am_i(void) { return who; }
bool     mpu_has_gyro(void) { return chip != CHIP_LSM303; }
bool     mpu_has_mag(void)  { return mag != MAG_NONE; }

/* Raw compass reading (any unit: only the direction is used). Robot axes as
   the accelerometer: x, y in the board plane, z up. */
bool mpu_mag_read(float *mx, float *my, float *mz)
{
	uint8_t b[6];
	int16_t x, y, z;
	bool    ok = true;
	uint32_t i;

	if (mag == MAG_NONE) { return false; }
	set_target(MAG_ADDR);
	if (mag == MAG_DLHC) {
		ok = reg_read(DLHC_OUT_X_H_M, b, 6u);		/* auto-increment */
		x = (int16_t)(((uint16_t)b[0] << 8) | b[1]);
		z = (int16_t)(((uint16_t)b[2] << 8) | b[3]);
		y = (int16_t)(((uint16_t)b[4] << 8) | b[5]);
	} else {
		for (i = 0; (i < 6u) && ok; i++) {		/* one register at a time: safe on every AGR */
			ok = reg_read((uint8_t)(AGR_OUTX_L_REG_M + i), &b[i], 1u);
		}
		x = (int16_t)(((uint16_t)b[1] << 8) | b[0]);
		y = (int16_t)(((uint16_t)b[3] << 8) | b[2]);
		z = (int16_t)(((uint16_t)b[5] << 8) | b[4]);
	}
	set_target(chip_addr);
	if (!ok) { return false; }
	*mx = (float)x; *my = (float)y; *mz = (float)z;
	return true;
}
const char *mpu_chip_name(void)
{
	if (chip == CHIP_LSM303) {
		return (mag == MAG_DLHC) ? "LSM303DLHC (GY-511, no gyro, compass ok)" :
		       (mag == MAG_AGR)  ? "LSM303AGR (GY-511, no gyro, compass ok)" :
					   "LSM303 (GY-511, no gyro, no compass)";
	}
	return (chip == CHIP_MPU6050) ? "MPU-6050 (GY-521)" : "none";
}
uint8_t  mpu_addr(void) { return chip_addr; }
uint32_t mpu_errors(void)   { return errors; }
