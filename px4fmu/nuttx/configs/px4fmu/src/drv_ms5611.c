/*
 *   Copyright (C) 2012 Michael Smith. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name of the author or the names of contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

/*
 * Driver for the ST ms5611 MEMS gyroscope
 */

#include <nuttx/config.h>

#include <stdint.h>
#include <stdbool.h>
#include <debug.h>
#include <errno.h>

#include <nuttx/i2c.h>
#include <arch/board/board.h>

#include "up_arch.h"
#include "chip.h"
#include "stm32_internal.h"
#include "px4fmu-internal.h"

#include <arch/board/drv_ms5611.h>

/* internal conversion time: 9.17 ms, so should not be read at rates higher than 100 Hz */

#define MS5611_ADDRESS_1	0x76
#define MS5611_ADDRESS_2	0x77

#define ADDR_RESET_CMD			0x1E /* read from this address to reset chip (0b0011110 on bus) */
#define ADDR_CMD_CONVERT_D1		0x48 /* 4096 samples, xxx to this address to start conversion (0b01001000 on bus) */
#define ADDR_CMD_CONVERT_D2		0x58 /* 4096 samples */
#define ADDR_DATA				0x00 /* address of 4 bytes pressure data */
#define ADDR_PROM				0xA0 /* address of 16 bytes calibration data */

static FAR struct ms5611_dev_s	dev;

static ssize_t ms5611_read(struct file *filp, FAR char *buffer, size_t buflen);
static int ms5611_ioctl(struct file *filp, int cmd, unsigned long arg);

static const struct file_operations ms5611_fops = {
	.read  = ms5611_read,
	.ioctl = ms5611_ioctl,
};

struct ms5611_dev_s
{
	uint16_t			prom[8];
	struct i2c_dev_s	*i2c;
	struct ms5611_buffer	*buffer;
} __attribute__((packed));

static uint8_t MS5611_ADDRESS;

static int write_reg(uint8_t address, uint8_t data);
static int read_reg(uint8_t address);

static int
write_reg(uint8_t address, uint8_t data)
{
	return I2C_WRITE(dev.i2c, &data, 1);
}

static int
read_reg(uint8_t address)
{
	uint8_t cmd = address;
	uint8_t data;

	int ret = I2C_WRITEREAD(dev.i2c, &cmd, 1, &data, 1);
	/* return data on success, error code on failure */
	if (ret == 0) ret = data;
	return ret;
}

static int
set_range(uint8_t range)
{
	I2C_SETADDRESS(dev.i2c, MS5611_ADDRESS, 7);
//	/* mask out illegal bit positions */
//	uint8_t write_range = range & REG4_RANGE_MASK;
//	/* immediately return if user supplied invalid value */
//	if (write_range != range) return EINVAL;
//	/* set remaining bits to a sane value */
//	write_range |= REG4_BDU;
//	/* write to device */
//	write_reg(ADDR_CTRL_REG4, write_range);
//	/* return 0 if register value is now written value, 1 if unchanged */
//	return !(read_reg(ADDR_CTRL_REG4) == write_range);
	return 0;
}

static int
set_rate(uint8_t rate)
{
	I2C_SETADDRESS(dev.i2c, MS5611_ADDRESS, 7);
//	/* mask out illegal bit positions */
//	uint8_t write_rate = rate & REG1_RATE_LP_MASK;
//	/* immediately return if user supplied invalid value */
//	if (write_rate != rate) return EINVAL;
//	/* set remaining bits to a sane value */
//	write_rate |= REG1_POWER_NORMAL | REG1_Z_ENABLE | REG1_Y_ENABLE | REG1_X_ENABLE;
//	/* write to device */
//	write_reg(ADDR_CTRL_REG1, write_rate);
//	/* return 0 if register value is now written value, 1 if unchanged */
//	return !(read_reg(ADDR_CTRL_REG1) == write_rate);
	return 0;
}

static bool
read_values(int16_t *data)
{
	struct { /* status register and data as read back from the device */
		uint32_t		pressure;
		uint32_t		temperature;
	} __attribute__((packed))	report;

	/* exchange the report structure with the device */
//	I2C_LOCK(dev.i2c, true);
	I2C_SETADDRESS(dev.i2c, MS5611_ADDRESS_2, 7);

	uint8_t cmd = ADDR_DATA;
	int ret = 1;

	cmd = ADDR_DATA;
	ret = I2C_WRITEREAD(dev.i2c, &cmd, 1, (uint8_t*)&(report), sizeof(report));

	/* start conversion for next update */

//	I2C_LOCK(dev.i2c, false);

	/* write values and clamp them to 12 bit */
	data[0] = report.pressure;
	data[1] = report.temperature;

	/* return 1 if new data is available, 0 else */
	/* XXX Check if last read was at least 9.5 ms ago */
	return ret;
}

static ssize_t
ms5611_read(struct file *filp, char *buffer, size_t buflen)
{
	/* if the buffer is large enough, and data are available, return success */
	if (buflen >= 6) {
		if (read_values((int16_t *)buffer))
			return 6;

		/* no data */
		return 0;
	}

	/* buffer too small */
	errno = ENOSPC;
	return ERROR;
}

static int
ms5611_ioctl(struct file *filp, int cmd, unsigned long arg)
{
	int	result = ERROR;

//	switch (cmd) {
//        case MS5611_SETRATE:
//            if ((arg & REG1_RATE_LP_MASK) == arg) {
//                set_rate(arg);
//                result = 0;
//                dev.rate = arg;
//            }
//            break;
//
//        case MS5611_SETRANGE:
//            if ((arg & REG4_RANGE_MASK) == arg) {
//                set_range(arg);
//                result = 0;
//            }
//            break;
//
//        case MS5611_SETBUFFER:
//            dev.buffer = (struct ms5611_buffer *)arg;
//            result = 0;
//            break;
//	}

	if (result)
		errno = EINVAL;
	return result;
}

int
ms5611_attach(struct i2c_dev_s *i2c)
{
	int	result = ERROR;

	dev.i2c = i2c;

	MS5611_ADDRESS = MS5611_ADDRESS_1;
	uint8_t cmd = ADDR_PROM;

	/* reset */

//	I2C_LOCK(dev.i2c, true);
	I2C_SETADDRESS(dev.i2c, MS5611_ADDRESS, 7);
	int ret = read_reg(ADDR_RESET_CMD);

	/* check if the address was wrong */
	if (ret < 0)
	{
		/* try second address */
		MS5611_ADDRESS = MS5611_ADDRESS_2;
		I2C_SETADDRESS(dev.i2c, MS5611_ADDRESS, 7);
		ret = read_reg(ADDR_RESET_CMD);
	}

	if (ret < 0) return EIO;

	/* wait for PROM contents to be in the device */
	usleep(10000);

	/* read PROM */
	ret = I2C_WRITEREAD(i2c, &cmd, 1, (uint8_t*)dev.prom, sizeof(dev.prom));

	/* OR PROM contents as poor-man's alive check, PROM cannot be all-zero */
	int sum = dev.prom[0] | dev.prom[1] | dev.prom[2] | dev.prom[3] | dev.prom[4] | dev.prom[5] | dev.prom[6] | dev.prom[7];

	/* verify that the device is attached and functioning */
	if ((ret >= 0) && (sum > 0)) {

		/* start first conversion */
		read_reg(ADDR_CMD_CONVERT_D1);
		/* wait */
		usleep(10000);
		read_reg(ADDR_CMD_CONVERT_D2);

		/* make ourselves available */
		register_driver("/dev/ms5611", &ms5611_fops, 0666, NULL);

		result = 0;

	} else {
		errno = EIO;
	}



	return result;
}
