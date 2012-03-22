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
 * Driver for the ST L3GD20 MEMS gyroscope
 */

#include <nuttx/config.h>

#include <stdint.h>
#include <stdbool.h>
#include <debug.h>
#include <errno.h>

#include <nuttx/spi.h>
#include <arch/board/board.h>

#include "up_arch.h"
#include "chip.h"
#include "stm32_internal.h"
#include "px4fmu-internal.h"

#include <arch/board/drv_l3gd20.h>

#define DIR_READ					(1<<7)
#define DIR_WRITE					(0<<7)
#define ADDR_INCREMENT				(1<<6)

#define ADDR_WHO_AM_I				0x0F
#define WHO_I_AM					0xD4
#define ADDR_CTRL_REG1				0x20
#define ADDR_CTRL_REG2				0x21
#define ADDR_CTRL_REG3				0x22
#define ADDR_CTRL_REG4				0x23
#define ADDR_CTRL_REG5				0x24
#define ADDR_REFERENCE				0x25
#define ADDR_OUT_TEMP				0x26
#define ADDR_STATUS_REG				0x27
#define ADDR_OUT_X_L				0x28
#define ADDR_OUT_X_H				0x29
#define ADDR_OUT_Y_L				0x2A
#define ADDR_OUT_Y_H				0x2B
#define ADDR_OUT_Z_L				0x2C
#define ADDR_OUT_Z_H				0x2D
#define ADDR_FIFO_CTRL_REG			0x2E
#define ADDR_FIFO_SRC_REG			0x2F
#define ADDR_INT1_CFG				0x30
#define ADDR_INT1_SRC				0x31
#define ADDR_INT1_TSH_XH			0x32
#define ADDR_INT1_TSH_XL			0x33
#define ADDR_INT1_TSH_YH			0x34
#define ADDR_INT1_TSH_YL			0x35
#define ADDR_INT1_TSH_ZH			0x36
#define ADDR_INT1_TSH_ZL			0x37
#define ADDR_INT1_DURATION			0x38

#define REG1_RATE_LP_MASK			0xF0 /* Mask to guard partial register update */
#define REG4_RANGE_MASK				0x30 /* Mask to guard partial register update */

/* Internal configuration values */
#define REG1_POWER_NORMAL			(1<<3)
#define REG1_Z_ENABLE				(1<<2)
#define REG1_Y_ENABLE				(1<<1)
#define REG1_X_ENABLE				(1<<0)

#define REG4_BDU					(1<<7)
#define REG4_BLE					(1<<6)
//#define REG4_SPI_3WIRE				(1<<0)

#define REG5_FIFO_ENABLE			(1<<6)
#define REG5_REBOOT_MEMORY			(1<<7)

#define STATUS_ZYXOR				(1<<7)
#define STATUS_ZOR					(1<<6)
#define STATUS_YOR					(1<<5)
#define STATUS_XOR					(1<<4)
#define STATUS_ZYXDA				(1<<3)
#define STATUS_ZDA					(1<<2)
#define STATUS_YDA					(1<<1)
#define STATUS_XDA					(1<<0)

#define FIFO_CTRL_BYPASS_MODE				(0<<5)
#define FIFO_CTRL_FIFO_MODE					(1<<5)
#define FIFO_CTRL_STREAM_MODE				(1<<6)
#define FIFO_CTRL_STREAM_TO_FIFO_MODE		(3<<5)
#define FIFO_CTRL_BYPASS_TO_STREAM_MODE		(1<<7)

FAR struct l3gd20_dev_s	dev;

static ssize_t l3gd20_read(struct file *filp, FAR char *buffer, size_t buflen);
static int l3gd20_ioctl(struct file *filp, int cmd, unsigned long arg);

static const struct file_operations l3gd20_fops = {
	.read  = l3gd20_read,
	.ioctl = l3gd20_ioctl,
};

struct l3gd20_dev_s
{
	struct spi_dev_s	*spi;
	int			spi_id;
	uint8_t			rate;
	struct l3gd20_buffer	*buffer;
};

static void	write_reg(uint8_t address, uint8_t data);
static uint8_t	read_reg(uint8_t address);

static void
write_reg(uint8_t address, uint8_t data)
{
	uint8_t cmd[2] = { address | DIR_WRITE, data };

	SPI_SELECT(dev.spi, dev.spi_id, true);
    SPI_SNDBLOCK(dev.spi, &cmd, sizeof(cmd));
	SPI_SELECT(dev.spi, dev.spi_id, false);
}

static uint8_t
read_reg(uint8_t address)
{
	uint8_t	cmd[2] = {address | DIR_READ, 0};
	uint8_t data[2];

	SPI_SELECT(dev.spi, dev.spi_id, true);
	SPI_EXCHANGE(dev.spi, cmd, data, sizeof(cmd));
	SPI_SELECT(dev.spi, dev.spi_id, false);

	return data[1];
}

static int
set_range(uint8_t range)
{
	/* mask out illegal bit positions */
	uint8_t write_range = range & REG4_RANGE_MASK;
	/* immediately return if user supplied invalid value */
	if (write_range != range) return EINVAL;
	/* set remaining bits to a sane value */
	write_range |= REG4_BDU;
	/* write to device */
	write_reg(ADDR_CTRL_REG4, write_range);
	/* return 0 if register value is now written value, 1 if unchanged */
	return !(read_reg(ADDR_CTRL_REG4) == write_range);
}

static int
set_rate(uint8_t rate)
{
	/* mask out illegal bit positions */
	uint8_t write_rate = rate & REG1_RATE_LP_MASK;
	/* immediately return if user supplied invalid value */
	if (write_rate != rate) return EINVAL;
	/* set remaining bits to a sane value */
	write_rate |= REG1_POWER_NORMAL | REG1_Z_ENABLE | REG1_Y_ENABLE | REG1_X_ENABLE;
	/* write to device */
	write_reg(ADDR_CTRL_REG1, write_rate);
	/* return 0 if register value is now written value, 1 if unchanged */
	return !(read_reg(ADDR_CTRL_REG1) == write_rate);
}

static bool
read_fifo(int16_t *data)
{
	struct {					/* status register and data as read back from the device */
		uint8_t		cmd;
		uint8_t		temp;
		uint8_t		status;
		int16_t		x;
		int16_t		y;
		int16_t		z;
	} __attribute__((packed))	report;

	report.cmd = ADDR_OUT_TEMP | DIR_READ | ADDR_INCREMENT;

	/* exchange the report structure with the device */
	SPI_LOCK(dev.spi, true);

	SPI_SELECT(dev.spi, dev.spi_id, true);

	read_reg(ADDR_WHO_AM_I);


	SPI_EXCHANGE(dev.spi, &report, &report, sizeof(report));
	SPI_SELECT(dev.spi, dev.spi_id, false);

	SPI_LOCK(dev.spi, false);

	data[0] = report.x;
	data[1] = report.y;
	data[2] = report.z;

	return (report.status & STATUS_ZYXDA);
}

static ssize_t
l3gd20_read(struct file *filp, char *buffer, size_t buflen)
{
	/* if the buffer is large enough, and data are available, return success */
	if (buflen >= 6) {
		if (read_fifo((int16_t *)buffer))
			return 6;

		/* no data */
		return 0;
	}

	/* buffer too small */
	errno = ENOSPC;
	return ERROR;
}

static int
l3gd20_ioctl(struct file *filp, int cmd, unsigned long arg)
{
	int	result = ERROR;

	switch (cmd) {
        case L3GD20_SETRATE:
            if ((arg & REG1_RATE_LP_MASK) == arg) {
                set_rate(arg);
                result = 0;
                dev.rate = arg;
            }
            break;

        case L3GD20_SETRANGE:
            if ((arg & REG4_RANGE_MASK) == arg) {
                set_range(arg);
                result = 0;
            }
            break;

        case L3GD20_SETBUFFER:
            dev.buffer = (struct l3gd20_buffer *)arg;
            result = 0;
            break;
	}

	if (result)
		errno = EINVAL;
	return result;
}

int
l3gd20_attach(struct spi_dev_s *spi, int spi_id)
{
	int	result = ERROR;

	dev.spi = spi;
	dev.spi_id = spi_id;

	SPI_LOCK(dev.spi, true);

	/* verify that the device is attached and functioning */
	if (read_reg(ADDR_WHO_AM_I) == WHO_I_AM) {

		/* reset device memory */
		write_reg(ADDR_CTRL_REG5, REG5_REBOOT_MEMORY);

		/* set default configuration */
		write_reg(ADDR_CTRL_REG2, 0);			/* disable high-pass filters */
		write_reg(ADDR_CTRL_REG3, 0);			/* no interrupts - we don't use them */
		write_reg(ADDR_CTRL_REG5, 0);

		//		write_reg(ADDR_CTRL_REG5, 0 | REG5_FIFO_ENABLE);	  /* disable wake-on-interrupt */
//		write_reg(ADDR_FIFO_CTRL_REG, FIFO_CTRL_STREAM_MODE); /* Enable FIFO, old data is overwritten */

		if ((set_range(L3GD20_RANGE_500DPS) != 0) ||
				(set_rate(L3GD20_RATE_760HZ) != 0))	/* takes device out of low-power mode */
		{
			SPI_LOCK(dev.spi, false);
			errno = EIO;
		} else {
			SPI_LOCK(dev.spi, false);
			/* Read out the first few funky values */
			int16_t dummy[3];
			read_fifo(dummy);
			read_fifo(dummy);
			read_fifo(dummy);

			/* make ourselves available */
			register_driver("/dev/l3gd20", &l3gd20_fops, 0666, NULL);

			result = 0;
		}

	} else {
		SPI_LOCK(dev.spi, false);
		errno = EIO;
	}



	return result;
}
