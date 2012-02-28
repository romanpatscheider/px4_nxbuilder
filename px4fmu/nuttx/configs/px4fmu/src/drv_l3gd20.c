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

#define ADDR_WHO_AM_I			0x0f
#define WHO_I_AM				0xd4

#define DIR_READ		(1<<7)
#define DIR_WRITE		(0<<7)
#define ADDR_INCREMENT		(1<<6)

static struct l3gd20_dev_s	dev;

static ssize_t	l3gd20_read(struct file *filp, FAR char *buffer, size_t buflen);
static int	l3gd20_ioctl(struct file *filp, int cmd, unsigned long arg);

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

static ssize_t
l3gd20_read(struct file *filp, char *buffer, size_t buflen)
{
//	/* if the buffer is large enough, and data are available, return success */
//	if (buflen >= 12) {
//		if (read_fifo((uint16_t *)buffer))
//			return 12;
//
//		/* no data */
//		return 0;
//	}

	/* buffer too small */
	errno = ENOSPC;
	return ERROR;
}

static int
l3gd20_ioctl(struct file *filp, int cmd, unsigned long arg)
{
	int	result = ERROR;

//	switch (cmd) {
//        case BMA180_SETRATE:
//            if ((arg & REG1_RATE_MASK) == arg) {
//                set_rate(arg);
//                result = 0;
//                dev.rate = arg;
//            }
//            break;
//
//        case BMA180_SETRANGE:
//            if ((arg & REG4_RANGE_MASK) == arg) {
//                set_range(arg);
//                result = 0;
//            }
//            break;
//
//        case BMA180_SETBUFFER:
//            dev.buffer = (struct bma180_buffer *)arg;
//            result = 0;
//            break;
//	}

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

		/* set default configuration */
		//write_reg(ADDR_CTRL_REG2, 0);	/* disable interrupt-generating high-pass filters */
		//write_reg(ADDR_CTRL_REG3, 0);	/* no interrupts - we don't use them */
		//write_reg(ADDR_CTRL_REG5, 0);	/* disable wake-on-interrupt */

		//set_range(LIS331_RANGE_4G);
		//set_rate(LIS331_RATE_400Hz);	/* takes device out of low-power mode */

		/* make ourselves available */
		register_driver("/dev/l3gd20", &l3gd20_fops, 0666, NULL);


		result = 0;
	} else {
		errno = EIO;
	}

	SPI_LOCK(dev.spi, false);

	return result;
}
