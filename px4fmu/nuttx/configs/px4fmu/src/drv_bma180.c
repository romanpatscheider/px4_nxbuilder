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
 * Driver for the Bosch BMA 180 MEMS accelerometer
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

#include <arch/board/drv_bma180.h>

/*
 * BMA180 registers
 */

/* Important Notes:
 *
 *   - MAX SPI clock: 25 MHz
 *   - Readout time: 0.417 ms in high accuracy mode
 *   - Boot / ready time: 1.27 ms
 *
 */

#define DIR_READ				(1<<7)
#define DIR_WRITE				(0<<7)
#define ADDR_INCREMENT			(1<<6)

#define ADDR_CHIP_ID			0x00
#define CHIP_ID					0x03
#define ADDR_VERSION			0x01

#define ADDR_CTRL_REG0			0x0D
#define ADDR_CTRL_REG1			0x0E
#define ADDR_CTRL_REG2			0x0F
#define ADDR_BWTCS				0x20
#define ADDR_CTRL_REG3			0x21
#define ADDR_CTRL_REG4			0x22
#define ADDR_OLSB1				0x35

#define ADDR_ACC_X_LSB			0x02
#define ADDR_ACC_Z_MSB			0x07
#define ADDR_TEMPERATURE		0x08

#define ADDR_STATUS_REG1		0x09
#define ADDR_STATUS_REG2		0x0A
#define ADDR_STATUS_REG3		0x0B
#define ADDR_STATUS_REG4		0x0C

#define ADDR_RESET				0x10
#define SOFT_RESET				0xB6

#define ADDR_DIS_I2C         	0x27

#define REG0_WRITE_ENABLE		0x10

#define BWTCS_LP_10HZ			(0<<4)
#define BWTCS_LP_20HZ			(1<<4)
#define BWTCS_LP_40HZ			(2<<4)
#define BWTCS_LP_75HZ			(3<<4)
#define BWTCS_LP_150HZ			(4<<4)
#define BWTCS_LP_300HZ			(5<<4)
#define BWTCS_LP_600HZ			(6<<4)
#define BWTCS_LP_1200HZ			(7<<4)

#define RANGE_1G				(0<<1)
#define RANGE_1_5G				(1<<1)
#define RANGE_2G				(2<<1)
#define RANGE_3G				(3<<1)
#define RANGE_4G				(4<<1)
#define RANGE_8G				(5<<1)
#define RANGE_16G				(6<<1)

#define RANGEMASK 0x0E
#define BWMASK 0xF0


static ssize_t	bma180_read(struct file *filp, FAR char *buffer, size_t buflen);
static int	bma180_ioctl(struct file *filp, int cmd, unsigned long arg);

static const struct file_operations bma180_fops = {
	.read  = bma180_read,
	.ioctl = bma180_ioctl,
};

struct bma180_dev_s
{
	struct spi_dev_s	*spi;
	int			spi_id;
    
	uint8_t			rate;
	struct bma180_buffer	*buffer;
};

static struct bma180_dev_s	bma180_dev;

static void	write_reg(uint8_t address, uint8_t data);
static uint8_t	read_reg(uint8_t address);
static bool	read_fifo(uint16_t *data);
static void	set_range(uint8_t range);
static void	set_rate(uint8_t rate);

static void
write_reg(uint8_t address, uint8_t data)
{
	uint8_t cmd[2] = { address | DIR_WRITE, data };
    
	SPI_SELECT(bma180_dev.spi, bma180_dev.spi_id, true);
    SPI_SNDBLOCK(bma180_dev.spi, &cmd, sizeof(cmd));
	SPI_SELECT(bma180_dev.spi, bma180_dev.spi_id, false);
}

static uint8_t
read_reg(uint8_t address)
{
	uint8_t	cmd[2] = {address | DIR_READ, 0};
	uint8_t data[2];
    
	SPI_SELECT(bma180_dev.spi, bma180_dev.spi_id, true);
	SPI_EXCHANGE(bma180_dev.spi, cmd, data, sizeof(cmd));
	SPI_SELECT(bma180_dev.spi, bma180_dev.spi_id, false);
    
	return data[1];	
}

static bool
read_fifo(uint16_t *data)
{
	struct {					/* status register and data as read back from the device */
		uint8_t		cmd;
		int16_t		x;
		int16_t		y;
		int16_t		z;
		uint8_t		temp;
	} __attribute__((packed))	report;

	report.cmd = ADDR_ACC_X_LSB | DIR_READ | ADDR_INCREMENT;

	/* exchange the report structure with the device */
//	SPI_LOCK(bma180_dev.spi, true);
//	SPI_SELECT(bma180_dev.spi, bma180_dev.spi_id, true);
//	SPI_EXCHANGE(bma180_dev.spi, &report, &report, sizeof(report));
//	SPI_SELECT(bma180_dev.spi, bma180_dev.spi_id, false);
//	SPI_LOCK(bma180_dev.spi, false);

	SPI_LOCK(bma180_dev.spi, true);
	report.x = read_reg(ADDR_ACC_X_LSB);
	report.x |= (read_reg(ADDR_ACC_X_LSB+1) << 8);
	report.y = read_reg(ADDR_ACC_X_LSB+2);
	report.y |= (read_reg(ADDR_ACC_X_LSB+3) << 8);
	report.z = read_reg(ADDR_ACC_X_LSB+4);
	report.z |= (read_reg(ADDR_ACC_X_LSB+5) << 8);
	report.temp = read_reg(ADDR_ACC_X_LSB+6);
	SPI_LOCK(bma180_dev.spi, false);

	// Collect status and remove two top bits

	uint8_t new_data = (report.x & 0x01) + (report.x & 0x01) + (report.x & 0x01);
	report.x = (report.x >> 2);
	report.y = (report.y >> 2);
	report.z = (report.z >> 2);

	data[0] = report.x;
	data[1] = report.y;
	data[2] = report.z;
    
	/* return 1 for all three axes new */
	return (new_data > 0); // bit funky, depends on timing
}

static void
set_range(uint8_t range)
{
//	range &= REG4_RANGE_MASK;
//	write_reg(ADDR_CTRL_REG4, range | REG4_BDU);
}

static void
set_rate(uint8_t rate)
{
//	rate &= REG1_RATE_MASK;
//	write_reg(ADDR_CTRL_REG1, rate | REG1_POWER_NORMAL | REG1_Z_ENABLE | REG1_Y_ENABLE | REG1_X_ENABLE);
}

static ssize_t
bma180_read(struct file *filp, char *buffer, size_t buflen)
{
	/* if the buffer is large enough, and data are available, return success */
	if (buflen >= 6) {
		if (read_fifo((uint16_t *)buffer))
			return 6;

		/* no data */
		return 0;
	}
    
	/* buffer too small */
	errno = ENOSPC;
	return ERROR;
}

static int
bma180_ioctl(struct file *filp, int cmd, unsigned long arg)
{
	int	result = ERROR;
    
	switch (cmd) {
        case BMA180_SETRATE:
//            if ((arg & REG1_RATE_MASK) == arg) {
//                set_rate(arg);
//                result = 0;
//                bma180_dev.rate = arg;
//            }
            break;

        case BMA180_SETRANGE:
//            if ((arg & REG4_RANGE_MASK) == arg) {
//                set_range(arg);
//                result = 0;
//            }
            break;

        case BMA180_SETBUFFER:
            bma180_dev.buffer = (struct bma180_buffer *)arg;
            result = 0;
            break;
	}
    
	if (result)
		errno = EINVAL;
	return result;
}

int
bma180_attach(struct spi_dev_s *spi, int spi_id)
{
	int	result = ERROR;
	
	bma180_dev.spi = spi;
	bma180_dev.spi_id = spi_id;
    
	SPI_LOCK(bma180_dev.spi, true);

	/* verify that the device is attached and functioning */
	if (read_reg(ADDR_CHIP_ID) == CHIP_ID) {
        
//		write_reg(ADDR_RESET, SOFT_RESET);             // page 48
//		usleep(12000);                        // wait 12 ms, see page 49

		// Configuring the BMA180

		/* enable writing to chip config */
		uint8_t ctrl0 = read_reg(ADDR_CTRL_REG0);
		ctrl0 |= REG0_WRITE_ENABLE;
		write_reg(ADDR_CTRL_REG0, ctrl0);

		/* disable I2C interface, datasheet page 31 */
		uint8_t disi2c = read_reg(ADDR_DIS_I2C);
		disi2c |= 0x01;
		write_reg(ADDR_DIS_I2C, disi2c);

		/* set bandwidth */
		uint8_t bwtcs = read_reg(ADDR_BWTCS);
		bwtcs &= (~BWMASK);
		bwtcs |= (BWTCS_LP_600HZ);// & BWMASK);
		write_reg(ADDR_BWTCS, bwtcs);

		/* set range */
		uint8_t olsb1 = read_reg(ADDR_OLSB1);
		olsb1 &= (~RANGEMASK);
		olsb1 |= (RANGE_4G);// & RANGEMASK);
		write_reg(ADDR_OLSB1, olsb1);

		/* block writing to chip config */
		ctrl0 = read_reg(ADDR_CTRL_REG0);
		ctrl0 &= (~REG0_WRITE_ENABLE);
		write_reg(ADDR_CTRL_REG0, ctrl0);
        
		/* make ourselves available */
		register_driver("/dev/bma180", &bma180_fops, 0666, NULL);
        
		result = 0;
	} else {
		errno = EIO;
	}

	SPI_LOCK(bma180_dev.spi, false);
    
	return result;
}

