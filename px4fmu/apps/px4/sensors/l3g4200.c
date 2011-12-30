/*
 * Operations for the l3g4200
 */

#include <nuttx/config.h>

#include <sys/types.h>

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <debug.h>

#include <arch/board/board.h>

#include <nuttx/spi.h>

#include "sensors.h"

#define DIR_READ			(1<<7)
#define DIR_WRITE			(0<<7)
#define ADDR_INCREMENT			(1<<6)

#define ADDR_WHO_AM_I			0x0f
#define WHO_I_AM				0xd3

#define ADDR_CTRL_REG1			0x20		/* sample rate constants are in the public header */
#define REG1_POWER_NORMAL			(1<<3)
#define REG1_Z_ENABLE				(1<<2)
#define REG1_Y_ENABLE				(1<<1)
#define REG1_X_ENABLE				(1<<0)

#define ADDR_CTRL_REG2			0x21
/* high-pass filter - usefulness TBD */

#define ADDR_CTRL_REG3			0x22

#define ADDR_CTRL_REG4			0x23
#define REG4_BDU				(1<<7)
#define REG4_BIG_ENDIAN				(1<<6)
#define REG4_SPI_3WIRE				(1<<0)

#define ADDR_CTRL_REG5			0x24
#define REG5_BOOT				(1<<7)
#define REG5_FIFO_EN				(1<<6)
#define REG5_HIGHPASS_ENABLE			(1<<4)

#define ADDR_REFERENCE			0x25
#define ADDR_TEMPERATURE		0x26

#define ADDR_STATUS_REG			0x27
#define STATUS_ZYXOR				(1<<7)
#define SATAUS_ZOR				(1<<6)
#define STATUS_YOR				(1<<5)
#define STATUS_XOR				(1<<4)
#define STATUS_ZYXDA				(1<<3)
#define STATUS_ZDA				(1<<2)
#define STATUS_YDA				(1<<1)
#define STATUS_XDA				(1<<0)

#define ADDR_OUT_X			0x28	/* 16 bits */
#define ADDR_OUT_Y			0x2A	/* 16 bits */
#define ADDR_OUT_Z			0x2C	/* 16 bits */

#define ADDR_FIFO_CTRL			0x2e
#define FIFO_MODE_BYPASS			(0<<5)
#define FIFO_MODE_FIFO				(1<<5)
#define FIFO_MODE_STREAM			(2<<5)
#define FIFO_MODE_STREAM_TO_FIFO		(3<<5)
#define FIFO_MODE_BYPASS_TO_STREAM		(4<<5)
#define FIFO_THRESHOLD_MASK			0x1f

#define ADDR_FIFO_SRC			0x2f
#define FIFO_THREHSHOLD_OVER			(1<<7)
#define FIFO_OVERRUN				(1<<6)
#define FIFO_EMPTY				(1<<5)

#define L3G4200_RATE_100Hz		((0<<6) | (0<<4))
#define L3G4200_RATE_200Hz		((1<<6) | (0<<4))
#define L3G4200_RATE_400Hz		((2<<6) | (1<<4))
#define L3G4200_RATE_800Hz		((3<<6) | (2<<4))

#define L3G4200_RANGE_250dps		(0<<4)
#define L3G4200_RANGE_500dps		(1<<4)
#define L3G4200_RANGE_2000dps		(3<<4)

static void
write_reg(struct spi_dev_s *spi, uint8_t address, uint8_t data)
{
	uint8_t cmd[2] = { address | DIR_WRITE, data };

	SPI_SELECT(spi, PX4_SPIDEV_GYRO, true);
      	SPI_SNDBLOCK(spi, &cmd, sizeof(cmd));
	SPI_SELECT(spi, PX4_SPIDEV_GYRO, false);
}

static uint8_t
read_reg(struct spi_dev_s *spi, uint8_t address)
{
	uint8_t	cmd[2] = {address | DIR_READ, 0};
	uint8_t data[2];

	SPI_SELECT(spi, PX4_SPIDEV_GYRO, true);
	SPI_EXCHANGE(spi, cmd, data, sizeof(cmd));
	SPI_SELECT(spi, PX4_SPIDEV_GYRO, false);

	return data[1];
}

static bool
read_fifo(struct spi_dev_s *spi)
{
	struct {					/* status register and data as read back from the device */
		uint8_t		cmd;
		uint8_t		status;
		int16_t		x;
		int16_t		y;
		int16_t		z;
	} __attribute__((packed))	report;

	report.cmd = ADDR_STATUS_REG | DIR_READ | ADDR_INCREMENT;

	/* exchange the report structure with the device */
	SPI_SELECT(spi, PX4_SPIDEV_GYRO, true);
	SPI_EXCHANGE(spi, &report, &report, sizeof(report));
	SPI_SELECT(spi, PX4_SPIDEV_GYRO, false);

#if 0
	data->x = report.x;
	data->y = report.y;
	data->z = report.z;
#endif

	return report.status & STATUS_ZYXDA;
}

int
l3g4200_test(struct spi_dev_s *spi)
{
	uint8_t	id;

	id = read_reg(spi, ADDR_WHO_AM_I);

	message("got id 0x%02x\n", id);

	return 0;
}
