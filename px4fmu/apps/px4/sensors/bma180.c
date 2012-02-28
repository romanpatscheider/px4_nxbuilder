/*
 * Operations for the Bosch BMA180 3D Accelerometer
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

#define ADDR_CHIP_ID			0x00
#define CHIP_ID					0x03


static void
write_reg(struct spi_dev_s *spi, uint8_t address, uint8_t data)
{
	uint8_t cmd[2] = { address | DIR_WRITE, data };

	SPI_SELECT(spi, PX4_SPIDEV_ACCEL, true);
      	SPI_SNDBLOCK(spi, &cmd, sizeof(cmd));
	SPI_SELECT(spi, PX4_SPIDEV_ACCEL, false);
}

static uint8_t
read_reg(struct spi_dev_s *spi, uint8_t address)
{
	uint8_t	cmd[2] = {address | DIR_READ, 0};
	uint8_t data[2];

	SPI_SELECT(spi, PX4_SPIDEV_ACCEL, true);
	SPI_EXCHANGE(spi, cmd, data, sizeof(cmd));
	SPI_SELECT(spi, PX4_SPIDEV_ACCEL, false);

	return data[1];
}

int
bma180_test(struct spi_dev_s *spi)
{
	uint8_t	id;

	id = read_reg(spi, ADDR_CHIP_ID);

	if (id == CHIP_ID)
	{
		message("BMA180 SUCCESS: 0x%02x\n", id);
	}
	else
	{
		message("BMA180 FAIL: 0x%02x\n", id);
	}
	//message("got id 0x%02x, expected ID 0x03\n", id);

	return 0;
}
