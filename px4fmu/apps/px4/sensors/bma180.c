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

	struct {					/* status register and data as read back from the device */
		uint8_t		cmd;
		int16_t		x;
		int16_t		y;
		int16_t		z;
		uint8_t		temp;
		uint8_t		status1;
		uint8_t		status2;
		uint8_t		status3;
		uint8_t		status4;
	} __attribute__((packed))	report;

	report.cmd = 0x02 | DIR_READ | ADDR_INCREMENT;

//	write_reg(spi, ADDR_CTRL_REG2, 0);			/* disable high-pass filters */
//	write_reg(spi, ADDR_CTRL_REG3, 0);			/* no interrupts - we don't use them */
//	write_reg(spi, ADDR_CTRL_REG5, 0);			/* turn off FIFO mode */
//
//	write_reg(spi, ADDR_CTRL_REG4, ((3<<4) & 0x30) | REG4_BDU);
//
//
//	write_reg(spi, ADDR_CTRL_REG1,
//			(((2<<6) | (1<<4)) & 0xf0) | REG1_POWER_NORMAL | REG1_Z_ENABLE | REG1_Y_ENABLE | REG1_X_ENABLE);

	SPI_SELECT(spi, PX4_SPIDEV_ACCEL, true);
	SPI_EXCHANGE(spi, &report, &report, sizeof(report));
	SPI_SELECT(spi, PX4_SPIDEV_ACCEL, false);

	message("ACC: x: %d\ty: %d\tz: %d\n", report.x, report.y, report.z);
	usleep(1000);

	return 0;
}
