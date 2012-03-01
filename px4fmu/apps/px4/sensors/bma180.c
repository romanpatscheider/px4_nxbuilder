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

#define REG0_WRITE_ENABLE		0x10

#define BWTCS_LP_10HZ			(0<<4)
#define BWTCS_LP_20HZ			(1<<4)
#define BWTCS_LP_40HZ			(2<<4)
#define BWTCS_LP_75HZ			(3<<4)
#define BWTCS_LP_150HZ			(4<<4)
#define BWTCS_LP_300HZ			(5<<4)
#define BWTCS_LP_600HZ			(6<<4)
#define BWTCS_LP_1200HZ			(7<<4)

#define SOFT_RESET				0xB6

#define I2C_DISABLE				(1<<)

#define MODE_LOW_NOISE			(0<<)

#define RANGE_1G				(0<<1)
#define RANGE_1_5G				(1<<1)
#define RANGE_2G				(2<<1)
#define RANGE_3G				(3<<1)
#define RANGE_4G				(4<<1)
#define RANGE_8G				(5<<1)
#define RANGE_16G				(6<<1)


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

	// Configuring the BMA180

	/* enable writing to chip config */
	uint8_t ctrl0 = read_reg(spi, ADDR_CTRL_REG0);
	ctrl0 |= REG0_WRITE_ENABLE;
	write_reg(spi, ADDR_CTRL_REG0, ctrl0);

#define RANGEMASK 0x0E
#define BWMASK 0xF0

	/* set bandwidth */
	uint8_t bwtcs = read_reg(spi, ADDR_BWTCS);
	bwtcs &= (~BWMASK);
	bwtcs |= (BWTCS_LP_40HZ & BWMASK);
	write_reg(spi, ADDR_BWTCS, bwtcs);

	/* set range */
	uint8_t olsb1 = read_reg(spi, ADDR_OLSB1);
	olsb1 &= (~RANGEMASK);
	olsb1 |= (RANGE_4G & RANGEMASK);
	write_reg(spi, ADDR_OLSB1, olsb1);

//	/* disable interrupts */
//	write_reg(spi, ADDR_CTRL_REG3, 0x10);
//	write_reg(spi, ADDR_CTRL_REG0, 0x50);


//	//-------------------------------------------------------------------------------------
//	// Set ee_w bit
//	temp = read(CTRLREG0);
//	temp |= 0x10;
//	write(CTRLREG0, temp);	// Have to set ee_w to write any other registers
//	//-------------------------------------------------------------------------------------
//	// Set BW
//	temp = read(BWTCS);
//	temp1 = bw;
//	temp1 = temp1<<4;
//	temp &= (~BWMASK);
//	temp |= temp1;
//	write(BWTCS, temp);		// Keep tcs<3:0> in BWTCS, but write new BW
//	//-------------------------------------------------------------------------------------
//	// Set Range
//	temp = read(OLSB1);
//	temp1 = range;
//	temp1 = (temp1<<RANGESHIFT);
//	temp &= (~RANGEMASK);
//	temp |= temp1;
//	write(OLSB1, temp); //Write new range data, keep other bits the same
//	//-------------------------------------------------------------------------------------

	/* block writing to chip config */
	ctrl0 = read_reg(spi, ADDR_CTRL_REG0);
	ctrl0 &= (~REG0_WRITE_ENABLE);
	write_reg(spi, ADDR_CTRL_REG0, ctrl0);




	struct {					/* status register and data as read back from the device */
		uint8_t		cmd;
		int16_t		x;
		int16_t		y;
		int16_t		z;
	} __attribute__((packed))	report;

	report.x = 0;
	report.y = 0;
	report.z = 0;

//	uint8_t		temp;
//	uint8_t		status1;
//	uint8_t		status2;
//	uint8_t		status3;
//	uint8_t		status4;

	report.cmd = ADDR_ACC_X_LSB | DIR_READ | ADDR_INCREMENT;

	SPI_SELECT(spi, PX4_SPIDEV_ACCEL, true);
	SPI_EXCHANGE(spi, &report, &report, sizeof(report));
	SPI_SELECT(spi, PX4_SPIDEV_ACCEL, false);

	message("ACC: x: %d\ty: %d\tz: %d\n", report.x, report.y, report.z);
	usleep(1000);

	return 0;
}
