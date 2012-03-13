/****************************************************************************
 * px4/sensors/sensors_main.c
 *
 *   Copyright (C) 2011 Gregory Nutt. All rights reserved.
 *   Copyright (C) 2011-2012 Michael Smith. All rights reserved.
 *   Authors: Michael Smith <DrZiplok@me.com>
 *    Gregory Nutt <gnutt@nuttx.org>
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
 * 3. Neither the name NuttX nor the names of its contributors may be
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
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

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
#include <nuttx/i2c.h>

#include "sensors.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Types
 ****************************************************************************/

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Private Data
 ****************************************************************************/

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: user_start/adc_main
 ****************************************************************************/

int sensors_main(int argc, char *argv[])
{
	struct spi_dev_s *spi;
	int result = -1;

	spi = up_spiinitialize(1);
	if (!spi) {
		message("Failed to initialize SPI port 1\n");
		goto out;
	}

	struct i2c_dev_s *i2c;
	i2c = up_i2cinitialize(2);
	if (!i2c) {
		message("Failed to initialize I2C bus 2\n");
		goto out;
	}

	int ret;

#define EEPROM_ADDRESS		0x50
#define HMC5883L_ADDRESS	0x1E

	//uint8_t devaddr = EEPROM_ADDRESS;

	I2C_SETFREQUENCY(i2c, 100000);
//
//	uint8_t subaddr = 0x00;
//	int ret = 0;
//
//	// ATTEMPT HMC5883L CONFIG
//	I2C_SETADDRESS(i2c, HMC5883L_ADDRESS, 7);
//	subaddr = 0x02; // mode register
//	ret = I2C_WRITE(i2c, &subaddr, 0);
//	if (ret < 0)
//	{
//		message("I2C_WRITE failed: %d\n", ret);
//	}
//	else
//	{
//		message("I2C_WRITE SUCCEEDED: %d\n", ret);
//	}

	//fflush(stdout);
//
//
//


#define STATUS_REGISTER		0x09 // Of HMC5883L



	// ATTEMPT HMC5883L WRITE
	I2C_SETADDRESS(i2c, HMC5883L_ADDRESS, 7);
	uint8_t cmd = 0x09;
	uint8_t status_id[4] = {0, 0, 0, 0};


	ret = I2C_WRITEREAD(i2c, &cmd, 1, status_id, 4);

	if (ret >= 0 && status_id[1] == 'H' && status_id[2] == '4' && status_id[3] == '3')
	{
		message("HMC5883L identified, device status: %d\n", status_id[0]);
	} else {
		message("HMC5883L identification failed: %d\n", ret);
	}


	//usleep(100);

//	// ATTEMPT HMC5883L WRITE
//	I2C_SETADDRESS(i2c, EEPROM_ADDRESS, 7);
//	uint8_t eeprom[2] = {0x00, 0x00};
//
//
//	ret = I2C_WRITEREAD(i2c, eeprom, 1, eeprom, 1);
//
//
//
//	//ret = I2C_WRITE(i2c, hmc5883l_continuous, 1);
//	if (ret < 0)
//	{
//		message("EEPROM WRITEREAD failed: %d\n", ret);
//	}
//	else
//	{
//		message("EEPROM WRITEREAD SUCCEEDED: %d\n", ret);
//	}
//
//	//up_i2cuninitialize(i2c);
//
//	//usleep(100);
//
//
	// ATTEMPT HMC5883L READ
	I2C_SETADDRESS(i2c, HMC5883L_ADDRESS, 7);
	uint8_t hmc5883l_status[5] = {'-', '-', '-', '-', '\0'};
	uint8_t who_am_i = 0x10;
	I2C_WRITE(i2c, &who_am_i, 1);
	ret = I2C_READ(i2c, hmc5883l_status, 3);
	if (ret < 0)
	{
		message("HMC5883L READ failed: %d, val: %d\n", ret, hmc5883l_status[1]);
	}
	else
	{
		message("HMC5883L READ SUCCEEDED: %d, val:%s\n", ret, (char*)hmc5883l_status);
	}
//
//	// ATTEMPT HMC5883L WRITE
////		I2C_SETADDRESS(i2c, HMC5883L_ADDRESS, 7);
//		hmc5883l_continuous[0] = STATUS_REGISTER;
//		ret = I2C_WRITE(i2c, hmc5883l_continuous, 1);
//		if (ret < 0)
//		{
//			message("HMC5883L WRITE failed: %d\n", ret);
//		}
//		else
//		{
//			message("HMC5883L WRITE SUCCEEDED: %d\n", ret);
//		}

	//fflush(stdout);
//
//	// ATTEMPT MS5611-01ba WRITE
//
//	// Possible addresses: 0x77 or 0x76
//#define MS5611_ADDRESS_1	0x76
//#define MS5611_ADDRESS_2	0x77
//	I2C_SETADDRESS(i2c, MS5611_ADDRESS_1, 7);
//		uint8_t ms5611_cmd[2] = {0x00, 0x00};
//		ret = I2C_WRITE(i2c, ms5611_cmd, 2);
//		if (ret < 0)
//		{
//			message("MS5611 #1 WRITE failed: %d\n", ret);
//		}
//		else
//		{
//			message("MS5611 #2 WRITE SUCCEEDED: %d\n", ret);
//		}
//
//		fflush(stdout);
//
//		I2C_SETADDRESS(i2c, MS5611_ADDRESS_2, 7);
//		ret = I2C_WRITE(i2c, ms5611_cmd, 2);
//		if (ret < 0)
//		{
//			message("MS5611 #1 WRITE failed: %d\n", ret);
//		}
//		else
//		{
//			message("MS5611 #2 WRITE SUCCEEDED: %d\n", ret);
//		}
//
//		fflush(stdout);



//	// TESTING CODE, I2C TRANSACTION
//	uint8_t val[1];
//
//	struct i2c_msg_s msgv[2] = {
//	        {
//	            .addr   = HMC5883L_ADDRESS,
//	            .flags  = 0,
//	            .buffer = &subaddr,
//	            .length = 1
//	        },
//	        {
//	            .addr   = HMC5883L_ADDRESS,
//	            .flags  = I2C_M_READ,
//	            .buffer = val,
//	            .length = 1
//	        }
//	    };
//
//	int retval;
//
//	if ( (retval = I2C_TRANSFER(i2c, msgv, 2)) == OK )
//	{
//		printf("SUCCESS ACESSING HMC5883L: %d", retval);
//	}

	// Configure sensors
	l3gd20_test_configure(spi);
	bma180_test_configure(spi);

	int i;
	for (i = 0; i < 3; i++)
	{
		l3gd20_test_read(spi);
		bma180_test_read(spi);
		printf("# %d of 10\n", i+1);
		usleep(50000);
	}


	out:
	//SPI_LOCK(spi, false);
   	msgflush();
	return result;
}
