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

	int i;
	for (i = 0; i < 10; i++)
	{
		l3gd20_test(spi);
		bma180_test(spi);
		printf("# %d of 10\n", i+1);
		usleep(50000);
	}

	struct i2c_dev_s *i2c;
	i2c = up_i2cinitialize(2);
	up_i2cuninitialize(i2c);
	i2c = up_i2cinitialize(2);
	if (!i2c) {
		message("Failed to initialize I2C bus 2\n");
		goto out;
	}

#define EEPROM_ADDRESS		0x50
#define HMC5883L_ADDRESS	0x1E

#define STATUS_REGISTER		0x09 // Of HMC5883L

	uint8_t devaddr = EEPROM_ADDRESS;

	// ATTEMPT EEPROM READ
	I2C_SETADDRESS(i2c, devaddr, 7);
	uint8_t subaddr = 0x00; // 10
	int ret = I2C_WRITE(i2c, &subaddr, 1);
	if (ret < 0)
	{
		message("I2C_WRITE failed: %d\n", ret);
	}
	else
	{
		message("I2C_WRITE SUCCEEDED: %d\n", ret);
	}

	fflush(stdout);

	// ATTEMPT HMC5883L READ
	I2C_SETADDRESS(i2c, HMC5883L_ADDRESS, 7);
	subaddr = STATUS_REGISTER; // 10
	ret = I2C_READ(i2c, &subaddr, 1);
	if (ret < 0)
	{
		message("I2C_WRITE failed: %d\n", ret);
	}
	else
	{
		message("I2C_WRITE SUCCEEDED: %d\n", ret);
	}

	fflush(stdout);


	// TESTING CODE, I2C TRANSACTION
	uint8_t val[1];

	struct i2c_msg_s msgv[2] = {
	        {
	            .addr   = devaddr,
	            .flags  = 0,
	            .buffer = &subaddr,
	            .length = 1
	        },
	        {
	            .addr   = devaddr,
	            .flags  = I2C_M_READ,
	            .buffer = val,
	            .length = 1
	        }
	    };

	int retval;

	    if ( (retval = I2C_TRANSFER(i2c, msgv, 2)) == OK )
	    {
	    	printf("SUCCESS ACESSING EEPROM: %d", retval);
	    }


	out:
	//SPI_LOCK(spi, false);
   	msgflush();
	return result;
}
