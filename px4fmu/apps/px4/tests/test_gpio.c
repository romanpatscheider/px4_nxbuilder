/****************************************************************************
 * px4/sensors/test_gpio.c
 *
 *   Copyright (C) 2012 Michael Smith. All rights reserved.
 *   Authors: Michael Smith <DrZiplok@me.com>
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

#include <nuttx/analog/adc.h>

#include "tests.h"

#include <arch/board/drv_gpio.h>


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
 * Name: test_gpio
 ****************************************************************************/

int test_gpio(int argc, char *argv[])
{
	int		fd;
	int		adc;
	int		ret = 0;

	fd = open("/dev/gpio", O_RDONLY | O_NONBLOCK);
	if (fd < 0) {
		printf("GPIO: open fail\n");
		return ERROR;
	}

	//	adc = open("/dev/adc0", O_RDONLY);
	//	if (adc < 0) {
	//		printf("GPIO: adc open fail\n");
	//		return ERROR;
	//	}

	if (ioctl(fd, GPIO_DIRECTION, GPIO_ALL_OUTPUTS)) {

		printf("GPIO: output direction set fail\n");
		return ERROR;
	}

	/* change all gpios */
	if (ioctl(fd, GPIO_SET, 0)) {

		printf("GPIO: set fail for index 0\n");
		return ERROR;
	} else {
		printf("GPIO: set success for index 0\n");
	}

	/* Wait for 20 ms */
	usleep(20000);

	if (ioctl(fd, GPIO_CLEAR, 0)) {

		printf("GPIO: clear fail for index 0\n");
		return ERROR;
	} else {
		printf("GPIO: clear success for index 0\n");
	}

	/* Wait for 20 ms */
	usleep(20000);

	if (ioctl(fd, GPIO_SET, 1)) {

		printf("GPIO: set fail for index 1\n");
		return ERROR;
	} else {
		printf("GPIO: set success for index 1\n");
	}

	/* Wait for 20 ms */
	usleep(20000);

	if (ioctl(fd, GPIO_CLEAR, 1)) {

		printf("GPIO: clear fail for index 1\n");
		return ERROR;
	} else {
		printf("GPIO: clear success for index 1\n");
	}
#ifdef CONFIG_PX4_UART2_RTS_CTS_AS_GPIO
	if (ioctl(fd, GPIO_SET, 2)) {

		printf("GPIO: set fail for index 2\n");
		return ERROR;
	}

	/* Wait for 20 ms */
	usleep(20000);

	if (ioctl(fd, GPIO_CLEAR, 2)) {

		printf("GPIO: clear fail for index 2\n");
		return ERROR;
	}
	if (ioctl(fd, GPIO_SET, 3)) {

		printf("GPIO: set fail for index 3\n");
		return ERROR;
	}

	/* Wait for 20 ms */
	usleep(20000);

	if (ioctl(fd, GPIO_CLEAR, 3)) {

		printf("GPIO: clear fail for index 3\n");
		return ERROR;
	}
#endif

	/* Go back to default */
	ioctl(fd, GPIO_DIRECTION, GPIO_ALL_INPUTS);
	close(fd);
	close(adc);

	printf("\t GPIO test successful.\n");

	return ret;
}
