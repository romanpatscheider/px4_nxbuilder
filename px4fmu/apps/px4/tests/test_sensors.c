/****************************************************************************
 * px4/sensors/test_sensors.c
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

#include <arch/board/board.h>

#include <nuttx/spi.h>

#include "tests.h"

#include <arch/board/drv_lis331.h>
#include <arch/board/drv_bma180.h>
#include <arch/board/drv_l3gd20.h>
#include <arch/board/drv_hmc5883l.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Types
 ****************************************************************************/

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

//static int	lis331(int argc, char *argv[]);
static int  l3gd20(int argc, char *argv[]);
static int	bma180(int argc, char *argv[]);
static int hmc5883l(int argc, char *argv[]);
static int ms5611(int argc, char *argv[]);

/****************************************************************************
 * Private Data
 ****************************************************************************/

struct {
	const char	*name;
	const char	*path;
	int		(* test)(int argc, char *argv[]);
} sensors[] = {
	{"l3gd20",	"/dev/l3gd20",	l3gd20},
    {"bma180",	"/dev/bma180",	bma180},
    {"hmc5883l",	"/dev/hmc5883l",	hmc5883l},
    {"ms5611",	"/dev/ms5611",	ms5611},
//    {"lis331",	"/dev/lis331",	lis331},
	{NULL, NULL, NULL}
};

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

//static int
//lis331(int argc, char *argv[])
//{
//	int		fd;
//	int16_t	buf[3];
//	int		ret;
//
//	fd = open("/dev/lis331", O_RDONLY);
//	if (fd < 0) {
//		printf("\tlis331: not present on PX4FMU v1.5 and later\n");
//		return ERROR;
//	}
//
//	if (ioctl(fd, LIS331_SETRATE, LIS331_RATE_50Hz) ||
//	    ioctl(fd, LIS331_SETRANGE, LIS331_RANGE_4G)) {
//
//		printf("LIS331: ioctl fail\n");
//		return ERROR;
//	}
//
//	/* wait at least 100ms, sensor should have data after no more than 20ms */
//	usleep(100000);
//
//	/* read data - expect samples */
//	ret = read(fd, buf, sizeof(buf));
//	if (ret != sizeof(buf)) {
//		printf("LIS331: read1 fail (%d)\n", ret);
//		return ERROR;
//	}
//
//	/* read data - expect no samples (should not be ready again yet) */
//	ret = read(fd, buf, sizeof(buf));
//	if (ret != 0) {
//		printf("LIS331: read2 fail (%d)\n", ret);
//		return ERROR;
//	}
//
//	/* XXX more tests here */
//
//	return 0;
//}

static int
l3gd20(int argc, char *argv[])
{
	printf("\tl3gd20: test start\n");
	fflush(stdout);

	int		fd;
	int16_t	buf[3] = {0, 0, 0};
	int		ret;

	fd = open("/dev/l3gd20", O_RDONLY | O_NONBLOCK);
	if (fd < 0) {
		printf("L3GD20: open fail\n");
		return ERROR;
	}

//	if (ioctl(fd, L3GD20_SETRATE, L3GD20_RATE_760HZ_LP_50HZ) ||
//	    ioctl(fd, L3GD20_SETRANGE, L3GD20_RANGE_500DPS)) {
//
//		printf("L3GD20: ioctl fail\n");
//		return ERROR;
//	} else {
//		printf("\tconfigured..\n");
//	}
//
//	/* wait at least 100ms, sensor should have data after no more than 2ms */
//	usleep(100000);



	/* read data - expect samples */
	ret = read(fd, buf, sizeof(buf));
	if (ret != sizeof(buf)) {
		printf("\tl3gd20: read1 fail (%d should have been %d)\n", ret, sizeof(buf));
		//return ERROR;
	} else {
		printf("\tl3gd20 values #1: x:%d\ty:%d\tz:%d\n", buf[0], buf[1], buf[2]);
	}

	/* wait at least 2 ms, sensor should have data after no more than 1.5ms */
	usleep(2000);

	/* read data - expect no samples (should not be ready again yet) */
	ret = read(fd, buf, sizeof(buf));
	if (ret != sizeof(buf)) {
		printf("\tl3gd20: read2 fail (%d)\n", ret);
		return ERROR;
	} else {
		printf("\tl3gd20 values #2: x:%d\ty:%d\tz:%d\n", buf[0], buf[1], buf[2]);
	}

	/* empty sensor buffer */
	ret = 0;
	while(ret != sizeof(buf))
	{
		// Keep reading until successful
		ret = read(fd, buf, sizeof(buf));
	}

	/* test if FIFO is operational */
	usleep(14800); // Expecting 10 measurements

	ret = 0;
	int count = 0;
	bool dataready = true;
	while(dataready)
	{
		// Keep reading until successful
		ret = read(fd, buf, sizeof(buf));
		if (ret != sizeof(buf))
		{
			dataready = false;
		} else {
			count++;
		}
	}

	printf("\tl3gd20: Drained FIFO with %d values (expected 8-12)\n", count);

	/* read data - expect no samples (should not be ready again yet) */
	ret = read(fd, buf, sizeof(buf));
	if (ret != 0) {
		printf("\tl3gd20: ERROR: read3 fail - there should not have been data ready\n", ret);
		return ERROR;
	}

	close(fd);

	/* XXX more tests here */

	/* Let user know everything is ok */
	printf("\tOK: L3GD20 passed all tests successfully\n");
	return ret;
}

static int
bma180(int argc, char *argv[])
{
	printf("\tbma180: test start\n");
	fflush(stdout);

	int		fd;
	int16_t	buf[3] = {0, 0, 0};
	int		ret;
    
	fd = open("/dev/bma180", O_RDONLY);
	if (fd < 0) {
		printf("\tbma180: open fail\n");
		return ERROR;
	}
    
//	if (ioctl(fd, LIS331_SETRATE, LIS331_RATE_50Hz) ||
//	    ioctl(fd, LIS331_SETRANGE, LIS331_RANGE_4G)) {
//	 	
//		printf("BMA180: ioctl fail\n");
//		return ERROR;
//	}
//    
	/* wait at least 100ms, sensor should have data after no more than 20ms */
	usleep(100000);

	/* read data - expect samples */
	ret = read(fd, buf, sizeof(buf));
	if (ret != sizeof(buf)) {
		printf("\tbma180: read1 fail (%d)\n", ret);
		return ERROR;
	} else {
		printf("\tbma180 values: x:%d\ty:%d\tz:%d\n", buf[0], buf[1], buf[2]);
	}

	/* wait at least 10ms, sensor should have data after no more than 2ms */
	usleep(100000);

	ret = read(fd, buf, sizeof(buf));
	if (ret != sizeof(buf)) {
		printf("\tbma180: read2 fail (%d)\n", ret);
		return ERROR;
	} else {
		printf("\tbma180 values: x:%d\ty:%d\tz:%d\n", buf[0], buf[1], buf[2]);
	}

	/* empty sensor buffer */
	ret = 0;
	while(ret != sizeof(buf))
	{
		// Keep reading until successful
		ret = read(fd, buf, sizeof(buf));
	}

	ret = read(fd, buf, sizeof(buf));
	if (ret != 0) {
		printf("\tbma180: ERROR: read3 fail - there should not have been data ready\n", ret);
		return ERROR;
	}
    
	/* XXX more tests here */
    
	/* Let user know everything is ok */
	printf("\tOK: BMA180 passed all tests successfully\n");

	return 0;
}

static int
ms5611(int argc, char *argv[])
{
	printf("\tms5611: test start\n");
	fflush(stdout);

	int		fd;
	uint32_t	buf[2] = {0, 0};
	int		ret;

	fd = open("/dev/ms5611", O_RDONLY);
	if (fd < 0) {
		printf("\tms5611: open fail\n");
		return ERROR;
	}
//
////	if (ioctl(fd, LIS331_SETRATE, LIS331_RATE_50Hz) ||
////	    ioctl(fd, LIS331_SETRANGE, LIS331_RANGE_4G)) {
////
////		printf("BMA180: ioctl fail\n");
////		return ERROR;
////	}
////
	/* wait at least 10ms, sensor should have data after no more than 6.5ms */
	usleep(10000);

	/* read data - expect samples */
	ret = read(fd, buf, sizeof(buf));
	if (ret != sizeof(buf)) {
		printf("\tms5611: read1 fail (%d)\n", ret);
		return ERROR;
	} else {
		printf("\tms5611 values: pressure raw:%d\ttemp raw:%d\n", buf[0], buf[1]);
	}

	/* XXX more tests here */

	/* Let user know everything is ok */
	printf("\tOK: MS5611 passed all tests successfully\n");

	return 0;
}

static int
hmc5883l(int argc, char *argv[])
{
	printf("\thmc5883l: test start\n");
	fflush(stdout);

	int		fd;
	int16_t	buf[3] = {0, 0, 0};
	int		ret;

	fd = open("/dev/hmc5883l", O_RDONLY);
	if (fd < 0) {
		printf("\thmc5883l: open fail\n");
		return ERROR;
	}

//	if (ioctl(fd, LIS331_SETRATE, LIS331_RATE_50Hz) ||
//	    ioctl(fd, LIS331_SETRANGE, LIS331_RANGE_4G)) {
//
//		printf("BMA180: ioctl fail\n");
//		return ERROR;
//	}
//
	/* wait at least 10ms, sensor should have data after no more than 6.5ms */
	usleep(10000);

	/* read data - expect samples */
	ret = read(fd, buf, sizeof(buf));
	if (ret != sizeof(buf)) {
		printf("\thmc5883l: read1 fail (%d) values: x:%d\ty:%d\tz:%d\n", ret, buf[0], buf[1], buf[2]);
		return ERROR;
	} else {
		printf("\thmc5883l values: x:%d\ty:%d\tz:%d\n", buf[0], buf[1], buf[2]);
	}

	/* XXX more tests here */

	/* Let user know everything is ok */
	printf("\tOK: HMC5883L passed all tests successfully\n");

	return 0;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: test_sensors
 ****************************************************************************/

int test_sensors(int argc, char *argv[])
{
	unsigned	i;

	printf("Running sensors tests:\n\n");
	fflush(stdout);

	for (i = 0; sensors[i].name; i++) {
		printf("  sensor: %s\n", sensors[i].name);
		sensors[i].test(argc, argv);
		fflush(stdout);
	}
	return 0;
}
