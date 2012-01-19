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

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Types
 ****************************************************************************/

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int	st_lis331(int argc, char *argv[]);

/****************************************************************************
 * Private Data
 ****************************************************************************/

struct {
	const char	*name;
	const char	*path;
	int		(* test)(int argc, char *argv[]);
} sensors[] = {
	{"lis331",	"/dev/lis331",	st_lis331},
	{NULL, NULL, NULL}
};

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static int
st_lis331(int argc, char *argv[])
{
	int		fd;
	uint16_t	buf[3];
	int		ret;

	fd = open("/dev/lis331", O_RDONLY);
	if (fd < 0) {
		printf("LIS331: open fail\n");
		return ERROR;
	}

	if (ioctl(fd, LIS331_SETRATE, LIS331_RATE_50Hz) ||
	    ioctl(fd, LIS331_SETRANGE, LIS331_RANGE_4G)) {
	 	
		printf("LIS331: ioctl fail\n");
		return ERROR;
	}

	/* wait at least 100ms, sensor should have data after no more than 20ms */
	usleep(100000);

	/* read data - expect samples */
	ret = read(fd, buf, sizeof(buf));
	if (ret != sizeof(buf)) {
		printf("LIS331: read1 fail (%d)\n", ret);
		return ERROR;
	}

	/* read data - expect no samples (should not be ready again yet) */
	ret = read(fd, buf, sizeof(buf));
	if (ret != 0) {
		printf("LIS331: read2 fail (%d)\n", ret);
		return ERROR;
	}

	/* XXX more tests here */

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

	for (i = 0; sensors[i].name; i++) {
		printf("  sensor: %s\n", sensors[i].name);
		sensors[i].test(argc, argv);
	}
	return 0;
}
