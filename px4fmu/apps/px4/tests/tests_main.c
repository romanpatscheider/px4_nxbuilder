/****************************************************************************
 * px4/sensors/tests_main.c
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
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <debug.h>

#include <arch/board/board.h>

#include <nuttx/spi.h>

#include "tests.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Types
 ****************************************************************************/

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int test_help(int argc, char *argv[]);
static int test_all(int argc, char *argv[]);

/****************************************************************************
 * Private Data
 ****************************************************************************/

struct {
	const char 	*name;
	int		(* fn)(int argc, char *argv[]);
	unsigned	options;
#define OPT_NOHELP	(1<<0)
#define OPT_NOALLTEST	(1<<1)
} tests[] = {
	{"sensors",	test_sensors,	0},
	{"gpio",	test_gpio,	0},
	{"hrt",		test_hrt,	0},
	{"led",		test_led,	0},
	{"ppm",		test_ppm,	0},
	{"adc",		test_adc,	0},
	{"all",		test_all,	OPT_NOALLTEST},
	{"help",	test_help,	OPT_NOALLTEST | OPT_NOHELP},
	{NULL,		NULL}
};

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static int
test_help(int argc, char *argv[])
{
	unsigned	i;
	
	printf("Available tests:\n");
	for (i = 0; tests[i].name; i++)
		printf("  %s\n", tests[i].name);
	return 0;
}

static int
test_all(int argc, char *argv[])
{
	unsigned	i;
	char		*args[2] = {"all", NULL};
	
	printf("Running all tests...\n\n");
	for (i = 0; tests[i].name; i++) {
		printf("  %s:\n", tests[i].name);
		if (tests[i].fn(1, args)) {
			printf("  FAIL\n");
		} else {
			printf("  PASS\n");			
		}
	}
	return 0;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: tests_main
 ****************************************************************************/

int tests_main(int argc, char *argv[])
{
	unsigned	i;

	if (argc < 2) {
		printf("tests: missing test name - 'tests help' for a list of tests\n");
		return 1;
	}

	for (i = 0; tests[i].name; i++) {
		if (!strcmp(tests[i].name, argv[1]))
			return tests[i].fn(argc - 1, argv + 1);
	}

	printf("tests: no test called '%s' - 'tests help' for a list of tests\n", argv[1]);
	return 1;
}
