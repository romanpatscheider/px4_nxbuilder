/****************************************************************************
 * apps/reboot.c
 *
 *   Copyright (C) 2012 Lorenz Meier. All rights reserved.
 *   Author: Lorenz Meier <lm@inf.ethz.ch>
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
#include <stdio.h>
#include <signal.h>
#include <fcntl.h>
#include <unistd.h>
#include <termios.h>

#include <sys/types.h>
#include <stdio.h>
#include <stdlib.h>
#include <signal.h>
#include <string.h>
#include <errno.h>

/****************************************************************************
 * Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Data
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * user_start
 ****************************************************************************/



/****************************************************************************
 * Definitions
 ****************************************************************************/

int fd;

static int gps_init()
{
	struct termios options;

	fd = open("/dev/ttyS3", O_RDONLY | O_NOCTTY);
	if (fd < 0) {
		printf("GPS UART6: open fail\n");
		return ERROR;
	} else {
		tcgetattr(fd, &options);
		/* Set baud rate to 38400 */
		//cfsetispeed(&options, B38400);
		//cfsetospeed(&options, B38400);
		/* Enable receiver and set local mode */
		options.c_cflag |= (CLOCAL | CREAD);
		/* Set the new options */
		tcsetattr(fd, TCSANOW, &options);
		printf("\tGPS configured..\n");
	}
	return 0;
}

/****************************************************************************
 * Name: interrupt
 ****************************************************************************/

static void interrupt(int signo)
{
	printf("Exiting...\n");
	fflush(stdout);
	close(fd);
	exit(0);
}

int gpsd_main(int argc, char *argv[])
{
    // print text
    printf("GPS deamon ready\n\n");


    int	ret = gps_init();

    fflush(stdout);

//	sigset_t sigset;
//	sigemptyset(&sigset);
//	sigaddset(SI_USER, &sigset);
	struct sigaction act;

	act.sa_u._sa_handler = interrupt;
//	act.sa_mask = sigset;

    sigaction(SI_USER, &act, NULL);

    while(true)
    {
    	char ch;
    	/* blocking read on next byte */
    	read (fd, &ch, 1);
    	printf("%c", ch);
    }

    /* Should never reach here, only on error */
    return ret;
}


