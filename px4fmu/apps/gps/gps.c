/****************************************************************************
 * examples/hello/main.c
 *
 *   Copyright (C) 2008, 2011 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <spudmonkey@racsa.co.cr>
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

#include "gps.h"



/****************************************************************************
 * Definitions
 ****************************************************************************/

/* Struct for storage of gps information and transmission to mavlink app  */
global_data_gps_t global_data_gps = {.access_conf.initialized = 0};
//gps_bin_ubx_state_t * ubx_state;

/****************************************************************************
 * Pthread loops
 ****************************************************************************/







/****************************************************************************
 * user_start
 ****************************************************************************/

int gps_main(int argc, char *argv[])
{
    // print text
    printf("Hello, gps!\n");
    usleep(100000);

    /* initialize shared data structures */
    global_data_init(&global_data_gps.access_conf);
    global_data_init(&global_data_sys_status.access_conf);


    /* default values */
	char * commandline_usage = "\tusage: gps -d devicename -m mode\n";
	char * device = "/dev/ttyS3";
	char * mode = "nmea";

	/* read arguments */
	int i;
	for (i = 1; i < argc; i++) //argv[0] is "mavlink"
	{
		if (strcmp(argv[i], "-d") == 0 || strcmp(argv[i], "--device") == 0)  //device set
		{
			if(argc > i+1)
			{
				device = argv[i+1];
			}
			else
			{
				printf(commandline_usage);
				return 0;
			}
		}
		if (strcmp(argv[i], "-m") == 0 || strcmp(argv[i], "--mode") == 0)  //device set
		{
			if(argc > i+1)
			{
				mode = argv[i+1];
			}
			else
			{
				printf(commandline_usage);
				return 0;
			}
		}
	}


	/* open port (baud rate is set in defconfig file) */
	int fd = open_port(device);
	if(fd != -1)
	{
		printf("\tgps: Port opened: %s\n", device);
	}
	else
	{
		printf("\tgps: Could not open port, exiting gps app!\n");
		sleep(1);
		return 0;
	}

//TODO: add mode if here (reads from arguments and configuration, watchdog will look for changes in mode configuration while running (?))
	ubx_init();



    /* create pthreads */
    pthread_create (&ubx_thread, NULL, ubx_loop, (void *)&fd);


    /* wait before starting watchdog */
	sleep(5);
    pthread_create (&ubx_watchdog_thread, NULL, ubx_watchdog_loop, (void *)&fd);

    /* wait for threads to complete */
	pthread_join(ubx_thread, NULL);
	pthread_join(ubx_watchdog_thread, (void *)&fd);

	return 0;

}

int open_port(char * port)
{
	int fd; // File descriptor for the gps port

	/* Open serial port */
	fd = open(port, O_CREAT|O_RDWR | O_NOCTTY); // O_RDWR - Read and write O_NOCTTY - Ignore special chars like CTRL-C
	return (fd);
}


void close_port(int fd)
{
	/* Close serial port */
    close(fd);
}



