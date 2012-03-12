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

/****************************************************************************
 * Included Files
 ****************************************************************************/


#include <nuttx/config.h>
#include <stdlib.h>
#include <stdio.h>
#include <fcntl.h>
#include "gps.h"
#include "nmealib/nmea/nmea.h"


/****************************************************************************
 * Definitions
 ****************************************************************************/



/****************************************************************************
 * user_start
 ****************************************************************************/

int gps_main(int argc, char *argv[])
{
    // print text
    printf("Hello, gps!!\n");
    usleep(100000);

    char * device = "/dev/ttyS3";


    int buffer_size = 1000;
    nmeaINFO * info = malloc(sizeof(nmeaINFO));

    //open port (baud rate is set in defconfig file)
	int fd = open_port(device);
	if(fd != -1)
	{
		printf("port opened: %s\n", device);
	}
	else
	{
		printf("Could not open port\n");
		sleep(1);
		return 0;
	}

	//gps parser (nmea)
	nmeaPARSER parser;
	nmea_parser_init(&parser);
	nmea_zero_INFO(info);
	float lat_dec = 0;
	float lon_dec = 0;

	char * gps_rx_buffer = malloc(buffer_size*sizeof(char));


	while(1)
	{
		//get gps data into info
		read_gps_nmea(fd, gps_rx_buffer, buffer_size, info, &parser);

		//convert latitude longitude
		lat_dec = ndeg2degree(info->lat);
		lon_dec = ndeg2degree(info->lon);

		//Test output
		printf("Lat:%d, Lon:%d,Elev:%d, Sig:%d, Fix:%d, Inview:%d\n", (int)(lat_dec*1e6), (int)(lon_dec*1e6), (int)(info->elv*1e6), info->sig, info->fix, info->satinfo.inview);

	}

	free( gps_rx_buffer );



	free(info);

	//close port
	close_port(fd);


    return 0;
}


