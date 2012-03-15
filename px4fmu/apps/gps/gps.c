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
#include "custom.h" //header files for the custom protocol for the mediatek diydrones chip
#include "ubx.h" //header files for the ubx protocol
#include <mqueue.h>


/****************************************************************************
 * Definitions
 ****************************************************************************/
//Definition for custom mode
#define MEDIATEK_REFRESH_RATE_4HZ "$PMTK220,250*29\r\n" //refresh rate - 4Hz - 250 milliseconds
#define MEDIATEK_REFRESH_RATE_5HZ "$PMTK220,200*2C\r\n"
#define MEDIATEK_REFRESH_RATE_10HZ "$PMTK220,100*2F\r\n" //refresh rate - 10Hz - 100 milliseconds
#define MEDIATEK_FACTORY_RESET "$PMTK104*37\r\n" //clear current settings
#define MEDIATEK_CUSTOM_BINARY_MODE "$PGCMD,16,0,0,0,0,0*6A\r\n"
#define MEDIATEK_FULL_COLD_RESTART "$PMTK104*37\r\n"
//#define NMEA_GGA_ENABLE "$PMTK314,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0*27\r\n" //Set GGA messages



/****************************************************************************
 * user_start
 ****************************************************************************/

int gps_main(int argc, char *argv[])
{
    // print text
    printf("Hello, GPS!\n");
    usleep(100000);

    // Define attributes for message queue to send GPS information
    struct mq_attr attr;
    attr.mq_flags = 0;
    attr.mq_maxmsg = 10;
    attr.mq_msgsize = 5;
    attr.mq_curmsgs = 0;

    // open message queue to write
    mqd_t gps_queue;
    gps_queue = mq_open( "gps_queue", O_WRONLY, 0666, &attr );
    if(-1 == gps_queue)
	{
		printf("GPS Queue creation failed\n");
	}
    //test for queue
    char msg[5] = "HELLO";
    int send_result;


    //read input arguments
    char * device = argv[1];
    char * mode = "nmea"; //TODO: write enum once all modes are defined
    if(3 <= argc)
	{
		mode = argv[2];
	}

    int buffer_size = 2000;
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

	//mtk state
	gps_bin_custom_state_t * mtk_state = malloc(sizeof(gps_bin_custom_state_t));
	mtk_decode_init(mtk_state);
	mtk_state->print_errors = false;
	size_t result_write;

	//ubx state
	gps_bin_ubx_state_t * ubx_state = malloc(sizeof(gps_bin_ubx_state_t));
	ubx_decode_init(ubx_state);
	ubx_state->print_errors = false;


	if	( !strcmp("custom",mode) )
	{
		printf("custom mode\n");
		//set 10Hz
		result_write =  write(fd, MEDIATEK_REFRESH_RATE_10HZ, strlen(MEDIATEK_REFRESH_RATE_10HZ));
		if(result_write != strlen(MEDIATEK_REFRESH_RATE_10HZ))
		{
			printf("Set update speed to 10 Hz failed\n");
		}
		else
		{
			printf("Set update speed to 10 Hz successful\n");
		}
		//set custom mode
		result_write =  write(fd, MEDIATEK_CUSTOM_BINARY_MODE, strlen(MEDIATEK_CUSTOM_BINARY_MODE));
		if(result_write != strlen(MEDIATEK_CUSTOM_BINARY_MODE))
		{
			printf("Set custom mode failed");
		}
		else
		{
			printf("Set custom mode successful");
		}
	}


    if( !strcmp("ubx",mode) )
	{
    	printf("ubx mode\n");
    	//set parameters for ubx

		if (configure_gps_ubx(fd) != 0)
		{
			printf("Configuration of gps module to ubx failed\n");
		}
		else
		{
			printf("Configuration of gps module to ubx successful\n");
		}

	}




	while(1)
	{
		if( !strcmp("nmea",mode) )
		{
			//get gps data into info
			read_gps_nmea(fd, gps_rx_buffer, buffer_size, info, &parser);
			//convert latitude longitude
			lat_dec = ndeg2degree(info->lat);
			lon_dec = ndeg2degree(info->lon);

			//Test output
			printf("Lat:%d, Lon:%d,Elev:%d, Sig:%d, Fix:%d, Inview:%d\n", (int)(lat_dec*1e6), (int)(lon_dec*1e6), (int)(info->elv*1e6), info->sig, info->fix, info->satinfo.inview);
		}
		else if ( !strcmp("ubx",mode) )
		{
			//get gps data into info
			read_gps_ubx(fd, gps_rx_buffer, buffer_size, info, ubx_state); //TODO: atm using the info struct from the nmea library, once the gps/mavlink structures are clear--> use own struct
			lat_dec = info->lat;
			lon_dec = info->lon;
			printf("Lat:%d, Lon:%d,Elev:%d, Sig:%d, Fix:%d, Inview:%d\n", (int)(lat_dec*1e6), (int)(lon_dec*1e6), (int)(info->elv*1e6), info->sig, info->fix, info->satinfo.inview);
		}
		else if	( !strcmp("custom",mode) )
		{
			//info is used as storage of the gps information. lat lon are already in fractional degree format * 10e6
			//see custom.h/mtk_parse for more information on which variables are stored in info
			nmea_zero_INFO(info);

			//get gps data into info
			read_gps_custom(fd, gps_rx_buffer, buffer_size, info, mtk_state);

			//Test output
			printf("Lat:%d, Lon:%d,Elev:%d, Sig:%d, Fix:%d, Inview:%d\n", (int)(info->lat), (int)info->lon, (int)info->elv, info->sig, info->fix, info->satinfo.inview);

		}

    	//Send GPS information in message queue
    	send_result = mq_send(gps_queue, msg, 5*sizeof(char),0);
    	printf("Send result: %d ", send_result);
		if (-1 == send_result) //TODO Priority?
		{
			printf("Message could not be sent\n");
		}
	}

	free(gps_rx_buffer);
	free(info);


	//close port
	close_port(fd);

	//destroy gps parser
	nmea_parser_destroy(&parser);

	//close GPS queue
	mq_close(gps_queue);

    return 0;
}


