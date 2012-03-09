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
    printf("Hello, gps!!");


    //some testing
	float sample = 5.5;
	sample *= 1e6;
	printf("sample=%f, cast as int: %d\n", sample, (int) sample);   //storing floats works (float)



//	double sample2 = (double)sample;
//
//	printf("sample2=%f, cast as int: %d\n", sample2, (int) sample2);
//	float test = 0;




////	char * number = "52.2\0";
//	double res = (double)strtod(&number[0], NULL); //strtod works (float and double)
//	res *= 1e6;
//	printf("res=%d\n", (int)res);
//	printf("res as float=%f\n", res);

//	//int test2 = (int)atof(number);
//	sscanf(number, "%f", &test);
//	printf("test2=%d\n", (int)test);
//	printf("test2 as float=%f\n", test);
//    return 0;
    usleep(100000);

//    //read arguments
////    char * device = argv[1];
//    char * device = "/dev/ttyS1";
//
//    int buffer_size = 1000;
//    nmeaINFO * info = malloc(sizeof(nmeaINFO));
//
//    //open port (baud rate is set in defconfig file)
//	int fd = open_port(device);
//	if(fd != -1)
//	{
//		printf("port opened: %s\n", device);
//	}
//	else
//	{
//		printf("Could not open port\n");
//		sleep(1);
//		return 0;
//	}
//
//	//gps parser (nmea)
//	nmeaPARSER parser;
//	nmea_parser_init(&parser);
//	nmea_zero_INFO(info);
//	double lat_dec = 0;
//	double lon_dec = 0;

//
//
//	while(1)
//	{

//		char * gps_rx_buffer = malloc(buffer_size*sizeof(char)); //TODO: move out of loop
//
//		//get gps data into info
//		read_gps_nmea(fd, gps_rx_buffer, buffer_size, info, &parser);
//
//		//convert latitude longitude
////		lat_dec = nmea_ndeg2degree(info->lat);
////		lon_dec = nmea_ndeg2degree(info->lon);
//		lat_dec = info->lat;
//		lat_dec = info->lat;
//
//		//Test output
//		printf("Lat:%f, Lon:%f,Elev:%f, Sig:%d, Fix:%d, Inview:%d\n", lat_dec, lon_dec, info->elv, info->sig, info->fix, info->satinfo.inview);
//
//		free( gps_rx_buffer );


		const char *buff[] = {
		        "$GPRMC,173843,A,3349.896,N,11808.521,W,000.0,360.0,230108,013.4,E*69\r\n",
		        "$GPGGA,111609.14,5001.27,N,3613.06,E,3,08,0.0,10.2,M,0.0,M,0.0,0000*70\r\n",
		        "$GPGSV,2,1,08,01,05,005,80,02,05,050,80,03,05,095,80,04,05,140,80*7f\r\n",
		        "$GPGSV,2,2,08,05,05,185,80,06,05,230,80,07,05,275,80,08,05,320,80*71\r\n",
		        "$GPGSA,A,3,01,02,03,04,05,06,07,08,00,00,00,00,0.0,0.0,0.0*3a\r\n",
		        "$GPRMC,111609.14,A,5001.27,N,3613.06,E,11.2,0.0,261206,0.0,E*50\r\n",
		        "$GPVTG,217.5,T,208.8,M,000.00,N,000.01,K*4C\r\n"
		    };

		    int it;
		    nmeaINFO info;
		    nmeaPARSER parser;

		    nmea_zero_INFO(&info);
		    nmea_parser_init(&parser);

		    for(it = 0; it < 6; ++it)
		    {
		        nmea_parse(&parser, buff[it], (int)strlen(buff[it]), &info);
		         printf("gpstest: info sig: %d\n", info.sig); //TODO: removeme
		         printf("gpstest: info sig: %d\n", info.sig); //TODO: removeme
		         printf("Lat:%d, Lon:%d, Elev:%d, Sig:%d, Fix:%d, Inview:%d\n", (int)info.lat, (int)info.lon, (int)info.elv, info.sig, info.fix, info.satinfo.inview);
		    }

		    nmea_parser_destroy(&parser);


//	}



//	free(info);
//
//	//close port
//	close_port(fd);


    return 0;
}


