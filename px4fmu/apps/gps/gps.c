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
gps_bin_ubx_state_t * ubx_state;

/****************************************************************************
 * Pthread loops
 ****************************************************************************/


static void *ubx_watchdog_loop(void * arg) //runs as a pthread and listens to uart1 ("/dev/ttyS0")
{
	/* Retrieve file descriptor */
	int fd = *((int *)arg);

	while(1)
	{
		/* if some values are to old reconfigure gps */
		int i;
		pthread_mutex_lock(&ubx_mutex);
		bool all_okay = true;
		uint64_t timestamp_now = global_data_get_timestamp_milliseconds();
		for(i = 0; i < UBX_NO_OF_MESSAGES; i++)
		{
			if(timestamp_now - ubx_state->last_message_timestamps[i] > GPS_WATCHDOG_CRITICAL_TIME_MILLISECONDS)
			{
//				printf("Warning: GPS ubx message %d not received for a long time\n", i);
				all_okay = false;
			}
		}
		pthread_mutex_unlock(&ubx_mutex);

		if(!all_okay)
		{
			/* gps error */
			printf("GPS Watchdog detected gps not running or having problems\n");

			global_data_lock(&global_data_sys_status.access_conf);
			global_data_sys_status.onboard_control_sensors_present |= 1 << 5;//TODO: write wrapper for bitmask
			global_data_sys_status.onboard_control_sensors_enabled |= 1 << 5;
			global_data_sys_status.onboard_control_sensors_health &= ~(1 << 5);
			global_data_sys_status.counter++;
			global_data_sys_status.timestamp = global_data_get_timestamp_milliseconds();
			global_data_unlock(&global_data_sys_status.access_conf);

			/* trying to reconfigure the gps configuration */
			configure_gps_ubx(fd);
			fflush(stdout);
			sleep(1);
			printf("sleep finished\n");

	//		int killres = pthread_kill(&ubx_thread, SIGKILL);
	//		printf("killres=%d",killres);
	//		sleep(1);
	//		pthread_create (&ubx_thread, NULL, ubx_loop, (void *)&fd);
		}
		else
		{
			/* gps healthy */

			global_data_lock(&global_data_sys_status.access_conf);
			global_data_sys_status.onboard_control_sensors_present |= 1 << 5;//TODO: write wrapper for bitmask
			global_data_sys_status.onboard_control_sensors_enabled |= 1 << 5;
			global_data_sys_status.onboard_control_sensors_health |= 1 << 5;
			global_data_sys_status.counter++;
			global_data_sys_status.timestamp = global_data_get_timestamp_milliseconds();
			global_data_unlock(&global_data_sys_status.access_conf);
		}

		usleep(GPS_WATCHDOG_WAIT_TIME_MICROSECONDS);
	}
}


static void *ubx_loop(void * arg) //runs as a pthread and listens to uart1 ("/dev/ttyS0")
{

	/* Initialize ubx state */
//	ubx_state = malloc(sizeof(gps_bin_ubx_state_t));
//	ubx_decode_init();


		/* Retrieve file descriptor */
		int fd = *((int *)arg);

		/* Initialize gps stuff */
	    int buffer_size = 1000;
//	    nmeaINFO * info = malloc(sizeof(nmeaINFO));
	    bool health_set = false;
		char * gps_rx_buffer = malloc(buffer_size*sizeof(char));

		/* gps parser (nmea) */
//		nmeaPARSER parser;
//		nmea_parser_init(&parser);
//		nmea_zero_INFO(info);
//		float lat_dec = 0;
//		float lon_dec = 0;

		/* custom (mediatek custom) */
//		gps_bin_custom_state_t * mtk_state = malloc(sizeof(gps_bin_custom_state_t));
//		mtk_decode_init(mtk_state);
//		mtk_state->print_errors = false;



//		if( !strcmp("custom",mode) )
//		{
//			printf("\t%s: custom mode\n",APPNAME);
//	//		configure_gps_custom(fd); // ?
//
//
//	//		while(1)
//	//
//	//		if (configure_gps_ubx(fd, ubx_state) != 0)
//	//
//	//		{
//	//			//TODO: execute custom read
//	//		}
//
//		}
//		else if( !strcmp("ubx",mode) )
//		{
	    	printf("\t%s: ubx mode\n",APPNAME);
	    	//set parameters for ubx


	    	//ubx state
			gps_bin_ubx_state_t * ubx_state = malloc(sizeof(gps_bin_ubx_state_t));
		   	printf("%s: ubx_state created\n",APPNAME);
			ubx_decode_init();
			ubx_state->print_errors = false;

	    	int config_not_finished = 1; //is set to 0 as soon as all configurations are completed
	    	bool configured = false;

	    	/* set parameters for ubx */

			if (configure_gps_ubx(fd) != 0)
			{
				printf("Configuration of gps module to ubx failed\n");

				/* Write shared variable sys_status */

				global_data_lock(&global_data_sys_status.access_conf);
				global_data_sys_status.onboard_control_sensors_present |= 1 << 5;//TODO: write wrapper for bitmask
				global_data_sys_status.onboard_control_sensors_enabled &= ~(1 << 5);
				global_data_sys_status.onboard_control_sensors_health &= ~(1 << 5);
				global_data_sys_status.counter++;
				global_data_sys_status.timestamp = global_data_get_timestamp_milliseconds();
				global_data_unlock(&global_data_sys_status.access_conf);
			}
			else
			{
				printf("Configuration of gps module to ubx successful\n");


				/* Write shared variable sys_status */

				global_data_lock(&global_data_sys_status.access_conf);
				global_data_sys_status.onboard_control_sensors_present |= 1 << 5;//TODO: write wrapper for bitmask
				global_data_sys_status.onboard_control_sensors_enabled |= 1 << 5;
				global_data_sys_status.counter++;
				global_data_sys_status.timestamp = global_data_get_timestamp_milliseconds();
				global_data_unlock(&global_data_sys_status.access_conf);
			}

			/* Inform the other processes that there is new gps data available */
			global_data_broadcast(&global_data_sys_status.access_conf);





	    	while(1)
	    	{




	    		read_gps_ubx(fd, gps_rx_buffer, buffer_size, &ubx_mutex);



	//
	//    		/* set health to true if config is finished after certain time (only executed once) */
	//			if(config_not_finished == 0 && counter >= GPS_COUNTER_LIMIT && false == health_set)
	//			{
	//				global_data_lock(&global_data_sys_status.access_conf);
	//				global_data_sys_status.onboard_control_sensors_health |= 1 << 5;
	//				global_data_sys_status.counter++;
	//				global_data_sys_status.timestamp = global_data_get_timestamp_milliseconds();
	//				global_data_unlock(&global_data_sys_status.access_conf);
	//
	//				health_set = true;
	//
	//				printf("%s: gps configuration successful\n",APPNAME);
	//			}
	//			else if (config_not_finished != 0 && counter >= GPS_COUNTER_LIMIT)
	//			{
	//				//reset state machine
	//				ubx_decode_init(ubx_state);
	//				ubx_state->print_errors = false;
	//				ubx_state->last_config_message_sent = UBX_CONFIGURE_NOTHING;
	//				ubx_state->last_ack_message_received = UBX_CONFIGURE_NOTHING;
	//				ubx_state->last_config_failed = false;
	//		    	config_not_finished = 1; //is set to 0 as soon as all configurations are completed
	//		    	bool configured = false;
	//		    	counter = 0;
	//
	//
	//				printf("%s: gps configuration probably failed, exiting now\n",APPNAME);
	////				sleep(1);
	////				return 0;
	//			}

	    		/* Inform the other processes that there is new gps data available */
	    		global_data_broadcast(&global_data_gps.access_conf);
	    	}

//		}
//		else if( !strcmp("nmea",mode) )
//		{
//			printf("\t%s: nmea mode\n",APPNAME);
//		}


	//	while(1)
	//	{
	//		if( !strcmp("nmea",mode) ) //TODO: implement use of global_data-gps also in nmea mode (currently only in ubx mode)
	//		{
	//			printf("\t%s: nmea mode\n");
	//			//get gps data into info
	//			read_gps_nmea(fd, gps_rx_buffer, buffer_size, info, &parser);
	//			//convert latitude longitude
	//			lat_dec = ndeg2degree(info->lat);
	//			lon_dec = ndeg2degree(info->lon);
	//
	//			//Test output
	////			printf("Lat:%d, Lon:%d,Elev:%d, Sig:%d, Fix:%d, Inview:%d\n", (int)(lat_dec*1e6), (int)(lon_dec*1e6), (int)(info->elv*1e6), info->sig, info->fix, info->satinfo.inview);
	//		}
	////		else if ( !strcmp("ubx",mode) )
	////		{
	////
	////			//get gps data into info
	////			read_gps_ubx(fd, gps_rx_buffer, buffer_size, ubx_state); //TODO: atm using the info struct from the nmea library, once the gps/mavlink structures are clear--> use own struct
	//////			lat_dec = info->lat;
	//////			lon_dec = info->lon;
	//////			printf("Lat:%d, Lon:%d,Elev:%d, Sig:%d, Fix:%d, Inuse:%d, PDOP:%d\n", (int)(lat_dec*1e6), (int)(lon_dec*1e6), (int)(info->elv*1e6), info->sig, info->fix, info->satinfo.inuse, (int)(info->PDOP*1e4));
	////		}
	//		else if	( !strcmp("custom",mode) ) //TODO: implement use of global_data-gps also in custom mode (currently only in ubx mode)
	//		{
	//			//info is used as storage of the gps information. lat lon are already in fractional degree format * 10e6
	//			//see custom.h/mtk_parse for more information on which variables are stored in info
	//			nmea_zero_INFO(info);
	//
	//			//get gps data into info
	//			read_gps_custom(fd, gps_rx_buffer, buffer_size, info, mtk_state);
	//
	//			//Test output
	////			printf("Lat:%d, Lon:%d,Elev:%d, Sig:%d, Fix:%d, Inview:%d\n", (int)(info->lat), (int)info->lon, (int)info->elv, info->sig, info->fix, info->satinfo.inview);
	//
	//		}
	//
	//
	//
	//
	//
	//	}

		free(gps_rx_buffer);
//		free(info);


		//close port
		close_port(fd);

		//destroy gps parser
//		nmea_parser_destroy(&parser);

	    return 0;

}


/****************************************************************************
 * user_start
 ****************************************************************************/

int gps_main(int argc, char *argv[])
{
    // print text
    printf("Hello, %s!\n",APPNAME);
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
		printf("\t%s: Port opened: %s\n", APPNAME, device);
	}
	else
	{
		printf("\t%s: Could not open port, exiting gps app!\n", APPNAME);
		sleep(1);
		return 0;
	}

//TODO: add mode if here (reads from arguments and configuration, watchdog will look for changes in mode configuration while running (?))


	/* Init mutex for datasharing between ubx gps reading thrad (ubx_thread) and  ubx_watchdog thread*/
	pthread_mutex_init(&ubx_mutex, NULL);

	/* Initialize ubx state */
	ubx_state = malloc(sizeof(gps_bin_ubx_state_t));
	ubx_decode_init();


    /* create pthreads */
    pthread_create (&ubx_thread, NULL, ubx_loop, (void *)&fd);


    /* wait before starting watchdog */

    fflush(stdout);
    sleep(2);
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



