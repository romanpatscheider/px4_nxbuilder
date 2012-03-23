/*
 * gps.h
 *
 *  Created on: Mar 8, 2012
 *      Author: thomasgubler
 */

#ifndef GPS_H_
#define GPS_H_
/****************************************************************************
 * Included Files
 ****************************************************************************/


#include <nuttx/config.h>
#include <stdlib.h>
#include <stdio.h>
#include <fcntl.h>
#include "nmealib/nmea/nmea.h" // the nmea library
#include "nmea_helper.h" //header files for interacting with the nmea library
#include "custom.h" //header files for the custom protocol for the mediatek diydrones chip
#include "ubx.h" //header files for the ubx protocol
#include <mqueue.h>
#include "../global_data_gps_t.h" //for storage of gps information
#include "../global_data_sys_status_t.h"
#include "../global_data.h"
#include <termios.h>
#include <signal.h>

#define APPNAME "gps"
#define GPS_WATCHDOG_CRITICAL_TIME_MILLISECONDS 2000
#define GPS_WATCHDOG_WAIT_TIME_MICROSECONDS 300000


/* Threads */
pthread_t ubx_thread;
pthread_t ubx_watchdog_thread;

/* Mutexes */
pthread_mutex_t ubx_mutex;

gps_bin_ubx_state_t * ubx_state;

int gps_fd;



//extern gps_bin_ubx_state_t * ubx_state;

int open_port(char * port);

void close_port(int fd);


#endif /* GPS_H_ */
