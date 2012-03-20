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
#include "../mq_config.h"
#include "../global_data_gps_t.h" //for storage of gps information
#include "../global_data_sys_status_t.h"
#include "../global_data.h"
#include <termios.h>



int open_port(char * port);


void close_port(int fd);


#endif /* GPS_H_ */
