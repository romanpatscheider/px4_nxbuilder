/*
 * nmea_helper.h
 *
 *  Created on: Mar 15, 2012
 *      Author: thomasgubler
 */

#ifndef NMEA_H_
#define NMEA_H_

#include "nmealib/nmea/nmea.h"

int read_gps_nmea(int fd, char * gps_rx_buffer, int buffer_size, nmeaINFO * info, nmeaPARSER * parser);

/**
 * \brief Convert NDEG (NMEA degree) to fractional degree
 */
float ndeg2degree(float val);


#endif /* NMEA_H_ */
