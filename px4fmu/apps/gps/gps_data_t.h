/*
 * gps_data_t.h
 *
 *  Created on: Mar 16, 2012
 *      Author: thomasgubler
 */

#ifndef GPS_DATA_T_H_
#define GPS_DATA_T_H_

#include "v1.0/common/mavlink.h"

typedef struct //TODO: make me global
	{
		mavlink_gps_raw_int_t gps_raw_int_data;
		mavlink_gps_status_t gps_status_data;

	} __attribute__((__packed__)) gps_data_t;

#endif /* GPS_DATA_T_H_ */
