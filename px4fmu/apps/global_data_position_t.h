/* Structure for storage of shared variables */

/* global_data_position the filtered and fused position is stored. (No raw sensor values!) */

#ifndef GLOBAL_DATA_POSITION_T_H_ //adjust this line!
#define GLOBAL_DATA_POSITION_T_H_ //adjust this line!

#include "global_data.h"

typedef struct
{
	/* Struct which stores the access configuration, this is the same for all shared structs */

	access_conf_t access_conf; //don't remove this line!

	/* use of a counter and timestamp recommended (but not necessary) */

	uint16_t counter; //incremented by the writing thread everytime new data is stored
	uint32_t timestamp; //in milliseconds

	/* Actual data, this is specific to the type of data which is stored in this struct */

	//***** Start: Add your variables here *****

	/* This is similar to the mavlink message GLOBAL_POSITION_INT */

	int32_t lat; //Latitude, expressed as * 1E7
	int32_t lon; //Longitude, expressed as * 1E7
	int32_t alt; //Altitude in meters, expressed as * 1000 (millimeters), above MSL
	int32_t relative_alt; //Altitude above ground in meters, expressed as * 1000 (millimeters)
	int16_t vx; //Ground X Speed (Latitude), expressed as m/s * 100
	int16_t vy; //Ground Y Speed (Latitude), expressed as m/s * 100
	int16_t vz; //Ground Z Speed (Latitude), expressed as m/s * 100
	uint16_t hdg; //Compass heading in degrees * 100, 0.0..359.99 degrees. If unknown, set to: 65535

	//*****END: Add your variables here *****

} global_data_position_t; //adjust this line!

extern global_data_position_t global_data_position; //adjust this line!

#endif
