/* Structure for storage of shared variables */

/* global_data_attitude the filtered and fused attitude is stored. (No raw sensor values!) */

#ifndef GLOBAL_DATA_ATTITUDE_T_H_ //adjust this line!
#define GLOBAL_DATA_ATTITUDE_T_H_ //adjust this line!

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

	/* This is similar to the mavlink message ATTITUDE */

	int32_t lat; //Latitude, expressed as * 1E7
	float roll; //Roll angle (rad)
	float pitch; //Pitch angle (rad)
	float yaw; //Yaw angle (rad)
	float rollspeed; //Roll angular speed (rad/s)
	float pitchpeed; //Pitch angular speed (rad/s)
	float yawspeed; //Yaw angular speed (rad/s)

	//*****END: Add your variables here *****

} global_data_attitude_t; //adjust this line!

extern global_data_attitude_t global_data_attitude; //adjust this line!

#endif
