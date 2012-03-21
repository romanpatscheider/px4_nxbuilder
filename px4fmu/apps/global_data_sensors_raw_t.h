//Structure for storage of shared variables

#ifndef GLOBAL_DATA_SENSORS_RAW_T_H_ //adjust this line!
#define GLOBAL_DATA_SENSORS_RAW_T_H_ //adjust this line!

#include "global_data.h"

typedef struct
{
	/* Struct which stores the access configuration, this is the same for all shared structs */

	access_conf_t access_conf; //don't remove this line!

	/* use of a counter and timestamp recommended (but not necessary) */

//	uint16_t counter; //incremented by the writing thread everytime new data is stored
	uint32_t timestamp; //in milliseconds

	/* Actual data, this is specific to the type of data which is stored in this struct */

	//***** Start: Add your variables here *****

	int16_t	gyro_raw[3]; // l3gd20
	uint16_t gyro_raw_counter;
	int16_t	accelerometer_raw[3]; // bma180
	uint16_t accelerometer_raw_counter;
	int16_t	magnetometer_raw[3]; //hmc5883l
	uint16_t magnetometer_raw_counter;
	uint32_t pressure_sensor_raw[2]; //ms5611
	uint16_t pressure_sensor_raw_counter;


	//*****END: Add your variables here *****

} global_data_sensors_raw_t; //adjust this line!

extern global_data_sensors_raw_t global_data_sensors_raw; //adjust this line!

#endif
