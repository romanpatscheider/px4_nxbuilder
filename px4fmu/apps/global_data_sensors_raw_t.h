/* Structure for storage of shared variables */

/* global_data_sensors_raw stores the raw sensor values for gyro, accelerometer, magnetometer and pressure sensor */

#ifndef GLOBAL_DATA_SENSORS_RAW_T_H_ //adjust this line!
#define GLOBAL_DATA_SENSORS_RAW_T_H_ //adjust this line!

#include "global_data.h"

typedef struct
{
	/* Struct which stores the access configuration, this is the same for all shared structs */

	access_conf_t access_conf; //don't remove this line!

	/* global_data_sensors_raw has individual counters for each sensor --> no counter and timestamp defined here */

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
