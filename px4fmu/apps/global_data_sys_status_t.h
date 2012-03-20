//Structure for storage of shared variables

#ifndef GLOBAL_DATA_SYS_STATUS_T_H_ //adjust this line!
#define GLOBAL_DATA_SYS_STATUS_T_H_ //adjust this line!

#include "global_data.h"

typedef struct
{
	/* Struct which stores the access configuration, this is the same for all shared structs */

	access_conf_t access_conf; //don't remove this line!

	/* Actual data, this is specific to the type of data which is stored in this struct */

	//***** Start: Add your variables here *****
	uint32_t onboard_control_sensors_present;
	uint32_t onboard_control_sensors_enabled;
	uint32_t onboard_control_sensors_health;
	uint16_t load;
	uint16_t voltage_battery;
	int16_t current_battery;
	int8_t battery_remaining;
	uint16_t drop_rate_comm;
	uint16_t errors_comm;
	uint16_t errors_count1;
	uint16_t errors_count2;
	uint16_t errors_count3;
	uint16_t errors_count4;

	//*****END: Add your variables here *****

} global_data_sys_status_t; //adjust this line!

extern global_data_sys_status_t global_data_sys_status; //adjust this line!

#endif
