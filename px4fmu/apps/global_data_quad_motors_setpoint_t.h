/* Structure for storage of shared variables */

/* global_data_gps  stores the gps sensor values */

#ifndef GLOBAL_DATA_QUAD_MOTORS_SETPOINT_T_H_ //adjust this line!
#define GLOBAL_DATA_QUAD_MOTORS_SETPOINT_T_H_ //adjust this line!

#include "global_data.h"

typedef struct
{
	/* Struct which stores the access configuration, this is the same for all shared structs */

	access_conf_t access_conf; //don't remove this line!

	/* use of a counter and timestamp recommended (but not necessary) */

	uint16_t counter; //incremented by the writing thread everytime new data is stored
	uint64_t timestamp; //in milliseconds since system start, is set whenever the writing thread stores new data

	/* Actual data, this is specific to the type of data which is stored in this struct */

	//***** Start: Add your variables here *****

	 uint16_t motor_front_nw; ///< Front motor in + configuration, front left motor in x configuration
	 uint16_t motor_right_ne; ///< Right motor in + configuration, front right motor in x configuration
	 uint16_t motor_back_se; ///< Back motor in + configuration, back right motor in x configuration
	 uint16_t motor_left_sw; ///< Left motor in + configuration, back left motor in x configuration

	//*****END: Add your variables here *****

} global_data_quad_motors_setpoint_t; //adjust this line!

extern global_data_quad_motors_setpoint_t global_data_quad_motors_setpoint; //adjust this line!

#endif
