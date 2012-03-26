/*
 * ardrone_control.h
 *
 *  Created on: Mar 23, 2012
 *      Author: thomasgubler
 */

#ifndef ARDRONE_CONTROL_H_
#define ARDRONE_CONTROL_H_

#include <nuttx/config.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <debug.h>
#include "../global_data_quad_motors_setpoint_t.h"
#include "../global_data_sensors_raw_t.h"
#include "../global_data_gps_t.h"
#include "../global_data.h"
#include <termios.h>
#include "ardrone_motor_control_new.h"


/****************************************************************************
 * Internal Definitions
 ****************************************************************************/

#define CONTROL_LOOP_USLEEP 5000

typedef struct {
	int16_t	gyro_raw[3]; // l3gd20
	int16_t	accelerometer_raw[3]; // bma180
	int16_t	magnetometer_raw[3]; //hmc5883l
	uint32_t pressure_sensor_raw[2]; //ms5611
} sensors_raw_t;

typedef struct  {
	uint16_t motor_front_nw; ///< Front motor in + configuration, front left motor in x configuration
	uint16_t motor_right_ne; ///< Right motor in + configuration, front right motor in x configuration
	uint16_t motor_back_se; ///< Back motor in + configuration, back right motor in x configuration
	uint16_t motor_left_sw; ///< Left motor in + configuration, back left motor in x configuration
	uint8_t target_system; ///< System ID of the system that should set these motor commands
} quad_motors_setpoint_t;


/****************************************************************************
 * Private Data
 ****************************************************************************/

/* Pthreads */
pthread_t control_thread;

/* Raw sensor values (last copy of the global values), The controller should access these */
sensors_raw_t sensors_raw;

/* Quad Motors Setpoint (last copy of the global values), The controller should access these*/
quad_motors_setpoint_t quad_motors_setpoint_desired;

/*File descriptors */
static int ardrone_write;
static int gpios;


/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* Copy raw sensor values from global memory to private variables */
void read_sensors_raw(void);

/* Copy quad_motors_setpoint values from global memory to private variables */
void read_quad_motors_setpoint(void);

/* Copy gps values from global memory to private variables (only the ones needed for the controller) */
void read_gps(void);

/* The controlloop which runs as a pthread */
static void *controlloop(void * arg);

void control_step(void);


#endif /* ARDRONE_CONTROL_H_ */
