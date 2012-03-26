/****************************************************************************
 * px4/sensors/tests_main.c
 *
 *   Copyright (C) 2012 Michael Smith. All rights reserved.
 *   Authors: Michael Smith <DrZiplok@me.com>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name NuttX nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include "ardrone_control.h"




/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

void read_sensors_raw(void)
{

	global_data_trylock(&global_data_sensors_raw.access_conf);//TODO: check if trylock is the right choice, maybe only lock?

	memcpy(sensors_raw.gyro_raw, global_data_sensors_raw.gyro_raw, sizeof(sensors_raw.gyro_raw));
	memcpy(sensors_raw.accelerometer_raw, global_data_sensors_raw.accelerometer_raw, sizeof(sensors_raw.accelerometer_raw));
	memcpy(sensors_raw.magnetometer_raw, global_data_sensors_raw.magnetometer_raw, sizeof(sensors_raw.magnetometer_raw));
	memcpy(sensors_raw.pressure_sensor_raw, global_data_sensors_raw.pressure_sensor_raw, sizeof(sensors_raw.pressure_sensor_raw));

	global_data_unlock(&global_data_sensors_raw.access_conf);
}

void read_quad_motors_setpoint(void)
{

	global_data_trylock(&global_data_quad_motors_setpoint.access_conf);//TODO: check if trylock is the right choice, maybe only lock?

	quad_motors_setpoint_desired.motor_front_nw = global_data_quad_motors_setpoint.motor_front_nw;
	quad_motors_setpoint_desired.motor_right_ne = global_data_quad_motors_setpoint.motor_right_ne;
	quad_motors_setpoint_desired.motor_back_se = global_data_quad_motors_setpoint.motor_back_se;
	quad_motors_setpoint_desired.motor_left_sw = global_data_quad_motors_setpoint.motor_left_sw;

	global_data_unlock(&global_data_quad_motors_setpoint.access_conf);
}

void read_gps(void)
{
			global_data_trylock(&global_data_gps.access_conf);
			//TODO: determine which gps data the controller needs and copy these here...
			global_data_unlock(&global_data_gps.access_conf);
}

static void *controlloop(void * arg)
{
	while(1)
	{
		/* Get sensor data */

		read_sensors_raw();
		read_quad_motors_setpoint();
//		read_gps(); //TODO: uncomment once gps is used by the controller, read_gps is not yet fully implemented!

		/* Control */
		//TODO: implement a fancy controller here

		sleep(CONTROL_LOOP_USLEEP);
	}
}


/****************************************************************************
 * Name: ardrone_control
 ****************************************************************************/

int ardrone_control_main(int argc, char *argv[])
{
    // print text
    printf("Hello, Ardrone Control!\n");
    usleep(100000);

    /* initialize shared data structures */
	global_data_init(&global_data_gps.access_conf);
	global_data_init(&global_data_sensors_raw.access_conf);
	global_data_init(&global_data_quad_motors_setpoint.access_conf);

	//create pthreads
	pthread_create (&control_thread, NULL, controlloop, NULL);


	//wait for threads to complete:
	pthread_join(control_thread, NULL);



	return 0;
}

