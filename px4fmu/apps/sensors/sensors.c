/****************************************************************************
 * examples/hello/main.c
 *
 *   Copyright (C) 2008, 2011 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <spudmonkey@racsa.co.cr>
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

#include "sensors.h"


/****************************************************************************
 * Definitions
 ****************************************************************************/

/* Struct for storage of gps information and transmission to mavlink app  */
global_data_sensors_raw_t global_data_sensors_raw = {.access_conf.initialized = 0};


/****************************************************************************
 * user_start
 ****************************************************************************/

int sensors_main(int argc, char *argv[])
{
	// print text
	printf("Hello, Sensors!\n");
	usleep(100000);

	/* initialize shared data structures */
	global_data_init(&global_data_sensors_raw.access_conf);

	//create pthreads
	pthread_create (&gyro_accelerometer_thread, NULL, gyro_accelerometer_loop, NULL);
//	pthread_create (&magnetometer_thread, NULL, magnetometer_loop, NULL);
//	pthread_create (&pressure_sensor_thread, NULL, pressure_sensor_loop, NULL);

	//wait for threads to complete:
	pthread_join(gyro_accelerometer_thread, NULL);
//    pthread_join(magnetometer_thread, NULL);
//  pthread_join(pressure_sensor_thread, NULL);




    return 0;
}


static void *gyro_accelerometer_loop(void * arg)
{
	int	fd_gyro;
	int	fd_accelerometer;
	int	ret_gyro;
	int	ret_accelerometer;

	int16_t buf_gyro[3];
	int16_t buf_accelerometer[3];

//	int counter = 0; // only printf every 20iest
	uint8_t read_success = 0;

	/* open gyro */
	fd_gyro = open("/dev/l3gd20", O_RDONLY ); //| O_NONBLOCK
	if (fd_gyro < 0)
	{
		printf("L3GD20: open fail\n");
//		return ERROR;
	}
	/* open accelerometer */
	fd_accelerometer = open("/dev/bma180", O_RDONLY);
	if (fd_accelerometer < 0)
	{
		printf("\tbma180: open fail\n");
		usleep(1000000);
//		return ERROR;
	}


	/* configure gyro */
	if (ioctl(fd_gyro, L3GD20_SETRATE, L3GD20_RATE_760HZ_LP_50HZ) || ioctl(fd_gyro, L3GD20_SETRANGE, L3GD20_RANGE_500DPS))
	{
		printf("L3GD20: ioctl fail\n");
		usleep(1000000);
//		return ERROR;
	}
	else
	{
		printf("\tgyro configured..\n");
	}

	/* configure accelerometer */ // to be used or not to be used?
	//	if (ioctl(fd, LIS331_SETRATE, LIS331_RATE_50Hz) ||
	//	    ioctl(fd, LIS331_SETRANGE, LIS331_RANGE_4G)) {
	//
	//		printf("BMA180: ioctl fail\n");
	//		return ERROR;
	//	}

	usleep(10000);

	/* initiialize counter */
	global_data_sensors_raw.gyro_raw_counter = 0;
	global_data_sensors_raw.accelerometer_raw_counter = 0;


    while(1)
    {
    	read_success = 0;
    	/* try reading */
    	ret_gyro = read(fd_gyro,buf_gyro, sizeof(buf_gyro));
    	ret_accelerometer = read(fd_accelerometer, buf_accelerometer, sizeof(buf_accelerometer));

    	/* check gyro result */
    	if (ret_gyro != sizeof(buf_gyro))
    	{
//    		printf("\tl3gd20: read fail (%d should have been %d)\n", ret_gyro, sizeof(buf_gyro));
    	}
    	else
    	{
    		read_success = 1;

    		/* lock, increment counter, read, and unlock */
			global_data_lock(&global_data_sensors_raw.access_conf);
			memcpy(global_data_sensors_raw.gyro_raw, buf_gyro, sizeof(buf_gyro));
			global_data_sensors_raw.gyro_raw_counter++;
			global_data_unlock(&global_data_sensors_raw.access_conf);

			/* printf to debug, every 20iest time */
//			if(counter%20==0)
//    		{
//    			printf("\tl3gd20 values: x:%d\ty:%d\tz:%d\n", global_data_sensors_raw.gyro_raw[0], global_data_sensors_raw.gyro_raw[1], global_data_sensors_raw.gyro_raw[2]);
//    		}
    	}

    	/* check accelerometer result */
    	if (ret_accelerometer != sizeof(buf_accelerometer))
    	{
//    		printf("\tbma180: read fail (%d should have been %d)\n", ret_accelerometer, sizeof(buf_accelerometer));
    	}
    	else
    	{
    		read_success = 1;

    		/* lock, increment counter, read, and unlock */
			global_data_lock(&global_data_sensors_raw.access_conf);
			memcpy(global_data_sensors_raw.accelerometer_raw, buf_accelerometer, sizeof(buf_accelerometer));
			global_data_sensors_raw.accelerometer_raw_counter++;
			global_data_unlock(&global_data_sensors_raw.access_conf);

//    		if(counter%20==0)
//    		{
//    			printf("\tbma180 values: x:%d\ty:%d\tz:%d\n", global_data_sensors_raw.accelerometer_raw[0], global_data_sensors_raw.accelerometer_raw[1], global_data_sensors_raw.accelerometer_raw[2]);
//    		}
    	}

    	if(1==read_success) // broadcast if at least one new sensor reading is available
    	{
    		global_data_broadcast(&global_data_sensors_raw.access_conf);
    	}

//    	counter++;
    	usleep(2000); // 500 Hz !?
    }

	close(fd_gyro);
	close(fd_accelerometer);
}

static void *magnetometer_loop(void * arg)
{
	int	fd_magnetometer;
	int	ret_magnetometer;
	int16_t buf_magnetometer[3];

	int counter = 0; // only printf every 20iest

	printf("\tbefore 10seconds of sleep\n");
	sleep(10); // does not work

	/* open magnetometer */
	fd_magnetometer = open("/dev/hmc5883l", O_RDONLY);
	if (fd_magnetometer < 0)
	{
		printf("\thmc5883l: open fail\n");
//		return ERROR;
	}
	usleep(1000000); // does not work, something is wrong

	/* configure magnetometer */ // to be used or not to be used?
	//	if (ioctl(fd, LIS331_SETRATE, LIS331_RATE_50Hz) ||
	//	    ioctl(fd, LIS331_SETRANGE, LIS331_RANGE_4G)) {
	//
	//		printf("BMA180: ioctl fail\n");
	//		return ERROR;
	//	}

	while(1)
	{
		/* try reading */
		ret_magnetometer = read(fd_magnetometer, buf_magnetometer, sizeof(buf_magnetometer));

		if (ret_magnetometer != sizeof(buf_magnetometer))
		{
//			printf("\thmc5883l: read fail (%d should have been %d)\n", ret_magnetometer,  sizeof(buf_magnetometer));
//			return ERROR;
		}
		else
		{
			/* lock, read, and unlock */
			global_data_lock(&global_data_sensors_raw.access_conf);
			memcpy(global_data_sensors_raw.magnetometer_raw, buf_magnetometer, sizeof(buf_magnetometer));
			global_data_sensors_raw.magnetometer_raw_counter++;
			global_data_unlock(&global_data_sensors_raw.access_conf);
			global_data_broadcast(&global_data_sensors_raw.access_conf);

//			if(counter%10==0)
//			{
//				printf("\thmc5883l values: x:%d\ty:%d\tz:%d\n", global_data_sensors_raw.magnetometer_raw[0], global_data_sensors_raw.magnetometer_raw[1], global_data_sensors_raw.magnetometer_raw[2]);
//			}
		}

		counter++;
		usleep(50000); // 50 Hz !?
	}
}


static void *pressure_sensor_loop(void * arg)
{
	int	fd_pressure_sensor;
	int	ret_pressure_sensor;
	int16_t buf_pressure_sensor[2];

	int counter = 0; // only printf every 20iest

	/* open pressure sensor */
	fd_pressure_sensor = open("/dev/ms5611", O_RDONLY);
	if (fd_pressure_sensor < 0)
	{
		printf("\tms5611: open fail\n");
//		return ERROR;
	}

	/* configure pressure sensor */ // to be used or not to be used?
//	if (ioctl(fd, LIS331_SETRATE, LIS331_RATE_50Hz) ||
//	    ioctl(fd, LIS331_SETRANGE, LIS331_RANGE_4G)) {
//
//		printf("BMA180: ioctl fail\n");
//		return ERROR;
//	}

	while(1)
	{
		/* try reading */
		ret_pressure_sensor = read(fd_pressure_sensor, buf_pressure_sensor, sizeof(buf_pressure_sensor));


		if (ret_pressure_sensor != sizeof(buf_pressure_sensor))
		{
//			printf("\tms5611: read fail (%d should have been %d)\n", ret_pressure_sensor, sizeof(buf_pressure_sensor));
//			return ERROR;
		}
		else
		{
			/* lock, read, and unlock */
			global_data_lock(&global_data_sensors_raw.access_conf);
			memcpy(global_data_sensors_raw.pressure_sensor_raw, buf_pressure_sensor, sizeof(buf_pressure_sensor));
			global_data_sensors_raw.pressure_sensor_raw_counter++;
			global_data_unlock(&global_data_sensors_raw.access_conf);
			global_data_broadcast(&global_data_sensors_raw.access_conf);

//			if(counter%20==0)
//			{
//				printf("\tms5611 values: pressure raw:%d\ttemp raw:%d\n", global_data_sensors_raw.pressure_sensor_raw[0], global_data_sensors_raw.pressure_sensor_raw[1]);
//			}
		}

//		counter++;
		usleep(20000); // 50 Hz !?
	}
}

