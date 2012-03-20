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
//    pthread_create (&accelerometer_thread, NULL, accelerometer_loop, NULL);
//    pthread_create (&magnetometer_thread, NULL, magnetometer_loop, NULL);

    //wait for threads to complete:
    pthread_join(gyro_accelerometer_thread, NULL);
//    pthread_join(receive_thread, NULL);
//    pthread_join(gps_receive_thread, NULL);




    return 0;
}


static void *gyro_accelerometer_loop(void * arg)
{
	int		gyro_fd;
	int		accelerometer_fd;
	int		ret_gyro;
	int		ret_accelerometer;

	/* open gyro */
	gyro_fd = open("/dev/l3gd20", O_RDONLY | O_NONBLOCK); //
	if (gyro_fd < 0) {
		printf("L3GD20: open fail\n");
//		return ERROR;
	}
	/* open accelerometer */
	accelerometer_fd = open("/dev/bma180", O_RDONLY);
	if (accelerometer_fd < 0) {
		printf("\tbma180: open fail\n");
//		return ERROR;
	}
	usleep(100000);

	/* configure gyro */
	if (ioctl(gyro_fd, L3GD20_SETRATE, L3GD20_RATE_760HZ_LP_50HZ) || ioctl(gyro_fd, L3GD20_SETRANGE, L3GD20_RANGE_500DPS))
	{
		printf("L3GD20: ioctl fail\n");
		return ERROR;
	} else {
		printf("\tgyro configured..\n");
	}

	/* configure accelerometer */ // to be used or not to be used?
	//	if (ioctl(fd, LIS331_SETRATE, LIS331_RATE_50Hz) ||
	//	    ioctl(fd, LIS331_SETRANGE, LIS331_RANGE_4G)) {
	//
	//		printf("BMA180: ioctl fail\n");
	//		return ERROR;
	//	}

	usleep(100000);
	int counter = 0;

    while(1)
    {
    	/* lock, read, and unlock */
    	global_data_lock(&global_data_sensors_raw.access_conf);
    	ret_gyro = read(gyro_fd, global_data_sensors_raw.gyro_raw, sizeof(global_data_sensors_raw.gyro_raw));
    	ret_accelerometer = read(accelerometer_fd, global_data_sensors_raw.accelerometer_raw, sizeof(global_data_sensors_raw.accelerometer_raw));
    	global_data_unlock(&global_data_sensors_raw.access_conf);

    	/* check gyro result */
    	if (ret_gyro != sizeof(global_data_sensors_raw.gyro_raw)) {
    		printf("\tl3gd20: read1 fail (%d should have been %d)\n", ret_gyro, sizeof(global_data_sensors_raw.gyro_raw));
    	} else {
    		if(counter%20==0)
    		{
    			printf("\tl3gd20 values #1: x:%d\ty:%d\tz:%d\n", global_data_sensors_raw.gyro_raw[0], global_data_sensors_raw.gyro_raw[1], global_data_sensors_raw.gyro_raw[2]);
    		}
    	}

    	/* check accelerometer result */
    	if (ret_accelerometer != sizeof(global_data_sensors_raw.accelerometer_raw)) {
    		printf("\tbma180: read2 fail (%d)\n", ret_accelerometer);
    	} else {
    		if(counter%20==0)
    		{
    			printf("\tbma180 values: x:%d\ty:%d\tz:%d\n", global_data_sensors_raw.accelerometer_raw[0], global_data_sensors_raw.accelerometer_raw[1], global_data_sensors_raw.accelerometer_raw[2]);
    		}
    	}



    	counter++;
    	usleep(2000);
    }

	close(gyro_fd);
	close(accelerometer_fd);
}

