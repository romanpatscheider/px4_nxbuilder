/****************************************************************************
 * apps/reboot.c
 *
 *   Copyright (C) 2012 Lorenz Meier. All rights reserved.
 *   Author: Lorenz Meier <lm@inf.ethz.ch>
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


#include <nuttx/config.h>
#include <stdio.h>
#include <unistd.h>
#include <sys/time.h>
#include <stdbool.h>
#include <fcntl.h>

#include <arch/board/drv_l3gd20.h>
#include <arch/board/drv_bma180.h>
#include <arch/board/drv_hmc5883l.h>

#include "attitude_bm.h"

/****************************************************************************
 * Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Data
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * user_start
 ****************************************************************************/

/* gyro x, y, z raw values */
static int16_t	gyro_raw[3] = {0, 0, 0};
/* gyro x, y, z metric values in rad/s */
static float gyro_rad_s[3] = {0.0f, 0.0f, 0.0f};


/* acc x, y, z raw values */
static int16_t	acc_raw[3] = {0, 0, 0};
/* acc x, y, z metric values in g */
static float acc_g[3] = {0.0f, 0.0f, 0.0f};

/* mag x, y, z raw values */
static int16_t	mag_raw[3] = {0, 0, 0};
/* mag x, y, z raw offsets */
static int16_t	mag_offset[3] = {0, 0, 0};
/* mag x, y, z metric values in Gauss */
static float mag_ga[3] = {0.0f, 0.0f, 0.0f};


/* file descriptors */
static int gyro;  // L3GD20 gyroscope
static int acc;   // BMA180 accelerometer
static int mag;   // HMC5883L magnetometer
static int baro;  // MS5611-03BA barometer
static int mpu;   // MPU-6000 combined gyroscope and accelerometer

static uint64_t loop_interval = 2000;	// = 500Hz, loop interval in microseconds

static int attitude_estimator_bm_sensor_init(void);
static int attitude_estimator_bm_read_sensors(void);

static int attitude_estimator_bm_sensor_init()
{
	gyro = open("/dev/l3gd20", O_RDONLY);
	if (gyro < 0) {
		printf("L3GD20: open fail\n");
		return ERROR;
	}

	if (ioctl(gyro, L3GD20_SETRATE, L3GD20_RATE_760HZ_LP_50HZ) != 0 ||
			ioctl(gyro, L3GD20_SETRANGE, L3GD20_RANGE_500DPS) != 0) {

		printf("L3GD20: ioctl fail\n");
		return ERROR;
	} else {
		printf("\tL3GD20 configured..\n");
	}


	acc = open("/dev/bma180", O_RDONLY);
	if (acc < 0) {
		printf("BMA180: open fail\n");
		return ERROR;
	}

	//		if (ioctl(gyro, L3GD20_SETRATE, L3GD20_RATE_760HZ_LP_50HZ) != 0 ||
	//				ioctl(gyro, L3GD20_SETRANGE, L3GD20_RANGE_500DPS) != 0) {
	//
	//			printf("BMA180: ioctl fail\n");
	//			return ERROR;
	//		} else {
	//			printf("\tBMA180 configured..\n");
	//		}

	mag = open("/dev/hmc5883l", O_RDONLY);
	if (mag < 0) {
		printf("HMC5883L: open fail\n");
		return ERROR;
	}

	return 0;
}

static int attitude_estimator_bm_read_sensors()
{
	int ret = read(acc, acc_raw, sizeof(acc_raw));
	if (ret != sizeof(acc_raw)) {
		printf("acc read failed!\n");
		return ERROR;
	}

	ret = read(gyro, gyro_raw, sizeof(gyro_raw));
	if (ret != sizeof(gyro_raw)) {
		printf("Gyro read failed!\n");
		return ERROR;
	}

	static int magcounter = 1;

	// Read out every 5th time
	if (magcounter == 5)
	{
		ret = read(mag, mag_raw, sizeof(mag_raw));
		if (ret != sizeof(mag_raw)) {
			printf("Mag read failed!\n");
		}
		magcounter = 0;
	}
	magcounter++;

	//printf("RAW: %d, %d, %d, %d, %d, %d, %d, %d, %d\n", acc_raw[0], acc_raw[1], acc_raw[2], gyro_raw[0], gyro_raw[1], gyro_raw[2], mag_raw[0], mag_raw[1], mag_raw[2]);

	// XXX determine range overflows based on magic sensor values for these cases

	// XXX implement elegant switch between MPU and normal readings

	return ret;
}

int attitude_estimator_bm_update(float_vect3 * euler, float_vect3 * rates, float_vect3 * x_n_b, float_vect3 * y_n_b, float_vect3 * z_n_b)
{
	// all measurement vectors need to be turn into the body frame
	// z negative; x and y exchanged.
	float_vect3 gyro_values; //rad/s
	// XXX Read out gyro range via SPI on init, assuming 500 DPS range at 16 bit res here
	gyro_values.x =  gyro_raw[0] * 0.00026631611f /* = gyro * (500.0f / 180.0f * pi / 32768.0f ) */;
	gyro_values.y =  gyro_raw[1] * 0.00026631611f /* = gyro * (500.0f / 180.0f * pi / 32768.0f ) */;
	gyro_values.z = -gyro_raw[2] * 0.00026631611f /* = gyro * (500.0f / 180.0f * pi / 32768.0f ) */;

	float_vect3 accel_values;
	// XXX Read out accel range via SPI on init, assuming 4G range at 14 bit res here
	accel_values.x =  acc_raw[0] * 0.000244140625f; // = accel * (1 / 32768.0f / 8.0f * 9.81f);
	accel_values.y =  acc_raw[1] * 0.000244140625f; // = accel * (1 / 32768.0f / 8.0f * 9.81f);
	accel_values.z = -acc_raw[2] * 0.000244140625f; // = accel * (1 / 32768.0f / 8.0f * 9.81f);

	float_vect3 mag_values;
	// XXX Read out mag range via I2C on init, assuming 0.88 Ga and 12 bit res here
	mag_values.x =   (mag_raw[0] - mag_offset[0]) * 0.1f;
	mag_values.y =   (mag_raw[1] - mag_offset[1]) * 0.1f;
	mag_values.z = - (mag_raw[2] - mag_offset[2]) * 0.1f;


	attitude_blackmagic(&accel_values, &mag_values, &gyro_values);

	/* read out values */
	attitude_blackmagic_get_all(euler, rates, x_n_b, y_n_b, z_n_b);

	return 0;
}

int attitude_estimator_bm_main(int argc, char *argv[])
{
    // print text
    printf("Black Magic Attitude Estimator initialized..\n\n");
    fflush(stdout);
    usleep(50000);
    

    /* store initial execution time to guarantee constant update rate */
    struct timeval tv;
    gettimeofday(&tv, NULL);
    uint64_t last_run = ((uint64_t)tv.tv_sec) * 1000000 + tv.tv_usec;

    /* data structures to read euler angles and rotation matrix back */
    float_vect3 euler;
    float_vect3 rates;
    float_vect3 x_n_b;
    float_vect3 y_n_b;
    float_vect3 z_n_b;

    /* initialize */
    if (attitude_estimator_bm_sensor_init() != 0) return ERROR;

    /* Spinning until the board is really reset */
    while(true)
    {
    	/* read sensors and put them in global structs */
    	attitude_estimator_bm_read_sensors();

    	/* filter values */
    	attitude_estimator_bm_update(&euler, &rates, &x_n_b, &y_n_b, &z_n_b);
    	/* send out as message queue */

    	// XXX Remove
    	printf("ATT:\t%d\t%d\t%d\n", (int)(euler.x*57), (int)(euler.y*57), (int)(acc_raw[0]));
    	//printf("ABC: %d - %d - %d\n", 0);
    	//printf("RAW X:\t%d\t%d\t%d\n", 0, 0, 0);//gyro_raw[0], acc_raw[0], mag_raw[0]);

    	/* measure how long the calculations took,
    	 * sleep this amount of time less.
    	 * As this calculation is done continously
    	 * from the start time, the update rate will
    	 * be on average absolutely sharp 200 Hz
    	 * and the inaccuracy per loop will be
    	 * in the microseconds range
    	 */
    	gettimeofday(&tv, NULL);
    	uint64_t now = ((uint64_t)tv.tv_sec) * 1000000 + tv.tv_usec;
    	uint64_t time_elapsed = now - last_run;
    	last_run = now;
    	if (time_elapsed < loop_interval)
    	{
    		//usleep(loop_interval - time_elapsed);
    	}
    	else
    	{
    		static int overloadcounter = 0;
    		//TODO: add warning, cpu overload here
    		if (overloadcounter == 50)
    		{
    			printf("CPU OVERLOAD DETECTED IN ATTITUDE ESTIMATOR BLACK MAGIC\n");
    			overloadcounter = 0;
    		}

    		overloadcounter++;
    	}

    	usleep(5000);

    }
    
    /* Should never reach here */
    return 0;
}


