/*
 * gps.h
 *
 *  Created on: Mar 8, 2012
 *      Author: thomasgubler
 */

#ifndef SENSORS_H_
#define SENSORS_H_
/****************************************************************************
 * Included Files
 ****************************************************************************/


#include <nuttx/config.h>
#include <arch/board/drv_lis331.h>
#include <arch/board/drv_bma180.h>
#include <arch/board/drv_l3gd20.h>
#include <arch/board/drv_hmc5883l.h>
#include <unistd.h>
#include <stdio.h>
#include <pthread.h>
#include <fcntl.h>
#include "../global_data_sensors_raw_t.h"

pthread_t gyro_accelerometer_thread;
pthread_t magnetometer_thread;
pthread_t pressure_sensor_thread;

static void *gyro_accelerometer_loop(void * arg);

static void *magnetometer_loop(void * arg);

static void *pressure_sensor_loop(void * arg);

#endif /* SENSORS_H_ */
