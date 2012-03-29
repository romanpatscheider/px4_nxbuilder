/*
 * sdlog.h
 *
 *  Created on: Mar 8, 2012
 *      Author: romanpatscheider
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
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <stdbool.h>
#include <sys/mount.h>
#include <time.h>
#include <nuttx/clock.h>
#include <pthread.h>
#include <fcntl.h>
#include "../global_data_sensors_raw_t.h"

int xuart;

pthread_t sensors_receive_thread;
pthread_t xsense_receive_thread;
//pthread_t mag_receive_thread;

static void *sensors_receiveloop(void * arg);
static void *xsense_receiveloop(void * arg);
//static void *mag_receiveloop(void * arg);

//define states of decoder
enum DECODE_STATES
{
	UNINIT = 0,
	GOT_PRE = 1,
	GOT_BID = 2,
	GOT_MID = 3,
	GOT_LEN = 4,
	GOT_DATA = 5

};

#endif /* SENSORS_H_ */
