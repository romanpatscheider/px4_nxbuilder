/*
 * global_data.h
 *
 *  Created on: Mar 20, 2012
 *      Author: thomasgubler
 */

#ifndef GLOBAL_DATA_H_
#define GLOBAL_DATA_H_

#include <pthread.h>
#include <time.h>

/* Definitions */
#define GLOBAL_DATA_WAIT 2 //seconds
static struct timespec time_to_wait = {0, 0};

typedef struct
{
	/* Pthread condition used for waking up threads when new data is available */
	pthread_cond_t cond;

	/* Pthread mutex used for locking access to data */
	pthread_mutex_t mutex;

	/* Pthread mutex used for locking access to data */
	uint8_t initialized;

} access_conf_t;


/* Prototypes */ //TODO:documentation

/* Initialize cond and mutex of access_conf, needs to be called at the start ov every app that wants to use a shared data structure */
void global_data_init(access_conf_t * access_conf);

/* Locks access to data structure, call before every write or direct read */
void global_data_lock(access_conf_t * access_conf);

/* Tries to lock access to data structure, if already locked returns with nonzero value, success returns 0 */
int global_data_trylock(access_conf_t * access_conf);

/* Unlocks access to data structure, call after every write and read */
void global_data_unlock(access_conf_t * access_conf);

/* Waits for new data and when new data is available performs global_data_lock */
int global_data_wait(access_conf_t * access_conf);

/* Send notification to all processes which are waiting that new data is available (no lock/unlock required here) */
void global_data_broadcast(access_conf_t * access_conf);

/* Get a timestamp in milliseconds, use this to set the timestamp in the shared struct (if available) */
uint64_t global_data_get_timestamp_milliseconds(void);

/* Get a timestamp in micro seconds */
uint64_t global_data_get_timestamp_useconds(void);

#endif /* GLOBAL_DATA_H_ */
