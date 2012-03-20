/* Functions for managing the global data access */

#include "global_data.h"

void global_data_init(access_conf_t * access_conf)
{
	if(!access_conf->initialized)
	{
		/* Initialize cond and mutex*/
		pthread_mutex_init(&(access_conf->mutex), NULL);
		pthread_cond_init (&(access_conf->cond), NULL);
		access_conf->initialized = 1;
	}
}

void global_data_lock(access_conf_t * access_conf)
{
	pthread_mutex_lock(&(access_conf->mutex));
}

int global_data_trylock(access_conf_t * access_conf)
{
	return pthread_mutex_trylock(&(access_conf->mutex));
}

void global_data_unlock(access_conf_t * access_conf)
{
	pthread_mutex_unlock(&(access_conf->mutex));
}

int global_data_wait(access_conf_t * access_conf)
{
	time_to_wait.tv_sec = time(NULL) + GLOBAL_DATA_WAIT;

	global_data_lock(access_conf);
	return pthread_cond_timedwait(&(access_conf->cond), &(access_conf->mutex), &time_to_wait);
}

void global_data_broadcast(access_conf_t * access_conf)
{
	pthread_cond_broadcast(&(access_conf->cond));
}
