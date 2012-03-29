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

#include "sdlog.h"


/****************************************************************************
 * Definitions
 ****************************************************************************/

/* Struct for receiving of sensor data */
//global_data_sensors_raw_t global_data_sensors_raw = {.access_conf.initialized = 0};


/****************************************************************************
 * user_start
 ****************************************************************************/

int sdlog_main(int argc, char *argv[])
{
	// print text
	printf("logging to SDcard!\n");
	usleep(100000);

	/* initialize shared data structures */
	global_data_init(&global_data_sensors_raw.access_conf);
	

	//try mounting sd card
	const char* src  = "/dev/mmcsd0";
	const char* trgt = "/mnt/sdcard";
	const char* type = "vfat";

	int result = mount(src, trgt, type, 0, "");

	if (result == 0)
	{
		printf("Mount created at %s...\n", trgt);
	
	}
	else
	{
		printf("Error : Failed to mount %s\n",src);
		return -1;
	}
	

	
	//define uart device
	char * uart_name = "/dev/ttyS1";
	//open uart
	printf("xSense UART is %s\n", uart_name);
	xuart = open(uart_name, O_CREAT|O_RDWR | O_NOCTTY);
	if (xuart < 0) {
		printf("Failed to open %s, terminating.\n", uart_name);
		return -1;
	}
	

	//create pthreads
	//pthread_create (&sensors_receive_thread, NULL, sensors_receiveloop, NULL);
	pthread_create (&xsense_receive_thread, NULL, xsense_receiveloop, NULL);
	//pthread_create (&mag_receive_thread, NULL, mag_receive_loop, NULL);

	//wait for threads to complete:
	//pthread_join(sensors_receive_thread, NULL);
	pthread_join(xsense_receive_thread, NULL);
	//pthread_join(mag_receive_thread, NULL);
	printf("ending...");
	//unmounting sdcard
	umount(trgt);
	//close uart
	close(xuart);

    return 0;
}



static void *xsense_receiveloop(void * arg) //runs as a pthread and listens to uart1 ("/dev/ttyS0")
{
	printf("logging xsense...");
	uint8_t ch;
	enum DECODE_STATES decode_state = UNINIT;
	int rx_count = 0;
	int buffer_size = 259;
	char * rx_buffer = malloc(buffer_size*sizeof(char));
	uint8_t checksum = 0;
	uint8_t length = 0;
	printf("debug1");
	
	fflush(stdout);	
	
	while(read(xuart, &ch, 1)) {
		//printf("got a %i \n",ch);
		//fflush(stdout);
		switch(decode_state){
		case UNINIT:
			checksum = 0;
			rx_count = 0;
			length = 0;
			if(ch==0xFA){  //received correct preamble...
				decode_state = GOT_PRE;
			}
			//printf("receiving...%i\n",ch);fflush(stdout);
			
		break;
		case GOT_PRE:
			if(ch==0xFF){  //received correct Bus identifier...
				decode_state = GOT_BID;
				checksum += ch;
			}
			else { //error...
				decode_state = UNINIT;
				checksum = 0;
			}
		break;
		case GOT_BID:
			if(ch==0x32){ //received MTData Message
				decode_state = GOT_MID;
				checksum += ch;
				printf("got right MID\n");fflush(stdout);
			}
			else{ //received some other message (at the moment not intesested what it is...)
				decode_state = UNINIT;
				checksum = 0;
				printf("received some other MID %i \n",ch);
			}
		break;
		case GOT_MID:
			if(ch<=0xFE){ //save length...
				printf("got length: %i \n",ch);fflush(stdout);
				decode_state = GOT_LEN;
				length = ch;
				checksum += ch;
			}
			else{ //some error...
				printf("wrong length");
				decode_state = UNINIT;
				checksum = 0;
			}
				
		break;
		case GOT_LEN:
			if(rx_count<length && rx_count<buffer_size){ //save data in buffer...
				rx_buffer[rx_count]=ch;
				rx_count++;
				checksum += ch;
				//printf("buffering data");fflush(stdout);
			}
			else { //some unexpected error...
				printf("some error\n");fflush(stdout);
				decode_state = UNINIT;
				checksum = 0;
				rx_count = 0;
			}
			if (rx_count==length) { //was last byte of data...
				decode_state = GOT_DATA;
				printf("got all data :D  \n");fflush(stdout);
			}
		break;
		case GOT_DATA:
			checksum += ch;
			if(checksum == 0){ //last byte of chechsum needs to be 0
				printf("successfully received data!!!\n");fflush(stdout);
				printf(rx_buffer);
			}
			else{
				printf("wrong checksum...\n");fflush(stdout);
			}
			checksum = 0;
			rx_count = 0;
			decode_state = UNINIT;
		break;
		default:  //should not happen...
			decode_state=UNINIT;
			rx_count = 0;
			checksum = 0;
		break;
		}
 

		

	}

	free(rx_buffer);
}



static void *sensors_receiveloop(void * arg) //runs as a pthread and listens messages from GPS
{
	struct timespec tp;
	int counter = 0;
	
	FILE * logfile;

	logfile = fopen ("/mnt/sdcard/logfile001.txt","w");
	if (logfile!=NULL)
	{
		clock_gettime(CLOCK_REALTIME,&tp);
		fprintf (logfile, "Logging of raw sensor data started at %i seconds and %i nanoseconds \n",tp.tv_sec,tp.tv_nsec);
		fprintf (logfile, "Seconds\tNanoseconds\tGyrX\tGyrY\tGyrZ\tGyrCount\tAccX\tAccY\tAccZ\tAccCount\tMagX\tMagY\tMagZ\tMagCount\tPress1\tPress2\tPressCount \n");
	}
	else{
		printf("failed to open logfile");
		return;
	}
		
	
	uint64_t timestamp;
/*
	int16_t	gyro_raw[3]; // l3gd20
	uint16_t gyro_raw_counter;
	int16_t	accelerometer_raw[3]; // bma180
	uint16_t accelerometer_raw_counter;
	int16_t	magnetometer_raw[3]; //hmc5883l
	uint16_t magnetometer_raw_counter;
	uint32_t pressure_sensor_raw[2]; //ms5611
	uint16_t pressure_sensor_raw_counter;*/

	while(counter<10000)
	{
		
		if(0 == global_data_wait(&global_data_sensors_raw.access_conf)) //only send if pthread_cond_timedwait received a con signal
		{
			/*for(int i=0; i<3; i++){
				gyro_raw[i]=global_data_sensors_raw.gyro_raw[i];
			}
			gyro_raw_counter=global_data_sensors_raw.gyro_raw_counter;
			for(int i=0; i<3; i++){
				accelerometer_raw[i]=global_data_sensors_raw.accelerometer_raw[i];
			}
			accelerometer_raw_counter=global_data_sensors_raw.accelerometer_raw_counter;
			for(int i=0; i<3; i++){
				magnetometer_raw[i]=global_data_sensors_raw.magnetometer_raw[i];
			}
			magnetometer_raw_counter=global_data_sensors_raw.magnetometer_raw_counter;
			for(int i=0; i<2; i++){		
				pressure_sensor_raw[i]=global_data_sensors_raw.pressure_sensor_raw[i];
			}
			pressure_sensor_raw_counter=global_data_sensors_raw.pressure_sensor_raw_counter;*/


			//clock_gettime(CLOCK_REALTIME,&tp);

			timestamp =  global_data_get_timestamp_useconds();
			fprintf(logfile,"%i\t%i\t%i\t%i\t%i\t%i\t%i\t%i\t%i\t%i\t%i\t%i\t%i\t%i\t%i\t%i\n",timestamp,global_data_sensors_raw.gyro_raw[0],global_data_sensors_raw.gyro_raw[1],global_data_sensors_raw.gyro_raw[2],global_data_sensors_raw.gyro_raw_counter,global_data_sensors_raw.accelerometer_raw[0],global_data_sensors_raw.accelerometer_raw[1],global_data_sensors_raw.accelerometer_raw[2],global_data_sensors_raw.accelerometer_raw_counter,global_data_sensors_raw.magnetometer_raw[0],global_data_sensors_raw.magnetometer_raw[1],global_data_sensors_raw.magnetometer_raw[2],global_data_sensors_raw.magnetometer_raw_counter,global_data_sensors_raw.pressure_sensor_raw[0],global_data_sensors_raw.pressure_sensor_raw[1],global_data_sensors_raw.pressure_sensor_raw_counter);
			counter++;
		}


		global_data_unlock(&global_data_sensors_raw.access_conf);
		
		
		
		
		
			

		
	}	
	clock_gettime(CLOCK_REALTIME,&tp);
	fprintf (logfile, "Logging of raw sensor data ended at %i seconds and %i nanoseconds \n",tp.tv_sec,tp.tv_nsec);
	fclose(logfile);

	printf("written to logfile001.txt");

}



