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

/****************************************************************************
 * Included Files
 ****************************************************************************/


#include <nuttx/config.h>
#include <pthread.h>
#include <poll.h>
#include <stdio.h>
#include <stdbool.h>
#include <fcntl.h>
#include <mqueue.h>
#include "mavlink_bridge_header.h"
#include "v1.0/common/mavlink.h"
#include "v1.0/pixhawk/pixhawk.h"
#include "../mq_config.h"
#include <arch/board/drv_led.h>
#include "../gps_data_t.h"


/****************************************************************************
 * Definitions
 ****************************************************************************/

//threads:
pthread_t heartbeat_thread;
pthread_t receive_thread;
pthread_t gps_receive_thread;

int leds;

int system_type = MAV_TYPE_FIXED_WING;
mavlink_system_t mavlink_system = {100,50}; // System ID, 1-255, Component/Subsystem ID, 1-255
uint8_t chan = MAVLINK_COMM_0;
// TODO get correct custom_mode
uint32_t custom_mode = 0;
mavlink_status_t status;

/* initialize global GPS data */
//extern gps_data_t gps_data;

/* 3: Define waypoint helper functions */
void mavlink_wpm_send_message(mavlink_message_t* msg);
void mavlink_wpm_send_gcs_string(const char* string);
uint64_t mavlink_wpm_get_system_timestamp();
void mavlink_missionlib_send_message(mavlink_message_t* msg);
void mavlink_missionlib_send_gcs_string(const char* string);
uint64_t mavlink_missionlib_get_system_timestamp();

/* 4: Include waypoint protocol */
#include "waypoints.h"
mavlink_wpm_storage wpm;


#include "mavlink_missionlib_data.h"
#include "mavlink_parameters.h"

mavlink_pm_storage pm;

uint32_t onboard_control_sensors_present;
uint32_t onboard_control_sensors_enabled;
uint32_t onboard_control_sensors_health;
uint16_t load;
uint16_t voltage_battery;
uint16_t current_battery;
int8_t battery_remaining;
int16_t drop_rate_comm;
int16_t errors_comm;
int16_t external_i2c_err_count;
int16_t internal_i2c_err_count;
int16_t spi_err_count;
int16_t sd_err_count;

/**
 * @brief reset all parameters to default
 * @warning DO NOT USE THIS IN FLIGHT!
 */
void mavlink_pm_reset_params(mavlink_pm_storage* pm)
{
	pm->size = MAVLINK_PM_MAX_PARAM_COUNT;
	// 1) MAVLINK_PM_PARAM_SYSTEM_ID
	pm->param_values[MAVLINK_PM_PARAM_SYSTEM_ID] = 12;
	strcpy(pm->param_names[MAVLINK_PM_PARAM_SYSTEM_ID], "SYS_ID");
	// 2) MAVLINK_PM_PARAM_ATT_K_D
	pm->param_values[MAVLINK_PM_PARAM_ATT_K_D] = 0.3f;
	strcpy(pm->param_names[MAVLINK_PM_PARAM_ATT_K_D], "ATT_K_D");
}

void mavlink_missionlib_send_message(mavlink_message_t* msg)
{
	uint8_t buf[MAVLINK_MAX_PACKET_LEN];
	uint16_t len = mavlink_msg_to_send_buffer(buf, msg);
	write(uart, buf, len);
}

void mavlink_missionlib_send_gcs_string(const char* string)
{
	const int len = 50;
	mavlink_statustext_t status;
	int i = 0;
	while (i < len - 1)
	{
		status.text[i] = string[i];
		if (string[i] == '\0')
			break;
		i++;
	}
	status.text[i] = '\0'; // Enforce null termination
	mavlink_message_t msg;

	mavlink_msg_statustext_encode(mavlink_system.sysid, mavlink_system.compid, &msg, &status);
	mavlink_missionlib_send_message(&msg);
}

uint64_t mavlink_missionlib_get_system_timestamp()
{
	struct timeval tv;
	gettimeofday(&tv, NULL);
	return ((uint64_t)tv.tv_sec) * 1000000 + tv.tv_usec;
}

extern void mavlink_missionlib_current_waypoint_changed(uint16_t index, float param1,
		float param2, float param3, float param4, float param5_lat_x,
		float param6_lon_y, float param7_alt_z, uint8_t frame, uint16_t command) {

	/* Update controller setpoints */
}


static int led_init()
{
	leds = open("/dev/led", O_RDONLY | O_NONBLOCK);
	if (leds < 0) {
		printf("LED: open fail\n");
		return ERROR;
	}

	if (ioctl(leds, LED_ON, LED_BLUE) ||
			ioctl(leds, LED_ON, LED_AMBER)) {

		printf("LED: ioctl fail\n");
		return ERROR;
	}
	return 0;
}

static int led_toggle(int led)
{
	static int last_blue = LED_ON;
	static int last_amber = LED_ON;

	if (led == LED_BLUE) last_blue = (last_blue == LED_ON) ? LED_OFF : LED_ON;
	if (led == LED_AMBER) last_amber = (last_amber == LED_ON) ? LED_OFF : LED_ON;

	return ioctl(leds, ((led == LED_BLUE) ? last_blue : last_amber), led);
}

static int led_on(int led)
{
	return ioctl(leds, LED_ON, led);
}

static int led_off(int led)
{
	return ioctl(leds, LED_OFF, led);
}

void handleMessage(mavlink_message_t * msg);


/****************************************************************************
 * Private Data
 ****************************************************************************/
static void *receiveloop(void * arg) //runs as a pthread and listens to uart1 ("/dev/ttyS0")
{

	uint8_t ch;

	mavlink_message_t msg;

	while(1) {
		/* blocking read on next byte */
		int nread = read(uart, &ch, 1);


		if (nread > 0 && mavlink_parse_char(chan,ch,&msg,&status)) {//parse the char
			// handle generic messages and commands
			handleMessage(&msg);

			// Handle packet with waypoint component
			mavlink_wpm_message_handler(&msg);

			// Handle packet with parameter component
			mavlink_pm_message_handler(MAVLINK_COMM_0, &msg);
			led_toggle(LED_AMBER);
		}

	}
}

static void *gps_receiveloop(void * arg) //runs as a pthread and listens messages from GPS
{

	while(1)
	{
		mavlink_msg_statustext_send(chan,0,"gps mavlink raw sending");
		mavlink_msg_gps_raw_int_send(MAVLINK_COMM_0, gps_data.time_usec, gps_data.fix_type, gps_data.lat, gps_data.lon, gps_data.alt, gps_data.eph, gps_data.epv, gps_data.vel, gps_data.cog, gps_data.satellites_visible);

		mavlink_msg_gps_status_send(MAVLINK_COMM_0, gps_data.satellites_visible, gps_data.satellite_prn, gps_data.satellite_used, gps_data.satellite_elevation, gps_data.satellite_azimuth, gps_data.satellite_snr);

		usleep(10000);
	}
//	//Open Message queue to receive GPS information
//
//	int prio;
//	char * msg = malloc(5*sizeof(char));
//
//    typedef struct
//    {
//    	char str1;
//    	char str2;
//
//    } __attribute__((__packed__)) test_struct;
//    test_struct mystruct;
//
//	ssize_t result_receive;
//	while(1)
//	{
//		if(mq_receive(gps_queue, &mystruct, sizeof(test_struct), &prio) > 0)
//		{
//			mavlink_msg_statustext_send(chan,0,"gps received msg");
//			mavlink_msg_statustext_send(chan,0,&(mystruct.str1));
//			mavlink_msg_statustext_send(chan,0,&(mystruct.str2));
//		}
//	}
}


static void *heartbeatloop(void * arg)
{

	/* initialize waypoint manager */
	mavlink_wpm_init(&wpm);
	/* initialize parameter storage */
	mavlink_pm_reset_params(&pm);

	int lowspeed_counter = 0;

	while(1) {

		// sleep
		usleep(50000);

		// 1 Hz
		if (lowspeed_counter == 10)
		{
			// Send heartbeat and status
			mavlink_msg_heartbeat_send(chan,system_type,MAV_AUTOPILOT_GENERIC,MAV_MODE_PREFLIGHT,custom_mode,MAV_STATE_UNINIT);
			mavlink_msg_sys_status_send(chan, onboard_control_sensors_present, onboard_control_sensors_enabled, onboard_control_sensors_health, load, voltage_battery, current_battery, battery_remaining, drop_rate_comm, errors_comm, external_i2c_err_count, internal_i2c_err_count, spi_err_count, sd_err_count);

			if (!(mavlink_system.mode & MAV_MODE_FLAG_SAFETY_ARMED)) {
				// System is not armed, blink at 1 Hz
				led_toggle(LED_BLUE);
			}

			lowspeed_counter = 0;
		}
		lowspeed_counter++;

		// Send one parameter
		mavlink_pm_queued_send();

		if (mavlink_system.mode & MAV_MODE_FLAG_SAFETY_ARMED) {
			// System is armed, blink at 10 Hz
			led_toggle(LED_BLUE);
		}
		usleep(50000);
	}

}
/****************************************************************************
 * Public Functions
 ****************************************************************************/
void handleMessage(mavlink_message_t * msg) {


	//check for terminate command
	if(msg->msgid == MAVLINK_MSG_ID_COMMAND_LONG)
	{
		mavlink_command_long_t cmd;
		mavlink_msg_command_long_decode(msg, &cmd);

		if (cmd.target_system == mavlink_system.sysid && ((cmd.target_component == mavlink_system.compid) || (cmd.target_component == MAV_COMP_ID_ALL)))
		{
			bool terminate_link = false;
			bool reboot = false;
			/* result of the command */
			uint8_t result = MAV_RESULT_UNSUPPORTED;


			/* supported command handling start */

			/* request to set different system mode */
			if (cmd.command == MAV_CMD_DO_SET_MODE)
			{
				mavlink_system.mode = cmd.param1;
			}

			/* request to arm */
			// XXX

			/* request for an autopilot reboot */
			if (cmd.command == MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN && cmd.param1 == 1.0f)
			{
				reboot = true;
				result = MAV_RESULT_ACCEPTED;
				mavlink_msg_statustext_send(chan,0,"Rebooting autopilot.. ");
			}

			/* request for a link shutdown */
			if (cmd.command == MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN && cmd.param1 == 3.0f)
			{
				terminate_link = true;
				result = MAV_RESULT_ACCEPTED;
				mavlink_msg_statustext_send(chan,0,"Terminating MAVLink.. ");
			}

			/* supported command handling stop */


			/* send any requested ACKs */
			if (cmd.confirmation > 0)
			{
				mavlink_msg_command_ack_send(chan, cmd.command, result);
			}

			/* the two termination / reset commands need special handling */
			if (terminate_link)
			{
				printf("MAVLink: Terminating.. \n");
				fflush(stdout);
				/* sleep 100 ms to allow UART buffer to empty */
				led_off(LED_BLUE);
				led_on(LED_AMBER);
				int i;
				for (i = 0; i < 5; i++)
				{
					led_toggle(LED_AMBER);
					usleep(20000);
				}

				//terminate other threads:
				pthread_cancel(heartbeat_thread);

				//terminate this thread (receive_thread)
				pthread_exit(NULL);
			}

			if (reboot)
			{
				printf("MAVLink: Rebooting system.. \n");
				fflush(stdout);
				/* sleep 100 ms to allow UART buffer to empty */
				led_off(LED_BLUE);
				led_on(LED_AMBER);
				int i;
				for (i = 0; i < 5; i++)
				{
					led_toggle(LED_BLUE);
					led_toggle(LED_AMBER);
					usleep(20000);
				}

				//terminate other threads:
				pthread_cancel(heartbeat_thread);

				// Reset whole system
				/* Resetting CPU */
				// FIXME Need check for ARM architecture here
#ifndef NVIC_AIRCR
#define NVIC_AIRCR (*((uint32_t*)0xE000ED0C))
#endif

				/* Set the SYSRESETREQ bit to force a reset */
				NVIC_AIRCR = 0x05fa0004;

				/* Spinning until the board is really reset */
				while(true);
			}
		}
	}
}

/****************************************************************************
 * user_start
 ****************************************************************************/

int mavlink_main(int argc, char *argv[])
{
	// print text
	printf("MAVLink v1.0 serial interface starting..\n");
	usleep(10000);

	//default values for arguments
	char * uart_name = "/dev/ttyS0";

	//read arguments
	int i;
	for (i = 1; i < argc; i++) //argv[0] is "mavlink"
	{
		if (strcmp(argv[i], "-d") == 0 || strcmp(argv[i], "--device") == 0)  //uart set
		{
			if(argc > i+1)
			{
				uart_name = argv[i+1];
			}
			else
			{
				printf("\tusage: mavlink -d devicename\n");
				return 0;
			}
		}
	}

	//open uart
	printf("MAVLink UART is %s\n", uart_name);
	uart = open(uart_name, O_RDWR | O_NOCTTY);
	if (uart < 0) {
		printf("Failed to open %s, terminating.\n", uart_name);
		return -1;
	}

	if (led_init() != 0)
	{
		printf("Failed to initialize leds, terminating.\n");
		return -2;
	}

	// Initialize leds
	led_on(LED_BLUE);
	led_off(LED_AMBER);

//	struct mq_attr mq_attr_gps = MQ_ATTR_GPS;
//	// open gps queue
//    gps_queue = mq_open( MQ_NAME_GPS, O_CREAT|O_RDONLY, 0666, &mq_attr_gps );
//
//	if(-1 == gps_queue)
//	{
//		mavlink_msg_statustext_send(chan,0,"gps queue creation in receiveloop failed");
//	}

    //create pthreads
    pthread_create (&heartbeat_thread, NULL, heartbeatloop, NULL);
    pthread_create (&receive_thread, NULL, receiveloop, NULL);
    pthread_create (&gps_receive_thread, NULL, gps_receiveloop, NULL);

    //wait for threads to complete:
    pthread_join(heartbeat_thread, NULL);
    pthread_join(receive_thread, NULL);
    pthread_join(gps_receive_thread, NULL);


	//close uart
	close(uart);


	//close GPS queue
	mq_close(gps_queue);

    return 0;
}


