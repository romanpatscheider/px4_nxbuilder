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
#include <fcntl.h>
#include <stdbool.h>
#include "mavlink_bridge_header.h"
#include "v1.0/common/mavlink.h"
#include "v1.0/pixhawk/pixhawk.h"

#include "ardrone_motor_control.h"

#include <arch/board/drv_led.h>
#include <arch/board/drv_l3gd20.h>

/****************************************************************************
 * Definitions
 ****************************************************************************/
static uint8_t system_type = MAV_TYPE_QUADROTOR;
mavlink_system_t mavlink_system;// = {100,50}; // System ID, 1-255, Component/Subsystem ID, 1-255
// TODO get correct custom_mode
static uint32_t custom_mode = 0;

//threads:
static pthread_t heartbeat_thread;
static pthread_t receive_thread;

/* file descriptors */
static int leds;
static int gyro;
static int gpios;
static int ardrone_write;

/* gyro x, y, z raw values */
static int16_t	gyro_raw[3] = {0, 0, 0};
/* gyro x, y, z metric values in rad/s */
static float gyro_rad_s[3] = {0.0f, 0.0f, 0.0f};


struct QuadMotorsDesired {
	uint16_t motorRight_NE;
	uint16_t motorFront_NW;
	uint16_t motorBack_SE;
	uint16_t motorLeft_SW;
};

static struct QuadMotorsDesired actuatorDesired;


static int led_init();
static int led_toggle(int led);
static int gyro_init();
static int gyro_read();
static void *receiveloop(void * arg);


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

static int gyro_init()
{
	gyro = open("/dev/l3gd20", O_RDONLY);
	if (gyro < 0) {
		printf("L3GD20: open fail\n");
		return ERROR;
	}

	if (ioctl(gyro, L3GD20_SETRATE, L3GD20_RATE_760HZ_LP_50HZ) ||
			ioctl(gyro, L3GD20_SETRANGE, L3GD20_RANGE_500DPS)) {

		printf("L3GD20: ioctl fail\n");
		return ERROR;
	} else {
		printf("\tgyro configured..\n");
	}
	return 0;
}

static int gyro_read()
{
	int ret = read(gyro, gyro_raw, sizeof(gyro_raw));
	if (ret != sizeof(gyro_raw)) {
		printf("Gyro read failed!\n");
	}
	return ret;
}

void arHandleMessage(mavlink_message_t * msg);
/****************************************************************************
 * Private Data
 ****************************************************************************/
static void *receiveloop(void * arg) //runs as a pthread and listens to uart1 ("/dev/ttyS0")
{

	uint8_t ch;
	mavlink_message_t msg;
	mavlink_status_t status;

	while(1) {

		/* blocking read on next byte */
		int nread = read (uart, &ch, 1);


		if (nread > 0)
		{
			if (mavlink_parse_char(MAVLINK_COMM_0,ch,&msg,&status)) //parse the char
			{
				arHandleMessage(&msg);
				led_toggle(LED_AMBER);
			}
		}
	}

}

static void ar_init_motors()
{
	// Initialize multiplexing
	gpios = ar_multiplexing_init();

	// Write ARDrone commands on UART2
	uint8_t initbuf[] = {0xE0, 0x91, 0xA1, 0x00, 0x40};
	uint8_t multicastbuf[] = {0xA0, 0xA0, 0xA0, 0xA0, 0xA0, 0xA0};

	// XXX remove
	if (ar_select_motor(gpios, 0) != 0)
	{
		printf("AR: Motor select failed!\n");
	}

	/* initialize all motors
	 * - select one motor at a time
	 * - configure motor
	 */
	int i;
	int errcounter = 0;
	for (i = 1; i < 5; ++i)
	{
		// Initialize motors 1-4
		initbuf[3]=i;
		errcounter += ar_select_motor(gpios, i);

		write(ardrone_write, initbuf+0, 1);

		/* sleep one second */
		usleep(200000);
		usleep(200000);
//		usleep(200000);
//		usleep(200000);
//		usleep(200000);

		write(ardrone_write, initbuf+1, 1);
		/* wait 5 ms */
		usleep(5000);

		write(ardrone_write, initbuf+2, 1);
		/* wait 5 ms */
		usleep(5000);

		write(ardrone_write, initbuf+3, 1);
		/* wait 5 ms */
		usleep(5000);

		write(ardrone_write, initbuf+4, 1);
		/* wait 5 ms */
		usleep(5000);

		/* enable multicast */
		write(ardrone_write, multicastbuf+0, 1);
		/* wait 1 ms XXX change to 100 us */
		usleep(1000);

		write(ardrone_write, multicastbuf+1, 1);
		/* wait 1 ms XXX change to 100 us */
		usleep(1000);

		write(ardrone_write, multicastbuf+2, 1);
		/* wait 1 ms XXX change to 100 us */
		usleep(1000);

		write(ardrone_write, multicastbuf+3, 1);
		/* wait 1 ms XXX change to 100 us */
		usleep(1000);

		write(ardrone_write, multicastbuf+4, 1);
		/* wait 1 ms XXX change to 100 us */
		usleep(1000);

		write(ardrone_write, multicastbuf+5, 1);
		/* wait 5 ms XXX change to 100 us */
		usleep(5000);
	}

	//start the multicast part
	errcounter += ar_select_motor(gpios, 0);

	if (errcounter != 0)
	{
		printf("AR: init sequence incomplete, failed %d times", -errcounter);
	}
}


static void *control_loop(void * arg)
{

	int16_t outputBand=0;

	float setpointXrate;
	float setpointYrate;
	float setpointZrate;

	float setpointRateCast[3]={0,0,0};
	float Kp=0;
	float Ki=0;
	float setpointThrustCast=0;
	float startpointFullControll=0;
	float maxThrustSetpoints=0;

	float gyro_filtered[3] = {0,0,0};
	float gyro_filtered_offset[3] = {0,0,0};
	float gyro_alpha = 0;
	float gyro_alpha_offset = 0;
	float errXrate=0;
	float attRatesScaled[3]={0,0,0};

	uint16_t offsetCnt=0;
	float antiwindup=0;


	ar_init_motors();

	while(1) {

		// Update gyro
		gyro_read();

		gyro_filtered_offset[0] = 0.00026631611f*gyro_raw[0];
		gyro_filtered_offset[1] = 0.00026631611f*gyro_raw[1];
		gyro_filtered_offset[2] = 0.00026631611f*gyro_raw[2];

		gyro_filtered[0] = 0.00026631611f*gyro_raw[1];
		gyro_filtered[1] = 0.00026631611f*gyro_raw[0];
		gyro_filtered[2] = 0.00026631611f*gyro_raw[2];

		outputBand=0;
		startpointFullControll = 150.0f;
		maxThrustSetpoints = 511.0f;
		//Kp=60;
		Kp=40.0f;
		//Kp=45;
		Ki=0.0f;
		antiwindup=50.0f;

		//Rate Controller

		setpointRateCast[0]=((float)actuatorDesired.motorRight_NE-9999.0f)*0.01f/180.0f*3.141f;
		setpointRateCast[1]=((float)actuatorDesired.motorFront_NW-9999.0f)*0.01f/180.0f*3.141f;
		setpointRateCast[2]=-((float)actuatorDesired.motorBack_SE-127.0f)/180.0f*3.141f;
		//Ki=actuatorDesired.motorRight_NE*0.001f;
		setpointThrustCast=actuatorDesired.motorLeft_SW;

		attRatesScaled[0]=0.00026631611f*gyro_raw[1];
		attRatesScaled[1]=0.00026631611f*gyro_raw[0];
		attRatesScaled[2]=0.00026631611f*gyro_raw[2];


		//filtering of the gyroscope values

		//compute filter coefficient alpha

		//gyro_alpha=0.005/(2.0f*3.1415f*200.0f+0.005f);
		//gyro_alpha=0.009;
		gyro_alpha=0.09f;
		gyro_alpha_offset=0.001f;
		//gyro_alpha=0.001;
		//offset estimation and filtering
		offsetCnt++;
		uint8_t i;
		for(i=0; i< 3; i++)
		{	if(offsetCnt<5000){
			gyro_filtered_offset[i] = attRatesScaled[i] * gyro_alpha_offset +  gyro_filtered_offset[i]* (1 - gyro_alpha_offset);
		}
		gyro_filtered[i] = 1.0f*((attRatesScaled[i]-gyro_filtered_offset[i]) * gyro_alpha + gyro_filtered[i] * (1 - gyro_alpha))-0*setpointRateCast[i];
		}

		//rate controller

		//X-axis
		if((Ki*errXrate<antiwindup)&&(Ki*errXrate>-antiwindup)){
			errXrate=errXrate+(setpointRateCast[0]-gyro_filtered[1]);
		}


		setpointXrate=Kp*(setpointRateCast[0]-gyro_filtered[0])+Ki*errXrate;

		//Y-axis
		setpointYrate=Kp*(setpointRateCast[1]-gyro_filtered[1])+Ki*errXrate;
		//Z-axis
		setpointZrate=Kp*(setpointRateCast[2]-gyro_filtered[2])+Ki*errXrate;

		//Mixing
		if (setpointThrustCast<=0){
			setpointThrustCast=0;
			outputBand=0;
		}

		if ((setpointThrustCast<startpointFullControll)&&(setpointThrustCast>0)){
			outputBand=0.75f*setpointThrustCast;
		}

		if((setpointThrustCast>=startpointFullControll)&&(setpointThrustCast<maxThrustSetpoints-0.75f*startpointFullControll)){
			outputBand=0.75f*startpointFullControll;
		}

		if(setpointThrustCast>=maxThrustSetpoints-0.75f*startpointFullControll){
			setpointThrustCast=0.75f*startpointFullControll;
			outputBand=0.75f*startpointFullControll;
		}

		actuatorDesired.motorFront_NW=setpointThrustCast+(setpointXrate+setpointYrate+setpointZrate);
		actuatorDesired.motorRight_NE=setpointThrustCast+(-setpointXrate+setpointYrate-setpointZrate);
		actuatorDesired.motorBack_SE=setpointThrustCast+(-setpointXrate-setpointYrate+setpointZrate);
		actuatorDesired.motorLeft_SW=setpointThrustCast+(setpointXrate-setpointYrate-setpointZrate);


		if((setpointThrustCast+setpointXrate+setpointYrate+setpointZrate)>(setpointThrustCast+outputBand)){
			actuatorDesired.motorFront_NW=setpointThrustCast+outputBand;
		}
		if((setpointThrustCast+setpointXrate+setpointYrate+setpointZrate)<(setpointThrustCast-outputBand)){
			actuatorDesired.motorFront_NW=setpointThrustCast-outputBand;
		}

		if((setpointThrustCast+(-setpointXrate)+setpointYrate-setpointZrate)>(setpointThrustCast+outputBand)){
			actuatorDesired.motorRight_NE=setpointThrustCast+outputBand;
		}
		if((setpointThrustCast+(-setpointXrate)+setpointYrate-setpointZrate)<(setpointThrustCast-outputBand)){
			actuatorDesired.motorRight_NE=setpointThrustCast-outputBand;
		}

		if((setpointThrustCast+(-setpointXrate)+(-setpointYrate)+setpointZrate)>(setpointThrustCast+outputBand)){
			actuatorDesired.motorBack_SE=setpointThrustCast+outputBand;
		}
		if((setpointThrustCast+(-setpointXrate)+(-setpointYrate)+setpointZrate)<(setpointThrustCast-outputBand)){
			actuatorDesired.motorBack_SE=setpointThrustCast-outputBand;
		}

		if((setpointThrustCast+setpointXrate+(-setpointYrate)-setpointZrate)>(setpointThrustCast+outputBand)){
			actuatorDesired.motorLeft_SW=setpointThrustCast+outputBand;
		}
		if((setpointThrustCast+setpointXrate+(-setpointYrate)-setpointZrate)<(setpointThrustCast-outputBand)){
			actuatorDesired.motorLeft_SW=setpointThrustCast-outputBand;
		}

		// Turn on the error LED if a packet reached here
		static int ledcounter = 0;
		if (ledcounter == 5)
		{
			//uint8_t* motorSpeedBuf = ar_get_motor_packet(actuatorDesired.motorFront_NW, actuatorDesired.motorRight_NE, actuatorDesired.motorBack_SE, actuatorDesired.motorLeft_SW);
			uint8_t* motorSpeedBuf = ar_get_motor_packet(15, 15, 15, 15);

			write(ardrone_write, motorSpeedBuf, 5);
			ledcounter = 0;
		}
		ledcounter++;

		// Send heartbeat every 400th iteration
		static int beatcount = 0;
		if (beatcount == 400)
		{
			mavlink_msg_heartbeat_send(MAVLINK_COMM_0,system_type,MAV_AUTOPILOT_GENERIC,MAV_MODE_PREFLIGHT,custom_mode,MAV_STATE_UNINIT);

			if (beatcount % 4 == 0)
			{
				mavlink_msg_raw_imu_send(MAVLINK_COMM_0, 0, 0, 0, 0, gyro_raw[0], gyro_raw[1], gyro_raw[2], 0, 0, 0);
			}


			if (!(mavlink_system.mode & MAV_MODE_FLAG_SAFETY_ARMED)) {
				// System is not armed, blink at 1 Hz
				led_toggle(LED_BLUE);
			}

			beatcount = 0;
		}

		if (beatcount == 40)
		{
			if (mavlink_system.mode & MAV_MODE_FLAG_SAFETY_ARMED) {
				// System is armed, blink at 10 Hz
				led_toggle(LED_BLUE);
			}
		}

		beatcount++;

		usleep(2000);

	}
	return 0;
}
/****************************************************************************
 * Public Functions
 ****************************************************************************/
void arHandleMessage(mavlink_message_t * msg) {

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
				mavlink_msg_statustext_send(MAVLINK_COMM_0,0,"Rebooting autopilot.. ");
			}

			/* request for a link shutdown */
			if (cmd.command == MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN && cmd.param1 == 3.0f)
			{
				terminate_link = true;
				result = MAV_RESULT_ACCEPTED;
				mavlink_msg_statustext_send(MAVLINK_COMM_0,0,"Terminating MAVLink.. ");
			}

			/* supported command handling stop */


			/* send any requested ACKs */
			if (cmd.confirmation > 0)
			{
				mavlink_msg_command_ack_send(MAVLINK_COMM_0, cmd.command, result);
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

int ardrone_offboard_control_main(int argc, char *argv[])
{
	int	ret = 0;

	// print text
	printf("ARDRONE OFFBOARD CONTROL ACTIVATED\n");
	fflush(stdout);
	usleep(100000);

	//default values for arguments
	char * mavlink_uart_name = "/dev/ttyS0";
	char * ardrone_uart_name = "/dev/ttyS1";

	//read arguments
	int i;
	for (i = 1; i < argc; i++) //argv[0] is "mavlink"
	{
		if (strcmp(argv[i], "-u") == 0 || strcmp(argv[i], "--uart") == 0)  //uart set
		{
			if(argc > i+1)
			{
				mavlink_uart_name = argv[i+1];
			}
			else
			{
				printf("usage: ardrone_offboard_control -u devicename\n");
				return 0;
			}
		}
	}

	//open uart
	printf("MAVLink UART is %s\n", mavlink_uart_name);
	printf("ARDrone UART is %s\n", ardrone_uart_name);
	fflush(stdout);
	uart = open(mavlink_uart_name, O_RDWR | O_NOCTTY);
	ardrone_write = open(ardrone_uart_name, O_RDWR | O_NOCTTY | O_NDELAY);

	/* initialize leds and sensor */


	if ((led_init() != 0) || (gyro_init() != 0)) ret = ERROR;

	fflush(stdout);


	if (ret == ERROR) return ret;

	//create pthreads
	pthread_create (&heartbeat_thread, NULL, control_loop, NULL);
	pthread_create (&receive_thread, NULL, receiveloop, NULL);

	//wait for threads to complete:
	pthread_join(heartbeat_thread, NULL);
	pthread_join(receive_thread, NULL);

	//close uart
	close(uart);
	close(ardrone_write);
	close(leds);
	ar_multiplexing_deinit(gpios);
	close(gyro);

	return ret;
}


