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
#include "mavlink_bridge_header.h"
#include "mavlink-1.0/common/mavlink.h"
#include "mavlink-1.0/pixhawk/pixhawk.h"


/****************************************************************************
 * Definitions
 ****************************************************************************/
int system_type = MAV_TYPE_FIXED_WING;
mavlink_system_t mavlink_system = {100,50}; // System ID, 1-255, Component/Subsystem ID, 1-255
uint8_t chan = MAVLINK_COMM_0;
// TODO get correct custom_mode
uint32_t custom_mode = 0;

//threads:
pthread_t heartbeat_thread;
pthread_t receive_thread;

void handleMessage(mavlink_message_t * msg);
/****************************************************************************
 * Private Data
 ****************************************************************************/
static void *receiveloop(void * arg) //runs as a pthread and listens to uart1 ("/dev/ttyS0")
{
	uint8_t ch = EOF;
	mavlink_message_t msg;
	mavlink_status_t status;


	while(1) {

		ch = comm_receive_ch(chan); //get one char
//		printf("Mavlink: read one char\n");
//		if (mavlink_parse_char(chan,ch,&msg,&status)) //parse the char
//		{
//			handleMessage(&msg);
//		}

//		usleep(1); //pthread_yield seems not to work
//		printf("Mavlink yieldres = %d", yieldres);
		int schedres = sched_yield();
//		printf("Mavlink: schedres=%d\n",schedres);
	}

}


static void *heartbeatloop(void * arg)
{
	while(1) {
		// sleep
		usleep(100000);

		mavlink_msg_heartbeat_send(chan,system_type,MAV_AUTOPILOT_GENERIC,MAV_MODE_PREFLIGHT,custom_mode,MAV_STATE_UNINIT);

		int schedres = sched_yield();
//		printf("Mavlink: schedres=%d\n",schedres);
	}

}
/****************************************************************************
 * Public Functions
 ****************************************************************************/
void handleMessage(mavlink_message_t * msg) {
	printf("Mavlink: got a message \n");

	//check for terminate command
//	if(msg->msgid == MAVLINK_MSG_ID_COMMAND_LONG)
//	{
//		printf("Mavlink: Terminating... \n");
//
//		//terminate other threads:
//		pthread_cancel(heartbeat_thread);
//
//		//terminate this thread (receive_thread)
//		pthread_exit(NULL);
//
//	}
    mavlink_msg_statustext_send(chan,0,"received msg");
}

/****************************************************************************
 * user_start
 ****************************************************************************/

int mavlink_main(int argc, char *argv[])
{
    // print text
    printf("Hello, mavlink!!\n");
    usleep(100000);

    //default values for arguments
    char * uart_name = "/dev/ttyS0";

    //read arguments
    int i;
    for (i = 1; i < argc; i++) //argv[0] is "mavlink"
	{
		if (strcmp(argv[i], "-u") == 0 || strcmp(argv[i], "--uart") == 0)  //uart set
		{
			if(argc > i+1)
			{
				uart_name = argv[i+1];
			}
			else
			{
				printf("useage: mavlink -u devicename\n");
				return 0;
			}
		}
	}

    //open uart
    printf("Mavlink uart is %s\n", uart_name);



	uart_read = fopen (uart_name,"rb");
	uart_write = fopen (uart_name,"wb");


	//set policy
	pthread_attr_t attr;
	pthread_attr_init(&attr);
	size_t stacksize;
	pthread_attr_getstacksize (&attr, &stacksize);
	printf("Mavlink: stacksize=%d\n",stacksize);

	pthread_attr_setstacksize(&attr, 2000); //TODO: look up, also sync with makefile
	pthread_attr_getstacksize (&attr, &stacksize);
	printf("Mavlink: stacksize=%d\n",stacksize);




//	int policy;
//	int res = pthread_attr_getschedpolicy(&attr, &policy);
//	printf("Mavlink: res=%d\n",res);
//	printf("Mavlink:policy before: %d, %d, %d, %d, %d\n", policy, SCHED_FIFO, SCHED_RR, SCHED_SPORADIC, SCHED_OTHER);
//	res = pthread_attr_setschedpolicy(&attr, SCHED_RR);
//	printf("Mavlink: res=%d\n",res);
//	pthread_attr_getschedpolicy(&attr, &policy);
//	printf("Mavlink:policy after: %d, %d, %d, %d, %d\n", policy, SCHED_FIFO, SCHED_RR, SCHED_SPORADIC, SCHED_OTHER);

//	struct sched_param  param;
//	param.sched_priority = 100;
//	res = pthread_attr_setschedparam(&attr, &param);
//	printf("Mavlink: res=%d\n",res);

//	struct sched_param  param;
//	param.sched_priority = 0;
//	int res = pthread_setschedparam(heartbeat_thread, SCHED_RR, &param);
//	printf("Mavlink: res=%d\n",res);


//	res = pthread_setschedparam(receive_thread, SCHED_RR, &param);
//	printf("Mavlink: res=%d\n",res);

    //create pthreads
    pthread_create (&heartbeat_thread, &attr, heartbeatloop, NULL);
    pthread_create (&receive_thread, &attr, receiveloop, NULL);

    //wait for threads to complete:
    pthread_join(heartbeat_thread, NULL);
    pthread_join(receive_thread, NULL);

    //close uart
	fclose(uart_read);
	fclose(uart_write);

    return 0;
}


