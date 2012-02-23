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
#include <stdio.h>
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

pthread_mutex_t mutex_stdin;


void handleMessage(mavlink_message_t * msg);
/****************************************************************************
 * Private Data
 ****************************************************************************/
static void *receiveloop(void *arg)
{


	for(;;) {
		printf("This is the pthread looping... \n");
		uint8_t ch = EOF;
		mavlink_message_t msg;
		mavlink_status_t status;


		while(comm_receive_ch(chan,ch, &mutex_stdin)!=EOF) {
		// printf("DEBUG: waiting for data \n");
				if (mavlink_parse_char(chan,ch,&msg,&status)) {
					handleMessage(&msg);
				}
		}


		if (ch==EOF) {
				 status.packet_rx_drop_count = status.packet_rx_drop_count + 1;
			 }

		usleep(100000);
		// Read from the socket
  }
}
/****************************************************************************
 * Public Functions
 ****************************************************************************/
void handleMessage(mavlink_message_t * msg) {
	printf("DEBUG: received msg \n");
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

    // initialize
    mavlink_message_t msg;
    mavlink_status_t status;
    status.packet_rx_drop_count = 0;

    //init mutex
   	pthread_mutex_init (&mutex_stdin, NULL);
   	pthread_mutex_unlock (&mutex_stdin);

    //create pthread for receiving commands
    pthread_t receive_thread;
    pthread_create (&receive_thread, NULL, receiveloop, NULL);


    // start comm loop
    while(1) {
        // sleep
        usleep(100000);

        // send
	    // TODO give correct MAV_MODE

        int result = pthread_mutex_lock (&mutex_stdin);
        printf("main:lock \n");
        //printf("DEBUG: result = %d \n", result);

        mavlink_msg_heartbeat_send(chan,system_type,MAV_AUTOPILOT_GENERIC,MAV_MODE_PREFLIGHT,custom_mode,MAV_STATE_UNINIT);
        pthread_mutex_unlock (&mutex_stdin);
        printf("main:unlock \n");

        // receive
        // TODO figure out how to do non-blocking read
        /*
        uint8_t ch = EOF;
        while(comm_receive_ch(chan,ch)!=EOF) {
            if (mavlink_parse_char(chan,ch,&msg,&status)) {
                handleMessage(&msg);
            }
        }
        if (ch==EOF) {
            status.packet_rx_drop_count = status.packet_rx_drop_count + 1;
        }
        */



    }
    return 0;
}


