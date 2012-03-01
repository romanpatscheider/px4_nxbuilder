/****************************************************************************
 * examples/hello/main.c
 *
 *   Copyright (C) 2012 Lorenz Meier. All rights reserved.
 *   Author: Lorenz Meier <spudmonkey@racsa.co.cr>
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

#include <drv_led.h>

/****************************************************************************
 * Definitions
 ****************************************************************************/

void handleMessage(mavlink_message_t * msg);
/****************************************************************************
 * Private Data
 ****************************************************************************/
static void *receiveloop(void * arg) //runs as a pthread and listens to uart1 ("/dev/ttyS0")
{
	while(1) {

		usleep(10000); //pthread_yield seems not to work

	}

}


static void *heartbeatloop(void * arg)
{
	while(1) {
		// sleep
		usleep(100000);

		mavlink_msg_heartbeat_send(chan,system_type,MAV_AUTOPILOT_GENERIC,MAV_MODE_PREFLIGHT,custom_mode,MAV_STATE_UNINIT);
	}

}
/****************************************************************************
 * Public Functions
 ****************************************************************************/
void handleMessage(mavlink_message_t * msg) {
	printf("DEBUG: got a message \n");
    mavlink_msg_statustext_send(chan,0,"received msg");
}

/****************************************************************************
 * user_start
 ****************************************************************************/

int mavlink_main(int argc, char *argv[])
{
    // print text
    printf("#INFO: SYSCTL ENABLED\n");
    usleep(100000);

    //create pthreads
    pthread_t heartbeat_thread;
    pthread_t receive_thread;

    pthread_create (&heartbeat_thread, NULL, heartbeatloop, NULL);
    pthread_create (&receive_thread, NULL, receiveloop, NULL);

    //wait for threads to complete:
    pthread_join(heartbeat_thread, NULL);
    pthread_join(receive_thread, NULL);

    //close uart
	fclose(uart_read);
	fclose(uart_write);

    return 0;
}


