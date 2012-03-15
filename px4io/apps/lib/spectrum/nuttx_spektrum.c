/**
 ******************************************************************************
 * @addtogroup NUTTX NUTTX Core hardware abstraction layer
 * @{
 * @addtogroup NUTTX_SPEKTRUM Nuttx Spektrum receiver functions
 * @brief Code to handle Spektrum input
 * @{
 *
 * @file       nuttx_spektrum.c
 * @author     The OpenPilot Team, http://www.openpilot.org Copyright (C) 2011.
	       ported by px4fixedwing
 * @brief      USART commands. Inits USARTs, controls USARTs & Interrupt handlers. (STM32 dependent)
 * @see        The GNU Public License (GPL) Version 3
 *
 *****************************************************************************/
/* 
 * This program is free software; you can redistribute it and/or modify 
 * it under the terms of the GNU General Public License as published by 
 * the Free Software Foundation; either version 3 of the License, or 
 * (at your option) any later version.
 * 
 * This program is distributed in the hope that it will be useful, but 
 * WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY 
 * or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License 
 * for more details.
 * 
 * You should have received a copy of the GNU General Public License along 
 * with this program; if not, write to the Free Software Foundation, Inc., 
 * 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA
 */

/* Project Includes */
#include <nuttx/config.h>

#include <sys/types.h>
#include <sys/assert.h>

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <debug.h>

#include <arch/board/board.h>

#include "nuttx_sbus.h"


/**
 * @Note Framesyncing:
 * The code resets the watchdog timer whenever a single byte is received, so what watchdog code
 * is never called if regularly getting bytes.
 * RTC timer is running @625Hz, supervisor timer has divider 5 so frame sync comes every 1/125Hz=8ms.
 * Good for both 11ms and 22ms framecycles
 */



/* Global Variables */

/* Local Variables */
static uint16_t channel_data[NUTTX_SPEKTRUM_NUM_INPUTS],channel_data_temp[NUTTX_SPEKTRUM_NUM_INPUTS];
static uint8_t prev_byte = 0xFF, sync = 0, bytecount = 0, datalength=0, frame_error=0, byte_array[20] = { 0 };
uint8_t sync_of = 0;
uint16_t supv_timer=0;

static uint8_t NUTTX_SPEKTRUM_Reset(){

  for (int i = 0; i < SPEKTRUM_NUMBER_OF_INPUTS; i++) {
    channel_data[i] = 0;
    channel_data_temp[i] = 0;
  }
  return 0;
}


/**
* Decodes a byte
* \param[in] b byte which should be spektrum decoded
* \return 0 if no error
* \return -1 if USART not available
* \return -2 if buffer full (retry)
* \note Applications shouldn't call these functions directly
*/
static int32_t NUTTX_SPEKTRUM_Decode(uint8_t b)
{
	static uint16_t channel = 0; /*, sync_word = 0;*/
	uint8_t channeln = 0, frame = 0;
	uint16_t data = 0;
	byte_array[bytecount] = b;
	bytecount++;
	if (sync == 0) {
		//sync_word = (prev_byte << 8) + b;
#if 0
		/* maybe create object to show this  data */
		if(bytecount==1)
		{
			/* record losscounter into channel8 */
			channel_data_temp[7]=b;
			/* instant write */
			channel_data[7]=b;
		}
#endif
		/* Known sync bytes, 0x01, 0x02, 0x12, 0xb2 */
		/* 0xb2 DX8 3bind pulses only */
		if (bytecount == 2) {
			if ((b == 0x01) || (b == 0xb2)) {
				datalength=0; // 10bit
				//frames=1;
				sync = 1;
				bytecount = 2;
			}
			else if(b == 0x02) {
				datalength=0; // 10bit
				//frames=2;
				sync = 1;
				bytecount = 2;
			}
			else if(b == 0x12) {
				datalength=1; // 11bit
				//frames=2;
				sync = 1;
				bytecount = 2;
			}
			else
			{
				bytecount = 0;
			}
		}
	} else {
		if ((bytecount % 2) == 0) {
			channel = (prev_byte << 8) + b;
			frame = channel >> 15;
			channeln = (channel >> (10+datalength)) & 0x0F;
			data = channel & (0x03FF+(0x0400*datalength));
			if(channeln==0 && data<10) // discard frame if throttle misbehaves
			{
				frame_error=1;
			}
			if (channeln < SPEKTRUM_NUMBER_OF_CHANNELS && !frame_error)
				channel_data_temp[channeln] = data;
	    
	}
	if (bytecount == 16) {
		bytecount = 0;
		sync = 0;
		sync_of = 0;
		if (!frame_error)
		{
			for(int i=0;i< SPEKTRUM_NUMBER_OF_CHANNELS;i++)
			{
				channel_data[i] = channel_data_temp[i];
			}
		}
		frame_error=0;
	}
	prev_byte = b;

	return 0;
}



/**
 * Read the stream provided by NUTTX_SPEKTRUM_INIT
 * and write the channel information in the provided buffer
 * \param filedescriptor for SPEKTRUM_INPUT_PORT
 * \param Pointer to buffer: uint8_t, length: SPEKTRUM_NUMBER_OF_CHANNELS
 * \return 0 if succesfull
 */
 
uint8_t NUTTX_SPEKTRUM_Rx(int fd, uint8_t * buf, buf_len)
{
  /* read data from SPECTRUM_IN_PORT */
  ret = read(fd, buf, sizeof(buf));
  if (ret != 0 ) {
    printf("SPEKTRUM_INPUT_PORT: read error\n")
    return ERROR;
  }


  /* process byte(s) and clear receive timer */
  for (uint8_t i = 0; i < buf_len; i++) {
    NUTTX_SPEKTRUM_Decode(buf[i]);
    receive_timer = 0;
  }

  /* put the channel data in the buf variable */
  buf = channel_data;


  /* Always indicate that all bytes were consumed */
  return 0;  
	
}

	
/**
 * Initialise Spektrum receiver interface
 * \return filedescriptor for a readonly stream pointing 
   at the initialized SPEKTRUM_INPUT_PORT port
 */
int NUTTX_SPEKTRUM_Init(void)
{
    NUTTX_SPEKTRUM_Reset;
    int fd;
    fd = open("SPEKTRUM_INPUT_PORT", O_RDONLY);
    if (fd < 0) {
      printf("error opening SPEKTRUM_INPUT_PORT\n")
      return ERROR;
    }
    //At the moment the settings of SPEKTRUM_INPUT_PORT are hard-coded,
    //so no need to set them here.
                      
    return fd;
      
}


/**
 *@brief This function is called between frames and when a spektrum word hasnt been decoded for too long
 *@brief clears the channel values
 */
static void NUTTX_SPEKTRUM_Supervisor() {
	/* 625hz */
	supv_timer++;
	if(supv_timer > 4) {
		/* sync between frames */
		sync = 0;
		bytecount = 0;
		prev_byte = 0xFF;
		frame_error = 0;
		sync_of++;
		/* watchdog activated after 200ms silence */
		if (sync_of > 30) {
			/* signal lost */
			sync_of = 0;
                        NUTTX_SPEKTRUM_Reset();
		}
		supv_timer = 0;
	}
}


