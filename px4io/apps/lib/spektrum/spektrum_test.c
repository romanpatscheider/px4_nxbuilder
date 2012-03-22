/****************************************************************************
 * /apps/lib/spektrum/spektrum_test.c
 *
 *   Copyright (C) 2012 Nils Wenzler. All rights reserved.
 *   Authors: Nils Wenzler <wenzlern@ethz.ch>
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

#include <sys/types.h>

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <debug.h>
#include <string.h>


#include "nuttx_spektrum.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Types
 ****************************************************************************/

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/


/****************************************************************************
 * Private Data
 ****************************************************************************/

uint8_t rx_buf[NUTTX_SPEKTRUM_NUMBER_OF_CHANNELS];

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/
int spektrum_main(int argc, char *argv[]) {

  int fd;
  fd = NUTTX_SPEKTRUM_Init();
  
  if (fd < 0) {
    printf("Could not open NUTTX_SPEKTRUM_INPUT_PORT\n");
    return ERROR;
  } 
  
  
  if(argc >= 2) {
    char* cmd1= "bind";
    char* cmd2= "read";

    if(!strcmp(argv[1], cmd1)){
      
    //  if (!NUTTX_SPEKTRUM_Bind()){
    //    printf("Binding failed\n");
    //    return ERROR;
      printf("Binding not yet implemented\n");
      return ERROR;
      }

      if (!strcmp(argv[1], cmd2) && argc > 2) {
        int n_reads;
        n_reads = atoi(argv[2]);
        while(n_reads){
      
         if(NUTTX_SPEKTRUM_Rx(fd, rx_buf, NUTTX_SPEKTRUM_NUMBER_OF_CHANNELS)){
            printf("No succes receiving Sbus channel data\n");
            return ERROR;
          }

          else {
            printf("Received channel Data from Spektrum\n");
            int i;
            for (i=0; i < NUTTX_SPEKTRUM_NUMBER_OF_CHANNELS; i++)
            printf("\tChannel %d : %d\n", (i+1), rx_buf[i]);
          }

        n_reads--;
        }
    
      }
    }
    else {
      printf("Usage: spektrum_test <cmd> <value>\n Availabale commands:\t bind, read");
      return ERROR;
  
  
  
    }

  
  return 0;
}

