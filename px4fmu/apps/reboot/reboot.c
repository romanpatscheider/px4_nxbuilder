/****************************************************************************
 * apps/reboot.c
 *
 *   Copyright (C) 2012 Lorenz Meier. All rights reserved.
 *   Author: Lorenz Meier <lm@inf.ethz.ch>
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
#include <stdio.h>
#include <fcntl.h>

/****************************************************************************
 * Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Data
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * user_start
 ****************************************************************************/

int reboot_main(int argc, char *argv[])
{
    // print text
    printf("Rebooting system - ending tasks and performing hard reset");
    usleep(100000);

    /* Sending kill signal to other tasks */
    // FIXME Implement
    
    /* Waiting maximum time for all to exit */
    
    /* Resetting CPU */
    // FIXME Need check for ARM architecture here
    #ifndef NVIC_AIRCR
    #define NVIC_AIRCR (*((uint32_t*)0xE000ED0C))
    //#define NVIC_AIRCR 0xE000ED0C
    #endif
    
    /* Set the SYSRESETREQ bit to force a reset */
    NVIC_AIRCR |= (1 << 2);
    
//    uint32_t regval = getreg32(NVIC_AIRCR);
//    regval |= (1 << 2);
//    putreg32(regval, NVIC_AIRCR);
    
    /* Should never reach here */
    return 0;
}


