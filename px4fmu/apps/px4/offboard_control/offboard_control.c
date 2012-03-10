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

#include <arch/board/drv_led.h>
#include <arch/board/drv_l3gd20.h>

/****************************************************************************
 * Definitions
 ****************************************************************************/

int leds;
int gyro;

/* gyro x, y, z raw values */
int16_t	gyro_raw[3] = {0, 0, 0};
/* gyro x, y, z metric values in rad/s */
float gyro_rad_s[3] = {0.0f, 0.0f, 0.0f};

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

int offboard_control_main(int argc, char *argv[])
{
    // print text
    printf("Offboard Vicon Space Control Ready\n\n");

    // Leds and sensors
    int	ret = 0;

    if ((led_init() != 0) || (gyro_init() != 0)) ret = ERROR;

    fflush(stdout);

    int testcounter = 0;
    // Remove after testing

    while(true)
    {

    	/* ROUGHLY 20hz, needs the high res-timer */
    	gyro_read();
    	printf("\tl3gd20 values: x:%d\ty:%d\tz:%d\n", gyro_raw[0], gyro_raw[1], gyro_raw[2]);
    	led_toggle(LED_BLUE);
    	led_toggle(LED_AMBER);

    	if (testcounter > 100) break;
    }

    
    
    /* Should never reach here, only on error */
    return ret;
}


