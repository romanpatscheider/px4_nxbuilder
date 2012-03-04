/****************************************************************************
 * px4/ardrone_offboard_control.c
 *
 *   Copyright (C) 2012 PX4 Autopilot Project. All rights reserved.
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
#include <pthread.h>
#include <poll.h>
#include <stdio.h>
#include <fcntl.h>
#include <unistd.h>
#include "ardrone_motor_control.h"

#include <arch/board/drv_gpio.h>

typedef union {
	uint16_t motor_value;
	uint8_t bytes[2];
} motor_union_t;

/**
 * @brief Generate the 8-byte motor set packet
 *
 * @return the number of bytes (8)
 */
uint8_t* ar_get_motor_packet(uint16_t motor1, uint16_t motor2, uint16_t motor3, uint16_t motor4)
{
	static uint8_t motor_buf[5];
	motor_buf[0] = 0x20;
	motor_buf[1] = 0x00;
	motor_buf[2] = 0x00;
	motor_buf[3] = 0x00;
	motor_buf[4] = 0x00;
	//{0x20, 0x00, 0x00, 0x00, 0x00};
	// 0x20 is start sign / motor command
	motor_union_t curr_motor;
	uint16_t nineBitMask = 0x1FF;

	// Set motor 1
	curr_motor.motor_value = (motor1 & nineBitMask) << 4;
	motor_buf[0] |= curr_motor.bytes[1];
	motor_buf[1] |= curr_motor.bytes[0];

	// Set motor 2
	curr_motor.motor_value = (motor2 & nineBitMask) << 3;
	motor_buf[1] |= curr_motor.bytes[1];
	motor_buf[2] |= curr_motor.bytes[0];

	// Set motor 3
	curr_motor.motor_value = (motor3 & nineBitMask) << 2;
	motor_buf[2] |= curr_motor.bytes[1];
	motor_buf[3] |= curr_motor.bytes[0];

	// Set motor 4
	curr_motor.motor_value = (motor4 & nineBitMask) << 1;
	motor_buf[3] |= curr_motor.bytes[1];
	motor_buf[4] |= curr_motor.bytes[0];

	return motor_buf;
}

void ar_enable_broadcast(int fd)
{
	ar_select_motor(fd, 0);
}

int ar_multiplexing_init()
{
	int		fd;

	fd = open("/dev/gpio", O_RDONLY | O_NONBLOCK);
	if (fd < 0) {
		printf("GPIO: open fail\n");
		return fd;
	}

	if (ioctl(fd, GPIO_DIRECTION, GPIO_ALL_OUTPUTS) != 0)
	{
		printf("GPIO: output set fail\n");
		close(fd);
		return -1;
	}

	/* deactivate all outputs */
	int ret = 0;
	ret += ioctl(fd, GPIO_CLEAR, 0);
	ret += ioctl(fd, GPIO_CLEAR, 1);
	ret += ioctl(fd, GPIO_CLEAR, 2);
	ret += ioctl(fd, GPIO_CLEAR, 3);
	if (ret < 0)
	{
		printf("GPIO: clearing pins fail\n");
		close(fd);
		return -1;
	}

	return fd;
}

int ar_multiplexing_deinit(int fd)
{
	if (fd < 0) {
		printf("GPIO: no valid descriptor\n");
		return fd;
	}

	int ret = 0;
	ret += ioctl(fd, GPIO_CLEAR, 0); // CHECK!;//deselect 1
	ret += ioctl(fd, GPIO_CLEAR, 1); // CHECK!;//deselect 2
	ret += ioctl(fd, GPIO_CLEAR, 2); // CHECK!;//deselect 3
	ret += ioctl(fd, GPIO_CLEAR, 3); // CHECK!;//deselect 4

	if (ret != 0)
	{
		printf("GPIO: clear failed %d times\n", ret);
	}

	if (ioctl(fd, GPIO_DIRECTION, GPIO_ALL_INPUTS) != 0)
	{
		printf("GPIO: input set fail\n");
		return -1;
	}
	close(fd);

	return ret;
}

int ar_select_motor(int fd, uint8_t motor)
{
	int ret = 0;
	// Four GPIOS
	//		GPIO_EXT1
	//		GPIO_EXT2
	//		GPIO_UART2_CTS
	//		GPIO_UART2_RTS

	if (motor == 0)
	{
		// XXX FIXME CHECK!!!!!
		ret += ioctl(fd, GPIO_SET, 0); // CHECK!;//select 1
		ret += ioctl(fd, GPIO_SET, 1); // CHECK!;//select 1
		ret += ioctl(fd, GPIO_SET, 2); // CHECK!;//select 1
		ret += ioctl(fd, GPIO_SET, 3); // CHECK!;//select 1
	}
	else if (motor == 3)
	{
		// XXX FIXME CHECK!!!!!
		ret += ioctl(fd, GPIO_SET, 0); // CHECK!;//select 1
		// XXX FIXME CHECK!!!!!
		ret += ioctl(fd, GPIO_CLEAR, 1); // CHECK!;//deselect 2
		ret += ioctl(fd, GPIO_CLEAR, 2); // CHECK!;//deselect 3
		ret += ioctl(fd, GPIO_CLEAR, 3); // CHECK!;//deselect 4
	}
	else if (motor == 4)
	{
		// XXX FIXME CHECK!!!!!
		ret += ioctl(fd, GPIO_CLEAR, 0); // CHECK!;//select 1
		// XXX FIXME CHECK!!!!!
		ret += ioctl(fd, GPIO_SET, 1); // CHECK!;//select 2
		ret += ioctl(fd, GPIO_CLEAR, 2); // CHECK!;//deselect 3
		ret += ioctl(fd, GPIO_CLEAR, 3); // CHECK!;//deselect 4
	}
	else if (motor == 1)
	{
		// XXX FIXME CHECK!!!!!
		ret += ioctl(fd, GPIO_CLEAR, 0); // CHECK!;//select 1
		// XXX FIXME CHECK!!!!!
		ret += ioctl(fd, GPIO_CLEAR, 1); // CHECK!;//deselect 2
		ret += ioctl(fd, GPIO_SET, 2); // CHECK!;//select 3
		ret += ioctl(fd, GPIO_CLEAR, 3); // CHECK!;//deselect 4
	}
	else if (motor == 2)
	{
		// XXX FIXME CHECK!!!!!
		ret += ioctl(fd, GPIO_CLEAR, 0); // CHECK!;//select 1
		// XXX FIXME CHECK!!!!!
		ret += ioctl(fd, GPIO_CLEAR, 1); // CHECK!;//deselect 2
		ret += ioctl(fd, GPIO_CLEAR, 2); // CHECK!;//select 3
		ret += ioctl(fd, GPIO_SET, 3); // CHECK!;//deselect 4
	}
	return ret;
}

