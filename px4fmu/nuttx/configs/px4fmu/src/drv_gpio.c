/*
 *   Copyright (C) 2012 Michael Smith. All rights reserved.
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
 * 3. Neither the name of the author or the names of contributors may be
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
 */

/*
 * GPIO driver for PX4FMU
 *
 * This is something of an experiment currently (ha, get it?)
 */

#include <nuttx/config.h>

#include <stdint.h>
#include <stdbool.h>
#include <debug.h>

#include <nuttx/spi.h>
#include <arch/board/board.h>

#include "up_arch.h"
#include "chip.h"
#include "stm32_internal.h"
#include "px4fmu-internal.h"

#include <arch/board/drv_gpio.h>

static int	px4fmu_gpio_ioctl(struct file *filep, int cmd, unsigned long arg);

static const struct file_operations px4fmu_gpio_fops = {
	.open  = 0,
	.close = 0,
	.read  = 0,
	.write = 0,
	.seek  = 0,
	.ioctl = px4fmu_gpio_ioctl,
#ifndef CONFIG_DISABLE_POLL
	.poll  = 0
#endif
};

void
px4fmu_gpio_init(void)
{
	/* set up GPIOs */
	stm32_configgpio(GPIO_GPIO_DIR);
	stm32_configgpio(GPIO_GPIO0_INPUT);
	stm32_configgpio(GPIO_GPIO1_INPUT);

	/* register the driver */
	register_driver("/dev/gpio", &px4fmu_gpio_fops, 0666, NULL);
}

static int
px4fmu_gpio_ioctl(struct file *filep, int cmd, unsigned long arg)
{
	int	result = 0;

	switch (cmd) {
		
	case GPIO_SET:
		switch (arg) {
		case 0:
			stm32_gpiowrite(GPIO_GPIO0_OUTPUT, 1);
			break;
		case 1:
			stm32_gpiowrite(GPIO_GPIO1_OUTPUT, 1);
			break;
		default:
			result = -1;
		}
		break;

	case GPIO_CLEAR:
		switch (arg) {
		case 0:
			stm32_gpiowrite(GPIO_GPIO0_OUTPUT, 0);
			break;
		case 1:
			stm32_gpiowrite(GPIO_GPIO1_OUTPUT, 0);
			break;
		default:
			result = -1;
		}
		break;

	case GPIO_GET:
		switch (arg) {
		case 0:
			result = stm32_gpioread(GPIO_GPIO0_INPUT);
			break;
		case 1:
			result = stm32_gpioread(GPIO_GPIO1_INPUT);
			break;
		default:
			result = -1;
		}
		break;

	case GPIO_DIRECTION:
		if (arg) {
			/* note that switching to output will drive both GPIOs low */
			stm32_gpiowrite(GPIO_GPIO_DIR, 1);
			stm32_configgpio(GPIO_GPIO0_OUTPUT);
			stm32_configgpio(GPIO_GPIO1_OUTPUT);
		} else {
			stm32_configgpio(GPIO_GPIO0_INPUT);
			stm32_configgpio(GPIO_GPIO1_INPUT);			
			stm32_gpiowrite(GPIO_GPIO_DIR, 0);
		}
		break;
	}
	return result;
}

