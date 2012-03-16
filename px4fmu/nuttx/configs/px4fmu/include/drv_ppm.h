/*
 * drv_ppm.h
 *
 *  Created on: Mar 15, 2012
 *      Author: Ivan Ovinnikov
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

#ifndef DRV_PPM_H_
#define DRV_PPM_H_

#include <nuttx/irq.h>			// for IRQ handler
#include <nuttx/config.h>
#include <arch/board/board.h>

#include <sys/types.h>
#include "stm32_tim.h"

struct ppm_cfg
{
	/*
	TIM_TimeBaseInitTypeDef tim_base_init;	// prescaler, countermode, period, clk div, rep. counter
	TIM_ICInitTypeDef tim_ic_init;	// channel, polarity, selection, prescaler, filter
	struct stm32_irq irq;	// irq channel, preemption priority, sub priority, func. state, flags
	TIM_TypeDef * timer;	// timer: 19 registers struct
	// port parameter for gpio?
	*/
	int timer;		// timer identifier (values 2-5)
	uint8_t channel;
	uint16_t ccr;	// capture compare register
};

extern void PPM_irq_handler();

extern uint8_t ppm_num_channels;
extern const struct ppm_cfg ppm_cfg;
extern const struct rcvr_driver ppm_rcvr_driver;

extern void PPM_Init(void);

#endif /* DRV_PPM_H_ */
