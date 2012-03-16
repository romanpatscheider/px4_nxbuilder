/*
 * 	PX4FMU PPM Driver
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

#include <nuttx/config.h>

#include <stdint.h>
#include <stdbool.h>
#include <debug.h>
#include <errno.h>

#include <nuttx/spi.h>
#include <arch/board/board.h>
#include "px4fmu-internal.h"
#include "stm32_internal.h"

#include <drv_ppm.h>
#include "stm32_tim.h"
#include <irq.h>


/* Defines */

#define PPM_IN_MIN_NUM_CHANNELS		4
#define PPM_IN_MAX_NUM_CHANNELS		8	// TODO: check this value
#define PPM_STABLE_CHANNEL_COUNT		25	// frames
#define PPM_IN_MIN_SYNC_PULSE_US		3800	// microseconds
#define PPM_IN_MIN_CHANNEL_PULSE_US	750	// microseconds
#define PPM_IN_MAX_CHANNEL_PULSE_US	2250   // microseconds
#define PPM_INPUT_INVALID			0

/* Provide a RCVR driver */
static int32_t PPM_Get(uint32_t rcvr_id, uint8_t channel);

const struct nuttx_rcvr_driver ppm_rcvr_driver = {
	.read = PPM_Get,
};

/* Local Variables */
static struct stm32_tim_dev_s tim_str;
static uint8_t PulseIndex;
static uint32_t PreviousTime;
static uint32_t CurrentTime;
static uint32_t DeltaTime;
static uint32_t CaptureValue[PPM_IN_MAX_NUM_CHANNELS];
static uint32_t CaptureValueNewFrame[PPM_IN_MAX_NUM_CHANNELS];
static uint32_t LargeCounter;
static int8_t NumChannels;
static int8_t NumChannelsPrevFrame;
static uint8_t NumChannelCounter;
static uint8_t supv_timer = 0;
static uint8_t Tracking;
static uint8_t Fresh;

static void PPM_Supervisor(uint32_t ppm_id);

void PPM_Init(void)
{
	/* Flush counter variables */
	int32_t i;

	PulseIndex = 0;
	PreviousTime = 0;
	CurrentTime = 0;
	DeltaTime = 0;
	LargeCounter = 0;
	NumChannels = -1;
	NumChannelsPrevFrame = -1;
	NumChannelCounter = 0;
	Tracking = FALSE;
	Fresh = FALSE;

	for (i = 0; i < PPM_IN_MAX_NUM_CHANNELS; i++) {
		CaptureValue[i] = 0;
		CaptureValueNewFrame[i] = 0;
	}

	/* initialise interrupts
	 *
	 */
	NVIC_InitTypeDef NVIC_InitStructure = ppm_cfg.irq.init;

	up_irqinitialize();

	/* Enable appropriate clock to timer module */
	switch((int32_t) ppm_cfg.timer) {
		case (int32_t)TIM1:
			NVIC_InitStructure.NVIC_IRQChannel = TIM1_CC_IRQn;
			break;
		case (int32_t)TIM2:
			NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn;
			break;
		case (int32_t)TIM3:
			NVIC_InitStructure.NVIC_IRQChannel = TIM3_IRQn;
			break;
		case (int32_t)TIM4:
			NVIC_InitStructure.NVIC_IRQChannel = TIM4_IRQn;
			break;
#ifdef STM32F10X_HD
		case (int32_t)TIM5:
			NVIC_InitStructure.NVIC_IRQChannel = TIM5_IRQn;
			break;
		case (int32_t)TIM6:
			NVIC_InitStructure.NVIC_IRQChannel = TIM6_IRQn;
			break;
		case (int32_t)TIM7:
			NVIC_InitStructure.NVIC_IRQChannel = TIM7_IRQn;
			break;
		case (int32_t)TIM8:
			NVIC_InitStructure.NVIC_IRQChannel = TIM8_CC_IRQn;
			break;
#endif
	}
	/* Enable timer interrupts */
	NVIC_Init(&NVIC_InitStructure);

	/* Init GPIO TODO: check correct port (PPM Input : PA10) */
	px4fmu_gpio_init();

	/* initialise timer */
	tim_str = stm32_tim_init(ppm_cfg.timer);

	/* Set input capture mode */
	if(STM32_SET_CHANNEL(tim_str,ppm_cfg.channel,STM32_TIM_CH_INCAPTURE) != OK)
	{
		/* handle error here */
	}

	/* Configure timer clocks */
	//TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure = ppm_cfg.tim_base_init;
	//TIM_InternalClockConfig(ppm_cfg.timer);
	//TIM_TimeBaseInit(ppm_cfg.timer, &TIM_TimeBaseStructure);

	/* Enable the Capture Compare Interrupt Request */
	TIM_ITConfig(ppm_cfg.timer, ppm_cfg.ccr | TIM_IT_Update, ENABLE);


	/* Enable timers */
	//TIM_Cmd(ppm_cfg.timer, ENABLE);

	/* Setup local variable which stays in this scope */
	/* Doing this here and using a local variable saves doing it in the ISR */
	//TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;
	//TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;
	//TIM_ICInitStructure.TIM_ICFilter = 0x0;
	/*
	if (!PIOS_RTC_RegisterTickCallback(PPM_Supervisor, 0)) {
		DEBUG_Assert(0);
	}
	*/

	// TODO: register driver for access
	register_driver("/dev/ppm", &ppm_fops, 0666, NULL);
}

/**
* Get the value of an input channel
* \param[in] Channel Number of the channel desired
* \output -1 Channel not available
* \output >0 Channel value
*/
static int32_t PPM_Get(uint32_t rcvr_id, uint8_t channel)
{
	/* Return error if channel not available */
	if (channel >= PPM_IN_MAX_NUM_CHANNELS) {
		return -1;
	}
	return CaptureValue[channel];
}

/**
* Handle TIMx global interrupt request
* Some work and testing still needed, need to detect start of frame and decode pulses
*
*/
void PPM_irq_handler(void)
{
	/* Timer Overflow Interrupt
	 * The time between timer overflows must be greater than the PPM
	 * frame period. If a full frame has not decoded in the between timer
	 * overflows then capture values should be cleared.
	 */

	//if (TIM_GetITStatus(ppm_cfg.timer, TIM_IT_Update) == SET) {
	if (ENABLEINT(tim_str,irqchannel)
	{
		/* Clear TIMx overflow interrupt pending bit */
		TIM_ClearITPendingBit(ppm_cfg.timer, TIM_IT_Update);

		/* If sharing a timer with a servo output the ARR register will
		   be set according to the PWM period. When timer reaches the
		   ARR value a timer overflow interrupt will fire. We use the
		   interrupt accumulate a 32-bit timer. */
		LargeCounter = LargeCounter + ppm_cfg.timer->ARR;
	}

	/* Signal edge interrupt
	 *   itstatus = TIMx->SR & TIM_IT;

  	  itenable = TIMx->DIER & TIM_IT;
  */
	if (TIM_GetITStatus(ppm_cfg.timer, ppm_cfg.ccr) == SET) {	// checks if the interrupt has occurred
		PreviousTime = CurrentTime;

		switch((int32_t) ppm_cfg.ccr) {
			case (int32_t)TIM_IT_CC1:
				//CurrentTime = TIM_GetCapture1(ppm_cfg.timer);
				tim_str.ops->getcapture;
				STM32_TIM_GETCAPTURE(tim_str,1);
				break;
			case (int32_t)TIM_IT_CC2:
				CurrentTime = STM32_TIM_GETCAPTURE(tim_str,2);
				break;
			case (int32_t)TIM_IT_CC3:
				CurrentTime = STM32_TIM_GETCAPTURE(tim_str,3);
				break;
			case (int32_t)TIM_IT_CC4:
				CurrentTime = STM32_TIM_GETCAPTURE(tim_str,4);
				break;
		}

		/* Clear TIMx Capture compare interrupt pending bit */
		TIM_ClearITPendingBit(ppm_cfg.timer, ppm_cfg.ccr);

		/* Convert to 32-bit timer result */
		CurrentTime = CurrentTime + LargeCounter;

		/* Capture computation */
		DeltaTime = CurrentTime - PreviousTime;

		PreviousTime = CurrentTime;

		/* Sync pulse detection */
		if (DeltaTime > PPM_IN_MIN_SYNC_PULSE_US) {
			if (PulseIndex == NumChannelsPrevFrame
			 && PulseIndex >= PPM_IN_MIN_NUM_CHANNELS
			 && PulseIndex <= PPM_IN_MAX_NUM_CHANNELS)
			{
				/* If we see n simultaneous frames of the same
				 number of channels we save it as our frame size */
				if (NumChannelCounter < PPM_STABLE_CHANNEL_COUNT)
					NumChannelCounter++;
				else
					NumChannels = PulseIndex;
			} else {
				NumChannelCounter = 0;
			}

			/* Check if the last frame was well formed */
			if (PulseIndex == NumChannels && Tracking) {
				/* The last frame was well formed */
				for (uint32_t i = 0; i < NumChannels; i++) {
					CaptureValue[i] = CaptureValueNewFrame[i];
				}
				for (uint32_t i = NumChannels;
				     i < PPM_IN_MAX_NUM_CHANNELS; i++) {
					CaptureValue[i] = PPM_INPUT_INVALID;
				}
			}

			Fresh = TRUE;
			Tracking = TRUE;
			NumChannelsPrevFrame = PulseIndex;
			PulseIndex = 0;

			/* We rely on the supervisor to set CaptureValue to invalid
			 if no valid frame is found otherwise we ride over it */

		} else if (Tracking) {
			/* Valid pulse duration 0.75 to 2.5 ms*/
			if (DeltaTime > PPM_IN_MIN_CHANNEL_PULSE_US
			    && DeltaTime < PPM_IN_MAX_CHANNEL_PULSE_US
			    && PulseIndex < PPM_IN_MAX_NUM_CHANNELS) {

				CaptureValueNewFrame[PulseIndex] = DeltaTime;
				PulseIndex++;
			} else {
				/* Not a valid pulse duration */
				Tracking = FALSE;
				for (uint32_t i = 0; i < PPM_IN_MAX_NUM_CHANNELS ; i++) {
					CaptureValueNewFrame[i] = PPM_INPUT_INVALID;
				}
			}
		}
	}
}

static void PPM_Supervisor(uint32_t ppm_id) {
	/*
	 * RTC runs at 625Hz so divide down the base rate so
	 * that this loop runs at 25Hz.
	 */
	if(++supv_timer < 25) {
		return;
	}
	supv_timer = 0;

	if (!Fresh) {
		Tracking = FALSE;

		for (int32_t i = 0; i < PPM_IN_MAX_NUM_CHANNELS ; i++) {
			CaptureValue[i] = PPM_INPUT_INVALID;
			CaptureValueNewFrame[i] = PPM_INPUT_INVALID;
		}
	}

	Fresh = FALSE;
}

#endif

/**
  * @}
  * @}
  */


