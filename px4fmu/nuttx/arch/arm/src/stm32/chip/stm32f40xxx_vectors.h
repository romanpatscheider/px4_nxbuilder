/************************************************************************************
 * arch/arm/src/stm32/chip/stm32f40xxx_vectors.h
 *
 *   Copyright (C) 2011 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
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
 ************************************************************************************/

/************************************************************************************
 * Pre-processor definitions
 ************************************************************************************/

/* Reserve 84 interrupt table entries for I/O interrupts.
*/

#define ARMV7M_PERIPHERAL_INTERRUPTS	82

/* Vector 16+0:  Window Watchdog interrupt */
/* Vector 16+1:  PVD through EXTI Line detection interrupt */
/* Vector 16+2:  Tamper and time stamp interrupts */
/* Vector 16+3:  RTC global interrupt */
/* Vector 16+4:  Flash global interrupt */
/* Vector 16+5:  RCC global interrupt */
/* Vector 16+6:  EXTI Line 0 interrupt */
/* Vector 16+7:  EXTI Line 1 interrupt */
/* Vector 16+8:  EXTI Line 2 interrupt */
/* Vector 16+9:  EXTI Line 3 interrupt */
/* Vector 16+10: EXTI Line 4 interrupt */
/* Vector 16+11: DMA1 Stream 0 global interrupt */
/* Vector 16+12: DMA1 Stream 1 global interrupt */
/* Vector 16+13: DMA1 Stream 2 global interrupt */
/* Vector 16+14: DMA1 Stream 3 global interrupt */
/* Vector 16+15: DMA1 Stream 4 global interrupt */
/* Vector 16+16: DMA1 Stream 5 global interrupt */
/* Vector 16+17: DMA1 Stream 6 global interrupt */
/* Vector 16+18: ADC1, ADC2, and ADC3 global interrupt */
/* Vector 16+19: CAN1 TX interrupts */
/* Vector 16+20: CAN1 RX0 interrupts */
/* Vector 16+21: CAN1 RX1 interrupt */
/* Vector 16+22: CAN1 SCE interrupt */
/* Vector 16+23: EXTI Line[9:5] interrupts */
/* Vector 16+24: TIM1 Break interrupt/TIM9 global interrupt */
/* Vector 16+25: TIM1 Update interrupt/TIM10 global interrupt */
/* Vector 16+26: TIM1 Trigger and Commutation interrupts/TIM11 global interrupt */
/* Vector 16+27: TIM1 Capture Compare interrupt */
/* Vector 16+28: TIM2 global interrupt */
/* Vector 16+29: TIM3 global interrupt */
/* Vector 16+30: TIM4 global interrupt */
/* Vector 16+31: I2C1 event interrupt */
/* Vector 16+32: I2C1 error interrupt */
/* Vector 16+33: I2C2 event interrupt */
/* Vector 16+34: I2C2 error interrupt */
/* Vector 16+35: SPI1 global interrupt */
/* Vector 16+36: SPI2 global interrupt */
/* Vector 16+37: USART1 global interrupt */
/* Vector 16+38: USART2 global interrupt */
/* Vector 16+39: USART3 global interrupt */
/* Vector 16+40: EXTI Line[15:10] interrupts */
/* Vector 16+41: RTC alarm through EXTI line interrupt */
/* Vector 16+42: USB On-The-Go FS Wakeup through EXTI line interrupt */
/* Vector 16+43: TIM8 Break interrupt/TIM12 global interrupt */
/* Vector 16+44: TIM8 Update interrup/TIM13 global interrupt */
/* Vector 16+45: TIM8 Trigger and Commutation interrupts/TIM14 global interrupt */
/* Vector 16+46: TIM8 Capture Compare interrupt */
/* Vector 16+47: DMA1 Stream 7 global interrupt */
/* Vector 16+48: FSMC global interrupt */
/* Vector 16+49: SDIO global interrupt */
/* Vector 16+50: TIM5 global interrupt */
/* Vector 16+51: SPI3 global interrupt */
/* Vector 16+52: UART4 global interrupt */
/* Vector 16+53: UART5 global interrupt */
/* Vector 16+54: TIM6 global interrupt/DAC1 and DAC2 underrun error interrupts */
/* Vector 16+55: TIM7 global interrupt */
/* Vector 16+56: DMA2 Stream 0 global interrupt */
/* Vector 16+57: DMA2 Stream 1 global interrupt */
/* Vector 16+58: DMA2 Stream 2 global interrupt */
/* Vector 16+59: DMA2 Stream 3 global interrupt */
/* Vector 16+60: DMA2 Stream 4 global interrupt */
/* Vector 16+61: Ethernet global interrupt */
/* Vector 16+62: Ethernet Wakeup through EXTI line interrupt */
/* Vector 16+63: CAN2 TX interrupts */
/* Vector 16+64: CAN2 RX0 interrupts */
/* Vector 16+65: CAN2 RX1 interrupt */
/* Vector 16+66: CAN2 SCE interrupt */
/* Vector 16+67: USB On The Go FS global interrupt */
/* Vector 16+68: DMA2 Stream 5 global interrupt */
/* Vector 16+69: DMA2 Stream 6 global interrupt */
/* Vector 16+70: DMA2 Stream 7 global interrupt */
/* Vector 16+71: USART6 global interrupt */
/* Vector 16+72: I2C3 event interrupt */
/* Vector 16+73: I2C3 error interrupt */
/* Vector 16+74: USB On The Go HS End Point 1 Out global interrupt */
/* Vector 16+75: USB On The Go HS End Point 1 In global interrupt */
/* Vector 16+76: USB On The Go HS Wakeup through EXTI interrupt */
/* Vector 16+77: USB On The Go HS global interrupt */
/* Vector 16+78: DCMI global interrupt */
/* Vector 16+79: CRYP crypto global interrupt */
/* Vector 16+80: Hash and Rng global interrupt */
/* Vector 16+81: FPU global interrupt */

