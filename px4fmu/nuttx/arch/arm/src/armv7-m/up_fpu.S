/************************************************************************************
 * arch/arm/src/armv7-m/stm32_fpu.S
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
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>

#include <arch/irq.h>

#include "nvic.h"

#ifdef CONFIG_ARCH_FPU

/************************************************************************************
 * Preprocessor Definitions
 ************************************************************************************/

/************************************************************************************
 * Global Symbols
 ************************************************************************************/

 	.globl		up_initfpu
	.globl		up_savefpu
	.globl		up_restorefpu

	.syntax		unified
	.thumb
	.file		"up_fpu.S"

/************************************************************************************
 * Public Functions
 ************************************************************************************/

/************************************************************************************
 * Name: up_initfpu
 *
 * Description:
 *   Perform one-time FPU initialisation at early startup time.
 *
 *   Set CONTROL.FPCA so that we always get the extended context frame with
 *   the volatile FP registers stacked above the basic context.
 *
 *   Ensure that FPCCR.LSPEN is disabled, so that we don't have to contend
 *   with the lazy FP context save behaviour.  Clear FPCCR.ASPEN since we
 *   are going to turn on CONTROL.FPCA for all contexts.
 *
 * C Function Prototype:
 *   void up_initfpu(void);
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None
 * 
 ************************************************************************************/

	.thumb_func
	.type	up_initfpu, function
up_initfpu:
	mrs	r0, control			/* Set CONTROL.FPCA */
	orr	r0, r0, #4
	msr	control, r0

	movw	r1, #(NVIC_FPCCR & 0xffff)	/* Clear FPCCR.LSPEN, FPCCR.ASPEN */
	movt	r1, #(NVIC_FPCCR >> 16)
	ldr	r0, [r1]
	bic	r0, r0, #(1<<31 | 1<<30)
	str	r0, [r1]

	movw	r1, #(NVIC_CPACR & 0xffff)	/* Enable CP10 and CP11 */
	movt	r1, #(NVIC_CPACR >> 16)
	ldr	r0, [r1]
	orr	r0, r0, #((3<<(2*10)) | (3<<(2*11)))
	str	r0, [r1]

	bx		lr

	.size	up_initfpu, .-up_initfpu


/************************************************************************************
 * Name: up_restorefpu
 *
 * Description:
 *   Given the pointer to a register save area (in R0), save the state of the
 *   non-volatile floating point registers.
 *
 *   Note that we clobber R2 here.  R1 preserved to simplify the calling context.
 *
 * C Function Prototype:
 *   void up_savefpu(uint32_t *regs);
 *
 * Input Parameters:
 *   regs - A pointer to the register save area in which to save the floating point
 *     registers
 *
 * Returned Value:
 *   None
 * 
 ************************************************************************************/

	.thumb_func
	.type	up_savefpu, function
up_savefpu:
	add		r2, r0, #(4*REG_S16)    /* R2=Address of FP register storage */
	vstm	r2, {s16-s31}           /* save s16-s31 */
	bx		lr

	.size	up_savefpu, .-up_savefpu

/************************************************************************************
 * Name: up_restorefpu
 *
 * Description:
 *   Given the pointer to a register save area (in R0), restore the state of the
 *   non-volatile floating point registers.
 *
 *   Note that we clobber R2 here.  R1 preserved to simplify the calling context.
 *
 * C Function Prototype:
 *   void up_restorefpu(const uint32_t *regs);
 *
 * Input Parameters:
 *   regs - A pointer to the register save area containing the floating point
 *     registers
 *
 * Returned Value:
 *   None
 * 
 ************************************************************************************/

	.thumb_func
	.type	up_restorefpu, function
up_restorefpu:
	add		r2, r0, #(4*REG_S16)	/* R2=Address of FP register storage */
	vldm	r2, {s16-s31}			/* restore s16-s31 */
	bx		lr

	.size	up_restorefpu, .-up_restorefpu
	.end

#endif /* CONFIG_ARCH_FPU */
