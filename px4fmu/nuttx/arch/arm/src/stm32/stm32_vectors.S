/************************************************************************************
 * arch/arm/src/stm32/stm32_vectors.S
 * arch/arm/src/chip/stm32_vectors.S
 *
 *   Copyright (C) 2009-2011 Gregory Nutt. All rights reserved.
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
#include "exc_return.h"

#include "chip.h"

/************************************************************************************
 * Preprocessor Definitions
 ************************************************************************************/

/* Memory Map:
 *
 * 0x0800:0000 - Beginning of FLASH. Address of vectors (if not using bootloader)
 *               Mapped to address 0x0000:0000 at boot time.
 * 0x0800:3000 - Address of vectors if using bootloader
 * 0x0803:ffff - End of flash
 * 0x2000:0000 - Start of SRAM and start of .data (_sdata)
 *             - End of .data (_edata) abd start of .bss (_sbss)
 *             - End of .bss (_ebss) and bottom of idle stack
 *             - _ebss + CONFIG_IDLETHREAD_STACKSIZE = end of idle stack, start of heap
 * 0x2000:ffff - End of SRAM and end of heap
 */

#define IDLE_STACK      (_ebss+CONFIG_IDLETHREAD_STACKSIZE-4)
#define HEAP_BASE       (_ebss+CONFIG_IDLETHREAD_STACKSIZE-4)

/************************************************************************************
 * Global Symbols
 ************************************************************************************/

	.globl		__start
	.globl		exception_common

	.syntax		unified
	.thumb
	.file		"stm32_vectors.S"

/************************************************************************************
 * Macros
 ************************************************************************************/

/* On entry into an IRQ, the hardware automatically saves the xPSR, PC, LR, R12, R0-R3
 * registers on the stack, then branches to an instantantiation of the following
 * macro.  This macro simply loads the IRQ number into R0, then jumps to the common
 * IRQ handling logic.
 */

	.macro	HANDLER, label, irqno
	.thumb_func
\label:
	mov		r0, #\irqno
	b		exception_common
	.endm

/************************************************************************************
 * Vectors
 ************************************************************************************/

	.section	.vectors, "ax"
	.code		16
	.align		2
	.globl		stm32_vectors
	.type		stm32_vectors, function

stm32_vectors:

/* Processor Exceptions */

	.word	IDLE_STACK			/* Vector  0: Reset stack pointer */
	.word	__start				/* Vector  1: Reset vector */
	.word	stm32_nmi			/* Vector  2: Non-Maskable Interrupt (NMI) */
	.word	stm32_hardfault		/* Vector  3: Hard fault */
	.word	stm32_mpu			/* Vector  4: Memory management (MPU) */
	.word	stm32_busfault		/* Vector  5: Bus fault */
	.word	stm32_usagefault	/* Vector  6: Usage fault */
	.word	stm32_reserved		/* Vector  7: Reserved */
	.word	stm32_reserved		/* Vector  8: Reserved */
	.word	stm32_reserved		/* Vector  9: Reserved */
	.word	stm32_reserved		/* Vector 10: Reserved */
	.word	stm32_svcall		/* Vector 11: SVC call */
	.word	stm32_dbgmonitor	/* Vector 12: Debug monitor */
	.word	stm32_reserved		/* Vector 13: Reserved */
	.word	stm32_pendsv		/* Vector 14: Pendable system service request */
	.word	stm32_systick		/* Vector 15: System tick */

/* External Interrupts */

#undef VECTOR
#define VECTOR(l,i) .word l

#if defined(CONFIG_STM32_STM32F10XX)
#  include "chip/chip/stm32f10xxx_vectors.h"
#elif defined(CONFIG_STM32_STM32F40XX)
#  include "chip/chip/stm32f40xxx_vectors.h"
#else
#  error "No vectors for STM32 chip"
#endif
	.size	stm32_vectors, .-stm32_vectors

/************************************************************************************
 * .text
 ************************************************************************************/

	.text
	.type	handlers, function
	.thumb_func
handlers:
	HANDLER	stm32_reserved, STM32_IRQ_RESERVED		/* Unexpected/reserved vector */
	HANDLER	stm32_nmi, STM32_IRQ_NMI				/* Vector  2: Non-Maskable Interrupt (NMI) */
	HANDLER	stm32_hardfault, STM32_IRQ_HARDFAULT	/* Vector  3: Hard fault */
	HANDLER	stm32_mpu, STM32_IRQ_MEMFAULT			/* Vector  4: Memory management (MPU) */
	HANDLER	stm32_busfault, STM32_IRQ_BUSFAULT		/* Vector  5: Bus fault */
	HANDLER	stm32_usagefault, STM32_IRQ_USAGEFAULT	/* Vector  6: Usage fault */
	HANDLER	stm32_svcall, STM32_IRQ_SVCALL			/* Vector 11: SVC call */
	HANDLER	stm32_dbgmonitor, STM32_IRQ_DBGMONITOR	/* Vector 12: Debug Monitor */
	HANDLER	stm32_pendsv, STM32_IRQ_PENDSV			/* Vector 14: Penable system service request */
	HANDLER	stm32_systick, STM32_IRQ_SYSTICK		/* Vector 15: System tick */

#undef VECTOR
#define VECTOR(l,i) HANDLER l, i

#if defined(CONFIG_STM32_STM32F10XX)
#  include "chip/chip/stm32f10xxx_vectors.h"
#elif defined(CONFIG_STM32_STM32F40XX)
#  include "chip/chip/stm32f40xxx_vectors.h"
#else
#  error "No handlers for STM32 chip"
#endif

/************************************************************************************
 * .rodata
 ************************************************************************************/

	.section	.rodata, "a"

/* Variables: _sbss is the start of the BSS region (see ld.script) _ebss is the end
 * of the BSS regsion (see ld.script). The idle task stack starts at the end of BSS
 * and is of size CONFIG_IDLETHREAD_STACKSIZE.  The IDLE thread is the thread that
 * the system boots on and, eventually, becomes the idle, do nothing task that runs
 * only when there is nothing else to run.  The heap continues from there until the
 * end of memory.  See g_heapbase below.
 */

	.globl	g_heapbase
	.type	g_heapbase, object
g_heapbase:
	.long	_ebss+CONFIG_IDLETHREAD_STACKSIZE
	.size	g_heapbase, .-g_heapbase

	.end

