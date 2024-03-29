/****************************************************************************
 * arch/arm/src/stm32/stm32_start.c
 * arch/arm/src/chip/stm32_start.c
 *
 *   Copyright (C) 2009, 2011 Gregory Nutt. All rights reserved.
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
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>
#include <assert.h>
#include <debug.h>

#include <nuttx/init.h>
#include <arch/board/board.h>

#include "up_arch.h"
#include "up_internal.h"

#include "stm32_internal.h"
#include "stm32_gpio.h"

#ifdef CONFIG_ARCH_FPU
#  include "nvic.h"
#endif

/****************************************************************************
 * Name: showprogress
 *
 * Description:
 *   Print a character on the UART to show boot status.
 *
 ****************************************************************************/

#ifdef CONFIG_DEBUG
#  define showprogress(c) up_lowputc(c)
#else
#  define showprogress(c)
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

 /****************************************************************************
 * Name: stm32_fpuconfig
 *
 * Description:
 *   Configure the FPU.  The the MCU has an FPU, then enable full access
 *   to coprocessors CP10 and CP11.
 *
 *   Disable automatic FPU context save/restore, as the current context switch
 *   model does not know how to handle it.  This ensures that the processor
 *   will only stack a basic frame on exception entry.
 *
 ****************************************************************************/

#ifdef CONFIG_ARCH_FPU
#  define stm32_fpuconfig() \
{ \
  uint32_t regval = getreg32(NVIC_CPACR); \
  regval |= ((3 << (2*10)) | (3 << (2*11))); \
  putreg32(regval, NVIC_CPACR); \
  regval = getreg32(NVIC_FPCCR); \
  regval &= ~((1 << 31) | (1 << 30)); \
  putreg32(regval, NVIC_FPCCR); \
}
#else
#  define stm32_fpuconfig()
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: _start
 *
 * Description:
 *   This is the reset entry point.
 *
 ****************************************************************************/

void __start(void)
{
  const uint32_t *src;
  uint32_t *dest;

  /* Configure the uart so that we can get debug output as soon as possible */

  stm32_clockconfig();
  stm32_fpuconfig();
  stm32_lowsetup();
  stm32_gpioinit();
  showprogress('A');

  /* Clear .bss.  We'll do this inline (vs. calling memset) just to be
   * certain that there are no issues with the state of global variables.
   */

  for (dest = &_sbss; dest < &_ebss; )
    {
      *dest++ = 0;
    }
  showprogress('B');

  /* Move the intialized data section from his temporary holding spot in
   * FLASH into the correct place in SRAM.  The correct place in SRAM is
   * give by _sdata and _edata.  The temporary location is in FLASH at the
   * end of all of the other read-only data (.text, .rodata) at _eronly.
   */

  for (src = &_eronly, dest = &_sdata; dest < &_edata; )
    {
      *dest++ = *src++;
    }
  showprogress('C');

  /* Perform early serial initialization */

#ifdef CONFIG_USE_EARLYSERIALINIT
  up_earlyserialinit();
#endif
  showprogress('D');

  /* Initialize onboard resources */

  stm32_boardinitialize();
  showprogress('E');

  /* Then start NuttX */

  showprogress('\r');
  showprogress('\n');
  os_start();

  /* Shoulnd't get here */

  for(;;);
}
