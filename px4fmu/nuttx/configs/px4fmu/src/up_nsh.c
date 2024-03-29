/****************************************************************************
 * config/stm3210e_eval/src/up_nsh.c
 * arch/arm/src/board/up_nsh.c
 *
 *   Copyright (C) 2009, 2011 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <spudmonkey@racsa.co.cr>
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

#include <stdbool.h>
#include <stdio.h>
#include <unistd.h>
#include <debug.h>
#include <errno.h>

#include <nuttx/spi.h>
#include <nuttx/i2c.h>
#include <nuttx/mmcsd.h>
#include <nuttx/analog/adc.h>

#include "stm32_internal.h"
#include "px4fmu-internal.h"

#include "up_hrt.h"
#include "up_adc.h"
#include <arch/board/board.h>
#include <arch/board/drv_bma180.h>
#include <arch/board/drv_l3gd20.h>
#include <arch/board/drv_hmc5883l.h>
#include <arch/board/drv_ms5611.h>
#include <arch/board/drv_led.h>

/****************************************************************************
 * Pre-Processor Definitions
 ****************************************************************************/

/* Configuration ************************************************************/

/* Debug ********************************************************************/

#ifdef CONFIG_CPP_HAVE_VARARGS
#  ifdef CONFIG_DEBUG
#    define message(...) lib_lowprintf(__VA_ARGS__)
#  else
#    define message(...) printf(__VA_ARGS__)
#  endif
#else
#  ifdef CONFIG_DEBUG
#    define message lib_lowprintf
#  else
#    define message printf
#  endif
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: nsh_archinitialize
 *
 * Description:
 *   Perform architecture specific initialization
 *
 ****************************************************************************/

static struct spi_dev_s *spi1;
static struct spi_dev_s *spi3;
static struct i2c_dev_s *i2c2;

int nsh_archinitialize(void)
{
  int result;

  /* configure the high-resolution time/callout interface */
#ifdef CONFIG_HRT_TIMER
  hrt_init(CONFIG_HRT_TIMER);
#endif

  /* Initialise the user GPIOs */
  px4fmu_gpio_init();

  /* Initialize the user leds */
  up_ledinit(); // FIXME this might init the leds twice, but no harm

#ifdef CONFIG_ADC
  int adc_state = adc_devinit();
  if (adc_state != OK)
  {
	  message("adc_devinit failed: %d\n", adc_state);
	  return -ENODEV;
  }
#endif

  up_ledoff(LED_BLUE);
  up_ledoff(LED_AMBER);

  up_ledon(LED_BLUE);

  /* Configure user-space led driver */
  px4fmu_led_init();

  /* Configure SPI-based devices */

  spi1 = up_spiinitialize(1);
  if (!spi1)
  {
	  message("nsh_archinitialize: Failed to initialize SPI port 1\n");
	  up_ledon(LED_AMBER);
	  return -ENODEV;
  }

  // Setup 10 MHz clock (maximum rate the BMA180 can sustain)
  //10000000
  SPI_SETFREQUENCY(spi1, 10000000);
  SPI_SETBITS(spi1, 8);
  SPI_SETMODE(spi1, SPIDEV_MODE3);
  SPI_SELECT(spi1, PX4_SPIDEV_GYRO, false);
  SPI_SELECT(spi1, PX4_SPIDEV_ACCEL, false);
  usleep(20);

  message("nsh_archinitialize: Successfully initialized SPI port 1\n");

  /* initialize SPI peripherals redundantly */
  int gyro_attempts = 0;
  int gyro_fail = 0;

  while (gyro_attempts < 10)
  {
	  gyro_fail = l3gd20_attach(spi1, PX4_SPIDEV_GYRO);
	  gyro_attempts++;
	  if (gyro_fail == 0) break;
	  usleep(50);
  }

  int acc_attempts = 0;
  int acc_fail = 0;

  while (acc_attempts < 10)
  {
	  acc_fail = bma180_attach(spi1, PX4_SPIDEV_ACCEL);
	  acc_attempts++;
	  if (acc_fail == 0) break;
	  usleep(50);
  }


  /* initialize I2C peripherals redundantly */



  i2c2 = up_i2cinitialize(2);
  if (!i2c2) {
	  message("Failed to initialize I2C bus 2\n");
	  up_ledon(LED_AMBER);
	  return -ENODEV;
  }

  /* set I2C speed */
  I2C_SETFREQUENCY(i2c2, 400000);

  int mag_attempts = 0;
  int mag_fail = 0;

  while (mag_attempts < 10)
  {
	  mag_fail = hmc5883l_attach(i2c2);
	  mag_attempts++;
	  if (mag_fail == 0) break;
	  usleep(50);
  }

  int baro_attempts = 0;
  int baro_fail = 0;
  while (baro_attempts < 10)
  {
	  baro_fail = ms5611_attach(i2c2);
	  baro_attempts++;
	  if (baro_fail == 0) break;
	  usleep(50);
  }

  int eeprom_attempts = 0;
  int eeprom_fail = 0;

  // FIXME Report back sensor status
  if (acc_fail || gyro_fail || mag_fail || baro_fail || eeprom_fail)
  {
	  up_ledon(LED_AMBER);
  }

  // Init I2C bus



#if defined(CONFIG_STM32_SPI3)
  /* Get the SPI port */

  message("nsh_archinitialize: Initializing SPI port 3\n");
  spi3 = up_spiinitialize(3);
  if (!spi3)
    {
      message("nsh_archinitialize: Failed to initialize SPI port 3\n");
      up_ledon(LED_AMBER);
      return -ENODEV;
    }
  message("nsh_archinitialize: Successfully initialized SPI port 3\n");

  /* Now bind the SPI interface to the MMCSD driver */

  message("nsh_archinitialize: Bind SPI to the MMCSD driver\n");

  result = mmcsd_spislotinitialize(CONFIG_NSH_MMCSDMINOR, CONFIG_NSH_MMCSDSLOTNO, spi3);
  if (result != OK)
    {
      message("nsh_archinitialize: Failed to bind SPI port 3 to the MMCSD driver\n");
      up_ledon(LED_AMBER);
      return -ENODEV;
    }
  message("nsh_archinitialize: Successfully bound SPI port 3 to the MMCSD driver\n");
#endif /* SPI3 */

  return OK;
}
