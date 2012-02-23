/****************************************************************************
 * drivers/sensors/lm75.c
 * Character driver for the STMicro LM-75 Temperature Sensor
 *
 *   Copyright (C) 2011 Gregory Nutt. All rights reserved.
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

#include <sys/types.h>

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <debug.h>

#include <arch/board/board.h>


#include <nuttx/i2c.h>

int lm75_readb16(uint8_t regaddr,
                        FAR b16_t *regvalue)
{
  uint8_t buffer[2];
  int ret;
  uint8_t

  /* Write the register address */

  I2C_SETADDRESS(priv->i2c, priv->addr, 7);
  ret = I2C_WRITE(priv->i2c, &regaddr, 1);
  if (ret < 0)
    {
      lm75dbg("I2C_WRITE failed: %d\n", ret);
      return ret;
    }

  /* Restart and read 16-bits from the register (discarding 7) */

  ret = I2C_READ(priv->i2c, buffer, 2);
  if (ret < 0)
    {
      lm75dbg("I2C_READ failed: %d\n", ret);
      return ret;
    }

  /* Data format is:  TTTTTTTT Txxxxxxx where TTTTTTTTT is a nine-bit,
   * signed temperature value with LSB = 0.5 degrees centigrade.  So the
   * raw data is b8_t
   */

  *regvalue = b8tob16((b8_t)buffer[0] << 8 | (b8_t)buffer[1]);
  lm75dbg("addr: %02x value: %08x ret: %d\n", regaddr, *regvalue, ret);
  return OK;
}


//static int lm75_writeb16(FAR struct lm75_dev_s *priv, uint8_t regaddr,
//                         b16_t regval)
//{
//  uint8_t buffer[3];
//  b8_t regb8;
//
//  lm75dbg("addr: %02x value: %08x\n", regaddr, regval);
//
//  /* Set up a 3 byte message to send */
//
//  buffer[0] = regaddr;
//
//  regb8 = b16tob8(regval);
//  buffer[1] = (uint8_t)(regb8 >> 8);
//  buffer[2] = (uint8_t)regb8;
//
//  /* Write the register address followed by the data (no RESTART) */
//
//  I2C_SETADDRESS(priv->i2c, priv->addr, 7);
//  return I2C_WRITE(priv->i2c, buffer, 3);
//}
