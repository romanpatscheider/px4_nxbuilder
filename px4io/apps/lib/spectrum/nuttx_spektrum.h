/**
 ******************************************************************************
 * @addtogroup NUTTX NUTTX Core hardware abstraction layer
 * @{
 * @addtogroup NUTTX_SPEKTRUM Nuttx Spektrum receiver functions
 * @brief Code to handle Spektrum input
 * @{
 *
 * @file       nuttx_spektrum.h
 * @author     The OpenPilot Team, http://www.openpilot.org Copyright (C) 2011.
	       ported by px4fixedwing
 * @brief      USART commands. Inits USARTs, controls USARTs & Interrupt handlers. (STM32 dependent)
 * @see        The GNU Public License (GPL) Version 3
 *
 *****************************************************************************/
/*
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY
 * or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License
 * for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA
 */

#ifndef NUTTX_SPEKTRUM_H
#define NUTTX_SPEKTRUM_H


#define SPEKTRUM_INPUT_PORT                     /dev/ttyS2
#define	SPEKTRUM_NUMBER_OF_CHANNELS		8


void NUTTX_SPEKTRUM_Supervisor();
int NUTTX_SPEKTRUM_Init(void);
uint8_t NUTTX_SPEKTRUM_Rx(int fd, uint8_t * buf, buf_len);




#endif NUTTX_SPEKTRUM_H

/**
 * @}
 * @}
 */
