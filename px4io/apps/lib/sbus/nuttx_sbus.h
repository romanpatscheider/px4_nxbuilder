/**
 ******************************************************************************
 * @addtogroup PIOS PIOS Core hardware abstraction layer
 * @{
 * @addtogroup   PIOS_SBUS S.Bus Functions
 * @brief PIOS interface to read and write from Futaba S.Bus port
 * @{
 *
 * @file       nuttx_sbus.h
 * @author     The OpenPilot Team, http://www.openpilot.org Copyright (C) 2011.
 * @brief      Futaba S.Bus Private structures.
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

#ifndef NUTTX_SBUS_H
#define NUTTX_SBUS_H

/*
 * S.Bus serial port settings:
 *  100000bps inverted serial stream, 8 bits, even parity, 2 stop bits
 *  frame period is 7ms (HS) or 14ms (FS)
 *
 * Frame structure:
 *  1 byte  - 0x0f (start of frame byte)
 * 22 bytes - channel data (11 bit/channel, 16 channels, LSB first)
 *  1 byte  - bit flags:
 *                   0x01 - digital channel 1,
 *                   0x02 - digital channel 2,
 *                   0x04 - lost frame flag,
 *                   0x08 - failsafe flag,
 *                   0xf0 - reserved
 *  1 byte  - 0x00 (end of frame byte)
 */
#define SBUS_FRAME_LENGTH		(1+22+1+1)
#define SBUS_SOF_BYTE			0x0f
#define SBUS_EOF_BYTE			0x00
#define SBUS_FLAG_DG1			0x01
#define SBUS_FLAG_DG2			0x02
#define SBUS_FLAG_FL			0x04
#define SBUS_FLAG_FS			0x08

/*
 * S.Bus protocol provides up to 16 analog and 2 digital channels.
 * Only 8 channels are currently supported by the OpenPilot.
 */
#define	SBUS_NUMBER_OF_CHANNELS		8


void NUTTX_SBUS_Supervisor();
int NUTTX_SBUS_Init(void);
uint8_t NUTTX_SBUS_Rx(int fd, uint8_t * buf, buf_len);




#endif /* NUTTX_SBUS_H */

/**
 * @}
 * @}
 */
