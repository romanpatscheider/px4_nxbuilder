/* MAVLink adapter header */
#ifndef MAVLINK_BRIDGE_HEADER_H
#define MAVLINK_BRIDGE_HEADER_H
 
#define MAVLINK_USE_CONVENIENCE_FUNCTIONS

//use efficient approach, see mavlink_helpers.h
#define MAVLINK_SEND_UART_BYTES mavlink_send_uart_bytes
 
#include <unistd.h>
#include "mavlink-1.0/mavlink_types.h"

 
/* Struct that stores the communication settings of this system.
   you can also define / alter these settings elsewhere, as long
   as they're included BEFORE mavlink.h.
   So you can set the
 
   mavlink_system.sysid = 100; // System ID, 1-255
   mavlink_system.compid = 50; // Component/Subsystem ID, 1-255
 
   Lines also in your main.c, e.g. by reading these parameter from EEPROM.
 */
extern mavlink_system_t mavlink_system;

int uart_read;
int uart_write;
int ardrone_write;
 
/**
 * @brief Send one char (uint8_t) over a comm channel
 *
 * @param chan MAVLink channel to use, usually MAVLINK_COMM_0 = UART0
 * @param ch Character to send
 */
static inline void mavlink_send_uart_bytes(mavlink_channel_t chan, uint8_t * ch, uint16_t length)
{

    if (chan == MAVLINK_COMM_0)
    {
//
//		FILE *uart;
//		uart = fopen("/dev/ttyS0","wb");
		write (uart_write, ch, length);
//		fclose(uart);


    }
}


#endif /* MAVLINK_BRIDGE_HEADER_H */
