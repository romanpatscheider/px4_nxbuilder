/* MAVLink adapter header */
#ifndef MAVLINK_BRIDGE_HEADER_H
#define MAVLINK_BRIDGE_HEADER_H
 
#define MAVLINK_USE_CONVENIENCE_FUNCTIONS
 
#include "mavlink-0.9/mavlink_types.h"
 
/* Struct that stores the communication settings of this system.
   you can also define / alter these settings elsewhere, as long
   as they're included BEFORE mavlink.h.
   So you can set the
 
   mavlink_system.sysid = 100; // System ID, 1-255
   mavlink_system.compid = 50; // Component/Subsystem ID, 1-255
 
   Lines also in your main.c, e.g. by reading these parameter from EEPROM.
 */
extern mavlink_system_t mavlink_system;
 
/**
 * @brief Send one char (uint8_t) over a comm channel
 *
 * @param chan MAVLink channel to use, usually MAVLINK_COMM_0 = UART0
 * @param ch Character to send
 */
static inline void comm_send_ch(mavlink_channel_t chan, uint8_t ch)
{
    if (chan == MAVLINK_COMM_0)
    {
        fputc(ch,stdout);
    }
}

/**
 * @brief Read one char (uint8_t) over a comm channel
 *
 * @param chan MAVLink channel to use, usually MAVLINK_COMM_0 = UART0
 * @param ch Character to read
 */
static inline uint8_t comm_receive_ch(mavlink_channel_t chan, uint8_t ch)
{
    ch = EOF;
    if (chan == MAVLINK_COMM_0)
    {
        // TODO need non-blocking read
        //ch = fgetc(stdin);
    }
    return ch;
}

#endif /* MAVLINK_BRIDGE_HEADER_H */
