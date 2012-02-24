/* MAVLink adapter header */
#ifndef MAVLINK_BRIDGE_HEADER_H
#define MAVLINK_BRIDGE_HEADER_H
 
#define MAVLINK_USE_CONVENIENCE_FUNCTIONS

//use efficient approach, see mavlink_helpers.h
#define MAVLINK_SEND_UART_BYTES mavlink_send_uart_bytes
 
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
    	//printf("in mavlink_send_uart_bytes\n");

//    	FILE *s0;
//		s0 = fopen("/dev/ttyS0","wb");
//		fputs ("a" , s0);
//		fclose(s0);
//
//    	FILE *s1;
//		s1 = fopen("/dev/ttyS1","wb");
//		fputs ("b" , s1);
//		fclose(s1);
//
//    	FILE *s2;
//		s2 = fopen("/dev/ttyS2","wb");
//		fputs ("c" , s2);
//		fclose(s2);
//
//    	FILE *s3;
//		s3 = fopen("/dev/ttyS3","wb");
//		fputs ("d" , s3);
//		fclose(s3);

		FILE *s0;
		s0 = fopen("/dev/ttyS0","wb");
		fwrite (ch, 1, length, s0);
		fclose(s0);

//		FILE *s1;
//		s0 = fopen("/dev/ttyS1","wb");
//		fwrite (ch, 1, length, s1);
//		fclose(s1);
//
//		FILE *s2;
//		s0 = fopen("/dev/ttyS2","wb");
//		fwrite (ch, 1, length, s2);
//		fclose(s2);
//
//		FILE *s3;
//		s0 = fopen("/dev/ttyS3","wb");
//		fwrite (ch, 1, length, s3);
//		fclose(s3);



    }
}

/**
 * @brief Read one char (uint8_t) over a comm channel
 *
 * @param chan MAVLink channel to use, usually MAVLINK_COMM_0 = UART0
 * @param ch Character to read
 */
static inline uint8_t comm_receive_ch(mavlink_channel_t chan, uint8_t ch, FILE *uart)
{
	printf("DEBUG: start comm_receive_ch \n");
	uint8_t * ch2;
	*ch2 = EOF;

    ch = EOF;
    if (chan == MAVLINK_COMM_0)
    {
        // TODO need non-blocking read

//        ch = fgetc(stdin);

    	printf("DEBUG: start reading char \n");



		//int res = fread (ch2, 1, 1, s0);
        ch = fgetc(uart);

		printf("DEBUG: read char: %x \n", ch);

    }
    return ch;
}


#endif /* MAVLINK_BRIDGE_HEADER_H */
