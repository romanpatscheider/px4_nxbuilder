/*
 * gps.h
 *
 *  Created on: Mar 8, 2012
 *      Author: thomasgubler
 */

#ifndef GPS_H_
#define GPS_H_

#include <termios.h>
#include <stdio.h>
#include <string.h>
#include <fcntl.h>
#include "nmealib/nmea/nmea.h"

int open_port(char * port)
{
       int fd; /* File descriptor for the port */

       // Open serial port
       // O_RDWR - Read and write
       // O_NOCTTY - Ignore special chars like CTRL-C
       fd = open(port, O_RDWR | O_NOCTTY);
       if (fd == -1)
       {
               /* Could not open the port. */
               return(-1);
       }
//       else
//       {
//               fcntl(fd, F_SETFL, 0);
//       }

       return (fd);
}

void close_port(int fd)
{
       close(fd);
}

int read_gps_nmea(int fd, char * gps_rx_buffer, int buffer_size, nmeaINFO * info, nmeaPARSER * parser)
{

	char c;
	int start_flag = 0;
	int found_cr = 0;
	int rx_count = 0;
	int gpsRxOverflow = 0;
	int numChecksumErrors = 0;
	int numParsingErrors = 0;
	int numUpdates = 0;


	 // NMEA or SINGLE-SENTENCE GPS mode

	// This blocks the task until there is something on the buffer
	while (read(fd, &c, 1) > 0)
	{
//		printf("Read %c, res: %d\n",c, res);
//		sleep(1);
		// detect start while acquiring stream
		if (!start_flag && (c == '$'))
		{
			   start_flag = 1;
			   found_cr = 0;
			   rx_count = 0;
		}
		else
		if (!start_flag)
			   continue;


		if (rx_count >= buffer_size)
		{
		   // The buffer is already full and we haven't found a valid NMEA sentence.
		   // Flush the buffer and note the overflow event.
		   gpsRxOverflow++;
		   start_flag = 0;
		   found_cr = 0;
		   rx_count = 0;
		   printf("Buffer full\n");
		}
		else
		{
			   gps_rx_buffer[rx_count] = c;
			   rx_count++;

		}

		int msg_read = nmea_parse(parser, gps_rx_buffer, rx_count, info);
//		printf("nmea msg_read = %d\n", msg_read);

		if(msg_read > 0)
		{
//			printf("Found sequence\n");
			break;
		}

	}


	return 0;
}

/**
 * \brief Convert NDEG (NMEA degree) to fractional degree
 */
float ndeg2degree(float val)
{
    float deg = ((int)(val / 100));
    val = deg + (val - deg * 100) / 60;
    return val;
}




#endif /* GPS_H_ */
