/*
 * nmea_helper.c
 *
 *  Created on: Mar 19, 2012
 *      Author: julianoes
 */

#include "nmea_helper.h"
//#include "nmealib/nmea/nmea.h"


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
		// detect start while acquiring stream
		if (!start_flag && (c == '$'))
		{
			start_flag = 1;
			found_cr = 0;
			rx_count = 0;
		}
		else if (!start_flag) // keep looking for start sign
		{
			continue;
		}

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
			// store chars in buffer
		    gps_rx_buffer[rx_count] = c;
		    rx_count++;
		}

		// look for carriage return CR
		if (start_flag && c == 0x0d)
		{
			found_cr = 1;
		}

		// and then look for line feed LF
		if (start_flag && found_cr && c == 0x0a)
		{
			break;
		}
	}

	// parse one NMEA line, use buffer up to rx_count
	int msg_read = nmea_parse(parser, gps_rx_buffer, rx_count, info);

	// As soon as one NMEA message has been parsed, we break out of the loop and end here
	return(0);
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
