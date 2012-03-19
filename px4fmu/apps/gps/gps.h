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
#include <signal.h>


int open_port(char * port)
{
	int fd; // File descriptor for the port

	// Open serial port
	// O_RDWR - Read and write
	// O_NOCTTY - Ignore special chars like CTRL-C
	fd = open(port, O_CREAT|O_RDWR | O_NOCTTY);
	if (fd == -1)
	{
	   // Could not open the port.
	   printf("Error: could not open the GPS port\n");
	   return(-1);
	}
	return (fd);
}


void close_port(int fd)
{
       close(fd);
}

#endif /* GPS_H_ */
