/*
 * mtk.c
 *
 *  Created on: Mar 19, 2012
 *      Author: julianoes
 */

#include "mtk.h"

pthread_mutex_t mtk_mutex;
gps_bin_mtk_state_t * mtk_state;


void mtk_decode_init()
{
	mtk_state->ck_a = 0;
	mtk_state->ck_b = 0;
	mtk_state->rx_count = 0;
	mtk_state->decode_state = MTK_DECODE_UNINIT;
	mtk_state->print_errors = false;
}

void mtk_checksum(uint8_t b, uint8_t* ck_a, uint8_t* ck_b)
{
	*(ck_a) = *(ck_a) + b;
	*(ck_b) = *(ck_b) + *(ck_a);
//	printf("Checksum now: %d\n",*(ck_b));
}



int mtk_parse(uint8_t b,  char * gps_rx_buffer, gps_bin_mtk_state_t * mtk_state, nmeaINFO * info)
{
//	printf("b=%x\n",b);
	// Debug output to telemetry port
	//	PIOS_COM_SendBufferNonBlocking(PIOS_COM_TELEM_RF, &b, 1);
		if (mtk_state->decode_state == MTK_DECODE_UNINIT)
		{

			if (b == 0xd0)
				{
					mtk_state->decode_state = MTK_DECODE_GOT_CK_A;
				}
		}
		else if (mtk_state->decode_state == MTK_DECODE_GOT_CK_A)
		{
			if (b == 0xdd)
			{
				mtk_state->decode_state = MTK_DECODE_GOT_CK_B;
			}
			else
			{
				// Second start symbol was wrong, reset state machine
				mtk_decode_init(mtk_state);
			}
		}
		else if (mtk_state->decode_state == MTK_DECODE_GOT_CK_B)
		{
			// Add to checksum
			if (mtk_state->rx_count < 33) mtk_checksum(b, &(mtk_state->ck_a), &(mtk_state->ck_b));
			// Fill packet buffer
			gps_rx_buffer[mtk_state->rx_count] = b;
			(mtk_state->rx_count)++;
//			printf("Rx count: %d\n",mtk_state->rx_count);
			uint8_t ret = 0;

			// Packet size minus checksum
			if (mtk_state->rx_count >= 35)
			{
				gps_bin_mtk_packet_t* packet = (gps_bin_mtk_packet_t*) gps_rx_buffer;
				//Check if checksum is valid
				if (mtk_state->ck_a == packet->ck_a && mtk_state->ck_b == packet->ck_b)
				{
//					info->lat				= packet->latitude;   // degrees * 10e6  //TODO types, order
//					info->lon      = packet->longitude;	// degrees * 10e6
//					info->elv        = packet->msl_altitude;                                       // meters
//	//				GpsData.GeoidSeparation = 5000;                                 // meters
//	//				GpsData.Heading         = packet->heading / 1000;                                 // degrees
//	//				GpsData.Groundspeed     = packet->ground_speed / 3600;                                  // m/s
//					info->satinfo.inview    = packet->satellites;                                              //TODO: not sure if this are in use or in view satellites
//	//				GpsData.PDOP            = packet->hdop / 100.0f;                                                                            // not available in binary mode
//	//				GpsData.HDOP            = packet->hdop / 100.0f;                                                //
//	//				GpsData.VDOP            = 99.99;                                                                            // not available in binary mode
//					//				mtk_state.new_data = true;
//					//GPSPositionSet(&GpsData);
//
//					info->fix				= packet->fix_type;
//
//
//					uint32_t utc_time = packet->utc_time / 1000; //TODO: this code was commented out in the original file!
//					info->utc.sec = utc_time % 100;          // seconds
//					info->utc.min = (utc_time / 100) % 100;  // minutes
//					info->utc.hour = utc_time / 10000;          // hours
//					info->utc.day = packet->date;        // day //TODO: look at conent of date and convert also to month and year
//					info->utc.mon = 0;    // month
//					info->utc.year = 0;      // year

					global_data_lock(&global_data_sys_status.access_conf);
					global_data_gps.lat = packet->latitude;
					global_data_gps.lon = packet->longitude;
					global_data_gps.alt = packet.msl_altitude;
					global_data_gps.fix_type = packet->fix_type;
					global_data_gps.eph = packet->hdop;
					global_data_gps.epv = packet->vdop;
					global_data_gps.vel = packet->ground_speed;
					global_data_gps.cog = 65535;
					global_data_gps->satellites_visible = packet->satellites;

					global_data_gps.counter++;
					global_data_unlock(&global_data_sys_status.access_conf);

					pthread_mutex_lock(mtk_mutex);
					mtk_state->last_message_timestamp = global_data_get_timestamp_milliseconds();
					pthread_mutex_unlock(mtk_mutex);

					ret = 1;
					printf("found package\n");
				}
				else
				{
					printf("checksum invalid\n");
					ret = 0;
				}
				// Reset state machine to decode next packet
				mtk_decode_init(mtk_state);
				printf("prepared for next state\n");
				return ret;
			}
		}
		return 0;     // no valid packet found

}

int read_gps_mtk(int fd, char * gps_rx_buffer, int buffer_size)
{

	uint8_t c;
	int start_flag = 0;
	int found_cr = 0;
	int rx_count = 0;
	int gpsRxOverflow = 0;
	int numChecksumErrors = 0;
	int numParsingErrors = 0;
	int numUpdates = 0;

//	mtk_decode_init(&mtk_state);
//	mtk_state.print_errors = false;


	 // NMEA or SINGLE-SENTENCE GPS mode

	// This blocks the task until there is something on the buffer
	while (read(fd, &c, 1) > 0)
	{
		if (rx_count >= buffer_size)
		{
			// The buffer is already full and we haven't found a valid NMEA sentence.
			// Flush the buffer and note the overflow event.
			gpsRxOverflow++;
			start_flag = 0;
			found_cr = 0;
			rx_count = 0;
			mtk_decode_init(mtk_state);
			printf("Buffer full\n");
		}
		else
		{
			   //gps_rx_buffer[rx_count] = c;
			   rx_count++;

		}

		int msg_read = mtk_parse(c, gps_rx_buffer, mtk_state, info);

//		printf("nmea msg_read = %d\n", msg_read);

		if(msg_read > 0)
		{
			printf("Found sequence\n");
			break;
		}

	}

	return 0;
}

int configure_gps_mtk(int fd)
{
	int success = 0;
	size_t result_write;
	result_write =  write(fd, MEDIATEK_REFRESH_RATE_10HZ, strlen(MEDIATEK_REFRESH_RATE_10HZ));
	if(result_write != strlen(MEDIATEK_REFRESH_RATE_10HZ))
	{
		printf("Set update speed to 10 Hz failed\n");
		success = 1;
	}
	else
	{
		printf("Set update speed to 10 Hz successful\n");
	}
	//set custom mode
	result_write =  write(fd, MEDIATEK_CUSTOM_BINARY_MODE, strlen(MEDIATEK_CUSTOM_BINARY_MODE));
	if(result_write != strlen(MEDIATEK_CUSTOM_BINARY_MODE))
	{
		printf("Set custom mode failed");
		success = 1;
	}
	else
	{
		printf("Set custom mode successful");
	}

	return success;
}

void *mtk_loop(void * arg)
{
	/* Retrieve file descriptor */
	int fd = *((int *)arg);

	/* Initialize gps stuff */
	int buffer_size = 1000;
	char * gps_rx_buffer = malloc(buffer_size*sizeof(char));

	int config_not_finished = 1; //is set to 0 as soon as all configurations are completed
	bool configured = false;

	/* set parameters for mtk custom */

	if (configure_gps_mtk(fd) != 0)
	{
		printf("Configuration of gps module to mtk custom failed\n");

		/* Write shared variable sys_status */

		global_data_lock(&global_data_sys_status.access_conf);
		global_data_sys_status.onboard_control_sensors_present |= 1 << 5;//TODO: write wrapper for bitmask
		global_data_sys_status.onboard_control_sensors_enabled &= ~(1 << 5);
		global_data_sys_status.onboard_control_sensors_health &= ~(1 << 5);
		global_data_sys_status.counter++;
		global_data_sys_status.timestamp = global_data_get_timestamp_milliseconds();
		global_data_unlock(&global_data_sys_status.access_conf);
	}
	else
	{
		printf("Configuration of gps module to mtk custom successful\n");


		/* Write shared variable sys_status */

		global_data_lock(&global_data_sys_status.access_conf);
		global_data_sys_status.onboard_control_sensors_present |= 1 << 5;//TODO: write wrapper for bitmask
		global_data_sys_status.onboard_control_sensors_enabled |= 1 << 5;
		global_data_sys_status.counter++;
		global_data_sys_status.timestamp = global_data_get_timestamp_milliseconds();
		global_data_unlock(&global_data_sys_status.access_conf);
	}

	/* Inform the other processes that there is new sys status data available */
	global_data_broadcast(&global_data_sys_status.access_conf);

	while(1)
	{
		/* Parse a message from the gps receiver */
		read_gps_mtk(fd, gps_rx_buffer, buffer_size);

		/* Inform the other processes that there is new gps data available */
		global_data_broadcast(&global_data_gps.access_conf);
	}

	//close port
	close_port(fd);

	return 0;

}

void mtk_init(void)
{

	/* Init mutex for datasharing between custom gps reading thread and  custom_watchdog thread*/
	pthread_mutex_init(&mtk_mutex, NULL);

	mtk_state = malloc(sizeof(gps_bin_mtk_state_t));
	mtk_decode_init();


}
