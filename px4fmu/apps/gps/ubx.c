/*
 * ubx.c
 *
 *  Created on: Mar 19, 2012
 *      Author: julianoes
 */

#include "ubx.h"

pthread_mutex_t ubx_mutex;

gps_bin_ubx_state_t * ubx_state;


//Definitions for ubx, last two bytes are checksum which is calculated below
uint8_t UBX_CONFIG_MESSAGE_PRT[] = {0xB5 , 0x62 , 0x06 , 0x00 , 0x14 , 0x00 , 0x01 , 0x00 , 0x00 , 0x00 , 0xD0 , 0x08 , 0x00 , 0x00 , 0x80 , 0x25 , 0x00 , 0x00 , 0x07 , 0x00 , 0x01 , 0x00 , 0x00 , 0x00 , 0x00 , 0x00};
uint8_t UBX_CONFIG_MESSAGE_MSG_NAV_POSLLH[] = {0xB5, 0x62, 0x06, 0x01, 0x08, 0x00,   0x01, 0x02,   0x00, 0x01, 0x00, 0x00, 0x00, 0x00};
uint8_t UBX_CONFIG_MESSAGE_MSG_NAV_TIMEUTC[] = {0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0x01, 0x21, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00};
uint8_t UBX_CONFIG_MESSAGE_MSG_NAV_DOP[] = {0xB5, 0x62, 0x06, 0x01, 0x08, 0x00,   0x01, 0x04,   0x00, 0x01, 0x00, 0x00, 0x00, 0x00};
uint8_t UBX_CONFIG_MESSAGE_MSG_NAV_SVINFO[] = {0xB5, 0x62, 0x06, 0x01, 0x08, 0x00,   0x01, 0x30,   0x00, 0x02, 0x00, 0x00, 0x00, 0x00};
uint8_t UBX_CONFIG_MESSAGE_MSG_NAV_SOL[] = {0xB5, 0x62, 0x06, 0x01, 0x08, 0x00,   0x01, 0x06,   0x00, 0x02, 0x00, 0x00, 0x00, 0x00};
uint8_t UBX_CONFIG_MESSAGE_MSG_NAV_VELNED[] = {0xB5, 0x62, 0x06, 0x01, 0x08, 0x00,   0x01, 0x12,   0x00, 0x01, 0x00, 0x00, 0x00, 0x00};
uint8_t UBX_CONFIG_MESSAGE_MSG_RXM_SVSI[] = {0xB5, 0x62, 0x06, 0x01, 0x08, 0x00,   0x02, 0x20,   0x00, 0x02, 0x00, 0x00, 0x00, 0x00};

void ubx_decode_init(void)
{
	ubx_state->ck_a = 0;
	ubx_state->ck_b = 0;
	ubx_state->rx_count = 0;
	ubx_state->decode_state = UBX_DECODE_UNINIT;
	ubx_state->message_class = CLASS_UNKNOWN;
	ubx_state->message_id = ID_UNKNOWN;
	ubx_state->payload_size = 0;
	ubx_state->print_errors = false;
}

void ubx_checksum(uint8_t b, uint8_t* ck_a, uint8_t* ck_b)
{
	*(ck_a) = *(ck_a) + b;
	*(ck_b) = *(ck_b) + *(ck_a);
}





int ubx_parse(uint8_t b,  char * gps_rx_buffer)
{
//	printf("b=%x\n",b);
		if (ubx_state->decode_state == UBX_DECODE_UNINIT)
		{

			if (b == 0xb5)
				{
				ubx_state->decode_state = UBX_DECODE_GOT_SYNC1;
				}
		}
		else if (ubx_state->decode_state == UBX_DECODE_GOT_SYNC1)
		{
			if (b == 0x62)
			{
				ubx_state->decode_state = UBX_DECODE_GOT_SYNC2;
			}
			else
			{
				// Second start symbol was wrong, reset state machine
				ubx_decode_init();
			}
		}
		else if (ubx_state->decode_state == UBX_DECODE_GOT_SYNC2)
		{
			// Add to checksum
			ubx_checksum(b, &(ubx_state->ck_a), &(ubx_state->ck_b));

			//check for known class
			switch (b)
			{
			case UBX_CLASS_NAV: //NAV
				ubx_state->decode_state = UBX_DECODE_GOT_CLASS;
				ubx_state->message_class = NAV;
			break;
			case UBX_CLASS_RXM: //RXM
				ubx_state->decode_state = UBX_DECODE_GOT_CLASS;
				ubx_state->message_class = RXM;
			break;
			default: //unknown class: reset state machine
				ubx_decode_init();
			break;
			}

		}
		else if (ubx_state->decode_state == UBX_DECODE_GOT_CLASS)
		{
			// Add to checksum
			ubx_checksum(b, &(ubx_state->ck_a), &(ubx_state->ck_b));

			//depending on class look for message id
			switch (ubx_state->message_class)
			{
			case NAV:
				switch (b)
				{
				case UBX_MESSAGE_NAV_POSLLH: //NAV-POSLLH: Geodetic Position Solution
					ubx_state->decode_state = UBX_DECODE_GOT_MESSAGEID;
					ubx_state->message_id = NAV_POSLLH;
					break;
				case UBX_MESSAGE_NAV_SOL:
					ubx_state->decode_state = UBX_DECODE_GOT_MESSAGEID;
					ubx_state->message_id = NAV_SOL;
					break;
				case UBX_MESSAGE_NAV_TIMEUTC:
					ubx_state->decode_state = UBX_DECODE_GOT_MESSAGEID;
					ubx_state->message_id = NAV_TIMEUTC;
					break;
				case UBX_MESSAGE_NAV_DOP:
					ubx_state->decode_state = UBX_DECODE_GOT_MESSAGEID;
					ubx_state->message_id = NAV_DOP;
					break;
				case UBX_MESSAGE_NAV_SVINFO:
					ubx_state->decode_state = UBX_DECODE_GOT_MESSAGEID;
					ubx_state->message_id = NAV_SVINFO;
					break;
				case UBX_MESSAGE_NAV_VELNED:
					ubx_state->decode_state = UBX_DECODE_GOT_MESSAGEID;
					ubx_state->message_id = NAV_VELNED;
					break;
				default: //unknown class: reset state machine, should not happen
					ubx_decode_init();
					break;
				}
				break;
			case RXM:
				switch(b)
				{
				case UBX_MESSAGE_RXM_SVSI:
					ubx_state->decode_state = UBX_DECODE_GOT_MESSAGEID;
					ubx_state->message_id = RXM_SVSI;
					break;
				default: //unknown class: reset state machine, should not happen
					ubx_decode_init();
					break;
				}
				break;
			default: //should not happen
				ubx_decode_init();
				break;
			}
		}
		else if (ubx_state->decode_state == UBX_DECODE_GOT_MESSAGEID)
		{
			// Add to checksum
			ubx_checksum(b, &(ubx_state->ck_a), &(ubx_state->ck_b));

			ubx_state->payload_size = b;
			ubx_state->decode_state = UBX_DECODE_GOT_LENGTH1;
		}
		else if (ubx_state->decode_state == UBX_DECODE_GOT_LENGTH1)
		{
			// Add to checksum
			ubx_checksum(b, &(ubx_state->ck_a), &(ubx_state->ck_b));

			ubx_state->payload_size += b << 8;
			ubx_state->decode_state = UBX_DECODE_GOT_LENGTH2;
		}
		else if (ubx_state->decode_state == UBX_DECODE_GOT_LENGTH2)
		{
			uint8_t ret = 0;

			// Add to checksum if not yet at checksum byte
			if (ubx_state->rx_count < ubx_state->payload_size) ubx_checksum(b, &(ubx_state->ck_a), &(ubx_state->ck_b));

			// Fill packet buffer
			gps_rx_buffer[ubx_state->rx_count] = b;

			//if whole payload + checksum is in buffer:
			if (ubx_state->rx_count >= ubx_state->payload_size + 1)
			{
				//convert to correct struct
				switch (ubx_state->message_id) //this enum is unique for all ids --> no need to check the class
				{
				case NAV_POSLLH:
				{
//					printf("GOT NAV_POSLLH MESSAGE\n");
					gps_bin_nav_posllh_packet_t* packet = (gps_bin_nav_posllh_packet_t*) gps_rx_buffer;

					//Check if checksum is valid and the store the gps information
					if (ubx_state->ck_a == packet->ck_a && ubx_state->ck_b == packet->ck_b)
					{
						global_data_lock(&global_data_gps.access_conf);
						global_data_gps.lat = packet->lat;
						global_data_gps.lon = packet->lon;
						global_data_gps.alt = packet->height_msl;
						global_data_gps.counter++;
						global_data_unlock(&global_data_gps.access_conf);

						pthread_mutex_lock(ubx_mutex);
						ubx_state->last_message_timestamps[NAV_POSLLH-1] = global_data_get_timestamp_milliseconds();
						pthread_mutex_unlock(ubx_mutex);
						ret = 1;
					}
					else
					{
						printf("NAV_POSLLH: checksum invalid\n");
						ret = 0;
					}
					// Reset state machine to decode next packet
					ubx_decode_init();
					return ret;

					break;
				}
				case NAV_SOL:
				{
//					printf("GOT NAV_SOL MESSAGE\n");
					gps_bin_nav_sol_packet_t* packet = (gps_bin_nav_sol_packet_t*) gps_rx_buffer;

					//Check if checksum is valid and the store the gps information
					if (ubx_state->ck_a == packet->ck_a && ubx_state->ck_b == packet->ck_b)
					{
						global_data_lock(&global_data_gps.access_conf);
						global_data_gps.fix_type = packet->gpsFix;
						global_data_gps.counter++;
						global_data_unlock(&global_data_gps.access_conf);

						pthread_mutex_lock(ubx_mutex);
						ubx_state->last_message_timestamps[NAV_SOL-1] = global_data_get_timestamp_milliseconds();
						pthread_mutex_unlock(ubx_mutex);
						ret = 1;
					}
					else
					{
						printf("NAV_SOL: checksum invalid\n");
						ret = 0;
					}
					// Reset state machine to decode next packet
					ubx_decode_init();
					return ret;

					break;
				}
				case NAV_DOP:
				{
//					printf("GOT NAV_DOP MESSAGE\n");
					gps_bin_nav_dop_packet_t* packet = (gps_bin_nav_dop_packet_t*) gps_rx_buffer;

					//Check if checksum is valid and the store the gps information
					if (ubx_state->ck_a == packet->ck_a && ubx_state->ck_b == packet->ck_b)
					{
						global_data_lock(&global_data_gps.access_conf);
						global_data_gps.eph =  packet->hDOP;
						global_data_gps.epv =  packet->vDOP;
						global_data_gps.counter++;
						global_data_unlock(&global_data_gps.access_conf);

						pthread_mutex_lock(ubx_mutex);
						ubx_state->last_message_timestamps[NAV_DOP-1] = global_data_get_timestamp_milliseconds();
						pthread_mutex_unlock(ubx_mutex);
						ret = 1;
					}
					else
					{
						printf("NAV_DOP: checksum invalid\n");
						ret = 0;
					}
					// Reset state machine to decode next packet
					ubx_decode_init();
					return ret;

					break;
				}
				case NAV_TIMEUTC:
				{
//					printf("GOT NAV_TIMEUTC MESSAGE\n");
					gps_bin_nav_timeutc_packet_t* packet = (gps_bin_nav_timeutc_packet_t*) gps_rx_buffer;
					//Check if checksum is valid and the store the gps information
					if (ubx_state->ck_a == packet->ck_a && ubx_state->ck_b == packet->ck_b)
					{
						//convert to unix timestamp
						struct tm timeinfo;
						timeinfo.tm_year = packet->year;
						timeinfo.tm_mon = packet->month;
						timeinfo.tm_mday = packet->day;
						timeinfo.tm_hour = packet->hour;
						timeinfo.tm_min = packet->min;
						timeinfo.tm_sec = packet->sec;

						time_t epoch = mktime(&timeinfo);

						global_data_lock(&global_data_gps.access_conf);
						global_data_gps.time_usec = epoch * 1e6; //TODO: test this
						global_data_gps.time_usec += packet->time_nanoseconds*1e-3;
						global_data_gps.counter++;
						global_data_unlock(&global_data_gps.access_conf);

						pthread_mutex_lock(ubx_mutex);
						ubx_state->last_message_timestamps[NAV_TIMEUTC-1] = global_data_get_timestamp_milliseconds();
						pthread_mutex_unlock(ubx_mutex);
						ret = 1;
					}
					else
					{
						printf("NAV_TIMEUTC: checksum invalid\n");
						ret = 0;
					}
					// Reset state machine to decode next packet
					ubx_decode_init();
					return ret;

					break;
				}
				case NAV_SVINFO:
				{
//					printf("GOT NAV_SVINFO MESSAGE\n");

					//this is a more complicated message: the length depends on the number of satellites. This number is extracted from the first part of the message
					const int length_part1 = 8;
					char gps_rx_buffer_part1[length_part1];
					memcpy(gps_rx_buffer_part1, gps_rx_buffer, length_part1);
					gps_bin_nav_svinfo_part1_packet_t* packet_part1 = (gps_bin_nav_svinfo_part1_packet_t*) gps_rx_buffer_part1;

					//read checksum
					const int length_part3 = 2;
					char gps_rx_buffer_part3[length_part3];
					memcpy(gps_rx_buffer_part3, &(gps_rx_buffer[ubx_state->rx_count - 1]), length_part3);
					gps_bin_nav_svinfo_part3_packet_t* packet_part3 = (gps_bin_nav_svinfo_part3_packet_t*) gps_rx_buffer_part3;

					//Check if checksum is valid and then store the gps information
					if (ubx_state->ck_a == packet_part3->ck_a && ubx_state->ck_b == packet_part3->ck_b)
					{
						//definitions needed to read numCh elements from the buffer:
						const int length_part2 = 12;
						gps_bin_nav_svinfo_part2_packet_t* packet_part2;
						char gps_rx_buffer_part2[length_part2]; //for temporal storage

						global_data_lock(&global_data_gps.access_conf);
						int i;
						for(i = 0; i < packet_part1->numCh; i++) //for each channel
						{

							/* Get satellite information from the buffer */

							memcpy(gps_rx_buffer_part2, &(gps_rx_buffer[length_part1+i*length_part2]), length_part2);
							packet_part2 = (gps_bin_nav_svinfo_part2_packet_t*) gps_rx_buffer_part2;


							/* Write satellite information in the global storage */

							global_data_gps.satellite_prn[i] = packet_part2->svid;

							//if satellite information is healthy store the data
							uint8_t unhealthy = packet_part2->flags & 1 << 4; //flags is a bitfield

							if(!unhealthy)
							{

								if((packet_part2->flags) & 1) //flags is a bitfield
								{
									global_data_gps.satellite_used[i] = 1;
								}
								else
								{
									global_data_gps.satellite_used[i] = 0;
								}
								global_data_gps.satellite_snr[i] = packet_part2->cno;
								global_data_gps.satellite_elevation[i] = (uint8_t)(packet_part2->elev);
								global_data_gps.satellite_azimuth[i] = (uint8_t)((float)packet_part2->azim*255.0f/360.0f);
							}
							else
							{
								global_data_gps.satellite_used[i] = 0;
								global_data_gps.satellite_snr[i] = 0;
								global_data_gps.satellite_elevation[i] = 0;
								global_data_gps.satellite_azimuth[i] = 0;

							}

						}

						for(i = packet_part1->numCh; i < 20; i++) //these channels are unused //TODO: check with mavlink if it's necessary to set these to zero
						{
							global_data_gps.satellite_prn[i] = 0;
							global_data_gps.satellite_used[i] = 0;
							global_data_gps.satellite_snr[i] = 0;
							global_data_gps.satellite_elevation[i] = 0;
							global_data_gps.satellite_azimuth[i] = 0;
						}
						global_data_gps.counter++;
						global_data_unlock(&global_data_gps.access_conf);

						pthread_mutex_lock(ubx_mutex);
						ubx_state->last_message_timestamps[NAV_SVINFO-1] = global_data_get_timestamp_milliseconds();
						pthread_mutex_unlock(ubx_mutex);
						ret = 1;
					}
					else
					{
						printf("NAV_SVINFO: checksum invalid\n");
						ret = 0;
					}

					// Reset state machine to decode next packet
					ubx_decode_init();
					return ret;

					break;
				}
				case NAV_VELNED:
				{
//					printf("GOT NAV_VELNED MESSAGE\n");
					gps_bin_nav_velned_packet_t* packet = (gps_bin_nav_velned_packet_t*) gps_rx_buffer;

					//Check if checksum is valid and the store the gps information
					if (ubx_state->ck_a == packet->ck_a && ubx_state->ck_b == packet->ck_b)
					{
						global_data_lock(&global_data_gps.access_conf);
						global_data_gps.vel = (uint16_t)packet->speed;
						global_data_gps.cog = (uint16_t)((float)(packet->heading) *1e-3) ;
						global_data_gps.counter++;
						global_data_unlock(&global_data_gps.access_conf);

						pthread_mutex_lock(ubx_mutex);
						ubx_state->last_message_timestamps[NAV_VELNED-1] = global_data_get_timestamp_milliseconds();
						pthread_mutex_unlock(ubx_mutex);
						ret = 1;
					}
					else
					{
						printf("NAV_VELNED: checksum invalid\n");
						ret = 0;
					}
					// Reset state machine to decode next packet
					ubx_decode_init();
					return ret;

					break;
				}
				case RXM_SVSI:
				{
//					printf("GOT RXM_SVSI MESSAGE\n");
					const int length_part1 = 7;
					char gps_rx_buffer_part1[length_part1];
					memcpy(gps_rx_buffer_part1, gps_rx_buffer, length_part1);
					gps_bin_rxm_svsi_packet_t* packet = (gps_bin_rxm_svsi_packet_t*) gps_rx_buffer_part1;
					//Check if checksum is valid and the store the gps information
					if (ubx_state->ck_a == gps_rx_buffer[ubx_state->rx_count -1] && ubx_state->ck_b == gps_rx_buffer[ubx_state->rx_count])
					{
						global_data_lock(&global_data_gps.access_conf);
						global_data_gps.satellites_visible = packet->numVis;
						global_data_gps.counter++;
						global_data_unlock(&global_data_gps.access_conf);

						pthread_mutex_lock(ubx_mutex);
						ubx_state->last_message_timestamps[RXM_SVSI-1] = global_data_get_timestamp_milliseconds();
						pthread_mutex_unlock(ubx_mutex);
						ret = 1;
					}
					else
					{
						printf("RXM_SVSI: checksum invalid\n");
						ret = 0;
					}
					// Reset state machine to decode next packet
					ubx_decode_init();
					return ret;

					break;
				}
				default: //something went wrong
					ubx_decode_init();

					break;
				}
			}

			(ubx_state->rx_count)++;



		}


		return 0;     // no valid packet found

}

void calculate_ubx_checksum(uint8_t * message, uint8_t length)
{
	uint8_t ck_a;
	uint8_t ck_b;

	int i;
	for(i = 2; i < length - 2; i++)
	{
		ck_a = ck_a + message[i];
		ck_b = ck_b + ck_a;
	}

	message[length-2] = ck_a;
	message[length-1] = ck_b;

	printf("[%x,%x]", ck_a, ck_b);

	printf("[%x,%x]\n", message[length-2], message[length-1]);
}

int configure_gps_ubx(int fd)
{
	fflush(fd);

	int success = 0;
	size_t result_write;

	//TODO: write this in a loop once it is tested
	//UBX_CFG_PRT_PART:
	write_config_message_ubx(UBX_CONFIG_MESSAGE_PRT, sizeof(UBX_CONFIG_MESSAGE_PRT)/sizeof(uint8_t) ,fd);

	usleep(100000);

	//NAV_POSLLH:
	write_config_message_ubx(UBX_CONFIG_MESSAGE_MSG_NAV_POSLLH, sizeof(UBX_CONFIG_MESSAGE_MSG_NAV_POSLLH)/sizeof(uint8_t) ,fd);
	usleep(100000);

	//NAV_TIMEUTC:
	write_config_message_ubx(UBX_CONFIG_MESSAGE_MSG_NAV_TIMEUTC, sizeof(UBX_CONFIG_MESSAGE_MSG_NAV_TIMEUTC)/sizeof(uint8_t) ,fd);
	usleep(100000);

	//NAV_DOP:
	write_config_message_ubx(UBX_CONFIG_MESSAGE_MSG_NAV_DOP, sizeof(UBX_CONFIG_MESSAGE_MSG_NAV_DOP)/sizeof(uint8_t) ,fd);
	usleep(100000);

	//NAV_SOL:
	write_config_message_ubx(UBX_CONFIG_MESSAGE_MSG_NAV_SOL, sizeof(UBX_CONFIG_MESSAGE_MSG_NAV_SOL)/sizeof(uint8_t) ,fd);
	usleep(100000);


	//NAV_SVINFO:
	write_config_message_ubx(UBX_CONFIG_MESSAGE_MSG_NAV_SVINFO, sizeof(UBX_CONFIG_MESSAGE_MSG_NAV_SVINFO)/sizeof(uint8_t) ,fd);
	usleep(100000);

	//NAV_VELNED:
	write_config_message_ubx(UBX_CONFIG_MESSAGE_MSG_NAV_VELNED, sizeof(UBX_CONFIG_MESSAGE_MSG_NAV_VELNED)/sizeof(uint8_t) ,fd);
	usleep(100000);


	//RXM_SVSI:
	write_config_message_ubx(UBX_CONFIG_MESSAGE_MSG_RXM_SVSI, sizeof(UBX_CONFIG_MESSAGE_MSG_RXM_SVSI)/sizeof(uint8_t) ,fd);
	usleep(100000);

	return 0;
//	int success = 0;
//	    size_t result_write;
//
//		//TODO: write this in a loop once it is tested
//		//UBX_CONFIG_MESSAGE_PRT_PART1:
//		//calculate and write checksum to the end
//		int length = sizeof(UBX_CONFIG_MESSAGE_PRT)/sizeof(uint8_t);
//		calculate_ubx_checksum(UBX_CONFIG_MESSAGE_PRT, length);
//
//		result_write =  write(fd, UBX_CONFIG_MESSAGE_PRT, length);
//		if(result_write != length) success = 1;
//		usleep(100000);
//
//
//		//NAV_POSLLH:
//		//calculate and write checksum to the end
//		length = sizeof(UBX_CONFIG_MESSAGE_MSG_NAV_POSLLH)/sizeof(uint8_t);
//		calculate_ubx_checksum(UBX_CONFIG_MESSAGE_MSG_NAV_POSLLH, length);
//
//		result_write =  write(fd, UBX_CONFIG_MESSAGE_MSG_NAV_POSLLH, length);
//		if(result_write != length) success = 1;
//		usleep(100000);
//
//		//NAV_TIMEUTC:
//		//calculate and write checksum to the end
//		length = sizeof(UBX_CONFIG_MESSAGE_MSG_NAV_TIMEUTC)/sizeof(uint8_t);
//		calculate_ubx_checksum(UBX_CONFIG_MESSAGE_MSG_NAV_TIMEUTC, length);
//
//		result_write =  write(fd, UBX_CONFIG_MESSAGE_MSG_NAV_TIMEUTC, length);
//		if(result_write != length) success = 1;
//		usleep(100000);
//
//
//		//NAV_DOP:
//		//calculate and write checksum to the end
//		length = sizeof(UBX_CONFIG_MESSAGE_MSG_NAV_DOP)/sizeof(uint8_t);
//		calculate_ubx_checksum(UBX_CONFIG_MESSAGE_MSG_NAV_DOP, length);
//
//		result_write =  write(fd, UBX_CONFIG_MESSAGE_MSG_NAV_DOP, length);
//		if(result_write != length) success = 1;
//		usleep(100000);
//
//		//NAV_SOL:
//		//calculate and write checksum to the end
//		length = sizeof(UBX_CONFIG_MESSAGE_MSG_NAV_SOL)/sizeof(uint8_t);
//		calculate_ubx_checksum(UBX_CONFIG_MESSAGE_MSG_NAV_SOL, length);
//
//		result_write =  write(fd, UBX_CONFIG_MESSAGE_MSG_NAV_SOL, length);
//		if(result_write != length) success = 1;
//		usleep(100000);
//
//
//		//NAV_SVINFO:
//		//calculate and write checksum to the end
//		length = sizeof(UBX_CONFIG_MESSAGE_MSG_NAV_SVINFO)/sizeof(uint8_t);
//		calculate_ubx_checksum(UBX_CONFIG_MESSAGE_MSG_NAV_SVINFO, length);
//
//		result_write =  write(fd, UBX_CONFIG_MESSAGE_MSG_NAV_SVINFO, length);
//		if(result_write != length) success = 1;
//		usleep(100000);
//
//		//NAV_VELNED:
//		//calculate and write checksum to the end
//		length = sizeof(UBX_CONFIG_MESSAGE_MSG_NAV_VELNED)/sizeof(uint8_t);
//		calculate_ubx_checksum(UBX_CONFIG_MESSAGE_MSG_NAV_VELNED, length);
//
//		result_write =  write(fd, UBX_CONFIG_MESSAGE_MSG_NAV_VELNED, length);
//		if(result_write != length) success = 1;
//		usleep(100000);
//
//		//RXM_SVSI:
//		//calculate and write checksum to the end
//		length = sizeof(UBX_CONFIG_MESSAGE_MSG_RXM_SVSI)/sizeof(uint8_t);
//		calculate_ubx_checksum(UBX_CONFIG_MESSAGE_MSG_RXM_SVSI, length);
//
//		result_write =  write(fd, UBX_CONFIG_MESSAGE_MSG_RXM_SVSI, length);
//		if(result_write != length) success = 1;
//		usleep(100000);
//
//		//SAVE: (not needed, changes configuration permanently)
//		//		//calculate and write checksum to the end of UBX_CFG_MSG_NAV_POSLLH
//		//		length = sizeof(UBX_CFG_SAVE)/sizeof(uint8_t);
//		//		calculate_ubx_checksum(UBX_CFG_SAVE, length);
//		//
//		//		result_write =  write(fd, UBX_CFG_SAVE, length);
//		//		if(result_write != length) success = false;
//
//		return 0;

}



int read_gps_ubx(int fd, char * gps_rx_buffer, int buffer_size)

{

	uint8_t c;
	int rx_count = 0;
	int gpsRxOverflow = 0;
	int start_flag = 0;
	int found_cr = 0;

	// UBX GPS mode

	// This blocks the task until there is something on the buffer
	while (read(fd, &c, 1) > 0)
	{
//		printf("Read %x\n",c);
		if (rx_count >= buffer_size)
		{
			// The buffer is already full and we haven't found a valid ubx sentence.
			// Flush the buffer and note the overflow event.
			gpsRxOverflow++;
			start_flag = 0;
			found_cr = 0;
			rx_count = 0;
			ubx_decode_init();
			printf("Buffer full\n");
		}
		else
		{
			   //gps_rx_buffer[rx_count] = c;
			   rx_count++;

		}

		int msg_read = ubx_parse(c, gps_rx_buffer, ubx_mutex);

		if(msg_read > 0)
		{
//			printf("Found sequence\n");
			break;
		}

	}

	return 0;
}

int write_config_message_ubx(uint8_t * message, size_t length, int fd)
{
	//calculate and write checksum to the end
	uint8_t ck_a = 0;
	uint8_t ck_b = 0;

	int i;
	for(i = 2; i < length; i++)
	{
		ck_a = ck_a + message[i];
		ck_b = ck_b + ck_a;
	}

//	printf("[%x,%x]\n", ck_a, ck_b);

	int result_write =  write(fd, message, length);
	result_write +=  write(fd, &ck_a, 1);
	result_write +=  write(fd, &ck_b, 1);

	return (result_write != length+2); //return 0 as success

}

void *ubx_watchdog_loop(void * arg)
{
	/* Retrieve file descriptor */
	int fd = *((int *)arg);

	while(1)
	{
		/* if some values are to old reconfigure gps */
		int i;
		pthread_mutex_lock(&ubx_mutex);
		bool all_okay = true;
		uint64_t timestamp_now = global_data_get_timestamp_milliseconds();
		for(i = 0; i < UBX_NO_OF_MESSAGES; i++)
		{
			if(timestamp_now - ubx_state->last_message_timestamps[i] > GPS_WATCHDOG_CRITICAL_TIME_MILLISECONDS)
			{
//				printf("Warning: GPS ubx message %d not received for a long time\n", i);
				all_okay = false;
			}
		}
		pthread_mutex_unlock(&ubx_mutex);

		if(!all_okay)
		{
			/* gps error */
			printf("GPS Watchdog detected gps not running or having problems\n");

			global_data_lock(&global_data_sys_status.access_conf);
			global_data_sys_status.onboard_control_sensors_present |= 1 << 5;//TODO: write wrapper for bitmask
			global_data_sys_status.onboard_control_sensors_enabled |= 1 << 5;
			global_data_sys_status.onboard_control_sensors_health &= ~(1 << 5);
			global_data_sys_status.counter++;
			global_data_sys_status.timestamp = global_data_get_timestamp_milliseconds();
			global_data_unlock(&global_data_sys_status.access_conf);

			/* trying to reconfigure the gps configuration */
			configure_gps_ubx(fd);
			fflush(stdout);
			sleep(1);

		}
		else
		{
			/* gps healthy */

			global_data_lock(&global_data_sys_status.access_conf);
			global_data_sys_status.onboard_control_sensors_present |= 1 << 5;//TODO: write wrapper for bitmask
			global_data_sys_status.onboard_control_sensors_enabled |= 1 << 5;
			global_data_sys_status.onboard_control_sensors_health |= 1 << 5;
			global_data_sys_status.counter++;
			global_data_sys_status.timestamp = global_data_get_timestamp_milliseconds();
			global_data_unlock(&global_data_sys_status.access_conf);
		}

		usleep(GPS_WATCHDOG_WAIT_TIME_MICROSECONDS);
	}
}

void *ubx_loop(void * arg)
{
		/* Retrieve file descriptor */
		int fd = *((int *)arg);

		/* Initialize gps stuff */
	    int buffer_size = 1000;
//	    nmeaINFO * info = malloc(sizeof(nmeaINFO));
		char * gps_rx_buffer = malloc(buffer_size*sizeof(char));

		/* gps parser (nmea) */
//		nmeaPARSER parser;
//		nmea_parser_init(&parser);
//		nmea_zero_INFO(info);
//		float lat_dec = 0;
//		float lon_dec = 0;

		/* custom (mediatek custom) */
//		gps_bin_custom_state_t * mtk_state = malloc(sizeof(gps_bin_custom_state_t));
//		mtk_decode_init(mtk_state);
//		mtk_state->print_errors = false;



//		if( !strcmp("custom",mode) )
//		{
//			printf("\t%s: custom mode\n",APPNAME);
//	//		configure_gps_custom(fd); // ?
//
//
//	//		while(1)
//	//
//	//		if (configure_gps_ubx(fd, ubx_state) != 0)
//	//
//	//		{
//	//			//TODO: execute custom read
//	//		}
//
//		}

	    	printf("\tgps: ubx mode\n");
	    	//set parameters for ubx


//	    	//ubx state
//			gps_bin_ubx_state_t * ubx_state = malloc(sizeof(gps_bin_ubx_state_t));
//		   	printf("gps: ubx_state created\n");
//			ubx_decode_init();
//			ubx_state->print_errors = false;

	    	int config_not_finished = 1; //is set to 0 as soon as all configurations are completed
	    	bool configured = false;

	    	/* set parameters for ubx */

			if (configure_gps_ubx(fd) != 0)
			{
				printf("Configuration of gps module to ubx failed\n");

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
				printf("Configuration of gps module to ubx successful\n");


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
	    		read_gps_ubx(fd, gps_rx_buffer, buffer_size, &ubx_mutex);

	    		/* Inform the other processes that there is new gps data available */
	    		global_data_broadcast(&global_data_gps.access_conf);
	    	}


	//	while(1)
	//	{
	//		if( !strcmp("nmea",mode) ) //TODO: implement use of global_data-gps also in nmea mode (currently only in ubx mode)
	//		{
	//			printf("\t%s: nmea mode\n");
	//			//get gps data into info
	//			read_gps_nmea(fd, gps_rx_buffer, buffer_size, info, &parser);
	//			//convert latitude longitude
	//			lat_dec = ndeg2degree(info->lat);
	//			lon_dec = ndeg2degree(info->lon);
	//
	//			//Test output
	////			printf("Lat:%d, Lon:%d,Elev:%d, Sig:%d, Fix:%d, Inview:%d\n", (int)(lat_dec*1e6), (int)(lon_dec*1e6), (int)(info->elv*1e6), info->sig, info->fix, info->satinfo.inview);
	//		}
	////		else if ( !strcmp("ubx",mode) )
	////		{
	////
	////			//get gps data into info
	////			read_gps_ubx(fd, gps_rx_buffer, buffer_size, ubx_state); //TODO: atm using the info struct from the nmea library, once the gps/mavlink structures are clear--> use own struct
	//////			lat_dec = info->lat;
	//////			lon_dec = info->lon;
	//////			printf("Lat:%d, Lon:%d,Elev:%d, Sig:%d, Fix:%d, Inuse:%d, PDOP:%d\n", (int)(lat_dec*1e6), (int)(lon_dec*1e6), (int)(info->elv*1e6), info->sig, info->fix, info->satinfo.inuse, (int)(info->PDOP*1e4));
	////		}
	//		else if	( !strcmp("custom",mode) ) //TODO: implement use of global_data-gps also in custom mode (currently only in ubx mode)
	//		{
	//			//info is used as storage of the gps information. lat lon are already in fractional degree format * 10e6
	//			//see custom.h/mtk_parse for more information on which variables are stored in info
	//			nmea_zero_INFO(info);
	//
	//			//get gps data into info
	//			read_gps_custom(fd, gps_rx_buffer, buffer_size, info, mtk_state);
	//
	//			//Test output
	////			printf("Lat:%d, Lon:%d,Elev:%d, Sig:%d, Fix:%d, Inview:%d\n", (int)(info->lat), (int)info->lon, (int)info->elv, info->sig, info->fix, info->satinfo.inview);
	//
	//		}
	//
	//
	//
	//
	//
	//	}

		free(gps_rx_buffer);
//		free(info);


		//close port
		close_port(fd);

		//destroy gps parser
//		nmea_parser_destroy(&parser);

	    return 0;

}

void ubx_init(void)
{

	/* Init mutex for datasharing between ubx gps reading thread (ubx_thread) and  ubx_watchdog thread*/
	pthread_mutex_init(&ubx_mutex, NULL);

	/* Initialize ubx state */
	ubx_state = malloc(sizeof(gps_bin_ubx_state_t));
	ubx_decode_init();
}

