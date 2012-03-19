/*
 * ubx.c
 *
 *  Created on: Mar 19, 2012
 *      Author: julianoes
 */

#include "ubx.h"


//Definitions for ubx, last two bytes are checksum which is calculated below
uint8_t UBX_CFG_PRT_PART1[] = {0xB5 , 0x62 , 0x06 , 0x00 , 0x14 , 0x00 , 0x01 , 0x00 , 0x00 , 0x00 , 0xD0 , 0x08 , 0x00 , 0x00 , 0x80 , 0x25 , 0x00 , 0x00 , 0x07 , 0x00 , 0x01 , 0x00 , 0x00 , 0x00 , 0x00 , 0x00 , 0xA0 , 0xA9}; //it may be possible to do this with one message, this was just copied from ucenter
uint8_t UBX_CFG_PRT_PART2[] = {0xB5 , 0x62 , 0x06 , 0x00 , 0x14 , 0x00 , 0x01 , 0x00 , 0x00 , 0x00 , 0xC0 , 0x08 , 0x00 , 0x00 , 0x80 , 0x25 , 0x00 , 0x00 , 0x07 , 0x00 , 0x01 , 0x00 , 0x00 , 0x00 , 0x00 , 0x00 , 0xA0 , 0xA9};
uint8_t UBX_CFG_MSG_NAV_POSLLH[] = {0xB5, 0x62, 0x06, 0x01, 0x08, 0x00,   0x01, 0x02,   0x00, 0x01, 0x00, 0x00, 0x00, 0x00,   0x00, 0x00};
uint8_t UBX_CFG_MSG_NAV_TIMEUTC[] = {0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0x01, 0x21, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
uint8_t UBX_CFG_MSG_NAV_DOP[] = {0xB5, 0x62, 0x06, 0x01, 0x08, 0x00,   0x01, 0x04,   0x00, 0x01, 0x00, 0x00, 0x00, 0x00,   0x00, 0x00};
uint8_t UBX_CFG_MSG_NAV_SVINFO[] = {0xB5, 0x62, 0x06, 0x01, 0x08, 0x00,   0x01, 0x30,   0x00, 0x01, 0x00, 0x00, 0x00, 0x00,   0x00, 0x00};
uint8_t UBX_CFG_MSG_NAV_SOL[] = {0xB5, 0x62, 0x06, 0x01, 0x08, 0x00,   0x01, 0x06,   0x00, 0x01, 0x00, 0x00, 0x00, 0x00,   0x00, 0x00};
uint8_t UBX_CFG_MSG_NAV_VELNED[] = {0xB5, 0x62, 0x06, 0x01, 0x08, 0x00,   0x01, 0x12,   0x00, 0x01, 0x00, 0x00, 0x00, 0x00,   0x00, 0x00};
uint8_t UBX_CFG_MSG_RXM_SVSI[] = {0xB5, 0x62, 0x06, 0x01, 0x08, 0x00,   0x02, 0x20,   0x00, 0x01, 0x00, 0x00, 0x00, 0x00,   0x00, 0x00};
uint8_t UBX_CFG_SAVE[] = {0xB5, 0x62, 0x06, 0x09, 0x0d, 0x00,   0x00, 0x00, 0x00,0x00,   0xFF,0xFF,0x00,0x00,   0x00,0x00,0x00,0x00, 0x07,  0x00,0x00};


void ubx_decode_init(gps_bin_ubx_state_t* ubx_state)
{
	ubx_state->ck_a = 0;
	ubx_state->ck_b = 0;
	ubx_state->rx_count = 0;
	ubx_state->decode_state = UBX_DECODE_UNINIT;
	ubx_state->message_class = CLASS_UNKNOWN;
	ubx_state->message_id = ID_UNKNOWN;
	ubx_state->payload_size = 0;
}

void ubx_checksum(uint8_t b, uint8_t* ck_a, uint8_t* ck_b)
{
	*(ck_a) = *(ck_a) + b;
	*(ck_b) = *(ck_b) + *(ck_a);
//	printf("Checksum now: %d\n",*(ck_b));
}



int ubx_parse(uint8_t b,  char * gps_rx_buffer, gps_bin_ubx_state_t * ubx_state) //adapted from GTOP_BIN_CUSTOM_update_position
{
//	printf("b=%x\n",b);
	// Debug output to telemetry port
	//	PIOS_COM_SendBufferNonBlocking(PIOS_COM_TELEM_RF, &b, 1);
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
				ubx_decode_init(ubx_state);
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
				ubx_decode_init(ubx_state);
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
					ubx_decode_init(ubx_state);
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
					ubx_decode_init(ubx_state);
					break;
				}
				break;
			default: //should not happen
				ubx_decode_init(ubx_state);
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
						gps_data.lat = packet->lat;
						gps_data.lon = packet->lon;
						gps_data.alt = packet->height_msl;

						ret = 1;
					}
					else
					{
						printf("checksum invalid\n");
						ret = 0;
					}
					// Reset state machine to decode next packet
					ubx_decode_init(ubx_state);
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
						gps_data.fix_type = packet->gpsFix;

						ret = 1;
					}
					else
					{
						printf("checksum invalid\n");
						ret = 0;
					}
					// Reset state machine to decode next packet
					ubx_decode_init(ubx_state);
					return ret;

					break;
				}
				case NAV_DOP:
				{
//					printf("GOT NAV_DOP MESSAGE\n");
					gps_bin_nav_dop_packet_t* packet = (gps_bin_nav_dop_packet_t*) gps_rx_buffer;
//					printf("DEBUG: got NAV_DOP packet\n");
					//Check if checksum is valid and the store the gps information
					if (ubx_state->ck_a == packet->ck_a && ubx_state->ck_b == packet->ck_b)
					{
						gps_data.eph =  packet->hDOP;
						gps_data.epv =  packet->vDOP;

						ret = 1;
					}
					else
					{
						printf("checksum invalid\n");
						ret = 0;
					}
					// Reset state machine to decode next packet
					ubx_decode_init(ubx_state);
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
						gps_data.time_usec = epoch * 1e6; //TODO: test this
						gps_data.time_usec += packet->time_nanoseconds*1e-3;

						ret = 1;
					}
					else
					{
						printf("checksum invalid\n");
						ret = 0;
					}
					// Reset state machine to decode next packet
					ubx_decode_init(ubx_state);
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

					//read numCH elements from the message
					int i;
					const int length_part2 = 12;
					gps_bin_nav_svinfo_part2_packet_t* packet_part2[20]; //TODO: use numCH

					char gps_rx_buffer_part2[20][length_part2];

					for(i = 0; i < packet_part1->numCh; i++)
					{

						memcpy(gps_rx_buffer_part2[i], &(gps_rx_buffer[length_part1+i*length_part2]), length_part2);
						packet_part2[i] = (gps_bin_nav_svinfo_part2_packet_t*) gps_rx_buffer_part2[i];
					}

					//read checksum
					const int length_part3 = 2;
					char gps_rx_buffer_part3[length_part3];
					memcpy(gps_rx_buffer_part3, &(gps_rx_buffer[length_part1+packet_part1->numCh*length_part2]), length_part3);
					gps_bin_nav_svinfo_part3_packet_t* packet_part3 = (gps_bin_nav_svinfo_part3_packet_t*) gps_rx_buffer_part3;

					//Check if checksum is valid and then store the gps information
					if (ubx_state->ck_a == packet_part3->ck_a && ubx_state->ck_b == packet_part3->ck_b)
					{

						/* Write satellite information */

						int i;
						for(i = 0; i < packet_part1->numCh; i++)
						{
//							printf("flags %x, ", packet_part2[i]->flags);

							gps_data.satellite_prn[i] = packet_part2[i]->svid;
							//if satellite information is healthy store the data
							uint8_t unhealthy = packet_part2[i]->flags & 1 << 4;
//							printf("unhealthy: %d,", unhealthy);
							if(!unhealthy)
							{

	//							printf("svid: [%d, %d]",gps_data.satellite_prn[i], packet_part2[i]->svid);//DEBUG
								if((packet_part2[i]->flags) & 1) //flags is a bitfield
								{
									gps_data.satellite_used[i] = 1;
								}
								else
								{
									gps_data.satellite_used[i] = 0;
								}
								gps_data.satellite_snr[i] = packet_part2[i]->cno;
//								printf("cno: [%d, %d]",gps_data.satellite_snr[i], packet_part2[i]->cno);//DEBUG
								gps_data.satellite_elevation[i] = (uint8_t)(packet_part2[i]->elev); //TODO: correct conversion: 360 to 255
								gps_data.satellite_azimuth[i] = (uint8_t)(packet_part2[i]->azim);
	//							printf("azim: [%d, %d, %d]",gps_data.satellite_azimuth[i], packet_part2[i]->azim);//DEBUG
							}
							else
							{
								gps_data.satellite_prn[i] = 0;
								gps_data.satellite_used[i] = 0;
								gps_data.satellite_elevation[i] = 0;
								gps_data.satellite_azimuth[i] = 0;
								gps_data.satellite_snr[i] = 0;
							}



						}
//						printf("\n");//DEBUG

						for(i = packet_part1->numCh; i < 20; i++)
						{
							gps_data.satellite_prn[i] = 0;
							gps_data.satellite_used[i] = 0;
							gps_data.satellite_elevation[i] = 0;
							gps_data.satellite_azimuth[i] = 0;
							gps_data.satellite_snr[i] = 0;
						}

						ret = 1;
					}
					else
					{
						printf("checksum invalid\n");
						ret = 0;
					}

					// Reset state machine to decode next packet
					ubx_decode_init(ubx_state);
					return ret;

					break;
				}
				case NAV_VELNED:
				{
//					printf("GOT NAV_VELNED MESSAGE\n");
					gps_bin_nav_velned_packet_t* packet = (gps_bin_nav_velned_packet_t*) gps_rx_buffer;
//					printf("DEBUG: got NAV_DOP packet\n");
					//Check if checksum is valid and the store the gps information
					if (ubx_state->ck_a == packet->ck_a && ubx_state->ck_b == packet->ck_b)
					{
						gps_data.vel = (uint16_t)packet->speed;
						gps_data.cog = (uint16_t)((float)(packet->heading) *1e-3) ;

						ret = 1;
					}
					else
					{
						printf("checksum invalid\n");
						ret = 0;
					}
					// Reset state machine to decode next packet
					ubx_decode_init(ubx_state);
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
					if (ubx_state->ck_a == gps_rx_buffer[ubx_state->rx_count -1] && gps_rx_buffer[ubx_state->rx_count])
					{
						gps_data.satellites_visible = packet->numVis;

						ret = 1;
					}
					else
					{
						printf("checksum invalid\n");
						ret = 0;
					}
					// Reset state machine to decode next packet
					ubx_decode_init(ubx_state);
					return ret;

					break;
				}
				default: //something went wrong
					ubx_decode_init(ubx_state);
				break;
				}


			}

			// TODO: update that new data is available

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
}

int configure_gps_ubx(int fd)
{
	int success = 0;
    size_t result_write;

	//TODO: write this in a loop once it is tested
	//UBX_CFG_PRT_PART1:
	//calculate and write checksum to the end
	int length = sizeof(UBX_CFG_PRT_PART1)/sizeof(uint8_t);
	calculate_ubx_checksum(UBX_CFG_PRT_PART1, length);

	result_write =  write(fd, UBX_CFG_PRT_PART1, length);
	if(result_write != length) success = 1;
	usleep(100000); //TODO: parse acknowledgment message

	//UBX_CFG_PRT_PART2:
	//calculate and write checksum to the end
	length = sizeof(UBX_CFG_PRT_PART2)/sizeof(uint8_t);
	calculate_ubx_checksum(UBX_CFG_PRT_PART2, length);

	result_write =  write(fd, UBX_CFG_PRT_PART2, length);
	if(result_write != length) success = 1;
	usleep(100000);

	//NAV_POSLLH:
	//calculate and write checksum to the end
	length = sizeof(UBX_CFG_MSG_NAV_POSLLH)/sizeof(uint8_t);
	calculate_ubx_checksum(UBX_CFG_MSG_NAV_POSLLH, length);

	result_write =  write(fd, UBX_CFG_MSG_NAV_POSLLH, length);
	if(result_write != length) success = 1;
	usleep(100000);

	//NAV_TIMEUTC:
	//calculate and write checksum to the end
	length = sizeof(UBX_CFG_MSG_NAV_TIMEUTC)/sizeof(uint8_t);
	calculate_ubx_checksum(UBX_CFG_MSG_NAV_TIMEUTC, length);

	result_write =  write(fd, UBX_CFG_MSG_NAV_TIMEUTC, length);
	if(result_write != length) success = 1;
	usleep(100000);


	//NAV_DOP:
	//calculate and write checksum to the end
	length = sizeof(UBX_CFG_MSG_NAV_DOP)/sizeof(uint8_t);
	calculate_ubx_checksum(UBX_CFG_MSG_NAV_DOP, length);

	result_write =  write(fd, UBX_CFG_MSG_NAV_DOP, length);
	if(result_write != length) success = 1;
	usleep(100000);

	//NAV_SOL:
	//calculate and write checksum to the end
	length = sizeof(UBX_CFG_MSG_NAV_SOL)/sizeof(uint8_t);
	calculate_ubx_checksum(UBX_CFG_MSG_NAV_SOL, length);

	result_write =  write(fd, UBX_CFG_MSG_NAV_SOL, length);
	if(result_write != length) success = 1;
	usleep(100000);


	//NAV_SVINFO:
	//calculate and write checksum to the end
	length = sizeof(UBX_CFG_MSG_NAV_SVINFO)/sizeof(uint8_t);
	calculate_ubx_checksum(UBX_CFG_MSG_NAV_SVINFO, length);

	result_write =  write(fd, UBX_CFG_MSG_NAV_SVINFO, length);
	if(result_write != length) success = 1;
	usleep(100000);

	//NAV_VELNED:
	//calculate and write checksum to the end
	length = sizeof(UBX_CFG_MSG_NAV_VELNED)/sizeof(uint8_t);
	calculate_ubx_checksum(UBX_CFG_MSG_NAV_VELNED, length);

	result_write =  write(fd, UBX_CFG_MSG_NAV_VELNED, length);
	if(result_write != length) success = 1;
	usleep(100000);

	//RXM_SVSI:
	//calculate and write checksum to the end
	length = sizeof(UBX_CFG_MSG_RXM_SVSI)/sizeof(uint8_t);
	calculate_ubx_checksum(UBX_CFG_MSG_RXM_SVSI, length);

	result_write =  write(fd, UBX_CFG_MSG_RXM_SVSI, length);
	if(result_write != length) success = 1;
	usleep(100000);

	//SAVE: (not needed, changes configuration permanently)
	//		//calculate and write checksum to the end of UBX_CFG_MSG_NAV_POSLLH
	//		length = sizeof(UBX_CFG_SAVE)/sizeof(uint8_t);
	//		calculate_ubx_checksum(UBX_CFG_SAVE, length);
	//
	//		result_write =  write(fd, UBX_CFG_SAVE, length);
	//		if(result_write != length) success = false;

	return 0;
}

int read_gps_ubx(int fd, char * gps_rx_buffer, int buffer_size, gps_bin_ubx_state_t * ubx_state)
{

	uint8_t c;
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
		printf("Read %x\n",c);
		if (rx_count >= buffer_size)
		{
			// The buffer is already full and we haven't found a valid NMEA sentence.
			// Flush the buffer and note the overflow event.
			gpsRxOverflow++;
			start_flag = 0;
			found_cr = 0;
			rx_count = 0;
			ubx_decode_init(ubx_state);
			printf("Buffer full\n");
		}
		else
		{
			   //gps_rx_buffer[rx_count] = c;
			   rx_count++;

		}

		int msg_read = ubx_parse(c, gps_rx_buffer, ubx_state);

		if(msg_read > 0)
		{
//			printf("Found sequence\n");
			break;
		}

	}

	return 0;
}
