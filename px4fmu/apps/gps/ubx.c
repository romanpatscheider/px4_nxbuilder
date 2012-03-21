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
//uint8_t UBX_CFG_SAVE[] = {0xB5, 0x62, 0x06, 0x09, 0x0d, 0x00,   0x00, 0x00, 0x00,0x00,   0xFF,0xFF,0x00,0x00,   0x00,0x00,0x00,0x00, 0x07,  0x00,0x00};


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
}



int ubx_parse(uint8_t b,  char * gps_rx_buffer, gps_bin_ubx_state_t * ubx_state) //adapted from GTOP_BIN_CUSTOM_update_position
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
			case UBX_CLASS_ACK: //ACK
				ubx_state->decode_state = UBX_DECODE_GOT_CLASS;
				ubx_state->message_class = ACK;
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
			case ACK:
				switch(b)
				{
				case UBX_MESSAGE_ACK_ACK:
					ubx_state->decode_state = UBX_DECODE_GOT_MESSAGEID;
					ubx_state->message_id = ACK_ACK;
					break;
				case UBX_MESSAGE_ACK_NAK:
					ubx_state->decode_state = UBX_DECODE_GOT_MESSAGEID;
					ubx_state->message_id = ACK_NAK;
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
						global_data_lock(&global_data_gps.access_conf);
						global_data_gps.lat = packet->lat;
						global_data_gps.lon = packet->lon;
						global_data_gps.alt = packet->height_msl;
						global_data_gps.counter++;
						global_data_unlock(&global_data_gps.access_conf);
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
						global_data_lock(&global_data_gps.access_conf);
						global_data_gps.fix_type = packet->gpsFix;
						global_data_gps.counter++;
						global_data_unlock(&global_data_gps.access_conf);

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

					//Check if checksum is valid and the store the gps information
					if (ubx_state->ck_a == packet->ck_a && ubx_state->ck_b == packet->ck_b)
					{
						global_data_lock(&global_data_gps.access_conf);
						global_data_gps.eph =  packet->hDOP;
						global_data_gps.epv =  packet->vDOP;
						global_data_gps.counter++;
						global_data_unlock(&global_data_gps.access_conf);

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

						global_data_lock(&global_data_gps.access_conf);
						global_data_gps.time_usec = epoch * 1e6; //TODO: test this
						global_data_gps.time_usec += packet->time_nanoseconds*1e-3;
						global_data_gps.counter++;
						global_data_unlock(&global_data_gps.access_conf);

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

					//Check if checksum is valid and the store the gps information
					if (ubx_state->ck_a == packet->ck_a && ubx_state->ck_b == packet->ck_b)
					{
						global_data_lock(&global_data_gps.access_conf);
						global_data_gps.vel = (uint16_t)packet->speed;
						global_data_gps.cog = (uint16_t)((float)(packet->heading) *1e-3) ;
						global_data_gps.counter++;
						global_data_unlock(&global_data_gps.access_conf);

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
					if (ubx_state->ck_a == gps_rx_buffer[ubx_state->rx_count -1] && ubx_state->ck_b == gps_rx_buffer[ubx_state->rx_count])
					{
						global_data_lock(&global_data_gps.access_conf);
						global_data_gps.satellites_visible = packet->numVis;
						global_data_gps.counter++;
						global_data_unlock(&global_data_gps.access_conf);

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
				case ACK_ACK:
				{
					if (ubx_state->ck_a == gps_rx_buffer[ubx_state->rx_count -1] && ubx_state->ck_b == gps_rx_buffer[ubx_state->rx_count])
					{
						printf("Received acknowledge: %x %x\n", gps_rx_buffer[ubx_state->rx_count -3],gps_rx_buffer[ubx_state->rx_count -2]);
						ubx_state->last_ack_message[0] = gps_rx_buffer[ubx_state->rx_count -3];
						ubx_state->last_ack_message[1] = gps_rx_buffer[ubx_state->rx_count -2];
						ret = 1;
					}
					else
					{
						printf("Received acknowledge, but checksum invalid\n");
						ret = 0;
					}
					return ret;

					break;
				}
				case ACK_NAK:
				{
					if (ubx_state->ck_a == gps_rx_buffer[ubx_state->rx_count -1] && ubx_state->ck_b == gps_rx_buffer[ubx_state->rx_count])
					{
						printf("Received not-acknowledge\n");
						ret = 1;
					}
					else
					{
						printf("Received not-acknowledge and checksum invalid\n");
						ret = 0;
					}
					break;
				}
				default: //something went wrong
					ubx_decode_init(ubx_state);

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
}

int configure_gps_ubx(int fd, gps_bin_ubx_state_t * ubx_state)
{
	int success = 0;
    size_t result_write;
//    ssize_t result_read;
//
    char buffer[1000];
    int ack_result;
//    size_t bytes_read;

	//TODO: write this in a loop once it is tested
	//UBX_CFG_PRT_PART1:
	//calculate and write checksum to the end
	int length = sizeof(UBX_CFG_PRT_PART1)/sizeof(uint8_t);
	calculate_ubx_checksum(UBX_CFG_PRT_PART1, length);

	result_write =  write(fd, UBX_CFG_PRT_PART1, length);
	if(result_write == length)
	{
		printf("write success\n");
	}
//	usleep(100000);

	printf("start reading...\n");
	read_gps_ubx(fd, buffer, sizeof(buffer), ubx_state);

	if(ubx_state->last_ack_message[0] == UBX_CFG_PRT_PART1[2] && ubx_state->last_ack_message[1] == UBX_CFG_PRT_PART1[3])
	{
		printf("configuration 1 successful\n");
	}
	else
	{
		printf("configuration 1 not successful\n");
	}

	printf("\nfinished reading\n");

	//UBX_CFG_PRT_PART2:
	//calculate and write checksum to the end
	length = sizeof(UBX_CFG_PRT_PART2)/sizeof(uint8_t);
	calculate_ubx_checksum(UBX_CFG_PRT_PART2, length);

	usleep(100000);

	result_write =  write(fd, UBX_CFG_PRT_PART2, length);
	if(result_write == length)
	{
		printf("write success\n");
	}

	printf("start reading...\n");
	read_gps_ubx(fd, buffer, sizeof(buffer), ubx_state);

	if(ubx_state->last_ack_message[0] == UBX_CFG_PRT_PART2[2] && ubx_state->last_ack_message[1] == UBX_CFG_PRT_PART2[3])
	{
		printf("configuration 2 successful\n");
	}
	else
	{
		printf("configuration 2 not successful\n");
	}

	printf("\nfinished reading\n");

	usleep(100000);
//
//	//NAV_POSLLH:
//	//calculate and write checksum to the end
//	length = sizeof(UBX_CFG_MSG_NAV_POSLLH)/sizeof(uint8_t);
//	calculate_ubx_checksum(UBX_CFG_MSG_NAV_POSLLH, length);
//
//	result_write =  write(fd, UBX_CFG_MSG_NAV_POSLLH, length);
//	if(result_write != length) success = 1;
//	usleep(100000);
//
//	//NAV_TIMEUTC:
//	//calculate and write checksum to the end
//	length = sizeof(UBX_CFG_MSG_NAV_TIMEUTC)/sizeof(uint8_t);
//	calculate_ubx_checksum(UBX_CFG_MSG_NAV_TIMEUTC, length);
//
//	result_write =  write(fd, UBX_CFG_MSG_NAV_TIMEUTC, length);
//	if(result_write != length) success = 1;
//	usleep(100000);
//
//
//	//NAV_DOP:
//	//calculate and write checksum to the end
//	length = sizeof(UBX_CFG_MSG_NAV_DOP)/sizeof(uint8_t);
//	calculate_ubx_checksum(UBX_CFG_MSG_NAV_DOP, length);
//
//	result_write =  write(fd, UBX_CFG_MSG_NAV_DOP, length);
//	if(result_write != length) success = 1;
//	usleep(100000);
//
//	//NAV_SOL:
//	//calculate and write checksum to the end
//	length = sizeof(UBX_CFG_MSG_NAV_SOL)/sizeof(uint8_t);
//	calculate_ubx_checksum(UBX_CFG_MSG_NAV_SOL, length);
//
//	result_write =  write(fd, UBX_CFG_MSG_NAV_SOL, length);
//	if(result_write != length) success = 1;
//	usleep(100000);
//
//
//	//NAV_SVINFO:
//	//calculate and write checksum to the end
//	length = sizeof(UBX_CFG_MSG_NAV_SVINFO)/sizeof(uint8_t);
//	calculate_ubx_checksum(UBX_CFG_MSG_NAV_SVINFO, length);
//
//	result_write =  write(fd, UBX_CFG_MSG_NAV_SVINFO, length);
//	if(result_write != length) success = 1;
//	usleep(100000);
//
//	//NAV_VELNED:
//	//calculate and write checksum to the end
//	length = sizeof(UBX_CFG_MSG_NAV_VELNED)/sizeof(uint8_t);
//	calculate_ubx_checksum(UBX_CFG_MSG_NAV_VELNED, length);
//
//	result_write =  write(fd, UBX_CFG_MSG_NAV_VELNED, length);
//	if(result_write != length) success = 1;
//	usleep(100000);
//
//	//RXM_SVSI:
//	//calculate and write checksum to the end
//	length = sizeof(UBX_CFG_MSG_RXM_SVSI)/sizeof(uint8_t);
//	calculate_ubx_checksum(UBX_CFG_MSG_RXM_SVSI, length);
//
//	result_write =  write(fd, UBX_CFG_MSG_RXM_SVSI, length);
//	if(result_write != length) success = 1;
//	usleep(100000);

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
