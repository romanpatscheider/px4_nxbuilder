/*
 * ubx.h
 *
 *  Created on: Mar 12, 2012
 *      Author: thomasgubler
 */

#ifndef UBX_H_
#define UBX_H_

#include <stdint.h>

//Definitions for ubx, last two bytes are checksum which is calculated below
uint8_t UBX_CFG_PRT_PART1[] = {0xB5 , 0x62 , 0x06 , 0x00 , 0x14 , 0x00 , 0x01 , 0x00 , 0x00 , 0x00 , 0xD0 , 0x08 , 0x00 , 0x00 , 0x80 , 0x25 , 0x00 , 0x00 , 0x07 , 0x00 , 0x01 , 0x00 , 0x00 , 0x00 , 0x00 , 0x00 , 0xA0 , 0xA9}; //it may be possible to do this with one message, this was just copied from ucenter
uint8_t UBX_CFG_PRT_PART2[] = {0xB5 , 0x62 , 0x06 , 0x00 , 0x14 , 0x00 , 0x01 , 0x00 , 0x00 , 0x00 , 0xC0 , 0x08 , 0x00 , 0x00 , 0x80 , 0x25 , 0x00 , 0x00 , 0x07 , 0x00 , 0x01 , 0x00 , 0x00 , 0x00 , 0x00 , 0x00 , 0xA0 , 0xA9};
uint8_t UBX_CFG_MSG_NAV_POSLLH[] = {0xB5, 0x62, 0x06, 0x01, 0x08, 0x00,   0x01, 0x02,   0x00, 0x01, 0x00, 0x00, 0x00, 0x00,   0x00, 0x00};
uint8_t UBX_CFG_MSG_NAV_TIMEUTC[] = {0xB5, 0x62, 0x06, 0x01, 0x08, 0x00,   0x01, 0x21,   0x00, 0x01, 0x00, 0x00, 0x00, 0x00,   0x00, 0x00};
uint8_t UBX_CFG_MSG_NAV_DOP[] = {0xB5, 0x62, 0x06, 0x01, 0x08, 0x00,   0x01, 0x04,   0x00, 0x01, 0x00, 0x00, 0x00, 0x00,   0x00, 0x00};
uint8_t UBX_CFG_MSG_NAV_SVINFO[] = {0xB5, 0x62, 0x06, 0x01, 0x08, 0x00,   0x01, 0x30,   0x00, 0x01, 0x00, 0x00, 0x00, 0x00,   0x00, 0x00};
uint8_t UBX_CFG_MSG_NAV_SOL[] = {0xB5, 0x62, 0x06, 0x01, 0x08, 0x00,   0x01, 0x06,   0x00, 0x01, 0x00, 0x00, 0x00, 0x00,   0x00, 0x00};
uint8_t UBX_CFG_MSG_NAV_VELNED[] = {0xB5, 0x62, 0x06, 0x01, 0x08, 0x00,   0x01, 0x12,   0x00, 0x01, 0x00, 0x00, 0x00, 0x00,   0x00, 0x00};
uint8_t UBX_CFG_SAVE[] = {0xB5, 0x62, 0x06, 0x09, 0x0d, 0x00,   0x00, 0x00, 0x00,0x00,   0xFF,0xFF,0x00,0x00,   0x00,0x00,0x00,0x00, 0x07,  0x00,0x00};

//UBX Protocoll definitions (this is the subset of the messages that are parsed)
#define UBX_CLASS_NAV 0x01
#define UBX_MESSAGE_NAV_POSLLH 0x02
#define UBX_MESSAGE_NAV_SOL 0x06
#define UBX_MESSAGE_NAV_TIMEUTC 0x21
#define UBX_MESSAGE_NAV_DOP 0x04
#define UBX_MESSAGE_NAV_SVINFO 0x30
#define UBX_MESSAGE_NAV_VELNED 0x12

// ************
// the structures of the binary packets

typedef struct
{
	uint32_t time_milliseconds; // GPS Millisecond Time of Week
	int32_t lon;  // Longitude * 1e-7, deg
	int32_t lat; // Latitude * 1e-7, deg
	int32_t height;  // Height above Ellipsoid, mm
	int32_t height_msl;  // Height above mean sea level, mm
	uint32_t hAcc;  // Horizontal Accuracy Estimate, mm
	uint32_t vAcc;  // Vertical Accuracy Estimate, mm

	uint8_t ck_a;
	uint8_t ck_b;
}  __attribute__((__packed__)) type_gps_bin_nav_posllh_packet;

typedef type_gps_bin_nav_posllh_packet gps_bin_nav_posllh_packet_t;

typedef struct
{
	uint32_t time_milliseconds; // GPS Millisecond Time of Week
	int32_t time_nanoseconds; // Fractional Nanoseconds remainder of rounded ms above, range -500000 .. 500000
	int16_t week; // GPS week (GPS time)
	uint8_t gpsFix; //GPS Fix: 0 = No fix, 1 = Dead Reckoning only, 2 = 2D fix, 3 = 3d-fix, 4 = GPS + dead reckoning, 5 = time only fix
	uint8_t flags;
	int32_t ecefX;
	int32_t ecefY;
	int32_t ecefZ;
	uint32_t pAcc;
	int32_t ecefVX;
	int32_t ecefVY;
	int32_t ecefVZ;
	uint32_t sAcc;
	uint16_t pDOP;
	uint8_t reserved1;
	uint8_t numSV;
	uint32_t reserved2;

	uint8_t ck_a;
	uint8_t ck_b;
}  __attribute__((__packed__)) type_gps_bin_nav_sol_packet;

typedef type_gps_bin_nav_sol_packet gps_bin_nav_sol_packet_t;

typedef struct
{
	uint32_t time_milliseconds; // GPS Millisecond Time of Week
	uint32_t time_accuracy;  //Time Accuracy Estimate, ns
	int32_t time_nanoseconds; //Nanoseconds of second, range -1e9 .. 1e9 (UTC)
	uint16_t year; //Year, range 1999..2099 (UTC)
	uint8_t month; //Month, range 1..12 (UTC)
	uint8_t day; //Day of Month, range 1..31 (UTC)
	uint8_t hour; //Hour of Day, range 0..23 (UTC)
	uint8_t min; //Minute of Hour, range 0..59 (UTC)
	uint8_t sec; //Seconds of Minute, range 0..59 (UTC)
	uint8_t valid_flag; //Validity Flags (see ubx documentation)


	uint8_t ck_a;
	uint8_t ck_b;
}  __attribute__((__packed__)) type_gps_bin_nav_timeutc_packet;

typedef type_gps_bin_nav_timeutc_packet gps_bin_nav_timeutc_packet_t;

typedef struct
{
	uint32_t time_milliseconds; // GPS Millisecond Time of Week
	uint16_t gDOP; //Geometric DOP (scaling 0.01)
	uint16_t pDOP; //Position DOP (scaling 0.01)
	uint16_t tDOP; //Time DOP (scaling 0.01)
	uint16_t vDOP; //Vertical DOP (scaling 0.01)
	uint16_t hDOP; //Horizontal DOP (scaling 0.01)
	uint16_t nDOP; //Northing DOP (scaling 0.01)
	uint16_t eDOP; //Easting DOP (scaling 0.01)


	uint8_t ck_a;
	uint8_t ck_b;
}  __attribute__((__packed__)) type_gps_bin_nav_dop_packet;

typedef type_gps_bin_nav_dop_packet gps_bin_nav_dop_packet_t;

typedef struct
{
	uint32_t time_milliseconds; // GPS Millisecond Time of Week
	uint8_t numCH; //Number of channels
	uint8_t globalFlags;
	uint16_t reserved2;

}  __attribute__((__packed__)) type_gps_bin_nav_svinfo_part1_packet;

typedef type_gps_bin_nav_svinfo_part1_packet gps_bin_nav_svinfo_part1_packet_t;

typedef struct
{
	uint8_t chn; //Channel number, 255 for SVs not assigned to a channel
	uint8_t svid; //Satellite ID
	uint8_t flags;
	uint8_t quality;
	uint8_t cno; //Carrier to Noise Ratio (Signal Strength), dbHz
	int8_t elev; //Elevation in integer degrees
	int16_t azim; //Azimuth in integer degrees
	int32_t prRes; //Pseudo range residual in centimetres

}  __attribute__((__packed__)) type_gps_bin_nav_svinfo_part2_packet;

typedef type_gps_bin_nav_svinfo_part2_packet gps_bin_nav_svinfo_part2_packet_t;

typedef struct
{
	uint8_t ck_a;
	uint8_t ck_b;

}  __attribute__((__packed__)) type_gps_bin_nav_svinfo_part3_packet;

typedef type_gps_bin_nav_svinfo_part3_packet gps_bin_nav_svinfo_part3_packet_t;


typedef struct
{
	uint32_t time_milliseconds; // GPS Millisecond Time of Week
	int32_t velN; //NED north velocity, cm/s
	int32_t velE; //NED east velocity, cm/s
	int32_t velD; //NED down velocity, cm/s
	uint32_t speed; //Speed (3-D), cm/s
	uint32_t gSpeed; //Ground Speed (2-D), cm/s
	int32_t heading; //Heading of motion 2-D, deg, scaling: 1e-5
	uint32_t sAcc; //Speed Accuracy Estimate, cm/s
	uint32_t cAcc; //Course / Heading Accuracy Estimate, scaling: 1e-5

	uint8_t ck_a;
	uint8_t ck_b;
}  __attribute__((__packed__)) type_gps_bin_nav_velned_packet;

typedef type_gps_bin_nav_velned_packet gps_bin_nav_velned_packet_t;


// END the structures of the binary packets
// ************

enum UBX_MESSAGE_CLASSES
{
	CLASS_UNKNOWN = 0,
	NAV = 1
};

enum UBX_MESSAGE_IDS
{
	//these numbers do NOT correspond to the message id numbers of the ubx protocol
	ID_UNKNOWN = 0,
	NAV_POSLLH = 1,
	NAV_SOL = 2,
	NAV_TIMEUTC = 3,
	NAV_DOP = 4,
	NAV_SVINFO = 5,
	NAV_VELNED = 6

};

enum UBX_DECODE_STATES
{
	UBX_DECODE_UNINIT = 0,
	UBX_DECODE_GOT_SYNC1 = 1,
	UBX_DECODE_GOT_SYNC2 = 2,
	UBX_DECODE_GOT_CLASS = 3,
	UBX_DECODE_GOT_MESSAGEID = 4,
	UBX_DECODE_GOT_LENGTH1 = 5,
	UBX_DECODE_GOT_LENGTH2 = 6
};

typedef struct
{
	union {
		uint16_t ck;
		struct {
		    uint8_t ck_a;
		    uint8_t ck_b;
		};
	};
    enum UBX_DECODE_STATES decode_state;
//    bool new_data;
//    uint8_t fix;
    bool print_errors;
    int16_t rx_count;
    uint16_t payload_size;

    enum UBX_MESSAGE_CLASSES  message_class;
    enum UBX_MESSAGE_IDS message_id;

}  __attribute__((__packed__)) type_gps_bin_ubx_state;

typedef type_gps_bin_ubx_state gps_bin_ubx_state_t;

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



int ubx_parse(uint8_t b,  char * gps_rx_buffer, gps_bin_ubx_state_t * ubx_state, nmeaINFO * info) //adapted from GTOP_BIN_CUSTOM_update_position
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
			case NAV: //NAV
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
						info->lat = (float)packet->lat*1e-7;
						info->lon =(float)packet->lon*1e-7;
						info->elv = (float)packet->height_msl*1e-3; //(converting mm to m)

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
						info->fix = packet->gpsFix;
						info->satinfo.inuse = packet->numSV;

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
						info->PDOP = packet->pDOP*0.01;
						info->VDOP = packet->vDOP*0.01;
						info->HDOP = packet->hDOP*0.01;

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
						info->utc.year = packet->year;
						info->utc.mon = packet->month;
						info->utc.day = packet->day;
						info->utc.hour = packet->hour;
						info->utc.min = packet->min;
						info->utc.sec = packet->sec;
						info->utc.hsec = packet->time_nanoseconds*1e7;

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
					gps_bin_nav_svinfo_part2_packet_t* packet_part2[packet_part1->numCH];
					for(i =0; i < packet_part1->numCH; i++)
					{
						char gps_rx_buffer_part2[length_part2]; //TODO: save these in array once this works
						memcpy(gps_rx_buffer_part2, &(gps_rx_buffer[length_part1+i*length_part2]), length_part2);
						packet_part2[i] = (gps_bin_nav_svinfo_part2_packet_t*) gps_rx_buffer_part2;
					}

					//read checksum
					const int length_part3 = 2;
					char gps_rx_buffer_part3[length_part3];
					memcpy(gps_rx_buffer_part3, &(gps_rx_buffer[length_part1+i*length_part2]), length_part3);
					gps_bin_nav_svinfo_part3_packet_t* packet_part3 = (gps_bin_nav_svinfo_part3_packet_t*) gps_rx_buffer_part3;
					//Check if checksum is valid and the store the gps information
					if (ubx_state->ck_a == packet_part3->ck_a && ubx_state->ck_b == packet_part3->ck_b)
					{

//						printf("checksum ok\n"); //TODO: store values once (which are in packet_part2)  structure of mavlink gps message is clear...
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
						info->speed = (float)packet->speed; //TODO: save more data once mavlink/gps data structures are clear

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

	message[length -2] = ck_a;
	message[length -1] = ck_b;
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

	//SAVE: (not needed, changes configuration permanently)
	//		//calculate and write checksum to the end of UBX_CFG_MSG_NAV_POSLLH
	//		length = sizeof(UBX_CFG_SAVE)/sizeof(uint8_t);
	//		calculate_ubx_checksum(UBX_CFG_SAVE, length);
	//
	//		result_write =  write(fd, UBX_CFG_SAVE, length);
	//		if(result_write != length) success = false;

	return 0;
}

int read_gps_ubx(int fd, char * gps_rx_buffer, int buffer_size, nmeaINFO * info, gps_bin_ubx_state_t * ubx_state)
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
//		printf("Read %x\n",c);
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

		int msg_read = ubx_parse(c, gps_rx_buffer, ubx_state, info);

		if(msg_read > 0)
		{
//			printf("Found sequence\n");
			break;
		}

	}

	return 0;
}


#endif /* UBX_H_ */
