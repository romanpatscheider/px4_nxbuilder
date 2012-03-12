/*
 * mtk.h
 *
 *  Created on: Mar 6, 2012
 *      Author: thomasgubler
 */

#ifndef MTK_H_
#define MTK_H_

#include <stdint.h>


// ************
// the structure of the binary packet

typedef struct
{
	uint8_t payload; ///< Number of payload bytes
	int32_t latitude;  ///< Latitude in degrees * 10^7
	int32_t longitude; ///< Longitude in degrees * 10^7
	int32_t msl_altitude;  ///< MSL altitude in meters * 10^2
	uint32_t ground_speed; ///< FIXME SPEC UNCLEAR
	int32_t heading;
	uint8_t satellites;
	uint8_t fix_type;
	uint32_t date;
	uint32_t utc_time;
	uint16_t hdop;
	uint8_t ck_a;
	uint8_t ck_b;
}  __attribute__((__packed__)) type_gps_bin_custom_packet;

typedef type_gps_bin_custom_packet gps_bin_custom_packet_t;

enum MTK_DECODE_STATES
{
	MTK_DECODE_UNINIT = 0,
	MTK_DECODE_GOT_CK_A = 1,
	MTK_DECODE_GOT_CK_B = 2
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
    uint8_t decode_state;
//    bool new_data;
//    uint8_t fix;
    bool print_errors;
    int16_t rx_count;
}  __attribute__((__packed__)) type_gps_bin_custom_state;

typedef type_gps_bin_custom_state gps_bin_custom_state_t;

void mtk_decode_init(gps_bin_custom_state_t* mtk_state)
{
	mtk_state->ck_a = 0;
	mtk_state->ck_b = 0;
	mtk_state->rx_count = 0;
	mtk_state->decode_state = MTK_DECODE_UNINIT;
}

void mtk_checksum(uint8_t b, uint8_t* ck_a, uint8_t* ck_b)
{
	*(ck_a) = *(ck_a) + b;
	*(ck_b) = *(ck_b) + *(ck_a);
//	printf("Checksum now: %d\n",*(ck_b));
}



int mtk_parse(uint8_t b,  char * gps_rx_buffer, gps_bin_custom_state_t * mtk_state, nmeaINFO * info) //adapted from GTOP_BIN_CUSTOM_update_position
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
				gps_bin_custom_packet_t* packet = (gps_bin_custom_packet_t*) gps_rx_buffer;
				//Check if checksum is valid
				if (mtk_state->ck_a == packet->ck_a && mtk_state->ck_b == packet->ck_b)
				{
					info->lat				= packet->latitude;   // degrees * 10e6  //TODO types, order
					info->lon      = packet->longitude;	// degrees * 10e6
					info->elv        = packet->msl_altitude;                                       // meters
	//				GpsData.GeoidSeparation = 5000;                                 // meters
	//				GpsData.Heading         = packet->heading / 1000;                                 // degrees
	//				GpsData.Groundspeed     = packet->ground_speed / 3600;                                  // m/s
					info->satinfo.inview    = packet->satellites;                                              //TODO: not sure if this are in use or in view satellites
	//				GpsData.PDOP            = packet->hdop / 100.0f;                                                                            // not available in binary mode
	//				GpsData.HDOP            = packet->hdop / 100.0f;                                                //
	//				GpsData.VDOP            = 99.99;                                                                            // not available in binary mode
					//				mtk_state.new_data = true;
					//GPSPositionSet(&GpsData);

					info->fix				= packet->fix_type;


					uint32_t utc_time = packet->utc_time / 1000; //TODO: this code was commented out in the original file!
					info->utc.sec = utc_time % 100;          // seconds
					info->utc.min = (utc_time / 100) % 100;  // minutes
					info->utc.hour = utc_time / 10000;          // hours
					info->utc.day = packet->date;        // day //TODO: look at conent of date and convert also to month and year
					info->utc.mon = 0;    // month
					info->utc.year = 0;      // year

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

int read_gps_custom(int fd, char * gps_rx_buffer, int buffer_size, nmeaINFO * info, gps_bin_custom_state_t * mtk_state)
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

#endif /* MTK_H_ */
