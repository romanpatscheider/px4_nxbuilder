/*
 * mtk.h
 *
 *  Created on: Mar 6, 2012
 *      Author: thomasgubler
 */

#ifndef MTK_H_
#define MTK_H_

#include <stdint.h>
#include <string.h>
#include <stdbool.h>
#include "nmealib/nmea/nmea.h"

//Definition for custom mode
#define MEDIATEK_REFRESH_RATE_4HZ "$PMTK220,250*29\r\n" //refresh rate - 4Hz - 250 milliseconds
#define MEDIATEK_REFRESH_RATE_5HZ "$PMTK220,200*2C\r\n"
#define MEDIATEK_REFRESH_RATE_10HZ "$PMTK220,100*2F\r\n" //refresh rate - 10Hz - 100 milliseconds
#define MEDIATEK_FACTORY_RESET "$PMTK104*37\r\n" //clear current settings
#define MEDIATEK_CUSTOM_BINARY_MODE "$PGCMD,16,0,0,0,0,0*6A\r\n"
#define MEDIATEK_FULL_COLD_RESTART "$PMTK104*37\r\n"
//#define NMEA_GGA_ENABLE "$PMTK314,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0*27\r\n" //Set GGA messages




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

void mtk_decode_init(gps_bin_custom_state_t* mtk_state);

void mtk_checksum(uint8_t b, uint8_t* ck_a, uint8_t* ck_b);



int mtk_parse(uint8_t b,  char * gps_rx_buffer, gps_bin_custom_state_t * mtk_state, nmeaINFO * info); //adapted from GTOP_BIN_CUSTOM_update_position

int read_gps_custom(int fd, char * gps_rx_buffer, int buffer_size, nmeaINFO * info, gps_bin_custom_state_t * mtk_state);

int configure_gps_custom(int fd);

#endif /* MTK_H_ */
