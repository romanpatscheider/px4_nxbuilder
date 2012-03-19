/*
 * ubx.h
 *
 *  Created on: Mar 12, 2012
 *      Author: thomasgubler
 */

#ifndef UBX_H_
#define UBX_H_

#include <stdint.h>
#include <time.h>
#include "../gps_data_t.h" //for storage of gps information
#include <math.h>
#include <stdbool.h>

//Definitions for ubx, last two bytes are checksum which is calculated below

// defines do not work like this because the last two bytes (checksum) need to be changed, therefore they are defined in the ubx.c file for now (TODO)
//#define UBX_CFG_PRT_PART1 (uint8_t[28]){0xB5,0x62,0x06,0x00,0x14,0x00,0x01,0x00,0x00,0x00,0xD0,0x08,0x00,0x00,0x80,0x25,0x00,0x00,0x07,0x00,0x01,0x00,0x00,0x00,0x00,0x00,0xA0,0xA9}//itmaybepossibletodothiswithonemessage,thiswasjustcopiedfromucenter
//#define UBX_CFG_PRT_PART2 (uint8_t[28]){0xB5,0x62,0x06,0x00,0x14,0x00,0x01,0x00,0x00,0x00,0xC0,0x08,0x00,0x00,0x80,0x25,0x00,0x00,0x07,0x00,0x01,0x00,0x00,0x00,0x00,0x00,0xA0,0xA9}
//#define UBX_CFG_MSG_NAV_POSLLH (uint8_t[16]){0xB5,0x62,0x06,0x01,0x08,0x00,0x01,0x02,0x00,0x01,0x00,0x00,0x00,0x00,0x00,0x00}
//#define UBX_CFG_MSG_NAV_TIMEUTC (uint8_t[16]){0xB5,0x62,0x06,0x01,0x08,0x00,0x01,0x21,0x00,0x01,0x00,0x00,0x00,0x00,0x00,0x00}
//#define UBX_CFG_MSG_NAV_DOP (uint8_t[16]){0xB5,0x62,0x06,0x01,0x08,0x00,0x01,0x04,0x00,0x01,0x00,0x00,0x00,0x00,0x00,0x00}
//#define UBX_CFG_MSG_NAV_SVINFO (uint8_t[16]){0xB5,0x62,0x06,0x01,0x08,0x00,0x01,0x30,0x00,0x01,0x00,0x00,0x00,0x00,0x00,0x00}
//#define UBX_CFG_MSG_NAV_SOL (uint8_t[16]){0xB5,0x62,0x06,0x01,0x08,0x00,0x01,0x06,0x00,0x01,0x00,0x00,0x00,0x00,0x00,0x00}
//#define UBX_CFG_MSG_NAV_VELNED (uint8_t[16]){0xB5,0x62,0x06,0x01,0x08,0x00,0x01,0x12,0x00,0x01,0x00,0x00,0x00,0x00,0x00,0x00}
//#define UBX_CFG_SAVE (uint8_t[21]){0xB5,0x62,0x06,0x09,0x0d,0x00,0x00,0x00,0x00,0x00,0xFF,0xFF,0x00,0x00,0x00,0x00,0x00,0x00,0x07,0x00,0x00}


//UBX Protocoll definitions (this is the subset of the messages that are parsed)
#define UBX_CLASS_NAV 0x01
#define UBX_CLASS_RXM 0x02
#define UBX_MESSAGE_NAV_POSLLH 0x02
#define UBX_MESSAGE_NAV_SOL 0x06
#define UBX_MESSAGE_NAV_TIMEUTC 0x21
#define UBX_MESSAGE_NAV_DOP 0x04
#define UBX_MESSAGE_NAV_SVINFO 0x30
#define UBX_MESSAGE_NAV_VELNED 0x12
#define UBX_MESSAGE_RXM_SVSI 0x20

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
	uint8_t numCh; //Number of channels
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

typedef struct
{
	int32_t time_milliseconds; // Measurement integer millisecond GPS time of week
	int16_t week; //Measurement GPS week number
	uint8_t numVis; //Number of visible satellites

	//... rest of package is not used in this implementation

}  __attribute__((__packed__)) type_gps_bin_rxm_svsi_packet;

typedef type_gps_bin_rxm_svsi_packet gps_bin_rxm_svsi_packet_t;


// END the structures of the binary packets
// ************

enum UBX_MESSAGE_CLASSES
{
	CLASS_UNKNOWN = 0,
	NAV = 1,
	RXM = 2
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
	NAV_VELNED = 6,
	RXM_SVSI = 7

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

void ubx_decode_init(gps_bin_ubx_state_t* ubx_state);

void ubx_checksum(uint8_t b, uint8_t* ck_a, uint8_t* ck_b);

int ubx_parse(uint8_t b,  char * gps_rx_buffer, gps_bin_ubx_state_t * ubx_state); //adapted from GTOP_BIN_CUSTOM_update_position

void calculate_ubx_checksum(uint8_t * message, uint8_t length);

int configure_gps_ubx(int fd);

int read_gps_ubx(int fd, char * gps_rx_buffer, int buffer_size, gps_bin_ubx_state_t * ubx_state);


#endif /* UBX_H_ */
