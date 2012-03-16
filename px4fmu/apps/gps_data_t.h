/*
 * gps_data_t.h
 *
 *  Created on: Mar 16, 2012
 *      Author: thomasgubler
 */

#ifndef GPS_DATA_T_H_
#define GPS_DATA_T_H_

#include "v1.0/common/mavlink.h"

typedef struct //TODO: make me global
{
	 uint64_t time_usec; ///< Timestamp (microseconds since UNIX epoch or microseconds since system boot)
	 int32_t lat; ///< Latitude in 1E7 degrees
	 int32_t lon; ///< Longitude in 1E7 degrees
	 int32_t alt; ///< Altitude in 1E3 meters (millimeters) above MSL
	 uint16_t eph; ///< GPS HDOP horizontal dilution of position in cm (m*100). If unknown, set to: 65535
	 uint16_t epv; ///< GPS VDOP horizontal dilution of position in cm (m*100). If unknown, set to: 65535
	 uint16_t vel; ///< GPS ground speed (m/s * 100). If unknown, set to: 65535
	 uint16_t cog; ///< Course over ground (NOT heading, but direction of movement) in degrees * 100, 0.0..359.99 degrees. If unknown, set to: 65535
	 uint8_t fix_type; ///< 0-1: no fix, 2: 2D fix, 3: 3D fix. Some applications will not use the value of this field unless it is at least two, so always correctly fill in the fix.
	 uint8_t satellites_visible; ///< Number of satellites visible. If unknown, set to 255

	 uint8_t satellite_prn[20]; ///< Global satellite ID
	 uint8_t satellite_used[20]; ///< 0: Satellite not used, 1: used for localization
	 uint8_t satellite_elevation[20]; ///< Elevation (0: right on top of receiver, 90: on the horizon) of satellite
	 uint8_t satellite_azimuth[20]; ///< Direction of satellite, 0: 0 deg, 255: 360 deg.
	 uint8_t satellite_snr[20]; ///< Signal to noise ratio of satellite


} __attribute__((__packed__)) gps_data_t;

extern gps_data_t gps_data;

#endif /* GPS_DATA_T_H_ */
