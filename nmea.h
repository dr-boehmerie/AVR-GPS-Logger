/*
 * Author: Michael Böhme
 *
 * This file is free software; you can redistribute it and/or modify
 * it under the terms of either the GNU General Public License version 2
 * or the GNU Lesser General Public License version 2.1, both as
 * published by the Free Software Foundation.
 */

#ifndef NMEA_H
#define NMEA_H

#include <inttypes.h>

#include "config.h"

#if NMEA_ENABLE

#define NMEA_GGA_ENABLE		1
#define NMEA_RMC_ENABLE		1
#define NMEA_VTG_ENABLE		0
#define NMEA_GSA_ENABLE		0
#define NMEA_GSV_ENABLE		0

#define NMEA_DATE_AS_INT	1
#define NMEA_TIME_AS_INT	1


#define NMEA_RESULT_STRING_LENGTH	3

enum NMEA_RESULT_NUMBER_LIST
{
	NMEA_RESULT_NONE,

#if NMEA_RMC_ENABLE || NMEA_GGA_ENABLE
	NMEA_RESULT_LAT,
	NMEA_RESULT_LONG,
	NMEA_RESULT_SPEED,
//	NMEA_RESULT_ELEVATION,
	NMEA_RESULT_TRACK,
#endif
#if NMEA_GGA_ENABLE
	NMEA_RESULT_SAT_NO,
	NMEA_RESULT_ALT,
#endif

	NMEA_RESULT_NUMBER_END,

	NMEA_RESULT_STRING_START	= NMEA_RESULT_NUMBER_END,

#if NMEA_RMC_ENABLE || NMEA_GGA_ENABLE
	NMEA_RESULT_STATUS			= NMEA_RESULT_STRING_START,
	NMEA_RESULT_LAT_NS,
	NMEA_RESULT_LONG_EW,
	NMEA_RESULT_QUAL,
#endif
	NMEA_RESULT_STRING_END,

	NMEA_RESULT_DATE			= NMEA_RESULT_STRING_END,
	NMEA_RESULT_TIME,

};
#define NMEA_RESULT_NUMBER_NO		(NMEA_RESULT_NUMBER_END)
#define NMEA_RESULT_STRING_NO		(NMEA_RESULT_STRING_END - NMEA_RESULT_STRING_START)

enum NMEA_PARAM_GROUP_LIST
{
#if NMEA_GGA_ENABLE
	NMEA_GRP_GPGGA,		/*!< fix information */
#endif
#if NMEA_RMC_ENABLE
	NMEA_GRP_GPRMC,		/*!< recommended minimum data */
#endif
#if NMEA_VTG_ENABLE
	NMEA_GRP_GPVTG,		/*!< vector track and speed over ground */
#endif
#if NMEA_GSA_ENABLE
	NMEA_GRP_GPGSA,		/*!< overall satellite data */
#endif
#if NMEA_GSV_ENABLE
	NMEA_GRP_GPGSV,		/*!< detailed satellite data */
#endif

	NMEA_GRP_NO,
	NMEA_GRP_UNKNOWN = NMEA_GRP_NO
};

enum NMEA_PARAM_ID_LIST
{
#if NMEA_GGA_ENABLE
	NMEA_GPGGA_0 = 0,	/*!< GGA */
	NMEA_GPGGA_TIME,	/*!< time (utc) of fix in bcd: 120000 */
	NMEA_GPGGA_LAT,		/*!< latitude */
	NMEA_GPGGA_NS,		/*!< N / S */
	NMEA_GPGGA_LONG,	/*!< longitude */
	NMEA_GPGGA_EW,		/*!< E / W */
	NMEA_GPGGA_QUAL,	/*!< fix quality: 0 invalid, 1 gps fix, 2 dgps fix, 6 estimated*/
	NMEA_GPGGA_SAT_NO,	/*!< no of satellites */
	NMEA_GPGGA_HDIL,	/*!< horizontal dilution of position */
	NMEA_GPGGA_ALT,		/*!< altitude */
	NMEA_GPGGA_ALTM,	/*!< M */
	NMEA_GPGGA_HEIGHT,	/*!< height of geoid */
	NMEA_GPGGA_HEIGHTM,	/*!< M */
	NMEA_GPGGA_13,		/*!< % */
	NMEA_GPGGA_14,		/*!< % */
	NMEA_GPGGA_CKSUM,	/*!< checksum */
#endif

#if NMEA_RMC_ENABLE
	NMEA_GPRMC_0 = 0,	/*!< RMC */
	NMEA_GPRMC_TIME,	/*!< time (utc) of fix in bcd: 120000 */
	NMEA_GPRMC_STATUS,	/*!< status Active / Void */
	NMEA_GPRMC_LAT,		/*!< Latitude */
	NMEA_GPRMC_NS,		/*!< N / S */
	NMEA_GPRMC_LONG,	/*!< Longitude */
	NMEA_GPRMC_EW,		/*!< E / W */
	NMEA_GPRMC_SPD,		/*!< Speed over ground in knots, 1 knot = 1.85200 km/h */
	NMEA_GPRMC_TRK,		/*!< Track angle in degrees */
	NMEA_GPRMC_DATE,	/*!< Date in bcd: 180311 */
	NMEA_GPRMC_MAGN,	/*!< Magnetic variation */
	NMEA_GPRMC_W,		/*!< W */
	NMEA_GPRMC_QUAL,	/*!< A=autonomous, D=differential, E=estimated, N=not valid */
	NMEA_GPRMC_CKSUM,	/*!< checksum */
#endif

#if NMEA_VTG_ENABLE
	NMEA_GPVTG_0 = 0,	/*!< VTG */
	NMEA_GPVTG_1,		/*!< true track made good */
	NMEA_GPVTG_2,		/*!< T */
	NMEA_GPVTG_3,		/*!< magnetic track made good */
	NMEA_GPVTG_4,		/*!< M */
	NMEA_GPVTG_5,		/*!< ground speed knots */
	NMEA_GPVTG_6,		/*!< N */
	NMEA_GPVTG_7,		/*!< ground speed kmh */
	NMEA_GPVTG_8,		/*!< K */
	NMEA_GPVTG_CKSUM,	/*!< checksum */
#endif

#if NMEA_GSV_ENABLE
	NMEA_GPGSV_0 = 0,
	NMEA_GPGSV_1,		/*!< GSV */
	NMEA_GPGSV_2,		/*!<  */
	NMEA_GPGSV_3,		/*!<  */
	NMEA_GPGSV_4,		/*!<  */
	NMEA_GPGSV_5,		/*!<  */
	NMEA_GPGSV_6,		/*!<  */
	NMEA_GPGSV_7,		/*!<  */
	NMEA_GPGSV_8,		/*!<  */
	NMEA_GPGSV_9,		/*!<  */
	NMEA_GPGSV_10,		/*!<  */
	NMEA_GPGSV_11,		/*!<  */
	NMEA_GPGSV_12,		/*!<  */
	NMEA_GPGSV_13,		/*!<  */
	NMEA_GPGSV_14,		/*!<  */
	NMEA_GPGSV_15,		/*!<  */
	NMEA_GPGSV_16,		/*!<  */
	NMEA_GPGSV_17,		/*!<  */
	NMEA_GPGSV_18,		/*!<  */
	NMEA_GPGSV_19,		/*!<  */
	NMEA_GPGSV_CKSUM,	/*!< checksum */
#endif

#if NMEA_GSA_ENABLE
	NMEA_GPGSA_0 = 0,	/*!< GSA */
	NMEA_GPGSA_1,		/*!< Auto / Manual */
	NMEA_GPGSA_2,		/*!< 1 no fix, 2 2D fix, 3 3D fix */
	NMEA_GPGSA_3,		/*!< PRN 1 */
	NMEA_GPGSA_4,		/*!< PRN 2 */
	NMEA_GPGSA_5,		/*!< PRN 3 */
	NMEA_GPGSA_6,		/*!< PRN 4 */
	NMEA_GPGSA_7,		/*!< PRN 5 */
	NMEA_GPGSA_8,		/*!< PRN 6 */
	NMEA_GPGSA_9,		/*!< PRN 7 */
	NMEA_GPGSA_10,		/*!< PRN 8 */
	NMEA_GPGSA_11,		/*!< PRN 9 */
	NMEA_GPGSA_12,		/*!< PRN 10 */
	NMEA_GPGSA_13,		/*!< PRN 11 */
	NMEA_GPGSA_14,		/*!< PRN 12 */
	NMEA_GPGSA_15,		/*!< PDOP */
	NMEA_GPGSA_16,		/*!< horizontal dilution */
	NMEA_GPGSA_17,		/*!< vertical dilution */
	NMEA_GPGSA_CKSUM,	/*!< checksum */
#endif
};

typedef struct nmea_mumber_s
{
	int16_t		integer;
	uint8_t		fraction;
	uint8_t		changed;
} nmea_number_t;

typedef struct nmea_date_s
{
#if NMEA_DATE_AS_INT
	uint8_t		day;
	uint8_t		month;
	uint8_t		year;		/*!< nur die letzten 2 Ziffern */
#else
	char		str[6 + 1];
#endif
	uint8_t		changed;
} nmea_date_t;

typedef struct nmea_time_s
{
#if NMEA_TIME_AS_INT
	uint8_t		hour;
	uint8_t		minute;
	uint8_t		second;
#else
	char		str[6 + 1];
#endif
	uint8_t		changed;
} nmea_time_t;

extern uint8_t nmea_param_grp;
extern uint8_t nmea_param_id;

extern nmea_number_t	nmea_numbers[NMEA_RESULT_NUMBER_NO];
extern char				nmea_strings[NMEA_RESULT_STRING_NO][3];
extern nmea_date_t		nmea_date;
extern nmea_time_t		nmea_time;

extern uint8_t			nmea_group_complete[NMEA_GRP_NO];


int8_t nmea_parser (char data);

void nmea_init (void);

#endif
#endif
