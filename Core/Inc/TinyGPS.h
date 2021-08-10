/*
TinyGPS - a small GPS library for Arduino providing basic NMEA parsing
Based on work by and "distance_to" and "course_to" courtesy of Maarten Lamers.
Suggestion to add satellites(), course_to(), and cardinal(), by Matt Monson.
Precision improvements suggested by Wayne Holder.
Copyright (C) 2008-2013 Mikal Hart
All rights reserved.

This library is free software; you can redistribute it and/or
modify it under the terms of the GNU Lesser General Public
License as published by the Free Software Foundation; either
version 2.1 of the License, or (at your option) any later version.

This library is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
Lesser General Public License for more details.

You should have received a copy of the GNU Lesser General Public
License along with this library; if not, write to the Free Software
Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
*/

#ifdef __cplusplus
extern "C" {
#endif


#include <stdlib.h>
#include <math.h>
#include "stm32f4xx_hal.h"
#include "stdbool.h"



// #define _GPS_NO_STATS


typedef enum gps_const{
    GPS_INVALID_AGE = 0xFFFFFFFF,      GPS_INVALID_ANGLE = 999999999,
    GPS_INVALID_ALTITUDE = 999999999,  GPS_INVALID_DATE = 0,
    GPS_INVALID_TIME = 0xFFFFFFFF,     GPS_INVALID_SPEED = 999999999,
    GPS_INVALID_FIX_TIME = 0xFFFFFFFF, GPS_INVALID_SATELLITES = 0xFF,
    GPS_INVALID_HDOP = 0xFFFFFFFF
}gps_const;

 // static const float_t GPS_INVALID_F_ANGLE, GPS_INVALID_F_ALTITUDE, GPS_INVALID_F_SPEED;

void TinyGPS();
bool encode(uint8_t c); // process one character received from GPS

  // lat/long in MILLIONTHs of a degree and age of fix in milliseconds
  // (note: versions 12 and earlier gave lat/long in 100,000ths of a degree.
void get_position(int32_t *latitude, int32_t *longitude, uint32_t *fix_age);

  // date as ddmmyy, time as hhmmsscc, and age in milliseconds
void get_datetime(uint32_t *date, uint32_t *time, uint32_t *age);

  // signed altitude in centimeters (from GPGGA sentence)
int32_t altitude();

  // course in last full GPRMC sentence in 100th of a degree
 uint32_t course();

  // speed in last full GPRMC sentence in 100ths of a knot
 uint32_t speed();

  // satellites used in last full GPGGA sentence
 uint8_t satellites();

  // horizontal dilution of precision in 100ths
 uint32_t hdop();

void f_get_position(float_t *latitude, float_t *longitude, uint32_t *fix_age);

void crack_datetime(int16_t *year, uint8_t *month, uint8_t *day,
    uint8_t *hour, uint8_t *minute, uint8_t *second, uint8_t *hundredths, uint32_t *fix_age);
float_t f_altitude();
float_t f_course();
float_t f_speed_knots();
float_t f_speed_mph();
float_t f_speed_mps();
float_t f_speed_kmph();

float_t distance_between (float_t lat1, float_t long1, float_t lat2, float_t long2);
float_t course_to (float_t lat1, float_t long1, float_t lat2, float_t long2);

#ifndef _GPS_NO_STATS
  void stats(uint32_t *chars, uint8_t *good_sentences, uint8_t *failed_cs);
#endif

#ifdef __cplusplus
} // "C"
#endif
