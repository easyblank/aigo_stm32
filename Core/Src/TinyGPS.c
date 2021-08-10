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
MERCHANTABILITY or FITNESS FOR A PARTICuAR PURPOSE.  See the GNU
Lesser General Public License for more details.

You should have received a copy of the GNU Lesser General Public
License along with this library; if not, write to the Free Software
Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
*/

#include "TinyGPS.h"
#include "math.h"
#include "stm32f4xx_hal.h"
#define TWO_PI M_PI*2
#define sq(n) (n*n)
#define radians(angleDegrees) (angleDegrees * M_PI / 180.0)
#define degrees(angleRadians) (angleRadians * 180.0 / M_PI)

uint32_t millis() {return 0;}

const float_t GPS_INVALID_F_ANGLE = 1000.0;
const float_t GPS_INVALID_F_ALTITUDE = 1000000.0;
const float_t GPS_INVALID_F_SPEED = -1.0;

typedef enum gps_sentence{_GPS_SENTENCE_GPGGA, _GPS_SENTENCE_GPRMC, _GPS_SENTENCE_OTHER}gps_sentence;

  // properties
  uint32_t _time, _new_time;
  uint32_t _date, _new_date;
  int32_t _latitude, _new_latitude;
  int32_t _longitude, _new_longitude;
  int32_t _altitude, _new_altitude;
  uint32_t  _speed, _new_speed;
  uint32_t  _course, _new_course;
  uint32_t  _hdop, _new_hdop;
  uint8_t _numsats, _new_numsats;

  uint32_t _last_time_fix, _new_time_fix;
  uint32_t _last_position_fix, _new_position_fix;

  // parsing state variables
  uint8_t _parity;
  bool _is_checksum_term;
  uint8_t _term[15];
  uint8_t _sentence_type;
  uint8_t _term_number;
  uint8_t _term_offset;
  bool _gps_data_good;

#ifndef _GPS_NO_STATS
  // statistics
  uint32_t _encoded_characters;
  uint8_t _good_sentences;
  uint8_t _failed_checksum;
  uint8_t _passed_checksum;
#endif

  // internal utilities
  int16_t from_hex(uint8_t a);
  uint32_t parse_decimal();
  uint32_t parse_degrees();
  bool term_complete();
  bool gpsisdigit(uint8_t c) { return c >= '0' && c <= '9'; }
  int32_t gpsatol(const uint8_t *str);
  int16_t gpsstrcmp(const uint8_t *str1, const uint8_t *str2);

const char* _GPRMC_TERM = "GPRMC";
const char* _GPGGA_TERM = "GPGGA";

void TinyGPS()
{
  _time = GPS_INVALID_TIME;
  _date = GPS_INVALID_DATE;
  _latitude = GPS_INVALID_ANGLE;
  _longitude = GPS_INVALID_ANGLE;
  _altitude = GPS_INVALID_ALTITUDE;
  _speed = GPS_INVALID_SPEED;
  _course = GPS_INVALID_ANGLE;
  _hdop = GPS_INVALID_HDOP;
  _numsats = GPS_INVALID_SATELLITES;
  _last_time_fix = GPS_INVALID_FIX_TIME;
  _last_position_fix = GPS_INVALID_FIX_TIME;
  _parity = 0;
  _is_checksum_term = false;
  _sentence_type = _GPS_SENTENCE_OTHER;
  _term_number = 0;
  _term_offset = 0;
  _gps_data_good = false;
  _term[0] = '\0';
}

//
// public methods
//

bool encode(uint8_t c)
{
  bool valid_sentence = false;

#ifndef _GPS_NO_STATS
  ++_encoded_characters;
#endif
  switch(c)
  {
  case ',': // term terminators
    _parity ^= c;
  case '\r':
  case '\n':
  case '*':
    if (_term_offset < sizeof(_term))
    {
      _term[_term_offset] = 0;
      valid_sentence = term_complete();
    }
    ++_term_number;
    _term_offset = 0;
    _is_checksum_term = c == '*';
    return valid_sentence;

  case '$': // sentence begin
    _term_number = _term_offset = 0;
    _parity = 0;
    _sentence_type = _GPS_SENTENCE_OTHER;
    _is_checksum_term = false;
    _gps_data_good = false;
    return valid_sentence;
  }

  // ordinary characters
  if (_term_offset < sizeof(_term) - 1)
    _term[_term_offset++] = c;
  if (!_is_checksum_term)
    _parity ^= c;

  return valid_sentence;
}

//
// internal utilities
//
int16_t from_hex(uint8_t a)
{
  if (a >= 'A' && a <= 'F')
    return a - 'A' + 10;
  else if (a >= 'a' && a <= 'f')
    return a - 'a' + 10;
  else
    return a - '0';
}

uint32_t parse_decimal()
{
  uint8_t *p = _term;
  bool isneg = *p == '-';
  if (isneg) ++p;
  uint32_t ret = 100u * gpsatol(p);
  while (gpsisdigit(*p)) ++p;
  if (*p == '.')
  {
    if (gpsisdigit(p[1]))
    {
      ret += 10 * (p[1] - '0');
      if (gpsisdigit(p[2]))
        ret += p[2] - '0';
    }
  }
  return isneg ? -ret : ret;
}

// Parse a string in the form ddmm.mmmmmmm...
uint32_t parse_degrees()
{
  uint8_t *p;
  uint32_t left_of_decimal = gpsatol(_term);
  uint32_t hundred1000ths_of_minute = (left_of_decimal % 100u) * 100000u;
  for (p=_term; gpsisdigit(*p); ++p);
  if (*p == '.')
  {
    uint32_t mult = 10000;
    while (gpsisdigit(*++p))
    {
      hundred1000ths_of_minute += mult * (*p - '0');
      mult /= 10;
    }
  }
  return (left_of_decimal / 100) * 1000000 + (hundred1000ths_of_minute + 3) / 6;
}

#define COMBINE(sentence_type, term_number) (((unsigned)(sentence_type) << 5) | term_number)

// Processes a just-completed term
// Returns true if new sentence has just passed checksum test and is validated
bool term_complete()
{
  if (_is_checksum_term)
  {
    uint8_t checksum = 16 * from_hex(_term[0]) + from_hex(_term[1]);
    if (checksum == _parity)
    {
      if (_gps_data_good)
      {
#ifndef _GPS_NO_STATS
        ++_good_sentences;
#endif
        _last_time_fix = _new_time_fix;
        _last_position_fix = _new_position_fix;

        switch(_sentence_type)
        {
        case _GPS_SENTENCE_GPRMC:
          _time      = _new_time;
          _date      = _new_date;
          _latitude  = _new_latitude;
          _longitude = _new_longitude;
          _speed     = _new_speed;
          _course    = _new_course;
          break;
        case _GPS_SENTENCE_GPGGA:
          _altitude  = _new_altitude;
          _time      = _new_time;
          _latitude  = _new_latitude;
          _longitude = _new_longitude;
          _numsats   = _new_numsats;
          _hdop      = _new_hdop;
          break;
        }

        return true;
      }
    }

#ifndef _GPS_NO_STATS
    else
      ++_failed_checksum;
#endif
    return false;
  }

  // the first term determines the sentence type
  if (_term_number == 0)
  {
    if (!gpsstrcmp(_term, _GPRMC_TERM))
      _sentence_type = _GPS_SENTENCE_GPRMC;
    else if (!gpsstrcmp(_term, _GPGGA_TERM))
      _sentence_type = _GPS_SENTENCE_GPGGA;
    else
      _sentence_type = _GPS_SENTENCE_OTHER;
    return false;
  }

  if (_sentence_type != _GPS_SENTENCE_OTHER && _term[0])
    switch(COMBINE(_sentence_type, _term_number))
  {
    case COMBINE(_GPS_SENTENCE_GPRMC, 1): // Time in both sentences
    case COMBINE(_GPS_SENTENCE_GPGGA, 1):
      _new_time = parse_decimal();
      _new_time_fix = millis();
      break;
    case COMBINE(_GPS_SENTENCE_GPRMC, 2): // GPRMC validity
      _gps_data_good = _term[0] == 'A';
      break;
    case COMBINE(_GPS_SENTENCE_GPRMC, 3): // Latitude
    case COMBINE(_GPS_SENTENCE_GPGGA, 2):
      _new_latitude = parse_degrees();
      _new_position_fix = millis();
      break;
    case COMBINE(_GPS_SENTENCE_GPRMC, 4): // N/S
    case COMBINE(_GPS_SENTENCE_GPGGA, 3):
      if (_term[0] == 'S')
        _new_latitude = -_new_latitude;
      break;
    case COMBINE(_GPS_SENTENCE_GPRMC, 5): // Longitude
    case COMBINE(_GPS_SENTENCE_GPGGA, 4):
      _new_longitude = parse_degrees();
      break;
    case COMBINE(_GPS_SENTENCE_GPRMC, 6): // E/W
    case COMBINE(_GPS_SENTENCE_GPGGA, 5):
      if (_term[0] == 'W')
        _new_longitude = -_new_longitude;
      break;
    case COMBINE(_GPS_SENTENCE_GPRMC, 7): // Speed (GPRMC)
      _new_speed = parse_decimal();
      break;
    case COMBINE(_GPS_SENTENCE_GPRMC, 8): // Course (GPRMC)
      _new_course = parse_decimal();
      break;
    case COMBINE(_GPS_SENTENCE_GPRMC, 9): // Date (GPRMC)
      _new_date = gpsatol(_term);
      break;
    case COMBINE(_GPS_SENTENCE_GPGGA, 6): // Fix data (GPGGA)
      _gps_data_good = _term[0] > '0';
      break;
    case COMBINE(_GPS_SENTENCE_GPGGA, 7): // Satellites used (GPGGA)
      _new_numsats = (uint8_t)atoi(_term);
      break;
    case COMBINE(_GPS_SENTENCE_GPGGA, 8): // HDOP
      _new_hdop = parse_decimal();
      break;
    case COMBINE(_GPS_SENTENCE_GPGGA, 9): // Altitude (GPGGA)
      _new_altitude = parse_decimal();
      break;
  }

  return false;
}

int32_t gpsatol(const uint8_t *str)
{
  int32_t ret = 0;
  while (gpsisdigit(*str))
    ret = 10 * ret + *str++ - '0';
  return ret;
}

int16_t gpsstrcmp(const uint8_t *str1, const uint8_t *str2)
{
  while (*str1 && *str1 == *str2)
    ++str1, ++str2;
  return *str1;
}

/* static */
float_t distance_between (float_t lat1, float_t long1, float_t lat2, float_t long2)
{
  // returns distance in meters between two positions, both specified
  // as signed decimal-degrees latitude and longitude. Uses great-circle
  // distance computation for hypothetical sphere of radius 6372795 meters.
  // Because Earth is no exact sphere, rounding errors may be up to 0.5%.
  // Courtesy of Maarten Lamers
  float_t delta = radians(long1-long2);
  float_t sdlong = sin(delta);
  float_t cdlong = cos(delta);
  lat1 = radians(lat1);
  lat2 = radians(lat2);
  float_t slat1 = sin(lat1);
  float_t clat1 = cos(lat1);
  float_t slat2 = sin(lat2);
  float_t clat2 = cos(lat2);
  delta = (clat1 * slat2) - (slat1 * clat2 * cdlong);
  delta = sq(delta);
  delta += sq(clat2 * sdlong);
  delta = sqrt(delta);
  float_t denom = (slat1 * slat2) + (clat1 * clat2 * cdlong);
  delta = atan2(delta, denom);
  return delta * 6372795;
}

float_t course_to (float_t lat1, float_t long1, float_t lat2, float_t long2)
{
  // returns course in degrees (North=0, West=270) from position 1 to position 2,
  // both specified as signed decimal-degrees latitude and longitude.
  // Because Earth is no exact sphere, calculated course may be off by a tiny fraction.
  // Courtesy of Maarten Lamers
  float_t dlon = radians(long2-long1);
  lat1 = radians(lat1);
  lat2 = radians(lat2);
  float_t a1 = sin(dlon) * cos(lat2);
  float_t a2 = sin(lat1) * cos(lat2) * cos(dlon);
  a2 = cos(lat1) * sin(lat2) - a2;
  a2 = atan2(a1, a2);
  if (a2 < 0.0)
  {
    a2 += TWO_PI;
  }
  return degrees(a2);
}


// lat/int32_t in MILLIONTHs of a degree and age of fix in milliseconds
// (note: versions 12 and earlier gave this value in 100,000ths of a degree.
void get_position(int32_t *latitude, int32_t *longitude, uint32_t *fix_age)
{
  if (latitude) *latitude = _latitude;
  if (longitude) *longitude = _longitude;
  if (fix_age) *fix_age = _last_position_fix == GPS_INVALID_FIX_TIME ?
   GPS_INVALID_AGE : millis() - _last_position_fix;
}

// date as ddmmyy, time as hhmmsscc, and age in milliseconds
void get_datetime(uint32_t *date, uint32_t *time, uint32_t *age)
{
  if (date) *date = _date;
  if (time) *time = _time;
  if (age) *age = _last_time_fix == GPS_INVALID_FIX_TIME ?
   GPS_INVALID_AGE : millis() - _last_time_fix;
}

void f_get_position(float_t *latitude, float_t *longitude, uint32_t *fix_age)
{
  int32_t lat, lon;
  get_position(&lat, &lon, fix_age);
  *latitude = lat == GPS_INVALID_ANGLE ? GPS_INVALID_F_ANGLE : (lat / 1000000.0);
  *longitude = lat == GPS_INVALID_ANGLE ? GPS_INVALID_F_ANGLE : (lon / 1000000.0);
}

void crack_datetime(int16_t *year, uint8_t *month, uint8_t *day,
  uint8_t *hour, uint8_t *minute, uint8_t *second, uint8_t *hundredths, uint32_t *age)
{
  uint32_t date, time;
  get_datetime(&date, &time, age);
  if (year)
  {
    *year = date % 100;
    *year += *year > 80 ? 1900 : 2000;
  }
  if (month) *month = (date / 100) % 100;
  if (day) *day = date / 10000;
  if (hour) *hour = time / 1000000;
  if (minute) *minute = (time / 10000) % 100;
  if (second) *second = (time / 100) % 100;
  if (hundredths) *hundredths = time % 100;
}

float_t f_altitude()
{
  return _altitude == GPS_INVALID_ALTITUDE ? GPS_INVALID_F_ALTITUDE : _altitude / 100.0;
}

float_t f_course()
{
  return _course == GPS_INVALID_ANGLE ? GPS_INVALID_F_ANGLE : _course / 100.0;
}

float_t f_speed_knots()
{
  return _speed == GPS_INVALID_SPEED ? GPS_INVALID_F_SPEED : _speed / 100.0;
}
/**
float_t f_speed_mph()
{
  float_t sk = f_speed_knots();
  return sk == GPS_INVALID_F_SPEED ? GPS_INVALID_F_SPEED : _GPS_MPH_PER_KNOT * sk;
}

float_t f_speed_mps()
{
  float_t sk = f_speed_knots();
  return sk == GPS_INVALID_F_SPEED ? GPS_INVALID_F_SPEED : _GPS_MPS_PER_KNOT * sk;
}

float_t f_speed_kmph()
{
  float_t sk = f_speed_knots();
  return sk == GPS_INVALID_F_SPEED ? GPS_INVALID_F_SPEED : _GPS_KMPH_PER_KNOT * sk;
}
**/
// signed altitude in centimeters (from GPGGA sentence)
int32_t altitude() { return _altitude; }

  // course in last full GPRMC sentence in 100th of a degree
uint32_t course() { return _course; }

  // speed in last full GPRMC sentence in 100ths of a knot
uint32_t speed() { return _speed; }

  // satellites used in last full GPGGA sentence
uint8_t satellites() { return _numsats; }

  // horizontal dilution of precision in 100ths
uint32_t hdop() { return _hdop; }

