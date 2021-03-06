/*
  TinyGPS - a small GPS library for Arduino providing basic NMEA parsing
  Copyright (C) 2008-9 Mikal Hart
  All rights reserved.

  Satellite Count Mod - by Brett Hagman
  http://www.roguerobotics.com/

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

#include "WProgram.h"
#include "TinyGPS.h"

#define _GPRMC_TERM   "GPRMC"
#define _GPGGA_TERM   "GPGGA"
#define _GPGSV_TERM   "GPGSV"
#define _GPGSA_TERM   "GPGSA"

TinyGPS::TinyGPS()
:  _time(GPS_INVALID_TIME)
,  _date(GPS_INVALID_DATE)
,  _latitude(GPS_INVALID_ANGLE)
,  _longitude(GPS_INVALID_ANGLE)
,  _altitude(GPS_INVALID_ALTITUDE)
,  _speed(GPS_INVALID_SPEED)
,  _course(GPS_INVALID_ANGLE)
,  _satsinview(0)
,  _satsused(0)
,  _fixtype(GPS_FIX_NO_FIX)
,  _last_time_fix(GPS_INVALID_FIX_TIME)
,  _last_position_fix(GPS_INVALID_FIX_TIME)
,  _parity(0)
,  _is_checksum_term(false)
,  _sentence_type(_GPS_SENTENCE_OTHER)
,  _term_number(0)
,  _term_offset(0)
#ifndef _GPS_NO_STATS
,  _encoded_characters(0)
,  _good_sentences(0)
,  _failed_checksum(0)
#endif
{
  _term[0] = '\0';
}

//
// public methods
//

bool TinyGPS::encode(char c)
{
  bool valid_sentence = false;

  ++_encoded_characters;
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
    return valid_sentence;
  }

  // ordinary characters
  if (_term_offset < sizeof(_term) - 1)
    _term[_term_offset++] = c;
  if (!_is_checksum_term)
    _parity ^= c;

  return valid_sentence;
}

#ifndef _GPS_NO_STATS
void TinyGPS::stats(unsigned long *chars, unsigned short *sentences, unsigned short *failed_cs)
{
  if (chars) *chars = _encoded_characters;
  if (sentences) *sentences = _good_sentences;
  if (failed_cs) *failed_cs = _failed_checksum;
}
#endif

//
// internal utilities
//
int TinyGPS::from_hex(char a) 
{
  if (a >= 'A' && a <= 'F')
    return a - 'A' + 10;
  else if (a >= 'a' && a <= 'f')
    return a - 'a' + 10;
  else
    return a - '0';
}

unsigned long TinyGPS::parse_decimal()
{
  char *p = _term;
  bool isneg = *p == '-';
  if (isneg) ++p;
  unsigned long ret = 100UL * gpsatol(p);
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

unsigned long TinyGPS::parse_degrees()
{
  char *p;
  unsigned long left = gpsatol(_term);
  unsigned long tenk_minutes = (left % 100UL) * 10000UL;
  for (p=_term; gpsisdigit(*p); ++p);
  if (*p == '.')
  {
    unsigned long mult = 1000;
    while (gpsisdigit(*++p))
    {
      tenk_minutes += mult * (*p - '0');
      mult /= 10;
    }
  }
  return (left / 100) * 100000 + tenk_minutes / 6;
}

// Processes a just-completed term
// Returns true if new sentence has just passed checksum test and is validated
bool TinyGPS::term_complete()
{
  if (_is_checksum_term)
  {
    byte checksum = 16 * from_hex(_term[0]) + from_hex(_term[1]);
    if (checksum == _parity)
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
          break;
        case _GPS_SENTENCE_GPGSV:
          _satsinview = _new_satsinview;
          break;
        case _GPS_SENTENCE_GPGSA:
          _satsused = _new_satsused;
          _fixtype = _new_fixtype;
          break;
      }
      return true;
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
    else if (!gpsstrcmp(_term, _GPGSV_TERM))
    {
      _sentence_type = _GPS_SENTENCE_GPGSV;
    }
    else if (!gpsstrcmp(_term, _GPGSA_TERM))
    {
      _sentence_type = _GPS_SENTENCE_GPGSA;
      _new_satsused = 0;
    }
    else
      _sentence_type = _GPS_SENTENCE_OTHER;
    return false;
  }

  if (_sentence_type == _GPS_SENTENCE_GPGSV)
  {
    if (_term_number == 3 && _term[0])
    {
      // we've got our number of sats
      // NOTE: we will more than likely hit this a few times in a row, because
      // there are usually multiple GPGSV sentences to describe all of the sats
      _new_satsinview = (unsigned char) gpsatol(_term);
    }
  }
  else if (_sentence_type == _GPS_SENTENCE_GPGSA)
  {
    if (_term_number == 2 && _term[0])  // Fix type
    {
      _new_fixtype = (unsigned char) gpsatol(_term);
    }
    else if (_term_number >= 3 && _term_number <= 14 && _term[0]) // Count our sats used
    {
      _new_satsused++;
    }
//    if (_term_number == 15)  // PDOP
//    if (_term_number == 16)  // HDOP
//    if (_term_number == 17)  // VDOP
  }   
  else if (_sentence_type != _GPS_SENTENCE_OTHER && _term[0])
  {
    switch((_sentence_type == _GPS_SENTENCE_GPGGA ? 200 : 100) + _term_number)
    {
      case 101: // Time in both sentences
      case 201:
        _new_time = parse_decimal();
        _new_time_fix = millis();
        break;
      case 103: // Latitude
      case 202:
        _new_latitude = parse_degrees();
        _new_position_fix = millis();
        break;
      case 104: // N/S
      case 203:
        if (_term[0] == 'S')
          _new_latitude = -_new_latitude;
        break;
      case 105: // Longitude
      case 204:
        _new_longitude = parse_degrees();
        break;
      case 106: // E/W
      case 205:
        if (_term[0] == 'W')
          _new_longitude = -_new_longitude;
        break;
      case 107: // Speed (GPRMC)
        _new_speed = parse_decimal();
        break;
      case 108: // Course (GPRMC)
        _new_course = parse_decimal();
        break;
      case 109: // Date (GPRMC)
        _new_date = gpsatol(_term);
        break;
      case 209: // Altitude (GPGGA)
        _new_altitude = parse_decimal();
        break;
    }
  }

  return false;
}

long TinyGPS::gpsatol(const char *str)
{
  long ret = 0;
  while (gpsisdigit(*str))
    ret = 10 * ret + *str++ - '0';
  return ret;
}

int TinyGPS::gpsstrcmp(const char *str1, const char *str2)
{
  while (*str1 && *str1 == *str2)
    ++str1, ++str2;
  return *str1;
}