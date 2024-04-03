/**
* Copyright 2018 Woods Hole Oceanographic Institution
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions are met:
*
* 1. Redistributions of source code must retain the above copyright notice,
*    this list of conditions and the following disclaimer.
*
* 2. Redistributions in binary form must reproduce the above copyright notice,
*    this list of conditions and the following disclaimer in the documentation
*    and/or other materials provided with the distribution.
*
* 3. Neither the name of the copyright holder nor the names of its contributors
*    may be used to endorse or promote products derived from this software
*    without specific prior written permission.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
* AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
* IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
* ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
* LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
* CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
* SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
* INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
* CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
* ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
* POSSIBILITY OF SUCH DAMAGE.
*/
//
// Created by zac on 1/21/18.
//

#ifndef DS_NMEA_PARSERS_UTIL_H
#define DS_NMEA_PARSERS_UTIL_H

#include <cstdint>
#include <string>

#include <ros/time.h>

namespace ds_nmea_msgs
{

uint8_t calculate_checksum(const std::string& nmea_msg);

/// \@brief Convert nmea-style latitude to decimal degrees
///
/// Latitudes in most NMEA messages are in decimal minutes packed
/// as a single number.  For example:
///
///    5423.453
///
/// is really 54deg 23.453min, which is 54.39088deg
///
/// \param nmea_decmin
/// \return
double nmea_dec_min_dec_degrees(double nmea_decmin) noexcept;

/// @brief Convert a clock time to a full UTC date
///
/// Many NMEA sentances provide timestamps in UTC that only contain
/// the time.  For example, GGA's have a timestamp that looks like:
///
///   HHMMSS.SSSS
///
/// This method provides the missing date information by using
/// ros::Time::now() and replacing the time information with the values
/// provided.
///
///
/// \param hours
/// \param minutes
/// \param seconds
/// \return
ros::Time from_nmea_utc(int hours, int minutes, double seconds);

/// @brief Convert a full rostime into a UTC clock time string
///
/// \param time
/// \return
std::string to_nmea_utc_str(ros::Time time);

/// @brief Convert a ZDA time to a full UTC date
///
/// The ZDA string provides year/month/day in addition to time.  Do the conversion.
///
/// \param year
/// \param month
/// \param day
/// \param hours
/// \param minutes
/// \param seconds
/// \return
ros::Time from_nmea_utc_date(int year, int month, int day, int hours, int minutes, double seconds);

std::string to_nmea_lat_string(double lat_deg, double minutes = 0.0, double seconds = 0.0);

std::string to_nmea_lon_string(double lon_deg, double minutes = 0.0, double seconds = 0.0);

}
#endif //DS_NMEA_PARSERS_UTIL_H
