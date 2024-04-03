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
#include "ds_nmea_parsers/Gga.h"
#include "ds_nmea_parsers/util.h"

#include <boost/algorithm/string/split.hpp>
#include <boost/algorithm/string/classification.hpp>

namespace ds_nmea_msgs
{

bool from_string(Gga& output, const std::string &nmea_string)
{
  auto hour = int{0};
  auto minute = int{0};
  auto second = 0.0;

  // Set defaults
  output.talker.clear();
  output.latitude = Gga::GGA_NO_DATA;
  output.latitude_dir = 0;
  output.longitude = Gga::GGA_NO_DATA;
  output.longitude_dir = 0;
  output.fix_quality = 0;
  output.num_satellites = 0;
  output.hdop = Gga::GGA_NO_DATA;
  output.antenna_alt = Gga::GGA_NO_DATA;
  output.antenna_alt_unit = 0;
  output.geoid_separation = Gga::GGA_NO_DATA;
  output.geoid_separation_unit = 0;
  output.dgps_age = Gga::GGA_NO_DATA;
  output.dgps_ref = 0;
  output.checksum = 0;

  // The GGA NMEA string can have lots of empty fields.  For example:
  //
  //    $GPGGA,212354.657,,,,,0,00,50.0,,M,0.0,M,,0000*4A
  //
  // is perfectly valid.  So we can't rely on a standard sscanf
  // with the whole message definition.
  //
  // Instead we'll split the message into substrings and use sscanf on each
  // part.  This is certainly not the most efficient way to do things BUT
  // we know that sscanf never generates exceptions (unlike boost::lexical_cast,
  // or some of the newer std::strtoX methods).  At worst we get bad data,
  // but at lest we don't have a lot of try/catch blocks.

  auto fields = std::vector<std::string>{};
  boost::split(fields, nmea_string, boost::is_any_of(",*"));

  // Expect at LEAST 15 fields (14 paylaod + talker)
  if (fields.size() < 15) {
    return false;
  }

  //         1         2      3 4        5 6 7  8   9   10 11  12 13  14   15
  //         |         |      | |        | | |  |   |   |  |  |  |   |    |
  // $--GGA,hhmmss.ss,llll.ll,a,yyyyy.yy,a,x,xx,x.x,x.x,M,x.x,M,x.x,xxxx*hh
  //  1) Time (UTC)
  //  2) Latitude
  //  3) N or S (North or South)
  //  4) Longitude
  //  5) E or W (East or West)
  //  6) GPS Quality Indicator,
  //  0 - fix not available,
  //  1 - GPS fix,
  //  2 - Differential GPS fix
  //  7) Number of satellites in view, 00 - 12
  //  8) Horizontal Dilution of precision
  //  9) Antenna Altitude above/below mean-sea-level (geoid)
  // 10) Units of antenna altitude, meters
  // 11) Geoidal separation, the difference between the WGS-84 earth ellipsoid and mean-sea-level (geoid), "-" means mean-sea-level below ellipsoid
  // 12) Units of geoidal separation, meters
  // 13) Age of differential GPS data, time in seconds since last SC104 type 1 or 9 update, null field when DGPS is not used
  // 14) Differential reference station ID, 0000-1023
  // 15) Checksum

  auto i = 0;
  char talker[2];
  if (!sscanf(fields.at(i++).c_str(), "$%2cGGA", talker)) {
    return false;
  }
  output.talker = std::string{std::begin(talker), std::end(talker)};
  // Break the first field into time components and create a ros::Time from it.
  if (sscanf(fields.at(i++).c_str(),"%02d%02d%lf", &hour, &minute, &second) != 3) {
    return false;
  }
  output.timestamp = from_nmea_utc(hour, minute, second);

  if(sscanf(fields.at(i++).c_str(), "%lf", &output.latitude) > 0) {
    output.latitude = nmea_dec_min_dec_degrees(output.latitude);
  }
  sscanf(fields.at(i++).c_str(), "%1c", &output.latitude_dir);

  if(sscanf(fields.at(i++).c_str(), "%lf", &output.longitude) > 0) {
    output.longitude = nmea_dec_min_dec_degrees(output.longitude);
  }
  sscanf(fields.at(i++).c_str(), "%1c", &output.longitude_dir);

  sscanf(fields.at(i++).c_str(), "%hhu", &output.fix_quality);
  sscanf(fields.at(i++).c_str(), "%hhu", &output.num_satellites);
  sscanf(fields.at(i++).c_str(), "%lf", &output.hdop);
  sscanf(fields.at(i++).c_str(), "%lf", &output.antenna_alt);
  sscanf(fields.at(i++).c_str(), "%1c", &output.antenna_alt_unit);
  sscanf(fields.at(i++).c_str(), "%lf", &output.geoid_separation);
  sscanf(fields.at(i++).c_str(), "%1c", &output.geoid_separation_unit);
  sscanf(fields.at(i++).c_str(), "%lf", &output.dgps_age);
  sscanf(fields.at(i++).c_str(), "%hu", &output.dgps_ref);

  if (fields.size() > 15) {
    sscanf(fields.at(i++).c_str(), "%hhx", &output.checksum);
  }

  return true;
}

}
