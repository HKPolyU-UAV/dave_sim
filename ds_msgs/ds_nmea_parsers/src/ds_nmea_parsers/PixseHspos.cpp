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

#include "ds_nmea_parsers/PixseHspos.h"
#include "ds_nmea_parsers/util.h"

namespace ds_nmea_msgs
{

bool from_string(PixseHspos& output, const std::string &nmea_string)
{

  auto hour = int{0};
  auto minute = int{0};
  auto second = 0.0;

  const auto n = sscanf(
      nmea_string.c_str(), "$PIXSE,HSPOS_,%02d%02d%lf,%lf,%1c,%lf,%1c,%lf,%lf,%lf,%lf,%lf,%hd,%1c,%lf,%lf,%lf,%lf,%lf\r\n",
      &hour, &minute, &second, &output.latitude, &output.latitude_dir, &output.longitude, &output.longitude_dir,
      &output.depth, &output.altitude, &output.latitude_stdev, &output.longitude_stdev, &output.depth_stdev,
      &output.longitude_utm_zone, &output.latitude_utm_zone, &output.eastings, &output.northings,
      &output.heading_misalignment, &output.heading_misalignment_scale_factor, &output.sound_velocity);

  if (n < 19) {
    return false;
  }

  output.timestamp = from_nmea_utc(hour,  minute, second);

  return true;
}


}
