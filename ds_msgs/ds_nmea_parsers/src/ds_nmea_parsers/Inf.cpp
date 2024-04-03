/**
* Copyright 2019 Woods Hole Oceanographic Institution
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
// Created by jvaccaro on 9/2/19.
//

#include "ds_nmea_parsers/Inf.h"
#include "ds_nmea_parsers/util.h"

namespace ds_nmea_msgs {

bool from_string(Inf& output, const std::string& nmea_string){
  unsigned char hex0 = 0;
  unsigned char hex1 = 0;
  unsigned char hex2 = 0;
  unsigned char hex3 = 0;
  unsigned char hex4 = 0;
  unsigned char hex5 = 0;
  unsigned char hex6 = 0;
  unsigned char hex7 = 0;
  const auto n = sscanf(
      nmea_string.c_str(), "$%*2cINF,%01hhx%01hhx%01hhx%01hhx%01hhx%01hhx%01hhx%01hhx*%02hhx\r\n",
      &hex7, &hex6, &hex5, &hex4, &hex3, &hex2, &hex1, &hex0, &output.checksum);
  if (n < 9){
    return false;
  }
  output.heading_invalid = (hex0 >> 0) & 0x01;
  output.pitch_invalid = (hex0 >> 1 ) & 0x01;
  output.roll_invalid = (hex0 >> 2) & 0x01;
  return true;
//  if (output.checksum == calculate_checksum(nmea_string)){
//    return true;
//  }
  return false;
}

} //namespace