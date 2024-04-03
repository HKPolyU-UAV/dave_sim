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

#include <list>

#include <gtest/gtest.h>

TEST(PIXSE_GGA, valid_strings)
{

  auto gen = [](
      std::string talker, ros::Time timestamp, double latitude, uint8_t latitude_dir, double longitude, uint8_t longitude_dir,
      uint8_t fix_quality, uint8_t num_satellites, double hdop, double antenna_alt, uint8_t antenna_alt_unit,
      double geoid_separation, uint8_t geoid_separation_unit, double dgps_age, uint16_t dgps_ref, uint8_t checksum)
  {
    auto msg = ds_nmea_msgs::Gga{};
    msg.talker = talker;
    msg.timestamp = timestamp;
    msg.latitude = latitude;
    msg.latitude_dir = latitude_dir;
    msg.longitude = longitude;
    msg.longitude_dir = longitude_dir;
    msg.fix_quality = fix_quality;
    msg.num_satellites = num_satellites;
    msg.hdop = hdop;
    msg.antenna_alt = antenna_alt;
    msg.antenna_alt_unit = antenna_alt_unit;
    msg.geoid_separation = geoid_separation;
    msg.geoid_separation_unit = geoid_separation_unit;
    msg.dgps_age = dgps_age;
    msg.dgps_ref = dgps_ref;
    msg.checksum = checksum;
    return msg;
  };

  const auto test_pairs =
      std::list<std::pair<std::string, ds_nmea_msgs::Gga>>{
          {
              "$GPGGA,212354.657,,,,,0,00,50.0,,M,0.0,M,,0000*4A",
              gen(
                  "GP",
                  ds_nmea_msgs::from_nmea_utc(21, 23, 54.657),
                  ds_nmea_msgs::Gga::GGA_NO_DATA,
                  0,
                  ds_nmea_msgs::Gga::GGA_NO_DATA,
                  0,
                  0,
                  0,
                  50.0,
                  ds_nmea_msgs::Gga::GGA_NO_DATA,
                  'M',
                  0,
                  'M',
                  ds_nmea_msgs::Gga::GGA_NO_DATA,
                  0,
                  0x4A
              )
          },
          {
              "$GPGGA,140555.824,,,,,0,00,50.0,,M,0.0,M,,0000*43",
              gen(
                  "GP",
                  ds_nmea_msgs::from_nmea_utc(14, 05, 55.824),
                  ds_nmea_msgs::Gga::GGA_NO_DATA,
                  0,
                  ds_nmea_msgs::Gga::GGA_NO_DATA,
                  0,
                  0,
                  0,
                  50.0,
                  ds_nmea_msgs::Gga::GGA_NO_DATA,
                  'M',
                  0,
                  'M',
                  ds_nmea_msgs::Gga::GGA_NO_DATA,
                  0,
                  0x43
              )
          },
          {
              "$GPGGA,143010.357,4131.5546,N,07040.0115,W,1,04,2.5,13.4,M,-34.4,M,,0000*50\r\n",
              gen(
                  "GP",
                  ds_nmea_msgs::from_nmea_utc(14, 30, 10.357),
                  41.52591,
                  'N',
                  70.66686,
                  'W',
                  1,
                  4,
                  2.5,
                  13.4,
                  'M',
                  -34.4,
                  'M',
                  ds_nmea_msgs::Gga::GGA_NO_DATA,
                  0,
                  0x50
              )
          }
      };

  // Loop through all provided cases
  for (const auto& test_pair : test_pairs)
  {
    auto msg = ds_nmea_msgs::Gga{};
    auto expected = test_pair.second;
    const auto ok = ds_nmea_msgs::from_string(msg, test_pair.first);

    // Should have succeeded
    ASSERT_TRUE(ok);

    // All fields should match.
    EXPECT_STREQ(expected.talker.data(), msg.talker.data());
    EXPECT_EQ(expected.timestamp, msg.timestamp);
    EXPECT_FLOAT_EQ(expected.latitude, msg.latitude);
    EXPECT_EQ(expected.latitude_dir, msg.latitude_dir);
    EXPECT_FLOAT_EQ(expected.longitude, msg.longitude);
    EXPECT_EQ(expected.longitude_dir, msg.longitude_dir);
    EXPECT_EQ(expected.fix_quality, msg.fix_quality);
    EXPECT_EQ(expected.num_satellites, msg.num_satellites);
    EXPECT_FLOAT_EQ(expected.hdop, msg.hdop);
    EXPECT_FLOAT_EQ(expected.antenna_alt, msg.antenna_alt);
    EXPECT_EQ(expected.antenna_alt_unit, msg.antenna_alt_unit);
    EXPECT_FLOAT_EQ(expected.geoid_separation, msg.geoid_separation);
    EXPECT_EQ(expected.geoid_separation_unit, msg.geoid_separation_unit);
    EXPECT_FLOAT_EQ(expected.dgps_age, msg.dgps_age);
    EXPECT_EQ(expected.dgps_ref, msg.dgps_ref);
    EXPECT_EQ(expected.checksum, msg.checksum);
  }
}

TEST(PIXSE_GGA, invalid_strings) {
  // This is a failed string observed from the xeos
  auto test_strs = std::list<std::pair<std::string, bool> > {
      {"$GPGGA,223513.000,4131.5356,N,07039.9989W0,1,08,0.9,22.8,M,-34.4,M,,0000*53", true},
      {"$GPGGA,223513.000,4131.5356,N,07039.9989W0,1,08,0.9,22.8,M,-34.40M,,0000*53", false},
      {"$GPGGA,202819.000,1343.7726,N,04501.4020,W,1,10,0.7,8.4,M,-26.3,M,,0000*69\r\n", true}
  };
  for (const auto& test_pair : test_strs) {
    auto msg = ds_nmea_msgs::Gga{};
    const auto ok = ds_nmea_msgs::from_string(msg, test_pair.first);

    // Should have failed, but some tests succeed anyway.
    EXPECT_EQ(test_pair.second, ok);
  }
}

// Run all the tests that were declared with TEST()
int main(int argc, char** argv)
{
  ros::Time::init();
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

