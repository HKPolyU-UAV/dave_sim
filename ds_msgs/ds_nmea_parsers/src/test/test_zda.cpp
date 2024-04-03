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
#include "ds_nmea_parsers/Zda.h"
#include "ds_nmea_parsers/util.h"

#include <list>

#include <gtest/gtest.h>

using namespace ds_nmea_msgs;

TEST(PIXSE_ZDA, valid_strings)
{

  auto gen = [](
      std::string talker, ros::Time utc_time, ros::Duration zone_offset, uint8_t checksum)
  {
    auto msg = ds_nmea_msgs::Zda{};
    msg.talker = talker;
    msg.utc_time = utc_time;
    msg.zone_offset = zone_offset;
    msg.checksum = checksum;
    return msg;
  };

  const auto test_pairs =
      std::list<std::pair<std::string, ds_nmea_msgs::Zda>>{
          {
              "$GNZDA,000000.00,01,01,2000,,*7A\r\n",
              gen("GN", ros::Time(946684800,0), ros::Duration(0,0), 0x7A)
          },
          {
              "$GNZDA,010203.00,05,04,2006,,*7D\r\n",
              gen("GN", ros::Time(1144198923,0), ros::Duration(0,0), 0x7D)
          },
          {
              "$GNZDA,010203.95,05,04,2006,,*7D\r\n",
              gen("GN", ros::Time(1144198923,0.95e9), ros::Duration(0,0), 0x7D)
          },
          {
              "$GNZDA,010203.95,05,04,2006,3,0*72\r\n",
              gen("GN", ros::Time(1144198923,0.95e9), ros::Duration(3*3600,0), 0x72)
          },
          {
              "$GNZDA,010203.95,05,04,2006,3,30*41\r\n",
              gen("GN", ros::Time(1144198923,0.95e9), ros::Duration(3*3600+30*60,0), 0x41)
          },
          {
              "$GNZDA,010203.95,05,04,2006,-3,30*6C\r\n",
              gen("GN", ros::Time(1144198923,0.95e9), ros::Duration(-3*3600-30*60,0), 0x6C)
          }
      };

  // Loop through all provided cases
  for (const auto& test_pair : test_pairs)
  {
    auto msg = ds_nmea_msgs::Zda{};
    auto expected = test_pair.second;
    const auto ok = ds_nmea_msgs::from_string(msg, test_pair.first);

    // Should have succeeded
    ASSERT_TRUE(ok);

    // All fields should match.
    EXPECT_STREQ(expected.talker.data(), msg.talker.data());
    EXPECT_EQ(expected.utc_time, msg.utc_time);
    EXPECT_EQ(expected.zone_offset, msg.zone_offset);
    EXPECT_EQ(expected.checksum, msg.checksum);
  }
}

// Run all the tests that were declared with TEST()
int main(int argc, char** argv)
{
  ros::Time::init();
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

