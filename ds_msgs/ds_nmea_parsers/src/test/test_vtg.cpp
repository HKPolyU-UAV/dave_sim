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
#include "ds_nmea_parsers/Vtg.h"
#include "ds_nmea_parsers/util.h"

#include <list>

#include <gtest/gtest.h>

using namespace ds_nmea_msgs;

TEST(VTG, valid_strings)
{

  auto gen = [](
      std::string talker, double degrees_true, double degrees_mag, double speed_knots, double speed_kph, uint8_t mode, uint8_t checksum)
  {
    auto msg = ds_nmea_msgs::Vtg{};
    msg.talker = talker;
    msg.track_degrees_true = degrees_true;
    msg.track_degrees_magnetic = degrees_mag;
    msg.speed_knots = speed_knots;
    msg.speed_km_per_hour = speed_kph;
    msg.mode = mode;
    msg.checksum = checksum;
    return msg;
  };

  const auto test_pairs =
      std::list<std::pair<std::string, ds_nmea_msgs::Vtg>>{
          {
              "$GNVTG,,T,,M,,N,,K,N*32\r\n",
              gen("GN", Vtg::VTG_NO_DATA, Vtg::VTG_NO_DATA, Vtg::VTG_NO_DATA, Vtg::VTG_NO_DATA, 'N', 0x32)
          },
          {
              "$GNVTG,285.09,T,,M,0.84,N,1.6,K,A*1E\r\n",
              gen("GN", 285.09, Vtg::VTG_NO_DATA, 0.84, 1.6, 'A', 0x1E)
          },
          {
              "$GNVTG,176.77,T,,M,0.00,N,0.0,K,A*13\r\n",
              gen("GN", 176.77, Vtg::VTG_NO_DATA, 0.00, 0.0, 'A', 0x13)
          }
      };

  // Loop through all provided cases
  for (const auto& test_pair : test_pairs)
  {
    auto msg = ds_nmea_msgs::Vtg{};
    auto expected = test_pair.second;
    const auto ok = ds_nmea_msgs::from_string(msg, test_pair.first);

    // Should have succeeded
    ASSERT_TRUE(ok);

    // All fields should match.
    EXPECT_STREQ(expected.talker.data(), msg.talker.data());
    EXPECT_FLOAT_EQ(expected.track_degrees_true, msg.track_degrees_true);
    EXPECT_FLOAT_EQ(expected.track_degrees_magnetic, msg.track_degrees_magnetic);
    EXPECT_FLOAT_EQ(expected.speed_knots, msg.speed_knots);
    EXPECT_FLOAT_EQ(expected.speed_km_per_hour, msg.speed_km_per_hour);
    EXPECT_STREQ(expected.mode.data(), msg.mode.data());
    EXPECT_FLOAT_EQ(expected.checksum, msg.checksum);
  }
}

TEST(VTG, InvalidStrings){
// This is a failed string observed from the xeos
auto test_strs = std::list<std::pair<std::string, bool> > {
    {"$GNVTG,0,2.77T,,,M,.040,N0\n", false},
    {".7,KAG*10\r\n", false},
    {"$GNVTG,162.87,T,,M,0.84,N,16G,K,A112\r\n", false}
};
for (const auto& test_pair : test_strs) {
auto msg = ds_nmea_msgs::Vtg{};
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

