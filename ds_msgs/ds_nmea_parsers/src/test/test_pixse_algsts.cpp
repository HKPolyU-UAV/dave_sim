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
#include "ds_nmea_parsers/PixseAlgsts.h"

#include <list>
#include <gtest/gtest.h>


/// Enum flags are coded as decimal values in the message definitions....
/// make sure they're actually the expected powers of 2
TEST(PIXSE_ALGSTS, enum_values)
{

  const auto expected_flag_order = std::vector<uint64_t>
      {
        ds_nmea_msgs::PixseAlgsts::OPERATION_MODE_NAVIGATION,
        ds_nmea_msgs::PixseAlgsts::OPERATION_MODE_ALIGNMENT,
        ds_nmea_msgs::PixseAlgsts::OPERATION_MODE_FINE_ALIGNMENT,
        ds_nmea_msgs::PixseAlgsts::OPERATION_MODE_DECK_RECKONING,

        ds_nmea_msgs::PixseAlgsts::ALTITUDE_USING_GPS,
        ds_nmea_msgs::PixseAlgsts::ALTITUDE_USING_DEPTH,
        ds_nmea_msgs::PixseAlgsts::ALTITUDE_SABLILIZED,
        ds_nmea_msgs::PixseAlgsts::ALTITUDE_HYDRO,

        ds_nmea_msgs::PixseAlgsts::LOG_USED,
        ds_nmea_msgs::PixseAlgsts::LOG_DATA_VALID,
        ds_nmea_msgs::PixseAlgsts::LOG_WAITING_FOR_DATA,
        ds_nmea_msgs::PixseAlgsts::LOG_DATA_REJECTED,

        ds_nmea_msgs::PixseAlgsts::GPS_USED,
        ds_nmea_msgs::PixseAlgsts::GPS_DATA_VALID,
        ds_nmea_msgs::PixseAlgsts::GPS_WAITING_FOR_DATA,
        ds_nmea_msgs::PixseAlgsts::GPS_DATA_REJECTED,

        ds_nmea_msgs::PixseAlgsts::USBL_USED,
        ds_nmea_msgs::PixseAlgsts::USBL_DATA_VALID,
        ds_nmea_msgs::PixseAlgsts::USBL_WAITING_FOR_DATA,
        ds_nmea_msgs::PixseAlgsts::USBL_DATA_REJECTED,

        ds_nmea_msgs::PixseAlgsts::DEPTH_USED,
        ds_nmea_msgs::PixseAlgsts::DEPTH_DATA_VALID,
        ds_nmea_msgs::PixseAlgsts::DEPTH_WAITING_FOR_DATA,
        ds_nmea_msgs::PixseAlgsts::DEPTH_DATA_REJECTED,

        ds_nmea_msgs::PixseAlgsts::LBL_USED,
        ds_nmea_msgs::PixseAlgsts::LBL_DATA_VALID,
        ds_nmea_msgs::PixseAlgsts::LBL_WAITING_FOR_DATA,
        ds_nmea_msgs::PixseAlgsts::LBL_DATA_REJECTED,

        ds_nmea_msgs::PixseAlgsts::ALTITUDE_SATURATED,
        ds_nmea_msgs::PixseAlgsts::SPEED_SATURATED,
        ds_nmea_msgs::PixseAlgsts::RESERVED_1,
        ds_nmea_msgs::PixseAlgsts::RESERVED_2,

        ds_nmea_msgs::PixseAlgsts::WATER_TRACK_USED,
        ds_nmea_msgs::PixseAlgsts::WATER_TRACK_DATA_VALID,
        ds_nmea_msgs::PixseAlgsts::WATER_TRACK_WAITING_FOR_DATA,
        ds_nmea_msgs::PixseAlgsts::WATER_TRACK_REJECTED,

        ds_nmea_msgs::PixseAlgsts::GPS2_USED,
        ds_nmea_msgs::PixseAlgsts::GPS2_DATA_VALID,
        ds_nmea_msgs::PixseAlgsts::GPS2_WAITING_FOR_DATA,
        ds_nmea_msgs::PixseAlgsts::GPS2_DATA_REJECTED,

        ds_nmea_msgs::PixseAlgsts::METROLOGY_USED,
        ds_nmea_msgs::PixseAlgsts::METROLOGY_DATA_VALID,
        ds_nmea_msgs::PixseAlgsts::METROLOGY_WAITING_FOR_DATA,
        ds_nmea_msgs::PixseAlgsts::METROLOGY_DATA_REJECTED,

        ds_nmea_msgs::PixseAlgsts::ALTITUDE_USED,
        ds_nmea_msgs::PixseAlgsts::ALTITUDE_DATA_VALID,
        ds_nmea_msgs::PixseAlgsts::ALTITUDE_WAITING_FOR_DATA,
        ds_nmea_msgs::PixseAlgsts::ALTITUDE_DATA_REJECTED,

        ds_nmea_msgs::PixseAlgsts::OPERATION_MODE_ZUP,
        ds_nmea_msgs::PixseAlgsts::ZUP_VALID,
        ds_nmea_msgs::PixseAlgsts::OPERATION_MODE_ZUP_VALID,
        ds_nmea_msgs::PixseAlgsts::ZUP_BENCH_VALID,

        ds_nmea_msgs::PixseAlgsts::STATIC_ALIGNMENT,
        ds_nmea_msgs::PixseAlgsts::GO_TO_NAV,
        ds_nmea_msgs::PixseAlgsts::RESERVED_3,
        ds_nmea_msgs::PixseAlgsts::RESERVED_4,

        ds_nmea_msgs::PixseAlgsts::EM_LOG_USED,
        ds_nmea_msgs::PixseAlgsts::EM_LOG_DATA_VALID,
        ds_nmea_msgs::PixseAlgsts::EM_LOG_WAITING_FOR_DATA,
        ds_nmea_msgs::PixseAlgsts::EM_LOG_DATA_REJECTED,

        ds_nmea_msgs::PixseAlgsts::GPS_MANUAL_USED,
        ds_nmea_msgs::PixseAlgsts::GPS_MANUAL_DATA_VALID,
        ds_nmea_msgs::PixseAlgsts::GPS_MANUAL_WAITING_FOR_DATA,
        ds_nmea_msgs::PixseAlgsts::GPS_MANUAL_DATA_REJECTED,
  };

  const auto ONE = static_cast<uint64_t>(1);

  for(uint64_t i=0; i<expected_flag_order.size(); ++i)
  {
    EXPECT_EQ(expected_flag_order.at(i), ONE << i);
  }
}

TEST(PIXSE_ALGSTS, valid_strings)
{

  auto gen = [](uint32_t lsb, uint32_t msb, uint8_t checksum) {
    auto msg = ds_nmea_msgs::PixseAlgsts{};
    msg.status = (static_cast<uint64_t>(msb) << 32) + lsb;
    msg.checksum = checksum;
    return msg;
  };

  const auto test_pairs =
      std::list<std::pair<std::string, ds_nmea_msgs::PixseAlgsts>>{
          {"$PIXSE,ALGSTS,00000041,00030000*63", gen(0x41, 0x30000, 0x63)}
      };

  // Loop through all provided cases
  for (const auto& test_pair : test_pairs)
  {
    auto msg = ds_nmea_msgs::PixseAlgsts{};
    auto expected = test_pair.second;
    const auto ok = ds_nmea_msgs::from_string(msg, test_pair.first);

    // Should have succeeded
    EXPECT_TRUE(ok);

    // All fields should match.
    EXPECT_EQ(expected.status, msg.status);
    EXPECT_EQ(expected.checksum, msg.checksum);
  }
}

// Run all the tests that were declared with TEST()
int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

