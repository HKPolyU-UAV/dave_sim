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
#include "ds_nmea_parsers/PixseStatus.h"

#include <list>
#include <gtest/gtest.h>

/// Enum flags are coded as decimal values in the message definitions....
/// make sure they're actually the expected powers of 2
TEST(PIXSE_STATUS, enum_values)
{

  const auto expected_flag_order = std::vector<uint64_t>
      {

        ds_nmea_msgs::PixseStatus::SERIAL_INPUT_R_ERROR,
        ds_nmea_msgs::PixseStatus::SERIAL_INPUT_A_ERROR,
        ds_nmea_msgs::PixseStatus::SERIAL_INPUT_B_ERROR,
        ds_nmea_msgs::PixseStatus::SERIAL_INPUT_C_ERROR,

        ds_nmea_msgs::PixseStatus::SERIAL_INPUT_D_ERROR,
        ds_nmea_msgs::PixseStatus::SERIAL_INPUT_E_ERROR,
        ds_nmea_msgs::PixseStatus::RESERVED_01,
        ds_nmea_msgs::PixseStatus::RESERVED_02,

        ds_nmea_msgs::PixseStatus::SERIAL_INPUT_R_ACTIVITY,
        ds_nmea_msgs::PixseStatus::SERIAL_INPUT_A_ACTIVITY,
        ds_nmea_msgs::PixseStatus::SERIAL_INPUT_B_ACTIVITY,
        ds_nmea_msgs::PixseStatus::SERIAL_INPUT_C_ACTIVITY,

        ds_nmea_msgs::PixseStatus::SERIAL_INPUT_D_ACTIVITY,
        ds_nmea_msgs::PixseStatus::SERIAL_INPUT_E_ACTIVITY,
        ds_nmea_msgs::PixseStatus::RESERVED_03,
        ds_nmea_msgs::PixseStatus::RESERVED_04,

        ds_nmea_msgs::PixseStatus::SERIAL_OUTPUT_R_FULL,
        ds_nmea_msgs::PixseStatus::SERIAL_OUTPUT_A_FULL,
        ds_nmea_msgs::PixseStatus::SERIAL_OUTPUT_B_FULL,
        ds_nmea_msgs::PixseStatus::SERIAL_OUTPUT_C_FULL,

        ds_nmea_msgs::PixseStatus::SERIAL_OUTPUT_D_FULL,
        ds_nmea_msgs::PixseStatus::SERIAL_OUTPUT_E_FULL,
        ds_nmea_msgs::PixseStatus::RESERVED_05,
        ds_nmea_msgs::PixseStatus::RESERVED_06,

        ds_nmea_msgs::PixseStatus::RESERVED_07,
        ds_nmea_msgs::PixseStatus::RESERVED_08,
        ds_nmea_msgs::PixseStatus::ETHERNET_ACTIVITY,
        ds_nmea_msgs::PixseStatus::USER_CONTROL_BIT_A,

        ds_nmea_msgs::PixseStatus::USERECONTROL_BIT_B,
        ds_nmea_msgs::PixseStatus::USER_CONTROL_BIT_C,
        ds_nmea_msgs::PixseStatus::USER_CONTROL_BIT_D,
        ds_nmea_msgs::PixseStatus::RESERVED_09,

        ds_nmea_msgs::PixseStatus::DVL_BOTTOM_TRACK_DETECTED,
        ds_nmea_msgs::PixseStatus::DVL_WATER_TRACK_DETECTED,
        ds_nmea_msgs::PixseStatus::GPS1_DETECTED,
        ds_nmea_msgs::PixseStatus::GPS2_DETECTED,

        ds_nmea_msgs::PixseStatus::USBL_DETECTED,
        ds_nmea_msgs::PixseStatus::LBL_DETECTED,
        ds_nmea_msgs::PixseStatus::DEPTH_DETECTED,
        ds_nmea_msgs::PixseStatus::LOG_EM_DETECTED,

        ds_nmea_msgs::PixseStatus::ODOMETER_DETECTED,
        ds_nmea_msgs::PixseStatus::UTC_DETECTED,
        ds_nmea_msgs::PixseStatus::ALTITUDE_DETECTED,
        ds_nmea_msgs::PixseStatus::PPS_DETECTED,

        ds_nmea_msgs::PixseStatus::ZUP_ACTIVATED,
        ds_nmea_msgs::PixseStatus::METROLOGY_DETECTED,
        ds_nmea_msgs::PixseStatus::MANUAL_GPS_DETECTED,
        ds_nmea_msgs::PixseStatus::CTD_DETECTED,

        ds_nmea_msgs::PixseStatus::HRP_DEGRADED,
        ds_nmea_msgs::PixseStatus::HRP_NOT_VALID,
        ds_nmea_msgs::PixseStatus::RESERVED_10,
        ds_nmea_msgs::PixseStatus::RESERVED_11,

        ds_nmea_msgs::PixseStatus::RESERVED_12,
        ds_nmea_msgs::PixseStatus::RESERVED_13,
        ds_nmea_msgs::PixseStatus::RESERVED_14,
        ds_nmea_msgs::PixseStatus::RESERVED_15,

        ds_nmea_msgs::PixseStatus::RESERVED_16,
        ds_nmea_msgs::PixseStatus::RESERVED_17,
        ds_nmea_msgs::PixseStatus::RESERVED_18,
        ds_nmea_msgs::PixseStatus::MPC_OVERLOAD,

        ds_nmea_msgs::PixseStatus::FAULT_ALARM,
        ds_nmea_msgs::PixseStatus::MANUFACTURES_MODE,
        ds_nmea_msgs::PixseStatus::CONFIGURATION_SAVED,
        ds_nmea_msgs::PixseStatus::SYSTEM_RESTARTED,
      };

  const auto ONE = static_cast<uint64_t>(1);

  for(auto i=0; i<expected_flag_order.size(); ++i)
  {
    EXPECT_EQ(expected_flag_order.at(i), ONE << i);
  }
}

TEST(PIXSE_STATUS, valid_strings)
{

  auto gen = [](uint32_t lsb, uint32_t msb, uint8_t checksum) {
    auto msg = ds_nmea_msgs::PixseStatus{};
    msg.status = (static_cast<uint64_t>(msb) << 32) + lsb;
    msg.checksum = checksum;
    return msg;
  };

  const auto test_pairs =
      std::list<std::pair<std::string, ds_nmea_msgs::PixseStatus>>{
          {"$PIXSE,STATUS,08000200,00001A00*15\r\n", gen(0x8000200, 0x1A00, 0x15)}
      };

  // Loop through all provided cases
  for (const auto& test_pair : test_pairs)
  {
    auto msg = ds_nmea_msgs::PixseStatus{};
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

