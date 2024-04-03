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
// Created by jvaccaro on 9/2/19.
//

#include "ds_nmea_parsers/Inf.h"
#include <list>
#include <gtest/gtest.h>

using namespace ds_nmea_msgs;

TEST(PHINF, valid_strings)
{
  auto gen = [](
      bool heading_invalid, bool pitch_invalid, bool roll_invalid, uint8_t checksum)
  {
    auto msg = ds_nmea_msgs::Inf{};
    msg.heading_invalid = heading_invalid;
    msg.pitch_invalid = pitch_invalid;
    msg.roll_invalid = roll_invalid;
    msg.checksum = checksum;
    return msg;
  };

  const auto test_pairs =
      std::list<std::pair<std::string, ds_nmea_msgs::Inf>>{
          {
              "$PHINF,0B000007*00\r\n",
              gen(true, true, true, 0x00)
          }, {
              "$PHINF,0B000009*00\r\n",
              gen(true, false, false, 0x00)
          }
      };
  for (const auto& test_pair : test_pairs){
    auto msg = ds_nmea_msgs::Inf{};
    auto expected = test_pair.second;
    const auto ok = ds_nmea_msgs::from_string(msg, test_pair.first);
    ASSERT_TRUE(ok);
    EXPECT_EQ(msg.heading_invalid, expected.heading_invalid);
    EXPECT_EQ(msg.pitch_invalid, expected.pitch_invalid);
    EXPECT_EQ(msg.roll_invalid, expected.roll_invalid);
    EXPECT_EQ(msg.checksum, expected.checksum);
  }
}

// Run all the tests that were declared with TEST()
int main(int argc, char** argv)
{
  ros::Time::init();
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}