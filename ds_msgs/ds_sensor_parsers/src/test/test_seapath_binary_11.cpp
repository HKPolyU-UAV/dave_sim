//
// Created by jvaccaro on 6/28/19.
//
#include <gtest/gtest.h>
#include "ds_sensor_parsers/SeapathBinary11.h"
#include <list>

ds_sensor_parsers::seapath_binary_11 list_to_seapath(std::vector<double> vals){
  ds_sensor_parsers::seapath_binary_11 msg{};
  msg.synch_byte = vals[0];
  msg.ins_time_seconds = vals[1];
  msg.ins_time_hundredth = vals[2];
  msg.latitude = vals[3];
  msg.longitude = vals[4];
  msg.altitude = vals[5];
  msg.heave = vals[6];
  msg.velocity_north = vals[7];
  msg.velocity_east = vals[8];
  msg.velocity_down = vals[9];
  msg.roll = vals[10]; // NO ROLL
  msg.pitch = vals[11]; // NO PITCH
  msg.heading = vals[12]; // NO HDG
  msg.roll_rate = vals[13]; // NO ROLL_RATE
  msg.pitch_rate = vals[14];
  msg.heading_rate = vals[15];
  msg.status = vals[16];
  msg.checksum = vals[17];
  return msg;
}
ds_sensor_msgs::Gyro list_to_gyro(std::vector<double> vals){
  ds_sensor_msgs::Gyro gyro{};
  gyro.roll = vals[0];
  gyro.pitch = vals[1];
  gyro.heading = vals[2];
  gyro.roll_covar = gyro.GYRO_NO_DATA;
  gyro.pitch_covar = gyro.GYRO_NO_DATA;
  gyro.heading_covar = gyro.GYRO_NO_DATA;
  //  Eigen::Quaterniond q(Eigen::AngleAxisd(-(gyro.heading) + (M_PI / 2.0), Eigen::Vector3d::UnitZ()) *
//      Eigen::AngleAxisd(gyro.pitch, Eigen::Vector3d::UnitY()) *
//      Eigen::AngleAxisd(gyro.roll, Eigen::Vector3d::UnitX()));
//  gyro.orientation.x = q.x();
//  gyro.orientation.y = q.y();
//  gyro.orientation.z = q.z();
//  gyro.orientation.w = q.w();
  return gyro;
}

void seapath_eq(ds_sensor_parsers::seapath_binary_11 actual_in, ds_sensor_parsers::seapath_binary_11 expected_in)
{
  EXPECT_EQ(expected_in.synch_byte, actual_in.synch_byte);
  EXPECT_EQ(expected_in.ins_time_seconds, actual_in.ins_time_seconds);
  EXPECT_EQ(expected_in.ins_time_hundredth, actual_in.ins_time_hundredth);
  EXPECT_EQ(expected_in.latitude, actual_in.latitude);
  EXPECT_EQ(expected_in.longitude, actual_in.longitude);
  EXPECT_EQ(expected_in.altitude, actual_in.altitude);
  EXPECT_EQ(expected_in.heave, actual_in.heave);
  EXPECT_EQ(expected_in.velocity_north, actual_in.velocity_north);
  EXPECT_EQ(expected_in.velocity_east, actual_in.velocity_east);
  EXPECT_EQ(expected_in.velocity_down, actual_in.velocity_down);
  EXPECT_EQ(expected_in.roll, actual_in.roll);
  EXPECT_EQ(expected_in.pitch, actual_in.pitch);
  EXPECT_EQ(expected_in.heading, actual_in.heading);
  EXPECT_EQ(expected_in.roll_rate, actual_in.roll_rate);
  EXPECT_EQ(expected_in.pitch_rate, actual_in.pitch_rate);
  EXPECT_EQ(expected_in.heading_rate, actual_in.heading_rate);
  EXPECT_EQ(expected_in.status, actual_in.status);
  EXPECT_EQ(expected_in.checksum, actual_in.checksum);
}

TEST(SeapathBinary, seapath_parse)
{

  const std::list<std::pair<std::vector<uint8_t>, ds_sensor_parsers::seapath_binary_11>> test_pairs = {
      // [113, 0, 0, 4, 255, 85, 227, 42, 46, 180, 32, 55, 61, 18, 0, 0, 0, 2, 0, 0, 0, 15, 255, 255, 0, 0, 0, 102, 255, 248, 184, 85, 0, 0, 0, 0, 0, 0, 0, 0, 28, 176]
      {{113, 0, 0, 4, 255, 85, 227, 42, 46, 180, 32, 55, 61, 18, 0, 0, 0, 2, 0, 0, 0, 15, 255, 255, 0, 0, 0, 102, 255, 248, 184, 85, 0, 0, 0, 0, 0, 0, 0, 0, 28, 176},
       list_to_seapath({113, 719541759, 46, 4669, 512, -61696, 0, 26112, -1793, 21944, 0, 0, 0, 0, -20452, 0, 8192, 49390})} //,
//      {{0x90, 0x90, 56, 0, 252, 255, 0, 0, 74, 101},
//       list_to_seapath({144, 144, 56, -4, 0, 25930})}
  };
  for (const auto test_pair : test_pairs){
    auto raw_data = test_pair.first;
    auto seapath_expected = test_pair.second;
    ds_core_msgs::RawData raw{};
    raw.data.resize(raw_data.size());
    for (int i=0; i<raw_data.size(); i++)
      raw.data[i] = raw_data[i];
    auto seapath_parsed = ds_sensor_parsers::seapath_parse_bytes(raw);
//    ROS_INFO_STREAM("Seapath roll: "<<seapath_parsed.roll);
//    ROS_INFO_STREAM("Seapath pitch: "<<seapath_parsed.pitch);
//    ROS_INFO_STREAM("Seapath heading "<<seapath_parsed.heading);
//    ROS_INFO_STREAM("Seapath heave: "<<seapath_parsed.heave);
    seapath_eq(seapath_parsed, seapath_expected);
  }
};

void gyro_eq(ds_sensor_msgs::Gyro g1, ds_sensor_msgs::Gyro g2){
  EXPECT_FLOAT_EQ(g2.roll, g1.roll);
  EXPECT_FLOAT_EQ(g2.pitch, g1.pitch);
  EXPECT_FLOAT_EQ(g2.heading, g1.heading);
  EXPECT_FLOAT_EQ(g2.roll_covar, g1.roll_covar);
  EXPECT_FLOAT_EQ(g2.pitch_covar, g1.pitch_covar);
  EXPECT_FLOAT_EQ(g2.heading_covar, g1.heading_covar);
};

TEST(seapathEMTest, gyro_parse)
{

  const std::list<std::pair<std::vector<uint8_t>, ds_sensor_msgs::Gyro>> test_pairs = {
      {{113, 0, 0, 4, 255, 85, 227, 42, 46, 180, 32, 55, 61, 18, 0, 0, 0, 2, 0, 0, 0, 15, 255, 255, 0, 0, 0, 102, 255, 248, 184, 85, 0, 0, 0, 0, 0, 0, 0, 0, 28, 176},
       list_to_gyro({71.719841, -4.9246969, 120.54567})} //,
  };
  for (const auto test_pair : test_pairs){
    auto raw_data = test_pair.first;
    auto gyro_expected = test_pair.second;
    ds_core_msgs::RawData raw{};
    raw.data.resize(raw_data.size());
    for (int i=0; i<raw_data.size(); i++)
      raw.data[i] = raw_data[i];
//    ROS_ERROR_STREAM("Before gyro_parsed");
    auto seapath_parsed = ds_sensor_parsers::seapath_parse_bytes(raw);
    auto gyro_parsed = ds_sensor_parsers::raw_seapath_to_gyro(raw);
    ROS_INFO_STREAM("Roll seapath: "<<seapath_parsed.roll<<" gyro: "<<gyro_parsed.roll);
    ROS_INFO_STREAM("Pitch seapath: "<<seapath_parsed.pitch<<" gyro: "<<gyro_parsed.pitch);
    ROS_INFO_STREAM("Heading seapath: "<<seapath_parsed.heading<<" gyro: "<<gyro_parsed.heading);
//    ROS_ERROR_STREAM("After gyro_parsed");
    gyro_eq(gyro_parsed, gyro_expected);

//    EXPECT_TRUE(seapath_eq(seapath_expected, seapath_parsed));

  }
//  EXPECT_TRUE(seapath_eq);
};

TEST(SeapathBinary, valid_msgs)
{
//  const auto test_pairs
  ASSERT_TRUE(true);
}

// Run all the tests that were declared with TEST()
int main(int argc, char** argv)
{
//  ros::Time::init();
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}