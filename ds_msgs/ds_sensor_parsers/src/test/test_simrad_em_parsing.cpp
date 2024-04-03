//
// Created by jvaccaro on 6/28/19.
//

#include <gtest/gtest.h>
#include <ds_sensor_parsers/SimradEM.h>
#include "ros/console.h"

ds_sensor_parsers::simrad_em list_to_simrad(std::vector<float> vals){
  ds_sensor_parsers::simrad_em em{};
  em.sensor_status = vals[0];
  em.synch_byte = vals[1];
  em.roll = vals[2];
  em.pitch = vals[3];
  em.heave = vals[4];
  em.heading = vals[5];
  return em;
}

ds_sensor_msgs::Gyro list_to_gyro(std::vector<float> vals){
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

void simrad_eq(ds_sensor_parsers::simrad_em em1, ds_sensor_parsers::simrad_em em2){
  EXPECT_EQ(em2.sensor_status, em1.sensor_status);
  EXPECT_EQ(em2.synch_byte, em1.synch_byte);
  EXPECT_EQ(em2.roll, em1.roll);
  EXPECT_EQ(em2.pitch, em1.pitch);
  EXPECT_EQ(em2.heave, em1.heave);
  EXPECT_EQ(em2.heading, em1.heading);
}

void gyro_eq(ds_sensor_msgs::Gyro g1, ds_sensor_msgs::Gyro g2){
  EXPECT_FLOAT_EQ(g2.roll, g1.roll);
  EXPECT_FLOAT_EQ(g2.pitch, g1.pitch);
  EXPECT_FLOAT_EQ(g2.heading, g1.heading);
  EXPECT_FLOAT_EQ(g2.roll_covar, g1.roll_covar);
  EXPECT_FLOAT_EQ(g2.pitch_covar, g1.pitch_covar);
  EXPECT_FLOAT_EQ(g2.heading_covar, g1.heading_covar);
};


TEST(SimradEMTest, em_parse)
{

  const std::list<std::pair<std::vector<uint8_t>, ds_sensor_parsers::simrad_em>> test_pairs = {
      {{0x90, 0x90, 0x00, 0x01, 0x00, 0xFF, 0x01, 0x00, 0x00, 0x05},
      list_to_simrad({144, 144, 256, -256, 1, 1280})} ,
      {{0x90, 0x90, 56, 0, 252, 255, 0, 0, 74, 101},
      list_to_simrad({144, 144, 56, -4, 0, 25930})}
  };
  for (const auto test_pair : test_pairs){
    auto raw_data = test_pair.first;
    auto simrad_expected = test_pair.second;
    ds_core_msgs::RawData raw{};
    raw.data.resize(raw_data.size());
    for (int i=0; i<raw_data.size(); i++)
      raw.data[i] = raw_data[i];
    auto simrad_parsed = ds_sensor_parsers::simrad_parse_bytes(raw);
    simrad_eq(simrad_expected, simrad_parsed);
  }
};

TEST(SimradEMTest, gyro_parse)
{

  const std::list<std::pair<std::vector<uint8_t>, ds_sensor_msgs::Gyro>> test_pairs = {
      {{0x90, 0x90, 0x00, 0x01, 0x00, 0xFF, 0x01, 0x00, 0x00, 0x05},
       list_to_gyro({2.56, -2.56, 12.8})},
      {{0x90, 0x90, 56, 0, 252, 255, 0, 0, 74, 101},
       list_to_gyro({0.56, -0.04, 259.30})}
  }; // 0.564, -0.046, 259.12 MATCHES PIXSE PHINS STD
  for (const auto test_pair : test_pairs){
    auto raw_data = test_pair.first;
    auto gyro_expected = test_pair.second;
    ds_core_msgs::RawData raw{};
    raw.data.resize(raw_data.size());
    for (int i=0; i<raw_data.size(); i++)
      raw.data[i] = raw_data[i];
//    ROS_ERROR_STREAM("Before gyro_parsed");
    auto simrad_parsed = ds_sensor_parsers::simrad_parse_bytes(raw);
    auto gyro_parsed = ds_sensor_parsers::raw_to_gyro(raw);
    ROS_INFO_STREAM("Roll simrad: "<<simrad_parsed.roll<<" gyro: "<<gyro_parsed.roll);
    ROS_INFO_STREAM("Pitch simrad: "<<simrad_parsed.pitch<<" gyro: "<<gyro_parsed.pitch);
    ROS_INFO_STREAM("Heading simrad: "<<simrad_parsed.heading<<" gyro: "<<gyro_parsed.heading);
//    ROS_ERROR_STREAM("After gyro_parsed");
    gyro_eq(gyro_parsed, gyro_expected);

//    EXPECT_TRUE(simrad_eq(simrad_expected, simrad_parsed));

  }
//  EXPECT_TRUE(simrad_eq);
};

// Run all the tests that were declared with TEST()
int main(int argc, char** argv)
{
//  ros::Time::init();
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}