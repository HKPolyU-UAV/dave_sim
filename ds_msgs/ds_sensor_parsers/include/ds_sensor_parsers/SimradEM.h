//
// Created by jvaccaro on 6/28/19.
//

#ifndef PROJECT_SIMRADEM_H
#define PROJECT_SIMRADEM_H

#include "ds_core_msgs/RawData.h"
#include "ds_sensor_msgs/Gyro.h"
//#include <Eigen/Core>
//#include <Eigen/Geometry>

namespace ds_sensor_parsers{
struct simrad_em_def{
  uint8_t sensor_status; // 0x90 if ok, 0x9A if alignment
  uint8_t synch_byte; // 0x90
  int16_t roll; // +/- 180 deg, lsb=0.01, + port up
  int16_t pitch; // +/- 180 deg, lsb=0.01 + bow up **WARNING** Opposite sign of INS usual convention
  int16_t heave; // +/- 10m lsb=0.01m + when INS goes up
  int32_t heading; // 0-160 lsb=0.01deg
} __attribute__((packed));
typedef struct simrad_em_def simrad_em;

simrad_em simrad_parse_bytes(ds_core_msgs::RawData& raw)
{
  if (raw.data.size() <= sizeof(simrad_em)){
    return *reinterpret_cast<simrad_em*>(raw.data.data());
  }
  return {};
}

ds_sensor_msgs::Gyro raw_to_gyro(ds_core_msgs::RawData raw)
{
  ds_sensor_msgs::Gyro gyro{};
  gyro.ds_header = raw.ds_header;
  gyro.header = raw.header;
  auto em = simrad_parse_bytes(raw);
  gyro.roll = em.roll / 100.0;
  gyro.pitch = em.pitch / 100.0;
  gyro.heading = em.heading / 100.0;
  gyro.roll_covar = gyro.GYRO_NO_DATA;
  gyro.pitch_covar = gyro.GYRO_NO_DATA;
  gyro.heading_covar = gyro.GYRO_NO_DATA;
  return gyro;
//  Eigen::Quaterniond q(Eigen::AngleAxisd(-(gyro.heading) + (M_PI / 2.0), Eigen::Vector3d::UnitZ()) *
//      Eigen::AngleAxisd(gyro.pitch, Eigen::Vector3d::UnitY()) *
//      Eigen::AngleAxisd(gyro.roll, Eigen::Vector3d::UnitX()));
//  gyro.orientation.x = q.x();
//  gyro.orientation.y = q.y();
//  gyro.orientation.z = q.z();
//  gyro.orientation.w = q.w();
}

ds_sensor_msgs::Gyro simrad_to_gyro(simrad_em em)
{
  ds_sensor_msgs::Gyro gyro{};
  gyro.roll = em.roll / 100.0;
  gyro.pitch = em.pitch / 100.0;
  gyro.heading = em.heading / 100.0;
  gyro.roll_covar = gyro.GYRO_NO_DATA;
  gyro.pitch_covar = gyro.GYRO_NO_DATA;
  gyro.heading_covar = gyro.GYRO_NO_DATA;
  return gyro;
}
}// namespace



#endif //PROJECT_SIMRADEM_H
