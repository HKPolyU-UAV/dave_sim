//
// Created by jvaccaro on 7/1/19.
//

#include "ds_sensor_parsers/SimradEM.h"

namespace ds_sensor_parsers{

//simrad_em parse_bytes(ds_core_msgs::RawData& raw)
//{
//  if (raw.data.size() <= sizeof(simrad_em)){
//    return *reinterpret_cast<simrad_em*>(raw.data.data());
//  }
//  return {};
//}
//
//ds_sensor_msgs::Gyro raw_to_gyro(ds_core_msgs::RawData raw)
//{
//  ds_sensor_msgs::Gyro gyro{};
//  gyro.ds_header = raw.ds_header;
//  gyro.header = raw.header;
//  auto em = parse_bytes(raw);
//  gyro.roll = em.roll / 100;
//  gyro.pitch = em.pitch / 100;
//  gyro.heading = em.heading / 100;
//  gyro.roll_covar = gyro.GYRO_NO_DATA;
//  gyro.pitch_covar = gyro.GYRO_NO_DATA;
//  gyro.heading_covar = gyro.GYRO_NO_DATA;
////  Eigen::Quaterniond q(Eigen::AngleAxisd(-(gyro.heading) + (M_PI / 2.0), Eigen::Vector3d::UnitZ()) *
////      Eigen::AngleAxisd(gyro.pitch, Eigen::Vector3d::UnitY()) *
////      Eigen::AngleAxisd(gyro.roll, Eigen::Vector3d::UnitX()));
////  gyro.orientation.x = q.x();
////  gyro.orientation.y = q.y();
////  gyro.orientation.z = q.z();
////  gyro.orientation.w = q.w();
//}
}// namespace