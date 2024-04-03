//
// Created by jvaccaro on 6/28/19.
//

#ifndef PROJECT_SEAPATHBINARY11_H
#define PROJECT_SEAPATHBINARY11_H

#include "ds_core_msgs/RawData.h"
#include "ros/console.h"
#include "ds_sensor_msgs/Gyro.h"
namespace ds_sensor_parsers {

struct seapath_binary_11_def {
  uint8_t synch_byte;
  int32_t ins_time_seconds; //seconds
  uint8_t ins_time_hundredth; // 0.01 seconds
  int32_t latitude; // +/-(2^31) = +/- 180 deg
  int32_t longitude; // +/-(2^31) = +/- 180 deg
  int32_t altitude; // cm
  int16_t heave; //cm **WARNING** opposite of usual INS convention
  int16_t velocity_north; // cm/sec
  int16_t velocity_east; // cm/sec
  int16_t velocity_down; // cm/sec
  int16_t roll; // +/-(2^15) = +/- 180deg, + port up
  int16_t pitch; // +/-(2^15) = +/- 180deg, + bow up **WARNING** opposite of usual INS convention
  uint16_t heading; // 2^15 = 180deg
  int16_t roll_rate; //+/-180°/s
  int16_t pitch_rate; //+/-180°/s **WARNING** opposite of usual INS convention
  int16_t heading_rate; //+/-180°/s **WARNING** opposite of usual INS convention
  uint16_t status; //0x00AA invalid data, 0x0000 data valid
  uint16_t checksum; //computed up bytes 1-39
} __attribute__((packed));
typedef struct seapath_binary_11_def seapath_binary_11;

unsigned short blkcrc(unsigned char *bufptr, unsigned len) {
  unsigned char i;
  unsigned short data;
  unsigned short crc = 0xffff;
  if (len == 0)
    return ~crc;
  do {
    for (i = 0, data = (unsigned short) (0xff & *bufptr++); i
        < 8; i++, data >>= 1) {
      if ((crc & 0x0001) ^ (data & 0x0001)) {
        crc = (crc >> 1) ^ 0x8408;
      } else {
        crc >>= 1;
      }
    }
  } while (--len);
  crc = ~crc;
  data = crc;
  crc = (crc << 8) | ((data >> 8) & 0xff);
  return crc;
}
seapath_binary_11 seapath_parse_bytes(ds_core_msgs::RawData &raw) {
  auto ptr = raw.data.data();
  auto size = raw.data.size();
  auto checksum = blkcrc(ptr, size);
  if (size <= sizeof(seapath_binary_11)) {
    auto dgm = *reinterpret_cast<seapath_binary_11 *>(raw.data.data());
    ROS_INFO_STREAM("synch_byte: "<<dgm.synch_byte);
    ROS_INFO_STREAM("time_sec: "<<dgm.ins_time_seconds);
    ROS_INFO_STREAM("time_hun: "<<dgm.ins_time_hundredth);
    ROS_INFO_STREAM("lat: "<<dgm.latitude);
    ROS_INFO_STREAM("lon: "<<dgm.longitude);
    ROS_INFO_STREAM("alt: "<<dgm.altitude);
    ROS_INFO_STREAM("heave: "<<dgm.heave);
    ROS_INFO_STREAM("velo n: "<<dgm.velocity_north);
    ROS_INFO_STREAM("velo e: "<<dgm.velocity_east);
    ROS_INFO_STREAM("velo d: "<<dgm.velocity_down);
    ROS_INFO_STREAM("roll: "<<dgm.roll);
    ROS_INFO_STREAM("pitch: "<<dgm.pitch);
    ROS_INFO_STREAM("heading: "<<dgm.heading);
    ROS_INFO_STREAM("roll r: "<<dgm.roll_rate);
    ROS_INFO_STREAM("pitch r: "<<dgm.pitch_rate);
    ROS_INFO_STREAM("heading r: "<<dgm.heading_rate);
    ROS_INFO_STREAM("status: "<<dgm.status);
    ROS_INFO_STREAM("checksum: "<<dgm.checksum);
    if (dgm.checksum == checksum) {
      ROS_INFO_STREAM("Checksum read="<<dgm.checksum<<" calculated="<<checksum);
    }
    ROS_ERROR_STREAM("Checksum read="<<dgm.checksum<<" calculated="<<checksum);
    return dgm;
  } else {
    ROS_ERROR_STREAM("raw.data.size()="<<size<<" sizeof(seapath_binary_11)="<<sizeof(seapath_binary_11_def));
  }
  return {};
}

ds_sensor_msgs::Gyro raw_seapath_to_gyro(ds_core_msgs::RawData raw)
{
  ds_sensor_msgs::Gyro gyro{};
  gyro.ds_header = raw.ds_header;
  gyro.header = raw.header;
  auto msg = seapath_parse_bytes(raw);
  gyro.roll = msg.roll * (180.0 / 0xFFFF);
  gyro.pitch = msg.pitch * (180.0 / 0xFFFF);
  gyro.heading = msg.heading * (180.0 / 0x7FFF);
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


}

#endif //PROJECT_SEAPATHBINARY11_H
