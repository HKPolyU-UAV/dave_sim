//
// Created by jvaccaro on 7/1/19.
//

#include "ds_sensor_parsers/SeapathBinary11.h"

namespace ds_sensor_parsers {

//seapath_binary_11 parse_bytes(ds_core_msgs::RawData &raw) {
//  auto ptr = raw.data.data();
//  auto size = raw.data.size();
//  auto checksum = blkcrc(ptr, size);
//  if (size <= sizeof(seapath_binary_11)) {
//    auto dgm = *reinterpret_cast<seapath_binary_11 *>(raw.data.data());
//    if (dgm.checksum == checksum) {
//      return dgm;
//    }
//  }
//  return {};
//}
//
//unsigned short blkcrc(unsigned char *bufptr, unsigned len) {
//  unsigned char i;
//  unsigned short data;
//  unsigned short crc = 0xffff;
//  if (len == 0)
//    return ~crc;
//  do {
//    for (i = 0, data = (unsigned short) (0xff & *bufptr++); i
//        < 8; i++, data >>= 1) {
//      if ((crc & 0x0001) ^ (data & 0x0001)) {
//        crc = (crc >> 1) ^ 0x8408;
//      } else {
//        crc >>= 1;
//      }
//    }
//  } while (--len);
//  crc = ~crc;
//  data = crc;
//  crc = (crc << 8) | ((data >> 8) & 0xff);
//  return crc;
//}
}