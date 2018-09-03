/******************************************************************************
 * Copyright 2017 The Apollo Authors. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *****************************************************************************/

/**
 * @file brake_61.h
 * @brief the class of Brake73 (for kia_soul_ev vehicle)
 */

#ifndef MODULES_CANBUS_VEHICLE_KIASOULEV_PROTOCOL_BRAKE_73_H_
#define MODULES_CANBUS_VEHICLE_KIASOULEV_PROTOCOL_BRAKE_73_H_

#include "modules/drivers/canbus/can_comm/protocol_data.h"
#include "modules/canbus/proto/chassis_detail.pb.h"

/**
 * @namespace apollo::canbus::kia_soul_ev
 * @brief apollo::canbus::kia_soul_ev
 */
namespace apollo {
namespace canbus {
namespace kia_soul_ev {

/**
 * @class Brake73
 *
 * @brief one of the protocol data of kia_soul_ev vehicle
 */
class Brake73 : public ::apollo::drivers::canbus::ProtocolData<
                    ::apollo::canbus::ChassisDetail> {
 public:
  static const int32_t ID;

  /*
   * @brief parse received data
   * @param bytes a pointer to the input bytes
   * @param length the length of the input bytes
   * @param timestamp the timestamp of input data
   * @param chassis_detail the parsed chassis_detail
   */
  virtual void Parse(const std::uint8_t *bytes, int32_t length,
                     ChassisDetail *chassis_detail) const;

 private:

  /**
   * @brief get pedal command for control from byte array
   * config detail: {'name': 'pc', 'offset': 0.0, 'precision':
   * 1.52590218966964e-05, 'len': 16, 'f_type': 'value', 'is_signed_var': False,
   * 'physical_range': '[0|1]', 'bit': 16, 'type': 'double', 'order': 'intel',
   * 'physical_unit': '"%"'}
   * @param bytes a pointer to the byte array
   * @param length the length of the byte array
   * @return the value of pedal command for control
   */
  double pedal_cmd(const std::uint8_t *bytes, int32_t length) const;


  double parse_two_frames(const std::uint8_t low_byte,
                          const std::uint8_t high_byte) const;


  /**
   * @brief check if enabled
   * config detail: {'name': 'en', 'offset': 0.0, 'precision': 1.0, 'len': 1,
   * 'f_type': 'valid', 'is_signed_var': False, 'physical_range': '[0|0]',
   * 'bit': 56, 'type': 'bool', 'order': 'intel', 'physical_unit': '""'}
   * @param bytes a pointer to the byte array
   * @param length the length of the byte array
   * @return true when enabled
   */
  bool is_enabled(const std::uint8_t *bytes, int32_t length) const;

  /**
   * brief check driver override
   * config detail: {'name': 'override', 'offset': 0.0, 'precision': 1.0, 'len':
   * 1, 'f_type': 'valid', 'is_signed_var': False, 'physical_range': '[0|0]',
   * 'bit': 57, 'type': 'bool', 'order': 'intel', 'physical_unit': '""'}
   * @param bytes a pointer to the byte array
   * @param length the length of the byte array
   * @return true if driver override
   */
  bool is_driver_override(const std::uint8_t *bytes, int32_t length) const;

};

}  // namespace kia_soul_ev
}  // namespace canbus
}  // namespace apollo

#endif  // MODULES_CANBUS_KIASOULEV_PROTOCOL_BRAKE_61_H_
