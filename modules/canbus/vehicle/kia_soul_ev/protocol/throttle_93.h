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
 * @file throttle_63.h
 * @brief the class of Throttle93 (for kia_soul_ev vehicle)
 */

#ifndef MODULES_CANBUS_KIASOULEV_PROTOCOL_THROTTLE_93_H_
#define MODULES_CANBUS_KIASOULEV_PROTOCOL_THROTTLE_93_H_

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
 * @class Throttle93
 *
 * @brief one of the protocol data of kia_soul_ev vehicle
 */
class Throttle93 : public ::apollo::drivers::canbus::ProtocolData<
                    ::apollo::canbus::ChassisDetail> {
 public:
  static const int32_t ID;

  /*
   * @brief parse received data
   * @param bytes a pointer to the input bytes
   * @param length the length of the input bytes
   * @param chassis_detail the parsed chassis_detail
   */
  virtual void Parse(const std::uint8_t *bytes, int32_t length,
                     ChassisDetail *chassis_detail) const;

  /**
   * @brief calculate pedal command based on byte array.
   * config detail: {'name': 'pc', 'offset': 0.0, 'precision':
   * 1.52590218966964e-05, 'len': 16, 'f_type': 'value', 'is_signed_var': False,
   * 'physical_range': '[0|1]', 'bit': 16, 'type': 'double', 'order': 'intel',
   * 'physical_unit': '"%"'}
   * @param bytes a pointer to the byte array
   * @param length the length of the byte array
   * @return the value of pedal command
   */
  double pedal_cmd(const std::uint8_t *bytes, int32_t length) const;


};

}  // namespace kia_soul_ev
}  // namespace canbus
}  // namespace apollo

#endif  // MODULES_CANBUS_KIASOULEV_PROTOCOL_THROTTLE_93_H_
