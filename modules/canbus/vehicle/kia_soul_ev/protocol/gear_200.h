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
 * @file steering_83.h
 * @brief the class of steering_83.h (for kia_soul_ev vehicle)
 */

#ifndef MODULES_CANBUS_VEHICL_KIASOULEV_PROTOCOL_GEAR_2B0_H_
#define MODULES_CANBUS_VEHICL_KIASOULEV_PROTOCOL_GEAR_2B0_H_

#include <sys/time.h>

#include "modules/canbus/proto/chassis_detail.pb.h"
#include "modules/drivers/canbus/can_comm/protocol_data.h"

/**
 * @namespace apollo::canbus::kia_soul_ev
 * @brief apollo::canbus::kia_soul_ev
 */
namespace apollo {
namespace canbus {
namespace kia_soul_ev {

/**
 * @class Gear200
 *
 * @brief one of the protocol data of kia_soul_ev vehicle
 */
class Gear200 : public ::apollo::drivers::canbus::ProtocolData<
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

  /*
   * @brief parse received data
   * @param bytes a pointer to the input bytes
   * @param length the length of the input bytes
   * @param timestamp the timestamp of input data
   * @param chassis_detail the parsed chassis_detail
   */
  virtual void Parse(const std::uint8_t *bytes, int32_t length,
                     const struct timeval &timestamp,
                     ChassisDetail *chassis_detail) const;

private:

  int32_t gear_state(const std::uint8_t *bytes, int32_t length) const;

};

}  // namespace kia_soul_ev
}  // namespace canbus
}  // namespace apollo

#endif  // MODULES_CANBUS_VEHICL_KIASOULEV_PROTOCOL_GEAR_65_H_
