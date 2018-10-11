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

#include "modules/canbus/vehicle/kia_soul_ev/protocol/steering_83.h"

#include "modules/drivers/canbus/common/byte.h"

namespace apollo {
namespace canbus {
namespace kia_soul_ev {

using ::apollo::drivers::canbus::Byte;

const int32_t Steering83::ID = 0x83;

void Steering83::Parse(const std::uint8_t *bytes, int32_t length,
                       ChassisDetail *chassis_detail) const {
  chassis_detail->mutable_eps()->set_steering_enabled(
      is_enabled(bytes, length));
  chassis_detail->mutable_eps()->set_driver_override(
      is_driver_override(bytes, length));
  chassis_detail->mutable_check_response()->set_is_eps_online(
      !is_driver_override(bytes, length));
}

void Steering83::Parse(const std::uint8_t *bytes, int32_t length,
                       const struct timeval &timestamp,
                       ChassisDetail *chassis_detail) const {
  chassis_detail->mutable_eps()->set_timestamp_65(
      timestamp.tv_sec + timestamp.tv_usec / 1000000.0);
  Parse(bytes, length, chassis_detail);
}

double Steering83::steering_angle(const std::uint8_t *bytes,
                                  int32_t length) const {
  //в этом сообщении угла нет
  return 0;
}


bool Steering83::is_enabled(const std::uint8_t *bytes, int32_t length) const {
  return bytes[2]>0;
}

bool Steering83::is_driver_override(const std::uint8_t *bytes,
                                    int32_t length) const {
  return bytes[3]>0;
}


}  // namespace kia_soul_ev
}  // namespace canbus
}  // namespace apollo
