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

#include "modules/canbus/vehicle/kia_soul_ev/protocol/gear_200.h"

#include "modules/drivers/canbus/common/byte.h"

namespace apollo {
namespace canbus {
namespace kia_soul_ev {

using ::apollo::drivers::canbus::Byte;

const int32_t Gear200::ID = 0x200;

void Gear200::Parse(const std::uint8_t *bytes, int32_t length,
                       ChassisDetail *chassis_detail) const {
  int32_t gear = gear_state(bytes, length);
  switch (gear) {
    case 0x00:
      chassis_detail->mutable_gear()->set_gear_state(Chassis::GEAR_PARKING);
      chassis_detail->mutable_gear()->set_gear_cmd(Chassis::GEAR_PARKING);
      ADEBUG << "Parking gear";
      break;
    case 0x38:
      chassis_detail->mutable_gear()->set_gear_state(Chassis::GEAR_REVERSE);
      chassis_detail->mutable_gear()->set_gear_cmd(Chassis::GEAR_REVERSE);
      ADEBUG << "Reverse gear";
      break;
    case 0x30:
      chassis_detail->mutable_gear()->set_gear_state(Chassis::GEAR_NEUTRAL);
      chassis_detail->mutable_gear()->set_gear_cmd(Chassis::GEAR_NEUTRAL);
      ADEBUG << "Neutral gear";
      break;
    case 0x28:
      chassis_detail->mutable_gear()->set_gear_state(Chassis::GEAR_DRIVE);
      chassis_detail->mutable_gear()->set_gear_cmd(Chassis::GEAR_DRIVE);
      ADEBUG << "Drive gear";
      break;
    case 0x08:
      chassis_detail->mutable_gear()->set_gear_state(Chassis::GEAR_LOW);
      chassis_detail->mutable_gear()->set_gear_cmd(Chassis::GEAR_LOW);
      ADEBUG << "Low gear";
      break;
    default:
      chassis_detail->mutable_gear()->set_gear_state(Chassis::GEAR_INVALID);
      chassis_detail->mutable_gear()->set_gear_cmd(Chassis::GEAR_INVALID);
      ADEBUG << "Invalid gear";
      break;
  }

  chassis_detail->mutable_gear()->set_is_shift_position_valid(true);
  chassis_detail->mutable_gear()->set_driver_override(false);
  chassis_detail->mutable_gear()->set_canbus_fault(false);
}

void Gear200::Parse(const std::uint8_t *bytes, int32_t length,
                       const struct timeval &timestamp,
                       ChassisDetail *chassis_detail) const {
  chassis_detail->mutable_eps()->set_timestamp_65(
      timestamp.tv_sec + timestamp.tv_usec / 1000000.0);
  Parse(bytes, length, chassis_detail);
}

int32_t Gear200::gear_state(const std::uint8_t *bytes, int32_t length) const {
  return bytes[1];
}

}  // namespace kia_soul_ev
}  // namespace canbus
}  // namespace apollo
