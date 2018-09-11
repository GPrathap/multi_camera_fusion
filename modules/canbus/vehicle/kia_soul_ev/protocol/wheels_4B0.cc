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

#include "modules/canbus/vehicle/kia_soul_ev/protocol/wheels_4B0.h"

#include "modules/drivers/canbus/common/byte.h"

namespace apollo {
namespace canbus {
namespace kia_soul_ev {

using ::apollo::drivers::canbus::Byte;

const int32_t Wheels4B0::ID = 0x4B0;

void Wheels4B0::Parse(const std::uint8_t *bytes, int32_t length,
                       ChassisDetail *chassis_detail) const {
  
  double fl_ws = front_left_wheel_speed(bytes, length);
  chassis_detail->mutable_vehicle_spd()->set_wheel_spd_fl(fl_ws);
  chassis_detail->mutable_vehicle_spd()->set_is_wheel_spd_fl_valid(true);
  // front right
  double fr_ws = front_right_wheel_speed(bytes, length);
  chassis_detail->mutable_vehicle_spd()->set_wheel_spd_fr(fr_ws);
  chassis_detail->mutable_vehicle_spd()->set_is_wheel_spd_fr_valid(true);

  // rear left
  double rl_ws = rear_left_wheel_speed(bytes, length);
  chassis_detail->mutable_vehicle_spd()->set_wheel_spd_rl(rl_ws);
  chassis_detail->mutable_vehicle_spd()->set_is_wheel_spd_rl_valid(true);

  // rear right
  double rr_ws = rear_right_wheel_speed(bytes, length);
  chassis_detail->mutable_vehicle_spd()->set_wheel_spd_rr(rr_ws);
  chassis_detail->mutable_vehicle_spd()->set_is_wheel_spd_rr_valid(true);

  double vel = (rl_ws+rr_ws) / 2.0;
  chassis_detail->mutable_vehicle_spd()->set_vehicle_spd(vel);
  chassis_detail->mutable_vehicle_spd()->set_is_vehicle_spd_valid(true);
}

void Wheels4B0::Parse(const std::uint8_t *bytes, int32_t length,
                       const struct timeval &timestamp,
                       ChassisDetail *chassis_detail) const {
  chassis_detail->mutable_eps()->set_timestamp_65(
      timestamp.tv_sec + timestamp.tv_usec / 1000000.0);
  Parse(bytes, length, chassis_detail);
}

double Wheels4B0::front_left_wheel_speed(const std::uint8_t *bytes,
                                            int32_t length) const {
  DCHECK_GE(length, 2);
  return parse_two_frames(bytes[0], bytes[1]);
}

double Wheels4B0::front_right_wheel_speed(const std::uint8_t *bytes,
                                             int32_t length) const {
  DCHECK_GE(length, 4);
  return parse_two_frames(bytes[2], bytes[3]);
}

double Wheels4B0::rear_left_wheel_speed(const std::uint8_t *bytes,
                                           int32_t length) const {
  DCHECK_GE(length, 6);
  return parse_two_frames(bytes[4], bytes[5]);
}

double Wheels4B0::rear_right_wheel_speed(const std::uint8_t *bytes,
                                            int32_t length) const {
  DCHECK_GE(length, 8);
  return parse_two_frames(bytes[6], bytes[7]);
}

double Wheels4B0::parse_two_frames(const std::uint8_t low_byte,
                                      const std::uint8_t high_byte) const {
  Byte high_frame(&high_byte);
  int32_t high = high_frame.get_byte(0, 8);
  Byte low_frame(&low_byte);
  int32_t low = low_frame.get_byte(0, 8);
  int32_t value = (high << 8) | low;
  AINFO << "Wheel speed: " << value * 0.020000 * 0.44704;
  return value * 0.020000 * 0.44704;
}

}  // namespace kia_soul_ev
}  // namespace canbus
}  // namespace apollo
