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

#include "modules/canbus/vehicle/kia_soul_ev/protocol/brake_73.h"

#include "glog/logging.h"

#include "modules/drivers/canbus/common/byte.h"
#include "modules/drivers/canbus/common/canbus_consts.h"

namespace apollo {
namespace canbus {
namespace kia_soul_ev {

using ::apollo::drivers::canbus::Byte;

const int32_t Brake73::ID = 0x73;

void Brake73::Parse(const std::uint8_t *bytes, int32_t length,
                    ChassisDetail *chassis_detail) const {

  double brake_cmd = pedal_cmd(bytes, length);                 
  chassis_detail->mutable_brake()->set_brake_input(0);
  chassis_detail->mutable_brake()->set_brake_cmd(brake_cmd);
  chassis_detail->mutable_brake()->set_brake_output(brake_cmd);
    chassis_detail->mutable_brake()->set_brake_enabled(is_enabled(bytes, length));
  chassis_detail->mutable_brake()->set_driver_override(
      is_driver_override(bytes, length));
  chassis_detail->mutable_brake()->set_driver_activity(true);
  chassis_detail->mutable_brake()->set_watchdog_fault(false);
  chassis_detail->mutable_brake()->set_channel_1_fault(false);
  chassis_detail->mutable_brake()->set_channel_2_fault(false);
  chassis_detail->mutable_brake()->set_boo_fault(false);
  chassis_detail->mutable_brake()->set_connector_fault(false);
  chassis_detail->mutable_check_response()->set_is_esp_online(
      !is_driver_override(bytes, length));
}

double Brake73::pedal_cmd(const std::uint8_t *bytes, int32_t length) const {

  return 0;
}

double Brake73::parse_two_frames(const std::uint8_t low_byte,
                                 const std::uint8_t high_byte) const {
  Byte frame_high(&high_byte);
  int32_t high = frame_high.get_byte(0, 8);
  Byte frame_low(&low_byte);
  int32_t low = frame_low.get_byte(0, 8);
  int32_t value = (high << 8) | low;
  // control needs a value in range [0, 100] %
  double output = 100.0 * value * 1.52590218966964e-05;
  output = ProtocolData::BoundedValue(0.0, 100.0, output);
  return output;
}

bool Brake73::is_enabled(const std::uint8_t *bytes, int32_t length) const {
  return bytes[2]>0;
}

bool Brake73::is_driver_override(const std::uint8_t *bytes,
                                 int32_t length) const {
  return bytes[3]>0;
}

}  // namespace kia_soul_ev
}  // namespace canbus
}  // namespace apollo
