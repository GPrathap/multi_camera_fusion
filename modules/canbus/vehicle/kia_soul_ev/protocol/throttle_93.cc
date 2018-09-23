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

#include "modules/canbus/vehicle/kia_soul_ev/protocol/throttle_93.h"

#include "modules/drivers/canbus/common/byte.h"

namespace apollo {
namespace canbus {
namespace kia_soul_ev {

using ::apollo::drivers::canbus::Byte;

const int32_t Throttle93::ID = 0x93;

void Throttle93::Parse(const std::uint8_t *bytes, int32_t length,
                       ChassisDetail *chassis_detail) const {

  double throttle_cmd = pedal_cmd(bytes, length);                         

  chassis_detail->mutable_gas()->set_throttle_input(0.0);
  chassis_detail->mutable_gas()->set_throttle_cmd(throttle_cmd);
  chassis_detail->mutable_gas()->set_throttle_output(throttle_cmd);
  chassis_detail->mutable_gas()->set_watchdog_source(0);
  chassis_detail->mutable_gas()->set_throttle_enabled(true);
  chassis_detail->mutable_gas()->set_driver_override(false);
  chassis_detail->mutable_gas()->set_driver_activity(false);
  chassis_detail->mutable_gas()->set_watchdog_fault(false);
  chassis_detail->mutable_gas()->set_channel_1_fault(false);
  chassis_detail->mutable_gas()->set_channel_2_fault(false);
  chassis_detail->mutable_gas()->set_connector_fault(false);
  chassis_detail->mutable_check_response()->set_is_vcu_online(true);

}


double Throttle93::pedal_cmd(const std::uint8_t *bytes, int32_t length) const {
  //нужно добавить в ардуино отсылку текущей позицию педали
}

}  // namespace kia_soul_ev
}  // namespace canbus
}  // namespace apollo
