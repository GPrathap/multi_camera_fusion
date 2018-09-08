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

#include "modules/canbus/vehicle/kia_soul_ev/protocol/steering_enable.h"

#include "modules/drivers/canbus/common/byte.h"

namespace apollo {
namespace canbus {
namespace kia_soul_ev {

using ::apollo::drivers::canbus::Byte;

const int32_t SteeringEnable::ID = 0x80;

uint32_t SteeringEnable::GetPeriod() const {
  // receive rate??
  // receive timeout would trigger fault, letting en=0 and etc.
  static const uint32_t PERIOD = 20 * 1000;
  return PERIOD;
}

bool SteeringEnable::NeedSend() {
  bool tmp = send_once_;
  send_once_ = false;
  return tmp;
}

void SteeringEnable::UpdateData(uint8_t *data) {

    oscc_steering_enable_s enable_cmd;
    enable_cmd.magic[0] = (uint8_t) OSCC_MAGIC_BYTE_0;
    enable_cmd.magic[1] = (uint8_t) OSCC_MAGIC_BYTE_1;

    memcpy(data, (void *) &enable_cmd, sizeof(enable_cmd));

}

void SteeringEnable::send_once(){
  AINFO << "Set send_once in true (enable)";
  send_once_ = true;
}


}  // namespace kia_soul_ev
}  // namespace canbus
}  // namespace apollo
