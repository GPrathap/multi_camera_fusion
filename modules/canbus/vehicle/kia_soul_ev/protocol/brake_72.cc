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

#include "modules/canbus/vehicle/kia_soul_ev/protocol/brake_72.h"

#include "modules/drivers/canbus/common/byte.h"

namespace apollo {
namespace canbus {
namespace kia_soul_ev {

using ::apollo::drivers::canbus::Byte;

// public
const int32_t Brake72::ID = 0x72;

uint32_t Brake72::GetPeriod() const {
  static const uint32_t PERIOD = 20 * 1000;
  return PERIOD;
}

void Brake72::UpdateData(uint8_t *data) {
  set_pedal_p(data, pedal_cmd_);
}

void Brake72::Reset() {
  pedal_cmd_ = 0.0;
  pedal_enable_ = false;
}

Brake72 *Brake72::set_pedal(double pedal) {
  pedal_cmd_ = pedal;
  return this;
}

Brake72 *Brake72::set_enable() {
  pedal_enable_ = true;
  return this;
}

Brake72 *Brake72::set_disable() {
  pedal_enable_ = false;
  return this;
}

void Brake72::set_pedal_p(uint8_t *data, double pedal) {

    float pedal_command = (float) pedal;

    ADEBUG << "Brake cmd: " << pedal_command;

    data[0] = (uint8_t) OSCC_MAGIC_BYTE_0;
    data[1] = (uint8_t) OSCC_MAGIC_BYTE_1;

    if (pedal_command>=0 && pedal_command<=1){
      memcpy(data+2, &pedal_command , sizeof(pedal_command));
    } else{
      data[2] = (uint8_t) 0;
      data[3] = (uint8_t) 0;
      data[4] = (uint8_t) 0;
      data[5] = (uint8_t) 0;
    }

    data[6] = (uint8_t) 0;
    data[7] = (uint8_t) 0;

}


void Brake72::set_enable_p(uint8_t *bytes, bool enable) {

}


}  // namespace kia_soul_ev
}  // namespace canbus
}  // namespace apollo
