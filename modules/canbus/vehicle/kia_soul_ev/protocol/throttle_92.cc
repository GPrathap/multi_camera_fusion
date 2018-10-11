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

#include "modules/canbus/vehicle/kia_soul_ev/protocol/throttle_92.h"

#include "modules/drivers/canbus/common/byte.h"

namespace apollo {
namespace canbus {
namespace kia_soul_ev {

using ::apollo::drivers::canbus::Byte;

// public

const int32_t Throttle92::ID = 0x92;

uint32_t Throttle92::GetPeriod() const {
  static const uint32_t PERIOD = 20 * 1000;
  return PERIOD;
}

void Throttle92::UpdateData(uint8_t *data) {
  set_pedal_p(data, pedal_cmd_);
}

void Throttle92::Reset() {
  pedal_cmd_ = 0.0;
  pedal_enable_ = false;
}

Throttle92 *Throttle92::set_pedal(double pedal) {
  pedal_cmd_ = pedal;
  return this;
}

Throttle92 *Throttle92::set_enable() {
  pedal_enable_ = true;
  return this;
}

Throttle92 *Throttle92::set_disable() {
  pedal_enable_ = false;
  return this;
}

// private

void Throttle92::set_pedal_p(uint8_t *data, double pedal) {

    float torque_request = (float) pedal;

    ADEBUG << "Throttle cmd: " << torque_request;

    data[0] = (uint8_t) OSCC_MAGIC_BYTE_0;
    data[1] = (uint8_t) OSCC_MAGIC_BYTE_1;

    if (torque_request>=0 && torque_request<=1){
      memcpy(data+2, &torque_request , sizeof(torque_request ));
    } else{
      data[2] = (uint8_t) 0;
      data[3] = (uint8_t) 0;
      data[4] = (uint8_t) 0;
      data[5] = (uint8_t) 0;
    }
    

    data[6] = (uint8_t) 0;
    data[7] = (uint8_t) 0;

}


}  // namespace kia_soul_ev
}  // namespace canbus
}  // namespace apollo
