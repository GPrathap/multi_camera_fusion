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

#include "modules/canbus/vehicle/kia_soul_ev/protocol/steering_84.h"

#include "modules/drivers/canbus/common/byte.h"

namespace apollo {
namespace canbus {
namespace kia_soul_ev {

using ::apollo::drivers::canbus::Byte;

const int32_t Steering84::ID = 0x84;

uint32_t Steering84::GetPeriod() const {
  // receive rate??
  // receive timeout would trigger fault, letting en=0 and etc.
  static const uint32_t PERIOD = 20 * 1000;
  return PERIOD;
}

void Steering84::UpdateData(uint8_t *data) {
  AINFO_EVERY(100) << "UpdateData Steering84 steering angle: " << steering_angle_;
  set_steering_angle_p(data, steering_angle_);
}

void Steering84::Reset() {
  steering_angle_ = 0.0;
}

Steering84 *Steering84::set_steering_angle(double angle) {
  steering_angle_ = -angle; //Invert steering angle because KIA steerwheel has plus to right
  return this;
}

// private

// positive for left, negative for right
void Steering84::set_steering_angle_p(uint8_t *data, double angle) {

    float steering_angle = (float) angle;

    data[0] = (uint8_t) OSCC_MAGIC_BYTE_0;
    data[1] = (uint8_t) OSCC_MAGIC_BYTE_1;

    memcpy(data+2, &steering_angle, sizeof(steering_angle));

    data[6] = (uint8_t) 0;
    data[7] = (uint8_t) 0;

}


}  // namespace kia_soul_ev
}  // namespace canbus
}  // namespace apollo
