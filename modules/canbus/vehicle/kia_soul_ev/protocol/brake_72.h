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
 * @file brake_60.h
 * @brief the class of Brake72 (for kia_soul_ev vehicle)
 */

#ifndef MODULES_CANBUS_VEHICLE_KIASOULEV_PROTOCOL_BRAKE_60_H_
#define MODULES_CANBUS_VEHICLE_KIASOULEV_PROTOCOL_BRAKE_60_H_

#include "modules/drivers/canbus/can_comm/protocol_data.h"
#include "modules/canbus/proto/chassis_detail.pb.h"
#include "modules/canbus/vehicle/kia_soul_ev/protocol/oscc_magic.h"

/**
 * @namespace apollo::canbus::kia_soul_ev
 * @brief apollo::canbus::kia_soul_ev
 */
namespace apollo {
namespace canbus {
namespace kia_soul_ev {

/**
 * @class Brake72
 *
 * @brief one of the protocol data of kia_soul_ev vehicle
 */
class Brake72 : public ::apollo::drivers::canbus::ProtocolData<
                    ::apollo::canbus::ChassisDetail> {
 public:
  static const int32_t ID;
  /**
   * @brief get the data period
   * @return the value of data period
   */
  uint32_t GetPeriod() const override;

  /**
   * @brief update the data
   * @param data a pointer to the data to be updated
   */
  void UpdateData(uint8_t *data) override;

  /**
   * @brief reset the private variables
   */
  void Reset() override;

  /**
   * @brief set pedal based on pedal command
   * @return a this pointer to the instance itself
   */
  Brake72 *set_pedal(double pcmd);

  /**
   * @brief set pedal_enable_ to true
   * @return a this pointer to the instance itself
   */
  Brake72 *set_enable();

  /**
   * @brief set pedal_enable_ to false
   * @return a this pointer to the instance itself
   */
  Brake72 *set_disable();

 private:
  
  /**
   * config detail: {'name': 'pcmd', 'offset': 0.0, 'precision':
   * 1.52590218966964e-05, 'len': 16, 'f_type': 'value', 'is_signed_var': False,
   * 'physical_range': '[0|1]', 'bit': 0, 'type': 'double', 'order': 'intel',
   * 'physical_unit': '"%"'}
   */
  void set_pedal_p(uint8_t *data, double pedal);

  /**
   * config detail: {'name': 'en', 'offset': 0.0, 'precision': 1.0, 'len': 1,
   * 'f_type': 'valid', 'is_signed_var': False, 'physical_range': '[0|0]',
   * 'bit': 24, 'type': 'bool', 'order': 'intel', 'physical_unit': '""'}
   */
  void set_enable_p(uint8_t *bytes, bool en);

 
 private:
  double pedal_cmd_ = 0.0;
  bool pedal_enable_ = false;
};

}  // namespace kia_soul_ev
}  // namespace canbus
}  // namespace apollo

#endif  // MODULES_CANBUS_VEHICLE_KIASOULEV_PROTOCOL_BRAKE_60_H_
