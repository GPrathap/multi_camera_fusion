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
 * @file kia_soul_ev_controller.h
 * @brief The class of KiaSoulEvController
 */

#ifndef MODULES_CANBUS_VEHICLE_KIASOULEV_KIASOULEV_CONTROLLER_H_
#define MODULES_CANBUS_VEHICLE_KIASOULEV_KIASOULEV_CONTROLLER_H_

#include <memory>
#include <thread>

#include "gtest/gtest_prod.h"

#include "modules/canbus/vehicle/vehicle_controller.h"

#include "modules/canbus/proto/canbus_conf.pb.h"
#include "modules/canbus/proto/chassis.pb.h"
#include "modules/canbus/proto/chassis_detail.pb.h"
#include "modules/canbus/proto/vehicle_parameter.pb.h"
#include "modules/canbus/vehicle/kia_soul_ev/protocol/brake_72.h"
#include "modules/canbus/vehicle/kia_soul_ev/protocol/steering_84.h"
#include "modules/canbus/vehicle/kia_soul_ev/protocol/throttle_92.h"
#include "modules/common/macro.h"
#include "modules/common/proto/error_code.pb.h"
#include "modules/control/proto/control_cmd.pb.h"

/**
 * @namespace apollo::canbus::kia_soul_ev
 * @brief apollo::canbus::kia_soul_ev
 */
namespace apollo {
namespace canbus {
namespace kia_soul_ev {

/**
 * @class KiaSoulEvController
 *
 * @brief this class implements the vehicle controller for kia_soul_ev vehicle.
 */
class KiaSoulEvController final : public VehicleController {
 public:
  /**
   * @brief initialize the kia_soul_ev vehicle controller.
   * @return init error_code
   */
  common::ErrorCode Init(
      const VehicleParameter &params,
      CanSender<::apollo::canbus::ChassisDetail> *const can_sender,
      MessageManager<::apollo::canbus::ChassisDetail> *const message_manager)
      override;

  /**
   * @brief start the vehicle controller.
   * @return true if successfully started.
   */
  bool Start() override;

  /**
   * @brief stop the vehicle controller.
   */
  void Stop() override;

  /**
   * @brief calculate and return the chassis.
   * @returns a copy of chassis. Use copy here to avoid multi-thread issues.
   */
  Chassis chassis() override;

  FRIEND_TEST(KiaSoulEvControllerTest, SetDrivingMode);
  FRIEND_TEST(KiaSoulEvControllerTest, Status);
  FRIEND_TEST(KiaSoulEvControllerTest, UpdateDrivingMode);

 private:
  // main logical function for operation the car enter or exit the auto driving
  void Emergency() override;
  common::ErrorCode EnableAutoMode() override;
  common::ErrorCode DisableAutoMode() override;
  common::ErrorCode EnableSteeringOnlyMode() override;
  common::ErrorCode EnableSpeedOnlyMode() override;

  // NEUTRAL, REVERSE, DRIVE
  void Gear(Chassis::GearPosition state) override;

  // brake with new acceleration
  // acceleration:0.00~99.99, unit:%
  // acceleration_spd: 60 ~ 100, suggest: 90
  void Brake(double acceleration) override;

  // drive with old acceleration
  // gas:0.00~99.99 unit:%
  void Throttle(double throttle) override;

  // steering with old angle speed
  // angle:-99.99~0.00~99.99, unit:%, left:+, right:-
  void Steer(double angle) override;

  // steering with new angle speed
  // angle:-99.99~0.00~99.99, unit:%, left:+, right:-
  // angle_spd:0.00~99.99, unit:deg/s
  void Steer(double angle, double angle_spd) override;

  // set Electrical Park Brake
  void SetEpbBreak(const control::ControlCommand &command) override;
  void SetBeam(const control::ControlCommand &command) override;
  void SetHorn(const control::ControlCommand &command) override;
  void SetTurningSignal(const control::ControlCommand &command) override;

  void ResetProtocol();
  bool CheckChassisError();
  bool CheckSafetyError(const canbus::ChassisDetail &chassis);

 private:
  void SecurityDogThreadFunc();
  virtual bool CheckResponse(const int32_t flags, bool need_wait);
  void set_chassis_error_mask(const int32_t mask);
  int32_t chassis_error_mask();
  Chassis::ErrorCode chassis_error_code();
  void set_chassis_error_code(const Chassis::ErrorCode &error_code);

 private:
  // control protocol
  Brake72 *brake_72_ = nullptr;
  Throttle92 *throttle_92_ = nullptr;
  Steering84 *steering_84_ = nullptr;

  Chassis chassis_;
  std::unique_ptr<std::thread> thread_;
  bool is_chassis_error_ = false;

  std::mutex chassis_error_code_mutex_;
  Chassis::ErrorCode chassis_error_code_ = Chassis::NO_ERROR;

  std::mutex chassis_mask_mutex_;
  int32_t chassis_error_mask_ = 0;

  bool received_vin_ = false;
};

}  // namespace kia_soul_ev
}  // namespace canbus
}  // namespace apollo

#endif  // MODULES_CANBUS_VEHICLE_KIASOULEV_KIASOULEV_CONTROLLER_H_
