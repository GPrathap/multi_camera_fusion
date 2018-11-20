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
#include "modules/parking/parking.h"

#include <iomanip>
#include <string>

#include "ros/include/std_msgs/String.h"

#include "modules/localization/proto/localization.pb.h"

#include "modules/common/adapters/adapter_manager.h"
#include "modules/common/log.h"
#include "modules/common/time/time.h"
#include "modules/common/vehicle_state/vehicle_state_provider.h"
#include "modules/parking/common/parking_gflags.h"

namespace apollo {
namespace parking {

using apollo::canbus::Chassis;
using apollo::common::ErrorCode;
using apollo::common::Status;
using apollo::common::VehicleStateProvider;
using apollo::common::adapter::AdapterManager;
using apollo::common::monitor::MonitorMessageItem;
using apollo::common::time::Clock;
using apollo::localization::LocalizationEstimate;

std::string Parking::Name() const { return "parking"; }

Status Parking::Init() {
  init_time_ = Clock::NowInSeconds();

  AINFO << "Parking init, starting ...";
  CHECK(common::util::GetProtoFromFile(FLAGS_parking_conf_file, &parking_conf_))
      << "Unable to load parking conf file: " + FLAGS_parking_conf_file;

  AINFO << "Conf file: " << FLAGS_parking_conf_file << " is loaded.";

  AdapterManager::Init(FLAGS_parking_adapter_config_filename);

  common::monitor::MonitorLogBuffer buffer(&monitor_logger_);

  // lock it in case for after sub, init_vehicle not ready, but msg trigger
  // come
  CHECK(AdapterManager::GetLocalization())
      << "Localization is not initialized.";

  CHECK(AdapterManager::GetChassis()) << "Chassis is not initialized.";

  CHECK(AdapterManager::GetPad()) << "Pad is not initialized.";

  CHECK(AdapterManager::GetMonitor()) << "Monitor is not initialized.";

  CHECK(AdapterManager::GetControlCommand())
      << "ControlCommand publisher is not initialized.";

  AdapterManager::AddPadCallback(&Parking::OnPad, this);
  AdapterManager::AddMonitorCallback(&Parking::OnMonitor, this);

  parking_state_.set_status(ParkingStatus::WAITING_START);

  return Status::OK();
}

Status Parking::Start() {
  // set initial vehicle state by cmd
  // need to sleep, because advertised channel is not ready immediately
  // simple test shows a short delay of 80 ms or so
  AINFO << "Parking resetting vehicle state, sleeping for 1000 ms ...";
  std::this_thread::sleep_for(std::chrono::milliseconds(1000));

  timer_ = AdapterManager::CreateTimer(
      ros::Duration(parking_conf_.control_period()), &Parking::OnTimer, this);

  AINFO << "Parking init done!";

  common::monitor::MonitorLogBuffer buffer(&monitor_logger_);
  buffer.INFO("parking started");

  return Status::OK();
}

void Parking::OnPad(const apollo::control::PadMessage &pad) {
  pad_msg_ = pad;
  ADEBUG << "Received Pad Msg:" << pad.DebugString();
  AERROR_IF(!pad_msg_.has_action()) << "pad message check failed!";

  // do something according to pad message
  if (pad_msg_.action() == control::DrivingAction::RESET) {
    AINFO << "Parking received RESET action!";
    estop_ = false;
  } else if (pad_msg_.action() == control::DrivingAction::START) {
    AINFO << "Parking received START action!";
    if (parking_state_.status() == ParkingStatus::WAITING_START)
    {
      parking_state_.set_status(ParkingStatus::SEEKING_PLACE);
      AINFO << "Set SEEKING_PLACE state";
      estop_ = false;
    }
  }
  pad_received_ = true;
}

void Parking::OnMonitor(
    const common::monitor::MonitorMessage &monitor_message) {
  for (const auto &item : monitor_message.item()) {
    if (item.log_level() == MonitorMessageItem::FATAL) {
      estop_ = true;
      return;
    }
  }
}

Status Parking::ProduceControlCommand(control::ControlCommand *control_command) {
  Status status = CheckInput();
  // check data

  if (!status.ok()) {
    AERROR_EVERY(100) << "Parking input data failed: "
                      << status.error_message();
    control_command->mutable_engage_advice()->set_advice(
        apollo::common::EngageAdvice::DISALLOW_ENGAGE);
    control_command->mutable_engage_advice()->set_reason(
        status.error_message());
    estop_ = true;
  } else {
    Status status_ts = CheckTimestamp();
    if (!status_ts.ok()) {
      AERROR << "Input messages timeout";
      // estop_ = true;
      status = status_ts;
      if (chassis_.driving_mode() !=
          apollo::canbus::Chassis::COMPLETE_AUTO_DRIVE) {
        control_command->mutable_engage_advice()->set_advice(
            apollo::common::EngageAdvice::DISALLOW_ENGAGE);
        control_command->mutable_engage_advice()->set_reason(
            status.error_message());
      }
    } else {
      control_command->mutable_engage_advice()->set_advice(
          apollo::common::EngageAdvice::READY_TO_ENGAGE);
    }
  }

  if (!estop_) {

    auto debug = control_command->mutable_debug()->mutable_input_debug();
    debug->mutable_localization_header()->CopyFrom(localization_.header());
    debug->mutable_canbus_header()->CopyFrom(chassis_.header());

    Status status_compute;

    //TODO здесь нужно добавить всю логику парковки в зависимости от текущего parking_state_
    double throttle = 0.0;
    double brake = 0.0;
    switch (parking_state_.status()){
      case ParkingStatus::WAITING_START: 
          control_command->set_speed(0);
          control_command->set_throttle(0);
          control_command->set_brake(50);
          control_command->set_steering_target(0.0);
          control_command->set_gear_location(Chassis::GEAR_DRIVE);
          AINFO_EVERY(10) << "WAITING_START";
        break;
      case ParkingStatus::SEEKING_PLACE: {
          double dx = parking_conf_.start_parking_x() - VehicleStateProvider::instance()->x();
          double dy = parking_conf_.start_parking_y() - VehicleStateProvider::instance()->y();
          double dist =sqrt(dx*dx+dy*dy); //distance to start parking point
          double speed_err = (parking_conf_.parking_speed()-chassis_.speed_mps());
          AINFO << "X: " << VehicleStateProvider::instance()->x() << " Y: " << VehicleStateProvider::instance()->y();
          AINFO << "SEEKING PLACE Distance to parking start: " << dist << " dx: "<< dx<< " dy: "<< dy << " speed err: "<< speed_err;
          if (dist >= 1.5){
            throttle = 0.05 * speed_err;
          } else if (dist < 1.5 && dist > 0.9){
            throttle = 0.02 * (speed_err - 1.5);
          } else {
            throttle = 0.0;
            brake = 0.2;
            if (chassis_.speed_mps() < 0.1)
            {
              parking_state_.set_status(ParkingStatus::WAITING_R_GEAR);
              AINFO << "Set WAITING_R_GEAR state";
            }
          }
          if (throttle<0.0){
            brake = -0.7*throttle;
            throttle = 0.0;
          }
          control_command->set_steering_target(0.0);
          control_command->set_throttle(throttle*100);
          control_command->set_brake(brake*100);
          control_command->set_gear_location(Chassis::GEAR_DRIVE); 
      }         
      break;
      case ParkingStatus::WAITING_R_GEAR:
          AINFO << "WAITING_R_GEAR";
          control_command->set_steering_target(0.0);
          control_command->set_speed(0);
          control_command->set_throttle(0);
          control_command->set_brake(30); 
          control_command->set_gear_location(Chassis::GEAR_REVERSE);
          if (VehicleStateProvider::instance()->gear() == Chassis::GEAR_REVERSE){
            parking_state_.set_status(ParkingStatus::REVERSE_TURN);
          }   
        break;
      case ParkingStatus::REVERSE_TURN: {
          double speed_err = (parking_conf_.reverse_speed()-chassis_.speed_mps());
          double yaw = localization_.mutable_pose()->heading() / M_PI * 180;
          double heading_err = fabs(parking_conf_.parking_heading() - yaw);
          AINFO << "REVERSE_TURN Heading err: " << heading_err << " Speed err: " << speed_err << " Yaw: " << yaw;   
          if (heading_err >= 1.0){
            throttle = 0.06 * speed_err;
          }else {
            throttle = 0.0;
            brake = 0.2;
            parking_state_.set_status(ParkingStatus::REVERSE_STRAIGHT);
            AINFO << "Set REVERSE_STRAIGHT";
          }
          if (throttle<0.0){
            brake = -0.7*throttle;
            throttle = 0.0;
          }
          control_command->set_steering_target(parking_conf_.parking_steering_angle());
          control_command->set_throttle(throttle*100);
          control_command->set_brake(brake*100);
          control_command->set_gear_location(Chassis::GEAR_REVERSE);  
      }
      break;
      case ParkingStatus::REVERSE_STRAIGHT: {
          double speed_err = (parking_conf_.reverse_speed()-chassis_.speed_mps());
          double dx = parking_conf_.stop_parking_x() - VehicleStateProvider::instance()->x();
          double dy = parking_conf_.stop_parking_y() - VehicleStateProvider::instance()->y();
          double dist =sqrt(dx*dx+dy*dy); //distance to stop parking point
          AINFO << "REVERSE_STRAIGHT Distance to parking start: " << dist << " dx: "<< dx<< " dy: "<< dy << " speed err: "<< speed_err;
          if (dist >= 0.7){
            throttle = 0.04 * speed_err;
          } else {
            throttle = 0.0;
            brake = 0.2;
            parking_state_.set_status(ParkingStatus::STOP);
          }
          if (throttle<0.0){
            brake = -0.7*throttle;
            throttle = 0.0;
          }
          control_command->set_throttle(throttle*100);
          control_command->set_brake(brake*100);
          control_command->set_gear_location(Chassis::GEAR_REVERSE);
      }
      break;
      case ParkingStatus::STOP:    
          control_command->set_speed(0);
          control_command->set_throttle(0);
          control_command->set_brake(50);
        break;
    }

    if (!status_compute.ok()) {
      AERROR << "Parking main function failed"
             << " with localization: " << localization_.ShortDebugString()
             << " with chassis: " << chassis_.ShortDebugString()
             << " with cmd: " << control_command->ShortDebugString()
             << " status:" << status_compute.error_message();
      estop_ = true;
      status = status_compute;
    }
  }

  if (estop_) {
    AWARN_EVERY(100) << "Estop triggered! No control core method executed!";
    // set Estop command
    control_command->set_speed(0);
    control_command->set_throttle(0);
    control_command->set_brake(30);
    control_command->set_gear_location(Chassis::GEAR_DRIVE);
  }
  return status;
}

void Parking::OnTimer(const ros::TimerEvent &) {
  double start_timestamp = Clock::NowInSeconds();


  control::ControlCommand control_command;

  Status status = ProduceControlCommand(&control_command);
  AERROR_IF(!status.ok()) << "Failed to produce control command:"
                          << status.error_message();

  double end_timestamp = Clock::NowInSeconds();

  if (pad_received_) {
    control_command.mutable_pad_msg()->CopyFrom(pad_msg_);
    pad_received_ = false;
  }

  const double time_diff_ms = (end_timestamp - start_timestamp) * 1000;
  control_command.mutable_latency_stats()->set_total_time_ms(time_diff_ms);
  control_command.mutable_latency_stats()->set_total_time_exceeded(
      time_diff_ms < parking_conf_.control_period());
  ADEBUG << "control cycle time is: " << time_diff_ms << " ms.";
  status.Save(control_command.mutable_header()->mutable_status());

  SendCmd(&control_command);
}

Status Parking::CheckInput() {
  AdapterManager::Observe();
  auto localization_adapter = AdapterManager::GetLocalization();
  if (localization_adapter->Empty()) {
    AWARN_EVERY(100) << "No Localization msg yet. ";
    return Status(ErrorCode::CONTROL_COMPUTE_ERROR, "No localization msg");
  }
  localization_ = localization_adapter->GetLatestObserved();
  ADEBUG << "Received localization:" << localization_.ShortDebugString();

  auto chassis_adapter = AdapterManager::GetChassis();
  if (chassis_adapter->Empty()) {
    AWARN_EVERY(100) << "No Chassis msg yet. ";
    return Status(ErrorCode::CONTROL_COMPUTE_ERROR, "No chassis msg");
  }
  chassis_ = chassis_adapter->GetLatestObserved();
  ADEBUG << "Received chassis:" << chassis_.ShortDebugString();

  VehicleStateProvider::instance()->Update(localization_, chassis_);

  return Status::OK();
}

Status Parking::CheckTimestamp() {
  if (!FLAGS_enable_input_timestamp_check) {
    ADEBUG << "Skip input timestamp check by gflags.";
    return Status::OK();
  }
  double current_timestamp = Clock::NowInSeconds();
  double localization_diff =
      current_timestamp - localization_.header().timestamp_sec();
  if (localization_diff >
      (FLAGS_max_localization_miss_num * parking_conf_.localization_period())) {
    AERROR << "Localization msg lost for " << std::setprecision(6)
           << localization_diff << "s";
    common::monitor::MonitorLogBuffer buffer(&monitor_logger_);
    buffer.ERROR("Localization msg lost");
    return Status(ErrorCode::CONTROL_COMPUTE_ERROR, "Localization msg timeout");
  }

  double chassis_diff = current_timestamp - chassis_.header().timestamp_sec();
  if (chassis_diff >
      (FLAGS_max_chassis_miss_num * parking_conf_.chassis_period())) {
    AERROR << "Chassis msg lost for " << std::setprecision(6) << chassis_diff
           << "s";
    common::monitor::MonitorLogBuffer buffer(&monitor_logger_);
    buffer.ERROR("Chassis msg lost");
    return Status(ErrorCode::CONTROL_COMPUTE_ERROR, "Chassis msg timeout");
  }

  return Status::OK();
}

void Parking::SendCmd(control::ControlCommand *control_command) {
  // set header
  AdapterManager::FillControlCommandHeader(Name(), control_command);

  ADEBUG << control_command->ShortDebugString();
  AdapterManager::PublishControlCommand(*control_command);
}

void Parking::Stop() {}

}  // namespace control
}  // namespace apollo
