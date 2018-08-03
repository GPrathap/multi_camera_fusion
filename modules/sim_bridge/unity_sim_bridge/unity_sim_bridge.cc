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

#include "modules/sim_bridge/unity_sim_bridge/unity_sim_bridge.h"

#include "modules/common/configs/vehicle_config_helper.h"
#include "modules/common/adapters/adapter_manager.h"
#include "modules/common/math/quaternion.h"
#include "modules/common/time/time.h"
#include "modules/sim_bridge/common/sim_bridge_gflags.h"

namespace apollo {
namespace sim_bridge {

using apollo::common::Status;
using apollo::common::adapter::AdapterManager;
using apollo::common::adapter::ImuAdapter;
using apollo::common::monitor::MonitorMessageItem;
using apollo::common::time::Clock;
using apollo::common::VehicleConfigHelper;
using ::Eigen::Vector3d;

UnitySimBridge::UnitySimBridge()  {}

UnitySimBridge::~UnitySimBridge() {}

Status UnitySimBridge::Start() {
  AdapterManager::Init(FLAGS_sim_bridge_adapter_config_file);

  //add callbacks here
  AdapterManager::AddImuRosCallback(&UnitySimBridge::OnImu, this); 
  AdapterManager::AddOdometryRosCallback(&UnitySimBridge::OnOdometry, this);
  AdapterManager::AddControlCommandCallback(&UnitySimBridge::OnControl, this);
  return Status::OK();
}

Status UnitySimBridge::Stop() {
  return Status::OK();
}

void UnitySimBridge::OnImu(const sensor_msgs::Imu &msg) {
  
  localization::CorrectedImu imu_msg;
  FillImuMsg(msg, &imu_msg);

  // publish imu messages
  AdapterManager::PublishImu(imu_msg);
  AINFO << "[OnImu]: Imu message publish success!";
}



void UnitySimBridge::FillImuMsg(const sensor_msgs::Imu &msg, localization::CorrectedImu *imu_msg) {

  // header
  AdapterManager::FillImuHeader(FLAGS_sim_bridge_module_name,
                                         imu_msg);

  auto mutable_imu = imu_msg->mutable_imu();
  mutable_imu->mutable_linear_acceleration()->set_x(msg.linear_acceleration.x);
  mutable_imu->mutable_linear_acceleration()->set_y(msg.linear_acceleration.y);
  mutable_imu->mutable_linear_acceleration()->set_z(msg.linear_acceleration.z);

  mutable_imu->mutable_angular_velocity()->set_x(msg.angular_velocity.x);
  mutable_imu->mutable_angular_velocity()->set_y(msg.angular_velocity.y);
  mutable_imu->mutable_angular_velocity()->set_z(msg.angular_velocity.z);

  mutable_imu->mutable_orientation()->set_qx(msg.orientation.x);
  mutable_imu->mutable_orientation()->set_qy(msg.orientation.y);
  mutable_imu->mutable_orientation()->set_qz(msg.orientation.z);
  mutable_imu->mutable_orientation()->set_qw(msg.orientation.w);

}

void UnitySimBridge::OnOdometry(const nav_msgs::Odometry &msg)
{
  localization::Gps gps_msg;

  FillGpsMsg(msg, &gps_msg);
  // publish gps messages
  AdapterManager::PublishGps(gps_msg);
  AINFO << "[OnGps]: Gps message publish success!";

  /*car_unity_simulator::CarControl control_msg;
  FillUnityCarControlMsg(&control_msg);
  AdapterManager::PublishUnityCarControl(control_msg);
  AINFO << "[OnGps]: Control message publish success!";*/

  canbus::Chassis chassis;

  chassis.set_error_code(canbus::Chassis::NO_ERROR);
  chassis.set_driving_mode(canbus::Chassis::COMPLETE_AUTO_DRIVE);
  chassis.set_gear_location(canbus::Chassis::GEAR_DRIVE);
  chassis.set_engine_started(true);
  //chassis.mutable_engage_advice()->set_advice(common::EngageAdvice::KEEP_ENGAGED);
  double vel = sqrt(msg.twist.twist.linear.x*msg.twist.twist.linear.x + msg.twist.twist.linear.y*msg.twist.twist.linear.y);
  if (vel < 0.1)
    vel = 0.0; 
  chassis.set_speed_mps(vel);
  //chassis.engage_advice().set_advice(common::EngageAdvice_Advice::EngageAdvice_Advice_KEEP_ENGAGED); //NOT WORKING
  AdapterManager::PublishChassis(chassis);
  AINFO << "[OnChassis]: Chassis message publish success!";
}

void UnitySimBridge::FillGpsMsg(const nav_msgs::Odometry &msg, localization::Gps *gps_msg)
{
  // header
  AdapterManager::FillGpsHeader(FLAGS_sim_bridge_module_name,
                                         gps_msg);

  auto mutable_loc = gps_msg->mutable_localization();
  float corrAngle = 1.5708f; //90 degrees in radians

  double pos_x, pos_y;

  pos_x = msg.pose.pose.position.x;
  pos_y = msg.pose.pose.position.y;
  AINFO << "[OLD COORDS]: x: " << pos_x << ", y: " << pos_y;
  double new_pos_x, new_pos_y;
  new_pos_x = pos_x * cos(corrAngle) - pos_y * sin(corrAngle);
  new_pos_y = pos_x * sin(corrAngle) + pos_y * cos(corrAngle);
  AINFO << "[NEW COORDS]: x: " << new_pos_x << ", y: " << new_pos_y;
  mutable_loc->mutable_position()->set_x(new_pos_x);
  mutable_loc->mutable_position()->set_y(new_pos_y);
  mutable_loc->mutable_position()->set_z(msg.pose.pose.position.z);

  mutable_loc->mutable_linear_velocity()->set_x(msg.twist.twist.linear.x);
  mutable_loc->mutable_linear_velocity()->set_y(msg.twist.twist.linear.y);
  mutable_loc->mutable_linear_velocity()->set_z(msg.twist.twist.linear.z);

  mutable_loc->mutable_orientation()->set_qx(msg.pose.pose.orientation.x);
  mutable_loc->mutable_orientation()->set_qy(msg.pose.pose.orientation.y);
  mutable_loc->mutable_orientation()->set_qz(msg.pose.pose.orientation.z);
  mutable_loc->mutable_orientation()->set_qw(msg.pose.pose.orientation.w);

}

void UnitySimBridge::FillUnityCarControlMsg(const control::ControlCommand &control_cmd, car_unity_simulator::CarControl *control_msg)
{
  const auto &vehicle_param = VehicleConfigHelper::GetConfig().vehicle_param();
  
  //In Unity we use accel command from [-1 +1]
  double accel = 0;
  if (control_cmd.brake() > 0)
    accel = - control_cmd.brake() / 100.0;
  else if (control_cmd.throttle() > 0)
    accel = control_cmd.throttle() / 100.0;
  control_msg->throttle = accel;

  double steering_angle =
      control_cmd.steering_target() / 100.0 * vehicle_param.max_steer_angle();

  const double max_unity_steer_angle = 25.0; //degrees

  control_msg->steering = steering_angle / max_unity_steer_angle;
  AINFO << "[CONTROL] Throttle: " << control_msg->throttle << " Steering: "<< control_msg->steering;

}

void UnitySimBridge::OnControl(const control::ControlCommand &msg)
{
  car_unity_simulator::CarControl control_msg;

  FillUnityCarControlMsg(msg, &control_msg);
  AdapterManager::PublishUnityCarControl(control_msg);
  AINFO << "[OnControl]: Control message publish success!";
}



}  // namespace sim_bridge
}  // namespace apollo
