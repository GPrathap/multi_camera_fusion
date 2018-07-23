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
using ::Eigen::Vector3d;

UnitySimBridge::UnitySimBridge()  {}

UnitySimBridge::~UnitySimBridge() {}

Status UnitySimBridge::Start() {
  AdapterManager::Init(FLAGS_sim_bridge_adapter_config_file);

  //add callbacks here
  AdapterManager::AddImuRosCallback(&UnitySimBridge::OnImu, this); 
  AdapterManager::AddOdometryRosCallback(&UnitySimBridge::OnOdometry, this); 
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
  // publish imu messages
  AdapterManager::PublishGps(gps_msg);
  AINFO << "[OnGps]: Gps message publish success!";

  car_unity_simulator::CarControl control_msg;
  FillUnityCarControlMsg(&control_msg);
  AdapterManager::PublishUnityCarControl(control_msg);
  AINFO << "[OnGps]: Control message publish success!";
}

void UnitySimBridge::FillGpsMsg(const nav_msgs::Odometry &msg, localization::Gps *gps_msg)
{
  // header
  AdapterManager::FillGpsHeader(FLAGS_sim_bridge_module_name,
                                         gps_msg);

  auto mutable_loc = gps_msg->mutable_localization();
  mutable_loc->mutable_position()->set_x(msg.pose.pose.position.x);
  mutable_loc->mutable_position()->set_y(msg.pose.pose.position.y);
  mutable_loc->mutable_position()->set_z(msg.pose.pose.position.z);

  mutable_loc->mutable_linear_velocity()->set_x(msg.twist.twist.linear.x);
  mutable_loc->mutable_linear_velocity()->set_y(msg.twist.twist.linear.y);
  mutable_loc->mutable_linear_velocity()->set_z(msg.twist.twist.linear.z);

  mutable_loc->mutable_orientation()->set_qx(msg.pose.pose.orientation.x);
  mutable_loc->mutable_orientation()->set_qy(msg.pose.pose.orientation.y);
  mutable_loc->mutable_orientation()->set_qz(msg.pose.pose.orientation.z);
  mutable_loc->mutable_orientation()->set_qw(msg.pose.pose.orientation.w);

}

void UnitySimBridge::FillUnityCarControlMsg(car_unity_simulator::CarControl *control_msg)
{
  control_msg->throttle = 0.1;
}

}  // namespace sim_bridge
}  // namespace apollo
