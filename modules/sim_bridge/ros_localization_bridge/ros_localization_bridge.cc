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

#include "modules/sim_bridge/ros_localization_bridge/ros_localization_bridge.h"

#include "modules/common/configs/vehicle_config_helper.h"
#include "modules/common/adapters/adapter_manager.h"
#include "modules/common/math/quaternion.h"
#include "modules/common/time/time.h"
#include "modules/sim_bridge/common/sim_bridge_gflags.h"
#include "tf/tf.h"

namespace apollo {
namespace sim_bridge {

using apollo::common::Status;
using apollo::common::adapter::AdapterManager;
using apollo::common::monitor::MonitorMessageItem;
using apollo::common::time::Clock;
using apollo::common::VehicleConfigHelper;
using ::Eigen::Vector3d;

RosLocalizationBridge::RosLocalizationBridge()  {}

RosLocalizationBridge::~RosLocalizationBridge() {}

Status RosLocalizationBridge::Start() {
  AdapterManager::Init(FLAGS_sim_bridge_adapter_config_file);
  has_imu = false;
  tf2_broadcaster_.reset(new tf2_ros::TransformBroadcaster);

  AdapterManager::AddImuRosCallback(&RosLocalizationBridge::OnImu, this);
  AdapterManager::AddOdometryRosCallback(&RosLocalizationBridge::OnOdometry, this);

  AdapterManager::AddChassisCallback(&RosLocalizationBridge::OnChassis, this);

  AdapterManager::AddOdometryGnssCallback(&RosLocalizationBridge::OnGNSSOdometry, this);


  x = 0;
  y = 0;
  yaw = 0;

  origin_init = false;

  return Status::OK();
}

Status RosLocalizationBridge::Stop() {
  return Status::OK();
}


void RosLocalizationBridge::OnOdometry(const nav_msgs::Odometry &msg)
{
  localization::LocalizationEstimate loc_msg;

  //AINFO << "OnOdometry";
  if (has_imu)
  {
    FillLocalizationMsg(msg, &loc_msg);
    // publish localization messages
    //loc_msg.set_measurement_time(msg.header.stamp.toSec());
    loc_msg.set_measurement_time(ros::Time::now().toSec());
    AdapterManager::PublishLocalization(loc_msg);
    PublishPoseBroadcastTF(loc_msg);
    AINFO << "[OnOdometry]: Gps message publish success!";
  }
}


void RosLocalizationBridge::OnGNSSOdometry(const nav_msgs::Odometry &msg)
{
  if(!origin_init)
  {
    x = msg.pose.pose.position.x;
    y = -msg.pose.pose.position.y;
    yaw = -tf2::getYaw(msg.pose.pose.orientation) - M_PI/2.0;
    origin_init = true;
    AINFO << "OnGNSSOdometry: Init wheel odometry origin {" << x << ", " << y << ", " << yaw << "}";
  }  
}

void RosLocalizationBridge::OnChassis(const canbus::Chassis &msg)
{
  nav_msgs::Odometry wheels_odom_msg;

  // msg.steering_percentage;
  
  // msg.wheel_speed.wheel_direction_fr;
  // msg.wheel_speed.wheel_direction_fl;
  // msg.wheel_speed.wheel_direction_rr;
  // msg.wheel_speed.wheel_direction_rl;

  // msg.wheel_speed.wheel_spd_fr;
  // msg.wheel_speed.wheel_spd_fl;
  // msg.wheel_speed.wheel_spd_rr;
  // msg.wheel_speed.wheel_spd_rl;

  float base_length = 2.570;
  float max_steering_angle = 37 * (M_PI/180.0);

  double steering_angle = (3.0/4.0)* 1.0 * (msg.steering_percentage() / 110.0) * max_steering_angle;
  double angular_vel = msg.speed_mps() * tan(steering_angle) / base_length;


  // double dt = 1.0/100.0;
  // x += msg.speed_mps() * cos(yaw) * dt;
  // y += msg.speed_mps() * sin(yaw) * dt;
  // yaw += angular_vel * dt;

  // wheels_odom_msg.pose.pose.position.x = x;
  // wheels_odom_msg.pose.pose.position.y = y;
  // wheels_odom_msg.pose.pose.position.z = 0.0;

  // tf::Quaternion q = tf::createQuaternionFromRPY(0, 0, -M_PI/2.0 + yaw);
  // quaternionTFToMsg(q, wheels_odom_msg.pose.pose.orientation);


  // wheels_odom_msg.pose.covariance.assign(0);
  // wheels_odom_msg.pose.covariance[6 * 0 + 0] = 0.05;
  // wheels_odom_msg.pose.covariance[6 * 1 + 1] = 0.05;
  // wheels_odom_msg.pose.covariance[6 * 2 + 2] = 100.0;
  // wheels_odom_msg.pose.covariance[6 * 3 + 3] = 100.0;
  // wheels_odom_msg.pose.covariance[6 * 4 + 4] = 100.0;
  // wheels_odom_msg.pose.covariance[6 * 5 + 5] = 0.01;


  // wheels_odom_msg.twist.twist.linear.x = msg.speed_mps() * cos(yaw);
  // wheels_odom_msg.twist.twist.linear.y = msg.speed_mps() * sin(yaw);


  // velocity in fixed base_link frame 
  wheels_odom_msg.twist.twist.linear.x = msg.speed_mps();
  wheels_odom_msg.twist.twist.linear.y = 0;

  wheels_odom_msg.twist.twist.linear.z = 0.0;
  wheels_odom_msg.twist.twist.angular.x = 0.0;
  wheels_odom_msg.twist.twist.angular.y = 0.0;
  wheels_odom_msg.twist.twist.angular.z = angular_vel;

  wheels_odom_msg.twist.covariance.assign(0);
  wheels_odom_msg.twist.covariance[6 * 0 + 0] = 0.3;
  wheels_odom_msg.twist.covariance[6 * 1 + 1] = 0.3;
  wheels_odom_msg.twist.covariance[6 * 2 + 2] = 1.0;
  wheels_odom_msg.twist.covariance[6 * 3 + 3] = 100.0;
  wheels_odom_msg.twist.covariance[6 * 4 + 4] = 100.0;
  wheels_odom_msg.twist.covariance[6 * 5 + 5] = 0.1;

  wheels_odom_msg.header.stamp = ros::Time(msg.header().timestamp_sec());
  wheels_odom_msg.header.frame_id = "base_link"; // FLAGS_localization_tf2_frame_id;
  wheels_odom_msg.child_frame_id = "base_link"; //FLAGS_localization_tf2_child_frame_id;

  AdapterManager::PublishOdometryChassis(wheels_odom_msg);
  AINFO << "[OnChassis]: Odometry message publish";

}

void RosLocalizationBridge::PublishPoseBroadcastTF(
    const localization::LocalizationEstimate &localization) {
  if (!tf2_broadcaster_) {
    AERROR << "tf broadcaster is not created.";
    return;
  }

  // broadcast tf message
  geometry_msgs::TransformStamped tf2_msg;
  tf2_msg.header.stamp = ros::Time(localization.measurement_time());
  tf2_msg.header.frame_id = FLAGS_localization_tf2_frame_id;
  tf2_msg.child_frame_id = FLAGS_localization_tf2_child_frame_id;

  tf2_msg.transform.translation.x = localization.pose().position().x();
  tf2_msg.transform.translation.y = localization.pose().position().y();
  tf2_msg.transform.translation.z = localization.pose().position().z();


  tf2::Quaternion orientation_quat(localization.pose().orientation().qx(),localization.pose().orientation().qy(),localization.pose().orientation().qz(),localization.pose().orientation().qw());

  double roll, pitch, yaw;
  tf2::Matrix3x3 orTmp(orientation_quat);
  orTmp.getRPY(roll, pitch, yaw);

  tf2::Quaternion q_fin;
  q_fin.setRPY(roll, pitch, yaw+M_PI_2); //

  tf2_msg.transform.rotation.x = q_fin.x();
  tf2_msg.transform.rotation.y = q_fin.y();
  tf2_msg.transform.rotation.z = q_fin.z();
  tf2_msg.transform.rotation.w = q_fin.w();

  tf2_broadcaster_->sendTransform(tf2_msg);
}

void RosLocalizationBridge::FillLocalizationMsg(const nav_msgs::Odometry &msg, localization::LocalizationEstimate *loc_msg)
{
  // header
  AdapterManager::FillLocalizationHeader(FLAGS_sim_bridge_module_name,
                                         loc_msg);

  auto mutable_loc = loc_msg->mutable_pose();
  float corrAngle = 0.0; //1.5708f; //90 degrees in radians

  double pos_x, pos_y;

  pos_x = msg.pose.pose.position.x;
  pos_y = msg.pose.pose.position.y;
  double new_pos_x, new_pos_y;
  new_pos_x = pos_x * cos(corrAngle) - pos_y * sin(corrAngle);
  new_pos_y = pos_x * sin(corrAngle) + pos_y * cos(corrAngle);
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

  mutable_loc->mutable_linear_acceleration_vrf()->set_x(last_imu.linear_acceleration.x);
  mutable_loc->mutable_linear_acceleration_vrf()->set_y(last_imu.linear_acceleration.y);
  mutable_loc->mutable_linear_acceleration_vrf()->set_z(last_imu.linear_acceleration.z);

  mutable_loc->mutable_angular_velocity_vrf()->set_x(last_imu.angular_velocity.x);
  mutable_loc->mutable_angular_velocity_vrf()->set_y(last_imu.angular_velocity.y);
  mutable_loc->mutable_angular_velocity_vrf()->set_z(last_imu.angular_velocity.z);

  auto heading = apollo::common::math::QuaternionToHeading(
      msg.pose.pose.orientation.w, msg.pose.pose.orientation.x, msg.pose.pose.orientation.y,
      msg.pose.pose.orientation.z);

  mutable_loc->set_heading(heading);
}

void RosLocalizationBridge::OnImu(const sensor_msgs::Imu &msg) {
  last_imu = msg;
  has_imu = true;
  //AINFO << "OnImu";
}

}  // namespace sim_bridge
}  // namespace apollo
