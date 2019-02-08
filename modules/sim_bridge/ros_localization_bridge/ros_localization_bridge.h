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
 * @file ros_localization_bridge.h
 * @brief The class of RosLocalizationBridge
 */

#ifndef MODULES_ROS_LOCALIZATION_BRIDGE_SIM_BRIDGE_H_
#define MODULES_ROS_LOCALIZATION_BRIDGE_SIM_BRIDGE_H_

#include <sstream>
#include <string>
#include <utility>
#include <vector>

#include "ros/include/ros/ros.h"

#include "glog/logging.h"
#include "gtest/gtest_prod.h"

#include "modules/localization/proto/localization.pb.h"
#include "modules/control/proto/control_cmd.pb.h"
#include "modules/canbus/proto/chassis.pb.h"
#include "modules/common/status/status.h"
#include "modules/sim_bridge/sim_bridge_base.h"
#include "nav_msgs/Odometry.h"
#include "sensor_msgs/Imu.h"
#include "tf2_ros/transform_broadcaster.h"
#include <tf2/LinearMath/Transform.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/utils.h>

/**
 * @namespace apollo::sim_bridge
 * @brief apollo::sim_bridge
 */
namespace apollo {
namespace sim_bridge {

/**
 * @class RosLocalizationBridge
 *
 * @brief It processes standart ROS messages from a Unity simulator as input,
 * to generate protobuf specific apollo messages.
 */
class RosLocalizationBridge : public SimBridgeBase {
 public:
  RosLocalizationBridge();
  virtual ~RosLocalizationBridge();

  /**
   * @brief module start function
   * @return start status
   */
  apollo::common::Status Start() override;

  /**
   * @brief module stop function
   * @return stop status
   */
  apollo::common::Status Stop() override;

 protected:
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf2_broadcaster_;

 private:

  void OnOdometry(const nav_msgs::Odometry &msg);
  void OnChassis(const canbus::Chassis &msg);
  void FillLocalizationMsg(const nav_msgs::Odometry &msg, localization::LocalizationEstimate *loc_msg);
  void OnImu(const sensor_msgs::Imu &msg);
  void PublishPoseBroadcastTF(const localization::LocalizationEstimate &localization);

  double x, y, yaw;

 private:

  sensor_msgs::Imu last_imu;
  bool has_imu;
};

}  // namespace sim_bridge
}  // namespace apollo

#endif  // MODULES_ROS_LOCALIZATION_BRIDGE_SIM_BRIDGE_H_
