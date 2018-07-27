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
 * @file unity_sim_bridge.h
 * @brief The class of UnitySimBridge
 */

#ifndef MODULES_UNITY_SIM_BRIDGE_SIM_BRIDGE_H_
#define MODULES_UNITY_SIM_BRIDGE_SIM_BRIDGE_H_

#include <sstream>
#include <string>
#include <utility>
#include <vector>

#include "ros/include/ros/ros.h"

#include "glog/logging.h"
#include "gtest/gtest_prod.h"

#include "modules/localization/proto/imu.pb.h"
#include "modules/localization/proto/gps.pb.h"
#include "modules/common/status/status.h"
#include "modules/sim_bridge/sim_bridge_base.h"
#include "sensor_msgs/Imu.h"
#include "nav_msgs/Odometry.h"
#include "car_unity_simulator/CarControl.h"

/**
 * @namespace apollo::sim_bridge
 * @brief apollo::sim_bridge
 */
namespace apollo {
namespace sim_bridge {

/**
 * @class UnitySimBridge
 *
 * @brief It processes standart ROS messages from a Unity simulator as input,
 * to generate protobuf specific apollo messages.
 */
class UnitySimBridge : public SimBridgeBase {
 public:
  UnitySimBridge();
  virtual ~UnitySimBridge();

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

 private:
  void OnImu(const sensor_msgs::Imu &msg);
  void FillImuMsg(const sensor_msgs::Imu &msg, localization::CorrectedImu *imu_msg);
  void OnOdometry(const nav_msgs::Odometry &msg);
  void FillGpsMsg(const nav_msgs::Odometry &msg, localization::Gps *gps_msg);
  void FillUnityCarControlMsg(car_unity_simulator::CarControl *control_msg);

 private:

};

}  // namespace sim_bridge
}  // namespace apollo

#endif  // MODULES_UNITY_SIM_BRIDGE_SIM_BRIDGE_H_
