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

#ifndef MODULES_PARKING_PARKING_H_
#define MODULES_PARKING_PARKING_H_

#include <cstdio>
#include <memory>
#include <mutex>
#include <string>

#include "modules/canbus/proto/chassis.pb.h"
#include "modules/common/monitor_log/monitor_log_buffer.h"
#include "modules/control/proto/control_cmd.pb.h"
#include "modules/localization/proto/localization.pb.h"
#include "modules/control/proto/pad_msg.pb.h"
#include "modules/parking/proto/parking_conf.pb.h"
#include "modules/parking/proto/parking_status.pb.h"

#include "modules/common/apollo_app.h"
#include "modules/common/util/util.h"

/**
 * @namespace apollo::parking
 * @brief apollo::parking
 */
namespace apollo {
namespace parking {

/**
 * @class Parking
 *
 * @brief parking module main class, it processes localization, chasiss, and
 * pad data to compute throttle, brake and steer values.
 */
class Parking : public apollo::common::ApolloApp {

 public:
  Parking()
      : monitor_logger_(apollo::common::monitor::MonitorMessageItem::CONTROL) {}

  /**
   * @brief module name
   */
  std::string Name() const override;

  /**
   * @brief module initialization function
   * @return initialization status
   */
  apollo::common::Status Init() override;

  /**
   * @brief module start function
   * @return start status
   */
  apollo::common::Status Start() override;

  /**
   * @brief module stop function
   */
  void Stop() override;

  /**
   * @brief destructor
   */
  virtual ~Parking() = default;

 private:
  // Watch dog timer
  void OnTimer(const ros::TimerEvent &);

  void SendRouteRequest(double x_s, double y_s, double x, double y);

 private:

  void SendPAD();
  double init_time_ = 0.0;
  double park_x, park_y;

  localization::LocalizationEstimate localization_;
  canbus::Chassis chassis_;
  ParkingState parking_state_;



  bool estop_ = false;

  unsigned int status_lost_ = 0;
  unsigned int status_sanity_check_failed_ = 0;
  unsigned int total_status_lost_ = 0;
  unsigned int total_status_sanity_check_failed_ = 0;

  ParkingConf parking_conf_;

  apollo::common::monitor::MonitorLogger monitor_logger_;
  ros::Timer timer_;
};

}  // namespace parking
}  // namespace apollo

#endif  // MODULES_PARKING_PARKING_H_
