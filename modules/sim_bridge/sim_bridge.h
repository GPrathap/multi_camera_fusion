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
 * @file sim_bridge.h
 * @brief The class of SimBridge
 */

#ifndef MODULES_SIMBRIDGE_SIMBRIDGE_H_
#define MODULES_SIMBRIDGE_SIMBRIDGE_H_

#include <memory>
#include <string>

#include "modules/sim_bridge/proto/sim_bridge_config.pb.h"

#include "modules/common/apollo_app.h"
#include "modules/common/status/status.h"
#include "modules/common/util/factory.h"
#include "modules/sim_bridge/sim_bridge_base.h"

/**
 * @namespace apollo::sim_bridge
 * @brief apollo::sim_bridge
 */
namespace apollo {
namespace sim_bridge {

/**
 * @class SimBridge
 *
 * @brief SimBridge module main class. It processes standart ROS messages from a simulator as input,
 * to generate protobuf specific apollo messages.
 */
class SimBridge : public apollo::common::ApolloApp {
 public:
  /**
   * @brief module name
   * @return module name
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
   * @return stop status
   */
  void Stop() override;

 private:
  void RegisterSimBridgeMethods();

  std::unique_ptr<SimBridgeBase> sim_bridge_;
  apollo::common::util::Factory<SimBridgeConfig::SimBridgeType,
                                SimBridgeBase>
      sim_bridge_factory_;
  SimBridgeConfig config_;
};

}  // namespace sim_bridge
}  // namespace apollo

#endif  // MODULES_SIMBRIDGE_SIMBRIDGE_H_
