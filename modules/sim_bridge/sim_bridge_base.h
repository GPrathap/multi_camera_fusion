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
 * @file sim_bridge_base.h
 * @brief The class of SimBridgeBase
 */

#ifndef MODULES_SIMBRIDGE_SIMBRIDGE_BASE_H_
#define MODULES_SIMBRIDGE_SIMBRIDGE_BASE_H_

#include <memory>

#include "modules/common/status/status.h"

/**
 * @namespace apollo::sim_bridge
 * @brief apollo::sim_bridge
 */
namespace apollo {
namespace sim_bridge {

/**
 * @class SimBridgeBase
 *
 * @brief base class for SimBridge factory
 */
class SimBridgeBase {
 public:
  virtual ~SimBridgeBase() = default;

  /**
   * @brief module start function
   * @return start status
   */
  virtual apollo::common::Status Start() = 0;

  /**
   * @brief module stop function
   * @return stop status
   */
  virtual apollo::common::Status Stop() = 0;


 protected:

};

}  // namespace localization
}  // namespace apollo

#endif  // MODULES_SIMBRIDGE_SIMBRIDGE_BASE_H_
