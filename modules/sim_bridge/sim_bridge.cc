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

#include "modules/sim_bridge/sim_bridge.h"

#include "modules/common/log.h"
#include "modules/common/util/file.h"
#include "modules/sim_bridge/common/sim_bridge_gflags.h"
#include "modules/sim_bridge/unity_sim_bridge/unity_sim_bridge.h"

namespace apollo {
namespace sim_bridge {

using apollo::common::ErrorCode;
using apollo::common::Status;

std::string SimBridge::Name() const {
  return FLAGS_sim_bridge_module_name;
}

void SimBridge::RegisterSimBridgeMethods() {
  sim_bridge_factory_.Register(
      SimBridgeConfig::Unity,
      []() -> SimBridgeBase* { return new UnitySimBridge(); });
}

Status SimBridge::Init() {
  RegisterSimBridgeMethods();
  if (!apollo::common::util::GetProtoFromFile(FLAGS_sim_bridge_config_file,
                                              &config_)) {
    AERROR << "failed to load sim_bridge config file "
           << FLAGS_sim_bridge_config_file;
    return Status(ErrorCode::LOCALIZATION_ERROR,
                  "failed to load sim_bridge config file: " +
                      FLAGS_sim_bridge_config_file);
  }

  return Status::OK();
}

Status SimBridge::Start() {
  sim_bridge_ =
      sim_bridge_factory_.CreateObject(config_.sim_bridge_type());
  if (!sim_bridge_) {
    return Status(ErrorCode::LOCALIZATION_ERROR,
                  "sim_bridge is not initialized with config : " +
                      config_.DebugString());
  }
  sim_bridge_->Start();

  return Status::OK();
}

void SimBridge::Stop() { sim_bridge_->Stop(); }

}  // namespace sim_bridge
}  // namespace apollo
