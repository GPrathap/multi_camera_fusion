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

#include "modules/sim_bridge/common/sim_bridge_gflags.h"

DEFINE_string(sim_bridge_module_name, "sim_bridge",
              "sim_bridge module name");

DEFINE_string(sim_bridge_adapter_config_file,
              "modules/sim_bridge/conf/unity_sim_bridge_adapter.conf",
              "unity_sim_bridge adapter configuration");

DEFINE_string(sim_bridge_config_file,
              "modules/sim_bridge/conf/unity_sim_bridge_config.pb.txt",
              "sim_bridge config file");
