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
#include "modules/perception/traffic_light/projection/multi_camera_projection.h"

#include <unordered_map>

#include "Eigen/Core"
#include "Eigen/Dense"

#include "modules/common/util/file.h"
#include "modules/perception/traffic_light/base/tl_shared_data.h"

namespace apollo {
namespace perception {
namespace traffic_light {

using apollo::common::util::GetProtoFromFile;

bool MultiCamerasProjection::Init() {
    AERROR << "INIt";
  if (!GetProtoFromFile(FLAGS_traffic_light_multi_camera_projection_config,
                        &config_)) {
    AERROR << "Cannot get config proto from file: "
           << FLAGS_traffic_light_multi_camera_projection_config;
    return false;
  }
  // Read camera names from config file
  const std::string &single_projection_name =
      config_.multi_camera_projection_config().single_projection();

  // Read each camera's config
  std::unordered_map<std::string, CameraCoeffient> camera_coeffients;
  for (const auto &camera_focus_config :
       config_.multi_camera_projection_config().camera_focus_config()) {
    const auto &camera_model_name = camera_focus_config.name();
    CameraCoeffient camera_coeffient;
    if (!camera_coeffient.init(camera_model_name,
                               camera_focus_config.camera_extrinsic_file(),
                               camera_focus_config.camera_intrinsic_file())) {
      AERROR << camera_model_name << " Projection init failed.";
      return false;
    }
    AERROR << "Name "<<camera_model_name;
    camera_coeffients[camera_model_name] = camera_coeffient;
    camera_names_.push_back(camera_model_name);
  }
  AERROR << "Before";
  projection_.reset(
      BaseProjectionRegisterer::GetInstanceByName(single_projection_name));
  if (projection_ == nullptr) {
    AERROR << "MultiCamerasProjection new projection failed. name:"
           << single_projection_name;
    return false;
  }
  AERROR << "Before";
  for (size_t i = 0; i < camera_names_.size(); ++i) {
    auto &camera_coeffient = camera_coeffients[camera_names_[i]];
    camera_coeffient.camera_extrinsic =
        camera_coeffient.camera_extrinsic.inverse().eval();

    AERROR << "Lidar to " << camera_names_[i] << " transform: ";
    AERROR << camera_coeffient.camera_extrinsic;
  }
  AERROR << "Before " << camera_names_.size();
  camera_coeffient_.resize(camera_names_.size());
  AERROR << "Params 6mm "<< camera_coeffients["camera_front_focus"].camera_extrinsic;
  camera_coeffient_[kLongFocusIdx] = camera_coeffients["camera_25mm_focus"];
  AERROR << "AFTER "<< camera_names_.size();
  camera_coeffient_[kShortFocusIdx] = camera_coeffients["camera_6mm_focus"];
  AERROR << "AFTER "<<kFrontCameraFocusIdx;
  camera_coeffient_[kFrontCameraFocusIdx] = camera_coeffients["camera_front_focus"];
  AERROR << "AFTER";
  // auto &short_focus_camera_coeffient = camera_coeffients["camera_6mm_focus"];
  // auto &long_focus_camera_coeffient = camera_coeffients["camera_25mm_focus"];
  camera_coeffient_[kLongFocusIdx].camera_extrinsic =
      camera_coeffient_[kLongFocusIdx].camera_extrinsic *
      camera_coeffient_[kShortFocusIdx].camera_extrinsic;
  AERROR << "Lidar to long(25mm): ";
  AINFO << camera_coeffient_[kLongFocusIdx].camera_extrinsic;
  AERROR << "Params "<< camera_coeffient_[kFrontCameraFocusIdx].camera_extrinsic;
  return true;
}

bool MultiCamerasProjection::Project(const CarPose &pose,
                                     const ProjectOption &option,
                                     Light *light) const {
  const Eigen::Matrix4d mpose = pose.pose();
  const apollo::hdmap::Signal &tl_info = light->info;
  bool ret = true;
  AERROR <<  "Into Project";
  auto camera_id = static_cast<int>(option.camera_id);
  if (camera_id < 0 || camera_id >= kCountCameraId) {
    AERROR << "Projection get invalid camera_id: " << camera_id
           << ", check camera parameters file.";
    return false;
  }
  AINFO << "Begin project camera: " << option.camera_id;
  ret =
      projection_->Project(camera_coeffient_[camera_id], mpose, tl_info, light);

  if (!ret) {
    AWARN << "Projection failed projection the traffic light. "
          << "camera_id: " << camera_id;
    return false;
  }
  return true;
}
}  // namespace traffic_light
}  // namespace perception
}  // namespace apollo
