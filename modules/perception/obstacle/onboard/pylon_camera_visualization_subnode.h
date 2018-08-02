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

#ifndef MODULES_PERCEPTION_OBSTACLE_ONBOARD_TRACK_VISUALIZATION_SUBNODE_H_
#define MODULES_PERCEPTION_OBSTACLE_ONBOARD_TRACK_VISUALIZATION_SUBNODE_H_

#include <memory>
#include <string>
#include <vector>
#include "modules/perception/obstacle/camera/visualizer/base_visualizer.h"
#include "modules/perception/obstacle/camera/visualizer/frame_content.h"
#include "modules/perception/obstacle/camera/visualizer/gl_fusion_visualizer.h"
#include "modules/perception/obstacle/onboard/pylon_camera_shared_data.h"
#include "modules/perception/obstacle/onboard/fusion_shared_data.h"
#include "modules/perception/obstacle/onboard/lane_shared_data.h"
#include "modules/perception/obstacle/onboard/motion_service.h"
#include "modules/perception/obstacle/onboard/object_shared_data.h"
#include "modules/perception/onboard/subnode.h"
#include "modules/perception/onboard/subnode_helper.h"

namespace apollo {
namespace perception {
namespace trackvisualizer {

class TrackVisualizationSubnode : public Subnode {
 public:
    TrackVisualizationSubnode() = default;

  // @breif: c++ spec ask for explicit destructor because of pointer class
  // member
  virtual ~TrackVisualizationSubnode() {}

  bool InitInternal() override;

  apollo::common::Status ProcEvents() override;

 private:
  bool InitStream();

  bool SubscribeEvents(const EventMeta& event_meta,
                       std::vector<Event>* events) const;

  void SetFrameContent(const Event& event, const std::string& device_id,
                       const std::string& data_key, const double timestamp,
                       apollo::perception::lowcostvisualizer::FrameContent* content);
  void SetFusionContent(const std::string& data_key,  apollo::perception::lowcostvisualizer::FrameContent* content,
                        double timestamp);
  void SetCameraContent(const std::string& data_key,  apollo::perception::lowcostvisualizer::FrameContent* content,
                        double timestamp);
  void SetRadarContent(const std::string& data_key,  apollo::perception::lowcostvisualizer::FrameContent* content,
                       double timestamp);
  void SetLaneContent(const std::string& data_key,  apollo::perception::lowcostvisualizer::FrameContent* content,
                      double timestamp);

  RadarObjectData* radar_object_data_ = nullptr;
  PylonCameraObjectData* camera_object_data_ = nullptr;
  CIPVObjectData* cipv_object_data_ = nullptr;
  PylonCameraSharedData* camera_shared_data_ = nullptr;
  LaneSharedData* lane_shared_data_ = nullptr;
  FusionSharedData* fusion_data_ = nullptr;
  std::unique_ptr<apollo::perception::lowcostvisualizer::BaseVisualizer> frame_visualizer_;
  MotionService* motion_service_ = nullptr;
  apollo::perception::lowcostvisualizer::FrameContent content_;

  EventID vis_driven_event_id_;
  EventID radar_event_id_;
  EventID camera_event_id_;
  EventID fusion_event_id_;
  EventID motion_event_id_;
  EventID cipv_event_id_;
  EventID lane_event_id_;

  //    MotionBufferPtr motion_buffer_;
  Eigen::Matrix4d camera_to_car_pose_;

  bool init_ = false;
  DISALLOW_COPY_AND_ASSIGN(TrackVisualizationSubnode);
};

REGISTER_SUBNODE(TrackVisualizationSubnode);

}  // namespace lowcostvisualizer
}  // namespace perception
}  // namespace apollo

#endif  // MODULES_PERCEPTION_OBSTACLE_ONBOARD_TRACK_VISUALIZATION_SUBNODE_H
