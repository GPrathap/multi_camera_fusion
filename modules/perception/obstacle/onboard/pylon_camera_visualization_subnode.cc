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

#include "modules/perception/obstacle/onboard/pylon_camera_visualization_subnode.h"

#include <string>
#include <unordered_map>
#include <vector>
#include "modules/common/log.h"
#include "modules/perception/lib/config_manager/calibration_config_manager.h"
#include "modules/perception/obstacle/camera/common/util.h"
#include "modules/perception/obstacle/camera/visualizer/base_visualizer.h"
#include "modules/perception/obstacle/camera/visualizer/frame_content.h"
#include "modules/perception/obstacle/onboard/motion_service.h"
#include "modules/perception/onboard/dag_streaming.h"

#include "modules/perception/onboard/event_manager.h"
#include "modules/perception/onboard/shared_data_manager.h"
#include "modules/perception/onboard/subnode_helper.h"

namespace apollo {
namespace perception {
namespace trackvisualizer {

using apollo::common::ErrorCode;
using apollo::common::Status;
using apollo::perception::lowcostvisualizer::FrameContent;
using apollo::perception::lowcostvisualizer::BaseVisualizer;


bool TrackVisualizationSubnode::InitInternal() {
  AERROR << "trying to init modules";
  CHECK(shared_data_manager_ != NULL);
  // init stream
  if (!InitStream()) {
    AERROR << "Failed to init stream.";
    return false;
  }
  AERROR << "trying to init 2";
  std::unordered_map<std::string, std::string> reserve_field_map;
  if (!SubnodeHelper::ParseReserveField(reserve_, &reserve_field_map)) {
    AERROR << "Failed to parse reserve string: " << reserve_;
    return false;
  }
  // init camera object data
  if (camera_event_id_ != -1 || fusion_event_id_ != -1) {
    camera_orientation = reserve_field_map["camera_orientation"];
    AINFO << "init camera object----> " << camera_orientation;
    if( reserve_field_map["camera_orientation"] == "right_side"){
      camera_object_right_side_data_ = dynamic_cast<PylonCameraRightSideObjectData*>(
              shared_data_manager_->GetSharedData("PylonCameraRightSideObjectData"));
      camera_shared_right_side_data_ = dynamic_cast<PylonCameraRightSideSharedData*>(
              shared_data_manager_->GetSharedData("PylonCameraRightSideSharedData"));
      if (camera_object_right_side_data_ == nullptr) {
        AERROR << "Failed to get  PylonCameraRightSideObjectData.";
        return false;
      }
      AINFO << "Init shared datas successfully, data: "
            << camera_object_right_side_data_->name();

      if (camera_shared_right_side_data_ == nullptr) {
        AERROR << "Failed to get  PylonCameraSharedData.";
        return false;
      }
      AINFO << "Init shared datas successfully, data: "
            << camera_shared_right_side_data_->name();
    }else if( reserve_field_map["camera_orientation"] == "left_side"){
      camera_object_left_side_data_ = dynamic_cast<PylonCameraLeftSideObjectData*>(
              shared_data_manager_->GetSharedData("PylonCameraLeftSideObjectData"));
      camera_shared_left_side_data_ = dynamic_cast<PylonCameraLeftSideSharedData*>(
              shared_data_manager_->GetSharedData("PylonCameraLeftSideSharedData"));
      if (camera_object_left_side_data_ == nullptr) {
        AERROR << "Failed to get  PylonCameraObjectData.";
        return false;
      }
      AINFO << "Init shared datas successfully, data: "
            << camera_object_left_side_data_->name();

      if (camera_shared_left_side_data_ == nullptr) {
        AERROR << "Failed to get  PylonCameraLeftSideSharedData.";
        return false;
      }
      AINFO << "Init shared datas successfully, data: "
            << camera_shared_left_side_data_->name();
    }else if( reserve_field_map["camera_orientation"] == "right"){
      camera_object_right_data_ = static_cast<PylonCameraRightObjectData*>(
              shared_data_manager_->GetSharedData("PylonCameraRightObjectData"));
      camera_shared_right_data_ = static_cast<PylonCameraRightSharedData*>(
              shared_data_manager_->GetSharedData("PylonCameraRightSharedData"));
      if (camera_object_right_data_ == nullptr) {
        AERROR << "Failed to get  camera_object_right_data_ object.";
        return false;
      }
      AINFO << "Init object camera_shared_right_data_ successfully, data: "
            << camera_object_right_data_->name();

      if (camera_shared_right_data_ == nullptr) {
        AERROR << "Failed to get  camera_shared_right_data_.";
        return false;
      }
      AINFO << "Init shared camera_shared_right_data_ successfully, data: "
            << camera_shared_right_data_->name();
    }else if( reserve_field_map["camera_orientation"] == "left"){
      camera_object_left_data_ = static_cast<PylonCameraLeftObjectData *>(
              shared_data_manager_->GetSharedData("PylonCameraLeftObjectData"));

      camera_shared_left_data_ = static_cast<PylonCameraLeftSharedData *>(
              shared_data_manager_->GetSharedData("PylonCameraLeftSharedData"));

      if (camera_object_left_data_ == nullptr) {
        AERROR << "Failed to get  camera_object_left_data_ object.";
        return false;
      }
      AINFO << "Init object camera_shared_left_data_ successfully, data: "
            << camera_object_left_side_data_->name();

      if (camera_shared_left_data_ == nullptr) {
        AERROR << "Failed to get  camera_shared_left_data_.";
        return false;
      }
      AINFO << "Init shared camera_shared_left_data_ successfully, data: "
            << camera_shared_left_data_->name();
    }else if( reserve_field_map["camera_orientation"] == "right_backwards_side"){
      camera_object_right_backwards_side_data_ = static_cast<PylonCameraRightBackwardsSideObjectData *>(
              shared_data_manager_->GetSharedData("PylonCameraRightBackwardsSideObjectData"));
      camera_shared_right__backwards_side_data_ = static_cast<PylonCameraRightBackwardsSideSharedData *>(
              shared_data_manager_->GetSharedData("PylonCameraRightBackwardsSideSharedData"));
      if (camera_object_right_backwards_side_data_ == nullptr) {
        AERROR << "Failed to get  camera_object_left_data_ object.";
        return false;
      }
      AINFO << "Init object camera_object_right_backwards_side_data_ successfully, data: "
            << camera_object_right_backwards_side_data_->name();

      if (camera_shared_right__backwards_side_data_ == nullptr) {
        AERROR << "Failed to get  camera_shared_right__backwards_side_data_.";
        return false;
      }
      AINFO << "Init shared camera_shared_right__backwards_side_data_ successfully, data: "
            << camera_shared_right__backwards_side_data_->name();
    }else if( reserve_field_map["camera_orientation"] == "left_backwards_side"){
      camera_object_left_backwards_side_data_ = static_cast<PylonCameraLeftBackwardsSideObjectData *>(
              shared_data_manager_->GetSharedData("PylonCameraLeftBackwardsSideObjectData"));
      camera_shared_left__backwards_side_data_ = static_cast<PylonCameraLeftBackwardsSideSharedData *>(
              shared_data_manager_->GetSharedData("PylonCameraLeftBackwardsSideSharedData"));
      if (camera_object_left_backwards_side_data_ == nullptr) {
        AERROR << "Failed to get  camera_object_left_backwards_side_data_ object.";
        return false;
      }
      AINFO << "Init object camera_object_left_backwards_side_data_ successfully, data: "
            << camera_object_right_backwards_side_data_->name();

      if (camera_shared_left__backwards_side_data_ == nullptr) {
        AERROR << "Failed to get  camera_shared_left__backwards_side_data_.";
        return false;
      }
      AINFO << "Init shared camera_shared_right__backwards_side_data_ successfully, data: "
            << camera_shared_left__backwards_side_data_->name();
    }else if( reserve_field_map["camera_orientation"] == "front"){
      cam_obj_data_ = static_cast<PylonCameraObjectData *>(
              shared_data_manager_->GetSharedData("PylonCameraObjectData"));
      cam_shared_data_ = static_cast<PylonCameraSharedData *>(
              shared_data_manager_->GetSharedData("PylonCameraSharedData"));
      if (camera_object_left_backwards_side_data_ == nullptr) {
        AERROR << "Failed to get  cam_obj_data_ object.";
        return false;
      }
      AINFO << "Init object cam_obj_data_ successfully, data: "
            << cam_obj_data_->name();

      if (cam_shared_data_ == nullptr) {
        AERROR << "Failed to get  cam_shared_data_.";
        return false;
      }
      AINFO << "Init shared camera_shared_right__backwards_side_data_ successfully, data: "
            << cam_shared_data_->name();
    }

  }

  // init cipv object data
  if (cipv_event_id_ != -1) {
    cipv_object_data_ = dynamic_cast<CIPVObjectData*>(
        shared_data_manager_->GetSharedData("CIPVObjectData"));
    if (cipv_object_data_ == nullptr) {
      AERROR << "Failed to get CIPVObjectData.";
      return false;
    }
    AINFO << "Init shared datas successfully, data: "
          << cipv_object_data_->name();
  }

  //  init radar object data
  if (radar_event_id_ != -1 || fusion_event_id_ != -1) {
    radar_object_data_ = dynamic_cast<RadarObjectData*>(
        shared_data_manager_->GetSharedData("RadarObjectData"));
    if (radar_object_data_ == nullptr) {
      AERROR << "Failed to get RadarObjectData.";
      return false;
    }
    AINFO << "Init shared datas successfully, data: "
          << radar_object_data_->name();
  }

  // init fusion data
  if (fusion_event_id_ != -1) {
    fusion_data_ = dynamic_cast<FusionSharedData*>(
        shared_data_manager_->GetSharedData("FusionSharedData"));
    if (fusion_data_ == nullptr) {
      AERROR << "Failed to get FusionSharedData.";
      return false;
    }
    AINFO << "Init shared datas successfully, data: " << fusion_data_->name();
  }

  // init motion service
  if (motion_event_id_ != -1) {
    motion_service_ = dynamic_cast<MotionService*>(
        DAGStreaming::GetSubnodeByName("MotionService"));
    if (motion_service_ == nullptr) {
      AERROR << "motion service not inited";
      return false;
    }
  }

  if (lane_event_id_ != -1 || fusion_event_id_ != -1) {
    lane_shared_data_ = dynamic_cast<LaneSharedData*>(
        shared_data_manager_->GetSharedData("LaneSharedData"));
    if (lane_shared_data_ == nullptr) {
      AERROR << "Failed to get LaneSharedData.";
      return false;
    }
    AINFO << "Init shared data successfully, data: "
          << lane_shared_data_->name();
  }

  // init frame_visualizer
  apollo::perception::lowcostvisualizer::RegisterFactoryGLFusionVisualizer();
  frame_visualizer_.reset(
          apollo::perception::lowcostvisualizer::BaseVisualizerRegisterer::GetInstanceByName(FLAGS_frame_visualizer));
  if (!frame_visualizer_) {
    AERROR << "Failed to get instance: " << FLAGS_frame_visualizer;
    return false;
  }
  content_.set_pose_type(FrameContent::IMAGE_CONTINUOUS);
  AINFO << "visualize according to continuous image: ";

  CalibrationConfigManager* config_manager =
      Singleton<CalibrationConfigManager>::get();
  CameraCalibrationPtr calibrator = config_manager->get_camera_calibration();
  camera_to_car_pose_ = calibrator->get_camera_extrinsics();
  AINFO << "Init camera to car transform successfully.";
  content_.set_camera2car_pose(camera_to_car_pose_);
  return true;
}

bool TrackVisualizationSubnode::InitStream() {
  std::unordered_map<std::string, std::string> reserve_field_map;
  if (!SubnodeHelper::ParseReserveField(reserve_, &reserve_field_map)) {
    AERROR << "Failed to parse reserve string: " << reserve_;
    return false;
  }

  auto iter = reserve_field_map.find("vis_driven_event_id");
  if (iter == reserve_field_map.end()) {
    AERROR << "Failed to find vis_driven_event_id:" << reserve_;
    return false;
  }
  vis_driven_event_id_ = static_cast<EventID>(atoi((iter->second).c_str()));

  iter = reserve_field_map.find("camera_event_id");
  if (iter == reserve_field_map.end()) {
    AWARN << "Failed to find camera_event_id_: " << reserve_;
    camera_event_id_ = -1;
  } else {
    camera_event_id_ = static_cast<EventID>(atoi((iter->second).c_str()));
  }

  iter = reserve_field_map.find("radar_event_id");
  if (iter == reserve_field_map.end()) {
    AWARN << "Failed to find radar_event_id_: " << reserve_;
    radar_event_id_ = -1;
  } else {
    radar_event_id_ = static_cast<EventID>(atoi((iter->second).c_str()));
  }

  iter = reserve_field_map.find("fusion_event_id");
  if (iter == reserve_field_map.end()) {
    AWARN << "Failed to find fusion_event_id_: " << reserve_;
    fusion_event_id_ = -1;
  } else {
    fusion_event_id_ = static_cast<EventID>(atoi((iter->second).c_str()));
  }

  iter = reserve_field_map.find("cipv_event_id");
  if (iter == reserve_field_map.end()) {
    AWARN << "Failed to find cipv_event_id_: " << reserve_;
    cipv_event_id_ = -1;
  } else {
    cipv_event_id_ = static_cast<EventID>(atoi((iter->second).c_str()));
  }

  iter = reserve_field_map.find("motion_event_id");
  if (iter == reserve_field_map.end()) {
    AWARN << "Failed to find motion_event_id_: " << reserve_;
    motion_event_id_ = -1;
  } else {
    motion_event_id_ = static_cast<EventID>(atoi((iter->second).c_str()));
  }

  iter = reserve_field_map.find("lane_event_id");
  if (iter == reserve_field_map.end()) {
    AWARN << "Failed to find lane_event_id_: " << reserve_;
    lane_event_id_ = -1;
  } else {
    lane_event_id_ = static_cast<EventID>(atoi((iter->second).c_str()));
  }

  return true;
}

bool TrackVisualizationSubnode::SubscribeEvents(const EventMeta& event_meta,
                                           std::vector<Event>* events) const {
  Event event;
  if (event_meta.event_id == vis_driven_event_id_) {
    event_manager_->Subscribe(event_meta.event_id, &event);
    events->insert(events->begin(), event);
  } else {
    // no blocking
    while (event_manager_->Subscribe(event_meta.event_id, &event, true)) {
      events->push_back(event);
    }
  }

  return true;
}

void TrackVisualizationSubnode::SetRadarContent(const std::string& data_key,
                                           FrameContent* content,
                                           double timestamp) {
  if (radar_object_data_) {
    std::shared_ptr<SensorObjects> objs;

    if (!radar_object_data_->Get(data_key, &objs) || objs == nullptr) {
      AERROR << "Failed to get shared data: " << radar_object_data_->name();
      return;
    }
    content->set_radar_content(timestamp, objs->objects);
  }
}

void TrackVisualizationSubnode::SetCameraContent(const std::string& data_key,
                                            FrameContent* content,
                                            double timestamp) {
  std::shared_ptr<CameraItem> camera_item;
  if(camera_orientation == "left_side"){
    if (!camera_shared_left_side_data_->Get(data_key, &camera_item) ||
        camera_item == nullptr) {
      AERROR << "Failed to get shared data: " << camera_shared_left_side_data_->name();
      return;
    }
  }else if(camera_orientation == "right_side"){
    if (!camera_shared_right_side_data_->Get(data_key, &camera_item) ||
        camera_item == nullptr) {
      AERROR << "Failed to get shared data: " << camera_shared_right_side_data_->name();
      return;
    }
  }

  cv::Mat image = camera_item->image_src_mat.clone();
  content->set_image_content(timestamp, image);

  std::shared_ptr<SensorObjects> objs;
  if(camera_orientation == "left_side"){
    if (!camera_object_left_side_data_->Get(data_key, &objs) ||
            objs == nullptr) {
      AERROR << "Failed to get shared data: " << camera_object_left_side_data_->name();
      return;
    }
  }else if(camera_orientation == "right_side"){
    if (!camera_object_right_side_data_->Get(data_key, &objs) ||
            objs == nullptr) {
      AERROR << "Failed to get shared data: " << camera_object_right_side_data_->name();
      return;
    }
  }
  content->set_camera_content(timestamp, objs->sensor2world_pose,
                              objs->sensor2world_pose_static, objs->objects,
                              (*(objs->camera_frame_supplement)));
}

void TrackVisualizationSubnode::SetFusionContent(const std::string& data_key,
                                            FrameContent* content,
                                            double timestamp) {
  SharedDataPtr<FusionItem> fusion_item;
  if (!fusion_data_->Get(data_key, &fusion_item) || fusion_item == nullptr) {
    AERROR << "Failed to get shared data: " << fusion_data_->name();
    return;
  }
  std::string trigger_device_id = fusion_item->fused_sensor_device_id;
  double trigger_ts = fusion_item->fused_sensor_ts;
  std::string data_key_sensor;
  if (trigger_device_id == "camera") {
    SubnodeHelper::ProduceSharedDataKey(trigger_ts, trigger_device_id,
                                        &data_key_sensor);
    SetCameraContent(data_key_sensor, content, timestamp);
    SetLaneContent(data_key, content, timestamp);
  } else if (trigger_device_id == "radar_front") {
    SubnodeHelper::ProduceSharedDataKey(trigger_ts, trigger_device_id,
                                        &data_key_sensor);
    SetRadarContent(data_key_sensor, content, timestamp);
  }

  content->set_fusion_content(timestamp, fusion_item->obstacles);
  AINFO << "Set fused objects : " << fusion_item->obstacles.size();
}

void TrackVisualizationSubnode::SetLaneContent(const std::string& data_key,
                                          FrameContent* content,
                                          double timestamp) {
  if (lane_shared_data_) {
    LaneObjectsPtr lane_objs;
    if (!lane_shared_data_->Get(data_key, &lane_objs) || lane_objs == nullptr) {
      AERROR << "Failed to get shared data: " << lane_shared_data_->name();
      return;
    }
    content->set_lane_content(timestamp, *lane_objs);
  }
}

void TrackVisualizationSubnode::SetFrameContent(const Event& event,
                                           const std::string& device_id,
                                           const std::string& data_key,
                                           const double timestamp,
                                           FrameContent* content) {
  if (event.event_id == camera_event_id_) {
    SetCameraContent(data_key, content, timestamp);
  } else if (event.event_id == motion_event_id_) {
    //    AINFO << "Vis_subnode: motion_event_id_" << motion_event_id_;
    // TODO(gchen-apollo): add lock to read motion_buffer
    MotionBuffer motion_buffer = motion_service_->GetMotionBuffer();
    if (motion_buffer.empty()) {
      AINFO << "motion_buffer is null";
    } else {
      content->set_motion_content(timestamp, motion_buffer);
    }
  } else if (event.event_id == radar_event_id_) {
    if (device_id == "radar_front" && FLAGS_show_radar_objects) {
      SetRadarContent(data_key, content, timestamp);
    }
  } else if (event.event_id == fusion_event_id_) {
    bool show_fused_objects = true;
    if (show_fused_objects) {
      AINFO << "vis_driven_event data_key = " << data_key;
      SetFusionContent(data_key, content, timestamp);
    }
  } else if (event.event_id == cipv_event_id_) {
    if (FLAGS_show_camera_objects || FLAGS_show_camera_objects2d ||
        FLAGS_show_camera_parsing) {
      std::shared_ptr<CameraItem> camera_item;
      if(camera_orientation == "left_side"){
        if (!camera_shared_left_side_data_->Get(data_key, &camera_item) ||
            camera_item == nullptr) {
          AERROR << "Failed to get shared data: " << camera_shared_left_side_data_->name();
          return;
        }
      }else if(camera_orientation == "right_side"){
        if (!camera_shared_right_side_data_->Get(data_key, &camera_item) ||
            camera_item == nullptr) {
          AERROR << "Failed to get shared data: " << camera_shared_right_side_data_->name();
          return;
        }
      }

      cv::Mat clone_image = camera_item->image_src_mat;
      cv::Mat image = camera_item->image_src_mat.clone();
      content->set_image_content(timestamp, image);

      std::shared_ptr<SensorObjects> objs;
      if (!cipv_object_data_->Get(data_key, &objs) || objs == nullptr) {
        AERROR << "Failed to get shared data: " << cipv_object_data_->name();
        return;
      }

      LOG(INFO) << "number of objects in cipv is " << objs->objects.size()
                << timestamp << " with cipv index is " << objs->cipv_index;

      //   content->set_camera2velo_pose(_camera_to_velo64_pose);

      if (FLAGS_show_camera_parsing) {
        // content->set_camera_content(timestamp, objs->sensor2world_pose,
        //                            objs->objects,
        //                            (*(objs->camera_frame_supplement)));
      } else {
        content->set_camera_content(timestamp, objs->sensor2world_pose,
                                    objs->objects);
      }
    }
  } else if (event.event_id == lane_event_id_) {
    SetLaneContent(data_key, content, timestamp);
  }

  if (event.event_id == vis_driven_event_id_) {
    content->update_timestamp(timestamp);
  }
}

apollo::common::Status TrackVisualizationSubnode::ProcEvents() {
  for (auto event_meta : sub_meta_events_) {
    //    AINFO <<"Vis_sub: event_meta id: " << event_meta.event_id;
    std::vector<Event> events;
    if (!SubscribeEvents(event_meta, &events)) {
      return Status(ErrorCode::PERCEPTION_ERROR, "Failed to proc events.");
    }
    if (events.empty()) continue;

    for (size_t j = 0; j < events.size(); j++) {
      double timestamp = events[j].timestamp;
      const std::string& device_id = events[j].reserve;
      std::string data_key;

      if (!SubnodeHelper::ProduceSharedDataKey(timestamp, device_id,
                                               &data_key)) {
        AERROR << "Failed to produce shared data key. timestamp:" << timestamp
               << " device_id:" << device_id;
        return Status(ErrorCode::PERCEPTION_ERROR, "Failed to proc events.");
      }
      AINFO << "event: " << events[j].event_id << " device_id:" << device_id
            << " timestamp: " << timestamp << "data key" << data_key;
      AINFO << std::fixed << std::setprecision(64) << timestamp;

      SetFrameContent(events[j], device_id, data_key, timestamp, &content_);
    }

    if (event_meta.event_id == vis_driven_event_id_) {
      // Init of frame_visualizer must be in one thread with render,
      if (!init_) {
        frame_visualizer_->init();
        init_ = true;
      }
      frame_visualizer_->update_camera_system(&content_);
      frame_visualizer_->render(&content_);
    }
  }
  return Status::OK();
}

}  // namespace lowcostvisualizer
}  // namespace perception
}  // namespace apollo
