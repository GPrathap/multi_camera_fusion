/******************************************************************************
 * Copyright 2018 The Apollo Authors. All Rights Reserved.
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

#include "modules/perception/obstacle/onboard/camera_process_subnode.h"
#include "eigen_conversions/eigen_msg.h"
#include "modules/perception/common/perception_gflags.h"
#include "modules/perception/onboard/transform_input.h"
#include "modules/common/math/math_utils.h"

namespace apollo {
namespace perception {

using apollo::common::adapter::AdapterManager;
using Eigen::Affine3d;
using Eigen::Matrix4d;

bool CameraProcessSubnode::InitInternal() {
  // Subnode config in DAG streaming
  std::unordered_map<std::string, std::string> fields;
  SubnodeHelper::ParseReserveField(reserve_, &fields);

  if (fields.count("device_id")) {
    device_id_ = fields["device_id"];
  }
  if (fields.count("pb_obj") && stoi(fields["pb_obj"])) {
    pb_obj_ = true;
  }
  if (fields.count("pb_ln_msk") && stoi(fields["pb_ln_msk"])) {
    pb_ln_msk_ = true;
  }

  // Shared Data
  cam_obj_data_ = static_cast<CameraObjectData *>(
      shared_data_manager_->GetSharedData("CameraObjectData"));
  cam_shared_data_ = static_cast<CameraSharedData *>(
      shared_data_manager_->GetSharedData("CameraSharedData"));

  InitCalibration();

  InitModules();

  if (FLAGS_use_externel_detector)
  {
    AINFO << "AddExternelObjDetectionCallback";
    AdapterManager::AddExternelObjDetectionCallback(&CameraProcessSubnode::ExtObjDetectionCallback,
                                                           this);
  } else {

      if (device_id_=="camera"){ //standard Apollo camera ID
    AdapterManager::AddImageFrontCallback(&CameraProcessSubnode::ImgCallback,
                                          this);
    } else if (device_id_=="front_right_side_camera") {
      AdapterManager::AddImageFrontRightSideCallback(&CameraProcessSubnode::ImgCallback,
                                                            this);
    } else if (device_id_=="front_left_side_camera") {
      AdapterManager::AddImageFrontLeftSideCallback(&CameraProcessSubnode::ImgCallback,
                                                            this);
    } else if (device_id_=="right_side_camera") {
      AdapterManager::AddImageRightCallback(&CameraProcessSubnode::ImgCallback,
                                                            this);
    } else if (device_id_=="left_side_camera") {
      AdapterManager::AddImageRightCallback(&CameraProcessSubnode::ImgCallback,
                                                            this);
    } else if (device_id_=="backwards_right_side_camera") {
      AdapterManager::AddImageRightBackwardsSideCallback(&CameraProcessSubnode::ImgCallback,
                                                            this);
    } else if (device_id_=="backwards_left_side_camera") {
      AdapterManager::AddImageLeftBackwardsSideCallback(&CameraProcessSubnode::ImgCallback,
                                                            this);
    } else if (device_id_=="front_camera") {
      AdapterManager::AddImageFrontCameraCallback(&CameraProcessSubnode::ImgCallback,
                                                            this);
    } 

    AdapterManager::AddCompressedImageCallback(&CameraProcessSubnode::ImgCompressCallback,
                                              this);
  }



  if (pb_obj_) {
    AdapterManager::AddChassisCallback(&CameraProcessSubnode::ChassisCallback,
                                       this);
  }
  return true;
}

bool CameraProcessSubnode::InitCalibration() {
  auto calibration_config_manager = Singleton<CalibrationConfigManager>::get();
  calibration_config_manager->set_device_id_and_calibration_config_manager_init(device_id_);
  CameraCalibrationPtr calibrator = calibration_config_manager->get_camera_calibration();

  calibrator->get_image_height_width(&image_height_, &image_width_);
  camera_to_car_ = calibrator->get_camera_extrinsics();
  intrinsics_ = calibrator->get_camera_intrinsic();
  return true;
}

bool CameraProcessSubnode::InitModules() {
  RegisterFactoryYoloCameraDetector();
  RegisterFactoryGeometryCameraConverter();
  RegisterFactoryCascadedCameraTracker();
  RegisterFactoryFlatCameraTransformer();
  RegisterFactoryObjectCameraFilter();

  if (!FLAGS_use_externel_detector)
  {
    detector_.reset(
        BaseCameraDetectorRegisterer::GetInstanceByName("YoloCameraDetector"));
    detector_->Init();
  }

  converter_.reset(BaseCameraConverterRegisterer::GetInstanceByName(
      "GeometryCameraConverter"));
  converter_->Init();

  tracker_.reset(
      BaseCameraTrackerRegisterer::GetInstanceByName("CascadedCameraTracker"));
  tracker_->Init();

  transformer_.reset(BaseCameraTransformerRegisterer::GetInstanceByName(
      "FlatCameraTransformer"));
  transformer_->Init();
  transformer_->SetExtrinsics(camera_to_car_);

  filter_.reset(
      BaseCameraFilterRegisterer::GetInstanceByName("ObjectCameraFilter"));
  filter_->Init();

  return true;
}

void CameraProcessSubnode::ProcessImage(cv::Mat &img, double timestamp, std_msgs::Header msg_header)
{
  PERF_FUNCTION("ProcessImage");
  PERF_BLOCK_START();

  ADEBUG << "Received message height: " << img.rows << " width: " << img.cols;
  cv::resize(img, img, cv::Size(1280, 512), 0, 0);
  std::vector<std::shared_ptr<VisualObject>> objects;
  cv::Mat mask;

  PERF_BLOCK_END("CameraProcessSubnode_Image_Preprocess");
  detector_->Multitask(img, CameraDetectorOptions(), &objects, &mask);

  cv::Mat mask_color(mask.rows, mask.cols, CV_32FC1);
  if (FLAGS_use_whole_lane_line) {
    std::vector<cv::Mat> masks;
    detector_->Lanetask(img, &masks);
    mask_color.setTo(cv::Scalar(0));
    ln_msk_threshold_ = 0.9;
    for (int c = 0; c < num_lines; ++c) {
      for (int h = 0; h < masks[c].rows; ++h) {
        for (int w = 0; w < masks[c].cols; ++w) {
          if (masks[c].at<float>(h, w) >= ln_msk_threshold_) {
            mask_color.at<float>(h, w) = static_cast<float>(c);
          }
        }
      }
    }
  } else {
    mask.copyTo(mask_color);
    ln_msk_threshold_ = 0.5;
    for (int h = 0; h < mask_color.rows; ++h) {
      for (int w = 0; w < mask_color.cols; ++w) {
        if (mask_color.at<float>(h, w) >= ln_msk_threshold_) {
          mask_color.at<float>(h, w) = static_cast<float>(5);
        }
      }
    }
  }
  
  PERF_BLOCK_END("CameraProcessSubnode_detector_");

  ProcessDetectedObjects(objects, mask_color, img, timestamp, msg_header);
}

void CameraProcessSubnode::ProcessDetectedObjects(std::vector<std::shared_ptr<VisualObject>> &objects, cv::Mat &mask_color, cv::Mat &img, double timestamp, std_msgs::Header msg_header)
{
  

  PERF_FUNCTION("ProcessDetectedObjects");
  PERF_BLOCK_START();

  converter_->Convert(&objects);
  PERF_BLOCK_END("CameraProcessSubnode_converter_");

  if (FLAGS_use_navigation_mode) {
    transformer_->Transform(&objects);
    adjusted_extrinsics_ =
        transformer_->GetAdjustedExtrinsics(&camera_to_car_adj_);
    PERF_BLOCK_END("CameraProcessSubnode_transformer_");
  }

  tracker_->Associate(img, timestamp, &objects);
  PERF_BLOCK_END("CameraProcessSubnode_tracker_");

  FilterOptions options;

  if (FLAGS_use_navigation_mode) {
      options.camera_trans = std::make_shared<Eigen::Matrix4d>();
      options.camera_trans->setIdentity();
  } else {
      options.camera_trans = std::make_shared<Eigen::Matrix4d>();
      if (!GetCameraTrans(timestamp, options.camera_trans.get(), device_id_)) {
          AERROR << "Failed to get trans at timestamp: " << timestamp;
          return;
      }
  }

  camera_to_world_ = *(options.camera_trans);
  // need to create camera options here for filter
  AINFO << "camera_to_world_: " << camera_to_world_;
  filter_->Filter(timestamp, &objects, options);
  PERF_BLOCK_END("CameraProcessSubnode_filter_");

  auto calibration_config_manager = Singleton<CalibrationConfigManager>::get();
  calibration_config_manager->set_device_id_and_calibration_config_manager_init(device_id_);
  auto calibrator = calibration_config_manager->get_camera_calibration();
  calibrator->SetCar2CameraExtrinsicsAdj(camera_to_car_adj_,
                                         adjusted_extrinsics_);

  std::shared_ptr<SensorObjects> out_objs(new SensorObjects);
  out_objs->timestamp = timestamp;
  VisualObjToSensorObj(objects, &out_objs, options);

  SharedDataPtr<CameraItem> camera_item_ptr(new CameraItem);
  camera_item_ptr->image_src_mat = img;
  mask_color.copyTo(out_objs->camera_frame_supplement->lane_map);
  PublishDataAndEvent(timestamp, out_objs, camera_item_ptr);
  PERF_BLOCK_END("CameraProcessSubnode publish in DAG");

  if (pb_obj_) PublishPerceptionPbObj(out_objs);
  if (pb_ln_msk_) PublishPerceptionPbLnMsk(mask_color, msg_header);
}

void CameraProcessSubnode::ImgCallback(const sensor_msgs::Image &message) {
  double timestamp = message.header.stamp.toSec();
  ADEBUG << "CameraProcessSubnode ImgCallback: timestamp: ";
  ADEBUG << std::fixed << std::setprecision(64) << timestamp;
  AINFO << "camera received image : " << GLOG_TIMESTAMP(timestamp)
        << " at time: " << GLOG_TIMESTAMP(TimeUtil::GetCurrentTime());
  double curr_timestamp = timestamp * 1e9;

  if (FLAGS_skip_camera_frame && timestamp_ns_ > 0.0) {
    if ((curr_timestamp - timestamp_ns_) < (1e9 / FLAGS_camera_hz) &&
        curr_timestamp > timestamp_ns_) {
      ADEBUG << "CameraProcessSubnode Skip frame";
      return;
    }
  }

  timestamp_ns_ = curr_timestamp;
  ADEBUG << "CameraProcessSubnode Process:  frame: " << ++seq_num_;

  cv::Mat img;
  if (!FLAGS_image_file_debug) {
    MessageToMat(message, &img);
  } else {
    img = cv::imread(FLAGS_image_file_path, CV_LOAD_IMAGE_COLOR);
  }

  ProcessImage(img, timestamp, message.header);

}

void CameraProcessSubnode::ImgCompressCallback(const sensor_msgs::CompressedImage &message) {
  double timestamp = message.header.stamp.toSec();
  ADEBUG << "CameraProcessSubnode ImgCallback: timestamp: ";
  ADEBUG << std::fixed << std::setprecision(64) << timestamp;
  AINFO << "camera received image : " << GLOG_TIMESTAMP(timestamp)
        << " at time: " << GLOG_TIMESTAMP(TimeUtil::GetCurrentTime());
  double curr_timestamp = timestamp * 1e9;

  if (FLAGS_skip_camera_frame && timestamp_ns_ > 0.0) {
    if ((curr_timestamp - timestamp_ns_) < (1e9 / FLAGS_camera_hz) &&
        curr_timestamp > timestamp_ns_) {
      ADEBUG << "CameraProcessSubnode Skip frame";
      return;
    }
  }

  timestamp_ns_ = curr_timestamp;
  ADEBUG << "CameraProcessSubnode Process: "
         << " frame: " << ++seq_num_;
  PERF_FUNCTION("CameraProcessSubnode");
  PERF_BLOCK_START();

  cv::Mat img;
  if (!FLAGS_image_file_debug) {
    CompMessageToMat(message, &img);
  } else {
    img = cv::imread(FLAGS_image_file_path, CV_LOAD_IMAGE_COLOR);
  }

  ProcessImage(img, timestamp, message.header);

}

void CameraProcessSubnode::recover_bbox(int roi_w, int roi_h, int offset_y,
                  std::vector<std::shared_ptr<VisualObject>> *objects) {
  for (auto &obj : *objects) {
    float xmin = obj->upper_left[0];
    float ymin = obj->upper_left[1];
    float xmax = obj->lower_right[0];
    float ymax = obj->lower_right[1];
    int x = xmin * roi_w;
    int w = (xmax - xmin) * roi_w;
    int y = ymin * roi_h + offset_y;
    int h = (ymax - ymin) * roi_h;
    cv::Rect rect_det(x, y, w, h);
    cv::Rect rect_img(0, 0, roi_w, roi_h + offset_y);
    cv::Rect rect = rect_det & rect_img;
    obj->upper_left[0] = rect.x;
    obj->upper_left[1] = rect.y;
    obj->lower_right[0] = rect.x + rect.width;
    obj->lower_right[1] = rect.y + rect.height;
    double eps = 1e-2;

    // Truncation assignment based on bbox positions
    if ((ymin < eps) || (ymax >= 1.0 - eps)) {
      obj->trunc_height = 0.5;
    } else {
      obj->trunc_height = 0.0;
    }
    if ((xmin < eps) || (xmax >= 1.0 - eps)) {
      obj->trunc_width = 0.5;
    } else {
      obj->trunc_width = 0.0;
    }
  }
}

void CameraProcessSubnode::ExtObjDetectionCallback(const detection_msgs::DetectedObjectsWithImage &message) {
  double timestamp = message.header.stamp.toSec();
  ADEBUG << "CameraProcessSubnode ExtObjDetectionCallback: timestamp: ";
  ADEBUG << std::fixed << std::setprecision(64) << timestamp;
  AINFO << "camera received image : " << GLOG_TIMESTAMP(timestamp)
        << " at time: " << GLOG_TIMESTAMP(TimeUtil::GetCurrentTime());
  double curr_timestamp = timestamp * 1e9;

  if (FLAGS_skip_camera_frame && timestamp_ns_ > 0.0) {
    if ((curr_timestamp - timestamp_ns_) < (1e9 / FLAGS_camera_hz) &&
        curr_timestamp > timestamp_ns_) {
      ADEBUG << "CameraProcessSubnode Skip frame";
      return;
    }
  }

  timestamp_ns_ = curr_timestamp;
  ADEBUG << "CameraProcessSubnode Process: "
         << " frame: " << ++seq_num_;

  std::vector<std::shared_ptr<VisualObject>> objects;
  
  std::vector<ObjectType> types_ = { apollo::perception::ObjectType::VEHICLE,  //Car
                                apollo::perception::ObjectType::VEHICLE,  //Van 
                                apollo::perception::ObjectType::VEHICLE,  //Truck
                                apollo::perception::ObjectType::PEDESTRIAN, //Pedestrian
                                apollo::perception::ObjectType::UNKNOWN_UNMOVABLE, //Person_sitting
                                apollo::perception::ObjectType::BICYCLE,  //Cyclist
                                apollo::perception::ObjectType::VEHICLE, //Tram
                                apollo::perception::ObjectType::UNKNOWN //Misc
                              };


  for (auto det_obj : message.objects)
  {
      std::shared_ptr<VisualObject> obj(new VisualObject);
      obj->type = static_cast<ObjectType>(det_obj.class_id);
      obj->type_probs.assign(static_cast<int>(ObjectType::MAX_OBJECT_TYPE),
                             0.0f);
      int type_k = static_cast<int>(types_[det_obj.class_id]);
      obj->type_probs[type_k] = det_obj.confidence;

      obj->upper_left[0] = det_obj.x1;
      obj->upper_left[1] = det_obj.y1;
      obj->lower_right[0] = det_obj.x2;
      obj->lower_right[1] = det_obj.y2;

      obj->alpha = det_obj.yaw;

      obj->height = det_obj.height;
      obj->width = det_obj.width;
      obj->length = det_obj.length;

      obj->object_feature.clear();

      int feat_dim = det_obj.features.size();
      float* feat_data = det_obj.features.data();
      obj->object_feature.resize(feat_dim);
      memcpy(obj->object_feature.data(), feat_data,
            feat_dim * sizeof(feat_data[0]));
      apollo::common::math::L2Norm(feat_dim, obj->object_feature.data());
           
      ADEBUG << "Object added! Class: " << obj->type << " x1: " << obj->upper_left[0] << "y1: " << obj->upper_left[1] << 
            " x2: " << obj->lower_right[0] << " y2: " << obj->lower_right[1];
      objects.push_back(obj);
  }

  recover_bbox(message.image_width, message.image_height, 0.0, &objects);

  int det_id = 0;
  for (auto &obj : objects) {
    //projector_->project(&(obj->object_feature));

    // Assign detection id per box and score
    obj->id = det_id;
    for (auto prob : obj->type_probs) {
      obj->score = std::max(obj->score, prob);
    }

    ADEBUG << "obj-" << det_id << ": " << obj->object_feature.size();
    det_id++;
  }

  cv::Mat mask_color;

  cv::Mat img;
  CompMessageToMat(message.image, &img);

  ProcessDetectedObjects(objects, mask_color, img, timestamp, message.header);
  
}

void CameraProcessSubnode::ChassisCallback(
    const apollo::canbus::Chassis &message) {
  chassis_.CopyFrom(message);
}

bool CameraProcessSubnode::MessageToMat(const sensor_msgs::Image &msg,
                                        cv::Mat *img) {
  *img = cv::Mat(msg.height, msg.width, CV_8UC3);
  int pixel_num = msg.width * msg.height;
  if (msg.encoding.compare("yuyv") == 0) {
    unsigned char *yuv = (unsigned char *)&(msg.data[0]);
    yuyv2bgr(yuv, img->data, pixel_num);
  } else {
    cv_bridge::CvImagePtr cv_ptr =
        cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    *img = cv_ptr->image;
  }

  return true;
}

bool CameraProcessSubnode::CompMessageToMat(const sensor_msgs::CompressedImage &msg,
                                        cv::Mat *img) {

  cv_bridge::CvImagePtr cv_ptr(new cv_bridge::CvImage);

  // Copy message header
  cv_ptr->header = msg.header;

  cv_ptr->image = cv::imdecode(cv::Mat(msg.data), CV_LOAD_IMAGE_UNCHANGED);

  cv_ptr->encoding = sensor_msgs::image_encodings::BGR8;


  *img = cv::Mat(cv_ptr->image.rows, cv_ptr->image.cols, CV_8UC3);
  *img = cv_ptr->image;


  return true;
}

bool CameraProcessSubnode::MatToMessage(const cv::Mat &img,
                                        sensor_msgs::Image *msg) {
  if (img.type() == CV_8UC1) {
    sensor_msgs::fillImage(*msg, sensor_msgs::image_encodings::MONO8, img.rows,
                           img.cols, static_cast<unsigned int>(img.step),
                           img.data);
    return true;
  } else if (img.type() == CV_32FC1) {
    cv::Mat uc_img(img.rows, img.cols, CV_8UC1);
    uc_img.setTo(cv::Scalar(0));
    for (int h = 0; h < uc_img.rows; ++h) {
      for (int w = 0; w < uc_img.cols; ++w) {
        if (img.at<float>(h, w) >= ln_msk_threshold_) {
          uc_img.at<unsigned char>(h, w) = 1;
        }
      }
    }

    sensor_msgs::fillImage(*msg, sensor_msgs::image_encodings::MONO8,
                           uc_img.rows, uc_img.cols,
                           static_cast<unsigned int>(uc_img.step), uc_img.data);
    return true;
  } else {
    AERROR << "invalid input Mat type: " << img.type();
    return false;
  }
}

void CameraProcessSubnode::VisualObjToSensorObj(
    const std::vector<std::shared_ptr<VisualObject>> &objects,
    SharedDataPtr<SensorObjects> *sensor_objects, FilterOptions options) {
  (*sensor_objects)->sensor_type = SensorType::CAMERA;
  (*sensor_objects)->sensor_id = device_id_;
  (*sensor_objects)->seq_num = seq_num_;
  if (FLAGS_use_navigation_mode) {
    (*sensor_objects)->sensor2world_pose_static = camera_to_car_;
    (*sensor_objects)->sensor2world_pose = camera_to_car_adj_;
  } else {
    (*sensor_objects)->sensor2world_pose_static = *(options.camera_trans);
    (*sensor_objects)->sensor2world_pose = *(options.camera_trans);
    AINFO << "camera process sensor2world pose is "
          << (*sensor_objects)->sensor2world_pose;
  }
  ((*sensor_objects)->camera_frame_supplement).reset(new CameraFrameSupplement);

  if (!CameraFrameSupplement::state_vars.initialized_) {
    CameraFrameSupplement::state_vars.process_noise *= 10;
    CameraFrameSupplement::state_vars.trans_matrix.block(0, 0, 1, 4) << 1.0f,
        0.0f, 0.33f, 0.0f;
    CameraFrameSupplement::state_vars.trans_matrix.block(1, 0, 1, 4) << 0.0f,
        1.0f, 0.0f, 0.33f;
    ADEBUG << "state trans matrix in CameraFrameSupplement is \n"
           << CameraFrameSupplement::state_vars.trans_matrix << std::endl;
    CameraFrameSupplement::state_vars.initialized_ = true;
  }

  for (auto vobj : objects) {
    std::unique_ptr<Object> obj(new Object());

    obj->id = vobj->id;
    obj->score = vobj->score;
    obj->direction = vobj->direction.cast<double>();
    obj->theta = vobj->theta;
    obj->center = vobj->center.cast<double>();
    obj->length = vobj->length;
    obj->width = vobj->width;
    obj->height = vobj->height;
    obj->type = vobj->type;
    obj->track_id = vobj->track_id;
    obj->tracking_time = vobj->track_age;
    obj->latest_tracked_time = vobj->last_track_timestamp;
    obj->velocity = vobj->velocity.cast<double>();
    obj->anchor_point = obj->center;
    obj->state_uncertainty = vobj->state_uncertainty;

    (obj->camera_supplement).reset(new CameraSupplement());
    obj->camera_supplement->upper_left = vobj->upper_left.cast<double>();
    obj->camera_supplement->lower_right = vobj->lower_right.cast<double>();
    obj->camera_supplement->alpha = vobj->alpha;
    obj->camera_supplement->pts8 = vobj->pts8;
    ((*sensor_objects)->objects).emplace_back(obj.release());
    ADEBUG << "Obj Id:"<< vobj->id << " Score: " << vobj->score << " Direction: " << vobj->direction.cast<double>()
          << " Theta: " << vobj->theta << " Center: " << vobj->center.cast<double>();
  }
}

void CameraProcessSubnode::PublishDataAndEvent(
    const double timestamp, const SharedDataPtr<SensorObjects> &sensor_objects,
    const SharedDataPtr<CameraItem> &camera_item) {
  const CommonSharedDataKey key(timestamp, device_id_);
  cam_obj_data_->Add(key, sensor_objects);
  cam_shared_data_->Add(key, camera_item);

  for (const EventMeta& event_meta : pub_meta_events_) {
    Event event;
    event.event_id = event_meta.event_id;
    event.timestamp = timestamp;
    event.reserve = device_id_;
    event_manager_->Publish(event);
  }
}

void CameraProcessSubnode::PublishPerceptionPbObj(
    const SharedDataPtr<SensorObjects> &sensor_objects) {
  PerceptionObstacles obstacles;

  // Header
  AdapterManager::FillPerceptionObstaclesHeader("perception_obstacle",
                                                &obstacles);
  common::Header *header = obstacles.mutable_header();
  header->set_lidar_timestamp(0);
  header->set_camera_timestamp(timestamp_ns_);
  header->set_radar_timestamp(0);
  obstacles.set_error_code(sensor_objects->error_code);

  // Serialize each Object
  for (const auto &obj : sensor_objects->objects) {
    PerceptionObstacle *obstacle = obstacles.add_perception_obstacle();
    obj->Serialize(obstacle);
  }

  // Relative speed of objects + latest ego car speed in X
  for (auto obstacle : obstacles.perception_obstacle()) {
    obstacle.mutable_velocity()->set_x(
        obstacle.velocity().x() + chassis_.speed_mps());
  }

  AdapterManager::PublishPerceptionObstacles(obstacles);
  ADEBUG << "PublishPerceptionObstacles: " << obstacles.ShortDebugString();
}

void CameraProcessSubnode::PublishPerceptionPbLnMsk(
    const cv::Mat &mask, std_msgs::Header msg_header) {
  sensor_msgs::Image lane_mask_msg;
  lane_mask_msg.header = msg_header;
  lane_mask_msg.header.frame_id = "lane_mask";
  MatToMessage(mask, &lane_mask_msg);

  AdapterManager::PublishPerceptionLaneMask(lane_mask_msg);
  ADEBUG << "PublishPerceptionLaneMask";
}

}  // namespace perception
}  // namespace apollo
