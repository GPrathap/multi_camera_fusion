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
#include "modules/perception/traffic_light/onboard/tl_preprocessor_subnode.h"

#include "image_transport/image_transport.h"
#include "modules/perception/traffic_light/rectify/detection.h"
#include "modules/common/adapters/adapter_manager.h"
#include "modules/perception/common/perception_gflags.h"
#include "modules/perception/onboard/transform_input.h"
#include "modules/perception/traffic_light/base/utils.h"
#include "modules/perception/traffic_light/onboard/tl_proc_subnode.h"
#include "modules/perception/traffic_light/projection/projection.h"
#include "modules/perception/traffic_light/recognizer/unity_recognize.h"
#include "modules/perception/traffic_light/rectify/unity_rectify.h"
#include "modules/perception/traffic_light/rectify/detection.h"
#include "modules/perception/traffic_light/reviser/color_decision.h"

namespace apollo {
namespace perception {
namespace traffic_light {

using apollo::common::util::GetProtoFromFile;
using apollo::common::adapter::AdapterManager;

bool TLPreprocessorSubnode::InitInternal() {
  RegisterFactoryBoundaryProjection();
  if (!InitSharedData()) {
    AERROR << "TLPreprocessorSubnode init failed. Shared Data init failed.";
    return false;
  }

  if (!GetProtoFromFile(FLAGS_traffic_light_subnode_config, &config_)) {
    AERROR << "Cannot get config proto from file: "
           << FLAGS_traffic_light_subnode_config;
    return false;
  }

  // init preprocessor
  if (!InitPreprocessor()) {
    AERROR << "TLPreprocessorSubnode init failed.";
    return false;
  }
  // init hd_map
  if (!InitHdmap()) {
    AERROR << "TLPreprocessorSubnode Failed to init hdmap";
    return false;
  }
   if (FLAGS_use_externel_detector)
  {

    AdapterManager::AddExternelObjDetectionCallback(&TLPreprocessorSubnode::ExtObjDetectionCallback,
                                                           this);     
                                                                                    
  }
  else if (FLAGS_use_compressed_images)
  {

    AdapterManager::AddCompressedImageFrontCameraCallback(&TLPreprocessorSubnode::SubCameraCompressedImage,
                                                           this);        
                                                                                    
  }
  else{
    CHECK(AdapterManager::GetImageLong())
        << "TLPreprocessorSubnode init failed.ImageLong is not initialized.";
    AdapterManager::AddImageLongCallback(
        &TLPreprocessorSubnode::SubLongFocusCamera, this);
    CHECK(AdapterManager::GetImageShort())
        << "TLPreprocessorSubnode init failed.ImageShort is not initialized.";
    AdapterManager::AddImageShortCallback(
        &TLPreprocessorSubnode::SubShortFocusCamera, this);
  }
  return true;
  
}

bool TLPreprocessorSubnode::InitSharedData() {
  CHECK_NOTNULL(shared_data_manager_);

  const std::string preprocessing_data_name("TLPreprocessingData");
  preprocessing_data_ = dynamic_cast<TLPreprocessingData *>(
      shared_data_manager_->GetSharedData(preprocessing_data_name));
  if (preprocessing_data_ == nullptr) {
    AERROR << "TLPreprocessorSubnode failed to get shared data instance "
           << preprocessing_data_name;
    return false;
  }
  AINFO << "TLPreprocessorSubnode init shared data. name:"
        << preprocessing_data_->name();
  return true;
}

bool TLPreprocessorSubnode::InitPreprocessor() {
  if (!preprocessor_.Init()) {
    AERROR << "TLPreprocessorSubnode init preprocessor failed";
    return false;
  }

  return true;
}

bool TLPreprocessorSubnode::InitHdmap() {
  hd_map_ = HDMapInput::instance();
  if (hd_map_ == nullptr) {
    AERROR << "TLPreprocessorSubnode get hdmap failed.";
    return false;
  }
  return true;
}

bool TLPreprocessorSubnode::AddDataAndPublishEvent(
    const std::shared_ptr<ImageLights> &data, const CameraId &camera_id,
    double timestamp) {
  // add data down-stream
  std::string device_str = kCameraIdToStr.at(camera_id);
  std::string key;
  if (!SubnodeHelper::ProduceSharedDataKey(timestamp, device_str, &key)) {
    AERROR << "TLPreprocessorSubnode gen share data key failed. ts:"
           << GLOG_TIMESTAMP(timestamp);
    return false;
  }

  if (!preprocessing_data_->Add(key, data)) {
    AERROR << "TLPreprocessorSubnode push data into shared_data failed.";
    data->image.reset();
    return false;
  }
 
  // pub events
  for (size_t i = 0; i < this->pub_meta_events_.size(); ++i) {
    const EventMeta &event_meta = this->pub_meta_events_[i];
    Event event;
    event.event_id = event_meta.event_id;
    event.reserve = device_str;
    event.timestamp = timestamp;
    this->event_manager_->Publish(event);
  }
  return true;
}

void TLPreprocessorSubnode::SubLongFocusCamera(const sensor_msgs::Image &msg) {
  AdapterManager::Observe();
  SubCameraImage(AdapterManager::GetImageLong()->GetLatestObservedPtr(),
                 LONG_FOCUS);
  PERF_FUNCTION("SubLongFocusCamera");
}

void TLPreprocessorSubnode::SubShortFocusCamera(const sensor_msgs::Image &msg) {
  AdapterManager::Observe();
  SubCameraImage(AdapterManager::GetImageShort()->GetLatestObservedPtr(),
                 SHORT_FOCUS);
  PERF_FUNCTION("SubShortFocusCamera");
}
void TLPreprocessorSubnode::ExtObjDetectionCallback(const detection_msgs::DetectedObjectsWithImage &message) {
  
  double timestamp = message.header.stamp.toSec();
  ADEBUG << "TLPreprocessorSubnode ExtObjDetectionCallback: timestamp: ";
  ADEBUG << std::fixed << std::setprecision(64) << timestamp;
  AINFO << "camera received image : " << GLOG_TIMESTAMP(timestamp)
        << " at time: " << GLOG_TIMESTAMP(TimeUtil::GetCurrentTime());
  double curr_timestamp = timestamp * 1e9;

  if (FLAGS_skip_camera_frame && timestamp_ns_ > 0.0) {
    if ((curr_timestamp - timestamp_ns_) < (1e9 / FLAGS_camera_hz) &&
        curr_timestamp > timestamp_ns_) {
      ADEBUG << "TLPreprocessorSubnode Skip frame";
      return;
    }
  }

  timestamp_ns_ = curr_timestamp;
  
  CameraId camera_id=CameraId::FRONTCAMERA_FOCUS;
  std::vector<std::shared_ptr<DetectedRoadStuff>> lights;
  for ( auto& detObject : message.objects)
  {
    if (detObject.class_id!=9)
      continue;

    std::shared_ptr<DetectedRoadStuff> obj(new DetectedRoadStuff);
    obj->type = apollo::perception::ObjectType::LIGHT;
    obj->confidence=detObject.confidence;
    obj->upper_left[0]=detObject.x1;
    obj->upper_left[1]=detObject.y1;

    obj->lower_right[0]=detObject.x2;
    obj->lower_right[1]=detObject.y2;

    lights.push_back(obj);
  }
  if (lights.size()==0)
    return;
  std::shared_ptr<ImageLights> image_lights(new ImageLights); 
  AERROR<<lights.size();
  MutexLock lock(&mutex_);
  const double sub_camera_image_start_ts = TimeUtil::GetCurrentTime();
  std::shared_ptr<Image> image(new Image);
  cv::Mat cv_mat;

  //From compressed image
  cv::Mat cv_img;
   image_lights->camera_id=camera_id;
  CompMessageToMat(message.image,&cv_img);
  bool should_pub = false;
  image->Init(timestamp, camera_id, cv_img);
   const double before_sync_image_ts = TimeUtil::GetCurrentTime();
   //image->GenerateMat();
   //CameraSelection(timestamp);
  image_lights->image=image;

  const double sync_image_latency =TimeUtil::GetCurrentTime() - before_sync_image_ts;
    image_lights->camera_id=camera_id;
  if (!VerifyLightsProjection(image_lights)) {
    AINFO << "verify_lights_projection on image failed, ts:"
          << GLOG_TIMESTAMP(image->ts())
          << ", camera_id: " << kCameraIdToStr.at(camera_id);
    return;
   }
   AERROR<<"verify Last Protection:"<< (image_lights->lights).get()->size();
    std::vector<LightPtr> detected_bboxes;
    cv::Size img_size=cv_img.size();
    for (size_t candidate_id = 0; candidate_id < image_lights->num_signals; ++candidate_id) {
        if (candidate_id>=lights.size())
          break; 
        LightPtr tmp(new Light);
        tmp->region.rectified_roi.x =
            static_cast<int>(lights[candidate_id]->upper_left[0]*1280);
        tmp->region.rectified_roi.y =
            static_cast<int>(lights[candidate_id]->upper_left[1]*512);
        tmp->region.rectified_roi.width = static_cast<int>(
            (lights[candidate_id]->lower_right[0] - lights[candidate_id]->upper_left[0])*1280);
        tmp->region.rectified_roi.height = static_cast<int>(
            (lights[candidate_id]->lower_right[1] - lights[candidate_id]->upper_left[1] )*512);
        tmp->region.detect_score = lights[candidate_id]->confidence;
        if (!BoxIsValid(tmp->region.rectified_roi, img_size)) {
          AINFO << "Invalid width or height or x or y: "
                << tmp->region.rectified_roi.width << " | "
                << tmp->region.rectified_roi.height << " | "
                << tmp->region.rectified_roi.x << " | "
                << tmp->region.rectified_roi.y;
          continue;
        }
        std::vector<LightPtr> &lights_ref = *((image_lights->lights).get());
        //tmp->region.rectified_roi = RefinedBox(tmp->region.rectified_roi, img_size);
        tmp->region.is_detected = true;
        lights_ref[candidate_id]->region.detect_score= tmp->region.detect_score ;
        lights_ref[candidate_id]->region.detect_class_id = DetectionClassId(VERTICAL_CLASS);
        lights_ref[candidate_id]->region.rectified_roi=RefinedBox(tmp->region.rectified_roi, img_size);
        lights_ref[candidate_id]->region.is_detected =true;
         //CarPose pose;
        //GetCarPose(timestamp, &pose);
         //auto light_distance = Distance2Stopline(pose.pose(), lights_ref[candidate_id]->info.stop_line());
        //AERROR<<"light_distance:" <<light_distance;
        //detected_bboxes.push_back(tmp);
    }
   AINFO << "TLPreprocessorSubnode received a image msg"
        << ", camera_id: " << kCameraIdToStr.at(camera_id)
        << ", ts:" << GLOG_TIMESTAMP(message.header.stamp.toSec());

   // record current frame timestamp
  last_proc_image_ts_ = sub_camera_image_start_ts;

  image_lights->preprocess_receive_timestamp = sub_camera_image_start_ts;
  image_lights->preprocess_send_timestamp = TimeUtil::GetCurrentTime();
  if (AddDataAndPublishEvent(image_lights, camera_id, image->ts())) {
    preprocessor_.set_last_pub_camera_id(camera_id);
   
    image->GenerateMat();
    AINFO << "TLPreprocessorSubnode::sub_camera_image msg_time: "
          << GLOG_TIMESTAMP(image->ts())
          << " sync_image_latency: " << sync_image_latency * 1000 << " ms."
          << " sub_camera_image_latency: "
          << (TimeUtil::GetCurrentTime() - sub_camera_image_start_ts) * 1000
          << " ms."
          << " camera_id: " << kCameraIdToStr.at(camera_id);      
   //ProcessImage(image, camera_id,timestamp,sub_camera_image_start_ts);
  }
}
void TLPreprocessorSubnode::SubCameraImage(
    boost::shared_ptr<const sensor_msgs::Image> msg, CameraId camera_id) {
  // Only one image could be used in a while.
  // Ohters will be discarded
  // The pipeline turn to a single thread
  MutexLock lock(&mutex_);
  const double sub_camera_image_start_ts = TimeUtil::GetCurrentTime();
  std::shared_ptr<Image> image(new Image);
  cv::Mat cv_mat;
  double timestamp = msg->header.stamp.toSec();
  //From compressed image
  image->Init(timestamp, camera_id, msg);
  //Other function
   if (FLAGS_output_raw_img) {
    // user should create folders
    image->GenerateMat();
    char filename[100];
    snprintf(filename, sizeof(filename), "%s/%lf.jpg",
             image->camera_id_str().c_str(), timestamp);
    cv::imwrite(filename, image->mat());
  }
  AINFO << "TLPreprocessorSubnode received a image msg"
        << ", camera_id: " << kCameraIdToStr.at(camera_id)
        << ", ts:" << GLOG_TIMESTAMP(msg->header.stamp.toSec());
  ProcessImage(image, camera_id,timestamp,sub_camera_image_start_ts);
}
void TLPreprocessorSubnode::SubCameraCompressedImage(
    const sensor_msgs::CompressedImage &message) {

    CameraId camera_id=CameraId::FRONTCAMERA_FOCUS;

  // Only one image could be used in a while.
  // Ohters will be discarded
  // The pipeline turn to a single thread

  MutexLock lock(&mutex_);
  const double sub_camera_image_start_ts = TimeUtil::GetCurrentTime();
  std::shared_ptr<Image> image(new Image);
  cv::Mat cv_mat;
  double timestamp = message.header.stamp.toSec();
  //From compressed image
   cv::Mat cv_img;

  CompMessageToMat(message,&cv_img);
  image->Init(timestamp, camera_id, cv_img);
   if (FLAGS_output_raw_img) {
    // user should create folders
    image->GenerateMat();
    char filename[100];

    snprintf(filename, sizeof(filename), "%s/%lf.jpg",
             image->camera_id_str().c_str(), timestamp);
    cv::imwrite(filename, image->mat());
  }

  AINFO << "TLPreprocessorSubnode received a image msg"
        << ", camera_id: " << kCameraIdToStr.at(camera_id)
        << ", ts:" << GLOG_TIMESTAMP(message.header.stamp.toSec());
  ProcessImage(image, camera_id,timestamp,sub_camera_image_start_ts);
}

bool TLPreprocessorSubnode::CompMessageToMat(const sensor_msgs::CompressedImage &msg,
                                       cv::Mat *img) {
  cv_bridge::CvImagePtr cv_ptr(new cv_bridge::CvImage);
  // Copy message header
  cv_ptr->header = msg.header;
  cv_ptr->image = cv::imdecode(cv::Mat(msg.data), CV_LOAD_IMAGE_UNCHANGED);
   //cv_ptr->image = cv::imread(FLAGS_image_file_path, CV_LOAD_IMAGE_COLOR);
  cv_ptr->encoding = sensor_msgs::image_encodings::BGR8;
  *img = cv::Mat(cv_ptr->image.rows, cv_ptr->image.cols, CV_8UC3);
  *img = cv_ptr->image;
  return true;
}

void TLPreprocessorSubnode::ProcessImage(std::shared_ptr<Image> image, CameraId camera_id,double timestamp, double sub_camera_image_start_ts)
{

  // here the lights which visible on camera 
 // CameraSelection(timestamp);

  AINFO << "sub_camera_image_start_ts: "
        << GLOG_TIMESTAMP(sub_camera_image_start_ts)
        << " , last_proc_image_ts_: " << GLOG_TIMESTAMP(last_proc_image_ts_)
        << " , diff: "
        << GLOG_TIMESTAMP(sub_camera_image_start_ts - last_proc_image_ts_);
  const float proc_interval_seconds_ =
      1.0f / config_.tl_preprocessor_subnode_config().max_process_image_fps();

  if (last_proc_image_ts_ > 0.0 &&
      sub_camera_image_start_ts - last_proc_image_ts_ <
          proc_interval_seconds_) {
    AINFO << "skip current image, img_ts: " << GLOG_TIMESTAMP(timestamp)
          << " ,because proc_interval_seconds_: "
          << GLOG_TIMESTAMP(proc_interval_seconds_);
    return;
  }
  // sync image and publish data
  const double before_sync_image_ts = TimeUtil::GetCurrentTime();
  std::shared_ptr<ImageLights> image_lights(new ImageLights);
  bool should_pub = false;

  if (!preprocessor_.SyncImage(image, &image_lights, &should_pub)) {
    AINFO << "sync image failed ts: " << GLOG_TIMESTAMP(image->ts())
          << ", camera_id: " << kCameraIdToStr.at(camera_id);
  } else {
    AINFO << "sync image succeed ts: " << GLOG_TIMESTAMP(image->ts())
          << ", camera_id: " << kCameraIdToStr.at(camera_id);
  }

  const double sync_image_latency =
      TimeUtil::GetCurrentTime() - before_sync_image_ts;
  // Monitor image time and system time difference
  int max_cached_lights_size = preprocessor_.max_cached_lights_size();
  // tf frequency is 100Hz, 0.01 sec per frame，
  // cache frame num: max_cached_image_lights_array_size * 0.005 tf info
  const float tf_interval = 0.01;
  double image_sys_ts_diff_threshold = max_cached_lights_size * tf_interval;
  if (fabs(image_lights->diff_image_sys_ts) > image_sys_ts_diff_threshold) {
    std::string debug_string = "";
    debug_string += ("diff_image_sys_ts:" +
                     std::to_string(image_lights->diff_image_sys_ts));
    debug_string += (",camera_id:" + kCameraIdToStr.at(camera_id));
    debug_string += (",camera_ts:" + std::to_string(timestamp));

    AWARN << "image_ts - system_ts(in seconds): "
          << std::to_string(image_lights->diff_image_sys_ts)
          << ". Check if image timestamp drifts."
          << ", camera_id: " + kCameraIdToStr.at(camera_id)
          << ", debug_string: " << debug_string;
  }
   
  // if (!should_pub) {
  //   AINFO << "TLPreprocessorSubnode not publish image, ts:"
  //         << GLOG_TIMESTAMP(image->ts())
  //         << ", camera_id: " << kCameraIdToStr.at(camera_id);
  //   return;
  // }
  // verify lights projection based on image time

   if (!VerifyLightsProjection(image_lights)) {
    AINFO << "verify_lights_projection on image failed, ts:"
          << GLOG_TIMESTAMP(image->ts())
          << ", camera_id: " << kCameraIdToStr.at(camera_id);
    return;
   }
  // record current frame timestamp
  last_proc_image_ts_ = sub_camera_image_start_ts;

  image_lights->preprocess_receive_timestamp = sub_camera_image_start_ts;
  image_lights->preprocess_send_timestamp = TimeUtil::GetCurrentTime();
 
  if (AddDataAndPublishEvent(image_lights, camera_id, image->ts())) {
    preprocessor_.set_last_pub_camera_id(camera_id);
   
    image->GenerateMat();
    AINFO << "TLPreprocessorSubnode::sub_camera_image msg_time: "
          << GLOG_TIMESTAMP(image->ts())
          << " sync_image_latency: " << sync_image_latency * 1000 << " ms."
          << " sub_camera_image_latency: "
          << (TimeUtil::GetCurrentTime() - sub_camera_image_start_ts) * 1000
          << " ms."
          << " camera_id: " << kCameraIdToStr.at(camera_id);
  }
}


bool TLPreprocessorSubnode::GetSignals(double ts, CarPose *pose,
                                       std::vector<Signal> *signals) {
  // get pose
  if (!GetCarPose(ts, pose)) {
    AERROR << "camera_selection failed to get car pose, ts:"
          << GLOG_TIMESTAMP(ts);
    return false;
  }
  


  // get signals
  if (!hd_map_->GetSignals(pose->pose(), signals)) {
    if (ts - last_signals_ts_ < valid_hdmap_interval_) {
      *signals = last_signals_;
      AERROR << "camera_selection failed to get signals info. "
            << "Now use last info. ts:" << GLOG_TIMESTAMP(ts)
            << " pose:" << *pose;
    } else {
      AERROR << "camera_selection failed to get signals info. "
             << "ts:" << GLOG_TIMESTAMP(ts) << " pose:" << *pose;
      return false;
    }
  } else {
    last_signals_ = *signals;
    last_signals_ts_ = ts;
  }
  return true;
}
bool TLPreprocessorSubnode::GetCarPose(const double ts, CarPose *pose) {
  Eigen::Matrix4d pose_matrix;

  if (!GetVelodyneTrans(ts, &pose_matrix)) {
    AERROR << "TLPreprocessorSubnode failed to query pose ts:"
           << GLOG_TIMESTAMP(ts);
    return false;
  }
  pose->set_pose(pose_matrix);
  return true;
}
bool TLPreprocessorSubnode::VerifyLightsProjection(
    ImageLightsPtr image_lights) {
  std::vector<Signal> signals;
  CarPose pose;

  if (!GetSignals(image_lights->timestamp, &pose, &signals)) {
    return false;
  }

  // TODO(ghdawn): no need to init lights before this line
  image_lights->num_signals = signals.size();
  image_lights->lights.reset(new LightPtrs);
  image_lights->lights_outside_image.reset(new LightPtrs);
 
  if (!preprocessor_.ProjectLights(pose, signals, image_lights->camera_id,
                                   image_lights->lights.get(),
                                   image_lights->lights_outside_image.get())) {
    AINFO << "preprocessor_.select_camera_by_lights_projection failed";
    return false;
  }

  return true;
}
void TLPreprocessorSubnode::CameraSelection(double ts) {
  const double current_ts = TimeUtil::GetCurrentTime();
  AINFO << "current_ts: " << GLOG_TIMESTAMP(current_ts)
        << " , last_query_tf_ts: " << GLOG_TIMESTAMP(last_query_tf_ts_)
        << " , diff: " << GLOG_TIMESTAMP(current_ts - last_query_tf_ts_);
  // if (last_query_tf_ts_ > 0.0 &&
  //     current_ts - last_query_tf_ts_ < config_.tl_preprocessor_subnode_config()
  //                                          .query_tf_inverval_seconds()) {
  //   AINFO << "skip current tf msg, img_ts: " << GLOG_TIMESTAMP(ts);
  //   return;
  // }

  CarPose pose;
  std::vector<Signal> signals;
  if (!GetSignals(ts, &pose, &signals)) {
    return;
  }

  if (!preprocessor_.CacheLightsProjections(pose, signals, ts)) {
    AERROR << "add_cached_lights_projections failed, ts: "
           << GLOG_TIMESTAMP(ts);
  } else {
    AINFO << "add_cached_lights_projections succeed, ts: "
          << GLOG_TIMESTAMP(ts);
  }
  last_query_tf_ts_ = current_ts;
}

}  // namespace traffic_light
}  // namespace perception
}  // namespace apollo
