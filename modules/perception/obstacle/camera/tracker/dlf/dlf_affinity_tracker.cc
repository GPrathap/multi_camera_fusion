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

#include "opencv2/core/core.hpp"
#include "opencv2/features2d/features2d.hpp"
#include "opencv2/highgui/highgui.hpp"
#include <stdio.h>
#include <cmath>
#include "modules/perception/obstacle/camera/tracker/dlf/dlf_affinity_tracker.h"


namespace apollo {
namespace perception {

bool DLFAffinityTracker::Init() { return true; }

bool DLFAffinityTracker::GetAffinityMatrix(
    const cv::Mat &img, const std::vector<Tracked> &tracked,
    const std::vector<Detected> &detected,
    std::vector<std::vector<float>> *affinity_matrix) {
  affinity_matrix->clear();

  if (tracked.empty() || detected.empty()) return true;

  // Output. Default as 0.0 for not selected entries
  *affinity_matrix = std::vector<std::vector<float>>(
      tracked.size(), std::vector<float>(detected.size(), 0.0f));

  int minHessian = 400;
  size_t dim = tracked[0].features_.size();
  std::vector<std::vector<cv::Point>> tempery_featureset_holder;
  std::vector<uchar> status1, status2;
  std::vector<float> err1, err2;
  // size_t dim = tracked[0].features_.size();
  //AINFO<<"-------------------dtw distance-------------------tracked: "<< selected_entry_matrix_.size() << "detected: " << selected_entry_matrix_[0].size() << "\n";
  for (size_t i = 0; i < selected_entry_matrix_.size(); ++i) {
    double min_distance = 2000;
    double corresponding_id = -1;
    double total_error = min_distance;
    float sum = 0.0f;
    for (size_t j = 0; j < selected_entry_matrix_[0].size(); ++j) {

      // std::vector<double> tracked_feature(tracked[i].features_.begin(), tracked[i].features_.end());
      // std::vector<double> detected_feature(detected[j].features_.begin(), detected[j].features_.end());
      // //double result = CalculateDynamicTimeWarpedDistance(tracked[i].features_, detected[j].features_);
      // Eigen::MatrixXd tracked_feature_set = Eigen::Map<Eigen::MatrixXd>(tracked_feature.data(), 24, 24);
      // Eigen::MatrixXd detected_feature_set = Eigen::Map<Eigen::MatrixXd>(detected_feature.data(), 24, 24);
      //double result;
      //calculateMahalanobiasDistance(tracked_feature_set, detected_feature_set, result);
      std::vector<cv::Point2f> features_next;
      std::vector<cv::Point2f> features_previous;
      // std::cout<< "------------------vectorsize--------------" << tracked[i].image_prev.size() << "-------||" << std::endl;
      // std::cout<< "------------------vectorsize--------------" << detected[j].image_prev.size() << "-------||" << std::endl;
      // std::cout<< "------------------vectorsize--------------" <<  tracked[i].features_prev_.size() << "-------||" << std::endl;
      
      
      double width_ratio = 0.0;
      double height_ratio = 0.0;
      bool width_sign = tracked[i].box_.width < detected[j].box_.width;
      bool height_sign = tracked[i].box_.height < detected[j].box_.height;
      if(width_sign){
          width_ratio = detected[j].box_.width/tracked[i].box_.width;
      }else{
          width_ratio = tracked[i].box_.width/detected[j].box_.width;
      }
      if(height_sign){
          width_ratio = detected[j].box_.height*1.0/tracked[i].box_.height;
      }else{
          height_ratio = tracked[i].box_.height*1.0/detected[j].box_.height;
      }
      int tracked_object_area = tracked[i].box_.height *  tracked[i].box_.width;
      int detected_object_area = detected[j].box_.height * detected[j].box_.width;

      // std::cout<< "------------------vectorsize--------------" <<  tracked_object_area << "-------|| " << detected_object_area <<std::endl;
      // std::cout<< "------------------vectorsize--------------" <<  width_ratio << "-------|| " << height_ratio <<std::endl;

      //if(width_ratio< 1.35 && height_ratio < 1.35 && !tracked[i].image_prev.empty() && !tracked[i].features_prev_.empty()){
      if(width_ratio< 1.35 && height_ratio < 1.35 ){
            // cv::Mat current_image;
            // cv::Mat next_image;

            std::vector<double> tracked_feature(tracked[i].features_.begin(), tracked[i].features_.end());
            std::vector<double> detected_feature(detected[j].features_.begin(), detected[j].features_.end());
            //double result = CalculateDynamicTimeWarpedDistance(tracked[i].features_, detected[j].features_);
            Eigen::MatrixXd tracked_feature_set = Eigen::Map<Eigen::MatrixXd>(tracked_feature.data(), 24, 24);
            Eigen::MatrixXd detected_feature_set = Eigen::Map<Eigen::MatrixXd>(detected_feature.data(), 24, 24);
            double mismatch;
            calculateMahalanobiasDistance(tracked_feature_set, detected_feature_set, mismatch);
            

            //Eigen::MatrixXd tracked_feature_set = Eigen::Map<Eigen::MatrixXd>(tracked_feature.data(), 24, 24);
            //Eigen::MatrixXd detected_feature_set = Eigen::Map<Eigen::MatrixXd>(detected_feature.data(), 24, 24); 
            // if(tracked_object_area > detected_object_area){
            //   next_image = tracked[i].image_prev;  
            //   cv::resize(detected[j].image_prev, current_image, cv::Size(tracked[i].box_.width, tracked[i].box_.height), 0, 0, cv::INTER_CUBIC);
            // }else{
            //   current_image = detected[j].image_prev;  
            //   cv::resize(tracked[i].image_prev, next_image, cv::Size(detected[j].box_.width, detected[j].box_.height), 0, 0, cv::INTER_CUBIC);
            // }
            // if(current_image.empty() || next_image.empty()){
            //   continue;
            // }
            // cv::calcOpticalFlowPyrLK(next_image, current_image, tracked[i].features_prev_, features_previous, status1, err1);
            // cv::calcOpticalFlowPyrLK(current_image, next_image, features_previous, features_next, status2, err2);
            
            // if((features_next.size() <=  features_previous.size())&& status1[0] == 1 && status2[0] == 1 ){
            //     total_error = 0;
            //     for(int k=0; k< features_next.size(); k++){
            //       cv::Point2f point;
            //       point.x = std::abs(features_next.at(k).x - features_previous.at(k).x);
            //       point.y = std::abs(features_next.at(k).y - features_previous.at(k).y);
            //       total_error += std::sqrt(std::pow(point.x*1.0,2) + std::pow(point.y*1.0,2));
            //       // if ( (point.x < 1) && (point.y < 1)){
            //       //    std::cout<< "------------------just in10 --------------"  <<std::endl;
            //       //   tempery_featureset_holder[i].push_back(features_next.at(k));
            //       // }
            //     }
            //     // if(tempery_featureset_holder[i].size()>track_len){
            //     //    std::cout<< "------------------just in11-------------"  <<std::endl;
            //     //   tempery_featureset_holder[i].erase(tempery_featureset_holder[i].begin(), 
            //     //   tempery_featureset_holder[i].begin()+(track_len -tempery_featureset_holder[i].size() -1));   
            //     // }
            // }else{
            //   AWARN << "Cant find good feature to track the object...";
            // }

            if(mismatch<min_distance){
                min_distance = mismatch;
                corresponding_id = j;
            }
      }

    AINFO<<" ----------------- dtw distance ---------------------- average_disatnce " << min_distance << " tracking id: "<< i << " detected objects :" << j;
    //if(min_distance < kFilterThreshold_){
    if (corresponding_id != -1) {
      (*affinity_matrix)[i][corresponding_id] = 1.0f;
    }

    // }else{
    //   (*affinity_matrix)[i][corresponding_id] = 0.0f;
    // }
    // for (size_t k = 0; k < dim; ++k) {
    //   sum += tracked[i].features_[k] * detected[j].features_[k];
    // }

    // // Filtering. Ad-hoc
    // if (sum >= kConfThreshold_) {
    //   sum = 1.0f;
    // } else if (sum <= kFilterThreshold_) {
    //   sum = 0.0f;
    // }
  }
    AINFO<<" ----------------- dtw distance ----------------------";


   

    // cv::Mat descriptors_of_tracker = tracked[i].descriptor_of_features;
    // cv::Mat descriptors_of_detector = detected[corresponding_id].descriptor_of_features;
    // if( !descriptors_of_tracker.empty() && !descriptors_of_detector.empty()){
    //   cv::FlannBasedMatcher matcher;
    //   std::vector< cv::DMatch > matches;
    //   matcher.match( descriptors_of_tracker,  descriptors_of_detector, matches );
    //   double average_distance = 0;
    //   for( int i = 0; i < descriptors_of_tracker.rows; i++ )
    //   {
    //     average_distance+=matches[i].distance;
    //   }
    //    average_distance/=descriptors_of_tracker.rows;
    //    if(average_distance < kFilterThreshold_){
    //      (*affinity_matrix)[i][corresponding_id] = 1.0f;
    //    }else{
    //      (*affinity_matrix)[i][corresponding_id] = 0.0f;
    //    }
    //   AINFO<<" ----------------- dtw distance ---------------------- average_disatnce " << average_distance ;
    // }else{
    //   AINFO<<" --------------------------------------------- no feature point found ------------------------------------------------ ";
    // } 
    
  }

  return true;
}

bool DLFAffinityTracker::UpdateTracked(const cv::Mat &img,
                                       const std::vector<Detected> &detected,
                                       std::vector<Tracked> *tracked) {
  int d_cnt = detected.size();
  for (auto &obj : *tracked) {
    int d_id = obj.detect_id_;
    if (0 <= d_id && d_id < d_cnt) {
      obj.features_ = detected[d_id].features_;
    //   obj.image_prev = detected[d_id].image_prev;
    //   obj.features_prev_ = detected[d_id].features_prev_;
    }
  }
  return true;
}

void DLFAffinityTracker::getCovarianceMatrix(Eigen::MatrixXd input_mat, Eigen::MatrixXd& covariance_mat){
   long rows = input_mat.rows();
   if(rows==1){
       Eigen::MatrixXd meanMat = input_mat;
       covariance_mat = (meanMat.transpose()*meanMat)/rows;
   }else if(rows>0){
       Eigen::MatrixXd meanMat = input_mat.rowwise() - input_mat.colwise().mean();
       covariance_mat = (meanMat.transpose()*meanMat)/rows;
   }
}

void DLFAffinityTracker::calculateMahalanobiasDistance(Eigen::MatrixXd object1, Eigen::MatrixXd object2, double& distance){
   long rows_obj1 = object1.rows();
   long cols_obj1 = object1.cols();
   long rows_obj2 = object2.rows();
   long cols_obj2 = object2.cols();
   double total_length = rows_obj1 + rows_obj2;
   if(cols_obj1 != cols_obj2){
       std::cout<<"Number of feature points should be matched" << std::endl;
       return;
   }
   Eigen::MatrixXd mean_diff = object1.colwise().mean() - object2.colwise().mean();
   Eigen::MatrixXd covariance_obj1;
   getCovarianceMatrix(object1, covariance_obj1);
   Eigen::MatrixXd covariance_obj2;
   getCovarianceMatrix(object2, covariance_obj2);
   Eigen::MatrixXd total_covariance = (rows_obj1/total_length)*covariance_obj1
           + (rows_obj2/total_length)*covariance_obj2;
   distance = (mean_diff*total_covariance.inverse()*mean_diff.transpose())(0,0);
}

}  // namespace perception
}  // namespace apollo
