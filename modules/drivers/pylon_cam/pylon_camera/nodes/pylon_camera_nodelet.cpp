
#include <boost/thread.hpp>

#include <ros/ros.h>
#include <pluginlib/class_list_macros.h>
#include <nodelet/nodelet.h>
#include "pylon_camera/pylon_camera_wrapper.h"

namespace pylon_camera {

class PylonCamNodelet: public nodelet::Nodelet
{
 public:
    PylonCamNodelet() {}
  ~PylonCamNodelet() {
  ROS_INFO("shutting down driver thread");
  if (device_thread_ != nullptr && device_thread_->joinable()) {
    device_thread_->join();
  }
  ROS_INFO("driver thread stopped");
};

 private:
  virtual void onInit();
  boost::shared_ptr<PylonCamWrapper> pylon_cam_ = nullptr;
  boost::shared_ptr<boost::thread> device_thread_ = nullptr;
};

void PylonCamNodelet::onInit()
{
  ROS_INFO("Pylon camera nodelet init");
  pylon_cam_.reset(new PylonCamWrapper(getNodeHandle(), getPrivateNodeHandle()));
  // spawn device poll thread
  device_thread_ = boost::shared_ptr<boost::thread>
        (new boost::thread(boost::bind(&PylonCamWrapper::spin, pylon_cam_)));
}

}

// Register this plugin with pluginlib.  Names must match nodelets.xml.
//
// parameters: package, class name, class type, base class type
PLUGINLIB_DECLARE_CLASS(pylon_camera, PylonCamNodelet,
                        pylon_camera::PylonCamNodelet, nodelet::Nodelet);
